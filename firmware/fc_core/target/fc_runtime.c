/*
 * fc_runtime: runs fc_step inside ESP-Drone's stabilizer task. See fc_runtime.h and docs/fc_core.md.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_runtime.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "commander.h"
#include "crtp.h"
#include "fc_step.h"
#include "log.h"
#include "param.h"
#include "pm_esplane.h"
#include "system.h"
#include "usec_time.h"

enum { MEAS_POSITION, MEAS_TOF, MEAS_FLOW };
typedef struct {
    uint8_t kind;
    float a, b, c, std;
} meas_t;

#define FC_RUNTIME_QUEUE_LEN 32
static StaticQueue_t queueStorage;
static uint8_t queueBuffer[FC_RUNTIME_QUEUE_LEN * sizeof(meas_t)];
static QueueHandle_t queue;
static uint32_t lastSetpointTimestamp;
static bool lastSystemArmed;
static uint32_t batteryTick;
static uint32_t stepUs, stepMaxUs, overruns; /* timing budget (G3): fc_step cost per tick */
static uint8_t safeMode; /* param fcRun.safeMode: latched hold-and-land supervisor, applied while disarmed */

void fcRuntimeInit(void)
{
    if (!queue) queue = xQueueCreateStatic(FC_RUNTIME_QUEUE_LEN, sizeof(meas_t), queueBuffer, &queueStorage);
    lastSetpointTimestamp = 0;
    lastSystemArmed = false;
    fc_init();
}

static bool put(uint8_t kind, float a, float b, float c, float std)
{
    meas_t m = {.kind = kind, .a = a, .b = b, .c = c, .std = std};
    return queue && xQueueSend(queue, &m, 0) == pdTRUE;
}

bool fcRuntimeEnqueuePosition(const positionMeasurement_t *p) { return put(MEAS_POSITION, p->x, p->y, p->z, p->stdDev); }
bool fcRuntimeEnqueuePose(const poseMeasurement_t *p) { return put(MEAS_POSITION, p->x, p->y, p->z, p->stdDevPos); }
bool fcRuntimeEnqueueTof(const tofMeasurement_t *t) { return put(MEAS_TOF, t->distance, 0, 0, t->stdDev); }
bool fcRuntimeEnqueueFlow(const flowMeasurement_t *f) { return put(MEAS_FLOW, f->dpixelx, f->dpixely, f->dt, f->stdDevX); }

void fcRuntimeStep(const sensorData_t *sensors, setpoint_t *setpoint, state_t *state, bool emergencyStop)
{
    meas_t m;
    while (xQueueReceive(queue, &m, 0) == pdTRUE) {
        if (m.kind == MEAS_POSITION) fc_push_position(m.a, m.b, m.c, m.std);
        else if (m.kind == MEAS_TOF) fc_push_tof(m.a, m.std);
        else fc_push_flow(m.a, m.b, m.c, m.std);
    }

    /* Only new packets refresh fc_step's command watchdog. After its own timeouts the commander returns
     * nullSetpoint (timestamp 0) or a back-dated copy: a changed timestamp but no packet, so also require
     * the stamp to be recent. Past its 2 s shutdown timeout it returns the high-level commander's
     * trajectory point (Crazyswarm2 takeoff/goTo/land) behind the old stamp, or nullSetpoint (mode.z
     * disabled) when that planner is stopped or commander.enHighLevel is off. The planner runs on board
     * without a ground link, so forward its points only while the link sees packets (Crazyswarm2 streams
     * mocap); otherwise fc_step's watchdog levels, holds and lands. */
    commanderGetSetpoint(setpoint, state);
    uint32_t age = xTaskGetTickCount() - setpoint->timestamp;
    if (setpoint->timestamp != lastSetpointTimestamp && age <= pdMS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS)) {
        lastSetpointTimestamp = setpoint->timestamp;
        fc_set_setpoint(setpoint);
    } else if (age > pdMS_TO_TICKS(FC_SETPOINT_TIMEOUT_SHUTDOWN_MS) && setpoint->mode.z == modeAbs && crtpIsConnected()) {
        fc_set_setpoint(setpoint);
    }

    /* ponytail: 1S pack assumed (ESP-Drone S3 kit); divide by the cell count for larger packs. 0 V before
     * pmInit (e.g. the P4 diagnostic profile) is rejected by fc_push_battery, so no low-battery action. */
    if (++batteryTick >= FC_MS_TO_TICKS(100)) {
        batteryTick = 0;
        fc_push_battery(pmGetBatteryVoltage());
    }

    if (!fc_is_armed()) fc_set_failsafe_enabled(safeMode != 0);
    if (emergencyStop) fc_emergency_stop(); /* latched until reboot, unlike the stabilizer flag */
    /* systemIsArmed() is a level (forceArm param). Arm only on its rising edge, so a refused arm or a
     * supervisor disarm is never retried behind the operator's back; dropping it always disarms. */
    bool armed = systemIsArmed();
    if (armed && !lastSystemArmed) fc_arm(true);
    if (!armed && fc_is_armed()) fc_arm(false);
    lastSystemArmed = armed;

    /* Driver samples are bias-corrected, scaled and gravity-aligned; its LPF is compiled out in this mode
     * (sensors_mpu6050_hm5883L_ms5611.c) because fc_step applies the same 80/30 Hz filters. */
    const float gyro[3] = {sensors->gyro.x, sensors->gyro.y, sensors->gyro.z};
    const float acc[3] = {sensors->acc.x, sensors->acc.y, sensors->acc.z};
    uint64_t start = usecTimestamp();
    fc_step(gyro, acc);
    stepUs = (uint32_t)(usecTimestamp() - start);
    if (stepUs > stepMaxUs) stepMaxUs = stepUs;
    if (stepUs > 1000000u / FC_STEP_RATE_HZ) overruns++;
    *state = *fc_state();
}

LOG_GROUP_START(fcTime)
LOG_ADD(LOG_UINT32, stepUs, &stepUs)
LOG_ADD(LOG_UINT32, stepMaxUs, &stepMaxUs)
LOG_ADD(LOG_UINT32, overruns, &overruns)
LOG_GROUP_STOP(fcTime)

PARAM_GROUP_START(fcRun)
PARAM_ADD(PARAM_UINT8, safeMode, &safeMode)
PARAM_GROUP_STOP(fcRun)
