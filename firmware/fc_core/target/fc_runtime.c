/*
 * fc_runtime: runs fc_step inside ESP-Drone's stabilizer task. See fc_runtime.h and docs/fc_core.md.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_runtime.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "commander.h"
#include "fc_step.h"
#include "log.h"
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
static uint32_t batteryTick;
static uint32_t stepUs, stepMaxUs, overruns; /* timing budget (G3): fc_step cost per tick */

void fcRuntimeInit(void)
{
    if (!queue) queue = xQueueCreateStatic(FC_RUNTIME_QUEUE_LEN, sizeof(meas_t), queueBuffer, &queueStorage);
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

    /* Only new packets refresh fc_step's command watchdog; the commander's own timeout handling is bypassed. */
    commanderGetSetpoint(setpoint, state);
    if (setpoint->timestamp != lastSetpointTimestamp) {
        lastSetpointTimestamp = setpoint->timestamp;
        fc_set_setpoint(setpoint);
    }

    /* ponytail: 1S pack assumed (ESP-Drone S3 kit); divide by the cell count for larger packs. 0 V before
     * pmInit (e.g. the P4 diagnostic profile) is rejected by fc_push_battery, so no low-battery action. */
    if (++batteryTick >= FC_MS_TO_TICKS(100)) {
        batteryTick = 0;
        fc_push_battery(pmGetBatteryVoltage());
    }

    if (emergencyStop) fc_emergency_stop(); /* latched until reboot, unlike the stabilizer flag */
    bool armed = systemIsArmed();
    if (armed != fc_is_armed()) fc_arm(armed); /* fc_arm refuses (and keeps refusing) unsafe arms */

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
