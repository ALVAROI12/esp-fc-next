/*
 * fc_step: synchronous flight loop (see fc_step.h). Mirrors stabilizer.c's loop order:
 * estimator (with the previous control) -> commander -> controller -> mixer.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_step.h"

#include <math.h>
#include <string.h>

#include "controller.h"
#include "fc_kalman.h"
#include "filter.h"
#include "log.h"
#include "param.h"
#include "power_distribution.h"
#include "usec_time.h"

/* Sensor pre-processing, as in sensors_mpu6050_hm5883L_ms5611.c */
#define GYRO_LPF_CUTOFF_HZ 80.0f
#define ACCEL_LPF_CUTOFF_HZ 30.0f
#define BIAS_SAMPLES 1024              /* SENSORS_NBR_OF_BIAS_SAMPLES */
#define BIAS_MAX_STD_DPS 4.0f          /* about GYRO_VARIANCE_BASE at +/-2000 dps full scale */
#define ACC_SCALE_SAMPLES 200          /* SENSORS_ACC_SCALE_SAMPLES */

static struct {
    uint32_t tick;
    bool armed;
    bool emergencyStop;
    uint8_t phase; /* fc_phase_t, stored as one byte for the LOG table */

    /* calibration */
    double gyroSum[3], gyroSqSum[3];
    uint32_t biasCount;
    float gyroBias[3];
    double accScaleSum;
    uint32_t accScaleCount;
    float accScale;

    lpf2pData gyroLpf[3], accLpf[3];

    sensorData_t sensors;
    state_t state;
    control_t control;
    setpoint_t setpoint;      /* last commanded */
    setpoint_t activeSetpoint; /* after the watchdog */
    uint32_t setpointTick;
    bool haveSetpoint;
} fc;

void fc_init(void)
{
    memset(&fc, 0, sizeof(fc));
    fc.phase = FC_PHASE_CALIBRATING;
    fc.accScale = 1.0f;
    for (int i = 0; i < 3; i++) {
        lpf2pInit(&fc.gyroLpf[i], FC_STEP_RATE_HZ, GYRO_LPF_CUTOFF_HZ);
        lpf2pInit(&fc.accLpf[i], FC_STEP_RATE_HZ, ACCEL_LPF_CUTOFF_HZ);
    }
    fc.state.attitudeQuaternion.w = 1.0f;
    fcKalmanInit();
    controllerInit(ControllerTypePID);
    powerDistributionInit();
    powerStop();
}

void fc_arm(bool armed) { fc.armed = armed; }
bool fc_is_armed(void) { return fc.armed; }
void fc_emergency_stop(void) { fc.emergencyStop = true; }
fc_phase_t fc_phase(void) { return (fc_phase_t)fc.phase; }
uint32_t fc_tick(void) { return fc.tick; }
const state_t *fc_state(void) { return &fc.state; }
const control_t *fc_control(void) { return &fc.control; }
const setpoint_t *fc_active_setpoint(void) { return &fc.activeSetpoint; }

void fc_set_setpoint(const setpoint_t *setpoint)
{
    fc.setpoint = *setpoint;
    fc.setpoint.timestamp = fc.tick;
    fc.setpointTick = fc.tick;
    fc.haveSetpoint = true;
}

void fc_set_position_setpoint(float x, float y, float z, float yaw_deg)
{
    setpoint_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.mode.x = sp.mode.y = sp.mode.z = modeAbs;
    sp.mode.yaw = modeAbs;
    sp.position.x = x;
    sp.position.y = y;
    sp.position.z = z;
    sp.attitude.yaw = yaw_deg;
    fc_set_setpoint(&sp);
}

bool fc_push_position(float x, float y, float z, float std_dev)
{
    positionMeasurement_t m = {.x = x, .y = y, .z = z, .stdDev = std_dev};
    return fcKalmanPushPosition(&m);
}

bool fc_push_tof(float distance_m, float std_dev)
{
    tofMeasurement_t m = {.timestamp = fc.tick, .distance = distance_m, .stdDev = std_dev};
    return fcKalmanPushTof(&m);
}

bool fc_push_flow(float dpixel_x, float dpixel_y, float dt, float std_dev)
{
    flowMeasurement_t m = {.timestamp = fc.tick, .dpixelx = dpixel_x, .dpixely = dpixel_y,
                           .stdDevX = std_dev, .stdDevY = std_dev, .dt = dt};
    return fcKalmanPushFlow(&m);
}

static void calibrate(const float gyro[3], const float acc[3])
{
    if (fc.biasCount < BIAS_SAMPLES) {
        for (int i = 0; i < 3; i++) {
            fc.gyroSum[i] += gyro[i];
            fc.gyroSqSum[i] += (double)gyro[i] * gyro[i];
        }
        if (++fc.biasCount == BIAS_SAMPLES) {
            bool still = true;
            for (int i = 0; i < 3; i++) {
                double mean = fc.gyroSum[i] / BIAS_SAMPLES;
                double var = fc.gyroSqSum[i] / BIAS_SAMPLES - mean * mean;
                still &= var < (double)(BIAS_MAX_STD_DPS * BIAS_MAX_STD_DPS);
                fc.gyroBias[i] = (float)mean;
            }
            if (!still) { /* moved during calibration: start over, as the firmware does */
                memset(fc.gyroSum, 0, sizeof(fc.gyroSum));
                memset(fc.gyroSqSum, 0, sizeof(fc.gyroSqSum));
                fc.biasCount = 0;
            }
        }
        return;
    }
    fc.accScaleSum += sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (++fc.accScaleCount == ACC_SCALE_SAMPLES) {
        fc.accScale = (float)(fc.accScaleSum / ACC_SCALE_SAMPLES);
        fc.phase = FC_PHASE_READY;
    }
}

/* commander.c watchdog: level after 500 ms without setpoints, cut motors after 2 s. */
static void apply_watchdog(void)
{
    static const setpoint_t nullSetpoint;
    uint32_t age = fc.tick - fc.setpointTick;
    if (!fc.haveSetpoint || age > FC_SETPOINT_TIMEOUT_SHUTDOWN_MS) {
        fc.activeSetpoint = nullSetpoint;
    } else {
        fc.activeSetpoint = fc.setpoint;
        if (age > FC_SETPOINT_TIMEOUT_STABILIZE_MS) {
            fc.activeSetpoint.mode.x = modeDisable;
            fc.activeSetpoint.mode.y = modeDisable;
            fc.activeSetpoint.mode.roll = modeAbs;
            fc.activeSetpoint.mode.pitch = modeAbs;
            fc.activeSetpoint.mode.yaw = modeVelocity;
            fc.activeSetpoint.attitude.roll = 0;
            fc.activeSetpoint.attitude.pitch = 0;
            fc.activeSetpoint.attitudeRate.yaw = 0;
        }
    }
}

void fc_step(const float gyro_dps[3], const float acc_g[3])
{
    fc.tick++;

    if (fc.phase == FC_PHASE_CALIBRATING) {
        calibrate(gyro_dps, acc_g);
    }
    for (int i = 0; i < 3; i++) {
        fc.sensors.gyro.axis[i] = lpf2pApply(&fc.gyroLpf[i], gyro_dps[i] - fc.gyroBias[i]);
        fc.sensors.acc.axis[i] = lpf2pApply(&fc.accLpf[i], acc_g[i] / fc.accScale);
    }
    fc.sensors.interruptTimestamp = usecTimestamp();

    if (fc.phase != FC_PHASE_READY) {
        powerStop();
        return;
    }

    fcKalmanUpdate(&fc.state, &fc.sensors, &fc.control, fc.tick);
    apply_watchdog();
    controller(&fc.control, &fc.activeSetpoint, &fc.sensors, &fc.state, fc.tick);

    if (fc.armed && !fc.emergencyStop) {
        powerDistribution(&fc.control);
    } else {
        powerStop();
    }
}

void fc_get_state_array(float out[13])
{
    const state_t *s = &fc.state;
    const float v[13] = {
        s->position.x, s->position.y, s->position.z,
        s->velocity.x, s->velocity.y, s->velocity.z,
        s->attitude.roll, -s->attitude.pitch, s->attitude.yaw,
        s->attitudeQuaternion.w, s->attitudeQuaternion.x, s->attitudeQuaternion.y, s->attitudeQuaternion.z,
    };
    memcpy(out, v, sizeof(v));
}

LOG_GROUP_START(fcStep)
LOG_ADD(LOG_UINT32, tick, &fc.tick)
LOG_ADD(LOG_UINT8, armed, &fc.armed)
LOG_ADD(LOG_UINT8, phase, &fc.phase)
LOG_ADD(LOG_FLOAT, biasX, &fc.gyroBias[0])
LOG_ADD(LOG_FLOAT, biasY, &fc.gyroBias[1])
LOG_ADD(LOG_FLOAT, biasZ, &fc.gyroBias[2])
LOG_ADD(LOG_FLOAT, accScale, &fc.accScale)
LOG_GROUP_STOP(fcStep)
