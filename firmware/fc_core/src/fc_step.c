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
#include "controller_pid.h"
#include "fc_kalman.h"
#include "fc_rpm_notch.h"
#include "filter.h"
#include "log.h"
#include "param.h"
#include "position_controller.h"
#include "power_distribution.h"
#include "usec_time.h"

/* Sensor pre-processing, as in sensors_mpu6050_hm5883L_ms5611.c (cutoffs in fc_step.h) */
#define BIAS_SAMPLES 1024              /* SENSORS_NBR_OF_BIAS_SAMPLES */
#define BIAS_MAX_STD_DPS 4.0f          /* about GYRO_VARIANCE_BASE at +/-2000 dps full scale */
#define ACC_SCALE_SAMPLES 200          /* SENSORS_ACC_SCALE_SAMPLES */

static struct {
    uint32_t tick;
    bool armed;
    uint32_t armTick;
    bool emergencyStop;
    bool tumbled;
    uint32_t tumbleTicks;
    float goodGyro[3], goodAcc[3]; /* last plausible IMU sample, held across short glitches */
    uint32_t imuBadTicks;
    bool airborne;      /* latched once the estimate rises FC_LIFTOFF_HEIGHT_M above the arming height */
    float groundZ;
    uint8_t phase; /* fc_phase_t, stored as one byte for the LOG table */

    /* calibration */
    double gyroSum[3], gyroSqSum[3];
    uint32_t biasCount;
    float gyroBias[3];
    double accScaleSum;
    uint32_t accScaleCount;
    float accScale;

    lpf2pData gyroLpf[3], accLpf[3];
    fc_rpm_notch_t rpmNotch;
    float rpm[4], rawGyro[3], notchGyro[3];
    uint32_t rpmTick, rpmTuneTick;
    bool haveRpm;
    uint8_t rpmActive;

    sensorData_t sensors;
    state_t state;
    control_t control;
    setpoint_t setpoint;      /* last commanded */
    setpoint_t activeSetpoint; /* after the watchdog */
    uint32_t setpointTick;
    bool haveSetpoint;

    /* Opt-in safety history; all storage is fixed-size and cleared by fc_init(). */
    uint8_t safetyState, safetyReason, posFresh, tofFresh;
    bool havePosition, haveTof, landXYDisabled, touchdownPending;
    uint32_t positionTick, tofTick, holdTick, landTick, touchdownTick, estimatorResets;
    float holdX, holdY, holdZ, holdYaw, tofHeight, height;
    bool haveFlow, flowLanding;
    uint32_t flowTick;
    uint8_t flowFresh, flowBrake;
} fc;
static bool controllerInitialized;
static bool rpmNotchEnabled;
static float rpmNotchQ = 5.0f;
static bool failsafeEnabled;

static float vertical_cosine(void)
{
    const quaternion_t *q = &fc.state.attitudeQuaternion;
    return 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
}

static bool position_fresh(void)
{
    return fc.havePosition &&
           (uint32_t)(fc.tick - fc.positionTick) <= FC_MS_TO_TICKS(FC_FAILSAFE_POSITION_TIMEOUT_MS);
}

static bool tof_fresh(void)
{
    return fc.haveTof && vertical_cosine() >= 0.5f &&
           (uint32_t)(fc.tick - fc.tofTick) <= FC_MS_TO_TICKS(FC_FAILSAFE_TOF_TIMEOUT_MS);
}

static bool state_valid(void)
{
    float values[13];
    fc_get_state_array(values);
    for (unsigned i = 0; i < 13; i++) if (!isfinite(values[i])) return false;
    /* Match the estimator's default position/velocity envelope; also reject corrupt quaternions. */
    for (unsigned i = 0; i < 3; i++) {
        if (fabsf(values[i]) > 100.0f || fabsf(values[i + 3]) > 10.0f) return false;
    }
    float qnorm = 0;
    for (unsigned i = 9; i < 13; i++) qnorm += values[i] * values[i];
    return qnorm > 0.9f && qnorm < 1.1f;
}

static bool flow_fresh(void)
{
    return fc.haveFlow && tof_fresh() && fc.tofHeight >= FC_FAILSAFE_FLOW_MIN_HEIGHT_M &&
           (uint32_t)(fc.tick - fc.flowTick) <= FC_MS_TO_TICKS(FC_FAILSAFE_FLOW_TIMEOUT_MS);
}

bool fc_set_failsafe_enabled(bool enabled)
{
    if (fc.armed) return false;
    failsafeEnabled = enabled;
    return true;
}

fc_failsafe_state_t fc_failsafe_state(void) { return (fc_failsafe_state_t)fc.safetyState; }

void fc_init(void)
{
    memset(&fc, 0, sizeof(fc));
    fc.phase = FC_PHASE_CALIBRATING;
    fc.accScale = 1.0f;
    fc_rpm_notch_init(&fc.rpmNotch, FC_STEP_RATE_HZ, rpmNotchQ);
    for (int i = 0; i < 3; i++) {
        lpf2pInit(&fc.gyroLpf[i], FC_STEP_RATE_HZ, FC_GYRO_LPF_HZ);
        lpf2pInit(&fc.accLpf[i], FC_STEP_RATE_HZ, FC_ACCEL_LPF_HZ);
    }
    fc.state.attitudeQuaternion.w = 1.0f;
    fcKalmanInit();
    fc.estimatorResets = fcKalmanResetCount();
    if (!controllerInitialized) {
        controllerInit(ControllerTypePID);
        controllerInitialized = true;
    }
    controllerPidReset(&fc.state);
    powerDistributionInit();
    powerStop();
}

uint32_t fc_rate_hz(void) { return FC_STEP_RATE_HZ; }

bool fc_set_rpm_notch(bool enabled, float q)
{
    if (fc.armed || !isfinite(q) || q < 1 || q > 50) return false;
    rpmNotchEnabled = enabled;
    rpmNotchQ = q;
    fc_rpm_notch_init(&fc.rpmNotch, FC_STEP_RATE_HZ, q);
    fc.haveRpm = false;
    fc.rpmActive = 0;
    return true;
}

bool fc_push_rpm(const float rpm[4])
{
    for (int m = 0; m < FC_RPM_MOTORS; m++) {
        if (!isfinite(rpm[m]) || rpm[m] < 0 || rpm[m] > 200000.0f) return false;
    }
    memcpy(fc.rpm, rpm, sizeof(fc.rpm));
    fc.rpmTick = fc.tick;
    fc.haveRpm = true;
    return true;
}

static void filter_gyro(const float gyro[3])
{
    for (int a = 0; a < 3; a++) fc.rawGyro[a] = fc.notchGyro[a] = gyro[a] - fc.gyroBias[a];
    bool fresh = rpmNotchEnabled && fc.haveRpm &&
                 (uint32_t)(fc.tick - fc.rpmTick) <= FC_MS_TO_TICKS(FC_RPM_TIMEOUT_MS);
    if (!fresh) {
        if (fc.rpmActive) fc_rpm_notch_clear(&fc.rpmNotch);
        fc.rpmActive = 0;
    } else {
        if (!fc.rpmActive || (uint32_t)(fc.tick - fc.rpmTuneTick) >= FC_MS_TO_TICKS(FC_RPM_TUNE_MS)) {
            fc_rpm_notch_update(&fc.rpmNotch, fc.rpm);
            fc.rpmTuneTick = fc.tick;
        }
        fc.rpmActive = 1; /* fresh telemetry; individual out-of-band/stopped sections still bypass */
        fc_rpm_notch_apply(&fc.rpmNotch, fc.notchGyro);
    }
}

static float tilt_deg(void)
{
    /* angle between body z and world z from the estimated quaternion */
    float r22 = vertical_cosine();
    if (r22 > 1.0f) r22 = 1.0f;
    if (r22 < -1.0f) r22 = -1.0f;
    return acosf(r22) * 57.29578f;
}

bool fc_arm(bool armed)
{
    if (!armed) {
        fc.armed = false;
        return true;
    }
    // The estimator starts level and may not have converged immediately after calibration.
    // Check measured gravity as well, so a tilted/inverted stationary vehicle cannot arm.
    float ax = fc.sensors.acc.x, ay = fc.sensors.acc.y, az = fc.sensors.acc.z;
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (fc.phase != FC_PHASE_READY || fc.emergencyStop || fc.tumbled ||
        (!fc.armed && fc.safetyState >= FC_FAILSAFE_LAND) ||
        (failsafeEnabled && (!position_fresh() || !tof_fresh() || !state_valid())) ||
        !isfinite(norm) || norm < 0.8f || norm > 1.2f ||
        az / norm < cosf(FC_ARM_MAX_TILT_DEG * 0.0174532925f) || tilt_deg() > FC_ARM_MAX_TILT_DEG) {
        return false;
    }
    if (fc.armed) return true; /* idempotent: do not reset the lift-off reference midflight */
    /* A fresh arm never inherits a pre-landing HOLD pose or reason (LAND and later are refused above). */
    fc.safetyState = FC_FAILSAFE_NORMAL;
    fc.safetyReason = 0;
    fc.armed = true;
    fc.armTick = fc.tick;
    fc.airborne = false;
    fc.groundZ = fc.state.position.z;
    fc.estimatorResets = fcKalmanResetCount();
    return true;
}

bool fc_is_armed(void) { return fc.armed; }
bool fc_is_tumbled(void) { return fc.tumbled; }
void fc_emergency_stop(void) { fc.emergencyStop = true; fc.armed = false; }
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

void fc_set_trajectory_setpoint(float x, float y, float z, float vx, float vy, float vz,
                                float ax, float ay, float az, float yaw_deg)
{
    setpoint_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.mode.x = sp.mode.y = sp.mode.z = modeAbs;
    sp.mode.yaw = modeAbs;
    sp.position.x = x;
    sp.position.y = y;
    sp.position.z = z;
    sp.velocity.x = vx;
    sp.velocity.y = vy;
    sp.velocity.z = vz;
    sp.acceleration.x = ax;
    sp.acceleration.y = ay;
    sp.acceleration.z = az;
    sp.attitude.yaw = yaw_deg;
    fc_set_setpoint(&sp);
}

/* ESP-FC Next: even legacy mode never enqueues a value that would make the EKF state NaN. */
static bool finite_measurement(float a, float b, float c, float std_dev)
{
    return isfinite(a) && isfinite(b) && isfinite(c) && isfinite(std_dev) && std_dev > 0;
}

bool fc_push_position(float x, float y, float z, float std_dev)
{
    if (!finite_measurement(x, y, z, std_dev)) return false;
    /* Once braking in the local frame, never fuse a jumped camera frame during this landing. */
    if (failsafeEnabled && fc.flowLanding && fc.safetyState == FC_FAILSAFE_LAND) return false;
    bool qualified = isfinite(x) && isfinite(y) && isfinite(z) &&
        fabsf(x) <= 100.0f && fabsf(y) <= 100.0f && fabsf(z) <= 100.0f &&
        isfinite(std_dev) && std_dev > 0 && std_dev <= 10.0f;
    if (failsafeEnabled && !qualified) return false;
    positionMeasurement_t m = {.x = x, .y = y, .z = z, .stdDev = std_dev};
    if (!fcKalmanPushPosition(&m)) return false;
    if (qualified) {
        fc.havePosition = true;
        fc.positionTick = fc.tick;
    }
    return true;
}

bool fc_push_tof(float distance_m, float std_dev)
{
    if (!finite_measurement(distance_m, 0, 0, std_dev)) return false;
    float r22 = vertical_cosine();
    bool qualified = isfinite(distance_m) && distance_m >= 0 && distance_m <= FC_FAILSAFE_MAX_TOF_M &&
        isfinite(std_dev) && std_dev > 0 && std_dev <= 10.0f && isfinite(r22) && r22 >= 0.5f;
    /* Do not impose the supervisor's narrower operating envelope on legacy EKF input. */
    if (failsafeEnabled && !qualified) return false;
    tofMeasurement_t m = {.timestamp = fc.tick, .distance = distance_m, .stdDev = std_dev};
    if (!fcKalmanPushTof(&m)) return false;
    if (qualified) {
        fc.haveTof = true;
        fc.tofTick = fc.tick;
        fc.tofHeight = distance_m * fminf(r22, 1.0f);
    }
    return true;
}

bool fc_push_flow(float dpixel_x, float dpixel_y, float dt, float std_dev)
{
    if (!finite_measurement(dpixel_x, dpixel_y, dt, std_dev) || dt <= 0) return false;
    bool qualified = isfinite(dpixel_x) && isfinite(dpixel_y) && fabsf(dpixel_x) <= 100 && fabsf(dpixel_y) <= 100 &&
        isfinite(dt) && dt >= 0.001f && dt <= 0.05f && isfinite(std_dev) && std_dev > 0 && std_dev <= 5 &&
        tof_fresh() && fc.tofHeight >= FC_FAILSAFE_FLOW_MIN_HEIGHT_M;
    if (failsafeEnabled && !qualified) return false;
    flowMeasurement_t m = {.timestamp = fc.tick, .dpixelx = dpixel_x, .dpixely = dpixel_y,
                           .stdDevX = std_dev, .stdDevY = std_dev, .dt = dt};
    if (!fcKalmanPushFlow(&m)) return false;
    if (qualified) { fc.haveFlow = true; fc.flowTick = fc.tick; }
    return true;
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
        float scale = (float)(fc.accScaleSum / ACC_SCALE_SAMPLES);
        /* ESP-FC Next: a dead/miswired accelerometer (0 g -> NaN after scaling) must not finish calibration. */
        if (isfinite(scale) && scale >= FC_ACC_SCALE_MIN && scale <= FC_ACC_SCALE_MAX) {
            fc.accScale = scale;
            fc.phase = FC_PHASE_READY;
        } else {
            fc.accScaleSum = 0;
            fc.accScaleCount = 0;
        }
    }
}

/* commander.c watchdog: level after 500 ms without setpoints, cut motors after 2 s. */
static void apply_watchdog(void)
{
    static const setpoint_t nullSetpoint;
    uint32_t age = fc.tick - fc.setpointTick;
    if (!fc.haveSetpoint || age > FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_SHUTDOWN_MS)) {
        fc.activeSetpoint = nullSetpoint;
    } else {
        fc.activeSetpoint = fc.setpoint;
        if (age > FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS)) {
            fc.activeSetpoint.mode.x = modeDisable;
            fc.activeSetpoint.mode.y = modeDisable;
            fc.activeSetpoint.mode.roll = modeAbs;
            fc.activeSetpoint.mode.pitch = modeAbs;
            fc.activeSetpoint.mode.yaw = modeVelocity;
            fc.activeSetpoint.attitude.roll = 0;
            fc.activeSetpoint.attitude.pitch = 0;
            fc.activeSetpoint.attitudeRate.yaw = 0;
            memset(&fc.activeSetpoint.velocity, 0, sizeof(fc.activeSetpoint.velocity));
            memset(&fc.activeSetpoint.acceleration, 0, sizeof(fc.activeSetpoint.acceleration));
        }
    }
}

static void safety_fault(uint8_t reason)
{
    fc.safetyReason |= reason;
    fc.safetyState = FC_FAILSAFE_FAULT;
    fc.armed = false;
    memset(&fc.activeSetpoint, 0, sizeof(fc.activeSetpoint));
}

static void capture_hold(void)
{
    fc.holdX = fc.state.position.x;
    fc.holdY = fc.state.position.y;
    fc.holdZ = fc.state.position.z;
    fc.holdYaw = fc.state.attitude.yaw;
    fc.holdTick = fc.tick;
    positionControllerResetAllPID();
}

static void start_land(void)
{
    fc.safetyState = FC_FAILSAFE_LAND;
    fc.landTick = fc.tick;
    fc.landXYDisabled = !fc.posFresh;
}

static void apply_failsafe(const float gyro[3], const float acc[3])
{
    fc.posFresh = position_fresh();
    fc.tofFresh = tof_fresh();
    fc.flowFresh = flow_fresh();
    fc.flowBrake = 0;
    if (fc.safetyState >= FC_FAILSAFE_LANDED) {
        fc.armed = false; /* Terminal latches cannot be bypassed by disabling this option. */
        memset(&fc.activeSetpoint, 0, sizeof(fc.activeSetpoint));
        return;
    }
    if (!failsafeEnabled || !fc.armed || fc.emergencyStop || fc.tumbled) return;

    bool valid = state_valid() && fcKalmanResetCount() == fc.estimatorResets;
    for (unsigned i = 0; i < 3; i++) valid &= isfinite(gyro[i]) && isfinite(acc[i]);
    if (!valid) {
        safety_fault(FC_FAILSAFE_REASON_INVALID_STATE);
        return;
    }
    bool commandFresh = fc.haveSetpoint &&
        (uint32_t)(fc.tick - fc.setpointTick) <= FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS);
    if (fc.safetyState < FC_FAILSAFE_LAND) {
        if (!fc.haveSetpoint) {
            /* A 100 Hz commander may deliver its first point after arming. Grace is thrust-free. */
            if ((uint32_t)(fc.tick - fc.armTick) > FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS))
                fc.armed = false;
            memset(&fc.activeSetpoint, 0, sizeof(fc.activeSetpoint));
            return;
        }
        /* Never turn a missing/stale ground command into an autonomous takeoff. */
        if (!fc.airborne && (!commandFresh || !fc.posFresh)) {
            fc.armed = false;
            memset(&fc.activeSetpoint, 0, sizeof(fc.activeSetpoint));
            return;
        }
        if (!fc.posFresh) {
            fc.safetyReason |= FC_FAILSAFE_REASON_TRACK;
            if (fc.safetyState == FC_FAILSAFE_NORMAL) capture_hold();
            start_land();
        } else if (!commandFresh) {
            fc.safetyReason |= FC_FAILSAFE_REASON_CMD;
            if (fc.safetyState == FC_FAILSAFE_NORMAL) {
                capture_hold();
                fc.safetyState = FC_FAILSAFE_HOLD;
            } else if ((uint32_t)(fc.tick - fc.holdTick) >= FC_MS_TO_TICKS(FC_FAILSAFE_HOLD_MS)) {
                start_land();
            }
        } else if (fc.safetyState == FC_FAILSAFE_HOLD) {
            fc.safetyState = FC_FAILSAFE_NORMAL;
        }
    }
    if (fc.safetyState == FC_FAILSAFE_NORMAL) return;

    /* Build from zero every tick: no mission velocity, acceleration or attitude-rate feedforward. */
    memset(&fc.activeSetpoint, 0, sizeof(fc.activeSetpoint));
    fc.activeSetpoint.mode.x = fc.activeSetpoint.mode.y = fc.activeSetpoint.mode.z = modeAbs;
    fc.activeSetpoint.mode.yaw = modeAbs;
    fc.activeSetpoint.position.x = fc.holdX;
    fc.activeSetpoint.position.y = fc.holdY;
    fc.activeSetpoint.position.z = fc.holdZ;
    fc.activeSetpoint.attitude.yaw = fc.holdYaw;
    if (fc.safetyState == FC_FAILSAFE_HOLD) return;

    if (!commandFresh) fc.safetyReason |= FC_FAILSAFE_REASON_CMD;
    if (!fc.posFresh) {
        fc.safetyReason |= FC_FAILSAFE_REASON_TRACK;
        fc.landXYDisabled = true; /* Never reacquire a potentially jumped camera reference. */
    }
    if (fc.landXYDisabled) {
        fc.activeSetpoint.mode.x = fc.activeSetpoint.mode.y = modeDisable;
        fc.activeSetpoint.mode.roll = fc.activeSetpoint.mode.pitch = modeAbs;
        if (fc.flowFresh) {
            /* Zero world-frame velocity, not a stale absolute position or unaided dead reckoning. */
            fc.activeSetpoint.mode.x = fc.activeSetpoint.mode.y = modeVelocity;
            fc.flowBrake = 1;
            fc.flowLanding = true;
        }
    }
    if (!fc.tofFresh && !fc.posFresh) {
        safety_fault(FC_FAILSAFE_REASON_HEIGHT);
        return;
    }
    fc.height = fc.tofFresh ? fc.tofHeight : fc.state.position.z - fc.groundZ;
    if ((uint32_t)(fc.tick - fc.landTick) >= FC_MS_TO_TICKS(FC_FAILSAFE_LAND_TIMEOUT_MS)) {
        safety_fault(FC_FAILSAFE_REASON_TIMEOUT);
        return;
    }
    fc.activeSetpoint.mode.z = modeVelocity;
    fc.activeSetpoint.velocity.z = fc.height < FC_FAILSAFE_SLOW_HEIGHT_M ?
        -FC_FAILSAFE_SLOW_DESCENT_MPS : -FC_FAILSAFE_DESCENT_MPS;
    if (fc.height <= FC_FAILSAFE_TOUCHDOWN_HEIGHT_M &&
        fabsf(fc.state.velocity.z) <= FC_FAILSAFE_TOUCHDOWN_SPEED_MPS) {
        if (!fc.touchdownPending) {
            fc.touchdownTick = fc.tick;
            fc.touchdownPending = true;
        }
        if ((uint32_t)(fc.tick - fc.touchdownTick) >= FC_MS_TO_TICKS(FC_FAILSAFE_TOUCHDOWN_MS)) {
            fc.safetyState = FC_FAILSAFE_LANDED;
            fc.armed = false;
            memset(&fc.activeSetpoint, 0, sizeof(fc.activeSetpoint));
        }
    } else {
        fc.touchdownPending = false;
    }
}

/* ESP-FC Next: a non-finite or out-of-range IMU sample never reaches calibration, LPF or EKF. */
static bool imu_sample(const float gyro_dps[3], const float acc_g[3])
{
    bool ok = true;
    for (int i = 0; i < 3; i++) {
        ok &= isfinite(gyro_dps[i]) && fabsf(gyro_dps[i]) <= FC_IMU_MAX_GYRO_DPS &&
              isfinite(acc_g[i]) && fabsf(acc_g[i]) <= FC_IMU_MAX_ACC_G;
    }
    if (ok) {
        memcpy(fc.goodGyro, gyro_dps, sizeof(fc.goodGyro));
        memcpy(fc.goodAcc, acc_g, sizeof(fc.goodAcc));
        fc.imuBadTicks = 0;
    } else if (fc.imuBadTicks < UINT32_MAX) {
        fc.imuBadTicks++;
    }
    return fc.imuBadTicks <= FC_MS_TO_TICKS(FC_IMU_BAD_MAX_MS);
}

void fc_step(const float gyro_raw[3], const float acc_raw[3])
{
    fc.tick++;
    bool imuOk = imu_sample(gyro_raw, acc_raw);
    const float *gyro_dps = fc.goodGyro, *acc_g = fc.goodAcc;
    if (!imuOk && fc.armed) {
        if (failsafeEnabled) safety_fault(FC_FAILSAFE_REASON_INVALID_STATE);
        fc.armed = false;
    }

    if (fc.phase == FC_PHASE_CALIBRATING && fc.imuBadTicks == 0) {
        calibrate(gyro_dps, acc_g);
    }
    filter_gyro(gyro_dps);
    for (int i = 0; i < 3; i++) {
        fc.sensors.gyro.axis[i] = lpf2pApply(&fc.gyroLpf[i], fc.notchGyro[i]);
        fc.sensors.acc.axis[i] = lpf2pApply(&fc.accLpf[i], acc_g[i] / fc.accScale);
    }
    fc.sensors.interruptTimestamp = usecTimestamp();

    if (fc.phase != FC_PHASE_READY) {
        powerStop();
        return;
    }

    control_t appliedControl = fc.control;
    if (!fc.armed || fc.emergencyStop) appliedControl.thrust = 0;
    fcKalmanUpdate(&fc.state, &fc.sensors, &appliedControl, fc.tick);
    /*
     * Anti-windup while landed: until lift-off the vehicle cannot follow the setpoint, so position and
     * velocity integrators would wind up and overshoot after lift-off. Proportional terms still act.
     */
    if (fc.armed && !fc.airborne) {
        if (fc.state.position.z > fc.groundZ + FC_LIFTOFF_HEIGHT_M) {
            fc.airborne = true;
        } else {
            positionControllerResetAllPID();
        }
    }

    apply_watchdog();
    apply_failsafe(gyro_dps, acc_g);

    /* Tumble protection also runs during the thrust-free first-command grace period. */
    if (fc.armed && tilt_deg() > FC_TUMBLE_TILT_DEG) {
        if (++fc.tumbleTicks >= FC_MS_TO_TICKS(FC_TUMBLE_TIME_MS)) {
            fc.tumbled = true;
            fc.armed = false;
        }
    } else {
        fc.tumbleTicks = 0;
    }
    if (!fc.armed || fc.emergencyStop || (failsafeEnabled && !fc.haveSetpoint)) {
        controllerPidReset(&fc.state);
        memset(&fc.control, 0, sizeof(fc.control));
        powerStop();
        return;
    }

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
LOG_ADD(LOG_UINT8, tumbled, &fc.tumbled)
LOG_ADD(LOG_UINT8, airborne, &fc.airborne)
LOG_ADD(LOG_UINT8, phase, &fc.phase)
LOG_ADD(LOG_FLOAT, biasX, &fc.gyroBias[0])
LOG_ADD(LOG_FLOAT, biasY, &fc.gyroBias[1])
LOG_ADD(LOG_FLOAT, biasZ, &fc.gyroBias[2])
LOG_ADD(LOG_FLOAT, accScale, &fc.accScale)
LOG_GROUP_STOP(fcStep)

LOG_GROUP_START(fcSafety)
LOG_ADD(LOG_UINT8, flowFresh, &fc.flowFresh)
LOG_ADD(LOG_UINT8, flowBrake, &fc.flowBrake)
LOG_ADD(LOG_UINT8, state, &fc.safetyState)
LOG_ADD(LOG_UINT8, reason, &fc.safetyReason)
LOG_ADD(LOG_UINT8, posFresh, &fc.posFresh)
LOG_ADD(LOG_UINT8, tofFresh, &fc.tofFresh)
LOG_ADD(LOG_FLOAT, holdX, &fc.holdX)
LOG_ADD(LOG_FLOAT, holdY, &fc.holdY)
LOG_ADD(LOG_FLOAT, holdZ, &fc.holdZ)
LOG_ADD(LOG_FLOAT, height, &fc.height)
LOG_GROUP_STOP(fcSafety)

LOG_GROUP_START(fcGyro)
LOG_ADD(LOG_FLOAT, rawX, &fc.rawGyro[0])
LOG_ADD(LOG_FLOAT, notchX, &fc.notchGyro[0])
LOG_ADD(LOG_FLOAT, filteredX, &fc.sensors.gyro.x)
LOG_ADD(LOG_UINT8, rpmFresh, &fc.rpmActive)
LOG_GROUP_STOP(fcGyro)
