/*
 * fc_step: the complete flight loop as one synchronous call per main-loop tick.
 *
 * Replaces the FreeRTOS stabilizerTask + kalmanTask + sensor-task glue with a single
 * deterministic function so the same code runs in SITL (host) and on the ESP32-P4:
 *
 *   IMU sample -> gyro bias / accel scale calibration -> optional RPM notch -> 2-pole LPF
 *   -> Kalman estimator (synchronous port of estimator_kalman.c)
 *   -> setpoint + commander watchdog (as commander.c) -> controller -> supervisor -> mixer -> motors HAL
 *
 * Loop rate is RATE_MAIN_LOOP (stock 1 kHz; the ESP-FC Next build uses 4 kHz). No RTOS calls and no
 * dynamic allocation. Not thread-safe: call from one context.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FC_STEP_RATE_HZ RATE_MAIN_LOOP
#define FC_MS_TO_TICKS(ms) ((uint32_t)(ms) * (uint32_t)FC_STEP_RATE_HZ / 1000u)

#ifndef FC_GYRO_LPF_HZ
#define FC_GYRO_LPF_HZ 80.0f /* GYRO_LPF_CUTOFF_FREQ in the ESP-Drone sensor driver */
#endif
#ifndef FC_ACCEL_LPF_HZ
#define FC_ACCEL_LPF_HZ 30.0f
#endif

/* Watchdog behaviour copied from commander.c */
#define FC_SETPOINT_TIMEOUT_STABILIZE_MS 500
#define FC_SETPOINT_TIMEOUT_SHUTDOWN_MS 2000

/* Supervisor limits */
#define FC_ARM_MAX_TILT_DEG 20.0f    /* refuse to arm unless roughly level */
#define FC_TUMBLE_TILT_DEG 70.0f     /* tilt beyond this ... */
#define FC_TUMBLE_TIME_MS 300        /* ... for this long while armed -> stop motors */
#define FC_LIFTOFF_HEIGHT_M 0.03f    /* position/velocity integrators held at zero until this high */

typedef enum {
    FC_PHASE_CALIBRATING = 0, /* keep still: gyro bias + accel scale are being measured */
    FC_PHASE_READY = 1,
} fc_phase_t;

/* Opt-in link-loss supervisor; configuration persists across fc_init(), history does not. */
typedef enum {
    FC_FAILSAFE_NORMAL = 0,
    FC_FAILSAFE_HOLD = 1,
    FC_FAILSAFE_LAND = 2,
    FC_FAILSAFE_LANDED = 3,
    FC_FAILSAFE_FAULT = 4,
} fc_failsafe_state_t;

#define FC_FAILSAFE_REASON_CMD 1u
#define FC_FAILSAFE_REASON_TRACK 2u
#define FC_FAILSAFE_REASON_HEIGHT 4u
#define FC_FAILSAFE_REASON_TIMEOUT 8u
#define FC_FAILSAFE_REASON_INVALID_STATE 16u
#define FC_FAILSAFE_POSITION_TIMEOUT_MS 300
#define FC_FAILSAFE_TOF_TIMEOUT_MS 200
#define FC_FAILSAFE_HOLD_MS 1500
#define FC_FAILSAFE_LAND_TIMEOUT_MS 20000
#define FC_FAILSAFE_TOUCHDOWN_MS 500
#define FC_FAILSAFE_DESCENT_MPS 0.25f
#define FC_FAILSAFE_SLOW_DESCENT_MPS 0.10f
#define FC_FAILSAFE_SLOW_HEIGHT_M 0.15f
#define FC_FAILSAFE_TOUCHDOWN_HEIGHT_M 0.04f
#define FC_FAILSAFE_TOUCHDOWN_SPEED_MPS 0.15f
#define FC_FAILSAFE_MAX_TOF_M 4.0f

/* Disarmed-only. Disabled in a fresh process, preserving the legacy commander watchdog.
 * LANDED/FAULT (and an interrupted LAND) require fc_init() before rearming. */
bool fc_set_failsafe_enabled(bool enabled);
fc_failsafe_state_t fc_failsafe_state(void);

/* (Re)initialize estimator, controller and mixer. Starts disarmed and calibrating. */
void fc_init(void);

/* Optional RPM gyro filter, configured while disarmed. Disabled by default. Q is in [1, 50]. */
bool fc_set_rpm_notch(bool enabled, float q);
/* Mechanical rpm from ESC telemetry (not eRPM); rejects nonfinite, negative or implausible packets.
 * No throttle-derived estimate is substituted when telemetry is missing. */
bool fc_push_rpm(const float rpm[4]);
#define FC_RPM_TIMEOUT_MS 100
#define FC_RPM_TUNE_MS 10

uint32_t fc_rate_hz(void);

/*
 * Arming: returns false (and stays disarmed) unless calibrated, roughly level, not tumbled and not
 * emergency-stopped. Disarming always succeeds.
 * Safe mode additionally requires fresh position/ToF. Its first command may arrive up to 500 ms
 * after arming; until then motor output remains zero. A stale ground command disarms, never lands.
 */
bool fc_arm(bool armed);
bool fc_is_armed(void);
void fc_emergency_stop(void);
bool fc_is_tumbled(void);
fc_phase_t fc_phase(void);

/* Setpoints refresh the commander watchdog, like a CRTP setpoint packet would. */
void fc_set_setpoint(const setpoint_t *setpoint);
void fc_set_position_setpoint(float x, float y, float z, float yaw_deg);
/*
 * Full trajectory point (world frame): position, velocity and acceleration. Velocity and acceleration are
 * used as feedforward when posCtlPid.velFF / posCtlPid.accFF > 0.
 */
void fc_set_trajectory_setpoint(float x, float y, float z, float vx, float vy, float vz,
                                float ax, float ay, float az, float yaw_deg);

/* External measurements, queued and fused on the next fc_step (like the estimator queues).
 * Only accepted, finite plausible position/ToF packets refresh supervisor receipt freshness.
 * Safety-mode ToF accepts ground distance zero through 4 m, with estimated cosine >= 0.5.
 * Disabled mode preserves legacy enqueueing; unqualified packets never refresh safety evidence.
 * Fresh receipt cannot detect a frozen/replayed sensor or prove estimator accuracy. */
bool fc_push_position(float x, float y, float z, float std_dev);
bool fc_push_tof(float distance_m, float std_dev);
bool fc_push_flow(float dpixel_x, float dpixel_y, float dt, float std_dev);

/* One loop iteration. gyro in deg/s, acc in g, both in the body frame (x fwd, y left, z up). */
void fc_step(const float gyro_dps[3], const float acc_g[3]);

uint32_t fc_tick(void);
const state_t *fc_state(void);
const control_t *fc_control(void);
const setpoint_t *fc_active_setpoint(void);

/*
 * Estimated state as plain floats: x y z vx vy vz (world, m, m/s), roll pitch yaw (deg, ZYX, pitch as a
 * right-handed rotation about body +y, i.e. -state.attitude.pitch which uses the legacy CF2 sign), qw qx qy qz.
 */
void fc_get_state_array(float out[13]);

#ifdef __cplusplus
}
#endif
