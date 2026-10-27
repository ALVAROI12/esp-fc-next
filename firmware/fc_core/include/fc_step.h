/*
 * fc_step: the complete flight loop as one synchronous call per 1 kHz tick.
 *
 * Replaces the FreeRTOS stabilizerTask + kalmanTask + sensor-task glue with a single
 * deterministic function so the same code runs in SITL (host) and on the ESP32-P4:
 *
 *   IMU sample -> gyro bias / accel scale calibration -> 2-pole LPF (as the sensor driver)
 *   -> Kalman estimator (synchronous port of estimator_kalman.c)
 *   -> setpoint + commander watchdog (as commander.c) -> controller -> mixer -> motors HAL
 *
 * No RTOS calls and no dynamic allocation. Not thread-safe: call from one context.
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

#define FC_STEP_RATE_HZ 1000 /* RATE_MAIN_LOOP */

/* Watchdog behaviour copied from commander.c */
#define FC_SETPOINT_TIMEOUT_STABILIZE_MS 500
#define FC_SETPOINT_TIMEOUT_SHUTDOWN_MS 2000

typedef enum {
    FC_PHASE_CALIBRATING = 0, /* keep still: gyro bias + accel scale are being measured */
    FC_PHASE_READY = 1,
} fc_phase_t;

/* (Re)initialize estimator, controller and mixer. Starts disarmed and calibrating. */
void fc_init(void);

/* Motors only spin when armed, calibrated, and not emergency-stopped. */
void fc_arm(bool armed);
bool fc_is_armed(void);
void fc_emergency_stop(void);
fc_phase_t fc_phase(void);

/* Setpoints refresh the commander watchdog, like a CRTP setpoint packet would. */
void fc_set_setpoint(const setpoint_t *setpoint);
void fc_set_position_setpoint(float x, float y, float z, float yaw_deg);

/* External measurements, queued and fused on the next fc_step (like the estimator queues). */
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
