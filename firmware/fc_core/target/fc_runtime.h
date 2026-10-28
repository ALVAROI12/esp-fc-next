/*
 * fc_runtime: target-only glue that runs fc_step inside ESP-Drone's stabilizer task (CONFIG_ESPFC_FC_STEP).
 * Measurement producers in other tasks hand off through one FreeRTOS queue; fc_core itself stays RTOS-free.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

void fcRuntimeInit(void);

/* Safe from any task; false when the queue is full or not initialised. */
bool fcRuntimeEnqueuePosition(const positionMeasurement_t *pos);
bool fcRuntimeEnqueuePose(const poseMeasurement_t *pose); /* position only; orientation is dropped */
bool fcRuntimeEnqueueTof(const tofMeasurement_t *tof);
bool fcRuntimeEnqueueFlow(const flowMeasurement_t *flow);

/* One flight-loop tick on the stabilizer task: drain measurements, forward new commander setpoints and the
 * arm/stop state, then fc_step on this tick's IMU sample. state receives fc_step's estimate for logging. */
void fcRuntimeStep(const sensorData_t *sensors, setpoint_t *setpoint, state_t *state, bool emergencyStop);
