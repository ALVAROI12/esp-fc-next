/*
 * Synchronous Kalman estimator: the logic of estimator_kalman.c's kalmanTask without the task,
 * queues or mutexes. Called once per fc_step tick.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

void fcKalmanInit(void);
void fcKalmanUpdate(state_t *state, const sensorData_t *sensors, const control_t *control, uint32_t tick);

bool fcKalmanPushPosition(const positionMeasurement_t *m);
bool fcKalmanPushTof(const tofMeasurement_t *m);
bool fcKalmanPushFlow(const flowMeasurement_t *m);

/* Number of estimator resets triggered by the supervisor since fcKalmanInit. */
uint32_t fcKalmanResetCount(void);
