/*
 * Synchronous port of the Crazyflie estimator_kalman.c task loop (see fc_kalman.h).
 * Rates and thresholds are the same as estimator_kalman.c.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_kalman.h"

#include <string.h>

#include "cf_math.h"
#include "kalman_core.h"
#include "kalman_supervisor.h"
#include "log.h"
#include "physicalConstants.h"

#define TICKS_PER_SECOND 1000.0f
#define PREDICT_EVERY_TICKS 10 /* PREDICT_RATE = 100 Hz */
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE * 60.0f / (CF_MASS * 1000.0f) / 65536.0f)
#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE * 0.1f)
#define IN_FLIGHT_TIME_THRESHOLD 500 /* ticks */

/* ---- fixed-size measurement queues (no dynamic allocation) ---- */

#define QUEUE_LEN 8
#define DEFINE_QUEUE(NAME, TYPE)                                                      \
    static struct { TYPE items[QUEUE_LEN]; uint8_t head, count; } NAME;               \
    static bool NAME##_push(const TYPE *m)                                            \
    {                                                                                 \
        if (NAME.count == QUEUE_LEN) return false;                                    \
        NAME.items[(NAME.head + NAME.count++) % QUEUE_LEN] = *m;                      \
        return true;                                                                  \
    }                                                                                 \
    static bool NAME##_pop(TYPE *m)                                                   \
    {                                                                                 \
        if (NAME.count == 0) return false;                                            \
        *m = NAME.items[NAME.head];                                                   \
        NAME.head = (uint8_t)((NAME.head + 1) % QUEUE_LEN);                           \
        NAME.count--;                                                                 \
        return true;                                                                  \
    }

DEFINE_QUEUE(posQueue, positionMeasurement_t)
DEFINE_QUEUE(tofQueue, tofMeasurement_t)
DEFINE_QUEUE(flowQueue, flowMeasurement_t)

/* ---- estimator state ---- */

static kalmanCoreData_t coreData;
static Axis3f accAccumulator, gyroAccumulator;
static uint32_t accCount, gyroCount, thrustCount;
static float thrustAccumulator;
static uint32_t lastPrediction, nextPrediction, lastProcessNoise, lastFlightCmd;
static bool quadIsFlying;
static uint32_t resetCount;
static bool needsTickSync;

void fcKalmanInit(void)
{
    kalmanCoreInit(&coreData);
    accAccumulator = (Axis3f){.axis = {0}};
    gyroAccumulator = (Axis3f){.axis = {0}};
    accCount = gyroCount = thrustCount = 0;
    thrustAccumulator = 0.0f;
    quadIsFlying = false;
    lastFlightCmd = 0;
    posQueue.count = tofQueue.count = flowQueue.count = 0;
    needsTickSync = true;
}

uint32_t fcKalmanResetCount(void) { return resetCount; }

bool fcKalmanPushPosition(const positionMeasurement_t *m) { return posQueue_push(m); }
bool fcKalmanPushTof(const tofMeasurement_t *m) { return tofQueue_push(m); }
bool fcKalmanPushFlow(const flowMeasurement_t *m) { return flowQueue_push(m); }

static bool predict(uint32_t tick, float dt)
{
    if (gyroCount == 0 || accCount == 0 || thrustCount == 0) {
        return false;
    }
    Axis3f gyro = {.x = gyroAccumulator.x * DEG_TO_RAD / gyroCount,
                   .y = gyroAccumulator.y * DEG_TO_RAD / gyroCount,
                   .z = gyroAccumulator.z * DEG_TO_RAD / gyroCount};
    Axis3f acc = {.x = accAccumulator.x * GRAVITY_MAGNITUDE / accCount,
                  .y = accAccumulator.y * GRAVITY_MAGNITUDE / accCount,
                  .z = accAccumulator.z * GRAVITY_MAGNITUDE / accCount};
    float thrust = thrustAccumulator * CONTROL_TO_ACC / thrustCount;

    accAccumulator = (Axis3f){.axis = {0}};
    gyroAccumulator = (Axis3f){.axis = {0}};
    thrustAccumulator = 0.0f;
    accCount = gyroCount = thrustCount = 0;

    if (thrust > IN_FLIGHT_THRUST_THRESHOLD) {
        lastFlightCmd = tick;
    }
    quadIsFlying = (tick - lastFlightCmd) < IN_FLIGHT_TIME_THRESHOLD;

    kalmanCorePredict(&coreData, thrust, &acc, &gyro, dt, quadIsFlying);
    return true;
}

void fcKalmanUpdate(state_t *state, const sensorData_t *sensors, const control_t *control, uint32_t tick)
{
    if (needsTickSync) {
        lastPrediction = nextPrediction = lastProcessNoise = tick;
        lastFlightCmd = tick - IN_FLIGHT_TIME_THRESHOLD;
        needsTickSync = false;
    }

    accAccumulator.x += sensors->acc.x;
    accAccumulator.y += sensors->acc.y;
    accAccumulator.z += sensors->acc.z;
    accCount++;
    gyroAccumulator.x += sensors->gyro.x;
    gyroAccumulator.y += sensors->gyro.y;
    gyroAccumulator.z += sensors->gyro.z;
    gyroCount++;
    thrustAccumulator += control->thrust;
    thrustCount++;

    if (coreData.resetEstimation) {
        fcKalmanInit();
        resetCount++;
        lastPrediction = nextPrediction = lastProcessNoise = tick;
        needsTickSync = false;
    }

    bool doneUpdate = false;

    if (tick >= nextPrediction) {
        float dt = (tick - lastPrediction) / TICKS_PER_SECOND;
        if (dt > 0.0f && predict(tick, dt)) {
            lastPrediction = tick;
            doneUpdate = true;
        }
        nextPrediction = tick + PREDICT_EVERY_TICKS;
    }

    float pnDt = (tick - lastProcessNoise) / TICKS_PER_SECOND;
    if (pnDt > 0.0f) {
        kalmanCoreAddProcessNoise(&coreData, pnDt);
        lastProcessNoise = tick;
    }

    tofMeasurement_t tof;
    while (tofQueue_pop(&tof)) {
        kalmanCoreUpdateWithTof(&coreData, &tof);
        doneUpdate = true;
    }
    positionMeasurement_t pos;
    while (posQueue_pop(&pos)) {
        kalmanCoreUpdateWithPosition(&coreData, &pos);
        doneUpdate = true;
    }
    flowMeasurement_t flow;
    while (flowQueue_pop(&flow)) {
        kalmanCoreUpdateWithFlow(&coreData, &flow, &sensors->gyro);
        doneUpdate = true;
    }

    if (doneUpdate) {
        kalmanCoreFinalize(&coreData, tick);
        if (!kalmanSupervisorIsStateWithinBounds(&coreData)) {
            coreData.resetEstimation = true;
        }
    }

    kalmanCoreExternalizeState(&coreData, state, &sensors->acc, tick);
}

LOG_GROUP_START(fcKalman)
LOG_ADD(LOG_UINT32, resets, &resetCount)
LOG_ADD(LOG_UINT8, flying, &quadIsFlying)
LOG_GROUP_STOP(fcKalman)
