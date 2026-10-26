/*
 * Tests for the extended Kalman filter core (modules/src/kalman_core.c), driven the
 * same way estimator_kalman.c does it: predict at 100 Hz, add process noise and
 * apply measurement updates every 1 ms loop, finalize after updates.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <math.h>
#include <string.h>

#include "kalman_core.h"
#include "kalman_supervisor.h"
#include "unity.h"

#define G 9.81f
#define LOOP_DT 0.001f
#define PREDICT_EVERY 10 /* 100 Hz prediction, as in estimator_kalman.c */

static kalmanCoreData_t kf;

void setUp(void)
{
    kalmanCoreInit(&kf);
}

void tearDown(void) {}

/*
 * Run the filter for `seconds` with a stationary, level IMU. If `pos` is non-NULL a
 * position measurement is fused every loop (like motion capture).
 */
static void run_stationary(float seconds, positionMeasurement_t *pos)
{
    Axis3f acc = {.x = 0, .y = 0, .z = G};
    Axis3f gyro = {.x = 0, .y = 0, .z = 0};
    int loops = (int)(seconds / LOOP_DT);

    for (int i = 1; i <= loops; i++) {
        bool updated = false;
        if (i % PREDICT_EVERY == 0) {
            kalmanCorePredict(&kf, 0.0f, &acc, &gyro, PREDICT_EVERY * LOOP_DT, false);
            updated = true;
        }
        kalmanCoreAddProcessNoise(&kf, LOOP_DT);
        if (pos) {
            kalmanCoreUpdateWithPosition(&kf, pos);
            updated = true;
        }
        if (updated) {
            kalmanCoreFinalize(&kf, (uint32_t)i);
        }
    }
}

static bool state_and_covariance_are_finite(void)
{
    for (int i = 0; i < KC_STATE_DIM; i++) {
        if (!isfinite(kf.S[i])) {
            return false;
        }
        for (int j = 0; j < KC_STATE_DIM; j++) {
            if (!isfinite(kf.P[i][j])) {
                return false;
            }
        }
    }
    return true;
}

static void test_init_state_is_origin_and_level(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.S[KC_STATE_X]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.S[KC_STATE_Y]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.S[KC_STATE_Z]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, kf.q[0]);
    TEST_ASSERT_TRUE(kalmanSupervisorIsStateWithinBounds(&kf));
}

static void test_position_fusion_converges_to_measurement(void)
{
    positionMeasurement_t pos = {.x = 1.0f, .y = -0.5f, .z = 0.8f, .stdDev = 0.01f};
    run_stationary(3.0f, &pos);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, kf.S[KC_STATE_X]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.5f, kf.S[KC_STATE_Y]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.8f, kf.S[KC_STATE_Z]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, kf.S[KC_STATE_PX]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, kf.S[KC_STATE_PY]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, kf.S[KC_STATE_PZ]);
}

static void test_position_fusion_shrinks_position_variance(void)
{
    float initial = kf.P[KC_STATE_X][KC_STATE_X];
    positionMeasurement_t pos = {.x = 0, .y = 0, .z = 0, .stdDev = 0.01f};
    run_stationary(1.0f, &pos);
    TEST_ASSERT_LESS_THAN_FLOAT(initial / 100.0f, kf.P[KC_STATE_X][KC_STATE_X]);
}

static void test_covariance_stays_symmetric_with_positive_diagonal(void)
{
    positionMeasurement_t pos = {.x = 0.3f, .y = 0.2f, .z = 0.5f, .stdDev = 0.05f};
    run_stationary(5.0f, &pos);

    TEST_ASSERT_TRUE(state_and_covariance_are_finite());
    for (int i = 0; i < KC_STATE_DIM; i++) {
        TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, kf.P[i][i]);
        for (int j = 0; j < i; j++) {
            TEST_ASSERT_FLOAT_WITHIN(1e-6f + 1e-4f * fabsf(kf.P[i][j]), kf.P[i][j], kf.P[j][i]);
        }
    }
}

static void test_dead_reckoning_without_measurements_stays_bounded(void)
{
    run_stationary(10.0f, NULL);
    TEST_ASSERT_TRUE(state_and_covariance_are_finite());
    TEST_ASSERT_TRUE(kalmanSupervisorIsStateWithinBounds(&kf));
    /* Uncertainty must grow when nothing is measured. */
    TEST_ASSERT_GREATER_THAN_FLOAT(1e-3f, kf.P[KC_STATE_X][KC_STATE_X]);
}

static void test_externalized_state_matches_internal_position(void)
{
    positionMeasurement_t pos = {.x = 0.4f, .y = 0.1f, .z = 1.2f, .stdDev = 0.01f};
    run_stationary(3.0f, &pos);

    state_t out;
    memset(&out, 0, sizeof(out));
    Axis3f acc_g = {.x = 0, .y = 0, .z = 1.0f};
    kalmanCoreExternalizeState(&kf, &out, &acc_g, 3000);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kf.S[KC_STATE_X], out.position.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kf.S[KC_STATE_Y], out.position.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kf.S[KC_STATE_Z], out.position.z);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, out.attitude.roll);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, out.attitude.pitch);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_state_is_origin_and_level);
    RUN_TEST(test_position_fusion_converges_to_measurement);
    RUN_TEST(test_position_fusion_shrinks_position_variance);
    RUN_TEST(test_covariance_stays_symmetric_with_positive_diagonal);
    RUN_TEST(test_dead_reckoning_without_measurements_stays_bounded);
    RUN_TEST(test_externalized_state_matches_internal_position);
    return UNITY_END();
}
