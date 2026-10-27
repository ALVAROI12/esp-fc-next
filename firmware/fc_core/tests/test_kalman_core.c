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

/* Flow sensor constants, copied from kalmanCoreUpdateWithFlow() in kalman_core.c. */
#define NPIX 30.0f
#define THETAPIX (4.2f * (float)M_PI / 180.0f)
#define FLOW_EVERY 10 /* 100 Hz flow samples */
#define FLOW_DT (FLOW_EVERY * LOOP_DT)

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

/*
 * True motion for the flow tests: level attitude at height h, body velocity ramping
 * linearly from rest to (vx, vy) over ramp_s seconds (0 = moving from the start), and a
 * body rate omega_y_dps that only the flow sensor sees.
 */
typedef struct {
    float h;
    float vx, vy;
    float ramp_s;
    float omega_y_dps;
} flow_scenario_t;

static float ramp_fraction(const flow_scenario_t *sc, float t)
{
    return (sc->ramp_s <= 0.0f || t >= sc->ramp_s) ? 1.0f : t / sc->ramp_s;
}

/*
 * Same as run_stationary, but fuses a ToF range every loop (if `tof` is non-NULL) and, if
 * `sc` is non-NULL, a flow sample every FLOW_EVERY loops generated from the scenario with
 * the flow model of kalmanCoreUpdateWithFlow(). The accelerometer reports the ramp's
 * acceleration so the IMU agrees with the flow. The predict step always sees zero gyro.
 */
static void run_level_tof_flow(float seconds, tofMeasurement_t *tof, const flow_scenario_t *sc)
{
    Axis3f gyro = {.x = 0, .y = 0, .z = 0};
    int loops = (int)(seconds / LOOP_DT);

    for (int i = 1; i <= loops; i++) {
        float t = i * LOOP_DT;
        bool updated = false;
        if (i % PREDICT_EVERY == 0) {
            Axis3f acc = {.x = 0, .y = 0, .z = G};
            float t_mid = t - 0.5f * PREDICT_EVERY * LOOP_DT;
            if (sc && sc->ramp_s > 0.0f && t_mid < sc->ramp_s) {
                acc.x = sc->vx / sc->ramp_s;
                acc.y = sc->vy / sc->ramp_s;
            }
            kalmanCorePredict(&kf, 0.0f, &acc, &gyro, PREDICT_EVERY * LOOP_DT, false);
            updated = true;
        }
        kalmanCoreAddProcessNoise(&kf, LOOP_DT);
        if (tof) {
            kalmanCoreUpdateWithTof(&kf, tof);
            updated = true;
        }
        if (sc && i % FLOW_EVERY == 0) {
            /* Mean velocity over the accumulation window [t - FLOW_DT, t]. */
            float k = ramp_fraction(sc, t - 0.5f * FLOW_DT);
            float omega_y = sc->omega_y_dps * (float)M_PI / 180.0f;
            Axis3f flow_gyro_dps = {.x = 0.0f, .y = sc->omega_y_dps, .z = 0.0f};
            flowMeasurement_t flow = {
                .dpixelx = FLOW_DT * NPIX / THETAPIX * (k * sc->vx / sc->h - 1.25f * omega_y),
                .dpixely = FLOW_DT * NPIX / THETAPIX * (k * sc->vy / sc->h),
                .stdDevX = 2.0f,
                .stdDevY = 2.0f,
                .dt = FLOW_DT,
            };
            kalmanCoreUpdateWithFlow(&kf, &flow, &flow_gyro_dps);
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

static void test_tof_fusion_converges_height(void)
{
    float initial = kf.P[KC_STATE_Z][KC_STATE_Z];
    tofMeasurement_t tof = {.distance = 0.8f, .stdDev = 0.005f};
    run_level_tof_flow(3.0f, &tof, NULL);

    TEST_ASSERT_TRUE(state_and_covariance_are_finite());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.8f, kf.S[KC_STATE_Z]);
    TEST_ASSERT_LESS_THAN_FLOAT(initial / 100.0f, kf.P[KC_STATE_Z][KC_STATE_Z]);
}

static void assert_tof_update_is_ignored(void)
{
    float S_before[KC_STATE_DIM];
    float P_before[KC_STATE_DIM][KC_STATE_DIM];
    memcpy(S_before, kf.S, sizeof(S_before));
    memcpy(P_before, kf.P, sizeof(P_before));

    tofMeasurement_t tof = {.distance = 0.8f, .stdDev = 0.005f};
    kalmanCoreUpdateWithTof(&kf, &tof);

    TEST_ASSERT_EQUAL_MEMORY(S_before, kf.S, sizeof(S_before));
    TEST_ASSERT_EQUAL_MEMORY(P_before, kf.P, sizeof(P_before));
}

static void test_tof_ignored_when_inverted(void)
{
    /* Upside down (180 deg roll): finalize rebuilds R from q, giving R[2][2] = -1. */
    kf.q[0] = 0.0f;
    kf.q[1] = 1.0f;
    kf.q[2] = 0.0f;
    kf.q[3] = 0.0f;
    kalmanCoreFinalize(&kf, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, kf.R[2][2]);
    assert_tof_update_is_ignored();

    /* On its side (90 deg roll): R[2][2] = 0, below the 0.1 reliability threshold. */
    kf.q[0] = sqrtf(0.5f);
    kf.q[1] = sqrtf(0.5f);
    kalmanCoreFinalize(&kf, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, kf.R[2][2]);
    assert_tof_update_is_ignored();
}

/*
 * Accelerate from rest to (0.3, -0.2) m/s over 1 s, then cruise. The ramp matters: a
 * velocity step with no matching accelerometer reading contradicts the IMU and the tight
 * initial velocity prior, and the filter then explains the flow with a false ~1.4 deg
 * tilt that only decays through ROLLPITCH_ZERO_REVERSION (seconds), biasing velocity by
 * ~0.1 m/s. Only velocities and height are checked; the position keeps growing.
 */
static const flow_scenario_t cruise = {.h = 1.0f, .vx = 0.3f, .vy = -0.2f, .ramp_s = 1.0f};

static void test_flow_and_tof_estimate_body_velocity(void)
{
    tofMeasurement_t tof = {.distance = 1.0f, .stdDev = 0.005f};
    run_level_tof_flow(3.0f, &tof, &cruise);

    TEST_ASSERT_TRUE(state_and_covariance_are_finite());
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.3f, kf.S[KC_STATE_PX]);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, -0.2f, kf.S[KC_STATE_PY]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, kf.S[KC_STATE_Z]);
}

static void test_flow_rotation_compensation(void)
{
    /*
     * Pure rotation about body y at +20 deg/s, no translation. Simplification: only the
     * flow update sees the rotation; the predict step gets zero gyro so the attitude
     * stays level and the ToF/flow geometry does not change during the test.
     */
    flow_scenario_t rotation = {.h = 1.0f, .omega_y_dps = 20.0f};
    tofMeasurement_t tof = {.distance = 1.0f, .stdDev = 0.005f};
    run_level_tof_flow(2.0f, &tof, &rotation);

    TEST_ASSERT_TRUE(state_and_covariance_are_finite());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, kf.S[KC_STATE_PX]);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, kf.S[KC_STATE_PY]);

    /*
     * Without the gyro the same pixels read as backwards motion (-1.25 * omega_y * h); a
     * single 2-pixel-stdDev sample moves the estimate by only a few mm/s.
     */
    float before = kf.S[KC_STATE_PX];
    float omega_y = rotation.omega_y_dps * (float)M_PI / 180.0f;
    flowMeasurement_t flow = {
        .dpixelx = FLOW_DT * NPIX / THETAPIX * (-1.25f * omega_y),
        .stdDevX = 2.0f,
        .stdDevY = 2.0f,
        .dt = FLOW_DT,
    };
    Axis3f no_gyro = {.x = 0, .y = 0, .z = 0};
    kalmanCoreUpdateWithFlow(&kf, &flow, &no_gyro);
    TEST_ASSERT_LESS_THAN_FLOAT(before - 0.003f, kf.S[KC_STATE_PX]);
}

static void test_flow_and_tof_keep_covariance_healthy(void)
{
    tofMeasurement_t tof = {.distance = 1.0f, .stdDev = 0.005f};
    run_level_tof_flow(10.0f, &tof, &cruise);

    TEST_ASSERT_TRUE(state_and_covariance_are_finite());
    for (int i = 0; i < KC_STATE_DIM; i++) {
        TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, kf.P[i][i]);
        for (int j = 0; j < i; j++) {
            TEST_ASSERT_FLOAT_WITHIN(1e-6f + 1e-4f * fabsf(kf.P[i][j]), kf.P[i][j], kf.P[j][i]);
        }
    }
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
    RUN_TEST(test_tof_fusion_converges_height);
    RUN_TEST(test_tof_ignored_when_inverted);
    RUN_TEST(test_flow_and_tof_estimate_body_velocity);
    RUN_TEST(test_flow_rotation_compensation);
    RUN_TEST(test_flow_and_tof_keep_covariance_healthy);
    return UNITY_END();
}
