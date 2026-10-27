/* Portable supervisor: measurement evidence, transitions and motor containment.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>

#include "fc_host.h"
#include "fc_step.h"
#include "fc_vars.h"
#include "unity.h"

static const float LEVEL[3] = {0, 0, 1};
static const float STILL[3] = {0, 0, 0};

static void step(const float gyro[3], const float acc[3])
{
    fc_host_advance_time_us(1000000u / FC_STEP_RATE_HZ);
    fc_step(gyro, acc);
}

static void drive(unsigned ms, bool position, bool tof, bool command, float z)
{
    for (uint32_t i = 0; i < FC_MS_TO_TICKS(ms); i++) {
        if (i % FC_MS_TO_TICKS(10) == 0) {
            if (position) TEST_ASSERT_TRUE(fc_push_position(0.3f, -0.2f, z, 0.01f));
            if (tof) TEST_ASSERT_TRUE(fc_push_tof(z, 0.01f));
            if (command) fc_set_position_setpoint(0.3f, -0.2f, 1, 0);
        }
        step(STILL, LEVEL);
    }
}

static double safety_log(const char *name)
{
    double value;
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcSafety", name, &value));
    return value;
}

static bool motors_on(void)
{
    for (unsigned m = 0; m < 4; m++) if (fc_host_motor_ratio(m)) return true;
    return false;
}

static void ground(void)
{
    for (unsigned i = 0; i < 1024 + 200; i++) step(STILL, LEVEL);
    drive(300, true, true, false, 0);
    TEST_ASSERT_EQUAL(FC_PHASE_READY, fc_phase());
}

static void hover(void)
{
    ground();
    TEST_ASSERT_TRUE(fc_arm(true));
    /* Arm on the actual ground, then move the estimate to 1 m; do not fake the airborne flag. */
    for (unsigned i = 1; i <= 100; i++) drive(10, true, true, true, i * 0.01f);
    drive(1000, true, true, true, 1);
    TEST_ASSERT_TRUE(fc_state()->position.z > 0.9f);
    TEST_ASSERT_TRUE(fabsf(fc_state()->velocity.z) < 0.15f);
    TEST_ASSERT_TRUE(fc_is_armed());
    TEST_ASSERT_TRUE(motors_on());
}

void setUp(void)
{
    fc_host_set_log_level(1);
    fc_init();
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(true));
}
void tearDown(void) {}

static void test_invalid_position_is_not_accepted(void)
{
    TEST_ASSERT_FALSE(fc_push_position(NAN, 0, 0, 0.01f));
    TEST_ASSERT_FALSE(fc_push_position(0, INFINITY, 0, 0.01f));
    TEST_ASSERT_FALSE(fc_push_position(0, 0, 0, 0));
    TEST_ASSERT_FALSE(fc_push_position(101, 0, 0, 0.01f));
    TEST_ASSERT_FALSE(fc_push_position(0, 0, 0, -1));
    TEST_ASSERT_FALSE(fc_push_position(0, 0, 0, NAN));
}

static void test_invalid_tof_is_not_accepted(void)
{
    TEST_ASSERT_FALSE(fc_push_tof(-0.01f, 0.01f));
    TEST_ASSERT_FALSE(fc_push_tof(4.01f, 0.01f));
    TEST_ASSERT_FALSE(fc_push_tof(NAN, 0.01f));
    TEST_ASSERT_FALSE(fc_push_tof(1, INFINITY));
    TEST_ASSERT_FALSE(fc_push_tof(1, 0));
    TEST_ASSERT_FALSE(fc_push_tof(1, -1));
}

static void test_config_persists_but_history_and_measurements_clear_on_init(void)
{
    ground();
    TEST_ASSERT_TRUE(fc_arm(true));
    TEST_ASSERT_FALSE(fc_set_failsafe_enabled(false));
    fc_init();
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state());
    TEST_ASSERT_EQUAL(0, safety_log("reason"));
    for (unsigned i = 0; i < 1024 + 200; i++) step(STILL, LEVEL);
    TEST_ASSERT_FALSE(fc_arm(true)); /* enable persisted, input history did not */
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(false));
    TEST_ASSERT_TRUE(fc_arm(true)); /* opt-out retains legacy arming */
}

static void test_disabled_mode_preserves_legacy_tof_acceptance(void)
{
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(false));
    ground();
    for (uint32_t i = 0; i < FC_MS_TO_TICKS(350); i++) {
        float angle = 200.0f * (i + 1) / FC_STEP_RATE_HZ * 0.0174532925f;
        const float gyro[3] = {200, 0, 0};
        const float acc[3] = {0, sinf(angle), cosf(angle)};
        if (i % FC_MS_TO_TICKS(10) == 0)
            TEST_ASSERT_TRUE(fc_push_position(0, 0, 0, 0.01f));
        step(gyro, acc);
    }
    const quaternion_t *q = &fc_state()->attitudeQuaternion;
    float cosine = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    TEST_ASSERT_TRUE(cosine > 0.1f && cosine < 0.5f); /* legacy EKF accepts this tilt */
    TEST_ASSERT_TRUE(fc_push_tof(1, 0.01f));
    TEST_ASSERT_TRUE(fc_push_tof(5, 0.01f)); /* legacy estimator, not the 4 m safety policy */
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(true));
    TEST_ASSERT_FALSE(fc_push_tof(1, 0.01f));
    step(STILL, LEVEL);
    TEST_ASSERT_EQUAL(0, safety_log("tofFresh")); /* legacy acceptance is not safety evidence */
}

static void test_arm_requires_both_fresh_inputs_and_zero_tof_is_ground(void)
{
    for (unsigned i = 0; i < 1024 + 200; i++) step(STILL, LEVEL);
    TEST_ASSERT_FALSE(fc_arm(true));
    TEST_ASSERT_TRUE(fc_push_position(0, 0, 0, 0.01f));
    TEST_ASSERT_FALSE(fc_arm(true));
    TEST_ASSERT_TRUE(fc_push_tof(0, 0.01f));
    TEST_ASSERT_TRUE(fc_arm(true));
    fc_arm(false);
    drive(201, true, false, false, 0);
    TEST_ASSERT_FALSE(fc_arm(true));
    TEST_ASSERT_TRUE(fc_push_tof(0, 0.01f));
    TEST_ASSERT_TRUE(fc_arm(true));
}

static void test_rejected_packets_do_not_refresh_freshness(void)
{
    ground();
    drive(190, false, false, false, 0);
    TEST_ASSERT_FALSE(fc_push_position(NAN, 0, 0, 0.01f));
    TEST_ASSERT_FALSE(fc_push_tof(5, 0.01f));
    drive(120, false, false, false, 0);
    TEST_ASSERT_EQUAL(0, safety_log("posFresh"));
    TEST_ASSERT_EQUAL(0, safety_log("tofFresh"));
    TEST_ASSERT_FALSE(fc_arm(true));
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state()); /* disarmed loss is not a fault */
}

static void test_full_queues_cannot_refresh_receipt_age(void)
{
    /* While calibrating queues are not consumed. Fill them, then let the original receipts expire. */
    for (unsigned i = 0; i < 8; i++) {
        TEST_ASSERT_TRUE(fc_push_position(0, 0, 0, 0.01f));
        TEST_ASSERT_TRUE(fc_push_tof(0, 0.01f));
    }
    for (unsigned i = 0; i < 1024; i++) step(STILL, LEVEL);
    TEST_ASSERT_FALSE(fc_push_position(0, 0, 0, 0.01f));
    TEST_ASSERT_FALSE(fc_push_tof(0, 0.01f));
    for (unsigned i = 0; i < 200; i++) step(STILL, LEVEL);
    TEST_ASSERT_FALSE(fc_arm(true));
    TEST_ASSERT_EQUAL(0, safety_log("posFresh"));
    TEST_ASSERT_EQUAL(0, safety_log("tofFresh"));
}

static void test_rearm_after_disarm_in_hold_starts_clean(void)
{
    hover();
    for (unsigned i = 0; i < 510 && fc_failsafe_state() == FC_FAILSAFE_NORMAL; i++)
        drive(1, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_HOLD, fc_failsafe_state());
    TEST_ASSERT_TRUE(fc_arm(false));
    drive(50, true, true, false, 0);
    TEST_ASSERT_TRUE(fc_arm(true));
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state()); /* no stale HOLD pose/reason after rearm */
    TEST_ASSERT_EQUAL(0, safety_log("reason"));
}

static void test_hold_captures_estimate_not_stale_trajectory_and_can_cancel(void)
{
    hover();
    fc_set_trajectory_setpoint(4, 5, 6, 1, 2, 3, 4, 5, 6, 40);
    for (unsigned i = 0; i < 510 && fc_failsafe_state() == FC_FAILSAFE_NORMAL; i++)
        drive(1, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_HOLD, fc_failsafe_state());
    const setpoint_t *sp = fc_active_setpoint();
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, fc_state()->position.x, sp->position.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, fc_state()->position.y, sp->position.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, fc_state()->position.z, sp->position.z);
    TEST_ASSERT_EQUAL_FLOAT(fc_state()->attitude.yaw, sp->attitude.yaw);
    TEST_ASSERT_EQUAL_FLOAT(0, sp->velocity.x);
    TEST_ASSERT_EQUAL_FLOAT(0, sp->velocity.y);
    TEST_ASSERT_EQUAL_FLOAT(0, sp->velocity.z);
    TEST_ASSERT_EQUAL_FLOAT(0, sp->acceleration.x);
    TEST_ASSERT_EQUAL_FLOAT(0, sp->acceleration.y);
    TEST_ASSERT_EQUAL_FLOAT(0, sp->acceleration.z);
    const float x = sp->position.x, y = sp->position.y, z = sp->position.z;
    drive(100, true, true, false, 1);
    TEST_ASSERT_EQUAL_FLOAT(x, safety_log("holdX"));
    TEST_ASSERT_EQUAL_FLOAT(y, safety_log("holdY"));
    TEST_ASSERT_EQUAL_FLOAT(z, safety_log("holdZ"));
    TEST_ASSERT_EQUAL(FC_FAILSAFE_REASON_CMD, (unsigned)safety_log("reason"));
    fc_set_position_setpoint(9, 8, 7, 0);
    drive(1, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state());
    TEST_ASSERT_EQUAL_FLOAT(9, fc_active_setpoint()->position.x);
}

static void test_exact_command_hold_and_tracking_deadlines(void)
{
    hover();
    fc_set_position_setpoint(0.3f, -0.2f, 1, 0);
    drive(500, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state());
    step(STILL, LEVEL);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_HOLD, fc_failsafe_state());
    drive(1499, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_HOLD, fc_failsafe_state());
    drive(1, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());

    fc_init();
    hover();
    TEST_ASSERT_TRUE(fc_push_position(0.3f, -0.2f, 1, 0.01f));
    drive(300, false, true, true, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state());
    step(STILL, LEVEL);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
}

static void test_land_latches_and_ignores_recovered_commands(void)
{
    hover();
    drive(2100, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    const float x = safety_log("holdX"), y = safety_log("holdY");
    for (unsigned i = 0; i < 100; i++) {
        fc_set_position_setpoint(8, 9, 10, 90);
        drive(10, true, true, false, 1);
        TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
        TEST_ASSERT_EQUAL_FLOAT(x, fc_active_setpoint()->position.x);
        TEST_ASSERT_EQUAL_FLOAT(y, fc_active_setpoint()->position.y);
        TEST_ASSERT_EQUAL(modeVelocity, fc_active_setpoint()->mode.z);
        TEST_ASSERT_EQUAL_FLOAT(-0.25f, fc_active_setpoint()->velocity.z);
    }
    TEST_ASSERT_TRUE(motors_on());
}

static void test_tracking_only_loss_lands_and_never_reacquires_xy(void)
{
    hover();
    drive(320, false, true, true, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_EQUAL(FC_FAILSAFE_REASON_TRACK, (unsigned)safety_log("reason"));
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.y);
    TEST_ASSERT_EQUAL_FLOAT(0, fc_active_setpoint()->attitude.roll);
    TEST_ASSERT_EQUAL_FLOAT(0, fc_active_setpoint()->attitude.pitch);
    TEST_ASSERT_TRUE(fc_push_position(5, 4, 1, 0.01f)); /* reference returns jumped */
    drive(300, true, true, true, 1);
    TEST_ASSERT_EQUAL(1, safety_log("posFresh"));
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.y);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
}

static void test_combined_loss_uses_local_height_beyond_legacy_shutdown(void)
{
    hover();
    drive(3000, false, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_TRUE(fc_is_armed());
    TEST_ASSERT_TRUE(motors_on());
    TEST_ASSERT_EQUAL(0, safety_log("posFresh"));
    TEST_ASSERT_EQUAL(1, safety_log("tofFresh"));
    TEST_ASSERT_EQUAL_FLOAT(-0.25f, fc_active_setpoint()->velocity.z);
}

static void test_xy_loss_during_landing_is_permanent(void)
{
    hover();
    drive(2100, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.x);
    drive(320, false, true, false, 1);
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
    drive(200, true, true, true, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.y);
    TEST_ASSERT_EQUAL_FLOAT(0, fc_active_setpoint()->attitude.roll);
    TEST_ASSERT_EQUAL_FLOAT(0, fc_active_setpoint()->attitude.pitch);
}

static void test_all_height_loss_is_fault_not_success_and_cannot_rearm(void)
{
    hover();
    drive(320, false, false, true, 1);
    /* Airborne: bounded blind descent on the IMU-propagated estimate instead of a motor cut in the air. */
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_BITS_HIGH(FC_FAILSAFE_REASON_BLIND | FC_FAILSAFE_REASON_TRACK, (unsigned)safety_log("reason"));
    TEST_ASSERT_TRUE(motors_on());
    TEST_ASSERT_EQUAL(modeVelocity, fc_active_setpoint()->mode.z);
    TEST_ASSERT_EQUAL_FLOAT(-FC_FAILSAFE_DESCENT_MPS, fc_active_setpoint()->velocity.z);
    drive(FC_FAILSAFE_BLIND_MS, false, false, true, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_FAULT, fc_failsafe_state());
    TEST_ASSERT_BITS_HIGH(FC_FAILSAFE_REASON_HEIGHT | FC_FAILSAFE_REASON_TRACK, (unsigned)safety_log("reason"));
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_FALSE(motors_on());
    drive(1000, true, true, true, 0);
    TEST_ASSERT_FALSE(fc_arm(true));
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(false));
    TEST_ASSERT_FALSE(fc_arm(true)); /* changing config cannot bypass a terminal latch */
    TEST_ASSERT_FALSE(motors_on());
}

static void test_blind_descent_hands_back_to_tof_and_lands(void)
{
    hover();
    drive(320, false, false, true, 1); /* camera lost while above ToF range */
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    drive(1000, false, false, true, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    for (int i = 0; i < 100; i++) drive(10, false, true, false, 0.8f - i * 0.0077f); /* ToF back in range */
    drive(2000, false, true, false, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LANDED, fc_failsafe_state());
    TEST_ASSERT_BITS_LOW(FC_FAILSAFE_REASON_HEIGHT, (unsigned)safety_log("reason"));
    TEST_ASSERT_FALSE(motors_on());
}

static unsigned mean_motor_ratio(void)
{
    unsigned sum = 0;
    for (unsigned m = 0; m < 4; m++) sum += fc_host_motor_ratio(m);
    return sum / 4;
}

/* A z accelerometer bias the estimator cannot see (no height source) must not make blind descent climb:
 * motor output stays just below the hover thrust learned while aided, whatever the biased estimate says. */
static void test_blind_descent_thrust_stays_below_hover_despite_accel_bias(void)
{
    hover();
    const float biased[3] = {0, 0, 0.98f}; /* -0.02 g: the estimate believes it sinks faster than it does */
    unsigned maxRatio = 0, minRatio = 65535;
    for (uint32_t i = 0; i < FC_MS_TO_TICKS(3000); i++) {
        if (i % FC_MS_TO_TICKS(10) == 0) fc_set_position_setpoint(0.3f, -0.2f, 1, 0);
        step(STILL, biased);
        if (i < FC_MS_TO_TICKS(320) || !motors_on()) continue;
        unsigned r = mean_motor_ratio();
        if (r > maxRatio) maxRatio = r;
        if (r < minRatio) minRatio = r;
    }
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_BITS_HIGH(FC_FAILSAFE_REASON_BLIND, (unsigned)safety_log("reason"));
    double hoverRatio = safety_log("hoverThrust");
    TEST_ASSERT_TRUE(hoverRatio > 20000);
    TEST_ASSERT_LESS_OR_EQUAL_UINT(hoverRatio * (FC_FAILSAFE_SINK_THRUST_MAX + 0.002f), maxRatio);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(hoverRatio * (FC_FAILSAFE_SINK_THRUST_MIN - 0.002f), minRatio);
}

static void test_camera_height_fallback_and_slow_descent(void)
{
    hover();
    drive(2100, true, false, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_EQUAL(0, safety_log("tofFresh"));
    TEST_ASSERT_TRUE(motors_on());
    for (unsigned i = 100; i > 10; i--) drive(10, true, false, false, i * 0.01f);
    drive(500, true, false, false, 0.1f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_EQUAL_FLOAT(-0.10f, fc_active_setpoint()->velocity.z);
}

static void test_touchdown_requires_actual_low_height_low_speed_and_dwell(void)
{
    hover();
    drive(2100, true, true, false, 1);
    drive(1000, true, true, true, 0.1f); /* stationary, but 10 cm is NOT touchdown */
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_TRUE(motors_on());
    TEST_ASSERT_EQUAL_FLOAT(-0.10f, fc_active_setpoint()->velocity.z);
    /* Consistent ground observations let the real estimator settle, without a physics mock. */
    bool qualifying = false, sawFastGround = false;
    uint32_t qualifyingTick = 0, start = fc_tick();
    for (uint32_t i = 0; i < FC_MS_TO_TICKS(2000) && fc_is_armed(); i++) {
        if (i % FC_MS_TO_TICKS(10) == 0) {
            TEST_ASSERT_TRUE(fc_push_position(0.3f, -0.2f, 0, 0.01f));
            TEST_ASSERT_TRUE(fc_push_tof(0, 0.01f));
            fc_set_position_setpoint(0.3f, -0.2f, 1, 0);
        }
        step(STILL, LEVEL);
        bool slow = fabsf(fc_state()->velocity.z) <= 0.15f;
        if (!slow) sawFastGround = true;
        if (slow && !qualifying) qualifyingTick = fc_tick();
        qualifying = slow;
        if (fc_failsafe_state() == FC_FAILSAFE_LANDED) {
            TEST_ASSERT_TRUE(qualifying);
            TEST_ASSERT_TRUE((uint32_t)(fc_tick() - qualifyingTick) >= FC_MS_TO_TICKS(500));
        } else {
            TEST_ASSERT_TRUE(motors_on());
        }
    }
    TEST_ASSERT_TRUE(sawFastGround); /* actually exercised the vertical-speed veto */
    TEST_ASSERT_TRUE((uint32_t)(fc_tick() - start) >= FC_MS_TO_TICKS(500));
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LANDED, fc_failsafe_state());
    TEST_ASSERT_TRUE(fabsf(fc_state()->velocity.z) <= 0.15f);
    TEST_ASSERT_TRUE(fc_state()->position.z <= 0.04f);
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_FALSE(motors_on());
    drive(600, true, true, true, 0);
    TEST_ASSERT_FALSE(fc_arm(true));
    TEST_ASSERT_FALSE(motors_on());
    fc_init();
    ground();
    TEST_ASSERT_TRUE(fc_arm(true)); /* explicit initialization permits a new flight */
}

/* With a z accel bias the ToF-only estimate reads a faster sink than the real one; the vehicle can hover a few
 * cm up through the whole dwell and the motor stop then drops it. During the dwell motor output stays below
 * hover, so a vehicle still in the air keeps settling instead of being held up by the biased estimate. */
static void test_touchdown_dwell_thrust_stays_below_hover(void)
{
    hover();
    drive(2100, true, true, false, 1);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    double hoverRatio = safety_log("hoverThrust");
    TEST_ASSERT_TRUE(hoverRatio > 20000);
    const float biased[3] = {0, 0, 0.98f};
    unsigned maxRatio = 0, dwellTicks = 0;
    for (uint32_t i = 0; i < FC_MS_TO_TICKS(3000) && fc_is_armed(); i++) {
        if (i % FC_MS_TO_TICKS(10) == 0) {
            TEST_ASSERT_TRUE(fc_push_position(0.3f, -0.2f, 0.03f, 0.01f));
            TEST_ASSERT_TRUE(fc_push_tof(0.03f, 0.01f));
        }
        step(STILL, biased);
        if (safety_log("height") > FC_FAILSAFE_TOUCHDOWN_HEIGHT_M ||
            fabsf(fc_state()->velocity.z) > FC_FAILSAFE_TOUCHDOWN_SPEED_MPS || !motors_on()) continue;
        dwellTicks++;
        unsigned r = mean_motor_ratio();
        if (r > maxRatio) maxRatio = r;
    }
    TEST_ASSERT_GREATER_THAN(FC_MS_TO_TICKS(100), dwellTicks);
    TEST_ASSERT_LESS_OR_EQUAL_UINT(hoverRatio * (FC_FAILSAFE_SINK_THRUST_MAX + 0.002f), maxRatio);
}

static void test_landing_timeout_is_fault(void)
{
    hover();
    drive(22500, true, true, false, 1); /* no touchdown evidence, despite fresh height */
    TEST_ASSERT_EQUAL(FC_FAILSAFE_FAULT, fc_failsafe_state());
    TEST_ASSERT_BITS_HIGH(FC_FAILSAFE_REASON_TIMEOUT, (unsigned)safety_log("reason"));
    TEST_ASSERT_FALSE(motors_on());
    TEST_ASSERT_FALSE(fc_arm(true));
}

static void test_touchdown_dwell_restarts_after_height_rises(void)
{
    hover();
    drive(2100, true, true, false, 1);
    for (unsigned i = 100; i > 3; i--) drive(20, true, true, false, i * 0.01f);
    drive(300, true, true, false, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    drive(10, true, true, false, 0.10f); /* breaks otherwise qualifying ground evidence */
    drive(499, true, true, false, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_TRUE(motors_on());
    drive(1000, true, true, false, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LANDED, fc_failsafe_state());
    TEST_ASSERT_FALSE(motors_on());
}

static void test_persistent_bad_imu_is_fault(void)
{
    hover();
    const float corrupt[3] = {NAN, 0, 0};
    for (uint32_t i = 0; i <= FC_MS_TO_TICKS(FC_IMU_BAD_MAX_MS); i++) step(corrupt, LEVEL);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_FAULT, fc_failsafe_state());
    TEST_ASSERT_BITS_HIGH(FC_FAILSAFE_REASON_INVALID_STATE, (unsigned)safety_log("reason"));
    TEST_ASSERT_FALSE(motors_on());
}

static void test_persistent_bad_imu_disarms_legacy_mode(void)
{
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(false));
    hover();
    const float corrupt[3] = {0, 0, INFINITY};
    for (uint32_t i = 0; i <= FC_MS_TO_TICKS(FC_IMU_BAD_MAX_MS); i++) step(STILL, corrupt);
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_FALSE(motors_on());
}

/* One corrupt IMU sample must not reach the LPF/EKF: before this fix it made the estimate NaN for good
 * and tripped cf_sqrt asserts (motor stop on target) in both modes. */
static void single_bad_imu_sample_is_held(bool safeMode)
{
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(safeMode));
    hover();
    fc_host_set_assert_aborts(false);
    uint32_t asserts = fc_host_assert_count();
    const float nanGyro[3] = {NAN, 0, 0}, hugeAcc[3] = {0, 0, 1e6f};
    step(nanGyro, LEVEL);
    drive(20, true, true, true, 1);
    step(STILL, hugeAcc);
    drive(200, true, true, true, 1);
    fc_host_set_assert_aborts(true);
    TEST_ASSERT_EQUAL_UINT32(asserts, fc_host_assert_count());
    TEST_ASSERT_TRUE(isfinite(fc_state()->position.z) && fabsf(fc_state()->position.z - 1) < 0.1f);
    TEST_ASSERT_TRUE(fc_is_armed());
    TEST_ASSERT_TRUE(motors_on());
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state());
}

static void test_legacy_mode_drops_non_finite_measurements(void)
{
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(false));
    hover();
    TEST_ASSERT_FALSE(fc_push_position(NAN, 0, 1, 0.01f));
    TEST_ASSERT_FALSE(fc_push_position(0, 0, 1, 0));
    TEST_ASSERT_FALSE(fc_push_tof(INFINITY, 0.01f));
    TEST_ASSERT_FALSE(fc_push_tof(1, NAN));
    TEST_ASSERT_FALSE(fc_push_flow(NAN, 0, 0.01f, 2));
    TEST_ASSERT_FALSE(fc_push_flow(0, 0, 0.01f, -1));
    drive(200, true, true, true, 1);
    TEST_ASSERT_TRUE(isfinite(fc_state()->position.z) && fabsf(fc_state()->position.z - 1) < 0.1f);
    TEST_ASSERT_TRUE(fc_is_armed());
}

static void test_single_bad_imu_sample_is_held_safe_mode(void) { single_bad_imu_sample_is_held(true); }
static void test_single_bad_imu_sample_is_held_legacy_mode(void) { single_bad_imu_sample_is_held(false); }

static void test_tof_expiry_during_dwell_uses_camera_height_and_restarts_dwell(void)
{
    hover();
    drive(2100, true, true, false, 1);
    for (unsigned i = 100; i > 3; i--) drive(20, true, true, false, i * 0.01f);
    drive(150, true, true, false, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    /* Local range stops. The camera reports higher ground clearance: dwell must not survive. */
    drive(300, true, false, false, 0.10f);
    TEST_ASSERT_EQUAL(0, safety_log("tofFresh"));
    TEST_ASSERT_TRUE(safety_log("height") > 0.04);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_TRUE(motors_on());
    drive(499, true, true, false, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    drive(1000, true, true, false, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LANDED, fc_failsafe_state());
    TEST_ASSERT_FALSE(motors_on());
}

static void test_all_height_expiry_during_dwell_is_fault_not_touchdown(void)
{
    hover();
    drive(2100, true, true, false, 1);
    for (unsigned i = 100; i > 3; i--) drive(20, true, true, false, i * 0.01f);
    drive(100, true, true, false, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    drive(320, false, false, true, 0.03f);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_FAULT, fc_failsafe_state());
    TEST_ASSERT_BITS_HIGH(FC_FAILSAFE_REASON_HEIGHT, (unsigned)safety_log("reason"));
    TEST_ASSERT_FALSE(motors_on());
}

static void test_invalid_flow_is_rejected_without_poisoning_estimator(void)
{
    hover();
    TEST_ASSERT_FALSE(fc_push_flow(NAN, 0, 0.01f, 2));
    TEST_ASSERT_FALSE(fc_push_flow(0, 0, 0, 2));
    TEST_ASSERT_FALSE(fc_push_flow(0, 0, 0.01f, -1));
    step(STILL, LEVEL);
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state());
    TEST_ASSERT_TRUE(fc_is_armed());
}

static void test_fresh_flow_and_range_select_zero_velocity_braking(void)
{
    hover();
    for (unsigned i = 0; i < 320; i++) {
        if (i % 10 == 0) TEST_ASSERT_TRUE(fc_push_flow(0, 0, 0.01f, 2));
        drive(1, false, true, true, 1);
    }
    TEST_ASSERT_EQUAL(FC_FAILSAFE_LAND, fc_failsafe_state());
    TEST_ASSERT_EQUAL(modeVelocity, fc_active_setpoint()->mode.x);
    TEST_ASSERT_EQUAL(modeVelocity, fc_active_setpoint()->mode.y);
    TEST_ASSERT_EQUAL_FLOAT(0, fc_active_setpoint()->velocity.x);
    TEST_ASSERT_EQUAL_FLOAT(0, fc_active_setpoint()->velocity.y);
    TEST_ASSERT_TRUE(motors_on());
    TEST_ASSERT_FALSE(fc_push_position(0.3f, -0.2f, 1, 0.01f)); /* camera stays blocked after braking */
    drive(90, false, true, true, 1);
    TEST_ASSERT_FALSE(fc_push_flow(NAN, 0, 0.01f, 2));
    drive(20, false, true, true, 1);
    TEST_ASSERT_EQUAL(0, safety_log("flowBrake"));
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
    TEST_ASSERT_TRUE(motors_on());
    TEST_ASSERT_FALSE(fc_push_position(0.3f, -0.2f, 1, 0.01f)); /* dropout cannot reopen camera fusion */
    fc_init();
    ground();
    TEST_ASSERT_TRUE(fc_push_position(0.3f, -0.2f, 0, 0.01f)); /* explicit reset starts a new frame */
}

static void test_ground_stale_or_absent_command_never_starts_landing_motors(void)
{
    ground();
    TEST_ASSERT_TRUE(fc_arm(true));
    drive(600, true, true, false, 0);
    TEST_ASSERT_FALSE(motors_on());
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state());
    TEST_ASSERT_TRUE(fc_arm(true));
    fc_set_position_setpoint(0.3f, -0.2f, 0, 0);
    drive(600, true, true, false, 0);
    TEST_ASSERT_FALSE(motors_on());
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_EQUAL(FC_FAILSAFE_NORMAL, fc_failsafe_state());
}

static void test_first_command_has_bounded_arm_grace_without_motor_output(void)
{
    ground();
    TEST_ASSERT_TRUE(fc_arm(true));
    drive(10, true, true, false, 0); /* 100 Hz commander has not delivered its first point yet. */
    TEST_ASSERT_TRUE(fc_is_armed());
    TEST_ASSERT_FALSE(motors_on());
    fc_set_position_setpoint(0.3f, -0.2f, 1, 0);
    drive(20, true, true, false, 0);
    TEST_ASSERT_TRUE(fc_is_armed());
    TEST_ASSERT_TRUE(motors_on());
    for (unsigned i = 1; i <= 100; i++) drive(10, true, true, true, i * 0.01f);
    TEST_ASSERT_TRUE(fc_state()->position.z > 0.9f);

    fc_init();
    ground();
    TEST_ASSERT_TRUE(fc_arm(true));
    drive(500, true, true, false, 0);
    TEST_ASSERT_TRUE(fc_is_armed());
    TEST_ASSERT_FALSE(motors_on());
    step(STILL, LEVEL);
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_FALSE(motors_on());
}

static void test_emergency_stop_and_disarm_override_latched_landing(void)
{
    hover();
    drive(2100, true, true, false, 1);
    fc_arm(false);
    drive(100, true, true, true, 1);
    TEST_ASSERT_FALSE(motors_on());
    TEST_ASSERT_FALSE(fc_arm(true));
    fc_init();
    hover();
    drive(2100, true, true, false, 1);
    fc_emergency_stop();
    TEST_ASSERT_FALSE(fc_is_armed()); /* reported state matches the motors */
    drive(100, true, true, true, 1);
    TEST_ASSERT_FALSE(motors_on());
    TEST_ASSERT_FALSE(fc_arm(true));
}

static void test_dead_accelerometer_never_completes_calibration(void)
{
    const float dead[3] = {0, 0, 0}, weak[3] = {0, 0, 0.3f};
    for (unsigned i = 0; i < 2 * (1024 + 200); i++) step(STILL, dead);
    TEST_ASSERT_EQUAL(FC_PHASE_CALIBRATING, fc_phase());
    for (unsigned i = 0; i < 2 * (1024 + 200); i++) step(STILL, weak);
    TEST_ASSERT_EQUAL(FC_PHASE_CALIBRATING, fc_phase());
    for (unsigned i = 0; i < 1024 + 200; i++) step(STILL, LEVEL); /* recovers once gravity reads ~1 g */
    TEST_ASSERT_EQUAL(FC_PHASE_READY, fc_phase());
    TEST_ASSERT_TRUE(isfinite(fc_state()->position.z));
}

static void test_tumble_protection_remains_active_during_first_command_grace(void)
{
    ground();
    TEST_ASSERT_TRUE(fc_arm(true));
    const float flipping[3] = {400, 0, 0};
    for (uint32_t i = 0; i < FC_MS_TO_TICKS(499) && fc_is_armed(); i++) {
        if (i % FC_MS_TO_TICKS(10) == 0)
            TEST_ASSERT_TRUE(fc_push_position(0.3f, -0.2f, 0, 0.01f));
        step(flipping, LEVEL);
    }
    TEST_ASSERT_TRUE(fc_is_tumbled());
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_FALSE(motors_on());
    TEST_ASSERT_FALSE(fc_arm(true));
}

static void test_tumble_still_overrides_landing_and_tilted_tof_is_rejected(void)
{
    hover();
    drive(2100, true, true, false, 1);
    const float flipping[3] = {400, 0, 0};
    bool rejected = false;
    for (uint32_t i = 0; i < FC_MS_TO_TICKS(1000) && fc_is_armed(); i++) {
        if (i % FC_MS_TO_TICKS(10) == 0) {
            TEST_ASSERT_TRUE(fc_push_position(0.3f, -0.2f, 1, 0.01f));
            if (!fc_push_tof(1, 0.01f)) rejected = true;
        }
        step(flipping, LEVEL);
    }
    TEST_ASSERT_TRUE(rejected);
    TEST_ASSERT_TRUE(fc_is_tumbled());
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_FALSE(motors_on());
    TEST_ASSERT_FALSE(fc_arm(true));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_invalid_position_is_not_accepted);
    RUN_TEST(test_invalid_tof_is_not_accepted);
    RUN_TEST(test_config_persists_but_history_and_measurements_clear_on_init);
    RUN_TEST(test_disabled_mode_preserves_legacy_tof_acceptance);
    RUN_TEST(test_arm_requires_both_fresh_inputs_and_zero_tof_is_ground);
    RUN_TEST(test_rejected_packets_do_not_refresh_freshness);
    RUN_TEST(test_full_queues_cannot_refresh_receipt_age);
    RUN_TEST(test_hold_captures_estimate_not_stale_trajectory_and_can_cancel);
    RUN_TEST(test_exact_command_hold_and_tracking_deadlines);
    RUN_TEST(test_land_latches_and_ignores_recovered_commands);
    RUN_TEST(test_tracking_only_loss_lands_and_never_reacquires_xy);
    RUN_TEST(test_combined_loss_uses_local_height_beyond_legacy_shutdown);
    RUN_TEST(test_xy_loss_during_landing_is_permanent);
    RUN_TEST(test_all_height_loss_is_fault_not_success_and_cannot_rearm);
    RUN_TEST(test_blind_descent_hands_back_to_tof_and_lands);
    RUN_TEST(test_blind_descent_thrust_stays_below_hover_despite_accel_bias);
    RUN_TEST(test_camera_height_fallback_and_slow_descent);
    RUN_TEST(test_touchdown_requires_actual_low_height_low_speed_and_dwell);
    RUN_TEST(test_touchdown_dwell_thrust_stays_below_hover);
    RUN_TEST(test_landing_timeout_is_fault);
    RUN_TEST(test_touchdown_dwell_restarts_after_height_rises);
    RUN_TEST(test_tof_expiry_during_dwell_uses_camera_height_and_restarts_dwell);
    RUN_TEST(test_all_height_expiry_during_dwell_is_fault_not_touchdown);
    RUN_TEST(test_persistent_bad_imu_is_fault);
    RUN_TEST(test_persistent_bad_imu_disarms_legacy_mode);
    RUN_TEST(test_legacy_mode_drops_non_finite_measurements);
    RUN_TEST(test_single_bad_imu_sample_is_held_safe_mode);
    RUN_TEST(test_single_bad_imu_sample_is_held_legacy_mode);
    RUN_TEST(test_invalid_flow_is_rejected_without_poisoning_estimator);
    RUN_TEST(test_fresh_flow_and_range_select_zero_velocity_braking);
    RUN_TEST(test_ground_stale_or_absent_command_never_starts_landing_motors);
    RUN_TEST(test_first_command_has_bounded_arm_grace_without_motor_output);
    RUN_TEST(test_emergency_stop_and_disarm_override_latched_landing);
    RUN_TEST(test_rearm_after_disarm_in_hold_starts_clean);
    RUN_TEST(test_dead_accelerometer_never_completes_calibration);
    RUN_TEST(test_tumble_protection_remains_active_during_first_command_grace);
    RUN_TEST(test_tumble_still_overrides_landing_and_tilted_tof_is_rejected);
    return UNITY_END();
}
