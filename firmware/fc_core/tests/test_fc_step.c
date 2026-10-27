/*
 * Tests for the synchronous flight loop (src/fc_step.c) and host variable access (fc_vars).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <math.h>
#include <string.h>

#include "fc_host.h"
#include "fc_profile.h"
#include "fc_step.h"
#include "fc_vars.h"
#include "pid.h"
#include "unity.h"

extern PidObject pidRoll;

static const float LEVEL_ACC[3] = {0.0f, 0.0f, 1.0f};
static const float ZERO_GYRO[3] = {0.0f, 0.0f, 0.0f};

#define CALIBRATION_TICKS (1024 + 200)

static void run(int ticks, const float gyro[3], const float acc[3])
{
    for (int i = 0; i < ticks; i++) {
        fc_host_advance_time_us(1000);
        fc_step(gyro, acc);
    }
}

static int any_motor_on(void)
{
    for (uint32_t m = 0; m < 4; m++) {
        if (fc_host_motor_ratio(m) != 0) {
            return 1;
        }
    }
    return 0;
}

void setUp(void)
{
    fc_host_set_log_level(1);
    fc_init();
}

void tearDown(void) {}

static void test_starts_calibrating_and_disarmed(void)
{
    TEST_ASSERT_EQUAL(FC_PHASE_CALIBRATING, fc_phase());
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_EQUAL_UINT32(FC_STEP_RATE_HZ, fc_rate_hz());
}

static void test_calibration_measures_gyro_bias_and_accel_scale(void)
{
    const float biased[3] = {0.5f, -0.25f, 1.0f};
    const float acc[3] = {0.0f, 0.0f, 1.02f};
    run(CALIBRATION_TICKS - 1, biased, acc);
    TEST_ASSERT_EQUAL(FC_PHASE_CALIBRATING, fc_phase());
    run(1, biased, acc);
    TEST_ASSERT_EQUAL(FC_PHASE_READY, fc_phase());

    double v;
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcStep", "biasX", &v));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.5, v);
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcStep", "biasZ", &v));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.0, v);
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcStep", "accScale", &v));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.02, v);
}

static void test_calibration_restarts_when_moving(void)
{
    for (int i = 0; i < 1024; i++) {
        const float shaking[3] = {(i % 2) ? 20.0f : -20.0f, 0.0f, 0.0f};
        run(1, shaking, LEVEL_ACC);
    }
    run(200, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_EQUAL(FC_PHASE_CALIBRATING, fc_phase());
}

static void test_motors_stay_off_until_calibrated_and_armed(void)
{
    TEST_ASSERT_FALSE(fc_arm(true)); /* refused while calibrating */
    fc_set_position_setpoint(0, 0, 1.0f, 0);
    run(100, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on()); /* armed but still calibrating */

    fc_arm(false);
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    fc_set_position_setpoint(0, 0, 1.0f, 0);
    run(10, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on()); /* ready but disarmed */

    TEST_ASSERT_TRUE(fc_arm(true));
    run(FC_MS_TO_TICKS(10), ZERO_GYRO, LEVEL_ACC); /* allow the 100 Hz position loop to run */
    TEST_ASSERT_TRUE(any_motor_on()); /* climb command at z = 0 */

    fc_emergency_stop();
    run(1, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on());
}

static void test_watchdog_levels_then_shuts_down_like_commander(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_TRUE(fc_arm(true));
    fc_set_position_setpoint(0.5f, 0, 1.0f, 0);
    run(100, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.x);

    run(FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS), ZERO_GYRO, LEVEL_ACC); /* no refresh */
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.roll);
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.z); /* altitude kept while leveling */
    TEST_ASSERT_TRUE(any_motor_on());

    run(FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_SHUTDOWN_MS), ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on());
}

static void test_refuses_to_arm_when_tilted(void)
{
    /* 30 deg roll: the estimator learns the tilt once position measurements make velocity observable. */
    const float tilted[3] = {0.0f, 0.5f, 0.866f};
    run(CALIBRATION_TICKS, ZERO_GYRO, tilted);
    for (int i = 0; i < (int)FC_MS_TO_TICKS(3000); i++) {
        if (i % (int)FC_MS_TO_TICKS(10) == 0) {
            fc_push_position(0, 0, 0, 0.01f);
        }
        run(1, ZERO_GYRO, tilted);
    }
    float s[13];
    fc_get_state_array(s);
    TEST_ASSERT_TRUE(fabsf(s[6]) > FC_ARM_MAX_TILT_DEG);
    TEST_ASSERT_FALSE(fc_arm(true));
    TEST_ASSERT_FALSE(fc_is_armed());
}

static void test_immediate_tilted_and_inverted_arming_is_refused(void)
{
    const float tilted[3] = {0, 0.5f, 0.866f};
    run(CALIBRATION_TICKS, ZERO_GYRO, tilted);
    TEST_ASSERT_FALSE(fc_arm(true)); /* no mocap and no estimator convergence wait */
    fc_init();
    const float inverted[3] = {0, 0, -1};
    run(CALIBRATION_TICKS, ZERO_GYRO, inverted);
    TEST_ASSERT_FALSE(fc_arm(true));
}

static void test_watchdog_discards_trajectory_feedforward(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    fc_set_trajectory_setpoint(0, 0, 1, 0.2f, 0.1f, 1, 1, 2, 2, 0);
    run(FC_MS_TO_TICKS(600), ZERO_GYRO, LEVEL_ACC);
    const setpoint_t *sp = fc_active_setpoint();
    TEST_ASSERT_EQUAL_FLOAT(0, sp->velocity.z);
    TEST_ASSERT_EQUAL_FLOAT(0, sp->acceleration.z);
    TEST_ASSERT_EQUAL_FLOAT(1, sp->position.z);
}

static void test_reinitialization_clears_pid_dynamics(void)
{
    pidRoll.integ = 5;
    pidRoll.prevError = -20;
    fc_init();
    TEST_ASSERT_EQUAL_FLOAT(0, pidRoll.integ);
    TEST_ASSERT_EQUAL_FLOAT(0, pidRoll.prevError);
    TEST_ASSERT_FALSE(any_motor_on());
}

static void test_tumble_stops_motors_and_blocks_rearming(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_TRUE(fc_arm(true));
    const float flipping[3] = {400.0f, 0.0f, 0.0f}; /* deg/s roll: past 70 deg within 0.2 s */
    for (int i = 0; i < (int)FC_MS_TO_TICKS(1000) && fc_is_armed(); i++) {
        fc_set_position_setpoint(0, 0, 1.0f, 0);
        run(1, flipping, LEVEL_ACC);
    }
    TEST_ASSERT_TRUE(fc_is_tumbled());
    TEST_ASSERT_FALSE(fc_is_armed());
    run(1, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on());
    TEST_ASSERT_FALSE(fc_arm(true));
}

static void test_no_setpoint_means_no_thrust(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    fc_arm(true);
    run(50, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on());
}

static void test_disarmed_commands_do_not_mark_estimator_flying(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    for (int i = 0; i < (int)FC_MS_TO_TICKS(600); i++) {
        fc_set_position_setpoint(0, 0, 1, 0);
        run(1, ZERO_GYRO, LEVEL_ACC);
    }
    double flying;
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcKalman", "flying", &flying));
    TEST_ASSERT_EQUAL_FLOAT(0, (float)flying);
    TEST_ASSERT_EQUAL_FLOAT(0, fc_control()->thrust);
    TEST_ASSERT_FALSE(any_motor_on());
}

static void test_position_measurements_move_the_estimate(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    for (int i = 0; i < (int)FC_MS_TO_TICKS(300); i++) {
        if (i % (int)FC_MS_TO_TICKS(10) == 0) {
            TEST_ASSERT_TRUE(fc_push_position(0.3f, -0.2f, 0.0f, 0.01f));
        }
        run(1, ZERO_GYRO, LEVEL_ACC);
    }
    float s[13];
    fc_get_state_array(s);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.3f, s[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.2f, s[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, sqrtf(s[9] * s[9] + s[10] * s[10] + s[11] * s[11] + s[12] * s[12]));
}

static void test_params_and_logs_by_name(void)
{
    double v = 0;
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_get("pid_rate", "roll_kp", &v));
    TEST_ASSERT_EQUAL_FLOAT(250.0f, (float)v); /* attitudeControllerInit ran in fc_init */
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_get("posCtlPid", "velFF", &v));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)v); /* stock default: no feedforward */

    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_set("posCtlPid", "thrustBase", 39750));
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_get("posCtlPid", "thrustBase", &v));
    TEST_ASSERT_EQUAL_FLOAT(39750.0f, (float)v);

    run(7, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcStep", "tick", &v));
    TEST_ASSERT_EQUAL(7, (int)v);

    TEST_ASSERT_EQUAL(FC_VAR_NOT_FOUND, fc_param_get("nope", "nothing", &v));
    TEST_ASSERT_EQUAL(FC_VAR_NOT_FOUND, fc_log_get("fcStep", "nothing", &v));

    char list[8192];
    size_t needed = fc_vars_list(0, list, sizeof(list));
    TEST_ASSERT_TRUE(needed > 1 && needed <= sizeof(list));
    TEST_ASSERT_NOT_NULL(strstr(list, "pid_rate.roll_kp\n"));
}

static void test_rpm_timeout_bypasses_and_invalid_packets_do_not_refresh(void)
{
    TEST_ASSERT_TRUE(fc_set_rpm_notch(true, 5));
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);

    /* In-band target for this build. At 1 kHz the 0.45*fs cap (450 Hz) is below the 500 Hz
     * floor, so the bank must stay an exact bypass there; only the 4 kHz build can filter. */
#if FC_STEP_RATE_HZ >= 2000
    const float target_hz = 600.0f; /* 36000 rpm, near the modeled hover speed */
#else
    const float target_hz = 300.0f; /* below the floor: exercises the bypass path only */
#endif
    const float rpm[4] = {target_hz * 60.0f, target_hz * 60.0f, target_hz * 60.0f, target_hz * 60.0f};
    TEST_ASSERT_TRUE(fc_push_rpm(rpm));

    /* Feed the target tone and compare the logged raw and notched gyro. */
    double raw_ss = 0, notch_ss = 0;
    for (int i = 0; i < 4000; i++) {
        if (i % FC_MS_TO_TICKS(10) == 0) TEST_ASSERT_TRUE(fc_push_rpm(rpm));
        float gyro[3] = {sinf(6.28318530718f * target_hz * i / FC_STEP_RATE_HZ), 0.0f, 0.0f};
        run(1, gyro, LEVEL_ACC);
        if (i >= 2000) {
            double raw, notch;
            fc_log_get("fcGyro", "rawX", &raw);
            fc_log_get("fcGyro", "notchX", &notch);
            raw_ss += raw * raw;
            notch_ss += notch * notch;
        }
    }
    TEST_ASSERT_TRUE(raw_ss > 1.0);
#if FC_STEP_RATE_HZ >= 2000
    TEST_ASSERT_TRUE(notch_ss < 0.05 * raw_ss); /* in-band tone is suppressed */
#else
    TEST_ASSERT_TRUE(notch_ss == raw_ss); /* exact bypass below the floor */
#endif

    double fresh;
    fc_log_get("fcGyro", "rpmFresh", &fresh);
    TEST_ASSERT_EQUAL(1, (int)fresh);

    /* Stale telemetry must bypass the bank, not merely clear the flag. */
    run(FC_MS_TO_TICKS(90), ZERO_GYRO, LEVEL_ACC);
    const float corrupt[4] = {30000, NAN, -1, 1000};
    TEST_ASSERT_FALSE(fc_push_rpm(corrupt));
    run(FC_MS_TO_TICKS(11), ZERO_GYRO, LEVEL_ACC);
    fc_log_get("fcGyro", "rpmFresh", &fresh);
    TEST_ASSERT_EQUAL(0, (int)fresh);
    for (int i = 0; i < 200; i++) {
        float gyro[3] = {sinf(6.28318530718f * target_hz * i / FC_STEP_RATE_HZ), 0.0f, 0.0f};
        run(1, gyro, LEVEL_ACC);
        double raw, notch;
        fc_log_get("fcGyro", "rawX", &raw);
        fc_log_get("fcGyro", "notchX", &notch);
        TEST_ASSERT_TRUE(raw == notch);
    }

    TEST_ASSERT_TRUE(fc_arm(true));
    TEST_ASSERT_FALSE(fc_set_rpm_notch(false, 5)); /* configuration cannot change while armed */
    fc_arm(false);
    TEST_ASSERT_TRUE(fc_set_rpm_notch(false, 5));
    TEST_ASSERT_FALSE(fc_set_rpm_notch(true, NAN));
}

/* Runs last: profiles change firmware parameters, which persist across fc_init() as on hardware. */
static void test_board_profiles(void)
{
    const char *native = FC_STEP_RATE_HZ == 4000 ? "whoop2_4k" : "whoop2_interim";
    const char *other = FC_STEP_RATE_HZ == 4000 ? "whoop2_interim" : "whoop2_4k";
    double v = 0;

    TEST_ASSERT_EQUAL(FC_PROFILE_OK, fc_profile_apply_by_name("stock"));
    TEST_ASSERT_EQUAL(FC_PROFILE_WRONG_RATE, fc_profile_apply_by_name(other));
    TEST_ASSERT_EQUAL(FC_PROFILE_NOT_FOUND, fc_profile_apply_by_name("nope"));

    TEST_ASSERT_EQUAL(FC_PROFILE_OK, fc_profile_apply_by_name(native));
    TEST_ASSERT_EQUAL(FC_PROFILE_RESET_REQUIRED, fc_profile_apply_by_name("stock"));
    fc_param_get("posCtlPid", "velFF", &v);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, (float)v);
    fc_param_get("posCtlPid", "zKi", &v);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)v);
    fc_param_get("posCtlPid", "thrustBase", &v);
    TEST_ASSERT_TRUE(v > 30000 && v < 50000);
    fc_param_get("pid_rate", "roll_kp", &v);
    TEST_ASSERT_TRUE(v != 250.0); /* G1 gains replaced the Crazyflie default */

    fc_init(); /* dynamics reset preserves the entire configuration */
    fc_param_get("posCtlPid", "zKi", &v);
    TEST_ASSERT_EQUAL_FLOAT(0, (float)v);
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_TRUE(fc_arm(true));
    TEST_ASSERT_EQUAL(FC_PROFILE_ARMED, fc_profile_apply_by_name(native));

    char list[256];
    fc_profile_list(list, sizeof(list));
    TEST_ASSERT_NOT_NULL(strstr(list, "whoop2_4k\n"));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_calibrating_and_disarmed);
    RUN_TEST(test_calibration_measures_gyro_bias_and_accel_scale);
    RUN_TEST(test_calibration_restarts_when_moving);
    RUN_TEST(test_motors_stay_off_until_calibrated_and_armed);
    RUN_TEST(test_watchdog_levels_then_shuts_down_like_commander);
    RUN_TEST(test_refuses_to_arm_when_tilted);
    RUN_TEST(test_immediate_tilted_and_inverted_arming_is_refused);
    RUN_TEST(test_watchdog_discards_trajectory_feedforward);
    RUN_TEST(test_reinitialization_clears_pid_dynamics);
    RUN_TEST(test_tumble_stops_motors_and_blocks_rearming);
    RUN_TEST(test_no_setpoint_means_no_thrust);
    RUN_TEST(test_disarmed_commands_do_not_mark_estimator_flying);
    RUN_TEST(test_position_measurements_move_the_estimate);
    RUN_TEST(test_params_and_logs_by_name);
    RUN_TEST(test_rpm_timeout_bypasses_and_invalid_packets_do_not_refresh);
    RUN_TEST(test_board_profiles);
    return UNITY_END();
}
