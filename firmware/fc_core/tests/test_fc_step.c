/*
 * Tests for the synchronous flight loop (src/fc_step.c) and host variable access (fc_vars).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <math.h>
#include <string.h>

#include "fc_host.h"
#include "fc_step.h"
#include "fc_vars.h"
#include "unity.h"

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
    fc_arm(true);
    fc_set_position_setpoint(0, 0, 1.0f, 0);
    run(100, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on()); /* armed but still calibrating */

    fc_arm(false);
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    fc_set_position_setpoint(0, 0, 1.0f, 0);
    run(10, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on()); /* ready but disarmed */

    fc_arm(true);
    run(10, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_TRUE(any_motor_on()); /* climb command at z = 0 */

    fc_emergency_stop();
    run(1, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on());
}

static void test_watchdog_levels_then_shuts_down_like_commander(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    fc_arm(true);
    fc_set_position_setpoint(0.5f, 0, 1.0f, 0);
    run(100, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.x);

    run(FC_SETPOINT_TIMEOUT_STABILIZE_MS, ZERO_GYRO, LEVEL_ACC); /* no refresh */
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.roll);
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.z); /* altitude kept while leveling */
    TEST_ASSERT_TRUE(any_motor_on());

    run(FC_SETPOINT_TIMEOUT_SHUTDOWN_MS, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on());
}

static void test_no_setpoint_means_no_thrust(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    fc_arm(true);
    run(50, ZERO_GYRO, LEVEL_ACC);
    TEST_ASSERT_FALSE(any_motor_on());
}

static void test_position_measurements_move_the_estimate(void)
{
    run(CALIBRATION_TICKS, ZERO_GYRO, LEVEL_ACC);
    for (int i = 0; i < 300; i++) {
        if (i % 10 == 0) {
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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_calibrating_and_disarmed);
    RUN_TEST(test_calibration_measures_gyro_bias_and_accel_scale);
    RUN_TEST(test_calibration_restarts_when_moving);
    RUN_TEST(test_motors_stay_off_until_calibrated_and_armed);
    RUN_TEST(test_watchdog_levels_then_shuts_down_like_commander);
    RUN_TEST(test_no_setpoint_means_no_thrust);
    RUN_TEST(test_position_measurements_move_the_estimate);
    RUN_TEST(test_params_and_logs_by_name);
    return UNITY_END();
}
