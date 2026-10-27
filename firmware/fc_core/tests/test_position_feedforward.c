/* Feedforward equations and stock-default compatibility. SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>
#include <string.h>

#include "fc_vars.h"
#include "position_controller.h"
#include "unity.h"

static setpoint_t sp;
static state_t state;
static attitude_t attitude;
static float thrust;

void setUp(void)
{
    positionControllerInit();
    positionControllerResetAllPID();
    memset(&sp, 0, sizeof(sp));
    memset(&state, 0, sizeof(state));
    memset(&attitude, 0, sizeof(attitude));
    sp.mode.x = sp.mode.y = sp.mode.z = modeAbs;
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_set("posCtlPid", "velFF", 0));
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_set("posCtlPid", "accFF", 0));
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_set("posCtlPid", "thrustPerAcc", 0));
}

void tearDown(void) {}

static void test_defaults_ignore_trajectory_derivatives(void)
{
    sp.velocity.x = 0.1f;
    sp.acceleration.x = 1.0f;
    positionController(&thrust, &attitude, &sp, &state);
    TEST_ASSERT_EQUAL_FLOAT(0, sp.velocity.x);
    TEST_ASSERT_EQUAL_FLOAT(0, attitude.roll);
    TEST_ASSERT_EQUAL_FLOAT(0, attitude.pitch);
}

static void test_velocity_feedforward_reaches_inner_controller(void)
{
    fc_param_set("posCtlPid", "velFF", 1);
    sp.velocity.x = 0.1f;
    sp.velocity.y = -0.1f;
    positionController(&thrust, &attitude, &sp, &state);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, sp.velocity.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.1f, sp.velocity.y);
    TEST_ASSERT_TRUE(attitude.pitch < 0);
    TEST_ASSERT_TRUE(attitude.roll > 0);
}

static void test_acceleration_feedforward_tilt_and_vertical_thrust(void)
{
    double base;
    fc_param_get("posCtlPid", "thrustBase", &base);
    fc_param_set("posCtlPid", "accFF", 1);
    fc_param_set("posCtlPid", "thrustPerAcc", 2000);
    sp.acceleration.x = 1.0f;
    sp.acceleration.y = -0.5f;
    sp.acceleration.z = 0.5f;
    positionController(&thrust, &attitude, &sp, &state);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -atanf(1.0f / 9.81f) * 180.0f / (float)M_PI, attitude.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, atanf(0.5f / 9.81f) * 180.0f / (float)M_PI, attitude.roll);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, (float)base + 1000, thrust);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_defaults_ignore_trajectory_derivatives);
    RUN_TEST(test_velocity_feedforward_reaches_inner_controller);
    RUN_TEST(test_acceleration_feedforward_tilt_and_vertical_thrust);
    return UNITY_END();
}
