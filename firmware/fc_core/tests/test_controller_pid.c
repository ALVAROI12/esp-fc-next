/*
 * Tests for the cascaded PID controller (controller_pid.c + attitude_pid_controller.c)
 * driving the mixer and the host motor HAL.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdlib.h>
#include <string.h>

#include "controller.h"
#include "fc_host.h"
#include "power_distribution.h"
#include "stabilizer_types.h"
#include "unity.h"

#define HOVER 30000.0f

static setpoint_t sp;
static state_t state;
static sensorData_t sensors;
static control_t control;

void setUp(void)
{
    controllerInit(ControllerTypePID);
    powerDistributionInit();

    memset(&sp, 0, sizeof(sp));
    sp.thrust = HOVER;
    sp.mode.x = modeDisable;
    sp.mode.y = modeDisable;
    sp.mode.z = modeDisable;
    sp.mode.roll = modeAbs;
    sp.mode.pitch = modeAbs;
    sp.mode.yaw = modeVelocity;

    memset(&state, 0, sizeof(state));
    state.attitudeQuaternion.w = 1.0f;

    memset(&sensors, 0, sizeof(sensors));
    sensors.acc.z = 1.0f;

    memset(&control, 0, sizeof(control));

    /*
     * Reset integrators left over from the previous test. The zero-thrust reset only
     * takes effect on ticks where the 500 Hz attitude loop runs, so use tick 0.
     */
    control_t zero = {0};
    sp.thrust = 0.0f;
    controller(&zero, &sp, &sensors, &state, 0);
    sp.thrust = HOVER;
}

void tearDown(void) {}

static void run(int ticks)
{
    for (int tick = 1; tick <= ticks; tick++) {
        controller(&control, &sp, &sensors, &state, (uint32_t)tick);
        powerDistribution(&control);
    }
}

static void test_selftest_passes(void)
{
    TEST_ASSERT_TRUE(controllerTest());
    TEST_ASSERT_TRUE(powerDistributionTest());
    TEST_ASSERT_EQUAL_STRING("PID", controllerGetName());
}

static void test_level_hover_is_torque_free_and_balanced(void)
{
    run(1000);
    TEST_ASSERT_EQUAL_INT16(0, control.roll);
    TEST_ASSERT_EQUAL_INT16(0, control.pitch);
    TEST_ASSERT_EQUAL_INT16(0, control.yaw);
    for (uint32_t i = 1; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT16(fc_host_motor_ratio(0), fc_host_motor_ratio(i));
    }
    TEST_ASSERT_UINT16_WITHIN(1, (uint16_t)HOVER, fc_host_motor_ratio(0));
}

static void test_roll_error_is_corrected(void)
{
    state.attitude.roll = 10.0f;
    run(10);
    TEST_ASSERT_LESS_THAN_INT16(0, control.roll);
}

static void test_roll_response_is_antisymmetric(void)
{
    state.attitude.roll = 8.0f;
    run(10);
    int16_t positive = control.roll;

    setUp();
    state.attitude.roll = -8.0f;
    run(10);
    TEST_ASSERT_INT_WITHIN(2, -positive, control.roll);
}

static void test_pitch_error_produces_torque(void)
{
    state.attitude.pitch = 10.0f;
    run(10);
    TEST_ASSERT_NOT_EQUAL(0, control.pitch);
    TEST_ASSERT_EQUAL_INT16(0, control.roll);
}

static void test_yaw_rate_command_produces_yaw_torque(void)
{
    sp.attitudeRate.yaw = 90.0f;
    run(10);
    int16_t positive = control.yaw;
    TEST_ASSERT_NOT_EQUAL(0, positive);

    setUp();
    sp.attitudeRate.yaw = -90.0f;
    run(10);
    TEST_ASSERT_TRUE((positive > 0) != (control.yaw > 0));
}

static void test_zero_thrust_outputs_nothing(void)
{
    state.attitude.roll = 20.0f;
    sp.thrust = 0.0f;
    run(5);
    TEST_ASSERT_EQUAL_INT16(0, control.roll);
    TEST_ASSERT_EQUAL_INT16(0, control.pitch);
    TEST_ASSERT_EQUAL_INT16(0, control.yaw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, control.thrust);
}

int main(void)
{
    fc_host_set_log_level(1);
    UNITY_BEGIN();
    RUN_TEST(test_selftest_passes);
    RUN_TEST(test_level_hover_is_torque_free_and_balanced);
    RUN_TEST(test_roll_error_is_corrected);
    RUN_TEST(test_roll_response_is_antisymmetric);
    RUN_TEST(test_pitch_error_produces_torque);
    RUN_TEST(test_yaw_rate_command_produces_yaw_torque);
    RUN_TEST(test_zero_thrust_outputs_nothing);
    return UNITY_END();
}
