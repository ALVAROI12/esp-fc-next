/*
 * Unit tests for the quad-X motor mixer (modules/src/power_distribution_stock.c).
 * Motor layout (Crazyflie): M1 front-right, M2 back-right, M3 back-left, M4 front-left.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_host.h"
#include "power_distribution.h"
#include "stabilizer_types.h"
#include "unity.h"

#define HOVER 30000.0f

static uint16_t m(int i) { return fc_host_motor_ratio((uint32_t)i); }

void setUp(void)
{
    powerDistributionInit();
}

void tearDown(void) {}

static void apply(int16_t roll, int16_t pitch, int16_t yaw, float thrust)
{
    control_t c = {.roll = roll, .pitch = pitch, .yaw = yaw, .thrust = thrust};
    powerDistribution(&c);
}

static void test_pure_thrust_is_equal_on_all_motors(void)
{
    apply(0, 0, 0, HOVER);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT16((uint16_t)HOVER, m(i));
    }
}

static void test_positive_roll_speeds_up_left_side(void)
{
    apply(2000, 0, 0, HOVER);
    TEST_ASSERT_LESS_THAN_UINT16((uint16_t)HOVER, m(0));
    TEST_ASSERT_LESS_THAN_UINT16((uint16_t)HOVER, m(1));
    TEST_ASSERT_GREATER_THAN_UINT16((uint16_t)HOVER, m(2));
    TEST_ASSERT_GREATER_THAN_UINT16((uint16_t)HOVER, m(3));
}

static void test_positive_pitch_speeds_up_front(void)
{
    apply(0, 2000, 0, HOVER);
    TEST_ASSERT_GREATER_THAN_UINT16((uint16_t)HOVER, m(0));
    TEST_ASSERT_LESS_THAN_UINT16((uint16_t)HOVER, m(1));
    TEST_ASSERT_LESS_THAN_UINT16((uint16_t)HOVER, m(2));
    TEST_ASSERT_GREATER_THAN_UINT16((uint16_t)HOVER, m(3));
}

static void test_yaw_uses_diagonal_pairs(void)
{
    apply(0, 0, 1000, HOVER);
    TEST_ASSERT_EQUAL_UINT16(m(0), m(2));
    TEST_ASSERT_EQUAL_UINT16(m(1), m(3));
    TEST_ASSERT_GREATER_THAN_UINT16(m(1), m(0));
}

static void test_torque_commands_preserve_total_thrust(void)
{
    apply(1000, -1500, 700, HOVER);
    uint32_t sum = (uint32_t)m(0) + m(1) + m(2) + m(3);
    TEST_ASSERT_UINT32_WITHIN(4, (uint32_t)(4 * HOVER), sum);
}

static void test_outputs_saturate_without_wrapping(void)
{
    apply(0, 0, 0, 70000.0f);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, m(i));
    }
    apply(-32000, 0, 0, 1000.0f);
    TEST_ASSERT_EQUAL_UINT16(0, m(2));
    TEST_ASSERT_EQUAL_UINT16(0, m(3));
}

static void test_power_stop_zeroes_all_motors(void)
{
    apply(0, 0, 0, HOVER);
    powerStop();
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT16(0, m(i));
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pure_thrust_is_equal_on_all_motors);
    RUN_TEST(test_positive_roll_speeds_up_left_side);
    RUN_TEST(test_positive_pitch_speeds_up_front);
    RUN_TEST(test_yaw_uses_diagonal_pairs);
    RUN_TEST(test_torque_commands_preserve_total_thrust);
    RUN_TEST(test_outputs_saturate_without_wrapping);
    RUN_TEST(test_power_stop_zeroes_all_motors);
    return UNITY_END();
}
