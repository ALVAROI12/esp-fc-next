/* Unit tests for modules/src/pid.c. SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>

#include "pid.h"
#include "unity.h"

#define DT 0.002f

static PidObject pid;

void setUp(void)
{
    pidInit(&pid, 0.0f, 0.0f, 0.0f, 0.0f, DT, 500.0f, 30.0f, false);
    pidSetIntegralLimit(&pid, 0.0f);
    pid.outputLimit = 0.0f;
}

void tearDown(void) {}

static void test_proportional_only(void)
{
    pidSetKp(&pid, 2.0f);
    pidSetDesired(&pid, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f * (10.0f - 4.0f), pidUpdate(&pid, 4.0f, true));
}

static void test_integral_accumulates_error_times_dt(void)
{
    pidSetKi(&pid, 1.0f);
    pidSetDesired(&pid, 1.0f);
    float out = 0.0f;
    for (int i = 0; i < 100; i++) {
        out = pidUpdate(&pid, 0.0f, true);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100 * DT, out);
}

static void test_integral_limit_prevents_windup(void)
{
    pidSetKi(&pid, 1.0f);
    pidSetIntegralLimit(&pid, 0.05f);
    pidSetDesired(&pid, 100.0f);
    for (int i = 0; i < 10000; i++) {
        pidUpdate(&pid, 0.0f, true);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, pid.integ);
    /* After the error reverses, recovery is immediate because the integral never wound up. */
    pidSetDesired(&pid, -100.0f);
    pidUpdate(&pid, 0.0f, true);
    TEST_ASSERT_LESS_THAN_FLOAT(0.05f, pid.integ);
}

static void test_output_limit(void)
{
    pidSetKp(&pid, 1000.0f);
    pid.outputLimit = 50.0f;
    pidSetDesired(&pid, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, pidUpdate(&pid, 0.0f, true));
    pidSetDesired(&pid, -1.0f);
    TEST_ASSERT_EQUAL_FLOAT(-50.0f, pidUpdate(&pid, 0.0f, true));
}

static void test_derivative_of_step(void)
{
    pidSetKd(&pid, 1.0f);
    pidSetDesired(&pid, 0.0f);
    pidUpdate(&pid, 0.0f, true);
    /* Error steps from 0 to -1 in one sample: derivative = -1 / dt. */
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, -1.0f / DT, pidUpdate(&pid, 1.0f, true));
}

static void test_filtered_derivative_is_attenuated(void)
{
    pidInit(&pid, 0.0f, 0.0f, 0.0f, 1.0f, DT, 500.0f, 30.0f, true);
    pid.outputLimit = 0.0f;
    pidUpdate(&pid, 0.0f, true);
    float out = pidUpdate(&pid, 1.0f, true);
    TEST_ASSERT_TRUE(fabsf(out) < 1.0f / DT);
    TEST_ASSERT_TRUE(out < 0.0f);
}

static void test_reset_clears_state(void)
{
    pidSetKi(&pid, 1.0f);
    pidSetDesired(&pid, 1.0f);
    pidUpdate(&pid, 0.0f, true);
    pidReset(&pid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integ);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.error);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.prevError);
}

static void test_set_error_without_update(void)
{
    pidSetKp(&pid, 3.0f);
    pidSetError(&pid, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 6.0f, pidUpdate(&pid, 123.0f, false));
}

static void test_is_active(void)
{
    TEST_ASSERT_FALSE(pidIsActive(&pid));
    pidSetKd(&pid, 0.1f);
    TEST_ASSERT_TRUE(pidIsActive(&pid));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_proportional_only);
    RUN_TEST(test_integral_accumulates_error_times_dt);
    RUN_TEST(test_integral_limit_prevents_windup);
    RUN_TEST(test_output_limit);
    RUN_TEST(test_derivative_of_step);
    RUN_TEST(test_filtered_derivative_is_attenuated);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_set_error_without_update);
    RUN_TEST(test_is_active);
    return UNITY_END();
}
