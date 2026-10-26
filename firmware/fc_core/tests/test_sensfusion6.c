/*
 * Tests for the Mahony complementary attitude filter (modules/src/sensfusion6.c).
 * The filter keeps static state, so tests run in a fixed order that builds on it.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <math.h>

#include "sensfusion6.h"
#include "unity.h"

#define DT 0.002f /* 500 Hz */
#define DEG2RAD ((float)M_PI / 180.0f)

void setUp(void) {}
void tearDown(void) {}

static void run(float gx, float gy, float gz, float ax, float ay, float az, float seconds)
{
    int steps = (int)(seconds / DT);
    for (int i = 0; i < steps; i++) {
        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, DT);
    }
}

static void test_level_and_stationary_stays_level(void)
{
    sensfusion6Init();
    TEST_ASSERT_TRUE(sensfusion6Test());
    run(0, 0, 0, 0, 0, 1.0f, 5.0f);

    float roll, pitch, yaw;
    sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, yaw);

    float qx, qy, qz, qw;
    sensfusion6GetQuaternion(&qx, &qy, &qz, &qw);
    /*
     * invSqrt() is the one-Newton-step fast inverse square root (max ~0.175% error), so the
     * quaternion is only approximately unit length. Tighten to 1e-6 once task 1.6 replaces it
     * with 1/sqrtf() (hardware FPU on the P4).
     */
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, 1.0f, sqrtf(qx * qx + qy * qy + qz * qz + qw * qw));
}

static void test_gyro_integrates_yaw(void)
{
    /* Accelerometer gives no yaw information, so yaw comes from gyro integration alone. */
    run(0, 0, 45.0f, 0, 0, 1.0f, 1.0f);
    float roll, pitch, yaw;
    sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 45.0f, fabsf(yaw));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, roll);
}

static void test_static_tilt_converges_to_gravity_direction(void)
{
    const float tilt = 10.0f;
    run(0, 0, 0, 0, sinf(tilt * DEG2RAD), cosf(tilt * DEG2RAD), 20.0f);
    float roll, pitch, yaw;
    sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, tilt, fabsf(roll));
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, pitch);
}

static void test_zero_accel_does_not_produce_nan(void)
{
    run(1.0f, -1.0f, 0.5f, 0, 0, 0, 0.5f);
    float roll, pitch, yaw;
    sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
    TEST_ASSERT_TRUE(isfinite(roll) && isfinite(pitch) && isfinite(yaw));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_level_and_stationary_stays_level);
    RUN_TEST(test_gyro_integrates_yaw);
    RUN_TEST(test_static_tilt_converges_to_gravity_direction);
    RUN_TEST(test_zero_accel_does_not_produce_nan);
    return UNITY_END();
}
