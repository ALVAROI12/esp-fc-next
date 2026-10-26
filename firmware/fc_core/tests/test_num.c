/* Unit tests for utils/src/num.c. SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>

#include "num.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

static void test_limitUint16_clamps_to_range(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, limitUint16(-5));
    TEST_ASSERT_EQUAL_UINT16(0, limitUint16(INT32_MIN));
    TEST_ASSERT_EQUAL_UINT16(1234, limitUint16(1234));
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, limitUint16(UINT16_MAX));
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, limitUint16(70000));
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, limitUint16(INT32_MAX));
}

static void test_constrain(void)
{
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, constrain(-3.0f, -1.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, constrain(0.5f, -1.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, constrain(9.0f, -1.0f, 1.0f));
}

static void test_deadband_removes_threshold_and_keeps_continuity(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, deadband(0.05f, 0.1f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, deadband(-0.05f, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, deadband(0.5f, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.4f, deadband(-0.5f, 0.1f));
    /* Output is continuous at the threshold edge (no jump from 0 to threshold). */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, deadband(0.1f, 0.1f));
}

static void test_half_precision_round_trip(void)
{
    const float values[] = {0.0f, 1.0f, -1.0f, 0.5f, 3.14159f, -123.25f, 1000.0f, 65504.0f};
    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        float back = half2single(single2half(values[i]));
        /* fp16 keeps ~11 significant bits. */
        TEST_ASSERT_FLOAT_WITHIN(fabsf(values[i]) * 1e-3f + 1e-6f, values[i], back);
    }
}

static void test_half_precision_special_values(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x7C00, single2half(1e9f));
    TEST_ASSERT_EQUAL_HEX16(0xFC00, single2half(-1e9f));
    TEST_ASSERT_EQUAL_HEX16(0x7E00, single2half(NAN));
    TEST_ASSERT_TRUE(isinf(half2single(0x7C00)));
    TEST_ASSERT_TRUE(isnan(half2single(0x7E00)));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_limitUint16_clamps_to_range);
    RUN_TEST(test_constrain);
    RUN_TEST(test_deadband_removes_threshold_and_keeps_continuity);
    RUN_TEST(test_half_precision_round_trip);
    RUN_TEST(test_half_precision_special_values);
    return UNITY_END();
}
