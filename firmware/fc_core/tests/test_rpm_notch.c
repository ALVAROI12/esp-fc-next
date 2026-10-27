/* RPM notch response, retuning and guard tests. SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>
#include "fc_rpm_notch.h"
#include "unity.h"

#define FS 4000.0f
#define TWO_PI 6.28318530718f
static fc_rpm_notch_t bank;

void setUp(void) { TEST_ASSERT_TRUE(fc_rpm_notch_init(&bank, FS, 5)); }
void tearDown(void) {}

static float amplitude(float frequency, float *phase_deg)
{
    double ss = 0, cc = 0;
    const int settle = 2000, samples = 8000;
    for (int i = 0; i < settle + samples; i++) {
        float w = TWO_PI * frequency * i / FS;
        float v[3] = {sinf(w), 0, 0};
        fc_rpm_notch_apply(&bank, v);
        if (i >= settle) { ss += v[0] * sinf(w); cc += v[0] * cosf(w); }
    }
    if (phase_deg) *phase_deg = atan2f((float)cc, (float)ss) * 360.0f / TWO_PI;
    return (float)(2 * sqrt(ss * ss + cc * cc) / samples);
}

static void test_bypass_is_exact_without_rpm(void)
{
    float v[3] = {1.25f, -3.5f, 2.75f};
    fc_rpm_notch_apply(&bank, v);
    TEST_ASSERT_EQUAL_FLOAT(1.25f, v[0]);
    TEST_ASSERT_EQUAL_FLOAT(-3.5f, v[1]);
    const float stopped[4] = {0};
    TEST_ASSERT_TRUE(fc_rpm_notch_update(&bank, stopped));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1, amplitude(10, NULL));
}

static void test_suppresses_rotation_and_bladepass_harmonic(void)
{
    const float rpm[4] = {33000, 0, 0, 0}; /* 550 Hz, 1100 Hz */
    TEST_ASSERT_TRUE(fc_rpm_notch_update(&bank, rpm));
    TEST_ASSERT_LESS_THAN_FLOAT(0.01f, amplitude(550, NULL));
    TEST_ASSERT_LESS_THAN_FLOAT(0.01f, amplitude(1100, NULL));
}

static void test_preserves_low_frequency_motion(void)
{
    const float rpm[4] = {34000, 35000, 36000, 37000};
    TEST_ASSERT_TRUE(fc_rpm_notch_update(&bank, rpm));
    float phase;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1, amplitude(10, &phase));
    TEST_ASSERT_LESS_THAN_FLOAT(6.0f, fabsf(phase));
}

static void test_floor_is_enforced_at_500_hz(void)
{
    const float below[4] = {29900, 0, 0, 0}; /* 498.3 Hz */
    TEST_ASSERT_TRUE(fc_rpm_notch_update(&bank, below));
    TEST_ASSERT_FALSE(bank.sections[0].active);
    TEST_ASSERT_TRUE(bank.sections[1].active); /* 996.7 Hz harmonic remains in band */
    const float above[4] = {30100, 0, 0, 0}; /* 501.7 Hz */
    TEST_ASSERT_TRUE(fc_rpm_notch_update(&bank, above));
    TEST_ASSERT_TRUE(bank.sections[0].active);
    TEST_ASSERT_TRUE(bank.sections[1].active);
    /* A motor crossing the floor must not produce an unbounded or NaN sample. */
    float v[3] = {1, -2, 3};
    fc_rpm_notch_update(&bank, below);
    for (int i = 0; i < 100; i++) {
        v[0] = sinf(TWO_PI * 500 * i / FS);
        fc_rpm_notch_apply(&bank, v);
        TEST_ASSERT_TRUE(isfinite(v[0]));
        TEST_ASSERT_LESS_THAN_FLOAT(2.0f, fabsf(v[0]));
    }
}

static void test_bounded_when_retuning_through_frequency_sweep(void)
{
    double before = 0, after = 0;
    float phase = 0;
    for (int i = 0; i < 16000; i++) {
        float hz = 500 + 500.0f * i / 16000;
        phase = fmodf(phase + TWO_PI * hz / FS, TWO_PI);
        if (i % 40 == 0) { /* 100 Hz telemetry, slightly inaccurate */
            float rpm[4] = {hz * 60 * 1.02f, 0, 0, 0};
            TEST_ASSERT_TRUE(fc_rpm_notch_update(&bank, rpm));
        }
        float v[3] = {sinf(phase), -0.5f * sinf(phase), 0};
        float raw = v[0];
        fc_rpm_notch_apply(&bank, v);
        TEST_ASSERT_TRUE(isfinite(v[0]) && isfinite(v[1]));
        TEST_ASSERT_LESS_THAN_FLOAT(3, fabsf(v[0]));
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f * v[0], v[1]);
        if (i > 2000) { before += raw * raw; after += v[0] * v[0]; }
    }
    TEST_ASSERT_LESS_THAN_FLOAT(0.35f, sqrtf((float)(after / before)));
}

static void test_nonfinite_sample_is_not_filtered_or_remembered(void)
{
    const float rpm[4] = {30000, 0, 0, 0};
    fc_rpm_notch_update(&bank, rpm);
    for (int i = 0; i < 100; i++) { float v[3] = {2, -3, 4}; fc_rpm_notch_apply(&bank, v); }
    float bad[3] = {NAN, 0, 0};
    fc_rpm_notch_apply(&bank, bad);
    TEST_ASSERT_TRUE(isnan(bad[0]));
    float v[3] = {2, -3, 4};
    fc_rpm_notch_apply(&bank, v);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2, v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -3, v[1]);
}

static void test_dc_is_continuous_at_activation_and_clear(void)
{
    const float rpm[4] = {30000, 33000, 36000, 39000};
    fc_rpm_notch_update(&bank, rpm);
    for (int i = 0; i < 100; i++) {
        float v[3] = {2, -3, 4};
        fc_rpm_notch_apply(&bank, v);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2, v[0]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, -3, v[1]);
    }
    fc_rpm_notch_clear(&bank);
    TEST_ASSERT_FALSE(bank.sections[0].active);
}

static void test_invalid_and_unfilterable_rpm_is_not_folded_into_band(void)
{
    float rpm[4] = {24000, 0, 0, 0}; /* 400 Hz: below the floor, excluded to protect control motion */
    TEST_ASSERT_TRUE(fc_rpm_notch_update(&bank, rpm));
    TEST_ASSERT_FALSE(bank.sections[0].active);
    TEST_ASSERT_TRUE(bank.sections[1].active); /* 800 Hz harmonic is valid */
    rpm[0] = 120000; /* 2000 Hz = Nyquist at 4 kHz */
    TEST_ASSERT_TRUE(fc_rpm_notch_update(&bank, rpm));
    TEST_ASSERT_FALSE(bank.sections[0].active || bank.sections[1].active);
    rpm[0] = NAN;
    TEST_ASSERT_FALSE(fc_rpm_notch_update(&bank, rpm));
    rpm[0] = -1;
    TEST_ASSERT_FALSE(fc_rpm_notch_update(&bank, rpm));
    TEST_ASSERT_FALSE(fc_rpm_notch_init(&bank, 0, 5));
    TEST_ASSERT_FALSE(fc_rpm_notch_update(&bank, rpm));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_bypass_is_exact_without_rpm);
    RUN_TEST(test_suppresses_rotation_and_bladepass_harmonic);
    RUN_TEST(test_preserves_low_frequency_motion);
    RUN_TEST(test_floor_is_enforced_at_500_hz);
    RUN_TEST(test_bounded_when_retuning_through_frequency_sweep);
    RUN_TEST(test_nonfinite_sample_is_not_filtered_or_remembered);
    RUN_TEST(test_dc_is_continuous_at_activation_and_clear);
    RUN_TEST(test_invalid_and_unfilterable_rpm_is_not_folded_into_band);
    return UNITY_END();
}
