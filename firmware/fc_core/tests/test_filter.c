/* Unit tests for utils/src/filter.c. SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>

#include "filter.h"
#include "unity.h"

#define FS 1000.0f /* sample rate, Hz */
#define FC 80.0f   /* cutoff, Hz */

void setUp(void) {}
void tearDown(void) {}

/* Steady-state amplitude of a unit sine through a fresh lpf2p filter. */
static float lpf2p_gain_at(float freq_hz)
{
    lpf2pData f;
    lpf2pInit(&f, FS, FC);
    const int settle = 2000, measure = 2000;
    float peak = 0.0f;
    for (int n = 0; n < settle + measure; n++) {
        float y = lpf2pApply(&f, sinf(2.0f * (float)M_PI * freq_hz * n / FS));
        if (n >= settle && fabsf(y) > peak) {
            peak = fabsf(y);
        }
    }
    return peak;
}

static void test_lpf2p_unity_dc_gain(void)
{
    lpf2pData f;
    lpf2pInit(&f, FS, FC);
    float y = 0.0f;
    for (int n = 0; n < 1000; n++) {
        y = lpf2pApply(&f, 1.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, y);
}

static void test_lpf2p_minus_3db_at_cutoff(void)
{
    float gain_db = 20.0f * log10f(lpf2p_gain_at(FC));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -3.01f, gain_db);
}

static void test_lpf2p_passband_flat(void)
{
    float gain_db = 20.0f * log10f(lpf2p_gain_at(FC / 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, gain_db);
}

static void test_lpf2p_stopband_rolloff_second_order(void)
{
    /* 2nd-order Butterworth: about -40 dB/decade; bilinear warping adds attenuation near Nyquist. */
    float gain_db = 20.0f * log10f(lpf2p_gain_at(4.0f * FC));
    TEST_ASSERT_LESS_THAN_FLOAT(-22.0f, gain_db);
}

static void test_lpf2p_rejects_non_finite_state(void)
{
    lpf2pData f;
    lpf2pInit(&f, FS, FC);
    lpf2pApply(&f, INFINITY);
    float y = 0.0f;
    for (int n = 0; n < 1000; n++) {
        y = lpf2pApply(&f, 1.0f);
    }
    TEST_ASSERT_TRUE(isfinite(y));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, y);
}

static void test_lpf2p_reset_starts_at_steady_state(void)
{
    lpf2pData f;
    lpf2pInit(&f, FS, FC);
    lpf2pReset(&f, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, lpf2pApply(&f, 5.0f));
}

static void test_iir_single_converges_to_input(void)
{
    int32_t state = 0;
    int16_t out = 0;
    for (int n = 0; n < 500; n++) {
        out = iirLPFilterSingle(1000, 16, &state);
    }
    TEST_ASSERT_INT_WITHIN(2, 1000, out);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_lpf2p_unity_dc_gain);
    RUN_TEST(test_lpf2p_minus_3db_at_cutoff);
    RUN_TEST(test_lpf2p_passband_flat);
    RUN_TEST(test_lpf2p_stopband_rolloff_second_order);
    RUN_TEST(test_lpf2p_rejects_non_finite_state);
    RUN_TEST(test_lpf2p_reset_starts_at_steady_state);
    RUN_TEST(test_iir_single_converges_to_input);
    return UNITY_END();
}
