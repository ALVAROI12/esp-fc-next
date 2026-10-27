/*
 * RBJ second-order notch, direct form I: one section per motor at 1x/2x mechanical rotation.
 * Coefficients are shared across axes, histories are not. Re-tuning preserves history; activation
 * seeds history with the current sample to avoid a DC jump. No automatic FFT peak detection here.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_rpm_notch.h"

#include <math.h>
#include <string.h>

bool fc_rpm_notch_init(fc_rpm_notch_t *bank, float sample_hz, float q)
{
    memset(bank, 0, sizeof(*bank));
    if (!isfinite(sample_hz) || sample_hz <= 0 || !isfinite(q) || q < 1 || q > 50) return false;
    bank->sample_hz = sample_hz;
    bank->q = q;
    return true;
}

void fc_rpm_notch_clear(fc_rpm_notch_t *bank)
{
    memset(bank->sections, 0, sizeof(bank->sections));
}

bool fc_rpm_notch_update(fc_rpm_notch_t *bank, const float rpm[FC_RPM_MOTORS])
{
    if (bank->sample_hz <= 0 || bank->q < 1) return false;
    for (int m = 0; m < FC_RPM_MOTORS; m++) {
        if (!isfinite(rpm[m]) || rpm[m] < 0 || rpm[m] > 200000.0f) return false;
    }
    for (int m = 0; m < FC_RPM_MOTORS; m++) {
        for (int h = 1; h <= FC_RPM_HARMONICS; h++) {
            fc_notch_section_t *s = &bank->sections[m * FC_RPM_HARMONICS + h - 1];
            float hz = rpm[m] * h / 60.0f;
            if (hz < FC_RPM_NOTCH_MIN_HZ || hz >= bank->sample_hz * FC_RPM_NOTCH_MAX_RATIO) {
                memset(s, 0, sizeof(*s));
                continue;
            }
            float w = 6.28318530718f * hz / bank->sample_hz;
            float alpha = sinf(w) / (2.0f * bank->q);
            float norm = 1.0f / (1.0f + alpha);
            s->b0 = s->b2 = norm;
            s->a1 = s->b1 = -2.0f * cosf(w) * norm;
            s->a2 = (1.0f - alpha) * norm;
            s->active = true;
        }
    }
    return true;
}

void fc_rpm_notch_apply(fc_rpm_notch_t *bank, float gyro[3])
{
    /* An invalid sample passes through untouched and must not seed or advance history. */
    if (!isfinite(gyro[0]) || !isfinite(gyro[1]) || !isfinite(gyro[2])) return;
    for (int i = 0; i < FC_RPM_SECTIONS; i++) {
        fc_notch_section_t *s = &bank->sections[i];
        if (!s->active) continue;
        for (int a = 0; a < 3; a++) {
            float x = gyro[a];
            if (!s->warm) s->x1[a] = s->x2[a] = s->y1[a] = s->y2[a] = x;
            float y = s->b0 * x + s->b1 * s->x1[a] + s->b2 * s->x2[a]
                      - s->a1 * s->y1[a] - s->a2 * s->y2[a];
            s->x2[a] = s->x1[a]; s->x1[a] = x;
            s->y2[a] = s->y1[a]; s->y1[a] = y;
            gyro[a] = y;
        }
        s->warm = true;
    }
}
