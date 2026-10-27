/* Mechanical-RPM-tracked gyro notch bank. SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FC_RPM_MOTORS 4
#define FC_RPM_HARMONICS 2
#define FC_RPM_SECTIONS (FC_RPM_MOTORS * FC_RPM_HARMONICS)
/* Eight Q=5 notches add phase below their target. The 500 Hz floor leaves margin above
 * the G1 thresholds in the sampled RPM/corner sweep; a 150 Hz floor failed that sweep.
 * At 1 kHz the 450 Hz upper cap is below this floor, so every section bypasses. */
#define FC_RPM_NOTCH_MIN_HZ 500.0f
#define FC_RPM_NOTCH_MAX_RATIO 0.45f /* leave margin below Nyquist; never filter an aliased target */

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1[3], x2[3], y1[3], y2[3];
    bool active, warm;
} fc_notch_section_t;

typedef struct {
    float sample_hz, q;
    fc_notch_section_t sections[FC_RPM_SECTIONS];
} fc_rpm_notch_t;

/* No allocation or RTOS. One bank per vehicle. q controls width; q=5 is the initial stress-test setting. */
bool fc_rpm_notch_init(fc_rpm_notch_t *bank, float sample_hz, float q);
/* Call at the telemetry rate, not for every IMU sample. Input is MECHANICAL rpm, not electrical eRPM.
 * Rejects an entire invalid packet without changing coefficients. Zero stops that motor's sections. */
bool fc_rpm_notch_update(fc_rpm_notch_t *bank, const float rpm[FC_RPM_MOTORS]);
void fc_rpm_notch_clear(fc_rpm_notch_t *bank);
/* In-place three-axis gyro sample (deg/s). Inactive sections are an exact bypass. */
void fc_rpm_notch_apply(fc_rpm_notch_t *bank, float gyro[3]);

#ifdef __cplusplus
}
#endif
