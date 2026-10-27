/*
 * Board profiles (see fc_profile.h).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_profile.h"

#include <stdio.h>
#include <string.h>

#include "fc_profile_g1.h"
#include "fc_step.h"

#define COUNT(a) (sizeof(a) / sizeof((a)[0]))
static bool profileConfigured;

/* Position-controller settings found in SITL (sim/README.md, "SITL findings"). */
static const fc_param_value_t WHOOP2_POSITION[] = {
    {"posCtlPid", "thrustBase", FC_WHOOP2_THRUST_BASE},
    {"posCtlPid", "zKi", 0.0f},    /* velocity-loop integral already absorbs thrust offset */
    {"posCtlPid", "velFF", 1.0f},  /* track trajectory velocity without lag */
    {"posCtlPid", "accFF", 1.0f},  /* tilt / thrust in advance for trajectory acceleration */
    /* thrust ~ command^2 around hover (ESC + brushless): d(command)/d(acc) = thrustBase / (2 g) */
    {"posCtlPid", "thrustPerAcc", FC_WHOOP2_THRUST_BASE / (2.0f * 9.81f)},
};

static const fc_profile_t PROFILES[] = {
    {"stock", "Fresh-instance baseline only; no runtime restoration", 0, {{NULL, 0}}},
    {"whoop2_interim", "2-inch whoop on stock 1 kHz timing: G1 interim gains + SITL position settings", 1000,
     {{FC_G1_GAINS_INTERIM, COUNT(FC_G1_GAINS_INTERIM)}, {WHOOP2_POSITION, COUNT(WHOOP2_POSITION)}}},
    {"whoop2_4k", "2-inch whoop on the planned 4 kHz timing: G1 proposed gains + SITL position settings", 4000,
     {{FC_G1_GAINS_PLANNED, COUNT(FC_G1_GAINS_PLANNED)}, {WHOOP2_POSITION, COUNT(WHOOP2_POSITION)}}},
};

const fc_profile_t *fc_profile_find(const char *name)
{
    for (size_t i = 0; i < COUNT(PROFILES); i++) {
        if (strcmp(PROFILES[i].name, name) == 0) {
            return &PROFILES[i];
        }
    }
    return NULL;
}

int fc_profile_apply(const fc_profile_t *profile)
{
    if (!profile) {
        return FC_PROFILE_NOT_FOUND;
    }
    if (fc_is_armed()) return FC_PROFILE_ARMED;
    if (profileConfigured && profile->sets[0].count == 0) return FC_PROFILE_RESET_REQUIRED;
    if (profile->loop_rate_hz != 0 && profile->loop_rate_hz != FC_STEP_RATE_HZ) {
        return FC_PROFILE_WRONG_RATE;
    }
    for (size_t s = 0; s < FC_PROFILE_MAX_SETS; s++) {
        for (size_t i = 0; i < profile->sets[s].count; i++) {
            const fc_param_value_t *p = &profile->sets[s].values[i];
            if (fc_platform_param_set(p->group, p->name, p->value) != 0) {
                return FC_PROFILE_PARAM_FAILED;
            }
        }
    }
    if (profile->sets[0].count != 0) profileConfigured = true;
    return FC_PROFILE_OK;
}

int fc_profile_apply_by_name(const char *name) { return fc_profile_apply(fc_profile_find(name)); }

size_t fc_profile_list(char *buf, size_t len)
{
    size_t used = 0;
    for (size_t i = 0; i < COUNT(PROFILES); i++) {
        int room = buf && used < len;
        int w = snprintf(room ? buf + used : NULL, room ? len - used : 0, "%s\n", PROFILES[i].name);
        used += (size_t)(w > 0 ? w : 0);
    }
    return used + 1;
}
