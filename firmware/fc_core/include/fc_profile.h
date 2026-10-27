/*
 * Board profiles: named sets of firmware PARAM values applied at startup, so one firmware image can
 * carry tuned settings per airframe and loop timing.
 *
 * Parameters are set through fc_platform_param_set(), implemented by the host (fc_vars) and, on the
 * target, by the Crazyflie param system.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *group;
    const char *name;
    float value;
} fc_param_value_t;

#define FC_PROFILE_MAX_SETS 3

typedef struct {
    const char *name;
    const char *description;
    uint32_t loop_rate_hz; /* main-loop rate the profile was designed for */
    struct {
        const fc_param_value_t *values;
        size_t count;
    } sets[FC_PROFILE_MAX_SETS];
} fc_profile_t;

#define FC_PROFILE_OK 0
#define FC_PROFILE_NOT_FOUND -1
#define FC_PROFILE_WRONG_RATE -2
#define FC_PROFILE_PARAM_FAILED -3
#define FC_PROFILE_RESET_REQUIRED -4 /* stock is a fresh-instance baseline, not an undo operation */
#define FC_PROFILE_ARMED -5

/* Implemented by the platform. Returns 0 on success. */
int fc_platform_param_set(const char *group, const char *name, float value);

const fc_profile_t *fc_profile_find(const char *name);
int fc_profile_apply(const fc_profile_t *profile);
int fc_profile_apply_by_name(const char *name);

/* Newline-separated profile names; returns bytes needed. */
size_t fc_profile_list(char *buf, size_t len);

#ifdef __cplusplus
}
#endif
