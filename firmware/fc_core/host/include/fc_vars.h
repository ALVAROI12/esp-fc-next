/*
 * Host access to the firmware's Crazyflie PARAM and LOG variables by "group.name".
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FC_VAR_OK 0
#define FC_VAR_NOT_FOUND -1
#define FC_VAR_READ_ONLY -2
#define FC_VAR_UNSUPPORTED -3

/* Parameters (read/write). */
int fc_param_get(const char *group, const char *name, double *value);
int fc_param_set(const char *group, const char *name, double value);

/* Log variables (read only). */
int fc_log_get(const char *group, const char *name, double *value);

/* Newline-separated "group.name" list. kind: 0 = params, 1 = logs. Returns bytes needed. */
size_t fc_vars_list(int kind, char *buf, size_t len);

#ifdef __cplusplus
}
#endif
