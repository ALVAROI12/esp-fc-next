/*
 * Registry of the Crazyflie PARAM/LOG tables kept by the host param.h / log.h overrides.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_vars.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "log.h"
#include "param.h"

#define MAX_TABLES 128

typedef struct {
    const void *entries; /* struct param_s or struct log_s: identical layout */
    size_t count;
} table_t;

static table_t s_tables[2][MAX_TABLES];
static size_t s_table_count[2];

typedef struct {
    uint8_t type;
    char *name;
    void *address;
} entry_t;

_Static_assert(sizeof(entry_t) == sizeof(struct param_s), "param_s layout changed");
_Static_assert(sizeof(entry_t) == sizeof(struct log_s), "log_s layout changed");

static void register_table(int kind, const void *entries, size_t count)
{
    if (s_table_count[kind] < MAX_TABLES) {
        s_tables[kind][s_table_count[kind]++] = (table_t){entries, count};
    }
}

void fc_host_register_params(const struct param_s *table, size_t count) { register_table(0, table, count); }
void fc_host_register_logs(const struct log_s *table, size_t count) { register_table(1, table, count); }

static int is_group_start(int kind, uint8_t type)
{
    return kind == 0 ? (type & PARAM_GROUP) && (type & PARAM_START) : (type & LOG_GROUP) && (type & LOG_START);
}

static const entry_t *find(int kind, const char *group, const char *name)
{
    for (size_t t = 0; t < s_table_count[kind]; t++) {
        const entry_t *e = s_tables[kind][t].entries;
        size_t n = s_tables[kind][t].count;
        if (n < 2 || !is_group_start(kind, e[0].type) || strcmp(e[0].name, group) != 0) {
            continue;
        }
        for (size_t i = 1; i + 1 < n; i++) {
            if (strcmp(e[i].name, name) == 0) {
                return &e[i];
            }
        }
    }
    return NULL;
}

/* ---- parameters: size in bits 0-1, float in bit 2, unsigned in bit 3, read-only bit 6 ---- */

static int param_read(const entry_t *e, double *value)
{
    uint8_t t = e->type & ~PARAM_RONLY;
    switch (t) {
    case PARAM_UINT8: *value = *(uint8_t *)e->address; break;
    case PARAM_INT8: *value = *(int8_t *)e->address; break;
    case PARAM_UINT16: *value = *(uint16_t *)e->address; break;
    case PARAM_INT16: *value = *(int16_t *)e->address; break;
    case PARAM_UINT32: *value = *(uint32_t *)e->address; break;
    case PARAM_INT32: *value = *(int32_t *)e->address; break;
    case PARAM_FLOAT: *value = *(float *)e->address; break;
    default: return FC_VAR_UNSUPPORTED;
    }
    return FC_VAR_OK;
}

int fc_param_get(const char *group, const char *name, double *value)
{
    const entry_t *e = find(0, group, name);
    return e ? param_read(e, value) : FC_VAR_NOT_FOUND;
}

int fc_param_set(const char *group, const char *name, double value)
{
    const entry_t *e = find(0, group, name);
    if (!e) {
        return FC_VAR_NOT_FOUND;
    }
    if (e->type & PARAM_RONLY) {
        return FC_VAR_READ_ONLY;
    }
    switch (e->type) {
    case PARAM_UINT8: *(uint8_t *)e->address = (uint8_t)value; break;
    case PARAM_INT8: *(int8_t *)e->address = (int8_t)value; break;
    case PARAM_UINT16: *(uint16_t *)e->address = (uint16_t)value; break;
    case PARAM_INT16: *(int16_t *)e->address = (int16_t)value; break;
    case PARAM_UINT32: *(uint32_t *)e->address = (uint32_t)value; break;
    case PARAM_INT32: *(int32_t *)e->address = (int32_t)value; break;
    case PARAM_FLOAT: *(float *)e->address = (float)value; break;
    default: return FC_VAR_UNSUPPORTED;
    }
    return FC_VAR_OK;
}

/* ---- logs ---- */

int fc_log_get(const char *group, const char *name, double *value)
{
    const entry_t *e = find(1, group, name);
    if (!e) {
        return FC_VAR_NOT_FOUND;
    }
    if (e->type & LOG_BY_FUNCTION) {
        return FC_VAR_UNSUPPORTED;
    }
    switch (e->type) {
    case LOG_UINT8: *value = *(uint8_t *)e->address; break;
    case LOG_UINT16: *value = *(uint16_t *)e->address; break;
    case LOG_UINT32: *value = *(uint32_t *)e->address; break;
    case LOG_INT8: *value = *(int8_t *)e->address; break;
    case LOG_INT16: *value = *(int16_t *)e->address; break;
    case LOG_INT32: *value = *(int32_t *)e->address; break;
    case LOG_FLOAT: *value = *(float *)e->address; break;
    default: return FC_VAR_UNSUPPORTED;
    }
    return FC_VAR_OK;
}

size_t fc_vars_list(int kind, char *buf, size_t len)
{
    size_t used = 0;
    if (kind != 0 && kind != 1) {
        return 0;
    }
    for (size_t t = 0; t < s_table_count[kind]; t++) {
        const entry_t *e = s_tables[kind][t].entries;
        size_t n = s_tables[kind][t].count;
        for (size_t i = 1; i + 1 < n; i++) {
            int room = buf && used < len;
            int w = snprintf(room ? buf + used : NULL, room ? len - used : 0, "%s.%s\n", e[0].name, e[i].name);
            used += (size_t)(w > 0 ? w : 0);
        }
    }
    return used + 1;
}
