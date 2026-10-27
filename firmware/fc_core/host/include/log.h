/*
 * Host override of the Crazyflie log.h: keeps LOG tables and registers them at load
 * time (see param.h in this directory for the rationale).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include_next "log.h"

#include <stddef.h>

void fc_host_register_logs(const struct log_s *table, size_t count);

#undef LOG_ADD
#undef LOG_ADD_BY_FUNCTION
#undef LOG_ADD_GROUP
#undef LOG_GROUP_START
#undef LOG_GROUP_STOP

#define LOG_ADD(TYPE, NAME, ADDRESS) {.type = (TYPE), .name = #NAME, .address = (void *)(ADDRESS)},
#define LOG_ADD_BY_FUNCTION(TYPE, NAME, ADDRESS) {.type = (TYPE) | LOG_BY_FUNCTION, .name = #NAME, .address = (void *)(ADDRESS)},
#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS) {.type = (TYPE), .name = #NAME, .address = (void *)(ADDRESS)},

#define LOG_GROUP_START(NAME) \
    static const struct log_s __logs_##NAME[] = { LOG_ADD_GROUP(LOG_GROUP | LOG_START, NAME, 0x0)

#define LOG_GROUP_STOP(NAME)                                                 \
    LOG_ADD_GROUP(LOG_GROUP | LOG_STOP, stop_##NAME, 0x0)                    \
    };                                                                       \
    __attribute__((constructor)) static void fc_register_logs_##NAME(void)    \
    {                                                                        \
        fc_host_register_logs(__logs_##NAME, sizeof(__logs_##NAME) / sizeof(__logs_##NAME[0])); \
    }
