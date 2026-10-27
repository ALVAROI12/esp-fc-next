/*
 * Host override of the Crazyflie param.h.
 *
 * The firmware collects PARAM tables in a linker section (ELF only). On the host the
 * tables are kept as ordinary static arrays and registered at load time by a
 * constructor emitted in PARAM_GROUP_STOP, so SITL can read and write parameters
 * by the same "group.name" used by cfclient (see fc_vars.h).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include_next "param.h" /* types and flags; its macros are no-ops under UNIT_TEST_MODE */

#include <stddef.h>

void fc_host_register_params(const struct param_s *table, size_t count);

#undef PARAM_ADD
#undef PARAM_ADD_GROUP
#undef PARAM_GROUP_START
#undef PARAM_GROUP_STOP

#define PARAM_ADD(TYPE, NAME, ADDRESS) {.type = (TYPE), .name = #NAME, .address = (void *)(ADDRESS)},
#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS) {.type = (TYPE), .name = #NAME, .address = (void *)(ADDRESS)},

#define PARAM_GROUP_START(NAME) \
    static const struct param_s __params_##NAME[] = { PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, 0x0)

#define PARAM_GROUP_STOP(NAME)                                                     \
    PARAM_ADD_GROUP(PARAM_GROUP | PARAM_STOP, stop_##NAME, 0x0)                    \
    };                                                                             \
    __attribute__((constructor)) static void __register_params_##NAME(void)        \
    {                                                                              \
        fc_host_register_params(__params_##NAME, sizeof(__params_##NAME) / sizeof(__params_##NAME[0])); \
    }
