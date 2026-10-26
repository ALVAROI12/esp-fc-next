/*
 * Host-side hardware abstraction for fc_core (SITL and unit tests).
 *
 * The Crazyflie flight code talks to hardware through its existing driver
 * interfaces (motors.h, usec_time.h, console.h, cfassert.h). On the target those
 * are implemented by ESP-IDF drivers; on the host they are implemented in
 * host/src and controlled through this API.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Simulated clock returned by usecTimestamp(). */
void fc_host_set_time_us(uint64_t now_us);
void fc_host_advance_time_us(uint64_t delta_us);

/* Last motor command written by the flight code, as a 0..65535 ratio. */
uint16_t fc_host_motor_ratio(uint32_t motor);

/* Log sink for ESP_LOG* / DEBUG_PRINT*; messages above the threshold are dropped. */
void fc_host_set_log_level(int level);
void fc_host_log(int level, const char *tag, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

/* Assertion handling: by default an ASSERT aborts the process. Tests may count instead. */
void fc_host_set_assert_aborts(bool aborts);
uint32_t fc_host_assert_count(void);

#ifdef __cplusplus
}
#endif
