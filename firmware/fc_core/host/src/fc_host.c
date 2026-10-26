/*
 * Host implementations of the Crazyflie driver interfaces used by fc_core.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fc_host.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "cfassert.h"
#include "console.h"
#include "motors.h"
#include "platform.h"
#include "usec_time.h"

/* ---- Clock (usec_time.h) ---- */

static uint64_t s_now_us;

void fc_host_set_time_us(uint64_t now_us) { s_now_us = now_us; }
void fc_host_advance_time_us(uint64_t delta_us) { s_now_us += delta_us; }

void initUsecTimer(void) {}
uint64_t usecTimestamp(void) { return s_now_us; }

/* ---- Motors (motors.h) ---- */

static uint16_t s_motor_ratio[NBR_OF_MOTORS];

const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS];

bool motorsInit(const MotorPerifDef **motorMapSelect)
{
    (void)motorMapSelect;
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        s_motor_ratio[i] = 0;
    }
    return true;
}

bool motorsTest(void) { return true; }

bool motorsSetRatio(uint32_t id, uint16_t ratio)
{
    if (id >= NBR_OF_MOTORS) {
        return false;
    }
    s_motor_ratio[id] = ratio;
    return true;
}

int motorsGetRatio(uint32_t id)
{
    return id < NBR_OF_MOTORS ? s_motor_ratio[id] : -1;
}

uint16_t fc_host_motor_ratio(uint32_t motor)
{
    return motor < NBR_OF_MOTORS ? s_motor_ratio[motor] : 0;
}

/* ---- Platform (platform.h) ---- */

const MotorPerifDef **platformConfigGetMotorMapping(void)
{
    return motorMapDefaultBrushed;
}

/* ---- Logging (esp_log.h / console.h) ---- */

static int s_log_level = 2; /* ESP_LOG_WARN */

void fc_host_set_log_level(int level) { s_log_level = level; }

void fc_host_log(int level, const char *tag, const char *fmt, ...)
{
    if (level > s_log_level) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "[%s] ", tag);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

int consolePutchar(int ch) { return fputc(ch, stderr); }

/* ---- Assertions (cfassert.h) ---- */

static bool s_assert_aborts = true;
static uint32_t s_assert_count;

void fc_host_set_assert_aborts(bool aborts) { s_assert_aborts = aborts; }
uint32_t fc_host_assert_count(void) { return s_assert_count; }

void assertFail(char *exp, char *file, int line)
{
    s_assert_count++;
    fprintf(stderr, "ASSERT failed: %s (%s:%d)\n", exp, file, line);
    if (s_assert_aborts) {
        abort();
    }
}
