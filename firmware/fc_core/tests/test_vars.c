/* Contract of the host PARAM/LOG-by-name API used by the SITL and ground tools, plus the host HAL shims.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fc_host.h"
#include "fc_step.h"
#include "fc_vars.h"
#include "motors.h"
#include "unity.h"
#include "usec_time.h"

void setUp(void) { fc_init(); }
void tearDown(void) {}

static char *list(int kind)
{
    size_t n = fc_vars_list(kind, NULL, 0);
    char *buf = malloc(n + 1);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL(n, fc_vars_list(kind, buf, n + 1));
    return buf;
}

/* Every registered variable is readable by name (or explicitly unsupported), and writing a parameter's own
 * value back succeeds unless it is read-only. Covers each storage type the firmware registers. */
static void test_every_listed_variable_reads_and_params_write_back(void)
{
    for (int kind = 0; kind <= 1; kind++) {
        char *buf = list(kind), *save = NULL;
        unsigned count = 0, ok = 0;
        for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
            char *dot = strchr(line, '.');
            TEST_ASSERT_NOT_NULL_MESSAGE(dot, line);
            *dot = '\0';
            double v;
            int r = kind ? fc_log_get(line, dot + 1, &v) : fc_param_get(line, dot + 1, &v);
            TEST_ASSERT_TRUE_MESSAGE(r == FC_VAR_OK || r == FC_VAR_UNSUPPORTED, dot + 1);
            count++;
            if (r != FC_VAR_OK) continue;
            ok++;
            if (kind == 0) {
                int w = fc_param_set(line, dot + 1, v);
                TEST_ASSERT_TRUE_MESSAGE(w == FC_VAR_OK || w == FC_VAR_READ_ONLY, dot + 1);
                double back;
                TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_get(line, dot + 1, &back));
                TEST_ASSERT_TRUE_MESSAGE(v == back, dot + 1);
            }
        }
        TEST_ASSERT_GREATER_THAN(20, count);
        TEST_ASSERT_GREATER_THAN(count / 2, ok);
        free(buf);
    }
}

static void test_unknown_names_and_kinds_are_rejected(void)
{
    double v = 0;
    TEST_ASSERT_EQUAL(FC_VAR_NOT_FOUND, fc_param_get("nope", "x", &v));
    TEST_ASSERT_EQUAL(FC_VAR_NOT_FOUND, fc_param_set("nope", "x", 1));
    TEST_ASSERT_EQUAL(FC_VAR_NOT_FOUND, fc_log_get("fcSafety", "nope", &v));
    TEST_ASSERT_EQUAL(0, fc_vars_list(2, NULL, 0));
}

static void test_param_set_converts_to_storage_type(void)
{
    double v;
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_set("posCtlPid", "thrustBase", 41234.9));
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_get("posCtlPid", "thrustBase", &v));
    TEST_ASSERT_TRUE(v == 41234); /* uint16 storage truncates */
}

static void test_host_motor_clock_log_and_assert_shims(void)
{
    TEST_ASSERT_TRUE(motorsInit(NULL));
    TEST_ASSERT_TRUE(motorsTest());
    TEST_ASSERT_TRUE(motorsSetRatio(0, 1234));
    TEST_ASSERT_EQUAL(1234, motorsGetRatio(0));
    TEST_ASSERT_EQUAL(1234, fc_host_motor_ratio(0));
    TEST_ASSERT_FALSE(motorsSetRatio(NBR_OF_MOTORS, 1));
    TEST_ASSERT_EQUAL(-1, motorsGetRatio(NBR_OF_MOTORS));
    TEST_ASSERT_EQUAL(0, fc_host_motor_ratio(NBR_OF_MOTORS));

    fc_host_set_time_us(1000);
    fc_host_advance_time_us(500);
    initUsecTimer();
    TEST_ASSERT_EQUAL_UINT64(1500, usecTimestamp());

    fc_host_set_log_level(5);
    fc_host_log(3, "test", "host log %d\n", 1); /* printed */
    fc_host_set_log_level(0);
    fc_host_log(3, "test", "suppressed\n");

    fc_host_set_assert_aborts(false);
    uint32_t before = fc_host_assert_count();
    assertFail("x", "file", 1);
    TEST_ASSERT_EQUAL_UINT32(before + 1, fc_host_assert_count());
    fc_host_set_assert_aborts(true);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_every_listed_variable_reads_and_params_write_back);
    RUN_TEST(test_unknown_names_and_kinds_are_rejected);
    RUN_TEST(test_param_set_converts_to_storage_type);
    RUN_TEST(test_host_motor_clock_log_and_assert_shims);
    return UNITY_END();
}
