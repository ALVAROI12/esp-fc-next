/* Target glue (target/fc_runtime.c) on the host: measurement hand-off, setpoint forwarding, arm/stop sync.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include <string.h>

#include "commander.h"
#include "fc_host.h"
#include "fc_runtime.h"
#include "fc_step.h"
#include "fc_vars.h"
#include "queue.h"
#include "system.h"
#include "unity.h"

/* ---- fakes for the FreeRTOS queue, commander and system modules ---- */
QueueHandle_t xQueueCreateStatic(UBaseType_t len, UBaseType_t size, uint8_t *buf, StaticQueue_t *q)
{
    *q = (StaticQueue_t){.buf = buf, .len = len, .size = size};
    return q;
}

BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait)
{
    (void)wait;
    if (q->count == q->len) return pdFALSE;
    memcpy(q->buf + ((q->head + q->count) % q->len) * q->size, item, q->size);
    q->count++;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait)
{
    (void)wait;
    if (!q->count) return pdFALSE;
    memcpy(item, q->buf + q->head * q->size, q->size);
    q->head = (q->head + 1) % q->len;
    q->count--;
    return pdTRUE;
}

static setpoint_t commanded;
static bool armedFake;
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
    (void)state;
    *setpoint = commanded;
}
bool systemIsArmed(void) { return armedFake; }

/* ---- helpers ---- */
static sensorData_t level;
static setpoint_t sp;
static state_t st;

static void tick(bool estop)
{
    fc_host_advance_time_us(1000000u / FC_STEP_RATE_HZ);
    fcRuntimeStep(&level, &sp, &st, estop);
}

static void ready(void)
{
    for (unsigned i = 0; i < 1024 + 200; i++) tick(false);
    TEST_ASSERT_EQUAL(FC_PHASE_READY, fc_phase());
}

static double safety_log(const char *name)
{
    double v;
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcSafety", name, &v));
    return v;
}

void setUp(void)
{
    fc_host_set_log_level(1);
    memset(&commanded, 0, sizeof(commanded));
    commanded.mode.x = commanded.mode.y = commanded.mode.z = modeAbs;
    commanded.position.z = 1;
    armedFake = false;
    memset(&level, 0, sizeof(level));
    level.acc.z = 1;
    fcRuntimeInit();
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(false)); /* config survives fc_init */
}
void tearDown(void) {}

static void test_measurements_from_other_tasks_reach_fc_step(void)
{
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(true));
    ready();
    poseMeasurement_t pose = {.x = 0.1f, .y = 0.2f, .z = 0.0f, .stdDevPos = 0.01f};
    tofMeasurement_t tof = {.distance = 0.0f, .stdDev = 0.01f};
    TEST_ASSERT_TRUE(fcRuntimeEnqueuePose(&pose));
    TEST_ASSERT_TRUE(fcRuntimeEnqueueTof(&tof));
    TEST_ASSERT_EQUAL(0, (int)safety_log("posFresh"));
    tick(false);
    TEST_ASSERT_EQUAL(1, (int)safety_log("posFresh"));
    TEST_ASSERT_EQUAL(1, (int)safety_log("tofFresh"));

    positionMeasurement_t pos = {.x = 0, .y = 0, .z = 0, .stdDev = 0.01f};
    unsigned accepted = 0;
    while (fcRuntimeEnqueuePosition(&pos) && accepted < 100) accepted++;
    TEST_ASSERT_EQUAL(32, accepted); /* bounded, never blocks the producer */
    tick(false);
    TEST_ASSERT_TRUE(fcRuntimeEnqueuePosition(&pos)); /* drained by the flight loop */
}

/* The commander keeps returning its last packet; only a new timestamp may refresh fc_step's watchdog. */
static void test_only_new_setpoints_refresh_the_command_watchdog(void)
{
    ready();
    commanded.timestamp = 100;
    for (unsigned i = 0; i < FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS + 100); i++) {
        commanded.timestamp++;
        tick(false);
    }
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.x);
    for (unsigned i = 0; i < FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS + 100); i++) tick(false);
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x); /* stale packet: level, no XY hold */
}

static void test_arm_follows_system_and_emergency_stop_latches(void)
{
    ready();
    armedFake = true;
    tick(false);
    TEST_ASSERT_TRUE(fc_is_armed());
    TEST_ASSERT_TRUE(st.attitudeQuaternion.w > 0.99f); /* estimate copied back for logging */
    armedFake = false;
    tick(false);
    TEST_ASSERT_FALSE(fc_is_armed());
    armedFake = true;
    tick(true);
    TEST_ASSERT_FALSE(fc_is_armed());
    for (unsigned i = 0; i < 100; i++) tick(false); /* stabilizer flag cleared, system still armed */
    TEST_ASSERT_FALSE(fc_is_armed());
    for (unsigned m = 0; m < 4; m++) TEST_ASSERT_EQUAL(0, fc_host_motor_ratio(m));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_measurements_from_other_tasks_reach_fc_step);
    RUN_TEST(test_only_new_setpoints_refresh_the_command_watchdog);
    RUN_TEST(test_arm_follows_system_and_emergency_stop_latches);
    return UNITY_END();
}
