/* Target glue (target/fc_runtime.c) on the host: measurement hand-off, setpoint forwarding, arm/stop sync.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include <string.h>

#include "commander.h"
#include "fc_host.h"
#include "fc_runtime.h"
#include "fc_step.h"
#include "fc_vars.h"
#include "pm_esplane.h"
#include "queue.h"
#include "system.h"
#include "task.h"
#include "unity.h"

/* ---- fakes for the commander and system modules (queue fake: shim/queue.c) ---- */
static TickType_t rtosTick;
TickType_t xTaskGetTickCount(void) { return rtosTick; }

static setpoint_t commanded;
static bool armedFake;
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
    (void)state;
    *setpoint = commanded;
}
bool systemIsArmed(void) { return armedFake; }
static float batteryFake;
float pmGetBatteryVoltage(void) { return batteryFake; }

/* ---- helpers ---- */
static sensorData_t level;
static setpoint_t sp;
static state_t st;

static void tick(bool estop)
{
    rtosTick++; /* 1 kHz RTOS tick, same as the IMU-driven loop */
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
    batteryFake = 0; /* power management not initialised: no reading */
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
    for (unsigned i = 0; i < FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS + 100); i++) {
        commanded.timestamp = rtosTick;
        tick(false);
    }
    TEST_ASSERT_EQUAL(modeAbs, fc_active_setpoint()->mode.x);
    for (unsigned i = 0; i < FC_MS_TO_TICKS(FC_SETPOINT_TIMEOUT_STABILIZE_MS + 100); i++) tick(false);
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x); /* stale packet: level, no XY hold */
    /* After its own timeout the commander hands back nullSetpoint (timestamp 0) or a back-dated copy
     * (commanderNotifySetpointsStop): a changed timestamp, but not a new packet. */
    commanded.timestamp = 0;
    for (unsigned i = 0; i < 10; i++) tick(false);
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
    commanded.timestamp = rtosTick - pdMS_TO_TICKS(600);
    for (unsigned i = 0; i < 10; i++) tick(false);
    TEST_ASSERT_EQUAL(modeDisable, fc_active_setpoint()->mode.x);
}

/* The arm source is a level (forceArm param): a refused or supervisor-cancelled arm must not retry itself. */
static void test_arm_is_edge_triggered(void)
{
    armedFake = true;
    tick(false); /* calibrating: refused */
    ready();
    TEST_ASSERT_FALSE(fc_is_armed()); /* still requested, but no automatic re-arm once it becomes possible */
    armedFake = false;
    tick(false);
    armedFake = true;
    tick(false);
    TEST_ASSERT_TRUE(fc_is_armed());
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

/* Pack voltage reaches fc_step's low-battery check (refuses arming here); 0 V (pm not running) is ignored. */
static void test_battery_voltage_reaches_fc_step(void)
{
    ready();
    armedFake = true;
    tick(false);
    TEST_ASSERT_TRUE(fc_is_armed()); /* 0 V never blocks */
    armedFake = false;
    batteryFake = 3.1f;
    for (unsigned i = 0; i < FC_MS_TO_TICKS(200); i++) tick(false);
    TEST_ASSERT_EQUAL_FLOAT(3.1f, (float)safety_log("batteryV"));
}

/* The host clock only moves between ticks, so fc_step costs 0 us here; the counters must still be sane. */
static void test_step_timing_counters(void)
{
    ready();
    double us, maxUs, overruns;
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcTime", "stepUs", &us));
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcTime", "stepMaxUs", &maxUs));
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_log_get("fcTime", "overruns", &overruns));
    TEST_ASSERT_TRUE(us <= maxUs);
    TEST_ASSERT_EQUAL(0, (int)overruns);
}

/* Safe mode is configured from the ground (param fcRun.safeMode) and only takes effect while disarmed. */
static void test_safe_mode_param_applies_while_disarmed(void)
{
    ready();
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_set("fcRun", "safeMode", 1));
    tick(false);
    armedFake = true;
    tick(false);
    TEST_ASSERT_FALSE(fc_is_armed()); /* safe mode needs fresh camera position and ToF */
    positionMeasurement_t pos = {.stdDev = 0.01f};
    tofMeasurement_t tof = {.distance = 0, .stdDev = 0.01f};
    TEST_ASSERT_TRUE(fcRuntimeEnqueuePosition(&pos));
    TEST_ASSERT_TRUE(fcRuntimeEnqueueTof(&tof));
    armedFake = false;
    tick(false);
    armedFake = true;
    tick(false);
    TEST_ASSERT_TRUE(fc_is_armed());
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_set("fcRun", "safeMode", 0));
    tick(false);
    TEST_ASSERT_TRUE(fc_set_failsafe_enabled(true) == false); /* still armed: config is locked */
    armedFake = false;
    tick(false);
    TEST_ASSERT_FALSE(fc_is_armed());
    TEST_ASSERT_EQUAL(FC_VAR_OK, fc_param_set("fcRun", "safeMode", 0));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_measurements_from_other_tasks_reach_fc_step);
    RUN_TEST(test_only_new_setpoints_refresh_the_command_watchdog);
    RUN_TEST(test_arm_follows_system_and_emergency_stop_latches);
    RUN_TEST(test_arm_is_edge_triggered);
    RUN_TEST(test_battery_voltage_reaches_fc_step);
    RUN_TEST(test_step_timing_counters);
    RUN_TEST(test_safe_mode_param_applies_while_disarmed);
    return UNITY_END();
}
