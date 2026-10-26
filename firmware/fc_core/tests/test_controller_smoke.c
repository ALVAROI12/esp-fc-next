/*
 * Smoke test proving the Crazyflie flight path links and behaves on the host:
 * PID controller -> power distribution -> motor HAL.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "controller.h"
#include "fc_host.h"
#include "power_distribution.h"
#include "stabilizer_types.h"

static int failures;

#define CHECK(cond, ...)                                     \
    do {                                                     \
        if (!(cond)) {                                       \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                    \
            fprintf(stderr, "\n");                           \
            failures++;                                      \
        }                                                    \
    } while (0)

static void level_hover_setpoint(setpoint_t *sp, float thrust)
{
    memset(sp, 0, sizeof(*sp));
    sp->thrust = thrust;
    sp->mode.x = modeDisable;
    sp->mode.y = modeDisable;
    sp->mode.z = modeDisable;
    sp->mode.roll = modeAbs;
    sp->mode.pitch = modeAbs;
    sp->mode.yaw = modeVelocity;
}

static void run_controller(control_t *control, setpoint_t *sp, const state_t *state, int ticks)
{
    sensorData_t sensors;
    memset(&sensors, 0, sizeof(sensors));
    sensors.acc.z = 1.0f;

    for (int tick = 1; tick <= ticks; tick++) {
        controller(control, sp, &sensors, state, (uint32_t)tick);
        powerDistribution(control);
    }
}

static void test_level_hover_gives_equal_motors(void)
{
    control_t control = {0};
    setpoint_t sp;
    state_t state;
    memset(&state, 0, sizeof(state));
    state.attitudeQuaternion.w = 1.0f;

    level_hover_setpoint(&sp, 30000.0f);
    run_controller(&control, &sp, &state, 1000);

    CHECK(control.roll == 0 && control.pitch == 0 && control.yaw == 0,
          "level hover produced torque r=%d p=%d y=%d", control.roll, control.pitch, control.yaw);

    uint16_t m1 = fc_host_motor_ratio(0);
    for (uint32_t m = 1; m < 4; m++) {
        CHECK(fc_host_motor_ratio(m) == m1, "motor %u=%u differs from m1=%u", m, fc_host_motor_ratio(m), m1);
    }
    CHECK(m1 > 25000 && m1 < 35000, "hover motor ratio %u not near thrust 30000", m1);
}

static void test_roll_error_is_corrected(void)
{
    control_t control = {0};
    setpoint_t sp;
    state_t state;
    memset(&state, 0, sizeof(state));
    state.attitude.roll = 10.0f;
    state.attitudeQuaternion.w = 1.0f;

    level_hover_setpoint(&sp, 30000.0f);
    run_controller(&control, &sp, &state, 10);

    CHECK(control.roll < 0, "roll +10 deg should command negative roll torque, got %d", control.roll);
}

int main(void)
{
    fc_host_set_log_level(1);

    controllerInit(ControllerTypePID);
    powerDistributionInit();
    CHECK(controllerTest(), "controllerTest failed");
    CHECK(powerDistributionTest(), "powerDistributionTest failed");

    test_level_hover_gives_equal_motors();
    test_roll_error_is_corrected();

    if (failures) {
        fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    printf("controller smoke: all checks passed\n");
    return EXIT_SUCCESS;
}
