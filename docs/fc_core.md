# fc_core: portable flight core (Task 1.2)

`fc_core` is the set of Crazyflie flight algorithms that build both for the ESP32 target and for the
development machine (unit tests, SITL). It lives in `firmware/fc_core/` and compiles the algorithm
sources **in place** from the ESP-Drone subtree, so `git subtree pull` from upstream keeps working.

```sh
make core-test   # host build with ASan/UBSan + tests
```

## Where the cut is

Dependency mapping of `components/core/crazyflie` found two clear tiers:

| Tier | Files | Depends on | Host build |
| --- | --- | --- | --- |
| **A - algorithms** | `pid`, `attitude_pid_controller`, `controller`, `controller_pid`, `controller_indi`, `controller_mellinger`, `position_controller_pid`, `position_controller_indi`, `power_distribution_stock`, `kalman_core`, `kalman_supervisor`, `outlierFilter`, `sensfusion6`, `pptraj`, `pptraj_compressed`, `planner`, `filter`, `num`, `dsp_lib/MatrixFunctions` | Only `log.h`/`param.h` registration macros, `debug_cf.h`, `cfassert.h`, and driver headers (`motors.h`, `usec_time.h`) | **Yes** - in `fc_core` |
| **B - RTOS glue** | `stabilizer`, `estimator_kalman`, `estimator_complementary`, `position_estimator_altitude`, `commander`, `system`, `sensors*`, CRTP/link modules | FreeRTOS tasks, queues, semaphores, tick counts, ESP-IDF drivers | No - target only |

Tier A needs **no source changes**: Crazyflie's `log.h`/`param.h` already compile their registration
macros to nothing under `UNIT_TEST_MODE`.

## Hardware abstraction (HAL)

The seam is Crazyflie's existing driver interfaces rather than a new API:

| Interface | Target implementation | Host implementation (`host/src/fc_host.c`) |
| --- | --- | --- |
| `motors.h` | LEDC PWM / DShot driver | Records ratios; read with `fc_host_motor_ratio()` |
| `usec_time.h` | `esp_timer` | Simulated clock: `fc_host_set_time_us()` / `fc_host_advance_time_us()` |
| `console.h`, `esp_log.h` | UART / CRTP console | stderr with level filter |
| `cfassert.h` | Reset + snapshot | Abort (default) or count, for fault-injection tests |
| `platform.h` | Board profile | Default motor map |

Host shim headers in `host/include/` (`sdkconfig.h`, `esp_log.h`, `esp_err.h`, `FreeRTOS.h`,
`driver/ledc.h`) come first on the include path so they shadow the ESP-IDF versions.

## Next steps

- **Task 1.3:** add a unit-test framework, coverage, and tests per algorithm.
- **Task 1.5/1.6:** add a synchronous `fc_step()` loop (sensors -> estimator -> controller -> mixer)
  that replaces the Tier B `stabilizerTask` + `kalmanTask` pair, used by both SITL and the P4 target, so
  simulation and hardware run the same code path.

## Findings logged for gate G3

| Location | Finding | Impact |
| --- | --- | --- |
| `modules/interface/math3d.h:67` | `fabsf()` applied to an `int` subtraction in the ULP float compare; the subtraction can overflow (undefined behavior) for far-apart values | Low; fix when touched |
| `lib/dsp_lib/include/xtensa_math.h:2170,2183` | Pointer cast to `int32_t` in circular-buffer FIR helpers | Breaks on 64-bit hosts only; not used by fc_core; fine on 32-bit ESP32 |
