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

## Tests and coverage (Task 1.3)

Unit tests use [Unity](https://github.com/ThrowTheSwitch/Unity) v2.7.0 (fetched by CMake), one
executable per file in `firmware/fc_core/tests/`, all run under ASan/UBSan.

```sh
make core-test       # build + run all tests with sanitizers
make core-coverage   # coverage summary + HTML report in build/coverage/index.html
```

| Suite | What it checks |
| --- | --- |
| `test_num` | Saturation, constrain, deadband continuity, fp16 round trip and special values |
| `test_filter` | 2-pole LPF: unity DC gain, -3 dB at cutoff, flat passband, stop-band roll-off, NaN rejection, reset |
| `test_pid` | P/I/D terms, anti-windup integral limit, output limit, filtered derivative, reset |
| `test_power_distribution` | Quad-X mixer signs per axis, yaw pairs, thrust preservation, saturation, stop |
| `test_controller_pid` | Cascaded PID: torque-free level hover, roll correction and antisymmetry, pitch, yaw rate, zero-thrust reset |
| `test_sensfusion6` | Mahony filter: level hold, gyro yaw integration, tilt convergence, zero-accel safety |
| `test_kalman_core` | EKF: position convergence, variance shrink, covariance symmetry/positivity, bounded dead reckoning, externalized state |

Coverage of the flight path in use today (Task 1.3): `num.c` 100%, `pid.c` 91%, `sensfusion6.c` 91%,
`attitude_pid_controller.c` 88%, `filter.c` 88%, `controller_pid.c` 85%, `kalman_core.c` 54%.
Not yet tested: INDI, Mellinger, trajectory planner, outlier filter, and the Kalman flow/ToF/baro updates
(Task 1.6 adds the latter, which the indoor swarm needs).

## fc_step: the synchronous flight loop (Task 1.5)

`include/fc_step.h` / `src/fc_step.c` replace the Tier B `stabilizerTask` + `kalmanTask` + sensor-task glue
with one call per 1 kHz tick, portable to the P4 (no RTOS calls, no allocation):

1. Gyro-bias (1024 samples, restarts if moving) and accel-scale (200 samples) calibration, then the same
   2-pole LPFs as the sensor driver (gyro 80 Hz, accel 30 Hz).
2. `src/fc_kalman.c`: synchronous port of `estimator_kalman.c` (100 Hz prediction, process noise and
   measurement updates every tick, supervisor reset) with fixed-size queues for position / ToF / flow.
3. Commander watchdog copied from `commander.c` (level after 500 ms, cut motors after 2 s).
4. `controller()` (PID) and `powerDistribution()`; motors only when calibrated, armed and not e-stopped.

## Firmware variables on the host

`host/include/param.h` and `log.h` override the Crazyflie headers: instead of discarding the PARAM/LOG
tables (ELF-only linker sections), they keep them as static arrays registered by a constructor.
`fc_vars.h` reads/writes them by cfclient name (`posCtlPid.thrustBase`, `posCtl.Zi`, ...): 84 parameters
and 103 log variables, with no changes to the subtree.

## Next steps

- **Task 1.6:** run `fc_step` on the P4 at the planned 4 kHz, board profile with the G1 gains and SITL
  findings (zKi, thrustBase), velocity feedforward, exact `1/sqrtf`, flow/ToF fusion tests.

## Patches applied to the ESP-Drone subtree

Kept minimal and marked `ESP-FC Next:` in the source so they survive upstream pulls and can be
proposed upstream.

| File | Bug (found by Task 1.3 tests) | Fix |
| --- | --- | --- |
| `modules/src/kalman_core.c` (`kalmanCorePredict`) | Quaternion update divides `sin(a/2)` by `a`; when the gyro reads exactly zero on all axes this is 0/0 = NaN, which trips an `ASSERT` and **resets the flight controller**. Possible in flight with a stuck/failed gyro read; certain in simulation. | Use the limit `sin(a/2)/a -> 1/2` when `a < 1e-6` |
| `modules/src/sensfusion6.c` (`invSqrt`) | Reinterprets a 4-byte `float` as `long` (8 bytes on 64-bit PCs): stack over-read (ASan) and garbage attitude in SITL; type-punning UB on all targets | `int32_t` + `memcpy` |

## Findings logged for gate G3

| Location | Finding | Impact |
| --- | --- | --- |
| `modules/src/sensfusion6.c` (`invSqrt`) | One Newton step fast inverse sqrt: up to ~0.175% error, quaternion norm settles at ~0.998 | Small attitude scale error; replace with `1.0f/sqrtf()` (P4 has a hardware FPU) in Task 1.6 |
| `modules/src/controller_pid.c` | Zero-thrust PID reset only acts on ticks where the 500 Hz attitude loop runs | None in continuous flight; tests must reset on an even tick |
| `modules/interface/math3d.h:67` | `fabsf()` applied to an `int` subtraction in the ULP float compare; the subtraction can overflow (undefined behavior) for far-apart values | Low; fix when touched |
| `lib/dsp_lib/include/xtensa_math.h:2170,2183` | Pointer cast to `int32_t` in circular-buffer FIR helpers | Breaks on 64-bit hosts only; not used by fc_core; fine on 32-bit ESP32 |
