# fc_core: portable flight core (Task 1.2)

`fc_core` is the set of Crazyflie flight algorithms that build both for the ESP32 target and for the
development machine (unit tests, SITL). It lives in `firmware/fc_core/` and compiles the algorithm
sources **in place** from the ESP-Drone subtree, so `git subtree pull` from upstream keeps working.

**Integration boundary:** the algorithm files are shared with the ESP-IDF build, but the new
`fc_step` runtime and its 4 kHz configuration currently run on the host/SITL only. The production
P4 image still builds ESP-Drone's asynchronous stabilizer/estimator tasks. A passing P4 build does
not validate the new runtime, driver sampling, scheduling deadlines, or hardware safety.

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
| `test_kalman_core` | EKF: position, ToF height, flow velocity/gyro compensation, covariance symmetry/positivity, bounded dead reckoning |
| `test_fc_step` / `_4k` | Calibration, immediate tilted/inverted arm refusal, tumble latch, watchdog feedforward clearing, reset and profile semantics |
| `test_position_feedforward` / `_4k` | Stock-default compatibility and velocity/acceleration feedforward equations |

Run `make core-coverage` for current measured coverage. INDI, Mellinger, the trajectory planner,
and several optional Kalman measurements still lack comprehensive tests; the 80% whole-core
quality gate has not been met.

## fc_step: the synchronous flight loop (Task 1.5)

`include/fc_step.h` / `src/fc_step.c` replace the Tier B `stabilizerTask` + `kalmanTask` + sensor-task glue
with one deterministic call per tick (no RTOS calls, no allocation):

1. Gyro-bias (1024 samples, restarts if moving) and accel-scale (200 samples) calibration, then the same
   2-pole LPFs as the sensor driver (stock gyro 80 Hz, accel 30 Hz).
2. `src/fc_kalman.c`: synchronous port of `estimator_kalman.c` (100 Hz prediction, process noise and
   measurement updates every tick, supervisor reset) with fixed-size queues for position / ToF / flow.
3. Commander watchdog copied from `commander.c` (level after 500 ms, cut motors after 2 s).
4. `controller()` (PID) and `powerDistribution()`; motors only when calibrated, armed and not e-stopped.

The planned host variant uses a 4 kHz main/attitude loop, 150 Hz gyro LPF and 500 Hz Kalman prediction.
Process noise stays on a 1 ms cadence because this legacy EKF's per-call noise injection is not
rate-invariant. Watchdog/tumble times are converted from milliseconds at each variant's rate.
Calibration still uses fixed sample counts (1024 + 200), so its physical duration is shorter at 4 kHz;
this needs a measured startup-calibration policy before hardware use.

The supervisor checks measured gravity as well as the estimate before arming, because the estimate
initially assumes level. It latches tumble (>70 degrees for 300 ms), holds position integrators at
zero before lift-off, and does not feed unactuated commanded thrust to the EKF while disarmed.
This is a basic simulation supervisor, not a redundant or hardware-qualified safety system.

## Firmware variables on the host

`host/include/param.h` and `log.h` override the Crazyflie headers: instead of discarding the PARAM/LOG
tables (ELF-only linker sections), they keep them as static arrays registered by a constructor.
`fc_vars.h` reads/writes them by cfclient name (`posCtlPid.thrustBase`, `posCtl.Zi`, ...).

## Board profiles (Task 1.6a)

- `stock`: no parameter overrides in a **fresh** library instance; it is not an undo operation.
- `whoop2_interim`: 1 kHz main / 500 Hz attitude, G1 interim gains.
- `whoop2_4k`: 4 kHz main/attitude, G1 planned gains.

The whoop profiles add `zKi=0`, hover thrust, velocity feedforward, and acceleration feedforward
(horizontal tilt and vertical thrust linearized around hover). G1 generates `fc_profile_g1.h` only
after the planned and interim profile checks pass. Profile application rejects the wrong rate,
armed changes and attempts to restore `stock` after tuning. `fc_init()` clears dynamics while
preserving the entire runtime configuration; it must not silently restore only some PID gains.

## Next steps

- **Hardware integration:** SPI IMU/DShot drivers, target profile hook and startup/task integration;
  measure 4 kHz deadlines on the P4 before activation (not proved by a host timing configuration).
- **Task 1.6b:** motor vibration model and tested dynamic-notch/RPM filtering.
- **FLOW-01:** flow/ToF alone does not meet absolute XY positioning accuracy. See `sim/README.md`.

## Patches applied to the ESP-Drone subtree

Kept minimal and marked `ESP-FC Next:` in the source so they survive upstream pulls and can be
proposed upstream.

| File | Bug (found by Task 1.3 tests) | Fix |
| --- | --- | --- |
| `modules/src/kalman_core.c` (`kalmanCorePredict`) | Quaternion update divides `sin(a/2)` by `a`; when the gyro reads exactly zero on all axes this is 0/0 = NaN, which trips an `ASSERT` and **resets the flight controller**. Possible in flight with a stuck/failed gyro read; certain in simulation. | Use the limit `sin(a/2)/a -> 1/2` when `a < 1e-6` |
| `modules/src/sensfusion6.c` (`invSqrt`) | 64-bit host over-read and approximate quaternion normalization | Exact `1/sqrtf` on host/FPU targets; fixed-width `memcpy` approximation retained on S2 |
| `modules/interface/stabilizer_types.h` | Fixed loop rates prevented explicit host variants | Overridable main/attitude macros; target defaults unchanged |
| `modules/src/position_controller_pid.c` | Trajectory derivatives discarded, causing tracking lag | Optional velocity/acceleration feedforward with stock defaults of zero |
| `modules/src/controller_pid.c` / interface | Reinitialization kept stale integral and held commands | Explicit dynamics-only reset used by `fc_step` |

## Findings logged for gate G3

| Location | Finding | Impact |
| --- | --- | --- |
| `modules/src/sensfusion6.c` (`invSqrt`) | S2 retains the approximation because it has no FPU | Benchmark separately before promising accuracy/CPU budget on that chip |
| `modules/src/controller_pid.c` | Zero-thrust PID reset only acts on ticks where the 500 Hz attitude loop runs | None in continuous flight; tests must reset on an even tick |
| `modules/interface/math3d.h:67` | `fabsf()` applied to an `int` subtraction in the ULP float compare; the subtraction can overflow (undefined behavior) for far-apart values | Low; fix when touched |
| `lib/dsp_lib/include/xtensa_math.h:2170,2183` | Pointer cast to `int32_t` in circular-buffer FIR helpers | Breaks on 64-bit hosts only; not used by fc_core; fine on 32-bit ESP32 |
