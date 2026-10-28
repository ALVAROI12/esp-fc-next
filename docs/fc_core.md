# fc_core: portable flight core (Task 1.2)

`fc_core` is the set of Crazyflie flight algorithms that build both for the ESP32 target and for the
development machine (unit tests, SITL). It lives in `firmware/fc_core/` and compiles the algorithm
sources **in place** from the ESP-Drone subtree, so `git subtree pull` from upstream keeps working.

**Integration boundary:** the algorithm files are shared with the ESP-IDF build. The default
production P4 image still builds ESP-Drone's asynchronous stabilizer/estimator tasks. With
`CONFIG_ESPFC_FC_STEP=y` (`make flight-fcstep`, build dir `firmware/flight/build-<target>-fcstep`)
the stabilizer task instead calls `fc_step` once per 1 kHz IMU sample through the target-only glue
`firmware/fc_core/target/fc_runtime.c`. The P4 image is still the portable diagnostic profile
(`CONFIG_TARGET_PORTABLE_SAFE`: no stabilizer, board outputs off until a P4 board profile exists), so
the linker drops fc_step there; the real link check is the ESP32-S3 drone target,
`make flight-fcstep FLIGHT_TARGET=esp32s3`.

- `estimatorEnqueuePosition/Pose/TOF/Flow` hand measurements to one static FreeRTOS queue that the
  stabilizer drains each tick into `fc_push_*` (pose keeps position only; TDOA/height/yaw-error unused).
- Only commander setpoints with a new timestamp reach `fc_set_setpoint`, so fc_step's own watchdog
  and failsafe run instead of the commander timeouts.
- `systemIsArmed()` drives `fc_arm()` (which can refuse); the stabilizer emergency stop latches
  `fc_emergency_stop()` until reboot. fc_step drives the motors; `state` is copied back for logs.

- The ESP-Drone sensor driver keeps bias, scale and gravity alignment but skips its 80/30 Hz LPF in this
  mode; fc_step applies the same filters, so samples are filtered once.

The 4 kHz configuration remains
host/SITL only. A passing P4 build does not validate driver sampling, scheduling deadlines, or
hardware safety.

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
| `test_rpm_notch` | Bypass, 1x/2x rejection, low-frequency preservation, retuning sweep, activation continuity, invalid/aliased RPM |
| `test_failsafe` / `_4k` | Opt-in arming/grace, fixed-pose hold, latched descent, height freshness/dwell, terminal motor/rearm behavior, emergency overrides and disabled-mode legacy fusion |

Run `make core-coverage` for current measured coverage. INDI, Mellinger, the trajectory planner,
and several optional Kalman measurements still lack comprehensive tests; the 80% whole-core
quality gate has not been met.
After flow-braking regressions: 54.3% lines, 55.4% functions and 56.7% branches in the
configured coverage scope. Passing these tests does not close G3.

## fc_step: the synchronous flight loop (Task 1.5)

`include/fc_step.h` / `src/fc_step.c` replace the Tier B `stabilizerTask` + `kalmanTask` + sensor-task glue
with one deterministic call per tick (no RTOS calls, no allocation):

1. Gyro-bias (1024 samples, restarts if moving) and accel-scale (200 samples) calibration, then an optional
   RPM notch bank (Task 1.6b) and the same 2-pole LPFs as the sensor driver (stock gyro 80 Hz, accel 30 Hz).
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

Input plausibility (both modes): an IMU sample that is non-finite or beyond `FC_IMU_MAX_GYRO_DPS` /
`FC_IMU_MAX_ACC_G` never reaches calibration, LPF or EKF; the last good sample is held. More than
`FC_IMU_BAD_MAX_MS` (5 ms) of consecutive bad samples disarms (legacy) or FAULTs (safe mode). Before this,
one NaN gyro sample made the estimate NaN permanently and tripped `cf_sqrt` ASSERTs (motor stop on target).
Position/ToF/flow pushes with a non-finite value or non-positive std are dropped in legacy mode too.

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

## RPM notch bank (Task 1.6b)

`include/fc_rpm_notch.h` / `src/fc_rpm_notch.c` implement a fixed-size bank of RBJ second-order
notches: one per motor at the mechanical rotation frequency and one at its second harmonic, shared
coefficients across the three gyro axes, separate histories. No allocation, no RTOS, no FFT.

- Disabled in a fresh process. Like other runtime tuning, enable/Q persist across `fc_init()`;
  histories and telemetry clear. `fc_set_rpm_notch(enabled, q)` is rejected while armed or with
  Q outside [1, 50]; the default and the margin-checked setting are Q = 5.
- `fc_push_rpm()` takes **mechanical** RPM from ESC telemetry, not electrical eRPM. A packet with any
  non-finite, negative or implausible value is rejected as a whole and does not refresh the age.
- Targets below 500 Hz or at/above 45 % of the sample rate are skipped to limit control-band phase
  lag and avoid targeting aliased motor lines. At 1 kHz the upper cap is 450 Hz: every section
  bypasses. A below-floor fundamental may still have an in-band second harmonic.
- Coefficients retune at 100 Hz; history is preserved across retuning, and activation seeds history
  with the current sample to avoid a DC step. After 100 ms without a valid packet the bank clears and
  becomes an exact bypass.
- `fcGyro.{rawX,notchX,filteredX,rpmFresh}` log the three gyro stages and telemetry freshness for
  blackbox comparison; only the roll axis is exposed on the host.

Measured in the 4 kHz SITL (`make vibration`): the modeled 1x/2x artifact drops from 3.8 to
0.20 deg/s RMS at 12 deg/s assumed amplitude (11.5 to 0.50 at 36). High-frequency motor-command
activity falls from 3.1 % to 1.9 % (zero-vibration baseline 1.9 %). Hover and figure-8 tracking are
unchanged, because the existing 150 Hz gyro LPF already attenuates this band before the controller:
the notch is justified here by sensor/actuator high-frequency stress, not by demonstrated position
accuracy. At modeled hover RPM the bank adds about 0.9 deg of phase at 10 Hz (1.8 deg at 20 Hz);
the worst of 32 uncertainty corners at hover is 47.5 deg PM / 7.2 dB GM. Review found that the
initial 150 Hz floor failed away from hover (30.9 deg / 3.8 dB at a 150 Hz fundamental). The floor
is now 500 Hz per section, and `make vibration` gates a sampled RPM sweep over 32 corners,
including harmonic-only operation and activation/cap boundaries: worst sampled margins are
46.2 deg PM / 6.9 dB GM. See `sim/reports/vibration/README.md` for exact frequencies.
These fixed-coefficient Q=5 checks do not
prove continuous/arbitrary independent RPM, other Q values, or hardware stability.

Dropout regressions check actual `notchX == rawX` signal bypass as well as the freshness flag;
SITL records position through loss and recovery. Reported RPM errors are constant positive
common-mode scale biases, not per-motor random noise or jitter.

All vibration amplitudes are assumptions. Real prop/motor imbalance, structural resonances (which
this model does not represent), RPM calibration, pole-pair conversion and ESC telemetry latency need
hardware blackbox measurements before flight.

## Next steps

- **Hardware integration:** SPI IMU/DShot drivers, target profile hook and startup/task integration;
  measure 4 kHz deadlines on the P4 before activation (not proved by a host timing configuration).
- **Link-loss safety:** see the opt-in design and evidence below; validate range/ground offsets,
  stale-sensor detection and physical touchdown before enabling on hardware.
- **Four-drone swarm simulation:** coordinated takeoff/landing, formation, collision and packet loss.
- **FLOW-01:** flow/ToF alone does not meet absolute XY positioning accuracy. See `sim/README.md`.

## Opt-in latched hold-and-land

Design: `docs/plans/2026-10-27-link-loss-design.md`. The stock/reference watchdog remains the
default; the new policy is enabled explicitly with `fc_set_failsafe_enabled(true)` while disarmed.
Safe mode requires fresh external position and independent downward ToF before arming. It does
not activate a new production P4 task loop or change any CRTP packet formats.

| State | Behavior |
| --- | --- |
| NORMAL (0) | Use mission commands; after 500 ms command silence capture current estimated pose |
| HOLD (1) | Fixed pose, zero stale trajectory feedforward; after 1.5 s enter LAND; fresh commands may resume before LAND |
| LAND (2) | Latched descent, nominally 0.25 m/s, slowing to 0.10 m/s below 15 cm; renewed mission commands cannot cancel it |
| LAND, blind | Camera and ToF both stale above 15 cm (e.g. camera lost above the 1.3 m ToF range): descend for at most 6 s on the IMU-propagated estimate with thrust held at 0.97-0.99 x the hover thrust learned while height was aided, so an unseen accel bias cannot make it climb; the band stays 1 s after a height source returns (reason bit 32) |
| LANDED (3) | Height <= 4 cm and estimated vertical speed <= 0.15 m/s for 500 ms (thrust capped at 0.99 x hover during the dwell, so a biased estimate cannot hold it a few cm up): disarm and block automatic rearm |
| FAULT (4) | No trustworthy height near the ground or after the blind budget, invalid state, or landing timeout: motor stop; **not** successful controlled landing |

Camera input older than 300 ms triggers LAND even if commands still arrive. XY hold is allowed only
with fresh tracking; after tracking loss it uses local velocity braking when qualified flow/range
is available, otherwise level roll/pitch. It never resumes absolute XY hold in that landing.
Independent ToF (200 ms freshness, tilt checked)
supports vertical control; fresh camera height relative to the arming ground is a fallback. A 20 s
landing timeout bounds a stuck descent. Emergency stop, explicit disarm and tumble remain authoritative.
LANDED/FAULT require explicit ground reinitialization, not automatic restart when the radio recovers.

`fc_failsafe_state()` and `fcSafety` logs expose state/reason/held pose and input freshness.
`make failsafe` generates `sim/reports/failsafe/README.md` and height/speed/state plots from C SITL;
the viewer scenario is `?scenario=failsafe`. `SitlDrone(..., safe_landing=True)` adds independent
local ToF; `link_up`, `tracking_up` and `height_up` independently gate command, camera and range delivery.

Baseline without local flow, seed-9 SITL at 1/4 kHz: controlled cases reach LANDED about 5.24-7.05 s after loss,
max descent about 0.263 m/s, sampled final approach 0.093-0.102 m/s, and motor disarm at ground.
Restored commands during LAND do not restart flight. Moving camera-only loss still produces
about **1.7 m XY excursion**, rising to about **2.0 m** with the restored/jumped camera reference
(combined loss about 1.3 m), so this is not collision-safe swarm flight.
The all-height-loss cases instead stop motors around 1 m height and fall: a tested fault path,
not a safe-landing pass. See the report for the exact cases, gates and sampled metrics.

Review regressions also cover local-range expiry during the touchdown dwell (camera-height fallback
must reset it; losing both sources must FAULT), and restored camera input with a (+0.5, -0.4, 0) m
reference jump during LAND. Without local flow, controller roll/pitch targets remain zero after restoration;
with local braking engaged, restored camera packets are rejected until the landing ends.
Safety qualification does not narrow the disabled/legacy estimator's range/tilt acceptance; a
both-rate regression checks legacy ToF acceptance at >60-degree tilt separately from supervisor trust.

Thresholds are engineering assumptions, not certified limits. Input age is receipt age, not proof of
acquisition time, covariance or accuracy: plausible frozen/replayed data is not detected. Touchdown is
a height/vertical-speed dwell heuristic, not a contact sensor. No bounded XY drift is claimed after
camera loss, especially while moving. All-height-loss fault containment can cause a fall and must
never be counted as a successful controlled landing. Hardware sensor offsets, occlusion, ground
geometry and touchdown remain unvalidated; FLOW-01 is unchanged.

### Local flow emergency braking

`SitlDrone(..., safe_landing=True, local_flow=True)` fuses independent modeled local flow with
the EKF, then requests zero XY velocity after camera loss. In the portable runtime the existing
safe-mode switch is sufficient: flow must be successfully queued, finite, plausible, with positive
dt/std, recent (100 ms) and supported by fresh usable ToF at least 12 cm above ground. Missing or
rejected flow/range returns to degraded level descent, not unaided velocity hold. These checks
are not a substitute for a hardware driver's surface-quality/motion-valid flags or calibration.

`make flow-braking` compares baseline/candidate and stress cases. Nominal camera-loss excursion
falls from 1.71-1.72 m to **0.277-0.290 m**; the sampled stress envelope reaches **0.349 m** (three
seeds, +/-20% flow scale, doubled noise, 20 ms delay and 90-degree heading, both loop rates).
The predeclared gates are 0.35 m nominal / 0.50 m sampled stress, not the 5 cm positioning gate.
`fcSafety.flowFresh/flowBrake` expose qualification/active braking; the replay is `?scenario=flow-brake`.

This does not fix FLOW-01, all-height loss, arbitrary low-texture/lighting/wind failures or hardware
flight. Research and the barometer/contact-sensing follow-up are in
`reports/Resilient landing without camera.md`. Free research agents were quota-blocked; no independent
agent-review claim is made for this stage.

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
| `modules/src/stabilizer.c`, `modules/src/estimator.c`, `hal/src/sensors_mpu6050_hm5883L_ms5611.c`, `CMakeLists.txt`, `main/Kconfig.projbuild` | No way to run `fc_step` on target | `CONFIG_ESPFC_FC_STEP` (default off): stabilizer calls `fcRuntimeStep`; position/pose/ToF/flow enqueues go to `fc_runtime` |

## Findings logged for gate G3

Coverage (`make core-coverage`) counts code reachable from `fc_step`: the INDI and Mellinger controllers are
linked only for `controller.c`'s table (fc_step always selects PID) and are excluded; `planner.c`/`pptraj*.c`
(high-level commander only) are no longer built on the host. 2026-10-27: 79.7% lines, 67.4% branches;
the G3 target is > 80%.

| Location | Finding | Impact |
| --- | --- | --- |
| `modules/src/sensfusion6.c` (`invSqrt`) | S2 retains the approximation because it has no FPU | Benchmark separately before promising accuracy/CPU budget on that chip |
| `modules/src/controller_pid.c` | Zero-thrust PID reset only acts on ticks where the 500 Hz attitude loop runs | None in continuous flight; tests must reset on an even tick |
| `modules/interface/math3d.h:67` | `fabsf()` applied to an `int` subtraction in the ULP float compare; the subtraction can overflow (undefined behavior) for far-apart values | Low; fix when touched |
| `lib/dsp_lib/include/xtensa_math.h:2170,2183` | Pointer cast to `int32_t` in circular-buffer FIR helpers | Breaks on 64-bit hosts only; not used by fc_core; fine on 32-bit ESP32 |
