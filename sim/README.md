# sim

Python environment for gate G1 (math model) and G2 (software-in-the-loop), Python 3.12 via uv.

```sh
make sim-setup   # create sim/.venv and install requirements
make sim-test    # model consistency tests
make g1          # gate G1 report -> sim/reports/g1/README.md
make vibration   # Task 1.6b vibration/notch report -> sim/reports/vibration/README.md
make viewer      # interactive G1 studies + C SITL figure-8, square, flow-drift and vibration replays
```

The viewer (`visualize.py` + `viewer/viewer.html`, three.js) replays the nonlinear closed-loop simulation:
nominal airframe, worst-case airframe, and a Monte Carlo fleet, each flying a roll doublet and a gust, with
synced roll / roll-rate / motor charts, slow motion and scrubbing. `?scenario=worst&t=2.3` in the URL opens a
scenario paused at a time.

## Math model (`model/`, Task 1.4)

| Module | Contents |
| --- | --- |
| `params.py` | Airframe, motor, sensor and loop-timing parameters, each tagged with its source (`ASSUMED` values must be measured) |
| `firmware.py` | Reads PID gains directly from the firmware's `pid.h` |
| `dynamics.py` | Nonlinear 6-DOF quad-X model: quaternions, rotor lag, quadratic thrust, drag torque |
| `sensors.py` | ICM-42688-P, BMP581, VL53L1X and mocap noise models from datasheet values |
| `linear.py` | Discrete loop models matching `pid.c` / `controller_pid.c` / mixer; margins, step metrics, noise |
| `robust.py` | +/-20 % uncertainty corners, Monte Carlo, robust gain search |
| `closed_loop.py` | Python port of the firmware cascaded PID flying the 6-DOF model (nonlinear cross-check) |
| `vibration.py` | ASSUMED synthetic motor vibration (additive gyro artifact) and the firmware notch response used for margin analysis |

Current result: [`reports/g1/README.md`](reports/g1/README.md). Gains for the firmware board profile:
[`reports/g1/proposed_gains.json`](reports/g1/proposed_gains.json).

## Software-in-the-loop (`sitl/`, Task 1.5, gate G2)

The real C flight firmware flies the 6-DOF model. `firmware/fc_core` is built as a shared library
(`build/fc_sitl`, rebuilt automatically when sources change) and called through ctypes in lockstep.
Profiles select stock 1 kHz or planned 4 kHz host variants. This is **not** hardware runtime validation:

```python
from sitl import SitlDrone
d = SitlDrone(seed=1, profile="whoop2_4k") # physics + IMU noise + 100 Hz simulated mocap
d.calibrate_and_arm()              # firmware gyro-bias / accel-scale calibration on the ground
d.takeoff(1.0, duration=2.0)       # minimum-jerk setpoints at 100 Hz, like Crazyswarm
d.run(5.0)
d.fw.log("posCtl.Zi")              # any firmware LOG variable
```

| Module | Contents |
| --- | --- |
| `firmware.py` | ctypes binding; one library copy per instance so swarms have independent firmware globals |
| `drone.py` | `SitlDrone`: lockstep loop, mocap latency/noise, setpoint segments, link-loss switch, logging |
| `env.py` | `GotoEnv`: Gym-style (reset/step) environment for RL; actions are bounded position setpoints |

The physics is the validated G1 model (not pybullet): one model for G1 and G2, deterministic, faster than
real time. pybullet stays available for obstacles and camera rendering later.

### SITL findings (fed into Task 1.6)

| Finding | Evidence | Action |
| --- | --- | --- |
| Position-loop integrator `posCtlPid.zKi` = 0.5 winds up during takeoff | 11.4 cm overshoot, multi-second tail (`posCtl.Zi`); 0.2 cm with zKi = 0 | Board profile: zKi = 0 |
| 2021 position controller has no velocity feedforward | 0.3-0.4 s lag behind moving setpoints, about 3 cm overshoot | Port velocity feedforward from newer Crazyflie |
| Stock commander watchdog cuts motors 2 s after link loss | Drone falls from 1 m in SITL | Hold-and-land failsafe (Task 1.8) |
| `thrustBase` compiled as 24000; whoop hovers at about 39750 | Set per airframe via parameter | Board profile value |

## Task 1.6a: flight-core simulation

Both timing variants now carry G1-generated gains through C board profiles. The 4 kHz variant
predicts its EKF at 500 Hz. Setpoints include position, velocity and acceleration; optional
feedforward anticipates tilt and vertical thrust instead of waiting for position error.

The camera-tracked nominal figure-8 (0.6 m x 0.3 m amplitudes, 8 s period, two laps, 1 m height)
tracks within about **4-5 cm maximum 3D error** after a declared 2 s entry window; the reference
settings miss the 10 cm gate by a wide margin. This uses the C algorithms and synchronous loop,
not a Python controller. The viewer shows the full entry transient too; it is not hidden.

## Task 1.6b: motor vibration and RPM notch

`model/vibration.py` turns the four actual (motor-lagged) rotor speeds into a deterministic gyro
artifact: 1x mechanical rotation plus a second harmonic, amplitude scaling with rotor squared, and
slightly different per-motor speeds. It is **additive sensor noise only**, not body torque or frame
resonance, and every amplitude is assumed until blackbox data exists. Its phases use a separate
seeded RNG, so filter-on/off comparisons keep the same disturbance.

The firmware side is a four-motor notch bank (`firmware/fc_core/src/fc_rpm_notch.c`): two RBJ notches
per motor (1x, 2x), Q = 5, retuned at 100 Hz from mechanical RPM telemetry, bypassed after 100 ms
without a valid packet. It is disabled by default, cannot be reconfigured while armed, and never
filters a target below 500 Hz or at/above 45 % of the sample rate. At 1 kHz that leaves no active
band, so the bank bypasses. See `docs/fc_core.md`.

`make vibration` reproduces the comparison (fixed seeds, 4 kHz SITL, 150 Hz gyro LPF):

| Assumed amplitude | Gyro artifact, notch off -> on | Motor command >250 Hz, off -> on | Hover max error |
| --- | --- | --- | --- |
| 12 deg/s per motor | 3.8 -> 0.20 deg/s RMS | 3.1 -> 1.9 % (baseline 1.9 %) | 0.23-0.24 cm |
| 36 deg/s per motor | 11.5 -> 0.50 deg/s RMS | 7.4 -> 1.9 % | 0.23-0.24 cm |

Figure-8 tracking is 4.1 cm maximum in both cases, i.e. **unchanged**. The 150 Hz gyro LPF already
attenuates this frequency band before the controller, so the demonstrated benefit is lower high-frequency
sensor and actuator activity, not better position accuracy. Constant positive common-mode RPM scale
bias of 2/10/20 % leaves
0.20/1.32/2.63 deg/s of residual; a 100 ms dropout bypasses the bank and recovers. The bank adds about
0.9 deg of phase at 10 Hz (1.8 deg at 20 Hz) at modeled hover RPM; the worst of 32 corners at hover
keeps 47.5 deg phase / 7.2 dB gain margin. The original 150 Hz floor failed off-hover margins;
the 500 Hz floor is now checked with a sampled RPM/corner sweep (worst 46.2 deg / 6.9 dB). The gate includes
harmonic-only operation and section activation/cap boundaries, at Q=5. It is not a guarantee for
continuous/arbitrary independent motor speeds, other Q values, or hardware.

The dropout tests now assert actual gyro-path equality during bypass, and record position through
dropout and recovery. The viewer's vibration trace is recorded at 4 kHz rather than downsampled to
250 Hz; use the spectrum report to inspect frequency content (the full replay trace is dense).

Real frames can have structural resonances below the modeled motor lines, and real vibration must be
measured. This work does not validate the P4 runtime, RPM sensing, or hardware flight.

## Open accuracy issue: FLOW-01

Flow + ToF can hold height but do not anchor absolute horizontal position. In a deterministic
20 s hover (interim profile, seed 5), changing only modeled pixel noise produces:

| Flow noise (std, pixels) | End XY drift | Maximum XY drift |
| --- | --- | --- |
| 0 | 0.6 cm | 0.7 cm |
| 0.5 | 17.2 cm | 22.3 cm |
| 2.0 | 68.9 cm | 88.8 cm |

Height std stayed about 0.14 cm in these idealized runs. The **5 cm XY requirement remains unmet**,
recorded as a strict expected-failure test (`FLOW-01`), not relaxed to make CI green. An unexpected
pass also fails CI so the requirement must be reviewed explicitly when improved.

Flow pixels are generated from the legacy EKF's own guessed calibration/rotation model. These tests
check internal consistency, not independent sensor calibration or real lighting/texture performance.
Actual flow scale, noise, motion blur, time alignment and drift require hardware measurements.
Camera tracking (or another shared position reference) remains required for precise swarm flight.

## Safety/review boundaries

- Regression tests cover immediate tilted/inverted arm refusal, sustained tumble, dynamics reset,
  profile rate mismatch, disarmed integration and watchdog removal of stale trajectory feedforward.
- `fc_init()` resets dynamics, not tuned parameters. `stock` requires a fresh library instance after
  applying a whoop profile; applying profiles while armed is rejected.
- `GotoEnv` action duration uses the firmware's actual rate; training remains on the workstation.
- The P4 image has **not** switched to the new synchronous runtime. Deadline measurements, IMU/ESC
  integration, redundant protection, sensor fault handling and a safe link-loss landing remain work.
- The RPM notch is validated against a synthetic artifact; measured vibration, RPM calibration and
  P4 timing remain open. It does not fix `FLOW-01`.
