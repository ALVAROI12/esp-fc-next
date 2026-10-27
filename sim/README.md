# sim

Python environment for gate G1 (math model) and G2 (software-in-the-loop), Python 3.12 via uv.

```sh
make sim-setup   # create sim/.venv and install requirements
make sim-test    # model consistency tests
make g1          # gate G1 report -> sim/reports/g1/README.md
make viewer      # interactive 3D replay of the G1 scenarios in the browser
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

Current result: [`reports/g1/README.md`](reports/g1/README.md). Gains for the firmware board profile:
[`reports/g1/proposed_gains.json`](reports/g1/proposed_gains.json).

## Software-in-the-loop (`sitl/`, Task 1.5, gate G2)

The real C flight firmware flies the 6-DOF model. `firmware/fc_core` is built as a shared library
(`build/fc_sitl`, rebuilt automatically when sources change) and called through ctypes once per 1 ms tick:

```python
from sitl import SitlDrone
d = SitlDrone(seed=1)              # physics + ICM-42688-P noise + 100 Hz mocap + firmware
d.fw.set_param("posCtlPid.zKi", 0) # any firmware PARAM by its cfclient name
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
