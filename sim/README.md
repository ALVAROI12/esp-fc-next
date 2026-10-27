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

Planned: `sitl/` (fc_core <-> pybullet lockstep bridge - task 1.5), `swarm/` (4-drone Crazyswarm2 scenarios - task 1.8).
