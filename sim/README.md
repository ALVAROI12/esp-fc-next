# sim

Python environment for gate G1 (math model) and G2 (software-in-the-loop), Python 3.12 via uv.

```sh
make sim-setup   # create sim/.venv and install requirements
make sim-test    # run tests
```

Planned contents: `model/` (6-DOF dynamics, sensor noise, stability analysis - task 1.4),
`sitl/` (fc_core <-> pybullet lockstep bridge - task 1.5), `swarm/` (4-drone Crazyswarm2 scenarios - task 1.8).
