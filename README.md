# ESP-FC Next

Dual-chip **ESP32-P4 + ESP32-C5** flight controller and firmware for indoor autonomous swarm research.
Derived from [Espressif ESP-Drone](https://github.com/espressif/esp-drone) (Crazyflie-based), GPL-3.0.

The full plan, gap analysis and gates are in [`docs/ESP-FC-Next_Project_Plan.pdf`](docs/ESP-FC-Next_Project_Plan.pdf).

**Current status:** 1/4 kHz host/SITL variants, trajectory feedforward, and an optional RPM notch
bank (Task 1.6b, synthetic vibration only) are tested in simulation. The production P4 image still
uses ESP-Drone's original task loop; the new runtime has not been activated or timing-validated on
hardware. Flow-only absolute XY positioning remains an open failure.

An **opt-in latched hold-and-land supervisor** now has command/camera-loss SITL tests and a
[report](sim/reports/failsafe/README.md). Renewed commands cannot cancel LAND. Moving camera loss
still allows substantial XY drift. Camera loss above the 1.3 m ToF range triggers a bounded blind descent
in a thrust band below the learned hover thrust; permanent loss of every height source still ends in a
fault/motor stop, not guaranteed safe landing. It is not collision-safe or hardware-validated.

With qualified local flow/range, emergency velocity braking reduces nominal camera-loss excursion
from 1.71-1.72 m to **0.28-0.29 m** in SITL ([comparison](sim/reports/flow_braking/README.md)).
Total-height-loss recovery remains open; this does not close the absolute-positioning requirement.

`make swarm` flies four firmware copies in lockstep (layers, formation, diagonal swap) with link-loss,
camera-dropout and low-battery faults; all land safely in SITL ([report](sim/reports/swarm/README.md)).
No downwash model; low battery is a voltage step, not a discharge model.

## Layout

| Path | Contents |
| --- | --- |
| `firmware/flight/` | ESP32-P4 flight firmware (ESP-Drone fork, imported with `git subtree`) |
| `firmware/fc_core/` | Host (PC) build of the portable flight algorithms + HAL shims and tests ([docs](docs/fc_core.md)) |
| `firmware/radio/` | ESP32-C5 radio co-processor firmware (SPI link, ESP-NOW swarm, Wi-Fi) |
| `sim/` | G1 math model, G2 software-in-the-loop (real firmware flying the 6-DOF model), 3D viewer ([docs](sim/README.md)) |
| `hardware/` | KiCad flight-controller design (Phase 3) |
| `tools/` | Developer scripts, swarm dongle, ground tools |
| `docs/` | Project plan and protocol specs |

## Prerequisites (macOS)

```sh
brew install colima docker docker-buildx uv cmake ninja llvm cppcheck lcov
colima start --cpu 6 --memory 8 --disk 60 --vm-type vz
docker pull espressif/idf:release-v5.5
make sim-setup
```

## Common commands

| Command | What it does |
| --- | --- |
| `make flight` | Build P4 flight firmware (`FLIGHT_TARGET=esp32s3` for the old reference board) |
| `make radio` | Build C5 radio firmware |
| `make core-test` | Build flight algorithms on the PC with ASan/UBSan and run tests |
| `make core-coverage` | Test coverage report (`build/coverage/index.html`) |
| `tools/idf.sh firmware/flight -B build-esp32p4 menuconfig` | Any `idf.py` command inside the pinned container |
| `make sim-test` | Run simulation and model tests |
| `make g1` | Gate G1 math-model report ([`sim/reports/g1`](sim/reports/g1/README.md)) |
| `make vibration` | Task 1.6b vibration/notch report ([`sim/reports/vibration`](sim/reports/vibration/README.md)) |
| `make failsafe` | Opt-in latched landing/fault evidence ([`sim/reports/failsafe`](sim/reports/failsafe/README.md)) |
| `make flow-braking` | Camera-loss braking comparison and sampled stresses |
| `make link-fuzz` | libFuzzer run of the P4<->C5 link codec ([protocols](docs/protocols.md)); `FUZZ_SECONDS=60` for a smoke run |
| `make viewer` | Interactive 3D simulation replay with charts (opens in the browser) |
| `make pdf` | Regenerate the project plan PDF |

## Pulling upstream ESP-Drone changes

```sh
git subtree pull --prefix firmware/flight https://github.com/espressif/esp-drone.git master --squash
```

## Safety

Never fly hardware that has not passed the verification gates described in the plan. Always fly
with props guarded, in a netted area, with an independent kill switch.
