# ESP-FC Next

Dual-chip **ESP32-P4 + ESP32-C5** flight controller and firmware for indoor autonomous swarm research.
Derived from [Espressif ESP-Drone](https://github.com/espressif/esp-drone) (Crazyflie-based), GPL-3.0.

The full plan, gap analysis and gates are in [`docs/ESP-FC-Next_Project_Plan.pdf`](docs/ESP-FC-Next_Project_Plan.pdf).

## Layout

| Path | Contents |
| --- | --- |
| `firmware/flight/` | ESP32-P4 flight firmware (ESP-Drone fork, imported with `git subtree`) |
| `firmware/fc_core/` | Host (PC) build of the portable flight algorithms + HAL shims and tests ([docs](docs/fc_core.md)) |
| `firmware/radio/` | ESP32-C5 radio co-processor firmware (SPI link, ESP-NOW swarm, Wi-Fi) |
| `sim/` | Math models, software-in-the-loop bridge and swarm simulation (Python) |
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
| `make pdf` | Regenerate the project plan PDF |

## Pulling upstream ESP-Drone changes

```sh
git subtree pull --prefix firmware/flight https://github.com/espressif/esp-drone.git master --squash
```

## Safety

Never fly hardware that has not passed the verification gates described in the plan. Always fly
with props guarded, in a netted area, with an independent kill switch.
