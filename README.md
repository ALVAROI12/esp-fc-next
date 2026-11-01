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

## Hardware status (Phase 3, 2026-11-01)

Two 28 mm square boards as code (`hardware/boards.py`, SKiDL): the P4 flight controller and the C5-MINI-1
radio board, stacked on a 1.27 mm 1x12 header, 1S only. `make hw` passes 22 checks (ERC, drawn-vs-code
netlist, pin-to-pad, firmware pin defaults, ngspice power path, placement fit, routed copper DRC). Both boards
place and autoroute with Freerouting (`make hw-pcb`) and pass KiCad DRC against **JLCPCB's 4-layer 1 oz
rules** (0.09 mm track/space, 0.35/0.15 mm vias, 0.2 mm copper-to-edge, 0.1 mm mask dam, through-hole
0.28 mm hole-to-copper). Details and sources: [`hardware/DESIGN.md`](hardware/DESIGN.md).

**Fab: JLCPCB, 4 layers, 1 oz, 1.6 mm.** `make hw-gerbers` writes the upload set per board:

| File | Contents |
| --- | --- |
| `hardware/out/gerbers/<board>.zip` | Gerber X1 (Protel extensions), Excellon PTH/NPTH drills, drill maps |
| `hardware/out/assembly/<board>-bom.csv` | JLCPCB BOM: Comment, Designator, Footprint, LCSC Part # |
| `hardware/out/assembly/<board>-cpl.csv` | JLCPCB pick-and-place: Designator, Mid X, Mid Y, Layer, Rotation |

FC board: 75 placed parts, 37 BOM lines. Radio board: 8 parts, 7 BOM lines. Mounting holes and test
points are excluded. `hardware/out/` is git-ignored; regenerate with `make hw-pcb && make hw-gerbers`.

LCSC numbers (table in `hardware/assembly.py`, looked up 2026-11-01): ICM-42688-P C1850418, BMP581
C5362283, TPS63802 C2845237, TLV62569 C141836, INA226 C49851, W25Q128JVP C190862, DMP1022UFDF-7 C155358,
DFE201610E-2R2M C337891, USB4105-GF-A C3020560, JST BM08B-SRSS-TB C160394, BM04B-SRSS-TB C160390, B5819W
C64885, PTS810 switch C116501, 40 MHz 3225 crystal C426988. Passives are blank so JLCPCB matches them by
value and footprint.

Not ordered yet. Decisions left for the order form:

- **ESP32-P4NRW16X**: mapped to JLCPCB's "ESP32-P4NRW16" listing C9900140808 (a JLCPCB Assembly item).
  Confirm it is the X (QFN104 10x10 mm, 16 MB PSRAM) variant before paying.
- **ESP32-C5-MINI-1**: no LCSC listing. Consign it or hand-solder.
- **0.47 uH XFL4020-471ME (L1)**: Coilcraft dropped the XFL4020 series; the drop-in XGL4020-471MEC is not on
  LCSC either. Consign it or pick a stocked 0.47 uH 4x4 mm inductor.
- **1.27 mm stack header**: no LCSC match. Hand-solder.
- Surface finish: the P4's 0.35 mm pitch LGA and the 0.2 to 0.25 mm pads need ENIG per JLCPCB.
- Check D1, C2 and the ICs' pin 1 rotations in JLCPCB's placement preview.
- JLCPCB's DFM check on upload is the final word on the footprints.

Known DRC leftovers, not fab-related: the FC stack header courtyard overlaps the H1/H2 hole courtyards; the
radio board's H1/H3 holes sit inside the C5 antenna keep-out. The autorouted copper has not been reviewed by
a human (return paths, 40 MHz crystal, USB D+/D-, battery path width). No 3D models for the P4 or C5; the
ESPFC footprints have not been checked against a physical part. ICM-42688-P Rev 1.8 datasheet still
unreadable (TDK returns 403). Phase 2 kit (user's AliExpress cart) not bought.

## Layout

| Path | Contents |
| --- | --- |
| `firmware/flight/` | ESP32-P4 flight firmware (ESP-Drone fork, imported with `git subtree`) |
| `firmware/fc_core/` | Host (PC) build of the portable flight algorithms + HAL shims and tests ([docs](docs/fc_core.md)) |
| `firmware/radio/` | ESP32-C5 radio co-processor firmware (SPI link, ESP-NOW swarm, Wi-Fi) |
| `sim/` | G1 math model, G2 software-in-the-loop (real firmware flying the 6-DOF model), 3D viewer ([docs](sim/README.md)) |
| `hardware/` | Flight-controller and radio boards as SKiDL/KiCad code, JLCPCB rules and outputs (Phase 3, [docs](hardware/DESIGN.md)) |
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
| `make hw-setup` / `make hw` | Hardware venv; netlists, drawn KiCad sheets and the G4 checks (needs KiCad 10 + ngspice) |
| `make hw-pcb` | Placement, Freerouting autoroute, DRC gate, 3D renders, power-path plot (Java 25+, jar per `hardware/DESIGN.md`) |
| `make hw-gerbers` | JLCPCB Gerber/drill zips plus assembly BOM and pick-and-place CSVs |
| `make pdf` | Regenerate the project plan PDF |

## Pulling upstream ESP-Drone changes

```sh
git subtree pull --prefix firmware/flight https://github.com/espressif/esp-drone.git master --squash
```

## Safety

Never fly hardware that has not passed the verification gates described in the plan. Always fly
with props guarded, in a netted area, with an independent kill switch.
