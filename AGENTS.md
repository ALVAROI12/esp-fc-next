# ESP-FC Next - agent guide

Dual-chip ESP32-P4 (flight) + ESP32-C5 (radio) flight controller. Plan: `docs/ESP-FC-Next_Project_Plan.pdf`
(source: `docs/build_plan_pdf.py`; regenerate with `make pdf` after editing).

## Rules
- Simulation first: no hardware purchase or flight until gates G1-G6 pass, in order.
- ESP-IDF is pinned to `release-v5.5` and runs only in Docker via `tools/idf.sh` (Colima on macOS).
- Build each target into `build-<target>/` with `-D SDKCONFIG=build-<target>/sdkconfig`; never commit `sdkconfig`.
- `firmware/flight/` is a `git subtree` of espressif/esp-drone. Keep changes reviewable so upstream pulls stay possible:
  minimal patches, marked `ESP-FC Next:` in a comment, listed in `docs/fc_core.md`.
- Every bug fix in flight code needs a unit test that failed before the fix.
- Flight-loop code (future `fc_core`): no FreeRTOS calls, no dynamic allocation, must build on host and target.
- Keep the Crazyflie CRTP protocol compatible (cflib / Crazyswarm2).
- Review flight-code changes against `docs/review_checklist.md` (G3).
- License: GPL-3.0; keep upstream copyright headers.

## Commands
- `make flight` / `make radio` - firmware builds; `make flight-fcstep FLIGHT_TARGET=esp32s3` - image with fc_step in the stabilizer task (CONFIG_ESPFC_FC_STEP); `make flight-c5link` - P4 image with CRTP over the C5 SPI link
- `make core-test` - host build of fc_core with ASan/UBSan + Unity tests (sources used in place from the subtree; see `docs/fc_core.md`); first run copies mbedtls out of the IDF image (`make mbedtls-src`, needs Docker)
- `make core-lint` / `make core-tidy` - cppcheck / clang-tidy of fc_core (G3); `make link-fuzz [FUZZ_TARGET=fuzz_link_aead]` - libFuzzer on the link codec or AES-CCM envelope (docs/protocols.md)
- `make core-coverage` - coverage summary + HTML in `build/coverage/`
- `make sim-setup` / `make sim-test` - Python sim environment (uv, Python 3.12) and tests
- `make swarm` - four-drone SITL missions with injected faults (Task 1.8)
- `make hw-setup` / `make hw` - Phase 3 boards as code (`hardware/boards.py`, SKiDL; needs KiCad + ngspice): netlists, drawn KiCad sheets and G4 netlist/footprint/ERC/power checks, see `hardware/DESIGN.md`
- `make hw-pcb` - PCB placement + routing of both boards (KiCad Python; the clean copper is pinned in `hardware/routes/*.json`, re-routed with Freerouting only if it stops passing DRC: Java 25+, jar per `hardware/DESIGN.md`, optionally on a pod with `ROUTE_REMOTE`), 3D renders and the power-path plot in `hardware/out/`; `make hw-gerbers` - JLCPCB Gerber/drill zips in `hardware/out/gerbers/` and assembly BOM/CPL CSVs in `hardware/out/assembly/`
- `make g1` - gate G1 report; re-run after changing gains, loop timing, or `sim/model/params.py`
- `make viewer` - 3D replay viewer of the G1 and SITL scenarios (generated, git-ignored)
- SITL (`sim/sitl`) rebuilds `build/fc_sitl` automatically from `firmware/fc_core` when sources change
