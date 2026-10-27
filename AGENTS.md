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
- License: GPL-3.0; keep upstream copyright headers.

## Commands
- `make flight` / `make radio` - firmware builds
- `make core-test` - host build of fc_core with ASan/UBSan + Unity tests (sources used in place from the subtree; see `docs/fc_core.md`)
- `make core-coverage` - coverage summary + HTML in `build/coverage/`
- `make sim-setup` / `make sim-test` - Python sim environment (uv, Python 3.12) and tests
- `make g1` - gate G1 report; re-run after changing gains, loop timing, or `sim/model/params.py`
