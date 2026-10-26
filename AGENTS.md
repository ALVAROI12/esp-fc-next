# ESP-FC Next - agent guide

Dual-chip ESP32-P4 (flight) + ESP32-C5 (radio) flight controller. Plan: `docs/ESP-FC-Next_Project_Plan.pdf`
(source: `docs/build_plan_pdf.py`; regenerate with `make pdf` after editing).

## Rules
- Simulation first: no hardware purchase or flight until gates G1-G6 pass, in order.
- ESP-IDF is pinned to `release-v5.5` and runs only in Docker via `tools/idf.sh` (Colima on macOS).
- Build each target into `build-<target>/` with `-D SDKCONFIG=build-<target>/sdkconfig`; never commit `sdkconfig`.
- `firmware/flight/` is a `git subtree` of espressif/esp-drone. Keep changes reviewable so upstream pulls stay possible.
- Flight-loop code (future `fc_core`): no FreeRTOS calls, no dynamic allocation, must build on host and target.
- Keep the Crazyflie CRTP protocol compatible (cflib / Crazyswarm2).
- License: GPL-3.0; keep upstream copyright headers.

## Commands
- `make flight` / `make radio` - firmware builds
- `make sim-setup` / `make sim-test` - Python sim environment (uv, Python 3.12) and tests
