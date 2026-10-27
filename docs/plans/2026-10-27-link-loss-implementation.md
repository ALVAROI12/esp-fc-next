# Latched Hold-and-Land Implementation Plan

**Goal:** Demonstrate opt-in command/tracking-loss landing, latched through touchdown, in the real C runtime and SITL.

**Architecture:** Add a fixed-size safety state machine around the existing commander watchdog. Fresh camera
position permits XY hold; independent downward ToF supports level descent after camera loss. Terminal
LANDED and FAULT states block automatic rearm; production ESP-Drone tasks are unchanged.

**Tech Stack:** Portable C, Unity/ASan/UBSan, Python/NumPy ctypes SITL, matplotlib and the existing replay viewer.

---

### 1. Portable supervisor and regressions

Files: `firmware/fc_core/include/fc_step.h`, `src/fc_step.c`,
`tests/test_failsafe.c`, `tests/CMakeLists.txt`.

1. Write failing C tests for the approved transitions and thresholds in the design; build both rates.
2. Run `make core-test` and record failures before implementation.
3. Implement disarmed-only `fc_set_failsafe_enabled(bool)` and `fc_failsafe_state()`.
   Enum values: NORMAL=0, HOLD=1, LAND=2, LANDED=3, FAULT=4.
4. Validate queued position/ToF inputs and track receipt ages; enable arming checks only in safe mode.
5. Test hold pose capture/zero feedforward, LAND latch, rejected invalid packets, terminal motors off,
   no automatic rearm, emergency/tumble precedence and legacy behavior with safe mode disabled.
6. Re-run `make core-test`; all suites must pass, including both variants.

### 2. SITL inputs and loss scenarios

Files: `sim/sitl/firmware.py`, `sim/sitl/drone.py`, `sim/tests/test_failsafe_sitl.py`.

1. Bind the new API; add explicit `safe_landing=False` construction option.
2. Add separate `tracking_up` and `height_up` switches; keep `link_up` as the command path.
3. With safe mode enabled, add independent ToF using a separate RNG so existing sensor streams do not change.
4. Record safety state/reason and active safety targets, rather than displaying stale mission targets as flown commands.
5. Test both rates, hover and figure-8 loss, command recovery during HOLD/LAND, combined loss and FAULT.
   Assert actual descent/touchdown/motor state and no re-takeoff after command recovery.
6. Run `make sim-test`; retain FLOW-01 strict expected failure.

### 3. Reproducible evidence and documentation

Files: `sim/failsafe_report.py`, `sim/visualize.py`, `Makefile`,
`sim/reports/failsafe/`, `docs/fc_core.md`, `sim/README.md`, project-plan PDF source/output.

1. Export loss times, states, descent speed, touchdown/disarm height and horizontal drift.
2. Mark all-height-loss as FAULT, never successful touchdown.
3. Add 3D replay with safety state/height/drift charts and a `make failsafe` report target.
4. Independent read-only review; fix must-fix issues with regressions.
5. Run `make core-test`, `make sim-test`, `make failsafe`, `make core-coverage`, `make flight`.
6. Update and visually inspect PDF/replay. Run `git diff --check`, commit/push only verified changes,
   watch CI, and report measured results plus remaining hardware and localization limits.
