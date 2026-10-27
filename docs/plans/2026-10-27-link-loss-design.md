# Latched hold-and-land design

Approved policy: stop following stale trajectories; hold briefly if tracking is healthy;
land on tracking loss; never resume a mission once landing starts, even after link recovery.
This first implementation is opt-in in the portable runtime/SITL, not activated on the P4.

## State machine

- NORMAL: ordinary commands. After 500 ms without a command, snapshot estimated position/yaw
  and enter HOLD with zero trajectory velocity/acceleration. A recovered command can end HOLD.
- HOLD: retain that fixed pose for 1.5 s, then LAND. Tracking older than 300 ms skips HOLD.
- LAND: latched; ignore renewed mission commands. Descend at 0.25 m/s, slowing to 0.10 m/s
  below 15 cm. Hold XY only while the camera reference remains fresh. Once lost, keep level
  roll/pitch for the rest of this landing, without claiming bounded XY drift.
- LANDED: disarm after height <= 4 cm and |estimated vertical speed| <= 0.15 m/s for 500 ms.
  Prefer independent downward ToF; camera height relative to the arming ground is a fallback.
  Require explicit ground reinitialization before another flight; never auto-rearm.
- FAULT: no fresh camera height or usable ToF (200 ms freshness), invalid estimator state,
  or landing exceeds 20 s. Stop motors, latch and record the fault. This is failure containment,
  NOT a successful controlled landing. Emergency stop and tumble always override the state machine.

## Inputs and boundaries

Safe mode requires fresh camera position and local ToF before arming. Configure only disarmed;
enable persists across runtime init like other tuning; state/history clears on init. Remain disabled
in a fresh process, preserving the legacy watchdog for reference tests. No new CRTP packet formats.
Only finite, plausible, successfully queued measurements refresh input age. ToF requires adequate
tilt; measure freshness using wrap-safe tick subtraction. Receipt freshness is not an acquisition
timestamp or proof of camera/estimator accuracy; sensor freezes/replayed finite packets remain a limit.

No reliable height means controlled touchdown cannot be guaranteed. The FAULT branch must be
reported separately and must never be counted as a landing pass. Local flow is not required, and
FLOW-01 remains unmet. ToF offset, ground plane, latency and physical touchdown need hardware tests.

## Evidence

Run C tests at 1 and 4 kHz and SITL hover/moving trajectories with command-only, tracking-only and
combined loss, brief recovery, restored commands during LAND, and missing height. Check actual motor
outputs, bounded descent, measured touchdown speed, latched transitions and post-touchdown zero output.
Export state/reason/height/position charts and 3D replay. Keep production hardware claims unchanged.
