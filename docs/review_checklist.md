# Review checklist (gate G3)

Apply to every change in flight code (`firmware/fc_core`, subtree patches) before merge, whether a person or an
AI agent reviews it. Each item comes from a bug this project actually found; the reference says where.

## Numbers
- [ ] Every float that reaches the estimator, controller or mixer is finite. Check sensor inputs, measurements,
      divisions and `sqrtf`/`acosf` arguments. (Kalman 0/0 NaN at zero gyro; one NaN IMU sample poisoned the EKF.)
- [ ] Tick and time arithmetic uses unsigned differences (`(uint32_t)(now - then)`) so wraparound is safe.
- [ ] Integer division in a float context is intended (clang-tidy `bugprone-integer-division`).

## Real-time rules (AGENTS.md)
- [ ] No FreeRTOS calls, dynamic allocation or blocking I/O in fc_core flight-loop code. Target glue
      (`firmware/fc_core/target`) may use static queues, without blocking.
- [ ] State shared with other tasks crosses through a queue, never a plain struct written from two tasks.

## Safety logic
- [ ] Every new failsafe path ends in LANDED or a documented FAULT; nothing turns a stale ground command into a
      takeoff.
- [ ] Freshness uses receipt age and qualified values only; rejected packets never refresh it.
- [ ] Controller integrators are reset when an external limit (thrust band, landed) held them against the loop.
      (Blind-descent windup threw the drone up on release.)
- [ ] Behaviour is checked against an estimate that is wrong. Inject sensor bias or scale error in SITL; do not
      only test with truth-quality inputs. (A -0.02 g accel bias made blind descent climb to 4 m.)

## Tests and evidence
- [ ] A bug fix has a unit test that failed before the fix. Note in the commit how you saw it fail.
- [ ] A new test is mutation-checked: break the logic it guards and confirm it fails.
- [ ] SITL code feeds the firmware only what the vehicle or ground station can measure; no truth leaks.
- [ ] `make core-test core-lint core-tidy sim-test swarm` pass. Firmware changes also build with `make flight`,
      `make flight-fcstep FLIGHT_TARGET=esp32s3` and `make radio`.

## Subtree hygiene
- [ ] Subtree edits are minimal, marked `ESP-FC Next:` and listed in `docs/fc_core.md`. Default builds are
      unchanged unless the change is the point.
- [ ] `sdkconfig` is not committed.
