# Four-drone swarm missions (Task 1.8, SITL)

**Checks: PASS**

4 drones, profile `whoop2_interim`, opt-in safe mode with local flow/ToF, one real C firmware copy each. Mission: takeoff to layers [1.0, 1.35, 1.7, 2.05] m, 1.2 m square formation, diagonal swap, landing. Collision = truth 3D separation < 0.3 m. Safe landing = disarmed, motors off, sampled touchdown <= 0.35 m/s, no FAULT/tumble/assert; faulted drones must end LANDED.

| Scenario | Min separation | Landings (drone 0-3, touchdown speed) | Result |
| --- | --- | --- | --- |
| nominal mission | 0.35 m (drones 2-3) | ok (0.03 m/s), ok (0.11 m/s), ok (0.01 m/s), ok (0.04 m/s) | PASS |
| link loss in formation (drone 0) | 0.35 m (drones 2-3) | ok (0.15 m/s), ok (0.02 m/s), ok (0.00 m/s), ok (0.04 m/s) | PASS |
| mocap dropout in formation (drone 1) | 0.35 m (drones 2-3) | ok (0.01 m/s), ok (0.14 m/s), ok (0.00 m/s), ok (0.04 m/s) | PASS |
| link loss mid-swap, top layer (drone 3) | 0.35 m (drones 2-3) | ok (0.01 m/s), ok (0.20 m/s), ok (0.00 m/s), ok (0.01 m/s) | PASS |
| mocap dropout mid-swap, top layer (drone 3) | 0.35 m (drones 2-3) | ok (0.01 m/s), ok (0.20 m/s), ok (0.00 m/s), ok (0.14 m/s) | PASS |
| low battery in formation (drone 2) | 0.35 m (drones 2-3) | ok (0.01 m/s), ok (0.02 m/s), ok (0.14 m/s), ok (0.04 m/s) | PASS |
| low battery mid-swap, top layer (drone 3) | 0.35 m (drones 2-3) | ok (0.01 m/s), ok (0.20 m/s), ok (0.00 m/s), ok (0.14 m/s) | PASS |

## Limits

- No inter-vehicle aerodynamics: downwash on a lower drone, wake and contact are not modeled. Layer spacing of 0.35 m is a geometric plan, not a validated downwash clearance.
- A faulted drone descends through lower layers; it is safe here only because the others are not below it at that moment. The ground station does not yet replan around a landing drone.
- Low battery is a voltage step only (no discharge curve, sag or reduced thrust); radio is ideal apart from the injected link loss; ESP-NOW timing and the P4<->C5 link are not in the loop. Not Crazyswarm2.
- All-height loss still FAULTs (motor stop) and is excluded from safe-landing claims.
