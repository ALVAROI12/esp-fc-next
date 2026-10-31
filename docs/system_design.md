# ESP-FC Next: system design

State of the design on 2026-10-31. Everything below the "evidence" column marked *sim* has run only on the host
or in SITL; nothing has run on a board yet (Phase 2 kit not bought). Sources: `docs/build_plan_pdf.py`,
`docs/fc_core.md`, `docs/protocols.md`, `hardware/DESIGN.md`, `sim/README.md`.

## 1. Requirements

### Functional
- Fly a 2-inch ducted brushless quad indoors on the real Crazyflie flight algorithms (fc_core), 1S battery.
- Four drones at once, commanded from one PC through Crazyswarm2 / cflib (CRTP kept wire-compatible).
- Positioning: external mocap (PS3 Eye cameras first) fused on board with flow, ToF and IMU; flow/ToF alone
  only for short dropouts (FLOW-01: flow-only absolute XY does not meet accuracy).
- Every fault ends in a controlled landing: command loss, camera loss, low battery, and the swarm variants.
- Manual override on every real flight (ELRS handheld over CRSF) independent of Wi-Fi and the C5.
- Later (Phase 5): camera, H.264 over 5 GHz, GPS, Remote ID. Not designed yet; only the chip choice keeps it open.

### Non-functional
| Need | Target | Where it is checked |
| --- | --- | --- |
| Control loop | 4 kHz rate/attitude, 500 Hz EKF prediction, measured jitter < 5 us on the P4 | G1 margins (sim); jitter is G5 (hardware) |
| Hover accuracy | +/- 5 cm hover, < 10 cm figure-8 | G2 SITL: < 3 cm hover, 4-5 cm figure-8 |
| Swarm | 4 drones, minimum separation >= 0.35 m in the missions, zero collisions | `make swarm` (sim) |
| Link | commands authenticated, replay rejected, ground reaches all drones from one radio | unit tests + 1 h libFuzzer |
| Failsafe timing | 0.5 s silence -> hold, 1.5 s more -> land; battery < 3.3 V/cell for 1 s -> land | `make failsafe` (sim) |
| Cost | ~$25-40 per assembled FC at 5 boards; whole Phase 2 kit $120-180 | plan |
| Size | 28 mm square FC + radio board stack, 20 x 20 mm M2 mounts, 50-70 g all-up | `hardware/` (G4) |

### Constraints
- One developer, simulation-first: gates G1-G6 in order, no hardware until G3 passed (it has).
- ESP-IDF pinned to `release-v5.5` in Docker; `firmware/flight/` is a git subtree of espressif/esp-drone, so
  changes there stay minimal and marked.
- Flight-loop code: no FreeRTOS calls, no dynamic allocation, builds on host and target.
- GPL-3.0, upstream headers kept.

## 2. High-level design

```
 PC (Crazyswarm2 / cflib / scripts)            mocap (4x PS3 Eye)  --ROS 2--> Crazyswarm2
   | USB serial, framed stream (tools/espfc_link.py, espfc_cflib.py: espfc://<id>@<port>)
   v
 ground ESP32-C5 (radio id 0)  ===== ESP-NOW broadcast, AES-128-CCM envelope =====  drone C5 (id 1..254)
                                                                                      | 64-byte SPI frame, 500 Hz,
                                                                                      | P4 master, CRC-16
                                                                                      v
                                                                                  ESP32-P4 flight controller
                                                                                  +------------------------------+
   ELRS receiver --CRSF UART-------------------------------------------------->   | fc_runtime (FreeRTOS glue)   |
   ICM-42688-P (SPI, 4-8 kHz)  BMP581  PMW3901 + VL53L1X  INA226 ------------->   |   fc_step() per tick:        |
                                                                                  |   calib -> LPF/RPM notch ->  |
   4-in-1 ESC (DShot, RPM telemetry) <----------------------------------------    |   EKF -> watchdog/failsafe ->|
                                                                                  |   PID -> mixer               |
                                                                                  +------------------------------+
```

Components and ownership:

| Component | Where | Role |
| --- | --- | --- |
| `fc_core` (portable C) | `firmware/fc_core/{include,src}` | 18 Crazyflie algorithm files unchanged (PID, EKF, mixer, Mahony) + `fc_step.c` (synchronous loop), `fc_kalman.c`, `fc_rpm_notch.c`, `fc_link*.c`, `fc_radio.c`, board profiles |
| HAL | Crazyflie driver headers (`motors.h`, `usec_time.h`, `console.h`, `cfassert.h`, `platform.h`) | Target: ESP-IDF drivers. Host: `host/src/fc_host.c` records motor ratios, simulated clock |
| P4 runtime glue | `firmware/fc_core/target/fc_runtime.c`, `c5_link.c` | Runs `fc_step` in ESP-Drone's stabilizer task (`CONFIG_ESPFC_FC_STEP`), static measurement queue, CRTP over the C5 SPI link (`CONFIG_TRANSPORT_C5_SPI`) |
| P4 firmware | `firmware/flight/` (esp-drone subtree) | CRTP stack, params/logs, drivers, upstream high-level commander/planner |
| C5 radio firmware | `firmware/radio/` | ESP-NOW <-> SPI bridge (`fc_radio`), NVS counter blocks, ground mode over USB Serial/JTAG |
| Ground tools | `tools/` | framed serial codec, cflib driver, provisioning (`provision_radio.py`, key in git-ignored `secrets/`) |
| Simulation | `sim/model` (G1 6-DOF, sensors, vibration), `sim/sitl` (ctypes lockstep, one library copy per drone), reports, viewer | G1/G2 evidence, Gym-style env |
| Hardware | `hardware/boards.py` (SKiDL), `pcb.py`, `route.py`, tests | Two 28 mm boards as code, netlist/ERC/DRC/power gate (G4) |

### Data flow, one control tick (1 ms host, 0.25 ms planned)
1. IMU sample -> plausibility (non-finite or beyond limits held up to 5 ms, then disarm/FAULT) -> calibration ->
   RPM notch bank (per motor 1x/2x, 100 Hz RPM telemetry, bypass after 100 ms stale) -> 2-pole LPF.
2. Queued measurements (mocap pose, ToF, flow; fixed-size static queue) -> EKF update; prediction at 500 Hz.
3. Commander: newest CRTP setpoint, or high-level planner output while the link is alive; watchdog ages it.
4. Failsafe supervisor (opt-in `fcRun.safeMode`): NORMAL -> HOLD -> LAND -> LANDED / FAULT (section 3.3).
5. PID cascade -> mixer -> motor ratios (only calibrated, armed, not e-stopped, not tumbled).

### Command path
PC -> `GroundLink.send_crtp(dest, pkt)` -> framed serial -> ground C5 seals `CRTP_PEER` (dest + CRTP) in the
envelope -> ESP-NOW broadcast -> every drone C5 authenticates, checks replay, drops frames not addressed to it
(id or 255) or not from id 0 -> SPI `CRTP` frame -> P4 `c5LinkExchange` -> CRTP stack -> commander.
Telemetry goes back the same way, tagged with the sender id, routed per link by a reader thread on the PC.

## 3. Deep dive

### 3.1 Wire formats (`docs/protocols.md`)
- SPI frame: 64 bytes, magic `0xA5`, type (IDLE / CRTP / SWARM_STATUS / CRTP_PEER), per-direction seq, len,
  payload (raw CRTP <= 31 B), CRC-16/CCITT-FALSE, zero tail. Decoder rejects short, magic, type, length, CRC,
  non-zero tail in that order; a frame that lost only tail bytes otherwise swallows the next frame on the
  serial stream.
- Radio envelope: version, sender id, 32-bit counter, AES-128-CCM ciphertext, 8-byte tag. Nonce = network id +
  sender + counter; AAD = header. Tag check before the replay window (32 back, each counter once). Counters are
  reserved in NVS blocks of 4096 so a reboot never reuses a nonce; sending stalls rather than overruns.
- Swarm status: 20 bytes at 50 Hz (id, state, pos/vel int16 mm, battery mV, sender ms); refuses NaN/out of
  range instead of clamping. Received by the P4 today but ignored (no swarm-aware control yet).
- CRTP itself is unchanged, protocol version 4 (so cflib uses the old GO_TO/START_TRAJECTORY layouts).

### 3.2 Timing and tasks on the P4
- `fc_step` runs inside ESP-Drone's stabilizer task (opt-in image); measurement pushes come through a static
  queue from the CRTP/link tasks. `fcTime` logs cost per tick, max and overruns (G3 budget, unmeasured).
- C5 SPI master task at 500 Hz, one frame each way per transaction; `isConnected` = CRTP packet within 1 s.
- Default P4 image is still the upstream diagnostic profile (no stabilizer, outputs off); `fc_step` links on the
  ESP32-S3 ESP-Drone target. A P4 board profile (pins, IMU on SPI, DShot via RMT) does not exist yet.

### 3.3 Failsafe state machine (safe mode)
| State | Entry | Behaviour |
| --- | --- | --- |
| HOLD | 500 ms command silence, camera input > 300 ms old, or cell < 3.3 V for 1 s (0.1 V hysteresis) | fixed captured pose, feedforward zeroed; fresh commands resume before LAND |
| LAND | 1.5 s in HOLD | latched 0.25 m/s descent, 0.10 m/s below 15 cm; renewed commands cannot cancel; XY: fresh tracking, else flow/range braking, else level |
| LAND blind | camera and ToF both stale above 15 cm | IMU-propagated descent in a 0.97-0.99 x learned-hover thrust band for max(6 s, height / 0.2 m/s) |
| LANDED | <= 4 cm and vz <= 0.15 m/s for 500 ms | disarm, automatic rearm blocked |
| FAULT | no trustworthy height near the ground, budget or 20 s timeout exceeded, invalid input | motor stop; counted as a fault, not a landing |

Arming requires fresh external position, independent ToF, measured gravity level, pack above the low limit.
Emergency stop, explicit disarm and tumble (> 70 deg for 300 ms) override everything.

### 3.4 Storage and configuration
- NVS on the C5: `fclink` namespace (group key, network id, radio id, counter). Provisioned per radio by
  `tools/provision_radio.py --id N`; no default key, no key means the radio stays silent and the P4 lands.
- Flight parameters: ESP-Drone PARAM/LOG by cfclient name (84 params / 103 logs reachable in SITL).
- Blackbox to flash/microSD: planned, not designed.

### 3.5 Error handling and retries
- Link: ESP-NOW broadcast is unacknowledged; cflib `needs_resending` is on. Drops are counted at every stage
  (auth, replay, size, type, queue full at 16 deep on both C5 and P4). A ground station flooding one CRTP port
  can still trip upstream `crtpRxTask` asserts (motor stop): upstream behaviour, open.
- Sensors: bad IMU samples held, then disarm/FAULT; non-finite position/ToF/flow dropped; stale sources expire
  by receipt age (plausible frozen data is not detected).
- Firmware asserts on target reset the chip and snapshot; the supervisor is not a redundant safety system.

## 4. Scale and reliability

- Load: 4 drones x 50 Hz status + command traffic is far below ESP-NOW capacity; the replay window and the
  per-sender counters are O(members). Beyond ~10 drones, broadcast status at 50 Hz starts to matter; drop to
  20 Hz or time-slot it from the ground clock (time sync is specified, not implemented).
- Failure isolation: the C5 or Wi-Fi dying leaves the P4 stabilizing and landing on its own watchdog; the
  ELRS/CRSF path is the independent manual override. The P4's LP core is not a safety barrier.
- Single points of failure: the ground C5 (one radio for all drones; a second dongle on another channel is the
  cheap fix), the group key (a compromised drone can impersonate the ground; per-sender keys fix it).
- Determinism: SITL runs the firmware in lockstep (one library copy per drone), so swarm fault scenarios are
  repeatable; the autorouter is the one non-deterministic tool in the pipeline (retried, DRC-gated).
- Monitoring: `fcSafety` (state, reason, held pose, input ages), `fcTime` (loop cost), link drop counters,
  swarm status broadcast. No alerting beyond logs; the ground script decides.

## 5. Trade-offs made

| Decision | Chosen | Alternative | Why |
| --- | --- | --- | --- |
| Chips | P4 (flight) + C5 (radio) on two boards | one S3; P4 + its EV board's C6 | radio interrupts never touch the loop; C5 has 5 GHz + ESP-NOW; C6 on the EV board is SDIO-only |
| Flight code | Crazyflie 2021 algorithms via esp-drone subtree, unchanged | rewrite, or Betaflight | cflib/Crazyswarm2 compatibility for free; upstream pulls stay possible; the cost is a 5-year-old EKF whose process noise is tied to a 1 ms cadence |
| Loop | synchronous `fc_step` shared by SITL and target | upstream task soup | same code flies the sim and the board; the price is glue (`fc_runtime.c`) and a P4 task budget still unmeasured |
| Positioning | mocap first, flow/ToF as backup | UWB, flow-only | one shared frame and ground truth for ~$100; flow-only fails FLOW-01 |
| Security | app-level AES-CCM over ESP-NOW broadcast, group key | ESP-NOW built-in encryption, per-device keys | built-in covers unicast only; group key is simpler to provision, accepted risk until per-sender keys |
| Failsafe | opt-in latched hold-and-land, blind descent band | upstream cut-motors-after-2 s | upstream drops the drone; blind descent lands from 2-4 m under a 0.02 g bias in sim, but all-height loss still FAULTs |
| Board | 28 mm 4-layer, 0201 caps, autorouted, In1 plane plus outer pours | 25.5 mm (routes only sometimes), 6 layers, hand routing | still fits the whoop mount; density means signals cross the In1 plane and the pours do the GND stitching |
| Safety of learned parts | setpoints only, supervisor can override | end-to-end policy | keeps the verified loop as fallback; no on-board inference until benchmarked |

## 6. Assumptions to confirm on hardware (what changes the design if wrong)
- 4 kHz loop fits one P4 core with Wi-Fi/logging on (G5 criterion). If not: 2 kHz profile exists in G1.
- Startup calibration (1024 + 200 samples) is a time policy to be set, not a count.
- ToF offset, thrust sag at low battery, blind sink rate `FC_FAILSAFE_BLIND_MIN_SINK_MPS` (0.2) are calibration
  knobs; 1S cell count is assumed in the battery push.
- SPI pins and Wi-Fi channel are Kconfig placeholders; DRC rules are placeholders until a fab is chosen.
- The In1 plane is cut by signal tracks and vias; return paths are not analysed. Fine for a 4 kHz loop and
  10-20 MHz SPI on a 25 mm board, to be checked with the first boards (ringing on SPI, IMU noise floor).

## 7. Revisit as it grows
- More than 4 drones: time-slotted status, per-sender keys, second ground radio, swarm-aware control that
  actually consumes `SWARM_STATUS` on the P4 (today ignored).
- Phase 5 camera: the P4 H.264 path and 5 GHz video need their own SPI lane or a second interface; the current
  64-byte frame has no bulk lane.
- Blackbox: define before G6; the review tolerance "real logs match sim" needs it.
- Crazyswarm2 integration is a two-line cflib backend patch, untried (no ROS 2 here).
- If the 28 mm board starts fighting the router again (more parts, a fab with coarser rules), move to 6 layers
  rather than more placement heuristics.
