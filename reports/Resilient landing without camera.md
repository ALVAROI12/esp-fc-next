# Resilient landing without camera

## Method and delegation limit

Three independent free-model research launches were attempted (Flint, Nemotron and Beam).
All were rejected by the configured provider's free-usage limit. No subscription or paid
model fallback was used. The findings below are direct source inspection and experiments
in the current session, not an independent multi-agent consensus.

## Reproduced causes

Baseline: commit ea631d4, `SitlDrone(seed=9, profile="whoop2_4k", safe_landing=True)`,
1 m takeoff/hover, 3.5 s into the existing 0.6 m / 8 s figure-8, then camera input disabled.

- At loss, true velocity was (-0.444, +0.285) m/s; EKF (-0.467, +0.306) m/s.
- At 0.31 s, LAND commanded roll=pitch=0. These are level targets, **not braking**.
- At 4 s, true velocity was (-0.225, +0.024) m/s, but EKF still reported
  (-0.502, +0.462) m/s. Blindly enabling velocity hold with that unaided estimate is not justified.
- Model drag time constant is mass/drag = 0.060/0.020 = 3 s. Passive displacement limit
  from the initial speed is about 1.58 m; the actual run drifted 1.706 m before touchdown.
  Continued mission feedforward during the 300 ms detection window and modeled IMU bias also matter.

## Alternatives and selected first change

1. **Flow/range-aided velocity braking (selected):** already-planned PMW3901 + VL53L1X,
   no new component purchase. Fuse local flow continuously, then command zero XY velocity
   during camera-loss LAND only while flow/range inputs are fresh and plausible. Do not
   resume stale absolute XY hold or reacquire a jumped camera reference mid-landing.
2. **Unaided inertial braking:** smaller diff, but the experiment above shows velocity error
   grows after camera loss. Keep level descent as the explicitly degraded fallback, not a
   claimed position-hold solution.
3. **Redundant absolute positioning:** additional camera coverage, lighthouse/UWB, etc. can
   address the shared coordinate reference, but adds infrastructure/calibration and does not
   substitute for validating the short onboard emergency path.

Provisional gate before optimization: moving seed-9 camera-loss excursion <= 0.35 m from
input loss, no tumble/assertion, terminal LANDED, ground motor stop, approach <= 0.35 m/s.
Also test both loop rates, multiple seeds, noise/scale/latency, stale flow, range loss and
restored camera reference jumps. A failed gate remains a failure; do not relax it after seeing results.
This does not close FLOW-01 or guarantee arbitrary indoor textures, lighting, wind or collision avoidance.

## Height loss is a separate sensing problem

The current camera+ToF loss path has no independent vertical measurement. An IMU alone cannot
distinguish height/velocity drift indefinitely. A guaranteed safe landing for complete reliable
height loss cannot be manufactured by changing a timeout or guessing motor throttle.

The planned BMP581 is a sensible *redundant descent-control candidate*, not yet a validated
touchdown sensor. Bosch specifies 0.08 Pa noise only at lowest bandwidth/highest resolution,
but relative pressure accuracy is typically +/-6 Pa (about +/-0.5 m), with temperature and
installation effects. The current Gaussian-only `BaroModel` is therefore insufficient evidence
for a 4 cm touchdown decision. PX4 documents airflow pressure errors and uses staged ground
contact logic with low thrust, motion/rotation checks and dwell rather than altitude alone.

Next vertical stage must model pressure steps/drift/propwash, calibrate a ground reference,
gate health/freshness/innovations, and validate contact evidence before allowing barometer-only
automatic disarm. Keep total-height-loss FAULT explicit until that stage passes. No hardware
purchase or flight is authorized by these findings.

## Primary sources (accessed 2026-10-27)

- PX4 optical flow: velocity estimation requires flow plus downward distance, calibration and
  gyro compensation: https://docs.px4.io/main/en/sensor/optical_flow.html
- Bitcraze Flow deck v2: PMW3901 horizontal motion + VL53L1x range, surface/lighting dependence:
  https://www.bitcraze.io/products/flow-deck-v2/
- Bosch BMP581 technical data (noise vs relative accuracy):
  https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp581/
- PX4 static-pressure errors:
  https://docs.px4.io/main/en/advanced_config/static_pressure_buildup.html
- PX4 staged land detection and hover-throttle dependence:
  https://docs.px4.io/main/en/advanced_config/land_detector.html
- Repository evidence: `fc_step.c` LAND mode selection; `kalman_core.c` flow equations,
  `kalmanCoreUpdateWithBaro` and `kalmanCoreUpdateWithAbsoluteHeight`; model drag and sensor parameters.
