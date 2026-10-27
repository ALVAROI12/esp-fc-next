# Flow-aided emergency braking implementation

Goal: reduce camera-loss landing drift without pretending the camera reference or an unaided
velocity estimate remains reliable. Preserve baseline opt-in landing and legacy behavior.

1. Write a failing both-rate SITL regression for <= 0.35 m nominal moving camera-loss excursion.
2. Keep the existing disabled-by-default safe-mode switch. Within safe mode, fresh qualified
   local flow/range automatically enables braking; simulations add it explicitly with `local_flow=True`.
3. Track successfully queued, validated flow receipt; brake only with fresh usable flow and ToF.
   Use modeVelocity with zero XY target. Otherwise use explicit degraded level descent.
4. Do not fuse a restored camera reference into a latched camera-loss landing in this mode.
5. Add independent flow sensor RNG and loss/quality/scale/latency controls in SITL; do not alter
   baseline runs or feed truth directly to firmware.
6. Test both rates and dropped/stale/invalid flow/range and restored/jumped camera input.
7. Export baseline/candidate comparisons, run regression suite and sanitizer tests, inspect replay,
   document the unchanged total-height-loss limitation. Commit only evidence-supported results.

Research and vertical follow-up: `reports/Resilient landing without camera.md`.

Before broader experiments: require <= 0.50 m excursion for seeds 1/5/9, both rates,
common-mode flow scale 0.8/1.2, doubled pixel noise, and 20 ms delivery delay. Retain the
stricter <= 0.35 m nominal seed-9 gate. Preserve failures rather than weakening these targets.
