"""Compare passive camera-loss landing with flow/range-aided emergency braking.

Thresholds are declared in docs/plans/2026-10-27-flow-braking.md before the stress sweep.
This is not an absolute-positioning, total-height-loss, or hardware flight gate.
"""

import argparse
import math
from dataclasses import replace
from pathlib import Path

import numpy as np

from failsafe_report import fly_loss
from model.params import DEFAULT_SENSORS

OUT = Path(__file__).resolve().parent / "reports" / "flow_braking"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for profile in ("whoop2_interim", "whoop2_4k"):
        cases = [("baseline, no local flow", False, 9, {}, None),
                 ("flow, nominal", True, 9, {}, .35),
                 ("flow, seed 1", True, 1, {}, .50),
                 ("flow, seed 5", True, 5, {}, .50),
                 ("flow, scale -20%", True, 9, {"flow_scale": .8}, .50),
                 ("flow, scale +20%", True, 9, {"flow_scale": 1.2}, .50),
                 ("flow, delay 20ms", True, 9, {"flow_latency_s": .02}, .50),
                 ("flow, yaw 90deg", True, 9, {"yaw_deg": 90.}, .50),
                 ("flow, doubled pixel noise", True, 9, {"sensors": replace(DEFAULT_SENSORS,
                     flow=replace(DEFAULT_SENSORS.flow, std_pixels=4.))}, .50),
                 ("flow, rotation factor -20%", True, 9, {"flow_model": {"omega_factor": 1.0}}, .50),
                 ("flow, rotation factor +20%", True, 9, {"flow_model": {"omega_factor": 1.5}}, .50),
                 ("flow, sensor mount yaw 15deg", True, 9, {"flow_model": {"mount_yaw_rad": math.radians(15)}}, .50),
                 ("flow, ToF reads 2 cm long", True, 9, {"tof_bias_m": .02}, .50),
                 ("flow, restored jumped camera", True, 9,
                  {"recover_after": .8, "tracking_jump": (.5, -.4, 0)}, .35)]
        for name, local_flow, seed, kwargs, limit in cases:
            print(profile, name, flush=True)
            run = fly_loss(profile, "tracking", moving=True, local_flow=local_flow, seed=seed, **kwargs)
            passed = (run["passed"] and (not local_flow or np.any(run["log"]["flow_braking"]))
                      and (limit is None or run["max_drift"] <= limit))
            rows.append((profile, name, run, limit, passed))
    if not args.no_plots:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(9, 3.5), dpi=140)
        for profile, name, run, _, _ in rows:
            if profile != "whoop2_4k" or name not in ("baseline, no local flow", "flow, nominal"):
                continue
            a = run["log"]
            t = a["t"] - run["loss_time"]
            axes[0].plot(t, np.linalg.norm(a["pos"][:, :2] - a["pos"][0, :2], axis=1), label=name)
            axes[1].plot(t, np.linalg.norm(a["velocity"][:, :2], axis=1), label=name)
        axes[0].set_ylabel("XY excursion since loss (m)")
        axes[1].set_ylabel("true horizontal speed (m/s)")
        for ax in axes:
            ax.set_xlabel("seconds since camera loss"); ax.grid(alpha=.25); ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(args.out / "braking.png"); plt.close(fig)
    passed = all(row[-1] for row in rows)
    lines = ["# Flow-aided emergency braking", "", f"**Declared checks: {'PASS' if passed else 'FAIL'}**", "",
             "Same real C runtime and nominal 2-inch model, both loop rates. All losses start while moving. "
             "Local flow has its own seeded RNG; no ground-truth velocity/position is sent to the controller "
             "except through the explicitly modeled sensors. Baseline is the previous level-descent behavior.", "",
             "| Profile | Case | Max XY excursion | Max descent | Sampled approach | Drift gate | Result |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    fmt = lambda value: "n/a" if value is None else f"{value:.3f}"
    for profile, name, run, limit, ok in rows:
        lines.append(f"| {profile} | {name} | {run['max_drift']:.3f} m | {fmt(run['max_descent'])} m/s | "
                     f"{fmt(run['approach'])} m/s | {'baseline only' if limit is None else f'{limit:.2f} m'} | "
                     f"{'PASS' if ok else 'FAIL'} |")
    if not args.no_plots:
        lines += ["", "![Excursion and horizontal speed](braking.png)"]
    lines += ["", "## Policy and limits", "",
              "- Safe mode brakes XY to zero velocity only with fresh qualified flow and downward range. "
              "Missing/rejected flow or near-ground range falls back to explicitly degraded level descent.",
              "- Once local braking starts, restored camera packets are not fused during that LAND: no "
              "reacquisition of a jumped frame. LAND remains latched.",
              "- Nominal gate <= 0.35 m; declared sampled stress gate <= 0.50 m. Controlled-landing truth/motor "
              "checks from `failsafe_report.py` must also pass. These are not 5 cm positioning gates.",
              "- Sensor pixel noise/calibration are synthetic. Flow equations mirror the existing EKF except for the "
              "declared structural stresses (rotation-compensation factor +/-20%, 15 deg sensor mounting yaw, "
              "+2 cm ToF offset); this is not independent sensor calibration.",
              "- A ToF that reads 2 cm short reports negative range on the ground, so safe mode refuses to arm "
              "(fail-safe, but real sensors need offset calibration before flight).",
              "- Lighting, surface texture, wind, arbitrary speeds/heights, long latencies and all corruption "
              "patterns are not validated. Finite frozen sensor data is not detected by receipt freshness.",
              "- Camera+ToF loss without another qualified height source still FAULTs. This change does NOT "
              "solve total-height loss or FLOW-01. Barometer/contact redundancy requires a separate validated stage.",
              "- The free-model research attempts were quota-blocked; no independent multi-agent review is claimed. "
              "Research sources and vertical limitations are in `reports/Resilient landing without camera.md`.",
              "- Production P4 runtime/drivers and physical flight remain unvalidated.", ""]
    (args.out / "README.md").write_text("\n".join(lines))
    print(f"Report: {args.out / 'README.md'}")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
