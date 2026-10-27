"""Opt-in latched hold/landing evidence from the real C runtime, not hardware validation.

    make failsafe
    python sim/failsafe_report.py --no-plots --out /path/to/report

All-height loss is a containment FAULT, never counted as successful controlled landing.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from sitl import SitlDrone
from sitl.drone import figure8
from model.params import DEFAULT_SENSORS, Sensors
from sitl.firmware import FAILSAFE_FAULT, FAILSAFE_HOLD, FAILSAFE_LAND, FAILSAFE_LANDED, FAILSAFE_NORMAL

OUT = Path(__file__).resolve().parent / "reports" / "failsafe"
NAMES = ("NORMAL", "HOLD", "LAND", "LANDED", "FAULT")


def fly_loss(profile: str, kind: str = "command", *, moving: bool = False, seed: int = 9,
             recover_after: float | None = None, tracking_jump: tuple = (0., 0., 0.),
             local_flow: bool = False, flow_scale: float = 1., flow_latency_s: float = 0.,
             sensors: Sensors = DEFAULT_SENSORS, yaw_deg: float = 0.) -> dict:
    """Drop command, camera, both, or all-height inputs independently. Record through recovery."""
    d = SitlDrone(seed=seed, profile=profile, safe_landing=True, local_flow=local_flow, sensors=sensors)
    d.flow_scale = flow_scale
    d.flow_latency_s = flow_latency_s
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    if not d.fw.armed or abs(d.quad.state.pos[2] - 1.0) > 0.05:
        raise AssertionError("loss scenario did not establish a 1 m flying hover")
    if moving:
        trajectory = figure8(d.time, np.array([0., 0., 1.]), 0.6, 8.0)
        d.follow(lambda t: (*trajectory(t)[:3], yaw_deg))
        d.run(3.5, record_hz=None)
    loss_time = d.time
    loss_pos = d.quad.state.pos.copy()
    d.link_up = kind not in ("command", "both")
    d.tracking_up = kind not in ("tracking", "both", "height")
    d.height_up = kind != "height"
    restored = False
    for _ in range(900):  # 9 s, sample state/physics at 100 Hz, run every firmware tick
        if recover_after is not None and d.time - loss_time >= recover_after and not restored:
            d.link_up = d.tracking_up = d.height_up = True
            d.tracking_offset = np.asarray(tracking_jump, dtype=float)
            restored = True
        d.run(0.01, record_hz=100)
    log = d.log.arrays()
    states = log["safety"].astype(int)
    t = log["t"] - loss_time
    transitions = np.r_[True, states[1:] != states[:-1]]
    sequence = [(float(t[i]), NAMES[states[i]]) for i in np.flatnonzero(transitions)]
    landed = np.flatnonzero(states == FAILSAFE_LANDED)
    land = np.flatnonzero(states == FAILSAFE_LAND)
    max_descent = float(np.max(np.maximum(0., -log["velocity"][land, 2]))) if len(land) else None
    motors_off = np.flatnonzero(np.max(log["motors"], axis=1) == 0)
    disarm_height = float(log["pos"][motors_off[0], 2]) if len(motors_off) else None
    # Physics clamps vz to zero at ground contact. Use the last logged pre-contact sample,
    # not the clamped zero, as the sampled touchdown approach speed (100 Hz resolution).
    contact = np.flatnonzero(log["pos"][:, 2] <= 1e-6)
    approach = None
    if len(contact) and contact[0] > 0:
        approach = float(max(0., -log["velocity"][contact[0] - 1, 2]))
    drift = np.linalg.norm(log["pos"][:, :2] - loss_pos[:2], axis=1)
    hold = np.flatnonzero(states == FAILSAFE_HOLD)
    anchor = log["pos"][hold[0], :2] if len(hold) else loss_pos[:2]
    after_hold = np.arange(len(states)) >= (hold[0] if len(hold) else 0)
    hold_drift = float(np.linalg.norm(log["pos"][after_hold, :2] - anchor, axis=1).max())
    terminal = int(states[-1])
    if kind == "height":
        passed = terminal == FAILSAFE_FAULT and not d.fw.armed and np.all(d.command == 0)
    elif kind == "command" and recover_after is not None and recover_after < 1.0:
        settled = t > recover_after + 2.0
        passed = (terminal == FAILSAFE_NORMAL and len(land) == 0 and d.fw.armed
                  and np.max(np.abs(log["pos"][settled, 2] - 1.0)) < 0.05
                  and np.any(log["motors"][-1] > 0))
    else:
        passed = (terminal == FAILSAFE_LANDED and len(landed) > 0 and len(land) > 0 and not d.fw.armed
                  and np.all(d.command == 0) and disarm_height is not None and disarm_height < 0.04
                  and approach is not None and approach <= 0.35
                  and max_descent is not None and max_descent <= 0.5
                  and np.all(states[land[0]:] >= FAILSAFE_LAND) and FAILSAFE_FAULT not in states)
        if kind == "command":
            passed = passed and hold_drift < 0.25
        if kind == "tracking" and recover_after is not None:
            restored_window = t > recover_after + 0.05
            if local_flow:
                passed = (passed and np.any(log["flow_braking"][restored_window])
                          and not np.any(log["position_fresh"][restored_window & (states == FAILSAFE_LAND)]))
            else:
                passed = (passed and np.all(log["attitude_target"][restored_window] == 0)
                          and np.any(log["position_fresh"][restored_window]))
    passed = bool(passed and not d.fw.tumbled and d.fw.assert_count == 0)
    return {"drone": d, "log": log, "profile": profile, "kind": kind, "moving": moving,
            "recover_after": recover_after, "tracking_jump": tracking_jump, "loss_time": loss_time, "sequence": sequence,
            "disarm_height": disarm_height, "approach": approach, "max_drift": float(drift.max()),
            "hold_drift": hold_drift, "landed_at": float(t[landed[0]]) if len(landed) else None,
            "max_descent": max_descent,
            "passed": passed}


def plot_runs(runs: list[dict], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True, dpi=140)
    for r in runs:
        if r["profile"] != "whoop2_4k" or r["recover_after"] is not None:
            continue
        a = r["log"]
        label = ("camera + ToF lost (FAULT)" if r["kind"] == "height" else r["kind"]) + (", moving" if r["moving"] else ", hover")
        t = a["t"] - r["loss_time"]
        axes[0].plot(t, a["pos"][:, 2], label=label)
        axes[1].plot(t, a["velocity"][:, 2])
        axes[2].step(t, a["safety"], where="post")
    axes[0].set_ylabel("true height (m)")
    axes[0].legend(fontsize=8)
    axes[1].set_ylabel("true vertical speed (m/s)")
    axes[2].set_ylabel("supervisor state")
    axes[2].set_yticks(range(5), NAMES, fontsize=8)
    axes[2].set_xlabel("seconds since input loss")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    cases = [("command", False, None), ("command", True, None), ("tracking", True, None),
             ("both", True, None), ("command", False, 0.8), ("command", True, 2.5),
             ("tracking", True, 0.8), ("height", False, None)]
    runs = []
    for profile in ("whoop2_interim", "whoop2_4k"):
        for kind, moving, recovery in cases:
            print(profile, kind, "moving" if moving else "hover", "recovery", recovery, flush=True)
            jump = (0.5, -0.4, 0.) if kind == "tracking" and recovery is not None else (0., 0., 0.)
            runs.append(fly_loss(profile, kind, moving=moving, recover_after=recovery, tracking_jump=jump))
    if not args.no_plots:
        plot_runs(runs, args.out / "landing.png")
    passed = all(r["passed"] for r in runs)
    lines = ["# Latched hold-and-land: SITL evidence", "", f"**Checks: {'PASS' if passed else 'FAIL'}**", "",
             "Real C runtime with opt-in safety enabled, 1/4 kHz variants, nominal 2-inch whoop model, seed 9. "
             "Not hardware validation, a collision guarantee, or a position-accuracy improvement.", "",
             "Command loss: after 500 ms capture a fixed estimated pose, clear trajectory feedforward, hold 1.5 s, "
             "then descend. Tracking loss: after 300 ms start landing immediately; disable stale XY control and level. "
             "LAND remains latched when command/tracking input returns. Independent modeled ToF supports height. "
             "All-height loss produces FAULT/motor stop, **not** successful controlled landing.", "",
             "| Rate/profile | Input loss | Motion | Recovery | State sequence | Landed after | Disarm height | Max descent / approach speed | Max XY drift | Check |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in runs:
        fmt = lambda x, suffix: "n/a" if x is None else f"{x:.3f}{suffix}"
        sequence = " → ".join(f"{name}@{t:.2f}s" for t, name in r["sequence"])
        loss_label = "camera + ToF (FAULT case)" if r["kind"] == "height" else r["kind"]
        check_label = "PASS (fault containment only)" if r["passed"] and r["kind"] == "height" else "PASS" if r["passed"] else "FAIL"
        lines.append(f"| {r['profile']} | {loss_label} | {'moving' if r['moving'] else 'hover'} | "
                     f"{fmt(r['recover_after'], ' s')} | {sequence} | {fmt(r['landed_at'], ' s')} | "
                     f"{fmt(r['disarm_height'], ' m')} | {fmt(r['max_descent'], '')} / {fmt(r['approach'], ' m/s')} | "
                     f"{r['max_drift']:.3f} m | {check_label} |")
    lines += ["", "## Gates and interpretation", "",
              "- Controlled cases: terminal LANDED, motors zero, disarm below 4 cm truth height, sampled approach "
              "speed <= 0.35 m/s, max LAND descent <= 0.5 m/s, no FAULT/tumble/assertion and no return to NORMAL after LAND.",
              "- Command-only loss with healthy tracking: <= 25 cm XY excursion from the first HOLD sample. "
              "The table's drift starts at input loss and includes the 500 ms detection delay/braking transient.",
              "- Recovery during HOLD resumes commands; recovery during LAND must not cancel landing. "
              "The restored-camera case injects a (+0.5, -0.4, 0) m reference jump at 0.8 s. "
              "Controller roll/pitch targets must remain zero after that restoration (no XY reacquisition).",
              "- All-height-loss case passes only **fault containment**, not the controlled-landing gate. "
              "Its impact speed and drift must not be interpreted as successful landing.", "",
              "## Limits", "",
              "- No bounded XY drift after camera loss; a moving vehicle can continue drifting into obstacles. "
              "Flow-only absolute accuracy (FLOW-01) remains unmet. This is not collision-safe swarm flight.",
              "- ToF uses an ideal flat ground plane with Gaussian noise and no real offset/occlusion. "
              "Touchdown is a height/vertical-speed dwell heuristic, not a contact sensor.",
              "- Approach speed is the last 100 Hz logged pre-contact vertical speed; the ground model clamps "
              "vertical velocity on contact. It is not a measured physical impact speed.",
              "- Input freshness measures receipt, not acquisition time, replay detection, estimator covariance "
              "or sensor accuracy. Plausible-but-frozen measurements are not detected.",
              "- No fresh height means guaranteed controlled landing is impossible. FAULT containment stops motors; "
              "it can cause a fall. Faulted/landed states need explicit ground reinitialization.",
              "- The production P4 image still uses the original task loop. Drivers, deadlines and hardware "
              "touchdown are unvalidated. Safe mode remains opt-in.", ""]
    if not args.no_plots:
        lines += ["![Height, speed and state](landing.png)", ""]
    (args.out / "README.md").write_text("\n".join(lines))
    print(f"Report: {args.out / 'README.md'} ({'PASS' if passed else 'FAIL'})")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
