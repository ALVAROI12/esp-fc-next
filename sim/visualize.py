"""Generate the interactive 3D replay viewer for the G1 scenarios.

Runs the nonlinear 6-DOF closed-loop simulation (Python port of the firmware cascaded PID) for
several scenarios and embeds the results in a self-contained HTML viewer:

    python sim/visualize.py            # -> sim/reports/viewer/index.html
    python sim/visualize.py --open     # ...and open it in the browser
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import webbrowser

import numpy as np

from model import linear, robust
from model.closed_loop import CascadedPidSim
from model.firmware import AttitudeGains, PidGains, load_gains
from model.params import DEFAULT_TIMING, PLANNED_TIMING, WHOOP_2IN, LoopTiming, Vehicle

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / "viewer" / "viewer.html"
GAINS_JSON = HERE / "reports" / "g1" / "proposed_gains.json"
OUT = HERE / "reports" / "viewer" / "index.html"

DURATION = 3.5
PLAYBACK_HZ = 250
COLORS = ["#e4572e", "#f3a712", "#29b6f6", "#76b041", "#ab47bc", "#26a69a", "#ef5350", "#8d6e63"]


def roll_doublet(t: float) -> float:
    return 15.0 if 0.5 <= t < 1.0 else (-15.0 if 1.0 <= t < 1.5 else 0.0)


def gust(t: float) -> np.ndarray:
    """50 ms roll torque kick at t = 2.0 s (about 180 deg/s of rate disturbance)."""
    return np.array([2e-3, 0.0, 0.0]) if 2.0 <= t < 2.05 else np.zeros(3)


def load_designed_gains() -> tuple[AttitudeGains, AttitudeGains]:
    data = json.loads(GAINS_JSON.read_text())
    as_gains = lambda d: AttitudeGains(**{k: PidGains(**v) for k, v in d.items()})  # noqa: E731
    return as_gains(data["gains"]), as_gains(data["interim_gains_for_stock_timing"]["gains"])


def fly(label: str, note: str, vehicle: Vehicle, timing: LoopTiming, gains: AttitudeGains, color: str) -> dict:
    sim = CascadedPidSim(vehicle, timing, gains, substeps=2 if timing.sensor_rate_hz > 1000 else 4)
    tr = sim.run(DURATION, roll_sp_deg=roll_doublet, torque_disturbance=gust, altitude_hold=1.0)
    step = max(1, round(timing.sensor_rate_hz / PLAYBACK_HZ))
    idx = np.arange(0, len(tr.t), step)
    m = linear.margins(linear.rate_loop(vehicle, timing, "roll", gains.roll_rate))
    seg = tr.t > 2.6
    shake = float(np.std(tr.rate_deg[seg, 0])) if np.any(seg) else float("nan")
    r = lambda a, d=4: np.round(a, d).tolist()  # noqa: E731
    return {
        "label": label,
        "note": note,
        "color": color,
        "pm": round(m.phase_margin_deg, 1) if m.stable else None,
        "gm": round(m.gain_margin_db, 1) if m.stable else None,
        "stable": m.stable,
        "shake": round(shake, 1),
        "lost": tr.lost_control_at,
        "t": r(tr.t[idx], 4),
        "pos": r(tr.pos[idx]),
        "quat": r(tr.quat[idx], 5),
        "roll": r(tr.euler_deg[idx, 0], 2),
        "rollsp": r(tr.roll_sp[idx], 1),
        "rate": r(tr.rate_deg[idx, 0], 1),
        "motors": r(tr.command[idx], 4),
    }


def scenarios() -> list[dict]:
    fw = load_gains()
    proposed, interim = load_designed_gains()
    v = WHOOP_2IN
    worst = replace(v.scaled(mass=1.2, thrust_max=1.2, ixx=0.8, iyy=0.8, izz=0.8), motor_tau=0.015)
    slow = lambda t: replace(t, compute_delay_samples=t.compute_delay_samples + 1)  # noqa: E731

    trio = lambda veh, d: [  # noqa: E731
        fly("Stock firmware @ 500 Hz", "ESP-Drone today: Crazyflie gains, 500 Hz loop", veh, d(DEFAULT_TIMING), fw, COLORS[0]),
        fly("Interim gains @ 500 Hz", "G1 robust gains for the current loop timing", veh, d(DEFAULT_TIMING), interim, COLORS[1]),
        fly("ESP-FC Next @ 4 kHz", "G1 proposed gains, planned 4 kHz loop", veh, d(PLANNED_TIMING), proposed, COLORS[2]),
    ]

    rng = np.random.default_rng(11)
    unc = robust.Uncertainty()
    fleet = []
    for i in range(6):
        vv, tt = unc.sample(v, PLANNED_TIMING, rng)
        label = f"Airframe {i + 1}"
        note = (f"mass {vv.mass * 1000:.0f} g, thrust {vv.thrust_max:.2f} N/motor, inertia x{vv.ixx / v.ixx:.2f}, "
                f"motor {vv.motor_tau * 1000:.0f} ms, delay {tt.compute_delay_samples} sample(s)")
        fleet.append(fly(label, note, vv, tt, proposed, COLORS[(i + 2) % len(COLORS)]))

    maneuver = "Roll doublet (+15 deg, -15 deg, level) with altitude hold, then a gust (roll torque kick) at t = 2.0 s."
    return [
        {"id": "nominal", "title": "Nominal 2-inch whoop",
         "description": f"{maneuver} All three use the nominal airframe from sim/model/params.py.", "drones": trio(v, lambda t: t)},
        {"id": "worst", "title": "Worst-case airframe",
         "description": (f"{maneuver} Airframe at a +/-20 % corner: 20 % heavier but 20 % more thrust, 20 % less inertia, "
                         "fast 15 ms motors, and one extra sample of delay. The G1 linear model predicts the stock "
                         "firmware is unstable here; saturation turns that into a violent limit cycle."),
         "drones": trio(worst, slow)},
        {"id": "fleet", "title": "Monte Carlo fleet (ESP-FC Next @ 4 kHz)",
         "description": (f"{maneuver} Six randomly drawn airframes from the G1 uncertainty envelope, all flying the "
                         "same proposed gains: robustness made visible."), "drones": fleet},
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--open", action="store_true", help="open the viewer in the default browser")
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()

    data = {"duration": DURATION, "scenarios": scenarios()}
    html = TEMPLATE.read_text().replace("/*__SIM_DATA__*/null", json.dumps(data, separators=(",", ":")))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html)
    print(f"Viewer: {args.out} ({args.out.stat().st_size / 1e6:.1f} MB)")
    if args.open:
        webbrowser.open(args.out.as_uri())


if __name__ == "__main__":
    main()
