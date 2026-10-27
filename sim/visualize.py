"""Generate the interactive 3D replay viewer.

G1 scenarios fly the Python port of the firmware cascaded PID on the nonlinear 6-DOF model; the G2
scenario flies the real C flight firmware (SITL). Results are embedded in a self-contained HTML viewer:

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
from sitl import SitlDrone

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / "viewer" / "viewer.html"
GAINS_JSON = HERE / "reports" / "g1" / "proposed_gains.json"
OUT = HERE / "reports" / "viewer" / "index.html"

G1_DURATION = 3.5
PLAYBACK_HZ = 250
COLORS = ["#e4572e", "#f3a712", "#29b6f6", "#76b041", "#ab47bc", "#26a69a", "#ef5350", "#8d6e63"]
G1_CHARTS = [
    {"title": "Roll angle (deg) - dashed: setpoint", "series": "roll", "sp": "roll_sp"},
    {"title": "Roll rate (deg/s)", "series": "rate"},
    {"title": "Motor M1 command (% of full scale)", "series": "m1"},
]


def r(a, d=4):
    return np.round(np.asarray(a, dtype=float), d).tolist()


def roll_doublet(t: float) -> float:
    return 15.0 if 0.5 <= t < 1.0 else (-15.0 if 1.0 <= t < 1.5 else 0.0)


def gust(t: float) -> np.ndarray:
    """50 ms roll torque kick at t = 2.0 s (about 180 deg/s of rate disturbance)."""
    return np.array([2e-3, 0.0, 0.0]) if 2.0 <= t < 2.05 else np.zeros(3)


def load_designed_gains() -> tuple[AttitudeGains, AttitudeGains]:
    data = json.loads(GAINS_JSON.read_text())
    as_gains = lambda d: AttitudeGains(**{k: PidGains(**v) for k, v in d.items()})  # noqa: E731
    return as_gains(data["gains"]), as_gains(data["interim_gains_for_stock_timing"]["gains"])


def fly_g1(label: str, note: str, vehicle: Vehicle, timing: LoopTiming, gains: AttitudeGains, color: str) -> dict:
    sim = CascadedPidSim(vehicle, timing, gains, substeps=2 if timing.sensor_rate_hz > 1000 else 4)
    tr = sim.run(G1_DURATION, roll_sp_deg=roll_doublet, torque_disturbance=gust, altitude_hold=1.0)
    idx = np.arange(0, len(tr.t), max(1, round(timing.sensor_rate_hz / PLAYBACK_HZ)))
    m = linear.margins(linear.rate_loop(vehicle, timing, "roll", gains.roll_rate))
    seg = tr.t > 2.6
    shake = float(np.std(tr.rate_deg[seg, 0])) if np.any(seg) else float("nan")
    ok = m.stable and shake <= 20
    margins = f"PM {m.phase_margin_deg:.1f} deg, GM {m.gain_margin_db:.1f} dB" if m.stable else "linear model: UNSTABLE"
    return {
        "label": label, "note": note, "color": color,
        "status": "ok" if ok else "bad", "badge": "stable" if ok else "oscillating",
        "stats": f"rate loop {margins} | roll-rate std after gust {shake:.1f} deg/s",
        "lost": tr.lost_control_at,
        "t": r(tr.t[idx]), "pos": r(tr.pos[idx]), "quat": r(tr.quat[idx], 5), "motors": r(tr.command[idx]),
        "series": {
            "roll": r(tr.euler_deg[idx, 0], 2), "roll_sp": r(tr.roll_sp[idx], 1),
            "rate": r(tr.rate_deg[idx, 0], 1), "m1": r(tr.command[idx, 0] * 100, 2),
        },
    }


SQUARE = [(0.5, 0.0, 1.0), (0.5, 0.5, 1.0), (0.0, 0.5, 1.0), (0.0, 0.0, 1.0)]


def fly_sitl(label: str, note: str, color: str, z_ki: float | None) -> dict:
    """Real firmware: calibrate on the ground, arm, take off, fly a 0.5 m square, hover."""
    d = SitlDrone(seed=4)
    if z_ki is not None:
        d.fw.set_param("posCtlPid.zKi", z_ki)
    every = round(1000 / PLAYBACK_HZ)
    while not d.fw.ready:
        d.step(every)
    d.run(0.3, every)
    d.fw.arm(True)
    d.takeoff(1.0, 2.0)
    d.run(4.0, every)
    for corner in SQUARE:
        d.goto(*corner, 0.0, duration=2.0)
        d.run(3.0, every)
    d.run(2.0, every)

    log = d.log.arrays()
    hover = log["t"] > log["t"][-1] - 2.0
    hover_err = float(np.max(np.linalg.norm(log["pos"][hover] - np.array(SQUARE[-1]), axis=1)))
    overshoot = float(log["pos"][:, 2].max() - 1.0)
    est_err = float(np.max(np.linalg.norm(log["est"][:, :3] - log["pos"], axis=1)[log["t"] > 3.0]))
    sp = log["setpoint"]
    return {
        "label": label, "note": note, "color": color,
        "status": "ok" if hover_err < 0.05 else "bad",
        "badge": f"hover error {hover_err * 100:.1f} cm",
        "stats": (f"takeoff overshoot {overshoot * 100:.1f} cm | estimator error max {est_err * 100:.1f} cm | "
                  f"asserts {d.fw.assert_count} | posCtlPid.zKi = {d.fw.param('posCtlPid.zKi'):g}"),
        "lost": None,
        "t": r(log["t"]), "pos": r(log["pos"]), "quat": r(log["quat"], 5), "motors": r(log["motors"]),
        "series": {
            "z": r(log["pos"][:, 2]), "z_sp": r(sp[:, 2]), "x": r(log["pos"][:, 0]), "x_sp": r(sp[:, 0]),
            "y": r(log["pos"][:, 1]), "y_sp": r(sp[:, 1]), "m1": r(log["motors"][:, 0] * 100, 2),
        },
    }


def scenarios() -> list[dict]:
    fw = load_gains()
    proposed, interim = load_designed_gains()
    v = WHOOP_2IN
    worst = replace(v.scaled(mass=1.2, thrust_max=1.2, ixx=0.8, iyy=0.8, izz=0.8), motor_tau=0.015)
    slow = lambda t: replace(t, compute_delay_samples=t.compute_delay_samples + 1)  # noqa: E731

    trio = lambda veh, d: [  # noqa: E731
        fly_g1("Stock firmware @ 500 Hz", "ESP-Drone today: Crazyflie gains, 500 Hz loop", veh, d(DEFAULT_TIMING), fw, COLORS[0]),
        fly_g1("Interim gains @ 500 Hz", "G1 robust gains for the current loop timing", veh, d(DEFAULT_TIMING), interim, COLORS[1]),
        fly_g1("ESP-FC Next @ 4 kHz", "G1 proposed gains, planned 4 kHz loop", veh, d(PLANNED_TIMING), proposed, COLORS[2]),
    ]

    rng = np.random.default_rng(11)
    unc = robust.Uncertainty()
    fleet = []
    for i in range(6):
        vv, tt = unc.sample(v, PLANNED_TIMING, rng)
        note = (f"mass {vv.mass * 1000:.0f} g, thrust {vv.thrust_max:.2f} N/motor, inertia x{vv.ixx / v.ixx:.2f}, "
                f"motor {vv.motor_tau * 1000:.0f} ms, delay {tt.compute_delay_samples} sample(s)")
        fleet.append(fly_g1(f"Airframe {i + 1}", note, vv, tt, proposed, COLORS[(i + 2) % len(COLORS)]))

    sitl = [
        fly_sitl("Firmware as shipped", "posCtlPid defaults from the Crazyflie position controller", COLORS[0], None),
        fly_sitl("Tuned in SITL", "posCtlPid.zKi = 0: the velocity loop integral already absorbs thrust offset", COLORS[2], 0.0),
    ]

    maneuver = "Roll doublet (+15 deg, -15 deg, level) with altitude hold, then a gust (roll torque kick) at t = 2.0 s."
    return [
        {"id": "sitl", "title": "G2 SITL: real firmware, takeoff + square",
         "description": ("The actual C flight firmware (fc_step: calibration, Kalman filter with 100 Hz mocap, position "
                         "controller, cascaded PID, mixer) flying the 6-DOF model with ICM-42688-P noise. Calibrates on "
                         "the ground, arms, takes off along a 2 s minimum-jerk path, flies a 0.5 m square at 1 m, hovers. "
                         "Left: firmware defaults; right: one parameter changed after the SITL diagnosis."),
         "duration": round(sitl[0]["t"][-1], 2), "spacing": 1.3, "camera": [0.6, 1.9, 3.4], "target_y": 0.7,
         "charts": [
             {"title": "Height z (m) - dashed: setpoint", "series": "z", "sp": "z_sp"},
             {"title": "x (m) - dashed: setpoint", "series": "x", "sp": "x_sp"},
             {"title": "y (m) - dashed: setpoint", "series": "y", "sp": "y_sp"},
             {"title": "Motor M1 command (% of full scale)", "series": "m1"},
         ], "drones": sitl},
        {"id": "nominal", "title": "G1: nominal 2-inch whoop", "duration": G1_DURATION, "charts": G1_CHARTS,
         "description": f"{maneuver} All three use the nominal airframe from sim/model/params.py.", "drones": trio(v, lambda t: t)},
        {"id": "worst", "title": "G1: worst-case airframe", "duration": G1_DURATION, "charts": G1_CHARTS,
         "description": (f"{maneuver} Airframe at a +/-20 % corner: 20 % heavier but 20 % more thrust, 20 % less inertia, "
                         "fast 15 ms motors, and one extra sample of delay. The G1 linear model predicts the stock "
                         "firmware is unstable here; saturation turns that into a violent limit cycle."),
         "drones": trio(worst, slow)},
        {"id": "fleet", "title": "G1: Monte Carlo fleet (ESP-FC Next @ 4 kHz)", "duration": G1_DURATION, "charts": G1_CHARTS,
         "description": (f"{maneuver} Six randomly drawn airframes from the G1 uncertainty envelope, all flying the "
                         "same proposed gains: robustness made visible."), "drones": fleet},
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--open", action="store_true", help="open the viewer in the default browser")
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()

    data = {"scenarios": scenarios()}
    html = TEMPLATE.read_text().replace("/*__SIM_DATA__*/null", json.dumps(data, separators=(",", ":")))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html)
    print(f"Viewer: {args.out} ({args.out.stat().st_size / 1e6:.1f} MB)")
    if args.open:
        webbrowser.open(args.out.as_uri())


if __name__ == "__main__":
    main()
