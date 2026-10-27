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
from model.vibration import VibrationSpec
from sitl import SitlDrone
from sitl.drone import figure8
from failsafe_report import fly_loss, NAMES as SAFETY_NAMES

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
    if data.get("gate_passed") is not True:
        raise ValueError("viewer requires a passing G1 gain report; run make g1")
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


def fly_sitl(label: str, note: str, color: str, profile: str) -> dict:
    """Real firmware: calibrate on the ground, arm, take off, fly a 0.5 m square, hover."""
    d = SitlDrone(seed=4, profile=profile)
    while not d.fw.ready:
        d.step(record=False)
    d.run(0.3, PLAYBACK_HZ)
    if not d.fw.arm(True):
        raise RuntimeError("supervisor refused to arm viewer scenario")
    d.takeoff(1.0, 2.0)
    d.run(4.0, PLAYBACK_HZ)
    for corner in SQUARE:
        d.goto(*corner, 0.0, duration=2.0)
        d.run(3.0, PLAYBACK_HZ)
    d.run(2.0, PLAYBACK_HZ)

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
        "t": r(log["t"] - log["t"][0]), "pos": r(log["pos"]), "quat": r(log["quat"], 5), "motors": r(log["motors"]),
        "series": {
            "z": r(log["pos"][:, 2]), "z_sp": r(sp[:, 2]), "x": r(log["pos"][:, 0]), "x_sp": r(sp[:, 0]),
            "y": r(log["pos"][:, 1]), "y_sp": r(sp[:, 1]), "m1": r(log["motors"][:, 0] * 100, 2),
        },
    }


def fly_tracking(label: str, color: str, profile: str, positioning: str = "mocap") -> dict:
    """A figure-8 comparison, or the unresolved flow-only absolute-position check."""
    d = SitlDrone(seed=5, profile=profile, positioning=positioning)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    start = d.time
    if positioning == "mocap":
        d.follow(figure8(start, np.array([0.0, 0.0, 1.0]), 0.6, 8.0))
    d.run(20.0 if positioning == "flow" else 16.0, record_hz=PLAYBACK_HZ)
    log = d.log.arrays()
    t = log["t"] - start
    reference = log["setpoint"]
    if positioning == "flow":
        # Drift from the initial hover point, not from an unknowable global flow coordinate.
        reference = np.tile([*log["pos"][0, :2], 1.0], (len(t), 1))
    error = np.linalg.norm(log["pos"] - reference, axis=1)
    window = t > 2.0 if positioning == "mocap" else np.ones(len(t), dtype=bool)
    worst = float(error[window].max())
    gate = 0.1 if positioning == "mocap" else 0.05
    ok = worst < gate and d.fw.assert_count == 0 and not d.fw.tumbled
    return {
        "label": label, "color": color,
        "note": f"{profile}; {d.rate} Hz flight loop; {positioning} positioning. Simulated, not hardware validated.",
        "status": "ok" if ok else "bad", "badge": "tracking check passed" if ok else "accuracy NOT met",
        "stats": f"max 3D error {100 * worst:.1f} cm (limit {100 * gate:g} cm) | asserts {d.fw.assert_count}",
        "lost": None,
        "t": r(t - t[0]), "pos": r(log["pos"]), "quat": r(log["quat"], 5), "motors": r(log["motors"]),
        "series": {"error": r(error * 100, 2), "z": r(log["pos"][:, 2]),
                   "x": r(log["pos"][:, 0]), "x_sp": r(reference[:, 0]),
                   "y": r(log["pos"][:, 1]), "y_sp": r(reference[:, 1]),
                   "z_sp": r(reference[:, 2])},
    }


def fly_vibration(label: str, color: str, notch: bool) -> dict:
    """Hover with the assumed motor vibration; the charts isolate the gyro artifact from true motion."""
    d = SitlDrone(seed=4, profile="whoop2_4k", vibration=VibrationSpec(), rpm_notch=notch)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    d.run(3.0, record_hz=d.rate)
    log = d.log.arrays()
    t = log["t"] - log["t"][0]
    artifact = log["gyro_raw"] - log["roll_rate"]
    notched = log["gyro_notch"] - log["roll_rate"]
    error = np.linalg.norm(log["pos"] - np.array([0.0, 0.0, 1.0]), axis=1)
    worst = float(error.max())
    return {
        "label": label, "color": color,
        "note": ("4 kHz SITL and gyro recording (dense trace; see spectrum report for frequencies), "
                 "assumed 12 deg/s per-motor vibration, additive gyro artifact. "
                 "Not a measured airframe; the notch does not change tracking in this model."),
        "status": "ok" if worst < 0.05 and not d.fw.tumbled else "bad",
        "badge": f"artifact {np.std(notched):.2f} deg/s",
        "stats": (f"gyro artifact RMS {np.std(artifact):.2f} deg/s -> {np.std(notched):.2f} deg/s | "
                  f"hover max {100 * worst:.2f} cm | asserts {d.fw.assert_count}"),
        "lost": None,
        "t": r(t, 6), "pos": r(log["pos"]), "quat": r(log["quat"], 5), "motors": r(log["motors"]),
        "series": {"artifact": r(artifact, 2), "notched": r(notched, 2), "error": r(error * 100, 2),
                   "m1": r(log["motors"][:, 0] * 100, 2)},
    }


def fly_failsafe(label: str, color: str, kind: str, *, moving: bool = False,
                 recover_after: float | None = None) -> dict:
    result = fly_loss("whoop2_4k", kind, moving=moving, recover_after=recover_after)
    log = result["log"]
    terminal = SAFETY_NAMES[int(log["safety"][-1])]
    drift = np.linalg.norm(log["pos"][:, :2] - log["pos"][0, :2], axis=1)
    return {
        "label": label, "color": color,
        "note": ("Independent local ToF; XY hold only with fresh camera tracking. "
                 "LAND is latched even if commands return. No collision/XY-drift guarantee after camera loss."),
        "status": "ok" if result["passed"] and kind != "height" else "bad",
        "badge": "Final: " + terminal + (" (not a controlled landing)" if kind == "height" else ""),
        "stats": (f"max XY excursion {result['max_drift']:.2f} m | "
                  + " -> ".join(f"{name}@{t:.2f}s" for t, name in result["sequence"])),
        "lost": None, "t": r(log["t"] - log["t"][0]),
        "pos": r(log["pos"]), "quat": r(log["quat"], 5), "motors": r(log["motors"]),
        "series": {"height": r(log["pos"][:, 2]), "vz": r(log["velocity"][:, 2]),
                   "safety": r(log["safety"], 0), "drift": r(drift)},
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
        fly_sitl("Reference settings", "Stock PID settings in our synchronous SITL loop (not the production scheduler)", COLORS[0], "stock"),
        fly_sitl("4 kHz + feedforward", "G1 gains, velocity/acceleration feedforward and landed anti-windup", COLORS[2], "whoop2_4k"),
    ]
    tracking = [fly_tracking("Reference settings", COLORS[0], "stock"),
                fly_tracking("4 kHz + feedforward", COLORS[2], "whoop2_4k")]
    flow = [fly_tracking("Flow + height only", COLORS[0], "whoop2_interim", "flow")]
    vibration = [fly_vibration("Notch off", COLORS[0], False), fly_vibration("RPM notch on", COLORS[2], True)]
    failsafe = [fly_failsafe("Command loss, hover", COLORS[2], "command"),
                fly_failsafe("Both links lost, moving", COLORS[1], "both", moving=True),
                fly_failsafe("Radio returns during LAND", COLORS[3], "command", moving=True, recover_after=2.5)]

    maneuver = "Roll doublet (+15 deg, -15 deg, level) with altitude hold, then a gust (roll torque kick) at t = 2.0 s."
    return [
        {"id": "failsafe", "title": "Latched link-loss landing (opt-in C SITL)",
         "description": ("Input loss at t=0. Command-only loss captures a fixed pose after 500 ms, holds 1.5 s, "
                         "then descends. Camera loss starts level descent after 300 ms using local ToF, without "
                         "trusting stale XY. State 0=NORMAL, 1=HOLD, 2=LAND, 3=LANDED, 4=FAULT. "
                         "Renewed commands cannot cancel LAND. Drift after tracking loss remains unbounded; "
                         "not collision-safe swarm flight or hardware validation."),
         "duration": failsafe[0]["t"][-1], "spacing": 2.4, "camera": [1.6, 2.5, 6.8], "target_y": 0.6,
         "charts": [{"title": "True height (m)", "series": "height"},
                    {"title": "True vertical speed (m/s)", "series": "vz"},
                    {"title": "Safety state (0 normal / 1 hold / 2 land / 3 landed / 4 fault)", "series": "safety"},
                    {"title": "XY excursion since loss (m) - not bounded after camera loss", "series": "drift"}],
         "drones": failsafe},
        {"id": "figure8", "title": "Task 1.6: figure-8 tracking (C SITL)",
         "description": ("Two laps of a 1.2 m-wide figure-8 with an 8 s period, at 1 m altitude. "
                         "Both drones use 100 Hz external camera position measurements. Dashed curves are targets. "
                         "The 10 cm check excludes the first 2 s of entry. The 4 kHz runtime is tested on this computer, "
                         "not yet activated or timing-validated on the P4."),
         "duration": tracking[0]["t"][-1], "spacing": 1.6, "camera": [0.6, 2.0, 4.8], "target_y": 0.85,
         "charts": [{"title": "3D tracking error (cm)", "series": "error"},
                    {"title": "x (m) - dashed: target", "series": "x", "sp": "x_sp"},
                    {"title": "y (m) - dashed: target", "series": "y", "sp": "y_sp"}],
         "drones": tracking},
        {"id": "flow", "title": "Open issue FLOW-01: flow-only drift",
         "description": ("No external camera position measurements. Optical flow senses velocity, not absolute XY position. "
                         "Height stays near 1 m, but XY noise integrates into drift. This does NOT pass the 5 cm "
                         "positioning requirement. Sensor noise is assumed; hardware measurements are still required."),
         "duration": flow[0]["t"][-1], "camera": [1.0, 2.0, 3.2], "target_y": 0.85,
         "charts": [{"title": "Position drift (cm)", "series": "error"},
                    {"title": "Height (m) - dashed: target", "series": "z", "sp": "z_sp"},
                    {"title": "x (m) - dashed: initial hover position", "series": "x", "sp": "x_sp"}],
         "drones": flow},
        {"id": "vibration", "title": "Task 1.6b: motor vibration + RPM notch",
         "description": ("Both drones hover with the same seeded vibration disturbance, modeled as an additive gyro "
                         "artifact (not body motion). Left: notch off. Right: four motors x 1x/2x mechanical rotation, "
                         "Q=5, 100 Hz RPM telemetry with 10 ms latency. The notch removes the sensor artifact; the "
                         "150 Hz gyro low-pass filter already keeps it out of the controller, so tracking is unchanged. "
                         "Amplitudes are assumptions pending blackbox measurements."),
         "duration": round(vibration[0]["t"][-1], 2), "spacing": 1.1, "camera": [0.6, 1.5, 2.4], "target_y": 0.9,
         "charts": [{"title": "Roll-axis gyro artifact (deg/s)", "series": "artifact"},
                    {"title": "After RPM notch (deg/s)", "series": "notched"},
                    {"title": "Hover position error (cm)", "series": "error"}],
         "drones": vibration},
        {"id": "sitl", "title": "G2 SITL: real firmware, takeoff + square",
         "description": ("The actual C flight firmware (fc_step: calibration, Kalman filter with 100 Hz mocap, position "
                         "controller, cascaded PID, mixer) flying the 6-DOF model with ICM-42688-P noise. Calibrates on "
                         "the ground, arms, takes off along a 2 s minimum-jerk path, flies a 0.5 m square at 1 m, hovers. "
                          "Left: reference settings in the synchronous loop; right: the 4 kHz board profile. "
                          "Not a hardware flight validation."),
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
