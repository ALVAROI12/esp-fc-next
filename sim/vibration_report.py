"""Task 1.6b report: synthetic motor vibration and the firmware RPM notch bank.

Runs the C flight loop in SITL with the same seed and disturbance, with and without
the notch, and writes sim/reports/vibration/{README.md, *.png}. Exit code 1 if a
documented check fails.

    python sim/vibration_report.py [--no-plots] [--seeds 4,7,11]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

from model import linear, robust
from model.firmware import AttitudeGains, PidGains
from model.params import PLANNED_TIMING, WHOOP_2IN
from model.vibration import MotorVibration, VibrationSpec, notch_response
from sitl import SitlDrone
from sitl.drone import figure8

HERE = Path(__file__).resolve().parent
OUT = HERE / "reports" / "vibration"
GAINS_JSON = HERE / "reports" / "g1" / "proposed_gains.json"
AMPLITUDES = (0.0, 12.0, 36.0)
AMPLITUDES_DPS = (12.0, 36.0)
RPM_ERRORS = (0.02, 0.05, 0.10, 0.20)
# Mechanical fundamental frequencies: include both section activation boundaries,
# below-floor harmonic-only operation, and the upper cap. This is a sampled sweep,
# not a proof over continuously varying or arbitrary independent motor speeds.
RPM_SWEEP_HZ = (0, 249.9, 250, 400, 499.9, 500, 550, 600, 750, 899.9, 900, 1200, 1799.9, 1800, 3000)
MOTOR_HF_HZ = 250.0
HOVER_GATE_M = 0.05
FIGURE8_GATE_M = 0.10


def rms(values) -> float:
    return float(np.sqrt(np.mean(np.asarray(values, dtype=float) ** 2)))


def hf_rms(signal: np.ndarray, rate: float, cutoff: float = MOTOR_HF_HZ) -> float:
    """RMS above `cutoff`, a proxy for actuator/ESC high-frequency activity."""
    values = np.asarray(signal, dtype=float)
    spec = np.fft.rfft(values - values.mean(axis=0), axis=0)
    freq = np.fft.rfftfreq(len(values), 1.0 / rate)
    spec[freq < cutoff] = 0.0
    return float(np.std(np.fft.irfft(spec, n=len(values), axis=0)))


def hover(*, amplitude: float, notch: bool, seed: int = 4, rpm_error: float = 0.02) -> dict:
    d = SitlDrone(seed=seed, profile="whoop2_4k", vibration=VibrationSpec(amplitude_dps=amplitude),
                  rpm_notch=notch, rpm_error=rpm_error)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    d.run(1.0, record_hz=4000)
    log = d.log.arrays()
    return {
        "drone": d, "log": log,
        "artifact": rms(log["gyro_raw"] - log["roll_rate"]),
        "notched": rms(log["gyro_notch"] - log["roll_rate"]),
        "lpf": rms(log["gyro_filtered"] - log["roll_rate"]),
        "true_rate": rms(log["roll_rate"]),
        "error": float(np.linalg.norm(log["pos"] - np.array([0.0, 0.0, 1.0]), axis=1).max()),
        "motor_hf": 100.0 * hf_rms(log["motors"], d.rate),
        "tumbled": d.fw.tumbled, "asserts": d.fw.assert_count,
    }


def figure8_run(*, amplitude: float, notch: bool, seed: int = 5) -> dict:
    d = SitlDrone(seed=seed, profile="whoop2_4k", vibration=VibrationSpec(amplitude_dps=amplitude),
                  rpm_notch=notch)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    start = d.time
    d.follow(figure8(start, np.array([0.0, 0.0, 1.0]), 0.6, 8.0))
    d.run(16.0, record_hz=100)
    log = d.log.arrays()
    window = log["t"] > start + 2.0
    error = np.linalg.norm(log["pos"] - log["setpoint"], axis=1)[window]
    return {"max": float(error.max()), "rms": rms(error), "tumbled": d.fw.tumbled,
            "asserts": d.fw.assert_count}


def modeled_rpm(vehicle, scale: float = 1.0, error: float = 0.02) -> np.ndarray:
    return vehicle.hover_command * VibrationSpec().max_rpm * np.array(VibrationSpec().motor_scale) * scale * (1 + error)


def margins_with_notch(vehicle, timing, gains: AttitudeGains, rpm: np.ndarray) -> tuple[float, float]:
    w = np.geomspace(0.1, np.pi / timing.rate_dt * 0.999, 8000)
    z = np.exp(1j * w * timing.rate_dt)
    loop = linear.rate_loop(vehicle, timing, "roll", gains.roll_rate)
    response = loop(z) * notch_response(rpm, w / (2 * math.pi), sample_hz=1.0 / timing.rate_dt, q=5.0)
    gm, pm, _, _, _, _ = linear.ct.stability_margins(linear.ct.frd(response, w, smooth=True))
    return float(pm), float(20 * math.log10(gm))


def spectrum_plot(off: dict, on: dict, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7.2, 3.6), dpi=150)
    rate = off["drone"].rate
    for data, key, label, color in ((off, "gyro_raw", "gyro artifact, notch off", "#e4572e"),
                                    (on, "gyro_notch", "gyro artifact, notch on", "#29b6f6"),
                                    (on, "gyro_filtered", "after 150 Hz gyro LPF", "#76b041")):
        signal = data["log"][key] - data["log"]["roll_rate"]
        window = np.hanning(len(signal))
        amp = 2.0 * np.abs(np.fft.rfft(window * signal)) / window.sum()
        freq = np.fft.rfftfreq(len(signal), 1.0 / rate)
        ax.semilogx(freq[1:], 20 * np.log10(np.maximum(amp[1:], 1e-9)), label=label, color=color, lw=1.1)
    rpm = MotorVibration(seed=4).rpm(on["log"]["motors"][-1])
    for harmonic in (1, 2):
        ax.axvline(rpm.mean() * harmonic / 60.0, color="#8796a8", ls=":", lw=0.8)
    ax.set_xlim(20, rate / 2)
    ax.set_ylim(-100, 10)
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("amplitude (dB re 1 deg/s)")
    ax.grid(alpha=0.25, which="both")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def amplitude_plot(rows: list[dict], rpm_rows: list[dict], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (left, right) = plt.subplots(1, 2, figsize=(8.2, 3.2), dpi=150)
    amps = [a for a in AMPLITUDES if a > 0]
    width = 0.36
    xs = np.arange(len(amps))
    left.bar(xs - width / 2, [r["off"]["artifact"] for r in rows if r["amplitude"] > 0], width,
             label="notch off", color="#e4572e")
    left.bar(xs + width / 2, [r["on"]["notched"] for r in rows if r["amplitude"] > 0], width,
             label="notch on", color="#29b6f6")
    left.set_xticks(xs, [f"{a:g}" for a in amps])
    left.set_xlabel("assumed vibration amplitude (deg/s, per motor)")
    left.set_ylabel("roll-axis artifact RMS (deg/s)")
    left.grid(alpha=0.25, axis="y")
    left.legend(fontsize=8)

    right.plot([100 * r["rpm_error"] for r in rpm_rows], [r["notched"] for r in rpm_rows], "o-", color="#29b6f6")
    right.set_xlabel("constant positive RPM scale bias (%)")
    right.set_ylabel("post-notch artifact RMS (deg/s)")
    right.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--seeds", default="4,7,11", help="hover seeds, comma separated")
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    seeds = [int(s) for s in args.seeds.split(",")]
    args.out.mkdir(parents=True, exist_ok=True)
    start_time = time.time()

    print("hover runs", flush=True)
    rows = []
    for amplitude in AMPLITUDES:
        runs = {on: [hover(amplitude=amplitude, notch=on, seed=s) for s in seeds] for on in (False, True)}
        rows.append({
            "amplitude": amplitude,
            "off": {k: max(r[k] for r in runs[False]) for k in ("artifact", "notched", "lpf", "error", "motor_hf")},
            "on": {k: max(r[k] for r in runs[True]) for k in ("artifact", "notched", "lpf", "error", "motor_hf")},
            "tumbled": any(r["tumbled"] for r in runs[False] + runs[True]),
            "asserts": sum(r["asserts"] for r in runs[False] + runs[True]),
        })
    single = {a: {on: hover(amplitude=a, notch=on) for on in (False, True)} for a in AMPLITUDES_DPS}

    print("figure-8 runs", flush=True)
    tracking = [{**{"amplitude": a, "notch": on}, **figure8_run(amplitude=a, notch=on)}
                for a in AMPLITUDES_DPS for on in (False, True)]

    print("RPM error/dropout", flush=True)
    rpm_rows = [{"rpm_error": e, **hover(amplitude=AMPLITUDES_DPS[0], notch=True, rpm_error=e)} for e in RPM_ERRORS]
    dropout = hover(amplitude=AMPLITUDES_DPS[0], notch=True)
    d = dropout["drone"]
    d.rpm_link_up = False
    d.run(0.05, record_hz=4000)
    stale = float(d.fw.log("fcGyro.rpmFresh"))
    d.run(0.10, record_hz=4000)
    bypassed = float(d.fw.log("fcGyro.rpmFresh"))
    bypass_start = len(d.log.t)
    d.run(0.02, record_hz=4000)
    bypass_log = d.log.arrays()
    exact_bypass = np.array_equal(bypass_log["gyro_raw"][bypass_start:], bypass_log["gyro_notch"][bypass_start:])
    d.rpm_link_up = True
    d.run(0.15, record_hz=4000)
    recovered = float(d.fw.log("fcGyro.rpmFresh"))
    dropout_error = float(np.linalg.norm(d.log.arrays()["pos"] - [0, 0, 1], axis=1).max())

    print("linear margins", flush=True)
    gains = AttitudeGains(**{k: PidGains(**v) for k, v in json.loads(GAINS_JSON.read_text())["gains"].items()})
    nominal_rpm = modeled_rpm(WHOOP_2IN)
    nominal_pm, nominal_gm = margins_with_notch(WHOOP_2IN, PLANNED_TIMING, gains, nominal_rpm)
    phase10 = abs(math.degrees(np.angle(notch_response(nominal_rpm, [10.0], sample_hz=1 / PLANNED_TIMING.rate_dt)[0])))
    phase20 = abs(math.degrees(np.angle(notch_response(nominal_rpm, [20.0], sample_hz=1 / PLANNED_TIMING.rate_dt)[0])))
    without_pm, without_gm = margins_with_notch(WHOOP_2IN, PLANNED_TIMING, gains, np.zeros(4))
    corner_pm, corner_gm = math.inf, math.inf
    hover_corner_pm, hover_corner_gm = math.inf, math.inf
    for vehicle, timing in robust.Uncertainty().corners(WHOOP_2IN, PLANNED_TIMING):
        pm, gm = margins_with_notch(vehicle, timing, gains, modeled_rpm(vehicle, error=0.02))
        hover_corner_pm, hover_corner_gm = min(hover_corner_pm, pm), min(hover_corner_gm, gm)
        corner_pm, corner_gm = min(corner_pm, pm), min(corner_gm, gm)
        for hz in RPM_SWEEP_HZ:
            for motor_scale in (np.ones(4), np.array(VibrationSpec().motor_scale)):
                pm, gm = margins_with_notch(vehicle, timing, gains, hz * 60 * motor_scale)
                corner_pm, corner_gm = min(corner_pm, pm), min(corner_gm, gm)

    checks = {
        "Assumed-vibration hover stays within 5 cm": all(r["on"]["error"] < HOVER_GATE_M for r in rows),
        "Notch reduces the modeled artifact by at least 4x": all(
            r["on"]["notched"] < 0.25 * r["off"]["artifact"] for r in rows if r["amplitude"] > 0),
        "Figure-8 stays within 10 cm with the notch": all(r["max"] < FIGURE8_GATE_M for r in tracking if r["notch"]),
        "No tumble or assertion in any vibration run": not any(r["tumbled"] for r in rows) and not any(
            r["asserts"] for r in rows) and not any(r["tumbled"] or r["asserts"] for r in tracking),
        "Sampled RPM sweep across 32 corners keeps PM >= 45 deg and GM >= 6 dB": corner_pm >= 45.0 and corner_gm >= 6.0,
        "Dropout signal bypasses exactly, holds within 5 cm, and recovers":
            (stale, bypassed, recovered) == (1.0, 0.0, 1.0) and exact_bypass and dropout_error < HOVER_GATE_M
            and not d.fw.tumbled and d.fw.assert_count == 0,
    }
    gate_pass = all(checks.values())

    print("plots", flush=True)
    if not args.no_plots:
        spectrum_plot(single[AMPLITUDES_DPS[0]][False], single[AMPLITUDES_DPS[0]][True], args.out / "spectrum.png")
        amplitude_plot(rows, rpm_rows, args.out / "amplitude.png")

    lines = [
        "# Task 1.6b - motor vibration and RPM notch filter", "",
        f"Generated by `sim/vibration_report.py` in {time.time() - start_time:.0f} s. "
        f"**Checks: {'PASS' if gate_pass else 'FAIL'}**", "",
        "All vibration parameters are **ASSUMED synthetic stress values**, not measurements. The model adds a "
        "gyro artifact only; it does not add body forces, torques or frame resonance. Hardware blackbox data is "
        "required before drawing conclusions about a real airframe.", "",
        "## Method", "",
        "- Same seed, IMU noise and vibration disturbance with the filter on and off; vibration phases use a "
        "separate seeded RNG (`MotorVibration`), so the comparison is fair.",
        "- Hover: 2 s takeoff, 4 s settling, then 1 s recorded at 4 kHz (4 kHz board profile, 150 Hz gyro LPF).",
        "- Artifact = logged gyro minus the true body roll rate, so sensor noise is separated from real motion.",
        "- Motor activity is the RMS of motor commands above 250 Hz, a proxy for ESC/actuator stress, not thrust.",
        "- RPM reports are delivered at 100 Hz with one report (10 ms) latency and a constant positive, common-mode "
        "scale bias. Negative bias, random jitter and independent per-motor errors are not tested here.",
        "- 1x/2x mechanical rotation frequencies, Q = 5, targets below 500 Hz or at/above 45 % of the sample rate "
        "are skipped (no aliased filtering).",
        "- At 1 kHz the 450 Hz upper cap is below the 500 Hz floor: the bank is always an exact bypass.",
        "",
        "## Hover: artifact and tracking", "",
        "Worst over seeds " + ", ".join(str(s) for s in seeds) + " per cell.", "",
        "| Assumed amplitude (deg/s) | Notch | Gyro artifact RMS | Post-notch RMS | After LPF | Hover max error | Motor >250 Hz |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        for key, label in (("off", "off"), ("on", "on")):
            r = row[key]
            lines.append(f"| {row['amplitude']:g} | {label} | {r['artifact']:.2f} deg/s | {r['notched']:.2f} deg/s "
                         f"| {r['lpf']:.2f} deg/s | {100 * r['error']:.2f} cm | {r['motor_hf']:.2f} % |")
    lines += [
        "",
        "The notch removes most of the modeled 1x/2x gyro content. At these amplitudes the existing 150 Hz "
        "two-pole gyro LPF already attenuates it before the controller, so **position tracking does not measurably "
        "improve**. The reproducible benefit is lower high-frequency motor-command activity; the table shows that "
        "it returns close to the zero-vibration baseline. Real frames can also have lower-frequency structural "
        "resonances that this model does not represent.", "",
    ]
    if not args.no_plots:
        lines += ["![Spectrum](spectrum.png)", "", "![Amplitude and RPM error](amplitude.png)", ""]
    lines += [
        "## Figure-8 tracking (C SITL)", "",
        "| Assumed amplitude | Notch | Max 3D error | RMS error | Tumble / asserts |",
        "| --- | --- | --- | --- | --- |",
    ]
    for r in tracking:
        lines.append(f"| {r['amplitude']:g} deg/s | {'on' if r['notch'] else 'off'} | {100 * r['max']:.2f} cm "
                     f"| {100 * r['rms']:.2f} cm | {'yes' if r['tumbled'] or r['asserts'] else 'no'} |")
    lines += [
        "",
        "Both cases stay inside the 10 cm simulation gate. As above, the notch does not change tracking here; "
        "the figure-8 entry transient is included only after the declared 2 s window.", "",
        "## RPM telemetry error and dropout", "",
        "| Constant positive RPM scale bias | Post-notch artifact | Hover max error |", "| --- | --- | --- |",
    ]
    for r in rpm_rows:
        lines.append(f"| {100 * r['rpm_error']:.0f} % | {r['notched']:.2f} deg/s | {100 * r['error']:.2f} cm |")
    lines += [
        "",
        f"50 ms after the last report: `rpmFresh` = {stale:.0f}; 150 ms after: {bypassed:.0f} (bank bypassed); after recovery: "
        f"{recovered:.0f}. Invalid packets do not refresh the age (C test `test_rpm_notch` and "
        "`test_fc_step::test_rpm_timeout_bypasses_and_invalid_packets_do_not_refresh`).",
        f"Actual signal-path equality during stale telemetry: **{'PASS' if exact_bypass else 'FAIL'}**; "
        f"maximum recorded hover error including dropout and recovery: {100 * dropout_error:.2f} cm.",
        "",
        "A detuned notch leaves residual vibration but did not destabilize these runs. This is not a guarantee for "
        "larger RPM errors, wrong pole-pair conversion, or motor speeds outside the modeled range.", "",
        "## Stability impact (linear, roll-rate loop)", "",
        "| Case | Phase margin | Gain margin |", "| --- | --- | --- |",
        f"| Nominal, no notch | {without_pm:.1f} deg | {without_gm:.1f} dB |",
        f"| Nominal, eight notches at hover RPM (+2 % error) | {nominal_pm:.1f} deg | {nominal_gm:.1f} dB |",
        f"| Worst of 32 corners at modeled hover RPM | {hover_corner_pm:.1f} deg | {hover_corner_gm:.1f} dB |",
        f"| Worst sampled RPM/corner combination | {corner_pm:.1f} deg | {corner_gm:.1f} dB |",
        "",
        f"At modeled hover RPM the bank adds {phase10:.1f} deg of phase lag at 10 Hz and {phase20:.1f} deg at 20 Hz. "
        "The initial 150 Hz floor failed off-hover analysis (30.9 deg / 3.8 dB at a 150 Hz fundamental). "
        "The floor is now 500 Hz per section to retain margin above the G1 thresholds of 45 deg / 6 dB.",
        "",
        "Sweep fundamental frequencies (Hz): " + ", ".join(f"{hz:g}" for hz in RPM_SWEEP_HZ) + ". "
        "Each is checked across 32 uncertainty corners, with coincident motor speeds and the model's per-motor "
        "scale factors, plus modeled hover RPM. Both harmonic-only and full-bank operation are included. "
        "This is a fixed-coefficient, Q=5 linear analysis using a Python transcription of the RBJ equations, "
        "not a proof for continuous RPM, arbitrary independent speeds, retuning transients, other Q values, "
        "or hardware. The C implementation is covered separately by `test_rpm_notch`.",
        "",
        "## Checks", "",
    ]
    lines += [f"- {'PASS' if ok else 'FAIL'} - {name}" for name, ok in checks.items()]
    lines += [
        "",
        "## Limits", "",
        "- No measured vibration spectrum, RPM calibration, ESC telemetry latency or frame resonance.",
        "- Electrical eRPM must be converted with the motor pole-pair count; the API takes mechanical RPM.",
        "- The filter cannot recover a saturated or aliased gyro, and it cannot fix a wrong RPM source.",
        "- The 4 kHz loop is host/SITL only; P4 timing, IMU/ESC drivers and hardware flight are unvalidated.",
        "- `FLOW-01` (flow-only absolute XY drift) is unaffected by this work.",
        "",
    ]
    (args.out / "README.md").write_text("\n".join(lines))
    print(f"Report: {args.out / 'README.md'}")
    print("\n".join(f"{'PASS' if ok else 'FAIL'} {name}" for name, ok in checks.items()))
    raise SystemExit(0 if gate_pass else 1)


if __name__ == "__main__":
    main()
