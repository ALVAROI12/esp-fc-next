"""Gate G1 (math model) report.

Analyzes the cascaded PID attitude controller on the 2-inch whoop model:
  1. current firmware gains at the stock ESP-Drone loop timing,
  2. stock vs planned (4 kHz) timing architecture,
  3. robustly tuned gains for the planned timing (+ interim gains for the stock timing),
  4. Monte Carlo over +/-20 % mass, thrust and inertia, motor lag and delay,
  5. nonlinear 6-DOF cross-check and sensor-noise impact.

Writes sim/reports/g1/{README.md, proposed_gains.json, *.png}; exit code 1 if the gate fails.

    python sim/g1_report.py [--samples 1000] [--no-plots]
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

from model import linear, robust
from model.closed_loop import CascadedPidSim, step_overshoot_pct
from model.firmware import AttitudeGains, load_gains
from model.params import DEFAULT_SENSORS, DEFAULT_TIMING, PLANNED_TIMING, WHOOP_2IN, LoopTiming, Vehicle

OUT = Path(__file__).resolve().parent / "reports" / "g1"
STEP_DEG = 10.0
STEP_AT = 0.2


def tune(vehicle: Vehicle, timing: LoopTiming, base: AttitudeGains, unc: robust.Uncertainty) -> AttitudeGains:
    rr, _ = robust.tune_rate("roll", vehicle, timing, unc, base.roll_rate,
                             np.geomspace(30, 400, 30), np.linspace(0, 6, 31), base.roll_rate.ki / base.roll_rate.kp)
    yr, _ = robust.tune_rate("yaw", vehicle, timing, unc, base.yaw_rate,
                             np.geomspace(30, 800, 30), np.array([0.0]), base.yaw_rate.ki / base.yaw_rate.kp)
    ra, _ = robust.tune_attitude("roll", vehicle, timing, unc, rr, base.roll,
                                 np.geomspace(2, 150, 45), base.roll.ki / base.roll.kp)
    ya, _ = robust.tune_attitude("yaw", vehicle, timing, unc, yr, base.yaw,
                                 np.geomspace(1, 80, 40), base.yaw.ki / base.yaw.kp)
    # Symmetric airframe: pitch uses the roll design.
    return AttitudeGains(roll_rate=rr, pitch_rate=rr, yaw_rate=yr, roll=ra, pitch=ra, yaw=ya)


def margin_table(results: dict[str, linear.Margins]) -> str:
    rows = ["| Loop | Phase margin | Gain margin | Crossover | Sensitivity peak | Gate (PM>=45, GM>=6 dB) |",
            "| --- | --- | --- | --- | --- | --- |"]
    for name, m in results.items():
        ok = m.passes(robust.GATE_PM_DEG, robust.GATE_GM_DB)
        rows.append(f"| {name} | {m.phase_margin_deg:.1f} deg | {m.gain_margin_db:.1f} dB | {m.crossover_hz:.2f} Hz "
                    f"| {m.sensitivity_peak:.2f} | {'pass' if ok else '**FAIL**'} |")
    return "\n".join(rows)


def gains_table(gains: dict[str, AttitudeGains]) -> str:
    names = list(gains)
    rows = ["| Loop | " + " | ".join(names) + " |", "| --- |" + " --- |" * len(names)]
    for loop in robust.LOOPS:
        cells = []
        for n in names:
            g = getattr(gains[n], loop)
            cells.append(f"kp {g.kp:.1f}, ki {g.ki:.1f}, kd {g.kd:.2f}")
        rows.append(f"| {loop} | " + " | ".join(cells) + " |")
    return "\n".join(rows)


def ms(seconds: float) -> str:
    return f"{seconds * 1000:.0f} ms" if math.isfinite(seconds) else "> 3 s (slow integrator tail)"


def step_compare(vehicle: Vehicle, timing: LoopTiming, gains: AttitudeGains, imu=None):
    closed = linear.ct.feedback(linear.attitude_loop(vehicle, timing, "roll", gains.roll_rate, gains.roll), 1)
    lin = linear.step_metrics(closed)
    sim = CascadedPidSim(vehicle, timing, gains, imu=imu, substeps=2 if timing.sensor_rate_hz > 1000 else 4)
    trace = sim.run(1.5, roll_sp_deg=lambda t: STEP_DEG if t >= STEP_AT else 0.0)
    nl_os = step_overshoot_pct(trace.t, trace.euler_deg[:, 0], STEP_DEG, STEP_AT)
    return lin, nl_os, trace, closed


def hover_noise(vehicle: Vehicle, timing: LoopTiming, gains: AttitudeGains, seconds: float = 3.0):
    sim = CascadedPidSim(vehicle, timing, gains, imu=DEFAULT_SENSORS.imu, seed=7, substeps=2)
    trace = sim.run(seconds)
    settle = trace.t > 0.5
    att_rms = float(np.sqrt(np.mean(trace.euler_deg[settle, :2] ** 2)))
    cmd_std = float(np.mean(np.std(trace.command[settle], axis=0)))
    return att_rms, cmd_std


def plots(out: Path, stock, planned, traces, mc: robust.MonteCarloResult):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for label, loop in (stock, planned):
        w = np.logspace(0, math.log10(math.pi / loop.dt) - 1e-3, 2000)
        r = loop(np.exp(1j * w * loop.dt))
        ax_m.semilogx(w / (2 * math.pi), 20 * np.log10(np.abs(r)), label=label)
        ax_p.semilogx(w / (2 * math.pi), np.degrees(np.unwrap(np.angle(r))), label=label)
    ax_m.axhline(0, color="k", lw=0.5)
    ax_p.axhline(-180, color="k", lw=0.5)
    ax_m.set_ylabel("|L| (dB)")
    ax_p.set_ylabel("phase (deg)")
    ax_p.set_xlabel("frequency (Hz)")
    ax_m.set_title("Roll rate loop, open-loop frequency response")
    ax_m.legend()
    ax_m.grid(True, which="both", alpha=0.3)
    ax_p.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "rate_loop_bode.png", dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for label, (closed, trace) in traces.items():
        t = np.arange(0, 1.5 - STEP_AT, closed.dt)
        _, y = linear.ct.step_response(closed, T=t)
        ax.plot(t + STEP_AT, STEP_DEG * np.squeeze(y), "--", label=f"{label} (linear)")
        ax.plot(trace.t, trace.euler_deg[:, 0], label=f"{label} (nonlinear 6-DOF)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("roll (deg)")
    ax.set_title(f"{STEP_DEG:.0f} deg roll step")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "roll_step.png", dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for loop in robust.LOOPS:
        ax.hist(mc.pm[loop][np.isfinite(mc.pm[loop])], bins=40, histtype="step", label=loop)
    ax.axvline(robust.GATE_PM_DEG, color="r", lw=1, label="gate 45 deg")
    ax.set_xlabel("phase margin (deg)")
    ax.set_ylabel("samples")
    ax.set_title(f"Monte Carlo phase margins, proposed gains, planned timing (N={mc.samples})")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "monte_carlo_pm.png", dpi=110)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--samples", type=int, default=1000, help="Monte Carlo samples")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    t_start = time.time()

    v = WHOOP_2IN
    unc = robust.Uncertainty()
    fw = load_gains()

    fw_stock = robust.evaluate(fw, v, DEFAULT_TIMING)
    fw_planned = robust.evaluate(fw, v, PLANNED_TIMING)
    proposed = tune(v, PLANNED_TIMING, fw, unc)
    interim = tune(v, DEFAULT_TIMING, fw, unc)
    prop_nominal = robust.evaluate(proposed, v, PLANNED_TIMING)
    interim_nominal = robust.evaluate(interim, v, DEFAULT_TIMING)
    mc = robust.monte_carlo(proposed, v, PLANNED_TIMING, unc, samples=args.samples)
    mc_interim = robust.monte_carlo(interim, v, DEFAULT_TIMING, unc, samples=max(100, args.samples // 5), seed=2)

    lin_fw, nl_fw, tr_fw, cl_fw = step_compare(v, DEFAULT_TIMING, fw)
    lin_p, nl_p, tr_p, cl_p = step_compare(v, PLANNED_TIMING, proposed)
    att_rms, cmd_std = hover_noise(v, PLANNED_TIMING, proposed)
    noise_u = linear.gyro_noise_to_command_rms(v, PLANNED_TIMING, "roll", proposed.roll_rate,
                                               DEFAULT_SENSORS.imu.gyro_noise_density)

    checks = {
        "Nominal margins, all six loops (PM>=45 deg, GM>=6 dB)":
            all(m.passes(robust.GATE_PM_DEG, robust.GATE_GM_DB) for m in prop_nominal.values()),
        f"Monte Carlo, {mc.samples} samples, zero gate failures": mc.passed,
        "10 deg roll step overshoot <= 15 % (nonlinear)": nl_p <= 15.0,
        "Linear and nonlinear overshoot agree within 5 points": abs(lin_p.overshoot_pct - nl_p) <= 5.0,
        "Hover with ICM-42688-P noise: attitude RMS < 0.5 deg": att_rms < 0.5,
        "Tuned gains are interior to their search grids": not robust.GRID_EDGE_HITS,
    }
    gate_pass = all(checks.values())

    (args.out / "proposed_gains.json").write_text(json.dumps({
        "airframe": "2-inch ducted 1S whoop (assumed parameters, see README)",
        "timing": asdict(PLANNED_TIMING),
        "gains": asdict(proposed),
        "interim_gains_for_stock_timing": {"timing": asdict(DEFAULT_TIMING), "gains": asdict(interim)},
    }, indent=2) + "\n")

    if not args.no_plots:
        plots(args.out,
              ("firmware gains, stock 500 Hz timing", linear.rate_loop(v, DEFAULT_TIMING, "roll", fw.roll_rate)),
              ("proposed gains, planned 4 kHz timing", linear.rate_loop(v, PLANNED_TIMING, "roll", proposed.roll_rate)),
              {"firmware @ 500 Hz": (cl_fw, tr_fw), "proposed @ 4 kHz": (cl_p, tr_p)}, mc)

    sources = "\n".join(f"| {k} | {s} |" for k, s in v.sources.items())
    worst = "\n".join(f"| {k} | {mc.worst_pm[k]:.1f} deg | {mc.worst_gm[k]:.1f} dB |" for k in robust.LOOPS)
    check_rows = "\n".join(f"| {name} | {'pass' if ok else '**FAIL**'} |" for name, ok in checks.items())
    report = f"""# Gate G1 - math model report

Generated by `sim/g1_report.py` in {time.time() - t_start:.0f} s. **Gate G1: {'PASS' if gate_pass else 'FAIL'}**
for the target design (proposed gains + planned 4 kHz timing).

## Gate checks

| Check | Result |
| --- | --- |
{check_rows}

## 1. Model inputs

Airframe: mass {v.mass * 1000:.0f} g, arm {v.arm * 1000:.0f} mm, inertia {v.ixx:.1e} / {v.iyy:.1e} / {v.izz:.1e} kg m^2,
max thrust {v.thrust_max:.2f} N per motor (thrust-to-weight {v.thrust_to_weight:.2f}, hover command {v.hover_command:.2f}),
motor time constant {v.motor_tau * 1000:.0f} ms. Gains are read from `pid.h`; loop structure follows
`controller_pid.c`, `attitude_pid_controller.c`, `pid.c` and the quad-X mixer.

| Parameter | Source |
| --- | --- |
{sources}
| IMU noise | {DEFAULT_SENSORS.imu.source} |

Uncertainty envelope: mass, max thrust and inertia +/-{unc.mass * 100:.0f} %, motor time constant
{unc.motor_tau[0] * 1000:.0f}-{unc.motor_tau[1] * 1000:.0f} ms, plus 0-1 extra sample of delay.

## 2. Current firmware gains at stock ESP-Drone timing (500 Hz, 80 Hz gyro LPF)

{margin_table(fw_stock)}

The stock Crazyflie rate gains leave the roll/pitch rate loops with too little margin on this airframe:
expect oscillation, and instability once the real motors or mass differ from the model.

## 3. Timing architecture: stock vs planned

The same firmware gains at the planned timing (4 kHz sampling and rate loop, 150 Hz gyro LPF):

{margin_table(fw_planned)}

Lower sample-to-actuation delay is worth far more than gain tuning: at equal worst-case margins the
planned timing roughly triples the achievable rate-loop bandwidth (see the tuned crossovers below).

## 4. Proposed gains (planned timing) and interim gains (stock timing)

Tuned to maximize worst-case crossover subject to worst-case PM >= {robust.DESIGN_PM_DEG:.0f} deg and
GM >= {robust.DESIGN_GM_DB:.0f} dB over all {len(unc.corners(v, DEFAULT_TIMING))} uncertainty corners
(stricter than the gate, to leave room for unmodeled effects). Integral/proportional ratios are kept from the firmware.

{gains_table({"firmware (pid.h)": fw, "proposed @ 4 kHz": proposed, "interim @ 500 Hz": interim})}

Proposed gains at the planned timing, nominal airframe:

{margin_table(prop_nominal)}

Interim gains at the stock timing (for bring-up before the 4 kHz loop exists), nominal airframe:

{margin_table(interim_nominal)}

## 5. Monte Carlo robustness (proposed gains, planned timing, N={mc.samples})

Failures: {mc.failures}. Worst case over all samples:

| Loop | Worst phase margin | Worst gain margin |
| --- | --- | --- |
{worst}

Interim gains at stock timing (N={mc_interim.samples}): {mc_interim.failures} failures.

![Monte Carlo phase margins](monte_carlo_pm.png)

## 6. Nonlinear 6-DOF cross-check ({STEP_DEG:.0f} deg roll step)

| Configuration | Linear overshoot | Nonlinear overshoot | Linear rise time | Linear 2 % settling |
| --- | --- | --- | --- | --- |
| Firmware gains @ 500 Hz | {lin_fw.overshoot_pct:.1f} % | {nl_fw:.1f} % | {ms(lin_fw.rise_time_s)} | {ms(lin_fw.settling_time_s)} |
| Proposed gains @ 4 kHz | {lin_p.overshoot_pct:.1f} % | {nl_p:.1f} % | {ms(lin_p.rise_time_s)} | {ms(lin_p.settling_time_s)} |

The nonlinear response rises slightly slower than the linear one because a {STEP_DEG:.0f} deg step demands a
large rate setpoint that saturates the int16 rate-PID output and the motors on the initial rise; the linear
model has no saturation. The slow tail of the firmware response comes from the outer PI integrator
(ki/kp = 0.5 rad/s zero).

![Roll step](roll_step.png)
![Rate loop Bode](rate_loop_bode.png)

## 7. Sensor noise

- Gyro white noise (ICM-42688-P, {DEFAULT_SENSORS.imu.gyro_noise_density} deg/s/sqrt(Hz)) through the proposed roll
  rate PID: {noise_u:.0f} control units RMS = {noise_u / 2 / linear.RATIO_FULL_SCALE * 100:.3f} % of motor full scale.
- Nonlinear hover with IMU noise and bias walk: attitude RMS {att_rms:.2e} deg, motor command std
  {cmd_std * 100:.3f} % of full scale (attitude from truth here; estimator noise is covered in SITL).
- Not modeled yet: frame vibration and motor/prop imbalance, which usually dominate gyro noise on real
  quads. These drive the dynamic-notch / RPM-filter work in Task 1.6 and must be measured in G6 blackbox logs.

## 8. Required firmware changes and open items

1. Implement the planned timing on the P4 (4 kHz IMU + rate loop, 150 Hz gyro LPF) - Task 1.6.
2. Add a board profile carrying the proposed gains (`proposed_gains.json`); use the interim gains until (1) lands.
3. Replace every `ASSUMED` parameter with measurements (scale, thrust stand, step test) and re-run this report.
"""
    (args.out / "README.md").write_text(report)
    print(report.split("## 1.")[0])
    print(f"Report: {args.out / 'README.md'}")
    return 0 if gate_pass else 1


if __name__ == "__main__":
    sys.exit(main())
