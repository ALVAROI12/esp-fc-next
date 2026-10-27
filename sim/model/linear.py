"""Linearized loop models of the cascaded PID attitude controller around hover.

Units follow the firmware: rates in deg/s, angles in deg, rate-PID output in Crazyflie
"control units" (int16) which the quad-X mixer adds to motor ratios (0..65535).
Roll/pitch commands are split r/2 per motor; yaw is applied +/-y per motor.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import control as ct
import numpy as np

from .firmware import PidGains
from .params import LoopTiming, Vehicle

RATIO_FULL_SCALE = 65535.0
AXES = ("roll", "pitch", "yaw")


def axis_gain(vehicle: Vehicle, axis: str) -> float:
    """Angular acceleration (deg/s^2) per control unit at hover, before motor lag."""
    u_h = vehicle.hover_command
    # d(thrust)/d(ratio) per motor for thrust = thrust_max * (ratio / full)^2
    dthrust_dratio = 2.0 * vehicle.thrust_max * u_h / RATIO_FULL_SCALE
    if axis in ("roll", "pitch"):
        inertia = vehicle.ixx if axis == "roll" else vehicle.iyy
        torque_per_unit = 4.0 * (0.5 * dthrust_dratio) * vehicle.arm / math.sqrt(2.0)
    elif axis == "yaw":
        inertia = vehicle.izz
        torque_per_unit = 4.0 * dthrust_dratio * vehicle.torque_ratio
    else:
        raise ValueError(axis)
    return math.degrees(torque_per_unit / inertia)


def lpf2_continuous(cutoff_hz: float) -> ct.TransferFunction:
    wc = 2.0 * math.pi * cutoff_hz
    return ct.tf([wc * wc], [1.0, math.sqrt(2.0) * wc, wc * wc])


def rate_plant(vehicle: Vehicle, timing: LoopTiming, axis: str) -> ct.TransferFunction:
    """Discrete plant at the rate-loop period: control units -> filtered gyro rate (deg/s)."""
    k = axis_gain(vehicle, axis)
    motor = ct.tf([1.0], [vehicle.motor_tau, 1.0])
    integrator = ct.tf([k], [1.0, 0.0])
    plant_c = motor * integrator * lpf2_continuous(timing.gyro_lpf_hz)
    plant_d = ct.c2d(plant_c, timing.rate_dt, "zoh")
    delay = ct.tf([1.0], [1.0] + [0.0] * timing.compute_delay_samples, timing.rate_dt)
    return plant_d * delay


def pid_discrete(g: PidGains, dt: float) -> ct.TransferFunction:
    """pid.c: kp*e + kd*(e - e_prev)/dt + ki*sum(e*dt) (unfiltered derivative)."""
    # kp + kd (z-1)/(dt z) + ki dt z/(z-1), over common denominator dt z (z-1)
    num = np.polyadd(
        np.polyadd(g.kp * dt * np.array([1.0, -1.0, 0.0]), g.kd * np.array([1.0, -2.0, 1.0])),
        g.ki * dt * dt * np.array([1.0, 0.0, 0.0]),
    )
    den = dt * np.array([1.0, -1.0, 0.0])
    return ct.tf(num, den, dt)


@dataclass(frozen=True)
class Margins:
    gain_margin_db: float  # inf when the phase never crosses -180 deg
    phase_margin_deg: float
    crossover_hz: float
    sensitivity_peak: float  # max |1/(1+L)|; < 2 (6 dB) is a common robustness target
    stable: bool

    def passes(self, min_pm: float, min_gm_db: float) -> bool:
        return self.stable and self.phase_margin_deg >= min_pm and self.gain_margin_db >= min_gm_db


def closed_loop_stable(loop: ct.TransferFunction) -> bool:
    poles = ct.poles(ct.feedback(loop, 1))
    return bool(np.all(np.abs(poles) < 1.0 - 1e-9))


def margins(loop: ct.TransferFunction, points: int = 4000) -> Margins:
    """Margins from the discrete frequency response (vectorized; robust to low-frequency integrators).

    Phase margin is taken at the highest gain crossover; gain margin at the first -180 deg phase
    crossing above it (the gain-increase margin). Stability is checked from closed-loop poles.
    """
    nyquist = math.pi / loop.dt
    omega = np.logspace(-1, math.log10(nyquist) - 1e-4, points)
    num, den = loop.num[0][0], loop.den[0][0]
    z = np.exp(1j * omega * loop.dt)
    resp = np.polyval(num, z) / np.polyval(den, z)
    mag = np.abs(resp)
    phase = np.degrees(np.unwrap(np.angle(resp)))
    ms = float(np.max(np.abs(1.0 / (1.0 + resp))))
    stable = closed_loop_stable(loop)

    above = mag >= 1.0
    crossings = np.where(above[:-1] & ~above[1:])[0]
    if not len(crossings):
        return Margins(math.inf, math.inf, 0.0, ms, stable)
    i = crossings[-1]
    f = (math.log(mag[i]) - 0.0) / (math.log(mag[i]) - math.log(mag[i + 1]))
    phase_gc = phase[i] + f * (phase[i + 1] - phase[i])
    shift = -360.0 * math.floor((phase_gc + 360.0) / 360.0)  # bring phase at crossover into [-360, 0)
    pm = phase_gc + shift + 180.0
    wgc = omega[i] * (omega[i + 1] / omega[i]) ** f

    adj = phase[i + 1:] + shift
    below = np.where(adj <= -180.0)[0]
    gm_db = math.inf if not len(below) else -20.0 * math.log10(mag[i + 1 + below[0]])
    return Margins(gm_db, float(pm), float(wgc / (2 * math.pi)), ms, stable)


def rate_loop(vehicle: Vehicle, timing: LoopTiming, axis: str, gains: PidGains) -> ct.TransferFunction:
    return pid_discrete(gains, timing.rate_dt) * rate_plant(vehicle, timing, axis)


def attitude_loop(
    vehicle: Vehicle, timing: LoopTiming, axis: str, rate_gains: PidGains, angle_gains: PidGains
) -> ct.TransferFunction:
    """Outer loop: angle PID -> closed rate loop -> integration of rate to angle."""
    inner = rate_loop(vehicle, timing, axis, rate_gains)
    inner_closed = ct.feedback(inner, 1)
    integrate = ct.tf([timing.rate_dt, 0.0], [1.0, -1.0], timing.rate_dt)  # backward-Euler angle estimate
    return pid_discrete(angle_gains, timing.rate_dt) * ct.minreal(inner_closed, verbose=False) * integrate


@dataclass(frozen=True)
class StepMetrics:
    overshoot_pct: float
    rise_time_s: float  # 10% -> 90%
    settling_time_s: float  # within 2%
    final_value: float


def step_metrics(closed: ct.TransferFunction, duration: float = 3.0) -> StepMetrics:
    """Unit-step metrics measured against the reference (1.0), as integral action drives it there."""
    t = np.arange(0.0, duration, closed.dt)
    _, y = ct.step_response(closed, T=t)
    yn = np.squeeze(y)
    final = float(yn[-1])
    if not np.all(np.isfinite(yn)):
        return StepMetrics(math.inf, math.inf, math.inf, final)
    overshoot = max(0.0, float(np.max(yn) - 1.0) * 100.0)
    t10 = t[np.argmax(yn >= 0.1)]
    t90 = t[np.argmax(yn >= 0.9)]
    outside = np.where(np.abs(yn - 1.0) > 0.02)[0]
    settling = float(t[outside[-1] + 1]) if len(outside) and outside[-1] + 1 < len(t) else (0.0 if not len(outside) else math.inf)
    return StepMetrics(overshoot, float(t90 - t10), settling, final)


def gyro_noise_to_command_rms(
    vehicle: Vehicle, timing: LoopTiming, axis: str, gains: PidGains, gyro_noise_density: float
) -> float:
    """RMS rate-PID output (control units) caused by white gyro noise entering before the LPF."""
    loop = rate_loop(vehicle, timing, axis, gains)
    lpf = ct.c2d(lpf2_continuous(timing.gyro_lpf_hz), timing.rate_dt, "tustin")
    noise_to_u = ct.minreal(pid_discrete(gains, timing.rate_dt) * lpf * ct.feedback(1, loop), verbose=False)
    n = 4000
    _, h = ct.impulse_response(noise_to_u, T=np.arange(n) * timing.rate_dt)
    h = np.squeeze(h) * timing.rate_dt  # impulse_response scales discrete impulses by 1/dt
    sigma = gyro_noise_density * math.sqrt(timing.attitude_rate_hz / 2.0)
    return float(sigma * math.sqrt(np.sum(h * h)))
