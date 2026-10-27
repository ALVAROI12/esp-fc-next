"""ASSUMED synthetic motor vibration: additive gyro artifact, not body torque.

All parameters, axis projections and harmonic offsets are stress-model choices,
not measurements or datasheet claims. Calibrate with hardware blackbox data.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def _finite_scalar(value) -> bool:
    try:
        return np.isscalar(value) and math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


@dataclass(frozen=True)
class VibrationSpec:
    """Synthetic mechanical RPM and per-motor peak fundamental in deg/s."""

    max_rpm: float = 60000.0  # assumed mechanical full-scale loaded RPM
    amplitude_dps: float = 12.0  # per motor at rotor=1, before axis projection
    harmonic_ratio: float = 0.35
    motor_scale: tuple = (0.97, 1.0, 1.02, 1.05)
    source: str = "ASSUMED synthetic stress model; must calibrate with hardware blackbox data"

    def __post_init__(self):
        for name in ("max_rpm", "amplitude_dps", "harmonic_ratio"):
            value = getattr(self, name)
            if not _finite_scalar(value):
                raise ValueError(f"{name} must be finite")
            if value < 0 or (name == "max_rpm" and value == 0):
                limit = "positive" if name == "max_rpm" else "nonnegative"
                raise ValueError(f"{name} must be {limit}")
        try:
            scale = np.asarray(self.motor_scale, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("motor_scale must contain four finite positive values") from exc
        if scale.shape != (4,) or not np.all(np.isfinite(scale)) or np.any(scale <= 0):
            raise ValueError("motor_scale must contain four finite positive values")


# Unit-length columns: each motor projects onto all three gyro axes. Neither
# this synthetic coupling nor the fixed second-harmonic offsets model a frame.
_PROJECTION = np.array([
    [1.0, -1.0, -1.0, 1.0],
    [-1.0, -1.0, 1.0, 1.0],
    [-1.0, 1.0, -1.0, 1.0],
]) / math.sqrt(3.0)
_HARMONIC_OFFSETS = np.arange(4) * (math.pi / 2.0)


def notch_response(rpm, frequencies_hz, sample_hz: float = 4000.0, q: float = 5.0) -> np.ndarray:
    """Complex response of the firmware's RBJ notch bank (fc_rpm_notch.c).

    Analysis helper for margins/phase, not a measurement. It mirrors the same
    coefficient and bypass rules in float64; the C implementation is checked by
    firmware/fc_core/tests/test_rpm_notch.c.
    """
    speeds = np.asarray(rpm, dtype=float)
    if speeds.shape != (4,) or not np.all(np.isfinite(speeds)) or np.any(speeds < 0):
        raise ValueError("rpm must have shape (4,) with finite nonnegative mechanical RPM")
    if not _finite_scalar(sample_hz) or sample_hz <= 0:
        raise ValueError("sample_hz must be finite and positive")
    if not _finite_scalar(q) or not 1.0 <= q <= 50.0:
        raise ValueError("q must be finite and within [1, 50]")
    frequencies = np.asarray(frequencies_hz, dtype=float)
    if not np.all(np.isfinite(frequencies)) or np.any(frequencies < 0):
        raise ValueError("frequencies must be finite and nonnegative")
    z_inv = np.exp(-2j * np.pi * frequencies / sample_hz)
    response = np.ones(frequencies.shape, dtype=complex)
    for speed in speeds:
        for harmonic in (1, 2):
            hz = speed * harmonic / 60.0
            if hz < 500.0 or hz >= 0.45 * sample_hz:
                continue
            w = 2.0 * math.pi * hz / sample_hz
            alpha = math.sin(w) / (2.0 * q)
            norm = 1.0 / (1.0 + alpha)
            b0 = b2 = norm
            b1 = -2.0 * math.cos(w) * norm
            a1 = b1
            a2 = (1.0 - alpha) * norm
            response *= (b0 + b1 * z_inv + b2 * z_inv**2) / (1.0 + a1 * z_inv + a2 * z_inv**2)
    return response


class MotorVibration:
    """Phase-continuous gyro artifact driven by actual, motor-lagged rotor speeds.

    No additional motor lag or sample-rate frequency clamp is applied: low-rate
    sampling can alias. The private RNG initializes phases without consuming the
    IMU's random stream; subsequent samples use no random numbers.
    """

    def __init__(self, spec: VibrationSpec = VibrationSpec(), seed: int = 0):
        self.spec = spec
        self._rpm_scale = spec.max_rpm * np.array(spec.motor_scale, dtype=float)
        self._phase = np.random.default_rng(seed).uniform(0.0, math.tau, 4)

    @staticmethod
    def _rotor(rotor) -> np.ndarray:
        try:
            values = np.asarray(rotor, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("rotor must contain four finite numeric speeds") from exc
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            raise ValueError("rotor must have shape (4,) with finite speeds")
        return np.clip(values, 0.0, 1.0)

    def rpm(self, rotor) -> np.ndarray:
        """Fresh mechanical RPM vector; querying it does not advance phases."""
        return self._rotor(rotor) * self._rpm_scale

    def sample(self, rotor, dt: float) -> np.ndarray:
        """Advance phases by dt seconds at these speeds; return gyro deg/s.

        Speed is held constant over this interval (right-endpoint integration
        when called after a physics step). dt=0 evaluates without advancing.
        """
        speeds = self._rotor(rotor)
        if not _finite_scalar(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        self._phase = (self._phase + math.tau * (speeds * self._rpm_scale / 60.0) * dt) % math.tau
        waveform = np.sin(self._phase) + self.spec.harmonic_ratio * np.sin(
            2.0 * self._phase + _HARMONIC_OFFSETS
        )
        return _PROJECTION @ (self.spec.amplitude_dps * speeds**2 * waveform)
