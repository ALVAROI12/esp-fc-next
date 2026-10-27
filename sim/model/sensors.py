"""Sensor noise models built from the datasheet parameters in params.py."""

from __future__ import annotations

import math

import numpy as np

from .params import G, Baro, Imu, Mocap, Tof


def white_noise_std(density: float, sample_rate_hz: float) -> float:
    """Per-sample std of white noise with one-sided density `density`/sqrt(Hz), band-limited at Nyquist."""
    return density * math.sqrt(sample_rate_hz / 2.0)


class ImuModel:
    """Gyro (deg/s) and accelerometer (g) with white noise, initial bias and gyro bias random walk."""

    def __init__(self, spec: Imu, sample_rate_hz: float, rng: np.random.Generator):
        self.spec = spec
        self.rate = sample_rate_hz
        self.rng = rng
        self.gyro_bias = rng.normal(0.0, spec.gyro_bias_initial, 3)
        self.accel_bias = rng.normal(0.0, spec.accel_bias_initial, 3)
        self.gyro_std = white_noise_std(spec.gyro_noise_density, sample_rate_hz)
        self.accel_std = white_noise_std(spec.accel_noise_density, sample_rate_hz)

    def sample(self, omega_body_rad: np.ndarray, specific_force_body: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.gyro_bias += self.rng.normal(0.0, self.spec.gyro_bias_walk / math.sqrt(self.rate), 3)
        gyro = np.degrees(omega_body_rad) + self.gyro_bias + self.rng.normal(0.0, self.gyro_std, 3)
        accel = specific_force_body / G + self.accel_bias + self.rng.normal(0.0, self.accel_std, 3)
        return gyro, accel


class BaroModel:
    """Altitude above sea level (m) from pressure with white noise, via the barometric formula slope."""

    PA_PER_M = 12.0  # dp/dh near sea level

    def __init__(self, spec: Baro, rng: np.random.Generator, ground_asl: float = 100.0):
        self.spec = spec
        self.rng = rng
        self.ground_asl = ground_asl

    @property
    def altitude_std(self) -> float:
        return self.spec.pressure_noise / self.PA_PER_M

    def sample(self, height: float) -> float:
        return self.ground_asl + height + self.rng.normal(0.0, self.altitude_std)


class TofModel:
    """Downward range (m) along the body -z axis; None beyond range or when inverted."""

    def __init__(self, spec: Tof, rng: np.random.Generator):
        self.spec = spec
        self.rng = rng

    def sample(self, height: float, rot_body_to_world: np.ndarray) -> float | None:
        cos_tilt = rot_body_to_world[2, 2]
        if cos_tilt <= 0.1:
            return None
        distance = height / cos_tilt
        if distance > self.spec.max_range:
            return None
        return max(0.0, distance + self.rng.normal(0.0, self.spec.std))


class MocapModel:
    """World position (m) with white noise and a fixed latency, delivered at `rate_hz`."""

    def __init__(self, spec: Mocap, rng: np.random.Generator, sim_dt: float):
        self.spec = spec
        self.rng = rng
        self.delay_steps = max(0, round(spec.latency / sim_dt))
        self.history: list[np.ndarray] = []

    def sample(self, pos: np.ndarray) -> np.ndarray:
        self.history.append(pos.copy())
        if len(self.history) > self.delay_steps + 1:
            self.history.pop(0)
        return self.history[0] + self.rng.normal(0.0, self.spec.std, 3)
