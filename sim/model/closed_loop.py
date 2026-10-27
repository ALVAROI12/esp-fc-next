"""Nonlinear closed-loop simulation: Python port of the firmware cascaded PID on the 6-DOF model.

The PID and 2-pole LPF are line-for-line ports of pid.c / filter.c. Attitude comes from truth
(estimator effects are covered by SITL in Task 1.5); the rate loop sees the noisy, filtered gyro.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .dynamics import MOTOR_SPIN, MOTOR_XY, Quadrotor, State, quat_to_euler
from .firmware import AttitudeGains, PidGains
from .linear import RATIO_FULL_SCALE
from .params import Imu, LoopTiming, Vehicle
from .sensors import ImuModel


class Lpf2p:
    """filter.c lpf2p*."""

    def __init__(self, sample_hz: float, cutoff_hz: float):
        fr = sample_hz / cutoff_hz
        ohm = math.tan(math.pi / fr)
        c = 1.0 + 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm
        self.b0 = ohm * ohm / c
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (ohm * ohm - 1.0) / c
        self.a2 = (1.0 - 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm) / c
        self.d1 = 0.0
        self.d2 = 0.0

    def apply(self, x: float) -> float:
        d0 = x - self.d1 * self.a1 - self.d2 * self.a2
        if not math.isfinite(d0):
            d0 = x
        y = d0 * self.b0 + self.d1 * self.b1 + self.d2 * self.b2
        self.d2, self.d1 = self.d1, d0
        return y


class Pid:
    """pid.c pidUpdate (no D filter, as configured in attitude_pid_controller.c)."""

    def __init__(self, g: PidGains, dt: float):
        self.g, self.dt = g, dt
        self.integ = 0.0
        self.prev_error = 0.0

    def update(self, desired: float, measured: float) -> float:
        error = desired - measured
        out = self.g.kp * error + self.g.kd * (error - self.prev_error) / self.dt
        self.integ += error * self.dt
        if self.g.i_limit:
            self.integ = max(-self.g.i_limit, min(self.g.i_limit, self.integ))
        out += self.g.ki * self.integ
        self.prev_error = error
        return out


def _saturate_int16(x: float) -> float:
    return max(-32767.0, min(32767.0, x))


# Mixer signs consistent with dynamics.py: positive command -> positive body torque.
_ROLL_SIGN = np.sign(MOTOR_XY[:, 1])
_PITCH_SIGN = -np.sign(MOTOR_XY[:, 0])
_YAW_SIGN = MOTOR_SPIN


@dataclass
class Trace:
    t: np.ndarray
    euler_deg: np.ndarray  # (N, 3)
    rate_deg: np.ndarray  # (N, 3) true body rates
    command: np.ndarray  # (N, 4) normalized motor commands
    pos: np.ndarray  # (N, 3)
    quat: np.ndarray | None = None  # (N, 4) w, x, y, z
    roll_sp: np.ndarray | None = None  # (N,) deg
    lost_control_at: float | None = None  # time attitude exceeded 90 deg or went non-finite


class CascadedPidSim:
    def __init__(
        self, vehicle: Vehicle, timing: LoopTiming, gains: AttitudeGains,
        imu: Imu | None = None, seed: int = 0, substeps: int = 4,
    ):
        self.v, self.timing, self.gains = vehicle, timing, gains
        self.quad = Quadrotor(vehicle, ground=False)
        self.quad.state = State(pos=np.array([0.0, 0.0, 1.0]), rotor=np.full(4, vehicle.hover_command))
        self.rng = np.random.default_rng(seed)
        self.imu = ImuModel(imu, timing.sensor_rate_hz, self.rng) if imu else None
        if self.imu:
            self.imu.gyro_bias[:] = 0.0  # assume calibrated at start; bias walk still applies
            self.imu.accel_bias[:] = 0.0
        self.sensor_dt = 1.0 / timing.sensor_rate_hz
        self.substeps = substeps
        self.ticks_per_rate = round(timing.sensor_rate_hz / timing.attitude_rate_hz)
        dt = timing.rate_dt
        self.rate_pid = [Pid(gains.roll_rate, dt), Pid(gains.pitch_rate, dt), Pid(gains.yaw_rate, dt)]
        self.att_pid = [Pid(gains.roll, dt), Pid(gains.pitch, dt)]
        self.gyro_lpf = [Lpf2p(timing.sensor_rate_hz, timing.gyro_lpf_hz) for _ in range(3)]
        hover = vehicle.hover_command
        self.pending = deque([np.full(4, hover)] * (timing.compute_delay_samples * self.ticks_per_rate))
        self.command = np.full(4, hover)
        self.thrust_ratio = hover * RATIO_FULL_SCALE

    def _altitude_thrust_ratio(self, z_sp: float) -> float:
        """Simple PD altitude hold with tilt compensation (visualization aid, not the firmware's
        position controller): returns the per-motor ratio producing the desired total thrust."""
        s = self.quad.state
        accel = 25.0 * (z_sp - s.pos[2]) - 10.0 * s.vel[2]
        cos_tilt = max(0.5, float(1 - 2 * (s.quat[1] ** 2 + s.quat[2] ** 2)))
        thrust = self.v.mass * (9.81 + accel) / cos_tilt
        u = math.sqrt(max(0.0, thrust) / (4.0 * self.v.thrust_max))
        return min(1.0, u) * RATIO_FULL_SCALE

    def run(
        self, duration: float, roll_sp_deg=lambda t: 0.0, pitch_sp_deg=lambda t: 0.0, yaw_rate_sp=lambda t: 0.0,
        torque_disturbance=None, altitude_hold: float | None = None,
    ) -> Trace:
        """torque_disturbance(t) -> body torque (N m, 3-vector); altitude_hold: target height (m) or None."""
        n = int(duration / self.sensor_dt)
        log_t, log_e, log_w, log_u, log_p, log_q, log_sp = [], [], [], [], [], [], []
        lost_at = None
        gyro_f = np.zeros(3)
        inertia_inv = np.array([1 / self.v.ixx, 1 / self.v.iyy, 1 / self.v.izz])
        for tick in range(n):
            t = tick * self.sensor_dt
            s = self.quad.state
            if altitude_hold is not None and tick % self.ticks_per_rate == 0:
                self.thrust_ratio = self._altitude_thrust_ratio(altitude_hold)
            omega_deg = np.degrees(s.omega)
            if self.imu:
                gyro, _ = self.imu.sample(s.omega, np.zeros(3))
            else:
                gyro = omega_deg
            gyro_f = np.array([self.gyro_lpf[i].apply(gyro[i]) for i in range(3)])

            if tick % self.ticks_per_rate == 0:
                roll, pitch, _ = (math.degrees(a) for a in quat_to_euler(s.quat))
                rate_sp = [
                    self.att_pid[0].update(roll_sp_deg(t), roll),
                    self.att_pid[1].update(pitch_sp_deg(t), pitch),
                    yaw_rate_sp(t),
                ]
                out = [_saturate_int16(self.rate_pid[i].update(rate_sp[i], gyro_f[i])) for i in range(3)]
                ratio = self.thrust_ratio + _ROLL_SIGN * out[0] / 2 + _PITCH_SIGN * out[1] / 2 + _YAW_SIGN * out[2]
                self.pending.append(np.clip(ratio, 0.0, RATIO_FULL_SCALE) / RATIO_FULL_SCALE)
            if len(self.pending) > self.timing.compute_delay_samples * self.ticks_per_rate:
                self.command = self.pending.popleft()

            disturbance = None
            if torque_disturbance is not None:
                disturbance = np.concatenate([np.zeros(3), np.asarray(torque_disturbance(t)) * inertia_inv])
            for _ in range(self.substeps):
                self.quad.step(self.command, self.sensor_dt / self.substeps, disturbance)

            euler = [math.degrees(a) for a in quat_to_euler(self.quad.state.quat)]
            log_t.append(t)
            log_e.append(euler)
            log_w.append(np.degrees(self.quad.state.omega))
            log_u.append(self.command.copy())
            log_p.append(self.quad.state.pos.copy())
            log_q.append(self.quad.state.quat.copy())
            log_sp.append(roll_sp_deg(t))
            if not np.all(np.isfinite(self.quad.state.as_vector())) or max(abs(euler[0]), abs(euler[1])) > 90.0:
                lost_at = t
                break
        return Trace(np.array(log_t), np.array(log_e), np.array(log_w), np.array(log_u), np.array(log_p),
                     np.array(log_q), np.array(log_sp), lost_at)


def step_overshoot_pct(t: np.ndarray, y: np.ndarray, target: float, t_step: float) -> float:
    after = y[t >= t_step]
    return max(0.0, (float(np.max(after)) - target) / target * 100.0)
