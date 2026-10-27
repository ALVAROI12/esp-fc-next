"""Nonlinear 6-DOF quad-X dynamics.

Frames: world z up; body x forward, y left, z up (Crazyflie convention).
Motors (top view): M1 front-right, M2 back-right, M3 back-left, M4 front-left.
Each motor's normalized rotor speed s follows ds/dt = (u - s) / tau and produces
thrust T = thrust_max * s^2 along body +z and drag torque dir * torque_ratio * T.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .params import G, Vehicle

_H = 1.0 / math.sqrt(2.0)
MOTOR_XY = np.array([[_H, -_H], [-_H, -_H], [-_H, _H], [_H, _H]])  # unit arm directions
MOTOR_SPIN = np.array([-1.0, 1.0, -1.0, 1.0])  # +1: drag torque about +z


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Rotation matrix body -> world for quaternion (w, x, y, z)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """(roll, pitch, yaw) in radians, ZYX convention (pitch positive = nose down about +y)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


@dataclass
class State:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # world frame
    quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))  # body rad/s
    rotor: np.ndarray = field(default_factory=lambda: np.zeros(4))  # normalized speed 0..1

    def as_vector(self) -> np.ndarray:
        return np.concatenate([self.pos, self.vel, self.quat, self.omega, self.rotor])

    @staticmethod
    def from_vector(x: np.ndarray) -> "State":
        return State(x[0:3].copy(), x[3:6].copy(), x[6:10].copy(), x[10:13].copy(), x[13:17].copy())


class Quadrotor:
    def __init__(self, vehicle: Vehicle, state: State | None = None, ground: bool = True):
        self.v = vehicle
        self.state = state or State()
        self.ground = ground
        self.inertia = np.diag([vehicle.ixx, vehicle.iyy, vehicle.izz])
        self.inertia_inv = np.linalg.inv(self.inertia)

    def motor_thrusts(self, rotor: np.ndarray) -> np.ndarray:
        return self.v.thrust_max * np.square(rotor)

    def body_wrench(self, rotor: np.ndarray) -> tuple[float, np.ndarray]:
        """Total thrust (N) and body torque (N m) produced by the rotors."""
        thrust = self.motor_thrusts(rotor)
        arm_xy = MOTOR_XY * self.v.arm
        torque = np.array([
            np.sum(arm_xy[:, 1] * thrust),
            -np.sum(arm_xy[:, 0] * thrust),
            np.sum(MOTOR_SPIN * self.v.torque_ratio * thrust),
        ])
        return float(np.sum(thrust)), torque

    def derivative(self, x: np.ndarray, command: np.ndarray, disturbance: np.ndarray) -> np.ndarray:
        vel, quat, omega, rotor = x[3:6], x[6:10], x[10:13], x[13:17]
        thrust, torque = self.body_wrench(rotor)
        rot = quat_to_rot(quat)
        force = rot @ np.array([0.0, 0.0, thrust]) - np.array([0.0, 0.0, self.v.weight]) - self.v.drag_linear * vel
        acc = force / self.v.mass + disturbance[:3]
        omega_dot = self.inertia_inv @ (torque - np.cross(omega, self.inertia @ omega)) + disturbance[3:]
        quat_dot = 0.5 * quat_mul(quat, np.array([0.0, *omega]))
        rotor_dot = (np.clip(command, 0.0, 1.0) - rotor) / self.v.motor_tau
        return np.concatenate([vel, acc, quat_dot, omega_dot, rotor_dot])

    def step(self, command: np.ndarray, dt: float, disturbance: np.ndarray | None = None) -> State:
        """Advance by dt with RK4. command: normalized motor commands in [0, 1]."""
        d = np.zeros(6) if disturbance is None else disturbance
        x = self.state.as_vector()
        k1 = self.derivative(x, command, d)
        k2 = self.derivative(x + 0.5 * dt * k1, command, d)
        k3 = self.derivative(x + 0.5 * dt * k2, command, d)
        k4 = self.derivative(x + dt * k3, command, d)
        x = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        x[6:10] /= np.linalg.norm(x[6:10])
        if self.ground and x[2] < 0.0:
            x[2] = 0.0
            x[3:6] = np.maximum(x[3:6], 0.0) * np.array([0.0, 0.0, 1.0])
        self.state = State.from_vector(x)
        return self.state

    def specific_force_body(self, command: np.ndarray) -> np.ndarray:
        """What an ideal accelerometer measures (m/s^2, body frame)."""
        x = self.state.as_vector()
        acc_world = self.derivative(x, command, np.zeros(6))[3:6]
        rot = quat_to_rot(self.state.quat)
        return rot.T @ (acc_world + np.array([0.0, 0.0, G]))
