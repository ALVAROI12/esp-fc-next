"""Gym-style environment on top of SITL, for training high-level policies (RL) against the real firmware.

The policy never commands motors: each action is a bounded position-setpoint offset that the firmware's
position controller tracks (see the AI track in the project plan). API follows Gymnasium
(reset -> (obs, info); step -> (obs, reward, terminated, truncated, info)) without depending on it.
"""

from __future__ import annotations

import numpy as np

from model.params import WHOOP_2IN, Vehicle

from .drone import SitlDrone


class GotoEnv:
    """Reach a random goal within a box and hover there.

    Observation (12): estimated x y z, vx vy vz (finite differences of the estimate), roll pitch yaw (rad),
    goal - position (3). Action (3): setpoint offset from the current estimate, clipped to +/-max_step m.
    """

    observation_size = 12
    action_size = 3

    def __init__(self, vehicle: Vehicle = WHOOP_2IN, control_hz: float = 10.0, episode_seconds: float = 10.0,
                 max_step: float = 0.5, arena: float = 1.0, seed: int | None = None,
                 profile: str = "whoop2_interim"):
        self.vehicle = vehicle
        self.profile = profile
        self.control_hz = control_hz
        self.max_actions = int(episode_seconds * control_hz)
        self.max_step = max_step
        self.arena = arena
        self.rng = np.random.default_rng(seed)
        self.drone: SitlDrone | None = None
        self.goal = np.zeros(3)
        self.actions = 0
        self._last_est = np.zeros(3)

    def _obs(self) -> np.ndarray:
        s = np.array(self.drone.fw.state(), dtype=float)
        pos = s[0:3]
        vel = s[3:6]
        rpy = np.radians(s[6:9])
        return np.concatenate([pos, vel, rpy, self.goal - pos]).astype(np.float32)

    def reset(self, seed: int | None = None) -> tuple[np.ndarray, dict]:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.drone = SitlDrone(self.vehicle, seed=int(self.rng.integers(1 << 31)), profile=self.profile)
        self.ticks_per_action = int(round(self.drone.rate / self.control_hz))
        self.drone.calibrate_and_arm()
        self.drone.takeoff(1.0, 2.0)
        self.drone.run(3.0)
        a = self.arena
        self.goal = np.array([self.rng.uniform(-a, a), self.rng.uniform(-a, a), self.rng.uniform(0.5, 1.5)])
        self.actions = 0
        return self._obs(), {"goal": self.goal.copy()}

    def step(self, action) -> tuple[np.ndarray, float, bool, bool, dict]:
        offset = np.clip(np.asarray(action, dtype=float), -self.max_step, self.max_step)
        est = np.array(self.drone.fw.state()[0:3])
        target = est + offset
        target[2] = max(0.2, target[2])
        self.drone.goto(*target, 0.0, duration=0.0)
        for _ in range(self.ticks_per_action):
            self.drone.step(record=False)
        self.actions += 1

        obs = self._obs()
        truth = self.drone.quad.state
        dist = float(np.linalg.norm(self.goal - truth.pos))
        tilt = float(np.degrees(np.arccos(np.clip(1 - 2 * (truth.quat[1] ** 2 + truth.quat[2] ** 2), -1, 1))))
        crashed = truth.pos[2] < 0.05 or tilt > 60.0 or self.drone.fw.assert_count > 0
        reward = -dist - 0.01 * float(np.dot(offset, offset)) - (100.0 if crashed else 0.0)
        truncated = self.actions >= self.max_actions
        return obs, reward, crashed, truncated, {"distance": dist, "tilt_deg": tilt}
