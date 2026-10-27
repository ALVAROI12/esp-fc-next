"""One simulated drone: 6-DOF physics + sensor models + the real flight firmware, in 1 ms lockstep."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from model.dynamics import Quadrotor, State, quat_to_euler
from model.params import DEFAULT_SENSORS, WHOOP_2IN, Sensors, Vehicle
from model.sensors import ImuModel, MocapModel

from .firmware import FlightFirmware

RATIO_FULL_SCALE = 65535.0
MOCAP_STD_REPORTED = 0.01  # m, std passed to the estimator (Crazyswarm extPosStdDev default)
SETPOINT_RATE_HZ = 100  # ground-station setpoint rate


@dataclass
class Segment:
    """Minimum-jerk move from p0 to p1 (x, y, z, yaw) starting at t0, like Crazyswarm goTo/takeoff."""

    t0: float
    duration: float
    p0: np.ndarray
    p1: np.ndarray

    def at(self, t: float) -> np.ndarray:
        if self.duration <= 0.0:
            return self.p1
        s = min(1.0, max(0.0, (t - self.t0) / self.duration))
        blend = s ** 3 * (10 - 15 * s + 6 * s * s)
        return self.p0 + (self.p1 - self.p0) * blend


@dataclass
class Log:
    t: list = field(default_factory=list)
    pos: list = field(default_factory=list)  # truth
    quat: list = field(default_factory=list)  # truth (w, x, y, z)
    euler: list = field(default_factory=list)  # truth roll, pitch, yaw (deg)
    est: list = field(default_factory=list)  # firmware estimate x y z roll pitch yaw
    setpoint: list = field(default_factory=list)  # x y z
    motors: list = field(default_factory=list)  # normalized 0..1

    def arrays(self) -> dict[str, np.ndarray]:
        return {k: np.asarray(v) for k, v in self.__dict__.items()}


class SitlDrone:
    def __init__(
        self,
        vehicle: Vehicle = WHOOP_2IN,
        sensors: Sensors = DEFAULT_SENSORS,
        start: tuple[float, float, float] = (0.0, 0.0, 0.0),
        seed: int = 0,
        mocap: bool = True,
        substeps: int = 2,
        thrust_base: float | None = None,
    ):
        self.vehicle = vehicle
        self.rng = np.random.default_rng(seed)
        self.dt = 1.0 / FlightFirmware.RATE_HZ
        self.substeps = substeps
        self.quad = Quadrotor(vehicle, State(pos=np.array(start, dtype=float)), ground=True)
        self.imu = ImuModel(sensors.imu, FlightFirmware.RATE_HZ, self.rng)
        self.mocap_every = round(FlightFirmware.RATE_HZ / sensors.mocap.rate_hz)
        # MocapModel counts latency in calls, and it is sampled every mocap_every ticks.
        self.mocap = MocapModel(sensors.mocap, self.rng, self.mocap_every * self.dt) if mocap else None
        self.fw = FlightFirmware()
        # Board-profile value: the position controller's hover throttle for this airframe.
        self.fw.set_param("posCtlPid.thrustBase",
                          thrust_base if thrust_base is not None else vehicle.hover_command * RATIO_FULL_SCALE)
        self.command = np.zeros(4)
        self.tick = 0
        self.segment: Segment | None = None
        self.target: tuple[float, float, float, float] | None = None  # current x, y, z, yaw setpoint
        self.link_up = True
        self.log = Log()

    @property
    def time(self) -> float:
        return self.tick * self.dt

    def goto(self, x: float, y: float, z: float, yaw_deg: float = 0.0, duration: float = 0.0) -> None:
        """Fly to a point along a minimum-jerk path (duration 0 = step setpoint)."""
        start = np.array(self.target) if self.target else np.array([*self.quad.state.pos, 0.0])
        self.segment = Segment(self.time, duration, start, np.array([x, y, z, yaw_deg], dtype=float))

    def takeoff(self, height: float, duration: float = 2.0) -> None:
        p = self.quad.state.pos
        self.goto(p[0], p[1], height, 0.0, duration)

    @property
    def move_done(self) -> bool:
        return self.segment is None or self.time >= self.segment.t0 + self.segment.duration

    def step(self, record_every: int = 1) -> None:
        s = self.quad.state
        gyro, acc = self.imu.sample(s.omega, self.quad.specific_force_body(self.command))

        if self.mocap and self.tick % self.mocap_every == 0:
            p = self.mocap.sample(s.pos)
            self.fw.push_position(*p, MOCAP_STD_REPORTED)
        if self.segment and self.tick % (FlightFirmware.RATE_HZ // SETPOINT_RATE_HZ) == 0:
            self.target = tuple(self.segment.at(self.time))
            if self.link_up:
                self.fw.set_position_setpoint(*self.target)

        ratios = self.fw.step(gyro, acc, int(self.tick * 1e6 * self.dt))
        self.command = np.array(ratios) / RATIO_FULL_SCALE
        for _ in range(self.substeps):
            self.quad.step(self.command, self.dt / self.substeps)
        self.tick += 1

        if self.tick % record_every == 0:
            st = self.quad.state
            est = self.fw.state()
            self.log.t.append(self.time)
            self.log.pos.append(st.pos.copy())
            self.log.quat.append(st.quat.copy())
            self.log.euler.append([math.degrees(a) for a in quat_to_euler(st.quat)])
            self.log.est.append(est[0:3] + est[6:9])
            self.log.setpoint.append(list(self.target[:3]) if self.target else [math.nan] * 3)
            self.log.motors.append(self.command.copy())

    def run(self, seconds: float, record_every: int = 1) -> None:
        for _ in range(int(round(seconds / self.dt))):
            self.step(record_every)

    def calibrate_and_arm(self, timeout: float = 3.0) -> None:
        """Sit still until the firmware finishes gyro/accel calibration, then arm."""
        while not self.fw.ready:
            if self.time > timeout:
                raise RuntimeError("firmware did not finish calibration")
            self.step()
        self.fw.arm(True)
