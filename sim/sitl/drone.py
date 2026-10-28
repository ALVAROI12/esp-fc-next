"""One simulated drone: 6-DOF physics + sensor models + the real flight firmware, in main-loop lockstep."""

from __future__ import annotations

from dataclasses import dataclass, field
from collections import deque
import math
from typing import Callable

import numpy as np

from model.dynamics import Quadrotor, State, quat_to_euler, quat_to_rot
from model.params import DEFAULT_SENSORS, WHOOP_2IN, Sensors, Vehicle
from model.sensors import FlowModel, ImuModel, MocapModel, TofModel
from model.vibration import MotorVibration, VibrationSpec

from .firmware import PROFILE_VARIANT, FlightFirmware

RATIO_FULL_SCALE = 65535.0
MOCAP_STD_REPORTED = 0.01  # m, std passed to the estimator (Crazyswarm extPosStdDev default)
TOF_STD_REPORTED = 0.01  # m
FLOW_STD_REPORTED = 2.0  # pixels (Crazyflie flowdeck default)
SETPOINT_RATE_HZ = 100  # ground-station setpoint rate
FLOW_RATE_HZ = 100
TOF_RATE_HZ = 50

# A trajectory maps time (s) to (position xyz, velocity xyz, acceleration xyz, yaw deg).
Trajectory = Callable[[float], tuple[np.ndarray, np.ndarray, np.ndarray, float]]


def min_jerk(t0: float, duration: float, p0: np.ndarray, p1: np.ndarray, yaw: float) -> Trajectory:
    """Minimum-jerk move like Crazyswarm goTo/takeoff, with velocity and acceleration for feedforward."""
    def traj(t: float):
        if duration <= 0.0:
            return p1, np.zeros(3), np.zeros(3), yaw
        s = min(1.0, max(0.0, (t - t0) / duration))
        inside = 0.0 < s < 1.0
        blend = s ** 3 * (10 - 15 * s + 6 * s * s)
        dblend = (30 * s * s - 60 * s ** 3 + 30 * s ** 4) / duration if inside else 0.0
        ddblend = (60 * s - 180 * s * s + 120 * s ** 3) / duration ** 2 if inside else 0.0
        d = p1 - p0
        return p0 + d * blend, d * dblend, d * ddblend, yaw
    return traj


def figure8(t0: float, center: np.ndarray, size: float, period: float) -> Trajectory:
    """Lissajous figure-8 in the horizontal plane: x = A sin(wt), y = A/2 sin(2wt)."""
    w = 2 * math.pi / period

    def traj(t: float):
        tau = max(0.0, t - t0)
        pos = center + np.array([size * math.sin(w * tau), 0.5 * size * math.sin(2 * w * tau), 0.0])
        vel = np.array([size * w * math.cos(w * tau), size * w * math.cos(2 * w * tau), 0.0])
        acc = np.array([-size * w * w * math.sin(w * tau), -2 * size * w * w * math.sin(2 * w * tau), 0.0])
        return pos, vel, acc, 0.0
    return traj


@dataclass
class Log:
    t: list = field(default_factory=list)
    pos: list = field(default_factory=list)  # truth
    quat: list = field(default_factory=list)  # truth (w, x, y, z)
    euler: list = field(default_factory=list)  # truth roll, pitch, yaw (deg)
    est: list = field(default_factory=list)  # firmware estimate x y z roll pitch yaw
    setpoint: list = field(default_factory=list)  # x y z
    motors: list = field(default_factory=list)  # normalized 0..1
    gyro_raw: list = field(default_factory=list)  # roll-axis gyro (deg/s), only recorded in vibration runs
    gyro_notch: list = field(default_factory=list)
    gyro_filtered: list = field(default_factory=list)
    roll_rate: list = field(default_factory=list)  # ground-truth body roll rate (deg/s)
    safety: list = field(default_factory=list)  # optional opt-in supervisor state
    safety_reason: list = field(default_factory=list)
    hold_position: list = field(default_factory=list)  # captured safety reference, not mission target
    velocity: list = field(default_factory=list)  # true world velocity for landing evidence
    safety_height: list = field(default_factory=list)  # supervisor's selected landing height
    position_fresh: list = field(default_factory=list)
    tof_fresh: list = field(default_factory=list)
    attitude_target: list = field(default_factory=list)  # actual controller roll/pitch targets
    flow_braking: list = field(default_factory=list)

    def arrays(self) -> dict[str, np.ndarray]:
        return {k: np.asarray(v) for k, v in self.__dict__.items()}


class SitlDrone:
    """positioning: "mocap" (100 Hz external position) or "flow" (PMW3901 flow + VL53L1X ToF)."""

    def __init__(
        self,
        vehicle: Vehicle = WHOOP_2IN,
        sensors: Sensors = DEFAULT_SENSORS,
        start: tuple[float, float, float] = (0.0, 0.0, 0.0),
        seed: int = 0,
        profile: str = "stock",
        positioning: str = "mocap",
        thrust_base: float | None = None,
        vibration: VibrationSpec | None = None,
        rpm_notch: bool = False,
        rpm_error: float = 0.02,
        safe_landing: bool = False,
        local_flow: bool = False,
    ):
        if not math.isfinite(rpm_error) or abs(rpm_error) > 0.5:
            raise ValueError("RPM error must be finite and within +/-50 percent")
        self.vehicle = vehicle
        self.rng = np.random.default_rng(seed)
        self.fw = FlightFirmware(PROFILE_VARIANT[profile])
        self.fw.apply_profile(profile)
        self.fw.set_rpm_notch(rpm_notch)
        self.safe_landing = safe_landing
        self.fw.set_failsafe_enabled(safe_landing)
        if profile == "stock" or thrust_base is not None:
            # The stock firmware has no airframe profile: at least give it this airframe's hover throttle.
            self.fw.set_param("posCtlPid.thrustBase",
                              thrust_base if thrust_base is not None else vehicle.hover_command * RATIO_FULL_SCALE)
        self.rate = self.fw.rate_hz
        self.dt = 1.0 / self.rate
        self.substeps = max(1, round(2000 / self.rate))  # physics step <= 0.5 ms
        self.quad = Quadrotor(vehicle, State(pos=np.array(start, dtype=float)), ground=True)
        self.imu = ImuModel(sensors.imu, self.rate, self.rng)
        self.vibration = MotorVibration(vibration, seed=seed + 1009) if vibration else None
        self.rpm_error = rpm_error
        self.rpm_link_up = True
        self._rpm_history = deque(maxlen=2)  # 100 Hz reports with one-report (10 ms) delivery latency
        self.positioning = positioning
        self.every = {
            "mocap": round(self.rate / sensors.mocap.rate_hz),
            "setpoint": round(self.rate / SETPOINT_RATE_HZ),
            "flow": round(self.rate / FLOW_RATE_HZ),
            "tof": round(self.rate / TOF_RATE_HZ),
            "rpm": round(self.rate / 100),
            "battery": round(self.rate / 10),
        }
        # MocapModel counts latency in calls, and it is sampled every `every["mocap"]` ticks.
        self.mocap = MocapModel(sensors.mocap, self.rng, self.every["mocap"] * self.dt) if positioning == "mocap" else None
        self.flow = (FlowModel(sensors.flow, np.random.default_rng(seed + 3019)) if local_flow
                     else FlowModel(sensors.flow, self.rng) if positioning == "flow" else None)
        # In safety runs local range is independent of camera/IMU RNG and available with mocap.
        # Legacy flow runs preserve their original sensor stream exactly.
        self.tof = (TofModel(sensors.tof, np.random.default_rng(seed + 2017)) if safe_landing
                    else TofModel(sensors.tof, self.rng) if positioning == "flow" else None)
        self.command = np.zeros(4)
        self.tick = 0
        self.trajectory: Trajectory | None = None
        self.target: tuple[float, float, float, float] | None = None  # last sent x, y, z, yaw
        self.link_up = True
        self.tracking_up = True  # external position only; independent of command radio
        self.tracking_offset = np.zeros(3)  # injected camera reference jump for recovery tests
        self.height_up = True  # onboard downward range only
        self.flow_up = True
        self.battery_cell_v: float | None = None  # pushed at 10 Hz when set; no discharge or thrust-sag model
        self.flow_scale = 1.0  # injected calibration bias, not truth fed to control
        self.flow_latency_s = 0.0
        self.tof_bias_m = 0.0  # mounting offset: range reads this much long at every height
        self._flow_history = deque()
        self.log = Log()

    @property
    def time(self) -> float:
        return self.tick * self.dt

    # ---- ground-station commands ----
    def follow(self, trajectory: Trajectory) -> None:
        self.trajectory = trajectory

    def goto(self, x: float, y: float, z: float, yaw_deg: float = 0.0, duration: float = 0.0) -> None:
        """Fly to a point along a minimum-jerk path (duration 0 = step setpoint)."""
        start = np.array(self.target[:3]) if self.target else self.quad.state.pos.copy()
        self.follow(min_jerk(self.time, duration, start, np.array([x, y, z], dtype=float), yaw_deg))

    def takeoff(self, height: float, duration: float = 2.0) -> None:
        p = self.quad.state.pos
        self.goto(p[0], p[1], height, 0.0, duration)

    def calibrate_and_arm(self, timeout: float = 3.0) -> None:
        """Sit still until the firmware finishes gyro/accel calibration, then arm."""
        while not self.fw.ready:
            if self.time > timeout:
                raise RuntimeError("firmware did not finish calibration")
            self.step(record=False)
        if self.fw.arm(True):
            return
        if self.safe_landing:
            # Calibration does not drain estimator queues until READY. Those early
            # packets must not count as fresh arming evidence; await new samples.
            for _ in range(round(0.3 * self.rate)):
                self.step(record=False)
                if self.fw.arm(True):
                    return
        raise RuntimeError("firmware refused to arm")

    # ---- simulation ----
    def _sensors(self) -> None:
        s = self.quad.state
        rot = quat_to_rot(s.quat)
        if self.mocap and self.tick % self.every["mocap"] == 0:
            measured = self.mocap.sample(s.pos)
            if self.tracking_up:
                self.fw.push_position(*(measured + self.tracking_offset), MOCAP_STD_REPORTED)
        if self.flow and self.tick % self.every["flow"] == 0:
            m = self.flow.sample(s.pos[2], s.vel, rot, s.omega, self.every["flow"] * self.dt)
            if m:
                self._flow_history.append((self.time, m))
            latest = None
            while self._flow_history and self._flow_history[0][0] <= self.time - self.flow_latency_s + 1e-9:
                latest = self._flow_history.popleft()[1]
            if latest is not None and self.flow_up:
                self.fw.push_flow(latest[0] * self.flow_scale, latest[1] * self.flow_scale,
                                  self.every["flow"] * self.dt, FLOW_STD_REPORTED)
        if self.battery_cell_v is not None and self.tick % self.every["battery"] == 0:
            self.fw.push_battery(self.battery_cell_v)
        if self.tof and self.tick % self.every["tof"] == 0:
            d = self.tof.sample(s.pos[2], rot)
            if self.height_up and d is not None and (self.safe_landing or s.pos[2] > 0.03):
                self.fw.push_tof(d + self.tof_bias_m, TOF_STD_REPORTED)

    def step(self, record: bool = True) -> None:
        s = self.quad.state
        imu_gyro, imu_acc = self.imu.sample(s.omega, self.quad.specific_force_body(self.command))
        if self.vibration:
            imu_gyro += self.vibration.sample(s.rotor, self.dt)
            if self.tick % self.every["rpm"] == 0:
                self._rpm_history.append(self.vibration.rpm(s.rotor) * (1.0 + self.rpm_error))
                if len(self._rpm_history) == 2 and self.rpm_link_up:
                    if not self.fw.push_rpm(self._rpm_history[0]):
                        raise RuntimeError("simulated RPM packet was rejected")
        self._sensors()
        if self.trajectory and self.tick % self.every["setpoint"] == 0:
            sp_pos, sp_vel, sp_acc, sp_yaw = self.trajectory(self.time)
            self.target = (*sp_pos, sp_yaw)
            if self.link_up:
                self.fw.set_trajectory_setpoint(sp_pos, sp_vel, sp_acc, sp_yaw)

        ratios = self.fw.step(imu_gyro, imu_acc, int(self.tick * 1e6 * self.dt))
        self.command = np.array(ratios) / RATIO_FULL_SCALE
        for _ in range(self.substeps):
            self.quad.step(self.command, self.dt / self.substeps)
        self.tick += 1

        if record:
            st = self.quad.state
            est = self.fw.state()
            self.log.t.append(self.time)
            self.log.pos.append(st.pos.copy())
            self.log.quat.append(st.quat.copy())
            self.log.euler.append([math.degrees(a) for a in quat_to_euler(st.quat)])
            self.log.est.append(est[0:3] + est[6:9])
            self.log.setpoint.append(list(self.target[:3]) if self.target else [math.nan] * 3)
            self.log.motors.append(self.command.copy())
            if self.safe_landing:
                self.log.safety.append(self.fw.failsafe_state)
                self.log.safety_reason.append(self.fw.log("fcSafety.reason"))
                self.log.velocity.append(st.vel.copy())
                self.log.hold_position.append([self.fw.log(f"fcSafety.hold{a}") for a in "XYZ"])
                self.log.safety_height.append(self.fw.log("fcSafety.height"))
                self.log.position_fresh.append(self.fw.log("fcSafety.posFresh"))
                self.log.tof_fresh.append(self.fw.log("fcSafety.tofFresh"))
                self.log.attitude_target.append([self.fw.log(f"controller.{a}") for a in ("roll", "pitch")])
                self.log.flow_braking.append(self.fw.log("fcSafety.flowBrake"))
            if self.vibration:
                self.log.gyro_raw.append(self.fw.log("fcGyro.rawX"))
                self.log.gyro_notch.append(self.fw.log("fcGyro.notchX"))
                self.log.gyro_filtered.append(self.fw.log("fcGyro.filteredX"))
                self.log.roll_rate.append(math.degrees(st.omega[0]))

    def run(self, seconds: float, record_hz: float | None = 250.0) -> None:
        """Advance `seconds`; log at record_hz (None = no logging)."""
        every = 0 if record_hz is None else max(1, round(self.rate / record_hz))
        for _ in range(int(round(seconds / self.dt))):
            self.step(record=bool(every) and (self.tick + 1) % every == 0)
