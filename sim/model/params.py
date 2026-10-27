"""Vehicle, actuator, sensor and loop-timing parameters for the G1 model.

Every value records where it comes from. ``ASSUMED`` values are typical published
figures for a 2-inch ducted 1S whoop and must be replaced with measurements of the
real frame (scale + thrust stand) before gate G6.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math

G = 9.81

DATASHEET = "datasheet"
FIRMWARE = "firmware source"
ASSUMED = "assumed (typical 2-inch 1S whoop) - measure on real hardware"


@dataclass(frozen=True)
class Vehicle:
    """Rigid body + quad-X geometry + propulsion."""

    mass: float = 0.060  # kg, all-up incl. battery
    arm: float = 0.045  # m, motor axis to centre (about 90 mm motor-to-motor diagonal)
    ixx: float = 3.2e-5  # kg m^2
    iyy: float = 3.2e-5
    izz: float = 5.8e-5
    thrust_max: float = 0.40  # N per motor at full command (1103 + 40 mm prop, 1S, ducted)
    torque_ratio: float = 0.006  # m, rotor drag torque / thrust
    motor_tau: float = 0.020  # s, first-order rotor speed time constant
    drag_linear: float = 0.02  # N/(m/s), body translational drag

    sources: dict = field(
        default_factory=lambda: {
            "mass": ASSUMED,
            "arm": ASSUMED,
            "inertia": ASSUMED + " (point-mass estimate: motors, ducts, battery)",
            "thrust_max": ASSUMED,
            "torque_ratio": ASSUMED,
            "motor_tau": ASSUMED,
            "drag_linear": ASSUMED,
        },
        compare=False,
    )

    @property
    def weight(self) -> float:
        return self.mass * G

    @property
    def thrust_to_weight(self) -> float:
        return 4.0 * self.thrust_max / self.weight

    @property
    def hover_command(self) -> float:
        """Normalized motor command u in [0, 1] that hovers, with thrust = thrust_max * u^2."""
        return math.sqrt(self.weight / (4.0 * self.thrust_max))

    def scaled(self, **factors: float) -> "Vehicle":
        """Copy with fields multiplied by the given factors, e.g. scaled(mass=1.2)."""
        return replace(self, **{k: getattr(self, k) * v for k, v in factors.items()})


@dataclass(frozen=True)
class Imu:
    """TDK ICM-42688-P (DS-000347)."""

    gyro_noise_density: float = 0.0028  # deg/s/sqrt(Hz)
    accel_noise_density: float = 70e-6  # g/sqrt(Hz)
    gyro_bias_initial: float = 0.5  # deg/s, 1-sigma before calibration
    gyro_bias_walk: float = 0.001  # deg/s/sqrt(s), random walk (ASSUMED, not in datasheet)
    accel_bias_initial: float = 0.02  # g, 1-sigma
    source: str = DATASHEET + " ICM-42688-P (noise densities); bias walk " + ASSUMED


@dataclass(frozen=True)
class Baro:
    """Bosch BMP581."""

    pressure_noise: float = 0.1  # Pa RMS at the configured ODR/oversampling (verify for chosen mode)
    rate_hz: float = 50.0
    source: str = DATASHEET + " BMP581 (approximate; confirm for selected oversampling)"


@dataclass(frozen=True)
class Tof:
    """ST VL53L1X, short distance mode."""

    std: float = 0.005  # m, below about 1.3 m
    rate_hz: float = 50.0
    max_range: float = 1.3  # m (short mode)
    source: str = DATASHEET + " VL53L1X (short mode)"


@dataclass(frozen=True)
class Flow:
    """PixArt PMW3901 optical flow."""

    std_pixels: float = 2.0  # Crazyflie flowdeck default measurement std
    fov_deg: float = 42.0
    pixels: int = 35
    rate_hz: float = 100.0
    source: str = DATASHEET + " PMW3901 (FOV, resolution); std from Crazyflie flowdeck driver"

    @property
    def rad_per_pixel(self) -> float:
        return math.radians(self.fov_deg) / self.pixels


@dataclass(frozen=True)
class Mocap:
    """Low-cost camera motion capture (PS3 Eye class)."""

    std: float = 0.002  # m
    rate_hz: float = 100.0
    latency: float = 0.015  # s, capture + processing + radio
    source: str = ASSUMED + " (low-cost mocap; measure with the real setup)"


@dataclass(frozen=True)
class LoopTiming:
    """Rates and filters from the flight firmware (stabilizer_types.h, sensors_*.c)."""

    sensor_rate_hz: float = 1000.0  # stabilizer loop / IMU sample
    gyro_lpf_hz: float = 80.0  # GYRO_LPF_CUTOFF_FREQ, 2-pole Butterworth
    attitude_rate_hz: float = 500.0  # ATTITUDE_RATE: rate + attitude PID
    position_rate_hz: float = 100.0  # POSITION_RATE
    compute_delay_samples: int = 1  # sample-to-actuation latency in rate-loop samples
    source: str = FIRMWARE

    @property
    def rate_dt(self) -> float:
        return 1.0 / self.attitude_rate_hz


@dataclass(frozen=True)
class Sensors:
    imu: Imu = Imu()
    baro: Baro = Baro()
    tof: Tof = Tof()
    flow: Flow = Flow()
    mocap: Mocap = Mocap()


WHOOP_2IN = Vehicle()
DEFAULT_SENSORS = Sensors()
DEFAULT_TIMING = LoopTiming()  # ESP-Drone stock loop timing
PLANNED_TIMING = LoopTiming(  # ESP-FC Next target on the ESP32-P4 (project plan task 1.6)
    sensor_rate_hz=4000.0,
    gyro_lpf_hz=150.0,
    attitude_rate_hz=4000.0,
    source="project plan: 4 kHz ICM-42688-P sampling and rate loop on the ESP32-P4",
)
