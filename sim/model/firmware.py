"""Read controller gains straight from the flight firmware so the model never drifts from it."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re

REPO = Path(__file__).resolve().parents[2]
PID_H = REPO / "firmware/flight/components/core/crazyflie/modules/interface/pid.h"

# Every flight target except the old ESP32-S2-Drone uses the #else branch of pid.h.
DEFAULT_PROFILE = "default"
S2_DRONE_PROFILE = "esp32_s2_drone_v1_2"


@dataclass(frozen=True)
class PidGains:
    kp: float
    ki: float
    kd: float
    i_limit: float = 0.0  # 0 = no limit

    def scaled(self, factor: float) -> "PidGains":
        return PidGains(self.kp * factor, self.ki * factor, self.kd * factor, self.i_limit)


@dataclass(frozen=True)
class AttitudeGains:
    roll_rate: PidGains
    pitch_rate: PidGains
    yaw_rate: PidGains
    roll: PidGains
    pitch: PidGains
    yaw: PidGains


def _block(text: str, profile: str) -> str:
    match = re.search(r"#ifdef\s+CONFIG_TARGET_ESP32_S2_DRONE_V1_2(.*?)#else(.*?)#endif", text, re.S)
    if not match:
        raise ValueError(f"unexpected layout in {PID_H}")
    return match.group(1) if profile == S2_DRONE_PROFILE else match.group(2)


def load_gains(profile: str = DEFAULT_PROFILE, pid_h: Path = PID_H) -> AttitudeGains:
    values = {
        name: float(value)
        for name, value in re.findall(r"#define\s+(PID_\w+)\s+([-0-9.eE]+)", _block(pid_h.read_text(), profile))
    }

    def gains(axis: str) -> PidGains:
        return PidGains(
            values[f"PID_{axis}_KP"],
            values[f"PID_{axis}_KI"],
            values[f"PID_{axis}_KD"],
            values.get(f"PID_{axis}_INTEGRATION_LIMIT", 0.0),
        )

    return AttitudeGains(
        roll_rate=gains("ROLL_RATE"),
        pitch_rate=gains("PITCH_RATE"),
        yaw_rate=gains("YAW_RATE"),
        roll=gains("ROLL"),
        pitch=gains("PITCH"),
        yaw=gains("YAW"),
    )
