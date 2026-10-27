"""Task 1.6b SITL checks: synthetic motor vibration and the firmware RPM notch bank.

Fair comparisons use the same seed, sensor noise and vibration model. The vibration
phases come from a separate seeded RNG, so enabling the filter does not change the
disturbance. All vibration levels are assumptions until blackbox measurements exist.
"""

import numpy as np
import pytest

from model.vibration import VibrationSpec
from sitl import SitlDrone
from sitl.drone import figure8
from sitl.firmware import FirmwareError

HOVER_GATE_M = 0.05
FIGURE8_GATE_M = 0.10
AMPLITUDE_DPS = 12.0


def fly_hover(*, notch: bool, amplitude: float = AMPLITUDE_DPS, rpm_error: float = 0.02,
              seed: int = 4, vibration: bool = True) -> SitlDrone:
    d = SitlDrone(seed=seed, profile="whoop2_4k",
                  vibration=VibrationSpec(amplitude_dps=amplitude) if vibration else None,
                  rpm_notch=notch, rpm_error=rpm_error)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    d.run(1.0, record_hz=4000)
    return d


def artifact(d: SitlDrone) -> tuple[np.ndarray, np.ndarray]:
    """Roll-axis gyro artifact before and after the notch, relative to true motion."""
    log = d.log.arrays()
    return log["gyro_raw"] - log["roll_rate"], log["gyro_notch"] - log["roll_rate"]


def hover_error(d: SitlDrone) -> float:
    pos = d.log.arrays()["pos"]
    return float(np.linalg.norm(pos - np.array([0.0, 0.0, 1.0]), axis=1).max())


@pytest.fixture(scope="module")
def hover_pair() -> dict[bool, SitlDrone]:
    return {on: fly_hover(notch=on) for on in (False, True)}


def test_notch_suppresses_rotation_artifact_without_degrading_hover(hover_pair):
    raw_off, notched_off = artifact(hover_pair[False])
    raw_on, notched_on = artifact(hover_pair[True])

    # Same disturbance: the unfiltered artifact must match across runs.
    assert np.std(raw_off) == pytest.approx(np.std(raw_on), rel=0.1)
    assert np.std(notched_off) == pytest.approx(np.std(raw_off), rel=1e-6)  # exact bypass
    assert np.std(notched_on) < 0.25 * np.std(raw_on)
    assert np.std(raw_on) > 2.0  # the scenario is actually stressing the sensor

    assert hover_error(hover_pair[False]) < HOVER_GATE_M
    assert hover_error(hover_pair[True]) < HOVER_GATE_M
    assert not hover_pair[True].fw.tumbled
    assert hover_pair[True].fw.assert_count == 0


def test_filter_is_exact_bypass_without_rpm_telemetry():
    off = fly_hover(notch=False, vibration=False)
    on = fly_hover(notch=True, vibration=False)
    a, b = off.log.arrays(), on.log.arrays()
    np.testing.assert_array_equal(a["pos"], b["pos"])
    np.testing.assert_array_equal(a["motors"], b["motors"])
    assert on.fw.log("fcGyro.rpmFresh") == 0


def test_figure8_gate_holds_with_assumed_vibration():
    d = SitlDrone(seed=5, profile="whoop2_4k", vibration=VibrationSpec(amplitude_dps=AMPLITUDE_DPS),
                  rpm_notch=True)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    start = d.time
    d.follow(figure8(start, np.array([0.0, 0.0, 1.0]), 0.6, 8.0))
    d.run(16.0, record_hz=100)
    log = d.log.arrays()
    window = log["t"] > start + 2.0
    error = np.linalg.norm(log["pos"] - log["setpoint"], axis=1)
    assert error[window].max() < FIGURE8_GATE_M
    assert not d.fw.tumbled
    assert d.fw.assert_count == 0
    assert np.std(log["gyro_notch"] - log["roll_rate"]) < 1.0


def test_rpm_error_and_telemetry_dropout_stay_bounded():
    d = fly_hover(notch=True, rpm_error=0.10)
    _, notched = artifact(d)
    assert np.std(notched) < 2.0  # reject a fully bypassed notch (~3.8 deg/s)
    assert hover_error(d) < HOVER_GATE_M
    assert not d.fw.tumbled

    d.rpm_link_up = False
    d.run(0.05, record_hz=4000)
    assert d.fw.log("fcGyro.rpmFresh") == 1  # last report is still within the timeout
    d.run(0.10, record_hz=4000)
    assert d.fw.log("fcGyro.rpmFresh") == 0  # 100 ms without telemetry bypasses the bank
    start = len(d.log.t)
    d.run(0.02, record_hz=4000)
    log = d.log.arrays()
    np.testing.assert_array_equal(log["gyro_notch"][start:], log["gyro_raw"][start:])
    d.rpm_link_up = True
    d.run(0.15, record_hz=4000)
    assert d.fw.log("fcGyro.rpmFresh") == 1
    assert not d.fw.tumbled
    assert hover_error(d) < HOVER_GATE_M


def test_notch_cannot_be_reconfigured_while_armed():
    d = fly_hover(notch=True)
    with pytest.raises(FirmwareError, match="RPM notch"):
        d.fw.set_rpm_notch(False)
    d.fw.arm(False)
    d.fw.set_rpm_notch(False)
