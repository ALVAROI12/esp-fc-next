"""Camera-loss braking must beat the passive-drift baseline, without using truth control inputs."""

import numpy as np
import pytest
from dataclasses import replace

import flow_braking_report
from failsafe_report import fly_loss
from model.params import DEFAULT_SENSORS
from sitl import SitlDrone
from sitl.firmware import FAILSAFE_LAND


@pytest.mark.parametrize("profile", ["whoop2_interim", "whoop2_4k"])
def test_flow_brakes_moving_camera_loss(profile):
    result = fly_loss(profile, "tracking", moving=True, local_flow=True)
    assert result["passed"]
    assert result["max_drift"] <= 0.35
    assert np.any(result["log"]["flow_braking"])


@pytest.mark.parametrize("profile", ["whoop2_interim", "whoop2_4k"])
def test_restored_camera_jump_cannot_reenable_absolute_xy(profile):
    result = fly_loss(profile, "tracking", moving=True, local_flow=True,
                      recover_after=0.8, tracking_jump=(0.5, -0.4, 0.))
    assert result["passed"]
    assert result["max_drift"] <= 0.35


def test_flow_dropout_degrades_to_level_descent():
    d = SitlDrone(profile="whoop2_4k", safe_landing=True, local_flow=True)
    d.calibrate_and_arm(); d.takeoff(1, 2); d.run(4, record_hz=None)
    d.tracking_up = False
    d.run(0.4, record_hz=100)
    assert d.fw.failsafe_state == FAILSAFE_LAND
    assert d.fw.log("fcSafety.flowBrake") == 1
    d.flow_up = False
    d.run(0.15, record_hz=100)
    assert d.fw.log("fcSafety.flowBrake") == 0
    assert d.fw.log("controller.roll") == 0
    assert d.fw.log("controller.pitch") == 0
    assert np.any(d.command > 0)  # missing flow alone must not cut height-controlled descent


@pytest.mark.parametrize("changes", [
    {"seed": 1}, {"seed": 5}, {"flow_scale": 0.8}, {"flow_scale": 1.2},
    {"flow_latency_s": 0.02}, {"yaw_deg": 90.},
    {"sensors": replace(DEFAULT_SENSORS, flow=replace(DEFAULT_SENSORS.flow, std_pixels=4.))},
])
def test_declared_braking_stress_envelope(changes):
    result = fly_loss("whoop2_4k", "tracking", moving=True, local_flow=True, **changes)
    assert result["passed"]
    assert np.any(result["log"]["flow_braking"])
    assert result["max_drift"] <= 0.50


def test_failed_run_writes_fail_report_instead_of_crashing(monkeypatch, tmp_path):
    monkeypatch.setattr(flow_braking_report, "fly_loss", lambda *args, **kwargs: {
        "passed": False, "max_drift": 0.6, "max_descent": None, "approach": None})
    monkeypatch.setattr("sys.argv", ["flow_braking_report.py", "--no-plots", "--out", str(tmp_path)])
    with pytest.raises(SystemExit) as result:
        flow_braking_report.main()
    assert result.value.code == 1
    assert "Declared checks: FAIL" in (tmp_path / "README.md").read_text()
