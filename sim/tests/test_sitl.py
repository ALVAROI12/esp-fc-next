"""Gate G2 software-in-the-loop tests: the real flight firmware flying the 6-DOF model."""

import numpy as np
import pytest

from sitl import FlightFirmware, SitlDrone
from sitl.drone import figure8
from sitl.env import GotoEnv

HOVER_GATE_M = 0.05  # project plan G2: single drone hovers within +/-5 cm
FIGURE8_GATE_M = 0.10  # project plan G2 (Task 1.6): figure-8 error < 10 cm


def fly_hover(seconds: float = 14.0, seed: int = 1, profile: str = "stock", **params) -> tuple[SitlDrone, float]:
    d = SitlDrone(seed=seed, profile=profile)
    for name, value in params.items():
        d.fw.set_param(name.replace("__", "."), value)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    t_takeoff = d.time
    d.run(seconds, record_hz=100)
    return d, t_takeoff


def fly_figure8(profile: str, seed: int = 5, **params) -> float:
    """Max tracking error (m) over a 0.6 m x 0.3 m figure-8 with an 8 s period (peak speed about 0.7 m/s)."""
    d = SitlDrone(seed=seed, profile=profile)
    for name, value in params.items():
        d.fw.set_param(name.replace("__", "."), value)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    start = d.time
    d.follow(figure8(start, np.array([0.0, 0.0, 1.0]), 0.6, 8.0))
    d.run(16.0, record_hz=100)
    log = d.log.arrays()
    steady = log["t"] > start + 2.0
    assert d.fw.assert_count == 0 and not d.fw.tumbled
    return float(np.max(np.linalg.norm(log["pos"][steady] - log["setpoint"][steady], axis=1)))


def test_firmware_instances_are_independent():
    a, b = FlightFirmware(), FlightFirmware()
    a.set_param("posCtlPid.thrustBase", 12345)
    assert a.param("posCtlPid.thrustBase") == 12345
    assert b.param("posCtlPid.thrustBase") != 12345


def test_4k_variant_runs_at_4khz_and_rejects_1k_profile():
    fw = FlightFirmware("4k")
    assert fw.rate_hz == 4000
    with pytest.raises(Exception, match="different loop rate"):
        fw.apply_profile("whoop2_interim")


def test_firmware_calibration_finds_the_simulated_gyro_bias():
    d = SitlDrone(seed=3)
    assert not d.fw.arm(True)  # supervisor refuses before calibration
    d.calibrate_and_arm()
    measured = np.array([d.fw.log(f"fcStep.bias{a}") for a in "XYZ"])
    assert measured == pytest.approx(d.imu.gyro_bias, abs=0.02)


def test_g2_hover_within_5cm_with_stock_firmware():
    d, t0 = fly_hover()
    log = d.log.arrays()
    steady = log["t"] > t0 + 8.0
    err = np.linalg.norm(log["pos"][steady] - np.array([0.0, 0.0, 1.0]), axis=1)
    assert err.max() < HOVER_GATE_M
    assert d.fw.assert_count == 0
    assert d.fw.log("fcKalman.resets") == 0


@pytest.mark.parametrize("profile", ["whoop2_interim", "whoop2_4k"])
def test_profiles_hover_without_takeoff_overshoot(profile):
    d, t0 = fly_hover(seconds=8.0, profile=profile)
    log = d.log.arrays()
    assert log["pos"][:, 2].max() - 1.0 < 0.02
    steady = log["t"] > t0 + 4.0
    assert np.linalg.norm(log["pos"][steady] - np.array([0.0, 0.0, 1.0]), axis=1).max() < 0.02


def test_estimator_tracks_truth_and_velocity_is_unbiased():
    """Also a regression test: a SITL bug once fed trajectory acceleration as the accelerometer
    reading on setpoint ticks, which showed up as a -0.25 m/s vertical-velocity bias."""
    d, t0 = fly_hover(seconds=8.0, profile="whoop2_interim")
    log = d.log.arrays()
    steady = log["t"] > t0 + 4.0
    assert np.linalg.norm(log["est"][steady, :3] - log["pos"][steady], axis=1).max() < 0.01
    assert abs(d.fw.state()[5]) < 0.05


def test_tuning_finding_position_integrator_causes_takeoff_overshoot():
    """Locks in the SITL finding: posCtlPid.zKi = 0 removes the takeoff overshoot (see sim/README.md)."""
    stock, _ = fly_hover(seconds=6.0)
    tuned, _ = fly_hover(seconds=6.0, posCtlPid__zKi=0.0)
    overshoot = lambda d: d.log.arrays()["pos"][:, 2].max() - 1.0  # noqa: E731
    assert overshoot(stock) > 0.08
    assert overshoot(tuned) < 0.01


@pytest.mark.parametrize("profile", ["whoop2_interim", "whoop2_4k"])
def test_g2_figure8_gate(profile):
    assert fly_figure8(profile) < FIGURE8_GATE_M


def test_feedforward_is_what_makes_the_figure8_pass():
    without_ff = fly_figure8("whoop2_interim", posCtlPid__velFF=0.0, posCtlPid__accFF=0.0)
    with_ff = fly_figure8("whoop2_interim")
    assert without_ff > FIGURE8_GATE_M
    assert with_ff < without_ff / 2


@pytest.fixture(scope="module")
def flow_hover():
    """Keep one deterministic run for the height check and the unmet XY requirement."""
    d = SitlDrone(seed=5, profile="whoop2_interim", positioning="flow")
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0, record_hz=None)
    start_xy = d.quad.state.pos[:2].copy()
    d.run(20.0, record_hz=50)
    return d, start_xy


def test_flow_and_tof_height_hold(flow_hover):
    d, _ = flow_hover
    z = d.log.arrays()["pos"][:, 2]
    assert abs(z.mean() - 1.0) < 0.02 and z.std() < 0.01
    assert not d.fw.tumbled
    assert d.fw.assert_count == 0


@pytest.mark.xfail(strict=True, reason="FLOW-01: velocity-only optical flow cannot bound absolute XY drift; shared positioning required")
def test_flow_only_absolute_xy_accuracy_requirement(flow_hover):
    """An unmet requirement, NOT a passing positioning gate. Never raise this limit to hide drift."""
    d, start_xy = flow_hover
    xy = d.log.arrays()["pos"][:, :2]
    assert np.linalg.norm(xy - start_xy, axis=1).max() < HOVER_GATE_M


def test_link_loss_follows_crazyflie_commander_watchdog():
    """Stock behaviour: level after 0.5 s, cut motors after 2 s -> the drone falls. Task 1.8 adds hold-and-land."""
    d, _ = fly_hover(seconds=6.0, posCtlPid__zKi=0.0)
    d.link_up = False
    d.run(1.0)
    assert d.quad.state.pos[2] > 0.8  # still flying while leveling
    d.run(1.5)
    assert np.all(d.command == 0)
    d.run(1.0)
    assert d.quad.state.pos[2] == pytest.approx(0.0, abs=1e-6)


def test_emergency_stop_cuts_motors():
    d, _ = fly_hover(seconds=5.0)
    d.fw.emergency_stop()
    d.run(0.01)
    assert np.all(d.command == 0)


def test_gym_env_api():
    env = GotoEnv(seed=0, episode_seconds=1.0)
    obs, info = env.reset()
    assert obs.shape == (GotoEnv.observation_size,)
    done = False
    while not done:
        obs, reward, terminated, truncated, info = env.step(np.clip(obs[9:12], -0.5, 0.5))  # greedy policy
        done = terminated or truncated
    assert not terminated
    assert info["distance"] < np.linalg.norm(env.goal - np.array([0, 0, 1.0])) + 0.05


@pytest.mark.parametrize("profile", ["whoop2_interim", "whoop2_4k"])
def test_gym_action_and_episode_use_real_seconds(profile):
    env = GotoEnv(seed=3, control_hz=10, episode_seconds=0.2, profile=profile)
    env.reset()
    start = env.drone.time
    env.step(np.zeros(3))
    assert env.drone.time - start == pytest.approx(0.1)
    _, _, _, truncated, _ = env.step(np.zeros(3))
    assert truncated
    assert env.drone.time - start == pytest.approx(0.2)
