"""Gate G2 software-in-the-loop tests: the real flight firmware flying the 6-DOF model."""

import numpy as np
import pytest

from sitl import FlightFirmware, SitlDrone
from sitl.env import GotoEnv

HOVER_GATE_M = 0.05  # project plan G2: single drone hovers within +/-5 cm


def fly_hover(seconds: float = 14.0, seed: int = 1, **params) -> tuple[SitlDrone, float]:
    d = SitlDrone(seed=seed)
    for name, value in params.items():
        d.fw.set_param(name.replace("__", "."), value)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    t_takeoff = d.time
    d.run(seconds, record_every=10)
    return d, t_takeoff


def test_firmware_instances_are_independent():
    a, b = FlightFirmware(), FlightFirmware()
    a.set_param("posCtlPid.thrustBase", 12345)
    assert a.param("posCtlPid.thrustBase") == 12345
    assert b.param("posCtlPid.thrustBase") != 12345


def test_firmware_calibration_finds_the_simulated_gyro_bias():
    d = SitlDrone(seed=3)
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


def test_estimator_tracks_truth_in_hover():
    d, t0 = fly_hover(seconds=8.0)
    log = d.log.arrays()
    steady = log["t"] > t0 + 4.0
    est_err = np.linalg.norm(log["est"][steady, :3] - log["pos"][steady], axis=1)
    assert est_err.max() < 0.01


def test_tuning_finding_position_integrator_causes_takeoff_overshoot():
    """Locks in the SITL finding: posCtlPid.zKi = 0 removes the takeoff overshoot (see sim/README.md)."""
    stock, _ = fly_hover(seconds=6.0)
    tuned, _ = fly_hover(seconds=6.0, posCtlPid__zKi=0.0)
    overshoot = lambda d: d.log.arrays()["pos"][:, 2].max() - 1.0  # noqa: E731
    assert overshoot(stock) > 0.08
    assert overshoot(tuned) < 0.01


def test_square_path_tracking():
    d = SitlDrone(seed=2)
    d.fw.set_param("posCtlPid.zKi", 0.0)
    d.calibrate_and_arm()
    d.takeoff(1.0, 2.0)
    d.run(4.0)
    for corner in [(0.5, 0, 1), (0.5, 0.5, 1), (0, 0.5, 1), (0, 0, 1)]:
        d.goto(*corner, 0.0, duration=2.0)
        d.run(3.0)
        assert np.linalg.norm(d.quad.state.pos - np.array(corner)) < 0.05


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
    total, done = 0.0, False
    while not done:
        obs, reward, terminated, truncated, info = env.step(np.clip(obs[9:12], -0.5, 0.5))  # greedy policy
        total += reward
        done = terminated or truncated
    assert not terminated
    assert info["distance"] < np.linalg.norm(env.goal - np.array([0, 0, 1.0])) + 0.05
