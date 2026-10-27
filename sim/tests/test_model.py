"""Consistency tests for the G1 model: each piece is checked against an independent reference."""

from dataclasses import replace
import math
import warnings

import control as ct
import numpy as np
import pytest

from model import linear, robust
from model.closed_loop import Lpf2p, Pid
from model.dynamics import Quadrotor, State, quat_from_euler, quat_to_euler
from model.firmware import S2_DRONE_PROFILE, PidGains, load_gains
from model.params import DEFAULT_TIMING, PLANNED_TIMING, WHOOP_2IN, G, Imu
from model.sensors import ImuModel


def hovering(vehicle=WHOOP_2IN) -> Quadrotor:
    return Quadrotor(vehicle, State(pos=np.array([0, 0, 1.0]), rotor=np.full(4, vehicle.hover_command)), ground=False)


# ---- firmware gains ----

def test_gains_are_read_from_pid_h():
    g = load_gains()
    assert (g.roll_rate.kp, g.roll_rate.ki, g.roll_rate.kd) == (250.0, 500.0, 2.5)
    assert (g.roll.kp, g.roll.ki) == (5.9, 2.9)
    assert g.yaw_rate.i_limit == pytest.approx(166.7)
    assert load_gains(S2_DRONE_PROFILE).roll_rate.kp == 190.0


# ---- dynamics ----

def test_hover_command_is_an_equilibrium():
    quad = hovering()
    for _ in range(1000):
        quad.step(np.full(4, WHOOP_2IN.hover_command), 0.001)
    assert np.linalg.norm(quad.state.vel) < 1e-6
    assert np.linalg.norm(quad.state.omega) < 1e-9


def test_free_fall_without_thrust():
    quad = Quadrotor(replace(WHOOP_2IN, drag_linear=0.0), State(pos=np.array([0, 0, 10.0])), ground=False)
    for _ in range(100):
        quad.step(np.zeros(4), 0.001)
    assert quad.state.vel[2] == pytest.approx(-G * 0.1, rel=1e-6)


@pytest.mark.parametrize("motors, axis", [((2, 3), 0), ((1, 2), 1), ((1, 3), 2)])
def test_torque_sign_conventions(motors, axis):
    """Left motors -> +roll, back motors -> +pitch, CW-drag motors (M2, M4) -> +yaw."""
    quad = hovering()
    _, torque = quad.body_wrench(np.array([1.0 if i in motors else 0.0 for i in range(4)]))
    assert torque[axis] > 0


def test_euler_round_trip():
    angles = (0.3, -0.2, 1.1)
    assert quat_to_euler(quat_from_euler(*angles)) == pytest.approx(angles, abs=1e-12)


def test_accelerometer_reads_one_g_at_hover():
    quad = hovering()
    f = quad.specific_force_body(np.full(4, WHOOP_2IN.hover_command))
    assert f == pytest.approx([0, 0, G], abs=1e-9)


# ---- linearization vs nonlinear dynamics ----

@pytest.mark.parametrize("axis, index", [("roll", 0), ("pitch", 1), ("yaw", 2)])
def test_axis_gain_matches_nonlinear_model(axis, index):
    """Small mixer command -> angular acceleration in the 6-DOF model equals linear.axis_gain."""
    v = WHOOP_2IN
    u_h = v.hover_command
    cmd = 100.0  # control units
    from model.closed_loop import _PITCH_SIGN, _ROLL_SIGN, _YAW_SIGN
    sign = {"roll": _ROLL_SIGN / 2, "pitch": _PITCH_SIGN / 2, "yaw": _YAW_SIGN}[axis]
    rotor = u_h + sign * cmd / linear.RATIO_FULL_SCALE  # motors settled at the new command
    quad = hovering()
    _, torque = quad.body_wrench(rotor)
    inertia = (v.ixx, v.iyy, v.izz)[index]
    accel_deg = math.degrees(torque[index] / inertia)
    assert accel_deg / cmd == pytest.approx(linear.axis_gain(v, axis), rel=1e-3)


# ---- controller ports ----

def test_pid_transfer_function_matches_time_domain_port():
    g = PidGains(250.0, 500.0, 2.5)
    dt = 0.002
    pid = Pid(g, dt)
    err = np.sin(np.arange(200) * 0.37) + 0.3
    port = np.array([pid.update(e, 0.0) for e in err])
    _, tf_out = ct.forced_response(linear.pid_discrete(g, dt), T=np.arange(200) * dt, U=err)
    assert np.squeeze(tf_out) == pytest.approx(port, rel=1e-9, abs=1e-9)


def test_lpf2p_port_has_minus_3db_at_cutoff():
    f = Lpf2p(1000.0, 80.0)
    y = [f.apply(math.sin(2 * math.pi * 80.0 * n / 1000.0)) for n in range(4000)]
    assert 20 * math.log10(max(abs(v) for v in y[2000:])) == pytest.approx(-3.0, abs=0.5)


# ---- margins ----

@pytest.mark.parametrize("timing", [DEFAULT_TIMING, PLANNED_TIMING])
def test_margins_match_python_control(timing):
    loop = linear.rate_loop(WHOOP_2IN, timing, "roll", load_gains().roll_rate)
    ours = linear.margins(loop)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        gm, pm, _, _, wgc, _ = ct.stability_margins(loop)
    assert ours.phase_margin_deg == pytest.approx(pm, abs=0.2)
    assert ours.gain_margin_db == pytest.approx(20 * math.log10(gm), abs=0.2)
    assert ours.crossover_hz == pytest.approx(wgc / (2 * math.pi), rel=0.01)


def test_unstable_loop_is_detected():
    loop = linear.rate_loop(WHOOP_2IN, DEFAULT_TIMING, "roll", PidGains(1500.0, 3000.0, 10.0))
    assert not linear.margins(loop).stable


def test_uncertainty_has_32_corners():
    assert len(robust.Uncertainty().corners(WHOOP_2IN, DEFAULT_TIMING)) == 32


# ---- sensors ----

def test_imu_white_noise_matches_datasheet_density():
    spec = replace(Imu(), gyro_bias_initial=0.0, gyro_bias_walk=0.0)
    imu = ImuModel(spec, 4000.0, np.random.default_rng(3))
    samples = np.array([imu.sample(np.zeros(3), np.array([0, 0, G]))[0] for _ in range(20000)])
    expected = spec.gyro_noise_density * math.sqrt(4000.0 / 2.0)
    assert samples.std() == pytest.approx(expected, rel=0.03)


# ---- gate regression ----

def test_stock_firmware_gains_fail_and_planned_timing_passes_nominally():
    """Locks in the key G1 finding so a model change that hides it is noticed."""
    fw = load_gains()
    stock = linear.margins(linear.rate_loop(WHOOP_2IN, DEFAULT_TIMING, "roll", fw.roll_rate))
    planned = linear.margins(linear.rate_loop(WHOOP_2IN, PLANNED_TIMING, "roll", fw.roll_rate))
    assert not stock.passes(robust.GATE_PM_DEG, robust.GATE_GM_DB)
    assert planned.passes(robust.GATE_PM_DEG, robust.GATE_GM_DB)
