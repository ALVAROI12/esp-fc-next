"""Deterministic synthetic gyro-artifact checks, not physical validation."""

from dataclasses import FrozenInstanceError, replace
import math

import numpy as np
import pytest

from model.vibration import MotorVibration, VibrationSpec, notch_response


def series(model, rotor, count=4000, rate=4000.0):
    return np.array([model.sample(rotor, 1.0 / rate) for _ in range(count)])


def test_spec_defaults_are_frozen_and_explicitly_assumed():
    spec = VibrationSpec()
    assert spec.max_rpm == 60000.0
    assert spec.amplitude_dps == 12.0
    assert spec.harmonic_ratio == 0.35
    assert spec.motor_scale == (0.97, 1.0, 1.02, 1.05)
    assert "assumed" in spec.source.lower()
    assert "hardware blackbox" in spec.source.lower()
    with pytest.raises(FrozenInstanceError):
        spec.max_rpm = 1.0


def test_reproducible_seed_and_independent_random_stream():
    rotor = [0.2, 0.4, 0.7, 1.0]
    rng = np.random.default_rng(42)
    expected = np.random.default_rng(42).normal(size=10)
    legacy_state = np.random.get_state()
    a = series(MotorVibration(seed=7), rotor, count=100)
    b = series(MotorVibration(seed=7), rotor, count=100)
    c = series(MotorVibration(seed=8), rotor, count=100)
    np.testing.assert_array_equal(a, b)
    assert not np.array_equal(a, c)
    np.testing.assert_array_equal(rng.normal(size=10), expected)
    after = np.random.get_state()
    assert after[0] == legacy_state[0]
    np.testing.assert_array_equal(after[1], legacy_state[1])
    assert after[2:] == legacy_state[2:]


def test_stopped_motors_are_exactly_zero_and_do_not_advance_phase():
    model = MotorVibration(seed=3)
    initial = model.sample(np.ones(4), 0.0)
    for _ in range(20):
        np.testing.assert_array_equal(model.sample(np.zeros(4), 0.01), np.zeros(3))
    np.testing.assert_array_equal(model.sample(np.ones(4), 0.0), initial)


def test_rpm_mapping_clips_without_mutating_input_or_model():
    model = MotorVibration(seed=2)
    rotor = np.array([-0.2, 0.25, 0.5, 1.2])
    original = rotor.copy()
    before = model.sample(np.ones(4), 0.0)
    rpm = model.rpm(rotor)
    np.testing.assert_allclose(rpm, [0.0, 15000.0, 30600.0, 63000.0])
    assert rpm.shape == (4,)
    rpm[:] = -123.0
    np.testing.assert_array_equal(model.rpm(rotor), [0.0, 15000.0, 30600.0, 63000.0])
    np.testing.assert_array_equal(rotor, original)
    np.testing.assert_array_equal(model.sample(np.ones(4), 0.0), before)
    np.testing.assert_array_equal(
        model.sample(rotor, 0.001), MotorVibration(seed=2).sample(np.clip(rotor, 0.0, 1.0), 0.001)
    )


@pytest.mark.parametrize("motor", range(4))
def test_single_motor_has_fundamental_and_second_harmonic_at_4khz(motor):
    spec = VibrationSpec()
    rotor = np.zeros(4)
    rotor[motor] = 0.5
    samples = series(MotorVibration(spec, seed=9), rotor)
    spectrum = 2.0 * np.abs(np.fft.rfft(samples[:, 0])) / len(samples)
    fundamental = round(0.5 * spec.max_rpm * spec.motor_scale[motor] / 60.0)
    expected_peak = spec.amplitude_dps * 0.5**2 / math.sqrt(3.0)
    assert spectrum[fundamental] == pytest.approx(expected_peak, rel=1e-10)
    assert spectrum[2 * fundamental] == pytest.approx(expected_peak * spec.harmonic_ratio, rel=1e-10)
    assert set(np.argsort(spectrum)[-2:]) == {fundamental, 2 * fundamental}


def test_frequency_follows_motor_lagged_acceleration():
    model = MotorVibration(replace(VibrationSpec(), harmonic_ratio=0.0), seed=4)
    rate = 4000.0
    speeds = np.full(4000, 0.25)
    # Supply actual rotor speed, not command: a first-order acceleration to 0.75.
    speeds[1000:] = 0.75 - 0.5 * np.exp(-np.arange(1, 3001) / (rate * 0.02))
    output = np.array([model.sample([0.0, speed, 0.0, 0.0], 1.0 / rate)[0] for speed in speeds])
    frequencies = np.fft.rfftfreq(1000, 1.0 / rate)
    first = frequencies[np.argmax(np.abs(np.fft.rfft(output[:1000])))]
    last = frequencies[np.argmax(np.abs(np.fft.rfft(output[-1000:])))]
    assert first == pytest.approx(250.0, abs=4.0)
    assert last == pytest.approx(750.0, abs=4.0)
    # Independently integrate every supplied speed; no reset on acceleration.
    initial_phase = np.random.default_rng(4).uniform(0.0, math.tau, 4)[1]
    phase = initial_phase + math.tau * np.cumsum(speeds * 1000.0 / rate)
    expected = -model.spec.amplitude_dps * speeds**2 * np.sin(phase) / math.sqrt(3.0)
    np.testing.assert_allclose(output, expected, atol=1e-9)


def test_amplitude_scales_with_rotor_squared_at_matching_phase():
    slow, fast = MotorVibration(seed=5), MotorVibration(seed=5)
    # Half the speed for twice the interval gives the same phases, 1/4 amplitude.
    for _ in range(100):
        low = slow.sample([0.0, 0.4, 0.0, 0.0], 0.0005)
        high = fast.sample([0.0, 0.8, 0.0, 0.0], 0.00025)
        np.testing.assert_allclose(low, high * 0.25, atol=1e-12)


def test_finite_bounded_output_for_one_second_and_fresh_samples():
    model = MotorVibration()
    samples = series(model, np.ones(4))
    assert samples.shape == (4000, 3)
    assert np.all(np.isfinite(samples))
    bound = 4.0 * model.spec.amplitude_dps * (1.0 + model.spec.harmonic_ratio) / math.sqrt(3.0)
    assert np.max(np.abs(samples)) <= bound
    sample = model.sample(np.ones(4), 0.0)
    expected = sample.copy()
    sample[:] = 1e10
    np.testing.assert_array_equal(model.sample(np.ones(4), 0.0), expected)


def test_above_nyquist_frequency_aliases_instead_of_being_clamped():
    model = MotorVibration(replace(VibrationSpec(), harmonic_ratio=0.0))
    samples = series(model, [0.0, 0.8, 0.0, 0.0], count=1000, rate=1000.0)
    spectrum = np.abs(np.fft.rfft(samples[:, 0]))
    assert np.argmax(spectrum) == 200  # 800 Hz sampled at 1 kHz aliases to 200 Hz.


@pytest.mark.parametrize("rotor", [0.5, [], [0.0] * 3, [0.0] * 5, [[0.0] * 4],
                                   [0.0, np.nan, 0.0, 0.0], [0.0, np.inf, 0.0, 0.0],
                                   [0.0, -np.inf, 0.0, 0.0], ["bad"] * 4])
def test_invalid_rotor_rejected_without_advancing_state(rotor):
    model = MotorVibration()
    before = model.sample(np.ones(4), 0.0)
    with pytest.raises(ValueError, match="rotor"):
        model.rpm(rotor)
    with pytest.raises(ValueError, match="rotor"):
        model.sample(rotor, 0.01)
    np.testing.assert_array_equal(model.sample(np.ones(4), 0.0), before)


@pytest.mark.parametrize("dt", [-0.001, np.nan, np.inf, -np.inf, [0.001], None, "bad"])
def test_invalid_dt_rejected_without_advancing_state(dt):
    model = MotorVibration()
    before = model.sample(np.ones(4), 0.0)
    with pytest.raises(ValueError, match="dt"):
        model.sample(np.ones(4), dt)
    np.testing.assert_array_equal(model.sample(np.ones(4), 0.0), before)


@pytest.mark.parametrize("name, value", [
    ("max_rpm", 0.0), ("max_rpm", -1.0), ("max_rpm", np.nan), ("max_rpm", np.inf),
    ("amplitude_dps", -1.0), ("amplitude_dps", np.nan), ("amplitude_dps", np.inf),
    ("harmonic_ratio", -0.1), ("harmonic_ratio", np.nan), ("harmonic_ratio", np.inf),
    ("motor_scale", (1.0,) * 3), ("motor_scale", (1.0, 1.0, 1.0, np.nan)),
    ("motor_scale", (1.0, 1.0, 1.0, np.inf)), ("motor_scale", (1.0, 1.0, 1.0, -1.0)),
    ("motor_scale", (1.0, 1.0, 1.0, 0.0)),
    ("max_rpm", "bad"), ("amplitude_dps", None), ("harmonic_ratio", [0.35]),
    ("motor_scale", ("bad",) * 4),
])
def test_invalid_spec_rejected(name, value):
    with pytest.raises(ValueError, match=name):
        replace(VibrationSpec(), **{name: value})


def test_zero_amplitude_and_zero_harmonic_are_valid():
    spec = replace(VibrationSpec(), amplitude_dps=0.0, harmonic_ratio=0.0)
    np.testing.assert_array_equal(MotorVibration(spec).sample(np.ones(4), 0.001), np.zeros(3))


def test_notch_response_is_transparent_in_control_band():
    """Phase/gain added near the attitude loop bandwidth must be negligible."""
    response = notch_response([30000, 31000, 32000, 33000], [1.0, 5.0, 10.0, 20.0], sample_hz=4000.0, q=5.0)
    assert np.all(np.abs(np.abs(response) - 1.0) < 2e-3)
    # Eight Q=5 notches still add a few degrees at the top of the control band;
    # the margins experiment in sim/vibration_report.py includes this lag.
    assert np.all(np.abs(np.angle(response[:3], deg=True)) < 1.5)
    assert abs(np.angle(response[3], deg=True)) < 3.0


def test_notch_response_rejects_rotation_and_harmonic():
    response = notch_response([30000, 0, 0, 0], [500.0, 1000.0, 250.0], sample_hz=4000.0, q=5.0)
    assert abs(response[0]) < 1e-3
    assert abs(response[1]) < 1e-3
    assert abs(response[2]) > 0.9  # no target there


def test_notch_response_mirrors_firmware_band_limits():
    # 400 Hz fundamental is skipped; its 800 Hz harmonic remains active.
    response = notch_response([24000, 0, 0, 0], [400.0, 800.0], sample_hz=4000.0, q=5.0)
    assert abs(response[0]) > 0.98  # only the 800 Hz harmonic's skirt remains
    assert abs(response[1]) < 1e-2
    # All sections below the floor, and every section at 1 kHz, must bypass exactly.
    np.testing.assert_array_equal(notch_response([14999] * 4, [100, 200, 400]), np.ones(3))
    np.testing.assert_array_equal(notch_response([30000] * 4, [100, 200, 400], sample_hz=1000), np.ones(3))
    # 2000 Hz at 4 kHz is at Nyquist and must never be filtered (aliasing risk).
    assert abs(notch_response([120000, 0, 0, 0], [100.0], sample_hz=4000.0, q=5.0)[0]) == pytest.approx(1.0)


@pytest.mark.parametrize("rpm, hz, q", [
    ([1, 2, 3], 10.0, 5.0), ([1, 2, 3, np.nan], 10.0, 5.0), ([-1, 0, 0, 0], 10.0, 5.0),
    ([1, 2, 3, 4], -1.0, 5.0), ([1, 2, 3, 4], 10.0, 0.5), ([1, 2, 3, 4], 10.0, 51.0),
])
def test_notch_response_validates_inputs(rpm, hz, q):
    with pytest.raises(ValueError):
        notch_response(rpm, [hz], sample_hz=4000.0, q=q)
