"""Replay timing must retain the actual loop cadence, including quarter milliseconds."""

import numpy as np

from visualize import fly_vibration


def test_vibration_replay_retains_4khz_timestamps():
    replay = fly_vibration("test", "#ffffff", False)
    np.testing.assert_allclose(np.diff(replay["t"]), 1 / 4000, atol=1e-9, rtol=0)
    assert len(replay["t"]) == len(replay["pos"]) == len(replay["series"]["artifact"]) == 12000
