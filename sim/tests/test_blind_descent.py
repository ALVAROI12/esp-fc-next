"""Camera lost above the VL53L1X short-mode range (1.3 m): bounded blind descent until the ToF reacquires.

Before this behaviour the supervisor cut motors at once (about 6 m/s impact from 2 m). The extra accel
bias is injected after calibration (unmodeled drift), so the IMU-propagated descent is not given truth.
"""

import numpy as np
import pytest

from sitl import SitlDrone
from sitl.firmware import FAILSAFE_FAULT, FAILSAFE_LANDED


def blind_landing(profile: str, seed: int, accel_bias_z_g: float = 0.0, height: float = 2.0) -> dict:
    d = SitlDrone(seed=seed, profile=profile, safe_landing=True, local_flow=True)
    d.calibrate_and_arm()
    d.takeoff(height, 4.0)
    d.run(6.0, record_hz=None)
    d.imu.accel_bias[2] += accel_bias_z_g
    start = d.quad.state.pos.copy()
    d.tracking_up = False
    d.run(16.0, record_hz=100)
    log = d.log.arrays()
    contact = np.flatnonzero(log["pos"][:, 2] <= 1e-6)
    approach = float(max(0., -log["velocity"][contact[0] - 1, 2])) if len(contact) and contact[0] else None
    return {"state": int(log["safety"][-1]), "states": set(log["safety"].astype(int)), "approach": approach,
            "drift": float(np.linalg.norm(log["pos"][:, :2] - start[:2], axis=1).max()), "armed": d.fw.armed,
            "asserts": d.fw.assert_count}


@pytest.mark.parametrize("profile", ["whoop2_interim", "whoop2_4k"])
@pytest.mark.parametrize("seed,bias", [(1, 0.0), (5, 0.0), (9, 0.02), (9, -0.02)])
def test_camera_loss_above_tof_range_lands_instead_of_dropping(profile, seed, bias):
    r = blind_landing(profile, seed, bias)
    assert r["state"] == FAILSAFE_LANDED and FAILSAFE_FAULT not in r["states"], r
    assert not r["armed"] and r["asserts"] == 0
    assert r["approach"] is not None and r["approach"] <= 0.35, r
    assert r["drift"] <= 0.5, r  # level descent plus flow braking once ToF/flow are back in range
