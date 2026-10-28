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
    d.takeoff(height, 5.0)  # 4 s hid the fixed 6 s blind budget at 3 m (learned hover differs)
    d.run(6.0, record_hz=None)
    d.imu.accel_bias[2] += accel_bias_z_g
    start = d.quad.state.pos.copy()
    d.tracking_up = False
    d.run(20.0, record_hz=100)
    log = d.log.arrays()
    # Touchdown = motor stop (LANDED) or ground contact, whichever is first. The powered sink speed there and
    # the unpowered drop below it are the physical quantities; a sub-cm drop after the stop is harmless but
    # reads as ~0.4 m/s at contact, so contact speed alone is not the metric.
    contact = np.flatnonzero(log["pos"][:, 2] <= 1e-6)
    cut = np.flatnonzero(log["safety"] == FAILSAFE_LANDED)
    ends = [i for i in (contact[:1].tolist() + cut[:1].tolist()) if i > 0]
    stop = min(ends) if ends else None
    approach = float(max(0., -log["velocity"][stop - 1, 2])) if stop else None
    # Braking height: first height below the 1.3 m ToF range where the sink is under 0.3 m/s. A +0.02 g bias
    # arrives at ~1 m/s; a 1 s settle band left no braking until 0.06 m (4 m, no drag).
    z, vz = log["pos"][:, 2], log["velocity"][:, 2]
    i = int(np.argmax(z <= 1.3))
    brake_z = float(z[i + int(np.argmax(-vz[i:] < 0.3))])
    drop = float(log["pos"][stop, 2]) if stop else None
    return {"brake_z": brake_z, "state": int(log["safety"][-1]), "states": set(log["safety"].astype(int)), "approach": approach,
            "drop": drop,
            "drift": float(np.linalg.norm(log["pos"][:, :2] - start[:2], axis=1).max()), "armed": d.fw.armed,
            "asserts": d.fw.assert_count}


@pytest.mark.parametrize("profile", ["whoop2_interim", "whoop2_4k"])
@pytest.mark.parametrize("seed,bias,height", [(1, 0.0, 2.0), (5, 0.0, 2.0), (9, 0.02, 2.0), (9, -0.02, 2.0),
                                               (9, 0.02, 3.0), (9, -0.02, 3.0), (9, 0.02, 4.0)])
def test_camera_loss_above_tof_range_lands_instead_of_dropping(profile, seed, bias, height):
    r = blind_landing(profile, seed, bias, height)
    assert r["state"] == FAILSAFE_LANDED and FAILSAFE_FAULT not in r["states"], r
    assert not r["armed"] and r["asserts"] == 0
    assert r["approach"] is not None and r["approach"] <= 0.35, r  # powered sink speed at touchdown
    assert r["drop"] <= 0.01, r  # motors stop at most 1 cm above the ground
    assert r["brake_z"] >= 0.5, r  # braked well above the ground after the ToF came back
    assert r["drift"] <= 0.5, r  # level descent plus flow braking once ToF/flow are back in range
