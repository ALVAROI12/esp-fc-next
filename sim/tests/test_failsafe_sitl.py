"""Latched landing checks; truth/motor evidence, not merely a supervisor flag."""

import numpy as np
import pytest

from failsafe_report import fly_loss
from sitl import SitlDrone
from sitl.firmware import FAILSAFE_HOLD, FAILSAFE_LAND, FAILSAFE_LANDED, FAILSAFE_FAULT


@pytest.mark.parametrize("profile", ["whoop2_interim", "whoop2_4k"])
@pytest.mark.parametrize("kind,moving,recovery", [
    ("command", False, None), ("command", True, None), ("tracking", True, None),
    ("both", True, None), ("command", False, 0.8), ("command", True, 2.5),
    ("tracking", True, 0.8), ("height", False, None),
])
def test_loss_and_recovery(profile, kind, moving, recovery):
    jump = (0.5, -0.4, 0.) if kind == "tracking" and recovery is not None else (0., 0., 0.)
    r = fly_loss(profile, kind, moving=moving, recover_after=recovery, tracking_jump=jump)
    assert r["passed"], {k: v for k, v in r.items() if k not in ("drone", "log")}
    states = r["log"]["safety"]
    if kind == "height":
        assert FAILSAFE_FAULT in states and FAILSAFE_LANDED not in states
    elif recovery is None or recovery > 1 or kind == "tracking":
        assert FAILSAFE_LAND in states
        assert np.all(r["log"]["motors"][states == FAILSAFE_LANDED] == 0)
        assert not r["drone"].fw.arm(True)  # restored input never auto-rearms a terminal state
        if kind == "tracking" and recovery is not None:
            restored = r["log"]["t"] - r["loss_time"] > recovery + 0.05
            np.testing.assert_array_equal(r["log"]["attitude_target"][restored], 0)
            assert np.any(r["log"]["position_fresh"][restored])  # actually delivered restored camera
    else:
        assert FAILSAFE_HOLD in states and FAILSAFE_LAND not in states


def test_safe_mode_refuses_to_arm_without_local_height():
    d = SitlDrone(profile="whoop2_4k", safe_landing=True)
    d.height_up = False
    while not d.fw.ready:
        d.step(record=False)
    assert not d.fw.arm(True)


def test_emergency_stop_overrides_landing():
    d = SitlDrone(profile="whoop2_4k", safe_landing=True)
    d.calibrate_and_arm()
    d.takeoff(1, 2)
    d.run(4, record_hz=None)
    d.tracking_up = False
    d.run(0.4, record_hz=None)
    assert d.fw.failsafe_state == FAILSAFE_LAND
    d.fw.emergency_stop()
    d.run(0.02, record_hz=None)
    assert np.all(d.command == 0)
