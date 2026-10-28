"""Crazyswarm2-style missions: cflib's HighLevelCommander packets drive the upstream on-board planner in the SITL.

The packets are built by cflib itself (protocol version 4, what ESP-Drone reports), decoded by the subtree's
crtp_commander_high_level.c and forwarded to fc_step as target/fc_runtime.c does (host/src/fc_hl.c).
"""

import math

import numpy as np
import pytest

from sitl import SitlDrone
from sitl.firmware import FAILSAFE_LANDED

pytest.importorskip("cflib")
from cflib.crazyflie.high_level_commander import HighLevelCommander  # noqa: E402


def airborne(**kw) -> tuple[SitlDrone, HighLevelCommander]:
    d = SitlDrone(seed=4, profile="whoop2_4k", **kw)
    hl = d.high_level_commander()
    d.calibrate_and_arm()
    hl.takeoff(1.0, 2.0)
    d.run(3.0, record_hz=None)
    return d, hl


def test_takeoff_goto_land_mission():
    d, hl = airborne()
    assert d.quad.state.pos == pytest.approx([0, 0, 1.0], abs=0.05)

    hl.go_to(1.0, 0.5, 1.2, math.pi / 2, 3.0)
    d.run(4.0, record_hz=100)
    assert d.quad.state.pos == pytest.approx([1.0, 0.5, 1.2], abs=0.05)
    assert d.log.arrays()["euler"][-1][2] == pytest.approx(90, abs=3)  # yaw is radians on the wire
    peak = np.max(np.linalg.norm(d.log.arrays()["pos"] - [0, 0, 1.0], axis=1))
    assert peak < np.linalg.norm([1.0, 0.5, 0.2]) + 0.05  # no overshoot past the goal

    hl.go_to(-0.5, 0, 0, 0, 2.0, relative=True)
    d.run(3.0, record_hz=None)
    assert d.quad.state.pos == pytest.approx([0.5, 0.5, 1.2], abs=0.05)

    hl.land(0.0, 3.0)
    d.run(3.5, record_hz=None)
    assert d.quad.state.pos[2] < 0.05
    assert d.fw.assert_count == 0 and not d.fw.tumbled


def test_planner_stops_after_landing_and_the_watchdog_cuts_the_motors():
    d, hl = airborne()
    hl.land(0.0, 2.0)
    d.run(2.5, record_hz=None)
    assert not d.fw.hl_step()  # landing finished: the planner stops and forwards nothing
    d.run(2.5, record_hz=None)  # fc_step's 2 s command watchdog
    assert np.all(d.command == 0)


def test_ground_link_loss_during_a_goto_lands_instead_of_flying_on():
    """The planner runs on board; without the link fc_step's failsafe must take over, not the trajectory."""
    d, hl = airborne(safe_landing=True)
    hl.go_to(1.5, 0, 1.0, 0, 4.0)
    d.run(1.0, record_hz=None)
    d.link_up = False
    d.run(12.0, record_hz=None)
    assert d.fw.failsafe_state == FAILSAFE_LANDED
    assert d.quad.state.pos[0] < 1.0  # stopped short of the goal: no longer following the planner
    assert d.quad.state.pos[2] < 0.05


def test_stop_and_unknown_commands():
    d, hl = airborne()
    assert d.fw.hl_packet(bytes([99])) != 0  # ENOEXEC, as the firmware answers on the CRTP port
    hl.stop()
    assert not d.fw.hl_step()
