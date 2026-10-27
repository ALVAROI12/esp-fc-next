"""Task 1.8: four real firmware copies fly the swarm mission; faults end in safe landings."""

from swarm_report import MIN_SEPARATION, fly_mission
from sitl.firmware import FAILSAFE_LANDED


def test_nominal_mission_lands_all_without_conflict():
    r = fly_mission()
    assert r["min_separation"] >= MIN_SEPARATION
    assert all(o["safe_landing"] for o in r["outcomes"])
    assert r["passed"]


def test_link_loss_in_formation_lands_faulted_drone_and_finishes_mission():
    r = fly_mission(fault="command", fault_drone=0)
    assert r["outcomes"][0]["terminal"] == FAILSAFE_LANDED
    assert all(o["safe_landing"] for o in r["outcomes"])
    assert r["passed"]
