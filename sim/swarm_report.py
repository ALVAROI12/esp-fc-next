"""Task 1.8: four-drone swarm missions on the real C runtime (one firmware copy per drone), SITL only.

    make swarm
    python sim/swarm_report.py --out /path/to/report

Mission: takeoff to a per-drone altitude layer, square formation, diagonal position swap, landing. Commands are
cflib HighLevelCommander packets flown by each drone's on-board planner, as Crazyswarm2 sends them.
Horizontal moves happen only inside a drone's own layer (vertical deconfliction). Faults are injected
one scenario at a time. Drones do not interact physically: rotor downwash, wake and contact between
vehicles are NOT modeled, so separation here is geometric clearance, not aerodynamic safety.
Low battery is injected as a step in reported cell voltage (no discharge curve or thrust sag).
"""

from __future__ import annotations

import argparse
from itertools import combinations
from pathlib import Path

import numpy as np

from sitl import SitlDrone
from sitl.firmware import FAILSAFE_FAULT, FAILSAFE_LANDED

OUT = Path(__file__).resolve().parent / "reports" / "swarm"
PROFILE = "whoop2_interim"
HALF = 0.6  # half side of the formation square (m)
STARTS = [(-HALF, -HALF), (HALF, -HALF), (HALF, HALF), (-HALF, HALF)]
SWAP = [2, 3, 0, 1]  # diagonal partners
LAYERS = [1.0, 1.35, 1.7, 2.05]  # m; deconfliction altitude per drone
MIN_SEPARATION = 0.30  # m, truth 3D distance below this counts as a collision (2-inch frames + margin)
CHUNK = 0.01  # s, lockstep granularity and log period
MOVE_S = 4.0
APPROACH_MAX = 0.35  # m/s sampled touchdown speed, as failsafe_report.py


class Swarm:
    def __init__(self, seed: int = 0):
        self.drones = [SitlDrone(start=(x, y, 0.0), seed=seed * 10 + i, profile=PROFILE,
                                 safe_landing=True, local_flow=True) for i, (x, y) in enumerate(STARTS)]
        for d in self.drones:
            d.battery_cell_v = 3.9
        self.time = 0.0
        self.pos: list[np.ndarray] = []  # truth, one (4, 3) row per chunk
        self.land_done_at: list[float | None] = [None] * len(self.drones)  # landing trajectory end time
        self.hl = [d.high_level_commander() for d in self.drones]

    def advance(self, seconds: float) -> None:
        for _ in range(int(round(seconds / CHUNK))):
            self.time += CHUNK
            for d in self.drones:
                while d.time < self.time - 1e-9:
                    d.step(record=(d.tick + 1) % round(d.rate * CHUNK) == 0)
            self._ground_station()
            self.pos.append(np.array([d.quad.state.pos for d in self.drones]))

    def _ground_station(self) -> None:
        # Commanded landing: disarm only after the descent trajectory has ended and the drone's own estimate
        # is on the ground. Cutting motors at a few cm drops the vehicle (3 cm free fall = 0.77 m/s).
        for i, d in enumerate(self.drones):
            done = self.land_done_at[i]
            if (done is not None and self.time >= done + 0.3 and d.link_up and d.fw.armed
                    and d.fw.state()[2] < 0.01):
                d.fw.arm(False)

    def start(self) -> None:
        for d in self.drones:
            d.calibrate_and_arm()
        self.time = max(d.time for d in self.drones)
        self.advance(CHUNK)
        for i, hl in enumerate(self.hl):
            hl.takeoff(LAYERS[i], 3.0)
        self.advance(4.0)

    def send_moves(self, targets: list[tuple[float, float]]) -> None:
        for i, hl in enumerate(self.hl):
            hl.go_to(*targets[i], LAYERS[i], 0.0, MOVE_S)

    def move_all(self, targets: list[tuple[float, float]]) -> None:
        self.send_moves(targets)
        self.advance(MOVE_S + 1.5)

    def land_all(self) -> None:
        for i, d in enumerate(self.drones):
            if d.link_up and d.fw.armed:
                z = LAYERS[i]
                self.hl[i].land(0.0, z / 0.3)  # ~0.3 m/s average descent, zero speed at the ground
                self.land_done_at[i] = self.time + z / 0.3
        self.advance(max(LAYERS) / 0.3 + 3.0)


def drone_outcome(d: SitlDrone, faulted: bool) -> dict:
    log = d.log.arrays()
    states = log["safety"].astype(int)
    contact = np.flatnonzero(log["pos"][:, 2] <= 1e-6)
    airborne = np.flatnonzero(log["pos"][:, 2] > 0.05)
    # First ground contact after the flight (not the pre-takeoff samples).
    contact = contact[contact > airborne[0]] if len(airborne) else contact[:0]
    approach = float(max(0., -log["velocity"][contact[0] - 1, 2])) if len(contact) else None
    landed_ok = (not d.fw.armed and np.all(d.command == 0) and approach is not None and approach <= APPROACH_MAX
                 and FAILSAFE_FAULT not in states and not d.fw.tumbled and d.fw.assert_count == 0)
    if faulted:
        landed_ok = landed_ok and int(states[-1]) == FAILSAFE_LANDED
    return {"approach": approach, "terminal": int(states[-1]), "safe_landing": bool(landed_ok)}


def fly_mission(fault: str | None = None, fault_drone: int = 0, fault_phase: str = "hover", seed: int = 0) -> dict:
    """fault: None, "command" (link loss), "tracking" (mocap dropout) or "battery" (cell voltage drops below
    the firmware threshold); fault_phase: "hover" or "swap"."""
    s = Swarm(seed)
    s.start()
    s.move_all(STARTS)  # formation square at layer altitudes

    def inject() -> None:
        d = s.drones[fault_drone]
        if fault == "command":
            d.link_up = False
        elif fault == "tracking":
            d.tracking_up = False
        elif fault == "battery":
            d.battery_cell_v = 3.2

    swap_targets = [STARTS[j] for j in SWAP]
    if fault and fault_phase == "swap":
        s.send_moves(swap_targets)
        s.advance(MOVE_S / 2)  # mid-transit
        inject()
        s.advance(MOVE_S / 2 + 1.5)
    else:
        s.move_all(swap_targets)
        if fault:
            inject()
            s.advance(1.0)
    if fault:
        s.advance(12.0)  # failsafe hold + landing while the others keep station
    s.land_all()

    pos = np.array(s.pos)
    min_sep, pair = np.inf, None
    for a, b in combinations(range(len(s.drones)), 2):
        sep = np.linalg.norm(pos[:, a] - pos[:, b], axis=1)
        if sep.min() < min_sep:
            min_sep, pair = float(sep.min()), (a, b)
    outcomes = [drone_outcome(d, fault is not None and i == fault_drone) for i, d in enumerate(s.drones)]
    passed = min_sep >= MIN_SEPARATION and all(o["safe_landing"] for o in outcomes)
    return {"fault": fault, "fault_drone": fault_drone, "fault_phase": fault_phase, "min_separation": min_sep,
            "closest_pair": pair, "outcomes": outcomes, "passed": bool(passed), "swarm": s}


SCENARIOS = [
    ("nominal mission", {}),
    ("link loss in formation (drone 0)", {"fault": "command", "fault_drone": 0}),
    ("mocap dropout in formation (drone 1)", {"fault": "tracking", "fault_drone": 1}),
    ("link loss mid-swap, top layer (drone 3)", {"fault": "command", "fault_drone": 3, "fault_phase": "swap"}),
    ("mocap dropout mid-swap, top layer (drone 3)", {"fault": "tracking", "fault_drone": 3, "fault_phase": "swap"}),
    ("low battery in formation (drone 2)", {"fault": "battery", "fault_drone": 2}),
    ("low battery mid-swap, top layer (drone 3)", {"fault": "battery", "fault_drone": 3, "fault_phase": "swap"}),
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, kwargs in SCENARIOS:
        print(name, flush=True)
        r = fly_mission(**kwargs)
        landings = ", ".join(f"{'ok' if o['safe_landing'] else 'FAIL'} ({o['approach']:.2f} m/s)"
                             if o["approach"] is not None else "FAIL (no touchdown)" for o in r["outcomes"])
        rows.append(f"| {name} | {r['min_separation']:.2f} m (drones {r['closest_pair'][0]}-{r['closest_pair'][1]}) "
                    f"| {landings} | {'PASS' if r['passed'] else 'FAIL'} |")
        rows_passed = r["passed"]
        print(rows[-1], flush=True)
        if not rows_passed:
            print("  outcomes:", r["outcomes"], flush=True)
    passed = all("| PASS |" in row for row in rows)
    lines = ["# Four-drone swarm missions (Task 1.8, SITL)", "",
             f"**Checks: {'PASS' if passed else 'FAIL'}**", "",
             f"4 drones, profile `{PROFILE}`, opt-in safe mode with local flow/ToF, one real C firmware copy each. "
             f"Mission: takeoff to layers {LAYERS} m, {2 * HALF:.1f} m square formation, diagonal swap, landing. "
             f"Collision = truth 3D separation < {MIN_SEPARATION} m. Safe landing = disarmed, motors off, sampled "
             f"touchdown <= {APPROACH_MAX} m/s, no FAULT/tumble/assert; faulted drones must end LANDED.", "",
             "| Scenario | Min separation | Landings (drone 0-3, touchdown speed) | Result |",
             "| --- | --- | --- | --- |", *rows, "",
             "## Limits", "",
             "- No inter-vehicle aerodynamics: downwash on a lower drone, wake and contact are not modeled. Layer "
             "spacing of 0.35 m is a geometric plan, not a validated downwash clearance.",
             "- A faulted drone descends through lower layers; it is safe here only because the others are not "
             "below it at that moment. The ground station does not yet replan around a landing drone.",
             "- Low battery is a voltage step only (no discharge curve, sag or reduced thrust); radio is ideal "
             "apart from the injected link loss; "
             "ESP-NOW timing and the P4<->C5 link are not in the loop. Commands are cflib HighLevelCommander packets "
             "into each drone's upstream planner, not a running Crazyswarm2/ROS 2 stack.",
             "- All-height loss still FAULTs (motor stop) and is excluded from safe-landing claims.", ""]
    (args.out / "README.md").write_text("\n".join(lines))
    print(f"Report: {args.out / 'README.md'} ({'PASS' if passed else 'FAIL'})")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
