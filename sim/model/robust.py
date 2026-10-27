"""Robustness analysis: parameter uncertainty corners, Monte Carlo, and robust gain search."""

from __future__ import annotations

from dataclasses import dataclass, replace
import itertools
import math

import numpy as np

from . import linear
from .firmware import AttitudeGains, PidGains
from .params import LoopTiming, Vehicle

# G1 gate thresholds (project plan section 6) and the stricter design targets used for tuning.
GATE_PM_DEG = 45.0
GATE_GM_DB = 6.0
DESIGN_PM_DEG = 50.0
DESIGN_GM_DB = 8.0


@dataclass(frozen=True)
class Uncertainty:
    """Relative +/- ranges (0.2 = +/-20 %) and absolute ranges for timing."""

    mass: float = 0.20
    thrust_max: float = 0.20
    inertia: float = 0.20
    motor_tau: tuple[float, float] = (0.015, 0.035)  # s
    extra_delay_samples: tuple[int, int] = (0, 1)

    def corners(self, vehicle: Vehicle, timing: LoopTiming) -> list[tuple[Vehicle, LoopTiming]]:
        out = []
        for sm, st, si, tau, d in itertools.product(
            (-1, 1), (-1, 1), (-1, 1), self.motor_tau, self.extra_delay_samples
        ):
            fi = 1 + si * self.inertia
            v = replace(
                vehicle.scaled(mass=1 + sm * self.mass, thrust_max=1 + st * self.thrust_max, ixx=fi, iyy=fi, izz=fi),
                motor_tau=tau,
            )
            out.append((v, replace(timing, compute_delay_samples=timing.compute_delay_samples + d)))
        return out

    def sample(self, vehicle: Vehicle, timing: LoopTiming, rng: np.random.Generator) -> tuple[Vehicle, LoopTiming]:
        u = lambda r: 1 + rng.uniform(-r, r)  # noqa: E731
        v = replace(
            vehicle.scaled(
                mass=u(self.mass), thrust_max=u(self.thrust_max),
                ixx=u(self.inertia), iyy=u(self.inertia), izz=u(self.inertia),
            ),
            motor_tau=rng.uniform(*self.motor_tau),
        )
        d = int(rng.integers(self.extra_delay_samples[0], self.extra_delay_samples[1] + 1))
        return v, replace(timing, compute_delay_samples=timing.compute_delay_samples + d)


LOOPS = ("roll_rate", "pitch_rate", "yaw_rate", "roll", "pitch", "yaw")


def evaluate(gains: AttitudeGains, vehicle: Vehicle, timing: LoopTiming) -> dict[str, linear.Margins]:
    """Margins of all six loops (three rate loops and three attitude loops)."""
    out = {}
    for axis in linear.AXES:
        rate_g = getattr(gains, f"{axis}_rate")
        out[f"{axis}_rate"] = linear.margins(linear.rate_loop(vehicle, timing, axis, rate_g))
        out[axis] = linear.margins(linear.attitude_loop(vehicle, timing, axis, rate_g, getattr(gains, axis)))
    return out


@dataclass
class MonteCarloResult:
    samples: int
    worst_pm: dict[str, float]
    worst_gm: dict[str, float]
    pm: dict[str, np.ndarray]
    gm: dict[str, np.ndarray]
    failures: int  # samples where any loop misses the gate

    @property
    def passed(self) -> bool:
        return self.failures == 0


def monte_carlo(
    gains: AttitudeGains, vehicle: Vehicle, timing: LoopTiming, unc: Uncertainty, samples: int = 1000, seed: int = 1
) -> MonteCarloResult:
    rng = np.random.default_rng(seed)
    pm = {k: np.empty(samples) for k in LOOPS}
    gm = {k: np.empty(samples) for k in LOOPS}
    failures = 0
    for n in range(samples):
        v, t = unc.sample(vehicle, timing, rng)
        result = evaluate(gains, v, t)
        ok = True
        for k, m in result.items():
            pm[k][n] = m.phase_margin_deg if m.stable else -math.inf
            gm[k][n] = m.gain_margin_db if m.stable else -math.inf
            ok &= m.passes(GATE_PM_DEG, GATE_GM_DB)
        failures += 0 if ok else 1
    return MonteCarloResult(
        samples, {k: float(pm[k].min()) for k in LOOPS}, {k: float(gm[k].min()) for k in LOOPS}, pm, gm, failures
    )


def _worst(cases, build_loop) -> tuple[float, float, float]:
    """Worst-case (PM, GM, crossover) of a loop over uncertainty corners."""
    worst_pm, worst_gm, worst_fc = math.inf, math.inf, math.inf
    for v, t in cases:
        m = linear.margins(build_loop(v, t))
        if not m.stable:
            return -math.inf, -math.inf, 0.0
        worst_pm = min(worst_pm, m.phase_margin_deg)
        worst_gm = min(worst_gm, m.gain_margin_db)
        worst_fc = min(worst_fc, m.crossover_hz)
    return worst_pm, worst_gm, worst_fc


def tune_rate(
    axis: str, vehicle: Vehicle, timing: LoopTiming, unc: Uncertainty, template: PidGains,
    kp_grid: np.ndarray, kd_grid: np.ndarray, ki_ratio: float,
) -> tuple[PidGains, tuple[float, float, float]]:
    """Maximize worst-case rate-loop crossover subject to worst-case design margins."""
    cases = unc.corners(vehicle, timing)
    best, best_score = None, (-math.inf,) * 3
    for kp in kp_grid:
        for kd in kd_grid:
            g = PidGains(float(kp), float(kp * ki_ratio), float(kd), template.i_limit)
            score = _worst(cases, lambda v, t: linear.rate_loop(v, t, axis, g))
            if score[0] >= DESIGN_PM_DEG and score[1] >= DESIGN_GM_DB and score[2] > best_score[2]:
                best, best_score = g, score
    if best is None:
        raise RuntimeError(f"no {axis} rate gains meet the design margins on all corners")
    _check_interior(f"{axis} rate kp", best.kp, kp_grid)
    if len(kd_grid) > 1:
        _check_interior(f"{axis} rate kd", best.kd, kd_grid)
    return best, best_score


GRID_EDGE_HITS: list[str] = []


def _check_interior(name: str, value: float, grid: np.ndarray) -> None:
    """Record gains chosen at the top edge of the search grid: the optimum may lie beyond it."""
    if math.isclose(value, float(np.max(grid)), rel_tol=1e-9):
        GRID_EDGE_HITS.append(f"{name} = {value:.3g} is at the top of its search grid")


def tune_attitude(
    axis: str, vehicle: Vehicle, timing: LoopTiming, unc: Uncertainty, rate_gains: PidGains, template: PidGains,
    kp_grid: np.ndarray, ki_ratio: float, max_overshoot_pct: float = 10.0,
) -> tuple[PidGains, tuple[float, float, float]]:
    """Largest attitude kp meeting worst-case design margins and the nominal overshoot limit."""
    cases = unc.corners(vehicle, timing)
    best, best_score = None, (-math.inf,) * 3
    for kp in kp_grid:
        g = PidGains(float(kp), float(kp * ki_ratio), template.kd, template.i_limit)
        score = _worst(cases, lambda v, t: linear.attitude_loop(v, t, axis, rate_gains, g))
        if score[0] < DESIGN_PM_DEG or score[1] < DESIGN_GM_DB:
            continue
        loop = linear.attitude_loop(vehicle, timing, axis, rate_gains, g)
        if linear.step_metrics(linear.ct.feedback(loop, 1)).overshoot_pct > max_overshoot_pct:
            continue
        if score[2] > best_score[2]:
            best, best_score = g, score
    if best is None:
        raise RuntimeError(f"no {axis} attitude gains meet the design margins")
    _check_interior(f"{axis} attitude kp", best.kp, kp_grid)
    return best, best_score
