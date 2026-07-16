# SPDX-License-Identifier: AGPL-3.0-only
"""The inverse-calibration optimiser: fit tyre wear/grip parameters to a stint pace curve.

Mirrors the ``outlap.tirefit`` shape — a documented free-parameter init/bounds table, a
``residual(x)`` closure, and ``scipy.optimize.least_squares`` (``trf`` + ``soft_l1``) imported
lazily so scipy stays confined to the ``wear-cal`` / ``tire-fit`` extra. The forward model is the
reduced-order :func:`outlap.wearcal.model.stint_lap_times` surrogate (fast enough for a fit); a
faithful-but-slow variant that wraps the real Rust stint driver lives in :mod:`outlap.wearcal.sim`.

Wide-range parameters (``k_w``, ``tau_d``) are fitted in log₁₀ space for conditioning; the residual
compares simulated vs observed lap times in seconds, with the fresh-lap anchor ``t_ref`` set to the
stint's fastest lap so the fit is about the *shape* of the decay, not an absolute offset.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray

from .data import StintObservation
from .model import StintAnchor, SurrogateParams, stint_trace

F = NDArray[np.float64]


@dataclass(frozen=True)
class CalibParam:
    """One free parameter: the :class:`SurrogateParams` field, its init, bounds, and scaling."""

    name: str
    init: float
    lo: float
    hi: float
    log: bool = False  # fit in log10 space (wide-range magnitudes: k_w, tau_d)


#: The documented calibration table. Bounds bracket physically-plausible racing-slick values; the
#: default free set (the wear/cliff shape) is the subset that governs stint degradation.
CALIB_TABLE: dict[str, CalibParam] = {
    "k_w": CalibParam("k_w", 3.0e-9, 1.0e-11, 1.0e-6, log=True),
    "w_c": CalibParam("w_c", 2.0, 0.3, 6.0),
    "s_w": CalibParam("s_w", 0.5, 0.1, 2.5),
    "delta_c": CalibParam("delta_c", 0.15, 0.0, 0.6),
    "t_opt": CalibParam("t_opt", 95.0, 70.0, 120.0),
    "c_t": CalibParam("c_t", 2.2, 0.5, 6.0),
    "delta_d": CalibParam("delta_d", 0.30, 0.0, 0.6),
    "t_deg": CalibParam("t_deg", 120.0, 90.0, 160.0),
    "tau_d": CalibParam("tau_d", 600.0, 60.0, 6000.0, log=True),
    "beta": CalibParam("beta", 2.0, 1.0, 4.0),
}

#: The default parameters to free — the wear/cliff shape that drives stint pace loss (§7.3).
DEFAULT_FREE: tuple[str, ...] = ("k_w", "w_c", "s_w", "delta_c")


@dataclass(frozen=True)
class CalibConfig:
    """What to fit and against what operating point.

    ``free`` names the :data:`CALIB_TABLE` parameters to optimise; ``base`` provides the fixed
    values for everything else; ``anchor`` is the car/track operating point (its ``t_ref_s`` is
    overridden with the observation's fastest lap unless ``anchor_t_ref`` is False).
    """

    free: tuple[str, ...] = DEFAULT_FREE
    base: SurrogateParams = field(default_factory=SurrogateParams)
    anchor: StintAnchor = field(default_factory=StintAnchor)
    anchor_t_ref: bool = True

    def __post_init__(self) -> None:
        unknown = [n for n in self.free if n not in CALIB_TABLE]
        if unknown:
            raise ValueError(f"unknown calibration parameters: {unknown}")
        if not self.free:
            raise ValueError("at least one parameter must be free")


@dataclass(frozen=True)
class CalibResult:
    """The outcome of a fit: recovered parameters, residuals, and decay diagnostics."""

    params: SurrogateParams
    free: tuple[str, ...]
    fitted: dict[str, float]
    rms_s: float
    max_abs_s: float
    n_laps: int
    decay_s_per_lap: float
    cliff_lap: int | None
    sim_lap_time_s: F
    obs_lap_time_s: F
    success: bool

    @property
    def total_loss_s(self) -> float:
        """Fastest-to-slowest lap-time spread in the fitted stint (the total degradation)."""
        return float(np.max(self.sim_lap_time_s) - np.min(self.sim_lap_time_s))


def _encode(free: Sequence[CalibParam], p: SurrogateParams) -> F:
    """Pack the free parameters into the optimiser vector (log10 where flagged)."""
    return np.array(
        [np.log10(getattr(p, q.name)) if q.log else getattr(p, q.name) for q in free],
        dtype=np.float64,
    )


def _bounds(free: Sequence[CalibParam]) -> tuple[F, F]:
    """Lower/upper optimiser bounds (log10 where flagged)."""
    lo = np.array([np.log10(q.lo) if q.log else q.lo for q in free], dtype=np.float64)
    hi = np.array([np.log10(q.hi) if q.log else q.hi for q in free], dtype=np.float64)
    return lo, hi


def _apply(free: Sequence[CalibParam], base: SurrogateParams, x: F) -> SurrogateParams:
    """Reconstruct :class:`SurrogateParams` from the optimiser vector."""
    changes = {
        q.name: float(10.0**xi if q.log else xi) for q, xi in zip(free, x, strict=True)
    }
    return base.replace(**changes)


def calibrate(obs: StintObservation, config: CalibConfig | None = None) -> CalibResult:
    """Fit the free wear/grip parameters so the surrogate stint matches ``obs`` lap times."""
    cfg = config or CalibConfig()
    least_squares = _scipy_least_squares()
    free = [CALIB_TABLE[n] for n in cfg.free]
    anchor = cfg.anchor
    if cfg.anchor_t_ref:
        anchor = StintAnchor(
            t_ref_s=float(np.min(obs.lap_time_s)),
            p_pace=anchor.p_pace,
            phi_wear=anchor.phi_wear,
            t_op_c=anchor.t_op_c,
            t_c_c=anchor.t_c_c,
            lap_time_s=anchor.lap_time_s,
            warm_up_laps=anchor.warm_up_laps,
            t_s0_c=anchor.t_s0_c,
            w0_mm=anchor.w0_mm,
        )
    n_laps = obs.n_laps
    target = obs.lap_time_s

    def residual(x: F) -> F:
        params = _apply(free, cfg.base, x)
        sim = stint_trace(params, anchor, n_laps).lap_time_s
        return sim - target

    x0 = _encode(free, cfg.base.replace(**{q.name: q.init for q in free}))
    lo, hi = _bounds(free)
    x0 = np.clip(x0, lo, hi)
    result: Any = least_squares(
        residual, x0, bounds=(lo, hi), method="trf", loss="soft_l1"
    )
    best = _apply(free, cfg.base, np.asarray(result.x, dtype=np.float64))
    final = residual(np.asarray(result.x, dtype=np.float64))
    trace = stint_trace(best, anchor, n_laps)
    return CalibResult(
        params=best,
        free=cfg.free,
        fitted={q.name: float(getattr(best, q.name)) for q in free},
        rms_s=float(np.sqrt(np.mean(final**2))),
        max_abs_s=float(np.max(np.abs(final))),
        n_laps=n_laps,
        decay_s_per_lap=_decay_rate(trace.lap_time_s),
        cliff_lap=_cliff_lap(trace.wear_mm, best.w_c),
        sim_lap_time_s=trace.lap_time_s,
        obs_lap_time_s=target,
        success=bool(result.success),
    )


def _decay_rate(lap_time_s: F) -> float:
    """Mean linear pace loss (s/lap): the slope of a straight-line fit to the lap-time curve."""
    laps = np.arange(1, lap_time_s.size + 1, dtype=np.float64)
    slope = np.polyfit(laps, lap_time_s, 1)[0]
    return float(slope)


def _cliff_lap(wear_mm: F, w_c: float) -> int | None:
    """First 1-based lap at which wear crosses the cliff onset ``w_c`` (None if never)."""
    crossed = np.nonzero(wear_mm >= w_c)[0]
    return int(crossed[0] + 1) if crossed.size else None


def synth_observation(
    params: SurrogateParams,
    anchor: StintAnchor,
    n_laps: int,
    *,
    noise_s: float = 0.0,
    seed: int = 0,
    label: str = "synthetic",
) -> StintObservation:
    """A deterministic synthetic stint from known parameters — the recovery-test target.

    Optional Gaussian lap-time noise (counter-based on ``seed``) exercises the fit's robustness.
    """
    times = stint_trace(params, anchor, n_laps).lap_time_s.copy()
    if noise_s > 0.0:
        rng = np.random.default_rng(seed)
        times = times + rng.normal(0.0, noise_s, size=times.shape)
    laps = np.arange(1, n_laps + 1, dtype=np.float64)
    return StintObservation(lap=laps, lap_time_s=times, label=label)


def _scipy_least_squares() -> Callable[..., Any]:
    """Lazy scipy import with an actionable error (the ``tirefit`` precedent)."""
    try:
        from scipy.optimize import (
            least_squares,  # pyright: ignore[reportUnknownVariableType]
        )
    except ImportError as err:  # pragma: no cover - exercised only without the extra
        raise ImportError(
            "the wear calibrator needs scipy — install the extra: "
            "`uv sync --extra wear-cal` (or `pip install 'outlap[wear-cal]'`)"
        ) from err
    return cast("Callable[..., Any]", least_squares)
