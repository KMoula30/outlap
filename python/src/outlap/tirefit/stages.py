# SPDX-License-Identifier: AGPL-3.0-only
"""The staged MF6.1 fit: nominals → pure Fx0 → pure Fy0 → combined → Mz → Mx/My.

Each stage frees a documented coefficient subset (init + bounds tables below), holds everything
already fitted fixed, and minimises Fz-normalised residuals of one channel with
``scipy.optimize.least_squares`` (``method='trf'``, ``loss='soft_l1'``). The pipeline is fully
deterministic: fixed initial points from the tables, no random restarts (the only randomness in
the package is the seeded noise of :func:`synthesize`). scipy is imported lazily with an
actionable error — install the ``tire-fit`` extra.

Stages whose data support is absent are skipped and reported as such: camber terms are freed
only when the data has camber spread, combined stages only when κ and α overlap, moment stages
only when the channel is present (non-zero).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray

from . import mf61
from .data import TireTestData

F = NDArray[np.float64]

# Data-support thresholds (rad / ratio): what counts as "pure" slip and "has camber".
_PURE_ALPHA = 0.01
_PURE_KAPPA = 0.005
_GAMMA_SPREAD = 0.005


@dataclass(frozen=True)
class Param:
    """One free coefficient: initial value and (lo, hi) bounds."""

    name: str
    init: float
    lo: float
    hi: float


# Stage tables — the documented init/bounds choices. Inits sit in the physically-typical band
# for a road/racing tyre; PDY1/PKY1 inits pick the book's ISO-W sign convention (both negative),
# which the optimizer keeps unless the data clearly says otherwise.
FX0_PARAMS: list[Param] = [
    Param("PCX1", 1.6, 1.2, 2.5),
    Param("PDX1", 1.2, 0.3, 3.0),
    Param("PDX2", -0.05, -0.6, 0.2),
    Param("PEX1", 0.0, -6.0, 1.0),
    Param("PEX2", 0.0, -6.0, 1.0),
    Param("PEX3", 0.0, -6.0, 1.0),
    Param("PEX4", 0.0, -2.0, 2.0),
    Param("PKX1", 25.0, 4.0, 100.0),
    Param("PKX2", 0.0, -40.0, 40.0),
    Param("PKX3", 0.0, -5.0, 5.0),
    Param("PHX1", 0.0, -0.1, 0.1),
    Param("PHX2", 0.0, -0.1, 0.1),
    Param("PVX1", 0.0, -0.3, 0.3),
    Param("PVX2", 0.0, -0.3, 0.3),
]

FY0_PARAMS: list[Param] = [
    Param("PCY1", 1.3, 1.0, 2.5),
    Param("PDY1", -1.0, -3.0, 3.0),
    Param("PDY2", 0.1, -0.6, 0.6),
    Param("PEY1", -0.8, -6.0, 1.0),
    Param("PEY2", -0.5, -6.0, 1.0),
    Param("PKY1", -20.0, -200.0, -1.0),
    Param("PKY2", 2.0, 0.2, 8.0),
    Param("PHY1", 0.0, -0.1, 0.1),
    Param("PHY2", 0.0, -0.1, 0.1),
    Param("PVY1", 0.0, -0.3, 0.3),
    Param("PVY2", 0.0, -0.3, 0.3),
]

#: Extra Fy0 terms freed only when the data carries camber sweeps.
FY0_CAMBER_PARAMS: list[Param] = [
    Param("PDY3", 0.0, -20.0, 20.0),
    Param("PEY3", 0.0, -2.0, 2.0),
    Param("PEY4", 0.0, -20.0, 20.0),
    Param("PKY3", 0.0, -2.0, 2.0),
    Param("PKY6", -1.0, -5.0, 5.0),
    Param("PKY7", 0.0, -5.0, 5.0),
    Param("PVY3", 0.0, -5.0, 5.0),
    Param("PVY4", 0.0, -5.0, 5.0),
]

COMBINED_X_PARAMS: list[Param] = [
    Param("RBX1", 12.0, 2.0, 40.0),
    Param("RBX2", 10.0, -40.0, 40.0),
    Param("RCX1", 1.0, 0.5, 1.5),
    Param("RHX1", 0.0, -0.1, 0.1),
]

COMBINED_Y_PARAMS: list[Param] = [
    Param("RBY1", 8.0, 2.0, 40.0),
    Param("RBY2", 5.0, -40.0, 40.0),
    Param("RBY3", 0.0, -0.3, 0.3),
    Param("RCY1", 1.0, 0.5, 1.5),
    Param("RHY1", 0.0, -0.1, 0.1),
    Param("RVY1", 0.0, -0.3, 0.3),
    Param("RVY4", 0.0, -40.0, 40.0),
    Param("RVY5", 0.0, -3.0, 3.0),
    Param("RVY6", 0.0, -40.0, 40.0),
]

MZ_PARAMS: list[Param] = [
    Param("QBZ1", 8.0, 2.0, 30.0),
    Param("QBZ2", 0.0, -15.0, 15.0),
    Param("QBZ9", 10.0, 0.0, 60.0),
    Param("QBZ10", 0.0, -2.0, 2.0),
    Param("QCZ1", 1.1, 0.9, 1.8),
    Param("QDZ1", 0.1, 0.01, 0.4),
    Param("QDZ2", 0.0, -0.2, 0.2),
    Param("QDZ6", 0.0, -0.05, 0.05),
    Param("QDZ7", 0.0, -0.05, 0.05),
    Param("QEZ1", -1.0, -6.0, 1.0),
    Param("QEZ4", 0.0, -2.0, 2.0),
    Param("QHZ1", 0.0, -0.05, 0.05),
    Param("QHZ2", 0.0, -0.05, 0.05),
]

#: Mx only. TTC logs no My channel, so the rolling-resistance ``QSY*`` family is never freed —
#: a coefficient whose residual Jacobian is identically zero would simply be returned at its
#: init, fabricating a My model no data supports. Set ``QSY*`` from rig/coast-down data by hand.
MXMY_PARAMS: list[Param] = [
    Param("QSX1", 0.0, -0.1, 0.1),
    Param("QSX2", 0.0, -3.0, 3.0),
    Param("QSX3", 0.0, -0.3, 0.3),
]


@dataclass
class StageReport:
    """Outcome of one fit stage (or the reason it was skipped)."""

    name: str
    fitted: dict[str, float] = field(default_factory=lambda: dict[str, float]())
    rms_n: float = 0.0
    max_abs_n: float = 0.0
    n_samples: int = 0
    skipped: str | None = None


@dataclass
class FitConfig:
    """User-supplied nominals a test file cannot carry."""

    unloaded_radius_m: float
    fnomin_n: float | None = None
    nompres_pa: float | None = None
    longvl_mps: float = 16.7


@dataclass
class FitResult:
    """The fitted coefficient map plus the per-stage report."""

    coeffs: dict[str, float]
    stages: list[StageReport]


def staged_fit(data: TireTestData, config: FitConfig) -> FitResult:
    """Run the full staged fit over ``data`` (see the module docs for the stage plan)."""
    coeffs = _nominals(data, config)
    p = mf61.params_from_coeffs(coeffs)
    stages: list[StageReport] = [
        StageReport(
            "nominals",
            fitted={k: coeffs[k] for k in sorted(coeffs)},
            n_samples=len(data),
        )
    ]

    alpha_pure = np.abs(data.alpha_rad) < _PURE_ALPHA
    kappa_pure = np.abs(data.kappa) < _PURE_KAPPA
    has_camber = float(np.std(data.gamma_rad)) > _GAMMA_SPREAD
    has_kappa = float(np.std(data.kappa)) > _PURE_KAPPA

    # Stage: pure Fx0 (κ sweeps at α ≈ 0).
    mask = alpha_pure & ~kappa_pure
    if has_kappa and int(mask.sum()) >= len(FX0_PARAMS) * 4:
        stages.append(_run(p, "fx0", FX0_PARAMS, data, mask, "fx"))
    else:
        stages.append(StageReport("fx0", skipped="no pure longitudinal sweeps"))

    # Stage: pure Fy0 (α sweeps at κ ≈ 0), camber terms when supported.
    mask = kappa_pure & ~alpha_pure
    fy0_free = FY0_PARAMS + (FY0_CAMBER_PARAMS if has_camber else [])
    if int(mask.sum()) >= len(fy0_free) * 4:
        stages.append(_run(p, "fy0", fy0_free, data, mask, "fy"))
    else:
        stages.append(StageReport("fy0", skipped="no pure lateral sweeps"))

    # Stage: combined (both slips active).
    mask = ~alpha_pure & ~kappa_pure
    if int(mask.sum()) >= (len(COMBINED_X_PARAMS) + len(COMBINED_Y_PARAMS)) * 4:
        stages.append(_run(p, "combined_fx", COMBINED_X_PARAMS, data, mask, "fx"))
        stages.append(_run(p, "combined_fy", COMBINED_Y_PARAMS, data, mask, "fy"))
    else:
        stages.append(StageReport("combined", skipped="no combined-slip samples"))

    # Stage: Mz (pure lateral sweeps carry the trail). Gate on real signal, not noise: the
    # channel must exceed 1% of the characteristic trail moment Fz0·R0.
    moment_floor = 0.01 * p["FNOMIN"] * p["UNLOADED_RADIUS"]
    mask = kappa_pure & ~alpha_pure
    if (
        float(np.max(np.abs(data.mz_nm[mask]), initial=0.0)) > moment_floor
        and int(mask.sum()) >= len(MZ_PARAMS) * 4
    ):
        stages.append(
            _run(
                p,
                "mz",
                MZ_PARAMS,
                data,
                mask,
                "mz",
                scale=float(np.max(np.abs(data.mz_nm[mask]))) or 1.0,
            )
        )
    else:
        stages.append(StageReport("mz", skipped="no aligning-moment signal"))

    # Stage: Mx (same signal gate — a channel that is only measurement noise must not
    # synthesise QSX* terms). My is not fittable from TTC data (no channel); see MXMY_PARAMS.
    mask = np.abs(data.mx_nm) > 0.0
    if (
        float(np.max(np.abs(data.mx_nm), initial=0.0)) > moment_floor
        and int(mask.sum()) >= len(MXMY_PARAMS) * 4
    ):
        stages.append(_run(p, "mxmy", MXMY_PARAMS, data, mask, "mx"))
    else:
        stages.append(StageReport("mxmy", skipped="no overturning-moment signal"))

    fitted = {
        k: v
        for k, v in p.items()
        if not k.startswith("_") and v != mf61.DEFAULTS.get(k, 0.0)
    }
    # Always carry the structural nominals, even when they equal a default.
    for k in ("FNOMIN", "UNLOADED_RADIUS", "LONGVL"):
        fitted[k] = p[k]
    if p["_HAS_NOMPRES"] != 0.0:
        fitted["NOMPRES"] = p["NOMPRES"]
    return FitResult(coeffs=dict(sorted(fitted.items())), stages=stages)


def synthesize(
    coeffs: dict[str, float],
    *,
    seed: int = 0,
    noise: float = 0.01,
    fz_levels: tuple[float, ...] = (0.5, 1.0, 1.75),
    n_sweep: int = 60,
) -> TireTestData:
    """Generate a deterministic synthetic test dataset from a coefficient map.

    Pure κ sweeps, pure α sweeps (at three camber levels), and a combined grid, at
    ``fz_levels``×FNOMIN, with seeded Gaussian noise of ``noise``×(channel scale). This is the
    recovery-test harness and the ``synth`` CLI backend — synthetic data only, never a
    substitute for measurement.
    """
    p = mf61.params_from_coeffs(coeffs)
    fnomin = p["FNOMIN"]
    pres = p["NOMPRES"] if p["_HAS_NOMPRES"] != 0.0 else 0.0
    vx = p["LONGVL"]

    kappa_l: list[F] = []
    alpha_l: list[F] = []
    gamma_l: list[F] = []
    fz_l: list[F] = []

    sweep_k = np.linspace(-0.25, 0.25, n_sweep)
    sweep_a = np.linspace(-0.20, 0.20, n_sweep)
    zeros = np.zeros(n_sweep)
    for level in fz_levels:
        fz = np.full(n_sweep, level * fnomin)
        # Pure longitudinal.
        kappa_l.append(sweep_k)
        alpha_l.append(zeros)
        gamma_l.append(zeros)
        fz_l.append(fz)
        # Pure lateral at three camber levels.
        for gam in (0.0, -0.05, 0.05):
            kappa_l.append(zeros)
            alpha_l.append(sweep_a)
            gamma_l.append(np.full(n_sweep, gam))
            fz_l.append(fz)
        # Combined grid.
        gk, ga = np.meshgrid(np.linspace(-0.15, 0.15, 12), np.linspace(-0.12, 0.12, 12))
        kappa_l.append(gk.ravel())
        alpha_l.append(ga.ravel())
        gamma_l.append(np.zeros(gk.size))
        fz_l.append(np.full(gk.size, level * fnomin))

    kappa = np.concatenate(kappa_l)
    alpha = np.concatenate(alpha_l)
    gamma = np.concatenate(gamma_l)
    fz = np.concatenate(fz_l)
    pres_arr = np.full_like(fz, pres)
    vx_arr = np.full_like(fz, vx)

    out = mf61.forces(p, kappa, alpha, gamma, fz, pres_arr, vx_arr)
    rng = np.random.default_rng(seed)

    def noisy(x: F, scale: float) -> F:
        return x + rng.normal(0.0, noise * scale, x.shape)

    return TireTestData(
        kappa=kappa,
        alpha_rad=alpha,
        gamma_rad=gamma,
        fz_n=fz,
        p_pa=pres_arr,
        vx_mps=vx_arr,
        fx_n=noisy(out.fx, fnomin),
        fy_n=noisy(out.fy, fnomin),
        mz_nm=noisy(out.mz, 0.05 * fnomin * p["UNLOADED_RADIUS"]),
        mx_nm=noisy(out.mx, 0.05 * fnomin * p["UNLOADED_RADIUS"]),
    )


def _nominals(data: TireTestData, config: FitConfig) -> dict[str, float]:
    """Stage 0: the structural nominals a fit cannot invent (config first, data medians else)."""
    coeffs: dict[str, float] = {
        "UNLOADED_RADIUS": config.unloaded_radius_m,
        "LONGVL": config.longvl_mps,
    }
    if config.fnomin_n is not None:
        coeffs["FNOMIN"] = config.fnomin_n
    else:
        # Median load rounded to 250 N: a stable, documented derivation.
        coeffs["FNOMIN"] = float(np.round(np.median(data.fz_n) / 250.0) * 250.0)
    if config.nompres_pa is not None:
        coeffs["NOMPRES"] = config.nompres_pa
    elif float(np.median(data.p_pa)) > 0.0:
        coeffs["NOMPRES"] = float(np.round(np.median(data.p_pa) / 1000.0) * 1000.0)
    if coeffs["FNOMIN"] <= 0.0:
        raise ValueError("could not derive a positive FNOMIN from the data")
    return coeffs


def _run(
    p: dict[str, float],
    name: str,
    free: list[Param],
    data: TireTestData,
    mask: NDArray[np.bool_],
    channel: str,
    scale: float | None = None,
) -> StageReport:
    """Fit ``free`` on ``channel`` over ``mask``-selected samples; mutates ``p`` in place."""
    least_squares = _scipy_least_squares()

    kappa = data.kappa[mask]
    alpha = data.alpha_rad[mask]
    gamma = data.gamma_rad[mask]
    fz = data.fz_n[mask]
    pres = data.p_pa[mask]
    vx = data.vx_mps[mask]
    target = {
        "fx": data.fx_n,
        "fy": data.fy_n,
        "mz": data.mz_nm,
        "mx": data.mx_nm,
    }[channel][mask]
    # Fz-normalised force residuals weight every load level equally; moments use a channel scale.
    denom = fz if channel in ("fx", "fy") else np.full_like(fz, scale or 1.0)

    def residual(x: NDArray[np.float64]) -> NDArray[np.float64]:
        for param, value in zip(free, x, strict=True):
            p[param.name] = float(value)
        out = mf61.forces(p, kappa, alpha, gamma, fz, pres, vx)
        model: F = getattr(out, channel)
        return (model - target) / denom

    x0 = np.array([q.init for q in free])
    lo = np.array([q.lo for q in free])
    hi = np.array([q.hi for q in free])
    result: Any = least_squares(
        residual, x0, bounds=(lo, hi), method="trf", loss="soft_l1"
    )
    for param, value in zip(free, result.x, strict=True):
        p[param.name] = float(value)

    final = residual(np.asarray(result.x, dtype=np.float64))
    return StageReport(
        name=name,
        fitted={q.name: p[q.name] for q in free},
        rms_n=float(np.sqrt(np.mean(final**2))),
        max_abs_n=float(np.max(np.abs(final))),
        n_samples=int(mask.sum()),
    )


def _scipy_least_squares() -> Callable[..., Any]:
    """Lazy scipy import with an actionable error (the ``osm_track`` precedent)."""
    try:
        from scipy.optimize import (
            least_squares,  # pyright: ignore[reportUnknownVariableType]
        )
    except ImportError as err:  # pragma: no cover - exercised only without the extra
        raise ImportError(
            "the MF6.1 fitting stages need scipy — install the extra: "
            "`uv sync --extra tire-fit` (or `pip install 'outlap[tire-fit]'`)"
        ) from err
    # scipy's stubs leave least_squares partially unknown; pin the shape we rely on.
    return cast("Callable[..., Any]", least_squares)
