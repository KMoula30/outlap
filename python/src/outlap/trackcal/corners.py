# SPDX-License-Identifier: AGPL-3.0-only
"""Corner detection and robust apex-radius / apex-speed extraction.

Corners are detected as contiguous regions where the fitted ``|κ|`` exceeds a threshold
(``1 / min_radius_m``); each region's apex is its curvature extremum. The apex radius is then
*audited* — never read off ``1/κ`` alone — by a circle fit over a windowed short arc around the
apex: an algebraic fit initialises an iterative geometric (orthogonal-distance) refinement,
because algebraic fits alone are biased on short arcs (Chernov 2010, ch. 4–7).

**Algebraic stage — the "Hyper" fit.** A circle is the zero set of
``P(x, y) = A (x² + y²) + B x + C y + D``. With the data moment matrix ``M`` (built from rows
``[zᵢ, xᵢ, yᵢ, 1]``, ``zᵢ = xᵢ² + yᵢ²``, data centred), the Hyper fit solves the generalized
eigenproblem ``M A = η N A`` for the parameter vector ``A = (A, B, C, D)ᵀ`` with the
"hyperaccuracy" constraint matrix ``N`` (Al-Sharadqah & Chernov 2009, §5: the constraint that
cancels the leading essential bias term; for centred data ``N`` has rows
``[8 z̄, 0, 0, 2] / [0,1,0,0] / [0,0,1,0] / [2, 0, 0, 0]``), taking the eigenvector of the
smallest non-negative ``η``. Centre and radius follow as ``a = −B/2A``, ``b = −C/2A``,
``R = sqrt(B² + C² − 4AD) / 2|A|``.

**Geometric stage.** Gauss–Newton on the geometric residuals ``rᵢ = dᵢ − R``,
``dᵢ = sqrt((xᵢ−a)² + (yᵢ−b)²)`` — the maximum-likelihood circle under isotropic noise — with
step halving as a divergence guard.

Citations:

* G. Taubin (1991), "Estimation of planar curves, surfaces, and nonplanar space curves defined
  by implicit equations with applications to edge and range image segmentation", IEEE TPAMI
  13(11), 1115–1138 — the gradient-weighted algebraic fit family the Hyper fit descends from.
* A. Al-Sharadqah & N. Chernov (2009), "Error analysis for circle fitting algorithms",
  Electronic Journal of Statistics 3, 886–911 — the Hyper fit (essential-bias-free algebraic
  circle fit) used for initialization.
* N. Chernov (2010), *Circular and Linear Regression: Fitting Circles and Lines by Least
  Squares*, CRC — short-arc bias analysis and geometric-refinement practice.

Consulted repositories (approach only, no code taken; clean-room per CLAUDE.md hard rule 2):
TUMFTM ``trajectory_planning_helpers`` (LGPL-3.0) — windowed-extremum corner detection on
fitted racetrack curvature.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray

from .geometry import CenterlineFit, DegenerateInputError, TrackCalError

F = NDArray[np.float64]

#: Minimum samples for a windowed circle fit (fewer cannot constrain 3 parameters robustly).
MIN_ARC_POINTS = 5

_GN_ITERS = 50
_GN_TOL = 1e-12
_RADIUS_CAP_M = 1.0e6


@dataclass(frozen=True)
class CircleFit:
    """A fitted circle: centre (m), radius (m), and RMS orthogonal residual (m)."""

    center_x_m: float
    center_y_m: float
    radius_m: float
    rms_m: float


@dataclass(frozen=True)
class Corner:
    """One detected corner with its audited apex radius (and optional apex speed).

    ``turn_direction`` is the sign of κ at the apex (+1 left, −1 right; ISO 8855, z up).
    ``window_start_m``/``window_end_m`` bound the short arc used for the circle fit; on closed
    tracks a window crossing the seam has ``window_start_m > window_end_m`` (wrapped).
    """

    index: int
    s_apex_m: float
    apex_radius_m: float
    window_start_m: float
    window_end_m: float
    turn_direction: int
    circle_rms_m: float
    apex_speed_mps: float | None = None


def fit_circle(x_m: F, y_m: F) -> CircleFit:
    """Robust circle fit: Hyper algebraic initialization + geometric Gauss–Newton refinement.

    Raises :class:`DegenerateInputError` for < 3 points, collinear points, or arcs so flat the
    fitted radius exceeds 10⁶ m.
    """
    x = np.asarray(x_m, dtype=np.float64)
    y = np.asarray(y_m, dtype=np.float64)
    if x.ndim != 1 or y.ndim != 1 or x.shape != y.shape:
        raise TrackCalError("x_m and y_m must be 1-D arrays of equal length")
    if x.size < 3:
        raise DegenerateInputError(f"a circle fit needs >= 3 points, got {x.size}")
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        raise DegenerateInputError("circle-fit points contain non-finite coordinates")
    cx0, cy0 = float(np.mean(x)), float(np.mean(y))
    xc, yc = x - cx0, y - cy0
    sv = np.linalg.svd(np.stack([xc, yc], axis=1), compute_uv=False)
    if float(sv[1]) <= max(1e-12 * float(sv[0]), 1e-12):
        raise DegenerateInputError("circle-fit points are collinear")

    a, b, r = _hyper_fit(xc, yc)
    a, b, r, rms = _refine_geometric(xc, yc, a, b, r)
    if not (np.isfinite(r) and 0.0 < r < _RADIUS_CAP_M):
        raise DegenerateInputError(
            f"circle fit degenerate: radius {r!r} m (arc too flat or too short)"
        )
    return CircleFit(center_x_m=a + cx0, center_y_m=b + cy0, radius_m=r, rms_m=rms)


def detect_corners(
    fit: CenterlineFit,
    *,
    min_radius_m: float = 200.0,
    min_arc_m: float = 5.0,
    window_kappa_frac: float = 0.8,
    step_m: float = 1.0,
    speed: tuple[F, F] | None = None,
) -> list[Corner]:
    """Detect corners on a fitted centerline and audit each apex radius with a circle fit.

    A corner is a contiguous region with local radius below ``min_radius_m`` lasting at least
    ``min_arc_m`` of arc. The circle-fit window keeps the samples with
    ``|κ| >= window_kappa_frac · |κ_apex|`` around the apex (at least :data:`MIN_ARC_POINTS`).
    ``speed`` is an optional ``(s_m, v_mps)`` sample pair (e.g. pooled telemetry); the apex
    speed is the robust minimum (10th percentile) of the speed samples inside the window.
    Returns corners ordered by ``s_apex_m``, 1-indexed.
    """
    if not 0.0 < window_kappa_frac <= 1.0:
        raise TrackCalError("window_kappa_frac must be in (0, 1]")
    if min_radius_m <= 0.0 or min_arc_m <= 0.0:
        raise TrackCalError("min_radius_m and min_arc_m must be positive")
    samples = fit.sample_uniform(step_m)
    kappa = samples.kappa_per_m
    absk = np.abs(kappa)
    mask = absk >= 1.0 / min_radius_m
    min_run = max(int(round(min_arc_m / step_m)), 1)

    corners: list[Corner] = []
    for run in _true_runs(mask, closed=fit.closed):
        if run.size < min_run:
            continue
        peak_pos = int(np.argmax(absk[run]))
        lo, hi = _kappa_window(absk[run], peak_pos, window_kappa_frac)
        window = run[lo : hi + 1]
        circle = fit_circle(samples.x_m[window], samples.y_m[window])
        s_start = float(samples.s_m[window[0]])
        s_end = float(samples.s_m[window[-1]])
        apex_s = float(samples.s_m[run[peak_pos]])
        wrapped = s_start > s_end  # closed-track window crossing the seam
        corners.append(
            Corner(
                index=0,  # assigned after the s-sort below
                s_apex_m=apex_s,
                apex_radius_m=circle.radius_m,
                window_start_m=s_start,
                window_end_m=s_end,
                turn_direction=1 if float(kappa[run[peak_pos]]) >= 0.0 else -1,
                circle_rms_m=circle.rms_m,
                apex_speed_mps=_apex_speed(speed, s_start, s_end, wrapped=wrapped),
            )
        )
    corners.sort(key=lambda c: c.s_apex_m)
    return [replace(c, index=i + 1) for i, c in enumerate(corners)]


# --- internals ------------------------------------------------------------------------------


def _true_runs(mask: NDArray[np.bool_], *, closed: bool) -> list[NDArray[np.int64]]:
    """Contiguous True runs as ordered index arrays; wrap-merges the seam run when closed."""
    idx = np.nonzero(mask)[0].astype(np.int64)
    if idx.size == 0:
        return []
    n = int(mask.size)
    if idx.size == n:  # everything is corner (e.g. a pure circle): one run
        return [idx]
    breaks = np.nonzero(np.diff(idx) > 1)[0]
    runs = [r for r in np.split(idx, breaks + 1)]
    if closed and len(runs) > 1 and runs[0][0] == 0 and runs[-1][-1] == n - 1:
        runs = [np.concatenate([runs[-1], runs[0]])] + runs[1:-1]
    return runs


def _kappa_window(absk_run: F, peak_pos: int, frac: float) -> tuple[int, int]:
    """Expand from the apex while |κ| stays above ``frac`` of the peak; enforce a minimum."""
    threshold = frac * float(absk_run[peak_pos])
    lo = peak_pos
    while lo > 0 and float(absk_run[lo - 1]) >= threshold:
        lo -= 1
    hi = peak_pos
    last = absk_run.shape[0] - 1
    while hi < last and float(absk_run[hi + 1]) >= threshold:
        hi += 1
    while hi - lo + 1 < MIN_ARC_POINTS and (lo > 0 or hi < last):
        if lo > 0:
            lo -= 1
        if hi < last and hi - lo + 1 < MIN_ARC_POINTS:
            hi += 1
    return lo, hi


def _apex_speed(
    speed: tuple[F, F] | None, s_start: float, s_end: float, *, wrapped: bool
) -> float | None:
    """Robust apex speed: 10th percentile of the speed samples inside the corner window."""
    if speed is None:
        return None
    s_v = np.asarray(speed[0], dtype=np.float64)
    v = np.asarray(speed[1], dtype=np.float64)
    if s_v.ndim != 1 or v.ndim != 1 or s_v.shape != v.shape:
        raise TrackCalError("speed must be a pair of 1-D arrays of equal length")
    if wrapped:
        sel = (s_v >= s_start) | (s_v <= s_end)
    else:
        sel = (s_v >= s_start) & (s_v <= s_end)
    if not bool(np.any(sel)):
        return None
    return float(np.percentile(v[sel], 10.0))


def _hyper_fit(x: F, y: F) -> tuple[float, float, float]:
    """The Hyper algebraic circle fit (Al-Sharadqah & Chernov 2009) on centred data."""
    z = x * x + y * y
    zbar = float(np.mean(z))
    rows = np.stack([z, x, y, np.ones_like(x)], axis=1)
    m_mat = (rows.T @ rows) / x.size
    n_mat = np.array(
        [
            [8.0 * zbar, 0.0, 0.0, 2.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [2.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    eigvals, eigvecs = np.linalg.eig(np.linalg.solve(n_mat, m_mat))
    real = np.abs(eigvals.imag) <= 1e-8 * (1.0 + np.abs(eigvals.real))
    etas = eigvals.real
    candidates = np.nonzero(real & (etas > -1e-12))[0]
    if (
        candidates.size == 0
    ):  # pragma: no cover - defensive; centred data always yields one
        raise DegenerateInputError("Hyper circle fit found no admissible eigenvalue")
    best = candidates[int(np.argmin(etas[candidates]))]
    vec = eigvecs[:, best].real
    a_coef, b_coef, c_coef, d_coef = (float(v) for v in vec)
    if abs(a_coef) < 1e-15 * float(np.max(np.abs(vec))):
        raise DegenerateInputError("circle-fit arc is flat (algebraic A ≈ 0)")
    xc = -b_coef / (2.0 * a_coef)
    yc = -c_coef / (2.0 * a_coef)
    r_sq = (b_coef**2 + c_coef**2 - 4.0 * a_coef * d_coef) / (4.0 * a_coef**2)
    return xc, yc, float(np.sqrt(max(r_sq, 0.0)))


def _refine_geometric(
    x: F, y: F, a: float, b: float, r: float
) -> tuple[float, float, float, float]:
    """Gauss–Newton on rᵢ = dᵢ − R with step halving (Chernov 2010, geometric fit)."""

    def residuals(a_: float, b_: float, r_: float) -> tuple[F, F]:
        d = np.hypot(x - a_, y - b_)
        return d - r_, d

    res, d = residuals(a, b, r)
    cost = float(np.sum(res**2))
    for _ in range(_GN_ITERS):
        d_safe = np.maximum(d, 1e-12)
        jac = np.stack([-(x - a) / d_safe, -(y - b) / d_safe, -np.ones_like(d)], axis=1)
        try:
            step = np.linalg.solve(jac.T @ jac, -(jac.T @ res))
        except (
            np.linalg.LinAlgError
        ):  # pragma: no cover - singular only when degenerate
            break
        scale = 1.0
        improved = False
        for _ in range(20):  # step halving keeps the refinement from diverging
            a_n = a + scale * float(step[0])
            b_n = b + scale * float(step[1])
            r_n = r + scale * float(step[2])
            res_n, d_n = residuals(a_n, b_n, r_n)
            cost_n = float(np.sum(res_n**2))
            if cost_n <= cost:
                a, b, r, res, d, cost = a_n, b_n, r_n, res_n, d_n, cost_n
                improved = True
                break
            scale *= 0.5
        if not improved or float(np.linalg.norm(step)) <= _GN_TOL * (1.0 + abs(r)):
            break
    rms = float(np.sqrt(np.mean(res**2)))
    return a, b, r, rms
