# SPDX-License-Identifier: AGPL-3.0-only
"""Curvature-first centerline fitting: a penalised periodic smoothing spline.

Apex radii are the product this package exists for, and curvature is a *second derivative* of
position: differentiating noisy samples amplifies noise as ``O(σ/h²)``, so the naive route —
interpolate the raw points with a spline and read κ off its second derivatives — systematically
inflates ``|κ|`` and reads as too-tight apex radii (the M6 Barcelona ~31 m vs ~34 m bias). That
bias is a documented anti-pattern in the derivative-estimation literature (De Brabanter et al.
2013; Early & Sykulski 2020). Here curvature is a first-class target: the centerline is fitted
with a roughness-penalised (P-spline) least-squares problem and κ is evaluated analytically
from the fit — never finite-differenced from raw points.

**Method.** Uniform cubic B-spline basis (cyclic for closed tracks, so the fit is C² across the
``s = 0`` seam), coefficients ``c`` fitted per axis by minimising the P-spline objective

    ``‖z − B c‖² + λ ‖D₂ c‖²_Λ``            (Eilers & Marx 1996)

where ``B`` is the design matrix, ``D₂`` the (cyclic) second-difference operator on ``c`` — a
discrete roughness penalty — and ``Λ = diag(w)`` per-region penalty weights. The global ``λ`` is
chosen by the Morozov discrepancy principle: bisect ``log λ`` until the residual RMS matches the
declared measurement noise ``σ`` (``noise_std_m``). Regularization is **per-corner adaptive**
(physically motivated): a first pass fits with uniform weights, then weights are relaxed where
the fitted ``|κ|`` is high — ``w = w_min + (1 − w_min) / (1 + (|κ| R_adapt)²)`` — and ``λ`` is
re-matched. Under-smoothing leaves κ noise on straights; over-smoothing washes out exactly the
30–35 m apexes; the adaptive weights resolve that tension.

Curvature uses the parameterization-invariant form ``κ = (x'y'' − y'x'') / (x'² + y'²)^{3/2}``
with derivatives taken analytically in the spline parameter; arc length is integrated from the
fit so all public APIs speak metres of fitted arc length (SI, ISO 8855: z up, positive κ =
left turn).

**Dependency note.** Implemented with numpy dense linear algebra only — scipy is deliberately
not required (in this repo scipy lives in optional extras), so the trackcal core and its tests
run everywhere the base package installs. The dense solves are sized for importer-scale inputs
(K ≈ L/3 m coefficients; a 5 km track solves in seconds, offline).

Citations (paper symbols above follow these):

* P. H. C. Eilers & B. D. Marx (1996), "Flexible smoothing with B-splines and penalties",
  Statistical Science 11(2), 89–121 — the P-spline objective and difference penalty.
* K. De Brabanter, J. De Brabanter, B. De Moor & I. Gijbels (2013), "Derivative estimation with
  local polynomial fitting", JMLR 14, 281–301 — bias/variance of derivatives estimated from
  noisy samples (why raw second-derivative curvature is unusable).
* J. J. Early & A. M. Sykulski (2020), "Smoothing and interpolating noisy GPS data with
  smoothing splines", J. Atmos. Oceanic Technol. 37(3), 449–465 — noise-matched smoothing
  splines for GPS tracks.
* V. A. Morozov (1966), "On the solution of functional equations by the method of
  regularization" — the discrepancy principle used to pick ``λ``.

Consulted repositories (approach only, no code taken; clean-room per CLAUDE.md hard rule 2):
TUMFTM ``trajectory_planning_helpers`` and TUMFTM ``racetrack-database`` (both LGPL-3.0) —
approximate-spline regularization of racetrack centerlines.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

F = NDArray[np.float64]


class _Solver(Protocol):
    """The per-λ penalised solve closure (coefficients, per-axis residual RMS)."""

    def __call__(self, lam: float, weights: F) -> tuple[F, float]: ...


#: Minimum number of distinct input points for a centerline fit.
MIN_POINTS = 10

# Numerical guards for the penalised solve.
_LAMBDA_FLOOR = 1e-9
_LOG_LAMBDA_MAX = 6.0
_BISECT_ITERS = 40
_WEIGHT_FLOOR = 0.05


class TrackCalError(ValueError):
    """Base error for the trackcal package (typed, never a bare crash)."""


class DegenerateInputError(TrackCalError):
    """Input geometry cannot support a fit (too few points, collinear, non-finite)."""


@dataclass(frozen=True)
class CenterlineSamples:
    """A uniform-arc-length sampling of a :class:`CenterlineFit`."""

    s_m: F
    x_m: F
    y_m: F
    kappa_per_m: F


@dataclass(frozen=True)
class CenterlineFit:
    """A fitted centerline: evaluate position and curvature at any arc length.

    ``length_m`` is the arc length of the *fitted* curve; for closed fits every ``s`` query
    wraps modulo ``length_m`` (the basis is cyclic, so the fit is C² across the seam).
    ``residual_rms_m`` (per-axis) and ``smoothing_lambda`` record what the discrepancy search
    settled on — surfaced so importers can report the fit honestly.
    """

    closed: bool
    length_m: float
    residual_rms_m: float
    smoothing_lambda: float
    cells: int
    coeff_x: F
    coeff_y: F
    u_grid: F
    s_grid: F

    def _s_to_u(self, s_m: F) -> F:
        s = np.asarray(s_m, dtype=np.float64)
        if self.closed:
            s = np.mod(s, self.length_m)
        else:
            s = np.clip(s, 0.0, self.length_m)
        return np.interp(s, self.s_grid, self.u_grid)

    def evaluate(self, s_m: F) -> tuple[F, F]:
        """Position ``(x, y)`` in metres at arc length ``s_m`` (wraps when closed)."""
        u = self._s_to_u(s_m)
        x = _eval_spline(self.coeff_x, u, self.cells, self.closed, 0)
        y = _eval_spline(self.coeff_y, u, self.cells, self.closed, 0)
        return x, y

    def curvature(self, s_m: F) -> F:
        """Signed curvature κ (1/m) at arc length ``s_m``; positive = left turn (z up).

        Evaluated analytically from the fit via ``κ = (x'y'' − y'x'') / (x'² + y'²)^{3/2}``
        (parameterization-invariant, so spline-parameter derivatives suffice).
        """
        u = self._s_to_u(s_m)
        dx = _eval_spline(self.coeff_x, u, self.cells, self.closed, 1)
        dy = _eval_spline(self.coeff_y, u, self.cells, self.closed, 1)
        ddx = _eval_spline(self.coeff_x, u, self.cells, self.closed, 2)
        ddy = _eval_spline(self.coeff_y, u, self.cells, self.closed, 2)
        speed_sq = np.maximum(dx * dx + dy * dy, 1e-300)
        return (dx * ddy - dy * ddx) / speed_sq**1.5

    def sample_uniform(self, step_m: float) -> CenterlineSamples:
        """Sample the fit at ~``step_m`` metre spacing, uniform in fitted arc length."""
        if step_m <= 0.0:
            raise TrackCalError("step_m must be positive")
        n = max(int(round(self.length_m / step_m)), 4)
        if self.closed:
            s = np.arange(n, dtype=np.float64) * (self.length_m / n)
        else:
            s = np.linspace(0.0, self.length_m, n + 1)
        x, y = self.evaluate(s)
        return CenterlineSamples(s_m=s, x_m=x, y_m=y, kappa_per_m=self.curvature(s))


def fit_centerline(
    x_m: F,
    y_m: F,
    *,
    closed: bool,
    noise_std_m: float = 0.3,
    knot_spacing_m: float = 3.0,
    adaptive: bool = True,
    adapt_radius_m: float = 150.0,
    smoothing: float | None = None,
) -> CenterlineFit:
    """Fit a penalised (periodic) smoothing spline to noisy centerline points.

    ``noise_std_m`` is the per-axis measurement noise the discrepancy principle matches
    (FastF1-reconstructed positions sit around 0.3 m); ``0.0`` means the samples are exact and
    the fit (near-)interpolates. ``knot_spacing_m`` sets the B-spline resolution.
    ``adaptive`` enables the per-corner penalty relaxation around ``adapt_radius_m``.

    ``smoothing`` overrides the discrepancy search with an explicit ``λ`` (uniform weights).
    ``smoothing=0.0`` with ``knot_spacing_m`` at the sample spacing degenerates into the naive
    interpolating spline — the documented biased anti-pattern, exposed only so tests can keep
    demonstrating that it fails where the penalised fit succeeds.

    Raises :class:`DegenerateInputError` for < ``MIN_POINTS`` distinct points, collinear input,
    or non-finite coordinates.
    """
    if noise_std_m < 0.0:
        raise TrackCalError("noise_std_m must be >= 0")
    if knot_spacing_m <= 0.0:
        raise TrackCalError("knot_spacing_m must be positive")
    pts = _prepare_points(x_m, y_m, closed=closed)
    n = pts.shape[0]
    center = pts.mean(axis=0)
    z = pts - center

    u, total_chord = _chord_parameter(z, closed=closed)
    if closed:
        # A cyclic basis has no ends; a redundant basis (cells > n) stays benign because
        # the penalty's preference is translation-invariant around the loop.
        cells = int(np.clip(round(total_chord / knot_spacing_m), 8, 2 * n))
    else:
        # Open fits must stay data-determined (ncoef = cells + 3 <= n): otherwise the
        # penalty resolves the free end directions as a natural-spline end condition,
        # collapsing end curvature and ringing just inside the ends.
        cells = int(np.clip(round(total_chord / knot_spacing_m), 4, max(n - 3, 4)))
    ncoef = cells if closed else cells + 3

    b_mat = _design_matrix(u, cells, closed, 0)
    btb = b_mat.T @ b_mat
    btz = b_mat.T @ z
    d_mat = _second_difference(cells, closed)

    def solve(lam: float, weights: F) -> tuple[F, float]:
        pen = d_mat.T @ (weights[:, None] * d_mat)
        scale = float(np.trace(btb)) / max(float(np.trace(pen)), 1e-300)
        coeff = np.linalg.solve(btb + (lam * scale) * pen, btz)
        resid = z - b_mat @ coeff
        rms = float(np.sqrt(np.mean(resid**2)))  # per-axis RMS (isotropic noise)
        return coeff, rms

    uniform = np.ones(d_mat.shape[0], dtype=np.float64)
    if smoothing is not None:
        lam = max(float(smoothing), _LAMBDA_FLOOR)
        coeff, rms = solve(lam, uniform)
    else:
        lam, coeff, rms = _match_noise(solve, uniform, noise_std_m)
        if adaptive and noise_std_m > 0.0:
            weights = _adaptive_weights(
                coeff, cells, closed, ncoef, adapt_radius_m=adapt_radius_m
            )
            lam, coeff, rms = _match_noise(solve, weights, noise_std_m)

    coeff = coeff + center[None, :]  # partition of unity: shift restores the frame
    u_grid, s_grid = _arclength_tables(coeff[:, 0], coeff[:, 1], cells, closed)
    return CenterlineFit(
        closed=closed,
        length_m=float(s_grid[-1]),
        residual_rms_m=rms,
        smoothing_lambda=lam,
        cells=cells,
        coeff_x=np.ascontiguousarray(coeff[:, 0]),
        coeff_y=np.ascontiguousarray(coeff[:, 1]),
        u_grid=u_grid,
        s_grid=s_grid,
    )


# --- input preparation ----------------------------------------------------------------------


def _prepare_points(x_m: F, y_m: F, *, closed: bool) -> F:
    x = np.asarray(x_m, dtype=np.float64)
    y = np.asarray(y_m, dtype=np.float64)
    if x.ndim != 1 or y.ndim != 1 or x.shape != y.shape:
        raise TrackCalError("x_m and y_m must be 1-D arrays of equal length")
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        raise DegenerateInputError("centerline points contain non-finite coordinates")
    pts = np.stack([x, y], axis=1)
    if closed and pts.shape[0] > 1 and float(np.hypot(*(pts[-1] - pts[0]))) < 1e-9:
        pts = pts[:-1]  # drop a duplicated closing point
    if pts.shape[0] > 1:
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        keep = np.concatenate(([True], seg > 1e-9))
        pts = pts[keep]
    if pts.shape[0] < MIN_POINTS:
        raise DegenerateInputError(
            f"a centerline fit needs >= {MIN_POINTS} distinct points, got {pts.shape[0]}"
        )
    sv = np.linalg.svd(pts - pts.mean(axis=0), compute_uv=False)
    if float(sv[1]) <= max(1e-12 * float(sv[0]), 1e-9):
        raise DegenerateInputError(
            "centerline points are collinear (a straight line has no curvature signal)"
        )
    return pts


def _chord_parameter(pts: F, *, closed: bool) -> tuple[F, float]:
    """Chord-length parameter ``u`` (closed: [0,1); open: [0,1]) and the total chord."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    if closed:
        closing = float(np.hypot(pts[0, 0] - pts[-1, 0], pts[0, 1] - pts[-1, 1]))
        seg = np.concatenate([seg, [closing]])
    total = float(np.sum(seg))
    cum = np.concatenate(([0.0], np.cumsum(seg[:-1] if closed else seg)))
    return cum / total, total


# --- uniform cubic B-spline machinery -------------------------------------------------------


def _basis_weights(t: F, order: int) -> F:
    """The four uniform cubic B-spline blending weights (or d/dt, d²/dt²) at local ``t``."""
    if order == 0:
        w0 = (1.0 - t) ** 3 / 6.0
        w1 = (3.0 * t**3 - 6.0 * t**2 + 4.0) / 6.0
        w2 = (-3.0 * t**3 + 3.0 * t**2 + 3.0 * t + 1.0) / 6.0
        w3 = t**3 / 6.0
    elif order == 1:
        w0 = -0.5 * (1.0 - t) ** 2
        w1 = (9.0 * t**2 - 12.0 * t) / 6.0
        w2 = (-9.0 * t**2 + 6.0 * t + 3.0) / 6.0
        w3 = 0.5 * t**2
    elif order == 2:
        w0 = 1.0 - t
        w1 = 3.0 * t - 2.0
        w2 = -3.0 * t + 1.0
        w3 = t
    else:  # pragma: no cover - internal misuse guard
        raise TrackCalError(f"unsupported derivative order {order}")
    return np.stack([w0, w1, w2, w3], axis=1)


def _locate(u: F, cells: int, closed: bool) -> tuple[NDArray[np.int64], F]:
    """Map parameter ``u`` to (cell index, local t) on the uniform knot grid."""
    if closed:
        xi = np.mod(u, 1.0) * cells
    else:
        xi = np.clip(u, 0.0, 1.0) * cells
    j = np.minimum(np.floor(xi).astype(np.int64), cells - 1)
    return j, xi - j


def _coef_columns(j: NDArray[np.int64], cells: int, closed: bool) -> NDArray[np.int64]:
    """Coefficient indices touched by each sample's cell (cyclic when closed)."""
    offsets = np.arange(4, dtype=np.int64)
    if closed:
        return np.mod(j[:, None] - 1 + offsets[None, :], cells)
    return j[:, None] + offsets[None, :]


def _design_matrix(u: F, cells: int, closed: bool, order: int) -> F:
    j, t = _locate(u, cells, closed)
    w = _basis_weights(t, order) * float(cells) ** order
    cols = _coef_columns(j, cells, closed)
    ncoef = cells if closed else cells + 3
    mat = np.zeros((u.shape[0], ncoef), dtype=np.float64)
    rows = np.repeat(np.arange(u.shape[0], dtype=np.int64)[:, None], 4, axis=1)
    np.add.at(mat, (rows, cols), w)
    return mat


def _eval_spline(coeff: F, u: F, cells: int, closed: bool, order: int) -> F:
    j, t = _locate(u, cells, closed)
    w = _basis_weights(t, order) * float(cells) ** order
    cols = _coef_columns(j, cells, closed)
    return np.sum(w * coeff[cols], axis=1)


def _second_difference(cells: int, closed: bool) -> F:
    """The D₂ operator on coefficients (cyclic rows when closed) — the roughness penalty.

    Open fits swap the outermost second-difference rows (two per end) for fourth-difference
    (not-a-knot) rows: the boundary coefficients of an open B-spline are weakly determined by
    point data (the classic interpolation end-condition DOF), and a Δ² penalty there resolves
    them into a natural-spline end — κ collapses to 0 at the ends and rings just inside.
    Δ⁴ ≈ 0 rows instead continue the outer three cells as one cubic (a "double not-a-knot"
    condition), so end curvature is read from the data, not from an artificial boundary
    condition (verified against a clothoid: endpoint κ error drops from +8% to under 2%).
    """
    if closed:
        d = np.zeros((cells, cells), dtype=np.float64)
        idx = np.arange(cells)
        d[idx, (idx - 1) % cells] += 1.0
        d[idx, idx] += -2.0
        d[idx, (idx + 1) % cells] += 1.0
        return d
    ncoef = cells + 3
    rows = ncoef - 2
    d = np.zeros((rows, ncoef), dtype=np.float64)
    idx = np.arange(2, rows - 2)
    d[idx, idx] = 1.0
    d[idx, idx + 1] = -2.0
    d[idx, idx + 2] = 1.0
    nak = np.array([1.0, -4.0, 6.0, -4.0, 1.0])
    d[0, 0:5] = nak
    d[1, 1:6] = nak
    d[rows - 2, ncoef - 6 : ncoef - 1] = nak
    d[rows - 1, ncoef - 5 : ncoef] = nak
    return d


# --- regularization selection ---------------------------------------------------------------


def _match_noise(
    solve: _Solver,
    weights: F,
    noise_std_m: float,
) -> tuple[float, F, float]:
    """Morozov discrepancy: bisect ``log λ`` until residual RMS matches ``noise_std_m``."""
    lo, hi = np.log10(_LAMBDA_FLOOR), _LOG_LAMBDA_MAX
    coeff, rms = solve(_LAMBDA_FLOOR, weights)
    if (
        rms >= noise_std_m
    ):  # even the floor over-smooths (or exact data): keep the floor
        return _LAMBDA_FLOOR, coeff, rms
    coeff_hi, rms_hi = solve(10.0**hi, weights)
    if rms_hi <= noise_std_m:  # pragma: no cover - would need absurd declared noise
        return 10.0**hi, coeff_hi, rms_hi
    for _ in range(_BISECT_ITERS):
        mid = 0.5 * (lo + hi)
        _, rms_mid = solve(10.0**mid, weights)
        if rms_mid < noise_std_m:
            lo = mid
        else:
            hi = mid
    lam = 10.0 ** (0.5 * (lo + hi))
    coeff, rms = solve(lam, weights)
    return lam, coeff, rms


def _adaptive_weights(
    coeff: F, cells: int, closed: bool, ncoef: int, *, adapt_radius_m: float
) -> F:
    """Per-corner penalty weights from a first-pass fit: relax smoothing where |κ| is high.

    ``w = w_min + (1 − w_min) / (1 + (|κ| R_adapt)²)`` — straights (κ → 0) keep full weight,
    apexes tighter than ``R_adapt`` approach the floor, so the discrepancy re-match spends the
    smoothing budget on straights instead of washing out apexes.
    """
    if closed:
        u_c = np.arange(cells, dtype=np.float64) / cells
    else:
        u_c = np.clip(np.arange(ncoef - 2, dtype=np.float64) / cells, 0.0, 1.0)
    dx = _eval_spline(coeff[:, 0], u_c, cells, closed, 1)
    dy = _eval_spline(coeff[:, 1], u_c, cells, closed, 1)
    ddx = _eval_spline(coeff[:, 0], u_c, cells, closed, 2)
    ddy = _eval_spline(coeff[:, 1], u_c, cells, closed, 2)
    speed_sq = np.maximum(dx * dx + dy * dy, 1e-300)
    kappa = np.abs((dx * ddy - dy * ddx) / speed_sq**1.5)
    return _WEIGHT_FLOOR + (1.0 - _WEIGHT_FLOOR) / (1.0 + (kappa * adapt_radius_m) ** 2)


def _arclength_tables(cx: F, cy: F, cells: int, closed: bool) -> tuple[F, F]:
    """Dense parameter → arc-length table for the fitted curve (trapezoidal ∫|S'| du)."""
    m = max(2048, 12 * cells) + 1
    u = np.linspace(0.0, 1.0, m)
    dx = _eval_spline(cx, u, cells, closed, 1)
    dy = _eval_spline(cy, u, cells, closed, 1)
    speed = np.hypot(dx, dy)
    du = u[1] - u[0]
    ds = 0.5 * (speed[1:] + speed[:-1]) * du
    s = np.concatenate(([0.0], np.cumsum(ds)))
    return u, s
