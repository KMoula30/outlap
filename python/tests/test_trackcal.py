# SPDX-License-Identifier: AGPL-3.0-only
"""trackcal tests: synthetic ground-truth geometry (exact/noisy circles, a clothoid S-bend,
closed ovals, mixed-radius circuits), corner detection with robust circle fits, and the
reference-metrics CSV format.

Every scenario measures against analytic truth so estimator bias is *measured*, not assumed.
The noisy-circle pair (penalised fit vs naive interpolating-spline curvature) is the regression
guard for the KTD3 method choice: naive second-derivative curvature from noisy points is a
documented biased anti-pattern and must keep failing the bar the penalised fit passes.

The bias-correction guards (twicing, ``fit_centerline(bias_correction=...)``) live in their own
section at the end. A circle is the *degenerate* case for a curvature estimator — it is what a
rejected candidate was tuned on — so those guards run on mixed-radius closed circuits with real
straights, at more than one discretisation, more than one declared noise, and both handednesses.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from outlap.trackcal import (
    CenterlineFit,
    Corner,
    DegenerateInputError,
    GeorefTransform,
    MetricsFormatError,
    TrackMetrics,
    detect_corners,
    fit_centerline,
    fit_circle,
    load_metrics,
    metrics_from_corners,
    write_metrics,
)
from outlap.trackcal.geometry import (
    design_matrix,
    eval_spline,
    second_difference,
    system_matrix,
)

F = NDArray[np.float64]


# --- Synthetic ground-truth generators ------------------------------------------------------


def _circle(
    radius: float,
    spacing: float,
    *,
    noise: float = 0.0,
    seed: int = 0,
    center: tuple[float, float] = (0.0, 0.0),
) -> tuple[F, F]:
    """A full circle sampled at ~``spacing`` metres, optional per-axis Gaussian noise."""
    n = int(round(2.0 * math.pi * radius / spacing))
    theta = 2.0 * math.pi * np.arange(n, dtype=np.float64) / n
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    if noise > 0.0:
        rng = np.random.default_rng(seed)
        x = x + rng.normal(0.0, noise, n)
        y = y + rng.normal(0.0, noise, n)
    return x, y


def _arc(
    radius: float,
    arc_deg: float,
    n: int,
    *,
    noise: float = 0.0,
    seed: int = 0,
) -> tuple[F, F]:
    """A partial circular arc (short-arc circle-fit scenarios)."""
    theta = np.linspace(0.0, math.radians(arc_deg), n)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    if noise > 0.0:
        rng = np.random.default_rng(seed)
        x = x + rng.normal(0.0, noise, n)
        y = y + rng.normal(0.0, noise, n)
    return x, y


def _clothoid_s_bend(kappa_max: float, length: float, spacing: float) -> tuple[F, F]:
    """Two mirrored clothoids: κ(s) runs linearly +κ_max → −κ_max, sign change at s = L/2."""
    ds = 0.02
    s = np.arange(0.0, length + ds, ds)
    kappa = kappa_max * (1.0 - 2.0 * s / length)
    theta = np.concatenate(([0.0], np.cumsum(0.5 * (kappa[1:] + kappa[:-1]) * ds)))
    cx = np.cos(theta)
    cy = np.sin(theta)
    x = np.concatenate(([0.0], np.cumsum(0.5 * (cx[1:] + cx[:-1]) * ds)))
    y = np.concatenate(([0.0], np.cumsum(0.5 * (cy[1:] + cy[:-1]) * ds)))
    step = int(round(spacing / ds))
    return x[::step], y[::step]


def _stadium(
    radius: float,
    straight: float,
    spacing: float,
    *,
    noise: float = 0.0,
    seed: int = 0,
    roll: int = 0,
) -> tuple[F, F, tuple[float, float]]:
    """A closed stadium oval (two straights + two semicircles), counterclockwise.

    Returns (x, y, (s_apex1, s_apex2)) where the apex positions are the semicircle midpoints
    in the *unrolled* arc-length frame starting at (0, -radius).
    """
    a, r = straight, radius
    total = 2.0 * a + 2.0 * math.pi * r
    s = np.arange(0.0, total, spacing)
    x = np.empty_like(s)
    y = np.empty_like(s)
    m1 = s < a
    x[m1], y[m1] = s[m1], -r
    m2 = (s >= a) & (s < a + math.pi * r)
    phi = (s[m2] - a) / r - 0.5 * math.pi
    x[m2], y[m2] = a + r * np.cos(phi), r * np.sin(phi)
    m3 = (s >= a + math.pi * r) & (s < 2.0 * a + math.pi * r)
    x[m3], y[m3] = a - (s[m3] - a - math.pi * r), r
    m4 = s >= 2.0 * a + math.pi * r
    phi = 0.5 * math.pi + (s[m4] - 2.0 * a - math.pi * r) / r
    x[m4], y[m4] = r * np.cos(phi), r * np.sin(phi)
    if noise > 0.0:
        rng = np.random.default_rng(seed)
        x = x + rng.normal(0.0, noise, s.size)
        y = y + rng.normal(0.0, noise, s.size)
    if roll:
        x, y = np.roll(x, -roll), np.roll(y, -roll)
    apex1 = a + 0.5 * math.pi * r
    apex2 = 2.0 * a + 1.5 * math.pi * r
    return x, y, (apex1, apex2)


@dataclass(frozen=True)
class _Segment:
    """One exact piece of a synthetic circuit: a straight (``radius_m = inf``) or an arc."""

    radius_m: float
    turn_direction: int  # +1 left, −1 right, 0 straight (ISO 8855, z up)
    s_start_m: float
    s_end_m: float

    @property
    def is_corner(self) -> bool:
        return math.isfinite(self.radius_m)

    @property
    def s_apex_m(self) -> float:
        return 0.5 * (self.s_start_m + self.s_end_m)

    def span(self, lo: float, hi: float, n: int) -> F:
        """``n`` arc lengths across the fraction ``[lo, hi]`` of this segment."""
        length = self.s_end_m - self.s_start_m
        return np.linspace(
            self.s_start_m + lo * length, self.s_start_m + hi * length, n
        )


def _unit(v: F) -> F:
    return v / float(np.linalg.norm(v))


def _closed_polygon(headings_deg: Sequence[float], free_lengths: Sequence[float]) -> F:
    """Vertices of the closed polygon whose edges run along ``headings_deg``.

    The trailing edge lengths are given; the first two follow from the closure condition
    ``Σ Lᵢ uᵢ = 0`` (one 2×2 solve), so the loop closes to machine precision instead of
    leaving a seam step the periodic fit would have to absorb.
    """
    u = np.stack(
        [np.cos(np.radians(headings_deg)), np.sin(np.radians(headings_deg))], axis=1
    )
    free = np.asarray(free_lengths, dtype=np.float64)
    rhs = -(free[:, None] * u[2:]).sum(axis=0)
    lengths = np.concatenate(
        [np.linalg.solve(np.stack([u[0], u[1]], axis=1), rhs), free]
    )
    if np.any(lengths <= 0.0):
        raise ValueError(f"polygon does not close with positive edges: {lengths}")
    pts = np.zeros((len(headings_deg), 2), dtype=np.float64)
    for i in range(1, len(headings_deg)):
        pts[i] = pts[i - 1] + lengths[i - 1] * u[i - 1]
    return pts


def _line_at(p0: F, p1: F) -> Callable[[float], tuple[float, float]]:
    direction = _unit(p1 - p0)

    def at(loc: float) -> tuple[float, float]:
        q = p0 + loc * direction
        return float(q[0]), float(q[1])

    return at


def _arc_at(
    centre: F, angle0: float, sign: float, radius: float
) -> Callable[[float], tuple[float, float]]:
    def at(loc: float) -> tuple[float, float]:
        a = angle0 + sign * loc / radius
        return (
            float(centre[0] + radius * math.cos(a)),
            float(centre[1] + radius * math.sin(a)),
        )

    return at


def _rounded_circuit(
    vertices: F,
    radii: Sequence[float],
    spacing: float,
    *,
    noise: float = 0.0,
    seed: int = 0,
    stride: int = 1,
) -> tuple[F, F, list[_Segment]]:
    """A closed circuit: every polygon corner rounded to an exact arc of ``radii[i]``.

    Samples sit at uniform arc length on the exact geometry, so every corner carries an
    analytic truth radius, handedness and arc span. ``stride`` subsamples the *same* noise
    realisation, so changing sample density is a pure discretisation change — with a fresh
    realisation the apex-radius scatter at σ = 1 m swamps it (±10 pp on a 150 m sweeper).
    """
    n = vertices.shape[0]
    starts: list[F] = []
    ends: list[F] = []
    arcs: list[Callable[[float], tuple[float, float]]] = []
    sweeps: list[float] = []
    signs: list[int] = []
    for i in range(n):
        p_prev, p, p_next = vertices[i - 1], vertices[i], vertices[(i + 1) % n]
        d_in, d_out = _unit(p - p_prev), _unit(p_next - p)
        turn = math.atan2(
            float(d_in[0] * d_out[1] - d_in[1] * d_out[0]), float(d_in @ d_out)
        )
        sign = math.copysign(1.0, turn)
        tangent = radii[i] * math.tan(abs(turn) / 2.0)
        start = p - tangent * d_in
        centre = start + sign * radii[i] * np.array([-d_in[1], d_in[0]])
        starts.append(start)
        ends.append(p + tangent * d_out)
        arcs.append(
            _arc_at(
                centre,
                math.atan2(float(start[1] - centre[1]), float(start[0] - centre[0])),
                sign,
                radii[i],
            )
        )
        sweeps.append(abs(turn))
        signs.append(int(sign))

    plan: list[_Segment] = []
    pieces: list[Callable[[float], tuple[float, float]]] = []
    s0 = 0.0
    for i in range(n):
        straight = float(np.linalg.norm(starts[i] - ends[i - 1]))
        if straight < spacing:
            raise ValueError(f"corner {i} leaves no straight ({straight:.1f} m)")
        pieces.append(_line_at(ends[i - 1], starts[i]))
        plan.append(_Segment(math.inf, 0, s0, s0 + straight))
        s0 += straight
        arc = radii[i] * sweeps[i]
        pieces.append(arcs[i])
        plan.append(_Segment(radii[i], signs[i], s0, s0 + arc))
        s0 += arc

    edges = np.array([seg.s_start_m for seg in plan] + [s0], dtype=np.float64)
    s = np.arange(0.0, s0, spacing)
    pts = np.empty((s.size, 2), dtype=np.float64)
    for j, sj in enumerate(s):
        k = min(int(np.searchsorted(edges, sj, side="right")) - 1, len(plan) - 1)
        pts[j] = pieces[k](float(sj) - plan[k].s_start_m)
    if noise > 0.0:
        pts = pts + np.random.default_rng(seed).normal(0.0, noise, pts.shape)
    return pts[::stride, 0].copy(), pts[::stride, 1].copy(), plan


def _mixed_circuit(
    spacing: float = 5.0, *, noise: float = 0.0, seed: int = 0, stride: int = 1
) -> tuple[F, F, list[_Segment]]:
    """780 m closed circuit: 150/30/60/80 m left corners joined by 36–200 m straights."""
    vertices = _closed_polygon([0.0, 120.0, 220.0, 300.0], [180.0, 190.0])
    return _rounded_circuit(
        vertices,
        [150.0, 30.0, 60.0, 80.0],
        spacing,
        noise=noise,
        seed=seed,
        stride=stride,
    )


def _nonconvex_circuit(
    spacing: float = 5.0, *, noise: float = 0.0, seed: int = 0
) -> tuple[F, F, list[_Segment]]:
    """1008 m non-convex closed circuit: four left corners and two right (150 m, 80 m)."""
    vertices = _closed_polygon(
        [0.0, -30.0, 70.0, 30.0, 150.0, 240.0], [140.0, 140.0, 300.0, 320.0]
    )
    return _rounded_circuit(
        vertices, [30.0, 150.0, 60.0, 80.0, 45.0, 40.0], spacing, noise=noise, seed=seed
    )


def _audit(
    fit: CenterlineFit, plan: list[_Segment], corners: list[Corner]
) -> list[tuple[_Segment, Corner]]:
    """Pair every planned corner with the detected corner nearest its apex."""
    scale = fit.length_m / plan[-1].s_end_m  # fitted length drifts by well under 1%
    return [
        (seg, min(corners, key=lambda c: abs(c.s_apex_m - seg.s_apex_m * scale)))
        for seg in plan
        if seg.is_corner
    ]


def _radius_error_pct(seg: _Segment, corner: Corner) -> float:
    return 100.0 * (corner.apex_radius_m - seg.radius_m) / seg.radius_m


def _fitted_radius(fit: CenterlineFit) -> float:
    """Radius estimate from the fit's curvature (median of 1/|κ| over a dense grid)."""
    samples = fit.sample_uniform(0.5)
    return 1.0 / float(np.median(np.abs(samples.kappa_per_m)))


# --- Curvature-first fitting: circles -------------------------------------------------------


@pytest.mark.parametrize("spacing", [2.5, 5.0, 7.0])
def test_exact_circle_radius_within_half_percent(spacing: float) -> None:
    """Exact R=34 m circle at 10 Hz-equivalent spacing: fitted radius within 0.5%."""
    x, y = _circle(34.0, spacing)
    fit = fit_centerline(x, y, closed=True, noise_std_m=0.0)
    assert _fitted_radius(fit) == pytest.approx(34.0, rel=5e-3)
    assert fit.length_m == pytest.approx(2.0 * math.pi * 34.0, rel=5e-3)


def test_noisy_circle_radius_unbiased_within_two_percent() -> None:
    """FastF1-like noise (σ = 0.3 m per axis): penalised fit stays unbiased within 2%."""
    x, y = _circle(34.0, 5.0, noise=0.3, seed=42)
    fit = fit_centerline(x, y, closed=True, noise_std_m=0.3)
    assert _fitted_radius(fit) == pytest.approx(34.0, rel=2e-2)


def test_naive_interpolating_spline_curvature_fails_noisy_circle() -> None:
    """Regression guard for KTD3: naive interpolating-spline second-derivative curvature on
    the *same* noisy circle is biased beyond the 2% bar the penalised fit passes.

    ``smoothing=0.0`` with knot spacing = sample spacing collapses the penalised fit into the
    naive interpolating spline — the documented anti-pattern (De Brabanter et al. 2013)."""
    x, y = _circle(34.0, 5.0, noise=0.3, seed=42)
    good = fit_centerline(x, y, closed=True, noise_std_m=0.3)
    naive = fit_centerline(
        x, y, closed=True, smoothing=0.0, knot_spacing_m=5.0, adaptive=False
    )
    err_good = abs(_fitted_radius(good) - 34.0) / 34.0
    err_naive = abs(_fitted_radius(naive) - 34.0) / 34.0
    assert err_good < 2e-2
    assert err_naive > 2e-2  # the anti-pattern must keep failing the bar
    assert err_naive > err_good


def test_fit_is_deterministic() -> None:
    """Same inputs → bit-identical fit output (no hidden RNG in the fit)."""
    x, y = _circle(34.0, 5.0, noise=0.3, seed=3)
    a = fit_centerline(x, y, closed=True, noise_std_m=0.3)
    b = fit_centerline(x, y, closed=True, noise_std_m=0.3)
    sa, sb = a.sample_uniform(1.0), b.sample_uniform(1.0)
    assert np.array_equal(sa.x_m, sb.x_m)
    assert np.array_equal(sa.y_m, sb.y_m)
    assert np.array_equal(sa.kappa_per_m, sb.kappa_per_m)


# --- Clothoid S-bend: sign change + no ringing ----------------------------------------------


def test_clothoid_s_bend_sign_change_and_no_ringing() -> None:
    """κ sign change located within one sample spacing; no overshoot past |κ_max|."""
    kappa_max, length, spacing = 1.0 / 60.0, 240.0, 3.0
    x, y = _clothoid_s_bend(kappa_max, length, spacing)
    fit = fit_centerline(x, y, closed=False, noise_std_m=0.0)
    samples = fit.sample_uniform(0.25)
    kappa = samples.kappa_per_m
    sign_change = np.nonzero(np.diff(np.sign(kappa)) != 0)[0]
    assert sign_change.size >= 1
    s_cross = float(samples.s_m[sign_change[0]])
    assert abs(s_cross - 0.5 * length) <= spacing  # within one sample
    assert float(np.max(np.abs(kappa))) <= kappa_max * 1.03  # no overshoot ringing


# --- Closed periodic fit: C² at the seam ----------------------------------------------------


def test_closed_oval_curvature_continuous_across_seam() -> None:
    """Periodic fit is C² at s = 0: curvature continuous across the seam (seam mid-corner)."""
    x, y, (apex1, _) = _stadium(40.0, 150.0, 4.0)
    k = int(round(apex1 / 4.0))  # roll the seam into the middle of the first semicircle
    x, y = np.roll(x, -k), np.roll(y, -k)
    fit = fit_centerline(x, y, closed=True, noise_std_m=0.0)
    eps = 0.01
    k_lo = float(fit.curvature(np.array([eps]))[0])
    k_hi = float(fit.curvature(np.array([fit.length_m - eps]))[0])
    assert k_lo == pytest.approx(1.0 / 40.0, rel=0.05)  # the seam sits mid-corner
    assert abs(k_hi - k_lo) < 1e-4  # curvature continuous across the seam
    # Evaluation wraps: κ(s + L) is the same point as κ(s).
    s_probe = np.array([10.0, 100.0])
    assert np.allclose(
        fit.curvature(s_probe), fit.curvature(s_probe + fit.length_m), atol=1e-12
    )


# --- Corner detection + robust apex extraction ----------------------------------------------


def test_two_corner_track_detection() -> None:
    """Both stadium apexes found, circle-fit radii ≈ 40 m, windows disjoint."""
    x, y, (apex1, apex2) = _stadium(40.0, 150.0, 4.0)
    fit = fit_centerline(x, y, closed=True, noise_std_m=0.0)
    corners = detect_corners(fit)
    assert len(corners) == 2
    half_arc = 0.5 * math.pi * 40.0
    assert abs(corners[0].s_apex_m - apex1) <= half_arc
    assert abs(corners[1].s_apex_m - apex2) <= half_arc
    for c in corners:
        assert c.apex_radius_m == pytest.approx(40.0, rel=2e-2)
        assert c.turn_direction == 1  # counterclockwise → left turns (ISO 8855, z up)
        assert c.window_start_m < c.window_end_m
    # Windows must not overlap.
    assert corners[0].window_end_m < corners[1].window_start_m


def test_two_corner_track_noisy_with_apex_speeds() -> None:
    """Noisy stadium: adaptive fit keeps apex radii honest; robust apex speeds extracted."""
    x, y, (apex1, apex2) = _stadium(40.0, 150.0, 4.0, noise=0.2, seed=7)
    fit = fit_centerline(x, y, closed=True, noise_std_m=0.2)
    total = 2.0 * 150.0 + 2.0 * math.pi * 40.0
    s_v = np.arange(0.0, total, 2.0)
    v = (
        40.0
        - 22.0 * np.exp(-(((s_v - apex1) / 25.0) ** 2))
        - 22.0 * np.exp(-(((s_v - apex2) / 25.0) ** 2))
    )
    corners = detect_corners(fit, speed=(s_v, v))
    assert len(corners) == 2
    for c in corners:
        assert c.apex_radius_m == pytest.approx(40.0, rel=5e-2)
        assert c.apex_speed_mps is not None
        assert 17.5 <= c.apex_speed_mps <= 20.5  # robust minimum near the 18 m/s trough


# --- Robust circle fits (Hyper init + geometric refinement) ---------------------------------


def test_circle_fit_exact_full_circle() -> None:
    x, y = _circle(34.0, 5.0)
    c = fit_circle(x, y)
    assert c.radius_m == pytest.approx(34.0, rel=1e-3)
    assert c.rms_m < 1e-6


def test_circle_fit_exact_short_arc() -> None:
    """A 60° arc — algebraic fits alone are biased on short arcs; Hyper + refinement is not."""
    x, y = _arc(34.0, 60.0, 15)
    c = fit_circle(x, y)
    assert c.radius_m == pytest.approx(34.0, rel=1e-3)


def test_circle_fit_noisy_quarter_arc() -> None:
    x, y = _arc(34.0, 90.0, 40, noise=0.3, seed=5)
    c = fit_circle(x, y)
    assert c.radius_m == pytest.approx(34.0, rel=5e-2)


def test_circle_fit_collinear_points_is_degenerate() -> None:
    x = np.linspace(0.0, 100.0, 20)
    y = 2.0 * x + 1.0
    with pytest.raises(DegenerateInputError):
        fit_circle(x, y)


# --- Degenerate inputs ----------------------------------------------------------------------


def test_straight_line_input_is_degenerate() -> None:
    x = np.linspace(0.0, 100.0, 30)
    y = np.zeros_like(x)
    with pytest.raises(DegenerateInputError):
        fit_centerline(x, y, closed=False)


def test_too_few_points_is_degenerate() -> None:
    x, y = _circle(34.0, 30.0)  # 7 points < 10
    with pytest.raises(DegenerateInputError):
        fit_centerline(x, y, closed=True)


# --- Reference-metrics CSV format -----------------------------------------------------------


def _sample_metrics() -> TrackMetrics:
    return TrackMetrics(
        label="catalunya_osm",
        source_session="2026 Spanish Grand Prix R",
        transform=GeorefTransform(
            scale=0.9987654321012345,
            rotation_rad=0.5235987755982988,
            tx_m=-102345.6789012345,
            ty_m=4321987.123456789,
            residual_rms_m=0.8471,
        ),
        corner=np.array([1, 2, 3], dtype=np.int64),
        s_m=np.array([412.37891, 1002.5, 3999.999999], dtype=np.float64),
        radius_m=np.array([34.0612345, 87.5, 210.123456789], dtype=np.float64),
        apex_speed_mps=np.array([24.83, np.nan, 61.72], dtype=np.float64),
    )


def test_metrics_csv_round_trip_exact(tmp_path: Path) -> None:
    """Write-then-read preserves radii, speeds, and the transform header exactly (KTD7)."""
    metrics = _sample_metrics()
    path = tmp_path / "catalunya_track_metrics.csv"
    write_metrics(path, metrics)
    back = load_metrics(path)
    assert back.label == metrics.label
    assert back.source_session == metrics.source_session
    assert (
        back.transform == metrics.transform
    )  # exact float equality via repr round-trip
    assert np.array_equal(back.corner, metrics.corner)
    assert np.array_equal(back.s_m, metrics.s_m)
    assert np.array_equal(back.radius_m, metrics.radius_m)
    assert np.array_equal(back.apex_speed_mps, metrics.apex_speed_mps, equal_nan=True)


def test_metrics_missing_format_header_is_typed_error(tmp_path: Path) -> None:
    path = tmp_path / "bad.csv"
    path.write_text("corner,s_m,radius_m,apex_speed_mps\n1,10.0,34.0,20.0\n")
    with pytest.raises(MetricsFormatError):
        load_metrics(path)


def test_metrics_missing_transform_is_typed_error(tmp_path: Path) -> None:
    metrics = _sample_metrics()
    path = tmp_path / "metrics.csv"
    write_metrics(path, metrics)
    lines = [
        ln for ln in path.read_text().splitlines() if not ln.startswith("# georef:")
    ]
    path.write_text("\n".join(lines) + "\n")
    with pytest.raises(MetricsFormatError):
        load_metrics(path)


def test_metrics_from_corners(tmp_path: Path) -> None:
    """Detected corners flow into the metrics record (None speed → NaN)."""
    x, y, _ = _stadium(40.0, 150.0, 4.0)
    fit = fit_centerline(x, y, closed=True, noise_std_m=0.0)
    corners = detect_corners(fit)
    transform = GeorefTransform(
        scale=1.0, rotation_rad=0.0, tx_m=0.0, ty_m=0.0, residual_rms_m=0.0
    )
    metrics = metrics_from_corners(
        corners, label="stadium", source_session="synthetic", transform=transform
    )
    assert metrics.corner.tolist() == [1, 2]
    assert np.array_equal(
        metrics.radius_m, np.array([c.apex_radius_m for c in corners])
    )
    assert np.all(np.isnan(metrics.apex_speed_mps))  # no speed source provided
    path = tmp_path / "stadium_metrics.csv"
    write_metrics(path, metrics)
    assert np.array_equal(load_metrics(path).radius_m, metrics.radius_m)


# --- Live FastF1 loader stays opt-in --------------------------------------------------------


def test_live_fastf1_loader_is_opt_in() -> None:
    """The live FastF1 position loader is exercised only when fastf1 exists (never in CI)."""
    pytest.importorskip("fastf1")
    from outlap.trackcal import load_fastf1_positions

    assert callable(load_fastf1_positions)


# --- Bias correction (twicing): the guards the method choice rests on ------------------------
#
# The discrepancy principle shrinks a closed convex curve's radius by √2·σ whatever the
# penalty, so the fit ships one guarded twicing step. These tests are the gates that step had
# to clear: discretisation invariance (T1), curvature-noise transfer (T2), handedness (T3),
# the product path (T4), the mis-declaration veto (T6), the honest-reporting contract (T7) and
# the bit-identity escape hatch (T8). Circles are deliberately *not* the evidence here.


def test_mixed_radius_apex_radii_are_discretisation_invariant() -> None:
    """T1: the audited apex radii must not depend on knot spacing or sample density.

    A method whose bias rides on the discretisation is unusable across track sources (a
    rejected candidate spanned 33 pp with a sign flip on the equivalent grid). Measured on
    exact samples — where the *whole* spread is discretisation — over knot ∈ {2,3,5} m ×
    sample ∈ {5,10} m, every corner holds inside 0.21 pp against a 3 pp bar. The seeded-noise
    pass then pins the knot axis at a fixed sample set; the sample axis is not re-asserted
    under noise because at σ = 1 m the realisation scatter on the 150 m sweeper (±10 pp) is an
    order of magnitude above the discretisation effect being measured.
    """
    grid = [(knot, stride) for knot in (2.0, 3.0, 5.0) for stride in (1, 2)]

    exact: dict[float, list[float]] = {}
    for knot, stride in grid:
        x, y, plan = _mixed_circuit(5.0, stride=stride)
        fit = fit_centerline(x, y, closed=True, noise_std_m=1.0, knot_spacing_m=knot)
        corners = detect_corners(fit)
        assert len(corners) == 4
        for seg, corner in _audit(fit, plan, corners):
            exact.setdefault(seg.radius_m, []).append(_radius_error_pct(seg, corner))
    assert set(exact) == {30.0, 60.0, 80.0, 150.0}
    for radius, errors in exact.items():
        assert max(errors) - min(errors) < 3.0, f"R={radius} spread {errors}"

    for stride in (1, 2):
        noisy: dict[float, list[float]] = {}
        for knot in (2.0, 3.0, 5.0):
            x, y, plan = _mixed_circuit(5.0, noise=1.0, seed=4, stride=stride)
            fit = fit_centerline(
                x, y, closed=True, noise_std_m=1.0, knot_spacing_m=knot
            )
            for seg, corner in _audit(fit, plan, detect_corners(fit)):
                noisy.setdefault(seg.radius_m, []).append(
                    _radius_error_pct(seg, corner)
                )
        for radius, errors in noisy.items():
            assert max(errors) - min(errors) < 3.0, (
                f"R={radius} stride {stride} {errors}"
            )


def test_curvature_noise_transfer_stays_inside_two_times_uncorrected() -> None:
    """T2: exact (Monte-Carlo-free) noise transfer into κ, and the interpolation-drift check.

    Twicing buys apex accuracy with curvature variance, so the amount is the gate:
    ``sd(κ)(u) = σ ‖b₂(u) C‖₂ / speed²`` for the coefficient operator ``C`` the fit actually
    used (``A₁ = M⁻¹Bᵀ`` uncorrected, ``2A₁ − A₁BA₁`` corrected). Measured 1.66× on straights
    and 1.46–1.63× at the apexes — inside 2×, which is why exactly one step ships (two are
    2.19× on straights). ``effective_dof`` must also *fall* as the declared noise rises: a
    correction drifting toward interpolation would push it up.
    """
    sigma_dof: dict[float, float] = {}
    for sigma in (1.0, 2.0):
        x, y, plan = _mixed_circuit(5.0, noise=1.0, seed=11)
        # adaptive=False so the reported λ alone reconstructs the operator (uniform weights).
        on = fit_centerline(x, y, closed=True, noise_std_m=sigma, adaptive=False)
        off = fit_centerline(
            x, y, closed=True, noise_std_m=sigma, adaptive=False, bias_correction=False
        )
        assert on.bias_corrected and not off.bias_corrected
        assert on.smoothing_lambda == off.smoothing_lambda  # twicing never moves λ
        sigma_dof[sigma] = on.effective_dof
        assert (
            on.effective_dof > off.effective_dof
        )  # the boosted operator is less smooth
        for seg in plan:
            window = (
                seg.span(0.2, 0.8, 25)
                if not seg.is_corner
                else seg.span(0.25, 0.75, 25)
            )
            s_eval = window * (on.length_m / plan[-1].s_end_m)
            ratio = float(
                np.mean(_kappa_noise_sd(on, x, y, sigma, s_eval))
                / np.mean(_kappa_noise_sd(off, x, y, sigma, s_eval))
            )
            assert 1.0 < ratio < 2.0, f"{seg.radius_m} m: κ-noise transfer {ratio:.2f}×"
    assert sigma_dof[2.0] < sigma_dof[1.0]  # more declared noise ⇒ fewer effective dof


def _kappa_noise_sd(fit: CenterlineFit, x_m: F, y_m: F, sigma: float, s_eval: F) -> F:
    """Exact sd of the fitted κ under per-axis σ noise: ``σ ‖b₂(u) C‖₂ / speed²``.

    Rebuilt from the fit's own recorded λ through the shared smoother kernel, so it measures
    the operator that was actually reported (uniform weights only — hence ``adaptive=False``).
    """
    pts = np.stack([x_m, y_m], axis=1)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    seg = np.concatenate([seg, [float(np.hypot(*(pts[0] - pts[-1])))]])
    u = np.concatenate(([0.0], np.cumsum(seg[:-1]))) / float(np.sum(seg))
    b_mat = design_matrix(u, fit.cells, fit.closed, 0)
    d_mat = second_difference(fit.cells, fit.closed)
    m_mat = system_matrix(
        b_mat.T @ b_mat, d_mat, fit.smoothing_lambda, np.ones(d_mat.shape[0])
    )
    a1 = np.linalg.solve(m_mat, b_mat.T)
    coef_op = 2.0 * a1 - a1 @ (b_mat @ a1) if fit.bias_corrected else a1
    u_eval = np.interp(np.mod(s_eval, fit.length_m), fit.s_grid, fit.u_grid)
    b2 = design_matrix(u_eval, fit.cells, fit.closed, 2)
    dx = eval_spline(fit.coeff_x, u_eval, fit.cells, fit.closed, 1)
    dy = eval_spline(fit.coeff_y, u_eval, fit.cells, fit.closed, 1)
    return sigma * np.linalg.norm(b2 @ coef_op, axis=1) / (dx * dx + dy * dy)


def test_nonconvex_circuit_apex_errors_share_one_sign() -> None:
    """T3: on a loop that turns both ways, left- and right-hand errors must agree in sign.

    An estimator with a direction (an offset mode, a frame, a handed penalty) reads one
    handedness large and the other small; twicing is an isotropic operator on the residual and
    has none. Four left corners (30/60/45/40 m) and two right (150/80 m), σ = 1 m.
    """
    x, y, plan = _nonconvex_circuit(5.0, noise=1.0, seed=0)
    fit = fit_centerline(x, y, closed=True, noise_std_m=1.0)
    corners = detect_corners(fit)
    assert len(corners) == 6

    left: list[float] = []
    right: list[float] = []
    for seg, corner in _audit(fit, plan, corners):
        assert corner.turn_direction == seg.turn_direction
        error = _radius_error_pct(seg, corner)
        assert abs(error) < 30.0, f"R={seg.radius_m} error {error:+.1f}%"
        (left if seg.turn_direction > 0 else right).append(error)
    assert len(right) == 2
    # Same sign, and neither handedness is an outlier against the other.
    assert math.copysign(1.0, np.mean(left)) == math.copysign(1.0, np.mean(right))
    assert abs(float(np.mean(left)) - float(np.mean(right))) < 15.0


def test_detect_corners_product_path_at_two_noise_levels() -> None:
    """T4: the shipped path — fit then ``detect_corners`` — at σ = 1 m and σ = 2 m.

    Every true corner must be found with a sane radius, no corner may be invented on the
    200 m straight, and the straight's spurious ``max|κ|`` is recorded as the implied phantom
    radius (152–239 m measured; the fit is inside the 200 m detection gate and survives on
    ``min_arc_m`` alone, so the margin is worth watching). The exact-count assertion is scoped
    to σ = 1 m: at σ = 2 m the 150 m sweeper sits only 33% above the gate and fragments into
    two runs — measured identically with ``bias_correction=False``, so it is the geometry
    against the gate, not the correction.
    """
    straight = 200.0  # the long straight, 322–522 m of the plan
    for sigma in (1.0, 2.0):
        for seed in (0, 1, 2):
            x, y, plan = _mixed_circuit(5.0, noise=sigma, seed=seed)
            fit = fit_centerline(x, y, closed=True, noise_std_m=sigma)
            corners = detect_corners(fit)
            scale = fit.length_m / plan[-1].s_end_m
            if sigma == 1.0:
                assert len(corners) == 4, f"seed {seed}: {len(corners)} corners"
            bar = 30.0 if sigma == 1.0 else 45.0  # measured worst 22.7% / 38.7%
            for seg, corner in _audit(fit, plan, corners):
                # The apex must land on the true arc (a fragmented sweeper at σ = 2 m puts it
                # off-centre, but never on the wrong feature).
                assert (
                    seg.s_start_m * scale - 10.0
                    <= corner.s_apex_m
                    <= seg.s_end_m * scale + 10.0
                ), f"R={seg.radius_m} apex at s={corner.s_apex_m:.0f} m"
                assert abs(_radius_error_pct(seg, corner)) < bar

            long_straight = max(
                (s for s in plan if not s.is_corner),
                key=lambda s: s.s_end_m - s.s_start_m,
            )
            assert long_straight.s_end_m - long_straight.s_start_m == pytest.approx(
                straight, abs=1.0
            )
            probe = long_straight.span(0.05, 0.95, 80) * scale
            phantom_radius = 1.0 / float(np.max(np.abs(fit.curvature(probe))))
            assert phantom_radius > 100.0, (
                f"σ={sigma} seed {seed}: {phantom_radius:.0f} m"
            )
            for corner in corners:
                inside = (
                    long_straight.s_start_m * scale + 10.0
                    < corner.s_apex_m
                    < long_straight.s_end_m * scale - 10.0
                )
                assert not inside, f"phantom corner at s={corner.s_apex_m:.0f} m"


def test_underdeclared_noise_vetoes_the_bias_correction() -> None:
    """T6: declaring 0.3 m for 1.0 m noise must veto the step, never amplify the error.

    Under-declaring makes λ under-smooth, so what is left in the residual is noise; boosting
    it is actively harmful (−40% → −52% in the panel's measurement). The veto compares the
    step against what pure declared-σ noise pushes through the same operator, and fires here
    and only here — at a correct or over-stated declaration the step runs and pays.
    """
    for seed in range(4):
        x, y = _circle(34.0, 5.0, noise=1.0, seed=seed)
        vetoed = fit_centerline(x, y, closed=True, noise_std_m=0.3)
        plain = fit_centerline(
            x, y, closed=True, noise_std_m=0.3, bias_correction=False
        )
        assert vetoed.bias_corrected is False
        assert np.array_equal(vetoed.coeff_x, plain.coeff_x)  # the veto is a full skip
        assert abs(_fitted_radius(vetoed) - 34.0) <= abs(_fitted_radius(plain) - 34.0)

        honest = fit_centerline(x, y, closed=True, noise_std_m=1.0)
        assert honest.bias_corrected is True
        assert abs(_fitted_radius(honest) - 34.0) < abs(_fitted_radius(plain) - 34.0)


def test_declared_noise_stays_load_bearing_and_reported_honestly() -> None:
    """T7: the reporting contract — ``noise_std_m`` can never go inert, ``λ`` can never lie.

    ``discrepancy_rms_m`` is the residual the λ search matched, so it equals the declaration
    unless the search saturated (``lambda_capped``, measured live at σ = 3 m on this circuit —
    a pre-existing hole that used to fail silently). ``residual_rms_m`` is the *reported*
    curve's residual and drops to ~0.65–0.8 σ once corrected: still exactly what it claims,
    but no longer a tautology.
    """
    x, y, _ = _mixed_circuit(5.0, noise=1.0, seed=3)
    capped_seen = False
    for sigma in (0.3, 1.0, 2.0, 3.0):
        fit = fit_centerline(x, y, closed=True, noise_std_m=sigma)
        if fit.lambda_capped:
            capped_seen = True
            assert fit.discrepancy_rms_m < sigma
            assert fit.smoothing_lambda == pytest.approx(1e6)
        else:
            assert fit.discrepancy_rms_m == pytest.approx(sigma, rel=1e-6)
        assert fit.residual_rms_m <= fit.discrepancy_rms_m * (1.0 + 1e-9)
        assert 0.0 < fit.effective_dof <= float(x.size)
    assert capped_seen, "σ = 3 m must saturate the λ ceiling on this circuit"

    # Declared noise is the knob: two declarations can never produce the same curve.
    x, y = _circle(34.0, 5.0, noise=1.0, seed=2)
    low = fit_centerline(x, y, closed=True, noise_std_m=1.0).sample_uniform(1.0)
    high = fit_centerline(x, y, closed=True, noise_std_m=3.0).sample_uniform(1.0)
    n = min(low.x_m.size, high.x_m.size)
    separation = float(
        np.sqrt(
            np.mean(
                (low.x_m[:n] - high.x_m[:n]) ** 2 + (low.y_m[:n] - high.y_m[:n]) ** 2
            )
        )
    )
    assert separation > 0.5


def test_bias_correction_off_reproduces_the_uncorrected_operator() -> None:
    """T8: ``bias_correction=False`` is the bit-identity escape hatch.

    Two independent proofs. (1) In-process: the flag-off fit is *exactly* the penalised solve
    at the λ the discrepancy search picked — bit-identical to asking for that λ explicitly, so
    nothing in the correction path perturbs the base operator. (2) Against the shipped
    pre-correction implementation: λ, residual, fitted length and radius pinned to the values
    that implementation produced for this fixture (captured at 17 significant digits; they
    reproduce exactly).
    """
    x, y = _circle(34.0, 5.0, noise=0.3, seed=42)

    off = fit_centerline(
        x, y, closed=True, noise_std_m=0.3, adaptive=False, bias_correction=False
    )
    explicit = fit_centerline(
        x, y, closed=True, smoothing=off.smoothing_lambda, adaptive=False
    )
    assert np.array_equal(off.coeff_x, explicit.coeff_x)
    assert np.array_equal(off.coeff_y, explicit.coeff_y)
    assert off.residual_rms_m == explicit.residual_rms_m
    assert (
        explicit.bias_corrected is False
    )  # an explicit λ means "exactly that operator"

    on = fit_centerline(x, y, closed=True, noise_std_m=0.3, adaptive=False)
    assert on.bias_corrected is True
    assert not np.array_equal(on.coeff_x, off.coeff_x)

    # Pre-correction golden (adaptive path, the shipped default before this change).
    pinned = fit_centerline(x, y, closed=True, noise_std_m=0.3, bias_correction=False)
    assert pinned.smoothing_lambda == pytest.approx(2268.725965519367, rel=1e-12)
    assert pinned.residual_rms_m == pytest.approx(0.30000000000401539, rel=1e-12)
    assert pinned.length_m == pytest.approx(211.19245285815461, rel=1e-12)
    assert _fitted_radius(pinned) == pytest.approx(33.716961302237053, rel=1e-12)
    assert pinned.discrepancy_rms_m == pinned.residual_rms_m
    assert pinned.bias_corrected is False
