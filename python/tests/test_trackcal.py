# SPDX-License-Identifier: AGPL-3.0-only
"""trackcal tests: synthetic ground-truth geometry (exact/noisy circles, a clothoid S-bend,
closed ovals), corner detection with robust circle fits, and the reference-metrics CSV format.

Every scenario measures against analytic truth so estimator bias is *measured*, not assumed.
The noisy-circle pair (penalised fit vs naive interpolating-spline curvature) is the regression
guard for the KTD3 method choice: naive second-derivative curvature from noisy points is a
documented biased anti-pattern and must keep failing the bar the penalised fit passes.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from outlap.trackcal import (
    CenterlineFit,
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
