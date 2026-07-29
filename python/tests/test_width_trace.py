# SPDX-License-Identifier: AGPL-3.0-only
"""Offline tests for the semi-automated track-width tracer (`outlap.importers.width_trace`).

Every fixture is a synthetic orthophoto drawn in-memory through the same `ImageSource` seam the
real tile fetchers plug into later — no network, no real imagery, CI-safe. The load-bearing
contracts:

* widths are NEVER silently defaulted — unresolved stations are a typed error (R1);
* a hand-placed control point wins over the detected edge in its window and blends smoothly;
* an intensity feature outside the search band (a pit lane) must not widen the track;
* cross-check disagreement flags stations in the provenance but never mutates widths.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import ModuleType
from typing import cast

import numpy as np
import pytest
import yaml

from outlap.importers import width_trace as wt

RADIUS_M = 80.0
GRASS = 0.75
ASPHALT = 0.30
# Search band shared by most tests: the drawn edges (4–7 m) sit inside it; the pit lane outside.
PARAMS = wt.TraceParams(search_min_m=2.0, search_max_m=9.0, step_m=0.1)


def _circle_stations(n: int = 96, radius: float = RADIUS_M) -> wt.Stations:
    """A CCW circle: heading is the tangent, so LEFT (+y, ISO 8855) points INWARD."""
    theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return wt.Stations(
        s_m=radius * theta,
        x_m=radius * np.cos(theta),
        y_m=radius * np.sin(theta),
        heading_rad=theta + math.pi / 2.0,
        length_m=2.0 * math.pi * radius,
    )


def _annulus_source(
    width_left: float = 6.0,
    width_right: float = 6.0,
    *,
    pit: tuple[float, float] | None = None,
    wedge_halfwidth_rad: float | None = None,
    pixel_m: float = 0.2,
    radius: float = RADIUS_M,
) -> wt.ArrayImageSource:
    """Draw the CCW circle track: a dark asphalt annulus on light grass.

    Left of travel points toward the circle centre, so ``width_left`` is the inward
    half-width (inner edge at ``r = radius - width_left``) and ``width_right`` the outward
    one. ``pit=(r_lo, r_hi)`` paints a second dark annulus (a parallel pit lane);
    ``wedge_halfwidth_rad`` overwrites the sector ``|atan2(y, x)| <= value`` with flat grey
    (no edges there → those stations are unresolved).
    """
    half = radius + width_right + 25.0
    tf = wt.AffineTransform.from_origin(-half, half, pixel_m)
    npx = int(round(2.0 * half / pixel_m))
    xc, yc = tf.pixel_centers((npx, npx))
    r = np.hypot(xc, yc)
    img = np.full((npx, npx), GRASS)
    img[(r >= radius - width_left) & (r <= radius + width_right)] = ASPHALT
    if pit is not None:
        img[(r >= pit[0]) & (r <= pit[1])] = ASPHALT
    if wedge_halfwidth_rad is not None:
        img[np.abs(np.arctan2(yc, xc)) <= wedge_halfwidth_rad] = 0.5
    return wt.ArrayImageSource(img, tf)


# --- image seam ------------------------------------------------------------------------------


def test_array_image_source_bilinear_sampling() -> None:
    # from_origin(west=0, north=2, 1 m px): pixel centres at (0.5,1.5) (1.5,1.5) (0.5,0.5)...
    tf = wt.AffineTransform.from_origin(0.0, 2.0, 1.0)
    src = wt.ArrayImageSource(np.array([[0.0, 1.0], [0.5, 1.0]]), tf)
    v = src.sample(np.array([0.5, 1.5, 0.5, 1.0]), np.array([1.5, 1.5, 0.5, 1.5]))
    assert v == pytest.approx([0.0, 1.0, 0.5, 0.5])


# --- automatic edge detection ----------------------------------------------------------------


def test_traced_widths_within_tolerance() -> None:
    st = _circle_stations()
    res = wt.trace_widths(st, _annulus_source(6.0, 6.0), params=PARAMS)
    total = res.width_left_m + res.width_right_m
    assert float(np.max(np.abs(total - 12.0))) <= 0.25
    assert float(np.max(np.abs(res.width_left_m - 6.0))) <= 0.2
    assert float(np.max(np.abs(res.width_right_m - 6.0))) <= 0.2


def test_sides_mapped_left_inward_right_outward() -> None:
    # Asymmetric widths catch a side swap: LEFT is inward on a CCW circle.
    st = _circle_stations()
    res = wt.trace_widths(st, _annulus_source(4.0, 7.0), params=PARAMS)
    assert float(np.max(np.abs(res.width_left_m - 4.0))) <= 0.2
    assert float(np.max(np.abs(res.width_right_m - 7.0))) <= 0.2


def test_pit_lane_outside_band_does_not_widen() -> None:
    # A parallel dark branch 11–14 m inboard: outside the 9 m search band → left stays ~6 m.
    st = _circle_stations()
    src = _annulus_source(6.0, 6.0, pit=(RADIUS_M - 14.0, RADIUS_M - 11.0))
    res = wt.trace_widths(st, src, params=PARAMS)
    assert float(np.max(res.width_left_m)) < 7.0
    assert float(np.max(np.abs(res.width_left_m - 6.0))) <= 0.2


# --- control points --------------------------------------------------------------------------


def test_control_point_wins_and_blends_smoothly() -> None:
    st = _circle_stations(n=160)  # ~3.1 m spacing resolves the blend ramp
    k = 40
    cp = wt.ControlPoint(s_m=float(st.s_m[k]), side="left", offset_m=8.0)
    res = wt.trace_widths(
        st, _annulus_source(6.0, 6.0), params=PARAMS, control_points=[cp]
    )
    # The control point wins outright at its own station...
    assert float(res.width_left_m[k]) == pytest.approx(8.0, abs=0.05)
    # ...blends smoothly (a hard override would step 2.0 m between adjacent stations)...
    assert float(np.max(np.abs(np.diff(res.width_left_m)))) < 0.6
    # ...and leaves stations beyond the blend window, and the other side, untouched.
    far = np.abs(st.s_m - cp.s_m) > PARAMS.blend_window_m + 5.0
    assert float(np.max(np.abs(res.width_left_m[far] - 6.0))) <= 0.2
    assert float(np.max(np.abs(res.width_right_m - 6.0))) <= 0.2


def test_control_point_rescues_unresolved_stations() -> None:
    # A grey wedge (±0.25 rad ≈ ±20 m of arc) kills detection; control points at s=0 cover it
    # (including across the s-wrap) so no error is raised and the wedge takes the CP width.
    st = _circle_stations()
    src = _annulus_source(6.0, 6.0, wedge_halfwidth_rad=0.25)
    params = wt.TraceParams(
        search_min_m=2.0, search_max_m=9.0, step_m=0.1, blend_window_m=30.0
    )
    cps = [
        wt.ControlPoint(s_m=0.0, side="left", offset_m=7.0),
        wt.ControlPoint(s_m=0.0, side="right", offset_m=7.0),
    ]
    res = wt.trace_widths(st, src, params=params, control_points=cps)
    assert float(res.width_left_m[0]) == pytest.approx(7.0, abs=1e-6)
    assert float(res.width_right_m[0]) == pytest.approx(7.0, abs=1e-6)


def test_control_point_csv_roundtrip(tmp_path: Path) -> None:
    p = tmp_path / "cps.csv"
    p.write_text("# hand QA 2026\ns_m,side,offset_m\n120.0,left,7.5\n45.5,right,4.25\n")
    assert wt.load_control_points(p) == [
        wt.ControlPoint(s_m=120.0, side="left", offset_m=7.5),
        wt.ControlPoint(s_m=45.5, side="right", offset_m=4.25),
    ]
    bad = tmp_path / "bad.csv"
    bad.write_text("10.0,up,3.0\n")
    with pytest.raises(wt.WidthTraceError):
        wt.load_control_points(bad)


# --- R1: error, never default ----------------------------------------------------------------


def test_unresolved_stations_raise_typed_error() -> None:
    st = _circle_stations()
    src = _annulus_source(6.0, 6.0, wedge_halfwidth_rad=0.35)
    with pytest.raises(wt.UnresolvedStationsError) as excinfo:
        wt.trace_widths(st, src, params=PARAMS)
    err = excinfo.value
    assert isinstance(err, wt.WidthTraceError)
    assert err.stations
    theta = st.s_m / RADIUS_M
    ang = np.minimum(theta, 2.0 * math.pi - theta)
    # Every station deep inside the wedge is reported, on both sides...
    reported_left = {s for s, side in err.stations if side == "left"}
    reported_right = {s for s, side in err.stations if side == "right"}
    for s_val in st.s_m[ang <= 0.2]:
        assert any(abs(float(s_val) - r) < 1e-9 for r in reported_left)
        assert any(abs(float(s_val) - r) < 1e-9 for r in reported_right)
    # ...nothing outside the wedge is, and the message names the stations.
    for s_val, _side in err.stations:
        a = min(s_val / RADIUS_M, 2.0 * math.pi - s_val / RADIUS_M)
        assert a <= 0.35 + 12.0 / RADIUS_M
    assert f"s={err.stations[0][0]:.1f}" in str(err)


def test_bad_params_raise_typed_error() -> None:
    st = _circle_stations()
    src = _annulus_source()
    with pytest.raises(wt.WidthTraceError):
        wt.trace_widths(
            st, src, params=wt.TraceParams(search_min_m=5.0, search_max_m=4.0)
        )


# --- cross-checks ----------------------------------------------------------------------------


def test_crosscheck_disagreement_flags_but_never_mutates() -> None:
    st = _circle_stations()
    src = _annulus_source(6.0, 6.0)
    s_ref = np.asarray(st.s_m)
    w_ref = np.full(len(st), 12.0)
    w_ref[30] = 15.0  # LiDAR disagrees by 3 m at station 30 (band is 1 m)
    res = wt.trace_widths(st, src, params=PARAMS, lidar_width_m=(s_ref, w_ref))
    base = wt.trace_widths(st, src, params=PARAMS)
    np.testing.assert_allclose(res.width_left_m, base.width_left_m)
    np.testing.assert_allclose(res.width_right_m, base.width_right_m)
    flags = res.provenance.flags
    assert any(
        f.source == "lidar" and abs(f.s_m - float(st.s_m[30])) < 1e-9 for f in flags
    )
    assert all(abs(f.s_m - float(st.s_m[30])) < 1e-9 for f in flags)
    # An agreeing telemetry corridor raises no flags.
    agree = wt.trace_widths(
        st, src, params=PARAMS, telemetry_width_m=(s_ref, np.full(len(st), 12.0))
    )
    assert not agree.provenance.flags


# --- provenance ------------------------------------------------------------------------------


def test_provenance_record() -> None:
    st = _circle_stations()
    src = _annulus_source(6.0, 6.0)
    # Offset 6.5 keeps the CP-adjusted total (12.5 m) inside the 1 m cross-check band, so the
    # ONLY flag is the deliberate LiDAR bump at station 30.
    cp = wt.ControlPoint(s_m=float(st.s_m[8]), side="left", offset_m=6.5)
    s_ref = np.asarray(st.s_m)
    w_ref = np.full(len(st), 12.0)
    w_ref[30] = 15.0
    res = wt.trace_widths(
        st, src, params=PARAMS, control_points=[cp], lidar_width_m=(s_ref, w_ref)
    )
    prov = res.provenance
    assert prov.left.source == "orthophoto"
    assert prov.right.source == "orthophoto"
    assert "control_points" in prov.left.method
    assert "control_points" not in prov.right.method
    assert prov.left.control_point_count == 1
    assert prov.right.control_point_count == 0
    assert prov.flags and prov.flags[0].source == "lidar"
    meta = prov.as_meta()
    yaml.safe_dump(meta)  # track.yaml-embeddable: plain scalars/lists/dicts only
    left = cast("dict[str, object]", meta["left"])
    assert left["control_points"] == 1
    flagged = cast("list[float]", meta["flagged_stations_m"])
    assert flagged == [pytest.approx(float(st.s_m[30]), abs=0.05)]


# --- QA overlay renderer ---------------------------------------------------------------------


def _load_qa_tool() -> ModuleType:
    path = Path(__file__).resolve().parents[1] / "tools" / "plot_track_width_qa.py"
    spec = importlib.util.spec_from_file_location("plot_track_width_qa", path)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_qa_overlay_renders_png(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    out = tmp_path / "qa.png"
    tool = _load_qa_tool()
    assert tool.main(["--synthetic", "--out", str(out)]) == 0
    assert out.exists() and out.stat().st_size > 10_000


def test_out_of_range_control_point_stays_in_its_window() -> None:
    """An s_m past the lap length must not saturate the blend across a whole sector.

    Folding the wrap distance without reducing modulo the lap first yields a negative
    distance, which drives the smoothstep to full strength everywhere.
    """
    length = 400.0
    s = np.arange(0.0, length, 4.0)
    detected = np.full_like(s, 5.0)
    cp = wt.ControlPoint(s_m=length + 120.0, side="left", offset_m=9.0)
    blended, weight = wt._apply_control_points(  # pyright: ignore[reportPrivateUsage]
        s, detected, [cp], 25.0, length
    )
    # s_m wraps to 120 m, so only stations within the blend window may move. Measure that
    # with a proper circular distance — reduce modulo the lap, THEN fold.
    d = np.mod(np.abs(s - 120.0), length)
    d = np.minimum(d, length - d)
    far = d > 25.0
    assert np.allclose(blended[far], 5.0), "a stray control point leaked across the lap"
    assert float(weight.max()) <= 1.0 + 1e-12
