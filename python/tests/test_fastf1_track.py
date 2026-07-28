# SPDX-License-Identifier: AGPL-3.0-only
"""Offline tests for the FastF1 driven-line importer (`outlap.importers.fastf1_track`).

FastF1 is a network dependency CI never installs, so every scenario here drives the importer
through a **synthetic session**: a stand-in ``fastf1`` module (injected into ``sys.modules``)
serving laps of a closed circuit with two analytically-known radii, mapped into a local
"FastF1 frame" by a *known* similarity transform, expressed in FastF1's 1/10 m units, dosed
with seeded Gaussian noise and started at a sub-sample-jittered lap phase (real laps are
sliced at the timing line, so their starts do not coincide). Ground truth is therefore exact
and the estimator's error is measured, not assumed:

* the georeference recovers the known similarity transform (machine precision from exact
  anchors; within tolerance from jittered hand-picked anchors);
* the emitted geometry recovers the tightest true radius within 2%;
* ``Source == 'interpolated'`` rows — deliberately displaced by 500 m so leakage is
  unmissable — never reach the fit, and the surviving count is asserted;
* the residual gate rejects a deliberately misregistered anchor set and writes nothing;
* atomic writes: a crash between temp write and rename leaves the target untouched, and
  ``--force`` is required over an existing track dir;
* re-running from the same inputs reproduces ``centerline.csv`` byte-identically (R7);
* pooling eight laps measurably beats one (the line error is compared against truth).

The only test that touches the real package is ``importorskip``-guarded and never fetches.
"""

from __future__ import annotations

import math
import sys
from datetime import timedelta
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np
import pytest
import yaml
from numpy.typing import NDArray

from outlap.importers import osm_track
from outlap.importers.fastf1_track import (
    ANCHORS_FORMAT,
    IMPORTER_VERSION,
    MANIFEST_FILE,
    MAX_ANCHOR_RESIDUAL_M,
    METRICS_FILE,
    MIN_ANCHORS,
    Anchor,
    AnchorFormatError,
    GeoreferenceResidualError,
    SessionKey,
    TooFewAnchorsError,
    fit_similarity,
    load_anchors,
    run_import,
)
from outlap.importers.lidar_dem import EnuFrame
from outlap.trackcal import GeorefTransform, load_metrics

F = NDArray[np.float64]

# --- the synthetic world ---------------------------------------------------------------------

#: Truth geometry: the convex hull of two circles — a 120 m sweeper and a 34 m hairpin joined
#: by their external tangents. Two distinct known radii plus real straights, so the tightest
#: apex is a genuine target and not the degenerate "everything is one circle" case.
R_BIG = 120.0
R_SMALL = 34.0
GAP_M = 300.0

#: The ENU frame the emitted track speaks (anchors carry lat/lon; this pins the origin).
LAT0, LON0 = 41.57, 2.2611
FRAME = EnuFrame(lat0_deg=LAT0, lon0_deg=LON0)

#: The similarity transform the importer must recover: FastF1's local frame is unscaled and
#: arbitrarily rotated, so a 1.2% scale error and a 0.7 rad rotation are the realistic ask.
TRUTH = GeorefTransform(
    scale=1.012, rotation_rad=0.7, tx_m=1234.5, ty_m=-987.6, residual_rms_m=0.0
)

#: FastF1 position noise (metres, per axis) and sample spacing (~10 Hz at racing speed).
NOISE_M = 0.3
SAMPLE_SPACING_M = 4.0


def _circuit_geometry() -> tuple[float, float, float, float]:
    """``(alpha, tangent_len, arc_small, total)`` of the two-circle hull (analytic)."""
    delta = R_BIG - R_SMALL
    alpha = math.acos(delta / GAP_M)
    tangent = math.sqrt(GAP_M**2 - delta**2)
    arc_small = 2.0 * alpha * R_SMALL
    arc_big = (2.0 * math.pi - 2.0 * alpha) * R_BIG
    return alpha, tangent, arc_small, arc_small + arc_big + 2.0 * tangent


def truth_length_m() -> float:
    """Exact arc length of the synthetic circuit."""
    return _circuit_geometry()[3]


def truth_point(s_m: float) -> tuple[float, float]:
    """Exact world-frame position at arc length ``s_m`` (CCW from the hairpin's lower tangent).

    Pieces in order: the hairpin arc (R_SMALL, centred at ``(GAP_M, 0)``), the upper tangent,
    the sweeper arc (R_BIG, centred at the origin), the lower tangent.
    """
    alpha, tangent, arc_small, total = _circuit_geometry()
    arc_big = (2.0 * math.pi - 2.0 * alpha) * R_BIG
    s = s_m % total
    if s < arc_small:
        th = -alpha + s / R_SMALL
        return GAP_M + R_SMALL * math.cos(th), R_SMALL * math.sin(th)
    s -= arc_small
    if s < tangent:
        t = s / tangent
        x0, y0 = GAP_M + R_SMALL * math.cos(alpha), R_SMALL * math.sin(alpha)
        x1, y1 = R_BIG * math.cos(alpha), R_BIG * math.sin(alpha)
        return x0 + t * (x1 - x0), y0 + t * (y1 - y0)
    s -= tangent
    if s < arc_big:
        th = alpha + s / R_BIG
        return R_BIG * math.cos(th), R_BIG * math.sin(th)
    s -= arc_big
    t = s / tangent
    x0, y0 = R_BIG * math.cos(alpha), -R_BIG * math.sin(alpha)
    x1, y1 = GAP_M + R_SMALL * math.cos(alpha), -R_SMALL * math.sin(alpha)
    return x0 + t * (x1 - x0), y0 + t * (y1 - y0)


def truth_polyline(step_m: float = 0.25) -> F:
    """Dense ``(n, 2)`` sampling of the exact geometry (the line-error reference)."""
    s = np.arange(0.0, truth_length_m(), step_m)
    return np.array([truth_point(float(v)) for v in s], dtype=np.float64)


def to_local(x_w: F, y_w: F, transform: GeorefTransform) -> tuple[F, F]:
    """World ENU metres → the local FastF1 frame (the exact inverse of ``transform.apply``)."""
    c, s = math.cos(-transform.rotation_rad), math.sin(-transform.rotation_rad)
    dx = (np.asarray(x_w) - transform.tx_m) / transform.scale
    dy = (np.asarray(y_w) - transform.ty_m) / transform.scale
    return dx * c - dy * s, dx * s + dy * c


# --- the stand-in fastf1 module ---------------------------------------------------------------


class _FakePos:
    """A minimal stand-in for a FastF1 position frame: column access + boolean masking."""

    def __init__(self, columns: dict[str, Any]) -> None:
        self._columns = columns

    def __getitem__(self, key: Any) -> Any:
        if isinstance(key, str):
            return self._columns[key]
        return _FakePos({k: v[key] for k, v in self._columns.items()})

    def __len__(self) -> int:
        return int(len(self._columns["X"]))


class _FakeLap:
    def __init__(self, driver: str, lap_number: int, pos: _FakePos) -> None:
        self._fields: dict[str, Any] = {"Driver": driver, "LapNumber": lap_number}
        self._pos = pos

    def __getitem__(self, key: str) -> Any:
        return self._fields[key]

    def get_pos_data(self) -> _FakePos:
        return self._pos


class _FakeLaps:
    def __init__(self, laps: list[_FakeLap]) -> None:
        self._laps = laps

    def pick_drivers(self, drivers: list[str]) -> _FakeLaps:
        return _FakeLaps([lap for lap in self._laps if lap["Driver"] in drivers])

    def pick_quicklaps(self) -> _FakeLaps:
        return self

    def iterlaps(self) -> Any:
        return enumerate(self._laps)


class _FakeSession:
    def __init__(self, laps: list[_FakeLap]) -> None:
        self.laps = _FakeLaps(laps)

    def load(self, **_kwargs: Any) -> None:
        return None


class _FakeCache:
    def __init__(self) -> None:
        self.enabled: list[str] = []

    def enable_cache(self, path: str) -> None:
        self.enabled.append(path)


class _FakeFastF1(ModuleType):
    """A stand-in ``fastf1`` module — the synthetic session, no network, no package."""

    def __init__(self, laps: list[_FakeLap]) -> None:
        super().__init__("fastf1")
        self._laps = laps
        self.Cache = _FakeCache()

    def get_session(self, _year: int, _event: object, _session: str) -> _FakeSession:
        return _FakeSession(self._laps)


def _lap_pos(
    driver: str,
    lap_number: int,
    *,
    seed: int,
    phase_m: float,
    interpolate_every: int = 7,
) -> _FakeLap:
    """One lap of position samples in FastF1 units (1/10 m) with seeded noise.

    Every ``interpolate_every``-th row is flagged ``Source == 'interpolated'`` and displaced
    by 500 m: if the filter ever leaked, no fit could survive it.
    """
    total = truth_length_m()
    s = np.arange(phase_m, phase_m + total, SAMPLE_SPACING_M)
    world = np.array([truth_point(float(v)) for v in s], dtype=np.float64)
    lx, ly = to_local(world[:, 0], world[:, 1], TRUTH)
    rng = np.random.default_rng(seed)
    lx = lx + rng.normal(0.0, NOISE_M, lx.size)
    ly = ly + rng.normal(0.0, NOISE_M, ly.size)
    source = np.array(["car"] * lx.size, dtype="<U16")
    fake = np.arange(lx.size) % interpolate_every == 0
    source[fake] = "interpolated"
    lx = np.where(fake, lx + 500.0, lx)
    ly = np.where(fake, ly - 500.0, ly)
    return _FakeLap(
        driver,
        lap_number,
        _FakePos(
            {
                "X": lx * 10.0,
                "Y": ly * 10.0,
                "Source": source,
                "Time": np.array(
                    [timedelta(seconds=0.1 * i) for i in range(lx.size)], dtype=object
                ),
            }
        ),
    )


def _install_session(
    monkeypatch: pytest.MonkeyPatch,
    *,
    n_laps: int = 8,
    driver: str = "VER",
    interpolate_every: int = 7,
) -> _FakeFastF1:
    """Install the stand-in ``fastf1`` module with ``n_laps`` sub-sample-phased laps."""
    rng = np.random.default_rng(999)
    laps = [
        _lap_pos(
            driver,
            i + 1,
            seed=100 + i,
            phase_m=float(rng.uniform(0.0, SAMPLE_SPACING_M)),
            interpolate_every=interpolate_every,
        )
        for i in range(n_laps)
    ]
    module = _FakeFastF1(laps)
    monkeypatch.setitem(sys.modules, "fastf1", module)
    return module


SESSION = SessionKey(year=2026, event="Synthetic Grand Prix", session="R")


# --- anchor fixtures --------------------------------------------------------------------------

#: Arc lengths of the hand-picked registration anchors (spread around the lap).
ANCHOR_S = (0.0, 200.0, 500.0, 800.0, 1000.0)


def _anchor_rows(
    *, jitter_m: float = 0.0, seed: int = 7, swap: bool = False, n: int | None = None
) -> list[tuple[str, float, float, float, float]]:
    picks = list(ANCHOR_S if n is None else ANCHOR_S[:n])
    world = np.array([truth_point(s) for s in picks], dtype=np.float64)
    lx, ly = to_local(world[:, 0], world[:, 1], TRUTH)
    if jitter_m > 0.0:
        rng = np.random.default_rng(seed)
        lx = lx + rng.normal(0.0, jitter_m, lx.size)
        ly = ly + rng.normal(0.0, jitter_m, ly.size)
    order = list(range(len(picks)))
    if swap:  # deliberate misregistration: two far-apart anchors traded
        order[0], order[2] = order[2], order[0]
    lat, lon = FRAME.to_latlon(world[order, 0], world[order, 1])
    return [
        (f"a{i}", float(lx[i]), float(ly[i]), float(lat[i]), float(lon[i]))
        for i in range(len(picks))
    ]


def _write_anchors(path: Path, **kwargs: Any) -> Path:
    rows = _anchor_rows(**kwargs)
    lines = [f"# {ANCHORS_FORMAT}", "label,local_x_m,local_y_m,ref_lat_deg,ref_lon_deg"]
    lines += [
        f"{label},{lx!r},{ly!r},{lat!r},{lon!r}" for label, lx, ly, lat, lon in rows
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _import(track_dir: Path, **kwargs: Any) -> Any:
    anchors = kwargs.pop("anchors_path", None) or _write_anchors(
        track_dir / "georef_anchors.csv"
    )
    return run_import(
        track_dir,
        session=SESSION,
        anchors_path=anchors,
        name="Synthetic (driven line)",
        enu_origin=(LAT0, LON0),
        **kwargs,
    )


def _emitted_xy(track_dir: Path) -> F:
    rows = np.genfromtxt(
        track_dir / "centerline.csv", delimiter=",", names=True, skip_header=1
    )
    return np.stack([rows["x_m"], rows["y_m"]], axis=1)


def _line_error_m(track_dir: Path) -> float:
    """RMS distance from the emitted stations to the exact geometry (metres)."""
    pts = _emitted_xy(track_dir)
    truth = truth_polyline()
    d = np.empty(pts.shape[0], dtype=np.float64)
    for i in range(pts.shape[0]):
        d[i] = float(np.min(np.hypot(truth[:, 0] - pts[i, 0], truth[:, 1] - pts[i, 1])))
    return float(np.sqrt(np.mean(d**2)))


# --- georeference: the load-bearing stage -----------------------------------------------------


def test_similarity_fit_recovers_an_exact_transform() -> None:
    """Umeyama on noise-free correspondences: machine precision, residual ~0."""
    rows = _anchor_rows()
    anchors = [Anchor(*row) for row in rows]
    local = np.array([[a.local_x_m, a.local_y_m] for a in anchors])
    ref = np.stack(
        FRAME.to_enu(
            np.array([a.ref_lat_deg for a in anchors]),
            np.array([a.ref_lon_deg for a in anchors]),
        ),
        axis=1,
    )
    transform, residuals = fit_similarity(local, ref)
    assert transform.scale == pytest.approx(TRUTH.scale, rel=1e-9)
    assert transform.rotation_rad == pytest.approx(TRUTH.rotation_rad, abs=1e-9)
    assert transform.tx_m == pytest.approx(TRUTH.tx_m, abs=1e-6)
    assert transform.ty_m == pytest.approx(TRUTH.ty_m, abs=1e-6)
    assert float(np.max(residuals)) < 1e-6


def test_synthetic_session_recovers_transform_and_tightest_radius(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The headline: known transform + known radii recovered from a noisy pooled session."""
    _install_session(monkeypatch)
    track_dir = tmp_path / "synthetic"
    anchors = _write_anchors(track_dir / "georef_anchors.csv", jitter_m=0.3)
    result = _import(track_dir, anchors_path=anchors)

    transform = result.georeference.transform
    assert transform.scale == pytest.approx(TRUTH.scale, rel=5e-3)
    assert transform.rotation_rad == pytest.approx(TRUTH.rotation_rad, abs=5e-3)
    assert transform.residual_rms_m < 1.0
    assert result.georeference.residual_rms_m < MAX_ANCHOR_RESIDUAL_M

    assert result.length_m == pytest.approx(truth_length_m(), rel=5e-3)
    assert result.tightest_radius_m == pytest.approx(R_SMALL, rel=2e-2)
    assert _line_error_m(track_dir) < 0.5
    # Ringing guard: the circuit has exactly two corners. Fitting the raw pooled cloud
    # instead of the binned mean line invented eleven more (see `pool_positions`).
    assert result.n_corners == 2


def test_misregistered_anchors_raise_and_write_nothing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A swapped correspondence pair must fail the residual ceiling before any write."""
    _install_session(monkeypatch, n_laps=2)
    track_dir = tmp_path / "synthetic"
    anchors = _write_anchors(track_dir / "georef_anchors.csv", swap=True)
    with pytest.raises(GeoreferenceResidualError, match="residual"):
        _import(track_dir, anchors_path=anchors)
    assert sorted(p.name for p in track_dir.iterdir()) == ["georef_anchors.csv"]


def test_two_anchors_cannot_certify_a_similarity_fit(tmp_path: Path) -> None:
    """A 2-point similarity fit is exactly determined — residual 0 is vacuous, so it errors."""
    track_dir = tmp_path / "synthetic"
    anchors = _write_anchors(track_dir / "georef_anchors.csv", n=MIN_ANCHORS - 1)
    with pytest.raises(TooFewAnchorsError, match=str(MIN_ANCHORS)):
        _import(track_dir, anchors_path=anchors)
    assert not (track_dir / "track.yaml").exists()


def test_anchor_file_without_the_format_tag_is_a_typed_error(tmp_path: Path) -> None:
    path = tmp_path / "anchors.csv"
    path.write_text(
        "label,local_x_m,local_y_m,ref_lat_deg,ref_lon_deg\na,1,2,41.5,2.2\n",
        encoding="utf-8",
    )
    with pytest.raises(AnchorFormatError, match=ANCHORS_FORMAT):
        load_anchors(path)


def test_anchor_file_with_bad_columns_is_a_typed_error(tmp_path: Path) -> None:
    path = tmp_path / "anchors.csv"
    path.write_text(f"# {ANCHORS_FORMAT}\nlabel,x,y\na,1,2\n", encoding="utf-8")
    with pytest.raises(AnchorFormatError, match="columns"):
        load_anchors(path)


# --- pooling: the interpolated filter and multi-lap gain ---------------------------------------


def test_interpolated_rows_never_reach_the_fit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Assert the exact count that survived the ``Source`` filter (and that it fits)."""
    n_laps, every = 4, 7
    _install_session(monkeypatch, n_laps=n_laps, interpolate_every=every)
    per_lap = int(np.arange(0.0, truth_length_m(), SAMPLE_SPACING_M).size)
    dropped_per_lap = len(range(0, per_lap, every))
    result = _import(tmp_path / "synthetic")
    assert result.n_samples_raw == n_laps * per_lap
    assert result.n_dropped_interpolated == n_laps * dropped_per_lap
    assert result.n_samples == n_laps * (per_lap - dropped_per_lap)
    # The displaced rows would blow the line error to hundreds of metres if they leaked.
    assert _line_error_m(tmp_path / "synthetic") < 0.6


def test_multi_lap_pooling_reduces_the_line_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Eight pooled laps must measurably beat one — the reason the importer pools at all.

    Measured on this fixture: 0.163 m RMS from one lap, 0.112 m from eight (-31%), and the
    tightest apex radius moves from -4.8% to -1.6% of truth over the same range.
    """
    _install_session(monkeypatch, n_laps=1)
    one = tmp_path / "one"
    result_one = _import(one)
    err_one = _line_error_m(one)

    _install_session(monkeypatch, n_laps=8)
    many = tmp_path / "many"
    result_many = _import(many)
    err_many = _line_error_m(many)

    assert err_many < 0.8 * err_one, (
        f"pooling 8 laps gave {err_many:.3f} m vs {err_one:.3f} m for one lap"
    )
    assert abs(result_many.tightest_radius_m - R_SMALL) < abs(
        result_one.tightest_radius_m - R_SMALL
    )


# --- the emitted artifact: honesty, manifest, reproducibility ----------------------------------


def test_driven_line_meta_is_honest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The artifact must say what it is: a narrow driven line, not a corridor."""
    _install_session(monkeypatch, n_laps=3)
    track_dir = tmp_path / "synthetic"
    result = _import(track_dir, half_width_m=0.8)
    assert result.accuracy_class == "C"

    doc = yaml.safe_load((track_dir / "track.yaml").read_text(encoding="utf-8"))
    assert doc["schema"] == "track/1.1"
    assert doc["closed"] is True
    meta = doc["meta"]
    assert meta["source"] == "fastf1-position"
    assert meta["width_source"] == "driven-line"
    assert meta["accuracy_class"] == "C"
    assert meta["importer_version"] == IMPORTER_VERSION
    assert str(SESSION) in meta["attribution"]
    assert "driven line" in meta["notes"].lower()
    assert "corridor" in meta["notes"].lower()
    assert "residual_rms_m=" in meta["georef_transform"]

    rows = np.genfromtxt(
        track_dir / "centerline.csv", delimiter=",", names=True, skip_header=1
    )
    assert np.allclose(rows["width_left_m"], 0.8)
    assert np.allclose(rows["width_right_m"], 0.8)
    assert np.allclose(
        rows["z_m"], 0.0
    )  # position telemetry carries no usable elevation
    assert np.allclose(rows["banking_deg"], 0.0)


def test_manifest_pins_the_session_transform_and_counts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """KTD7: session key, laps/drivers, before/after counts, transform + residual, version."""
    _install_session(monkeypatch, n_laps=3)
    track_dir = tmp_path / "synthetic"
    result = _import(track_dir)
    manifest = yaml.safe_load((track_dir / MANIFEST_FILE).read_text(encoding="utf-8"))

    assert manifest["importer"] == "outlap.importers.fastf1_track"
    assert manifest["importer_version"] == IMPORTER_VERSION
    session = manifest["session"]
    assert session["year"] == SESSION.year
    assert session["event"] == SESSION.event
    assert session["session"] == SESSION.session
    assert session["key"] == str(SESSION)
    assert session["drivers"] == ["VER"]
    assert len(session["laps"]) == 3

    positions = manifest["inputs"]["positions"]
    assert positions["samples_raw"] == result.n_samples_raw
    assert positions["samples_used"] == result.n_samples
    assert positions["samples_dropped_interpolated"] == result.n_dropped_interpolated
    assert positions["committed"] is False  # §15: no raw telemetry redistribution

    anchors = manifest["inputs"]["anchors"]
    assert anchors["file"] == "georef_anchors.csv"
    assert anchors["count"] == len(ANCHOR_S)

    georef = manifest["georeference"]
    assert georef["method"] == "umeyama-similarity-2d"
    assert georef["max_residual_m"] == MAX_ANCHOR_RESIDUAL_M
    assert georef["residual_rms_m"] < MAX_ANCHOR_RESIDUAL_M
    assert georef["scale"] == pytest.approx(TRUTH.scale, rel=5e-3)
    assert manifest["parameters"]["noise_std_m"] == pytest.approx(NOISE_M)
    assert manifest["outputs"]["centerline_csv_sha256"]


def test_rerun_from_the_same_inputs_is_byte_identical(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """R7: the import is a pure function of its (pinned) inputs."""
    _install_session(monkeypatch, n_laps=4)
    track_dir = tmp_path / "synthetic"
    _import(track_dir)
    first = (track_dir / "centerline.csv").read_bytes()
    _install_session(monkeypatch, n_laps=4)
    _import(track_dir, force=True)
    assert (track_dir / "centerline.csv").read_bytes() == first


def test_reference_metrics_csv_carries_the_transform_and_radii(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """R4: the audit artifact ships per-corner circle-fit radii in the committed format."""
    _install_session(monkeypatch, n_laps=4)
    track_dir = tmp_path / "synthetic"
    result = _import(track_dir)
    metrics = load_metrics(track_dir / METRICS_FILE)
    assert metrics.source_session == str(SESSION)
    assert metrics.transform == result.georeference.transform
    assert metrics.n_corners >= 2
    assert float(np.min(metrics.radius_m)) == pytest.approx(R_SMALL, rel=2e-2)
    # Apex speeds stay recorded-not-derived here (KTD4 gives them to the U8 gate).
    assert bool(np.all(np.isnan(metrics.apex_speed_mps)))


def test_interrupted_build_leaves_the_target_untouched(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _install_session(monkeypatch, n_laps=2)
    track_dir = tmp_path / "synthetic"
    anchors = _write_anchors(track_dir / "georef_anchors.csv")

    def boom(src: str, dst: str) -> None:
        raise OSError("simulated crash between temp write and rename")

    monkeypatch.setattr(osm_track.os, "replace", boom)
    with pytest.raises(OSError, match="simulated crash"):
        _import(track_dir, anchors_path=anchors)
    assert sorted(p.name for p in track_dir.iterdir()) == ["georef_anchors.csv"]
    assert not [p for p in tmp_path.iterdir() if p.name.startswith(".synthetic.build-")]


def test_force_required_over_an_existing_track_dir(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _install_session(monkeypatch, n_laps=2)
    track_dir = tmp_path / "synthetic"
    _import(track_dir)
    with pytest.raises(osm_track.OutputExistsError, match="--force"):
        _import(track_dir)
    _import(track_dir, force=True)


# --- the live path stays opt-in ----------------------------------------------------------------


def test_live_fastf1_path_is_opt_in() -> None:
    """CI never installs fastf1; the live loader is only importable where it exists."""
    pytest.importorskip("fastf1")
    from outlap.trackcal import load_fastf1_positions

    assert callable(load_fastf1_positions)
