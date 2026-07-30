# SPDX-License-Identifier: AGPL-3.0-only
"""LiDAR DTM fusion tests: synthetic rasters and analytic samplers only, never live tiles.

Banking scenarios measure against analytic truth: a tilted plane with a known 1.0 deg banking,
a symmetric crown (which must NOT alias into banking — the odd/even decomposition rejects it
exactly), and a noise-dominated section that must fall back to 0 with the provenance flag set.
The rasterio/pyproj-dependent parts are importorskip-gated so the suite passes wherever the
base package installs; module import itself must succeed without either dependency.
"""

from __future__ import annotations

import builtins
import importlib
import math
import sys
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import pytest
from numpy.typing import NDArray

from outlap.importers.lidar_dem import (
    PRESETS,
    BankingProfile,
    DemSource,
    ElevationProfile,
    EnuFrame,
    LidarDemError,
    MissingTileError,
    SurfaceModelError,
    estimate_banking,
    fuse_elevation,
    make_enu_sampler,
    open_dtm,
    resolve_tiles,
)

if TYPE_CHECKING:
    from collections.abc import Callable

F = NDArray[np.float64]

_BANK_1DEG_SLOPE = math.tan(math.radians(1.0))


# --- helpers --------------------------------------------------------------------------------


def _straight_centerline(n: int = 21, spacing: float = 5.0) -> tuple[F, F]:
    """A straight track along +y at x = 0 (ENU): left of travel is -x."""
    y = np.arange(n, dtype=np.float64) * spacing
    x = np.zeros_like(y)
    return x, y


def _tilted_plane(x: F, y: F) -> F:
    """z rises toward -x (the left of a +y-heading track): +1.0 deg banking."""
    return -_BANK_1DEG_SLOPE * x


def _crown(x: F, y: F) -> F:
    """A symmetric 2% crown about the centerline: even in the lateral offset."""
    return -0.02 * np.abs(x)


def _write_plane_tif(
    path: Path,
    *,
    origin_e: float,
    origin_n: float,
    res: float,
    width: int,
    height: int,
    plane: Callable[[F, F], F],
    crs: str = "EPSG:25831",
    tags: dict[str, str] | None = None,
    nodata: float | None = None,
) -> None:
    """Write a small synthetic GeoTIFF whose pixel values follow ``plane(E, N)``."""
    rasterio = pytest.importorskip("rasterio")
    from rasterio.transform import from_origin

    cols = np.arange(width, dtype=np.float64)
    rows = np.arange(height, dtype=np.float64)
    ee = origin_e + (cols + 0.5) * res
    nn = origin_n - (rows + 0.5) * res
    grid_e, grid_n = np.meshgrid(ee, nn)
    data = plane(grid_e, grid_n).astype(np.float32)
    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        height=height,
        width=width,
        count=1,
        dtype="float32",
        crs=crs,
        transform=from_origin(origin_e, origin_n, res, res),
        nodata=nodata,
    ) as dst:
        dst.write(data, 1)
        if tags:
            dst.update_tags(**tags)


# --- module import without optional deps ----------------------------------------------------


def test_module_imports_without_optional_deps(monkeypatch: pytest.MonkeyPatch) -> None:
    """The module must import cleanly when rasterio/pyproj are absent (lazy imports)."""
    real_import = builtins.__import__

    def forbidding_import(name: str, *args: Any, **kwargs: Any) -> Any:
        if name.split(".")[0] in {"rasterio", "pyproj"}:
            raise ImportError(f"forced absence of {name}")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", forbidding_import)
    monkeypatch.delitem(sys.modules, "outlap.importers.lidar_dem", raising=False)
    module = importlib.import_module("outlap.importers.lidar_dem")

    frame = module.EnuFrame(lat0_deg=41.57, lon0_deg=2.26)
    with pytest.raises(ImportError, match="track-import"):
        module.crs_to_enu(
            np.array([438800.0]), np.array([4602000.0]), crs="EPSG:25831", frame=frame
        )
    with pytest.raises(ImportError, match="track-import"):
        module.open_dtm([Path("missing.tif")])


# --- presets --------------------------------------------------------------------------------


def test_presets_cover_the_reference_trio_as_dtms() -> None:
    assert set(PRESETS) == {"icgc_catalunya", "uk_ea_1m", "wallonia_mnt"}
    for source in PRESETS.values():
        assert source.product == "dtm"
        assert source.crs.startswith("EPSG:")
        assert source.noise_floor_m > 0.0
    assert PRESETS["icgc_catalunya"].crs == "EPSG:25831"
    assert PRESETS["icgc_catalunya"].license == "CC-BY 4.0"
    assert PRESETS["uk_ea_1m"].crs == "EPSG:27700"
    assert "OGL" in PRESETS["uk_ea_1m"].license
    assert PRESETS["wallonia_mnt"].crs == "EPSG:3812"
    assert PRESETS["wallonia_mnt"].license == "CC-BY 4.0"


# --- tile resolution (fetch seam) -----------------------------------------------------------


def test_missing_tile_error_names_the_tile(tmp_path: Path) -> None:
    source = PRESETS["icgc_catalunya"]
    (tmp_path / source.filename_template.format(tile_id="tile_a")).touch()
    with pytest.raises(MissingTileError, match="tile_b"):
        resolve_tiles(source, ["tile_a", "tile_b"], tmp_path)


def test_injected_fetcher_fills_the_cache_and_manifest(tmp_path: Path) -> None:
    source = PRESETS["wallonia_mnt"]
    fetched: list[str] = []

    def fetcher(src: DemSource, tile_id: str, dest: Path) -> None:
        fetched.append(tile_id)
        dest.touch()

    resolved = resolve_tiles(source, ["t1", "t2"], tmp_path, fetcher=fetcher)
    assert fetched == ["t1", "t2"]
    assert all(p.exists() for p in resolved.paths)

    manifest = resolved.manifest()
    assert manifest["source"] == source.source_id
    assert manifest["dataset"] == source.dataset
    assert manifest["version"] == source.version
    assert manifest["license"] == source.license
    assert manifest["tiles"] == ["t1", "t2"]


def test_fetcher_that_leaves_a_gap_is_still_an_error(tmp_path: Path) -> None:
    source = PRESETS["uk_ea_1m"]

    def broken_fetcher(src: DemSource, tile_id: str, dest: Path) -> None:
        return None  # never writes the file

    with pytest.raises(MissingTileError, match="t9"):
        resolve_tiles(source, ["t9"], tmp_path, fetcher=broken_fetcher)


# --- banking from cross-track sections (analytic samplers, no raster deps) ------------------


def test_tilted_plane_banking_recovered_within_005_deg() -> None:
    x, y = _straight_centerline()
    profile = estimate_banking(x, y, _tilted_plane, closed=False, half_width_m=5.0)
    assert isinstance(profile, BankingProfile)
    assert profile.banking_deg.shape == x.shape
    assert np.all(profile.resolved)
    assert float(np.max(np.abs(profile.banking_deg - 1.0))) < 0.05


def test_symmetric_crown_does_not_alias_into_banking() -> None:
    x, y = _straight_centerline()
    profile = estimate_banking(x, y, _crown, closed=False, half_width_m=5.0)
    assert float(np.max(np.abs(profile.banking_deg))) < 0.02


def test_crown_plus_tilt_recovers_the_tilt_only() -> None:
    x, y = _straight_centerline()

    def crowned_and_banked(px: F, py: F) -> F:
        return _tilted_plane(px, py) + _crown(px, py)

    profile = estimate_banking(x, y, crowned_and_banked, closed=False, half_width_m=5.0)
    assert np.all(profile.resolved)
    assert float(np.max(np.abs(profile.banking_deg - 1.0))) < 0.05


def test_low_snr_sections_fall_back_to_zero_with_provenance() -> None:
    x, y = _straight_centerline(n=25)
    rng = np.random.default_rng(42)
    faint_slope = math.tan(math.radians(0.02))  # ~1.7 mm across the section

    def noisy_faint(px: F, py: F) -> F:
        return -faint_slope * px + rng.normal(0.0, 0.1, px.shape)

    profile = estimate_banking(
        x, y, noisy_faint, closed=False, half_width_m=5.0, noise_floor_m=0.06
    )
    fallback = ~profile.resolved
    assert float(np.mean(fallback)) >= 0.8  # noise >= signal: sections don't resolve
    # No data, NOT a measurement of zero — a zero here would claim these sections were
    # measured flat, which is what track/1.2 exists to stop conflating.
    assert np.all(np.isnan(profile.banking_deg[fallback]))
    assert np.all(np.isfinite(profile.banking_deg[profile.resolved]))
    # The lossy pre-1.2 column collapses them back to zero, deliberately and on request.
    assert np.all(profile.as_column(nodata=False)[fallback] == 0.0)

    provenance = profile.provenance()
    assert provenance["sections"] == 25
    assert provenance["fallback_sections"] == int(np.sum(fallback))
    assert provenance["resolved_sections"] == int(np.sum(profile.resolved))


def test_banking_provenance_carries_the_source() -> None:
    x, y = _straight_centerline()
    source = PRESETS["icgc_catalunya"]
    profile = estimate_banking(
        x, y, _tilted_plane, closed=False, half_width_m=5.0, source=source
    )
    provenance = profile.provenance()
    assert provenance["dataset"] == source.dataset
    assert provenance["license"] == source.license


def test_banking_sign_positive_raises_the_left_edge() -> None:
    """outlap-track convention: banking > 0 raises the left edge (z up, ISO 8855)."""
    x, y = _straight_centerline()

    def right_edge_higher(px: F, py: F) -> F:
        return _BANK_1DEG_SLOPE * px  # z rises toward +x = to the RIGHT of travel

    profile = estimate_banking(x, y, right_edge_higher, closed=False, half_width_m=5.0)
    assert float(np.max(profile.banking_deg)) < -0.9  # negative: left edge is lower


# --- C2 elevation fusion --------------------------------------------------------------------


def test_fuse_elevation_recovers_a_smooth_closed_profile() -> None:
    """The fused profile must beat the raw samples, and report itself without a tautology.

    ``discrepancy_rms_m`` is the residual the λ search matched, so it equals the declaration
    exactly — that is the identity worth asserting. ``residual_rms_m`` is the *reported*
    curve's residual and drops below the declaration once the twicing step runs (0.1000 →
    0.0833 here), so it is no longer the same number wearing two hats. The accuracy bar
    tightened with it: this fixture's error against truth was 0.0568 m uncorrected and is
    0.0261 m now, against 0.0882 m of raw sample noise.
    """
    length = 1000.0
    n = 200
    s = np.arange(n, dtype=np.float64) * (length / n)
    z_true = 5.0 * np.sin(2.0 * np.pi * s / length) + 2.0 * np.cos(
        4.0 * np.pi * s / length
    )
    rng = np.random.default_rng(7)
    z_noisy = z_true + rng.normal(0.0, 0.1, n)

    profile = fuse_elevation(s, z_noisy, closed=True, length_m=length, noise_std_m=0.1)
    assert profile.z_m.shape == s.shape
    rms_error = float(np.sqrt(np.mean((profile.z_m - z_true) ** 2)))
    assert rms_error < 0.035  # the fit beats the raw noise (0.088) by averaging
    assert profile.discrepancy_rms_m == pytest.approx(0.1, rel=1e-6)
    assert not profile.lambda_capped
    assert profile.bias_corrected
    assert profile.residual_rms_m < profile.discrepancy_rms_m
    assert 0.0 < profile.effective_dof <= float(n)


def test_fused_elevation_is_c2_across_the_seam() -> None:
    length = 800.0
    n = 160
    s = np.arange(n, dtype=np.float64) * (length / n)
    z = 10.0 * np.sin(2.0 * np.pi * s / length)
    profile = fuse_elevation(s, z, closed=True, length_m=length, noise_std_m=0.05)

    eps = 1e-4
    for order, tol in ((0, 1e-3), (1, 1e-3), (2, 1e-3)):
        before = float(profile.evaluate(np.array([length - eps]), order=order)[0])
        after = float(profile.evaluate(np.array([eps]), order=order)[0])
        assert abs(before - after) < tol, f"order {order} jumps across the seam"


def test_fuse_elevation_open_profile_and_degenerate_input() -> None:
    s = np.linspace(0.0, 500.0, 100)
    z = 0.02 * s + 3.0 * np.sin(2.0 * np.pi * s / 250.0)
    profile = fuse_elevation(s, z, closed=False, noise_std_m=0.0)
    assert float(np.max(np.abs(profile.z_m - z))) < 0.05
    # Exact data (noise_std_m = 0) is a hard skip: λ is pinned at the floor, nothing to undo.
    assert profile.bias_corrected is False
    assert profile.discrepancy_rms_m == profile.residual_rms_m

    with pytest.raises(LidarDemError):
        fuse_elevation(s[:5], z[:5], closed=False)
    bad = z.copy()
    bad[3] = np.nan
    with pytest.raises(LidarDemError):
        fuse_elevation(s, bad, closed=False)


# The two densities ``osm_track`` actually fuses at. The DEM chain samples the pinned
# opentopodata elevations every ``_DEM_STEP_M`` = 20 m and fuses at ``_DEM_KNOT_SPACING_M``
# = 40 m knots with a 1.0 m declaration; the LiDAR stage fuses the DTM sampled at every
# fitted station (``ds_m`` = 3 m) with ``_LIDAR_KNOT_SPACING_M`` = 20 m knots and the
# source's published noise floor. The station-to-knot redundancy differs by 3×, and the
# twicing veto is sensitive to exactly that — hence both are exercised here.
_DEM_STEP_M = 20.0
_DEM_KNOT_M = 40.0
_LIDAR_STEP_M = 3.0
_LIDAR_KNOT_M = 20.0

_LAP_M = 4600.0
_UNDULATION_WAVELENGTH_M = 460.0
_UNDULATION_AMPLITUDE_M = 3.0


def _undulation(step_m: float, *, noise_m: float = 0.0, seed: int = 0) -> tuple[F, F]:
    """Stations and samples for a 3 m / 460 m elevation undulation over a 4.6 km lap."""
    n = int(round(_LAP_M / step_m))
    s = np.arange(n, dtype=np.float64) * (_LAP_M / n)
    z = _UNDULATION_AMPLITUDE_M * np.sin(2.0 * np.pi * s / _UNDULATION_WAVELENGTH_M)
    if noise_m > 0.0:
        z = z + np.random.default_rng(seed).normal(0.0, noise_m, n)
    return s, z


def _amplitude_error_pct(s: F, z_fit: F) -> float:
    """Fitted undulation amplitude vs truth, in percent (exact projection on the tone)."""
    basis = np.sin(2.0 * np.pi * s / _UNDULATION_WAVELENGTH_M)
    fitted = 2.0 * float(
        np.mean(z_fit * basis)
    )  # whole periods ⇒ the projection is exact
    return 100.0 * (fitted / _UNDULATION_AMPLITUDE_M - 1.0)


def _fuse_dem_density(s: F, z: F, *, bias_correction: bool) -> ElevationProfile:
    """Fuse at the DEM chain's declared noise, station step and knot spacing."""
    return fuse_elevation(
        s,
        z,
        closed=True,
        length_m=_LAP_M,
        noise_std_m=1.0,
        knot_spacing_m=_DEM_KNOT_M,
        bias_correction=bias_correction,
    )


def test_undulation_amplitude_survives_the_discrepancy_shrink() -> None:
    """T9: the 1-D half of the ``√2 σ`` shrink, at the densities the importer really fuses at.

    Constraining the *residual* to the declared noise shrinks a smooth undulation's amplitude,
    and here that error lands in grade and vertical curvature. On exact samples at the DEM
    chain's density and its 1.0 m declaration the uncorrected fit reads the 3 m / 460 m
    undulation **−47.1%** small; the shared twicing step brings it to −22.2%, so the bar is
    25%. The correction is a strict improvement, never a coin flip: it either runs (and pays)
    or vetoes into exactly the uncorrected curve.
    """
    s, z = _undulation(_DEM_STEP_M)
    off = _fuse_dem_density(s, z, bias_correction=False)
    on = _fuse_dem_density(s, z, bias_correction=True)
    assert off.bias_corrected is False
    assert (
        on.bias_corrected is True
    )  # exact-but-declared data always has signal to recover
    assert _amplitude_error_pct(s, off.z_m) < -40.0  # the defect this guard exists for
    assert abs(_amplitude_error_pct(s, on.z_m)) < 25.0


def test_undulation_amplitude_within_10pct_when_noise_is_declared_honestly() -> None:
    """T9 (cont.): true noise = declaration is the regime the correction is tuned for.

    At the DEM chain's density (20 m stations against 40 m knots — barely 2:1 redundancy) the
    twicing veto legitimately fires on a minority of noise realisations: with λ that low the
    residual is mostly noise, which is exactly what the veto is for. So ``bias_corrected`` is
    asserted, not assumed — when the step runs the amplitude lands inside 10% (measured mean
    −3.8%, worst 9.2% over 20 seeds), and when it vetoes the profile is bit-identical to the
    uncorrected fit. Measured 13/20 seeds corrected; the step is never worse than skipping it.
    """
    corrected = 0
    for seed in range(8):
        s, z = _undulation(_DEM_STEP_M, noise_m=1.0, seed=seed)
        off = _fuse_dem_density(s, z, bias_correction=False)
        on = _fuse_dem_density(s, z, bias_correction=True)
        error_off = _amplitude_error_pct(s, off.z_m)
        error_on = _amplitude_error_pct(s, on.z_m)
        if on.bias_corrected:
            corrected += 1
            assert abs(error_on) < 10.0, f"seed {seed}: {error_on:+.2f}%"
            assert abs(error_on) < abs(error_off)  # running the step always pays
        else:
            assert np.array_equal(on.z_m, off.z_m)  # a veto is a full skip
    assert corrected >= 4, f"the step vetoed on {8 - corrected}/8 seeds"

    # The LiDAR stage fuses 3 m stations against 20 m knots: well-conditioned, so the
    # correction runs on nearly every realisation and lands inside 1%.
    for seed in range(4):
        s, z = _undulation(_LIDAR_STEP_M, noise_m=0.15, seed=seed)
        profile = fuse_elevation(
            s,
            z,
            closed=True,
            length_m=_LAP_M,
            noise_std_m=0.15,
            knot_spacing_m=_LIDAR_KNOT_M,
        )
        if profile.bias_corrected:
            assert abs(_amplitude_error_pct(s, profile.z_m)) < 1.0


# --- CRS round-trip (pyproj) ----------------------------------------------------------------


def test_crs_round_trip_within_1_cm() -> None:
    pyproj = pytest.importorskip("pyproj")
    from outlap.importers.lidar_dem import crs_to_enu, enu_to_crs

    easting = np.array([438800.0, 439150.0])
    northing = np.array([4602000.0, 4602420.0])
    to_wgs = pyproj.Transformer.from_crs("EPSG:25831", "EPSG:4326", always_xy=True)
    lon0, lat0 = to_wgs.transform(438900.0, 4602100.0)
    frame = EnuFrame(lat0_deg=float(lat0) + 0.001, lon0_deg=float(lon0) + 0.001)

    x, y = crs_to_enu(easting, northing, crs="EPSG:25831", frame=frame)
    back_e, back_n = enu_to_crs(x, y, crs="EPSG:25831", frame=frame)
    assert float(np.max(np.abs(back_e - easting))) < 0.01
    assert float(np.max(np.abs(back_n - northing))) < 0.01


# --- DTM rasters (rasterio) -----------------------------------------------------------------


def test_dsm_tagged_raster_raises_a_typed_error(tmp_path: Path) -> None:
    pytest.importorskip("rasterio")
    tif = tmp_path / "surface.tif"
    _write_plane_tif(
        tif,
        origin_e=438000.0,
        origin_n=4602000.0,
        res=1.0,
        width=8,
        height=8,
        plane=lambda e, n: np.zeros_like(e),
        tags={"PRODUCT": "DSM"},
    )
    with pytest.raises(SurfaceModelError):
        open_dtm([tif])


def test_dsm_flagged_source_raises_before_reading(tmp_path: Path) -> None:
    pytest.importorskip("rasterio")
    import dataclasses

    tif = tmp_path / "ok.tif"
    _write_plane_tif(
        tif,
        origin_e=438000.0,
        origin_n=4602000.0,
        res=1.0,
        width=8,
        height=8,
        plane=lambda e, n: np.zeros_like(e),
    )
    dsm_source = dataclasses.replace(PRESETS["icgc_catalunya"], product="dsm")
    with pytest.raises(SurfaceModelError):
        open_dtm([tif], source=dsm_source)


def test_mosaic_sampling_is_bilinear_and_bounded(tmp_path: Path) -> None:
    pytest.importorskip("rasterio")

    def plane(e: F, n: F) -> F:
        return 0.01 * (e - 438000.0) + 0.002 * (n - 4602000.0) + 100.0

    left = tmp_path / "left.tif"
    right = tmp_path / "right.tif"
    _write_plane_tif(
        left,
        origin_e=438000.0,
        origin_n=4602040.0,
        res=1.0,
        width=40,
        height=40,
        plane=plane,
    )
    _write_plane_tif(
        right,
        origin_e=438040.0,
        origin_n=4602040.0,
        res=1.0,
        width=40,
        height=40,
        plane=plane,
    )
    mosaic = open_dtm([left, right])
    e = np.array([438010.3, 438050.7, 438039.2])  # spans both tiles
    n = np.array([4602020.6, 4602010.1, 4602030.9])
    z = mosaic.sample(e, n)
    assert float(np.max(np.abs(z - plane(e, n)))) < 1e-3

    with pytest.raises(LidarDemError, match="outside"):
        mosaic.sample(np.array([437000.0]), np.array([4602020.0]))


def test_nodata_holes_are_an_error_not_a_zero(tmp_path: Path) -> None:
    rasterio = pytest.importorskip("rasterio")
    tif = tmp_path / "holey.tif"
    _write_plane_tif(
        tif,
        origin_e=438000.0,
        origin_n=4602020.0,
        res=1.0,
        width=20,
        height=20,
        plane=lambda e, n: np.full_like(e, 50.0),
        nodata=-9999.0,
    )
    with rasterio.open(tif, "r+") as dst:
        band = dst.read(1)
        band[8:12, 8:12] = -9999.0
        dst.write(band, 1)

    mosaic = open_dtm([tif])
    ok = mosaic.sample(np.array([438002.0]), np.array([4602018.0]))
    assert ok[0] == pytest.approx(50.0)
    with pytest.raises(LidarDemError, match="no-data"):
        mosaic.sample(np.array([438010.0]), np.array([4602010.0]))


def test_banking_end_to_end_from_a_tilted_plane_raster(tmp_path: Path) -> None:
    pyproj = pytest.importorskip("pyproj")
    pytest.importorskip("rasterio")

    center_e, center_n = 438900.0, 4602100.0
    to_wgs = pyproj.Transformer.from_crs("EPSG:25831", "EPSG:4326", always_xy=True)
    lon0, lat0 = to_wgs.transform(center_e, center_n)
    frame = EnuFrame(lat0_deg=float(lat0), lon0_deg=float(lon0))

    tif = tmp_path / "banked.tif"
    _write_plane_tif(
        tif,
        origin_e=center_e - 80.0,
        origin_n=center_n + 80.0,
        res=1.0,
        width=160,
        height=160,
        plane=lambda e, n: -_BANK_1DEG_SLOPE * (e - center_e),
    )
    mosaic = open_dtm([tif], source=PRESETS["icgc_catalunya"])
    sampler = make_enu_sampler(mosaic, crs="EPSG:25831", frame=frame)

    y = np.linspace(-50.0, 50.0, 21)
    x = np.zeros_like(y)
    profile = estimate_banking(x, y, sampler, closed=False, half_width_m=5.0)
    assert np.all(profile.resolved)
    assert float(np.max(np.abs(profile.banking_deg - 1.0))) < 0.05
