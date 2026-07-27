# SPDX-License-Identifier: AGPL-3.0-only
"""National LiDAR DTM fusion: C² elevation and cross-section banking for track imports.

Coarse public DEMs (25-30 m posting, metre-level vertical noise) cannot resolve banking, so
the OSM importer writes ``banking_deg = 0`` everywhere and interpolates elevation through a
web service. National LiDAR DTMs can: the reference-trio sources ship ~0.5-1 m bare-earth
grids with a 6-15 cm noise floor. This module is the reusable fusion layer the import
pipeline composes (HANDOFF §12 MT, unit U3):

* **Presets** (:data:`PRESETS`): ICGC Territorial LiDAR DTM for Catalunya (CC-BY 4.0,
  EPSG:25831 — preferred over PNOA, whose commercial-use terms are ambiguous), the UK
  Environment Agency National LiDAR Programme 1 m DTM (OGL v3, EPSG:27700), and the Service
  public de Wallonie MNT 2021-2022 (CC-BY 4.0, EPSG:3812 Belgian Lambert 2008). Tiles are
  fetched by tile ID into a local cache directory that is **never committed**; tile IDs +
  dataset versions surface via :meth:`ResolvedTiles.manifest` for the track's input manifest
  (KTD7). The fetch seam is injectable (:func:`resolve_tiles` takes a fetcher callable) and
  defaults to cache-only, so tests never touch the network; the real endpoints are pinned in
  a later data session.
* **CRS handling**: pyproj transforms between each source CRS and WGS84; the track's local
  ENU frame (:class:`EnuFrame`) is the same equirectangular projection the OSM importer uses,
  so LiDAR samples land in exactly the frame ``centerline.csv`` speaks.
* **DTM guard**: fusion needs bare-earth ground. A raster whose metadata flags it as a
  surface model (DSM), or a source declared ``product != "dtm"``, raises
  :class:`SurfaceModelError` — cars, catch fencing, and grandstand roofs are not banking.
* **Elevation** (:func:`fuse_elevation`): DTM samples along the centerline are fused with the
  *same* penalised P-spline machinery as ``outlap.trackcal.geometry`` (one shared smoother —
  cyclic basis for closed tracks, so z, z' and z'' are continuous across the ``s = 0`` seam),
  replacing the service-interpolation + ``UnivariateSpline`` chain for preset tracks.
* **Banking** (:func:`estimate_banking`): per-row cross-track sections perpendicular to the
  centerline, multiple symmetric lateral samples per section, averaged over a small
  longitudinal window for SNR. Detrending is an **odd/even decomposition**: with symmetric
  offsets ``±t`` the crown/camber profile is an *even* function of the lateral offset and
  banking is the *odd* part, so ``z_odd(t) = (z(+t) − z(−t)) / 2`` cancels a symmetric crown
  exactly and the through-origin slope of ``z_odd`` is the banking gradient. Sections whose
  slope does not clear the noise (SNR below ``snr_min``, with the residual scatter floored at
  the source noise floor) fall back to banking 0 with the provenance flag set — honest zero,
  never fake banking. Sign convention matches ``outlap-track``: **banking > 0 raises the left
  edge** (ISO 8855: x forward, y left, z up).

Heavy dependencies (``rasterio`` — BSD-3-Clause, ``pyproj`` — MIT) live in the
``track-import`` extra and are imported lazily inside functions, so this module imports
cleanly without them. Like the other importers this is offline tooling: it never runs in CI
(synthetic raster fixtures cover the contract) and reads only open, redistributable data.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from numpy.typing import NDArray

# ONE shared smoother (CLAUDE.md interpolation rule, plan KTD2): elevation fusion reuses the
# trackcal penalised P-spline machinery on a 1-D z(s) target instead of adding a second one.
from outlap.trackcal.geometry import (
    _LAMBDA_FLOOR,  # pyright: ignore[reportPrivateUsage]
    _design_matrix,  # pyright: ignore[reportPrivateUsage]
    _eval_spline,  # pyright: ignore[reportPrivateUsage]
    _match_noise,  # pyright: ignore[reportPrivateUsage]
    _second_difference,  # pyright: ignore[reportPrivateUsage]
)

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Sequence

    #: Injectable tile fetcher: ``fetcher(source, tile_id, destination_path)`` must leave the
    #: tile at ``destination_path``. Tests inject local-file writers; the data session brings
    #: the real per-source downloaders.
    TileFetcher = Callable[["DemSource", str, Path], None]

F = NDArray[np.float64]

#: Minimum number of stations for an elevation fusion (mirrors trackcal's fit guard).
MIN_PROFILE_POINTS = 10

# The OSM importer's equirectangular ENU sphere radius — kept identical so both speak the
# same local frame.
_EARTH_R = 6_371_000.0

# Residual-scatter floor so exact synthetic sections do not divide by zero in the SNR gate.
_SIGMA_FLOOR = 1e-9

# Metadata markers that identify a surface model (DSM) rather than bare-earth terrain.
_DSM_MARKERS = re.compile(
    r"\bdsm\b|\bsurface\s+model\b|\bdigital\s+surface\b", re.IGNORECASE
)

__all__ = [
    "MIN_PROFILE_POINTS",
    "PRESETS",
    "BankingProfile",
    "DemSource",
    "DtmMosaic",
    "ElevationProfile",
    "EnuFrame",
    "LidarDemError",
    "MissingTileError",
    "ResolvedTiles",
    "SurfaceModelError",
    "crs_to_enu",
    "enu_to_crs",
    "estimate_banking",
    "fuse_elevation",
    "make_enu_sampler",
    "open_dtm",
    "resolve_tiles",
]


class LidarDemError(ValueError):
    """Base error for LiDAR DTM fusion (typed, never a bare crash)."""


class MissingTileError(LidarDemError):
    """A requested DTM tile is not in the cache (the message names the tile IDs)."""


class SurfaceModelError(LidarDemError):
    """The raster is a surface model (DSM), not the bare-earth DTM fusion requires."""


# --- sources --------------------------------------------------------------------------------


@dataclass(frozen=True)
class DemSource:
    """A national LiDAR DTM source preset (dataset + version pin for the input manifest).

    ``noise_floor_m`` is the published per-sample vertical accuracy (RMSE-class) used to
    floor the banking SNR gate; ``product`` must be ``"dtm"`` (bare earth) — anything else
    is rejected by :func:`open_dtm`.
    """

    source_id: str
    name: str
    dataset: str
    version: str
    crs: str
    license: str
    grid_res_m: float
    noise_floor_m: float
    product: str = "dtm"
    filename_template: str = "{tile_id}.tif"


#: Reference-trio DTM presets. Versions are the datasets verified at planning time; the
#: data session re-verifies each license/version when the real tiles are pulled (R9).
PRESETS: dict[str, DemSource] = {
    "icgc_catalunya": DemSource(
        source_id="icgc_catalunya",
        name="ICGC Territorial LiDAR DTM (Catalunya)",
        dataset="icgc-lidar-territorial-dtm",
        version="v3.1",
        crs="EPSG:25831",  # ETRS89 / UTM 31N
        license="CC-BY 4.0",
        grid_res_m=1.0,
        noise_floor_m=0.06,  # ~6 cm vertical RMSE (ICGC v3.1 spec)
    ),
    "uk_ea_1m": DemSource(
        source_id="uk_ea_1m",
        name="Environment Agency National LiDAR Programme DTM (England)",
        dataset="ea-national-lidar-programme-dtm",
        version="1m-composite",
        crs="EPSG:27700",  # OSGB36 / British National Grid
        license="OGL v3",
        grid_res_m=1.0,
        noise_floor_m=0.15,  # EA quotes +/-15 cm vertical RMSE
    ),
    "wallonia_mnt": DemSource(
        source_id="wallonia_mnt",
        name="Service public de Wallonie MNT LiDAR 2021-2022",
        dataset="spw-mnt-lidar",
        version="2021-2022",
        crs="EPSG:3812",  # ETRS89 / Belgian Lambert 2008
        license="CC-BY 4.0",
        grid_res_m=0.5,
        noise_floor_m=0.10,
    ),
}


# --- local ENU frame + CRS handling ---------------------------------------------------------


@dataclass(frozen=True)
class EnuFrame:
    """The track's local ENU frame: the OSM importer's equirectangular projection.

    ``x`` is east, ``y`` is north (metres) about ``(lat0_deg, lon0_deg)``. The projection is
    an exact bijection with WGS84, so round trips are lossless; it matches
    ``osm_track._to_enu`` so LiDAR-derived channels align with the imported centerline.
    """

    lat0_deg: float
    lon0_deg: float

    def to_enu(self, lat_deg: F, lon_deg: F) -> tuple[F, F]:
        """WGS84 degrees → local ENU metres."""
        lat = np.asarray(lat_deg, dtype=np.float64)
        lon = np.asarray(lon_deg, dtype=np.float64)
        coslat = math.cos(math.radians(self.lat0_deg))
        x = np.radians(lon - self.lon0_deg) * _EARTH_R * coslat
        y = np.radians(lat - self.lat0_deg) * _EARTH_R
        return x, y

    def to_latlon(self, x_m: F, y_m: F) -> tuple[F, F]:
        """Local ENU metres → WGS84 degrees (the exact inverse of :meth:`to_enu`)."""
        x = np.asarray(x_m, dtype=np.float64)
        y = np.asarray(y_m, dtype=np.float64)
        coslat = math.cos(math.radians(self.lat0_deg))
        lon = self.lon0_deg + np.degrees(x / (_EARTH_R * coslat))
        lat = self.lat0_deg + np.degrees(y / _EARTH_R)
        return lat, lon


def crs_to_enu(
    easting_m: F, northing_m: F, *, crs: str, frame: EnuFrame
) -> tuple[F, F]:
    """Projected CRS coordinates (e.g. ``EPSG:25831``) → the track's local ENU frame."""
    pyproj = _import_pyproj()
    to_wgs = pyproj.Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    lon, lat = to_wgs.transform(
        np.asarray(easting_m, dtype=np.float64),
        np.asarray(northing_m, dtype=np.float64),
    )
    return frame.to_enu(
        np.asarray(lat, dtype=np.float64), np.asarray(lon, dtype=np.float64)
    )


def enu_to_crs(x_m: F, y_m: F, *, crs: str, frame: EnuFrame) -> tuple[F, F]:
    """The track's local ENU frame → projected CRS coordinates."""
    pyproj = _import_pyproj()
    lat, lon = frame.to_latlon(x_m, y_m)
    from_wgs = pyproj.Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    easting, northing = from_wgs.transform(lon, lat)
    return (
        np.asarray(easting, dtype=np.float64),
        np.asarray(northing, dtype=np.float64),
    )


# --- tile resolution (the fetch seam) -------------------------------------------------------


@dataclass(frozen=True)
class ResolvedTiles:
    """Locally resolved DTM tiles plus the provenance the caller writes into the manifest."""

    source: DemSource
    tile_ids: tuple[str, ...]
    paths: tuple[Path, ...]

    def manifest(self) -> dict[str, object]:
        """The input-manifest entry (KTD7): source, dataset version, license, tile IDs."""
        return {
            "source": self.source.source_id,
            "dataset": self.source.dataset,
            "version": self.source.version,
            "license": self.source.license,
            "crs": self.source.crs,
            "tiles": list(self.tile_ids),
        }


def resolve_tiles(
    source: DemSource,
    tile_ids: Iterable[str],
    cache_dir: Path | str,
    *,
    fetcher: TileFetcher | None = None,
) -> ResolvedTiles:
    """Resolve DTM tiles by ID against a local cache directory (never committed).

    Cache-only by default: a tile absent from ``cache_dir`` raises
    :class:`MissingTileError` naming every missing tile ID — no partial output. Passing a
    ``fetcher`` (the injectable seam) lets the caller fill gaps; the fetcher is invoked once
    per missing tile and the cache is re-checked afterwards.
    """
    cache = Path(cache_dir)
    cache.mkdir(parents=True, exist_ok=True)
    ids = tuple(tile_ids)
    if not ids:
        raise LidarDemError(f"no tile IDs requested for source {source.source_id}")
    paths = tuple(cache / source.filename_template.format(tile_id=t) for t in ids)
    if fetcher is not None:
        for tile_id, path in zip(ids, paths, strict=True):
            if not path.exists():
                fetcher(source, tile_id, path)
    missing = [t for t, p in zip(ids, paths, strict=True) if not p.exists()]
    if missing:
        raise MissingTileError(
            f"missing {source.source_id} DTM tile(s) {', '.join(missing)} in cache "
            f"{cache} — fetch them in a data session or inject a fetcher (tiles are "
            "cached locally and never committed)"
        )
    return ResolvedTiles(source=source, tile_ids=ids, paths=paths)


# --- DTM rasters ----------------------------------------------------------------------------


@dataclass(frozen=True)
class _TileData:
    """One loaded tile: bounds, the inverse geo-transform, and nodata-masked elevations."""

    path: Path
    bounds: tuple[float, float, float, float]  # left, bottom, right, top
    inv_transform: (
        Any  # affine.Affine (world → pixel); typed Any: rasterio ships no stubs
    )
    data: F  # (rows, cols) elevations, nodata as NaN


class DtmMosaic:
    """An in-memory mosaic of bare-earth DTM tiles, strict about coverage and no-data.

    Sampling is bilinear in each tile's projected CRS. Points outside every tile, or landing
    on a no-data hole, raise :class:`LidarDemError` — fusion must never silently invent
    ground where LiDAR has none.
    """

    def __init__(self, tiles: Sequence[_TileData]) -> None:
        if not tiles:
            raise LidarDemError("a DTM mosaic needs at least one tile")
        self._tiles = list(tiles)

    def sample(self, easting_m: F, northing_m: F) -> F:
        """Bilinear elevation (m) at projected-CRS coordinates."""
        e = np.atleast_1d(np.asarray(easting_m, dtype=np.float64))
        n = np.atleast_1d(np.asarray(northing_m, dtype=np.float64))
        if e.shape != n.shape:
            raise LidarDemError("easting and northing must have the same shape")
        out = np.full(e.shape, np.nan, dtype=np.float64)
        assigned = np.zeros(e.shape, dtype=bool)
        for tile in self._tiles:
            left, bottom, right, top = tile.bounds
            m = ~assigned & (e >= left) & (e <= right) & (n >= bottom) & (n <= top)
            if not bool(np.any(m)):
                continue
            cols, rows = tile.inv_transform * (e[m], n[m])
            out[m] = _bilinear(
                tile.data, np.asarray(rows) - 0.5, np.asarray(cols) - 0.5
            )
            assigned[m] = True
        if not bool(np.all(assigned)):
            i = int(np.argmax(~assigned))
            raise LidarDemError(
                f"{int(np.sum(~assigned))} sample point(s) fall outside every DTM tile "
                f"(first: E={e[i]:.1f}, N={n[i]:.1f})"
            )
        holes = ~np.isfinite(out)
        if bool(np.any(holes)):
            i = int(np.argmax(holes))
            raise LidarDemError(
                f"{int(np.sum(holes))} sample point(s) hit a no-data hole in the DTM "
                f"(first: E={e[i]:.1f}, N={n[i]:.1f})"
            )
        return out


def open_dtm(
    paths: Sequence[Path | str], *, source: DemSource | None = None
) -> DtmMosaic:
    """Open DTM tiles as a :class:`DtmMosaic`, asserting they are bare-earth products.

    Raises :class:`SurfaceModelError` when ``source`` declares a non-DTM product or a
    raster's metadata flags it as a surface model (DSM); :class:`MissingTileError` for
    absent files. Tiles are read fully into memory (offline importer-scale tooling).
    """
    rasterio = _import_rasterio()
    if source is not None and source.product.lower() != "dtm":
        raise SurfaceModelError(
            f"source {source.source_id} declares product {source.product!r}: elevation and "
            "banking fusion need a bare-earth DTM, not a surface model"
        )
    tiles: list[_TileData] = []
    for p in paths:
        path = Path(p)
        if not path.exists():
            raise MissingTileError(f"DTM tile file not found: {path}")
        with rasterio.open(path) as ds:
            _assert_bare_earth(ds, path)
            data = np.asarray(ds.read(1), dtype=np.float64)
            nodata = ds.nodata
            if nodata is not None:
                data[data == float(nodata)] = np.nan
            b = ds.bounds
            tiles.append(
                _TileData(
                    path=path,
                    bounds=(
                        float(b.left),
                        float(b.bottom),
                        float(b.right),
                        float(b.top),
                    ),
                    inv_transform=~ds.transform,
                    data=data,
                )
            )
    return DtmMosaic(tiles)


def make_enu_sampler(
    mosaic: DtmMosaic, *, crs: str, frame: EnuFrame
) -> Callable[[F, F], F]:
    """An elevation sampler in the track's ENU frame, backed by a projected-CRS mosaic.

    The returned callable is the composition seam :func:`estimate_banking` and elevation
    sampling share — tests substitute plain analytic callables for it.
    """
    pyproj = _import_pyproj()
    from_wgs = pyproj.Transformer.from_crs("EPSG:4326", crs, always_xy=True)

    def sample(x_m: F, y_m: F) -> F:
        lat, lon = frame.to_latlon(x_m, y_m)
        easting, northing = from_wgs.transform(lon, lat)
        return mosaic.sample(
            np.asarray(easting, dtype=np.float64),
            np.asarray(northing, dtype=np.float64),
        )

    return sample


def _bilinear(data: F, frow: F, fcol: F) -> F:
    """Bilinear interpolation at fractional pixel-centre coordinates (NaN propagates)."""
    nrow, ncol = data.shape
    r0 = np.clip(np.floor(frow).astype(np.int64), 0, nrow - 1)
    c0 = np.clip(np.floor(fcol).astype(np.int64), 0, ncol - 1)
    r1 = np.minimum(r0 + 1, nrow - 1)
    c1 = np.minimum(c0 + 1, ncol - 1)
    tr = np.clip(frow - r0, 0.0, 1.0)
    tc = np.clip(fcol - c0, 0.0, 1.0)
    z00 = data[r0, c0]
    z01 = data[r0, c1]
    z10 = data[r1, c0]
    z11 = data[r1, c1]
    top = z00 * (1.0 - tc) + z01 * tc
    bot = z10 * (1.0 - tc) + z11 * tc
    return top * (1.0 - tr) + bot * tr


def _assert_bare_earth(ds: Any, path: Path) -> None:
    """Reject rasters whose metadata identifies a surface model (DSM)."""
    tags: dict[str, str] = dict(ds.tags())
    texts = [f"{k}={v}" for k, v in tags.items()]
    texts.extend(str(d) for d in (ds.descriptions or ()) if d)
    for text in texts:
        if _DSM_MARKERS.search(text):
            raise SurfaceModelError(
                f"raster {path} is flagged as a surface model ({text!r}); banking and "
                "elevation fusion require a bare-earth DTM"
            )


# --- C² elevation fusion --------------------------------------------------------------------


@dataclass(frozen=True)
class ElevationProfile:
    """A C²-consistent fused elevation profile z(s).

    ``z_m`` is the fused elevation at the input stations; :meth:`evaluate` gives z and its
    first two arc-length derivatives anywhere (closed profiles wrap modulo ``length_m`` and
    are C² across the seam — the basis is cyclic). ``residual_rms_m`` and
    ``smoothing_lambda`` record what the discrepancy search settled on, for honest reports.
    """

    closed: bool
    length_m: float
    s0_m: float
    z_m: F
    residual_rms_m: float
    smoothing_lambda: float
    cells: int
    coeff: F

    def evaluate(self, s_m: F, *, order: int = 0) -> F:
        """z (order 0), dz/ds (1) or d²z/ds² (2) at arc length ``s_m``."""
        s = np.asarray(s_m, dtype=np.float64)
        if self.closed:
            u = np.mod((s - self.s0_m) / self.length_m, 1.0)
        else:
            u = np.clip((s - self.s0_m) / self.length_m, 0.0, 1.0)
        val = _eval_spline(self.coeff, u, self.cells, self.closed, order)
        return val / self.length_m**order  # chain rule: u = (s - s0) / L is linear


def fuse_elevation(
    s_m: F,
    z_m: F,
    *,
    closed: bool,
    length_m: float | None = None,
    noise_std_m: float = 0.1,
    knot_spacing_m: float = 20.0,
    smoothing: float | None = None,
) -> ElevationProfile:
    """Fuse noisy DTM elevation samples into a C² profile z(s).

    The same penalised P-spline as the trackcal centerline fit, on a 1-D target: minimise
    ``‖z − B c‖² + λ ‖D₂ c‖²`` (Eilers & Marx 1996) with λ chosen by the Morozov discrepancy
    principle so the residual RMS matches ``noise_std_m`` (use the source's noise floor;
    ``0.0`` near-interpolates exact data). ``length_m`` is the closed lap length (station
    ``s[-1]`` is not the seam); ``smoothing`` overrides the discrepancy search with an
    explicit λ. Raises :class:`LidarDemError` on degenerate input.
    """
    s = np.asarray(s_m, dtype=np.float64)
    z = np.asarray(z_m, dtype=np.float64)
    if s.ndim != 1 or z.ndim != 1 or s.shape != z.shape:
        raise LidarDemError("s_m and z_m must be 1-D arrays of equal length")
    if s.size < MIN_PROFILE_POINTS:
        raise LidarDemError(
            f"elevation fusion needs >= {MIN_PROFILE_POINTS} stations, got {s.size}"
        )
    if not (np.all(np.isfinite(s)) and np.all(np.isfinite(z))):
        raise LidarDemError("elevation samples contain non-finite values")
    if np.any(np.diff(s) <= 0.0):
        raise LidarDemError("s_m must be strictly increasing")
    if noise_std_m < 0.0:
        raise LidarDemError("noise_std_m must be >= 0")
    if knot_spacing_m <= 0.0:
        raise LidarDemError("knot_spacing_m must be positive")

    n = int(s.size)
    s0 = float(s[0])
    span = float(s[-1] - s[0])
    if closed:
        length = float(length_m) if length_m is not None else span + span / (n - 1)
        if length <= span:
            raise LidarDemError(
                f"closed length_m ({length:.3f}) must exceed the station span ({span:.3f})"
            )
        cells = int(np.clip(round(length / knot_spacing_m), 8, 2 * n))
    else:
        if length_m is not None:
            raise LidarDemError("length_m applies to closed profiles only")
        length = span
        # Open fits must stay data-determined (ncoef = cells + 3 <= n) — see trackcal.
        cells = int(np.clip(round(length / knot_spacing_m), 4, max(n - 3, 4)))
        if cells + 3 > n:
            raise LidarDemError("too few stations for an open elevation fit")
    u = (s - s0) / length

    b_mat = _design_matrix(u, cells, closed, 0)
    btb = b_mat.T @ b_mat
    btz = b_mat.T @ z
    d_mat = _second_difference(cells, closed)

    def solve(lam: float, weights: F) -> tuple[F, float]:
        pen = d_mat.T @ (weights[:, None] * d_mat)
        scale = float(np.trace(btb)) / max(float(np.trace(pen)), 1e-300)
        coeff = np.linalg.solve(btb + (lam * scale) * pen, btz)
        resid = z - b_mat @ coeff
        return coeff, float(np.sqrt(np.mean(resid**2)))

    uniform = np.ones(d_mat.shape[0], dtype=np.float64)
    if smoothing is not None:
        lam = max(float(smoothing), _LAMBDA_FLOOR)
        coeff, rms = solve(lam, uniform)
    else:
        lam, coeff, rms = _match_noise(solve, uniform, noise_std_m)

    z_fused = _eval_spline(coeff, u, cells, closed, 0)
    return ElevationProfile(
        closed=closed,
        length_m=length,
        s0_m=s0,
        z_m=z_fused,
        residual_rms_m=rms,
        smoothing_lambda=lam,
        cells=cells,
        coeff=np.ascontiguousarray(coeff),
    )


# --- cross-section banking ------------------------------------------------------------------


@dataclass(frozen=True)
class BankingProfile:
    """Per-row banking with the per-section quality record.

    ``banking_deg[i]`` is the resolved banking at station ``i`` (**positive raises the left
    edge**, the ``outlap-track`` convention) or exactly ``0.0`` where the section did not
    clear the SNR gate (``resolved[i] = False`` — the honest fallback, flagged in
    :meth:`provenance`). ``snr`` is each section's slope-to-noise ratio.
    """

    banking_deg: F
    snr: F
    resolved: NDArray[np.bool_]
    snr_min: float
    source: DemSource | None = None

    def provenance(self) -> dict[str, object]:
        """The meta record the caller writes into ``track.yaml`` (KTD9)."""
        out: dict[str, object] = {
            "method": "lidar-cross-section",
            "detrend": "odd/even decomposition (symmetric offsets; crown is even)",
            "sections": int(self.banking_deg.size),
            "resolved_sections": int(np.sum(self.resolved)),
            "fallback_sections": int(np.sum(~self.resolved)),
            "snr_min": float(self.snr_min),
        }
        if self.source is not None:
            out["source"] = self.source.source_id
            out["dataset"] = self.source.dataset
            out["version"] = self.source.version
            out["license"] = self.source.license
        return out


def estimate_banking(
    x_m: F,
    y_m: F,
    sample_z: Callable[[F, F], F],
    *,
    closed: bool,
    half_width_m: float | F,
    edge_fraction: float = 0.9,
    n_offsets: int = 4,
    n_slices: int = 3,
    slice_spacing_m: float = 1.0,
    noise_floor_m: float = 0.0,
    snr_min: float = 2.0,
    source: DemSource | None = None,
) -> BankingProfile:
    """Estimate per-row banking from detrended cross-track LiDAR sections.

    At each station a section is taken perpendicular to the centerline: ``n_offsets``
    symmetric lateral pairs at ``±t`` (up to ``edge_fraction · half_width_m``, staying off
    kerbs and edges), repeated over ``n_slices`` longitudinal sub-slices ``slice_spacing_m``
    apart and pooled for SNR. Detrending is the odd/even decomposition (module docstring):
    ``z_odd = (z(+t) − z(−t)) / 2`` cancels any symmetric crown exactly; the through-origin
    slope ``b = Σ z_odd t / Σ t²`` is the banking gradient and ``banking = atan(b)``.

    The SNR gate is ``|b| / se(b) >= snr_min`` with the residual scatter floored at
    ``noise_floor_m / √2`` (odd-differencing of two independent samples halves the
    variance). Sections below the gate fall back to banking 0 with ``resolved = False``.

    ``sample_z`` is an ENU elevation sampler — :func:`make_enu_sampler` over a DTM mosaic,
    or any callable of the same shape.
    """
    x = np.asarray(x_m, dtype=np.float64)
    y = np.asarray(y_m, dtype=np.float64)
    if x.ndim != 1 or y.ndim != 1 or x.shape != y.shape:
        raise LidarDemError("x_m and y_m must be 1-D arrays of equal length")
    if x.size < 4:
        raise LidarDemError("banking estimation needs >= 4 centerline stations")
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        raise LidarDemError("centerline points contain non-finite coordinates")
    if not 0.0 < edge_fraction <= 1.0:
        raise LidarDemError("edge_fraction must be in (0, 1]")
    if n_offsets < 2 or n_slices < 1:
        raise LidarDemError("need n_offsets >= 2 and n_slices >= 1")
    if noise_floor_m < 0.0 or snr_min <= 0.0:
        raise LidarDemError("noise_floor_m must be >= 0 and snr_min > 0")
    hw = np.broadcast_to(np.asarray(half_width_m, dtype=np.float64), x.shape).astype(
        np.float64
    )
    if np.any(hw <= 0.0):
        raise LidarDemError("half_width_m must be positive")

    # Unit tangents (central differences; wrap when closed) and the ISO 8855 left normal.
    if closed:
        dx = np.roll(x, -1) - np.roll(x, 1)
        dy = np.roll(y, -1) - np.roll(y, 1)
    else:
        dx = np.gradient(x)
        dy = np.gradient(y)
    seg = np.hypot(dx, dy)
    if np.any(seg <= 1e-12):
        raise LidarDemError("centerline has coincident neighbouring stations")
    tx, ty = dx / seg, dy / seg
    nx, ny = -ty, tx  # left of travel (z up: +90 deg from the tangent)

    lat = (hw * edge_fraction)[:, None] * (
        (np.arange(n_offsets, dtype=np.float64) + 1.0) / n_offsets
    )[None, :]  # (n, K) lateral offsets, symmetric pairs at ±lat
    slide = (
        np.arange(n_slices, dtype=np.float64) - 0.5 * (n_slices - 1)
    ) * slice_spacing_m  # (S,) longitudinal sub-slice offsets, centred on the station

    base_x = x[:, None, None] + slide[None, :, None] * tx[:, None, None]  # (n, S, 1)
    base_y = y[:, None, None] + slide[None, :, None] * ty[:, None, None]
    off_x = lat[:, None, :] * nx[:, None, None]  # (n, 1→S, K)
    off_y = lat[:, None, :] * ny[:, None, None]
    shape = (x.size, n_slices, n_offsets)
    z_plus = _sample_batch(sample_z, base_x + off_x, base_y + off_y, shape)
    z_minus = _sample_batch(sample_z, base_x - off_x, base_y - off_y, shape)

    # Odd part: a symmetric crown (even in t) cancels exactly; banking is the odd slope.
    z_odd = 0.5 * (z_plus - z_minus)
    t = np.broadcast_to(lat[:, None, :], shape)
    sum_t2 = np.sum(t * t, axis=(1, 2))
    b = (
        np.sum(z_odd * t, axis=(1, 2)) / sum_t2
    )  # through-origin LSQ (odd ⇒ no intercept)
    resid = z_odd - b[:, None, None] * t
    dof = max(n_slices * n_offsets - 1, 1)
    sigma = np.sqrt(np.sum(resid * resid, axis=(1, 2)) / dof)
    sigma = np.maximum(sigma, max(noise_floor_m / math.sqrt(2.0), _SIGMA_FLOOR))
    snr = np.abs(b) * np.sqrt(sum_t2) / sigma  # |b| / se(b), se(b) = sigma / sqrt(Σt²)
    resolved = snr >= snr_min

    banking_deg = np.degrees(np.arctan(b))
    banking_deg[~resolved] = 0.0  # honest fallback, flagged in provenance
    return BankingProfile(
        banking_deg=banking_deg,
        snr=snr,
        resolved=resolved,
        snr_min=float(snr_min),
        source=source,
    )


def _sample_batch(
    sample_z: Callable[[F, F], F],
    px: F,
    py: F,
    shape: tuple[int, int, int],
) -> F:
    """One flattened sampler call for a (n, S, K) grid of section points."""
    px_full = np.broadcast_to(px, shape)
    py_full = np.broadcast_to(py, shape)
    z = np.asarray(
        sample_z(px_full.ravel().copy(), py_full.ravel().copy()), dtype=np.float64
    )
    if z.size != px_full.size:
        raise LidarDemError(
            "sample_z must return one elevation per query point "
            f"(got {z.size} for {px_full.size})"
        )
    return z.reshape(shape)


# --- lazy optional-dependency imports (wearcal/data.py pattern) -----------------------------


def _import_pyproj() -> Any:
    """Lazy pyproj import with an actionable error naming the extra."""
    try:
        import pyproj  # pyright: ignore[reportMissingImports]
    except ImportError as err:
        raise ImportError(
            "LiDAR DTM fusion needs pyproj — install the extra: "
            "`uv sync --extra track-import` (or `pip install 'outlap[track-import]'`)"
        ) from err
    return cast("Any", pyproj)


def _import_rasterio() -> Any:
    """Lazy rasterio import with an actionable error naming the extra."""
    try:
        import rasterio  # pyright: ignore[reportMissingImports]
    except ImportError as err:
        raise ImportError(
            "LiDAR DTM fusion needs rasterio — install the extra: "
            "`uv sync --extra track-import` (or `pip install 'outlap[track-import]'`)"
        ) from err
    return cast("Any", rasterio)
