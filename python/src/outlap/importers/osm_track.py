# SPDX-License-Identifier: AGPL-3.0-only
"""OSM snapshot → ``track.yaml`` + ``centerline.csv``: a staged, pinned, atomic pipeline (§9.3).

Since no open **3D** circuit data exists, the importer builds it (HANDOFF §12 MT, unit U5).
The import is a **pure function of committed inputs** (KTD7): the network is touched ONLY under
``--refresh-snapshot``, which writes the raw Overpass extract to ``<track_dir>/osm_snapshot.json``
(and, unless ``--no-dem``, the sampled DEM elevations to ``dem_samples.json``); every normal
import reads those committed files, so the same inputs reproduce ``centerline.csv``
byte-identically. A ``manifest.yaml`` in the track dir records the importer version, input
hashes, and per-stage provenance.

**Base import** (always runs): the fragmented ``highway=raceway`` ways are assembled into the
main closed lap (:func:`_assemble_circuit`: drop pit/kart/disused ways, prune spurs to the
2-core, resolve the pit-bypass theta junction to the two-longest-path cycle), projected to a
local ENU metric frame, and fitted with the :mod:`outlap.trackcal.geometry` penalised periodic
smoothing spline — curvature is evaluated analytically from the fit, never finite-differenced
from noisy nodes (KTD2/KTD3). Elevation, when DEM samples are committed, is fused C²-consistently
with the shared P-spline smoother (:func:`outlap.importers.lidar_dem.fuse_elevation`).

**Enrichment stages** (``--stages``, all optional — R8 honest degradation):

* ``widths`` — per-row edge-traced widths via :mod:`outlap.importers.width_trace` from a
  prepared orthophoto ``.npz`` (keys ``image`` + ``transform`` — the six rasterio-order affine
  coefficients) plus an optional hand-QA control-point CSV; the control-point file hash is
  pinned in the manifest and ``track.yaml`` meta.
* ``lidar`` — national LiDAR DTM z + cross-section banking via
  :mod:`outlap.importers.lidar_dem`, replacing the opentopodata chain; dataset + tile IDs are
  pinned in the manifest.
* ``telemetry-audit`` — not implemented yet; lands with the track-quality gate (U8).

**Widths never default silently** (R1): without the ``widths`` stage the import *errors* unless
``--half-width X`` declares an explicit constant half-width, which is recorded in meta as
``width_source: declared``.

**Accuracy class** derives from what ran (KTD10):

* ``C`` — base only (no resolved elevation; flat z),
* ``B`` — base + elevation (committed DEM samples or LiDAR z),
* ``A`` — base + ``widths`` + ``lidar`` (traced widths AND LiDAR z/banking).

Builds are **atomic** (KTD10): every emitted file is staged in a temp dir and moved into place
with ``os.replace``; an interrupted build leaves the target dir untouched, and overwriting any
existing file requires ``--force``.

This is network-facing tooling and is **never run in CI**. It reads only public data and never
touches proprietary sources (firewall, §1).

Example:
    # first import (fetches + commits the snapshot, declares degraded widths):
    python -m outlap.importers.osm_track --preset catalunya --out data/tracks/catalunya_osm \\
        --refresh-snapshot --half-width 6.0
    # reproducible re-import from the committed snapshot (no network):
    python -m outlap.importers.osm_track --preset catalunya --out data/tracks/catalunya_osm --force
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
from collections import defaultdict
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray

from outlap.importers.lidar_dem import (
    EARTH_RADIUS_M,
    EnuFrame,
    ResolvedTiles,
    estimate_banking,
    fuse_elevation,
    make_enu_sampler,
    open_dtm,
    resolve_tiles,
)
from outlap.importers.lidar_dem import (
    PRESETS as LIDAR_PRESETS,
)
from outlap.importers.width_trace import (
    AffineTransform,
    ArrayImageSource,
    ControlPoint,
    Stations,
    load_control_points,
    trace_widths,
)
from outlap.trackcal.geometry import fit_centerline

F = NDArray[np.float64]

#: Importer version recorded in every manifest + ``track.yaml`` meta (KTD7). Bump on any
#: change that can move the emitted geometry.
IMPORTER_VERSION = "2.0.0"

#: Committed input files (KTD7) and generated outputs in a track dir.
SNAPSHOT_FILE = "osm_snapshot.json"
DEM_SAMPLES_FILE = "dem_samples.json"
MANIFEST_FILE = "manifest.yaml"
OUTPUT_FILES = ("centerline.csv", "track.yaml", MANIFEST_FILE)

#: Optional enrichment stages (KTD10). Base import always runs.
STAGE_WIDTHS = "widths"
STAGE_LIDAR = "lidar"
STAGE_TELEMETRY_AUDIT = "telemetry-audit"
KNOWN_STAGES = (STAGE_WIDTHS, STAGE_LIDAR, STAGE_TELEMETRY_AUDIT)

# Known circuit presets (name, approximate center lat/lon, search radius m). Decision #23.
PRESETS: dict[str, tuple[str, float, float, int]] = {
    "catalunya": ("Circuit de Barcelona-Catalunya", 41.5700, 2.2611, 2200),
    "spa": ("Circuit de Spa-Francorchamps", 50.4372, 5.9714, 4000),
    "silverstone": ("Silverstone Circuit", 52.0733, -1.0147, 3000),
}

_OVERPASS_URLS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
)
# A descriptive User-Agent is required by OSM usage policy (bare requests get a 406/429).
_HEADERS = {
    "User-Agent": "outlap-track-importer/2.0 (+https://github.com/KMoula30/outlap)",
    "Accept": "application/json",
}
# opentopodata: free, no key; EU-DEM (25 m) covers the European circuits, SRTM is the global fallback.
_DEM_URL = "https://api.opentopodata.org/v1/{dataset}"
_DEM_DATASETS = ("eudem25m", "srtm30m")

# DEM sampling/fusion numerics (recorded in the manifest so a re-run is checkable).
_DEM_STEP_M = 20.0  # a 25-30 m DEM does not resolve 3 m spacing
_DEM_NOISE_STD_M = 1.0  # relative vertical noise the C² fusion smooths to
_DEM_KNOT_SPACING_M = 40.0
_LIDAR_KNOT_SPACING_M = 20.0
# Default per-axis OSM digitisation noise for the centerline fit (hand-traced ways).
_OSM_NOISE_STD_M = 1.0


class OsmTrackError(ValueError):
    """Base error for the OSM track importer (typed, never a bare crash)."""


class MissingSnapshotError(OsmTrackError):
    """No committed ``osm_snapshot.json`` in the track dir (run ``--refresh-snapshot``)."""


class MissingWidthSourceError(OsmTrackError):
    """R1: no width source resolves — widths are NEVER silently defaulted."""


class MissingElevationError(OsmTrackError):
    """Elevation requested but no committed DEM samples and no LiDAR stage."""


class OutputExistsError(OsmTrackError):
    """A file this import would write already exists and ``--force`` was not given."""


# --- OSM centerline -----------------------------------------------------------------------------


def _overpass(query: str) -> dict[str, Any]:
    import requests

    last: Exception | None = None
    for url in _OVERPASS_URLS:
        try:
            resp = requests.post(
                url, data={"data": query}, headers=_HEADERS, timeout=120
            )
            resp.raise_for_status()
            return resp.json()
        except Exception as exc:  # noqa: BLE001 - fall through to the next mirror
            print(
                f"  overpass {url} failed ({exc}); trying next mirror", file=sys.stderr
            )
            last = exc
    raise RuntimeError(f"all Overpass mirrors failed: {last}")


def fetch_raceway_ways(lat: float, lon: float, radius_m: int) -> dict[str, Any]:
    """Fetch ``highway=raceway`` ways (and their nodes) near ``(lat, lon)`` from Overpass."""
    query = (
        "[out:json][timeout:120];"
        f'(way["highway"="raceway"](around:{radius_m},{lat},{lon}););'
        "(._;>;);out body;"
    )
    return _overpass(query)


def _longest_way(osm: dict[str, Any]) -> list[int]:
    """Return the node-id sequence of the longest raceway way (a coarse single-way fallback)."""
    nodes = {e["id"]: e for e in osm["elements"] if e["type"] == "node"}
    ways = [
        e for e in osm["elements"] if e["type"] == "way" and len(e.get("nodes", [])) > 1
    ]
    if not ways:
        raise OsmTrackError("no raceway ways found near the given point")

    def way_length(w: dict[str, Any]) -> float:
        ns = [nodes[i] for i in w["nodes"] if i in nodes]
        return sum(
            _haversine(a["lat"], a["lon"], b["lat"], b["lon"])
            for a, b in zip(ns, ns[1:], strict=False)
        )

    return max(ways, key=way_length)["nodes"]


# Way names that are raceway but NOT part of the timed circuit lap (dropped before assembly).
_NON_CIRCUIT = ("pit", "kart", "support", "paddock", "service", "access")


def _polyline_len(node_ids: list[int], nodes: dict[int, Any]) -> float:
    pts = [nodes[i] for i in node_ids if i in nodes]
    return sum(
        _haversine(a["lat"], a["lon"], b["lat"], b["lon"])
        for a, b in zip(pts, pts[1:], strict=False)
    )


def _walk_cycle(adj: dict[int, set[int]]) -> list[int]:
    """Order a simple cycle (all nodes degree 2) into a node sequence."""
    start = next(iter(adj))
    loop = [start]
    prev, cur = None, start
    while True:
        nxts = [x for x in adj[cur] if x != prev]
        if not nxts:
            break
        nxt = nxts[0]
        if nxt == start:
            break
        loop.append(nxt)
        prev, cur = cur, nxt
        if len(loop) > len(adj) + 2:  # safety against a malformed graph
            break
    return loop


def _path_between(
    adj: dict[int, set[int]], start: int, first: int, end: int
) -> list[int] | None:
    """The degree-2 chain from ``start`` (leaving via ``first``) to the junction ``end``."""
    path = [start, first]
    prev, cur = start, first
    while cur != end:
        nxts = [x for x in adj[cur] if x != prev]
        if len(nxts) != 1:
            return None
        prev, cur = cur, nxts[0]
        path.append(cur)
    return path


def _assemble_circuit(osm: dict[str, Any]) -> list[int]:
    """Assemble the main **closed** circuit lap from the OSM ``highway=raceway`` ways.

    OSM splits a circuit into many corner-named ways plus non-circuit ways (pit lane, kart track).
    This drops the non-circuit ways by name — and any way tagged disused (``disused=yes`` or a
    ``disused:highway`` lifecycle tag: an abandoned layout must not enter the lap) — builds the
    node-segment graph, prunes dead-end spurs to the 2-core (all that is left is cycles), and
    returns the main loop: a simple cycle when the 2-core is one, or — for a *theta* 2-core (two
    degree-3 junctions joined by three paths, the classic pit-bypass chord) — the cycle formed by
    the **two longest** of the three paths (the short third path is the bypass/pit link). Falls
    back to the longest single way on any unexpected topology.
    """
    nodes = {e["id"]: e for e in osm["elements"] if e["type"] == "node"}
    ways = [
        e for e in osm["elements"] if e["type"] == "way" and len(e.get("nodes", [])) > 1
    ]

    def is_circuit(w: dict[str, Any]) -> bool:
        tags: dict[str, Any] = w.get("tags", {})
        if str(tags.get("disused", "")).lower() in ("yes", "true"):
            return False
        if any(k == "disused:highway" or k.startswith("disused:") for k in tags):
            return False
        name = str(tags.get("name", "")).lower()
        return not any(k in name for k in _NON_CIRCUIT)

    circ = [w for w in ways if is_circuit(w)]
    if not circ:
        return _longest_way(osm)

    adj: dict[int, set[int]] = defaultdict(set)
    for w in circ:
        ns = [i for i in w["nodes"] if i in nodes]
        for a, b in zip(ns, ns[1:], strict=False):
            adj[a].add(b)
            adj[b].add(a)

    # Prune dead-end spurs iteratively → the 2-core (cycles only). Degree-1 nodes are spur tips;
    # degree-0 nodes are the leftover of a fully-pruned tree branch.
    changed = True
    while changed:
        changed = False
        for n in [n for n, nb in list(adj.items()) if len(nb) < 2]:
            for other in adj[n]:
                adj[other].discard(n)
            del adj[n]
            changed = True

    if not adj:
        return _longest_way(osm)

    juncs = [n for n, nb in adj.items() if len(nb) > 2]
    if not juncs:
        loop = _walk_cycle(adj)
        return loop + [
            loop[0]
        ]  # close the ring so the closing edge enters the arc length

    if len(juncs) == 2 and all(len(adj[j]) == 3 for j in juncs):
        a, b = juncs
        seen: dict[tuple[int, ...], list[int]] = {}
        for first in adj[a]:
            p = _path_between(adj, a, first, b)
            if p is not None:
                seen[tuple(p)] = p
        paths = sorted(seen.values(), key=lambda p: _polyline_len(p, nodes))
        if len(paths) >= 2:
            long1, long2 = paths[-1], paths[-2]  # two longest = the racing loop
            # a → (long1 interior) → b → (long2 reversed interior) → back to a (ring closed)
            return long1[:-1] + long2[::-1][:-1] + [a]

    print(
        "  circuit graph has complex junctions; falling back to the longest single way",
        file=sys.stderr,
    )
    return _longest_way(osm)


def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def _to_enu(lats: Sequence[float], lons: Sequence[float]) -> tuple[F, F, EnuFrame]:
    """Equirectangular projection to a local ENU metric frame centred on the centroid.

    Only the origin is this function's own work; the projection itself is
    :meth:`outlap.importers.lidar_dem.EnuFrame.to_enu`, so the centerline cannot drift from the
    frame DEM/LiDAR/orthophoto stages sample in — and that frame (which implements the exact
    inverse) is returned alongside the metres.
    """
    # A closed loop repeats its first node so the closing edge enters the arc length. That
    # duplicate must not vote twice here: it drags the origin toward the first node by
    # R/(n+1) — 0.79 m on a 60 m ring — and since this frame is the georeference transform
    # every raster stage shares, the whole corridor would sit offset against its imagery.
    count = len(lats)
    if count > 1 and lats[0] == lats[-1] and lons[0] == lons[-1]:
        count -= 1
    frame = EnuFrame(
        lat0_deg=sum(lats[:count]) / count, lon0_deg=sum(lons[:count]) / count
    )
    x, y = frame.to_enu(
        np.asarray(lats, dtype=np.float64), np.asarray(lons, dtype=np.float64)
    )
    return x, y, frame


# --- snapshot pinning (KTD7) ---------------------------------------------------------------------


def _canonical_json(doc: Any) -> str:
    """Deterministic JSON serialization for committed inputs (sorted keys, stable layout)."""
    return json.dumps(doc, sort_keys=True, indent=1) + "\n"


def sha256_text(text: str) -> str:
    """SHA-256 (hex) of ``text`` as UTF-8 — the content pin for a generated file.

    Shared with the FastF1 driven-line importer: one hashing implementation between them, so a
    manifest hash means the same thing whichever importer wrote it.
    """
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    """SHA-256 (hex) of ``path``'s bytes — the content pin for a committed input (KTD7)."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_snapshot(track_dir: Path) -> dict[str, Any]:
    """Read the committed Overpass extract from ``<track_dir>/osm_snapshot.json``."""
    path = track_dir / SNAPSHOT_FILE
    if not path.exists():
        raise MissingSnapshotError(
            f"no committed OSM snapshot at {path} — run once with --refresh-snapshot to "
            "fetch and pin it (imports are a pure function of committed inputs)"
        )
    return json.loads(path.read_text(encoding="utf-8"))


# --- centerline fit (KTD2/KTD3) ------------------------------------------------------------------


@dataclass(frozen=True)
class FittedCenterline:
    """The fitted, uniformly resampled centerline in the local ENU frame (SI, ISO 8855).

    The fit-reporting fields carry through from
    :class:`outlap.trackcal.geometry.CenterlineFit`: ``discrepancy_rms_m`` is the residual the
    ``λ`` search matched (the declared noise, unless the search saturated), ``residual_rms_m``
    the reported curve's own residual — below the declaration once the bias correction runs,
    so the two must always be printed together — ``bias_corrected`` whether that step applied,
    and ``effective_dof`` the honest "am I interpolating" number.
    """

    name: str
    closed: bool
    length_m: float
    s: F
    x: F
    y: F
    kappa: F
    heading: F
    lat: F
    lon: F
    frame: EnuFrame
    residual_rms_m: float
    discrepancy_rms_m: float
    smoothing_lambda: float
    bias_corrected: bool
    effective_dof: float

    def __len__(self) -> int:
        return int(self.s.size)


def headings(x: F, y: F, *, closed: bool) -> F:
    """Travel direction ψ from +x per station (central differences; wraps when closed).

    Shared by every consumer of an emitted centerline (both importers and the width-trace QA
    tool), so the heading a station is traced/sampled along cannot drift between them.
    """
    if closed:
        dx = np.roll(x, -1) - np.roll(x, 1)
        dy = np.roll(y, -1) - np.roll(y, 1)
    else:
        dx = np.gradient(x)
        dy = np.gradient(y)
    return np.arctan2(dy, dx)


def fit_snapshot_centerline(
    snapshot: dict[str, Any],
    name: str,
    *,
    ds_m: float = 3.0,
    noise_std_m: float = _OSM_NOISE_STD_M,
) -> FittedCenterline:
    """Assemble the lap from a committed snapshot and fit the penalised periodic spline.

    Replaces the old linear-``np.interp`` resample: curvature comes analytically from the
    :func:`outlap.trackcal.geometry.fit_centerline` P-spline (KTD2/KTD3), with ``noise_std_m``
    the declared per-axis OSM digitisation noise the discrepancy principle matches.
    """
    node_ids = _assemble_circuit(snapshot)
    closed = len(node_ids) > 3 and node_ids[0] == node_ids[-1]
    nodes = {e["id"]: e for e in snapshot["elements"] if e["type"] == "node"}
    pts = [nodes[i] for i in node_ids if i in nodes]
    lats = [float(p["lat"]) for p in pts]
    lons = [float(p["lon"]) for p in pts]
    x_raw, y_raw, frame = _to_enu(lats, lons)
    fit = fit_centerline(
        np.asarray(x_raw, dtype=np.float64),
        np.asarray(y_raw, dtype=np.float64),
        closed=closed,
        noise_std_m=noise_std_m,
    )
    samples = fit.sample_uniform(ds_m)
    lat_s, lon_s = frame.to_latlon(samples.x_m, samples.y_m)
    return FittedCenterline(
        name=name,
        closed=closed,
        length_m=fit.length_m,
        s=samples.s_m,
        x=samples.x_m,
        y=samples.y_m,
        kappa=samples.kappa_per_m,
        heading=headings(samples.x_m, samples.y_m, closed=closed),
        lat=lat_s,
        lon=lon_s,
        frame=frame,
        residual_rms_m=fit.residual_rms_m,
        discrepancy_rms_m=fit.discrepancy_rms_m,
        smoothing_lambda=fit.smoothing_lambda,
        bias_corrected=fit.bias_corrected,
        effective_dof=fit.effective_dof,
    )


# --- DEM elevation (committed samples; network only under --refresh-snapshot) --------------------


def _dem_batch(dataset: str, chunk: list[tuple[float, float]]) -> list[float]:
    """One throttled DEM request (≤100 locations), with 429 back-off (public tier: 1 req/s)."""
    import time

    import requests

    loc = "|".join(f"{la:.6f},{lo:.6f}" for la, lo in chunk)
    for attempt in range(4):
        resp = requests.post(
            _DEM_URL.format(dataset=dataset),
            data={"locations": loc, "interpolation": "cubic"},
            headers=_HEADERS,
            timeout=60,
        )
        if resp.status_code == 429:
            time.sleep(2.0 * (attempt + 1))
            continue
        resp.raise_for_status()
        data = resp.json()
        if data.get("status") != "OK":
            raise ValueError(data.get("error", "DEM error"))
        return [
            float(r["elevation"]) if r["elevation"] is not None else 0.0
            for r in data["results"]
        ]
    raise RuntimeError("DEM rate-limited after retries")


def fetch_dem_elevation(
    lats: Sequence[float], lons: Sequence[float]
) -> tuple[list[float], str]:
    """Sample elevation from the first available open DEM dataset, throttled to the free tier."""
    import time

    pairs = list(zip(lats, lons, strict=True))
    batch = 100
    for dataset in _DEM_DATASETS:
        try:
            elevations: list[float] = []
            for i in range(0, len(pairs), batch):
                if i > 0:
                    time.sleep(1.1)  # public tier allows ~1 request/second
                elevations.extend(_dem_batch(dataset, pairs[i : i + batch]))
            return elevations, dataset
        except Exception as exc:  # noqa: BLE001 - try the next dataset, then give up
            print(f"  DEM {dataset} failed ({exc}); trying next", file=sys.stderr)
    raise RuntimeError("all DEM datasets failed")


def refresh_dem_samples(fc: FittedCenterline) -> dict[str, Any]:
    """Fetch DEM elevations at ~:data:`_DEM_STEP_M` stations (network; the pinned input)."""
    stride = max(int(round(_DEM_STEP_M / max(float(fc.s[1] - fc.s[0]), 1e-9))), 1)
    idx = list(range(0, len(fc), stride))
    if not fc.closed and idx[-1] != len(fc) - 1:
        idx.append(len(fc) - 1)
    print(
        f"  sampling DEM at {len(idx)} points (~{_DEM_STEP_M:.0f} m) …", file=sys.stderr
    )
    z, dataset = fetch_dem_elevation(
        [float(fc.lat[i]) for i in idx], [float(fc.lon[i]) for i in idx]
    )
    return {
        "dataset": dataset,
        "s_m": [round(float(fc.s[i]), 4) for i in idx],
        "lat_deg": [round(float(fc.lat[i]), 7) for i in idx],
        "lon_deg": [round(float(fc.lon[i]), 7) for i in idx],
        "z_m": [round(float(v), 3) for v in z],
    }


def load_dem_samples(track_dir: Path) -> dict[str, Any]:
    """Read the committed DEM samples from ``<track_dir>/dem_samples.json``."""
    path = track_dir / DEM_SAMPLES_FILE
    if not path.exists():
        raise MissingElevationError(
            f"no committed DEM samples at {path} — run once with --refresh-snapshot to "
            "fetch and pin them, use --stages lidar, or pass --no-dem for a flat track"
        )
    return json.loads(path.read_text(encoding="utf-8"))


def _fuse_dem(fc: FittedCenterline, samples: dict[str, Any]) -> F:
    """C² elevation at every station from the pinned coarse DEM samples (shared P-spline)."""
    profile = fuse_elevation(
        np.asarray(samples["s_m"], dtype=np.float64),
        np.asarray(samples["z_m"], dtype=np.float64),
        closed=fc.closed,
        length_m=fc.length_m if fc.closed else None,
        noise_std_m=_DEM_NOISE_STD_M,
        knot_spacing_m=_DEM_KNOT_SPACING_M,
    )
    return profile.evaluate(fc.s)


# --- enrichment stages (KTD10) -------------------------------------------------------------------


@dataclass(frozen=True)
class _WidthsResult:
    width_left: F
    width_right: F
    meta: dict[str, Any]  # track.yaml meta fields (width_source, control-point sha)
    manifest: dict[str, Any]


def run_widths_stage(
    fc: FittedCenterline,
    *,
    image_path: Path,
    control_points_path: Path | None = None,
) -> _WidthsResult:
    """Edge-trace per-row widths from a prepared orthophoto ``.npz`` (stage ``widths``).

    The ``.npz`` carries ``image`` (H×W grayscale or H×W×3 RGB) and ``transform`` (the six
    rasterio-order affine coefficients mapping pixel → world ENU); real tile fetchers plug in
    during the data session (U7). Unresolved stations raise (R1, via ``width_trace``).
    """
    if not image_path.exists():
        raise OsmTrackError(f"widths stage: orthophoto source not found: {image_path}")
    with np.load(image_path) as npz:
        if "image" not in npz or "transform" not in npz:
            raise OsmTrackError(
                f"widths stage: {image_path} must carry `image` and `transform` arrays"
            )
        image = np.asarray(npz["image"])
        coeffs = [float(v) for v in np.asarray(npz["transform"]).ravel()]
    if len(coeffs) != 6:
        raise OsmTrackError(
            f"widths stage: `transform` must be the 6 affine coefficients, got {len(coeffs)}"
        )
    cps: Sequence[ControlPoint] = ()
    cp_sha: str | None = None
    if control_points_path is not None:
        if not control_points_path.exists():
            raise OsmTrackError(
                f"widths stage: control-point file not found: {control_points_path}"
            )
        cps = load_control_points(control_points_path)
        cp_sha = sha256_file(control_points_path)
    source = ArrayImageSource(image, AffineTransform(*coeffs))
    stations = Stations(
        s_m=fc.s,
        x_m=fc.x,
        y_m=fc.y,
        heading_rad=fc.heading,
        length_m=fc.length_m if fc.closed else None,
    )
    result = trace_widths(stations, source, control_points=cps)
    meta: dict[str, Any] = {"width_source": "orthophoto"}
    if cp_sha is not None:
        meta["width_control_points_sha"] = cp_sha
    manifest: dict[str, Any] = {
        "image": {"file": image_path.name, "sha256": sha256_file(image_path)},
        "provenance": result.provenance.as_meta(),
    }
    if control_points_path is not None:
        manifest["control_points"] = {
            "file": control_points_path.name,
            "sha256": cp_sha,
        }
    return _WidthsResult(
        width_left=result.width_left_m,
        width_right=result.width_right_m,
        meta=meta,
        manifest=manifest,
    )


@dataclass(frozen=True)
class _LidarResult:
    z: F
    banking_deg: F
    meta: dict[str, Any]  # track.yaml meta fields (lidar_dataset, lidar_tiles)
    manifest: dict[str, Any]


def run_lidar_stage(
    fc: FittedCenterline,
    *,
    source_id: str,
    tile_ids: Sequence[str] = (),
    cache_dir: Path | None = None,
    half_width_m: F | float = 6.0,
    sampler: Callable[[F, F], F] | None = None,
) -> _LidarResult:
    """LiDAR DTM z + cross-section banking (stage ``lidar``), replacing the DEM chain.

    ``sampler`` is the injectable ENU elevation seam (tests pass analytic surfaces); when
    ``None`` the DTM tiles are resolved from ``cache_dir`` (never committed) and sampled via
    rasterio/pyproj. Tile IDs + dataset version are pinned in the manifest (KTD7).
    """
    if source_id not in LIDAR_PRESETS:
        known = ", ".join(sorted(LIDAR_PRESETS))
        raise OsmTrackError(f"unknown LiDAR source `{source_id}` (known: {known})")
    source = LIDAR_PRESETS[source_id]
    if sampler is None:
        if cache_dir is None:
            raise OsmTrackError(
                "lidar stage: --lidar-cache is required to resolve tiles"
            )
        tiles = resolve_tiles(source, tile_ids, cache_dir)
        mosaic = open_dtm(tiles.paths, source=source)
        sampler = make_enu_sampler(mosaic, crs=source.crs, frame=fc.frame)
    else:
        # An injected sampler resolves no files, but the provenance it pins is the same record.
        tiles = ResolvedTiles(source=source, tile_ids=tuple(tile_ids), paths=())
    manifest: dict[str, Any] = dict(tiles.manifest())
    profile = fuse_elevation(
        fc.s,
        np.asarray(sampler(fc.x, fc.y), dtype=np.float64),
        closed=fc.closed,
        length_m=fc.length_m if fc.closed else None,
        noise_std_m=source.noise_floor_m,
        knot_spacing_m=_LIDAR_KNOT_SPACING_M,
    )
    banking = estimate_banking(
        fc.x,
        fc.y,
        sampler,
        closed=fc.closed,
        half_width_m=half_width_m,
        noise_floor_m=source.noise_floor_m,
        source=source,
    )
    manifest["banking"] = banking.provenance()
    meta = {
        "lidar_dataset": f"{source.dataset} {source.version}",
        "lidar_tiles": list(tile_ids),
    }
    return _LidarResult(
        z=profile.z_m, banking_deg=banking.banking_deg, meta=meta, manifest=manifest
    )


def derive_accuracy_class(stages: Sequence[str], *, has_elevation: bool) -> str:
    """The honest accuracy class from what actually ran (KTD10; mapping in the module doc)."""
    if STAGE_WIDTHS in stages and STAGE_LIDAR in stages:
        return "A"
    if has_elevation:
        return "B"
    return "C"


# --- rendering + atomic writes (KTD10) -----------------------------------------------------------


def render_centerline_csv(
    fc: FittedCenterline,
    z: F,
    banking_deg: F,
    width_left: F,
    width_right: F,
    *,
    elevation_note: str | None,
    source_note: str = "OSM (ODbL) centerline",
) -> str:
    """Fixed-precision ``centerline.csv`` text — the byte-determinism surface (KTD7).

    ONE renderer serves every importer that emits this format, so the byte layout cannot
    drift between them; ``source_note`` names the provenance in the header comment (the
    FastF1 driven-line importer passes its own).
    """
    lines = [f"# {fc.name} — {source_note}"]
    if elevation_note:
        lines[0] += f" + {elevation_note}"
    lines.append("s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale")
    for i in range(len(fc)):
        lines.append(
            f"{fc.s[i]:.4f},{fc.x[i]:.4f},{fc.y[i]:.4f},{z[i]:.4f},"
            f"{banking_deg[i]:.3f},{width_left[i]:.3f},{width_right[i]:.3f},1.0000"
        )
    return "\n".join(lines) + "\n"


def render_yaml(doc: dict[str, Any], header: str) -> str:
    """Render a ``track.yaml``/``manifest.yaml`` document under its provenance header.

    Shared with the FastF1 driven-line importer for the same reason as
    :func:`render_centerline_csv`: one layout, so the emitted YAML cannot drift between them.
    """
    return header + yaml.safe_dump(
        doc, sort_keys=False, allow_unicode=True, default_flow_style=False
    )


def write_track_dir(
    track_dir: Path, files: Mapping[str, str], *, force: bool = False
) -> None:
    """Atomically materialise ``files`` in ``track_dir`` (KTD10).

    Every file is staged in a fresh temp dir on the same filesystem, then moved into place
    with ``os.replace`` — an interruption before the first replace leaves the target
    untouched (the temp dir is always cleaned up). Any pre-existing file among ``files``
    requires ``force``.
    """
    track_dir.mkdir(parents=True, exist_ok=True)
    existing = sorted(name for name in files if (track_dir / name).exists())
    if existing and not force:
        raise OutputExistsError(
            f"refusing to overwrite {', '.join(existing)} in {track_dir} (pass --force)"
        )
    tmp = Path(
        tempfile.mkdtemp(prefix=f".{track_dir.name}.build-", dir=track_dir.parent)
    )
    try:
        staged: list[tuple[Path, Path]] = []
        for name, content in files.items():
            src = tmp / name
            src.write_text(content, encoding="utf-8")
            staged.append((src, track_dir / name))
        for src, dst in staged:
            os.replace(src, dst)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# --- the staged import ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ImportResult:
    """What an import produced (paths are inside ``track_dir``)."""

    track_dir: Path
    stages: tuple[str, ...]
    accuracy_class: str
    length_m: float
    n_stations: int
    files: tuple[str, ...]


def run_import(
    track_dir: Path,
    *,
    name: str,
    lat: float | None = None,
    lon: float | None = None,
    radius_m: int = 2500,
    stages: Sequence[str] = (),
    half_width_m: float | None = None,
    elevation: bool = True,
    refresh_snapshot: bool = False,
    force: bool = False,
    ds_m: float = 3.0,
    noise_std_m: float = _OSM_NOISE_STD_M,
    width_image: Path | None = None,
    width_control_points: Path | None = None,
    lidar_source: str | None = None,
    lidar_tiles: Sequence[str] = (),
    lidar_cache: Path | None = None,
    lidar_sampler: Callable[[F, F], F] | None = None,
    georef_transform: str | None = None,
) -> ImportResult:
    """Run the staged, pinned, atomic import into ``track_dir``.

    Base import always runs (committed snapshot → fitted centerline); ``stages`` selects
    enrichment (KTD10). The ONLY network touches are under ``refresh_snapshot`` (Overpass +
    opentopodata) — everything else is a pure function of the committed inputs (KTD7).
    ``half_width_m`` is the explicit degraded width path, recorded as ``width_source:
    declared`` (R1: without it or the ``widths`` stage the import errors).
    """
    stage_list = list(dict.fromkeys(stages))  # de-dupe, keep order
    unknown = [st for st in stage_list if st not in KNOWN_STAGES]
    if unknown:
        raise OsmTrackError(
            f"unknown stage(s) {', '.join(unknown)} (known: {', '.join(KNOWN_STAGES)})"
        )
    if STAGE_TELEMETRY_AUDIT in stage_list:
        raise NotImplementedError(
            "the telemetry-audit stage lands with the track-quality gate (U8); "
            "run outlap.trackcal against FastF1 positions offline for now"
        )

    files: dict[str, str] = {}

    # 1) The pinned OSM snapshot (KTD7): network ONLY under refresh_snapshot.
    if refresh_snapshot:
        if lat is None or lon is None:
            raise OsmTrackError("--refresh-snapshot needs a --preset or --lat/--lon")
        print(
            f"fetching OSM raceway near {name} ({lat},{lon}, r={radius_m} m) …",
            file=sys.stderr,
        )
        snapshot = fetch_raceway_ways(lat, lon, radius_m)
        files[SNAPSHOT_FILE] = _canonical_json(snapshot)
        snapshot_sha = sha256_text(files[SNAPSHOT_FILE])
    else:
        snapshot = load_snapshot(track_dir)
        snapshot_sha = sha256_file(track_dir / SNAPSHOT_FILE)

    # 2) Base centerline: assembly + the curvature-first fit (KTD2/KTD3).
    fc = fit_snapshot_centerline(snapshot, name, ds_m=ds_m, noise_std_m=noise_std_m)
    # Never a bare "residual rms": after the bias correction the reported curve sits well
    # inside the declared noise, which read alone looks like an interpolating fit. Declared →
    # matched → final says what the λ search targeted and what actually shipped.
    print(
        f"  fitted {len(fc)} stations, {fc.length_m:.0f} m (noise declared "
        f"{noise_std_m:.2f} m, λ matched {fc.discrepancy_rms_m:.2f} m, final residual "
        f"{fc.residual_rms_m:.2f} m)",
        file=sys.stderr,
    )

    manifest_inputs: dict[str, Any] = {
        "osm_snapshot": {"file": SNAPSHOT_FILE, "sha256": snapshot_sha}
    }
    meta: dict[str, Any] = {}
    notes: list[str] = []

    # 3) Widths (R1: never silently defaulted).
    if STAGE_WIDTHS in stage_list and half_width_m is not None:
        raise OsmTrackError(
            "choose ONE width source: the `widths` stage or an explicit --half-width"
        )
    if STAGE_WIDTHS in stage_list:
        if width_image is None:
            raise OsmTrackError(
                "widths stage: pass --width-image (a prepared orthophoto .npz)"
            )
        widths = run_widths_stage(
            fc, image_path=width_image, control_points_path=width_control_points
        )
        width_left, width_right = widths.width_left, widths.width_right
        meta.update(widths.meta)
        manifest_inputs["widths"] = widths.manifest
    elif half_width_m is not None:
        if half_width_m <= 0.0:
            raise OsmTrackError(f"--half-width must be > 0, got {half_width_m}")
        width_left = np.full(len(fc), float(half_width_m))
        width_right = np.full(len(fc), float(half_width_m))
        meta["width_source"] = "declared"
        manifest_inputs["half_width_m"] = float(half_width_m)
        notes.append(
            f"widths DECLARED at a constant {half_width_m:g} m per side "
            "(explicit degraded import, not measured)"
        )
    else:
        raise MissingWidthSourceError(
            "no width source resolves: run the `widths` stage (orthophoto edge trace) or "
            "declare an explicit degraded constant with --half-width (recorded as "
            "width_source: declared) — widths are never silently defaulted"
        )

    # 4) Elevation + banking: lidar stage > committed DEM samples > explicitly flat.
    z: F
    banking_deg: F = np.zeros(len(fc))
    dem_dataset: str | None = None
    if STAGE_LIDAR in stage_list:
        if lidar_source is None:
            raise OsmTrackError("lidar stage: pass --lidar-source (a lidar_dem preset)")
        lidar = run_lidar_stage(
            fc,
            source_id=lidar_source,
            tile_ids=lidar_tiles,
            cache_dir=lidar_cache,
            half_width_m=np.minimum(width_left, width_right),
            sampler=lidar_sampler,
        )
        z, banking_deg = lidar.z, lidar.banking_deg
        meta.update(lidar.meta)
        manifest_inputs["lidar"] = lidar.manifest
        resolved = int(np.count_nonzero(banking_deg != 0.0))
        notes.append(
            f"z + banking from LiDAR cross-sections ({resolved}/{len(fc)} stations "
            "with resolved non-zero banking; unresolved sections are honest zeros)"
        )
    elif elevation:
        if refresh_snapshot:
            samples = refresh_dem_samples(fc)
            files[DEM_SAMPLES_FILE] = _canonical_json(samples)
            dem_sha = sha256_text(files[DEM_SAMPLES_FILE])
        else:
            samples = load_dem_samples(track_dir)
            dem_sha = sha256_file(track_dir / DEM_SAMPLES_FILE)
        z = _fuse_dem(fc, samples)
        dem_dataset = str(samples["dataset"])
        manifest_inputs["dem_samples"] = {
            "file": DEM_SAMPLES_FILE,
            "dataset": dem_dataset,
            "sha256": dem_sha,
        }
        notes.append("banking not resolved from the coarse DEM (run the lidar stage)")
    else:
        z = np.zeros(len(fc))
        notes.append("flat import (--no-dem): z = 0 everywhere")

    if georef_transform:
        manifest_inputs["georef_transform"] = georef_transform

    # 5) Meta + manifest (KTD7/KTD9/KTD10).
    stages_ran = ["base", *stage_list]
    has_elevation = STAGE_LIDAR in stage_list or dem_dataset is not None
    accuracy = derive_accuracy_class(stage_list, has_elevation=has_elevation)
    attribution = "© OpenStreetMap contributors (ODbL)"
    if dem_dataset:
        attribution += f"; elevation {dem_dataset} via opentopodata.org"
    if STAGE_LIDAR in stage_list and lidar_source in LIDAR_PRESETS:
        src = LIDAR_PRESETS[lidar_source]
        attribution += f"; z/banking {src.name} ({src.license})"

    full_meta: dict[str, Any] = {
        "source": "osm+dem" if dem_dataset else "osm",
        **({"dem": dem_dataset} if dem_dataset else {}),
        "accuracy_class": accuracy,
        "attribution": attribution,
        **meta,
        **({"georef_transform": georef_transform} if georef_transform else {}),
        "importer_version": IMPORTER_VERSION,
        "stages": stages_ran,
        **({"notes": "; ".join(notes)} if notes else {}),
    }
    track_doc: dict[str, Any] = {
        "schema": "track/1.1",
        "name": fc.name,
        "closed": fc.closed,
        "centerline": "centerline.csv",
        "meta": full_meta,
    }

    elevation_note = None
    if dem_dataset:
        elevation_note = f"{dem_dataset} elevation"
    elif STAGE_LIDAR in stage_list:
        elevation_note = "LiDAR z/banking"
    csv_text = render_centerline_csv(
        fc,
        z,
        banking_deg,
        width_left,
        width_right,
        elevation_note=elevation_note,
    )

    manifest: dict[str, Any] = {
        "importer": "outlap.importers.osm_track",
        "importer_version": IMPORTER_VERSION,
        "track": fc.name,
        "stages": stages_ran,
        "parameters": {
            "ds_m": float(ds_m),
            "noise_std_m": float(noise_std_m),
            "dem_noise_std_m": _DEM_NOISE_STD_M,
            "dem_knot_spacing_m": _DEM_KNOT_SPACING_M,
        },
        "inputs": manifest_inputs,
        "outputs": {"centerline_csv_sha256": sha256_text(csv_text)},
    }

    files["centerline.csv"] = csv_text
    files["track.yaml"] = render_yaml(
        track_doc, "# Imported by outlap.importers.osm_track — public data only.\n"
    )
    files[MANIFEST_FILE] = render_yaml(
        manifest,
        "# Input manifest (KTD7): this import is a pure function of these committed inputs.\n",
    )
    write_track_dir(track_dir, files, force=force)
    return ImportResult(
        track_dir=track_dir,
        stages=tuple(stages_ran),
        accuracy_class=accuracy,
        length_m=fc.length_m,
        n_stations=len(fc),
        files=tuple(files),
    )


# --- CLI -----------------------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        prog="outlap.importers.osm_track", description=__doc__
    )
    parser.add_argument("--preset", choices=sorted(PRESETS), help="a known circuit")
    parser.add_argument("--name", help="circuit name (with --lat/--lon)")
    parser.add_argument("--lat", type=float)
    parser.add_argument("--lon", type=float)
    parser.add_argument("--radius", type=int, default=2500, help="OSM search radius, m")
    parser.add_argument("--out", type=Path, required=True, help="the track directory")
    parser.add_argument(
        "--refresh-snapshot",
        action="store_true",
        help="fetch Overpass + DEM and pin them as committed inputs (the ONLY network path)",
    )
    parser.add_argument(
        "--stages",
        default="",
        help=f"comma-separated enrichment stages: {','.join(KNOWN_STAGES)}",
    )
    parser.add_argument(
        "--half-width",
        type=float,
        default=None,
        help="explicit degraded constant half-width, m (recorded as width_source: declared)",
    )
    parser.add_argument(
        "--force", action="store_true", help="overwrite existing generated files"
    )
    parser.add_argument("--ds", type=float, default=3.0, help="resample spacing, m")
    parser.add_argument(
        "--noise-std",
        type=float,
        default=_OSM_NOISE_STD_M,
        help="declared per-axis OSM digitisation noise for the centerline fit, m",
    )
    parser.add_argument(
        "--no-dem", action="store_true", help="skip elevation (flat track)"
    )
    parser.add_argument(
        "--width-image", type=Path, help="widths stage: prepared orthophoto .npz"
    )
    parser.add_argument(
        "--width-control-points",
        type=Path,
        help="widths stage: hand-QA control-point CSV (committed input)",
    )
    parser.add_argument(
        "--lidar-source",
        choices=sorted(LIDAR_PRESETS),
        help="lidar stage: DTM source preset",
    )
    parser.add_argument(
        "--lidar-tiles", default="", help="lidar stage: comma-separated tile IDs"
    )
    parser.add_argument(
        "--lidar-cache",
        type=Path,
        help="lidar stage: local tile cache (never committed)",
    )
    parser.add_argument(
        "--georef",
        help="fitted georeference transform (compact string), recorded in meta + manifest",
    )
    args = parser.parse_args(argv)

    if args.preset:
        name, lat, lon, radius = PRESETS[args.preset]
    elif args.name and args.lat is not None and args.lon is not None:
        name, lat, lon, radius = args.name, args.lat, args.lon, args.radius
    elif args.name:
        name, lat, lon, radius = args.name, None, None, args.radius
    else:
        parser.error(
            "give --preset or --name (with --lat/--lon to refresh the snapshot)"
        )

    stages = [s.strip() for s in args.stages.split(",") if s.strip()]
    tiles = [t.strip() for t in args.lidar_tiles.split(",") if t.strip()]
    result = run_import(
        args.out,
        name=name,
        lat=lat,
        lon=lon,
        radius_m=radius,
        stages=stages,
        half_width_m=args.half_width,
        elevation=not args.no_dem,
        refresh_snapshot=args.refresh_snapshot,
        force=args.force,
        ds_m=args.ds,
        noise_std_m=args.noise_std,
        width_image=args.width_image,
        width_control_points=args.width_control_points,
        lidar_source=args.lidar_source,
        lidar_tiles=tiles,
        lidar_cache=args.lidar_cache,
        georef_transform=args.georef,
    )
    print(
        f"wrote {', '.join(result.files)} in {result.track_dir} "
        f"({result.n_stations} points, {result.length_m:.0f} m, "
        f"class {result.accuracy_class}, stages: {', '.join(result.stages)})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
