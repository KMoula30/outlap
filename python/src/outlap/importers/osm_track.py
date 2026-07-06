# SPDX-License-Identifier: AGPL-3.0-only
"""OSM + DEM → ``track.yaml`` + ``centerline.csv`` (§9.3, Locked Decision #13).

Since no open **3D** circuit data exists, the importer builds it (HANDOFF §9.3):

1. **Centerline** from OpenStreetMap ``highway=raceway`` ways (ODbL — redistributable with
   attribution), assembled into the longest ordered polyline near the circuit and projected to a
   local ENU metric frame.
2. **Elevation** fused from an open DEM (Copernicus GLO-30 / EU-DEM via the free opentopodata API),
   sampled along the centerline and smoothed **C²-consistently** with a cubic smoothing spline
   (`z` and its derivatives are continuous — the outlap-track spline needs `z''` for vertical
   curvature).
3. **Banking** is left at zero here (coarse public DEMs cannot resolve it); it is supplied later as
   sparse ``banking_keypoints`` in ``track.yaml`` or hand-annotated. Accuracy class is recorded in
   the track meta accordingly.

This is network-facing tooling and is **never run in CI**. It reads only public data and never
touches proprietary sources (firewall, §1).

Example:
    python -m outlap.importers.osm_track --preset catalunya --out data/tracks/catalunya_osm
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

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
    "User-Agent": "outlap-track-importer/0.1 (+https://github.com/KMoula30/outlap)",
    "Accept": "application/json",
}
# opentopodata: free, no key; EU-DEM (25 m) covers the European circuits, SRTM is the global fallback.
_DEM_URL = "https://api.opentopodata.org/v1/{dataset}"
_DEM_DATASETS = ("eudem25m", "srtm30m")
_EARTH_R = 6_371_000.0


@dataclass
class Centerline3D:
    """An assembled, resampled centerline in the local ENU frame (metres)."""

    name: str
    s: list[float]
    x: list[float]
    y: list[float]
    z: list[float]
    width_left: list[float]
    width_right: list[float]
    grip_scale: list[float]
    lat: list[float] = field(default_factory=list)
    lon: list[float] = field(default_factory=list)
    dem_dataset: str | None = None

    def __len__(self) -> int:
        return len(self.s)


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
    """Return the node-id sequence of the longest raceway way (the main circuit layout)."""
    nodes = {e["id"]: e for e in osm["elements"] if e["type"] == "node"}
    ways = [
        e for e in osm["elements"] if e["type"] == "way" and len(e.get("nodes", [])) > 1
    ]
    if not ways:
        raise ValueError("no raceway ways found near the given point")

    def way_length(w: dict[str, Any]) -> float:
        ns = [nodes[i] for i in w["nodes"] if i in nodes]
        return sum(
            _haversine(a["lat"], a["lon"], b["lat"], b["lon"])
            for a, b in zip(ns, ns[1:], strict=False)
        )

    return max(ways, key=way_length)["nodes"]


def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _EARTH_R * math.asin(math.sqrt(a))


def _to_enu(lats: list[float], lons: list[float]) -> tuple[list[float], list[float]]:
    """Equirectangular projection to a local ENU metric frame centred on the centroid."""
    lat0 = sum(lats) / len(lats)
    lon0 = sum(lons) / len(lons)
    coslat = math.cos(math.radians(lat0))
    x = [math.radians(lon - lon0) * _EARTH_R * coslat for lon in lons]
    y = [math.radians(lat - lat0) * _EARTH_R for lat in lats]
    return x, y


# --- resampling ---------------------------------------------------------------------------------


def _cumulative_s(x: list[float], y: list[float]) -> list[float]:
    s = [0.0]
    for i in range(1, len(x)):
        s.append(s[-1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]))
    return s


def _resample_uniform(
    s: list[float],
    cols: dict[str, list[float]],
    ds: float,
) -> tuple[list[float], dict[str, list[float]]]:
    """Resample every column to a uniform arc-length grid by piecewise-linear interpolation."""
    import numpy as np

    total = s[-1]
    n = max(int(round(total / ds)), 4)
    new_s = [float(v) for v in np.linspace(0.0, total, n, endpoint=False)]
    out = {k: [float(x) for x in np.interp(new_s, s, v)] for k, v in cols.items()}
    return new_s, out


# --- DEM elevation --------------------------------------------------------------------------------


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
    lats: list[float], lons: list[float]
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


def _smooth_elevation(s: list[float], z: list[float], smoothing: float) -> list[float]:
    """C²-consistent elevation: fit a cubic smoothing spline so z, z', z'' are continuous."""
    import numpy as np
    from scipy.interpolate import UnivariateSpline

    arr_s = np.asarray(s)
    arr_z = np.asarray(z)
    # s= is the total squared-residual budget; scale by point count so it is resolution-independent.
    spline = UnivariateSpline(arr_s, arr_z, k=3, s=smoothing * len(s))
    smoothed = np.asarray(spline(arr_s), dtype=np.float64).ravel()
    return [float(v) for v in smoothed]


# --- build + emit -------------------------------------------------------------------------------


def build_centerline(
    name: str,
    lat: float,
    lon: float,
    radius_m: int,
    *,
    ds: float = 3.0,
    half_width_m: float = 6.0,
    with_dem: bool = True,
    smoothing: float = 0.5,
) -> Centerline3D:
    """Assemble a 3D centerline for the circuit near ``(lat, lon)``."""
    print(
        f"fetching OSM raceway near {name} ({lat},{lon}, r={radius_m} m) …",
        file=sys.stderr,
    )
    osm = fetch_raceway_ways(lat, lon, radius_m)
    node_ids = _longest_way(osm)
    nodes = {e["id"]: e for e in osm["elements"] if e["type"] == "node"}
    pts = [nodes[i] for i in node_ids if i in nodes]
    lats = [p["lat"] for p in pts]
    lons = [p["lon"] for p in pts]
    x, y = _to_enu(lats, lons)
    s = _cumulative_s(x, y)
    print(f"  assembled {len(pts)} nodes, ~{s[-1]:.0f} m", file=sys.stderr)

    cols = {"x": x, "y": y, "lat": lats, "lon": lons}
    new_s, r = _resample_uniform(s, cols, ds)

    dem_dataset = None
    if with_dem:
        import numpy as np

        # A 25–30 m DEM does not resolve 3 m spacing; sample every ~20 m, then interpolate + smooth.
        dem_step_m = 20.0
        stride = max(int(round(dem_step_m / ds)), 1)
        idx = list(range(0, len(new_s), stride))
        if idx[-1] != len(new_s) - 1:
            idx.append(len(new_s) - 1)
        print(
            f"  sampling DEM at {len(idx)} points (~{dem_step_m:.0f} m) …",
            file=sys.stderr,
        )
        z_coarse, dem_dataset = fetch_dem_elevation(
            [r["lat"][i] for i in idx], [r["lon"][i] for i in idx]
        )
        coarse_s = [new_s[i] for i in idx]
        z_interp = [float(v) for v in np.interp(new_s, coarse_s, z_coarse)]
        z = _smooth_elevation(new_s, z_interp, smoothing)
    else:
        z = [0.0] * len(new_s)

    n = len(new_s)
    return Centerline3D(
        name=name,
        s=new_s,
        x=r["x"],
        y=r["y"],
        z=z,
        width_left=[half_width_m] * n,
        width_right=[half_width_m] * n,
        grip_scale=[1.0] * n,
        lat=r["lat"],
        lon=r["lon"],
        dem_dataset=dem_dataset,
    )


def write_track(
    cl: Centerline3D, out_dir: Path, *, closed: bool = True
) -> tuple[Path, Path]:
    """Write ``centerline.csv`` + ``track.yaml`` for an assembled centerline."""
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "centerline.csv"
    yaml_path = out_dir / "track.yaml"

    with csv_path.open("w", encoding="utf-8") as f:
        f.write(f"# {cl.name} — OSM (ODbL) centerline")
        if cl.dem_dataset:
            f.write(f" + {cl.dem_dataset} elevation")
        f.write("\ns_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale\n")
        for i in range(len(cl)):
            f.write(
                f"{cl.s[i]:.4f},{cl.x[i]:.4f},{cl.y[i]:.4f},{cl.z[i]:.4f},"
                f"0.000,{cl.width_left[i]:.3f},{cl.width_right[i]:.3f},{cl.grip_scale[i]:.4f}\n"
            )

    accuracy = "B" if cl.dem_dataset else "C"
    attribution = "© OpenStreetMap contributors (ODbL)"
    if cl.dem_dataset:
        attribution += f"; elevation {cl.dem_dataset} via opentopodata.org"
    lines = [
        "# Imported by outlap.importers.osm_track — public data only.",
        "schema: track/1.0",
        f"name: {cl.name}",
        f"closed: {'true' if closed else 'false'}",
        "centerline: centerline.csv",
        "meta:",
        "  source: osm+dem" if cl.dem_dataset else "  source: osm",
        *([f"  dem: {cl.dem_dataset}"] if cl.dem_dataset else []),
        f"  accuracy_class: {accuracy}",
        f'  attribution: "{attribution}"',
        '  notes: "widths defaulted; banking not resolved from DEM (add keypoints to refine)"',
        "",
    ]
    yaml_path.write_text("\n".join(lines), encoding="utf-8")
    return yaml_path, csv_path


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
    parser.add_argument("--ds", type=float, default=3.0, help="resample spacing, m")
    parser.add_argument(
        "--no-dem", action="store_true", help="skip elevation (flat track)"
    )
    parser.add_argument("--out", type=Path, required=True, help="output directory")
    args = parser.parse_args(argv)

    if args.preset:
        name, lat, lon, radius = PRESETS[args.preset]
    elif args.name and args.lat is not None and args.lon is not None:
        name, lat, lon, radius = args.name, args.lat, args.lon, args.radius
    else:
        parser.error("give --preset or (--name --lat --lon)")

    cl = build_centerline(name, lat, lon, radius, ds=args.ds, with_dem=not args.no_dem)
    yaml_path, csv_path = write_track(cl, args.out)
    print(f"wrote {yaml_path} and {csv_path} ({len(cl)} points, {cl.s[-1]:.0f} m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
