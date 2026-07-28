# SPDX-License-Identifier: AGPL-3.0-only
"""Render the width-trace QA overlay (semi-automated per-row track widths).

Two panels from one :func:`outlap.importers.width_trace.trace_widths` run:

* (a) plan view — the orthophoto, the traced left/right edges, the raw automatic
  detections, the hand-placed control points, and any cross-check flags;
* (b) corridor width vs arc length — per-side and total traced widths, the cross-check
  reference with its agreement band, control-point stations, and the flagged stations.

Runs headless (Agg) and standalone:

    # self-contained synthetic fixture (no data needed):
    uv run --extra track-import python tools/plot_track_width_qa.py --synthetic

    # a real track dir + a local orthophoto bundle + the committed control points:
    uv run --extra track-import python tools/plot_track_width_qa.py \\
        --track data/tracks/catalunya_osm --ortho catalunya_ortho.npz \\
        --control-points catalunya_width_cps.csv --out scratch_figs/catalunya_width_qa.png

The ``--ortho`` bundle is an ``.npz`` with ``image`` (H×W grayscale or H×W×3 RGB) and
``transform`` (the six affine coefficients ``a,b,c,d,e,f`` mapping pixel (col,row) → local
ENU (x,y); ``AffineTransform.from_origin`` produces the north-up case). If the trace leaves
unresolved stations the tool prints the typed error's station list and exits nonzero — the
same error-not-default contract as the importer (R1).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from outlap.importers import width_trace as wt
from outlap.importers.osm_track import headings

mpl.use("Agg", force=True)  # QA renders headless (CI, remote sessions)

# --- design-system palette (validated categorical slots; text in ink tokens) -----------------
BLUE, AQUA, YELLOW, GREEN, VIOLET, RED = (
    "#2a78d6",
    "#1baf7a",
    "#eda100",
    "#008300",
    "#4a3aa7",
    "#e34948",
)
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e7e6e2", "#fcfcfb"

mpl.rcParams.update(
    {
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": MUTED,
        "axes.labelcolor": INK,
        "text.color": INK,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "grid.color": GRID,
        "axes.grid": True,
        "grid.linewidth": 0.6,
        "font.size": 9.0,
        "legend.framealpha": 0.9,
    }
)

_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_OUT = _ROOT / "scratch_figs" / "track_width_qa.png"


def _load_track_stations(track_dir: Path) -> wt.Stations:
    """Stations from a track dir's ``centerline.csv`` (columns by NAME, never position)."""
    csv_path = track_dir / "centerline.csv"
    rows = [
        ln.strip()
        for ln in csv_path.read_text(encoding="utf-8").splitlines()
        if ln.strip() and not ln.strip().startswith("#")
    ]
    header = [h.strip() for h in rows[0].split(",")]
    col = {name: i for i, name in enumerate(header)}
    for need in ("s_m", "x_m", "y_m"):
        if need not in col:
            raise SystemExit(f"{csv_path}: missing column '{need}' (found {header})")
    data = np.array([[float(v) for v in ln.split(",")] for ln in rows[1:]])
    s, x, y = data[:, col["s_m"]], data[:, col["x_m"]], data[:, col["y_m"]]
    length = float(s[-1] + math.hypot(x[0] - x[-1], y[0] - y[-1]))
    return wt.Stations(
        s_m=s, x_m=x, y_m=y, heading_rad=headings(x, y, closed=True), length_m=length
    )


def _load_ortho(path: Path) -> wt.ArrayImageSource:
    """The ``.npz`` orthophoto bundle: ``image`` + 6-coefficient ``transform``."""
    bundle = np.load(path)
    coeffs = [float(v) for v in np.asarray(bundle["transform"]).ravel()]
    if len(coeffs) != 6:
        raise SystemExit(f"{path}: 'transform' must hold 6 affine coefficients")
    return wt.ArrayImageSource(bundle["image"], wt.AffineTransform(*coeffs))


def _synthetic_fixture() -> tuple[
    wt.Stations,
    wt.ArrayImageSource,
    list[wt.ControlPoint],
    tuple[np.ndarray, np.ndarray],
]:
    """A drawn circular track exercising every overlay element.

    12 m corridor on grass; a flat-grey wedge at the start/finish kills detection there
    (rescued by two control points, showing the blend); a pit lane runs inboard beyond the
    search band; a LiDAR bump at half distance triggers a cross-check flag.
    """
    radius, n = 80.0, 144
    theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    stations = wt.Stations(
        s_m=radius * theta,
        x_m=radius * np.cos(theta),
        y_m=radius * np.sin(theta),
        heading_rad=theta + math.pi / 2.0,
        length_m=2.0 * math.pi * radius,
    )
    half = radius + 6.0 + 25.0
    tf = wt.AffineTransform.from_origin(-half, half, 0.25)
    npx = int(round(2.0 * half / 0.25))
    xc, yc = tf.pixel_centers((npx, npx))
    r = np.hypot(xc, yc)
    img = np.full((npx, npx), 0.75)
    img[(r >= radius - 6.0) & (r <= radius + 6.0)] = 0.30
    img[(r >= radius - 14.0) & (r <= radius - 11.0)] = 0.30  # pit lane, off-band
    img[np.abs(np.arctan2(yc, xc)) <= 0.15] = 0.5  # detection gap at start/finish
    cps = [
        wt.ControlPoint(s_m=0.0, side="left", offset_m=6.5),
        wt.ControlPoint(s_m=0.0, side="right", offset_m=6.5),
    ]
    s_ref = np.asarray(stations.s_m)
    w_ref = np.full(n, 12.0)
    w_ref[n // 2] = 15.0  # a deliberate LiDAR disagreement → one flagged station
    return stations, wt.ArrayImageSource(img, tf), cps, (s_ref, w_ref)


def _edge_xy(
    stations: wt.Stations, offsets: np.ndarray, sign: float
) -> tuple[np.ndarray, np.ndarray]:
    """World points of an edge polyline: station + sign·offset along the left normal."""
    nx = -np.sin(stations.heading_rad) * sign
    ny = np.cos(stations.heading_rad) * sign
    return stations.x_m + nx * offsets, stations.y_m + ny * offsets


def _closed(a: np.ndarray) -> np.ndarray:
    return np.append(a, a[0])


def render(
    stations: wt.Stations,
    image: wt.ArrayImageSource,
    result: wt.WidthTraceResult,
    control_points: list[wt.ControlPoint],
    crosscheck: tuple[np.ndarray, np.ndarray] | None,
    band_m: float,
    out: Path,
    title: str,
) -> Path:
    """Draw the two-panel QA overlay and write it to ``out``."""
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13.0, 5.6))

    # (a) plan view over the orthophoto ------------------------------------------------------
    tf = image.transform
    nrows, ncols = image.image.shape
    if tf.b == 0.0 and tf.d == 0.0:  # north-up raster: imshow can place it directly
        extent = (tf.c, tf.c + tf.a * ncols, tf.f + tf.e * nrows, tf.f)
        ax0.imshow(
            image.image,
            extent=extent,
            origin="upper",
            cmap="gray",
            vmin=0.0,
            vmax=1.0,
            alpha=0.85,
            zorder=0,
        )
    ax0.plot(
        _closed(stations.x_m),
        _closed(stations.y_m),
        color=MUTED,
        lw=0.8,
        ls="--",
        label="centerline",
    )
    for width, sign, color, name in (
        (result.width_left_m, +1.0, BLUE, "left edge (traced)"),
        (result.width_right_m, -1.0, AQUA, "right edge (traced)"),
    ):
        ex, ey = _edge_xy(stations, width, sign)
        ax0.plot(_closed(ex), _closed(ey), color=color, lw=1.6, label=name)
    for detected, sign in (
        (result.detected_left_m, +1.0),
        (result.detected_right_m, -1.0),
    ):
        ok = ~np.isnan(detected)
        dx, dy = _edge_xy(stations, np.where(ok, detected, 0.0), sign)
        ax0.plot(
            dx[ok][::3],
            dy[ok][::3],
            ".",
            color=YELLOW,
            ms=3.0,
            ls="none",
            label="auto detection" if sign > 0 else None,
        )
    for i, cp in enumerate(control_points):
        cx = float(np.interp(cp.s_m, stations.s_m, stations.x_m))
        cy = float(np.interp(cp.s_m, stations.s_m, stations.y_m))
        ax0.plot(
            cx,
            cy,
            "*",
            color=VIOLET,
            ms=12.0,
            mec=SURFACE,
            label="control point" if i == 0 else None,
        )
    for j, s_flag in enumerate(result.provenance.flagged_stations_m()):
        fx = float(np.interp(s_flag, stations.s_m, stations.x_m))
        fy = float(np.interp(s_flag, stations.s_m, stations.y_m))
        ax0.plot(
            fx,
            fy,
            "x",
            color=RED,
            ms=10.0,
            mew=2.0,
            label="cross-check flag" if j == 0 else None,
        )
    ax0.set_aspect("equal")
    ax0.set_xlabel("x [m]")
    ax0.set_ylabel("y [m]")
    ax0.set_title("(a) traced edges over the orthophoto")
    ax0.legend(loc="upper right", fontsize=8)

    # (b) width vs arc length ----------------------------------------------------------------
    s = stations.s_m
    ax1.plot(s, result.width_total_m, color=INK, lw=1.8, label="total (traced)")
    ax1.plot(s, result.width_left_m, color=BLUE, lw=1.2, label="left")
    ax1.plot(s, result.width_right_m, color=AQUA, lw=1.2, label="right")
    if crosscheck is not None:
        s_ref, w_ref = crosscheck
        ax1.plot(s_ref, w_ref, color=GREEN, lw=1.2, ls="--", label="cross-check ref")
        ax1.fill_between(
            s_ref,
            w_ref - band_m,
            w_ref + band_m,
            color=GREEN,
            alpha=0.12,
            lw=0.0,
            label=f"±{band_m:.1f} m band",
        )
    for i, cp in enumerate(control_points):
        ax1.axvline(
            cp.s_m,
            color=VIOLET,
            lw=0.9,
            alpha=0.6,
            label="control point" if i == 0 else None,
        )
    flagged = result.provenance.flagged_stations_m()
    if flagged:
        w_at = np.interp(flagged, s, result.width_total_m)
        ax1.plot(flagged, w_at, "x", color=RED, ms=9.0, mew=2.0, label="flagged")
    ax1.set_xlabel("s [m]")
    ax1.set_ylabel("width [m]")
    ax1.set_title("(b) corridor width vs arc length")
    ax1.legend(loc="best", fontsize=8, ncols=2)

    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        prog="tools/plot_track_width_qa.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--synthetic",
        action="store_true",
        help="render the built-in synthetic fixture (no data needed)",
    )
    parser.add_argument("--track", type=Path, help="track dir with centerline.csv")
    parser.add_argument(
        "--ortho", type=Path, help="orthophoto .npz bundle (image+transform)"
    )
    parser.add_argument(
        "--control-points",
        type=Path,
        help="hand-QA control-point CSV (s_m,side,offset_m)",
    )
    parser.add_argument(
        "--out", type=Path, default=_DEFAULT_OUT, help="output PNG path"
    )
    parser.add_argument("--search-min", type=float, help="search band inner edge [m]")
    parser.add_argument("--search-max", type=float, help="search band outer edge [m]")
    parser.add_argument("--band", type=float, help="cross-check agreement band [m]")
    args = parser.parse_args(argv)

    defaults = wt.TraceParams()
    params = wt.TraceParams(
        search_min_m=args.search_min
        if args.search_min is not None
        else defaults.search_min_m,
        search_max_m=args.search_max
        if args.search_max is not None
        else defaults.search_max_m,
        crosscheck_band_m=args.band
        if args.band is not None
        else defaults.crosscheck_band_m,
    )

    crosscheck: tuple[np.ndarray, np.ndarray] | None = None
    if args.synthetic:
        stations, image, cps, crosscheck = _synthetic_fixture()
        params = wt.TraceParams(
            search_min_m=2.0,
            search_max_m=9.0,
            crosscheck_band_m=params.crosscheck_band_m,
        )
        title = "width-trace QA — synthetic fixture (12 m corridor, CP-rescued gap, pit lane)"
    else:
        if args.track is None or args.ortho is None:
            parser.error("either --synthetic or both --track and --ortho are required")
        stations = _load_track_stations(args.track)
        image = _load_ortho(args.ortho)
        cps = wt.load_control_points(args.control_points) if args.control_points else []
        title = f"width-trace QA — {args.track.name}"

    try:
        result = wt.trace_widths(
            stations,
            image,
            params=params,
            control_points=cps,
            lidar_width_m=crosscheck,
        )
    except wt.UnresolvedStationsError as exc:
        # R1 surfaces here too: name the stations, write nothing.
        print(f"unresolved stations — no overlay written:\n{exc}", file=sys.stderr)
        return 1

    out = render(
        stations,
        image,
        result,
        cps,
        crosscheck,
        params.crosscheck_band_m,
        args.out,
        title,
    )
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
