# SPDX-License-Identifier: AGPL-3.0-only
"""Plot sanity for an imported track: map coloured by curvature, elevation profile, banking.

Reads a ``track.yaml`` + ``centerline.csv`` pair and writes three PNGs. Curvature here is computed
independently (numpy gradients on the raw centerline) purely as an import sanity check — the
authoritative κ(s) is the outlap-track Rust spline.

Example:
    python examples/plot_track.py data/tracks/catalunya --out examples/output
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load_centerline(track_dir: Path) -> dict[str, np.ndarray]:
    """Load the centerline columns from ``<track_dir>/centerline.csv``."""
    path = track_dir / "centerline.csv"
    cols: dict[str, list[float]] = {}
    with path.open(encoding="utf-8") as f:
        reader = csv.reader(row for row in f if not row.lstrip().startswith("#"))
        header = next(reader)
        for h in header:
            cols[h.strip()] = []
        for row in reader:
            for h, v in zip(header, row):
                cols[h.strip()].append(float(v))
    return {k: np.asarray(v) for k, v in cols.items()}


def plan_curvature(
    s: np.ndarray, x: np.ndarray, y: np.ndarray, closed: bool
) -> np.ndarray:
    """Signed plan-view curvature κ = (x'y'' − y'x'')/(x'²+y'²)^{3/2} via arc-length gradients."""
    xp = np.gradient(x, s, edge_order=2)
    yp = np.gradient(y, s, edge_order=2)
    if closed:
        # np.gradient has no periodic mode; pad to make the ends consistent.
        xp = np.gradient(
            np.r_[x[-1], x, x[0]],
            np.r_[s[0] - (s[-1] - s[-2]), s, s[-1] + (s[1] - s[0])],
        )[1:-1]
        yp = np.gradient(
            np.r_[y[-1], y, y[0]],
            np.r_[s[0] - (s[-1] - s[-2]), s, s[-1] + (s[1] - s[0])],
        )[1:-1]
    xpp = np.gradient(xp, s, edge_order=2)
    ypp = np.gradient(yp, s, edge_order=2)
    denom = np.power(xp * xp + yp * yp, 1.5)
    denom[denom == 0] = np.nan
    return (xp * ypp - yp * xpp) / denom


def plot(track_dir: Path, out_dir: Path, closed: bool = True) -> list[Path]:
    """Write the three sanity PNGs; return their paths."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    c = load_centerline(track_dir)
    s, x, y, z, bank = c["s_m"], c["x_m"], c["y_m"], c["z_m"], c["banking_deg"]
    kappa = plan_curvature(s, x, y, closed)
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    # 1) Track map coloured by |curvature| (viridis is perceptually uniform + colourblind-safe).
    fig, ax = plt.subplots(figsize=(8, 8))
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    # matplotlib's LineCollection stub under-specifies `segments`; the ndarray is valid at runtime.
    lc = LineCollection(segs, cmap="viridis", linewidth=3)  # pyright: ignore[reportArgumentType]
    lc.set_array(np.abs(kappa[:-1]))
    ax.add_collection(lc)
    ax.autoscale()
    ax.set_aspect("equal")
    ax.set_title(f"{track_dir.name}: plan view, coloured by |κ| (1/m)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.colorbar(lc, ax=ax, label="|curvature| (1/m)", shrink=0.8)
    p = out_dir / f"{track_dir.name}_map.png"
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    written.append(p)

    # 2) Elevation profile z(s).
    fig, ax = plt.subplots(figsize=(10, 3.5))
    ax.plot(s, z, color="#1f77b4")
    ax.fill_between(s, z.min() - 1, z, alpha=0.15, color="#1f77b4")
    ax.set_title(f"{track_dir.name}: elevation profile")
    ax.set_xlabel("s (m)")
    ax.set_ylabel("z (m)")
    ax.grid(True, alpha=0.3)
    p = out_dir / f"{track_dir.name}_elevation.png"
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    written.append(p)

    # 3) Banking(s).
    fig, ax = plt.subplots(figsize=(10, 3.0))
    ax.plot(s, bank, color="#d62728")
    ax.set_title(f"{track_dir.name}: banking")
    ax.set_xlabel("s (m)")
    ax.set_ylabel("banking (deg)")
    ax.grid(True, alpha=0.3)
    p = out_dir / f"{track_dir.name}_banking.png"
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    written.append(p)

    return written


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(prog="plot_track", description=__doc__)
    parser.add_argument(
        "track_dir", type=Path, help="directory with track.yaml + centerline.csv"
    )
    parser.add_argument("--out", type=Path, default=Path("examples/output"))
    parser.add_argument(
        "--open", action="store_true", help="treat as an open (non-closed) track"
    )
    args = parser.parse_args(argv)
    paths = plot(args.track_dir, args.out, closed=not args.open)
    for p in paths:
        print(f"wrote {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
