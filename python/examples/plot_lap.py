# SPDX-License-Identifier: AGPL-3.0-only
"""Plot a T0 lap: speed vs distance, and the track map coloured by speed.

Reads the CSV written by the `catalunya_lap` example (columns
``s_m,x_m,y_m,v_mps,ax_mps2,ay_mps2,t_s``) and writes two PNGs.

Example:
    cargo run -p outlap-qss --example catalunya_lap
    python examples/plot_lap.py python/examples/output/catalunya_t0.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load(path: Path) -> dict[str, np.ndarray]:
    """Load the lap CSV columns into arrays."""
    cols: dict[str, list[float]] = {}
    with path.open(encoding="utf-8") as f:
        reader = csv.reader(row for row in f if not row.lstrip().startswith("#"))
        header = next(reader)
        for h in header:
            cols[h] = []
        for record in reader:
            for h, val in zip(header, record):
                cols[h].append(float(val))
    return {k: np.asarray(v) for k, v in cols.items()}


def plot(csv_path: Path, out_dir: Path) -> list[Path]:
    """Write the speed-profile and speed-coloured-map PNGs."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    d = load(csv_path)
    s, x, y, v = d["s_m"], d["x_m"], d["y_m"], d["v_mps"]
    kmh = v * 3.6
    out_dir.mkdir(parents=True, exist_ok=True)
    name = csv_path.stem
    written: list[Path] = []

    # 1) Speed vs distance.
    fig, ax = plt.subplots(figsize=(11, 3.5))
    ax.plot(s, kmh, color="#1f77b4", lw=1.2)
    ax.fill_between(s, 0, kmh, alpha=0.12, color="#1f77b4")
    ax.set_title(f"{name}: speed profile")
    ax.set_xlabel("distance s (m)")
    ax.set_ylabel("speed (km/h)")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    p = out_dir / f"{name}_speed.png"
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    written.append(p)

    # 2) Track map coloured by speed (viridis: perceptually uniform, colourblind-safe).
    fig, ax = plt.subplots(figsize=(8, 8))
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidth=3)  # pyright: ignore[reportArgumentType]
    lc.set_array(kmh[:-1])
    ax.add_collection(lc)
    ax.autoscale()
    ax.set_aspect("equal")
    ax.set_title(f"{name}: plan view, coloured by speed")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.colorbar(lc, ax=ax, label="speed (km/h)", shrink=0.8)
    p = out_dir / f"{name}_map.png"
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    written.append(p)

    return written


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(prog="plot_lap", description=__doc__)
    parser.add_argument("csv", type=Path, help="lap CSV from the catalunya_lap example")
    parser.add_argument("--out", type=Path, default=Path("examples/output"))
    args = parser.parse_args(argv)
    for p in plot(args.csv, args.out):
        print(f"wrote {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
