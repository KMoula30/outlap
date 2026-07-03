# SPDX-License-Identifier: AGPL-3.0-only
"""Overlay the centerline lap and the min-curvature line lap for comparison.

Reads the two CSVs written by the `catalunya_line` example and writes one PNG showing both racing
lines on the track (left) and their speed profiles (right).

Example:
    cargo run -p outlap-raceline --example catalunya_line
    python examples/plot_line_compare.py --dir examples/output
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load(path: Path) -> dict[str, np.ndarray]:
    """Load `s_m,x_m,y_m,v_mps` columns."""
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


def plot(out_dir: Path) -> Path:
    """Write the comparison PNG."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    center = load(out_dir / "catalunya_centerline.csv")
    line = load(out_dir / "catalunya_raceline.csv")

    fig, (axmap, axv) = plt.subplots(1, 2, figsize=(15, 7.5))

    # Left: both lines on the track.
    axmap.plot(
        center["x_m"], center["y_m"], color="#888888", lw=1.2, label="centerline"
    )
    axmap.plot(
        line["x_m"], line["y_m"], color="#d62728", lw=1.4, label="min-curvature line"
    )
    axmap.set_aspect("equal")
    axmap.set_title("Racing line")
    axmap.set_xlabel("x (m)")
    axmap.set_ylabel("y (m)")
    axmap.legend(loc="upper right")

    # Right: speed profiles vs distance.
    axv.plot(
        center["s_m"],
        center["v_mps"] * 3.6,
        color="#888888",
        lw=1.0,
        label="centerline",
    )
    axv.plot(
        line["s_m"], line["v_mps"] * 3.6, color="#d62728", lw=1.0, label="min-curvature"
    )
    axv.set_title("Speed profile")
    axv.set_xlabel("distance s (m)")
    axv.set_ylabel("speed (km/h)")
    axv.set_ylim(bottom=0)
    axv.grid(True, alpha=0.3)
    axv.legend(loc="upper right")

    out_dir.mkdir(parents=True, exist_ok=True)
    p = out_dir / "catalunya_line_compare.png"
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return p


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(prog="plot_line_compare", description=__doc__)
    parser.add_argument("--dir", type=Path, default=Path("examples/output"))
    args = parser.parse_args(argv)
    print(f"wrote {plot(args.dir)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
