# SPDX-License-Identifier: AGPL-3.0-only
"""Render the Limebeer validation figure (docs/validation/img/limebeer_catalunya.png).

Drives the figure from the **real** model: it runs the committed Rust example
``crates/outlap-qss/examples/limebeer_lap.rs`` (the ``limebeer_2014_f1`` reference car on the
flat-track Catalunya min-curvature lap, production-resolution envelope) and overlays the
hand-digitised PL2014 Fig. 8 speed trace (``docs/validation/data/pl2014_fig8_speed.csv``).
Nothing here re-implements the model.

Two panels:
  (a) speed trace overlay, with our lap circularly aligned to the paper's s-origin (the imported
      track's start/finish and direction differ from the paper's).
  (b) sorted apex speeds (alignment-free): slow corners validate the car; the fast-corner deficit
      on this figure is the imported OSM geometry (see docs/validation/limebeer.md — on the
      paper's own geometry the fast apexes agree within 2%).

Run from anywhere:  python python/tools/plot_limebeer.py
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import argrelextrema

ROOT = Path(__file__).resolve().parents[2]
OUT_PNG = ROOT / "docs/validation/img/limebeer_catalunya.png"
ORACLE_CSV = ROOT / "docs/validation/data/pl2014_fig8_speed.csv"
LAP_CSV = ROOT / "python/examples/output/limebeer_catalunya.csv"

ORACLE_LAP_S = 82.43
ORACLE_LEN_M = 4655.0


def run_example() -> None:
    subprocess.run(
        [
            "cargo",
            "run",
            "--release",
            "-q",
            "-p",
            "outlap-qss",
            "--features",
            "parallel",
            "--example",
            "limebeer_lap",
            "--",
            "--out",
            str(LAP_CSV),
        ],
        cwd=ROOT,
        check=True,
    )


def align(s: np.ndarray, v: np.ndarray, so: np.ndarray, vo: np.ndarray) -> np.ndarray:
    """Best circular shift (and direction) of our v(s) onto the oracle trace's s-axis."""
    lap_len = float(s[-1])
    grid = np.linspace(0.0, lap_len, 2048, endpoint=False)
    theirs = np.interp(grid * ORACLE_LEN_M / lap_len, so, vo)
    best: tuple[float, float, bool] = (np.inf, 0.0, False)
    for rev in (False, True):
        vv = v[::-1] if rev else v
        ss = (lap_len - s[::-1]) if rev else s
        ours = np.interp(grid, ss, vv)
        for k in range(0, 2048, 4):
            err = float(np.mean((np.roll(ours, -k) - theirs) ** 2))
            if err < best[0]:
                best = (err, float(grid[k]), rev)
    _, shift, rev = best
    vv = v[::-1] if rev else v
    ss = (lap_len - s[::-1]) if rev else s
    ssh = (ss - shift) % lap_len
    order = np.argsort(ssh)
    return np.column_stack([ssh[order] * ORACLE_LEN_M / lap_len, vv[order]])


def apexes(s: np.ndarray, v: np.ndarray) -> list[float]:
    mins = argrelextrema(v, np.less_equal, order=25)[0]
    keep: list[int] = []
    for i in mins:
        if not keep or s[i] - s[keep[-1]] > 60.0:
            keep.append(int(i))
    return sorted(float(v[i]) for i in keep)


def main() -> None:
    if not LAP_CSV.exists():
        run_example()
    lap = np.genfromtxt(LAP_CSV, delimiter=",", names=True)
    # 8 provenance comment lines + the header row precede the data.
    oracle = np.loadtxt(ORACLE_CSV, delimiter=",", skiprows=9)
    s, v = lap["s_m"], lap["v_mps"]
    so, vo = oracle[:, 0], oracle[:, 1]
    lap_time = float(np.max(lap["t_s"]))

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(11, 8), gridspec_kw={"height_ratios": [3, 2]}
    )
    aligned = align(s, v, so, vo)
    ax1.plot(
        aligned[:, 0],
        aligned[:, 1],
        "tab:blue",
        lw=1.2,
        label=f"outlap QSS t0, flat track ({lap_time:.2f} s)",
    )
    ax1.plot(
        so,
        vo,
        "tab:red",
        marker=".",
        lw=1.0,
        ms=5,
        alpha=0.85,
        label=f"PL2014 Fig. 8, digitised ({ORACLE_LAP_S:.2f} s)",
    )
    ax1.set_xlabel("s [m] (PL2014 axis)")
    ax1.set_ylabel("u [m/s]")
    ax1.set_title(
        "Perantoni & Limebeer 2014 cross-check — Catalunya, flat track (Decision #48)"
    )
    ax1.legend(fontsize=9)
    ax1.grid(alpha=0.3)

    ours = apexes(s, v)
    theirs = sorted([17, 22, 24, 25, 32, 36, 37, 40, 43, 43, 46, 47, 52, 60, 60, 62])
    ax2.plot(
        range(1, len(ours) + 1), ours, "o-", color="tab:blue", label="outlap apexes"
    )
    ax2.plot(
        range(1, len(theirs) + 1),
        theirs,
        "s--",
        color="tab:red",
        label="PL2014 apexes (digitised)",
    )
    ax2.set_xlabel("apex rank (slowest → fastest)")
    ax2.set_ylabel("apex speed [m/s]")
    ax2.legend(fontsize=9)
    ax2.grid(alpha=0.3)
    fig.text(
        0.01,
        0.01,
        "Oracle points hand-digitised from the open-access manuscript (±50 m, ±2 m/s). "
        "Fast-corner deficit on this OSM-imported geometry is a track-data artefact — "
        "see docs/validation/limebeer.md.",
        fontsize=7,
        color="0.35",
        style="italic",
    )
    fig.tight_layout()
    OUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PNG, dpi=140)
    print(f"wrote {OUT_PNG}")


if __name__ == "__main__":
    main()
