# SPDX-License-Identifier: AGPL-3.0-only
"""M5 PR8/PR10 compound-crossover figure (docs/validation/img/compound_crossover.png).

Runs a real T0 stint on the F1 2026 car for each of the soft/medium/hard compound presets
(`data/tires/f1_2026_compounds/`) and shows the strategy trade-off: the soft is quickest fresh but
degrades fastest, the hard is slow to switch on but holds pace, and the cumulative-time curves cross
— which compound wins depends on the stint length (the undercut/overcut tease, dry only).

Run from anywhere:  python python/tools/plot_compound_crossover.py
"""

from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, solve_stint_dataset

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "validation" / "img" / "compound_crossover.png"
_CATALUNYA = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_F1 = _ROOT / "data" / "vehicles" / "f1_2026"
_COMPOUNDS = _ROOT / "data" / "tires" / "f1_2026_compounds"
_FAST: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 7, "ax_points": 6, "g_normal_points": 2},
}
_COLS = {"soft": "tab:red", "medium": "tab:green", "hard": "tab:blue"}


def _run(compound: str, n: int) -> np.ndarray:
    tmp = Path(tempfile.mkdtemp(prefix="cross_"))
    try:
        veh = tmp / "f1_2026"
        shutil.copytree(_F1, veh)
        shutil.copy(_COMPOUNDS / f"{compound}.tyr.yaml", veh / "tyr" / "slick.tyr.yaml")
        ds = solve_stint_dataset(
            str(veh), Track.load(_CATALUNYA), n_laps=n, tier="t0", ds_m=16.0, sim=_FAST,
            tire_thermal=True, initial_tire_temp_c=None,
        )
        return np.asarray(ds["lap_time_s"].values, dtype=np.float64)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main() -> None:
    n = 18
    runs = {c: _run(c, n) for c in ("soft", "medium", "hard")}
    laps = np.arange(1, n + 1)
    cum = {k: np.cumsum(v) for k, v in runs.items()}
    best = [min(runs, key=lambda k: cum[k][length - 1]) for length in laps]

    fig, ax = plt.subplots(1, 2, figsize=(13.0, 4.8))
    for k, v in runs.items():
        ax[0].plot(laps, v, "o-", color=_COLS[k], lw=2.0, ms=5, label=k)
    ax[0].set_title("(a) Per-lap pace — soft fastest fresh, then cliffs", fontsize=11)
    ax[0].set_xlabel("lap")
    ax[0].set_ylabel("lap time [s]")
    ax[0].legend(loc="upper left", fontsize=9)

    for k, v in cum.items():
        ax[1].plot(laps, v - cum["medium"], "o-", color=_COLS[k], lw=2.0, ms=5, label=k)
    ax[1].axhline(0, color="0.4", lw=1.0)
    for i, length in enumerate(laps):
        ax[1].axvspan(length - 0.5, length + 0.5, ymax=0.06, color=_COLS[best[i]], alpha=0.6, lw=0)
    ax[1].set_title("(b) Cumulative time vs medium — winner by stint length", fontsize=11)
    ax[1].set_xlabel("stint length [laps]")
    ax[1].set_ylabel("Σ time − Σ medium [s]")
    ax[1].legend(loc="upper left", fontsize=9)

    fig.suptitle(
        "M5 — soft/medium/hard compound crossover (F1 2026, Catalunya): the strategy trade-off",
        fontsize=12.5,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")
    print("fresh:", {k: round(float(v[0]), 2) for k, v in runs.items()})
    print("optimal by length:", "".join({"soft": "S", "medium": "M", "hard": "H"}[b] for b in best))


if __name__ == "__main__":
    main()
