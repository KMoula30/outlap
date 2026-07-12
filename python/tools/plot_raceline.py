# SPDX-License-Identifier: AGPL-3.0-only
"""Render the racing-line theory figure (docs/theory/img/raceline_time_weighted.png).

Drives both lines from the **real** generators: ``min_curvature`` and ``time_weighted`` on the
Limebeer 2014 car over Catalunya (flat-track analysis so the comparison is pure line geometry). The
time-weighted loop runs the car's own g-g-g-v speed pre-pass — nothing here re-implements the model.

Two panels:
  (a) plan view: the corridor, the min-curvature line, and the time-weighted line.
  (b) T0 speed profile of each line vs arc length — the time-weighted line carries more speed through
      the slow corners, which is where lap time is won.

Run from anywhere:  python python/tools/plot_raceline.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, min_curvature, solve_lap_dataset, time_weighted

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "raceline_time_weighted.png"
_TRACK = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_CAR = str(_ROOT / "data" / "vehicles" / "limebeer_2014_f1")
_HALF_WIDTH_M = 1.1
_SIM: dict[str, object] = {"flat_track": True}


def _line_xy(track: Track) -> tuple[np.ndarray, np.ndarray]:
    s = track.sample(ds_m=3.0)
    return s["x"], s["y"]


def main() -> None:
    track = Track.load(_TRACK)
    mc = min_curvature(track, _HALF_WIDTH_M)
    tw = time_weighted(_CAR, track, _HALF_WIDTH_M, iterations=4, sim=_SIM)

    lap_mc = solve_lap_dataset(_CAR, mc, tier="t0", sim=_SIM)
    lap_tw = solve_lap_dataset(_CAR, tw, tier="t0", sim=_SIM)
    t_mc = float(lap_mc.attrs["lap_time_s"])
    t_tw = float(lap_tw.attrs["lap_time_s"])

    cx, cy = _line_xy(track)
    mx, my = _line_xy(mc.line())
    wx, wy = _line_xy(tw.line())

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13, 5.4))

    ax0.plot(cx, cy, color="0.6", lw=0.8, ls="--", label="centerline")
    ax0.plot(mx, my, color="#1f77b4", lw=1.6, label=f"min-curvature  ({t_mc:.2f} s)")
    ax0.plot(
        wx,
        wy,
        color="#d62728",
        lw=1.6,
        label=f"time-weighted ×{tw.iterations}  ({t_tw:.2f} s)",
    )
    ax0.set_aspect("equal")
    ax0.set_xlabel("x [m]")
    ax0.set_ylabel("y [m]")
    ax0.set_title("(a) Catalunya — line comparison (Limebeer, flat)")
    ax0.legend(loc="best", fontsize=9)

    ax1.plot(
        lap_mc.s.to_numpy(),
        lap_mc.v.to_numpy(),
        color="#1f77b4",
        lw=1.3,
        label="min-curvature",
    )
    ax1.plot(
        lap_tw.s.to_numpy(),
        lap_tw.v.to_numpy(),
        color="#d62728",
        lw=1.3,
        label="time-weighted",
    )
    ax1.set_xlabel("arc length s [m]")
    ax1.set_ylabel("speed v [m/s]")
    gain = t_mc - t_tw
    ax1.set_title(f"(b) T0 speed profile — time-weighted saves {gain:.2f} s")
    ax1.legend(loc="best", fontsize=9)

    fig.tight_layout()
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}  (min-curv {t_mc:.3f} s, time-weighted {t_tw:.3f} s, {tw.iterations} iters)")


if __name__ == "__main__":
    main()
