# SPDX-License-Identifier: AGPL-3.0-only
"""Render the T2 3-D driver-stability figures (PR7.5) for the theory page / PR from the Python API.

The three figures show that the transient tier now laps the real elevated `catalunya_osm` in the full
3-D road frame at flat pace, where a rigid `κ_v·v²` normal-load coupling previously spun the car:

* `t2_3d_stability_trajectory.png` — before/after birds-eye laps coloured by speed;
* `t2_3d_stability_mechanism.png`  — the road-normal load factor and yaw rate over time;
* `t2_3d_pace_parity.png`          — flat vs 3-D lap time for the three reference cars.

Data capture (all from the shipped Python API, `speed_margin = 0.85`, coarse envelope):

1. `after`  — build the wheel as shipped and run each car's 3-D lap.
2. `flat`   — the same cars with `sim={"flat_track": True}` (the 2-D baseline).
3. `before` — the *diverging* trace. Reproduce it by temporarily disabling the crest-unloading floor:
   set `CREST_UNLOADING_FLOOR_G` to a large value in `crates/outlap-transient/src/lap.rs`, rebuild
   (`maturin develop --release --manifest-path crates/outlap-py/Cargo.toml`), capture, then restore
   `0.15` and rebuild. Only the `limebeer_2014_f1` before-trace is needed (the hardest car).

Run (after `maturin develop`):
    uv run --group notebooks python docs/derivations/plot_t2_3d_stability.py

Traces are cached as .npz under `debug_plots/t2_3d/` so the plotting is re-runnable without re-solving.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

from outlap.core import Track, solve_transient_lap  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "data"
CACHE = ROOT / "debug_plots" / "t2_3d"
IMG = ROOT / "docs" / "theory" / "img"
CACHE.mkdir(parents=True, exist_ok=True)
IMG.mkdir(parents=True, exist_ok=True)

CARS = ["limebeer_2014_f1", "f1_2026", "tesla_model3_rwd"]
LABELS = ["Limebeer 2014 F1", "F1 2026", "Model 3 RWD"]
COARSE = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}
MARGIN = 0.85
C_OK, C_BAD, C_REF, C_ACC, C_FLAT = (
    "#1b7837",
    "#b2182b",
    "#4d4d4d",
    "#2166ac",
    "#92c5de",
)

plt.rcParams.update(
    {
        "font.size": 10,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.axisbelow": True,
        "figure.dpi": 130,
    }
)


def _solve(veh: str, *, flat: bool) -> dict[str, np.ndarray]:
    """Solve one 3-D (or flat) transient lap and return the trace arrays."""
    track = Track.load(str(DATA / "tracks/catalunya_osm"))
    lap = solve_transient_lap(
        str(DATA / f"vehicles/{veh}"),
        track,
        sim={**COARSE, "flat_track": flat},
        speed_margin=MARGIN,
    )
    return {
        "t": np.array(lap.t()),
        "s": np.array(lap.s()),
        "x": np.array(lap.x()),
        "y": np.array(lap.y()),
        "vx": np.array(lap.vx()),
        "vy": np.array(lap.vy()),
        "yaw": np.array(lap.yaw_rate()),
        "lap_time": np.array([lap.lap_time_s]),
    }


def _cached(tag: str, veh: str, *, flat: bool) -> dict[str, np.ndarray]:
    path = CACHE / f"{tag}_{veh}.npz"
    if path.exists():
        return dict(np.load(path))
    trace = _solve(veh, flat=flat)
    np.savez(path, **trace)
    return trace


def _lt(trace: dict[str, np.ndarray]) -> float:
    """Lap time as a scalar (traces cache it as a 0-d or 1-d array)."""
    return float(np.ravel(trace["lap_time"])[0])


def _geom() -> dict[str, np.ndarray]:
    track = Track.load(str(DATA / "tracks/catalunya_osm"))
    sm = track.sample(1.0)
    return {
        "s": np.array(sm["s"]),
        "grade": np.array(sm["grade"]),
        "kappa_v": np.array(sm["kappa_v"]),
        "length": np.array([track.length()]),
    }


def _speed_track(ax, x, y, c, vmin, vmax, title):
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", norm=plt.Normalize(vmin, vmax))
    lc.set_array(c[:-1])
    lc.set_linewidth(2.4)
    ax.add_collection(lc)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=11)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.autoscale()
    return lc


def trajectory_figure(before: dict, after: dict) -> None:
    onset = np.where(np.abs(before["yaw"]) > 5)[0]
    onset = int(onset[0]) if len(onset) else len(before["yaw"])
    fig, axs = plt.subplots(1, 2, figsize=(11, 5.2))
    bv = np.hypot(before["vx"], before["vy"])[:onset]
    _speed_track(
        axs[0],
        before["x"][:onset],
        before["y"][:onset],
        bv,
        10,
        80,
        f"Before — 3-D lap SPINS\n(rigid $\\kappa_v$ load; diverges at t={before['t'][onset]:.0f}s)",
    )
    axs[0].plot(
        before["x"][onset],
        before["y"][onset],
        marker="X",
        ms=15,
        color=C_BAD,
        mec="white",
        mew=1.5,
        zorder=5,
        label="spin / divergence",
    )
    axs[0].legend(loc="upper right", fontsize=9)
    av = np.hypot(after["vx"], after["vy"])
    lc = _speed_track(
        axs[1],
        after["x"],
        after["y"],
        av,
        10,
        80,
        f"After — full 3-D lap completes\n(crest-unloading floor; {_lt(after):.1f}s)",
    )
    cb = fig.colorbar(lc, ax=axs, shrink=0.8, pad=0.02)
    cb.set_label("speed (m/s)")
    fig.suptitle(
        "T2 transient lap · catalunya_osm (real DEM elevation) · limebeer_2014_f1 · speed_margin 0.85",
        fontsize=11,
        y=1.02,
    )
    fig.savefig(IMG / "t2_3d_stability_trajectory.png", bbox_inches="tight")
    plt.close(fig)


def mechanism_figure(before: dict, after: dict, geom: dict) -> None:
    length = float(geom["length"][0])
    onset = np.where(np.abs(before["yaw"]) > 5)[0]
    onset = int(onset[0]) if len(onset) else len(before["yaw"])

    def load_factor(d, end):
        s, vx = d["s"][:end], d["vx"][:end]
        kv = np.interp(np.mod(s, length), geom["s"], geom["kappa_v"])
        gr = np.interp(np.mod(s, length), geom["s"], geom["grade"])
        raw = (9.80665 * np.cos(gr) + kv * vx**2) / 9.80665
        floored = (
            9.80665 * np.cos(gr) + np.maximum(kv * vx**2, -0.15 * 9.80665)
        ) / 9.80665
        return d["t"][:end], raw, floored

    tb, rawb, _ = load_factor(before, onset)
    ta, _, flooa = load_factor(after, len(after["t"]))
    fig, axs = plt.subplots(2, 1, figsize=(10, 6.4), sharex=True)
    axs[0].axhline(1.0, color=C_REF, lw=0.8, ls=":")
    axs[0].axhline(0.0, color=C_BAD, lw=0.9, ls="--", label="zero load (flight)")
    axs[0].axhline(
        0.85,
        color=C_OK,
        lw=0.9,
        ls=":",
        alpha=0.8,
        label="crest-unloading floor (0.85 g)",
    )
    axs[0].plot(
        tb, rawb, color=C_BAD, lw=1.0, alpha=0.85, label="before: raw $g_\\mathrm{n}/g$"
    )
    axs[0].plot(ta, flooa, color=C_OK, lw=1.0, label="after: floored $g_\\mathrm{n}/g$")
    axs[0].set_ylabel("road-normal load factor $g_\\mathrm{n}/g$")
    axs[0].set_ylim(-1.2, 2.6)
    axs[0].legend(loc="lower left", fontsize=8, ncol=2)
    axs[0].set_title(
        "Vertical-curvature load coupling: the rigid $\\kappa_v v^2$ over-unloads the tyres over "
        "crests; the floor only bites at those crests",
        fontsize=10,
    )
    axs[1].plot(
        before["t"][: onset + 200],
        before["yaw"][: onset + 200],
        color=C_BAD,
        lw=1.0,
        label="before: yaw rate → spin",
    )
    axs[1].plot(
        after["t"],
        after["yaw"],
        color=C_OK,
        lw=1.0,
        label="after: yaw rate stays bounded",
    )
    axs[1].set_ylabel("yaw rate (rad/s)")
    axs[1].set_xlabel("time (s)")
    axs[1].set_ylim(-6, 6)
    axs[1].legend(loc="lower left", fontsize=9)
    fig.savefig(IMG / "t2_3d_stability_mechanism.png", bbox_inches="tight")
    plt.close(fig)


def pace_figure(flat: list[float], td: list[float]) -> None:
    x = np.arange(len(CARS))
    w = 0.36
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    ax.bar(x - w / 2, flat, w, color=C_FLAT, label="flat-plane (2-D)")
    ax.bar(x + w / 2, td, w, color=C_ACC, label="full 3-D (grade+banking+elevation)")
    for i in range(len(CARS)):
        ax.text(x[i] - w / 2, flat[i] + 1.5, f"{flat[i]:.1f}", ha="center", fontsize=8)
        ax.text(x[i] + w / 2, td[i] + 1.5, f"{td[i]:.1f}", ha="center", fontsize=8)
        delta = (td[i] - flat[i]) / flat[i] * 100
        ax.text(
            x[i],
            max(flat[i], td[i]) + 9,
            f"{delta:+.2f}%",
            ha="center",
            fontsize=9,
            fontweight="bold",
            color=C_REF,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(LABELS)
    ax.set_ylabel("lap time (s)")
    ax.set_ylim(0, 212)
    ax.set_title(
        "3-D catalunya_osm now completes at flat pace for every reference car\n"
        "(the 3-D driver-stability envelope matches the flat one; before, the 3-D lap spun)",
        fontsize=10,
    )
    ax.legend(loc="upper left", fontsize=9)
    fig.savefig(IMG / "t2_3d_pace_parity.png", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    geom = _geom()
    after = {v: _cached("after", v, flat=False) for v in CARS}
    flat = {v: _cached("flat", v, flat=True) for v in CARS}
    # The `before` (diverging) trace needs the crest-unloading floor disabled — see the module
    # docstring. If it is not cached, skip the before/after comparison and note it.
    before_path = CACHE / "before_limebeer_2014_f1.npz"
    if before_path.exists():
        before = dict(np.load(before_path))
        trajectory_figure(before, after["limebeer_2014_f1"])
        mechanism_figure(before, after["limebeer_2014_f1"], geom)
    else:
        print(
            f"note: {before_path} not found — see the docstring to capture the diverging trace"
        )
    pace_figure(
        [_lt(flat[v]) for v in CARS],
        [_lt(after[v]) for v in CARS],
    )
    print("wrote t2_3d_* figures to", IMG)


if __name__ == "__main__":
    main()
