# SPDX-License-Identifier: AGPL-3.0-only
"""Render the Tesla Model 3 RWD (HV variant) figures (docs/vehicles/model3/img/*.png).

Drives every figure from the **real** model through the public Python surface —
``vehicle_report``, ``solve_lap_dataset``, and the returnable ``lap.envelope`` — on the
committed synthetic powertrain. Nothing here re-implements physics.

* ``model3_report.png``   — the loaded-model report: warning-clean, estimates surfaced.
* ``model3_lap.png``      — T1 lap on the Nürburgring GP: speed, SoC, winding temperature.
* ``model3_ggv.png``      — the g-g-g-v envelope: g-g sections vs speed + longitudinal limits.
* ``model3_sizing.png``   — the three synthetic DU sizings: speed traces + lap-time bars.
* ``model3_capstone.png`` — F1 (Catalunya 3D) vs Model 3 (Nürburgring GP): g-g + speed traces.

Solver pin: the notebook 07 CI-speed envelope grid (8×7×2), so the figures reproduce the
capstone's numbers exactly.

Run from anywhere:  python python/tools/plot_model3.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import (
    Track,
    min_curvature,
    solve_lap,
    solve_lap_dataset,
    vehicle_report,
)

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "vehicles" / "model3" / "img"

MODEL3 = str(_ROOT / "data" / "vehicles" / "tesla_model3_rwd")
F1 = str(_ROOT / "data" / "vehicles" / "f1_2026")
FAST_SIM = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}
GN = 9.81

plt.style.use("seaborn-v0_8-darkgrid")
BLUE, AQUA, RED, INK2 = "#2a78d6", "#1baf7a", "#e34948", "#52514e"
LOADS = ["#86b6ef", "#2a78d6", "#0d366b"]


def _save(fig: plt.Figure, name: str) -> None:
    _OUT.mkdir(parents=True, exist_ok=True)
    path = _OUT / name
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {path}")


def plot_report() -> None:
    rep = vehicle_report(MODEL3)
    lines = [
        f"{rep['name']}",
        f"resolved {rep['resolved_hash'][:16]}…",
        "",
        f"warnings: {len(rep['warnings'])}    degraded: {len(rep['degraded'])}"
        "    (loads warning-clean)",
        f"estimated ({len(rep['estimated'])} entries — noted, not warned):",
    ]
    lines += [f"  {ptr:.<44s} {detail}" for ptr, detail in rep["estimated"]]
    fig, ax = plt.subplots(figsize=(10.6, 0.22 * len(lines) + 0.7))
    ax.axis("off")
    ax.text(
        0.01,
        0.98,
        "\n".join(lines),
        family="monospace",
        fontsize=9.5,
        va="top",
        transform=ax.transAxes,
    )
    ax.set_title(
        "loaded-model report — every estimate visible (Decision #41)", loc="left"
    )
    _save(fig, "model3_report.png")


def main() -> None:
    plot_report()

    ring = Track.load(str(_ROOT / "data" / "tracks" / "nuerburgring"))
    rl_ring = min_curvature(ring, half_width_m=1.5)
    cat = Track.load(str(_ROOT / "data" / "tracks" / "catalunya_osm"))
    rl_cat = min_curvature(cat, half_width_m=1.1)

    laps = {
        size: solve_lap_dataset(
            MODEL3,
            rl_ring,
            tier="t1",
            sim=FAST_SIM,
            overrides={"drivetrain.units.0.source": f"ptm/du_{size}.ptm.yaml"},
        )
        for size in ("small", "medium", "large")
    }
    m3 = laps["medium"]
    f1 = solve_lap_dataset(F1, rl_cat, tier="t1", sim=FAST_SIM)

    # --- lap trace: speed, SoC, winding -----------------------------------------------------
    s_km = m3.s.to_numpy() / 1e3
    fig, axs = plt.subplots(3, 1, figsize=(9.6, 7.0), sharex=True)
    axs[0].plot(s_km, m3.v.to_numpy() * 3.6, color=BLUE)
    axs[0].set_ylabel("speed (km/h)")
    axs[0].set_title(
        f"Model 3 RWD (HV variant), medium DU — Nürburgring GP, T1: "
        f"{m3.attrs['lap_time_s']:.2f} s"
    )
    axs[1].plot(s_km, m3.state_of_charge.to_numpy() * 100, color=AQUA)
    axs[1].set_ylabel("SoC (%)")
    axs[2].plot(s_km, m3.machine_temp_c.to_numpy(), color=RED)
    axs[2].axhline(150.0, color=INK2, lw=1.0, ls="--", label="t_warn (derate onset)")
    axs[2].set_xlabel("s (km)")
    axs[2].set_ylabel("winding (°C)")
    axs[2].legend()
    _save(fig, "model3_lap.png")

    # --- envelope ---------------------------------------------------------------------------
    lap_obj = solve_lap(
        MODEL3, rl_ring.line(), raceline_ds_m=rl_ring.ds_m, tier="t1", sim=FAST_SIM
    )
    env = lap_obj.envelope
    (v_lo, v_hi), _, _ = env.domain()
    fig, (a, b) = plt.subplots(1, 2, figsize=(10.2, 4.4))
    for v, c in zip(
        [15.0, 30.0, 45.0, 60.0],
        ["#86b6ef", "#5598e7", "#2a78d6", "#0d366b"],
        strict=True,
    ):
        acc, brk = env.accel_limit(v, GN), env.brake_limit(v, GN)
        ax_grid = np.linspace(-brk, acc, 61)
        ay = np.array([env.ay_boundary(v, a, GN) for a in ax_grid])
        a.plot(
            np.r_[ay, -ay[::-1]] / GN,
            np.r_[ax_grid, ax_grid[::-1]] / GN,
            color=c,
            label=f"{v:.0f} m/s",
        )
    a.set_xlabel("a_y (g)")
    a.set_ylabel("a_x (g)")
    a.set_title("Model 3 g-g sections vs speed (flat road)")
    a.set_aspect("equal")
    a.legend()
    vv = np.linspace(v_lo, v_hi, 80)
    b.plot(
        vv * 3.6,
        [env.accel_limit(v, GN) / GN for v in vv],
        color=AQUA,
        label="drive limit",
    )
    b.plot(
        vv * 3.6,
        [-env.brake_limit(v, GN) / GN for v in vv],
        color=RED,
        label="brake limit",
    )
    b.plot(
        vv * 3.6,
        [-env.drag_accel(v) / GN for v in vv],
        color=INK2,
        ls="--",
        lw=1.2,
        label="drag alone",
    )
    b.set_xlabel("speed (km/h)")
    b.set_ylabel("a_x (g)")
    b.set_title("longitudinal limits vs speed")
    b.legend()
    _save(fig, "model3_ggv.png")

    # --- sizing sensitivity -----------------------------------------------------------------
    fig, (a, b) = plt.subplots(
        1, 2, figsize=(10.6, 4.2), gridspec_kw={"width_ratios": [2, 1]}
    )
    for (size, ds), c in zip(laps.items(), LOADS, strict=True):
        a.plot(
            ds.s.to_numpy() / 1e3,
            ds.v.to_numpy() * 3.6,
            color=c,
            lw=1.5,
            label=f"{size} — {ds.attrs['lap_time_s']:.1f} s",
        )
    a.set_xlabel("s (km)")
    a.set_ylabel("speed (km/h)")
    a.set_title("three synthetic DU sizings — Nürburgring GP")
    a.legend()
    times = [laps[s].attrs["lap_time_s"] for s in laps]
    tmaxs = [float(laps[s].machine_temp_c.max()) for s in laps]
    bars = b.bar(list(laps), times, color=LOADS)
    for r, t, tm in zip(bars, times, tmaxs, strict=True):
        b.text(
            r.get_x() + r.get_width() / 2,
            t + 0.5,
            f"{t:.1f} s\n{tm:.0f} °C",
            ha="center",
            fontsize=9,
            color=INK2,
        )
    b.set_ylim(min(times) - 8, max(times) + 9)
    b.set_ylabel("lap time (s)")
    b.set_title("lap time + peak winding")
    _save(fig, "model3_sizing.png")

    # --- capstone: F1 vs Model 3 ------------------------------------------------------------
    fig, (a, b) = plt.subplots(
        1, 2, figsize=(10.6, 4.4), gridspec_kw={"width_ratios": [1, 1.4]}
    )
    a.scatter(
        f1.ay.to_numpy() / GN,
        f1.ax.to_numpy() / GN,
        s=6,
        alpha=0.35,
        color=BLUE,
        label="F1 (Catalunya 3D)",
    )
    a.scatter(
        m3.ay.to_numpy() / GN,
        m3.ax.to_numpy() / GN,
        s=6,
        alpha=0.35,
        color=RED,
        label="Model 3 (Nürburgring GP)",
    )
    a.set_xlabel("a_y (g)")
    a.set_ylabel("a_x (g)")
    a.set_title("where each car lives")
    a.set_aspect("equal")
    a.legend(loc="lower right", markerscale=2.5)
    for ds, c, name in ((f1, BLUE, "F1"), (m3, RED, "Model 3")):
        s = ds.s.to_numpy()
        b.plot(
            s / s[-1] * 100,
            ds.v.to_numpy() * 3.6,
            color=c,
            lw=1.5,
            label=f"{name} — {ds.attrs['lap_time_s']:.1f} s",
        )
    b.set_xlabel("lap distance (%)")
    b.set_ylabel("speed (km/h)")
    b.set_title("speed traces, distance-normalised")
    b.legend()
    _save(fig, "model3_capstone.png")


if __name__ == "__main__":
    main()
