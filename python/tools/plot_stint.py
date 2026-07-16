# SPDX-License-Identifier: AGPL-3.0-only
"""Render the M5 PR6 multi-lap **stint** figure (docs/theory/img/stint.png).

Drives every panel from the **real** stint drivers through the public API
(`outlap.core.solve_stint_dataset`) — nothing here re-implements the physics; it only runs the
solvers and plots their datasets. The tyre thermal/wear parameters are the `outlap.wearcal`-calibrated
values (M5 PR7/PR8), so the figure shows the *machinery* — the slow state carrying lap-to-lap — with
physically-realistic degradation: gradual wear and a cliff, not the earlier saturating placeholder.

Six panels:
  (a) QSS pace over a warm-seeded stint — lap time climbs as the tyres degrade (wear + thermal);
  (b) QSS grip multiplier end-of-lap over the same stint — monotone decline drives (a);
  (c) cold-seeded warm-up — the representative-tyre surface temperature over the whole stint's arc
      length (continuous, laps concatenated), climbing out of the 20 °C cold seed toward the window;
  (d) slow-state continuity — the surface temperature at each lap boundary carries with no reset (a
      per-lap *reset* reference, dashed, shows what "start every lap fresh" would look like);
  (e) tread wear accumulating gradually across the stint (monotone; below the cliff onset w_c over
      these few laps, with the calibrated k_w);
  (f) the T2 transient stint — per-lap lap time + per-wheel end-of-lap wear, one continuous run.

Run from anywhere:  python python/tools/plot_stint.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, solve_stint_dataset

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "stint.png"
_CATALUNYA = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_LIMEBEER = str(_ROOT / "data" / "vehicles" / "limebeer_2014_f1")
_COARSE: dict[str, object] = {
    "envelope": {"v_points": 10, "ax_points": 9, "g_normal_points": 3}
}
_FLAT = {"flat_track": True, **_COARSE}

_C_WARM = "tab:red"
_C_COLD = "tab:blue"
_C_T2 = "tab:purple"
_C_GRIP = "tab:green"


def main() -> None:
    trk = Track.load(_CATALUNYA)
    n_laps = 6

    # Warm-seeded QSS stint (default seed at the grip optimum): pure wear + thermal degradation.
    warm = solve_stint_dataset(
        _LIMEBEER, trk, n_laps=n_laps, tier="t1", ds_m=6.0, sim=_FLAT, tire_thermal=True
    )
    # Cold-seeded QSS stint (out-lap onto cold tyres): the warm-up transient.
    cold = solve_stint_dataset(
        _LIMEBEER,
        trk,
        n_laps=n_laps,
        tier="t1",
        ds_m=6.0,
        sim=_FLAT,
        tire_thermal=True,
        initial_tire_temp_c=20.0,
    )
    # Transient (T2) stint: fewer laps (each is a full time integration).
    t2 = solve_stint_dataset(
        _LIMEBEER, trk, n_laps=3, tier="t2", ds_m=6.0, sim=_COARSE, tire_thermal=True
    )

    laps = warm["lap"].values
    s = warm["s"].values
    length_km = s[-1] / 1000.0

    fig, axes = plt.subplots(2, 3, figsize=(16.5, 8.6))

    # (a) Pace loss over the warm-seeded stint.
    ax = axes[0, 0]
    lt = warm["lap_time_s"].values
    ax.plot(laps, lt, "o-", lw=2.4, color=_C_WARM, ms=7)
    ax.set_title("(a) Stint pace loss — warm tyres degrade", fontsize=11)
    ax.set_xlabel("lap")
    ax.set_ylabel("lap time [s]")
    ax.set_xticks(laps)
    ax.annotate(
        f"+{lt[-1] - lt[0]:.2f} s over {n_laps} laps",
        xy=(laps[-1], lt[-1]),
        xytext=(0.35, 0.2),
        textcoords="axes fraction",
        fontsize=9,
        arrowprops={"arrowstyle": "->", "color": "0.4"},
    )

    # (b) Grip multiplier end-of-lap.
    ax = axes[0, 1]
    grip_end = warm["tire_grip"].values[:, -1]
    ax.plot(laps, grip_end, "s-", lw=2.4, color=_C_GRIP, ms=7)
    ax.set_title(r"(b) Grip $\lambda_{\mu,\mathrm{total}}$ end-of-lap", fontsize=11)
    ax.set_xlabel("lap")
    ax.set_ylabel(r"$\lambda_{\mu,\mathrm{total}}$")
    ax.set_xticks(laps)

    # (c) Cold-seed warm-up: surface temperature over the whole stint arc length (continuous).
    ax = axes[0, 2]
    surf_cold = cold["tire_surface_c"].values  # (lap, s)
    dist = np.concatenate([s / 1000.0 + k * length_km for k in range(n_laps)])
    ax.plot(dist, surf_cold.reshape(-1), "-", lw=1.6, color=_C_COLD)
    for k in range(1, n_laps):
        ax.axvline(k * length_km, color="0.6", lw=0.8, ls=":")
    ax.axhline(95.0, color=_C_GRIP, lw=1.2, ls="--", label=r"$T_\mathrm{opt}$ = 95 °C")
    ax.set_title("(c) Cold-seed warm-up (continuous over the stint)", fontsize=11)
    ax.set_xlabel("stint distance [km]")
    ax.set_ylabel(r"surface $T_s$ [°C]")
    ax.legend(loc="lower right", fontsize=8)

    # (d) Continuity vs a per-lap reset reference.
    ax = axes[1, 0]
    surf_warm = warm["tire_surface_c"].values  # (lap, s)
    ax.plot(
        dist,
        surf_warm.reshape(-1),
        "-",
        lw=1.6,
        color=_C_WARM,
        label="carried (stint)",
    )
    # A per-lap reset would restart every lap at lap 1's start value.
    reset = np.tile(surf_warm[0], n_laps)
    ax.plot(dist, reset, "--", lw=1.2, color="0.5", label="if reset each lap")
    for k in range(1, n_laps):
        ax.axvline(k * length_km, color="0.6", lw=0.8, ls=":")
    ax.set_title("(d) Slow-state continuity — no reset", fontsize=11)
    ax.set_xlabel("stint distance [km]")
    ax.set_ylabel(r"surface $T_s$ [°C]")
    ax.legend(loc="upper right", fontsize=8)

    # (e) Wear accumulation across the stint.
    ax = axes[1, 1]
    wear = warm["tire_wear_mm"].values  # (lap, s)
    ax.plot(dist, wear.reshape(-1), "-", lw=1.8, color="tab:brown")
    ax.axhline(2.0, color="tab:orange", lw=1.2, ls="--", label=r"$w_c$ (cliff onset)")
    ax.axhline(8.0, color="0.4", lw=1.0, ls=":", label=r"$w_\mathrm{max}$")
    for k in range(1, n_laps):
        ax.axvline(k * length_km, color="0.6", lw=0.8, ls=":")
    ax.set_title("(e) Tread wear accumulates (monotone)", fontsize=11)
    ax.set_xlabel("stint distance [km]")
    ax.set_ylabel("wear $w$ [mm]")
    ax.legend(loc="lower right", fontsize=8)

    # (f) The T2 transient stint: per-lap lap time + per-wheel end-of-lap wear.
    ax = axes[1, 2]
    t2_laps = t2["lap"].values
    ax.plot(t2_laps, t2["lap_time_s"].values, "o-", lw=2.4, color=_C_T2, ms=7)
    ax.set_title("(f) Transient (T2) stint — one continuous run", fontsize=11)
    ax.set_xlabel("lap")
    ax.set_ylabel("lap time [s]", color=_C_T2)
    ax.tick_params(axis="y", labelcolor=_C_T2)
    ax.set_xticks(t2_laps)
    ax2 = ax.twinx()
    wear_end = t2["tire_wear_mm"].values.mean(axis=1)  # mean over wheels
    ax2.plot(t2_laps, wear_end, "s--", lw=1.8, color="tab:brown", ms=6)
    ax2.set_ylabel("end-of-lap wear [mm]", color="tab:brown")
    ax2.tick_params(axis="y", labelcolor="tab:brown")
    ax2.grid(False)

    fig.suptitle(
        "M5 — multi-lap stint: the tyre-thermal slow state carries lap-to-lap "
        "(limebeer_2014_f1, Catalunya; outlap.wearcal-calibrated tyre params)",
        fontsize=12.5,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")
    print("warm-seed lap times:", np.round(lt, 3))
    print("cold-seed lap times:", np.round(cold["lap_time_s"].values, 3))
    print("T2 lap times:", np.round(t2["lap_time_s"].values, 3))


if __name__ == "__main__":
    main()
