# SPDX-License-Identifier: AGPL-3.0-only
"""Render the M6 PR5 fuel + u(s)-consumption theory figures, all from the real solver.

Writes (into ``docs/theory/img/``):
  * ``fuel_mass_isolated.png`` — the honest fuel-mass effect in ISOLATION: two ``f1_2026`` QSS laps,
    one at the full tank and one with the fuel emptied (``initial_kg`` overridden to 0), plus the
    per-lap on-board fuel mass draining over a stint. (The raw stint lap-time is ERS-SoC-dominated,
    so fuel is shown as the lighter-vs-heavier Δ, never as a "stint speeds up" claim — D-M6-4.)
  * ``lift_coast.png``        — a T2 lap with a ``u(s)`` ``lift_point`` schedule capping the driver's
    tracked reference into a zone: the speed lifts early and the ERS banks the freed energy (§8.3).

Run from anywhere:  python python/tools/plot_fuel_us.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt

from outlap.core import (
    Track,
    solve_lap_dataset,
    solve_stint_dataset,
    solve_transient_lap,
    transient_lap_dataset,
)

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_IMG = _ROOT / "docs" / "theory" / "img"
_TRACK = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_F1 = str(_ROOT / "data" / "vehicles" / "f1_2026")
# A coarse envelope keeps the figures fast; the mass/gear/lift physics is fidelity-robust.
_SIM: dict[str, object] = {
    "envelope": {"v_points": 12, "ax_points": 10, "g_normal_points": 2}
}
_DS = 10.0

FULL = "#b5651d"  # amber — full tank
EMPTY = "#2f6fb0"  # blue  — empty tank
MAP0 = "#8a8f98"  # grey  — default map
LIFT = "#3a9a54"  # green — lift-and-coast


def _fuel_block(initial_kg: float) -> dict[str, object]:
    # A full fuel block (the vehicle override replaces the block, so every field is restated) with
    # NO flow limit, so the only difference between the heavy and light laps is the initial load —
    # a clean mass isolation (the flat flow cap is dropped from both so it cannot confound the Δ).
    return {
        "initial_kg": initial_kg,
        "tank_kg": 110.0,
        "cg_offset_m": [-0.10, 0.05],
        "lhv_j_per_kg": 43.0e6,
    }


def _fig_fuel_isolated() -> None:
    track = Track.load(_TRACK)
    full = solve_lap_dataset(
        _F1, track, ds_m=_DS, tier="t0", sim=_SIM, overrides={"fuel": _fuel_block(80.0)}
    )
    empty = solve_lap_dataset(
        _F1, track, ds_m=_DS, tier="t0", sim=_SIM, overrides={"fuel": _fuel_block(0.0)}
    )
    t_full = float(full.attrs["lap_time_s"])
    t_empty = float(empty.attrs["lap_time_s"])
    # A 5-lap stint to show the tank draining (the on-board fuel mass, monotone non-increasing).
    stint = solve_stint_dataset(_F1, track, n_laps=5, ds_m=_DS, tier="t0", sim=_SIM)

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.3))
    for lab, d, col in (
        (f"full tank (848 kg) — {t_full:.2f} s", full, FULL),
        (f"empty (768 kg) — {t_empty:.2f} s", empty, EMPTY),
    ):
        ax[0].plot(
            d["s"].to_numpy(), d["v"].to_numpy() * 3.6, color=col, lw=1.3, label=lab
        )
    ax[0].set_xlabel("distance (m)")
    ax[0].set_ylabel("speed (km/h)")
    ax[0].set_title(
        f"Fuel mass in isolation: lighter = faster (Δ = {t_full - t_empty:+.2f} s / lap)"
    )
    ax[0].legend(loc="lower center", fontsize=8)

    if "fuel_mass_kg" in stint:
        fm = stint["fuel_mass_kg"].to_numpy()  # (lap, s)
        for k in range(fm.shape[0]):
            ax[1].plot(
                stint["s"].to_numpy(),
                fm[k],
                lw=1.2,
                color=plt.cm.autumn(k / max(1, fm.shape[0])),
            )
        ax[1].set_xlabel("distance (m)")
        ax[1].set_ylabel("on-board fuel mass (kg)")
        ax[1].set_title("Tank drains monotonically over a 5-lap stint")
    fig.tight_layout()
    fig.savefig(_IMG / "fuel_mass_isolated.png", dpi=120)
    plt.close(fig)
    print("wrote", _IMG / "fuel_mass_isolated.png")


def _fig_lift_coast() -> None:
    track = Track.load(_TRACK)
    n = 40
    base = transient_lap_dataset(solve_transient_lap(_F1, track, ds_m=_DS, sim=_SIM))
    # Lift over the middle third of the lap: cap the tracked reference at 55 m/s there (+∞ elsewhere).
    lift = [float("inf")] * n
    for i in range(n // 3, 2 * n // 3):
        lift[i] = 55.0
    ds_lift = transient_lap_dataset(
        solve_transient_lap(
            _F1,
            track,
            ds_m=_DS,
            sim=_SIM,
            us_schedule={"deploy_regen": [0.0] * n, "lift_point": lift},
        )
    )
    fig, ax = plt.subplots(figsize=(10, 4.4))
    ax.plot(
        base["s"].to_numpy(),
        base["vx"].to_numpy() * 3.6,
        color=MAP0,
        lw=1.2,
        label="no lift",
    )
    ax.plot(
        ds_lift["s"].to_numpy(),
        ds_lift["vx"].to_numpy() * 3.6,
        color=LIFT,
        lw=1.3,
        label="lift-and-coast",
    )
    ax.set_xlabel("distance (m)")
    ax.set_ylabel("speed (km/h)")
    ax.set_title("Lift-and-coast: the u(s) lift_point caps the driver reference (§8.3)")
    ax.legend(loc="lower right", fontsize=9)
    fig.tight_layout()
    fig.savefig(_IMG / "lift_coast.png", dpi=120)
    plt.close(fig)
    print("wrote", _IMG / "lift_coast.png")


def main() -> None:
    _IMG.mkdir(parents=True, exist_ok=True)
    # The named-shift-map gear trace is demonstrated at the Rust tier
    # (`outlap-transient/src/control.rs::shift_map_id_selects_a_different_map`): a shift_maps entry
    # is declared in the vehicle document, which the per-vehicle override path does not synthesize.
    for fn in (_fig_fuel_isolated, _fig_lift_coast):
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 — a figure failure must not abort the rest.
            print(f"SKIPPED {fn.__name__}: {exc!r}")


if __name__ == "__main__":
    main()
