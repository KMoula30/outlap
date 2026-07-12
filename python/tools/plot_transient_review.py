# SPDX-License-Identifier: AGPL-3.0-only
"""Regenerate the transient-tier review figures (docs/theory/img/) from the real models.

Four figures, all driven live through the Python API — nothing here re-implements physics:

  t2_gear_shift.png          transient_control.md §2 — the shift FSM stepping the 8-speed
                             f1_2026 through its gears with the torque cut at each shift.
  t2_traction_discharge.png  transient_control.md §5 — a full pack discharging under traction
                             to open regen headroom (Model 3, seeded at the top of its window).
  driver_parity_catalunya.png  driver.md §6 — the T2 closed loop tracking the corner-scaled
                             reference against the raw QSS profile on catalunya_osm.
  t2_3d_pace_parity.png      transient_chassis.md — flat vs 3-D T2 lap time, three cars.

Run from anywhere:  python python/tools/plot_transient_review.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import (
    Track,
    min_curvature,
    solve_lap_dataset,
    solve_transient_lap,
    transient_lap_dataset,
)

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_IMG = _ROOT / "docs" / "theory" / "img"
_TRACK = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_CARS = {
    "limebeer_2014_f1": "limebeer",
    "f1_2026": "f1_2026",
    "tesla_model3_rwd": "model3",
}
# The figures are physics illustrations, not accuracy gates: a moderate envelope grid keeps the
# regeneration quick while leaving every transient effect intact.
_ENV: dict[str, object] = {"v_points": 12, "ax_points": 10, "g_normal_points": 3}
BLUE, RED, GREEN, AMBER, GRID = "#2a78d6", "#e34948", "#1baf7a", "#d98f2b", "#e7e6e2"


def _t2(car: str, flat: bool, **kw: object):
    track = Track.load(_TRACK)
    rl = min_curvature(track, 1.1)
    lap = solve_transient_lap(
        str(_ROOT / "data" / "vehicles" / car),
        rl.line(),
        raceline_ds_m=rl.ds_m,
        sim={"flat_track": flat, "envelope": _ENV},
        **kw,  # type: ignore[arg-type]
    )
    return transient_lap_dataset(lap)


def gear_shift() -> None:
    ds = _t2("f1_2026", True)
    t = ds["time"].to_numpy()
    gear = ds["gear"].to_numpy() + 1  # display 1-based (1st..8th)
    tq = ds["torque_scale"].to_numpy()
    vx = ds["vx"].to_numpy() * 3.6
    n_shifts = int(np.count_nonzero(np.diff(gear)))

    fig, axes = plt.subplots(3, 1, figsize=(10, 6.6), sharex=True)
    axes[0].plot(t, vx, color=BLUE, lw=0.9)
    axes[0].set_ylabel("speed (km/h)")
    axes[0].set_title(
        f"f1_2026 shift FSM — 8-speed, {n_shifts} shift events over the lap "
        f"({ds.attrs['lap_time_s']:.1f} s)"
    )
    axes[1].step(t, gear, color=AMBER, lw=1.2, where="post")
    axes[1].set_yticks(range(1, 9))
    axes[1].set_ylabel("gear")
    axes[2].plot(t, tq, color=RED, lw=0.7)
    axes[2].set_ylabel("torque scale")
    axes[2].set_xlabel("time (s)")
    axes[2].set_title(
        "the torque cut at every shift (torque-cut → ratio swap → clutch ramp)",
        fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(_IMG / "t2_gear_shift.png", dpi=130)
    plt.close(fig)
    print(
        f"t2_gear_shift.png: gears {sorted(set(gear.astype(int).tolist()))}, {n_shifts} shifts"
    )


def traction_discharge() -> None:
    # Seed the pack at the TOP of its SoC window: it can accept no charge, so regen is refused
    # until traction discharge opens headroom — the §5 story.
    ds = _t2("tesla_model3_rwd", True, initial_soc=0.9)
    t = ds["time"].to_numpy()
    soc = ds["state_of_charge"].to_numpy() * 100.0
    regen = ds["regen_power_w"].to_numpy() / 1e3
    traction = ds["traction_power_w"].to_numpy() / 1e3

    fig, axes = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    axes[0].plot(t, soc, color=BLUE, lw=1.4)
    axes[0].set_ylabel("state of charge (%)")
    axes[0].set_title(
        "a full pack discharges under traction, opening the headroom that lets regen resume"
    )
    axes[1].fill_between(t, 0, traction, color=RED, alpha=0.6, label="traction draw")
    axes[1].fill_between(t, 0, -regen, color=GREEN, alpha=0.7, label="regen recovered")
    axes[1].axhline(0.0, color=GRID)
    axes[1].set_ylabel("pack power (kW)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(ncols=2, fontsize=9)
    fig.tight_layout()
    fig.savefig(_IMG / "t2_traction_discharge.png", dpi=130)
    plt.close(fig)
    print(f"t2_traction_discharge.png: SoC {soc[0]:.1f} → {soc[-1]:.1f} %")


def driver_parity() -> None:
    track = Track.load(_TRACK)
    rl = min_curvature(track, 1.1)
    t0 = solve_lap_dataset(
        str(_ROOT / "data" / "vehicles" / "limebeer_2014_f1"),
        rl,
        tier="t0",
        sim={"flat_track": True, "envelope": _ENV},
    )
    t2 = _t2("limebeer_2014_f1", True)
    s2 = t2["s"].to_numpy() % track.length()

    fig, ax = plt.subplots(figsize=(10.5, 4.4))
    ax.plot(
        t0.s.to_numpy() / 1e3,
        t0.v.to_numpy() * 3.6,
        color=BLUE,
        lw=1.3,
        label=f"T0 QSS profile ({t0.attrs['lap_time_s']:.1f} s)",
    )
    ax.scatter(
        s2 / 1e3,
        t2["vx"].to_numpy() * 3.6,
        s=1,
        color=RED,
        alpha=0.5,
        label=f"T2 closed loop ({t2.attrs['lap_time_s']:.1f} s)",
    )
    ax.set_xlabel("s (km)")
    ax.set_ylabel("speed (km/h)")
    ax.set_title(
        "limebeer on catalunya_osm — the corner-scaled reference: full profile speed on the "
        "straights, the stability margin in the corners"
    )
    ax.legend(loc="lower left", fontsize=9)
    fig.tight_layout()
    fig.savefig(_IMG / "driver_parity_catalunya.png", dpi=130)
    plt.close(fig)
    print(
        f"driver_parity_catalunya.png: T0 {t0.attrs['lap_time_s']:.2f} s, "
        f"T2 {t2.attrs['lap_time_s']:.2f} s"
    )


def pace_parity_3d() -> None:
    labels, flat_t, three_t = [], [], []
    for car, short in _CARS.items():
        f = float(_t2(car, True).attrs["lap_time_s"])
        d = float(_t2(car, False).attrs["lap_time_s"])
        labels.append(short)
        flat_t.append(f)
        three_t.append(d)
        print(
            f"pace parity {short}: flat {f:.2f} s, 3-D {d:.2f} s ({100 * (d - f) / f:+.2f}%)"
        )

    x = np.arange(len(labels))
    w = 0.38
    fig, ax = plt.subplots(figsize=(8.2, 4.2))
    b1 = ax.bar(x - w / 2, flat_t, w, color=BLUE, label="flat")
    b2 = ax.bar(x + w / 2, three_t, w, color=GREEN, label="3-D road frame")
    ax.set_xticks(x, labels)
    ax.set_ylabel("T2 lap time (s)")
    worst = max(abs(d - f) / f for f, d in zip(flat_t, three_t, strict=True)) * 100.0
    ax.set_title(
        f"flat vs 3-D T2 lap time — worst delta {worst:.2f}% (three reference cars)"
    )
    ax.set_ylim(0, max(three_t + flat_t) * 1.15)
    for bars in (b1, b2):
        for rect in bars:
            ax.text(
                rect.get_x() + rect.get_width() / 2,
                rect.get_height(),
                f"{rect.get_height():.1f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )
    ax.legend()
    fig.tight_layout()
    fig.savefig(_IMG / "t2_3d_pace_parity.png", dpi=130)
    plt.close(fig)


def main() -> None:
    _IMG.mkdir(parents=True, exist_ok=True)
    gear_shift()
    traction_discharge()
    driver_parity()
    pace_parity_3d()


if __name__ == "__main__":
    main()
