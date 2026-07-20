# SPDX-License-Identifier: AGPL-3.0-only
"""Render the M6 PR7 T3 tier-integration figures — from the real solver (a full lap).

Runs f1_2026 at BOTH tiers on catalunya and draws the comparison the PR is about: the live
suspension the T3 tier integrates, the pitch-under-braking → aero-balance behaviour (§6.1), and the
T2↔T3 agreement. Writes into ``docs/theory/img/``:

  * ``t3_tier_overlay.png``   — T2 vs T3 speed over distance (the tiers agree on pace) + the T3
                                sprung heave/pitch and the front/rear ride heights that only T3 has.
  * ``t3_aero_balance.png``   — pitch vs longitudinal accel (nose-down under braking) and the front
                                ride height dropping in the braking zones — the aero-balance-shift
                                mechanism.

Run (after `maturin develop --release`):  python python/tools/plot_t3_tier.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, solve_lap_dataset

_ROOT = Path(__file__).resolve().parents[2]
_IMG = _ROOT / "docs" / "theory" / "img"
_DATA = _ROOT / "data"

plt.style.use("seaborn-v0_8-darkgrid")

F1 = str(_DATA / "vehicles/f1_2026")
CAT = str(_DATA / "tracks/catalunya_osm")
SIM = {"flat_track": True, "envelope": {"v_points": 12, "ax_points": 9, "g_normal_points": 3}}


def main() -> None:
    _IMG.mkdir(parents=True, exist_ok=True)
    tr = Track.load(CAT)
    t2 = solve_lap_dataset(F1, tr, ds_m=5.0, tier="t2", sim=SIM)
    t3 = solve_lap_dataset(F1, tr, ds_m=5.0, tier="t3", sim=SIM)
    print(f"t2 lap {float(t2.lap_time_s):.3f}s   t3 lap {float(t3.lap_time_s):.3f}s")

    s2, v2 = t2["s"].values, t2["vx"].values
    s3, v3 = t3["s"].values, t3["vx"].values
    heave = t3["heave_m"].values * 1000.0
    pitch = np.degrees(t3["pitch_rad"].values)
    hf = t3["ride_height_f_m"].values * 1000.0
    hr = t3["ride_height_r_m"].values * 1000.0
    ax3 = t3["ax"].values

    # --- Figure 1: tier overlay + the T3-only suspension state -------------------------------------
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axs[0].plot(s2, v2 * 3.6, lw=1.3, label=f"T2 ({float(t2.lap_time_s):.2f} s)", color="#4C78A8")
    axs[0].plot(s3, v3 * 3.6, lw=1.3, label=f"T3 ({float(t3.lap_time_s):.2f} s)", color="#E45756")
    axs[0].set_ylabel("speed (km/h)")
    axs[0].legend(loc="lower right")
    axs[0].set_title("f1_2026 · catalunya — the tiers agree on pace; T3 adds ride fidelity")

    axs[1].plot(s3, heave, lw=1.2, color="#54A24B", label="sprung heave z")
    axs[1].plot(s3, pitch * 20.0, lw=1.0, color="#B279A2", label="pitch θ (×20, deg)")
    axs[1].axhline(0.0, color="k", lw=0.6, alpha=0.5)
    axs[1].set_ylabel("heave (mm) / pitch")
    axs[1].legend(loc="lower right")

    axs[2].plot(s3, hf, lw=1.2, color="#4C78A8", label="front ride height")
    axs[2].plot(s3, hr, lw=1.2, color="#F58518", label="rear ride height")
    axs[2].axhline(40.0, color="#4C78A8", lw=0.7, ls="--", alpha=0.6, label="front static (40 mm)")
    axs[2].axhline(90.0, color="#F58518", lw=0.7, ls="--", alpha=0.6, label="rear static (90 mm)")
    axs[2].set_ylabel("ride height (mm)")
    axs[2].set_xlabel("distance s (m)")
    axs[2].legend(loc="lower right", ncol=2, fontsize=8)
    fig.tight_layout()
    fig.savefig(_IMG / "t3_tier_overlay.png", dpi=120)
    print("wrote", _IMG / "t3_tier_overlay.png")

    # --- Figure 2: the aero-balance-shift mechanism -----------------------------------------------
    fig2, (a, b) = plt.subplots(1, 2, figsize=(12, 4.5))
    strong = np.abs(ax3) > 1.0
    corr = np.corrcoef(ax3[strong], pitch[strong])[0, 1]
    a.scatter(ax3[strong], pitch[strong], s=6, alpha=0.35, color="#E45756")
    a.axhline(0, color="k", lw=0.6)
    a.axvline(0, color="k", lw=0.6)
    a.set_xlabel("longitudinal accel a_x (m/s²)")
    a.set_ylabel("pitch θ (deg, +nose-down)")
    a.set_title(f"nose-down under braking (corr {corr:.2f})")

    braking = ax3 < -3.0
    b.plot(s3, hf, lw=1.0, color="#4C78A8", label="front ride height")
    b.fill_between(
        s3, hf.min(), hf.max(), where=braking, color="#E45756", alpha=0.15, label="braking (a_x<−3)"
    )
    b.set_xlabel("distance s (m)")
    b.set_ylabel("front ride height (mm)")
    b.set_title("front platform drops in the braking zones → aero balance moves forward")
    b.legend(loc="lower right", fontsize=8)
    fig2.tight_layout()
    fig2.savefig(_IMG / "t3_aero_balance.png", dpi=120)
    print("wrote", _IMG / "t3_aero_balance.png")


if __name__ == "__main__":
    main()
