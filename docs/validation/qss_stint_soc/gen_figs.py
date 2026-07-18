# SPDX-License-Identifier: AGPL-3.0-only
"""M6 PR3 review figures: the QSS stint SoC carry (before/after), EV decline, hybrid within-lap
recovery, and the lap-boundary continuity residual. Writes PNGs into docs/validation/qss_stint_soc/."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, solve_lap_dataset, solve_stint_dataset

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
OUT.mkdir(parents=True, exist_ok=True)

CATALUNYA = str(ROOT / "data" / "tracks" / "catalunya_osm")
F1 = str(ROOT / "data" / "vehicles" / "f1_2026")
MODEL3 = str(ROOT / "data" / "vehicles" / "tesla_model3_rwd")

COARSE = {"flat_track": True, "envelope": {"v_points": 10, "ax_points": 9, "g_normal_points": 2}}
EV_SIM = {"envelope": {"v_points": 10, "ax_points": 9, "g_normal_points": 2}}

INK = "#1b1b1f"
AFTER = "#2b6cb0"
BEFORE = "#c05621"
GRID = "#d9d9e0"
plt.rcParams.update(
    {
        "figure.dpi": 130,
        "font.size": 11,
        "axes.edgecolor": INK,
        "axes.labelcolor": INK,
        "text.color": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.7,
        "axes.axisbelow": True,
    }
)

N_LAPS = 6
print("solving f1 stint (after) …")
stint = solve_stint_dataset(F1, Track.load(CATALUNYA), n_laps=N_LAPS, tier="t0", sim=COARSE, tire_thermal=False)
soc = stint["state_of_charge"].values  # (lap, s)
n_st = soc.shape[1]

print("solving f1 single lap (before = per-lap reset to mid-window) …")
single = solve_lap_dataset(F1, Track.load(CATALUNYA), tier="t0", sim=COARSE)
soc_reset = single["state_of_charge"].values  # every reset lap is identical to this

laps = np.arange(1, N_LAPS + 1)

# --- Figure 1: SoC staircase, before vs after --------------------------------------------------
fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw={"width_ratios": [1.5, 1]})

# Concatenated within-lap traces over the whole stint (after) vs the repeated reset lap (before).
x = np.arange(N_LAPS * n_st) / n_st + 1.0
ax0.plot(x, soc.reshape(-1), color=AFTER, lw=1.6, label="after — carried (this PR)")
ax0.plot(x, np.tile(soc_reset, N_LAPS), color=BEFORE, lw=1.4, ls="--", alpha=0.9,
         label="before — pack reset every lap")
for b in range(1, N_LAPS):
    ax0.axvline(b + 1, color=GRID, lw=1.0)
ax0.set_xlabel("lap (arc-length within each)")
ax0.set_ylabel("pack state of charge")
ax0.set_title("f1_2026 QSS stint — SoC over 6 laps")
ax0.legend(loc="lower right", framealpha=0.95)

# Lap-start SoC staircase: after climbs toward the recharge ceiling; before is pinned at the seed.
start_after = soc[:, 0]
start_before = np.full(N_LAPS, soc_reset[0])
ax1.step(laps, start_after, where="mid", color=AFTER, lw=2.0, marker="o", label="after")
ax1.step(laps, start_before, where="mid", color=BEFORE, lw=1.6, ls="--", marker="s", label="before")
ax1.set_xlabel("lap")
ax1.set_ylabel("lap-start SoC")
ax1.set_title("lap-start SoC: carry vs reset")
ax1.legend(loc="best", framealpha=0.95)
fig.tight_layout()
fig.savefig(OUT / "fig1_soc_staircase.png", bbox_inches="tight")
print("wrote fig1")

# --- Figure 2: EV monotone decline (model3) -----------------------------------------------------
print("solving model3 EV stint …")
ev = solve_stint_dataset(MODEL3, Track.load(CATALUNYA), n_laps=4, tier="t0", sim=EV_SIM, tire_thermal=False)
ev_soc = ev["state_of_charge"].values
ev_n = ev_soc.shape[1]
xe = np.arange(4 * ev_n) / ev_n + 1.0
fig2, ax = plt.subplots(figsize=(7.5, 4.0))
ax.plot(xe, ev_soc.reshape(-1), color=AFTER, lw=1.7)
for b in range(1, 4):
    ax.axvline(b + 1, color=GRID, lw=1.0)
ax.set_xlabel("lap (arc-length within each)")
ax.set_ylabel("pack state of charge")
ax.set_title("tesla_model3_rwd — pure-EV QSS stint: monotone SoC decline (no manager → discharge-only)")
fig2.tight_layout()
fig2.savefig(OUT / "fig2_ev_decline.png", bbox_inches="tight")
print("wrote fig2")

# --- Figure 3: hybrid within-lap recovery -------------------------------------------------------
lap1 = solve_lap_dataset(F1, Track.load(CATALUNYA), tier="t0", sim=COARSE)
s = lap1["s"].values
soc1 = lap1["state_of_charge"].values
deploy = lap1["deploy_power_w"].values
harvest = lap1["harvest_power_w"].values
fig3, ax = plt.subplots(figsize=(9, 4.0))
ax.plot(s, soc1, color=INK, lw=1.8, label="pack SoC", zorder=5)
ax.fill_between(s, soc1.min(), soc1.max(), where=deploy > 1.0, color=BEFORE, alpha=0.18,
                label="deploying (SoC falls)")
ax.fill_between(s, soc1.min(), soc1.max(), where=harvest > 1.0, color=AFTER, alpha=0.22,
                label="harvesting (SoC rises)")
ax.set_xlabel("arc length s, m")
ax.set_ylabel("pack state of charge")
ax.set_title("f1_2026 — within-lap SoC: deploy dips + braking-harvest recovery")
ax.legend(loc="best", framealpha=0.95)
fig3.tight_layout()
fig3.savefig(OUT / "fig3_hybrid_recovery.png", bbox_inches="tight")
print("wrote fig3")

# --- Figure 4: lap-boundary continuity residual -------------------------------------------------
# After: |lap k+1 start − lap k terminal| ≈ one march segment. Before (reset): the jump back to the
# mid-window seed, ~0.3 SoC for the f1 pack that recharges toward the top of its window.
resid_after = np.abs(soc[1:, 0] - soc[:-1, -1])
resid_before = np.abs(np.full(N_LAPS - 1, soc_reset[0]) - soc[:-1, -1])
boundaries = np.arange(1, N_LAPS)
fig4, ax = plt.subplots(figsize=(7.5, 4.0))
w = 0.38
ax.bar(boundaries - w / 2, resid_before, w, color=BEFORE, label="before — reset jump")
ax.bar(boundaries + w / 2, np.maximum(resid_after, 1e-4), w, color=AFTER, label="after — carried (≈ 1 segment)")
ax.set_yscale("log")
ax.set_xlabel("lap boundary (k → k+1)")
ax.set_ylabel("|SoC discontinuity| at the boundary")
ax.set_title("lap-boundary SoC continuity: carry closes the reset jump")
ax.set_xticks(boundaries)
ax.legend(loc="best", framealpha=0.95)
fig4.tight_layout()
fig4.savefig(OUT / "fig4_continuity.png", bbox_inches="tight")
print("wrote fig4 ->", OUT)
