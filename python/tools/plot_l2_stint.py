# SPDX-License-Identifier: AGPL-3.0-only
"""Render the D-M6-13 Layer-2 stint figure (docs/theory/img/mgu_k_stint.png).

Runs a 10-lap f1_2026 stint in BOTH tiers (QSS point-mass + T2 transient, shared initial SoC, flat
+ coarse envelope — the same config `python/tests/test_stint_soc.py` gates) and plots the two headline
Layer-2 behaviours:

  (a) end-of-lap SoC per lap — with the MGU-K sized to deploy its rated 350 kW (223 N·m), the f1 is a
      hard-deploying, SoC-starved car: BOTH tiers out-deploy the harvest, drain the pack to the window
      floor and charge-sustain there — QSS and T2 agree;
  (b) the within-lap SoC trace, QSS lap 1 vs lap 10 — deploy drains, harvest refills, and the state
      CARRIES lap-to-lap (lap 10 cycles down at the floor, not back at the mid-window seed);
  (c) the MGU-K winding temperature per lap that the QSS march surfaces (`machine_temp_c`) — bounded
      and stable, the derate that trims the deploy. (The transient tier carries its winding internally
      with real-time cooling — gated by `crates/outlap-transient/tests/machine_thermal.rs` — but does
      not yet surface it as a T2 stint channel, so only the QSS line is drawn.)

Run from anywhere (needs the built extension):  python python/tools/plot_l2_stint.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, solve_stint_dataset

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"
_OUT = _ROOT / "docs" / "theory" / "img" / "mgu_k_stint.png"

F1 = str(_DATA / "vehicles/f1_2026")
CATALUNYA = str(_DATA / "tracks/catalunya_osm")
SEED = 0.6
N_LAPS = 10
COARSE = {"flat_track": True, "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}


def main() -> None:
    track = Track.load(CATALUNYA)
    q = solve_stint_dataset(F1, track, n_laps=N_LAPS, tier="t0", sim=COARSE,
                            tire_thermal=False, initial_soc=SEED)
    t2 = solve_stint_dataset(F1, track, n_laps=N_LAPS, tier="t2", sim=COARSE,
                             tire_thermal=False, initial_soc=SEED, speed_margin=0.85)

    laps = np.arange(1, N_LAPS + 1)
    q_end = q["state_of_charge"].values[:, -1]
    t2_end = t2["state_of_charge"].values

    fig, (axa, axb, axc) = plt.subplots(1, 3, figsize=(15.5, 4.6))
    fig.suptitle(
        "D-M6-13 Layer 2 — f1_2026 10-lap stint, both tiers (flat catalunya, seed SoC 0.60)",
        fontsize=13, weight="bold",
    )

    # (a) end-of-lap SoC: the two equilibria.
    axa.axhline(SEED, color="gray", ls=":", lw=1.0, label="seed 0.60")
    axa.plot(laps, q_end, color="#1f6feb", lw=2.2, marker="o", ms=5,
             label="QSS T0 (ideal limit)")
    axa.plot(laps, t2_end, color="#e67e22", lw=2.2, marker="s", ms=5,
             label="T2 (0.85 margin)")
    axa.set_xlabel("lap")
    axa.set_ylabel("end-of-lap pack SoC")
    axa.set_title("(a) both tiers drain to the window floor and agree")
    axa.set_ylim(0.0, 1.0)
    axa.legend(loc="center right", fontsize=8, framealpha=0.7)

    # (b) within-lap QSS trace, lap 1 vs lap 10 — cycling + carry.
    s = q["s"].values
    axb.plot(s / 1000.0, q["state_of_charge"].values[0], color="#1f6feb", lw=1.8, label="lap 1")
    axb.plot(s / 1000.0, q["state_of_charge"].values[-1], color="#c0392b", lw=1.8, label="lap 10")
    axb.set_xlabel("distance [km]")
    axb.set_ylabel("pack SoC")
    axb.set_title("(b) within-lap deploy↓ / harvest↑ (QSS), carried")
    axb.legend(loc="lower right", fontsize=9, framealpha=0.7)

    # (c) winding temperature per lap: QSS (within-lap, re-seeded) vs T2 (carried).
    axc.plot(laps, q["machine_temp_c"].values, color="#1f6feb", lw=2.2, marker="o", ms=5,
             label="QSS (marched within lap)")
    tkey = "machine_temp_c" if "machine_temp_c" in t2 else None
    if tkey is not None:
        axc.plot(laps, t2[tkey].values, color="#e67e22", lw=2.2, marker="s", ms=5,
                 label="T2 (carried across laps)")
    axc.set_xlabel("lap")
    axc.set_ylabel("MGU-K winding temperature [°C]")
    axc.set_title("(c) winding temp → deploy derate")
    axc.legend(loc="best", fontsize=9, framealpha=0.7)

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT.relative_to(_ROOT)}")
    print(f"  QSS end SoC lap1={q_end[0]:.3f} lap10={q_end[-1]:.3f}; "
          f"T2 end SoC lap1={t2_end[0]:.3f} lap10={t2_end[-1]:.3f}")


if __name__ == "__main__":
    main()
