# SPDX-License-Identifier: AGPL-3.0-only
"""M5 PR9 tyre-thermal validation figure (docs/validation/img/tire_thermal_bands.png).

Drives the real T0 stint driver (`outlap.core.solve_stint_dataset`) on the reference F1 + Catalunya
and cross-checks the ring against the published Farroni/broadcast racing-slick surface-temperature
band (~85–115 °C):

  (a) cold-start warm-up — lap-end + per-lap-peak surface temperature climbing into the band, with
      the 63 %-rise time constant marked;
  (b) node ordering over the stint arc — surface hotter than carcass hotter than gas (TRT 3-node);
  (c) settled surface temperature from the equilibrium (warm) seed sitting in the band.

Run from anywhere:  python python/tools/plot_tire_thermal_validation.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, solve_stint_dataset

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "validation" / "img" / "tire_thermal_bands.png"
_CATALUNYA = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_F1 = str(_ROOT / "data" / "vehicles" / "limebeer_2014_f1")
_FAST: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2},
}
_BAND = (85.0, 115.0)


def main() -> None:
    trk = Track.load(_CATALUNYA)
    n_cold = 15
    cold = solve_stint_dataset(
        _F1, trk, n_laps=n_cold, tier="t0", ds_m=12.0, sim=_FAST,
        tire_thermal=True, initial_tire_temp_c=20.0,
    )
    warm = solve_stint_dataset(
        _F1, trk, n_laps=5, tier="t0", ds_m=12.0, sim=_FAST,
        tire_thermal=True, initial_tire_temp_c=None,
    )

    laps = cold["lap"].values
    s = cold["s"].values
    length_km = s[-1] / 1000.0
    surf = cold["tire_surface_c"].values  # (lap, s)
    lap_end = surf[:, -1]
    peak = surf.max(axis=1)
    steady = float(np.mean(lap_end[-3:]))
    target = 20.0 + 0.63 * (steady - 20.0)
    tau_lap = int(np.argmax(lap_end >= target)) + 1

    fig, axes = plt.subplots(1, 3, figsize=(16.5, 5.0))

    # (a) Warm-up curve + band + time constant.
    ax = axes[0]
    ax.axhspan(*_BAND, color="tab:green", alpha=0.12, label="slick band 85–115 °C")
    ax.plot(laps, lap_end, "o-", lw=2.2, color="tab:blue", ms=6, label="lap-end $T_s$")
    ax.plot(laps, peak, "s--", lw=1.4, color="tab:red", ms=4, label="per-lap peak $T_s$")
    ax.axhline(95.0, color="0.4", lw=1.0, ls=":", label=r"$T_\mathrm{opt}$ = 95 °C")
    ax.axvline(tau_lap, color="tab:purple", lw=1.2, ls="--")
    ax.annotate(
        f"63% rise\n~lap {tau_lap}", xy=(tau_lap, target), xytext=(tau_lap + 1.5, target - 18),
        fontsize=8.5, arrowprops={"arrowstyle": "->", "color": "0.4"},
    )
    ax.set_title("(a) Cold-start warm-up into the slick band", fontsize=11)
    ax.set_xlabel("lap")
    ax.set_ylabel(r"surface $T_s$ [°C]")
    ax.legend(loc="lower right", fontsize=8)

    # (b) Node ordering over the stint arc.
    ax = axes[1]
    dist = np.concatenate([s / 1000.0 + k * length_km for k in range(n_cold)])
    ax.plot(dist, surf.reshape(-1), "-", lw=1.3, color="tab:red", label=r"surface $T_s$")
    ax.plot(dist, cold["tire_carcass_c"].values.reshape(-1), "-", lw=1.3,
            color="tab:orange", label=r"carcass $T_c$")
    ax.plot(dist, cold["tire_gas_c"].values.reshape(-1), "-", lw=1.3,
            color="tab:blue", label=r"gas $T_g$")
    ax.axhspan(*_BAND, color="tab:green", alpha=0.10)
    ax.set_title("(b) Three-node ordering: $T_s > T_c > T_g$", fontsize=11)
    ax.set_xlabel("stint distance [km]")
    ax.set_ylabel("temperature [°C]")
    ax.legend(loc="lower right", fontsize=8)

    # (c) Settled (warm-seed) peak surface temperature in band.
    ax = axes[2]
    wl = warm["lap"].values
    warm_peak = warm["tire_surface_c"].values.max(axis=1)
    ax.axhspan(*_BAND, color="tab:green", alpha=0.12, label="slick band")
    ax.plot(wl, warm_peak, "o-", lw=2.2, color="tab:green", ms=7)
    ax.axhline(steady, color="0.4", lw=1.0, ls=":")
    ax.annotate(
        f"settled peak ≈ {float(np.mean(warm_peak[-2:])):.0f} °C",
        xy=(wl[-1], warm_peak[-1]), xytext=(0.2, 0.3), textcoords="axes fraction", fontsize=9,
        arrowprops={"arrowstyle": "->", "color": "0.4"},
    )
    ax.set_title("(c) Equilibrium seed — settled $T_s$ in band", fontsize=11)
    ax.set_xlabel("lap")
    ax.set_ylabel(r"peak surface $T_s$ [°C]")
    ax.set_xticks(wl)
    ax.legend(loc="lower right", fontsize=8)

    fig.suptitle(
        "M5 PR9 — tyre thermal validation: warm-up + settled surface temperature vs the published "
        "slick band (limebeer_2014_f1, Catalunya)",
        fontsize=12.5,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")
    print(f"steady T_s ~ {steady:.1f} C; 63% rise at lap ~{tau_lap}; warm-seed settled peak "
          f"~ {float(np.mean(warm_peak[-2:])):.1f} C")


if __name__ == "__main__":
    main()
