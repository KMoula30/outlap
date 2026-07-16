# SPDX-License-Identifier: AGPL-3.0-only
"""M5 PR9 wear/cliff validation figure (docs/validation/img/wear_cliff.png).

Closes the calibration loop end-to-end: inverse-calibrate the committed derived stint fixture with
`outlap.wearcal`, then run the recovered parameters through the **real** T0 stint driver, and
compare the T0 and T2 decay rates.

  (a) the fixture vs the surrogate fit (the inverse-calibration target);
  (b) the real-driver 24-lap stint from the calibrated parameters — pace loss with the cliff lap;
  (c) wear accumulation crossing the critical depth `w_c`, and the per-lap pace-loss hump peaking
      at the cliff (the sigmoid inflection / positive-feedback signature);
  (d) QSS↔T2 stint-decay agreement (≤ 0.1 s/lap gate).

Run from anywhere:  python python/tools/plot_wear_cliff_validation.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, solve_stint_dataset
from outlap.wearcal import calibrate, load_fixture, stint_trace
from outlap.wearcal.model import StintAnchor
from outlap.wearcal.sim import sim_stint

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "validation" / "img" / "wear_cliff.png"
_CATALUNYA = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_F1 = _ROOT / "data" / "vehicles" / "limebeer_2014_f1"
_FIXTURE = _ROOT / "data" / "wear" / "f1_medium_catalunya_stint.csv"
_FAST: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2},
}


def main() -> None:
    trk = Track.load(_CATALUNYA)
    obs = load_fixture(_FIXTURE)
    result = calibrate(obs)
    anchor = StintAnchor(t_ref_s=float(np.min(obs.lap_time_s)))
    fit = stint_trace(result.params, anchor, obs.n_laps)

    n = 24
    sim = sim_stint(_F1, trk, result.params, n, tier="t0", ds_m=14.0, sim=_FAST)
    lt = sim.lap_time_s
    wear = sim.wear_mm
    laps = np.arange(1, n + 1)
    crossed = np.nonzero(wear >= result.params.w_c)[0]
    cliff = int(crossed[0] + 1) if crossed.size else n
    deltas = np.diff(lt)

    # QSS vs T2 decay.
    def decay(x: np.ndarray) -> float:
        return float(np.polyfit(np.arange(1, x.size + 1), x, 1)[0])

    t0 = solve_stint_dataset(str(_F1), trk, n_laps=6, tier="t0", ds_m=12.0, sim=_FAST,
                             tire_thermal=True, initial_tire_temp_c=None)
    t2 = solve_stint_dataset(str(_F1), trk, n_laps=6, tier="t2", ds_m=12.0, sim=_FAST,
                             tire_thermal=True, initial_tire_temp_c=None)
    d0 = decay(np.asarray(t0["lap_time_s"].values, float))
    d2 = decay(np.asarray(t2["lap_time_s"].values, float))

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.0))

    # (a) Fixture vs surrogate fit.
    ax = axes[0, 0]
    ax.plot(obs.lap, obs.lap_time_s, "o", color="tab:blue", ms=6, label="fixture (observed)")
    ax.plot(fit.lap, fit.lap_time_s, "-", lw=2.2, color="tab:red", label="surrogate fit")
    ax.set_title(f"(a) Inverse calibration (RMS {result.rms_s:.3f} s)", fontsize=11)
    ax.set_xlabel("lap")
    ax.set_ylabel("lap time [s]")
    ax.legend(loc="upper left", fontsize=8)
    ax.text(0.03, 0.72, f"$k_w$={result.fitted['k_w']:.2e}\n$w_c$={result.fitted['w_c']:.2f} mm\n"
            f"$\\Delta_c$={result.fitted['delta_c']:.3f}", transform=ax.transAxes, fontsize=8.5,
            va="top", bbox={"boxstyle": "round", "fc": "white", "alpha": 0.7})

    # (b) Real-driver stint pace with the cliff lap.
    ax = axes[0, 1]
    ax.plot(laps, lt, "o-", lw=2.2, color="tab:red", ms=5)
    ax.axvline(cliff, color="tab:orange", lw=1.4, ls="--", label=f"cliff lap {cliff}")
    ax.set_title(f"(b) Real T0 driver from calibrated params (+{lt.max() - lt.min():.1f} s)",
                 fontsize=11)
    ax.set_xlabel("lap")
    ax.set_ylabel("lap time [s]")
    ax.legend(loc="upper left", fontsize=8)

    # (c) Wear crossing w_c + the per-lap pace-loss hump.
    ax = axes[1, 0]
    ax.plot(laps, wear, "-", lw=2.0, color="tab:brown", label="tread wear $w$")
    ax.axhline(result.params.w_c, color="tab:orange", lw=1.2, ls="--", label="$w_c$ (cliff onset)")
    ax.axvline(cliff, color="tab:orange", lw=1.0, ls=":")
    ax.set_title("(c) Wear crosses $w_c$; pace-loss rate peaks at the cliff", fontsize=11)
    ax.set_xlabel("lap")
    ax.set_ylabel("wear $w$ [mm]", color="tab:brown")
    ax.tick_params(axis="y", labelcolor="tab:brown")
    ax.legend(loc="upper left", fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(laps[1:], deltas, "s-", lw=1.4, color="tab:purple", ms=4)
    ax2.set_ylabel("per-lap pace loss [s/lap]", color="tab:purple")
    ax2.tick_params(axis="y", labelcolor="tab:purple")
    ax2.grid(False)

    # (d) QSS vs T2 decay.
    ax = axes[1, 1]
    bars = ax.bar(["T0 (QSS)", "T2 (transient)"], [d0, d2],
                  color=["tab:blue", "tab:purple"], width=0.55)
    ax.axhline(0.1, color="tab:red", lw=1.2, ls="--", label="≤ 0.1 s/lap gate")
    for b, v in zip(bars, [d0, d2], strict=True):
        ax.annotate(f"{v:.3f}", xy=(b.get_x() + b.get_width() / 2, v), xytext=(0, 4),
                    textcoords="offset points", ha="center", fontsize=9)
    ax.set_title(f"(d) QSS↔T2 decay agreement (|Δ| = {abs(d0 - d2):.3f} s/lap)", fontsize=11)
    ax.set_ylabel("stint decay [s/lap]")
    ax.legend(loc="upper right", fontsize=8)

    fig.suptitle(
        "M5 PR9 — wear/cliff validation: inverse-calibrate the fixture, reproduce decay + cliff in "
        "the real driver, and cross-check QSS↔T2 decay (limebeer_2014_f1, Catalunya)",
        fontsize=12.5,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")
    print(f"cliff lap {cliff}; total loss {lt.max() - lt.min():.2f} s; "
          f"decay T0 {d0:.3f} / T2 {d2:.3f} s/lap (|Δ| {abs(d0 - d2):.3f})")


if __name__ == "__main__":
    main()
