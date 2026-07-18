# SPDX-License-Identifier: AGPL-3.0-only
"""Render the M6 PR4 T2 ERS-wiring theory + review figures.

Drives everything from the **real** solver: an ``f1_2026`` transient (T2) lap with the 2026 ERS
energy manager active, a T0 point-mass lap of the same car for the tier overlay, and an override
lap. Also draws the optional-2nd-RC-pair battery step response (the double-exponential Thevenin
closed form the integrator reproduces to machine precision).

Writes:
  * ``docs/theory/img/ers_t2_lap.png``       — deploy / harvest / SoC over an f1 T2 lap
  * ``docs/theory/img/ers_t2_tier_overlay.png`` — T0 vs T2 SoC + deploy (one rulebook, both tiers)
  * ``docs/theory/img/battery_2rc.png``      — the 2-RC vs 1-RC pack step response
  * ``docs/theory/img/ers_t2_override.png``  — override ("Overtake") vs the rule-based lap

Run from anywhere:  python python/tools/plot_ers_t2.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import (
    Track,
    solve_lap_dataset,
    solve_transient_lap,
    transient_lap_dataset,
)

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_IMG = _ROOT / "docs" / "theory" / "img"
_TRACK = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_F1 = str(_ROOT / "data" / "vehicles" / "f1_2026")
# A coarse envelope keeps the figure fast while the physics (deploy/harvest/SoC) is fidelity-robust.
_SIM: dict[str, object] = {
    "envelope": {"v_points": 12, "ax_points": 10, "g_normal_points": 2}
}
_DS = 8.0

DEPLOY = "#2f6fb0"  # blue  — deploy / draw
HARVEST = "#3a9a54"  # green — harvest / recovery
SOC = "#b5651d"  # amber — state of charge
REF = "#8a8f98"  # grey  — reference / rule-based


def _t2_lap(*, override: bool = False):
    ds = transient_lap_dataset(
        solve_transient_lap(
            _F1, Track.load(_TRACK), ds_m=_DS, sim=_SIM, override=override
        )
    )
    return ds


def _fig_t2_lap(ds) -> None:
    s = ds["s"].to_numpy() if "s" in ds else np.arange(ds.sizes["time"], dtype=float)
    vx = ds["vx"].to_numpy() * 3.6  # km/h
    deploy = ds["traction_power_w"].to_numpy() / 1e3  # kW (pack draw = MGU-K deploy)
    harvest = ds["regen_power_w"].to_numpy() / 1e3  # kW
    soc = ds["state_of_charge"].to_numpy() * 100.0

    fig, ax = plt.subplots(3, 1, figsize=(10, 8.5), sharex=True)
    ax[0].plot(s, vx, color="#333", lw=1.2)
    ax[0].set_ylabel("speed (km/h)")
    ax[0].set_title("f1_2026 T2 lap — the 2026 ERS energy manager (Catalunya)")

    ax[1].fill_between(
        s, 0, deploy, color=DEPLOY, alpha=0.85, label="deploy (pack draw)"
    )
    ax[1].fill_between(
        s, 0, -harvest, color=HARVEST, alpha=0.85, label="harvest (recovery)"
    )
    ax[1].axhline(350, color=DEPLOY, ls="--", lw=0.9)
    ax[1].axhline(-350, color=HARVEST, ls="--", lw=0.9)
    ax[1].text(
        s[-1],
        352,
        "  350 kW FIA cap",
        color=DEPLOY,
        va="bottom",
        ha="right",
        fontsize=8,
    )
    ax[1].set_ylabel("electrical power (kW)")
    ax[1].legend(loc="upper right", ncol=2, fontsize=9)

    ax[2].plot(s, soc, color=SOC, lw=1.4)
    ax[2].axhspan(20, 90, color=SOC, alpha=0.08)
    ax[2].set_ylabel("state of charge (%)")
    ax[2].set_xlabel("distance (m)")
    ax[2].text(
        s[0],
        91,
        " usable window [20, 90] % — the FIA C5.2.9 4 MJ swing",
        color=SOC,
        va="bottom",
        fontsize=8,
    )
    fig.tight_layout()
    fig.savefig(_IMG / "ers_t2_lap.png", dpi=120)
    plt.close(fig)
    print("wrote", _IMG / "ers_t2_lap.png")


def _deploy_kw(d):
    # The realized electrical deploy: the QSS Lap exposes it as `deploy_power_w`; the T2 lap
    # publishes it as the pack draw `traction_power_w` (the block writes it for an ERS car).
    name = "deploy_power_w" if "deploy_power_w" in d else "traction_power_w"
    return d[name].to_numpy() / 1e3


def _fig_tier_overlay(ds_t2) -> None:
    # T0 point-mass lap of the same car (arc-length indexed) through the SAME rulebook.
    t0 = solve_lap_dataset(_F1, Track.load(_TRACK), ds_m=_DS, tier="t0", sim=_SIM)
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    # SoC vs normalized lap distance (the two tiers run different station grids).
    for lab, d, col in (("T0 (point mass)", t0, REF), ("T2 (7-DOF)", ds_t2, SOC)):
        soc = d["state_of_charge"].to_numpy() * 100.0
        x = np.linspace(0, 1, len(soc))
        ax[0].plot(x, soc, color=col, lw=1.6, label=lab)
    ax[0].set_xlabel("lap fraction")
    ax[0].set_ylabel("state of charge (%)")
    ax[0].set_title("SoC — one rulebook, both tiers")
    ax[0].legend(loc="best", fontsize=9)

    for lab, d, col in (("T0", t0, REF), ("T2", ds_t2, DEPLOY)):
        dep = _deploy_kw(d)
        x = np.linspace(0, 1, len(dep))
        ax[1].plot(x, dep, color=col, lw=1.0, alpha=0.9, label=lab)
    ax[1].axhline(350, color="#444", ls="--", lw=0.8)
    ax[1].set_xlabel("lap fraction")
    ax[1].set_ylabel("deploy electrical power (kW)")
    ax[1].set_title("MGU-K deploy — capped at 350 kW in both tiers")
    ax[1].legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(_IMG / "ers_t2_tier_overlay.png", dpi=120)
    plt.close(fig)
    print("wrote", _IMG / "ers_t2_tier_overlay.png")


def _fig_battery_2rc() -> None:
    # The double-exponential Thevenin step response the 2-RC pack integrates exactly, vs the 1-RC
    # arc the shipped f1 pack extends. Parameters are the f1_es cell-scale values used in PR4.
    ocv, r0, r1, tau1, r2, tau2, i = 3.675, 2.0e-4, 1.0e-4, 8.0, 0.4e-4, 45.0, 60.0
    t = np.linspace(0, 120, 400)
    one_rc = ocv - i * r0 - i * r1 * (1 - np.exp(-t / tau1))
    two_rc = one_rc - i * r2 * (1 - np.exp(-t / tau2))
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    ax.plot(t, one_rc * 1e3, color=REF, lw=1.8, label="1 RC pair (fast arc)")
    ax.plot(t, two_rc * 1e3, color=DEPLOY, lw=1.8, label="2 RC pairs (fast + slow arc)")
    ax.fill_between(t, two_rc * 1e3, one_rc * 1e3, color=DEPLOY, alpha=0.12)
    ax.set_xlabel("time under a constant-current pulse (s)")
    ax.set_ylabel("cell terminal voltage (mV)")
    ax.set_title("Battery ECM completion — the optional 2nd RC pair (battery/1.2)")
    ax.legend(loc="upper right", fontsize=9)
    ax.text(
        60,
        (two_rc[200]) * 1e3,
        "  the slow diffusion arc (τ₂ = 45 s)\n  the fast arc alone misses",
        color=DEPLOY,
        va="top",
        fontsize=8,
    )
    fig.tight_layout()
    fig.savefig(_IMG / "battery_2rc.png", dpi=120)
    plt.close(fig)
    print("wrote", _IMG / "battery_2rc.png")


def _fig_override(base, over) -> None:
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    for lab, d, col in (("rule-based", base, REF), ("override", over, DEPLOY)):
        soc = d["state_of_charge"].to_numpy() * 100.0
        x = np.linspace(0, 1, len(soc))
        ax[0].plot(x, soc, color=col, lw=1.6, label=lab)
    ax[0].set_xlabel("lap fraction")
    ax[0].set_ylabel("state of charge (%)")
    ax[0].set_title("Override ('Overtake') vs the rule-based lap — SoC")
    ax[0].legend(loc="best", fontsize=9)

    for lab, d, col in (("rule-based", base, REF), ("override", over, DEPLOY)):
        dep = _deploy_kw(d)
        x = np.linspace(0, 1, len(dep))
        ax[1].plot(x, dep, color=col, lw=1.0, label=lab)
    ax[1].set_xlabel("lap fraction")
    ax[1].set_ylabel("deploy electrical power (kW)")
    ax[1].set_title("Deploy — the override envelope reaches higher speed")
    ax[1].legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(_IMG / "ers_t2_override.png", dpi=120)
    plt.close(fig)
    print("wrote", _IMG / "ers_t2_override.png")


def main() -> None:
    _IMG.mkdir(parents=True, exist_ok=True)
    _fig_battery_2rc()
    base = _t2_lap()
    _fig_t2_lap(base)
    _fig_tier_overlay(base)
    over = _t2_lap(override=True)
    _fig_override(base, over)


if __name__ == "__main__":
    main()
