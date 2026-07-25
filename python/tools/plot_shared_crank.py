# SPDX-License-Identifier: AGPL-3.0-only
"""Render the D-M6-13 Layer-3 shared-crank figure (docs/theory/img/shared_crank.png).

Layer 3 makes the drivetrain node that both the ICE and the MGU-K output onto a real shared shaft.
The machine is bolted to the crank, so it turns at whatever speed the ENGAGED gear dictates — it
does not get to pick a ratio of its own.

Panels (a)-(c) are drawn entirely from COMMITTED data (the shipped `f1_2026` gear ladder, the ICE
and MGU-K `.ptm` torque envelopes), reproducing the two selection rules exactly as documented:

  before — the machine argmaxed `τ(ω)·ω` over the 8 gears, bounded only by its OWN 50 000 rpm map;
  after  — the machine sits at the gear that maximises the ICE's wheel force (the gear the traction
           ceiling already assumes), bounded by the 15 000 rpm crank.

  (a) crank speed vs road speed — the same physical shaft, modelled at two speeds at once;
  (b) the deploy mechanical ceiling — where the gear-referenced envelope bites;
  (c) the deploy wheel force — the `η·P/v` singularity replaced by a flat torque-limited launch.

Panel (d) runs a real T2 lap and shows the shift cut reaching the machine: through a gear change the
deploy force AND the pack draw go to zero together.

Run from anywhere:  python python/tools/plot_shared_crank.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_CAR = _ROOT / "data" / "vehicles" / "f1_2026"
_OUT = _ROOT / "docs" / "theory" / "img" / "shared_crank.png"

RPM_TO_RAD = np.pi / 30.0
DEPLOY_ELEC_W = 350_000.0  # FIA C5.2.7 electrical cap
ELEC_MECH = 0.97  # C5.2.14 seam
ETA_DRIVELINE = 0.95  # the shipped gearbox constant efficiency


def _envelope(ptm: Path, key: str = "max_torque_nm_vs_speed"):
    """A `.ptm` torque envelope as (omega_rad_s, torque_nm) breakpoints."""
    lim = yaml.safe_load(ptm.read_text())["limits"][key]
    return np.asarray(lim["speed_rpm"], float) * RPM_TO_RAD, np.asarray(lim["torque_nm"], float)


def main() -> None:
    spec = yaml.safe_load((_CAR / "vehicle.yaml").read_text())
    gb = next(
        c["coupler"]["gearbox"]
        for c in spec["drivetrain"]["couplers"]
        if "gearbox" in c["coupler"]
    )
    ratios = np.asarray(gb["ratios"], float) * gb["final_drive"]
    r_wheel = yaml.safe_load((_CAR / "tyr/slick.tyr.yaml").read_text())["mf61"]["UNLOADED_RADIUS"]

    ice_w, ice_t = _envelope(_CAR / "ptm/ice_v6.ptm.yaml")
    k_w, k_t = _envelope(_CAR / "ptm/mgu_k.ptm.yaml")
    ice_tau = lambda w: np.interp(w, ice_w, ice_t)  # noqa: E731
    k_tau = lambda w: np.interp(w, k_w, k_t)  # noqa: E731
    ice_wmax, k_wmax = ice_w[-1], k_w[-1]

    v = np.linspace(0.5, 95.0, 600)
    om = np.outer(v, ratios / r_wheel)  # (speed, gear) shaft speed

    # BEFORE: the machine's own argmax of mech power over the gears, on ITS envelope.
    k_ok = om <= k_wmax
    k_power = np.where(k_ok, k_tau(om) * om, -np.inf)
    before_gear = np.argmax(k_power, axis=1)
    before_om = om[np.arange(len(v)), before_gear]

    # AFTER: the gear that maximises the ICE's wheel force — the engaged gear.
    ice_ok = om <= ice_wmax
    ice_force = np.where(ice_ok, ice_tau(om) * ratios * ETA_DRIVELINE / r_wheel, -np.inf)
    after_gear = np.argmax(ice_force, axis=1)
    after_om = om[np.arange(len(v)), after_gear]

    cap_before = np.where(np.isfinite(k_power[np.arange(len(v)), before_gear]), k_tau(before_om) * before_om, 0.0)
    cap_after = np.where(after_om <= k_wmax, k_tau(after_om) * after_om, 0.0)
    p_cmd = DEPLOY_ELEC_W * ELEC_MECH
    # The T0 pedal-availability rule (the one the blessed golden runs) capped by the RATIO-INVARIANT
    # scalar `max(τ·ω)` over the whole map — speed-independent, so the force went as η·P/v.
    cap_t0_before = float(np.max(k_tau(k_w) * k_w))
    f_before = ETA_DRIVELINE * np.minimum(p_cmd, cap_t0_before) / np.maximum(v, 1.0)
    f_after = ETA_DRIVELINE * np.minimum(p_cmd, cap_after) / np.maximum(v, 1.0)

    fig, ax = plt.subplots(2, 2, figsize=(13.5, 9.0))
    fig.suptitle(
        "D-M6-13 Layer 3 — the MGU-K is welded to the crank (f1_2026, shipped data)",
        fontsize=14, fontweight="bold",
    )

    a = ax[0, 0]
    a.plot(v, before_om / RPM_TO_RAD / 1e3, lw=2, ls="--", color="#c0392b",
           label="before — machine's own best gear")
    a.plot(v, after_om / RPM_TO_RAD / 1e3, lw=2, color="#1f77b4", label="after — the engaged gear")
    a.axhline(15.0, color="k", lw=1, ls=":", label="crank limiter, 15 000 rpm")
    a.set(xlabel="road speed [m/s]", ylabel="machine shaft speed [krpm]",
          title="(a) one shaft — but two speeds, before")
    a.legend(loc="upper left", fontsize=9)

    a = ax[0, 1]
    a.plot(v, cap_before / 1e3, lw=2, ls="--", color="#c0392b", label="before — own best gear")
    a.plot(v, cap_after / 1e3, lw=2, color="#1f77b4", label="after — engaged gear")
    a.axhline(p_cmd / 1e3, color="k", lw=1, ls=":", label="commanded 350 kW × 0.97")
    a.set(xlabel="road speed [m/s]", ylabel="deploy mech ceiling τ(ω)·ω [kW]",
          title="(b) the gear-referenced cap can only reduce")
    a.legend(loc="lower right", fontsize=9)

    a = ax[1, 0]
    a.plot(v, f_before / 1e3, lw=2, ls="--", color="#c0392b",
           label=f"before — flat {cap_t0_before / 1e3:.0f} kW cap, force = η·P/v")
    a.plot(v, f_after / 1e3, lw=2, color="#1f77b4", label="after — τ·ratio·η/r at the engaged gear")
    a.set(xlabel="road speed [m/s]", ylabel="MGU-K wheel force [kN]", ylim=(0, 60), xlim=(0, 40),
          title="(c) T0 pedal availability: a torque-limited launch,\nnot a 1/v singularity")
    a.legend(loc="upper right", fontsize=9)

    a = ax[1, 1]
    try:
        from outlap.core import Track, solve_transient_lap, transient_lap_dataset

        ds = transient_lap_dataset(
            solve_transient_lap(
                str(_CAR), Track.load(str(_ROOT / "data/tracks/catalunya_osm")), ds_m=12.0,
                sim={"flat_track": True, "envelope": {"v_points": 8, "ax_points": 7,
                     "g_normal_points": 2}},
            )
        )
        t = np.asarray(ds.coords["time"].to_numpy(), float)
        ts = ds["torque_scale"].to_numpy()
        cut = np.flatnonzero(ts <= 0.0)
        mid = cut[len(cut) // 2] if cut.size else len(t) // 2
        lo, hi = max(mid - 120, 0), min(mid + 120, len(t))
        sl = slice(lo, hi)
        a.plot(t[sl], ts[sl], lw=2, color="#7f7f7f", label="torque_scale (shift FSM)")
        a.plot(t[sl], ds["ers_deploy_force_n"].to_numpy()[sl] / 12e3, lw=2, color="#1f77b4",
               label="MGU-K deploy force / 12 kN")
        a.plot(t[sl], ds["traction_power_w"].to_numpy()[sl] / 350e3, lw=2, ls="--",
               color="#2ca02c", label="pack draw / 350 kW")
        a.set(xlabel="time [s]", ylabel="normalised", title="(d) the shift cut reaches the machine")
        a.legend(loc="center right", fontsize=9)
    except Exception as exc:  # pragma: no cover - the figure degrades, it never blocks
        a.text(0.5, 0.5, f"T2 lap unavailable:\n{exc}", ha="center", va="center", fontsize=9)
        a.set_axis_off()

    fig.tight_layout()
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=140)
    print(f"wrote {_OUT}")


if __name__ == "__main__":
    main()
