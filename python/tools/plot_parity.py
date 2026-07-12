# SPDX-License-Identifier: AGPL-3.0-only
"""Render the QSS↔T2 parity/decomposition figure (docs/validation/img/t2_parity.png).

Drives everything from the real solvers on the Limebeer car / Catalunya (flat): the T0 point-mass
profile, the T2 closed-loop lap, and the T1 g-g-g-v envelope. Two panels:
  (a) speed profiles — T0 (reference) vs T2 (corner-scaled stability margin: full speed on the
      straights, 0.85 at the lateral limit; the remaining corner gap is recorded, not gated).
  (b) hull containment — the T2 (a_x, a_y) operating points against the T1 envelope boundary at a
      representative speed: they sit inside the hull (the ASSERTED physics-parity gate).

Run from anywhere:  python python/tools/plot_parity.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import (
    Track,
    min_curvature,
    solve_lap,
    solve_lap_dataset,
    solve_transient_lap,
    transient_lap_dataset,
)

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "validation" / "img" / "t2_parity.png"
_CAR = str(_ROOT / "data" / "vehicles" / "limebeer_2014_f1")
_TRACK = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_SIM: dict[str, object] = {"flat_track": True}
_G = 9.80665


def main() -> None:
    track = Track.load(_TRACK)
    rl = min_curvature(track, 1.1)

    t0 = solve_lap_dataset(_CAR, rl, tier="t0", sim=_SIM)
    t1 = solve_lap(_CAR, rl.line(), tier="t1", sim=_SIM, raceline_ds_m=rl.ds_m)
    env = t1.envelope
    assert env is not None
    t2 = transient_lap_dataset(
        solve_transient_lap(_CAR, rl.line(), raceline_ds_m=rl.ds_m, sim=_SIM)
    )
    t0_time = float(t0.attrs["lap_time_s"])
    t2_time = float(t2.attrs["lap_time_s"])

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13, 5.2))

    # (a) speed profiles.
    ax0.plot(
        t0.s.to_numpy(),
        t0.v.to_numpy(),
        color="#1f77b4",
        lw=1.3,
        label=f"T0 QSS ({t0_time:.1f} s)",
    )
    # T2 s starts at the seeded straight; wrap it onto [0, L] so the profiles overlay.
    s_t2 = t2["s"].to_numpy() % track.length()
    ax0.scatter(
        s_t2,
        t2["vx"].to_numpy(),
        s=1,
        color="#d62728",
        alpha=0.5,
        label=f"T2 transient ({t2_time:.1f} s, +{100 * (t2_time - t0_time) / t0_time:.0f}%)",
    )
    ax0.set_xlabel("arc length s [m]")
    ax0.set_ylabel("speed [m/s]")
    ax0.set_title(
        "(a) Speed: corner-scaled margin — full on straights, 0.85 at the limit"
    )
    ax0.legend(loc="upper right", fontsize=9)

    # (b) hull containment: T2 (ax, ay) points vs the envelope boundary at a representative speed.
    vx = t2["vx"].to_numpy()
    ax_t2 = t2["ax"].to_numpy()
    ay_t2 = t2["ay"].to_numpy()
    ax1.scatter(
        ax_t2, ay_t2, s=2, color="#d62728", alpha=0.25, label="T2 operating points"
    )
    # Envelope boundary at a mid speed (the funnel section the car spends most time near).
    v_ref = float(np.median(vx))
    axs = np.linspace(
        env.brake_limit(v_ref, _G) * -1.0, env.accel_limit(v_ref, _G), 120
    )
    ayb = np.array([env.ay_boundary(v_ref, a, _G) for a in axs])
    ax1.plot(axs, ayb, color="#1f77b4", lw=1.6, label=f"T1 envelope @ {v_ref:.0f} m/s")
    ax1.plot(axs, -ayb, color="#1f77b4", lw=1.6)
    ax1.set_xlabel("longitudinal accel a_x [m/s²]")
    ax1.set_ylabel("lateral accel a_y [m/s²]")
    ax1.set_title("(b) Hull containment: T2 points inside the T1 envelope (gated ≤2%)")
    ax1.legend(loc="upper right", fontsize=9)

    fig.tight_layout()
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}  (T0 {t0_time:.2f}s, T2 {t2_time:.2f}s)")


if __name__ == "__main__":
    main()
