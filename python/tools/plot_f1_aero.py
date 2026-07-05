# SPDX-License-Identifier: AGPL-3.0-only
"""Plot the synthetic F1 ride-height/yaw aero map + the platform equilibrium (§7.4, PR3).

Reads the committed ``data/vehicles/f1_2026/aero/f1_2026.parquet`` and renders a 2×2 figure to
``docs/theory/img/t1_aero_map.png``:

    (a) ground effect vs front ride height       (b) rake vs rear ride height
    (c) yaw (even) sensitivity + DRS effect       (d) aero-platform equilibrium vs speed

Panel (d) mirrors ``AeroPlatform::equilibrium`` (outlap-qss) so the committed figure matches the
Rust trim's behaviour. Run from anywhere:  ``python python/tools/plot_f1_aero.py``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pyarrow.parquet as pq

_ROOT = Path(__file__).resolve().parents[2]
_PARQUET = _ROOT / "data" / "vehicles" / "f1_2026" / "aero" / "f1_2026.parquet"
_OUT = _ROOT / "docs" / "theory" / "img" / "t1_aero_map.png"

# Platform parameters (match the f1_2026 vehicle + AeroPlatform::equilibrium).
RHO = 1.2
K_F, K_R = 220_000.0, 240_000.0
H_F0, H_R0 = 0.040, 0.090  # static ride heights, m
DAMP, ITERS = 0.6, 60

_TBL = pq.read_table(_PARQUET).to_pandas()
RF = sorted(_TBL.ride_height_f_mm.unique())
RR = sorted(_TBL.ride_height_r_mm.unique())
YAW = sorted(_TBL.yaw_deg.unique())


def node(hf: float, hr: float, yaw: float, drs: float, col: str) -> float:
    """Exact grid-node lookup."""
    row = _TBL[
        np.isclose(_TBL.ride_height_f_mm, hf)
        & np.isclose(_TBL.ride_height_r_mm, hr)
        & np.isclose(_TBL.yaw_deg, yaw)
        & np.isclose(_TBL.drs_flag, drs)
    ]
    return float(row[col].iloc[0])


def bilinear(hf_mm: float, hr_mm: float, col: str) -> float:
    """Bilinear interp in (hf, hr) at yaw 0 / DRS closed for the equilibrium trace."""
    hf_mm = float(np.clip(hf_mm, RF[0], RF[-1]))
    hr_mm = float(np.clip(hr_mm, RR[0], RR[-1]))
    i = min(max(np.searchsorted(RF, hf_mm) - 1, 0), len(RF) - 2)
    j = min(max(np.searchsorted(RR, hr_mm) - 1, 0), len(RR) - 2)
    tx = (hf_mm - RF[i]) / (RF[i + 1] - RF[i])
    ty = (hr_mm - RR[j]) / (RR[j + 1] - RR[j])
    c00 = node(RF[i], RR[j], 0.0, 0.0, col)
    c10 = node(RF[i + 1], RR[j], 0.0, 0.0, col)
    c01 = node(RF[i], RR[j + 1], 0.0, 0.0, col)
    c11 = node(RF[i + 1], RR[j + 1], 0.0, 0.0, col)
    return (
        c00 * (1 - tx) * (1 - ty)
        + c10 * tx * (1 - ty)
        + c01 * (1 - tx) * ty
        + c11 * tx * ty
    )


def equilibrium(v: float) -> tuple[float, float, float, float]:
    """Return (h_front_mm, h_rear_mm, cz_front, cz_rear) at the platform equilibrium (a_x = 0)."""
    hf, hr = H_F0, H_R0
    qdyn = 0.5 * RHO * v * v
    for _ in range(ITERS):
        czf = bilinear(hf * 1000, hr * 1000, "cz_front_a_m2")
        czr = bilinear(hf * 1000, hr * 1000, "cz_rear_a_m2")
        hf += DAMP * (max(H_F0 - qdyn * czf / (2 * K_F), 0.0) - hf)
        hr += DAMP * (max(H_R0 - qdyn * czr / (2 * K_R), 0.0) - hr)
    return (
        hf * 1000,
        hr * 1000,
        bilinear(hf * 1000, hr * 1000, "cz_front_a_m2"),
        bilinear(hf * 1000, hr * 1000, "cz_rear_a_m2"),
    )


def main() -> None:
    plt.style.use("seaborn-v0_8-darkgrid")
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    fig.suptitle(
        "Reference F1 2026 — synthetic ride-height / yaw aero map (PR3)\n"
        "coefficients = C·A (m²); anchored at h_f=30 mm, h_r=70 mm -> (1.9, 2.6, 1.25)",
        fontsize=12,
    )

    ax = axes[0, 0]
    for col, mk, lab in [
        ("cz_front_a_m2", "o-", "Cz_front·A"),
        ("cz_rear_a_m2", "s-", "Cz_rear·A"),
        ("cx_a_m2", "^-", "Cx·A"),
    ]:
        ax.plot(RF, [node(h, 70, 0, 0, col) for h in RF], mk, label=lab)
    ax.axvline(30, ls=":", c="k", alpha=0.5)
    ax.set_xlabel("front ride height (mm)")
    ax.set_ylabel("coefficient × area (m²)")
    ax.set_title(
        "(a) ground effect vs front ride height\n(rear 70 mm, yaw 0, DRS closed)"
    )
    ax.legend()

    ax = axes[0, 1]
    for col, mk, lab in [
        ("cz_front_a_m2", "o-", "Cz_front·A"),
        ("cz_rear_a_m2", "s-", "Cz_rear·A"),
        ("cx_a_m2", "^-", "Cx·A"),
    ]:
        ax.plot(RR, [node(30, h, 0, 0, col) for h in RR], mk, label=lab)
    ax.axvline(70, ls=":", c="k", alpha=0.5)
    ax.set_xlabel("rear ride height (mm)")
    ax.set_ylabel("coefficient × area (m²)")
    ax.set_title("(b) rake vs rear ride height\n(front 30 mm, yaw 0, DRS closed)")
    ax.legend()

    ax = axes[1, 0]
    ax.plot(
        YAW,
        [node(30, 70, y, 0, "cz_rear_a_m2") for y in YAW],
        "s-",
        label="Cz_rear·A (DRS closed)",
    )
    ax.plot(
        YAW,
        [node(30, 70, y, 1, "cz_rear_a_m2") for y in YAW],
        "s--",
        label="Cz_rear·A (DRS open)",
    )
    ax.plot(
        YAW,
        [node(30, 70, y, 0, "cx_a_m2") for y in YAW],
        "^-",
        label="Cx·A (DRS closed)",
    )
    ax.plot(
        YAW,
        [node(30, 70, y, 1, "cx_a_m2") for y in YAW],
        "^--",
        label="Cx·A (DRS open)",
    )
    ax.set_xlabel("yaw angle (deg)")
    ax.set_ylabel("coefficient × area (m²)")
    ax.set_title("(c) yaw (even) sensitivity + DRS effect\n(30 mm / 70 mm)")
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    vs = np.linspace(15, 95, 40)
    eq = [equilibrium(v) for v in vs]
    hf = [e[0] for e in eq]
    hr = [e[1] for e in eq]
    balance = [100 * e[2] / (e[2] + e[3]) for e in eq]
    ax.plot(vs, hf, label="front ride height (mm)")
    ax.plot(vs, hr, label="rear ride height (mm)")
    ax2 = ax.twinx()
    ax2.plot(vs, balance, "r-.", label="front DF share (%)")
    ax2.set_ylabel("front downforce share (%)", color="r")
    ax2.tick_params(axis="y", labelcolor="r")
    ax.set_xlabel("speed (m/s)")
    ax.set_ylabel("ride height (mm)")
    ax.set_title(
        "(d) aero-platform equilibrium vs speed\n(platform sinks & rakes -> balance shifts)"
    )
    ax.legend(loc="center right", fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=110)
    print(f"wrote {_OUT}")


if __name__ == "__main__":
    main()
