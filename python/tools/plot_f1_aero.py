# SPDX-License-Identifier: AGPL-3.0-only
"""Plot the synthetic F1 ride-height/yaw aero map (§7.4, PR3).

Reads the committed ``data/vehicles/f1_2026/aero/f1_2026.parquet`` and renders two figures:

  * ``docs/theory/img/t1_aero_map.png`` — 1-D slices + the platform equilibrium vs speed:
      (a) ground effect vs front ride height   (b) rake vs rear ride height
      (c) yaw (even) sensitivity + DRS effect    (d) aero-platform equilibrium vs speed
  * ``docs/theory/img/t1_aero_map_2d.png`` — the classic 2-D ride-height maps: Front / Rear / Total
      downforce and Drag over the rear-RH × front-RH plane (yaw 0, DRS closed), with the platform
      equilibrium operating locus overlaid on the total-downforce panel.

Panel (d) and the 2-D locus mirror ``AeroPlatform::equilibrium`` (outlap-qss) so the committed
figures match the Rust trim's behaviour. Run from anywhere:  ``python python/tools/plot_f1_aero.py``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pyarrow.parquet as pq

_ROOT = Path(__file__).resolve().parents[2]
_PARQUET = _ROOT / "data" / "vehicles" / "f1_2026" / "aero" / "f1_2026.parquet"
_OUT_SLICES = _ROOT / "docs" / "theory" / "img" / "t1_aero_map.png"
_OUT_MAPS = _ROOT / "docs" / "theory" / "img" / "t1_aero_map_2d.png"

# Platform parameters (match the f1_2026 vehicle + AeroPlatform::equilibrium).
RHO = 1.2
K_F, K_R = 220_000.0, 240_000.0
H_F0, H_R0 = 0.040, 0.090  # static ride heights, m
DAMP, ITERS = 0.6, 60

# Anchor + sensitivities (identical to gen_f1_aero.py); the smooth form the parquet samples.
REF_HF, REF_HR = 30.0, 70.0
CZF0, CZR0, CX0 = 1.9, 2.6, 1.25
A_FF, A_FR, A_RR, A_RF, A_XF, A_XR = 0.35, 0.10, 0.30, 0.05, 0.05, 0.05

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


def coeffs(hf_mm, hr_mm):
    """Analytic (cz_front, cz_rear, cx)·A at yaw 0 / DRS closed (broadcasts; matches the nodes)."""
    df = (REF_HF - hf_mm) / REF_HF
    dr = (REF_HR - hr_mm) / REF_HR
    rake = (hr_mm - REF_HR) / REF_HR
    czf = CZF0 * (1.0 + A_FF * df + A_FR * rake)
    czr = CZR0 * (1.0 + A_RR * dr + A_RF * df)
    cx = CX0 * (1.0 + A_XF * df + A_XR * dr)
    return czf, czr, cx


def equilibrium(v: float) -> tuple[float, float, float, float]:
    """(h_front_mm, h_rear_mm, cz_front, cz_rear) at the platform equilibrium (a_x = 0)."""
    hf, hr = H_F0, H_R0
    qdyn = 0.5 * RHO * v * v
    for _ in range(ITERS):
        czf, czr, _ = coeffs(
            np.clip(hf * 1000, RF[0], RF[-1]), np.clip(hr * 1000, RR[0], RR[-1])
        )
        hf += DAMP * (max(H_F0 - qdyn * czf / (2 * K_F), 0.0) - hf)
        hr += DAMP * (max(H_R0 - qdyn * czr / (2 * K_R), 0.0) - hr)
    czf, czr, _ = coeffs(hf * 1000, hr * 1000)
    return hf * 1000, hr * 1000, czf, czr


def plot_slices() -> None:
    """1-D slices + the platform equilibrium vs speed."""
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
    _OUT_SLICES.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT_SLICES, dpi=110)
    print(f"wrote {_OUT_SLICES}")


def plot_maps() -> None:
    """The classic 2-D ride-height maps over the rear-RH × front-RH plane."""
    hf_axis = np.linspace(RF[0], RF[-1], 121)  # front RH, mm (y)
    hr_axis = np.linspace(RR[0], RR[-1], 121)  # rear RH, mm (x)
    hr_grid, hf_grid = np.meshgrid(hr_axis, hf_axis)
    czf, czr, cx = coeffs(hf_grid, hr_grid)
    panels = [
        ("Front downforce  Cz_front·A (m²)", czf),
        ("Rear downforce  Cz_rear·A (m²)", czr),
        ("Total downforce  (Cz_f+Cz_r)·A (m²)", czf + czr),
        ("Drag  Cx·A (m²)", cx),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8.6))
    fig.suptitle(
        "Reference F1 2026 — synthetic ride-height aero maps (§7.4, PR3)\n"
        "rear RH × front RH plane, yaw 0, DRS closed; anchored at (30, 70) mm -> (1.9, 2.6, 1.25)",
        fontsize=12,
    )

    vs = np.linspace(10, 95, 60)
    locus = np.array([equilibrium(v)[:2] for v in vs])  # (hf_mm, hr_mm)

    for ax, (title, field) in zip(axes.ravel(), panels, strict=True):
        pcm = ax.pcolormesh(hr_axis, hf_axis, field, cmap="jet", shading="gouraud")
        cs = ax.contour(hr_axis, hf_axis, field, colors="k", linewidths=0.4, alpha=0.5)
        ax.clabel(cs, inline=True, fontsize=6, fmt="%.2f")
        fig.colorbar(pcm, ax=ax, fraction=0.046, pad=0.03)
        if title.startswith("Total"):
            ax.plot(
                locus[:, 1],
                locus[:, 0],
                "w-",
                lw=2.2,
                label="platform equilibrium (10→95 m/s)",
            )
            ax.plot(REF_HR, REF_HF, "wo", ms=6, mec="k", label="anchor (30, 70) mm")
            ax.legend(loc="upper right", fontsize=7)
        ax.set_xlabel("rear ride height (mm)")
        ax.set_ylabel("front ride height (mm)")
        ax.set_title(title, fontsize=10)

    fig.tight_layout(rect=(0, 0, 1, 0.93))
    _OUT_MAPS.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT_MAPS, dpi=110)
    print(f"wrote {_OUT_MAPS}")


def main() -> None:
    plot_slices()
    plot_maps()


if __name__ == "__main__":
    main()
