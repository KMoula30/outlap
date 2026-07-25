# SPDX-License-Identifier: AGPL-3.0-only
"""Render the D-M6-13 Layer-2 machine-ingestion figure (docs/theory/img/mgu_k_ingestion.png).

Layer 2 routes the governed MGU-K through its REAL data instead of the flat-0.97 / scalar-cap
shortcut. This figure is drawn entirely from the COMMITTED machine data — the dense efficiency/loss
parquet and the `.ptm` torque envelope — so it shows exactly what the solver now consumes.

Three panels:
  (a) the imported η(rpm, torque) map (drive + regen quadrants) — the efficiency the deploy force
      now uses, DERIVED from the resampled reference loss;
  (b) the deploy mech-power ceiling vs machine speed, LAYER 1 (before) vs LAYER 2 (this PR): the old
      ratio-invariant scalar cap let the machine deploy its peak power at every speed; Layer 2
      follows the real torque envelope, so the machine is torque-limited (can't make 350 kW) below
      the crossover — the physical reason the f1 deploy (and lap time) moved;
  (c) the torque envelope: drive vs regen (symmetric, peak ratio 1.0 per the reference).

Run from anywhere:  python python/tools/plot_machine_ingestion.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_PTM = _ROOT / "data" / "vehicles" / "f1_2026" / "ptm" / "mgu_k.ptm.yaml"
_PARQUET = _ROOT / "data" / "vehicles" / "f1_2026" / "ptm" / "tables" / "mgu_k.parquet"
_OUT = _ROOT / "docs" / "theory" / "img" / "mgu_k_ingestion.png"

# The FIA C5.2.8 deploy taper the manager drives the bus at (electrical, W). The machine converts to
# mech at η; the deploy force is limited by whichever bites — the electrical taper or the machine.
DEPLOY_ELEC_W = 350_000.0
# The old Layer-1 shortcut: flat regulatory efficiency + a ratio-invariant scalar mech-power cap.
LAYER1_ETA = 0.97


def _envelope(limits_key: str) -> tuple[np.ndarray, np.ndarray]:
    """The (speed_rpm, torque_nm) breakpoints of a `.ptm` torque envelope."""
    lim = yaml.safe_load(_PTM.read_text())["limits"][limits_key]
    return np.asarray(lim["speed_rpm"], float), np.asarray(lim["torque_nm"], float)


def main() -> None:
    df = pd.read_parquet(_PARQUET)
    speeds = np.sort(df["speed_rpm"].unique())
    torques = np.sort(df["torque_nm"].unique())
    eta = (
        df.pivot(index="torque_nm", columns="speed_rpm", values="efficiency")
        .reindex(index=torques, columns=speeds)
        .to_numpy()
    )

    drv_rpm, drv_nm = _envelope("max_torque_nm_vs_speed")
    reg_rpm, reg_nm = _envelope("max_regen_torque_nm_vs_speed")

    # Dense machine speed axis (rad/s from rpm) for the power-envelope panel.
    rpm = np.linspace(0.0, float(speeds.max()), 400)
    omega = rpm * (2.0 * np.pi / 60.0)
    tau_env = np.interp(rpm, drv_rpm, drv_nm)  # N·m, the real torque envelope

    # Layer 2 (this PR): deploy mech power = min(electrical taper · η, torque envelope · ω). At the
    # operating torque the machine sits on its envelope when deploy-limited; use η along that envelope.
    eta_on_env = np.array(
        [
            float(np.interp(t, torques, eta[:, j]))
            for j, t in enumerate(np.interp(speeds, drv_rpm, drv_nm))
        ]
    )
    eta_env_dense = np.interp(rpm, speeds, eta_on_env)
    p_machine_mech = tau_env * omega  # torque-envelope mech ceiling (W)
    p_taper_mech = DEPLOY_ELEC_W * np.clip(eta_env_dense, 0.0, 1.0)  # electrical-taper mech (W)
    layer2_mech = np.minimum(p_machine_mech, p_taper_mech)

    # Layer 1 (before): a single ratio-invariant scalar cap = the machine's PEAK mech power, applied
    # at every speed with a flat 0.97 — so it deployed peak power even where the torque envelope can't.
    p_peak = float(np.max(tau_env * omega))
    layer1_mech = np.minimum(DEPLOY_ELEC_W * LAYER1_ETA, p_peak) * np.ones_like(rpm)

    fig, (axa, axb, axc) = plt.subplots(1, 3, figsize=(15.5, 4.6))
    fig.suptitle(
        "D-M6-13 Layer 2 — the f1_2026 MGU-K fully ingested (from the committed machine data)",
        fontsize=13,
        weight="bold",
    )

    # (a) η heatmap.
    im = axa.pcolormesh(
        speeds / 1000.0, torques, np.clip(eta, 0.0, 1.0), cmap="viridis", shading="nearest"
    )
    axa.plot(drv_rpm / 1000.0, drv_nm, "w--", lw=1.4, label="drive envelope")
    axa.plot(reg_rpm / 1000.0, -reg_nm, "w:", lw=1.4, label="regen envelope")
    axa.axhline(0.0, color="w", lw=0.6, alpha=0.5)
    axa.set_xlabel("machine speed [krpm]")
    axa.set_ylabel("torque [N·m]  (+ drive / − regen)")
    axa.set_title("(a) imported η(rpm, τ) map")
    axa.legend(loc="lower right", fontsize=8, framealpha=0.7)
    fig.colorbar(im, ax=axa, label="efficiency", fraction=0.046, pad=0.04)

    # (b) deploy power ceiling: Layer 1 vs Layer 2 — the comparison to main.
    axb.plot(rpm / 1000.0, layer1_mech / 1e3, color="#c0392b", lw=2.2,
             label="Layer 1 (main): flat 0.97 · scalar cap")
    axb.plot(rpm / 1000.0, layer2_mech / 1e3, color="#1f6feb", lw=2.4,
             label="Layer 2 (this PR): real η · torque envelope")
    axb.plot(rpm / 1000.0, p_machine_mech / 1e3, color="#1f6feb", lw=1.0, ls="--", alpha=0.6,
             label="torque-envelope ceiling τ(ω)·ω")
    axb.axhline(DEPLOY_ELEC_W / 1e3, color="gray", lw=1.0, ls=":", label="350 kW manager taper")
    axb.fill_between(rpm / 1000.0, layer2_mech / 1e3, layer1_mech / 1e3,
                     where=layer1_mech > layer2_mech, color="#c0392b", alpha=0.10)
    axb.set_xlabel("machine speed [krpm]")
    axb.set_ylabel("deploy mech power ceiling [kW]")
    axb.set_title("(b) deploy ceiling — torque-limited below the crossover")
    axb.legend(loc="lower right", fontsize=8, framealpha=0.7)
    axb.set_ylim(0, 400)

    # (c) torque envelope: drive vs regen.
    axc.plot(drv_rpm / 1000.0, drv_nm, color="#1f6feb", lw=2.2, marker="o", ms=4, label="drive")
    axc.plot(reg_rpm / 1000.0, -reg_nm, color="#e67e22", lw=2.2, marker="s", ms=4, label="regen")
    axc.axhline(0.0, color="k", lw=0.6)
    axc.set_xlabel("machine speed [krpm]")
    axc.set_ylabel("torque envelope [N·m]")
    axc.set_title("(c) torque envelope — symmetric (peak ratio 1.0)")
    axc.legend(loc="upper right", fontsize=9, framealpha=0.7)

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT.relative_to(_ROOT)}")
    print(f"  peak machine mech power = {p_peak / 1e3:.0f} kW; Layer-1 flat cap = "
          f"{min(DEPLOY_ELEC_W * LAYER1_ETA, p_peak) / 1e3:.0f} kW")


if __name__ == "__main__":
    main()
