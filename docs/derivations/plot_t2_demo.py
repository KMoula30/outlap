# SPDX-License-Identifier: AGPL-3.0-only
"""Render the T2 skeleton demo figures for the PR / theory page from the example CSV traces.

Run the example first: `cargo run --release -p outlap-transient --example transient_lap`, then
`uv run --group notebooks --extra track-import python docs/derivations/plot_t2_demo.py`.
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CSV_DIR = ROOT / "debug_plots" / "t2"
IMG_DIR = ROOT / "docs" / "theory" / "img"
IMG_DIR.mkdir(parents=True, exist_ok=True)


def load(name: str) -> dict[str, list[float]]:
    with (CSV_DIR / name).open() as fh:
        rows = list(csv.DictReader(fh))
    cols: dict[str, list[float]] = {k: [] for k in rows[0]}
    for r in rows:
        for k, v in r.items():
            cols[k].append(float(v))
    return cols


def skidpad_figure() -> None:
    d = load("skidpad.csv")
    fig, ax = plt.subplots(1, 3, figsize=(13, 4.0))
    # Birds-eye trajectory (closed-loop tracking of the circle).
    sc = ax[0].scatter(d["x"], d["y"], c=d["vx"], s=4, cmap="viridis")
    ax[0].set_aspect("equal")
    ax[0].set_title("Closed-loop skidpad trajectory")
    ax[0].set_xlabel("x [m]")
    ax[0].set_ylabel("y [m]")
    fig.colorbar(sc, ax=ax[0], label="v_x [m/s]", shrink=0.8)
    # Lateral offset from the line stays bounded (driver tracks the reference).
    ax[1].plot(d["t"], d["n"], color="tab:blue")
    ax[1].axhline(0.0, color="k", lw=0.6, ls="--")
    ax[1].set_title("Tracking error n(t) stays bounded")
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("n [m]")
    # Per-wheel load transfer: outer (right, +y-loaded in a left turn) wheels gain load.
    ax[2].plot(d["t"], d["fz_fl"], label="FL")
    ax[2].plot(d["t"], d["fz_fr"], label="FR")
    ax[2].plot(d["t"], d["fz_rl"], label="RL")
    ax[2].plot(d["t"], d["fz_rr"], label="RR")
    ax[2].set_title("Per-wheel normal load (lateral transfer)")
    ax[2].set_xlabel("t [s]")
    ax[2].set_ylabel("F_z [N]")
    ax[2].legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(IMG_DIR / "t2_skidpad.png", dpi=110)
    plt.close(fig)


def coastdown_figure() -> None:
    d = load("coastdown.csv")
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.6))
    ax[0].plot(d["t"], d["vx"], color="tab:red", label="T2 integrator")
    ax[0].set_title("Coastdown v_x(t) under aero drag")
    ax[0].set_xlabel("t [s]")
    ax[0].set_ylabel("v_x [m/s]")
    ax[0].legend(fontsize=8)
    # Deceleration vs speed² (drag law a ≈ −q_x·v²/m + rolling): expect a monotone parabola-ish curve.
    ax[1].plot(d["vx"], [-a for a in d["ax"]], color="tab:red")
    ax[1].set_title("Deceleration vs speed (drag ∝ v²)")
    ax[1].set_xlabel("v_x [m/s]")
    ax[1].set_ylabel("−a_x [m/s²]")
    fig.tight_layout()
    fig.savefig(IMG_DIR / "t2_coastdown.png", dpi=110)
    plt.close(fig)


def step_steer_figure() -> None:
    d = load("step_steer.csv")
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    ax[0].plot(d["t"], d["r"], color="tab:green")
    ax[0].set_title("Yaw-rate step response r(t)")
    ax[0].set_xlabel("t [s]")
    ax[0].set_ylabel("r [rad/s]")
    ax[1].plot(d["t"], d["vy"], color="tab:purple")
    ax[1].set_title("Sideslip velocity v_y(t)")
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("v_y [m/s]")
    ax[2].plot(d["t"], d["slip_alpha_fl"], color="tab:orange")
    ax[2].set_title("Front-left lagged slip angle α(t)\n(relaxation to steady state)")
    ax[2].set_xlabel("t [s]")
    ax[2].set_ylabel("α [rad]")
    fig.tight_layout()
    fig.savefig(IMG_DIR / "t2_step_steer.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    skidpad_figure()
    coastdown_figure()
    step_steer_figure()
    print(f"wrote figures to {IMG_DIR}")
