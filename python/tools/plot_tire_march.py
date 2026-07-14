# SPDX-License-Identifier: AGPL-3.0-only
"""Render the M5 PR5 QSS tyre-thermal-march figures from the Rust example's CSVs.

    cargo run --release -p outlap-qss --features parallel --example tire_march_lap
    uv run --extra track-import python tools/plot_tire_march.py

Reads `scratch_figs/tire_march_{lap,axes}.csv` and writes two PNGs alongside them:

* `tire_march_lap.png`   — the closed-loop effect: frozen vs tyre-thermal speed, the ring
                           temperatures, the tread wear crossing the cliff, and the grip multiplier.
* `tire_march_axes.png`  — the tyre-state grip surface the QSS lap indexes (peak lateral grip vs
                           `T_tire` and `wear`), plus its two 1-D sections (the grip window and the
                           wear cliff).
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# --- design-system palette (validated categorical slots; text in ink tokens) -----------------
BLUE, AQUA, YELLOW, GREEN, VIOLET, RED = (
    "#2a78d6",
    "#1baf7a",
    "#eda100",
    "#008300",
    "#4a3aa7",
    "#e34948",
)
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e7e6e2", "#fcfcfb"

mpl.rcParams.update(
    {
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": MUTED,
        "axes.labelcolor": INK,
        "axes.titlecolor": INK,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "text.color": INK,
        "font.size": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
    }
)

ROOT = Path(__file__).resolve().parents[2]
FIGS = ROOT / "scratch_figs"  # the example's CSV drop
IMG = ROOT / "docs" / "theory" / "img"  # the committed figure location


def _read(path: Path) -> dict[str, np.ndarray]:
    with path.open() as fh:
        rows = list(csv.DictReader(fh))
    cols = rows[0].keys()
    return {c: np.array([float(r[c]) for r in rows]) for c in cols}


def plot_lap() -> None:
    d = _read(FIGS / "tire_march_lap.csv")
    s_km = d["s"] / 1000.0
    fig, ax = plt.subplots(2, 2, figsize=(12, 7.5))
    fig.suptitle(
        "QSS tyre-thermal march — Limebeer F1 on Catalunya (T0, warm-seed at grip optimum)",
        fontsize=13,
        fontweight="bold",
        y=0.98,
    )

    # (a) speed: frozen vs tyre-thermal.
    a = ax[0, 0]
    a.plot(s_km, d["v_frozen"], color=MUTED, lw=1.6, label="frozen envelope")
    a.plot(s_km, d["v_tire"], color=BLUE, lw=1.8, label="tyre-thermal (marched)")
    a.set_ylabel("speed  [m/s]")
    a.set_title("(a) the lap responds to tyre state", loc="left", fontsize=11)
    a.legend(frameon=False, fontsize=9, loc="lower center", ncol=2)

    # (b) ring temperatures.
    b = ax[0, 1]
    b.plot(s_km, d["t_surface_c"], color=RED, lw=1.8, label="surface $T_s$")
    b.plot(s_km, d["t_carcass_c"], color=YELLOW, lw=1.8, label="carcass $T_c$")
    b.plot(s_km, d["t_gas_c"], color=BLUE, lw=1.8, label="gas $T_g$")
    b.set_ylabel("temperature  [°C]")
    b.set_title("(b) 3-node ring warms segment-to-segment", loc="left", fontsize=11)
    b.legend(frameon=False, fontsize=9, loc="center right")

    # (c) wear crossing the cliff.
    c = ax[1, 0]
    c.plot(s_km, d["wear_mm"], color=VIOLET, lw=1.8, label="tread wear $w$")
    c.axhline(2.0, color=MUTED, lw=1.0, ls="--")
    c.text(s_km[-1], 2.0, "  cliff $w_c$", color=MUTED, va="center", fontsize=9)
    c.set_ylabel("wear  [mm]")
    c.set_xlabel("distance  [km]")
    c.set_title("(c) Archard tread wear (illustrative $k_w$)", loc="left", fontsize=11)

    # (d) grip multiplier.
    e = ax[1, 1]
    e.plot(
        s_km, d["grip"], color=GREEN, lw=1.8, label=r"$\lambda_{\mu,\mathrm{total}}$"
    )
    e.set_ylabel(r"grip multiplier  $\lambda_{\mu,\mathrm{total}}$")
    e.set_xlabel("distance  [km]")
    e.set_ylim(0.7, 1.02)
    e.set_title(
        "(d) grip window × wear cliff feed the re-solve", loc="left", fontsize=11
    )

    for a in ax.flat:
        a.margins(x=0.01)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = IMG / "tire_march_lap.png"
    fig.savefig(out, dpi=130)
    print("wrote", out)


def plot_axes() -> None:
    d = _read(FIGS / "tire_march_axes.csv")
    t = np.unique(d["t_tire_c"])
    w = np.unique(d["wear_mm"])
    ay = d["ay_peak"].reshape(len(t), len(w))

    fig = plt.figure(figsize=(12, 4.6))
    fig.suptitle(
        "The tyre-state grip surface the QSS lap indexes  —  peak lateral grip $a_{y}$ "
        "vs $(T_{tire},\\ wear)$",
        fontsize=13,
        fontweight="bold",
        y=1.0,
    )
    gs = fig.add_gridspec(1, 3, width_ratios=[1.3, 1, 1], wspace=0.34)

    # (a) heatmap over both axes.
    ax0 = fig.add_subplot(gs[0])
    im = ax0.pcolormesh(w, t, ay, cmap="viridis", shading="gouraud")
    ax0.set_xlabel("wear  [mm]")
    ax0.set_ylabel("$T_{tire}$  [°C]")
    ax0.set_title("(a) $a_{y}(T_{tire}, wear)$", loc="left", fontsize=11)
    cb = fig.colorbar(im, ax=ax0)
    cb.set_label("peak $a_y$  [m/s²]")

    # (b) grip window: a_y vs T_tire at zero wear.
    ax1 = fig.add_subplot(gs[1])
    ax1.plot(t, ay[:, 0], color=RED, lw=1.9)
    ax1.set_xlabel("$T_{tire}$  [°C]")
    ax1.set_ylabel("peak $a_y$  [m/s²]")
    ax1.set_title("(b) grip window (wear = 0)", loc="left", fontsize=11)

    # (c) wear cliff: a_y vs wear at the optimum temperature (nearest column).
    jt = int(np.argmax(ay[:, 0]))
    ax2 = fig.add_subplot(gs[2])
    ax2.plot(w, ay[jt, :], color=VIOLET, lw=1.9)
    ax2.axvline(2.0, color=MUTED, lw=1.0, ls="--")
    ax2.text(2.0, ay[jt, :].min(), "  $w_c$", color=MUTED, va="bottom", fontsize=9)
    ax2.set_xlabel("wear  [mm]")
    ax2.set_ylabel("peak $a_y$  [m/s²]")
    ax2.set_title(f"(c) wear cliff (T ≈ {t[jt]:.0f} °C)", loc="left", fontsize=11)

    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = IMG / "tire_march_axes.png"
    fig.savefig(out, dpi=130)
    print("wrote", out)


if __name__ == "__main__":
    plot_lap()
    plot_axes()
