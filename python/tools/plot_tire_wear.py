# SPDX-License-Identifier: AGPL-3.0-only
"""Render the tire wear / thermal-damage theory figure (docs/theory/img/tire_wear.png).

Drives the figure from the **real** `outlap-tire` ring: it runs the committed Rust example
`crates/outlap-tire/examples/wear_cliff.rs` (which advances the actual `TireThermalRing` with its
§7.3 wear/damage states) and plots its CSV output. Nothing here re-implements the model.

Three panels:
  (a) a hard stint from new tires — tread wear w(t) and irreversible thermal damage D(t),
  (b) the grip factors vs wear — the total grip, the C¹ cliff, and the thermal-damage factor,
  (c) the C_s(w) positive feedback — fresh vs worn surface temperature under a corner/straight load.

Run from anywhere:  python python/tools/plot_tire_wear.py
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "tire_wear.png"


def _run_traces() -> tuple[dict[str, float], dict[str, np.ndarray]]:
    """Run the Rust example and parse its CSV into a params dict + per-scenario arrays."""
    proc = subprocess.run(
        ["cargo", "run", "-q", "-p", "outlap-tire", "--example", "wear_cliff"],
        cwd=_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    params: dict[str, float] = {}
    rows: dict[str, list[tuple[float, ...]]] = {}
    for line in proc.stdout.splitlines():
        if line.startswith("#"):
            for tok in line[1:].split():
                k, _, v = tok.partition("=")
                params[k] = float(v)
        elif line and not line.startswith("scenario"):
            parts = line.split(",")
            scen = parts[0]
            vals = tuple(float(p) if p else np.nan for p in parts[1:])
            rows.setdefault(scen, []).append(vals)
    arrays = {k: np.array(v) for k, v in rows.items()}
    return params, arrays


def main() -> None:
    params, rows = _run_traces()
    w_c, w_max = params["w_c"], params["w_max"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))

    # (a) A hard stint from new tires: tread wear grows (Archard) and, once the carcass runs hot,
    #     irreversible thermal damage accumulates.
    stint = rows["stint"]
    t, wear, dmg = stint[:, 0], stint[:, 1], stint[:, 2]
    ax = axes[0]
    ax.axhline(w_c, ls=":", color="0.4", lw=1.4)
    ax.annotate(
        "cliff onset $w_c$",
        (t[-1], w_c),
        xytext=(t[-1] * 0.55, w_c + 0.25),
        color="0.3",
    )
    ax.plot(t, wear, "-", lw=2.4, color="tab:red", label="tread wear $w$")
    ax.axhline(w_max, ls="--", color="tab:red", lw=1.0, alpha=0.5)
    ax.set(
        xlabel="time [s]",
        ylabel="tread wear $w$ [mm]",
        title="(a) wear + damage over a stint",
    )
    ax.set_ylim(0, w_max * 1.08)
    ax2 = ax.twinx()
    ax2.plot(t, dmg, "-", lw=2.0, color="tab:purple", label="thermal damage $D$")
    ax2.set_ylabel("thermal damage $D$ [-]", color="tab:purple")
    ax2.set_ylim(0, 1.05)
    ax2.grid(False)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="center right", fontsize=9)

    # (b) The grip factors swept over wear: the total grip and its C¹ cliff sigmoid.
    grip = rows["grip"]
    w, total, cliff, dmgf = grip[:, 0], grip[:, 1], grip[:, 2], grip[:, 3]
    ax = axes[1]
    ax.axvline(w_c, ls=":", color="0.4", lw=1.4)
    ax.annotate("$w_c$", (w_c, 0.66), xytext=(w_c + 0.15, 0.66), color="0.3")
    ax.plot(
        w,
        cliff,
        "-",
        lw=2.0,
        color="tab:orange",
        label=r"cliff factor $1-\Delta_c\sigma$",
    )
    ax.plot(
        w, dmgf, "--", lw=1.8, color="tab:purple", label=r"damage factor $1-\Delta_D D$"
    )
    ax.plot(
        w,
        total,
        "-",
        lw=2.6,
        color="tab:red",
        label=r"total grip $\lambda_{\mu,\mathrm{total}}$",
    )
    ax.set(
        xlabel="tread wear $w$ [mm]",
        ylabel="grip factor [-]",
        title="(b) the grip cliff (C¹ in wear)",
    )
    ax.set_ylim(0.55, 1.02)
    ax.legend(loc="lower left", fontsize=9)

    # (c) The C_s(w) positive feedback: a worn tire has less surface capacity, so under a
    #     corner/straight load oscillation it swings wider — higher peaks that tip it out of window.
    fb = rows["feedback"]
    ft, ts_fresh, ts_worn = fb[:, 0], fb[:, 1], fb[:, 2]
    ax = axes[2]
    ax.plot(ft, ts_fresh, "-", lw=2.0, color="tab:blue", label="fresh tire ($w=0$)")
    ax.plot(
        ft, ts_worn, "-", lw=2.0, color="tab:red", label="worn tire ($w=0.9\\,w_{max}$)"
    )
    ax.axhline(ts_worn.max(), ls=":", color="tab:red", lw=1.0, alpha=0.6)
    ax.axhline(ts_fresh.max(), ls=":", color="tab:blue", lw=1.0, alpha=0.6)
    ax.set(
        xlabel="time [s]",
        ylabel="surface temperature $T_s$ [°C]",
        title="(c) $C_s(w)$ feedback: worn runs hotter peaks",
    )
    ax.legend(loc="upper right", fontsize=9)

    fig.suptitle(
        "Tire wear / thermal damage (§7.3) — driven by the outlap-tire integrator",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")


if __name__ == "__main__":
    main()
