# SPDX-License-Identifier: AGPL-3.0-only
"""Render the tire-thermal-ring theory figure (docs/theory/img/tire_thermal.png).

Drives the figure from the **real** `outlap-tire` ring: it runs the committed Rust example
`crates/outlap-tire/examples/thermal_ring.rs` (which advances the actual `TireThermalRing`) and
plots its CSV output. Nothing here re-implements the model.

Three panels:
  (a) cold-start warm-up of the three nodes (surface / carcass / gas) — the ring's dynamics,
  (b) the three force-model couplings vs temperature (grip window, carcass stiffness, hot pressure),
  (c) the steady surface temperature vs sliding-power load at two speeds — the energy balance.

Run from anywhere:  python python/tools/plot_tire_thermal.py
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "tire_thermal.png"


def _run_traces() -> tuple[dict[str, float], dict[str, np.ndarray]]:
    """Run the Rust example and parse its CSV into a params dict + per-scenario arrays."""
    proc = subprocess.run(
        ["cargo", "run", "-q", "-p", "outlap-tire", "--example", "thermal_ring"],
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
    t_opt = params["t_opt_c"]
    lo, hi = params["window_lo"], params["window_hi"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))

    # (a) Warm-up of the three nodes from a cold start under a constant hard-cornering load.
    warm = rows["warm"]
    t, ts, tc, tg = warm[:, 0], warm[:, 1], warm[:, 2], warm[:, 3]
    ax = axes[0]
    ax.axhspan(lo, hi, color="tab:green", alpha=0.12, label="working window")
    ax.plot(t, ts, "-", lw=2.2, color="tab:red", label="surface $T_s$")
    ax.plot(t, tc, "-", lw=2.0, color="tab:orange", label="carcass $T_c$")
    ax.plot(t, tg, "-", lw=2.0, color="tab:blue", label="gas $T_g$")
    ax.set(xlabel="time [s]", ylabel="temperature [°C]", title="(a) cold-start warm-up")
    ax.legend(loc="lower right")

    # (b) The three force-model couplings swept over node temperature.
    cp = rows["couple"]
    temp, lam, stiff, pres = cp[:, 0], cp[:, 1], cp[:, 2], cp[:, 3]
    ax = axes[1]
    ax.plot(temp, lam, "-", lw=2.4, color="tab:red", label=r"grip window $\lambda_\mu(T_s)$")
    ax.plot(temp, stiff, "-", lw=2.0, color="tab:orange", label=r"stiffness $1-k_c\Delta T_c$")
    ax.axvline(t_opt, ls=":", color="0.4", lw=1.5)
    ax.annotate(r"$T_\mathrm{opt}$", (t_opt, 0.06), xytext=(t_opt + 4, 0.12), color="0.3")
    ax.set(xlabel="node temperature [°C]", ylabel="grip / stiffness factor [-]",
           title="(b) couplings back to the force model", ylim=(0.0, 1.08))
    ax2 = ax.twinx()
    ax2.plot(temp, pres, "--", lw=1.8, color="tab:blue", label="hot pressure $p(T_g)$")
    ax2.set_ylabel("inflation pressure [kPa]", color="tab:blue")
    ax2.grid(False)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    # (c) Steady surface temperature vs sliding-power load, at two speeds (the energy balance).
    st = rows["steady"]
    kw, ts40, ts70 = st[:, 0], st[:, 1], st[:, 2]
    ax = axes[2]
    ax.axhspan(lo, hi, color="tab:green", alpha=0.12, label="working window")
    ax.plot(kw, ts40, "-o", ms=3, lw=2.0, color="tab:red", label="v = 40 m/s")
    ax.plot(kw, ts70, "-o", ms=3, lw=2.0, color="tab:blue", label="v = 70 m/s (more cooling)")
    ax.set(xlabel="sliding-power load [kW]", ylabel="steady surface $T_s$ [°C]",
           title="(c) load ↔ cooling energy balance")
    ax.legend(loc="upper left")

    fig.suptitle(
        "Tire thermal ring (reduced Farroni-TRT) — driven by the outlap-tire integrator", fontsize=13
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")


if __name__ == "__main__":
    main()
