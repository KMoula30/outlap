# SPDX-License-Identifier: AGPL-3.0-only
"""Render the battery + Vdc–SoC-coupling theory figure (docs/theory/img/battery_coupling.png).

Drives the figure from the **real** `outlap-qss` battery model: it runs the committed Rust example
`crates/outlap-qss/examples/battery_coupling.rs` (which advances the actual `Pack` and evaluates the
Vdc-coupled `T1Powertrain` maps) and plots its CSV output. Nothing here re-implements the model —
the only curve computed in Python is the closed-form Thevenin reference the integrator is checked
against.

Three panels:
  (a) the Rust Thevenin pulse response vs the closed-form `OCV − I·R0 − I·R1(1 − e^{−t/τ})`,
  (b) an SoC sweep of the committed pack: terminal voltage and the coupled drive-unit efficiency,
  (c) drive-unit efficiency vs DC-link voltage, with the in-grid band shaded so the below/above-grid
      linear extrapolation is visible.

Run from anywhere:  python python/tools/plot_battery_coupling.py
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "battery_coupling.png"


def _run_traces() -> tuple[dict[str, float], dict[str, list[tuple[float, float, float | None]]]]:
    """Run the Rust example and parse its CSV into a params dict + rows grouped by scenario."""
    proc = subprocess.run(
        ["cargo", "run", "-q", "-p", "outlap-qss", "--example", "battery_coupling"],
        cwd=_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    params: dict[str, float] = {}
    rows: dict[str, list[tuple[float, float, float | None]]] = {}
    for line in proc.stdout.splitlines():
        if line.startswith("#"):
            for tok in line[1:].split():
                k, _, v = tok.partition("=")
                params[k] = float(v)
        elif line and not line.startswith("scenario"):
            scen, x, a, b = (line.split(",") + [""])[:4]
            rows.setdefault(scen, []).append((float(x), float(a), float(b) if b else None))
    return params, rows


def main() -> None:
    params, rows = _run_traces()
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))

    # (a) Rust Thevenin pulse vs the closed form.
    t = np.array([r[0] for r in rows["pulse"]])
    v = np.array([r[1] for r in rows["pulse"]])
    ocv, r0, r1, tau, i = (params[k] for k in ("ocv", "r0", "r1", "tau", "i"))
    closed = ocv - i * r0 - i * r1 * (1.0 - np.exp(-t / tau))
    ax = axes[0]
    ax.plot(t, closed, "-", lw=2.4, label="closed-form Thevenin")
    ax.plot(t[::10], v[::10], "o", ms=5, label="Rust integrator")
    ax.set(xlabel="time [s]", ylabel="terminal voltage [V]", title="(a) pulse response vs closed form")
    ax.legend(loc="upper right")

    # (b) SoC sweep: terminal voltage (left) and coupled drive-unit efficiency (right).
    soc = np.array([r[0] for r in rows["sweep"]])
    vt = np.array([r[1] for r in rows["sweep"]])
    eta = np.array([r[2] for r in rows["sweep"]])
    ax = axes[1]
    ax.plot(soc, vt, "-", lw=2.2, color="tab:blue", label="terminal voltage [V]")
    ax.axhspan(730.0, 850.0, color="tab:green", alpha=0.12, label="DU Vdc grid 730–850 V")
    ax.set(xlabel="state of charge [-]", ylabel="terminal voltage [V]", title="(b) SoC → Vdc coupling")
    ax.invert_xaxis()  # discharge runs high→low SoC
    ax2 = ax.twinx()
    ax2.plot(soc, eta, "--", lw=1.8, color="tab:red", label="coupled DU η")
    ax2.set_ylabel("drive-unit efficiency [-]", color="tab:red")
    ax2.grid(False)
    ax.legend(loc="lower left")

    # (c) Efficiency vs Vdc: in-grid interpolation + below/above-grid linear extrapolation.
    vdc = np.array([r[0] for r in rows["extrap"]])
    e = np.array([r[1] for r in rows["extrap"]])
    lo, hi = params["vdc_lo"], params["vdc_hi"]
    ax = axes[2]
    ax.axvspan(lo, hi, color="tab:green", alpha=0.12, label=f"map grid {lo:.0f}–{hi:.0f} V")
    ax.plot(vdc, e, "-", lw=2.4, color="tab:purple")
    ax.plot(vdc[vdc < lo], e[vdc < lo], "o", ms=4, color="tab:orange", label="extrapolated (below)")
    ax.plot(vdc[vdc > hi], e[vdc > hi], "o", ms=4, color="tab:orange")
    ax.set(xlabel="DC-link voltage [V]", ylabel="drive-unit efficiency [-]", title="(c) Vdc-axis extrapolation")
    ax.legend(loc="lower right")

    fig.suptitle("Battery Thevenin + Vdc–SoC coupling — driven by the outlap-qss model", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")


if __name__ == "__main__":
    main()
