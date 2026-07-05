# SPDX-License-Identifier: AGPL-3.0-only
"""Render the machine-thermal theory figure (docs/theory/img/machine_thermal.png).

Drives the figure from the **real** `outlap-thermal` integrator: it runs the committed Rust example
`crates/outlap-qss/examples/thermal_traces.rs` (which advances the actual `MachineThermal` on the
committed fixtures) and plots its CSV output. Nothing here re-implements the model — the only curve
computed in Python is the closed-form LTI reference the integrator is checked against.

Three panels:
  (a) the Rust Crank–Nicolson advance vs the analytic first-order LTI step response,
  (b) a stint on the committed `rear.emotor.yaml`: winding temperature rise → torque derate,
  (c) the air-gap film's speed-dependent cooling — rotor temperature vs time at two shaft speeds.

Run from anywhere:  python python/tools/plot_machine_thermal.py
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "machine_thermal.png"


def _run_traces() -> tuple[dict[str, float], dict[str, list[tuple[float, float, float | None]]]]:
    """Run the Rust example and parse its CSV into a params dict + rows grouped by scenario."""
    proc = subprocess.run(
        ["cargo", "run", "-q", "-p", "outlap-qss", "--example", "thermal_traces"],
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

    # (a) Rust Crank–Nicolson vs the analytic LTI step response.
    t = np.array([r[0] for r in rows["lti"]])
    cn = np.array([r[1] for r in rows["lti"]])
    cap, g, p, t_amb = params["lti_cap"], params["lti_g"], params["lti_p"], params["ambient_c"]
    analytic = t_amb + (p / g) * (1.0 - np.exp(-t * g / cap))
    ax = axes[0]
    ax.plot(t, analytic, "-", lw=2.4, label="analytic LTI")
    ax.plot(t[::12], cn[::12], "o", ms=5, label="Rust Crank–Nicolson")
    ax.set(xlabel="time [s]", ylabel="winding node [°C]", title="(a) integrator vs analytic")
    ax.legend(loc="lower right")

    # (b) Stint on rear.emotor.yaml: winding temperature + torque derate.
    laps = [r[0] for r in rows["stint"]]
    wind = [r[1] for r in rows["stint"]]
    der = [r[2] for r in rows["stint"]]
    ax = axes[1]
    ax.plot(laps, wind, "-o", ms=3, color="tab:red", label="winding [°C]")
    ax.axhspan(160, 180, color="tab:orange", alpha=0.15, label="derate band")
    ax.set(xlabel="lap", ylabel="winding temperature [°C]", title="(b) stint heat-soak → derate")
    ax2 = ax.twinx()
    ax2.plot(laps, der, "--s", ms=3, color="tab:blue", label="derate")
    ax2.set_ylabel("torque derate [-]", color="tab:blue")
    ax2.set_ylim(0.0, 1.05)
    ax2.grid(False)
    ax.legend(loc="center right")

    # (c) Speed-dependent air-gap cooling: rotor temperature at two shaft speeds.
    ts = [r[0] for r in rows["speed"]]
    slow = [r[1] for r in rows["speed"]]
    fast = [r[2] for r in rows["speed"]]
    ax = axes[2]
    ax.plot(ts, slow, "-", lw=2.2, label="ω = 100 rad/s")
    ax.plot(ts, fast, "--", lw=2.2, label="ω = 1500 rad/s")
    ax.set(xlabel="time [s]", ylabel="magnet temperature [°C]", title="(c) speed-dependent cooling")
    ax.legend(loc="lower right")

    fig.suptitle("Machine thermal (LPTN) — validated against the outlap-thermal integrator", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")


if __name__ == "__main__":
    main()
