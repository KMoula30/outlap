# SPDX-License-Identifier: AGPL-3.0-only
"""Render the M5 PR3 tire-thermal *lap-wiring* figure (docs/theory/img/tire_thermal_lap.png).

Drives the figure from the **real** `TransientSolver`: it runs the committed Rust example
`crates/outlap-transient/examples/tire_thermal_lap.rs` (which advances an actual T2 skidpad lap with
the ring + wear stack wired on the slow clock) and plots its CSV. Nothing here re-implements the model.

Four panels:
  (a) cold-start warm-up over a skidpad — the outer (loaded) vs inner (light) front surface temperature
      climbing toward the grip window;
  (b) the total grip multiplier λ_μ,total the force call uses, rising as the tyres warm (outer > inner);
  (c) a long stint — tread wear crossing the cliff (w_c) and grip falling with it;
  (d) the warm-up as a trajectory on the static grip window λ_μ(T_s) — the tyre climbing the curve.

Run from anywhere:  python python/tools/plot_tire_thermal_lap.py
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "tire_thermal_lap.png"


def _run_traces() -> tuple[dict[str, float], dict[str, np.ndarray]]:
    """Run the Rust example and parse its CSV into a params dict + per-scenario arrays."""
    proc = subprocess.run(
        [
            "cargo",
            "run",
            "-q",
            "--release",
            "-p",
            "outlap-transient",
            "--example",
            "tire_thermal_lap",
        ],
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
                if v:
                    params[k] = float(v)
        elif line and not line.startswith("scenario"):
            parts = line.split(",")
            scen = parts[0]
            vals = tuple(float(p) if p else np.nan for p in parts[1:])
            rows.setdefault(scen, []).append(vals)
    return params, {k: np.array(v) for k, v in rows.items()}


def main() -> None:
    params, rows = _run_traces()
    t_opt = params["t_opt_c"]
    lo, hi = params["window_lo"], params["window_hi"]
    w_c = params["w_c"]
    fig, axes = plt.subplots(1, 4, figsize=(19.5, 4.4))

    warm = rows["warmup"]
    t, ts_out, ts_in, grip_out, grip_in = (warm[:, i] for i in range(5))

    # (a) Warm-up of the outer vs inner front surface temperature.
    ax = axes[0]
    ax.axhspan(lo, hi, color="tab:green", alpha=0.12, label="grip window")
    ax.plot(t, ts_out, "-", lw=2.4, color="tab:red", label="outer front $T_s$")
    ax.plot(t, ts_in, "-", lw=2.0, color="tab:blue", label="inner front $T_s$")
    ax.axhline(t_opt, ls=":", color="0.4", lw=1.2)
    ax.set(
        xlabel="lap time [s]",
        ylabel="surface temperature [°C]",
        title="(a) tyres warm over the lap",
    )
    ax.legend(loc="lower right", fontsize=9)

    # (b) The grip multiplier the force call uses, rising as the tyres warm.
    ax = axes[1]
    ax.plot(t, grip_out, "-", lw=2.4, color="tab:red", label="outer front")
    ax.plot(t, grip_in, "-", lw=2.0, color="tab:blue", label="inner front")
    ax.set(
        xlabel="lap time [s]",
        ylabel=r"grip multiplier $\lambda_{\mu,\mathrm{total}}$ [-]",
        title="(b) grip rises with temperature",
    )
    ax.legend(loc="lower right", fontsize=9)

    # (c) A long stint: wear crosses the cliff and grip falls with it.
    st = rows["stint"]
    ts_t, wear, _dmg, sgrip = st[:, 0], st[:, 1], st[:, 2], st[:, 3]
    ax = axes[2]
    ax.plot(ts_t, wear, "-", lw=2.4, color="tab:brown", label="tread wear $w$")
    ax.axhline(w_c, ls="--", color="0.4", lw=1.4)
    ax.annotate(
        "cliff onset $w_c$", (ts_t[-1] * 0.02, w_c + 0.12), color="0.3", fontsize=9
    )
    ax.set(
        xlabel="stint time [s]", ylabel="tread wear [mm]", title="(c) stint: wear cliff"
    )
    ax2 = ax.twinx()
    ax2.plot(
        ts_t,
        sgrip,
        "-",
        lw=2.2,
        color="tab:purple",
        label=r"grip $\lambda_{\mu,\mathrm{total}}$",
    )
    ax2.set_ylabel(r"grip $\lambda_{\mu,\mathrm{total}}$ [-]", color="tab:purple")
    ax2.grid(False)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="center right", fontsize=9)

    # (d) The warm-up as a trajectory on the static grip window λ_μ(T_s).
    win = rows["window"]
    ax = axes[3]
    ax.plot(
        win[:, 0],
        win[:, 1],
        "-",
        lw=2.0,
        color="0.5",
        label=r"window $\lambda_\mu(T_s)$",
    )
    ax.plot(ts_out, grip_out, "-", lw=2.6, color="tab:red", label="warm-up trajectory")
    ax.plot(ts_out[0], grip_out[0], "o", color="tab:blue", ms=7, label="cold start")
    ax.plot(ts_out[-1], grip_out[-1], "o", color="tab:red", ms=7, label="warm")
    ax.axvline(t_opt, ls=":", color="0.4", lw=1.2)
    ax.set(
        xlabel="surface temperature $T_s$ [°C]",
        ylabel=r"grip $\lambda_\mu$ [-]",
        title="(d) climbing the grip window",
        xlim=(55, t_opt + 20),
    )
    ax.legend(loc="lower right", fontsize=9)

    fig.suptitle(
        "M5 PR3 — the tyre thermal ring + wear wired into the T2 lap (driven by the real "
        "TransientSolver)",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=125)
    print(f"wrote {_OUT}")


if __name__ == "__main__":
    main()
