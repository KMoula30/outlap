# SPDX-License-Identifier: AGPL-3.0-only
"""Plot the g-g-g-v **tyre-state** axes (M5 amendment to Decision #31; D-M5-2).

Runs the real `outlap-qss` example `envelope_tire_state`, which builds a
`GgvEnvelope::generate_with_tire_state(...)` for a downforce car and prints CSV traces of the two
grip couplings the axes carry, the boundary re-solved across tyre temperature and tread wear, and
the reference-slice bit-identity check. Every number here is the actual boundary re-solve — not a
re-implementation — so the figure cannot silently drift from the model.

Output: `docs/theory/img/ggv_tire_state.png`.
"""

from __future__ import annotations

import subprocess
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "ggv_tire_state.png"

# tab:* series palette (matches the other theory figures).
_COLD, _OPT, _WORN = "tab:blue", "tab:green", "tab:red"


def _run() -> tuple[dict[str, list[tuple[float, ...]]], dict[str, float]]:
    """Run the Rust example and parse its CSV; return (sections, params)."""
    # `--features parallel`: the tyre-state sweep re-solves the boundary at t·w grip states, so the
    # rayon fibre map keeps the (native, docs-only) figure build to tens of seconds.
    out = subprocess.run(
        [
            "cargo",
            "run",
            "--release",
            "-q",
            "--features",
            "parallel",
            "-p",
            "outlap-qss",
            "--example",
            "envelope_tire_state",
        ],
        cwd=_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    sections: dict[str, list[tuple[float, ...]]] = defaultdict(list)
    params: dict[str, float] = {}
    for line in out.splitlines():
        if line.startswith("#"):
            for tok in line.lstrip("# ").split():
                if "=" in tok:
                    key, _, val = tok.partition("=")
                    try:
                        params[key] = float(val)
                    except ValueError:
                        pass
            continue
        if not line or line.startswith("section"):
            continue
        sec, *rest = line.split(",")
        sections[sec].append(tuple(float(x) for x in rest))
    return sections, params


def main() -> None:
    sec, params = _run()
    t_opt = params.get("t_opt_c", 95.0)
    w_c = params.get("w_c", 2.0)

    fig, axes = plt.subplots(2, 3, figsize=(15.0, 8.8))
    fig.suptitle(
        "g-g-g-v tyre-state axes — the boundary re-solved across tyre temperature and wear "
        "(M5, amendment to Decision #31)",
        fontsize=13,
    )

    # --- Panel 1: the thermal grip window λ_μ(T_s) (Farroni) ---
    win = np.array(sec["win"])
    ax = axes[0, 0]
    ax.plot(win[:, 0], win[:, 1], color="tab:purple", lw=2)
    ax.axvline(t_opt, color=_OPT, ls="--", lw=1, label=f"$T_{{opt}}$ = {t_opt:.0f} °C")
    ax.set_title(
        "(1) grip window  $\\lambda_\\mu(T_s)=e^{-c_T((T_s-T_{opt})/T_{opt})^2}$"
    )
    ax.set_xlabel("surface temperature $T_s$ [°C]")
    ax.set_ylabel("grip multiplier $\\lambda_\\mu$")
    ax.set_ylim(0, 1.05)
    ax.legend(loc="lower center")

    # --- Panel 2: the wear grip cliff (Archard + sigmoid) ---
    ax = axes[0, 1]
    ax.plot(win[:, 2], win[:, 3], color="tab:orange", lw=2)
    ax.axvline(w_c, color=_WORN, ls="--", lw=1, label=f"$w_c$ = {w_c:.1f} mm")
    ax.set_title("(2) wear factor  $1-\\Delta_c\\,\\sigma((w-w_c)/s_w)$ (÷ new)")
    ax.set_xlabel("tread wear $w$ [mm]")
    ax.set_ylabel("grip multiplier")
    ax.legend(loc="lower left")

    # --- Panel 3: peak lateral grip vs T_tire, with the frozen reference ---
    temp = np.array(sec["temp"])
    ax = axes[0, 2]
    ax.plot(temp[:, 0], temp[:, 1], color="tab:blue", lw=2, label="re-solved boundary")
    ax.axhline(
        temp[0, 2], color="0.4", ls=":", lw=1.4, label="frozen envelope (tyre-blind)"
    )
    ax.axvline(t_opt, color=_OPT, ls="--", lw=1)
    ax.set_title("(3) peak lateral grip vs tyre temperature")
    ax.set_xlabel("surface temperature $T_s$ [°C]")
    ax.set_ylabel("max $a_y$ [m/s²]")
    ax.legend(loc="lower center")

    # --- Panel 4: peak lateral grip vs wear (the cliff on the envelope) ---
    wear = np.array(sec["wear"])
    ax = axes[1, 0]
    ax.plot(wear[:, 0], wear[:, 1], color="tab:orange", lw=2)
    ax.axvline(w_c, color=_WORN, ls="--", lw=1, label=f"$w_c$ = {w_c:.1f} mm")
    ax.set_title("(4) peak lateral grip vs tread wear (at $T_{opt}$)")
    ax.set_xlabel("tread wear $w$ [mm]")
    ax.set_ylabel("max $a_y$ [m/s²]")
    ax.legend(loc="lower left")

    # --- Panel 5: g-g sections breathing with tyre state ---
    gg = np.array(sec["gg"])
    ax = axes[1, 1]
    for sid, color, label in (
        (0, _COLD, "cold ($T_{opt}-55$°C)"),
        (1, _OPT, "optimum ($T_{opt}$, new)"),
        (2, _WORN, "worn ($T_{opt}$, cliff)"),
    ):
        m = gg[:, 2] == sid
        ax.plot(gg[m, 0], gg[m, 1], color=color, lw=2, label=label)
    ax.set_title("(5) g-g section breathes with tyre state")
    ax.set_xlabel("longitudinal $a_x$ [m/s²]")
    ax.set_ylabel("lateral $a_y$ [m/s²]")
    ax.legend(loc="lower center", fontsize=8)

    # --- Panel 6: the 2-D grip surface a_y(T_tire, wear) ---
    heat = np.array(sec["heat"])
    ts = np.unique(heat[:, 0])
    ws = np.unique(heat[:, 1])
    z = heat[:, 2].reshape(len(ts), len(ws))
    ax = axes[1, 2]
    pcm = ax.pcolormesh(ws, ts, z, shading="auto", cmap="viridis")
    ax.axhline(t_opt, color="white", ls="--", lw=1)
    ax.axvline(w_c, color="white", ls=":", lw=1)
    ax.set_title("(6) grip surface  $a_y(T_{tire}, w)$  (pure lateral)")
    ax.set_xlabel("tread wear $w$ [mm]")
    ax.set_ylabel("surface temperature $T_s$ [°C]")
    fig.colorbar(pcm, ax=ax, label="max $a_y$ [m/s²]")

    # Reference-slice bit-identity note (drawn from the `ident` section: it must overlie the frozen).
    ident = np.array(sec["ident"])
    max_gap = float(np.max(np.abs(ident[:, 1] - ident[:, 2]))) if ident.size else 0.0
    fig.text(
        0.5,
        0.005,
        f"reference slice $(T_{{opt}},\\,w{{=}}0)$ reproduces the frozen envelope to "
        f"{max_gap:.1e} m/s² — the invariant that keeps the QSS↔T2 parity gates + goldens green",
        ha="center",
        fontsize=9,
        color="0.35",
    )

    fig.tight_layout(rect=(0, 0.02, 1, 0.97))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}  (reference-slice max gap {max_gap:.2e} m/s²)")


if __name__ == "__main__":
    main()
