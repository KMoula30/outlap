# SPDX-License-Identifier: AGPL-3.0-only
"""Render the M6 PR6 T3 14-DOF chassis theory figures — from the authoritative model.

The dynamic figures integrate the SAME `sympy`-lambdified right-hand side the Rust `ChassisT3` block
is checked against to 1e-12 (`docs/derivations/t3_chassis_kane.py`), so they show the model of
record, not a re-implementation. Writes into ``docs/theory/img/``:

  * ``t3_dof.png``               — the 14-DOF layout (sprung heave/pitch/roll + 4 unsprung + tyre
                                   springs), a schematic.
  * ``t3_suspension_forces.png`` — the four force elements: linear spring, bump/rebound damper,
                                   C¹ progressive bumpstop, absolute ARB.
  * ``t3_dynamic_response.png``  — a braking-then-cornering manoeuvre integrated through the real
                                   RHS: pitch dives, the body rolls, and per-wheel F_z redistributes
                                   (the "downforce car is real" behaviour T3 exists to capture).
  * ``t3_kane_residual.png``     — the hand-written Rust RHS vs the SymPy derivation, all 24 states ×
                                   64 random states, well under the 1e-12 gate.

Run from anywhere:  python python/tools/plot_t3_chassis.py
(Regenerates the residual CSV via `cargo test -p outlap-vehicle --test kane_fixture_t3`.)
"""

from __future__ import annotations

import csv
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

_ROOT = Path(__file__).resolve().parents[2]
_IMG = _ROOT / "docs" / "theory" / "img"
_DERIV = _ROOT / "docs" / "derivations"

plt.style.use("seaborn-v0_8-darkgrid")
sys.path.insert(0, str(_DERIV))
import t3_chassis_kane as k  # noqa: E402

_FN, _NAMES = k.build_rhs_lambda()
_IDX = {nm: i for i, nm in enumerate(_NAMES)}
STD_G = 9.80665


def _base() -> dict[str, float]:
    """A symmetric F1-class car in static equilibrium (mirrors the Rust `equil` test fixture)."""
    d = {nm: 0.0 for nm in _NAMES}
    m, ms, m_u = 800.0, 740.0, 15.0
    xs = [1.70, 1.70, -1.70, -1.70]
    ys = [0.825, -0.825, 0.80, -0.80]
    kr = [220000.0, 220000.0, 240000.0, 240000.0]
    ktz = 250000.0
    d.update(
        m=m,
        izz=1000.0,
        ms=ms,
        ixx=180.0,
        iyy=950.0,
        hs=0.32,
        hcg=0.30,
        hra=0.05,
        g=STD_G,
        wheelbase=3.4,
        tf=1.65,
        tr=1.60,
        karb_f=4.0e5,
        karb_r=3.0e5,
        sbs=0.005,
        anti_d=0.35,
        anti_s=0.25,
    )
    corner = ms * STD_G / 4.0
    for i in range(4):
        d[f"x{i}"], d[f"y{i}"], d[f"R{i}"] = xs[i], ys[i], 0.33
        d[f"iw{i}"], d[f"mu{i}"] = 1.1, m_u
        d[f"kr{i}"] = kr[i]
        d[f"cb{i}"], d[f"cr{i}"] = 4000.0, 9000.0
        d[f"kbs{i}"], d[f"gbs{i}"] = 6.0e5, 0.035
        d[f"ktz{i}"], d[f"ctz{i}"] = ktz, 500.0
        d[f"d0_{i}"] = corner / kr[i]
        d[f"dtz0_{i}"] = (corner + m_u * STD_G) / ktz
    return d


def _flat(d: dict[str, float]) -> list[float]:
    return [d[nm] for nm in _NAMES]


# fast-state slots we integrate: index into the 24-vector by its state-order name.
_ST = {nm: i for i, nm in enumerate(k.STATE_ORDER)}

# Map each 24-vector state (k.STATE_ORDER) to its input-arg name (they differ: w_fl→w0, theta→th…).
_W = "fl fr rl rr".split()
_ARGMAP = {
    "n": "n",
    "psi_rel": "psi",
    "vx": "vx",
    "vy": "vy",
    "r": "r",
    "z": "z",
    "theta": "th",
    "phi": "ph",
    "zdot": "zd",
    "thetadot": "thd",
    "phidot": "phd",
}
for _i, _w in enumerate(_W):
    _ARGMAP[f"w_{_w}"] = f"w{_i}"
    _ARGMAP[f"zu_{_w}"] = f"zu{_i}"
    _ARGMAP[f"zudot_{_w}"] = f"zud{_i}"
# `s` (arc length) has no input arg — the RHS does not depend on it.


def _fz(d: dict[str, float], state: dict[str, float]) -> list[float]:
    """Per-wheel tyre normal load F_z = k_tz·(dtz0 + z_road − z_u)  (+ vertical damping)."""
    out = []
    for i in range(4):
        zu = state[f"zu_{'fl fr rl rr'.split()[i]}"]
        out.append(d[f"ktz{i}"] * (d[f"dtz0_{i}"] + 0.0 - zu))
    return out


def fig_dof() -> None:
    """Schematic of the 14-DOF layout (side + rear conceptual view)."""
    fig, (axs, axr) = plt.subplots(1, 2, figsize=(11, 4.4))
    # --- side view: heave/pitch + front/rear unsprung + tyre springs ---
    axs.set_title("Side view — heave $z$, pitch $\\theta$, 2 unsprung")
    body = Rectangle((-1.7, 0.9), 3.4, 0.5, fc="#4C78A8", ec="k", alpha=0.85)
    axs.add_patch(body)
    axs.text(
        0,
        1.15,
        "sprung mass $m_s$  ($I_{yy}$ pitch)",
        ha="center",
        color="w",
        fontsize=9,
    )
    for x in (-1.4, 1.4):
        axs.add_patch(Rectangle((x - 0.18, 0.2), 0.36, 0.28, fc="#F58518", ec="k"))
        axs.text(x, 0.34, "$m_u$", ha="center", va="center", fontsize=8)
        axs.plot([x, x], [0.48, 0.9], "k--", lw=1)  # suspension spring/damper
        axs.text(x + 0.22, 0.68, "spring+\ndamper+\nbumpstop", fontsize=7, va="center")
        axs.plot([x, x], [0.0, 0.2], color="#54A24B", lw=3)  # tyre spring
        axs.text(x + 0.22, 0.08, "$k_{tz}$ (tyre)", fontsize=7, va="center")
    axs.annotate(
        "", xy=(0, 1.7), xytext=(0, 1.42), arrowprops=dict(arrowstyle="->", lw=1.6)
    )
    axs.text(0.08, 1.6, "$z$ (heave)", fontsize=9)
    axs.plot([-2.0, 2.0], [0, 0], color="0.4", lw=2)
    axs.set_xlim(-2.3, 2.6)
    axs.set_ylim(-0.15, 1.9)
    axs.set_aspect("equal")
    axs.axis("off")
    # --- rear view: roll + L/R unsprung + ARB ---
    axr.set_title("Rear view — roll $\\phi$, ARB, L/R unsprung")
    axr.add_patch(Rectangle((-0.8, 0.9), 1.6, 0.5, fc="#4C78A8", ec="k", alpha=0.85))
    axr.text(0, 1.15, "$m_s$ ($I_{xx}$ roll)", ha="center", color="w", fontsize=9)
    for x in (-0.8, 0.8):
        axr.add_patch(Rectangle((x - 0.16, 0.2), 0.32, 0.28, fc="#F58518", ec="k"))
        axr.plot([x, x], [0.48, 0.9], "k--", lw=1)
        axr.plot([x, x], [0.0, 0.2], color="#54A24B", lw=3)
    axr.plot([-0.8, 0.8], [0.62, 0.62], color="#B279A2", lw=2.5)  # ARB
    axr.text(0, 0.68, "ARB $k_{arb}$", ha="center", fontsize=8, color="#6B3E7A")
    axr.annotate(
        "",
        xy=(0.55, 1.62),
        xytext=(-0.55, 1.62),
        arrowprops=dict(arrowstyle="->", lw=1.6, connectionstyle="arc3,rad=0.35"),
    )
    axr.text(0, 1.75, "$\\phi$ (roll)", ha="center", fontsize=9)
    axr.plot([-1.3, 1.3], [0, 0], color="0.4", lw=2)
    axr.set_xlim(-1.5, 1.5)
    axr.set_ylim(-0.15, 1.95)
    axr.set_aspect("equal")
    axr.axis("off")
    fig.suptitle(
        "T3 14-DOF chassis: 3 handling + 4 wheel-spin + 3 sprung ride + 4 unsprung",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(_IMG / "t3_dof.png", dpi=130)
    plt.close(fig)


def fig_forces() -> None:
    """The four suspension force elements."""
    fig, ax = plt.subplots(2, 2, figsize=(10, 7))
    # spring (linear)
    delta = np.linspace(-0.03, 0.05, 200)
    ax[0, 0].plot(delta * 1000, 220.0 * delta * 1000, color="#4C78A8")
    ax[0, 0].set_title("Linear ride-rate spring")
    ax[0, 0].set_xlabel("compression $\\delta$ (mm)")
    ax[0, 0].set_ylabel("force (kN)")
    # damper (bump/rebound)
    v = np.linspace(-1.0, 1.0, 200)
    fd = np.where(v >= 0, 4000.0 * v, 9000.0 * v)
    ax[0, 1].plot(v, fd / 1000, color="#F58518")
    ax[0, 1].set_title("Bump / rebound damper (asymmetric)")
    ax[0, 1].set_xlabel("compression rate $\\dot\\delta$ (m/s)")
    ax[0, 1].set_ylabel("force (kN)")
    ax[0, 1].axvline(0, color="0.6", lw=0.8)
    # bumpstop (C1 smooth ramp)
    p = np.linspace(-0.01, 0.03, 300)
    s = 0.005
    ramp = np.where(p < 0, 0.0, np.where(p < s, p * p / (2 * s), p - s / 2))
    ax[1, 0].plot(p * 1000, 600.0 * ramp * 1000, color="#54A24B", label="C1 smoothed")
    hard = np.clip(p, 0, None)
    ax[1, 0].plot(
        p * 1000, 600.0 * hard * 1000, "k--", lw=1, label="hard (C0) — avoided"
    )
    ax[1, 0].set_title("Progressive bumpstop (C¹ engagement)")
    ax[1, 0].set_xlabel("penetration past gap (mm)")
    ax[1, 0].set_ylabel("force (kN)")
    ax[1, 0].legend(fontsize=8)
    # ARB restoring couple vs roll
    phi = np.linspace(-0.04, 0.04, 200)
    tf = 1.65
    # force on the outer corner from the axle ARB (restoring); travel diff ≈ track·phi
    arb = 4.0e5 * (tf * phi) / tf**2
    ax[1, 1].plot(np.degrees(phi), arb / 1000, color="#B279A2")
    ax[1, 1].set_title("Anti-roll bar (restoring couple)")
    ax[1, 1].set_xlabel("roll angle $\\phi$ (deg)")
    ax[1, 1].set_ylabel("corner force (kN)")
    fig.suptitle(
        "T3 suspension force elements ($F_{susp}=k\\delta+c\\dot\\delta+F_{bump}+F_{arb}$)",
        fontsize=12,
    )
    fig.tight_layout()
    fig.savefig(_IMG / "t3_suspension_forces.png", dpi=130)
    plt.close(fig)


def fig_dynamic() -> None:
    """Integrate the real RHS through a brake→corner manoeuvre; show dive, roll, F_z transfer.

    The handling DOF (`vx, vy, r`, wheel spins) are held frozen so the imposed longitudinal /
    lateral accelerations are steady — an open-loop constant contact force with no tyre feedback
    would otherwise let the yaw run away (there is no tyre model at this tier yet). This isolates the
    *ride* response — exactly the pitch-under-braking / roll-in-corner behaviour T3 exists for.
    """
    d = _base()
    dt = 5e-4
    n = int(2.5 / dt)
    ride = [
        _ST[nm]
        for nm in (
            "z",
            "theta",
            "phi",
            "zdot",
            "thetadot",
            "phidot",
            "zu_fl",
            "zu_fr",
            "zu_rl",
            "zu_rr",
            "zudot_fl",
            "zudot_fr",
            "zudot_rl",
            "zudot_rr",
        )
    ]
    seed = np.zeros(24)
    seed[_ST["vx"]] = 60.0
    for w in "fl fr rl rr".split():
        seed[_ST[f"w_{w}"]] = 60.0 / 0.33

    def rhs(state: np.ndarray, brake: float, corner: float) -> np.ndarray:
        dd = dict(d)
        for (
            nm,
            arg,
        ) in _ARGMAP.items():  # scatter the 24 states into the input dict by arg name
            dd[arg] = state[_ST[nm]]
        for i in range(4):
            dd[f"fxw{i}"] = -brake
            dd[f"fyw{i}"] = (
                corner  # equal on all four so the imposed a_y is steady (no yaw couple)
            )
            dd[f"tau{i}"] = dd[f"R{i}"] * dd[f"fxw{i}"]  # freeze wheel spin
        return np.array(_FN(*_flat(dd)))

    t = np.arange(n) * dt
    th = np.zeros(n)
    ph = np.zeros(n)
    fz = np.zeros((n, 4))
    y = seed.copy()
    for step in range(n):
        # 0.0–0.5 s coast; 0.5–1.2 s brake (≈2.3g dive); 1.2–2.5 s left corner (≈2.5g roll).
        brake = 4600.0 if 0.5 <= t[step] < 1.2 else 0.0
        corner = 5000.0 if t[step] >= 1.2 else 0.0

        def deriv(
            state: np.ndarray, brake: float = brake, corner: float = corner
        ) -> np.ndarray:
            dv = rhs(state, brake, corner)
            frozen = np.zeros(
                24
            )  # advance only the ride/unsprung DOF; handling held at the seed
            for j in ride:
                frozen[j] = dv[j]
            return frozen

        k1 = deriv(y)
        k2 = deriv(y + 0.5 * dt * k1)
        k3 = deriv(y + 0.5 * dt * k2)
        k4 = deriv(y + dt * k3)
        y = y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        y[_ST["vx"]] = seed[_ST["vx"]]  # keep the handling frozen exactly
        th[step] = np.degrees(y[_ST["theta"]])
        ph[step] = np.degrees(y[_ST["phi"]])
        st = {f"zu_{s}": y[_ST[f"zu_{s}"]] for s in "fl fr rl rr".split()}
        fz[step] = _fz(d, st)

    fig, ax = plt.subplots(2, 1, figsize=(9.5, 7), sharex=True)
    ax[0].plot(t, th, label="pitch $\\theta$ (dive+)", color="#4C78A8")
    ax[0].plot(t, ph, label="roll $\\phi$ (right+)", color="#B279A2")
    ax[0].axvspan(0.5, 1.2, color="0.85", label="braking")
    ax[0].axvspan(1.2, 2.5, color="#FFF3CC", alpha=0.6, label="cornering (left)")
    ax[0].set_ylabel("body angle (deg)")
    ax[0].legend(fontsize=8, loc="upper left")
    ax[0].set_title("T3 dynamic response — real 14-DOF RHS (brake → corner)")
    labels = ["FL", "FR", "RL", "RR"]
    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756"]
    for i in range(4):
        ax[1].plot(t, fz[:, i] / 1000, label=labels[i], color=colors[i])
    ax[1].axvspan(0.5, 1.2, color="0.85")
    ax[1].axvspan(1.2, 2.5, color="#FFF3CC", alpha=0.6)
    ax[1].set_ylabel("per-wheel $F_z$ (kN)")
    ax[1].set_xlabel("time (s)")
    ax[1].legend(fontsize=8, ncol=4, loc="upper left")
    fig.tight_layout()
    fig.savefig(_IMG / "t3_dynamic_response.png", dpi=130)
    plt.close(fig)


def fig_residual() -> None:
    """The Rust ChassisT3 RHS vs the SymPy derivation, 24 states × 64 random states."""
    csv_path = Path(tempfile.gettempdir()) / "t3_resid.csv"
    env = {"OUTLAP_T3_RESIDUAL_CSV": str(csv_path)}
    try:
        subprocess.run(
            [
                "cargo",
                "test",
                "-p",
                "outlap-vehicle",
                "--test",
                "kane_fixture_t3",
                "-q",
            ],
            cwd=_ROOT,
            env={**_environ(), **env},
            check=True,
            capture_output=True,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"skip residual figure (cargo unavailable): {exc}", file=sys.stderr)
        return
    rels = []
    with csv_path.open() as fh:
        for row in csv.DictReader(fh):
            rels.append(float(row["rel"]))
    rels = np.array(rels)
    rels = np.where(rels <= 0, 1e-18, rels)
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    ax.hist(np.log10(rels), bins=40, color="#4C78A8", ec="w")
    ax.axvline(np.log10(1e-12), color="#E45756", lw=2, label="1e-12 gate")
    ax.axvline(
        np.log10(rels.max()),
        color="#54A24B",
        lw=2,
        ls="--",
        label=f"worst = {rels.max():.1e}",
    )
    ax.set_xlabel("$\\log_{10}$ relative |rust − sympy|")
    ax.set_ylabel("count (24 states × 64 states)")
    ax.set_title("T3 Kane fixture: hand-written RHS matches the symbolic derivation")
    ax.legend()
    fig.tight_layout()
    fig.savefig(_IMG / "t3_kane_residual.png", dpi=130)
    plt.close(fig)


def _environ() -> dict[str, str]:
    import os

    return dict(os.environ)


def main() -> None:
    _IMG.mkdir(parents=True, exist_ok=True)
    fig_dof()
    fig_forces()
    fig_dynamic()
    fig_residual()
    print(f"wrote T3 figures to {_IMG}")


if __name__ == "__main__":
    main()
