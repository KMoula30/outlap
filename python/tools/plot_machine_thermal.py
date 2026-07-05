# SPDX-License-Identifier: AGPL-3.0-only
"""Render the machine-thermal theory figure (docs/theory/img/machine_thermal.png).

Mirrors the `outlap-thermal` LPTN — the same Crank–Nicolson advance, coolant quasi-static balance,
copper feedback, and heat-transfer correlations — in numpy, and reproduces the PR5 validation story in
three panels:

  (a) Crank–Nicolson vs the analytic first-order LTI step response,
  (b) a stint on the lumped winding+housing+coolant model: winding temperature rise → torque derate,
  (c) the detailed network's speed-dependent cooling — magnet temperature vs time at two shaft speeds.

The equations mirror crates/outlap-thermal (network.rs + correlations.rs). Synthetic only.

Run from anywhere:  python python/tools/plot_machine_thermal.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.style.use("seaborn-v0_8-darkgrid")

_ROOT = Path(__file__).resolve().parents[2]
_OUT = _ROOT / "docs" / "theory" / "img" / "machine_thermal.png"
_K = 273.15


# --- correlations (mirror correlations.rs) ------------------------------------------------------


def _air(t_k):
    lam = 0.02416 + 7.7e-5 * (t_k - _K)
    mu = 1.716e-5 * (t_k / _K) ** 0.7
    rho = 101325.0 / (287.05 * t_k)
    nu = mu / rho
    return lam, nu


def _airgap_g(omega, t_i, t_j, area, r_gap=0.05, gap0=5e-4, kappa=10.4e-6):
    t_rot, t_sta = max(t_i, t_j), min(t_i, t_j)
    delta = max(gap0 - kappa * r_gap * (t_rot - _K - 20.0), 1e-6)
    lam, nu = _air(0.5 * (t_i + t_j))
    ta = omega**2 * r_gap * delta**3 / nu**2
    if ta < 1700:
        nu_ta = 2.0
    elif ta < 1e4:
        nu_ta = 0.128 * ta**0.367
    else:
        nu_ta = 0.409 * ta**0.241
    return (nu_ta * lam) * area / delta


def _channel_g(area, d_h=0.0089, vel=3.1, lam=0.401, nu=1.74e-6, pr=15.6):
    re = vel * d_h / nu
    if re < 2300:
        nu_ch = 4.36
    else:
        f = (0.79 * np.log(re) - 1.64) ** -2
        nu_ch = (
            (f / 8)
            * (re - 1000)
            * pr
            / (1 + 12.7 * (f / 8) ** 0.5 * (pr ** (2 / 3) - 1))
        )
    return nu_ch * lam / d_h * area


# --- Crank–Nicolson advance (mirror network.rs) -------------------------------------------------


def cn_step(temp, cap, edges, conv, p, omega, t_amb, amb_idx, coolant, cu, dt):
    n = len(temp)
    g = np.zeros((n, n))
    for i, j, w in edges:
        g[i, j] += w
        g[j, i] += w
    for i, j, fn in conv:
        gg = fn(omega, temp[i], temp[j])
        g[i, j] += gg
        g[j, i] += gg
    for i in range(n):
        g[i, i] = -(g[i].sum() - g[i, i])
    pv = p.copy()
    if cu is not None:
        nd, t_ref, alpha = cu
        pv[nd] *= max(1.0 + alpha * (temp[nd] - t_ref), 0.0)
    a = np.diag(cap / dt) - 0.5 * g
    b = (np.diag(cap / dt) + 0.5 * g) @ temp + pv
    a[amb_idx, :] = 0.0
    a[amb_idx, amb_idx] = 1.0
    b[amb_idx] = t_amb
    if coolant is not None:
        c_idx, inlet, rho_cp_m = coolant
        q_in = sum(
            g[c_idx, j] * (temp[j] - temp[c_idx])
            for j in range(n)
            if j not in (c_idx, amb_idx)
        )
        a[c_idx, :] = 0.0
        a[c_idx, c_idx] = 1.0
        b[c_idx] = inlet + q_in / (2.0 * rho_cp_m)
    out = np.linalg.solve(a, b)
    out[amb_idx] = t_amb
    return out


def derate(temp, limits):
    d = 1.0
    for i, (warn, tmax) in limits.items():
        t_c = temp[i] - _K
        d = min(d, np.clip((tmax - t_c) / (tmax - warn), 0.0, 1.0))
    return d


# --- panels -------------------------------------------------------------------------------------


def panel_lti(ax):
    cap = np.array([2000.0, 1e18])
    edges = [(0, 1, 5.0)]
    t_amb = 300.0
    p = np.array([1500.0, 0.0])
    dt, n = 2.0, 500
    temp = np.array([t_amb, t_amb])
    ts, num = [], []
    for k in range(n):
        temp = cn_step(temp, cap, edges, [], p, 0.0, t_amb, 1, None, None, dt)
        ts.append(k * dt)
        num.append(temp[0] - _K)
    ts = np.array(ts)
    tau = cap[0] / 5.0
    analytic = (t_amb + (1500.0 / 5.0) * (1 - np.exp(-ts / tau))) - _K
    ax.plot(ts, analytic, "-", lw=2.4, label="analytic LTI")
    ax.plot(ts[::25], np.array(num)[::25], "o", ms=5, label="Crank–Nicolson")
    ax.set(
        xlabel="time [s]",
        ylabel="winding ΔT node [°C]",
        title="(a) integrator vs analytic",
    )
    ax.legend(loc="lower right")


def panel_stint(ax):
    # winding + housing + coolant + ambient (mirrors rear.emotor.yaml), closed derate→loss loop.
    cap = np.array([9000.0, 32000.0, 1e18, 1e18])
    edges = [(0, 1, 12.0), (1, 2, 45.0), (1, 3, 5.0)]
    limits = {0: (160.0, 180.0)}
    cu = (0, 60.0 + _K, 0.0039)
    coolant = (2, 65.0 + _K, 900.0)
    t_amb = 20.0 + _K
    temp = np.array([t_amb, t_amb, 65.0 + _K, t_amb])
    base = 3000.0
    laps, wind, der = [], [], []
    for lap in range(26):
        for _ in range(200):
            d = derate(temp, limits)
            p = np.array([base * d * 0.7, base * d * 0.3, 0.0, 0.0])
            temp = cn_step(temp, cap, edges, [], p, 800.0, t_amb, 3, coolant, cu, 1.0)
        laps.append(lap)
        wind.append(temp[0] - _K)
        der.append(derate(temp, limits))
    ax.plot(laps, wind, "-o", ms=3, color="tab:red", label="winding [°C]")
    ax.axhspan(160, 180, color="tab:orange", alpha=0.15, label="derate band")
    ax.set(
        xlabel="lap",
        ylabel="winding temperature [°C]",
        title="(b) stint heat-soak → derate",
    )
    ax2 = ax.twinx()
    ax2.plot(laps, der, "--s", ms=3, color="tab:blue", label="derate")
    ax2.set_ylabel("torque derate [-]", color="tab:blue")
    ax2.set_ylim(0.0, 1.05)
    ax2.grid(False)
    ax.legend(loc="center right")


def panel_speed(ax):
    # detailed: slot_active, stator_iron, magnet, airgap, housing, coolant, ambient (mirrors pdt_synth).
    cap = np.array([370.0, 3000.0, 800.0, 5.0, 6000.0, 1e18, 1e18])
    edges = [(0, 1, 40.0), (1, 4, 60.0), (4, 6, 3.0)]
    coolant = (5, 65.0 + _K, 900.0)
    t_amb = 20.0 + _K
    conv = [
        (1, 3, lambda w, ti, tj: _airgap_g(w, ti, tj, 0.02)),
        (3, 2, lambda w, ti, tj: _airgap_g(w, ti, tj, 0.02)),
        (4, 5, lambda w, ti, tj: _channel_g(0.025)),
    ]
    p = np.array([2500.0, 800.0, 600.0, 0.0, 0.0, 0.0, 0.0])
    for omega, style in [(100.0, "-"), (1500.0, "--")]:
        temp = np.array([t_amb, t_amb, t_amb, t_amb, t_amb, 65.0 + _K, t_amb])
        ts, mag = [], []
        for k in range(1600):
            temp = cn_step(
                temp, cap, edges, conv, p, omega, t_amb, 6, coolant, None, 0.25
            )
            ts.append(k * 0.25)
            mag.append(temp[2] - _K)
        ax.plot(ts, mag, style, lw=2.2, label=f"ω = {omega:.0f} rad/s")
    ax.set(
        xlabel="time [s]",
        ylabel="magnet temperature [°C]",
        title="(c) speed-dependent cooling",
    )
    ax.legend(loc="lower right")


def main() -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))
    panel_lti(axes[0])
    panel_stint(axes[1])
    panel_speed(axes[2])
    fig.suptitle("Machine thermal (LPTN) — Crank–Nicolson validation", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(_OUT, dpi=130)
    print(f"wrote {_OUT}")


if __name__ == "__main__":
    main()
