# SPDX-License-Identifier: AGPL-3.0-only
"""Distill the PDT 19-node LPTN into the outlap 2-node ``.emotor`` model (§10.2 step 6, §8.5).

Least-squares fit of ``(C_w, C_c, G_wc, G_cool)`` so the 2-node network, driven by the exported loss
map, reproduces the PDT-solved continuous torque envelope and the 10/20/30 s overload torques at a
handful of speeds. numpy-only (the ``h5py+numpy+pyarrow`` rule forbids scipy): the model is LTI, so
steady state and the transient are closed-form 2×2, and the optimiser is a hand-rolled log-space
Nelder–Mead.

Model (§8.5), with copper resistance-rise feedback ``k_cu(T_w) = 1 + α·(T_w − T_ref)``:

    C_w·Ṫ_w = s_w·P·k_cu(T_w) − G_wc·(T_w − T_c)
    C_c·Ṫ_c = (1 − s_w)·P + G_wc·(T_w − T_c) − G_cool·(T_c − T_cool)
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class TwoNode:
    """A 2-node winding/case thermal network."""

    c_w: float
    c_c: float
    g_wc: float
    g_cool: float
    s_w: float  # winding loss fraction
    alpha: float  # copper temperature coefficient, per K
    t_ref: float  # copper feedback reference temperature, °C
    t_cool: float  # coolant/ambient temperature, °C

    def system(self, power: float) -> tuple[np.ndarray, np.ndarray]:
        """Linear system ``Ṫ = A·T + b`` for constant loss ``power``."""
        a = np.array(
            [
                [
                    (self.s_w * power * self.alpha - self.g_wc) / self.c_w,
                    self.g_wc / self.c_w,
                ],
                [self.g_wc / self.c_c, (-self.g_wc - self.g_cool) / self.c_c],
            ]
        )
        b = np.array(
            [
                self.s_w * power * (1.0 - self.alpha * self.t_ref) / self.c_w,
                ((1.0 - self.s_w) * power + self.g_cool * self.t_cool) / self.c_c,
            ]
        )
        return a, b

    def steady_state(self, power: float) -> np.ndarray:
        """Steady-state ``[T_w, T_c]`` for constant loss ``power`` (°C)."""
        a, b = self.system(power)
        return np.linalg.solve(a, -b)

    def transient(self, power: float, duration: float, t0: np.ndarray) -> np.ndarray:
        """``[T_w, T_c]`` after ``duration`` s of constant ``power`` from initial ``t0``."""
        a, b = self.system(power)
        t_ss = np.linalg.solve(a, -b)
        return t_ss + _expm2(a * duration) @ (t0 - t_ss)


def _expm2(m: np.ndarray) -> np.ndarray:
    """Matrix exponential of a 2×2 matrix (closed form via eigenvalues)."""
    tr = m[0, 0] + m[1, 1]
    det = m[0, 0] * m[1, 1] - m[0, 1] * m[1, 0]
    disc = tr * tr - 4.0 * det
    ident = np.eye(2)
    if disc > 1e-12:
        root = np.sqrt(disc)
        l1, l2 = (tr + root) / 2.0, (tr - root) / 2.0
        e1, e2 = np.exp(l1), np.exp(l2)
        return (l1 * e2 - l2 * e1) / (l1 - l2) * ident + (e1 - e2) / (l1 - l2) * m
    # Repeated (or near-repeated) eigenvalue: e^{λ}(I + (A − λI)).
    lam = tr / 2.0
    return np.exp(lam) * (ident + (m - lam * ident))


@dataclass
class ThermalTargets:
    """The PDT envelope targets to reproduce."""

    speeds_rad: np.ndarray  # (m,) shaft speed at each target
    p_cont: np.ndarray  # (m,) loss at the continuous torque
    p_overload: np.ndarray  # (m, n_dur) loss at each overload torque
    durations: np.ndarray  # (n_dur,)
    t_max_w: float
    t_max_c: float


def _residuals(model: TwoNode, tg: ThermalTargets) -> np.ndarray:
    """Normalised temperature residuals (0 when a node sits exactly on its limit)."""
    span_w = max(tg.t_max_w - model.t_cool, 1.0)
    span_c = max(tg.t_max_c - model.t_cool, 1.0)
    res = []
    for j in range(tg.speeds_rad.size):
        # Continuous: the binding node should sit at its limit at steady state.
        tw, tc = model.steady_state(float(tg.p_cont[j]))
        res.append(max((tw - tg.t_max_w) / span_w, (tc - tg.t_max_c) / span_c))
        # Overload: winding reaches its limit at the stated duration, from the continuous IC.
        t0 = model.steady_state(float(tg.p_cont[j]))
        for k, d in enumerate(tg.durations):
            tw_d = model.transient(float(tg.p_overload[j, k]), float(d), t0)[0]
            res.append((tw_d - tg.t_max_w) / span_w)
    return np.asarray(res)


def _safe_model(theta_log: np.ndarray, base: TwoNode) -> TwoNode:
    c_w, c_c, g_wc, g_cool = np.exp(np.clip(theta_log, -15.0, 15.0))
    return TwoNode(
        c_w, c_c, g_wc, g_cool, base.s_w, base.alpha, base.t_ref, base.t_cool
    )


def _objective(theta_log: np.ndarray, base: TwoNode, tg: ThermalTargets) -> float:
    if not np.all(np.isfinite(theta_log)):
        return 1e6
    try:
        model = _safe_model(theta_log, base)
        a, _ = model.system(float(tg.p_cont.max()))
        if (
            not np.all(np.isfinite(a)) or a[0, 0] + a[1, 1] >= 0.0
        ):  # non-finite / unstable
            return 1e6
        r = _residuals(model, tg)
    except (np.linalg.LinAlgError, FloatingPointError, ValueError):
        return 1e6
    if not np.all(np.isfinite(r)):
        return 1e6
    return float(np.sum(r * r))


def nelder_mead(
    fun: Callable[[np.ndarray], float], x0: np.ndarray, *, max_iter: int = 500
) -> np.ndarray:
    """Deterministic Nelder–Mead (Gao–Han adaptive), numpy-only."""
    n = x0.size
    alpha, gamma = 1.0, 1.0 + 2.0 / n
    rho, sigma = 0.75 - 1.0 / (2.0 * n), 1.0 - 1.0 / n
    simplex = np.vstack([x0] + [x0 + 0.25 * np.eye(n)[i] for i in range(n)])
    fvals = np.array([fun(x) for x in simplex])
    for _ in range(max_iter):
        order = np.argsort(fvals)
        simplex, fvals = simplex[order], fvals[order]
        if fvals[-1] - fvals[0] < 1e-10 and np.max(np.ptp(simplex, axis=0)) < 1e-4:
            break
        centroid = simplex[:-1].mean(axis=0)
        xr = centroid + alpha * (centroid - simplex[-1])
        fr = fun(xr)
        if fvals[0] <= fr < fvals[-2]:
            simplex[-1], fvals[-1] = xr, fr
        elif fr < fvals[0]:
            xe = centroid + gamma * (xr - centroid)
            fe = fun(xe)
            simplex[-1], fvals[-1] = (xe, fe) if fe < fr else (xr, fr)
        else:
            xc = centroid + rho * (simplex[-1] - centroid)
            fc = fun(xc)
            if fc < fvals[-1]:
                simplex[-1], fvals[-1] = xc, fc
            else:
                for i in range(1, n + 1):
                    simplex[i] = simplex[0] + sigma * (simplex[i] - simplex[0])
                    fvals[i] = fun(simplex[i])
    return simplex[int(np.argmin(fvals))]


def fit_two_node(
    tg: ThermalTargets,
    *,
    c_total: float,
    s_w: float,
    alpha: float,
    t_cool: float,
    t_max_w: float,
    t_max_c: float,
) -> tuple[TwoNode, float]:
    """Fit ``(C_w, C_c, G_wc, G_cool)`` to the envelope targets; return the model + torque-space RMS."""
    base = TwoNode(0, 0, 0, 0, s_w, alpha, t_cool, t_cool)
    tg = ThermalTargets(
        tg.speeds_rad, tg.p_cont, tg.p_overload, tg.durations, t_max_w, t_max_c
    )
    # Init from LPTN aggregates + a P/ΔT conductance guess.
    p_mid = float(np.median(tg.p_cont))
    g_cool0 = p_mid / max(t_max_c - t_cool, 10.0)
    g_wc0 = s_w * p_mid / max(t_max_w - t_max_c, 10.0)
    x0 = np.log(np.maximum([0.15 * c_total, 0.85 * c_total, g_wc0, g_cool0], 1e-6))
    with np.errstate(all="ignore"):
        best = nelder_mead(lambda x: _objective(x, base, tg), x0)
    model = _safe_model(best, base)
    try:
        rms = float(np.sqrt(np.mean(_residuals(model, tg) ** 2)))
    except (np.linalg.LinAlgError, ValueError):
        rms = float("nan")
    return model, rms


def build_emotor_doc(
    model: TwoNode,
    *,
    t_warn_w: float,
    t_warn_c: float,
    t_max_w: float,
    t_max_c: float,
    notes: str,
    copper_feedback: bool = True,
) -> dict[str, Any]:
    """Assemble an ``emotor/1.0`` document from a fitted model (wire-exact for the Rust schema)."""
    loss_routing: dict[str, Any] = {"winding_split": round(float(model.s_w), 4)}
    if copper_feedback:
        loss_routing["copper_alpha_per_k"] = round(float(model.alpha), 6)
    return {
        "schema": "emotor/1.0",
        "nodes": {
            "winding": {
                "c_j_per_k": round(float(model.c_w), 2),
                "t_max_c": round(float(t_max_w), 1),
                "t_warn_c": round(float(t_warn_w), 1),
            },
            "case": {
                "c_j_per_k": round(float(model.c_c), 2),
                "t_max_c": round(float(t_max_c), 1),
                "t_warn_c": round(float(t_warn_c), 1),
            },
        },
        "coupling": {
            "g_wc_w_per_k": round(float(model.g_wc), 4),
            "g_cool_w_per_k": round(float(model.g_cool), 4),
        },
        "cooling": {"liquid": {"coolant_temp_c": round(float(model.t_cool), 3)}},
        "loss_routing": loss_routing,
        "meta": {"source": "pdt_distilled", "notes": notes},
    }
