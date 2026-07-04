# SPDX-License-Identifier: AGPL-3.0-only
"""Vectorized MF6.1 steady-state forward model (Pacejka 2012, 3rd ed., §4.3.2).

Clean-room numpy mirror of outlap's Rust kernels (``crates/outlap-tire/src/mf61/``), equation
anchors 4.E9–4.E78 as cited there. This model is what the fitting stages evaluate, and it is
validated against THE SAME committed golden CSVs and tolerance rule as the Rust kernel
(``crates/outlap-tire/tests/golden/``), so a fit produced here evaluates identically in the
solver. The adopted operational conventions are mirrored too: zero-camber Mz lateral machinery,
the ``s·Fx`` term gated to κ ≠ 0, ``Et`` from the base trail angle, the book ``(atan x)²`` Mx
form, and turn-slip ζ ≡ 1.

Parameters are a plain ``dict[str, float]`` keyed by ``.tir`` coefficient names, built by
:func:`params_from_coeffs` (which applies the same default table as the Rust
``Mf61Params::from_coeffs``). All slip/load inputs are numpy arrays (SI, ISO 8855 / ISO-W).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray

F = NDArray[np.float64]

_EPS = 1e-6
_EXP_MAX = 80.0
_ALPHA_MAX = math.pi / 2 - 1e-3
_A_MU = 10.0
_P_RATIO_FLOOR = 1e-6
_TWO_OVER_PI = 2.0 / math.pi

#: Defaults mirroring ``Mf61Params::from_coeffs`` — every coefficient the kernel reads.
#: Unlisted keys in an input map are carried but ignored by the forward model.
DEFAULTS: dict[str, float] = {
    "FNOMIN": 0.0,
    "UNLOADED_RADIUS": 0.0,
    "NOMPRES": 0.0,
    "LONGVL": 16.7,
    "VXLOW": 1.0,
    # Fx0 pure.
    "PCX1": 0.0,
    "PDX1": 0.0,
    "PDX2": 0.0,
    "PDX3": 0.0,
    "PEX1": 0.0,
    "PEX2": 0.0,
    "PEX3": 0.0,
    "PEX4": 0.0,
    "PKX1": 0.0,
    "PKX2": 0.0,
    "PKX3": 0.0,
    "PHX1": 0.0,
    "PHX2": 0.0,
    "PVX1": 0.0,
    "PVX2": 0.0,
    "PPX1": 0.0,
    "PPX2": 0.0,
    "PPX3": 0.0,
    "PPX4": 0.0,
    # Fx combined.
    "RBX1": 0.0,
    "RBX2": 0.0,
    "RBX3": 0.0,
    "RCX1": 1.0,
    "REX1": 0.0,
    "REX2": 0.0,
    "RHX1": 0.0,
    # Fy0 pure.
    "PCY1": 0.0,
    "PDY1": 0.0,
    "PDY2": 0.0,
    "PDY3": 0.0,
    "PEY1": 0.0,
    "PEY2": 0.0,
    "PEY3": 0.0,
    "PEY4": 0.0,
    "PEY5": 0.0,
    "PKY1": 0.0,
    "PKY2": 1.0,
    "PKY3": 0.0,
    "PKY4": 2.0,
    "PKY5": 0.0,
    "PKY6": 0.0,
    "PKY7": 0.0,
    "PHY1": 0.0,
    "PHY2": 0.0,
    "PVY1": 0.0,
    "PVY2": 0.0,
    "PVY3": 0.0,
    "PVY4": 0.0,
    "PPY1": 0.0,
    "PPY2": 0.0,
    "PPY3": 0.0,
    "PPY4": 0.0,
    "PPY5": 0.0,
    # Fy combined.
    "RBY1": 0.0,
    "RBY2": 0.0,
    "RBY3": 0.0,
    "RBY4": 0.0,
    "RCY1": 1.0,
    "REY1": 0.0,
    "REY2": 0.0,
    "RHY1": 0.0,
    "RHY2": 0.0,
    "RVY1": 0.0,
    "RVY2": 0.0,
    "RVY3": 0.0,
    "RVY4": 0.0,
    "RVY5": 0.0,
    "RVY6": 0.0,
    # Mz.
    "QBZ1": 0.0,
    "QBZ2": 0.0,
    "QBZ3": 0.0,
    "QBZ4": 0.0,
    "QBZ5": 0.0,
    "QBZ9": 0.0,
    "QBZ10": 0.0,
    "QCZ1": 1.0,
    "QDZ1": 0.0,
    "QDZ2": 0.0,
    "QDZ3": 0.0,
    "QDZ4": 0.0,
    "QDZ6": 0.0,
    "QDZ7": 0.0,
    "QDZ8": 0.0,
    "QDZ9": 0.0,
    "QDZ10": 0.0,
    "QDZ11": 0.0,
    "QEZ1": 0.0,
    "QEZ2": 0.0,
    "QEZ3": 0.0,
    "QEZ4": 0.0,
    "QEZ5": 0.0,
    "QHZ1": 0.0,
    "QHZ2": 0.0,
    "QHZ3": 0.0,
    "QHZ4": 0.0,
    "PPZ1": 0.0,
    "PPZ2": 0.0,
    "SSZ1": 0.0,
    "SSZ2": 0.0,
    "SSZ3": 0.0,
    "SSZ4": 0.0,
    # Mx.
    "QSX1": 0.0,
    "QSX2": 0.0,
    "QSX3": 0.0,
    "QSX4": 0.0,
    "QSX5": 0.0,
    "QSX6": 0.0,
    "QSX7": 0.0,
    "QSX8": 0.0,
    "QSX9": 0.0,
    "QSX10": 0.0,
    "QSX11": 0.0,
    "PPMX1": 0.0,
    # My.
    "QSY1": 0.0,
    "QSY2": 0.0,
    "QSY3": 0.0,
    "QSY4": 0.0,
    "QSY5": 0.0,
    "QSY6": 0.0,
    "QSY7": 0.0,
    "QSY8": 0.0,
    # Scaling factors.
    "LFZO": 1.0,
    "LCX": 1.0,
    "LMUX": 1.0,
    "LEX": 1.0,
    "LKX": 1.0,
    "LHX": 1.0,
    "LVX": 1.0,
    "LCY": 1.0,
    "LMUY": 1.0,
    "LEY": 1.0,
    "LKY": 1.0,
    "LKYC": 1.0,
    "LKZC": 1.0,
    "LHY": 1.0,
    "LVY": 1.0,
    "LTR": 1.0,
    "LRES": 1.0,
    "LXAL": 1.0,
    "LYKA": 1.0,
    "LVYKA": 1.0,
    "LS": 1.0,
    "LMX": 1.0,
    "LMY": 1.0,
    "LVMX": 1.0,
    "LSGKP": 1.0,
    "LSGAL": 1.0,
}


def params_from_coeffs(coeffs: dict[str, float]) -> dict[str, float]:
    """Apply the default table (mirror of ``Mf61Params::from_coeffs``) to a coefficient map.

    The returned dict carries every kernel coefficient plus ``_HAS_NOMPRES`` (0/1: pressure
    terms enabled only when the source map declares ``NOMPRES``, matching the Rust loader).
    """
    p = dict(DEFAULTS)
    for key, value in coeffs.items():
        if not math.isfinite(value):
            raise ValueError(f"non-finite MF6.1 coefficient {key} = {value}")
        p[key] = float(value)
    for key in ("FNOMIN", "UNLOADED_RADIUS"):
        if p[key] <= 0.0:
            raise ValueError(f"MF6.1 requires {key} > 0 (got {p[key]})")
    # Mirror the Rust loader: NOMPRES/LONGVL must be positive WHEN present (a zero would
    # silently poison dpi / the My speed ratio with division by zero).
    for key in ("NOMPRES", "LONGVL"):
        if key in coeffs and p[key] <= 0.0:
            raise ValueError(f"MF6.1 requires {key} > 0 when present (got {p[key]})")
    p["_HAS_NOMPRES"] = 1.0 if "NOMPRES" in coeffs else 0.0
    return p


def params_from_tyr(tyr: dict[str, Any]) -> dict[str, float]:
    """Build the parameter dict from a loaded ``.tyr`` document dict."""
    mf61: dict[str, Any] = tyr["mf61"]
    return params_from_coeffs({str(k): float(v) for k, v in mf61.items()})


@dataclass
class Forces:
    """Evaluated steady-state channels (N / N·m), each shaped like the inputs."""

    fx: F
    fy: F
    mz: F
    mx: F
    my: F


def _sgn_pos(x: F) -> F:
    """sgn with sgn(0) = +1 (the kernel's sign convention everywhere)."""
    return np.where(x >= 0.0, 1.0, -1.0)


def _safe_denom(d: F, eps: float = _EPS) -> F:
    """Sign-preserving singularity guard ``d + eps*sgn(d)``."""
    return d + eps * _sgn_pos(d)


def _guard(d: F, eps: float = _EPS) -> F:
    """Magnitude-floored, sign-preserving guard for the combined-slip cosines."""
    return np.where(np.abs(d) < eps, eps * _sgn_pos(d), d)


def _cos_mf(b: F, c: float, e: F, x: F) -> F:
    """``cos(C·atan(B·x − E·(B·x − atan(B·x))))`` shared by both combined weights."""
    arg = b * x
    return np.cos(c * np.arctan(arg - e * (arg - np.arctan(arg))))


@dataclass
class _Norm:
    """Normalized per-evaluation quantities (mirror of the Rust ``Norm``)."""

    kappa: F
    alpha_star: F
    gamma: F
    gamma_sq: F
    gamma_star: F
    gamma_star_sq: F
    gamma_star_abs: F
    fz: F
    dfz: F
    dfz_sq: F
    dpi: F
    p_ratio: F
    sgn_vcx: F
    cos_alpha: F
    vx_abs: F
    lmux_eff: F
    lmuy_eff: F
    lmux_prime: F
    lmuy_prime: F

    def with_zero_camber(self) -> _Norm:
        zero = np.zeros_like(self.gamma)
        return _Norm(
            kappa=self.kappa,
            alpha_star=self.alpha_star,
            gamma=zero,
            gamma_sq=zero,
            gamma_star=zero,
            gamma_star_sq=zero,
            gamma_star_abs=zero,
            fz=self.fz,
            dfz=self.dfz,
            dfz_sq=self.dfz_sq,
            dpi=self.dpi,
            p_ratio=self.p_ratio,
            sgn_vcx=self.sgn_vcx,
            cos_alpha=self.cos_alpha,
            vx_abs=self.vx_abs,
            lmux_eff=self.lmux_eff,
            lmuy_eff=self.lmuy_eff,
            lmux_prime=self.lmux_prime,
            lmuy_prime=self.lmuy_prime,
        )


@dataclass
class _Fx0Out:
    fx0: F
    k_xk: F


@dataclass
class _Fy0Out:
    fy0: F
    k_ya_p: F
    by: F
    cy: F
    shy: F
    svy: F
    mu_y: F


def _norm(
    p: dict[str, float], kappa: F, alpha: F, gamma: F, fz: F, pres: F, vx: F
) -> _Norm:
    fz0p = p["LFZO"] * p["FNOMIN"]
    alpha_c = np.clip(alpha, -_ALPHA_MAX, _ALPHA_MAX)
    sgn_vcx = _sgn_pos(vx)
    dfz = (fz - fz0p) / fz0p
    if p["_HAS_NOMPRES"] != 0.0:
        dpi = (pres - p["NOMPRES"]) / p["NOMPRES"]
        p_ratio = np.maximum(pres / p["NOMPRES"], _P_RATIO_FLOOR)
    else:
        dpi = np.zeros_like(fz)
        p_ratio = np.ones_like(fz)

    lmux_eff = np.full_like(fz, p["LMUX"])
    lmuy_eff = np.full_like(fz, p["LMUY"])

    def digress(lam: F) -> F:
        return _A_MU * lam / (1.0 + (_A_MU - 1.0) * lam)

    gamma_star = np.sin(gamma)
    return _Norm(
        kappa=kappa,
        alpha_star=np.tan(alpha_c) * sgn_vcx,
        gamma=gamma,
        gamma_sq=gamma * gamma,
        gamma_star=gamma_star,
        gamma_star_sq=gamma_star * gamma_star,
        gamma_star_abs=np.abs(gamma_star),
        fz=fz,
        dfz=dfz,
        dfz_sq=dfz * dfz,
        dpi=dpi,
        p_ratio=p_ratio,
        sgn_vcx=sgn_vcx,
        cos_alpha=np.cos(alpha_c),
        vx_abs=np.abs(vx),
        lmux_eff=lmux_eff,
        lmuy_eff=lmuy_eff,
        lmux_prime=digress(lmux_eff),
        lmuy_prime=digress(lmuy_eff),
    )


def _fx0(p: dict[str, float], n: _Norm) -> _Fx0Out:
    """Pure longitudinal slip Fx0 (eqs. 4.E9–4.E18)."""
    s_hx = (p["PHX1"] + p["PHX2"] * n.dfz) * p["LHX"]
    kx = n.kappa + s_hx

    cx = p["PCX1"] * p["LCX"]
    mu_x = (
        (p["PDX1"] + p["PDX2"] * n.dfz)
        * (1.0 + p["PPX3"] * n.dpi + p["PPX4"] * n.dpi * n.dpi)
        * (1.0 - p["PDX3"] * n.gamma_sq)
        * n.lmux_eff
    )
    dx = mu_x * n.fz

    ex = np.minimum(
        (p["PEX1"] + p["PEX2"] * n.dfz + p["PEX3"] * n.dfz_sq)
        * (1.0 - p["PEX4"] * _sgn_pos(kx))
        * p["LEX"],
        1.0,
    )

    k_xk = (
        n.fz
        * (p["PKX1"] + p["PKX2"] * n.dfz)
        * np.exp(np.minimum(p["PKX3"] * n.dfz, _EXP_MAX))
        * (1.0 + p["PPX1"] * n.dpi + p["PPX2"] * n.dpi * n.dpi)
        * p["LKX"]
    )
    bx = k_xk / _safe_denom(cx * dx)

    s_vx = n.fz * (p["PVX1"] + p["PVX2"] * n.dfz) * p["LVX"] * n.lmux_prime

    arg = bx * kx
    fx0 = dx * np.sin(cx * np.arctan(arg - ex * (arg - np.arctan(arg)))) + s_vx
    return _Fx0Out(fx0=fx0, k_xk=k_xk)


def _fy0(p: dict[str, float], n: _Norm) -> _Fy0Out:
    """Pure lateral slip Fy0 (eqs. 4.E19–4.E30)."""
    fz0p = p["LFZO"] * p["FNOMIN"]
    gs = n.gamma_star

    load_arg = (n.fz / fz0p) / _safe_denom(
        (p["PKY2"] + p["PKY5"] * n.gamma_star_sq) * (1.0 + p["PPY2"] * n.dpi)
    )
    k_ya = (
        p["PKY1"]
        * fz0p
        * (1.0 + p["PPY1"] * n.dpi)
        * (1.0 - p["PKY3"] * n.gamma_star_abs)
        * np.sin(p["PKY4"] * np.arctan(load_arg))
        * p["LKY"]
    )
    k_ya_p = _safe_denom(k_ya)

    k_yg0 = (
        n.fz * (p["PKY6"] + p["PKY7"] * n.dfz) * (1.0 + p["PPY5"] * n.dpi) * p["LKYC"]
    )

    mu_y = (
        (p["PDY1"] + p["PDY2"] * n.dfz)
        * (1.0 + p["PPY3"] * n.dpi + p["PPY4"] * n.dpi * n.dpi)
        * (1.0 - p["PDY3"] * n.gamma_star_sq)
        * n.lmuy_eff
    )
    cy = p["PCY1"] * p["LCY"]
    dy = mu_y * n.fz

    s_vyg = n.fz * (p["PVY3"] + p["PVY4"] * n.dfz) * gs * p["LKYC"] * n.lmuy_prime
    s_vy = n.fz * (p["PVY1"] + p["PVY2"] * n.dfz) * p["LVY"] * n.lmuy_prime + s_vyg

    shy = (p["PHY1"] + p["PHY2"] * n.dfz) * p["LHY"] + (k_yg0 * gs - s_vyg) / k_ya_p
    ay = n.alpha_star + shy

    ey = np.minimum(
        (p["PEY1"] + p["PEY2"] * n.dfz)
        * (
            1.0
            + p["PEY5"] * n.gamma_star_sq
            - (p["PEY3"] + p["PEY4"] * gs) * _sgn_pos(ay)
        )
        * p["LEY"],
        1.0,
    )

    by = k_ya / _safe_denom(cy * dy)

    arg = by * ay
    fy0 = dy * np.sin(cy * np.arctan(arg - ey * (arg - np.arctan(arg)))) + s_vy
    return _Fy0Out(
        fy0=fy0,
        k_ya_p=k_ya_p,
        by=by,
        cy=np.full_like(fy0, cy),
        shy=shy,
        svy=s_vy,
        mu_y=mu_y,
    )


def _gx_alpha(p: dict[str, float], n: _Norm) -> F:
    """Combined weight G_xα (eqs. 4.E51–4.E57)."""
    s_hxa = p["RHX1"]
    alpha_s = n.alpha_star + s_hxa
    b_xa = (
        (p["RBX1"] + p["RBX3"] * n.gamma_star_sq)
        * np.cos(np.arctan(p["RBX2"] * n.kappa))
        * p["LXAL"]
    )
    c_xa = p["RCX1"]
    e_xa = np.minimum(p["REX1"] + p["REX2"] * n.dfz, 1.0)

    num = _cos_mf(b_xa, c_xa, e_xa, alpha_s)
    den = _guard(_cos_mf(b_xa, c_xa, e_xa, np.full_like(alpha_s, s_hxa)))
    return num / den


def _gy_kappa(p: dict[str, float], n: _Norm, mu_y: F) -> tuple[F, F]:
    """Combined weight G_yκ and ply-steer shift SV_yκ (eqs. 4.E58–4.E67)."""
    gs = n.gamma_star

    s_hyk = p["RHY1"] + p["RHY2"] * n.dfz
    kappa_s = n.kappa + s_hyk
    b_yk = (
        (p["RBY1"] + p["RBY4"] * n.gamma_star_sq)
        * np.cos(np.arctan(p["RBY2"] * (n.alpha_star - p["RBY3"])))
        * p["LYKA"]
    )
    c_yk = p["RCY1"]
    e_yk = np.minimum(p["REY1"] + p["REY2"] * n.dfz, 1.0)

    d_vyk = (
        mu_y
        * n.fz
        * (p["RVY1"] + p["RVY2"] * n.dfz + p["RVY3"] * gs)
        * np.cos(np.arctan(p["RVY4"] * n.alpha_star))
    )
    s_vyk = d_vyk * np.sin(p["RVY5"] * np.arctan(p["RVY6"] * n.kappa)) * p["LVYKA"]

    num = _cos_mf(b_yk, c_yk, e_yk, kappa_s)
    den = _guard(_cos_mf(b_yk, c_yk, e_yk, s_hyk))
    return num / den, s_vyk


def _mz(
    p: dict[str, float],
    n: _Norm,
    k_xk: F,
    fy0: _Fy0Out,
    fx: F,
    fy: F,
    fy_trail: F,
) -> F:
    """Combined aligning moment Mz (eqs. 4.E31–4.E49, 4.E71–4.E78)."""
    fz0p = p["LFZO"] * p["FNOMIN"]
    gs = n.gamma_star

    s_ht = p["QHZ1"] + p["QHZ2"] * n.dfz + (p["QHZ3"] + p["QHZ4"] * n.dfz) * gs
    alpha_t = n.alpha_star + s_ht
    s_hf = fy0.shy + fy0.svy / fy0.k_ya_p
    alpha_r = n.alpha_star + s_hf

    ratio = k_xk / fy0.k_ya_p
    rk2 = ratio * ratio * n.kappa * n.kappa
    alpha_t_eq = np.sqrt(alpha_t * alpha_t + rk2) * _sgn_pos(alpha_t)
    alpha_r_eq = np.sqrt(alpha_r * alpha_r + rk2) * _sgn_pos(alpha_r)

    bt = (
        (p["QBZ1"] + p["QBZ2"] * n.dfz + p["QBZ3"] * n.dfz_sq)
        * (1.0 + p["QBZ4"] * gs + p["QBZ5"] * n.gamma_star_abs)
        * p["LKY"]
        / _safe_denom(n.lmuy_eff)
    )
    ct = p["QCZ1"]
    dt = (
        n.fz
        * (p["UNLOADED_RADIUS"] / fz0p)
        * (p["QDZ1"] + p["QDZ2"] * n.dfz)
        * (1.0 - p["PPZ1"] * n.dpi)
        * (1.0 + p["QDZ3"] * n.gamma_star_abs + p["QDZ4"] * n.gamma_star_sq)
        * p["LTR"]
        * n.sgn_vcx
    )

    # Et from the BASE trail angle (adopted convention; not clamped to 1).
    et = (p["QEZ1"] + p["QEZ2"] * n.dfz + p["QEZ3"] * n.dfz_sq) * (
        1.0 + (p["QEZ4"] + p["QEZ5"] * gs) * _TWO_OVER_PI * np.arctan(bt * ct * alpha_t)
    )

    def trail(x: F) -> F:
        arg = bt * x
        return (
            dt * np.cos(ct * np.arctan(arg - et * (arg - np.arctan(arg)))) * n.cos_alpha
        )

    br = p["QBZ9"] * p["LKY"] / _safe_denom(n.lmuy_eff) + p["QBZ10"] * fy0.by * fy0.cy
    dr = (
        n.fz
        * p["UNLOADED_RADIUS"]
        * (
            (p["QDZ6"] + p["QDZ7"] * n.dfz) * p["LRES"]
            + (
                (p["QDZ8"] + p["QDZ9"] * n.dfz) * (1.0 + p["PPZ2"] * n.dpi)
                + (p["QDZ10"] + p["QDZ11"] * n.dfz) * n.gamma_star_abs
            )
            * gs
            * p["LKZC"]
        )
        * n.lmuy_eff
        * n.sgn_vcx
        * n.cos_alpha
    )

    def mzr(x: F) -> F:
        return dr * np.cos(np.arctan(br * x))

    # Fx lever arm s (4.E76), camber term dropped per the operational convention.
    s = p["UNLOADED_RADIUS"] * (p["SSZ1"] + p["SSZ2"] * (fy / fz0p)) * p["LS"]
    s_fx = np.where(n.kappa == 0.0, 0.0, s * fx)

    return -trail(alpha_t_eq) * fy_trail + mzr(alpha_r_eq) + s_fx


def _mx(p: dict[str, float], n: _Norm, fy: F) -> F:
    """Overturning moment Mx (eq. 4.E69, book ``(atan x)²`` form)."""
    fz0 = p["FNOMIN"]
    fy_n = fy / fz0
    fz_n = n.fz / fz0

    a1 = p["QSX1"] * p["LVMX"]
    a2 = p["QSX2"] * n.gamma * (1.0 + p["PPMX1"] * n.dpi)
    a3 = p["QSX3"] * fy_n
    atan_load = np.arctan(p["QSX6"] * fz_n)
    a4 = (
        p["QSX4"]
        * np.cos(p["QSX5"] * atan_load * atan_load)
        * np.sin(p["QSX7"] * n.gamma + p["QSX8"] * np.arctan(p["QSX9"] * fy_n))
    )
    a5 = p["QSX10"] * np.arctan(p["QSX11"] * fz_n) * n.gamma

    return p["UNLOADED_RADIUS"] * n.fz * p["LMX"] * (a1 - a2 + a3 + a4 + a5)


def _my(p: dict[str, float], n: _Norm, fx: F) -> F:
    """Rolling-resistance moment My (eq. 4.E70); opposes rotation (< 0 at V_cx > 0)."""
    fz0 = p["FNOMIN"]
    v_ratio = n.vx_abs / p["LONGVL"]
    fz_n = n.fz / fz0

    poly = (
        p["QSY1"]
        + p["QSY2"] * (fx / fz0)
        + p["QSY3"] * v_ratio
        + p["QSY4"] * v_ratio**4
        + (p["QSY5"] + p["QSY6"] * fz_n) * n.gamma_sq
    )
    load_pressure = (
        fz_n ** p["QSY7"] * np.maximum(n.p_ratio, _P_RATIO_FLOOR) ** p["QSY8"]
    )

    return -n.sgn_vcx * n.fz * p["UNLOADED_RADIUS"] * p["LMY"] * poly * load_pressure


def forces(
    p: dict[str, float],
    kappa: F,
    alpha: F,
    gamma: F,
    fz: F,
    pres: F,
    vx: F,
) -> Forces:
    """Evaluate the steady-state MF6.1 channels at the given contact-patch states.

    Inputs broadcast like numpy arrays (SI, ISO-W). ``fz <= 0`` rows return exactly zero on
    every channel, matching the Rust kernel's airborne-wheel contract.
    """
    kappa, alpha, gamma, fz_in, pres, vx = np.broadcast_arrays(
        np.asarray(kappa, dtype=np.float64),
        np.asarray(alpha, dtype=np.float64),
        np.asarray(gamma, dtype=np.float64),
        np.asarray(fz, dtype=np.float64),
        np.asarray(pres, dtype=np.float64),
        np.asarray(vx, dtype=np.float64),
    )
    loaded = fz_in > 0.0
    # Evaluate with a safe positive stand-in load where airborne, then zero those rows.
    fz_eval = np.where(loaded, fz_in, 1.0)

    n = _norm(p, kappa, alpha, gamma, fz_eval, pres, vx)

    fx0 = _fx0(p, n)
    fy0 = _fy0(p, n)

    g_xa = _gx_alpha(p, n)
    g_yk, sv_yk = _gy_kappa(p, n, fy0.mu_y)

    fx = g_xa * fx0.fx0
    fy = g_yk * fy0.fy0 + sv_yk

    # Zero-camber lateral bundle for the Mz trail/residual (identical to the cambered one when
    # γ = 0, so evaluating it unconditionally matches the Rust skip-if-zero optimization).
    n0 = n.with_zero_camber()
    fy0_0 = _fy0(p, n0)
    g_yk0, _ = _gy_kappa(p, n0, fy0_0.mu_y)
    fy_trail = g_yk0 * fy0_0.fy0

    mz = _mz(p, n, fx0.k_xk, fy0_0, fx, fy, fy_trail)
    mx = _mx(p, n, fy)
    my = _my(p, n, fx)

    zero = np.zeros_like(fx)
    return Forces(
        fx=np.where(loaded, fx, zero),
        fy=np.where(loaded, fy, zero),
        mz=np.where(loaded, mz, zero),
        mx=np.where(loaded, mx, zero),
        my=np.where(loaded, my, zero),
    )
