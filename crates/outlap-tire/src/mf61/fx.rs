// SPDX-License-Identifier: AGPL-3.0-only
//! Pure longitudinal slip `Fx0` — Pacejka 2012 eqs. 4.E9–4.E18.
//!
//! Turn-slip factor `ζ1 ≡ 1` (omitted in v1). Besselink inflation-pressure terms enter the
//! peak factor (`PPX3`/`PPX4`, eq. 4.E13 ~) and the slip stiffness (`PPX1`/`PPX2`, eq. 4.E15).

use num_traits::Float;

use super::{safe_denom, Norm, Precomp};
use crate::mf61::params::Mf61Params;
use crate::slip::sgn_pos;

/// Pure-slip longitudinal outputs shared with the combined/Mz stages.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Fx0Out<T> {
    /// `Fx0`, N (eq. 4.E9).
    pub fx0: T,
    /// Longitudinal slip stiffness `K_xκ = ∂Fx0/∂κ |₀`, N (eq. 4.E15); > 0 in ISO-W.
    pub k_xk: T,
}

/// Evaluate `Fx0` (eq. 4.E9) and its slip stiffness at the normalized state.
pub(crate) fn fx0<T: Float>(p: &Mf61Params<T>, pre: &Precomp<T>, n: &Norm<T>) -> Fx0Out<T> {
    let one = T::one();

    // SHx (4.E17 ~), κx (4.E10).
    let s_hx = (p.phx1 + p.phx2 * n.dfz) * p.lhx;
    let kx = n.kappa + s_hx;

    // Cx (4.E11), μx (4.E13 ~; raw γ per the book's Fx camber term), Dx (4.E12, ζ1 = 1).
    let cx = p.pcx1 * p.lcx;
    let mu_x = (p.pdx1 + p.pdx2 * n.dfz)
        * (one + p.ppx3 * n.dpi + p.ppx4 * n.dpi * n.dpi)
        * (one - p.pdx3 * n.gamma_sq)
        * n.lmux_eff;
    let dx = mu_x * n.fz;

    // Ex (4.E14 ~), clamped ≤ 1 (book requirement; beyond it the sine argument folds back).
    let ex = ((p.pex1 + p.pex2 * n.dfz + p.pex3 * n.dfz_sq) * (one - p.pex4 * sgn_pos(kx)) * p.lex)
        .min(one);

    // Kxκ (4.E15), Bx (4.E16, ε-guarded). The exp argument is clamped to keep the "finite for
    // all finite inputs" contract at physically-extreme loads (PKX3·dfz large): exp overflows to
    // +inf beyond ~709, which would poison Bx → arg → the atan magic formula with inf − inf = NaN.
    let k_xk = n.fz
        * (p.pkx1 + p.pkx2 * n.dfz)
        * (p.pkx3 * n.dfz).min(pre.exp_max).exp()
        * (one + p.ppx1 * n.dpi + p.ppx2 * n.dpi * n.dpi)
        * p.lkx;
    let bx = k_xk / safe_denom(cx * dx, pre.eps);

    // SVx (4.E18): digressive λ'μx (4.E8). Book-literal; the low-speed reduction is a property of
    // the separate low-speed model (M2 PR4, keyed on VXLOW), not of the steady-state SVx.
    let s_vx = n.fz * (p.pvx1 + p.pvx2 * n.dfz) * p.lvx * n.lmux_prime;

    // Fx0 (4.E9): the magic formula proper.
    let arg = bx * kx;
    let fx0 = dx * (cx * (arg - ex * (arg - arg.atan())).atan()).sin() + s_vx;

    Fx0Out { fx0, k_xk }
}
