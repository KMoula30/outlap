// SPDX-License-Identifier: AGPL-3.0-only
//! Combined-slip cosine weighting — Pacejka 2012 eqs. 4.E50–4.E67.
//!
//! `Fx = G_xα · Fx0` (eqs. 4.E50–4.E57 ~) and `Fy = G_yκ · Fy0 + SV_yκ` (eqs. 4.E58–4.E67 ~).
//! The weighting functions are normalized cosines of a magic formula in the *other* slip
//! quantity; each normalizing denominator is the cosine at the shifted origin and is ε-guarded
//! (magnitude floor, sign preserved) — a hostile-but-plausible parameter set can genuinely
//! drive it toward zero.

use num_traits::Float;

use super::{Norm, Precomp};
use crate::mf61::params::Mf61Params;
use crate::slip::sgn_pos;

/// The cosine magic formula `cos(C·atan(B·x − E·(B·x − atan(B·x))))` shared by both weights.
#[inline]
fn cos_mf<T: Float>(b: T, c: T, e: T, x: T) -> T {
    let arg = b * x;
    (c * (arg - e * (arg - arg.atan())).atan()).cos()
}

/// Magnitude-floored, sign-preserving guard for the normalizing cosines.
#[inline]
fn guard<T: Float>(d: T, eps: T) -> T {
    if d.abs() < eps {
        eps * sgn_pos(d)
    } else {
        d
    }
}

/// Longitudinal weight `G_xα` (eqs. 4.E51–4.E57 ~): the α-dependent reduction of `Fx0`.
pub(crate) fn gx_alpha<T: Float>(p: &Mf61Params<T>, pre: &Precomp<T>, n: &Norm<T>) -> T {
    let one = T::one();

    let s_hxa = p.rhx1; // 4.E57 ~
    let alpha_s = n.alpha_star + s_hxa; // 4.E53 ~
    let b_xa = (p.rbx1 + p.rbx3 * n.gamma_star_sq) * (p.rbx2 * n.kappa).atan().cos() * p.lxal; // 4.E54 ~
    let c_xa = p.rcx1; // 4.E55 ~
    let e_xa = (p.rex1 + p.rex2 * n.dfz).min(one); // 4.E56 ~, clamp ≤ 1

    let num = cos_mf(b_xa, c_xa, e_xa, alpha_s);
    let den = guard(cos_mf(b_xa, c_xa, e_xa, s_hxa), pre.eps); // 4.E52 ~
    num / den
}

/// Lateral weight `G_yκ` and the κ-induced ply-steer shift `SV_yκ` (eqs. 4.E58–4.E67 ~).
pub(crate) fn gy_kappa<T: Float>(
    p: &Mf61Params<T>,
    pre: &Precomp<T>,
    n: &Norm<T>,
    mu_y: T,
) -> (T, T) {
    let one = T::one();
    let gs = n.gamma_star;

    let s_hyk = p.rhy1 + p.rhy2 * n.dfz; // 4.E65 ~
    let kappa_s = n.kappa + s_hyk; // 4.E61 ~
    let b_yk = (p.rby1 + p.rby4 * n.gamma_star_sq)
        * (p.rby2 * (n.alpha_star - p.rby3)).atan().cos()
        * p.lyka; // 4.E62 ~
    let c_yk = p.rcy1; // 4.E63 ~
    let e_yk = (p.rey1 + p.rey2 * n.dfz).min(one); // 4.E64 ~, clamp ≤ 1

    // DVyκ (4.E67 ~, ζ2 = 1) and SVyκ (4.E66 ~).
    let d_vyk = mu_y
        * n.fz
        * (p.rvy1 + p.rvy2 * n.dfz + p.rvy3 * gs)
        * (p.rvy4 * n.alpha_star).atan().cos();
    let s_vyk = d_vyk * (p.rvy5 * (p.rvy6 * n.kappa).atan()).sin() * p.lvyka;

    let num = cos_mf(b_yk, c_yk, e_yk, kappa_s);
    let den = guard(cos_mf(b_yk, c_yk, e_yk, s_hyk), pre.eps); // 4.E60 ~
    (num / den, s_vyk)
}
