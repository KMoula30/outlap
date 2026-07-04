// SPDX-License-Identifier: AGPL-3.0-only
//! Aligning moment `Mz` — Pacejka 2012 eqs. 4.E31–4.E49 (pure) and 4.E71–4.E78 (combined).
//!
//! `Mz = −t(α_t,eq)·F'_y + M_zr(α_r,eq) + s·F_x` (eq. 4.E71): pneumatic-trail moment on the
//! κ-free part of the lateral force, plus the residual torque, plus the `Fx` lever arm from
//! the lateral carcass deflection. The equivalent slip angles (eqs. 4.E77/4.E78) fold the
//! longitudinal slip into the trail/residual arguments via the stiffness ratio `K_xκ/K'_yα`.
//!
//! `Mz` is restoring (`> 0` for α > 0) *because* `F_y` is negative there — the sign arithmetic
//! relies on the ISO-W contract in [`crate::slip`]; no absolute values. Reverse running enters
//! only through the `sgn(V_cx)` factors inside `D_t` (eq. 4.E42 ~) and `D_r` (eq. 4.E47 ~).
//! The trail and residual both carry the `cos'α` weighting that keeps `Mz` bounded at large
//! slip angles (eq. 4.E33 region ~); for velocity-consistent inputs `cos'α = cos α`.
//!
//! Turn-slip factors `ζ0, ζ2, ζ5..ζ8 ≡ 1` (omitted in v1).

use num_traits::Float;

use super::fy::Fy0Out;
use super::{safe_denom, Norm, Precomp};
use crate::mf61::params::Mf61Params;
use crate::slip::sgn_pos;

/// Evaluate the combined-slip aligning moment (eq. 4.E71).
#[allow(clippy::too_many_arguments)] // The Mz composition genuinely consumes all of these.
pub(crate) fn mz<T: Float>(
    p: &Mf61Params<T>,
    pre: &Precomp<T>,
    n: &Norm<T>,
    k_xk: T,
    fy0: &Fy0Out<T>,
    fx: T,
    fy: T,
    sv_yk: T,
) -> T {
    let one = T::one();
    let gs = n.gamma_star;

    // Trail and residual slip angles (4.E34/4.E35 ~, 4.E37/4.E38 ~).
    let s_ht = p.qhz1 + p.qhz2 * n.dfz + (p.qhz3 + p.qhz4 * n.dfz) * gs;
    let alpha_t = n.alpha_star + s_ht;
    let s_hf = fy0.shy + fy0.svy / fy0.k_ya_p;
    let alpha_r = n.alpha_star + s_hf;

    // Equivalent slip angles folding κ in via the stiffness ratio (4.E77/4.E78).
    let ratio = k_xk / fy0.k_ya_p;
    let rk2 = ratio * ratio * n.kappa * n.kappa;
    let alpha_t_eq = (alpha_t * alpha_t + rk2).sqrt() * sgn_pos(alpha_t);
    let alpha_r_eq = (alpha_r * alpha_r + rk2).sqrt() * sgn_pos(alpha_r);

    // Trail MF factors: Bt (4.E40 ~; camber form 1 + qBz4·γ* + qBz5·|γ*|), Ct (4.E41),
    // Dt (4.E42/4.E43 ~ with the R0/F'z0 normalization, Besselink PPZ1, sgn(Vcx)).
    let bt = (p.qbz1 + p.qbz2 * n.dfz + p.qbz3 * n.dfz_sq)
        * (one + p.qbz4 * gs + p.qbz5 * n.gamma_star_abs)
        * p.lky
        / safe_denom(n.lmuy_eff, pre.eps);
    let ct = p.qcz1;
    let dt = n.fz
        * (p.r0 * pre.inv_fz0p)
        * (p.qdz1 + p.qdz2 * n.dfz)
        * (one - p.ppz1 * n.dpi)
        * (one + p.qdz3 * n.gamma_star_abs + p.qdz4 * n.gamma_star_sq)
        * p.ltr
        * n.sgn_vcx;

    // Trail t(x) (4.E33 ~/4.E44 ~): Et carries the (2/π)·atan(Bt·Ct·x) curvature term and the
    // ≤ 1 clamp; the cos'α factor bounds the trail at large slip.
    let trail = |x: T| -> T {
        let et = ((p.qez1 + p.qez2 * n.dfz + p.qez3 * n.dfz_sq)
            * (one + (p.qez4 + p.qez5 * gs) * pre.two_over_pi * (bt * ct * x).atan()))
        .min(one);
        let arg = bt * x;
        dt * (ct * (arg - et * (arg - arg.atan())).atan()).cos() * n.cos_alpha
    };

    // Residual torque: Br (4.E45 ~), Cr = 1 (ζ7), Dr (4.E47 ~ with ζ ≡ 1).
    let br = p.qbz9 * p.lky / safe_denom(n.lmuy_eff, pre.eps) + p.qbz10 * fy0.by * fy0.cy;
    let dr = n.fz
        * p.r0
        * ((p.qdz6 + p.qdz7 * n.dfz) * p.lres
            + ((p.qdz8 + p.qdz9 * n.dfz) * (one + p.ppz2 * n.dpi)
                + (p.qdz10 + p.qdz11 * n.dfz) * n.gamma_star_abs)
                * gs
                * p.lkzc)
        * n.lmuy_eff
        * n.sgn_vcx
        * n.cos_alpha;
    let mzr = |x: T| -> T { dr * (br * x).atan().cos() };

    // Fx lever arm s (4.E76 ~) and the κ-free lateral force F'y (4.E74).
    let s = p.r0 * (p.ssz1 + p.ssz2 * (fy * pre.inv_fz0p) + (p.ssz3 + p.ssz4 * n.dfz) * gs) * p.ls;
    let fy_prime = fy - sv_yk;

    // Mz (4.E71).
    -trail(alpha_t_eq) * fy_prime + mzr(alpha_r_eq) + s * fx
}
