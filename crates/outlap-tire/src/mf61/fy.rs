// SPDX-License-Identifier: AGPL-3.0-only
//! Pure lateral slip `Fy0` — Pacejka 2012 eqs. 4.E19–4.E30.
//!
//! Turn-slip factors `ζ0, ζ2, ζ3, ζ4 ≡ 1` (omitted in v1). Besselink pressure terms enter the
//! cornering stiffness (`PPY1`/`PPY2`, eq. 4.E25), the peak factor (`PPY3`/`PPY4`, eq. 4.E23 ~)
//! and the camber stiffness (`PPY5`, eq. 4.E30 ~).
//!
//! Sign contract: `K_yα` carries the sign of `PKY1` (negative in ISO-W files) — no absolute
//! values anywhere in this module; the negative `B_y` is what makes `Fy(α > 0) < 0`.
//!
//! The horizontal shift `SHy` (eq. 4.E27 ~) is the documented 3rd-edition **errata hotspot**:
//! if a golden disagrees here, check the published errata before changing the code.

use num_traits::Float;

use super::{safe_denom, Norm, Precomp};
use crate::mf61::params::Mf61Params;
use crate::slip::sgn_pos;

/// Pure-slip lateral outputs shared with the combined and Mz stages.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Fy0Out<T> {
    /// `Fy0`, N (eq. 4.E19).
    pub fy0: T,
    /// ε-guarded cornering stiffness `K'_yα = K_yα + ε_K` (eq. 4.E25/4.E39 ~).
    pub k_ya_p: T,
    /// Stiffness factor `B_y` (eq. 4.E26); carries the sign of `PKY1`.
    pub by: T,
    /// Shape factor `C_y` (eq. 4.E21).
    pub cy: T,
    /// Horizontal shift `SH_y` (eq. 4.E27 ~).
    pub shy: T,
    /// Total vertical shift `SV_y` (eq. 4.E29).
    pub svy: T,
    /// Lateral peak friction `μ_y` (eq. 4.E23 ~), signed by the scaling only.
    pub mu_y: T,
}

/// Evaluate `Fy0` (eq. 4.E19) and the intermediates the Mz/combined stages reuse.
pub(crate) fn fy0<T: Float>(p: &Mf61Params<T>, pre: &Precomp<T>, n: &Norm<T>) -> Fy0Out<T> {
    let one = T::one();
    let gs = n.gamma_star;

    // Cornering stiffness Kyα (4.E25): the sin(pKy4·atan(...)) load curve with pressure and
    // camber corrections; the (1 − pKy3·|γ*|) factor multiplies outside the sine.
    let load_arg = n.fz * pre.inv_fz0p
        / safe_denom(
            (p.pky2 + p.pky5 * n.gamma_star_sq) * (one + p.ppy2 * n.dpi),
            pre.eps,
        );
    let k_ya = p.pky1
        * pre.fz0p
        * (one + p.ppy1 * n.dpi)
        * (one - p.pky3 * n.gamma_star_abs)
        * (p.pky4 * load_arg.atan()).sin()
        * p.lky;
    let k_ya_p = safe_denom(k_ya, pre.eps);

    // Camber stiffness Kyγ0 (4.E30 ~).
    let k_yg0 = n.fz * (p.pky6 + p.pky7 * n.dfz) * (one + p.ppy5 * n.dpi) * p.lkyc;

    // μy (4.E23 ~; γ* per the book's lateral camber terms), Cy (4.E21), Dy (4.E22, ζ2 = 1).
    let mu_y = (p.pdy1 + p.pdy2 * n.dfz)
        * (one + p.ppy3 * n.dpi + p.ppy4 * n.dpi * n.dpi)
        * (one - p.pdy3 * n.gamma_star_sq)
        * n.lmuy_eff;
    let cy = p.pcy1 * p.lcy;
    let dy = mu_y * n.fz;

    // Vertical shifts (4.E28/4.E29) with digressive λ'μy (4.E8).
    let s_vyg = n.fz * (p.pvy3 + p.pvy4 * n.dfz) * gs * p.lkyc * n.lmuy_prime;
    let s_vy = n.fz * (p.pvy1 + p.pvy2 * n.dfz) * p.lvy * n.lmuy_prime + s_vyg;

    // Horizontal shift SHy (4.E27 ~ — ERRATA HOTSPOT) and shifted slip αy (4.E20).
    let shy = (p.phy1 + p.phy2 * n.dfz) * p.lhy + (k_yg0 * gs - s_vyg) / k_ya_p;
    let ay = n.alpha_star + shy;

    // Ey (4.E24 ~), clamped ≤ 1; the sgn(αy) term is what skews the curve.
    let ey = ((p.pey1 + p.pey2 * n.dfz)
        * (one + p.pey5 * n.gamma_star_sq - (p.pey3 + p.pey4 * gs) * sgn_pos(ay))
        * p.ley)
        .min(one);

    // By (4.E26, ε-guarded; sign of Kyα preserved).
    let by = k_ya / safe_denom(cy * dy, pre.eps);

    // Fy0 (4.E19).
    let arg = by * ay;
    let fy0 = dy * (cy * (arg - ey * (arg - arg.atan())).atan()).sin() + s_vy;

    Fy0Out {
        fy0,
        k_ya_p,
        by,
        cy,
        shy,
        svy: s_vy,
        mu_y,
    }
}
