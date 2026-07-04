// SPDX-License-Identifier: AGPL-3.0-only
//! Overturning moment `Mx` (eq. 4.E69 ~) and rolling-resistance moment `My` (eq. 4.E70 ~).
//!
//! Both consume the *final combined* forces (`Fy` for `Mx`, `Fx` for `My`) and normalize loads
//! and forces by the unscaled nominal `Fz0 = FNOMIN` (not `F'_z0`). The Besselink pressure
//! terms enter `Mx` through `PPMX1` and `My` through the `(p/p0)^QSY8` power law.
//!
//! `My` sign: rolling resistance opposes rotation. In ISO 8855 (y left) forward rolling has
//! positive spin about +y, so `My < 0` at `V_cx > 0` — implemented as an overall
//! `−sgn(V_cx)` on a positive-magnitude polynomial. This sign is **provisional** until pinned
//! against the oracle goldens (flagged in the theory page).

use num_traits::Float;

use super::{Norm, Precomp};
use crate::mf61::params::Mf61Params;

/// Overturning moment `Mx` (eq. 4.E69 ~), from the combined lateral force.
pub(crate) fn mx<T: Float>(p: &Mf61Params<T>, n: &Norm<T>, fy: T) -> T {
    let one = T::one();
    let fz0 = p.fnomin;
    let fy_n = fy / fz0;
    let fz_n = n.fz / fz0;

    let a1 = p.qsx1 * p.lvmx;
    let a2 = p.qsx2 * n.gamma * (one + p.ppmx1 * n.dpi);
    let a3 = p.qsx3 * fy_n;
    // Book eq. 4.E69 squares the arctan: cos(QSX5·(atan(QSX6·Fz/Fz0))²). (MFeval evaluates
    // atan(x²) here instead — a known book-vs-MFeval discrepancy to reconcile against the golden
    // oracle in the goldens PR; the clean-room mandate takes the printed book form.)
    let atan_load = (p.qsx6 * fz_n).atan();
    let a4 = p.qsx4
        * (p.qsx5 * atan_load * atan_load).cos()
        * (p.qsx7 * n.gamma + p.qsx8 * (p.qsx9 * fy_n).atan()).sin();
    let a5 = p.qsx10 * (p.qsx11 * fz_n).atan() * n.gamma;

    p.r0 * n.fz * p.lmx * (a1 - a2 + a3 + a4 + a5)
}

/// Rolling-resistance moment `My` (eq. 4.E70 ~), from the combined longitudinal force.
pub(crate) fn my<T: Float>(p: &Mf61Params<T>, pre: &Precomp<T>, n: &Norm<T>, fx: T) -> T {
    let fz0 = p.fnomin;
    let v_ratio = n.vx_abs / p.longvl;
    let fz_n = n.fz / fz0;

    let poly = p.qsy1
        + p.qsy2 * (fx / fz0)
        + p.qsy3 * v_ratio
        + p.qsy4 * v_ratio * v_ratio * v_ratio * v_ratio
        + (p.qsy5 + p.qsy6 * fz_n) * n.gamma_sq;
    // Power-law load/pressure corrections; fz_n > 0 is guaranteed (Fz ≤ 0 short-circuits) and
    // p_ratio is floored at construction-time ε so the powf stays finite.
    let load_pressure = fz_n.powf(p.qsy7) * n.p_ratio.max(pre.p_ratio_floor).powf(p.qsy8);

    -n.sgn_vcx * n.fz * p.r0 * p.lmy * poly * load_pressure
}
