// SPDX-License-Identifier: AGPL-3.0-only
//! Steady-state MF6.1 evaluation (Pacejka 2012, 3rd ed., §4.3.2 "Full set of equations").
//!
//! Composition per evaluation: normalize inputs → pure-slip `Fx0`/`Fy0` → combined-slip cosine
//! weighting (`G_xα`, `G_yκ`, `SV_yκ`) → aligning moment from the shared `Fy` machinery →
//! `Mx`/`My`. Turn-slip is omitted in v1: every ζ factor of the book equations is unity; they
//! are written as named constants at their use sites so a later turn-slip upgrade is a diff,
//! not a rewrite.
//!
//! Equation-number anchors (4.E9, 4.E19, 4.E25, 4.E31, 4.E69, 4.E70, 4.E71–4.E78) are cited at
//! each kernel. Numbers marked `(~)` in kernel doc comments are transcribed from working
//! knowledge of the 3rd edition and must be verified against the physical book; where the book
//! and the MFeval oracle disagree, check the published 3rd-edition errata (the `SHy`/4.E27
//! region is the documented hotspot) before changing code.

pub mod params;
pub mod peak;

mod combined;
mod fx;
mod fy;
mod mxmy;
mod mz;

use num_traits::Float;
use outlap_schema::tyr::Tyr;

use outlap_schema::load::report::ReportEntry;

use crate::slip::{sgn_pos, SlipState, TireForces};
use params::{Mf61BuildError, Mf61Params};

/// Constants derived once at construction so the evaluation path does no `T::from` conversions.
#[derive(Clone, Debug)]
struct Precomp<T> {
    /// Scaled nominal load `F'_z0 = λ_Fz0 · F_z0` (eq. 4.E1).
    fz0p: T,
    /// `1 / F'_z0`.
    inv_fz0p: T,
    /// Singularity guard ε added sign-preservingly to denominators (book's ε_x/ε_y/ε_K device).
    eps: T,
    /// Upper clamp for `exp` arguments so `Kxκ`'s `exp(PKX3·dfz)` cannot overflow to `+inf`
    /// (which poisons the atan magic formula into `inf − inf = NaN`) at extreme load.
    exp_max: T,
    /// `2/π` (eq. 4.E44 trail curvature term).
    two_over_pi: T,
    /// Side-slip clamp bound, `π/2 − 10⁻³` rad (beyond it `tan α` is meaningless).
    alpha_max: T,
    /// Digressive-friction constant `A_μ = 10` (eq. 4.E8).
    a_mu: T,
    /// Floor for the pressure ratio `p/p0` under the `QSY8` power law (finite-output guard).
    p_ratio_floor: T,
}

/// Normalized per-evaluation quantities shared by every kernel.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Norm<T> {
    pub kappa: T,
    /// `α* = tan α · sgn(V_cx)` (eq. 4.E3), with α clamped to ±(π/2 − 10⁻³).
    pub alpha_star: T,
    /// Raw camber γ, rad (used where the book uses γ itself: eqs. 4.E13, 4.E69).
    pub gamma: T,
    /// `γ²` (raw camber squared), precomputed — shared by the Fx0 and My camber terms.
    pub gamma_sq: T,
    /// `γ* = sin γ` (eq. 4.E4).
    pub gamma_star: T,
    /// `γ*²`, precomputed — shared across Fy0, Mz, and both combined weights.
    pub gamma_star_sq: T,
    /// `|γ*|`, precomputed — shared across Fy0 and Mz.
    pub gamma_star_abs: T,
    pub fz: T,
    /// `dfz = (F_z − F'_z0)/F'_z0` (eq. 4.E2a).
    pub dfz: T,
    /// `dfz²`, precomputed — shared across the Fx0 and Mz load polynomials.
    pub dfz_sq: T,
    /// `dpi = (p − p0)/p0` (eq. 4.E2b); ≡ 0 when `NOMPRES` is absent.
    pub dpi: T,
    /// `p/p0` for the My power law; ≡ 1 when `NOMPRES` is absent.
    pub p_ratio: T,
    /// `sgn(V_cx)` with `sgn(0) = +1`.
    pub sgn_vcx: T,
    /// `cos'α` — for velocity-consistent inputs this equals `cos α` (guarded surrogate for
    /// `V_cx/V_c`; keeps the trail bounded at large slip angles, eq. 4.E33 region ~).
    pub cos_alpha: T,
    /// `|V_cx|`, m/s.
    pub vx_abs: T,
    /// Effective longitudinal friction scaling `λ*_μx = LMUX · mu_scale_x` (eq. 4.E7 with the
    /// velocity-digressive factor omitted — no `LMUV` in the v1 coefficient set).
    pub lmux_eff: T,
    /// Effective lateral friction scaling `λ*_μy = LMUY · mu_scale_y`.
    pub lmuy_eff: T,
    /// Digressive `λ'_μx = A_μ λ*_μx / (1 + (A_μ − 1) λ*_μx)` (eq. 4.E8), applied to `SV_x`.
    pub lmux_prime: T,
    /// Digressive `λ'_μy` (eq. 4.E8), applied to `SV_y`/`SV_yγ`.
    pub lmuy_prime: T,
}

/// Sign-preserving singularity guard: `d + ε·sgn(d)` with `sgn(0) = +1`.
///
/// Never cancels the denominator (unlike `d + ε`, which is zero at `d = −ε`).
#[inline]
pub(crate) fn safe_denom<T: Float>(d: T, eps: T) -> T {
    d + eps * sgn_pos(d)
}

/// A steady-state MF6.1 tire model, ready for allocation-free evaluation.
#[derive(Clone, Debug)]
pub struct Mf61<T> {
    p: Mf61Params<T>,
    pre: Precomp<T>,
}

impl<T: Float> Mf61<T> {
    /// Build the model from an extracted parameter set.
    pub fn new(p: Mf61Params<T>) -> Self {
        let one = T::one();
        let conv = |x: f64| T::from(x).unwrap_or_else(T::zero);
        let fz0p = p.lfzo * p.fnomin;
        let pre = Precomp {
            fz0p,
            inv_fz0p: one / fz0p,
            eps: conv(1e-6),
            exp_max: conv(80.0),
            two_over_pi: conv(core::f64::consts::FRAC_2_PI),
            alpha_max: conv(core::f64::consts::FRAC_PI_2 - 1e-3),
            a_mu: conv(10.0),
            p_ratio_floor: conv(1e-6),
        };
        Self { p, pre }
    }

    /// Build directly from a loaded `.tyr` document (parameter extraction + notes).
    pub fn from_tyr(tyr: &Tyr) -> Result<(Self, Vec<ReportEntry>), Mf61BuildError> {
        let (p, notes) = Mf61Params::from_tyr(tyr)?;
        Ok((Self::new(p), notes))
    }

    /// The underlying parameter set.
    pub fn params(&self) -> &Mf61Params<T> {
        &self.p
    }

    /// Evaluate steady-state forces and moments at the given contact-patch state.
    ///
    /// Pure, allocation-free, and finite for all finite inputs; `F_z ≤ 0` (airborne wheel)
    /// returns exactly zero for every channel.
    pub fn forces(&self, s: &SlipState<T>) -> TireForces<T> {
        debug_assert!(s.fz.is_finite(), "Fz must be finite");
        if s.fz <= T::zero() {
            return TireForces::zero();
        }

        let n = self.norm(s);
        let p = &self.p;
        let pre = &self.pre;

        let fx0 = fx::fx0(p, pre, &n);
        let fy0 = fy::fy0(p, pre, &n);

        let g_xa = combined::gx_alpha(p, pre, &n);
        let (g_yk, sv_yk) = combined::gy_kappa(p, pre, &n, fy0.mu_y);

        let fx = g_xa * fx0.fx0;
        let fy = g_yk * fy0.fy0 + sv_yk;

        let mz = mz::mz(p, pre, &n, fx0.k_xk, &fy0, fx, fy, sv_yk);
        let mx = mxmy::mx(p, &n, fy);
        let my = mxmy::my(p, pre, &n, fx);

        TireForces { fx, fy, mz, mx, my }
    }

    fn norm(&self, s: &SlipState<T>) -> Norm<T> {
        let p = &self.p;
        let pre = &self.pre;
        let one = T::one();

        let alpha_c = s.alpha.max(-pre.alpha_max).min(pre.alpha_max);
        let sgn_vcx = sgn_pos(s.vx);
        let dfz = (s.fz - pre.fz0p) * pre.inv_fz0p;
        let (dpi, p_ratio) = if p.has_nompres {
            (
                (s.p - p.nompres) / p.nompres,
                (s.p / p.nompres).max(pre.p_ratio_floor),
            )
        } else {
            (T::zero(), one)
        };

        let lmux_eff = p.lmux * s.mu_scale_x;
        let lmuy_eff = p.lmuy * s.mu_scale_y;
        let digress = |l: T| pre.a_mu * l / (one + (pre.a_mu - one) * l);

        let gamma_star = s.gamma.sin();

        Norm {
            kappa: s.kappa,
            alpha_star: alpha_c.tan() * sgn_vcx,
            gamma: s.gamma,
            gamma_sq: s.gamma * s.gamma,
            gamma_star,
            gamma_star_sq: gamma_star * gamma_star,
            gamma_star_abs: gamma_star.abs(),
            fz: s.fz,
            dfz,
            dfz_sq: dfz * dfz,
            dpi,
            p_ratio,
            sgn_vcx,
            cos_alpha: alpha_c.cos(),
            vx_abs: s.vx.abs(),
            lmux_eff,
            lmuy_eff,
            lmux_prime: digress(lmux_eff),
            lmuy_prime: digress(lmuy_eff),
        }
    }
}
