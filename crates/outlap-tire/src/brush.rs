// SPDX-License-Identifier: AGPL-3.0-only
//! Physical brush tire model with a parabolic pressure profile (Pacejka 2012, ch. 3).
//!
//! A first-principles alternative to the empirical MF6.1 force core: the contact patch is a row of
//! elastic bristles that deflect under slip and slide once the local shear exceeds the friction
//! bound `μ0·p(x)`. With a parabolic pressure `p(x) ∝ 1 − (x/a)²` the pure- and combined-slip
//! force integrates to a closed form parameterised by only the two tread stiffnesses `C_κ`/`C_α`,
//! the base friction `μ0`, and the contact half-length `a` — the whole [`crate::TyrBrush`] block.
//!
//! # Model (Pacejka 2012 §3.2–§3.3, combined slip)
//!
//! Theoretical slips (ε-guarded `1 + κ`, so a locked wheel is finite):
//! `σx = κ/(1+κ)`, `σy = tan α/(1+κ)`. The stiffness-weighted slip magnitude and its reduced form:
//! `‖·‖ = √((C_κ σx)² + (C_α σy)²)`, `ψ = ‖·‖ / (3 μ0 F_z)`. The force magnitude is the cubic
//! brush law `|F| = 3 μ0 F_z · ψ(1 − ψ + ψ²/3)` for `ψ < 1`, saturating at `μ0 F_z` for `ψ ≥ 1`.
//! It acts along the generalised-force direction `(+C_κ σx, −C_α σy)/‖·‖` — the longitudinal sign
//! flip is already carried by `κ` (driving `κ > 0` ⇒ `F_x > 0`), while the lateral force opposes
//! slip (`α > 0` ⇒ `F_y < 0`); the origin slopes are therefore `∂F_x/∂κ = +C_κ`, `∂F_y/∂α = −C_α`.
//!
//! The self-aligning moment uses the closed-form brush pneumatic trail
//! `t = (a/3)·(1−ψ)³/(1−ψ+ψ²/3)`, which runs from `t(0) = a/3` down to `0` at full sliding, with
//! `M_z = −t·F_y` (restoring, since `F_y < 0` for `α > 0` — the crate sign contract, see [`crate::slip`]).
//!
//! # Deliberate omissions (documented, not silent)
//!
//! Camber `γ` and inflation pressure `p` are accepted and ignored (the brush tier models neither);
//! overturning and rolling-resistance moments are `M_x = M_y ≡ 0`. These are surfaced as
//! loaded-model notes when a brush tire is assembled ([`crate::TireModel::from_tyr`]). The runtime
//! friction multipliers `mu_scale_x`/`mu_scale_y` scale `μ0` per axis (both `1.0` until the M5
//! thermal grip window); at `1.0` the model is isotropic in friction.

use num_traits::Float;

use crate::slip::{SlipState, TireForces};

/// A physical brush tire, ready for allocation-free evaluation.
#[derive(Clone, Debug)]
pub struct Brush<T> {
    /// Longitudinal tread stiffness `C_κ`, N.
    c_kappa: T,
    /// Lateral tread stiffness `C_α`, N/rad.
    c_alpha: T,
    /// Base sliding friction `μ0`.
    mu0: T,
    /// Contact half-length `a`, m.
    a: T,
    /// Denominator guard ε for `1 + κ` and the force direction.
    eps: T,
}

impl<T: Float> Brush<T> {
    /// Build from the four physical parameters (`C_κ` N, `C_α` N/rad, `μ0`, `a` m).
    pub fn new(c_kappa: T, c_alpha: T, mu0: T, a: T) -> Self {
        Self {
            c_kappa,
            c_alpha,
            mu0,
            a,
            eps: T::from(1e-6).unwrap_or_else(T::zero),
        }
    }

    /// Build from a loaded [`crate::TyrBrush`] block (the pressure profile is parabolic-only).
    pub fn from_tyr_brush(b: &outlap_schema::tyr::TyrBrush) -> Self {
        let conv = |x: f64| T::from(x).unwrap_or_else(T::zero);
        Self::new(
            conv(b.c_kappa_n),
            conv(b.c_alpha_n_per_rad),
            conv(b.mu0),
            conv(b.patch_half_length_m),
        )
    }

    /// The base friction coefficient `μ0` (the brush tier's peak `μ`, γ/p-independent).
    pub fn mu0(&self) -> T {
        self.mu0
    }

    /// Evaluate brush forces/moments at the contact-patch state.
    ///
    /// Pure, allocation-free, and finite for all finite inputs; `F_z ≤ 0` (airborne wheel) returns
    /// exactly zero. `γ` and `p` are ignored; `M_x = M_y = 0`.
    pub fn forces(&self, s: &SlipState<T>) -> TireForces<T> {
        debug_assert!(s.fz.is_finite(), "Fz must be finite");
        let zero = T::zero();
        if s.fz <= zero {
            return TireForces::zero();
        }
        let one = T::one();
        let three = T::from(3.0).unwrap_or_else(T::zero);

        // Theoretical slips with an ε-guarded 1 + κ (locked wheel κ = −1 stays finite).
        let opk = {
            let d = one + s.kappa;
            if d.abs() < self.eps {
                self.eps.copysign(d)
            } else {
                d
            }
        };
        let sigma_x = s.kappa / opk;
        let sigma_y = s.alpha.tan() / opk;

        // Stiffness-weighted generalised-force direction and its magnitude.
        let gx = self.c_kappa * sigma_x;
        let gy = self.c_alpha * sigma_y;
        let norm = (gx * gx + gy * gy).sqrt();
        if norm <= zero {
            return TireForces::zero(); // no slip → no force
        }

        // Reduced slip ψ and the cubic-law scalar k such that (Fx, Fy) = k·(+gx, −gy).
        let psi = norm / (three * self.mu0 * s.fz);
        let (k, sliding) = if psi < one {
            (one - psi + psi * psi / three, false)
        } else {
            (self.mu0 * s.fz / norm, true)
        };

        let fx = k * gx * s.mu_scale_x;
        let fy = -k * gy * s.mu_scale_y;

        // Brush pneumatic trail t(0) = a/3 → 0 at full sliding, acting on the lateral force.
        let mz = if sliding {
            zero
        } else {
            let one_minus_psi = one - psi;
            // Denominator (1 − ψ + ψ²/3) ≥ 1/3 on ψ ∈ [0, 1] — no guard needed.
            let t = (self.a / three) * one_minus_psi * one_minus_psi * one_minus_psi / k;
            -t * fy
        };

        TireForces {
            fx,
            fy,
            mz,
            mx: zero,
            my: zero,
        }
    }
}
