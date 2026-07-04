// SPDX-License-Identifier: AGPL-3.0-only
//! First-order tire relaxation: slip-channel lag and the exact-exponential update (HANDOFF §11.2).
//!
//! A tire does not reach its steady-state slip force instantly — the contact patch must roll a
//! *relaxation length* `σ` before the deflection (hence the force) catches up. Each slip channel
//! follows `σ·ẋ + |V_x|·x = |V_x|·x_ss` (Pacejka 2012 §7.2 / §8.5). The production stepper advances
//! it with the **exact-exponential** update `x ← x_ss + (x − x_ss)·exp(−|V_x|·dt/σ)`
//! ([`relax_step`]) — unconditionally stable at every speed, no implicit solve, the single most
//! important integrator decision (HANDOFF §11.2).
//!
//! # Relaxation lengths
//!
//! Longitudinal and lateral lengths come from the MF5.2 `PT*` transient coefficients when present,
//! else from the carcass-stiffness identity `σ = K_slip / C_carcass`, else a loud last-resort
//! `0.5·R0`. The route is chosen once at construction and reported. Equation forms marked `(~)` are
//! transcribed from the MF5.2/6.1 relaxation block and should be re-checked against the book PDF.

use num_traits::Float;

use crate::mf61::params::Mf61Params;
use outlap_schema::load::report::ReportEntry;

/// Minimum relaxation length, m — floors `σ` so the exponential decay stays finite at tiny/zero
/// lengths (a zero `σ` would divide by zero; a near-zero one would relax instantly and defeat the
/// point of the transient).
pub const SIGMA_FLOOR_M: f64 = 1e-3;

/// Advance one relaxation state by `dt` with the exact-exponential update (HANDOFF §11.2).
///
/// `x` is the current lagged slip, `x_ss` its steady-state target, `v_abs` the contact-patch speed
/// `|V_x|` (the caller passes `|V_x|.max(VXLOW)` so standstill still relaxes), `dt` the step (s),
/// and `sigma` the relaxation length (m, floored at [`SIGMA_FLOOR_M`]). Returns the new lagged
/// slip. Contracting toward `x_ss` for every `dt ≥ 0`; exact, so two half-steps equal one full step.
pub fn relax_step<T: Float>(x: T, x_ss: T, v_abs: T, dt: T, sigma: T) -> T {
    let zero = T::zero();
    let floor = T::from(SIGMA_FLOOR_M).unwrap_or_else(T::zero);
    let sigma = sigma.max(floor);
    // exp(−|V|·dt/σ) ∈ (0, 1] for |V|, dt ≥ 0 — a pure contraction toward x_ss.
    let decay = (-(v_abs.max(zero) * dt.max(zero)) / sigma).exp();
    x_ss + (x - x_ss) * decay
}

/// Which route supplied a relaxation length (recorded for the loaded-model report).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Route {
    /// MF5.2 `PT*` transient coefficients.
    Pt,
    /// Carcass-stiffness identity `σ = K_slip / C_carcass`.
    Carcass,
    /// Last-resort `0.5·R0` (nothing better available).
    LastResort,
}

/// Relaxation-length provider for one tire, with the route chosen once at construction.
///
/// The lengths depend on the operating point (`F_z`, `γ`), so they are evaluated per call; only the
/// *route* (and its constants) is fixed up front. Cheap to hold; the accessors are allocation-free.
#[derive(Clone, Debug)]
pub struct Relaxation<T> {
    p: Mf61Params<T>,
    route_x: Route,
    route_y: Route,
    exp_max: T,
}

impl<T: Float> Relaxation<T> {
    /// Choose relaxation routes from an MF6.1 parameter set, returning the provider plus notes.
    pub fn from_params(p: &Mf61Params<T>) -> (Self, Vec<ReportEntry>) {
        let zero = T::zero();
        let route_x = if p.ptx1 != zero {
            Route::Pt
        } else if p.c_long > zero {
            Route::Carcass
        } else {
            Route::LastResort
        };
        let route_y = if p.pty1 != zero {
            Route::Pt
        } else if p.c_lat > zero {
            Route::Carcass
        } else {
            Route::LastResort
        };
        let notes = [("σκ", route_x), ("σα", route_y)]
            .into_iter()
            .filter_map(|(sym, route)| route_note(sym, route))
            .collect();
        let this = Self {
            p: p.clone(),
            route_x,
            route_y,
            exp_max: T::from(80.0).unwrap_or_else(T::zero),
        };
        (this, notes)
    }

    /// `dfz = (F_z − F'_z0)/F'_z0` with `F'_z0 = λ_Fz0·FNOMIN`.
    fn dfz(&self, fz: T) -> T {
        let fz0p = self.p.lfzo * self.p.fnomin;
        (fz - fz0p) / fz0p
    }

    /// Longitudinal relaxation length `σ_κ`, m, at load `fz`.
    ///
    /// PT route (~): `σκ = F_z·(PTX1 + PTX2·dfz)·exp(−PTX3·dfz)·(R0/FNOMIN)·LSGKP`. Carcass route:
    /// `σκ = K_xκ / C_long`. Last resort: `0.5·R0`. Floored at [`SIGMA_FLOOR_M`].
    pub fn sigma_kappa(&self, fz: T) -> T {
        let p = &self.p;
        let raw = match self.route_x {
            Route::Pt => {
                let dfz = self.dfz(fz);
                fz * (p.ptx1 + p.ptx2 * dfz)
                    * (-p.ptx3 * dfz).min(self.exp_max).exp()
                    * (p.r0 / p.fnomin)
                    * p.lsgkp
            }
            Route::Carcass => self.k_x_kappa(fz).abs() / p.c_long,
            Route::LastResort => self.half_r0(),
        };
        floor_sigma(raw)
    }

    /// Lateral relaxation length `σ_α`, m, at load `fz` and camber `gamma` (rad).
    ///
    /// PT route (~): `σα = PTY1·sin(2·atan(F_z/(PTY2·FNOMIN·LFZO)))·(1 − PKY3·|γ*|)·R0·LFZO·LSGAL`.
    /// Carcass route: `σα = |K_yα| / C_lat`. Last resort: `0.5·R0`. Floored at [`SIGMA_FLOOR_M`].
    pub fn sigma_alpha(&self, fz: T, gamma: T) -> T {
        let p = &self.p;
        let raw = match self.route_y {
            Route::Pt => {
                let two = T::one() + T::one();
                let fz0p = p.fnomin * p.lfzo;
                let cam = T::one() - p.pky3 * gamma.sin().abs();
                p.pty1 * (two * (fz / (p.pty2 * fz0p)).atan()).sin() * cam * p.r0 * p.lfzo * p.lsgal
            }
            Route::Carcass => self.k_y_alpha(fz, gamma).abs() / p.c_lat,
            Route::LastResort => self.half_r0(),
        };
        floor_sigma(raw)
    }

    /// Nominal longitudinal slip stiffness `K_xκ = F_z·(PKX1 + PKX2·dfz)·exp(PKX3·dfz)·LKX`
    /// (eq. 4.E15, inflation terms omitted — this feeds only the carcass-stiffness fallback).
    fn k_x_kappa(&self, fz: T) -> T {
        let p = &self.p;
        let dfz = self.dfz(fz);
        fz * (p.pkx1 + p.pkx2 * dfz) * (p.pkx3 * dfz).min(self.exp_max).exp() * p.lkx
    }

    /// Nominal cornering stiffness `K_yα = PKY1·F'_z0·sin(PKY4·atan(F_z/(PKY2·F'_z0)))·(1 − PKY3|γ*|)·LKY`
    /// (eq. 4.E25, inflation terms omitted — carcass-stiffness fallback only).
    fn k_y_alpha(&self, fz: T, gamma: T) -> T {
        let p = &self.p;
        let fz0p = p.fnomin * p.lfzo;
        let cam = T::one() - p.pky3 * gamma.sin().abs();
        p.pky1 * fz0p * (p.pky4 * (fz / (p.pky2 * fz0p)).atan()).sin() * cam * p.lky
    }

    fn half_r0(&self) -> T {
        self.p.r0 * T::from(0.5).unwrap_or_else(T::zero)
    }
}

/// Floor a relaxation length at [`SIGMA_FLOOR_M`], mapping non-finite lengths to the floor.
fn floor_sigma<T: Float>(sigma: T) -> T {
    let floor = T::from(SIGMA_FLOOR_M).unwrap_or_else(T::zero);
    if sigma.is_finite() {
        sigma.max(floor)
    } else {
        floor
    }
}

/// A loaded-model note for a non-PT relaxation route (the PT route is the expected default).
fn route_note(sym: &str, route: Route) -> Option<ReportEntry> {
    match route {
        Route::Pt => None,
        Route::Carcass => Some(ReportEntry::new(
            "/mf61/PT*",
            format!("{sym}: PT* relaxation coefficients absent - using carcass-stiffness identity σ = K/C"),
        )),
        Route::LastResort => Some(ReportEntry::new(
            "/mf61/PT*",
            format!("{sym}: no PT* coefficients or carcass stiffness - falling back to σ = 0.5·R0"),
        )),
    }
}
