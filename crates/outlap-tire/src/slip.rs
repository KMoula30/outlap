// SPDX-License-Identifier: AGPL-3.0-only
//! Contact-patch state and force outputs, with the crate-wide sign contract.
//!
//! # Sign contract (ISO-W, the convention of modern `.tir` files)
//!
//! Axes are ISO 8855: x forward, y left, z up; all angles in rad, forces in N, moments in N·m.
//!
//! - **Longitudinal slip** `κ = −V_sx / |V_cx|`, dimensionless (NOT percent): κ > 0 when
//!   driving, κ < 0 when braking, κ = −1 is a locked wheel at forward roll. The sign flip
//!   relative to the sliding velocity is *embedded in κ's definition*.
//! - **Side-slip angle** `tan α = V_sy / |V_cx|`: α > 0 means the contact patch slides to +y
//!   (left), which produces `Fy < 0` on a normal tire. Consequently the cornering stiffness
//!   `K_yα = ∂Fy/∂α |₀` carries the sign of `PKY1` (negative in ISO-W `.tir` files) — never
//!   take an absolute value of `K_yα` or `B_y`; the negative B is what flips the sine.
//! - **Aligning moment**: `M_z = −t·F_y + M_zr` is restoring *because* `F_y` is negative for
//!   α > 0; a stray `abs()` silently breaks `M_z`.
//! - **Camber** γ is the inclination angle, rotation about the wheel x-axis (top of the tire
//!   leans to +y for γ > 0). Pinned against a γ = ±4° oracle sweep in the golden tests.
//! - **Reverse running** (`V_cx < 0`) enters only through `sgn(V_cx)` factors inside the MF
//!   equations (`α* `, `D_t`, `D_r`, `M_y`) — there is no global sign flip. `sgn(0)` maps to +1
//!   so standstill does not zero the lateral force (a true `signum` would create a 0/0 hazard).
//!
//! Normal load `F_z` is compressive-positive. `F_z ≤ 0` (airborne wheel) short-circuits every
//! model to exactly zero outputs.

use num_traits::Float;

/// Kinematic + operating state at the contact patch (ISO-W conventions, see module docs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SlipState<T> {
    /// Longitudinal slip ratio `κ = −V_sx/|V_cx|`, dimensionless; > 0 driving.
    pub kappa: T,
    /// Side-slip angle `α`, rad; `tan α = V_sy/|V_cx|`.
    pub alpha: T,
    /// Inclination (camber) angle `γ`, rad (ISO 8855 rotation about the wheel x-axis).
    pub gamma: T,
    /// Normal load `F_z`, N (compressive-positive; ≤ 0 yields all-zero forces).
    pub fz: T,
    /// Inflation pressure `p`, Pa (note: `.tyr` `thermal.p_cold` is kPa — convert at the seam).
    pub p: T,
    /// Contact-center forward velocity `V_cx`, m/s (sign meaningful; reverse running allowed).
    pub vx: T,
    /// Runtime longitudinal friction multiplier (M5 thermal grip-window hook; 1.0 in M2).
    pub mu_scale_x: T,
    /// Runtime lateral friction multiplier (M5 thermal grip-window hook; 1.0 in M2).
    pub mu_scale_y: T,
}

impl<T: Float> SlipState<T> {
    /// A pure-slip state at nominal camber/pressure scaling hooks: `μ` scales at 1.
    pub fn new(kappa: T, alpha: T, gamma: T, fz: T, p: T, vx: T) -> Self {
        Self {
            kappa,
            alpha,
            gamma,
            fz,
            p,
            vx,
            mu_scale_x: T::one(),
            mu_scale_y: T::one(),
        }
    }
}

/// Steady-state force/moment output at the contact patch (ISO 8855).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TireForces<T> {
    /// Longitudinal force `F_x`, N.
    pub fx: T,
    /// Lateral force `F_y`, N.
    pub fy: T,
    /// Aligning moment `M_z`, N·m (about +z, up).
    pub mz: T,
    /// Overturning moment `M_x`, N·m (about +x, forward).
    pub mx: T,
    /// Rolling-resistance moment `M_y`, N·m (about +y, left).
    pub my: T,
}

impl<T: Float> TireForces<T> {
    /// The all-zero output (airborne wheel).
    pub fn zero() -> Self {
        Self {
            fx: T::zero(),
            fy: T::zero(),
            mz: T::zero(),
            mx: T::zero(),
            my: T::zero(),
        }
    }
}

/// `sgn` with `sgn(0) = +1` (and `sgn(NaN) = +1`), per the module sign contract.
///
/// The MF equations use `sgn(V_cx)` and `sgn(κ_x)` as *branch selectors*; a true `signum`
/// returning 0 would zero whole force terms at exactly-zero inputs (0/0 hazards at standstill).
#[inline]
pub(crate) fn sgn_pos<T: Float>(x: T) -> T {
    if x < T::zero() {
        -T::one()
    } else {
        T::one()
    }
}
