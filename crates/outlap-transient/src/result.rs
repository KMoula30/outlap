// SPDX-License-Identifier: AGPL-3.0-only
//! The **time-indexed transient result** (Decision #13). One `TransientLap` carries the fast-state
//! traces a QSS lap cannot show — sideslip, yaw rate, per-wheel slip/`F_z`, control inputs — sampled
//! on the fixed time grid, plus the world trajectory reconstructed from the integrated `(s, n)`.
//!
//! Kept deliberately Rust-plain (parallel `Vec`s); the Python boundary (PR7) maps it to an xarray
//! `Dataset` with `(time, wheel)` dims.

use num_traits::Float;

use outlap_core::bus::WHEELS;

/// A per-wheel sample `[FL, FR, RL, RR]`.
pub type Wheels<T> = [T; WHEELS];

/// The recorded trajectory + channels of one transient lap. Every `Vec` has one entry per recorded
/// step and shares the `t` time base.
#[derive(Clone, Debug)]
pub struct TransientLap<T> {
    /// Time since the lap start, s.
    pub t: Vec<T>,
    /// Arc length along the reference line `s`, m.
    pub s: Vec<T>,
    /// Lateral offset from the reference line `n`, m (+left).
    pub n: Vec<T>,
    /// Heading relative to the road tangent `ψ_rel`, rad.
    pub psi_rel: Vec<T>,
    /// Body-frame longitudinal velocity `v_x`, m/s.
    pub vx: Vec<T>,
    /// Body-frame lateral velocity `v_y`, m/s (+left).
    pub vy: Vec<T>,
    /// Yaw rate `r`, rad/s (+CCW).
    pub yaw_rate: Vec<T>,
    /// Body-frame longitudinal acceleration `a_x`, m/s².
    pub ax: Vec<T>,
    /// Body-frame lateral acceleration `a_y`, m/s² (+left).
    pub ay: Vec<T>,
    /// Per-wheel angular speed `ω`, rad/s.
    pub omega: Vec<Wheels<T>>,
    /// Front road-wheel steer `δ`, rad.
    pub steer: Vec<T>,
    /// Throttle demand, 0..1.
    pub throttle: Vec<T>,
    /// Brake demand, 0..1.
    pub brake: Vec<T>,
    /// Per-wheel normal load `F_z`, N.
    pub fz: Vec<Wheels<T>>,
    /// Per-wheel lagged longitudinal slip `κ`.
    pub slip_kappa: Vec<Wheels<T>>,
    /// Per-wheel lagged slip angle `α`, rad.
    pub slip_alpha: Vec<Wheels<T>>,
    /// Per-wheel wheel-frame longitudinal force `F_x`, N.
    pub fx: Vec<Wheels<T>>,
    /// Per-wheel wheel-frame lateral force `F_y`, N.
    pub fy: Vec<Wheels<T>>,
    /// World trajectory `x`, m.
    pub x: Vec<T>,
    /// World trajectory `y`, m.
    pub y: Vec<T>,
    /// World trajectory `z`, m.
    pub z: Vec<T>,
    /// Total lap time (last `t` on a completed lap), s.
    pub lap_time_s: T,
}

impl<T: Float> Default for TransientLap<T> {
    fn default() -> Self {
        Self {
            t: Vec::new(),
            s: Vec::new(),
            n: Vec::new(),
            psi_rel: Vec::new(),
            vx: Vec::new(),
            vy: Vec::new(),
            yaw_rate: Vec::new(),
            ax: Vec::new(),
            ay: Vec::new(),
            omega: Vec::new(),
            steer: Vec::new(),
            throttle: Vec::new(),
            brake: Vec::new(),
            fz: Vec::new(),
            slip_kappa: Vec::new(),
            slip_alpha: Vec::new(),
            fx: Vec::new(),
            fy: Vec::new(),
            x: Vec::new(),
            y: Vec::new(),
            z: Vec::new(),
            lap_time_s: T::zero(),
        }
    }
}

impl<T: Copy> TransientLap<T> {
    /// The number of recorded steps.
    #[must_use]
    pub fn len(&self) -> usize {
        self.t.len()
    }

    /// Whether nothing was recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.t.is_empty()
    }
}
