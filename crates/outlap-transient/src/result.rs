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
    /// Engaged gear index (0-based), from the shift FSM.
    pub gear: Vec<T>,
    /// Drive-torque scale `∈ [0, 1]` applied this step (`< 1` during a gear shift's torque cut/ramp).
    pub torque_scale: Vec<T>,
    /// Torque-vectoring yaw moment actually applied `ΔM_z`, N·m (+CCW).
    pub yaw_moment_nm: Vec<T>,
    /// Recovered regen electrical power summed over the driven axles, W (≥ 0).
    pub regen_power_w: Vec<T>,
    /// Electrical traction power drawn from the pack, W (≥ 0) — the drive power the electric machines
    /// put down over their motoring efficiency. `regen_power_w − this` is the net pack charge power.
    pub traction_power_w: Vec<T>,
    /// Front-axle machine braking torque, N·m (≥ 0) — the share of the front axle's commanded brake
    /// torque the machine took. `front_axle_brake_torque − this` is what the front calipers supplied.
    pub regen_torque_front_nm: Vec<T>,
    /// Rear-axle machine braking torque, N·m (≥ 0) — the rear counterpart.
    pub regen_torque_rear_nm: Vec<T>,
    /// Pack state of charge, 0..1 (empty when no slow-state stack was attached).
    pub state_of_charge: Vec<T>,
    /// Pack temperature, °C (empty when no slow-state stack was attached).
    pub pack_temp_c: Vec<T>,
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
            gear: Vec::new(),
            torque_scale: Vec::new(),
            yaw_moment_nm: Vec::new(),
            regen_power_w: Vec::new(),
            traction_power_w: Vec::new(),
            regen_torque_front_nm: Vec::new(),
            regen_torque_rear_nm: Vec::new(),
            state_of_charge: Vec::new(),
            pack_temp_c: Vec::new(),
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
