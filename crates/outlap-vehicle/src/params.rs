// SPDX-License-Identifier: AGPL-3.0-only
//! Immutable per-vehicle parameters the T2 blocks read, plus the interned road-geometry channels.
//!
//! These are built once by the assembly pipeline (never in the loop) from the one canonical vehicle
//! description (HANDOFF §6.1) and handed to the blocks by value. Everything here is plain data — no
//! IO, no clock — so the crate stays wasm-clean.

use num_traits::Float;

use outlap_core::bus::{ChannelId, ChannelInterner, WHEELS};

/// Standard gravity, m/s² (SI; used to build the in-plane gravity force from grade/banking).
pub const G: f64 = 9.806_65;

/// The interned dynamic-bus channels the road publishes each step (Decision #39). The chassis reads
/// `kappa`/`grade`/`banking` for the curvilinear kinematics and the in-plane gravity projection; the
/// driver reads the current-station target-line channels (`n_ref`/`kappa_ref`/`v_ref`) plus the
/// **preview** channels (`*_preview`) the orchestrator samples at the look-ahead station
/// `s + v_x·t_preview` for the MacAdam-style preview steer and the speed feed-forward (PR5,
/// §7.7). Interned once at assembly.
#[derive(Clone, Copy, Debug)]
pub struct RoadChannels {
    /// Plan-view curvature `κ_h(s)` of the reference line at the current station (1/m, +left).
    pub kappa: ChannelId,
    /// Road grade `θ(s)` at the current station (rad, +uphill).
    pub grade: ChannelId,
    /// Road banking `φ(s)` at the current station (rad, + raises road-left edge).
    pub banking: ChannelId,
    /// Vertical curvature `κ_v(s)` (1/m, crest < 0, dip > 0) — normal-load modulation in the Fz block.
    pub kappa_v: ChannelId,
    /// Target lateral offset `n_ref(s)` (m, +left) at the current station (recorded; driver uses the
    /// preview offset for tracking).
    pub n_ref: ChannelId,
    /// Target-line curvature `κ_ref(s)` (1/m) at the current station.
    pub kappa_ref: ChannelId,
    /// Target speed `v_ref(s)` (m/s) at the current station.
    pub v_ref: ChannelId,
    /// Previewed target offset `n_ref(s + L_p)` (m, +left) at the look-ahead station.
    pub n_ref_preview: ChannelId,
    /// Previewed target curvature `κ_ref(s + L_p)` (1/m) — curvature feed-forward arm.
    pub kappa_ref_preview: ChannelId,
    /// Previewed target speed `v_ref(s + L_p)` (m/s) — speed feed-forward.
    pub v_ref_preview: ChannelId,
}

impl RoadChannels {
    /// Intern the fixed set of road channels on `interner` (idempotent; call once at assembly).
    #[must_use]
    pub fn intern(interner: &mut ChannelInterner) -> Self {
        Self {
            kappa: interner.intern("road.kappa"),
            grade: interner.intern("road.grade"),
            banking: interner.intern("road.banking"),
            kappa_v: interner.intern("road.kappa_v"),
            n_ref: interner.intern("road.n_ref"),
            kappa_ref: interner.intern("road.kappa_ref"),
            v_ref: interner.intern("road.v_ref"),
            n_ref_preview: interner.intern("road.n_ref_preview"),
            kappa_ref_preview: interner.intern("road.kappa_ref_preview"),
            v_ref_preview: interner.intern("road.v_ref_preview"),
        }
    }
}

/// Per-wheel body-frame geometry and inertia (ISO 8855: x forward, y left; `[FL, FR, RL, RR]`).
#[derive(Clone, Copy, Debug)]
pub struct WheelGeometry<T> {
    /// Longitudinal position relative to the CG (`+` forward), m.
    pub x: [T; WHEELS],
    /// Lateral position relative to the CG (`+` left), m.
    pub y: [T; WHEELS],
    /// Whether each wheel is on the front (steered) axle.
    pub front: [bool; WHEELS],
    /// Effective rolling radius per wheel, m.
    pub radius: [T; WHEELS],
    /// Spin inertia per wheel (rim + tire + hub-side driveline), kg·m².
    pub inertia: [T; WHEELS],
}

/// Immutable parameters of the rigid-body chassis + wheel-spin RHS (the SymPy-verified block).
///
/// Symbols follow the derivation notebook (`docs/derivations/t2_chassis_kane.ipynb`): mass `m`, yaw
/// inertia `Izz`, per-wheel geometry `(x_i, y_i)`, rolling radii `R_i`, wheel inertias `I_{w,i}`.
#[derive(Clone, Copy, Debug)]
pub struct ChassisParams<T> {
    /// Total mass `m`, kg.
    pub mass: T,
    /// Yaw inertia `I_zz`, kg·m².
    pub izz: T,
    /// Per-wheel geometry and spin inertia.
    pub wheels: WheelGeometry<T>,
    /// Gravitational acceleration `g`, m/s².
    pub gravity: T,
    /// Smoothing speed for the brake-torque sign near `ω = 0`, rad/s (avoids RHS chatter at rest).
    pub omega_eps: T,
}

impl<T: Float> ChassisParams<T> {
    /// Convenience constructor from `f64` assembly data, casting into `T`.
    ///
    /// # Panics
    /// Panics if any value is not representable in `T` (never for the finite physical values here).
    #[must_use]
    pub fn from_f64(
        mass: f64,
        izz: f64,
        x: [f64; WHEELS],
        y: [f64; WHEELS],
        front: [bool; WHEELS],
        radius: [f64; WHEELS],
        inertia: [f64; WHEELS],
    ) -> Self {
        let c = |v: f64| T::from(v).expect("finite parameter representable in T");
        let map = |a: [f64; WHEELS]| a.map(c);
        Self {
            mass: c(mass),
            izz: c(izz),
            wheels: WheelGeometry {
                x: map(x),
                y: map(y),
                front,
                radius: map(radius),
                inertia: map(inertia),
            },
            gravity: c(G),
            omega_eps: c(1.0),
        }
    }
}
