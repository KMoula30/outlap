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

/// The interned dynamic-bus channels the **rule-based control layer** (PR6) exchanges (HANDOFF §8).
/// The orchestrator publishes the shift-FSM outputs (`gear`, `torque_scale`) and the battery regen
/// ceiling (`regen_limit_w`) at each step — decided on the step-boundary / slow clock and frozen
/// across the RK sweep, exactly like the relaxation and load-transfer coupling; the control blocks
/// then publish their diagnostics (`yaw_moment_cmd`, `regen_power_w`) that the recorder logs and the
/// slow-state stack integrates. Interned once at assembly (Decision #39; never in the loop).
#[derive(Clone, Copy, Debug)]
pub struct ActuationChannels {
    /// Engaged gear index (0-based, as an `f64`) the shift FSM currently holds — telemetry only.
    pub gear: ChannelId,
    /// Drive-torque scale `∈ [0, 1]` during a gear shift: `0` through the torque-cut window, ramping
    /// back to `1` over the clutch re-engagement. Multiplies the traction ceiling (the torque
    /// interruption of §8.2). Solver-published each step.
    pub torque_scale: ChannelId,
    /// Battery **charge-acceptance** ceiling `P_regen,max`, W — what the pack will take at its current
    /// charge *and temperature* (a cold pack cannot accept a fast charge), refreshed on the slow clock;
    /// `0` when no battery is present. Caps the regen brake blend. Solver-published.
    pub regen_limit_w: ChannelId,
    /// Commanded torque-vectoring yaw moment `ΔM_z`, N·m (+CCW) — the ellipse-feasible moment the
    /// allocator actually applied through the per-wheel force deltas. TV-published (telemetry).
    pub yaw_moment_cmd: ChannelId,
    /// Recovered electrical regen power `P_regen`, W (≥ 0) summed over the driven axles this step —
    /// powertrain-published; the slow-state stack Coulomb-counts it into the pack state of charge.
    pub regen_power_w: ChannelId,
    /// Electrical **traction** power drawn from the pack, W (≥ 0) this step — the drive power the
    /// electric machines put down, divided by their efficiency (motoring). Powertrain-published; the
    /// slow-state stack discharges it from the pack, so the state of charge moves both ways over a lap.
    pub traction_power_w: ChannelId,
    /// Front-axle **machine** braking torque, N·m (≥ 0) — the share of the front axle's commanded
    /// brake torque the front machine took. The calipers supplied the rest. Powertrain-published.
    pub regen_torque_front_nm: ChannelId,
    /// Rear-axle **machine** braking torque, N·m (≥ 0) — the rear counterpart. Powertrain-published.
    pub regen_torque_rear_nm: ChannelId,
    /// ERS MGU-K **deploy wheel force**, N (M6/PR4) — the additive drive force the rule-based energy
    /// manager schedules on a hybrid (`+` deploy under power, `−` super-clip back-drive on a
    /// full-throttle straight). Decided once per step at the boundary and frozen across the RK sweep
    /// (the `torque_scale` pattern); `0` when the car has no `ers:` block. Governor-published.
    pub ers_deploy_force_n: ChannelId,
    /// ERS realized **electrical deploy draw**, W (≥ 0) — what the pack pays to deploy the MGU-K this
    /// step (D-M6-10: the pack draw is the manager's electrical deploy ONLY, the ICE covers the rest
    /// of traction). For an ERS car the powertrain publishes this as `traction_power_w`.
    /// Governor-published.
    pub ers_deploy_power_w: ChannelId,
    /// ERS realized **electrical harvest**, W (≥ 0) — what the pack banks this step through the
    /// five-ceiling harvest chain (braking regen, part-throttle, or super-clip). For an ERS car the
    /// powertrain publishes this as `regen_power_w`. Governor-published.
    pub ers_harvest_power_w: ChannelId,
}

impl ActuationChannels {
    /// Intern the fixed set of actuation channels on `interner` (idempotent; call once at assembly).
    #[must_use]
    pub fn intern(interner: &mut ChannelInterner) -> Self {
        Self {
            gear: interner.intern("ctrl.gear"),
            torque_scale: interner.intern("ctrl.torque_scale"),
            regen_limit_w: interner.intern("ctrl.regen_limit_w"),
            yaw_moment_cmd: interner.intern("ctrl.yaw_moment_cmd"),
            regen_power_w: interner.intern("ctrl.regen_power_w"),
            traction_power_w: interner.intern("ctrl.traction_power_w"),
            regen_torque_front_nm: interner.intern("ctrl.regen_torque_front_nm"),
            regen_torque_rear_nm: interner.intern("ctrl.regen_torque_rear_nm"),
            ers_deploy_force_n: interner.intern("ctrl.ers_deploy_force_n"),
            ers_deploy_power_w: interner.intern("ctrl.ers_deploy_power_w"),
            ers_harvest_power_w: interner.intern("ctrl.ers_harvest_power_w"),
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

/// Immutable **T3 suspension + sprung-body** parameters the 14-DOF chassis reads (the
/// SymPy-verified `ChassisT3` block; `docs/derivations/t3_chassis_kane.py`).
///
/// The sprung body carries heave/pitch/roll about its CG at height `h_s`; per-corner suspension
/// (linear ride-rate spring + bump/rebound damper + C¹ progressive bumpstop + absolute ARB) sits
/// between each sprung corner and its unsprung mass, which rests on the tyre vertical spring. Mass /
/// inertia convention (D-M6, user-locked Option A): `ixx`/`iyy` are the **sprung-mass** roll/pitch
/// inertia about the sprung CG; the whole-car yaw inertia stays on [`ChassisParams::izz`]. Static
/// compressions `static_defl`/`tyre_static_defl` carry the corner loads so the RHS admits a static
/// equilibrium (the heave/pitch/roll + unsprung states are displacements from that reference).
/// Plain data — no IO, no clock — so the crate stays wasm-clean.
#[derive(Clone, Copy, Debug)]
pub struct SuspensionParams<T> {
    /// Sprung mass `m_s = chassis.mass_kg − Σ m_unsprung`, kg.
    pub sprung_mass: T,
    /// Sprung-mass roll inertia about the sprung CG `I_xx`, kg·m².
    pub ixx: T,
    /// Sprung-mass pitch inertia about the sprung CG `I_yy`, kg·m².
    pub iyy: T,
    /// Sprung CG height above ground `h_s`, m (elastic roll/pitch moment arm).
    pub h_s: T,
    /// Whole-car CG height `h_cg`, m (the longitudinal load-transfer arm, matches T2).
    pub h_cg: T,
    /// Roll-axis height under the CG `h_ra`, m (the geometric lateral-transfer arm).
    pub h_ra: T,
    /// Wheelbase `L`, m.
    pub wheelbase: T,
    /// Front / rear track widths, m.
    pub track_f: T,
    /// Rear track width, m.
    pub track_r: T,
    /// Front anti-dive fraction (geometric share of the front longitudinal transfer).
    pub anti_dive: T,
    /// Rear anti-squat fraction (geometric share of the rear longitudinal transfer).
    pub anti_squat: T,
    /// Absolute front anti-roll-bar roll stiffness, N·m/rad.
    pub arb_f: T,
    /// Absolute rear anti-roll-bar roll stiffness, N·m/rad.
    pub arb_r: T,
    /// C¹ bumpstop smoothing scale `s`, m (the quadratic-to-linear knee width).
    pub bumpstop_smooth: T,
    /// Per-corner ride (wheel) rate, N/m.
    pub k_ride: [T; WHEELS],
    /// Per-corner static compression carrying the sprung corner load, m.
    pub static_defl: [T; WHEELS],
    /// Per-corner bump (compression) damping, N·s/m.
    pub damp_bump: [T; WHEELS],
    /// Per-corner rebound (extension) damping, N·s/m.
    pub damp_rebound: [T; WHEELS],
    /// Per-corner bumpstop rate, N/m.
    pub bumpstop_rate: [T; WHEELS],
    /// Per-corner bumpstop gap (compression before engagement), m.
    pub bumpstop_gap: [T; WHEELS],
    /// Per-corner tyre vertical stiffness `k_tz`, N/m.
    pub k_tyre: [T; WHEELS],
    /// Per-corner tyre vertical damping `c_tz`, N·s/m.
    pub c_tyre: [T; WHEELS],
    /// Per-corner static tyre compression carrying the corner load, m.
    pub tyre_static_defl: [T; WHEELS],
    /// Per-corner unsprung mass, kg.
    pub unsprung_mass: [T; WHEELS],
}
