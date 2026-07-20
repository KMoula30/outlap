// SPDX-License-Identifier: AGPL-3.0-only
//! The **T2 chassis block**: the curvilinear 3-D-road-frame right-hand side for the 7-DOF state
//! `[s, n, ψ_rel, v_x, v_y, r, ω₁..₄]` (HANDOFF §6.1; Perantoni & Limebeer 2014; Rowold 2023).
//!
//! # Model (ISO 8855: x forward, y left, z up; SI)
//!
//! The chassis is a planar rigid body (mass `m`, yaw inertia `I_zz`) tracked in the **curvilinear
//! road frame**: arc length `s` along the reference line, lateral offset `n` (+left), heading
//! `ψ_rel` relative to the road tangent. Four wheels spin with inertia `I_{w,i}` and effective radius
//! `R_i`. Grade `θ` and banking `φ` rotate gravity into the road-surface plane; the full 3-D
//! frame-transport terms (grade/bank rate along `s`) are a T3 refinement and are documented as out of
//! M4 scope (this block degenerates to the exact planar EOM when `θ = φ = 0`).
//!
//! Dynamics (body frame; `a_x = v̇_x − r·v_y`, `a_y = v̇_y + r·v_x` are the body-frame CG accels):
//! ```text
//! m (v̇_x − r v_y) = ΣF_x        v̇_x = ΣF_x/m + r v_y
//! m (v̇_y + r v_x) = ΣF_y   ⟺    v̇_y = ΣF_y/m − r v_x
//! I_zz ṙ          = ΣM_z        ṙ   = ΣM_z / I_zz
//! I_{w,i} ω̇_i     = τ_i − R_i F_{x,i}^w    (drive − brake·sgn(ω) − rolling reaction)
//! ```
//! Curvilinear kinematics (`κ = κ_h(s)` the reference-line plan-view curvature):
//! ```text
//! ṡ      = (v_x cos ψ_rel − v_y sin ψ_rel) / (1 − n κ)
//! ṅ      =  v_x sin ψ_rel + v_y cos ψ_rel
//! ψ̇_rel  =  r − κ ṡ
//! ```
//! `ΣF`/`ΣM` sum the four **wheel-frame** tyre forces `(F_x^w, F_y^w, M_z^w)` rotated into the body
//! frame by the per-wheel steer `δ_i` (front axle steers, rear does not), minus aero drag along `+x`,
//! plus the in-plane gravity projection and the yaw-moment demand `ΔM_z` from torque vectoring.
//!
//! The block is pure and generic over `f32`/`f64`; it allocates nothing and reads every input off the
//! [`Bus`]. Per-wheel tyre forces, aero, Fz (load transfer), steer and wheel torques are produced by
//! upstream `sense`/`control`/`actuate` blocks — the chassis is the sole `integrate`-phase block that
//! writes the fast-state derivative.

use num_traits::Float;

use outlap_core::block::{Block, Phase, Ports};
use outlap_core::bus::{Bus, ChannelId, ChannelInterner, CoreSignal, WheelSignal, WHEELS};
use outlap_core::state::{ChassisState, DerivView, SlowStateView, StateView};

use crate::params::{ChassisParams, RoadChannels, SuspensionParams};

/// Floor on `|1 − nκ|` in the curvilinear progress term, guarding the frame singularity at the
/// curvature centre `n = 1/κ` (the offset is edge-clamped upstream; this is the RHS backstop).
const DENOM_FLOOR: f64 = 0.1;

/// The T2 chassis right-hand side (see the module docs for the equations).
#[derive(Clone, Copy, Debug)]
pub struct Chassis<T> {
    /// Immutable rigid-body + wheel parameters.
    pub params: ChassisParams<T>,
    /// Interned road-geometry bus channels the chassis reads.
    pub road: RoadChannels,
}

impl<T: Float> Chassis<T> {
    /// Build a chassis block from its parameters and the interned road channels.
    #[must_use]
    pub fn new(params: ChassisParams<T>, road: RoadChannels) -> Self {
        Self { params, road }
    }

    /// The per-wheel road-wheel steer `δ_i`: the front axle takes the commanded steer, the rear 0.
    #[inline]
    fn wheel_steer(&self, steer: T, wheel: usize) -> T {
        if self.params.wheels.front[wheel] {
            steer
        } else {
            T::zero()
        }
    }
}

impl<T: Float> Block<T> for Chassis<T> {
    fn phase(&self) -> Phase {
        Phase::Integrate
    }

    fn ports(&self) -> Ports {
        // Reads: steer, aero drag, yaw-moment demand, per-wheel tyre forces + wheel torques, and the
        // road channels. Writes: nothing on the bus (it writes the state derivative, not a channel).
        let mut reads = vec![
            CoreSignal::Steer as usize,
            CoreSignal::AeroDrag as usize,
            CoreSignal::YawMomentDemand as usize,
            self.road.kappa.index(),
            self.road.grade.index(),
            self.road.banking.index(),
        ];
        // The per-wheel force + torque channels live in the fixed core region; declare them as raw
        // flat indices so the assembler orders the tyre/powertrain writers before this reader.
        let base = CoreSignal::COUNT as usize;
        for sig in [
            WheelSignal::TireFx,
            WheelSignal::TireFy,
            WheelSignal::TireMz,
            WheelSignal::WheelDriveTorque,
            WheelSignal::WheelBrakeTorque,
        ] {
            for w in 0..WHEELS {
                reads.push(base + (sig as usize) * WHEELS + w);
            }
        }
        Ports::new(reads, Vec::new())
    }

    fn derivatives(&self, x: &StateView<T>, bus: &mut Bus<T>, dx: &mut DerivView<T>, lane: usize) {
        let p = &self.params;
        let vx = x.chassis(ChassisState::Vx);
        let vy = x.chassis(ChassisState::Vy);
        let r = x.chassis(ChassisState::YawRate);
        let n = x.chassis(ChassisState::N);
        let psi = x.chassis(ChassisState::PsiRel);

        let steer = bus.get(CoreSignal::Steer, lane);

        // --- Sum the four wheel-frame tyre forces into the body frame + yaw moment about the CG. ---
        let mut sum_fx = T::zero();
        let mut sum_fy = T::zero();
        let mut sum_mz = T::zero();
        for i in 0..WHEELS {
            let d = self.wheel_steer(steer, i);
            let (sn, cs) = d.sin_cos();
            let fxw = bus.get_wheel(WheelSignal::TireFx, i, lane);
            let fyw = bus.get_wheel(WheelSignal::TireFy, i, lane);
            let mzw = bus.get_wheel(WheelSignal::TireMz, i, lane);
            // Rotate wheel-frame → body-frame.
            let fxb = fxw * cs - fyw * sn;
            let fyb = fxw * sn + fyw * cs;
            sum_fx = sum_fx + fxb;
            sum_fy = sum_fy + fyb;
            sum_mz = sum_mz + p.wheels.x[i] * fyb - p.wheels.y[i] * fxb + mzw;
        }

        // Aero drag opposes +x.
        sum_fx = sum_fx - bus.get(CoreSignal::AeroDrag, lane);

        // In-plane gravity from grade/banking, rotated from the road-surface frame into the body
        // frame by the heading ψ_rel. Road-plane components: along tangent `g_t = −g sinθ` (uphill
        // decelerates), along road-left `g_w = −g sinφ` (banking>0 raises the left edge, so the
        // in-plane pull is toward the low/right side). Both zero on flat ground ⇒ planar EOM.
        let grade = bus.get_channel(self.road.grade, lane);
        let bank = bus.get_channel(self.road.banking, lane);
        let mg = p.mass * p.gravity;
        let g_t = -mg * grade.sin();
        let g_w = -mg * bank.sin();
        let (spsi, cpsi) = psi.sin_cos();
        sum_fx = sum_fx + g_t * cpsi + g_w * spsi;
        sum_fy = sum_fy - g_t * spsi + g_w * cpsi;

        // External yaw-moment demand (torque vectoring; zero until PR6 writes it).
        sum_mz = sum_mz + bus.get(CoreSignal::YawMomentDemand, lane);

        // --- Body-frame chassis accelerations. ---
        let v_x_dot = sum_fx / p.mass + r * vy;
        let v_y_dot = sum_fy / p.mass - r * vx;
        let r_dot = sum_mz / p.izz;

        dx.set_chassis(ChassisState::Vx, v_x_dot);
        dx.set_chassis(ChassisState::Vy, v_y_dot);
        dx.set_chassis(ChassisState::YawRate, r_dot);

        // --- Wheel-spin dynamics. ---
        for i in 0..WHEELS {
            let omega = x.chassis(chassis_omega(i));
            let tau_drive = bus.get_wheel(WheelSignal::WheelDriveTorque, i, lane);
            let tau_brake = bus.get_wheel(WheelSignal::WheelBrakeTorque, i, lane);
            let fxw = bus.get_wheel(WheelSignal::TireFx, i, lane);
            // Brake torque opposes spin; smooth the sign near ω = 0 so the RHS stays continuous.
            let brake_signed = tau_brake * (omega / (omega.abs() + p.omega_eps));
            let omega_dot =
                (tau_drive - brake_signed - fxw * p.wheels.radius[i]) / p.wheels.inertia[i];
            dx.set_chassis(chassis_omega(i), omega_dot);
        }

        // --- Curvilinear kinematics (progress along the reference line). ---
        let kappa = bus.get_channel(self.road.kappa, lane);
        let one = T::one();
        // Floor |1 − nκ| away from zero so the frame stays non-singular if `n` transiently reaches
        // the curvature centre `n = 1/κ` (the offset is edge-clamped by the orchestrator; this guards
        // the RHS itself). Sign preserved.
        let raw_denom = one - n * kappa;
        let floor = T::from(DENOM_FLOOR).unwrap_or_else(T::one);
        let denom = if raw_denom.abs() >= floor {
            raw_denom
        } else if raw_denom < T::zero() {
            -floor
        } else {
            floor
        };
        let s_dot = (vx * cpsi - vy * spsi) / denom;
        let n_dot = vx * spsi + vy * cpsi;
        let psi_dot = r - kappa * s_dot;

        dx.set_chassis(ChassisState::S, s_dot);
        dx.set_chassis(ChassisState::N, n_dot);
        dx.set_chassis(ChassisState::PsiRel, psi_dot);
    }

    fn slow_derivatives(
        &self,
        _bus: &Bus<T>,
        _dslow: &mut outlap_core::state::SlowDerivView<T>,
        _lane: usize,
    ) {
        // The chassis owns no slow states.
    }

    fn equilibrium(&self, _bus: &mut Bus<T>, _slow: &SlowStateView<T>, _lane: usize) {
        // T0/T1 equilibrium is the QSS trim solver's job (outlap-qss); the chassis is T2/T3-only.
    }
}

/// The [`ChassisState`] wheel-speed slot for wheel `i` (`0..WHEELS` → FL, FR, RL, RR).
#[inline]
fn chassis_omega(wheel: usize) -> ChassisState {
    match wheel {
        0 => ChassisState::OmegaFl,
        1 => ChassisState::OmegaFr,
        2 => ChassisState::OmegaRl,
        _ => ChassisState::OmegaRr,
    }
}

/// The [`ChassisState`] unsprung-position slot for wheel `i`.
#[inline]
fn chassis_zu(wheel: usize) -> ChassisState {
    match wheel {
        0 => ChassisState::ZuFl,
        1 => ChassisState::ZuFr,
        2 => ChassisState::ZuRl,
        _ => ChassisState::ZuRr,
    }
}

/// The [`ChassisState`] unsprung-velocity slot for wheel `i`.
#[inline]
fn chassis_zu_rate(wheel: usize) -> ChassisState {
    match wheel {
        0 => ChassisState::ZuRateFl,
        1 => ChassisState::ZuRateFr,
        2 => ChassisState::ZuRateRl,
        _ => ChassisState::ZuRateRr,
    }
}

/// Left/right sign per corner (`+1` left, `−1` right; FL, FR, RL, RR) for the geometric lateral
/// load-transfer split — matches `SIDE` in `docs/derivations/t3_chassis_kane.py`.
const SIDE: [f64; WHEELS] = [1.0, -1.0, 1.0, -1.0];

/// C¹ one-sided ramp used by the bumpstop: `0` for `p<0`, `p²/(2s)` for `0≤p<s`, `p−s/2` for
/// `p≥s`. Value and slope are continuous at both knees, so the RK path never sees a discontinuous
/// bumpstop force *or* stiffness at engagement (matches `smooth_ramp` in the derivation).
#[inline]
fn smooth_ramp<T: Float>(p: T, s: T) -> T {
    if p < T::zero() {
        T::zero()
    } else if p < s {
        p * p / (s + s)
    } else {
        p - s / (T::one() + T::one())
    }
}

/// The interned per-corner **road vertical excitation** channels the T3 chassis reads: the road
/// surface height and its rate at each contact patch (`z_road,i`, `ż_road,i`, m and m/s, +up). On a
/// flat/smooth track these are zero; the 3-D track orchestrator publishes them in the tier wiring
/// (PR7). Interned once at assembly (Decision #39; never in the loop).
#[derive(Clone, Copy, Debug)]
pub struct T3RoadVertical {
    /// Per-corner road surface height `z_road,i`, m (+up).
    pub height: [ChannelId; WHEELS],
    /// Per-corner road surface vertical rate `ż_road,i`, m/s (+up).
    pub rate: [ChannelId; WHEELS],
}

impl T3RoadVertical {
    /// Intern the fixed set of per-corner road-vertical channels (idempotent; call once at assembly).
    #[must_use]
    pub fn intern(interner: &mut ChannelInterner) -> Self {
        let names_h = ["road.zr_fl", "road.zr_fr", "road.zr_rl", "road.zr_rr"];
        let names_r = ["road.zrd_fl", "road.zrd_fr", "road.zrd_rl", "road.zrd_rr"];
        Self {
            height: names_h.map(|n| interner.intern(n)),
            rate: names_r.map(|n| interner.intern(n)),
        }
    }
}

/// The **T3 14-DOF chassis block** (see `docs/theory/t3-chassis.md` and the `SymPy` derivation
/// `docs/derivations/t3_chassis_kane.py`). It writes all 24 [`ChassisState`] derivatives: the T2
/// planar handling + wheel spins + curvilinear kinematics (byte-identical structure to the T2
/// [`Chassis`], reused here) plus the sprung heave/pitch/roll ride block, the four unsprung
/// verticals, and the two refinement terms (gyroscopic wheel spin×yaw coupling; the 3-D
/// frame-transport vertical-curvature term `κ_v·v_x²`). Per-wheel `F_z` is produced internally from
/// the tyre vertical spring (`κ_z·(δ_static + z_road − z_u)`); the horizontal tyre forces are read
/// off the bus as in T2. The block is pure, generic over `f32`/`f64`, and allocation-free.
#[derive(Clone, Copy, Debug)]
pub struct ChassisT3<T> {
    /// Rigid-body + wheel parameters (mass, whole-car yaw inertia, wheel geometry/inertia, gravity).
    pub params: ChassisParams<T>,
    /// Sprung-body + per-corner suspension parameters.
    pub susp: SuspensionParams<T>,
    /// Interned road-geometry bus channels (`κ`, grade, banking, `κ_v`).
    pub road: RoadChannels,
    /// Interned per-corner road vertical excitation channels.
    pub road_v: T3RoadVertical,
}

impl<T: Float> ChassisT3<T> {
    /// Build a T3 chassis block from its parameters and the interned road channels.
    #[must_use]
    pub fn new(
        params: ChassisParams<T>,
        susp: SuspensionParams<T>,
        road: RoadChannels,
        road_v: T3RoadVertical,
    ) -> Self {
        Self {
            params,
            susp,
            road,
            road_v,
        }
    }

    #[inline]
    fn wheel_steer(&self, steer: T, wheel: usize) -> T {
        if self.params.wheels.front[wheel] {
            steer
        } else {
            T::zero()
        }
    }
}

impl<T: Float> Block<T> for ChassisT3<T> {
    fn phase(&self) -> Phase {
        Phase::Integrate
    }

    fn ports(&self) -> Ports {
        // Reads: the T2 chassis reads + κ_v + the per-corner road-vertical channels. Writes nothing
        // on the bus (it writes the 24-slot state derivative).
        let mut reads = vec![
            CoreSignal::Steer as usize,
            CoreSignal::AeroDrag as usize,
            CoreSignal::AeroFzFront as usize,
            CoreSignal::AeroFzRear as usize,
            CoreSignal::YawMomentDemand as usize,
            self.road.kappa.index(),
            self.road.grade.index(),
            self.road.banking.index(),
            self.road.kappa_v.index(),
        ];
        for w in 0..WHEELS {
            reads.push(self.road_v.height[w].index());
            reads.push(self.road_v.rate[w].index());
        }
        let base = CoreSignal::COUNT as usize;
        for sig in [
            WheelSignal::TireFx,
            WheelSignal::TireFy,
            WheelSignal::TireMz,
            WheelSignal::WheelDriveTorque,
            WheelSignal::WheelBrakeTorque,
        ] {
            for w in 0..WHEELS {
                reads.push(base + (sig as usize) * WHEELS + w);
            }
        }
        Ports::new(reads, Vec::new())
    }

    #[allow(clippy::too_many_lines)]
    fn derivatives(&self, x: &StateView<T>, bus: &mut Bus<T>, dx: &mut DerivView<T>, lane: usize) {
        let p = &self.params;
        let s = &self.susp;
        let two = T::one() + T::one();
        let half = T::one() / two;

        // --- states ---
        let vx = x.chassis(ChassisState::Vx);
        let vy = x.chassis(ChassisState::Vy);
        let r = x.chassis(ChassisState::YawRate);
        let n = x.chassis(ChassisState::N);
        let psi = x.chassis(ChassisState::PsiRel);
        let z = x.chassis(ChassisState::Heave);
        let th = x.chassis(ChassisState::Pitch);
        let ph = x.chassis(ChassisState::Roll);
        let zd = x.chassis(ChassisState::HeaveRate);
        let thd = x.chassis(ChassisState::PitchRate);
        let phd = x.chassis(ChassisState::RollRate);

        let steer = bus.get(CoreSignal::Steer, lane);
        let (spsi, cpsi) = psi.sin_cos();

        // --- handling: sum wheel-frame tyre forces into the body frame (as T2) ---
        let mut sum_fx = T::zero();
        let mut sum_fy = T::zero();
        let mut sum_mz = T::zero();
        for i in 0..WHEELS {
            let d = self.wheel_steer(steer, i);
            let (sn, cs) = d.sin_cos();
            let fxw = bus.get_wheel(WheelSignal::TireFx, i, lane);
            let fyw = bus.get_wheel(WheelSignal::TireFy, i, lane);
            let mzw = bus.get_wheel(WheelSignal::TireMz, i, lane);
            let fxb = fxw * cs - fyw * sn;
            let fyb = fxw * sn + fyw * cs;
            sum_fx = sum_fx + fxb;
            sum_fy = sum_fy + fyb;
            sum_mz = sum_mz + p.wheels.x[i] * fyb - p.wheels.y[i] * fxb + mzw;
        }
        sum_fx = sum_fx - bus.get(CoreSignal::AeroDrag, lane);

        let grade = bus.get_channel(self.road.grade, lane);
        let bank = bus.get_channel(self.road.banking, lane);
        let mg = p.mass * p.gravity;
        let g_t = -mg * grade.sin();
        let g_w = -mg * bank.sin();
        sum_fx = sum_fx + g_t * cpsi + g_w * spsi;
        sum_fy = sum_fy - g_t * spsi + g_w * cpsi;
        sum_mz = sum_mz + bus.get(CoreSignal::YawMomentDemand, lane);

        // --- gyroscopic reaction from the spinning wheels (closed forms from the derivation) ---
        // h_i = Iw·ω·(−wheel-lateral); Ω = φ̇ x̂ − θ̇ ŷ + r ẑ; M_gyro = −Ω × Σh_i.
        let mut m_gyro_x = T::zero(); // roll
        let mut m_gyro_y = T::zero(); // about +y
        let mut m_gyro_z = T::zero(); // yaw
        for i in 0..WHEELS {
            let d = self.wheel_steer(steer, i);
            let (sn, cs) = d.sin_cos();
            let l = p.wheels.inertia[i] * x.chassis(chassis_omega(i)); // Iw·ω
            m_gyro_x = m_gyro_x - r * (l * cs);
            m_gyro_y = m_gyro_y - r * (l * sn);
            m_gyro_z = m_gyro_z + l * (phd * cs - thd * sn);
        }
        sum_mz = sum_mz + m_gyro_z;

        let vx_dot = sum_fx / p.mass + r * vy;
        let vy_dot = sum_fy / p.mass - r * vx;
        let r_dot = sum_mz / p.izz;
        let ax = sum_fx / p.mass; // = vx_dot − r·vy
        let ay = sum_fy / p.mass; // = vy_dot + r·vx

        dx.set_chassis(ChassisState::Vx, vx_dot);
        dx.set_chassis(ChassisState::Vy, vy_dot);
        dx.set_chassis(ChassisState::YawRate, r_dot);

        // --- wheel spins (as T2; brake opposes spin, smoothed sign) ---
        for i in 0..WHEELS {
            let omega = x.chassis(chassis_omega(i));
            let tau_drive = bus.get_wheel(WheelSignal::WheelDriveTorque, i, lane);
            let tau_brake = bus.get_wheel(WheelSignal::WheelBrakeTorque, i, lane);
            let fxw = bus.get_wheel(WheelSignal::TireFx, i, lane);
            let brake_signed = tau_brake * (omega / (omega.abs() + p.omega_eps));
            let omega_dot =
                (tau_drive - brake_signed - fxw * p.wheels.radius[i]) / p.wheels.inertia[i];
            dx.set_chassis(chassis_omega(i), omega_dot);
        }

        // --- curvilinear kinematics (as T2, floored denom) ---
        let kappa = bus.get_channel(self.road.kappa, lane);
        let raw_denom = T::one() - n * kappa;
        let floor = T::from(DENOM_FLOOR).unwrap_or_else(T::one);
        let denom = if raw_denom.abs() >= floor {
            raw_denom
        } else if raw_denom < T::zero() {
            -floor
        } else {
            floor
        };
        let s_dot = (vx * cpsi - vy * spsi) / denom;
        let n_dot = vx * spsi + vy * cpsi;
        let psi_dot = r - kappa * s_dot;
        dx.set_chassis(ChassisState::S, s_dot);
        dx.set_chassis(ChassisState::N, n_dot);
        dx.set_chassis(ChassisState::PsiRel, psi_dot);

        // --- ride block ---
        // normal-direction gravity + vertical-curvature frame transport (crest κ_v<0 lightens).
        let kappa_v = bus.get_channel(self.road.kappa_v, lane);
        let g_n = p.gravity * grade.cos() * bank.cos() + kappa_v * vx * vx;

        // per-corner suspension compression + rate.
        let mut delta = [T::zero(); WHEELS];
        let mut delta_d = [T::zero(); WHEELS];
        let mut zc = [T::zero(); WHEELS];
        for i in 0..WHEELS {
            let zu = x.chassis(chassis_zu(i));
            let zud = x.chassis(chassis_zu_rate(i));
            zc[i] = z - p.wheels.x[i] * th + p.wheels.y[i] * ph;
            let zcd = zd - p.wheels.x[i] * thd + p.wheels.y[i] * phd;
            delta[i] = s.static_defl[i] + zu - zc[i];
            delta_d[i] = zud - zcd;
        }

        // ARB roll couple per axle (force up on the left corner opposes a right roll).
        let arb_f = s.arb_f
            * ((zc[0] - x.chassis(chassis_zu(0))) - (zc[1] - x.chassis(chassis_zu(1))))
            / (s.track_f * s.track_f);
        let arb_r = s.arb_r
            * ((zc[2] - x.chassis(chassis_zu(2))) - (zc[3] - x.chassis(chassis_zu(3))))
            / (s.track_r * s.track_r);
        let f_arb = [-arb_f, arb_f, -arb_r, arb_r];

        // per-corner suspension force (up on sprung / down on unsprung) and tyre vertical force.
        let mut f_susp = [T::zero(); WHEELS];
        let mut f_tyre = [T::zero(); WHEELS];
        for i in 0..WHEELS {
            let damper = if delta_d[i] >= T::zero() {
                s.damp_bump[i] * delta_d[i]
            } else {
                s.damp_rebound[i] * delta_d[i]
            };
            let bump =
                s.bumpstop_rate[i] * smooth_ramp(delta[i] - s.bumpstop_gap[i], s.bumpstop_smooth);
            f_susp[i] = s.k_ride[i] * delta[i] + damper + bump + f_arb[i];

            let zu = x.chassis(chassis_zu(i));
            let zud = x.chassis(chassis_zu_rate(i));
            let zr = bus.get_channel(self.road_v.height[i], lane);
            let zrd = bus.get_channel(self.road_v.rate[i], lane);
            f_tyre[i] = s.k_tyre[i] * (s.tyre_static_defl[i] + zr - zu) + s.c_tyre[i] * (zrd - zud);
        }

        // geometric load transfer routed straight to the contact patch (bypasses the springs).
        let long_axle = p.mass * ax * s.h_cg / s.wheelbase;
        let mut geom = [T::zero(); WHEELS];
        for i in 0..WHEELS {
            let (track_i, anti_i, lon_sign) = if p.wheels.front[i] {
                (s.track_f, s.anti_dive, -T::one())
            } else {
                (s.track_r, s.anti_squat, T::one())
            };
            let side = T::from(SIDE[i]).unwrap_or_else(T::zero);
            let lat = -(s.h_ra / track_i) * (p.mass * ay) * side * half;
            let lon = anti_i * long_axle * half * lon_sign;
            geom[i] = lat + lon;
        }

        // elastic transfer + suspension moments drive the ride DOF.
        let anti_mean = (s.anti_dive + s.anti_squat) * half;
        let m_roll_elastic = s.sprung_mass * ay * (s.h_s - s.h_ra);
        let m_pitch_elastic = -s.sprung_mass * ax * s.h_s * (T::one() - anti_mean);

        let mut m_pitch_susp = T::zero();
        let mut m_roll_susp = T::zero();
        let mut sum_susp = T::zero();
        for (i, &fs) in f_susp.iter().enumerate() {
            m_pitch_susp = m_pitch_susp - p.wheels.x[i] * fs;
            m_roll_susp = m_roll_susp + p.wheels.y[i] * fs;
            sum_susp = sum_susp + fs;
        }

        // Aero downforce on the sprung body (evaluated at the dynamic ride heights upstream, published
        // as the per-axle `AeroFzFront`/`AeroFzRear` channels, N, + = down). It heaves the sprung mass
        // and pitches it about the axle line (front at x[0]=a_f, rear at x[2]=−b_r); front downforce
        // ⇒ +pitch (nose-down). Reaching the tyres through the springs, it loads the contact patch —
        // the §6.1 "downforce car is real" coupling. Zero on a car with no aero (bus reads 0).
        let fz_aero_f = bus.get(CoreSignal::AeroFzFront, lane);
        let fz_aero_r = bus.get(CoreSignal::AeroFzRear, lane);
        let m_pitch_aero = fz_aero_f * p.wheels.x[0] + fz_aero_r * p.wheels.x[2];

        let heave = (sum_susp - s.sprung_mass * g_n - (fz_aero_f + fz_aero_r)) / s.sprung_mass;
        let pitch = (m_pitch_susp + m_pitch_elastic - m_gyro_y + m_pitch_aero) / s.iyy;
        let roll = (m_roll_susp + m_roll_elastic + m_gyro_x) / s.ixx;

        // position derivatives = the velocity states; velocity derivatives = the accelerations.
        dx.set_chassis(ChassisState::Heave, zd);
        dx.set_chassis(ChassisState::Pitch, thd);
        dx.set_chassis(ChassisState::Roll, phd);
        dx.set_chassis(ChassisState::HeaveRate, heave);
        dx.set_chassis(ChassisState::PitchRate, pitch);
        dx.set_chassis(ChassisState::RollRate, roll);

        for i in 0..WHEELS {
            let zud = x.chassis(chassis_zu_rate(i));
            let unsprung =
                (f_tyre[i] - f_susp[i] - s.unsprung_mass[i] * g_n + geom[i]) / s.unsprung_mass[i];
            dx.set_chassis(chassis_zu(i), zud);
            dx.set_chassis(chassis_zu_rate(i), unsprung);
        }
    }

    fn slow_derivatives(
        &self,
        _bus: &Bus<T>,
        _dslow: &mut outlap_core::state::SlowDerivView<T>,
        _lane: usize,
    ) {
    }

    fn equilibrium(&self, _bus: &mut Bus<T>, _slow: &SlowStateView<T>, _lane: usize) {}
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;
    use outlap_core::bus::ChannelInterner;
    use outlap_core::state::fast_slot_count;

    /// A simple symmetric test chassis: 1000 kg, Izz 1200, ±1.5 m axles, ±0.8 m track, R 0.33.
    fn test_chassis() -> (Chassis<f64>, RoadChannels, ChannelInterner) {
        let params = ChassisParams::<f64>::from_f64(
            1000.0,
            1200.0,
            [1.5, 1.5, -1.5, -1.5],
            [0.8, -0.8, 0.8, -0.8],
            [true, true, false, false],
            [0.33; WHEELS],
            [1.0; WHEELS],
        );
        let mut interner = ChannelInterner::new();
        let road = RoadChannels::intern(&mut interner);
        (Chassis::new(params, road), road, interner)
    }

    fn eval(chassis: &Chassis<f64>, bus: &mut Bus<f64>, fast: &[f64]) -> Vec<f64> {
        let mut dfast = vec![0.0; fast_slot_count()];
        let sv = StateView::new(fast, 1, 0);
        let mut dv = DerivView::new(&mut dfast, 1, 0);
        chassis.derivatives(&sv, bus, &mut dv, 0);
        dfast
    }

    #[test]
    fn flat_track_has_no_gravity_contribution() {
        // grade = bank = 0 ⇒ the planar EOM: ΣF is exactly the tyre-force sum, no gravity term.
        let (chassis, road, interner) = test_chassis();
        let mut bus = Bus::<f64>::with_interner(&interner, 1);
        bus.set_channel(road.grade, 0, 0.0);
        bus.set_channel(road.banking, 0, 0.0);
        // A pure longitudinal force on all four wheels (steer 0).
        for w in 0..WHEELS {
            bus.set_wheel(WheelSignal::TireFx, w, 0, 1000.0);
        }
        let mut fast = vec![0.0; fast_slot_count()];
        fast[ChassisState::Vx as usize] = 30.0;
        let dfast = eval(&chassis, &mut bus, &fast);
        // ΣFx = 4000 N ⇒ v̇x = 4000/1000 + r·vy = 4.0 (r = vy = 0). No gravity.
        assert!((dfast[ChassisState::Vx as usize] - 4.0).abs() < 1e-12);
        assert!(dfast[ChassisState::Vy as usize].abs() < 1e-12);
        assert!(dfast[ChassisState::YawRate as usize].abs() < 1e-12);
    }

    #[test]
    fn uphill_grade_decelerates() {
        // On a +grade (uphill), gravity has a −x component ⇒ v̇x < 0 with no other forces.
        let (chassis, road, interner) = test_chassis();
        let mut bus = Bus::<f64>::with_interner(&interner, 1);
        bus.set_channel(road.grade, 0, 0.1); // ~5.7° uphill
        let fast = vec![0.0; fast_slot_count()];
        let dfast = eval(&chassis, &mut bus, &fast);
        // v̇x = −g·sin(0.1) ≈ −0.979 m/s².
        let expect = -9.806_65 * 0.1_f64.sin();
        assert!((dfast[ChassisState::Vx as usize] - expect).abs() < 1e-9);
    }

    #[test]
    fn front_lateral_force_yaws_left() {
        // ISO 8855: a +y (leftward) force on the front axle ⇒ +yaw (CCW, left turn).
        let (chassis, road, interner) = test_chassis();
        let mut bus = Bus::<f64>::with_interner(&interner, 1);
        let _ = road;
        bus.set_wheel(WheelSignal::TireFy, 0, 0, 500.0); // FL
        bus.set_wheel(WheelSignal::TireFy, 1, 0, 500.0); // FR
        let fast = vec![0.0; fast_slot_count()];
        let dfast = eval(&chassis, &mut bus, &fast);
        // Front wheels at x = +1.5 ⇒ Mz = 1.5·(500+500) = 1500 ⇒ ṙ = 1500/1200 > 0.
        assert!(dfast[ChassisState::YawRate as usize] > 0.0);
        assert!((dfast[ChassisState::YawRate as usize] - 1500.0 / 1200.0).abs() < 1e-9);
    }

    #[test]
    fn curvilinear_kinematics_degenerate_on_a_straight() {
        // κ = 0, ψ_rel = 0 ⇒ ṡ = vx, ṅ = vy, ψ̇ = r.
        let (chassis, road, interner) = test_chassis();
        let mut bus = Bus::<f64>::with_interner(&interner, 1);
        bus.set_channel(road.kappa, 0, 0.0);
        let mut fast = vec![0.0; fast_slot_count()];
        fast[ChassisState::Vx as usize] = 25.0;
        fast[ChassisState::Vy as usize] = 1.5;
        fast[ChassisState::YawRate as usize] = 0.3;
        let dfast = eval(&chassis, &mut bus, &fast);
        assert!((dfast[ChassisState::S as usize] - 25.0).abs() < 1e-12);
        assert!((dfast[ChassisState::N as usize] - 1.5).abs() < 1e-12);
        assert!((dfast[ChassisState::PsiRel as usize] - 0.3).abs() < 1e-12);
    }

    #[test]
    fn drive_torque_spins_the_wheel_up_and_tire_force_slows_it() {
        let (chassis, road, interner) = test_chassis();
        let mut bus = Bus::<f64>::with_interner(&interner, 1);
        let _ = road;
        bus.set_wheel(WheelSignal::WheelDriveTorque, 2, 0, 500.0); // RL drive
        bus.set_wheel(WheelSignal::TireFx, 2, 0, 800.0); // reaction
        let mut fast = vec![0.0; fast_slot_count()];
        fast[ChassisState::OmegaRl as usize] = 90.0;
        let dfast = eval(&chassis, &mut bus, &fast);
        // I_w ω̇ = τ − R·Fx = 500 − 0.33·800 = 236 ⇒ ω̇ = 236 (I_w = 1).
        assert!((dfast[ChassisState::OmegaRl as usize] - (500.0 - 0.33 * 800.0)).abs() < 1e-9);
    }

    #[test]
    fn frame_denominator_is_floored_near_the_singularity() {
        // n·κ → 1 (curvature centre). ṡ must stay finite (floored denom), not blow up.
        let (chassis, road, interner) = test_chassis();
        let mut bus = Bus::<f64>::with_interner(&interner, 1);
        bus.set_channel(road.kappa, 0, 1.0);
        let mut fast = vec![0.0; fast_slot_count()];
        fast[ChassisState::N as usize] = 1.0; // 1 − n·κ = 0 exactly
        fast[ChassisState::Vx as usize] = 20.0;
        let dfast = eval(&chassis, &mut bus, &fast);
        assert!(dfast[ChassisState::S as usize].is_finite());
        assert!((dfast[ChassisState::S as usize] - 20.0 / DENOM_FLOOR).abs() < 1e-9);
    }
}

#[cfg(test)]
mod t3_tests {
    #![allow(clippy::float_cmp)]
    use super::*;
    use crate::params::SuspensionParams;
    use outlap_core::bus::ChannelInterner;
    use outlap_core::state::fast_slot_count;

    const STD_G: f64 = 9.806_65;

    /// A symmetric test car in static equilibrium: 800 kg total (740 sprung, 4×15 kg unsprung),
    /// ±1.7 m axles, 50/50 weight, with the spring and tyre static compressions set to carry each
    /// corner load, so `derivatives` at rest returns zero acceleration.
    fn equil<T: Float>() -> (ChassisParams<T>, SuspensionParams<T>) {
        let c = |v: f64| T::from(v).unwrap();
        let a = |v: [f64; WHEELS]| v.map(c);
        let params = ChassisParams::<T>::from_f64(
            800.0,
            1000.0,
            [1.7, 1.7, -1.7, -1.7],
            [0.825, -0.825, 0.80, -0.80],
            [true, true, false, false],
            [0.33; WHEELS],
            [1.1; WHEELS],
        );
        let m_unsprung = 15.0;
        let sprung_corner = 740.0 * STD_G / 4.0; // 50/50 front/rear on equal axles
        let kr = [220_000.0, 220_000.0, 240_000.0, 240_000.0];
        let ktz = [250_000.0; WHEELS];
        let mut d0 = [0.0; WHEELS];
        let mut dtz0 = [0.0; WHEELS];
        for i in 0..WHEELS {
            d0[i] = sprung_corner / kr[i];
            dtz0[i] = (sprung_corner + m_unsprung * STD_G) / ktz[i];
        }
        let susp = SuspensionParams::<T> {
            sprung_mass: c(740.0),
            ixx: c(180.0),
            iyy: c(950.0),
            h_s: c(0.32),
            h_cg: c(0.30),
            h_ra: c(0.05),
            wheelbase: c(3.4),
            track_f: c(1.65),
            track_r: c(1.60),
            anti_dive: c(0.0),
            anti_squat: c(0.0),
            arb_f: c(0.0),
            arb_r: c(0.0),
            bumpstop_smooth: c(0.005),
            k_ride: a(kr),
            static_defl: a(d0),
            damp_bump: a([4000.0; WHEELS]),
            damp_rebound: a([8000.0; WHEELS]),
            bumpstop_rate: a([5.0e5; WHEELS]),
            bumpstop_gap: a([0.03; WHEELS]),
            k_tyre: a(ktz),
            c_tyre: a([500.0; WHEELS]),
            tyre_static_defl: a(dtz0),
            unsprung_mass: a([m_unsprung; WHEELS]),
        };
        (params, susp)
    }

    fn build<T: Float>() -> (ChassisT3<T>, ChannelInterner) {
        let (params, susp) = equil::<T>();
        let mut interner = ChannelInterner::new();
        let road = RoadChannels::intern(&mut interner);
        let road_v = T3RoadVertical::intern(&mut interner);
        (ChassisT3::new(params, susp, road, road_v), interner)
    }

    fn eval(chassis: &ChassisT3<f64>, interner: &ChannelInterner, fast: &[f64]) -> Vec<f64> {
        let mut bus = Bus::<f64>::with_interner(interner, 1);
        let mut dfast = vec![0.0; fast_slot_count()];
        let sv = StateView::new(fast, 1, 0);
        let mut dv = DerivView::new(&mut dfast, 1, 0);
        chassis.derivatives(&sv, &mut bus, &mut dv, 0);
        dfast
    }

    /// A car at rest in its designed equilibrium has zero acceleration in every DOF (the ride block
    /// settled to the ride heights, the handling at rest). Ties the static compressions to gravity.
    #[test]
    fn static_equilibrium_settles() {
        let (chassis, interner) = build::<f64>();
        let fast = vec![0.0; fast_slot_count()]; // all states/rates zero, flat road, no tyre forces
        let d = eval(&chassis, &interner, &fast);
        for s in [
            ChassisState::Vx,
            ChassisState::Vy,
            ChassisState::YawRate,
            ChassisState::HeaveRate,
            ChassisState::PitchRate,
            ChassisState::RollRate,
            ChassisState::ZuRateFl,
            ChassisState::ZuRateFr,
            ChassisState::ZuRateRl,
            ChassisState::ZuRateRr,
        ] {
            assert!(
                d[s as usize].abs() < 1e-9,
                "{s:?} = {} not settled",
                d[s as usize]
            );
        }
    }

    /// Heave spring is restoring: dropping the sprung mass (z<0) drives it back up (heave accel>0).
    #[test]
    fn heave_spring_is_restoring() {
        let (chassis, interner) = build::<f64>();
        let mut fast = vec![0.0; fast_slot_count()];
        fast[ChassisState::Heave as usize] = -0.01;
        let d = eval(&chassis, &interner, &fast);
        assert!(d[ChassisState::HeaveRate as usize] > 0.0);
    }

    /// The damper opposes vertical velocity (negative feedback): the heave acceleration is smaller
    /// with the sprung mass rising than falling — the passive damper always removes energy.
    #[test]
    fn heave_damper_dissipates() {
        let (chassis, interner) = build::<f64>();
        let mut up = vec![0.0; fast_slot_count()];
        up[ChassisState::HeaveRate as usize] = 0.5; // rising
        let mut down = vec![0.0; fast_slot_count()];
        down[ChassisState::HeaveRate as usize] = -0.5; // falling
        let acc_up = eval(&chassis, &interner, &up)[ChassisState::HeaveRate as usize];
        let acc_down = eval(&chassis, &interner, &down)[ChassisState::HeaveRate as usize];
        assert!(
            acc_up < acc_down,
            "damper must oppose velocity: {acc_up} !< {acc_down}"
        );
    }

    /// Braking (a rearward force on all four contact patches) pitches the car nose-down (dive:
    /// θ>0 ⇒ pitch accel > 0). ISO-8855 sign convention, verified against the `SymPy` self-check.
    #[test]
    fn braking_dives() {
        let (chassis, interner) = build::<f64>();
        let mut bus = Bus::<f64>::with_interner(&interner, 1);
        for w in 0..WHEELS {
            bus.set_wheel(WheelSignal::TireFx, w, 0, -3000.0);
        }
        let mut dfast = vec![0.0; fast_slot_count()];
        let fast = vec![0.0; fast_slot_count()];
        let sv = StateView::new(&fast, 1, 0);
        let mut dv = DerivView::new(&mut dfast, 1, 0);
        chassis.derivatives(&sv, &mut bus, &mut dv, 0);
        assert!(dfast[ChassisState::PitchRate as usize] > 0.0);
    }

    /// Aero downforce (the per-axle `AeroFz*` channels) pushes the sprung platform DOWN: a positive
    /// downforce drives the heave acceleration negative (the §6.1 "downforce car is real" coupling —
    /// the load reaches the tyres by compressing the springs).
    #[test]
    fn aero_downforce_pushes_the_platform_down() {
        let (chassis, interner) = build::<f64>();
        let fast = vec![0.0; fast_slot_count()];
        // No aero ⇒ static equilibrium (zero heave accel).
        let d0 = eval(&chassis, &interner, &fast)[ChassisState::HeaveRate as usize];
        // With front+rear downforce the platform accelerates down.
        let mut bus = Bus::<f64>::with_interner(&interner, 1);
        bus.set(CoreSignal::AeroFzFront, 0, 5000.0);
        bus.set(CoreSignal::AeroFzRear, 0, 5000.0);
        let mut dfast = vec![0.0; fast_slot_count()];
        let sv = StateView::new(&fast, 1, 0);
        let mut dv = DerivView::new(&mut dfast, 1, 0);
        chassis.derivatives(&sv, &mut bus, &mut dv, 0);
        assert!(d0.abs() < 1e-9);
        assert!(
            dfast[ChassisState::HeaveRate as usize] < -1e-6,
            "downforce must push the platform down: {}",
            dfast[ChassisState::HeaveRate as usize]
        );
    }

    /// Front-axle aero downforce pitches the nose DOWN (θ̈ > 0 = dive), the mechanism behind the
    /// pitch-under-aero-load balance shift; rear downforce pitches it up.
    #[test]
    fn front_downforce_pitches_nose_down() {
        let (chassis, interner) = build::<f64>();
        let fast = vec![0.0; fast_slot_count()];
        let mut front = Bus::<f64>::with_interner(&interner, 1);
        front.set(CoreSignal::AeroFzFront, 0, 8000.0);
        let mut rear = Bus::<f64>::with_interner(&interner, 1);
        rear.set(CoreSignal::AeroFzRear, 0, 8000.0);
        let pitch = |bus: &mut Bus<f64>| {
            let mut dfast = vec![0.0; fast_slot_count()];
            let sv = StateView::new(&fast, 1, 0);
            let mut dv = DerivView::new(&mut dfast, 1, 0);
            chassis.derivatives(&sv, bus, &mut dv, 0);
            dfast[ChassisState::PitchRate as usize]
        };
        assert!(pitch(&mut front) > 1e-6, "front downforce ⇒ dive");
        assert!(pitch(&mut rear) < -1e-6, "rear downforce ⇒ nose-up");
    }

    /// The gyroscopic spin×yaw coupling is live: yaw rate on spinning wheels perturbs the roll
    /// acceleration (the term the T2 tier neglected).
    #[test]
    fn gyroscopic_coupling_is_live() {
        let (chassis, interner) = build::<f64>();
        let mut spin = vec![0.0; fast_slot_count()];
        for s in [
            ChassisState::OmegaFl,
            ChassisState::OmegaFr,
            ChassisState::OmegaRl,
            ChassisState::OmegaRr,
        ] {
            spin[s as usize] = 100.0;
        }
        let no_yaw = eval(&chassis, &interner, &spin)[ChassisState::RollRate as usize];
        spin[ChassisState::YawRate as usize] = 0.5;
        let with_yaw = eval(&chassis, &interner, &spin)[ChassisState::RollRate as usize];
        assert!((with_yaw - no_yaw).abs() > 1e-6);
    }

    /// The block is generic over `f32`: the f32 RHS tracks the f64 RHS to single-precision.
    #[test]
    #[allow(clippy::cast_possible_truncation)] // f64→f32 down-cast is the point of the parity test
    fn t3_rhs_is_generic_over_f32() {
        let (c64, _) = build::<f64>();
        let (c32, interner) = build::<f32>();
        // a mild perturbation so magnitudes stay moderate for the f32 comparison.
        let mut fast64 = vec![0.0f64; fast_slot_count()];
        fast64[ChassisState::Vx as usize] = 30.0;
        fast64[ChassisState::Heave as usize] = -0.005;
        fast64[ChassisState::Pitch as usize] = 0.01;
        fast64[ChassisState::ZuFl as usize] = 0.003;
        let d64 = eval(&c64, &interner, &fast64);

        let fast32: Vec<f32> = fast64.iter().map(|&v| v as f32).collect();
        let mut d32 = vec![0.0f32; fast_slot_count()];
        {
            let sv = StateView::new(&fast32, 1, 0);
            let mut dv = DerivView::new(&mut d32, 1, 0);
            let mut bus = Bus::<f32>::with_interner(&interner, 1);
            c32.derivatives(&sv, &mut bus, &mut dv, 0);
        }
        for s in [
            ChassisState::HeaveRate,
            ChassisState::PitchRate,
            ChassisState::ZuRateFl,
        ] {
            let a = d64[s as usize];
            let b = f64::from(d32[s as usize]);
            assert!(
                (a - b).abs() < 1e-3 * (1.0 + a.abs()),
                "{s:?}: f64={a} f32={b}"
            );
        }
    }
}
