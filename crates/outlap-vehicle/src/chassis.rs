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
use outlap_core::bus::{Bus, CoreSignal, WheelSignal, WHEELS};
use outlap_core::state::{ChassisState, DerivView, SlowStateView, StateView};

use crate::params::{ChassisParams, RoadChannels};

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
