// SPDX-License-Identifier: AGPL-3.0-only
//! The `actuate`-phase force blocks: [`Aero`] (drag + downforce), [`LoadTransfer`] (algebraic
//! per-wheel `F_z`, reusing the T1 expressions), and [`Tire`] (per-wheel slip → force, plus the
//! relaxation targets the split integrator's exact-exponential channel consumes).
//!
//! These blocks publish onto the [`Bus`]; the chassis (integrate phase) reads them back. Tyre slips
//! and the ISO-W sign contract match the T1 trim solver (`outlap_qss::t1::trim`) so the tiers agree.

use num_traits::Float;

use outlap_core::block::{Block, Phase, Ports};
use outlap_core::bus::{Bus, CoreSignal, WheelSignal, WHEELS};
use outlap_core::state::{ChassisState, StateView};
use outlap_tire::{relax_step, Relaxation, SlipState, TireModel};

use crate::params::WheelGeometry;

/// Minimum contact-patch speed used in the slip denominators, m/s (avoids the standstill 0/0).
const VX_LOW: f64 = 1.0;

/// Floor on per-wheel normal load `F_z`, N. A car that goes light over a crest at speed can drive
/// `F_z` to exactly zero, at which the tyre relaxation length `σ(F_z) → 0` and the exact-exponential
/// slip update becomes ill-posed (a zero-length filter has infinite bandwidth). Flooring `F_z` at a
/// small positive value keeps `σ` finite; the force it produces (≈ `μ·F_z_floor`) is negligible on
/// the ground — a few newtons against a multi-kN wheel load — so it never perturbs a planted lap.
pub const FZ_FLOOR_N: f64 = 10.0;

// ---------------------------------------------------------------------------------------------
// Aero
// ---------------------------------------------------------------------------------------------

/// Lumped aero block: drag `= q_x·v²` opposing `+x`, and per-axle downforce `q_{z,f/r}·v²` (+down).
///
/// Speed-dependent ride-height (raking) aero maps are a T1/T3 refinement; the T2 skeleton uses the
/// constant lumped coefficients (`½ρ·C·A`), matching the T1 `flat` aero platform.
#[derive(Clone, Copy, Debug)]
pub struct Aero<T> {
    /// Drag coefficient `q_x = ½ρ·C_xA`, N/(m/s)².
    pub qx: T,
    /// Front downforce coefficient `q_{z,f} = ½ρ·C_zA_f`, N/(m/s)².
    pub qz_f: T,
    /// Rear downforce coefficient `q_{z,r} = ½ρ·C_zA_r`, N/(m/s)².
    pub qz_r: T,
}

impl<T: Float> Block<T> for Aero<T> {
    fn phase(&self) -> Phase {
        Phase::Actuate
    }

    fn ports(&self) -> Ports {
        Ports::new(
            Vec::new(),
            vec![
                CoreSignal::AeroDrag as usize,
                CoreSignal::AeroFzFront as usize,
                CoreSignal::AeroFzRear as usize,
            ],
        )
    }

    fn derivatives(
        &self,
        x: &StateView<T>,
        bus: &mut Bus<T>,
        _dx: &mut outlap_core::state::DerivView<T>,
        lane: usize,
    ) {
        // Forward speed drives the platform; lateral velocity is second-order for lumped aero.
        let vx = x.chassis(ChassisState::Vx);
        let v2 = vx * vx;
        bus.set(CoreSignal::AeroDrag, lane, self.qx * v2);
        bus.set(CoreSignal::AeroFzFront, lane, self.qz_f * v2);
        bus.set(CoreSignal::AeroFzRear, lane, self.qz_r * v2);
    }
}

// ---------------------------------------------------------------------------------------------
// Load transfer (algebraic Fz)
// ---------------------------------------------------------------------------------------------

/// Algebraic per-wheel normal load `F_z` from the shared T1 load-transfer algebra
/// (`outlap_qss::t1::load_transfer`) — HANDOFF §6.1 "same expressions as T1".
///
/// The longitudinal/lateral accelerations that drive the transfer come from the resolved
/// `fz_coupling` (Decision #29): the orchestrator supplies `(a_x, a_y)` (one-step-lagged or from a
/// fixed-point iterate) via [`set_accel`]. The block is `f64` at the load-transfer boundary (the T1
/// algebra is `f64`) and casts back into `T`.
#[derive(Clone, Copy, Debug)]
pub struct LoadTransfer<T> {
    /// Mass/suspension geometry for the shared algebra.
    pub geom: outlap_qss::t1::LoadTransferGeometry,
    /// Apparent normal gravity `g cosθ cosφ` (+ vertical-curvature term), m/s²; set per step.
    pub g_normal: T,
    /// Reference speed `v` for the downforce terms, m/s; set per step.
    pub speed: T,
    /// Longitudinal acceleration feeding pitch transfer, m/s²; set per step per `fz_coupling`.
    pub ax: T,
    /// Lateral acceleration feeding roll transfer, m/s²; set per step per `fz_coupling`.
    pub ay: T,
    /// Front downforce coefficient (mirrors [`Aero::qz_f`]), N/(m/s)².
    pub qz_f: T,
    /// Rear downforce coefficient (mirrors [`Aero::qz_r`]), N/(m/s)².
    pub qz_r: T,
}

impl<T: Float> LoadTransfer<T> {
    /// Update the per-step operating point (speed, apparent gravity, coupling accelerations).
    pub fn set_operating_point(&mut self, speed: T, g_normal: T, ax: T, ay: T) {
        self.speed = speed;
        self.g_normal = g_normal;
        self.ax = ax;
        self.ay = ay;
    }
}

impl<T: Float> Block<T> for LoadTransfer<T> {
    fn phase(&self) -> Phase {
        Phase::Actuate
    }

    fn ports(&self) -> Ports {
        let base = CoreSignal::COUNT as usize;
        let writes = (0..WHEELS)
            .map(|w| base + (WheelSignal::TireFz as usize) * WHEELS + w)
            .collect();
        Ports::new(Vec::new(), writes)
    }

    fn derivatives(
        &self,
        _x: &StateView<T>,
        bus: &mut Bus<T>,
        _dx: &mut outlap_core::state::DerivView<T>,
        lane: usize,
    ) {
        let to_f = |v: T| v.to_f64().unwrap_or(0.0);
        let fz = outlap_qss::t1::load_transfer(
            &self.geom,
            to_f(self.speed),
            to_f(self.g_normal),
            to_f(self.ax),
            to_f(self.ay),
            to_f(self.qz_f),
            to_f(self.qz_r),
        );
        let floor = T::from(FZ_FLOOR_N).unwrap_or_else(T::zero);
        for (w, &fz_w) in fz.iter().enumerate() {
            // Floor at a small positive load so a wheel that goes light over a crest keeps a finite
            // relaxation length `σ(F_z)` (the exact-exponential slip update needs `σ > 0`).
            let fz_floored = T::from(fz_w).unwrap_or_else(T::zero).max(floor);
            bus.set_wheel(WheelSignal::TireFz, w, lane, fz_floored);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Tire
// ---------------------------------------------------------------------------------------------

/// Per-axle relaxation-length provider (Mf6.1 tyres carry the `PT*`/carcass data; others fall back
/// to a fixed length so the exact-exponential update still has a finite `σ`).
#[derive(Clone, Debug)]
pub enum RelaxProvider<T> {
    /// A Magic-Formula provider with load-dependent `σ_κ(F_z)`, `σ_α(F_z, γ)`.
    Mf(Relaxation<T>),
    /// A fixed longitudinal/lateral relaxation length (m).
    Fixed(T, T),
}

impl<T: Float> RelaxProvider<T> {
    /// Build from a tyre model: the Mf6.1 route when available, else a fixed `0.5·R0`-style length.
    #[must_use]
    pub fn from_model(model: &TireModel<T>, fallback_m: T) -> Self {
        match model {
            TireModel::Mf61(m) => RelaxProvider::Mf(Relaxation::from_params(m.params()).0),
            TireModel::Brush(_) => RelaxProvider::Fixed(fallback_m, fallback_m),
        }
    }

    /// Longitudinal relaxation length `σ_κ` at load `fz`, m.
    #[must_use]
    pub fn sigma_kappa(&self, fz: T) -> T {
        match self {
            RelaxProvider::Mf(r) => r.sigma_kappa(fz),
            RelaxProvider::Fixed(sk, _) => *sk,
        }
    }

    /// Lateral relaxation length `σ_α` at load `fz`, camber `gamma`, m.
    #[must_use]
    pub fn sigma_alpha(&self, fz: T, gamma: T) -> T {
        match self {
            RelaxProvider::Mf(r) => r.sigma_alpha(fz, gamma),
            RelaxProvider::Fixed(_, sa) => *sa,
        }
    }
}

/// The steady-state slip targets and relaxation data for one wheel this step.
#[derive(Clone, Copy, Debug)]
pub struct RelaxTargets<T> {
    /// Steady-state longitudinal slip `κ_ss`.
    pub kappa_ss: T,
    /// Steady-state slip angle `α_ss`, rad.
    pub alpha_ss: T,
    /// Contact-patch longitudinal speed magnitude `|V_x|`, m/s.
    pub v_abs: T,
    /// Longitudinal relaxation length `σ_κ`, m.
    pub sigma_kappa: T,
    /// Lateral relaxation length `σ_α`, m.
    pub sigma_alpha: T,
}

/// The tyre block: contact-patch kinematics → slip → force (with relaxation-lagged slip), for all
/// four wheels. Holds the per-axle force models, inflation pressures, friction scale, and relaxation
/// providers. Writes wheel-frame `F_x/F_y/M_z` and mirrors the lagged/steady slips onto the bus.
#[derive(Clone, Debug)]
pub struct Tire<T> {
    /// Front-axle force model.
    pub front: TireModel<T>,
    /// Rear-axle force model.
    pub rear: TireModel<T>,
    /// Front inflation pressure, Pa.
    pub p_front: T,
    /// Rear inflation pressure, Pa.
    pub p_rear: T,
    /// Uniform friction scale (1.0 until the M5 thermal grip model).
    pub mu_scale: T,
    /// Front relaxation provider.
    pub relax_front: RelaxProvider<T>,
    /// Rear relaxation provider.
    pub relax_rear: RelaxProvider<T>,
    /// Per-wheel geometry (positions, radii).
    pub wheels: WheelGeometry<T>,
}

impl<T: Float> Tire<T> {
    fn axle(&self, wheel: usize) -> (&TireModel<T>, T, &RelaxProvider<T>) {
        if self.wheels.front[wheel] {
            (&self.front, self.p_front, &self.relax_front)
        } else {
            (&self.rear, self.p_rear, &self.relax_rear)
        }
    }

    /// Contact-patch wheel-frame velocity `(V_wx, V_wy)` for wheel `i` at state `x`, given steer.
    #[inline]
    fn contact_velocity(&self, x: &StateView<T>, i: usize, steer: T) -> (T, T) {
        let vx = x.chassis(ChassisState::Vx);
        let vy = x.chassis(ChassisState::Vy);
        let r = x.chassis(ChassisState::YawRate);
        let d = if self.wheels.front[i] {
            steer
        } else {
            T::zero()
        };
        let (sn, cs) = d.sin_cos();
        let vcx = vx - r * self.wheels.y[i];
        let vcy = vy + r * self.wheels.x[i];
        (vcx * cs + vcy * sn, -vcx * sn + vcy * cs)
    }

    /// Steady-state slip + relaxation lengths for wheel `i` (consumed by the relaxation sub-step).
    #[must_use]
    pub fn relax_targets(&self, x: &StateView<T>, i: usize, steer: T, fz: T) -> RelaxTargets<T> {
        let (vwx, vwy) = self.contact_velocity(x, i, steer);
        let vx_low = T::from(VX_LOW).unwrap_or_else(T::one);
        let v_abs = vwx.abs().max(vx_low);
        let omega = x.chassis(omega_slot(i));
        let kappa_ss = (omega * self.wheels.radius[i] - vwx) / v_abs;
        let alpha_ss = vwy.atan2(vwx.abs());
        let (_, _, relax) = self.axle(i);
        let gamma = T::zero();
        RelaxTargets {
            kappa_ss,
            alpha_ss,
            v_abs,
            sigma_kappa: relax.sigma_kappa(fz),
            sigma_alpha: relax.sigma_alpha(fz, gamma),
        }
    }
}

impl<T: Float> Block<T> for Tire<T> {
    fn phase(&self) -> Phase {
        Phase::Actuate
    }

    fn ports(&self) -> Ports {
        let base = CoreSignal::COUNT as usize;
        let ch = |sig: WheelSignal, w: usize| base + (sig as usize) * WHEELS + w;
        let mut reads = vec![CoreSignal::Steer as usize];
        let mut writes = Vec::new();
        for w in 0..WHEELS {
            reads.push(ch(WheelSignal::TireFz, w));
            for sig in [
                WheelSignal::TireFx,
                WheelSignal::TireFy,
                WheelSignal::TireMz,
                WheelSignal::SlipKappa,
                WheelSignal::SlipAlpha,
                WheelSignal::SlipKappaSs,
                WheelSignal::SlipAlphaSs,
            ] {
                writes.push(ch(sig, w));
            }
        }
        Ports::new(reads, writes)
    }

    fn derivatives(
        &self,
        x: &StateView<T>,
        bus: &mut Bus<T>,
        _dx: &mut outlap_core::state::DerivView<T>,
        lane: usize,
    ) {
        let steer = bus.get(CoreSignal::Steer, lane);
        for i in 0..WHEELS {
            let (vwx, _) = self.contact_velocity(x, i, steer);
            let fz = bus.get_wheel(WheelSignal::TireFz, i, lane);
            // Lagged (relaxation) slip from the SoA state feeds the force model.
            let kappa = x.relax(outlap_core::state::RelaxState::Kappa, i);
            let alpha = x.relax(outlap_core::state::RelaxState::Alpha, i);
            let (model, p, _) = self.axle(i);
            let mut slip = SlipState::new(kappa, alpha, T::zero(), fz, p, vwx);
            slip.mu_scale_x = self.mu_scale;
            slip.mu_scale_y = self.mu_scale;
            let f = model.forces(&slip);
            bus.set_wheel(WheelSignal::TireFx, i, lane, f.fx);
            bus.set_wheel(WheelSignal::TireFy, i, lane, f.fy);
            bus.set_wheel(WheelSignal::TireMz, i, lane, f.mz);
            bus.set_wheel(WheelSignal::SlipKappa, i, lane, kappa);
            bus.set_wheel(WheelSignal::SlipAlpha, i, lane, alpha);
            // Publish the steady-state targets for the result surface / diagnostics.
            let targets = self.relax_targets(x, i, steer, fz);
            bus.set_wheel(WheelSignal::SlipKappaSs, i, lane, targets.kappa_ss);
            bus.set_wheel(WheelSignal::SlipAlphaSs, i, lane, targets.alpha_ss);
        }
    }
}

/// Advance one wheel's relaxation state pair with the exact-exponential update over `dt` (the split
/// integrator's stiff channel, HANDOFF §11.2). Returns the new `(κ, α)` lagged slip.
#[must_use]
pub fn relax_wheel<T: Float>(kappa: T, alpha: T, tg: &RelaxTargets<T>, dt: T) -> (T, T) {
    let k = relax_step(kappa, tg.kappa_ss, tg.v_abs, dt, tg.sigma_kappa);
    let a = relax_step(alpha, tg.alpha_ss, tg.v_abs, dt, tg.sigma_alpha);
    (k, a)
}

/// The [`ChassisState`] wheel-speed slot for wheel `i`.
#[inline]
fn omega_slot(wheel: usize) -> ChassisState {
    match wheel {
        0 => ChassisState::OmegaFl,
        1 => ChassisState::OmegaFr,
        2 => ChassisState::OmegaRl,
        _ => ChassisState::OmegaRr,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use outlap_core::bus::ChannelInterner;
    use outlap_core::state::fast_slot_count;
    use outlap_qss::t1::LoadTransferGeometry;

    /// A symmetric test geometry for the shared load-transfer algebra (values are representative,
    /// not a specific car — the property under test is the floor, not the exact split).
    fn geom() -> LoadTransferGeometry {
        LoadTransferGeometry {
            mass_kg: 800.0,
            wheelbase_m: 3.0,
            a_f: 1.5,
            b_r: 1.5,
            t_f: 1.6,
            t_r: 1.6,
            h_cg: 0.3,
            h_ra: 0.1,
            rc_f: 0.5,
            rc_r: 0.5,
            roll_share_f: 0.5,
            roll_share_r: 0.5,
        }
    }

    fn load_block(g_normal: f64, ax: f64, ay: f64, speed: f64) -> LoadTransfer<f64> {
        LoadTransfer {
            geom: geom(),
            g_normal,
            speed,
            ax,
            ay,
            qz_f: 0.0,
            qz_r: 0.0,
        }
    }

    fn eval_fz(block: &LoadTransfer<f64>) -> [f64; WHEELS] {
        let mut it = ChannelInterner::new();
        let mut bus = Bus::<f64>::with_interner(&it, 1);
        // `LoadTransfer` writes into the fixed per-wheel `TireFz` region, so no channels to intern;
        // touch the interner to keep the width correct.
        let _ = &mut it;
        let fast = vec![0.0; fast_slot_count()];
        let sv = StateView::new(&fast, 1, 0);
        let mut dfast = vec![0.0; fast_slot_count()];
        let mut dv = outlap_core::state::DerivView::new(&mut dfast, 1, 0);
        block.derivatives(&sv, &mut bus, &mut dv, 0);
        let mut fz = [0.0; WHEELS];
        for (w, slot) in fz.iter_mut().enumerate() {
            *slot = bus.get_wheel(WheelSignal::TireFz, w, 0);
        }
        fz
    }

    #[test]
    fn a_light_crest_floors_fz_at_a_small_positive_value() {
        // A negative road-normal specific gravity (a crest unloading beyond any downforce) drives the
        // raw load-transfer output to zero on every wheel. The block must still publish `≥ FZ_FLOOR_N`
        // so the relaxation length `σ(F_z)` stays finite downstream.
        let block = load_block(-5.0, 0.0, 0.0, 0.0);
        for fz in eval_fz(&block) {
            assert!(
                fz >= FZ_FLOOR_N,
                "a light wheel must floor at {FZ_FLOOR_N} N, got {fz}"
            );
        }
    }

    #[test]
    fn the_floor_never_perturbs_a_planted_lap() {
        // Under a normal 1 g load with a little combined accel, every wheel carries kN-scale load, far
        // above the floor — so the floor is inert (it only ever lifts a would-be-zero wheel).
        let block = load_block(9.806_65, 3.0, 5.0, 60.0);
        let fz = eval_fz(&block);
        assert!(
            fz.iter().all(|&f| f > 50.0 * FZ_FLOOR_N),
            "planted loads {fz:?} should dwarf the floor"
        );
        // ΣF_z == m·g_normal to the newton: no wheel was floored, so the floor added nothing.
        let sum: f64 = fz.iter().sum();
        assert!((sum - 800.0 * 9.806_65).abs() < 1.0, "sum {sum}");
    }
}
