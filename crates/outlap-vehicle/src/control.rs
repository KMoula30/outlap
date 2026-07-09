// SPDX-License-Identifier: AGPL-3.0-only
//! Minimal `control`/`actuate` blocks that close the loop for the M4 skeleton demo: a **placeholder**
//! [`Driver`] (curvature feed-forward steer + proportional path/speed tracking) and a **placeholder**
//! [`Powertrain`] (throttle/brake → per-wheel drive/brake torque, even split).
//!
//! These exist only so the transient tier produces a runnable closed-loop lap in this PR. They are
//! deliberately simple and are **superseded** by the real blocks later in M4:
//! * the MacAdam-preview driver with PI speed-tracking and gg-headroom feed-forward — **PR5**;
//! * the powertrain map + gear-shift FSM + torque-vectoring allocator — **PR6**.
//!
//! They add **no** controller state to the `SoA` (pure proportional laws), so the fast-state layout is
//! untouched. Gains are literature-flavoured defaults, surfaced as estimated in the loaded-model
//! report by the assembly pipeline that installs them.

use num_traits::Float;

use outlap_core::block::{Block, Phase, Ports};
use outlap_core::bus::{Bus, CoreSignal, WheelSignal, WHEELS};
use outlap_core::state::{ChassisState, DerivView, StateView};

use crate::params::RoadChannels;

/// Placeholder path/speed-tracking driver (Decision #21 deterministic; **superseded by PR5**).
///
/// Steer = curvature feed-forward `δ_ff = L·κ_ref` plus a proportional lateral-offset/heading
/// correction and a yaw-rate damping term `k_n·(n_ref − n) − k_ψ·ψ_rel − k_r·r`, saturated to a
/// physical steer limit `±δ_max`. The yaw-rate damping keeps the path loop from ringing (a real
/// `MacAdam` preview controller replaces this in PR5). Longitudinal demand is proportional to the
/// speed error `v_ref − v_x`: positive → throttle, negative → brake, each saturated to `[0, 1]`.
#[derive(Clone, Copy, Debug)]
pub struct Driver<T> {
    /// Wheelbase `L`, m (curvature feed-forward arm).
    pub wheelbase: T,
    /// Lateral-offset tracking gain `k_n`, rad/m.
    pub k_offset: T,
    /// Heading-error gain `k_ψ`, rad/rad.
    pub k_heading: T,
    /// Yaw-rate damping gain `k_r`, rad/(rad/s).
    pub k_yaw_rate: T,
    /// Speed-error gain `k_v`, per (m/s).
    pub k_speed: T,
    /// Steer saturation `δ_max`, rad.
    pub max_steer: T,
    /// Interned road channels (reads `n_ref`, `κ_ref`, `v_ref`).
    pub road: RoadChannels,
}

impl<T: Float> Block<T> for Driver<T> {
    fn phase(&self) -> Phase {
        Phase::Control
    }

    fn ports(&self) -> Ports {
        Ports::new(
            vec![
                self.road.n_ref.index(),
                self.road.kappa_ref.index(),
                self.road.v_ref.index(),
            ],
            vec![
                CoreSignal::Steer as usize,
                CoreSignal::Throttle as usize,
                CoreSignal::Brake as usize,
            ],
        )
    }

    fn derivatives(&self, x: &StateView<T>, bus: &mut Bus<T>, _dx: &mut DerivView<T>, lane: usize) {
        let n = x.chassis(ChassisState::N);
        let psi = x.chassis(ChassisState::PsiRel);
        let vx = x.chassis(ChassisState::Vx);
        let r = x.chassis(ChassisState::YawRate);

        let n_ref = bus.get_channel(self.road.n_ref, lane);
        let kappa_ref = bus.get_channel(self.road.kappa_ref, lane);
        let v_ref = bus.get_channel(self.road.v_ref, lane);

        let raw = self.wheelbase * kappa_ref + self.k_offset * (n_ref - n)
            - self.k_heading * psi
            - self.k_yaw_rate * r;
        let steer = raw.max(-self.max_steer).min(self.max_steer);
        bus.set(CoreSignal::Steer, lane, steer);

        let demand = self.k_speed * (v_ref - vx);
        let (zero, one) = (T::zero(), T::one());
        let throttle = demand.max(zero).min(one);
        let brake = (-demand).max(zero).min(one);
        bus.set(CoreSignal::Throttle, lane, throttle);
        bus.set(CoreSignal::Brake, lane, brake);
    }
}

/// Placeholder powertrain/brake actuation (**superseded by PR6**): throttle × a constant max drive
/// torque split evenly across the driven wheels; brake × a constant max brake torque split by the
/// front/rear balance bar. No gear FSM, no powertrain map, no torque vectoring yet.
#[derive(Clone, Copy, Debug)]
pub struct Powertrain<T> {
    /// Maximum total drive torque at the wheels, N·m.
    pub max_drive_torque: T,
    /// Maximum total brake torque, N·m.
    pub max_brake_torque: T,
    /// Front brake-force bias, `0..1`.
    pub brake_front_bias: T,
    /// Driven-wheel mask `[FL, FR, RL, RR]`.
    pub driven: [bool; WHEELS],
}

impl<T: Float> Block<T> for Powertrain<T> {
    fn phase(&self) -> Phase {
        Phase::Actuate
    }

    fn ports(&self) -> Ports {
        let base = CoreSignal::COUNT as usize;
        let ch = |sig: WheelSignal, w: usize| base + (sig as usize) * WHEELS + w;
        let mut writes = Vec::new();
        for w in 0..WHEELS {
            writes.push(ch(WheelSignal::WheelDriveTorque, w));
            writes.push(ch(WheelSignal::WheelBrakeTorque, w));
        }
        Ports::new(
            vec![CoreSignal::Throttle as usize, CoreSignal::Brake as usize],
            writes,
        )
    }

    fn derivatives(
        &self,
        _x: &StateView<T>,
        bus: &mut Bus<T>,
        _dx: &mut DerivView<T>,
        lane: usize,
    ) {
        let throttle = bus.get(CoreSignal::Throttle, lane);
        let brake = bus.get(CoreSignal::Brake, lane);
        let n_driven = self.driven.iter().filter(|&&d| d).count().max(1);
        let per_drive = throttle * self.max_drive_torque / T::from(n_driven).unwrap_or_else(T::one);

        let total_brake = brake * self.max_brake_torque;
        let two = T::one() + T::one();
        let front_share = self.brake_front_bias * total_brake / two; // per front wheel
        let rear_share = (T::one() - self.brake_front_bias) * total_brake / two; // per rear wheel

        for w in 0..WHEELS {
            let drive = if self.driven[w] { per_drive } else { T::zero() };
            bus.set_wheel(WheelSignal::WheelDriveTorque, w, lane, drive);
            let brake_w = if w < 2 { front_share } else { rear_share };
            bus.set_wheel(WheelSignal::WheelBrakeTorque, w, lane, brake_w);
        }
    }
}
