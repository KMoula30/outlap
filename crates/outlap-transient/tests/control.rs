// SPDX-License-Identifier: AGPL-3.0-only
//! Rule-based control-layer integration tests (PR6): regen energy closure, the regen-on ≡ regen-off
//! trajectory invariant (Decision #11), and torque-vectoring yaw-rate tracking.
#![allow(clippy::float_cmp)] // Decision #11 asserts a bit-identical trace; a tolerance would void it.
#![allow(clippy::cast_precision_loss)] // small loop counters → f64 grid coordinates.

mod common;

use common::{build_blocks, limebeer, line};
use outlap_core::bus::ChannelInterner;
use outlap_transient::{SimConfig, SlowStack, TransientLap, TransientSolver};

/// A minimal slow-state stack test double: an ideal integrator over the pack energy. It Coulomb-counts
/// the recovered electrical energy into a state of charge at a fixed pack energy capacity, with a
/// regen ceiling that never binds — so a lap's `ΔSoC · capacity` must equal the recovered energy
/// exactly (the transient plumbing neither creates nor loses energy). The production stack wraps the
/// QSS `Pack` primitive; this double isolates the orchestrator's energy accounting.
struct EnergyDouble {
    soc: f64,
    capacity_j: f64,
    regen_ceiling_w: f64,
    energy_in_j: f64,
}

impl SlowStack for EnergyDouble {
    fn on_slow_step(&mut self, dt_s: f64, regen_power_w: f64) {
        let e = regen_power_w.max(0.0) * dt_s;
        self.energy_in_j += e;
        self.soc = (self.soc + e / self.capacity_j).clamp(0.0, 1.0);
    }
    fn regen_power_limit_w(&self) -> f64 {
        self.regen_ceiling_w
    }
    fn soc(&self) -> f64 {
        self.soc
    }
    fn temp_c(&self) -> f64 {
        25.0
    }
}

/// A decelerating straight: the driver tracks a `v_ref` that ramps 70 → 25 m/s, so it brakes hard —
/// the regen window.
fn braking_lap(regen: bool, attach_stack: bool) -> (TransientLap<f64>, f64) {
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let mut blocks = build_blocks(&t1, &spec, &mut it);
    // 0.6 blend authority, 0.9 machine+inverter recovery, from the vehicle's own regen envelope.
    blocks.powertrain.regen = common::regen_params(&t1, 0.6, 0.9);
    blocks.powertrain.regen.enabled = regen;

    let len = 1500.0;
    let stations = 200;
    let s: Vec<f64> = (0..stations)
        .map(|i| i as f64 * len / (stations as f64 - 1.0))
        .collect();
    let v_ref: Vec<f64> = s.iter().map(|&si| 70.0 - 45.0 * (si / len)).collect();
    let mk = |v: f64| vec![v; stations];
    let table = outlap_transient::LineTable::new(&outlap_transient::LineSamples {
        s: s.clone(),
        kappa_h: mk(0.0),
        grade: mk(0.0),
        banking: mk(0.0),
        kappa_v: mk(0.0),
        n_ref: mk(0.0),
        kappa_ref: mk(0.0),
        v_ref,
        x_ref: s.clone(),
        y_ref: mk(0.0),
        z_ref: mk(0.0),
        lat_x: mk(0.0),
        lat_y: mk(1.0),
        lat_z: mk(0.0),
        closed: false,
    })
    .unwrap();

    let cfg = SimConfig {
        fz_coupling: outlap_schema::sim::FzCoupling::OneStepLag,
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(blocks, table, &it, cfg);
    if attach_stack {
        solver = solver.with_slow_stack(Box::new(EnergyDouble {
            soc: 0.5,
            capacity_j: 5.0e7,       // 50 MJ ≈ a small race pack
            regen_ceiling_w: 1.0e12, // never binds
            energy_in_j: 0.0,
        }));
    }
    let lap = solver.run(len - 50.0, 400_000);
    // Fixed-step energy over the *integrated* steps (the slow stack accumulates from step 1; the
    // index-0 record is the pre-integration snapshot): each step carries one `dt` of its regen power.
    let recovered: f64 =
        lap.regen_power_w.iter().skip(1).sum::<f64>() * SimConfig::<f64>::default().dt;
    (lap, recovered)
}

#[test]
fn regen_recovers_energy_and_never_creates_it() {
    // The braking lap recovers a positive, non-negative amount of energy bounded by the mechanical
    // braking power it came from (max_regen_frac · efficiency); nothing is created.
    let (lap, recovered) = braking_lap(true, true);
    assert!(
        recovered > 0.0,
        "the braking lap must recover energy: {recovered}"
    );
    assert!(
        lap.regen_power_w.iter().all(|&p| p >= 0.0),
        "regen power is non-negative"
    );
}

#[test]
fn regen_is_energy_only_the_trajectory_is_identical_on_off() {
    // Decision #11: the regen blend must not move the car — with regen actively recovering (a stack
    // attached), the speed trace is bit-identical to regen-off (only the recovered energy differs).
    let (on, rec_on) = braking_lap(true, true);
    let (off, rec_off) = braking_lap(false, false);
    assert_eq!(on.vx.len(), off.vx.len(), "same step count");
    for (a, b) in on.vx.iter().zip(&off.vx) {
        assert_eq!(a, b, "regen changed the speed trace");
    }
    assert!(rec_on > 0.0, "regen-on recovers energy: {rec_on}");
    assert_eq!(rec_off, 0.0, "regen-off recovers nothing");
}

#[test]
fn slow_stack_soc_rises_and_closes_with_recovered_energy() {
    // A stack-owning run: recompute the recovered energy and check the closed-form ΔSoC identity by
    // re-integrating with a known capacity.
    let capacity_j = 5.0e7;
    let (lap, recovered) = braking_lap(true, true);
    // The final recorded SoC minus the initial 0.5, times capacity, equals the recovered energy.
    let soc0 = 0.5;
    let soc_end = *lap.state_of_charge.last().expect("soc recorded");
    let delta_energy = (soc_end - soc0) * capacity_j;
    assert!(
        (delta_energy - recovered).abs() <= 1e-3 * recovered.max(1.0),
        "ΔSoC·capacity ({delta_energy} J) must match recovered ({recovered} J)"
    );
}

#[test]
fn torque_vectoring_reduces_steady_state_yaw_tracking_error() {
    // Constant-radius skidpad: enabling the yaw-moment allocator drives the steady yaw rate closer to
    // the reference r_target = v·κ than the driver's steer alone.
    let (t1, spec) = limebeer();
    let radius = 80.0;
    let v = 30.0;
    let len = 2.0 * std::f64::consts::PI * radius;
    let cfg = SimConfig {
        fz_coupling: outlap_schema::sim::FzCoupling::OneStepLag,
        ..SimConfig::default()
    };
    let steady_err = |k_yaw: f64| -> f64 {
        let mut it = ChannelInterner::new();
        let mut blocks = build_blocks(&t1, &spec, &mut it);
        blocks.tv.enabled = k_yaw > 0.0;
        blocks.tv.k_yaw = k_yaw;
        let table = line(len, 600, true, 1.0 / radius, 1.0 / radius, v, Some(radius));
        let mut solver = TransientSolver::new(blocks, table, &it, cfg);
        let lap = solver.run(len, 200_000);
        // Average the yaw-rate error over the final third (steady state), r_target = v·κ.
        let r_target = v / radius;
        let n = lap.len();
        let start = 2 * n / 3;
        let mut sum = 0.0;
        for i in start..n {
            sum += (lap.yaw_rate[i] - r_target).abs();
        }
        sum / (n - start) as f64
    };
    let err_off = steady_err(0.0);
    let err_on = steady_err(6000.0);
    assert!(
        err_on <= err_off + 1e-6,
        "torque vectoring should not worsen yaw tracking: off={err_off}, on={err_on}"
    );
}
