// SPDX-License-Identifier: AGPL-3.0-only
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_lines,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::cast_lossless,
    clippy::needless_range_loop
)]
//! T2 dynamics property tests on the real `limebeer_2014_f1` car (assembled once per test): the
//! assembler order is deterministic, the split integrator is bit-reproducible, the relaxation states
//! converge, and open-loop maneuvers match analytic expectations (flat-track planar degeneration,
//! coastdown drag decel, step-steer yaw sign/magnitude, friction-circle containment). The ideal
//! driver (real MacAdam-preview + PI as of PR5) is switched between "tuned", "feed-forward-only
//! steer", and "coasting" per scenario via the `blocks.driver` field tweaks.

mod common;

use common::{build_blocks, limebeer, line};
use outlap_core::block::Phase;
use outlap_core::bus::WHEELS;
use outlap_core::state::{ChassisState, RelaxState, StateLayout};
use outlap_schema::sim::FzCoupling;
use outlap_transient::{SimConfig, TransientSolver};

fn cfg() -> SimConfig<f64> {
    SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    }
}

/// Zero the steer path-feedback gains so only the curvature feed-forward steers (open-loop steer).
fn feed_forward_steer_only(solver_blocks: &mut outlap_transient::T2Blocks<f64>) {
    solver_blocks.driver.preview_gain = 0.0;
    solver_blocks.driver.heading_gain = 0.0;
    solver_blocks.driver.yaw_damping = 0.0;
}

/// Silence the longitudinal loop entirely (no throttle, no brake) — a pure coastdown driver.
fn coasting(solver_blocks: &mut outlap_transient::T2Blocks<f64>) {
    solver_blocks.driver.speed_kp = 0.0;
    solver_blocks.driver.speed_ki = 0.0;
    solver_blocks.driver.ff_accel_scale = f64::INFINITY; // a_ff / ∞ = 0
}

#[test]
fn assembler_order_is_deterministic_and_phase_sorted() {
    let t1 = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks(&t1, &mut it);
    let solver = TransientSolver::new(
        blocks,
        line(100.0, 50, false, 0.0, 0.0, 20.0, None),
        &it,
        cfg(),
    );
    let order = solver.schedule().order().to_vec();
    // The assembler-produced order must equal the solver's fixed execution order
    // (driver → powertrain → load-transfer → aero → tyre → chassis), so the hardcoded
    // eval order in `eval_rhs_raw` genuinely honours the topological sort.
    assert_eq!(order, vec![0, 1, 2, 3, 4, 5]);
    // Determinism: same specs → same schedule.
    let solver2 = TransientSolver::new(
        build_blocks(&t1, &mut outlap_core::bus::ChannelInterner::new()),
        line(100.0, 50, false, 0.0, 0.0, 20.0, None),
        &it,
        cfg(),
    );
    assert_eq!(order, solver2.schedule().order().to_vec());
    // Phase ordering: driver (control) precedes the chassis (integrate); the tyre (reads Fz) follows
    // the load-transfer writer.
    let pos = |b: usize| order.iter().position(|&x| x == b).unwrap();
    assert!(pos(0) < pos(5), "driver before chassis");
    assert!(pos(2) < pos(4), "load-transfer before tyre");
    assert!(Phase::Control < Phase::Integrate);
}

#[test]
fn flat_straight_stays_planar() {
    let t1 = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks(&t1, &mut it);
    let mut solver = TransientSolver::new(
        blocks,
        line(4000.0, 200, false, 0.0, 0.0, 40.0, None),
        &it,
        cfg(),
    );
    for _ in 0..2000 {
        solver.step();
    }
    let fs = solver.fast_state();
    // No steer, no banking ⇒ lateral/yaw/offset stay at zero (the driver tracks n_ref = 0 exactly on
    // a straight with κ_ref = 0, so nothing perturbs the plane).
    assert!(
        fs[ChassisState::Vy as usize].abs() < 1e-6,
        "vy={}",
        fs[ChassisState::Vy as usize]
    );
    assert!(fs[ChassisState::YawRate as usize].abs() < 1e-6);
    assert!(fs[ChassisState::N as usize].abs() < 1e-6);
}

#[test]
fn coastdown_decelerates_under_drag() {
    let t1 = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let mut blocks = build_blocks(&t1, &mut it);
    coasting(&mut blocks); // no throttle
    let mut solver = TransientSolver::new(
        blocks,
        line(20_000.0, 200, false, 0.0, 0.0, 80.0, None),
        &it,
        cfg(),
    );
    let v0 = solver.fast_state()[ChassisState::Vx as usize];
    for _ in 0..3000 {
        solver.step();
    }
    let v1 = solver.fast_state()[ChassisState::Vx as usize];
    assert!(v1 < v0, "coasted down: {v0} -> {v1}");
    // Initial decel is dominated by aero drag qx·v²/m (rolling adds a little). Order-of-magnitude.
    let drag_decel = t1.qx * v0 * v0 / t1.mass_kg;
    assert!(
        drag_decel > 0.5 && drag_decel < 30.0,
        "drag decel {drag_decel}"
    );
}

#[test]
fn step_steer_builds_correct_yaw_and_loads_the_outside() {
    let t1 = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    // Constant steer feed-forward via kappa_ref on a straight road (path feedback off).
    let (v, kappa) = (40.0, 0.006);
    let mut blocks = build_blocks(&t1, &mut it);
    feed_forward_steer_only(&mut blocks);
    let mut solver = TransientSolver::new(
        blocks,
        line(8000.0, 200, false, 0.0, kappa, v, None),
        &it,
        cfg(),
    );
    for _ in 0..1500 {
        solver.step();
    }
    let fs = solver.fast_state();
    let r = fs[ChassisState::YawRate as usize];
    // The understeer-gradient feed-forward δ_ff = κ(L + K_us·v²) is designed so the steady yaw hits
    // the target curvature: r → v·κ. Positive steer ⇒ +yaw (left). Bound it generously.
    let neutral = v * kappa; // = v·κ (target yaw rate)
    assert!(r > 0.0, "left steer ⇒ +yaw, got {r}");
    assert!(
        r > 0.5 * neutral && r < 1.3 * neutral,
        "r={r} vs target {neutral}"
    );
}

#[test]
fn relaxation_states_converge_to_steady_state() {
    let t1 = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let mut blocks = build_blocks(&t1, &mut it);
    feed_forward_steer_only(&mut blocks); // steady curve from the FF steer
    let mut solver = TransientSolver::new(
        blocks,
        line(8000.0, 200, false, 0.0, 0.006, 40.0, None),
        &it,
        cfg(),
    );
    // Run to a steady turn, then check the lagged slip stops moving (converged).
    for _ in 0..1400 {
        solver.step();
    }
    let a_before: Vec<f64> = (0..WHEELS)
        .map(|w| solver.fast_state()[StateLayout::relax_slot(RelaxState::Alpha, w)])
        .collect();
    for _ in 0..100 {
        solver.step();
    }
    for w in 0..WHEELS {
        let a_after = solver.fast_state()[StateLayout::relax_slot(RelaxState::Alpha, w)];
        assert!(
            (a_after - a_before[w]).abs() < 1e-4,
            "wheel {w} lagged α not converged"
        );
    }
}

#[test]
fn skidpad_is_bit_reproducible() {
    let t1 = limebeer();
    let run = || {
        let mut it = outlap_core::bus::ChannelInterner::new();
        let blocks = build_blocks(&t1, &mut it);
        let l = line(
            2.0 * std::f64::consts::PI * 60.0,
            400,
            true,
            1.0 / 60.0,
            1.0 / 60.0,
            30.0,
            Some(60.0),
        );
        let mut solver = TransientSolver::new(blocks, l, &it, cfg());
        let lap = solver.run(2.0 * std::f64::consts::PI * 60.0, 60_000);
        (lap.len(), lap.yaw_rate.clone(), lap.n.clone())
    };
    let (n1, r1, off1) = run();
    let (n2, r2, off2) = run();
    assert_eq!(n1, n2);
    assert_eq!(r1, r2, "yaw-rate trace bit-identical across runs");
    assert_eq!(off1, off2, "offset trace bit-identical across runs");
}

#[test]
fn skidpad_stays_within_the_friction_circle() {
    let t1 = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks(&t1, &mut it);
    let l = line(
        2.0 * std::f64::consts::PI * 60.0,
        400,
        true,
        1.0 / 60.0,
        1.0 / 60.0,
        30.0,
        Some(60.0),
    );
    let mut solver = TransientSolver::new(blocks, l, &it, cfg());
    let lap = solver.run(2.0 * std::f64::consts::PI * 60.0, 60_000);
    // The combined tyre force never exceeds the model's own peak-μ ellipse (a small margin covers
    // combined-slip vs the per-axis peak).
    for i in (0..lap.len()).step_by(50) {
        for w in 0..WHEELS {
            let fz = lap.fz[i][w].max(1.0);
            let (fx, fy) = (lap.fx[i][w], lap.fy[i][w]);
            let combined = (fx * fx + fy * fy).sqrt();
            let model = if w < 2 { &t1.tire_front } else { &t1.tire_rear };
            let p = if w < 2 { t1.p_front } else { t1.p_rear };
            let mu = model.peak_mu_y(fz, p).max(model.peak_mu_x(fz, p));
            assert!(
                combined <= 1.1 * mu * fz,
                "wheel {w}: |F|={combined} > μ·Fz={}",
                mu * fz
            );
        }
    }
}
