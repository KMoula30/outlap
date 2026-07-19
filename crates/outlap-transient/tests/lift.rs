// SPDX-License-Identifier: AGPL-3.0-only
//! M6 PR5A — the `u(s)` lift-and-coast hook (§8.3, D-M6-9). The lift caps the closed-loop driver's
//! tracked speed reference at the scheduled stations, so the car lifts off early and coasts. Two
//! properties matter: an absent / all-`+∞` schedule is byte-identical to the pre-lift path (the
//! `min(v_ref, +∞)` no-op), and a finite lift genuinely pulls the tracked speed down while the
//! closed loop stays stable (the Driver is the loop — a lift lap must complete, not spin).
#![allow(clippy::float_cmp)]

mod common;

use common::{build_blocks, limebeer, line};
use outlap_core::bus::ChannelInterner;
use outlap_core::state::ChassisState;
use outlap_schema::sim::FzCoupling;
use outlap_transient::{LiftSchedule, SimConfig, TransientSolver};

fn cfg() -> SimConfig<f64> {
    SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    }
}

/// A straight 2 km line at a constant 80 m/s reference (no curvature, no grade).
fn straight() -> outlap_transient::LineTable<f64> {
    line(2000.0, 200, false, 0.0, 0.0, 80.0, None)
}

/// Run `steps` and collect the per-step `v_x` bit patterns (the closed-loop speed trajectory).
fn vx_bits(mut solver: TransientSolver<f64>, steps: usize) -> Vec<u64> {
    let mut out = Vec::with_capacity(steps);
    for _ in 0..steps {
        solver.step();
        out.push(solver.fast_state()[ChassisState::Vx as usize].to_bits());
        if solver.diverged() {
            break;
        }
    }
    out
}

#[test]
fn absent_and_infinite_lift_are_bit_identical() {
    let (t1, spec) = limebeer();
    // Baseline: no lift schedule attached.
    let mut it = ChannelInterner::new();
    let base = TransientSolver::new(build_blocks(&t1, &spec, &mut it), straight(), &it, cfg());
    // An all-`+∞` schedule over the same grid: `min(v_ref, +∞)` must be a genuine no-op.
    let mut it2 = ChannelInterner::new();
    let inf = LiftSchedule::new(
        (0..=4).map(|i| f64::from(i) * 500.0).collect(),
        vec![f64::INFINITY; 5],
    );
    let lifted = TransientSolver::new(build_blocks(&t1, &spec, &mut it2), straight(), &it2, cfg())
        .with_lift(inf);
    assert_eq!(
        vx_bits(base, 2000),
        vx_bits(lifted, 2000),
        "an all-+∞ lift schedule must be byte-identical to no schedule"
    );
}

#[test]
fn lift_caps_the_tracked_speed_and_stays_stable() {
    let (t1, spec) = limebeer();
    // Baseline lap at the 80 m/s reference.
    let mut it = ChannelInterner::new();
    let mut base = TransientSolver::new(build_blocks(&t1, &spec, &mut it), straight(), &it, cfg());
    // A lift capping the reference to 45 m/s over the whole line.
    let mut it2 = ChannelInterner::new();
    let lift = LiftSchedule::new(
        (0..=4).map(|i| f64::from(i) * 500.0).collect(),
        vec![45.0; 5],
    );
    let mut lifted =
        TransientSolver::new(build_blocks(&t1, &spec, &mut it2), straight(), &it2, cfg())
            .with_lift(lift);

    let mut base_min = f64::INFINITY;
    let mut lift_min = f64::INFINITY;
    for _ in 0..4000 {
        base.step();
        lifted.step();
        base_min = base_min.min(base.fast_state()[ChassisState::Vx as usize]);
        lift_min = lift_min.min(lifted.fast_state()[ChassisState::Vx as usize]);
        assert!(
            !lifted.diverged(),
            "the lift lap must not spin (closed-loop stability)"
        );
    }
    let vx = lifted.fast_state()[ChassisState::Vx as usize];
    // The baseline holds near 80; the lift pulls the tracked speed down toward the 45 m/s cap.
    assert!(
        base_min > 70.0,
        "baseline holds the 80 m/s reference: min {base_min}"
    );
    assert!(
        vx < 55.0 && vx > 35.0,
        "the lift tracks the 45 m/s cap (settled at {vx})"
    );
    assert!(
        lift_min < base_min - 20.0,
        "the lift banks a large entry-speed reduction: lift {lift_min} vs base {base_min}"
    );
    // Determinism: a re-run reproduces the lift trajectory bit-for-bit.
    let mut it3 = ChannelInterner::new();
    let lift2 = LiftSchedule::new(
        (0..=4).map(|i| f64::from(i) * 500.0).collect(),
        vec![45.0; 5],
    );
    let rerun = TransientSolver::new(build_blocks(&t1, &spec, &mut it3), straight(), &it3, cfg())
        .with_lift(lift2);
    let mut it4 = ChannelInterner::new();
    let lift3 = LiftSchedule::new(
        (0..=4).map(|i| f64::from(i) * 500.0).collect(),
        vec![45.0; 5],
    );
    let rerun_ref =
        TransientSolver::new(build_blocks(&t1, &spec, &mut it4), straight(), &it4, cfg())
            .with_lift(lift3);
    assert_eq!(
        vx_bits(rerun, 1500),
        vx_bits(rerun_ref, 1500),
        "the lift lap is deterministic"
    );
}
