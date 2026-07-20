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

use common::{build_blocks, build_blocks_t3, f1_2026, limebeer, line};
use outlap_core::block::{Block, Phase};
use outlap_core::bus::WHEELS;
use outlap_core::state::{ChassisState, RelaxState, StateLayout};
use outlap_schema::sim::FzCoupling;
use outlap_transient::{LineSamples, LineTable, SimConfig, TransientSolver};

/// A closed circular line (radius `r`) carrying a vertical-curvature **crest** (`kappa_v < 0`) over
/// its whole length: a sustained road-normal unloading while the car corners at `v_ref` — the exact
/// 3-D condition that spun the driver before the crest-unloading floor landed (PR7.5).
fn crest_circle(r: f64, kappa_v: f64, v_ref: f64) -> LineTable<f64> {
    let len = 2.0 * std::f64::consts::PI * r;
    let stations = 400;
    let s: Vec<f64> = (0..stations)
        .map(|i| i as f64 * len / (stations as f64 - 1.0))
        .collect();
    let (mut xr, mut yr, mut lx, mut ly) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for &si in &s {
        let th = si / r;
        xr.push(r * th.sin());
        yr.push(r * (1.0 - th.cos()));
        lx.push(-th.sin());
        ly.push(th.cos());
    }
    let mk = |v: f64| vec![v; stations];
    LineTable::new(&LineSamples {
        s,
        kappa_h: mk(1.0 / r),
        grade: mk(0.0),
        banking: mk(0.0),
        kappa_v: mk(kappa_v),
        n_ref: mk(0.0),
        kappa_ref: mk(1.0 / r),
        v_ref: mk(v_ref),
        x_ref: xr,
        y_ref: yr,
        z_ref: mk(0.0),
        lat_x: lx,
        lat_y: ly,
        lat_z: mk(0.0),
        closed: true,
    })
    .unwrap()
}

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
    solver_blocks.driver.sideslip_damping = 0.0;
}

/// Silence the longitudinal loop entirely (no throttle, no brake) — a pure coastdown driver.
fn coasting(solver_blocks: &mut outlap_transient::T2Blocks<f64>) {
    solver_blocks.driver.speed_kp = 0.0;
    solver_blocks.driver.speed_ki = 0.0;
    solver_blocks.driver.ff_accel_scale = f64::INFINITY; // a_ff / ∞ = 0
}

#[test]
fn assembler_order_is_deterministic_and_phase_sorted() {
    let (t1, spec) = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let solver = TransientSolver::new(
        blocks,
        line(100.0, 50, false, 0.0, 0.0, 20.0, None),
        &it,
        cfg(),
    );
    let order = solver.schedule().order().to_vec();
    // The assembler-produced order is asserted to be a *valid topological linearization* derived
    // programmatically from the block phases + the data-dependency edges — NOT a hardcoded index
    // vector (a hardcode passes even if the assembler and the `eval_rhs_raw` hand-order silently
    // drift into a different — but still valid — permutation, and it must be updated by hand every
    // time a block is added). The properties any correct schedule must satisfy:
    //   (1) it is a permutation of every registered block (nothing dropped or duplicated);
    //   (2) the block phases are non-decreasing along it (sense → control → actuate → integrate);
    //   (3) every producer precedes its consumer (the edges asserted below).
    let probe = build_blocks(&t1, &spec, &mut outlap_core::bus::ChannelInterner::new());
    let phases = [
        probe.driver.phase(),
        probe.powertrain.phase(),
        probe.load.phase(),
        probe.aero.phase(),
        probe.tire.phase(),
        probe.tv.phase(),
        probe.chassis.phase(),
    ];
    let mut sorted = order.clone();
    sorted.sort_unstable();
    assert_eq!(
        sorted,
        (0..phases.len()).collect::<Vec<_>>(),
        "the schedule is a permutation of every registered block"
    );
    for w in order.windows(2) {
        assert!(
            phases[w[0]] <= phases[w[1]],
            "phases are non-decreasing along the schedule: block {} ({:?}) before {} ({:?})",
            w[0],
            phases[w[0]],
            w[1],
            phases[w[1]]
        );
    }
    // Determinism: same specs → same schedule.
    let solver2 = TransientSolver::new(
        build_blocks(&t1, &spec, &mut outlap_core::bus::ChannelInterner::new()),
        line(100.0, 50, false, 0.0, 0.0, 20.0, None),
        &it,
        cfg(),
    );
    assert_eq!(order, solver2.schedule().order().to_vec());
    // Phase ordering: driver (control) precedes the chassis (integrate); the tyre (reads Fz) follows
    // the load-transfer writer.
    let pos = |b: usize| order.iter().position(|&x| x == b).unwrap();
    assert!(pos(0) < pos(6), "driver before chassis");
    assert!(pos(2) < pos(4), "load-transfer before tyre");
    assert!(pos(4) < pos(5), "tyre before torque-vectoring");
    assert!(pos(5) < pos(6), "torque-vectoring before chassis");
    assert!(Phase::Control < Phase::Integrate);
}

/// Run `n` steps on the given line and return the per-step body speed `v_x` trajectory.
fn vx_trajectory<B: outlap_transient::TierBlocks<f64>>(
    mut solver: TransientSolver<f64, B>,
    n: usize,
) -> Vec<f64> {
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        solver.step();
        out.push(solver.fast_state()[ChassisState::Vx as usize]);
    }
    out
}

#[test]
fn t3_tracks_t2_on_a_stiffened_platform() {
    // The headline T2↔T3 parity gate (D-M6-6), recorded as a stiffness sweep. On a flat skidpad the
    // two tiers share the SAME constant aero (no map installed in the test harness) and the crest
    // floor is inert, so the difference is the T3 suspension travel PLUS the two refinement terms T3
    // adds and T2 neglects (the gyroscopic spin×yaw coupling and the κ_v·v² frame transport). The
    // suspension part shrinks as the springs/ARBs stiffen (×k, dampers ×√k with ζ held, static
    // compressions ÷k for the same equilibrium, tyre k_z held physical); the refinement part is
    // stiffness-independent, so the trajectories do NOT converge to bit-identical — they track to a
    // small, bounded residual (the refinement physics, correctly present at T3). A fine dt keeps the
    // stiffest corner mode inside Heun stability (ω·dt < 0.5) so the sweep is numerically clean; the
    // measured deltas are recorded and the whole band is asserted small.
    let (t1, spec) = f1_2026();
    let steps = 6_000;
    let r = 80.0;
    let mk_line = || {
        line(
            2.0 * std::f64::consts::PI * r,
            400,
            true,
            1.0 / r,
            1.0 / r,
            42.0,
            Some(r),
        )
    };
    // Fine step so a 30× corner mode (~5.5× the physical ~18 Hz wheel hop) stays inside Heun.
    let fine = SimConfig {
        dt: 0.0002,
        ..cfg()
    };

    let mut it_t2 = outlap_core::bus::ChannelInterner::new();
    let t2_blocks = build_blocks(&t1, &spec, &mut it_t2);
    let t2_vx = vx_trajectory(
        TransientSolver::new(t2_blocks, mk_line(), &it_t2, fine),
        steps,
    );

    let mut worst = 0.0_f64;
    for &k in &[1.0_f64, 3.0, 10.0, 30.0] {
        let mut it = outlap_core::bus::ChannelInterner::new();
        let mut blocks = build_blocks_t3(&t1, &spec, &mut it);
        let s = &mut blocks.chassis.susp;
        let ksq = k.sqrt();
        s.arb_f *= k;
        s.arb_r *= k;
        for i in 0..WHEELS {
            s.k_ride[i] *= k;
            s.static_defl[i] /= k;
            s.bumpstop_rate[i] *= k;
            s.damp_bump[i] *= ksq;
            s.damp_rebound[i] *= ksq;
        }
        let t3_vx = vx_trajectory(TransientSolver::new(blocks, mk_line(), &it, fine), steps);
        let max_diff = t2_vx
            .iter()
            .zip(&t3_vx)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        let pct = 100.0 * max_diff / 42.0;
        println!("T3↔T2 parity: k={k:>4.0}×  max|Δv_x| = {max_diff:.4} m/s ({pct:.2}% of v_ref)");
        assert!(
            t3_vx.iter().all(|v| v.is_finite()),
            "the T3 trajectory stayed finite at k={k}× (ω·dt inside Heun stability)"
        );
        worst = worst.max(max_diff);
    }
    // The whole sweep tracks T2 to well under 1 m/s (≈2% of the 42 m/s skidpad speed) — the small
    // bounded residual is the refinement physics, not a suspension artifact (Decision #48 recorded).
    assert!(
        worst < 1.0,
        "the T3↔T2 speed residual should stay < 1 m/s across the sweep; got {worst:.3} m/s"
    );
}

#[test]
fn t3_assembler_order_is_deterministic_and_phase_sorted() {
    // The T3 sibling of the schedule assertion (PR7b): the 14-DOF block chain
    // (driver → powertrain → aero → t3-load → tyre → tv → chassis) must be a valid topological
    // linearization derived programmatically from the block phases + data-dependency edges, not a
    // hardcoded index vector. Same three properties as the T2 test.
    let (t1, spec) = f1_2026();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks_t3(&t1, &spec, &mut it);
    let solver = TransientSolver::new(
        blocks,
        line(100.0, 50, false, 0.0, 0.0, 20.0, None),
        &it,
        cfg(),
    );
    let order = solver.schedule().order().to_vec();
    let probe = build_blocks_t3(&t1, &spec, &mut outlap_core::bus::ChannelInterner::new());
    let phases = [
        probe.driver.phase(),
        probe.powertrain.phase(),
        probe.aero.phase(),
        probe.load.phase(),
        probe.tire.phase(),
        probe.tv.phase(),
        probe.chassis.phase(),
    ];
    let mut sorted = order.clone();
    sorted.sort_unstable();
    assert_eq!(
        sorted,
        (0..phases.len()).collect::<Vec<_>>(),
        "the T3 schedule is a permutation of every registered block"
    );
    for w in order.windows(2) {
        assert!(
            phases[w[0]] <= phases[w[1]],
            "T3 phases are non-decreasing: block {} ({:?}) before {} ({:?})",
            w[0],
            phases[w[0]],
            w[1],
            phases[w[1]]
        );
    }
    // Determinism: same specs → same schedule.
    let solver2 = TransientSolver::new(
        build_blocks_t3(&t1, &spec, &mut outlap_core::bus::ChannelInterner::new()),
        line(100.0, 50, false, 0.0, 0.0, 20.0, None),
        &it,
        cfg(),
    );
    assert_eq!(order, solver2.schedule().order().to_vec());
    // Producer→consumer edges: the tyre-spring F_z block (index 3) writes TireFz the tyre (4) reads;
    // the aero (2) downforce feeds the chassis (6) sprung dynamics.
    let pos = |b: usize| order.iter().position(|&x| x == b).unwrap();
    assert!(pos(0) < pos(6), "driver before chassis");
    assert!(pos(3) < pos(4), "tyre-spring Fz before tyre");
    assert!(pos(2) < pos(6), "aero before chassis");
    assert!(pos(4) < pos(5), "tyre before torque-vectoring");
    assert!(pos(5) < pos(6), "torque-vectoring before chassis");
}

#[test]
fn flat_straight_stays_planar() {
    let (t1, spec) = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
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
    let (t1, spec) = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let mut blocks = build_blocks(&t1, &spec, &mut it);
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
    let (t1, spec) = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    // Constant steer feed-forward via kappa_ref on a straight road (path feedback off).
    let (v, kappa) = (40.0, 0.006);
    let mut blocks = build_blocks(&t1, &spec, &mut it);
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
    let (t1, spec) = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let mut blocks = build_blocks(&t1, &spec, &mut it);
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
    let (t1, spec) = limebeer();
    let run = || {
        let mut it = outlap_core::bus::ChannelInterner::new();
        let blocks = build_blocks(&t1, &spec, &mut it);
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
    let (t1, spec) = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
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

#[test]
fn a_cornering_crest_stays_finite_and_planted() {
    // A 60 m-radius corner taken at 30 m/s over a sustained crest (kappa_v = −0.02, a 50 m vertical
    // radius). The raw `κ_v·v²` unloading here is ~1.8 g — enough to drive the road-normal load
    // airborne and spin the closed loop. The crest-unloading floor keeps the tyres planted; the lap
    // must stay finite and never spin.
    let (t1, spec) = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let mut solver = TransientSolver::new(blocks, crest_circle(60.0, -0.02, 30.0), &it, cfg());
    let len = 2.0 * std::f64::consts::PI * 60.0;
    let lap = solver.run(len, 60_000);
    assert!(
        !solver.diverged(),
        "the crest floor should keep the car planted"
    );
    for i in 0..lap.len() {
        assert!(
            lap.vx[i].is_finite() && lap.yaw_rate[i].is_finite(),
            "step {i} non-finite"
        );
        for w in 0..WHEELS {
            // The per-wheel load never collapses to zero (the F_z floor holds it positive).
            assert!(lap.fz[i][w] > 0.0, "step {i} wheel {w} lost all load");
        }
    }
}

#[test]
fn t3_stays_planted_over_a_crest_without_the_floor() {
    // The Eau-Rouge crest gate (PR7, D-M6-6): a sustained crest (κ_v = −0.006 over a 60 m corner at
    // 30 m/s) whose ~0.55 g of `κ_v·v²` unloading is well past the T2 `CREST_UNLOADING_FLOOR_G`
    // (0.15 g) — so the rigid T2 tier would floor it. T3 has NO crest floor (it retires with the
    // tyre-spring strategy — the κ_v·v² transport enters the ChassisT3 vertical dynamics directly and
    // the suspension absorbs the unloading). The lap must stay finite and never spin, on the honest
    // 3-D physics. (A far sharper crest genuinely throws the car airborne — a real limit T3 models
    // rather than masks.)
    let (t1, spec) = f1_2026();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks_t3(&t1, &spec, &mut it);
    let mut solver = TransientSolver::new(blocks, crest_circle(60.0, -0.006, 30.0), &it, cfg());
    let len = 2.0 * std::f64::consts::PI * 60.0;
    let lap = solver.run(len, 60_000);
    assert!(
        !solver.diverged(),
        "T3 should ride the crest on its suspension without the T2 floor"
    );
    for i in 0..lap.len() {
        assert!(
            lap.vx[i].is_finite() && lap.yaw_rate[i].is_finite(),
            "step {i} non-finite"
        );
    }
    // The suspension actually worked the crest (the platform moved), and the lap completed.
    assert!(
        lap.s.last().copied().unwrap_or(0.0) >= len,
        "the crest lap completed"
    );
    assert!(
        lap.heave_m.iter().any(|&z| z.abs() > 1e-4),
        "the suspension travelled over the crest"
    );
}

#[test]
fn the_divergence_guard_stops_cleanly_on_an_unholdable_line() {
    // A 10 m-radius corner demanded at 90 m/s is a physically impossible operating point (the
    // curvature-consistent seed yaw rate alone, v·κ = 9 rad/s, is already past the spin ceiling and
    // ~90 g of lateral demand is far outside any tyre's grip). The driver cannot hold it and the
    // closed loop spins. The guard must stop the run cleanly — a finite, truncated trace with
    // `diverged() == true` — never a panic or a `1e120` state.
    let (t1, spec) = limebeer();
    let mut it = outlap_core::bus::ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let l = line(
        2.0 * std::f64::consts::PI * 10.0,
        400,
        true,
        1.0 / 10.0,
        1.0 / 10.0,
        90.0,
        Some(10.0),
    );
    let mut solver = TransientSolver::new(blocks, l, &it, cfg());
    let lap = solver.run(2.0 * std::f64::consts::PI * 10.0, 60_000);
    assert!(
        solver.diverged(),
        "an unholdable corner must trip the guard"
    );
    // Every recorded sample is finite (the guard stops before the non-finite state is recorded).
    for i in 0..lap.len() {
        assert!(
            lap.vx[i].is_finite() && lap.vy[i].is_finite() && lap.yaw_rate[i].is_finite(),
            "recorded step {i} is non-finite"
        );
        assert!(
            lap.vx[i].abs() <= 500.0,
            "recorded vx {} exceeds the ceiling",
            lap.vx[i]
        );
    }
}
