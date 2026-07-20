// SPDX-License-Identifier: AGPL-3.0-only
//! Throughput floor for the **T2 transient step** (Decision #15, HANDOFF §11.5).
//!
//! Release-only wall-clock median idiom (the `outlap-qss` `catalunya` timing template): warm, then
//! time a long run of `step()`, **record** the measured steps/s, and assert a regression tripwire.
//! Debug builds are far too slow to gate, so the test is `cfg(not(debug_assertions))` and the CI
//! wires it into the release test line. Runs in its own binary (no dhat allocator, which would skew
//! the timing) on a stable closed skidpad so the loop never diverges out early.
//!
//! **Recorded, not the Decision #15 250k floor.** The T2 step is bound by the RHS cost: at MF6.1
//! tyre fidelity a single right-hand-side evaluation is ~2.3 µs/core (four full Pacejka evaluations
//! for the four contact patches, plus the 7-DOF chassis). The step evaluates the RHS
//! `2 + coupling` times — Heun's two RK stages plus the algebraic `F_z` coupling (4 more for the
//! `fixed_point/3` default, 2 for `one_step_lag`) — so the throughput is `1 / (evals · 2.3 µs)`:
//! ~62k steps/s at the fixed-point default, ~108k at one-step-lag. Even the *theoretical* ceiling
//! (Heun's two evals, zero coupling) is `1 / (2 · 2.3 µs) ≈ 217k`, below the 250k floor. Reaching
//! 250k would require cheapening the tyre model (a fidelity cut, out of scope) — so we **record**
//! the honest number and gate a 2× regression tripwire; the 250k floor is decomposed and deferred
//! (see the PR's parity/perf decomposition). One line-lookup-sharing optimisation (`road_sample`)
//! bought ~+13% and is kept.
#![allow(clippy::many_single_char_names, clippy::cast_precision_loss)]

mod common;

#[cfg(not(debug_assertions))]
#[test]
fn t2_steps_per_second_floor() {
    use std::f64::consts::PI;
    use std::time::Instant;

    use common::{build_blocks, limebeer, line};
    use outlap_core::bus::ChannelInterner;
    use outlap_schema::sim::FzCoupling;
    use outlap_transient::{SimConfig, TransientSolver};

    // Decision #15's target is 250k/core; at MF6.1 tyre fidelity the RHS cost caps this tier well
    // below that (see the module docs). We record the measured value and gate a regression tripwire
    // at ~half the fixed-point-default throughput — a real slowdown trips it, the fidelity-bound
    // ceiling does not.
    const TARGET: f64 = 250_000.0; // Decision #15 (not met at this fidelity — recorded, deferred).
    const TRIPWIRE: f64 = 30_000.0; // regression guard (default measures ~62k/core here).
    const STEPS: usize = 200_000;

    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let r = 100.0;
    let ln = line(2.0 * PI * r, 400, true, 1.0 / r, 1.0 / r, 30.0, Some(r));
    let cfg = SimConfig {
        fz_coupling: FzCoupling::FixedPoint,
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(blocks, ln, &it, cfg);

    for _ in 0..2_000 {
        solver.step(); // warm
    }
    // Best (min time) of a few runs — the standard release-timing idiom.
    let mut best = f64::INFINITY;
    for _ in 0..3 {
        let t = Instant::now();
        for _ in 0..STEPS {
            solver.step();
        }
        best = best.min(t.elapsed().as_secs_f64());
    }
    let steps_per_s = STEPS as f64 / best;
    println!(
        "T2 step throughput: {steps_per_s:.0} steps/s/core \
         (Decision #15 target {TARGET:.0} — RHS-bound at MF6.1 fidelity, recorded; \
         regression tripwire {TRIPWIRE:.0})"
    );
    assert!(
        steps_per_s >= TRIPWIRE,
        "T2 throughput {steps_per_s:.0} steps/s/core fell below the {TRIPWIRE:.0} regression \
         tripwire — a real slowdown, not the fidelity-bound ceiling"
    );
}

/// Records the throughput with the M5 tire-thermal stack wired (per-step heat accumulation + the
/// decimated ring advance). The accumulation is a handful of flops per wheel off the RK path and the
/// ring advances once per slow window, so the cost is a few % over the frozen path — well inside the
/// same tripwire. Recorded so the PR carries the honest stack-on number.
#[cfg(not(debug_assertions))]
#[test]
fn t2_steps_per_second_with_tire_thermal() {
    use std::f64::consts::PI;
    use std::time::Instant;

    use common::{build_blocks, limebeer, line};
    use outlap_core::bus::ChannelInterner;
    use outlap_schema::sim::FzCoupling;
    use outlap_schema::tyr::{TyrThermal, TyrWear};
    use outlap_transient::{AxleGeometry, SimConfig, TireThermalStack, TransientSolver};

    const TRIPWIRE: f64 = 30_000.0;
    const STEPS: usize = 200_000;

    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let r = 100.0;
    let ln = line(2.0 * PI * r, 400, true, 1.0 / r, 1.0 / r, 30.0, Some(r));
    let cfg = SimConfig {
        fz_coupling: FzCoupling::FixedPoint,
        ..SimConfig::default()
    };
    let th = TyrThermal {
        c_s: 1500.0,
        c_c: 3500.0,
        c_g: 700.0,
        g_sc: 180.0,
        g_cg: 70.0,
        g_road: 180.0,
        h0: 8.0,
        h1: 3.0,
        p_t: 0.65,
        t_opt: 95.0,
        c_t: 2.2,
        k_c: 0.001,
        t_c_ref: 80.0,
        p_cold: 138.0,
        t_cold: 20.0,
    };
    let wr = TyrWear {
        k_w: 1.0e-6,
        w_max: 8.0,
        w_c: 2.0,
        tau_d: 500.0,
        t_deg: 110.0,
        delta_t_ref: 15.0,
        beta: 2.0,
        delta_c: 0.35,
        s_w: 0.35,
        delta_d: 0.25,
    };
    let g = AxleGeometry::new(0.33, Some(0.30), Some(250_000.0));
    let stack = TireThermalStack::new(&th, &wr, &th, &wr, g, g, 30.0, 40.0);
    let mut solver = TransientSolver::new(blocks, ln, &it, cfg).with_tire_thermal(stack);

    for _ in 0..2_000 {
        solver.step(); // warm
    }
    let mut best = f64::INFINITY;
    for _ in 0..3 {
        let t = Instant::now();
        for _ in 0..STEPS {
            solver.step();
        }
        best = best.min(t.elapsed().as_secs_f64());
    }
    let steps_per_s = STEPS as f64 / best;
    println!(
        "T2 step throughput WITH tire-thermal stack: {steps_per_s:.0} steps/s/core \
         (regression tripwire {TRIPWIRE:.0})"
    );
    assert!(
        steps_per_s >= TRIPWIRE,
        "T2+tire-thermal throughput {steps_per_s:.0} steps/s/core fell below the {TRIPWIRE:.0} \
         tripwire"
    );
}

/// Throughput floor for the **T3 (14-DOF) step** (PR7). T3 gets its OWN tripwire, set at ~half the
/// first honest T3 measurement (exactly how T2's 30k was derived from its ~62k — user-locked). The
/// T3 RHS is heavier (24 states + suspension) but resolves `F_z` from the state in ONE eval per RK
/// stage (no algebraic Picard coupling), so its per-step cost is comparable to T2's. T2's 30k floor
/// is untouched.
#[cfg(not(debug_assertions))]
#[test]
fn t3_steps_per_second_floor() {
    use std::f64::consts::PI;
    use std::time::Instant;

    use common::{build_blocks_t3, f1_2026, line};
    use outlap_core::bus::ChannelInterner;
    use outlap_schema::sim::FzCoupling;
    use outlap_transient::{SimConfig, TransientSolver};

    // First honest T3 measurement was ~96k steps/s/core (FASTER than T2's ~62k: the T3 tyre-spring
    // F_z resolves from the state in ONE eval per RK stage, where T2 runs 3 extra Picard evals for
    // the algebraic coupling — the heavier 24-DOF RHS is more than paid for). The tripwire is ~half
    // that (with margin for slower CI machines), the T2 methodology.
    const TRIPWIRE: f64 = 40_000.0;
    const STEPS: usize = 200_000;

    let (t1, spec) = f1_2026();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks_t3(&t1, &spec, &mut it);
    let r = 100.0;
    let ln = line(2.0 * PI * r, 400, true, 1.0 / r, 1.0 / r, 30.0, Some(r));
    let cfg = SimConfig {
        fz_coupling: FzCoupling::FixedPoint,
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(blocks, ln, &it, cfg);

    for _ in 0..2_000 {
        solver.step(); // warm
    }
    let mut best = f64::INFINITY;
    for _ in 0..3 {
        let t = Instant::now();
        for _ in 0..STEPS {
            solver.step();
        }
        best = best.min(t.elapsed().as_secs_f64());
    }
    let steps_per_s = STEPS as f64 / best;
    println!("T3 (14-DOF) step throughput: {steps_per_s:.0} steps/s/core (tripwire {TRIPWIRE:.0})");
    assert!(
        steps_per_s >= TRIPWIRE,
        "T3 throughput {steps_per_s:.0} steps/s/core fell below the {TRIPWIRE:.0} regression tripwire"
    );
}
