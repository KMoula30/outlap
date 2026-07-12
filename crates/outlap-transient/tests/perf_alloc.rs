// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the **T2 transient step** (CLAUDE.md: allocs/step is CI-enforced).
//!
//! The step loop (sense → control → actuate → integrate, with the exact-exponential relaxation
//! sub-step and the fixed-point `F_z` coupling) runs on the pre-allocated `SimArena` and must not
//! allocate. dhat's testing profiler counts heap blocks; we assert the count is unchanged across a
//! warmed run of steps. Its own test binary because the dhat profiler is process-global (a second
//! `#[test]` here would race it under the parallel runner — the `outlap-qss` alloc idiom).
#![allow(clippy::many_single_char_names)]

mod common;

use std::f64::consts::PI;

use common::{build_blocks, limebeer, line};
use outlap_core::bus::ChannelInterner;
use outlap_schema::sim::FzCoupling;
use outlap_transient::{SimConfig, TransientSolver};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

#[test]
fn t2_step_is_zero_alloc() {
    let _profiler = dhat::Profiler::builder().testing().build();

    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    // A stable closed skidpad (r = 100 m at 30 m/s ≈ 0.9 g): the driver tracks it without spinning,
    // so the step exercises the full closed loop (relaxation + fixed-point) every iteration.
    let r = 100.0;
    let ln = line(2.0 * PI * r, 400, true, 1.0 / r, 1.0 / r, 30.0, Some(r));
    let cfg = SimConfig {
        fz_coupling: FzCoupling::FixedPoint, // the T2 default — the iterative path must stay alloc-free
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(blocks, ln, &it, cfg);

    // Warm the caches, then assert no heap block is allocated across a run of steps.
    for _ in 0..64 {
        solver.step();
    }
    let before = dhat::HeapStats::get();
    for _ in 0..512 {
        solver.step();
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "the T2 step allocated {} heap block(s) — the hot loop must be zero-alloc",
        after.total_blocks - before.total_blocks
    );
}
