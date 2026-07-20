// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the **T3 (14-DOF) step** (PR7): the suspension RHS + the tyre-spring `F_z`
//! block + the dynamic-ride-height aero must not allocate on the hot loop, exactly like T2.
//!
//! Its own test binary because the dhat profiler is process-global (mirrors `perf_alloc.rs`).
#![allow(clippy::many_single_char_names)]

mod common;

use std::f64::consts::PI;

use common::{build_blocks_t3, f1_2026, line};
use outlap_core::bus::ChannelInterner;
use outlap_schema::sim::FzCoupling;
use outlap_transient::{SimConfig, TransientSolver};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

#[test]
fn t3_step_is_zero_alloc() {
    let _profiler = dhat::Profiler::builder().testing().build();

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

    // Warm the caches (spans several slow-clock windows), then assert no heap block is allocated
    // across a run of steps.
    for _ in 0..64 {
        solver.step();
    }
    let before = dhat::HeapStats::get();
    for _ in 0..512 {
        solver.step();
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks, before.total_blocks,
        "the T3 step allocated {} heap block(s) — the hot loop must be zero-alloc",
        after.total_blocks - before.total_blocks
    );
}
