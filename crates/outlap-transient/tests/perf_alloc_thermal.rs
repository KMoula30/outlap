// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the **T2 step with the tire-thermal stack wired** (M5 PR3): the per-step
//! heat accumulation and the decimated ring advance must not allocate on the hot loop.
//!
//! Its own test binary because the dhat profiler is process-global (mirrors `perf_alloc.rs`). The
//! stack advances on the slow clock (default every 20 steps), so the warmed run spans several ring
//! advances — none of which may touch the heap.
#![allow(clippy::many_single_char_names)]

mod common;

use std::f64::consts::PI;

use common::{build_blocks, limebeer, line};
use outlap_core::bus::ChannelInterner;
use outlap_schema::sim::FzCoupling;
use outlap_schema::tyr::{TyrThermal, TyrWear};
use outlap_transient::{AxleGeometry, SimConfig, TireThermalStack, TransientSolver};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

fn thermal() -> TyrThermal {
    TyrThermal {
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
    }
}

fn wear() -> TyrWear {
    TyrWear {
        k_w: 0.01,
        w_max: 8.0,
        w_c: 2.0,
        tau_d: 500.0,
        t_deg: 110.0,
        delta_t_ref: 15.0,
        beta: 2.0,
        delta_c: 0.35,
        s_w: 0.35,
        delta_d: 0.25,
    }
}

#[test]
fn t2_step_with_tire_thermal_is_zero_alloc() {
    let _profiler = dhat::Profiler::builder().testing().build();

    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let r = 100.0;
    let ln = line(2.0 * PI * r, 400, true, 1.0 / r, 1.0 / r, 30.0, Some(r));
    let cfg = SimConfig {
        fz_coupling: FzCoupling::FixedPoint,
        ..SimConfig::default()
    };
    let g = AxleGeometry::new(0.33, Some(0.30), Some(250_000.0));
    let (th, wr) = (thermal(), wear());
    let stack = TireThermalStack::new(&th, &wr, &th, &wr, g, g, 30.0, 40.0);
    let mut solver = TransientSolver::new(blocks, ln, &it, cfg).with_tire_thermal(stack);

    // Warm the caches (spans several slow-clock ring advances), then assert no heap block is
    // allocated across a run of steps that includes both accumulation and advances.
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
        "the T2 step with the tire-thermal stack allocated {} heap block(s) — the hot loop must be \
         zero-alloc",
        after.total_blocks - before.total_blocks
    );
}
