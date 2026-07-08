// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the scaffolding hot loop: bus/state access and one fixed-step RK step
//! (HANDOFF §11.5, CLAUDE.md alloc=0/step). dhat's profiler is process-global, so this lives in its
//! own test binary (one profiler per binary — the project idiom).
//!
//! Construction (`Bus::new`, `SimArena::new`, the `SoA` buffers) may allocate; the assertion window
//! opens only after every path is warmed.

#![allow(clippy::many_single_char_names)]

use outlap_core::{
    core_channel_count, fast_slot_count, Bus, ChassisState, CoreSignal, DerivView, RkMethod,
    SimArena, StateView, WheelSignal,
};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

#[test]
fn hot_loop_access_does_not_allocate() {
    let _profiler = dhat::Profiler::builder().testing().build();

    let batch = 4;
    let mut bus = Bus::<f64>::new(core_channel_count(), batch);
    let fast_len = fast_slot_count() * batch;
    let fast = vec![0.0f64; fast_len];
    let mut dfast = vec![0.0f64; fast_len];
    let mut arena = SimArena::for_method(RkMethod::Heun, 6);
    let mut state6 = [1.0f64, 0.0, 0.0, 40.0, 0.0, 0.0];

    // Warm the paths so any first-touch lazy allocation happens before the window.
    bus.set(CoreSignal::Steer, 0, 0.1);
    bus.set_wheel(WheelSignal::TireFz, 0, 0, 3000.0);
    let _ = StateView::new(&fast, batch, 0).chassis(ChassisState::Vx);
    DerivView::new(&mut dfast, batch, 0).set_chassis(ChassisState::Vx, 1.0);
    arena.step(&mut state6, 0.0, 1e-3, |_t, x, dx| dx[0] = -x[0]);

    let before = dhat::HeapStats::get().total_blocks;
    let mut sink = 0.0f64;
    for lane in 0..batch {
        for k in 0..64u32 {
            let v = f64::from(k);
            bus.set(CoreSignal::Throttle, lane, v);
            bus.set_wheel(WheelSignal::SlipKappa, lane % 4, lane, v * 0.01);
            let mut dx = DerivView::new(&mut dfast, batch, lane);
            dx.set_chassis(ChassisState::YawRate, v);
            let x = StateView::new(&fast, batch, lane);
            sink += bus.get(CoreSignal::Throttle, lane)
                + bus.get_wheel(WheelSignal::SlipKappa, lane % 4, lane)
                + x.chassis(ChassisState::S);
        }
    }
    for _ in 0..64 {
        arena.step(&mut state6, 0.0, 1e-3, |_t, x, dx| {
            dx[0] = x[3];
            dx[3] = -0.1 * x[3];
        });
    }
    sink += state6.iter().sum::<f64>();

    assert_eq!(
        before,
        dhat::HeapStats::get().total_blocks,
        "bus/state access or RK step allocated on the heap"
    );
    assert!(sink.is_finite());
}
