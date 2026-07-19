// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the T3 14-DOF chassis RHS (CLAUDE.md: allocs/step is CI-enforced).
//!
//! `ChassisT3::derivatives` is the T3 integrate-phase block; it must touch no heap. dhat's testing
//! profiler counts heap blocks; we assert the count is unchanged across warmed evaluations — the
//! same pattern as `outlap-core/tests/alloc.rs`.

use outlap_core::block::Block;
use outlap_core::bus::{Bus, ChannelInterner, WheelSignal, WHEELS};
use outlap_core::state::{fast_slot_count, ChassisState, DerivView, StateView};
use outlap_vehicle::{ChassisParams, ChassisT3, RoadChannels, SuspensionParams, T3RoadVertical};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

fn build() -> (ChassisT3<f64>, ChannelInterner) {
    let params = ChassisParams::<f64>::from_f64(
        800.0,
        1000.0,
        [1.7, 1.7, -1.7, -1.7],
        [0.825, -0.825, 0.80, -0.80],
        [true, true, false, false],
        [0.33; WHEELS],
        [1.1; WHEELS],
    );
    let susp = SuspensionParams::<f64> {
        sprung_mass: 740.0,
        ixx: 180.0,
        iyy: 950.0,
        h_s: 0.32,
        h_cg: 0.30,
        h_ra: 0.05,
        wheelbase: 3.4,
        track_f: 1.65,
        track_r: 1.60,
        anti_dive: 0.3,
        anti_squat: 0.2,
        arb_f: 4.0e5,
        arb_r: 3.0e5,
        bumpstop_smooth: 0.005,
        k_ride: [220_000.0, 220_000.0, 240_000.0, 240_000.0],
        static_defl: [0.03; WHEELS],
        damp_bump: [4000.0; WHEELS],
        damp_rebound: [8000.0; WHEELS],
        bumpstop_rate: [5.0e5; WHEELS],
        bumpstop_gap: [0.03; WHEELS],
        k_tyre: [250_000.0; WHEELS],
        c_tyre: [500.0; WHEELS],
        tyre_static_defl: [0.03; WHEELS],
        unsprung_mass: [15.0; WHEELS],
    };
    let mut interner = ChannelInterner::new();
    let road = RoadChannels::intern(&mut interner);
    let road_v = T3RoadVertical::intern(&mut interner);
    (ChassisT3::new(params, susp, road, road_v), interner)
}

#[test]
fn t3_derivatives_do_not_allocate() {
    let _profiler = dhat::Profiler::builder().testing().build();
    let (chassis, interner) = build();
    let mut bus = Bus::<f64>::with_interner(&interner, 1);
    for w in 0..WHEELS {
        bus.set_wheel(WheelSignal::TireFx, w, 0, -1500.0);
        bus.set_wheel(WheelSignal::TireFy, w, 0, 2000.0);
    }
    let mut fast = vec![0.0f64; fast_slot_count()];
    fast[ChassisState::Vx as usize] = 40.0;
    fast[ChassisState::YawRate as usize] = 0.3;
    fast[ChassisState::Heave as usize] = -0.01;
    fast[ChassisState::ZuFl as usize] = 0.02; // engage the bumpstop path
    let mut dfast = vec![0.0f64; fast_slot_count()];

    // Warm up (any lazy one-time alloc happens before the measurement window).
    {
        let sv = StateView::new(&fast, 1, 0);
        let mut dv = DerivView::new(&mut dfast, 1, 0);
        chassis.derivatives(&sv, &mut bus, &mut dv, 0);
    }

    let before = dhat::HeapStats::get().total_blocks;
    for _ in 0..256 {
        let sv = StateView::new(&fast, 1, 0);
        let mut dv = DerivView::new(&mut dfast, 1, 0);
        chassis.derivatives(&sv, &mut bus, &mut dv, 0);
    }
    let after = dhat::HeapStats::get().total_blocks;
    assert_eq!(
        before,
        after,
        "T3 chassis RHS allocated {} heap blocks",
        after - before
    );
}
