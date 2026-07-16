// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the energy-manager hot path (CLAUDE.md: allocs/step is CI-enforced).
//!
//! `decide` + `record` run once per step boundary inside both tiers' hot loops and must not
//! allocate. dhat's testing profiler counts heap blocks; we assert the count is unchanged across
//! a warmed trace (the outlap-qss/outlap-tire alloc-test pattern: one `#[test]`, one profiler).

mod common;

use common::{f1_ers, TestRng};
use outlap_powertrain::{
    DecideInput, EnergyManager, ErsCommand, ErsRulebook, LapEnergyLedger, Policy, UsSchedule,
};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

fn step(rng: &mut TestRng, prev: &ErsCommand<f64>) -> DecideInput<f64> {
    let phase = rng.next_f64();
    let (demand, brake) = if phase < 0.3 {
        (0.0, rng.range(50e3, 900e3))
    } else if phase < 0.6 {
        (rng.range(0.05, 1.0), 0.0)
    } else {
        (1.0, 0.0)
    };
    DecideInput {
        v: rng.range(5.0, 100.0),
        driver_demand: demand,
        brake_demand_w: brake,
        mech_regen_envelope_w: rng.range(0.0, 450e3),
        ice_surplus_w: rng.range(0.0, 200e3),
        soc: rng.range(0.0, 1.0),
        override_active: rng.next_f64() < 0.1,
        prev_k_power_w: prev.deploy_w - prev.harvest_w,
        ramp_reduced_w: 0.0,
        dt: 0.02,
        station: (rng.next_u64() % 64) as usize,
    }
}

#[test]
fn decide_and_record_are_zero_alloc() {
    let _profiler = dhat::Profiler::builder().testing().build();

    let ers = f1_ers();
    let rule_based = EnergyManager::new(
        ErsRulebook::<f64>::from_schema(&ers, None).unwrap(),
        Policy::RuleBased,
    );
    let schedule = UsSchedule::new(
        (0..64).map(|i| f64::from(i % 5) / 2.0 - 1.0).collect(),
        (0..64).map(|i| i % 7 == 0).collect(),
        vec![0.0; 64],
        vec![0; 64],
    )
    .unwrap();
    let scheduled = EnergyManager::new(
        ErsRulebook::<f64>::from_schema(&ers, None).unwrap(),
        Policy::Schedule(schedule),
    );

    for mgr in [&rule_based, &scheduled] {
        let mut rng = TestRng::new(0xA110C);
        let mut ledger = LapEnergyLedger::new();
        let mut prev = ErsCommand::idle();
        // Warm.
        for _ in 0..64 {
            let inp = step(&mut rng, &prev);
            prev = mgr.decide(&inp, &ledger);
            ledger.record(&prev, 0.02);
        }
        let before = dhat::HeapStats::get();
        for _ in 0..4096 {
            let inp = step(&mut rng, &prev);
            prev = mgr.decide(&inp, &ledger);
            ledger.record(&prev, 0.02);
        }
        let after = dhat::HeapStats::get();
        assert_eq!(
            after.total_blocks,
            before.total_blocks,
            "decide/record allocated {} block(s)",
            after.total_blocks - before.total_blocks
        );
        ledger.reset();
        #[allow(clippy::float_cmp)] // reset is exact zero by construction
        {
            assert_eq!(ledger.deploy_j(), 0.0);
        }
    }
}
