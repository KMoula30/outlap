// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the energy-manager hot path (CLAUDE.md: allocs/step is CI-enforced).
//!
//! `decide` + `record` run once per step boundary inside both tiers' hot loops and must not
//! allocate. dhat's testing profiler counts heap blocks; we assert the count is unchanged across
//! a warmed trace (the outlap-qss/outlap-tire alloc-test pattern: one `#[test]`, one profiler).
//!
//! The count is process-global, so a loaded CI runner can land an ambient one-off block inside
//! the window (e.g. the test harness's coordinator thread waking mid-measurement — observed
//! 2026-07-26, +4 blocks over 8192 calls, unreproducible on the same content and toolchain). A
//! stray window therefore REPLAYS: the seed, the ledger and the warm-up are rebuilt so the second
//! window feeds `decide`/`record` the identical call sequence. A genuine per-step allocation
//! reproduces on identical inputs; an ambient artifact does not. Replaying rather than continuing
//! matters — the lap ledger is monotonic, so a continued window would run with the harvest budget
//! already spent and take a different branch mix, which is not the thing under test. The gate
//! stays exact: the deciding window must allocate nothing.

mod common;

use common::{f1_policy, TestRng, F1_PACK_WINDOW};
use outlap_powertrain::{
    DecideInput, DeployPolicy, EnergyManager, ErsCommand, ErsRulebook, LapEnergyLedger, UsSchedule,
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

    let ers = f1_policy();
    let rule_based = EnergyManager::new(
        ErsRulebook::<f64>::from_schema(&ers, F1_PACK_WINDOW, None).unwrap(),
        DeployPolicy::RuleBased,
    );
    let schedule = UsSchedule::new(
        (0..64).map(|i| f64::from(i % 5) / 2.0 - 1.0).collect(),
        (0..64).map(|i| i % 7 == 0).collect(),
        vec![0.0; 64],
        vec![0; 64],
    )
    .unwrap();
    let scheduled = EnergyManager::new(
        ErsRulebook::<f64>::from_schema(&ers, F1_PACK_WINDOW, None).unwrap(),
        DeployPolicy::Schedule(schedule),
    );

    for mgr in [&rule_based, &scheduled] {
        // One measured pass from a FRESH state: same seed, same empty ledger, same warm-up. The
        // state must be rebuilt per pass — carrying it forward would make a second pass a
        // continuation rather than a replay, and the lap ledger is monotonic, so a continued
        // window runs with the harvest budget already spent and exercises a different branch mix.
        let measure = || {
            let mut rng = TestRng::new(0xA110C);
            let mut ledger = LapEnergyLedger::new();
            let mut prev = ErsCommand::idle();
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
            (after.total_blocks - before.total_blocks, ledger)
        };

        // A stray block replays the identical window once — see the module doc.
        let (mut delta, mut ledger) = measure();
        if delta != 0 {
            eprintln!("alloc gate: first window saw {delta} stray block(s); replaying it once");
            let replay = measure();
            delta = replay.0;
            ledger = replay.1;
        }
        assert_eq!(
            delta, 0,
            "decide/record allocated {delta} block(s) on an identical replay"
        );
        ledger.reset();
        #[allow(clippy::float_cmp)] // reset is exact zero by construction
        {
            assert_eq!(ledger.deploy_j(), 0.0);
        }
    }
}
