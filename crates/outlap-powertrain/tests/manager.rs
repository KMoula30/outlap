// SPDX-License-Identifier: AGPL-3.0-only
//! Manager property tests: budgets are never exceeded over random demand traces, the ledger
//! integrates ELECTRICAL power and closes exactly, the C5.12 ramp is respected, a null deploy
//! budget stays unenforced, the u(s) schedule reproduces a hand-computed trace, and the whole
//! stack is deterministic (bit-identical reruns) and f32/f64-consistent.
// Bit-exactness IS the assertion in the closure/determinism/hand-computed-trace tests.
#![allow(
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::unusual_byte_groupings
)]

mod common;

use common::{f1_ers, gt_ers, TestRng};
use outlap_powertrain::{
    DecideInput, EnergyManager, ErsCommand, ErsMode, ErsRulebook, LapEnergyLedger, Policy,
    UsSchedule,
};

const DT: f64 = 0.02;

/// One random step of a demand trace: braking / part throttle / full throttle / coast.
fn random_input(rng: &mut TestRng, v: f64, soc: f64, prev: &ErsCommand<f64>) -> DecideInput<f64> {
    let phase = rng.next_f64();
    let (demand, brake) = if phase < 0.3 {
        (0.0, rng.range(50e3, 900e3)) // braking
    } else if phase < 0.5 {
        (rng.range(0.05, 0.95), 0.0) // part throttle
    } else if phase < 0.9 {
        (1.0, 0.0) // full throttle
    } else {
        (0.0, 0.0) // coast
    };
    DecideInput {
        v,
        driver_demand: demand,
        brake_demand_w: brake,
        mech_regen_envelope_w: rng.range(0.0, 450e3),
        ice_surplus_w: rng.range(0.0, 200e3),
        soc,
        override_active: rng.next_f64() < 0.1,
        prev_k_power_w: prev.deploy_w - prev.harvest_w,
        ramp_reduced_w: 0.0,
        dt: DT,
        station: 0,
    }
}

/// Drive `n` random steps, recording into a ledger; returns (commands, ledger).
fn run_trace(
    mgr: &EnergyManager<f64>,
    seed: u64,
    n: usize,
) -> (Vec<ErsCommand<f64>>, LapEnergyLedger<f64>) {
    let mut rng = TestRng::new(seed);
    let mut ledger = LapEnergyLedger::new();
    let mut prev = ErsCommand::idle();
    let mut cmds = Vec::with_capacity(n);
    for _ in 0..n {
        let v = rng.range(10.0, 100.0);
        let soc = rng.range(0.2, 0.9);
        let inp = random_input(&mut rng, v, soc, &prev);
        let cmd = mgr.decide(&inp, &ledger);
        ledger.record(&cmd, DT);
        prev = cmd;
        cmds.push(cmd);
    }
    (cmds, ledger)
}

/// Property: over arbitrary demand traces the ledger NEVER exceeds a budget, the deploy command
/// never exceeds the taper curve, and every command power is non-negative — for both the f1 2026
/// block and the gt_hybrid fixture block (D-M6-12).
#[test]
fn budgets_are_never_exceeded() {
    for (ers, name) in [(f1_ers(), "f1_2026"), (gt_ers(), "gt_hybrid")] {
        let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&ers, None).unwrap();
        let mgr = EnergyManager::new(rb, Policy::RuleBased);
        for seed in 0..8u64 {
            let mut rng = TestRng::new(0xB0D6_E750 ^ seed);
            let mut ledger = LapEnergyLedger::new();
            let mut prev = ErsCommand::idle();
            let mut any_override = false;
            for _ in 0..20_000 {
                let v = rng.range(0.0, 110.0);
                let soc = rng.range(0.0, 1.0);
                let inp = random_input(&mut rng, v, soc, &prev);
                let cmd = mgr.decide(&inp, &ledger);
                assert!(
                    cmd.deploy_w >= 0.0 && cmd.harvest_w >= 0.0,
                    "{name}: negative power"
                );
                let curve = mgr
                    .rulebook()
                    .deploy_cap_electrical_w(inp.v, inp.override_active);
                assert!(
                    cmd.deploy_w <= curve + 1e-9,
                    "{name}: deploy {} exceeds the curve {} at v={}",
                    cmd.deploy_w,
                    curve,
                    inp.v
                );
                ledger.record(&cmd, DT);
                any_override |= inp.override_active;
                // The +0.5 MJ bonus applies on laps where the flag was set (D-M6-5); with the
                // flag toggling mid-trace the honest bound is the bonus-inclusive budget once any
                // override step has occurred.
                let harvest_budget = mgr.rulebook().harvest_budget_j(any_override);
                assert!(
                    ledger.harvest_j() <= harvest_budget + 1e-6,
                    "{name}: harvest ledger {} exceeds budget {harvest_budget}",
                    ledger.harvest_j()
                );
                if let Some(deploy_budget) = mgr.rulebook().per_lap_deploy_j() {
                    assert!(
                        ledger.deploy_j() <= deploy_budget + 1e-6,
                        "{name}: deploy budget"
                    );
                }
                prev = cmd;
            }
        }
    }
}

/// A null deploy budget stays unenforced: with no `per_lap_deploy_mj` the manager deploys the
/// full curve even after integrating far more energy than the store could hold — the estimation
/// heuristic that back-filled it with `capacity_mj` is dead (D-M6-5).
#[test]
fn a_null_deploy_budget_is_unenforced() {
    let ers = f1_ers();
    assert!(ers.deployment.per_lap_deploy_mj.is_none());
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&ers, None).unwrap();
    assert!(rb.per_lap_deploy_j().is_none());
    let mgr = EnergyManager::new(rb, Policy::RuleBased);
    let mut ledger = LapEnergyLedger::new();
    let inp = DecideInput {
        v: 50.0,
        driver_demand: 1.0,
        brake_demand_w: 0.0,
        mech_regen_envelope_w: 0.0,
        ice_surplus_w: 0.0,
        soc: 0.9, // above target: no recharge, pure deploy
        override_active: false,
        prev_k_power_w: 350e3,
        ramp_reduced_w: 0.0,
        dt: DT,
        station: 0,
    };
    // Integrate 60 s of full deployment ≈ 21 MJ >> the 4 MJ usable window.
    for _ in 0..3000 {
        let cmd = mgr.decide(&inp, &ledger);
        assert_eq!(
            cmd.deploy_w, 350e3,
            "full curve regardless of energy already deployed"
        );
        ledger.record(&cmd, DT);
    }
    assert!(ledger.deploy_j() > 20e6);
}

/// With an explicit deploy budget (the gt_hybrid pattern extended), deployment stops exactly at
/// the budget — the ledger can never exceed it by construction.
#[test]
fn an_explicit_deploy_budget_binds() {
    let mut ers = gt_ers();
    ers.deployment.per_lap_deploy_mj = Some(1.0);
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&ers, None).unwrap();
    let mgr = EnergyManager::new(rb, Policy::RuleBased);
    let mut ledger = LapEnergyLedger::new();
    let inp = DecideInput {
        v: 40.0,
        driver_demand: 1.0,
        brake_demand_w: 0.0,
        mech_regen_envelope_w: 0.0,
        ice_surplus_w: 0.0,
        soc: 0.9,
        override_active: false,
        prev_k_power_w: 0.0,
        ramp_reduced_w: 0.0,
        dt: DT,
        station: 0,
    };
    for _ in 0..2000 {
        let cmd = mgr.decide(&inp, &ledger);
        ledger.record(&cmd, DT);
        assert!(ledger.deploy_j() <= 1.0e6 + 1e-9);
    }
    assert!(
        (ledger.deploy_j() - 1.0e6).abs() < 1.0,
        "the budget is used in full, got {}",
        ledger.deploy_j()
    );
}

/// The ledger integrates ELECTRICAL harvest power: for a braking step the banked electrical
/// energy is the mechanical absorption × 0.97, never the raw mechanical number.
#[test]
fn ledger_integrates_electrical_not_mechanical() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let mgr = EnergyManager::new(rb, Policy::RuleBased);
    let ledger = LapEnergyLedger::new();
    let mech = 200e3;
    let inp = DecideInput {
        v: 60.0,
        driver_demand: 0.0,
        brake_demand_w: 500e3,
        mech_regen_envelope_w: mech,
        ice_surplus_w: 0.0,
        soc: 0.5,
        override_active: false,
        prev_k_power_w: 0.0,
        ramp_reduced_w: 0.0,
        dt: DT,
        station: 0,
    };
    let cmd = mgr.decide(&inp, &ledger);
    assert_eq!(cmd.mode, ErsMode::HarvestBrake);
    assert_eq!(
        cmd.harvest_w,
        mech * 0.97,
        "electrical = mechanical × 0.97 (C5.2.21)"
    );
    let mut l = LapEnergyLedger::new();
    l.record(&cmd, DT);
    assert_eq!(l.harvest_j(), mech * 0.97 * DT);
}

/// Ledger closure: Σ commands·dt == ledger, bit-for-bit (the ledger is the only accumulator and
/// uses the same additions).
#[test]
fn ledger_closes_over_the_commands() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let mgr = EnergyManager::new(rb, Policy::RuleBased);
    let (cmds, ledger) = run_trace(&mgr, 0xC105_0E5, 50_000);
    let mut deploy = 0.0;
    let mut harvest = 0.0;
    for c in &cmds {
        deploy += c.deploy_w * DT;
        harvest += c.harvest_w * DT;
    }
    assert_eq!(
        deploy.to_bits(),
        ledger.deploy_j().to_bits(),
        "deploy closure"
    );
    assert_eq!(
        harvest.to_bits(),
        ledger.harvest_j().to_bits(),
        "harvest closure"
    );
}

/// The C5.12 ramp: at full throttle with recharge wanted, the K's demand may fall by at most the
/// initial step on the first reduction and by rate·dt afterwards, down to the back-drive floor.
#[test]
fn recharge_ramp_limits_are_respected() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let mgr = EnergyManager::new(rb, Policy::RuleBased);
    let ledger = LapEnergyLedger::new();
    let mut inp = DecideInput {
        v: 80.0,
        driver_demand: 1.0,
        brake_demand_w: 0.0,
        mech_regen_envelope_w: 400e3,
        ice_surplus_w: 150e3,
        soc: 0.3, // below the recharge target (window top) → recharge wanted
        override_active: false,
        prev_k_power_w: 350e3, // arriving from full deployment
        ramp_reduced_w: 0.0,
        dt: DT,
        station: 0,
    };
    // First reduction: exactly the 150 kW initial step (350 → 200 kW deploy).
    let c1 = mgr.decide(&inp, &ledger);
    assert_eq!(c1.mode, ErsMode::HarvestStraight);
    assert_eq!(c1.deploy_w, 200e3, "initial step is 150 kW");
    // Subsequent reductions: 50 kW/s · dt = 1 kW per step.
    inp.prev_k_power_w = c1.deploy_w;
    inp.ramp_reduced_w = 150e3;
    let c2 = mgr.decide(&inp, &ledger);
    assert_eq!(c2.deploy_w, 200e3 - 50e3 * DT, "rate-limited to 50 kW/s");
    // Walk the whole transition: the per-step reduction never exceeds the allowed bound and the
    // command settles at the back-drive target.
    let mut prev = c1.deploy_w;
    let mut reduced = 150e3;
    for _ in 0..300_000 {
        inp.prev_k_power_w = prev;
        inp.ramp_reduced_w = reduced;
        let c = mgr.decide(&inp, &ledger);
        let k = c.deploy_w - c.harvest_w;
        let drop = prev - k;
        assert!(
            drop <= 50e3 * DT + 1e-9,
            "per-step reduction bound violated: {drop}"
        );
        reduced += drop;
        prev = k;
        if c.harvest_w > 0.0 && (c.harvest_w - 150e3 * 0.97).abs() < 1e-6 {
            break; // reached the ICE-surplus back-drive target (150 kW mech × 0.97)
        }
    }
    assert!(
        (prev + 150e3 * 0.97).abs() < 1e-6,
        "transition settles at the electrical back-drive target, got K = {prev}"
    );
}

/// A part-throttle step harvests the ICE-covered demand gap without any ramp involvement.
#[test]
fn part_throttle_harvests_the_demand_gap() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let mgr = EnergyManager::new(rb, Policy::RuleBased);
    let ledger = LapEnergyLedger::new();
    let inp = DecideInput {
        v: 45.0,
        driver_demand: 0.6,
        brake_demand_w: 0.0,
        mech_regen_envelope_w: 400e3,
        ice_surplus_w: 120e3,
        soc: 0.3,
        override_active: false,
        prev_k_power_w: 0.0,
        ramp_reduced_w: 0.0,
        dt: DT,
        station: 0,
    };
    let cmd = mgr.decide(&inp, &ledger);
    assert_eq!(cmd.mode, ErsMode::HarvestPartThrottle);
    assert_eq!(cmd.harvest_w, 120e3 * 0.97);
    // At or above the recharge target the same step deploys instead. The default target is now the
    // TOP of the usable window (0.9 for the f1 pack), so the store recharges toward full — the
    // deploy-instead boundary sits at the window top.
    let above = DecideInput { soc: 0.9, ..inp };
    let cmd = mgr.decide(&above, &ledger);
    assert_eq!(cmd.mode, ErsMode::Deploy);
}

/// A `Schedule` policy reproduces a hand-computed u(s) trace exactly.
#[test]
fn schedule_policy_reproduces_a_hand_computed_trace() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let schedule = UsSchedule::new(
        vec![1.0, 0.5, 0.0, -0.25, -1.0],
        vec![false, true, false, false, false],
        vec![0.0; 5],
        vec![0; 5],
    )
    .unwrap();
    let mgr = EnergyManager::new(rb, Policy::Schedule(schedule));
    let ledger = LapEnergyLedger::new();
    let base = DecideInput {
        v: 50.0, // 180 kph: inside the full-power plateau on both envelopes
        driver_demand: 1.0,
        brake_demand_w: 0.0,
        mech_regen_envelope_w: 300e3,
        ice_surplus_w: 0.0,
        soc: 0.5,
        override_active: false,
        prev_k_power_w: 0.0,
        ramp_reduced_w: 0.0,
        dt: DT,
        station: 0,
    };
    // Station 0: u = +1 → full curve, base envelope.
    let c = mgr.decide(&DecideInput { station: 0, ..base }, &ledger);
    assert_eq!(
        (c.deploy_w, c.harvest_w, c.mode),
        (350e3, 0.0, ErsMode::Deploy)
    );
    // Station 1: u = +0.5 with the schedule's override flag → half the override envelope.
    let c = mgr.decide(&DecideInput { station: 1, ..base }, &ledger);
    assert_eq!(
        (c.deploy_w, c.harvest_w, c.mode),
        (175e3, 0.0, ErsMode::OverrideDeploy)
    );
    // Station 2: u = 0 → idle.
    let c = mgr.decide(&DecideInput { station: 2, ..base }, &ledger);
    assert_eq!((c.deploy_w, c.harvest_w, c.mode), (0.0, 0.0, ErsMode::Idle));
    // Station 3: u = −0.25, not braking → a quarter of the harvest ceiling (300 kW mech × 0.97).
    let c = mgr.decide(&DecideInput { station: 3, ..base }, &ledger);
    assert_eq!(
        (c.deploy_w, c.harvest_w, c.mode),
        (0.0, 300e3 * 0.97 * 0.25, ErsMode::HarvestStraight)
    );
    // Station 4: u = −1 while braking → the full ceiling, brake-harvest mode.
    let braking = DecideInput {
        station: 4,
        brake_demand_w: 600e3,
        ..base
    };
    let c = mgr.decide(&braking, &ledger);
    assert_eq!(
        (c.deploy_w, c.harvest_w, c.mode),
        (0.0, 300e3 * 0.97, ErsMode::HarvestBrake)
    );
    // The per-run override flag wins unconditionally at a station whose schedule flag is off.
    let forced = DecideInput {
        station: 0,
        override_active: true,
        ..base
    };
    let c = mgr.decide(&forced, &ledger);
    assert_eq!(c.mode, ErsMode::OverrideDeploy);
}

/// `validate_shift_maps` accepts every in-range `shift_map_id` and names the first out-of-range one
/// (§8.3, D-M6-9): id 0 is always valid (the derived default); ids `≥ n_maps` are undefined.
#[test]
fn shift_map_ids_are_validated_against_the_resolved_map_count() {
    use outlap_powertrain::ScheduleError;
    // A vehicle with 1 derived default + 2 named maps ⇒ ids 0, 1, 2 all valid.
    let ok = UsSchedule::new(
        vec![0.0; 4],
        vec![false; 4],
        vec![f64::INFINITY; 4],
        vec![0, 2, 1, 0],
    )
    .unwrap();
    assert!(ok.validate_shift_maps(3).is_ok());
    // The default-only vehicle (n_maps = 1) rejects any id > 0, naming the offending station.
    assert_eq!(
        ok.validate_shift_maps(1),
        Err(ScheduleError::UnknownShiftMap {
            index: 1,
            id: 2,
            n_maps: 1,
        }),
    );
}

/// Determinism: the same trace twice produces bit-identical commands and ledgers.
#[test]
fn same_trace_twice_is_bit_identical() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let mgr = EnergyManager::new(rb, Policy::RuleBased);
    let (cmds_a, ledger_a) = run_trace(&mgr, 0xDE7E_2814, 20_000);
    let (cmds_b, ledger_b) = run_trace(&mgr, 0xDE7E_2814, 20_000);
    assert_eq!(ledger_a.deploy_j().to_bits(), ledger_b.deploy_j().to_bits());
    assert_eq!(
        ledger_a.harvest_j().to_bits(),
        ledger_b.harvest_j().to_bits()
    );
    for (a, b) in cmds_a.iter().zip(&cmds_b) {
        assert_eq!(a.deploy_w.to_bits(), b.deploy_w.to_bits());
        assert_eq!(a.harvest_w.to_bits(), b.harvest_w.to_bits());
        assert_eq!(a.mode, b.mode);
    }
}

/// f32 and f64 managers agree over a trace to single-precision tolerance.
#[test]
fn f32_f64_manager_parity() {
    let ers = f1_ers();
    let mgr64 = EnergyManager::new(
        ErsRulebook::<f64>::from_schema(&ers, None).unwrap(),
        Policy::RuleBased,
    );
    let mgr32 = EnergyManager::new(
        ErsRulebook::<f32>::from_schema(&ers, None).unwrap(),
        Policy::RuleBased,
    );
    let mut rng = TestRng::new(0xF3264);
    let ledger64 = LapEnergyLedger::<f64>::new();
    let ledger32 = LapEnergyLedger::<f32>::new();
    for _ in 0..5_000 {
        let v = rng.range(0.0, 110.0);
        let soc = rng.range(0.0, 1.0);
        let inp64 = random_input(&mut rng, v, soc, &ErsCommand::idle());
        #[allow(clippy::cast_possible_truncation)]
        let inp32 = DecideInput::<f32> {
            v: inp64.v as f32,
            driver_demand: inp64.driver_demand as f32,
            brake_demand_w: inp64.brake_demand_w as f32,
            mech_regen_envelope_w: inp64.mech_regen_envelope_w as f32,
            ice_surplus_w: inp64.ice_surplus_w as f32,
            soc: inp64.soc as f32,
            override_active: inp64.override_active,
            prev_k_power_w: inp64.prev_k_power_w as f32,
            ramp_reduced_w: inp64.ramp_reduced_w as f32,
            dt: inp64.dt as f32,
            station: inp64.station,
        };
        let c64 = mgr64.decide(&inp64, &ledger64);
        let c32 = mgr32.decide(&inp32, &ledger32);
        assert!(
            (c64.deploy_w - f64::from(c32.deploy_w)).abs() <= 1e-2 * 350e3,
            "deploy parity at v={v}"
        );
        assert!(
            (c64.harvest_w - f64::from(c32.harvest_w)).abs() <= 1e-2 * 350e3,
            "harvest parity at v={v}"
        );
    }
}
