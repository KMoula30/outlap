// SPDX-License-Identifier: AGPL-3.0-only
//! Emit a CSV trace of the energy manager over a synthetic lap (the source of the
//! `docs/theory/img/ers_manager_trace.png` figure — regenerate with
//! `cargo run -p outlap-powertrain --example manager_trace > trace.csv`).
//!
//! The lap is a caricature with the phases the rule-based policy distinguishes: a pit-straight
//! launch and long straight (deploy → taper), heavy braking (harvest), a medium corner + part
//! throttle (part-throttle recharge), a second straight starting SoC-poor (super-clip ramp), and
//! a final braking zone. The pack is a trivial integrator here — PR2/PR4 wire the real Thevenin
//! pack; this example only exercises the manager itself.

use outlap_powertrain::{
    DecideInput, EnergyManager, ErsCommand, ErsRulebook, LapEnergyLedger, Policy,
};
use outlap_schema::vehicle::{
    Activation, Deployment, EnergyStore, Ers, OverrideMode, Recovery, SpeedTaper,
};

const DT: f64 = 0.02;
/// Usable-window energy for the toy SoC integrator, J (D-M6-3: 4 MJ over [0.2, 0.9]).
const WINDOW_J: f64 = 4.0e6;

/// The verified FIA Issue-19 `ers:` block (D-M6-5 breakpoints; knee exactly 2/7).
fn f1_ers() -> Ers {
    Ers {
        mgu_k: "ptm/mgu_k.ptm.yaml".into(),
        es: EnergyStore {
            capacity_mj: 4.0,
            soc_window: [0.2, 0.9],
        },
        deployment: Deployment {
            power_limit_kw: 350.0,
            taper_vs_speed: SpeedTaper {
                speed_kph: vec![0.0, 290.0, 340.0, 345.0],
                power_frac: vec![1.0, 1.0, 2.0 / 7.0, 0.0],
            },
            per_lap_deploy_mj: None,
        },
        override_mode: Some(OverrideMode {
            power_limit_kw: 350.0,
            taper_vs_speed: SpeedTaper {
                speed_kph: vec![0.0, 337.5, 355.0],
                power_frac: vec![1.0, 1.0, 0.0],
            },
            extra_energy_per_lap_mj: Some(0.5),
            activation: Activation::Strategy,
        }),
        recovery: Recovery {
            braking_power_limit_kw: 350.0,
            per_lap_harvest_mj: 8.5,
            recharge_phases: true,
            recharge_target_soc: None,
            ramp_initial_step_kw: None,
            ramp_rate_kw_per_s: None,
            ramp_total_kw: None,
        },
        elec_mech_factor: None,
    }
}

/// (duration s, v_start kph, v_end kph, demand, brake MW, ICE surplus kW) per lap phase.
const PHASES: &[(f64, f64, f64, f64, f64, f64)] = &[
    (9.0, 80.0, 330.0, 1.0, 0.0, 0.0),    // pit straight: launch to near top speed
    (2.5, 330.0, 90.0, 0.0, 1.6e6, 0.0),  // T1 heavy braking
    (3.0, 90.0, 110.0, 0.55, 0.0, 180e3), // medium corner, part throttle
    (8.0, 110.0, 320.0, 1.0, 0.0, 150e3), // back straight (SoC low → super-clip at the end)
    (2.0, 320.0, 120.0, 0.0, 1.2e6, 0.0), // chicane braking
    (5.0, 120.0, 250.0, 1.0, 0.0, 0.0),   // final sector
];

fn main() {
    let rulebook: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let mgr = EnergyManager::new(rulebook, Policy::RuleBased);
    let mut ledger = LapEnergyLedger::new();
    let mut prev = ErsCommand::idle();
    let mut soc = 0.62_f64; // just above mid-window at the start line
    let mut ramp_reduced = 0.0_f64;
    let mut t = 0.0_f64;

    println!("t_s,v_kph,demand,brake_w,soc,mode,deploy_w,harvest_w,ledger_deploy_j,ledger_harvest_j");
    for &(dur, v0, v1, demand, brake_w, surplus_w) in PHASES {
        let steps = (dur / DT) as usize;
        for i in 0..steps {
            let frac = i as f64 / steps as f64;
            let v_kph = v0 + (v1 - v0) * frac;
            let inp = DecideInput {
                v: v_kph / 3.6,
                driver_demand: demand,
                brake_demand_w: brake_w,
                mech_regen_envelope_w: 420e3,
                ice_surplus_w: surplus_w,
                soc,
                override_active: false,
                prev_k_power_w: prev.deploy_w - prev.harvest_w,
                ramp_reduced_w: ramp_reduced,
                dt: DT,
                station: 0,
            };
            let cmd = mgr.decide(&inp, &ledger);
            // Caller-owned ramp episode accounting: accumulate reductions, reset when demand
            // rises back onto the deploy path.
            let k_prev = inp.prev_k_power_w;
            let k_now = cmd.deploy_w - cmd.harvest_w;
            if k_now < k_prev {
                ramp_reduced += k_prev - k_now;
            } else {
                ramp_reduced = 0.0;
            }
            ledger.record(&cmd, DT);
            // Toy pack: SoC over the usable window, clamped (PR2 wires the real Thevenin pack).
            soc = (soc + (cmd.harvest_w - cmd.deploy_w) * DT / WINDOW_J * 0.7).clamp(0.2, 0.9);
            println!(
                "{t:.3},{v_kph:.2},{demand},{brake_w},{soc:.5},{:?},{:.1},{:.1},{:.1},{:.1}",
                cmd.mode,
                cmd.deploy_w,
                cmd.harvest_w,
                ledger.deploy_j(),
                ledger.harvest_j()
            );
            prev = cmd;
            t += DT;
        }
    }
}
