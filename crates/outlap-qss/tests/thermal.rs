// SPDX-License-Identifier: AGPL-3.0-only
//! PR5 — machine LPTN thermal + derating (§8.5, Decision #25 as amended).
//!
//! Exercises the two assembly paths through the shared Crank–Nicolson integrator: the hand-authored
//! lumped network (constant conductances, total-loss split) and the detailed network (explicit
//! capacities + speed/temperature-dependent convection edges, per-component routing). Covers the
//! stint heat-soak → derate story, the speed-dependent cooling of the detailed path, energy closure
//! at steady state, mass-heuristic fill for an under-specified lumped model, and the remainder-to-
//! winding loss rule.
#![allow(clippy::float_cmp, clippy::doc_markdown)]

use outlap_qss::MachineThermal;
use outlap_schema::io::FsLoader;
use outlap_schema::load::load_emotor;
use outlap_schema::Conditions;

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

fn assemble(path: &str, mass_kg: f64) -> MachineThermal {
    let em = load_emotor(path, &fixtures()).expect("emotor loads");
    MachineThermal::assemble(&em, &Conditions::default(), mass_kg).expect("thermal assembles")
}

/// No named components ⇒ the whole total loss lands on the winding node (remainder rule).
fn no_components(_: &str) -> Option<f64> {
    None
}

#[test]
fn lumped_stint_heat_soak_reduces_derate() {
    // Realistic closed loop: torque (hence loss) scales with the derate, so heat-soak self-limits at
    // the winding limit instead of running away. The winding heats monotonically across laps and the
    // derate never rises.
    let mut m = assemble("emotor/rear.emotor.yaml", 45.0);
    let base_loss = 3000.0;
    let mut last_t = m.temp_c("winding").unwrap();
    let mut last_d = 1.0;
    for _ in 0..25 {
        for _ in 0..200 {
            let d = m.derate();
            m.step(base_loss * d, no_components, 800.0, 1.0).unwrap();
        }
        let t = m.temp_c("winding").unwrap();
        let d = m.derate();
        assert!(
            t >= last_t - 1e-6,
            "winding cooled unexpectedly: {t} < {last_t}"
        );
        assert!(d <= last_d + 1e-9, "derate rose: {d} > {last_d}");
        assert!(t.is_finite() && t < 1000.0, "temperature ran away: {t}");
        last_t = t;
        last_d = d;
    }
    // The stint has pushed the winding into the derating band.
    assert!(
        m.derate() < 1.0,
        "expected some derate after the stint, got {}",
        m.derate()
    );
    assert!(
        m.temp_c("winding").unwrap() > 155.0,
        "winding should be near its limit"
    );
}

#[test]
fn lumped_energy_closes_at_steady_state() {
    use outlap_schema::emotor::{
        Conductance, CoolantSpec, Cooling, Emotor, EmotorMeta, NodeRole, ThermalNode,
    };
    use outlap_schema::version::SchemaVersion;

    // Global energy balance with copper feedback OFF (so injected = nominal): every injected watt
    // leaves through the housing sink paths, coolant (45 W/K) and ambient (5 W/K).
    let node = |name: &str, role: NodeRole, c: Option<f64>| ThermalNode {
        name: name.into(),
        role: Some(role),
        c_j_per_k: c,
        t_warn_c: None,
        t_max_c: None,
    };
    let g = |a: &str, b: &str, w: f64| Conductance {
        between: (a.into(), b.into()),
        w_per_k: Some(w),
    };
    let em = Emotor {
        schema: SchemaVersion::new("emotor", 1, 1),
        nodes: vec![
            node("winding", NodeRole::Winding, Some(9000.0)),
            node("housing", NodeRole::Housing, Some(32000.0)),
            node("coolant", NodeRole::Coolant, None),
            node("ambient", NodeRole::Ambient, None),
        ],
        conductances: vec![
            g("winding", "housing", 12.0),
            g("housing", "coolant", 45.0),
            g("housing", "ambient", 5.0),
        ],
        convection: vec![],
        loss_routing: vec![],
        cooling: Cooling {
            ambient_node: "ambient".into(),
            ambient_fixed_c: None,
            jacket: None,
            air_gap: None,
            coolant: Some(CoolantSpec {
                node: "coolant".into(),
                inlet_c: 65.0,
                rho_cp_mdot_w_per_k: 900.0,
            }),
        },
        cu_feedback: None,
        initial_temp: None,
        meta: EmotorMeta::default(),
    };
    let mut m = MachineThermal::assemble(&em, &Conditions::default(), 45.0).unwrap();
    let p = 600.0;
    for _ in 0..400_000 {
        m.step(p, no_components, 500.0, 0.5).unwrap();
    }
    let t_amb = Conditions::default().ambient_c;
    let (t_h, t_c) = (m.temp_c("housing").unwrap(), m.temp_c("coolant").unwrap());
    let out = 45.0 * (t_h - t_c) + 5.0 * (t_h - t_amb);
    assert!(
        (out - p).abs() < 1.0,
        "energy not conserved: out {out} vs in {p}"
    );
}

#[test]
fn detailed_network_cooling_is_speed_dependent() {
    // The air-gap film in the detailed network stiffens with shaft speed, so the magnet runs cooler
    // at high speed for the same magnet loss. Compare steady magnet temperatures at two speeds.
    let component = |name: &str| -> Option<f64> {
        match name {
            "winding_active" => Some(2500.0),
            "core_stator" => Some(800.0),
            "magnet" => Some(600.0),
            _ => None,
        }
    };
    let steady_magnet = |omega: f64| {
        let mut m = assemble("emotor/pdt_synth.emotor.yaml", 40.0);
        for _ in 0..20_000 {
            m.step(3900.0, component, omega, 0.25).unwrap();
        }
        m.temp_c("magnet").unwrap()
    };
    let slow = steady_magnet(100.0);
    let fast = steady_magnet(1500.0);
    assert!(
        fast < slow,
        "magnet should run cooler at speed: fast {fast} !< slow {slow}"
    );
}

#[test]
fn mass_heuristics_fill_an_underspecified_lumped_model() {
    use outlap_schema::emotor::{
        Conductance, CoolantSpec, Cooling, Emotor, EmotorMeta, NodeRole, ThermalNode,
    };
    use outlap_schema::version::SchemaVersion;

    // A winding + housing + coolant + ambient model with NO capacities or conductances given.
    let node = |name: &str, role: NodeRole, warn: Option<f64>, max: Option<f64>| ThermalNode {
        name: name.into(),
        role: Some(role),
        c_j_per_k: None,
        t_warn_c: warn,
        t_max_c: max,
    };
    let em = Emotor {
        schema: SchemaVersion::new("emotor", 1, 1),
        nodes: vec![
            node("winding", NodeRole::Winding, Some(155.0), Some(180.0)),
            node("housing", NodeRole::Housing, None, None),
            node("coolant", NodeRole::Coolant, None, None),
            node("ambient", NodeRole::Ambient, None, None),
        ],
        conductances: vec![
            Conductance {
                between: ("winding".into(), "housing".into()),
                w_per_k: None,
            },
            Conductance {
                between: ("housing".into(), "coolant".into()),
                w_per_k: None,
            },
        ],
        convection: vec![],
        loss_routing: vec![],
        cooling: Cooling {
            ambient_node: "ambient".into(),
            ambient_fixed_c: None,
            jacket: None,
            air_gap: None,
            coolant: Some(CoolantSpec {
                node: "coolant".into(),
                inlet_c: 60.0,
                rho_cp_mdot_w_per_k: 800.0,
            }),
        },
        cu_feedback: None,
        initial_temp: None,
        meta: EmotorMeta::default(),
    };
    let mut m = MachineThermal::assemble(&em, &Conditions::default(), 50.0)
        .expect("assembles with heuristics");
    // Every filled value is surfaced as an estimate (2 capacities + 2 conductances).
    assert!(m.estimates().len() >= 4, "estimates: {:?}", m.estimates());
    // The network still runs and heats under load.
    let cold = m.temp_c("winding").unwrap();
    for _ in 0..500 {
        m.step(6000.0, no_components, 600.0, 0.1).unwrap();
    }
    assert!(m.temp_c("winding").unwrap() > cold);
}

#[test]
fn cooling_block_air_gap_cools_rotor_with_speed() {
    // The rear fixture's `cooling.air_gap` block derives a rotor↔stator air-gap film; that film
    // stiffens with shaft speed, so the rotor (which only escapes heat through the gap) runs cooler
    // at high speed for the same total loss.
    let steady_rotor = |omega: f64| {
        let mut m = assemble("emotor/rear.emotor.yaml", 45.0);
        for _ in 0..40_000 {
            m.step(1500.0, no_components, omega, 0.25).unwrap();
        }
        m.temp_c("rotor").unwrap()
    };
    let slow = steady_rotor(50.0);
    let fast = steady_rotor(1500.0);
    assert!(
        fast < slow,
        "rotor should run cooler at speed: fast {fast} !< slow {slow}"
    );
    assert!(fast.is_finite() && slow.is_finite());
}

#[test]
fn cold_start_begins_at_sink_temperatures() {
    // Default init: solid nodes at ambient, coolant node at its inlet.
    let m = assemble("emotor/rear.emotor.yaml", 45.0);
    let ambient_c = Conditions::default().ambient_c;
    assert!((m.temp_c("winding").unwrap() - ambient_c).abs() < 1e-9);
    assert!((m.temp_c("coolant").unwrap() - 65.0).abs() < 1e-9);
    assert_eq!(m.derate(), 1.0);
}
