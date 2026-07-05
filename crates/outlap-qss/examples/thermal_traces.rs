// SPDX-License-Identifier: AGPL-3.0-only
//! Emit machine-thermal validation traces from the **real** `outlap-thermal` integrator (via
//! `outlap-qss`'s `MachineThermal`) as CSV on stdout. `python/tools/plot_machine_thermal.py` runs
//! this and plots the output, so the theory figure is driven by the actual model, not a re-implementation.
//!
//! Three scenarios, one CSV with a `scenario` column:
//! - `lti`: a 2-node winding↔ambient network; `x` = time [s], `a` = winding [°C]. Python overlays the
//!   closed-form LTI step response `T_amb + (P/g)(1 − e^{−t g/C})`.
//! - `stint`: the committed `rear.emotor.yaml` under a closed derate→loss loop; `x` = lap,
//!   `a` = winding [°C], `b` = torque derate.
//! - `speed`: the detailed `pdt_synth.emotor.yaml` (air-gap film + per-component losses) at two shaft
//!   speeds; `x` = time [s], `a` = magnet [°C] at ω=100 rad/s, `b` at ω=1500 rad/s.

use outlap_qss::MachineThermal;
use outlap_schema::emotor::{Conductance, Cooling, Emotor, EmotorMeta, NodeRole, ThermalNode};
use outlap_schema::io::FsLoader;
use outlap_schema::load::load_emotor;
use outlap_schema::version::SchemaVersion;
use outlap_schema::Conditions;

/// LTI scenario parameters (Python recomputes the analytic curve from these — keep in sync).
const LTI_CAP: f64 = 2000.0;
const LTI_G: f64 = 5.0;
const LTI_P: f64 = 1500.0;

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

fn none(_: &str) -> Option<f64> {
    None
}

#[allow(clippy::too_many_lines)] // one linear trace-emitting script; splitting it hurts clarity.
fn main() {
    let cond = Conditions::default();
    println!(
        "# lti_cap={LTI_CAP} lti_g={LTI_G} lti_p={LTI_P} ambient_c={}",
        cond.ambient_c
    );
    println!("scenario,x,a,b");

    // --- (a) LTI: a 2-node winding↔ambient network through the real integrator. -----------------
    let node = |name: &str, role: NodeRole, c: Option<f64>| ThermalNode {
        name: name.into(),
        role: Some(role),
        c_j_per_k: c,
        t_warn_c: None,
        t_max_c: None,
    };
    let lti = Emotor {
        schema: SchemaVersion::new("emotor", 1, 1),
        nodes: vec![
            node("winding", NodeRole::Winding, Some(LTI_CAP)),
            node("ambient", NodeRole::Ambient, None),
        ],
        conductances: vec![Conductance {
            between: ("winding".into(), "ambient".into()),
            w_per_k: Some(LTI_G),
        }],
        convection: vec![],
        loss_routing: vec![],
        cooling: Cooling {
            ambient_node: "ambient".into(),
            ambient_fixed_c: None,
            coolant: None,
            jacket: None,
            air_gap: None,
        },
        cu_feedback: None,
        initial_temp: None,
        meta: EmotorMeta::default(),
    };
    let mut m = MachineThermal::assemble(&lti, &cond, 40.0).expect("lti assembles");
    let dt = 4.0;
    for k in 0..250 {
        println!(
            "lti,{:.1},{:.4},",
            f64::from(k) * dt,
            m.temp_c("winding").unwrap()
        );
        m.step(LTI_P, none, 0.0, dt).expect("lti step");
    }

    // --- (b) Stint on the committed rear.emotor.yaml under a closed derate→loss loop. -----------
    let mut rear = MachineThermal::assemble(
        &load_emotor("emotor/rear.emotor.yaml", &fixtures()).expect("rear loads"),
        &cond,
        45.0,
    )
    .expect("rear assembles");
    let base = 3000.0;
    let steps_per_lap = 200;
    for lap in 0..26 {
        for step in 0..steps_per_lap {
            if step % 5 == 0 {
                let frac = f64::from(lap) + f64::from(step) / f64::from(steps_per_lap);
                println!(
                    "stint,{frac:.4},{:.3},{:.4}",
                    rear.temp_c("winding").unwrap(),
                    rear.derate()
                );
            }
            let d = rear.derate();
            rear.step(base * d, none, 800.0, 1.0).expect("stint step");
        }
    }
    println!(
        "stint,26.0000,{:.3},{:.4}",
        rear.temp_c("winding").unwrap(),
        rear.derate()
    );

    // --- (c) Speed-dependent air-gap cooling: the detailed pdt_synth network at two shaft speeds.
    // Per-component losses drive the magnet, whose only escape is the (speed-dependent) air-gap film.
    let magnet_loss = |name: &str| -> Option<f64> {
        match name {
            "winding_active" => Some(2500.0),
            "core_stator" => Some(800.0),
            "magnet" => Some(600.0),
            _ => None,
        }
    };
    let magnet_trace = |omega: f64| -> Vec<f64> {
        let mut r = MachineThermal::assemble(
            &load_emotor("emotor/pdt_synth.emotor.yaml", &fixtures()).unwrap(),
            &cond,
            40.0,
        )
        .unwrap();
        let mut trace = Vec::with_capacity(1600);
        for _ in 0..1600 {
            trace.push(r.temp_c("magnet").unwrap());
            r.step(3900.0, magnet_loss, omega, 0.25).unwrap();
        }
        trace
    };
    let slow = magnet_trace(100.0);
    let fast = magnet_trace(1500.0);
    for (k, (s, f)) in slow.iter().zip(fast.iter()).enumerate() {
        println!(
            "speed,{:.2},{s:.3},{f:.3}",
            f64::from(u16::try_from(k).unwrap()) * 0.25
        );
    }
}
