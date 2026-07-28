// SPDX-License-Identifier: AGPL-3.0-only
//! Loading tests for the rest of the input quartet: `track.yaml`, `conditions.yaml`, `sim.yaml`.
// Fixture values parse exactly, so exact float comparison is intentional.
#![allow(clippy::float_cmp)]

use std::fmt::Write as _;

use outlap_schema::io::{FsLoader, MemLoader};
use outlap_schema::sim::{FzCoupling, RacelineGenerator, Tier};
use outlap_schema::{load_conditions, load_sim, load_track_doc};

fn loader() -> FsLoader {
    FsLoader::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
}

#[test]
fn track_doc_loads() {
    let doc = load_track_doc("track/synthetic_oval.track.yaml", &loader()).unwrap();
    assert_eq!(doc.name, "Synthetic Oval");
    assert!(doc.closed);
    assert_eq!(doc.centerline.as_str(), "synthetic_oval.centerline.csv");
    assert_eq!(
        doc.meta.accuracy_class,
        Some(outlap_schema::track::AccuracyClass::C)
    );
    // The track/1.1 provenance meta round-trips through the loader (MT/U5).
    assert_eq!(doc.meta.width_source.as_deref(), Some("declared"));
    assert_eq!(doc.meta.importer_version.as_deref(), Some("fixture/1"));
    assert_eq!(doc.meta.stages, Some(vec!["base".to_owned()]));
}

/// A minimal in-memory track dir: a `track.yaml` (with optional keypoints) plus a 4-row
/// centerline whose `banking_deg` column is the given values.
fn mem_track(keypoints: bool, banking: [f64; 4]) -> MemLoader {
    let kp = if keypoints {
        "banking_keypoints:\n  - { s_m: 0.0, banking_deg: 0.0 }\n  - { s_m: 20.0, banking_deg: 5.0 }\n"
    } else {
        ""
    };
    let yaml = format!(
        "schema: track/1.1\nname: Mem Track\nclosed: false\ncenterline: centerline.csv\n{kp}"
    );
    let mut csv =
        String::from("s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale\n");
    for (i, b) in banking.iter().enumerate() {
        let s = f64::from(u32::try_from(i).expect("4-row fixture")) * 10.0;
        writeln!(csv, "{s:.1},{s:.1},0.0,0.0,{b:.3},6.0,6.0,1.0").expect("write to String");
    }
    MemLoader::new()
        .with("track.yaml", yaml)
        .with("centerline.csv", csv)
}

#[test]
fn track_both_banking_forms_conflict() {
    // KTD9: keypoints + a non-zero dense `banking_deg` column is a config error.
    let err = load_track_doc("track.yaml", &mem_track(true, [0.0, 2.5, 0.0, 0.0])).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("banking") && msg.contains("twice"),
        "unexpected message: {msg}"
    );
}

#[test]
fn track_keypoints_over_zero_column_stay_valid() {
    // The hand-annotation path: keypoints + an all-zero placeholder column loads fine.
    let doc = load_track_doc("track.yaml", &mem_track(true, [0.0; 4])).unwrap();
    assert_eq!(doc.banking_keypoints.len(), 2);
    // And a non-zero column WITHOUT keypoints is the normal dense path.
    let doc = load_track_doc("track.yaml", &mem_track(false, [0.0, 2.5, 0.0, 0.0])).unwrap();
    assert!(doc.banking_keypoints.is_empty());
}

#[test]
fn track_unknown_meta_field_is_rejected() {
    // The unknown-field walk still hard-errors on a bogus non-`x-` field in track/1.1 meta.
    let yaml =
        "schema: track/1.1\nname: Mem Track\ncenterline: centerline.csv\nmeta:\n  bogus_field: 1\n";
    let err = load_track_doc("track.yaml", &MemLoader::new().with("track.yaml", yaml)).unwrap_err();
    let msg = format!("{err}");
    assert!(msg.contains("bogus_field"), "unexpected message: {msg}");
}

#[test]
fn conditions_load_with_values() {
    let c = load_conditions("conditions/hot_dry.conditions.yaml", &loader()).unwrap();
    assert_eq!(c.air.temperature_c, 28.0);
    assert_eq!(c.air.pressure_hpa, 1005.0);
    assert_eq!(c.wind.speed_mps, 3.5);
    assert_eq!(c.track_surface_c, 41.0);
}

#[test]
fn conditions_default_is_isa() {
    // The whole document is optional: the default is full ISA still air.
    let d = outlap_schema::Conditions::default();
    assert_eq!(d.air.temperature_c, 20.0);
    assert_eq!(d.air.pressure_hpa, 1013.25);
    assert_eq!(d.wind.speed_mps, 0.0);
}

#[test]
fn sim_loads_and_defaults() {
    let s = load_sim("sim/qss.sim.yaml", &loader()).unwrap();
    assert_eq!(s.tier, Tier::T1);
    assert_eq!(s.dt_s, 0.001);
    assert_eq!(s.fz_coupling, Some(FzCoupling::OneStepLag));
    assert_eq!(s.resolved_fz_coupling(), FzCoupling::OneStepLag);
    assert_eq!(s.raceline.generator, Some(RacelineGenerator::MinCurvature));
    assert!(!s.allow_degraded);
    // The sim/1.3 recorded track/path numerics round-trip through the loader.
    assert_eq!(s.path_curvature_smooth_m, Some(25.0));
    assert_eq!(s.vertical_baseline_m, 30.0);

    // Defaults fill an empty document.
    let d = outlap_schema::Sim::default();
    assert_eq!(d.envelope.v_points, 40);
    assert_eq!(d.integrator, outlap_schema::sim::Integrator::Heun);
    // fz_coupling defaults to auto (None); T1 resolves it to one_step_lag.
    assert_eq!(d.fz_coupling, None);
    assert_eq!(d.resolved_fz_coupling(), FzCoupling::OneStepLag);
    assert_eq!(d.slow_decimation, 20);
    // The smoothing window defaults to the per-consumer legacy behavior (None); the vertical
    // finite-difference baseline to its historical 30 m.
    assert_eq!(d.path_curvature_smooth_m, None);
    assert_eq!(d.vertical_baseline_m, 30.0);
}

#[test]
fn wrong_document_kind_is_rejected() {
    // Feeding a conditions file where a sim is expected fails the version gate, not deserialization.
    let err = load_sim("conditions/hot_dry.conditions.yaml", &loader()).unwrap_err();
    let msg = format!("{err}");
    assert!(msg.contains("sim") || msg.contains("conditions"), "{msg}");
}
