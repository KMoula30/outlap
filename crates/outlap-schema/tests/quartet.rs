// SPDX-License-Identifier: AGPL-3.0-only
//! Loading tests for the rest of the input quartet: `track.yaml`, `conditions.yaml`, `sim.yaml`.
// Fixture values parse exactly, so exact float comparison is intentional.
#![allow(clippy::float_cmp)]

use outlap_schema::io::FsLoader;
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
    assert_eq!(s.fz_coupling, FzCoupling::OneStepLag);
    assert_eq!(s.raceline.generator, Some(RacelineGenerator::MinCurvature));
    assert!(!s.allow_degraded);

    // Defaults fill an empty document.
    let d = outlap_schema::Sim::default();
    assert_eq!(d.envelope.v_points, 40);
    assert_eq!(d.integrator, outlap_schema::sim::Integrator::Heun);
}

#[test]
fn wrong_document_kind_is_rejected() {
    // Feeding a conditions file where a sim is expected fails the version gate, not deserialization.
    let err = load_sim("conditions/hot_dry.conditions.yaml", &loader()).unwrap_err();
    let msg = format!("{err}");
    assert!(msg.contains("sim") || msg.contains("conditions"), "{msg}");
}
