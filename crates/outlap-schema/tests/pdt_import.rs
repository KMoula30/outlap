// SPDX-License-Identifier: AGPL-3.0-only
//! The PDT importer's emitted `.ptm` documents load cleanly in the Rust core (round-trip, §13).
//!
//! Fixtures are importer output on *synthetic* PDT-shaped inputs (never real PDT data). This
//! exercises the full loader — version gate, unknown-key walk, and `check_ptm` semantic checks —
//! so a format drift between the Python emitter and the Rust contract fails CI.

use outlap_schema::battery::{BatteryModelKind, TableLevel};
use outlap_schema::emotor::EmotorSource;
use outlap_schema::io::FsLoader;
use outlap_schema::load::{load_battery, load_emotor, load_ptm};
use outlap_schema::ptm::{LoadAxis, PtmKind};

fn loader() -> FsLoader {
    FsLoader::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
}

#[test]
fn edrive_ptm_loads() {
    let ptm = load_ptm("ptm/pdt_synth_edrive.ptm.yaml", &loader()).expect("edrive .ptm loads");
    assert_eq!(ptm.kind, PtmKind::ElectricMachine);
    assert!(ptm.mass_kg > 0.0);
    // Speed axis strictly ascending (enforced by check_ptm) and the load axis is torque-gridded.
    assert!(ptm.axes.speed_rpm.windows(2).all(|w| w[1] > w[0]));
    assert!(matches!(ptm.axes.load_axis, LoadAxis::TorqueNm { .. }));
    assert!(ptm
        .meta
        .source
        .as_deref()
        .unwrap_or("")
        .contains("PDT EDrive"));
    // The overload block round-trips with its three durations.
    let overload = ptm.limits.overload.expect("overload present");
    assert_eq!(overload.durations_s, vec![10.0, 20.0, 30.0]);
}

#[test]
fn driveunit_ptm_loads() {
    let ptm = load_ptm("ptm/pdt_synth_du.ptm.yaml", &loader()).expect("driveunit .ptm loads");
    assert_eq!(ptm.kind, PtmKind::DriveUnit);
    assert_eq!(ptm.meta.upstream_ratio_applied, Some(true));
    assert!(ptm
        .meta
        .source
        .as_deref()
        .unwrap_or("")
        .contains("PDT DriveUnit"));
    // serde JSON round-trip is stable.
    let json = serde_json::to_string(&ptm).unwrap();
    let back: outlap_schema::ptm::Ptm = serde_json::from_str(&json).unwrap();
    assert_eq!(ptm, back);
}

#[test]
fn imported_emotor_loads() {
    use outlap_schema::emotor::NodeRole;

    let em = load_emotor("emotor/pdt_synth.emotor.yaml", &loader()).expect("emotor loads");
    // An import stamps the PdtImported source (wire form `pdt_imported`).
    assert_eq!(em.meta.source, Some(EmotorSource::PdtImported));
    // The detailed network declares a winding node, positive capacities, and convection edges
    // (the speed/temperature-dependent conductance path).
    assert!(em.nodes.iter().any(|n| n.role == Some(NodeRole::Winding)));
    assert!(em.nodes.iter().all(|n| n.c_j_per_k.is_none_or(|c| c > 0.0)));
    assert!(
        !em.convection.is_empty(),
        "detailed fixture carries convection edges"
    );
}

#[test]
fn vdc_stacked_ptm_loads() {
    // A ptm/1.1 drive-unit map carries an optional, strictly-ascending Vdc axis.
    let ptm = load_ptm("ptm/pdt_synth_du_vdc.ptm.yaml", &loader()).expect("vdc .ptm loads");
    assert_eq!(ptm.kind, PtmKind::DriveUnit);
    assert_eq!(ptm.schema.minor, 1, "the Vdc axis is a ptm/1.1 feature");
    let vdc = ptm.axes.vdc_v.clone().expect("vdc axis present");
    assert_eq!(vdc, vec![730.0, 790.0, 850.0]);
    assert!(vdc.windows(2).all(|w| w[1] > w[0]), "vdc axis ascending");
    // serde round-trip is stable with the new axis.
    let json = serde_json::to_string(&ptm).unwrap();
    assert_eq!(ptm, serde_json::from_str(&json).unwrap());
}

#[test]
fn battery_doc_loads() {
    // The battery/1.0 document round-trips through the full loader (version gate + check_battery).
    let b = load_battery("battery/synth_pack.battery.yaml", &loader()).expect("battery loads");
    assert_eq!(b.model, BatteryModelKind::RcPairs);
    assert_eq!((b.topology.ns, b.topology.np), (220, 1));
    assert_eq!(b.ecm.tables.level, TableLevel::Cell);
    assert_eq!(b.ecm.rc_pairs, 1);
    // SoC window ascending in [0,1]; ECM axes strictly ascending (check_battery enforces both).
    assert!(b.soc_window[0] < b.soc_window[1]);
    assert!(b.ecm.axes.soc.windows(2).all(|w| w[1] > w[0]));
    assert!(b.ecm.axes.temp_c.windows(2).all(|w| w[1] > w[0]));
}
