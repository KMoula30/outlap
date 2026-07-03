// SPDX-License-Identifier: AGPL-3.0-only
//! The PDT importer's emitted `.ptm` documents load cleanly in the Rust core (round-trip, §13).
//!
//! Fixtures are importer output on *synthetic* PDT-shaped inputs (never real PDT data). This
//! exercises the full loader — version gate, unknown-key walk, and `check_ptm` semantic checks —
//! so a format drift between the Python emitter and the Rust contract fails CI.

use outlap_schema::io::FsLoader;
use outlap_schema::load::load_ptm;
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
