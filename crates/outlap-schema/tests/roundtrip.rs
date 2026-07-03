// SPDX-License-Identifier: AGPL-3.0-only
//! Round-trip and resolution tests over the synthetic reference fixtures.
// Fixture values (1590.0, 2.92, 1500.0) parse exactly, so exact float comparison is intentional.
#![allow(clippy::float_cmp)]

use outlap_schema::io::FsLoader;
use outlap_schema::load::{load_emotor, load_ptm, load_tyr, resolve_vehicle, Overrides};
use outlap_schema::{load_vehicle, LoadOptions};

fn loader() -> FsLoader {
    FsLoader::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
}

const REFERENCE_VEHICLES: &[&str] = &[
    "ev_1du_rwd/vehicle.yaml",
    "ev_2du_awd/vehicle.yaml",
    "f1_2026/vehicle.yaml",
    "fwd_hatch/vehicle.yaml",
];

#[test]
fn all_reference_vehicles_load() {
    let l = loader();
    for path in REFERENCE_VEHICLES {
        let resolved = load_vehicle(path, &l, &LoadOptions::default())
            .unwrap_or_else(|e| panic!("{path} failed to load: {e:?}"));
        assert!(
            !resolved.report.resolved_hash.is_empty(),
            "{path} has no hash"
        );
        assert_eq!(
            resolved.report.resolved_hash.len(),
            64,
            "{path} blake3 hex is 64 chars"
        );
        assert!(
            resolved.spec.extends.is_none(),
            "{path} extends must be resolved away"
        );
    }
}

#[test]
fn resolved_vehicle_round_trips_through_types() {
    // load -> spec -> resolve in-memory -> spec2; semantically stable.
    let l = loader();
    for path in REFERENCE_VEHICLES {
        let a = load_vehicle(path, &l, &LoadOptions::default()).unwrap();
        let b = resolve_vehicle(&a.spec, &Overrides::default(), &l, &LoadOptions::default())
            .unwrap_or_else(|e| panic!("{path} in-memory resolve failed: {e:?}"));
        assert_eq!(a.spec, b.spec, "{path} not stable through the type system");
        assert_eq!(
            a.report.resolved_hash, b.report.resolved_hash,
            "{path} hash changed"
        );
    }
}

#[test]
fn yaml_serialization_round_trips() {
    // spec -> JSON -> spec; the types deserialize their own serialized form.
    let l = loader();
    let a = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default()).unwrap();
    let json = serde_json::to_string(&a.spec).unwrap();
    let back: outlap_schema::Vehicle = serde_json::from_str(&json).unwrap();
    assert_eq!(a.spec, back);
}

#[test]
fn referenced_files_load_standalone() {
    let l = loader();
    let ptm = load_ptm("ptm/ice_v6.ptm.yaml", &l).unwrap();
    let ptm_back: outlap_schema::ptm::Ptm =
        serde_json::from_str(&serde_json::to_string(&ptm).unwrap()).unwrap();
    assert_eq!(ptm, ptm_back);

    let (tyr, warnings) = load_tyr("tyr/slick.tyr.yaml", &l).unwrap();
    assert!(
        warnings.is_empty(),
        "clean tyr should have no warnings: {warnings:?}"
    );
    let tyr_back: outlap_schema::tyr::Tyr =
        serde_json::from_str(&serde_json::to_string(&tyr).unwrap()).unwrap();
    assert_eq!(tyr, tyr_back);

    let em = load_emotor("emotor/rear.emotor.yaml", &l).unwrap();
    let em_back: outlap_schema::emotor::Emotor =
        serde_json::from_str(&serde_json::to_string(&em).unwrap()).unwrap();
    assert_eq!(em, em_back);
}

#[test]
fn extends_deep_merges_and_tracks_provenance() {
    use outlap_schema::load::Origin;
    let l = loader();
    let r = load_vehicle("ev_child/vehicle.yaml", &l, &LoadOptions::default()).unwrap();

    // Override wins; sibling fields inherited from the preset.
    assert_eq!(r.spec.chassis.mass_kg, 1590.0);
    assert_eq!(r.spec.chassis.wheelbase_m, 2.92);
    assert_eq!(r.spec.name, "EV child — lightweight");

    // Provenance: the override is Base, the inherited value is Inherited.
    assert!(matches!(
        r.provenance.get("/chassis/mass_kg"),
        Some(Origin::Base { .. })
    ));
    assert!(matches!(
        r.provenance.get("/chassis/wheelbase_m"),
        Some(Origin::Inherited { .. })
    ));

    // The report lists at least one inherited value.
    assert!(
        !r.report.inherited.is_empty(),
        "expected inherited report lines"
    );
}

#[test]
fn estimation_fills_and_reports() {
    let l = loader();
    // ev_1du has no anti_dive/anti_squat -> estimated to 0 and reported.
    let r = load_vehicle("ev_1du_rwd/vehicle.yaml", &l, &LoadOptions::default()).unwrap();
    assert_eq!(r.spec.suspension.front.anti_dive, Some(0.0));
    assert!(
        r.report
            .estimated
            .iter()
            .any(|e| e.pointer.contains("anti_dive")),
        "estimation should be reported: {:?}",
        r.report.estimated
    );
}

#[test]
fn dotted_override_applies_and_is_recorded() {
    use outlap_schema::load::Origin;
    let l = loader();
    let base = load_vehicle("ev_1du_rwd/vehicle.yaml", &l, &LoadOptions::default()).unwrap();
    let overrides = Overrides::new().with("chassis.mass_kg", 1500.0);
    let r = resolve_vehicle(&base.spec, &overrides, &l, &LoadOptions::default()).unwrap();
    assert_eq!(r.spec.chassis.mass_kg, 1500.0);
    assert!(matches!(
        r.provenance.get("/chassis/mass_kg"),
        Some(Origin::DottedOverride { .. })
    ));
    assert_ne!(
        base.report.resolved_hash, r.report.resolved_hash,
        "override should change the hash"
    );
}
