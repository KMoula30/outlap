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
fn driver_section_resolves_explicit_values_and_surfaces_defaulted_gains() {
    // The f1_2026 fixture carries a *partial* driver section: the given fields resolve verbatim, the
    // omitted gains fall back to the MacAdam/PI literature defaults and are surfaced as estimated.
    let l = loader();
    let a = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default()).unwrap();
    let driver = a.spec.driver.as_ref().expect("f1 has a driver section");
    assert_eq!(driver.preview_time_s, Some(0.7));
    assert_eq!(driver.max_steer_rad, Some(0.35));
    assert_eq!(driver.speed_kp, Some(0.25));
    assert_eq!(driver.speed_ki, Some(0.06));
    // Resolved accessors: explicit where given, literature default where omitted.
    assert_eq!(driver.preview_time_s(), 0.7);
    assert_eq!(
        driver.preview_gain(),
        outlap_schema::vehicle::Driver::DEFAULT_PREVIEW_GAIN
    );
    // The omitted gains appear in the estimated report; the given ones do not.
    let estimated_ptrs: Vec<&str> = a
        .report
        .estimated
        .iter()
        .map(|e| e.pointer.as_str())
        .collect();
    assert!(estimated_ptrs.contains(&"/driver/preview_gain"));
    assert!(estimated_ptrs.contains(&"/driver/ff_accel_scale_mps2"));
    assert!(!estimated_ptrs.contains(&"/driver/preview_time_s"));
    assert!(!estimated_ptrs.contains(&"/driver/speed_kp"));
}

#[test]
fn absent_driver_section_surfaces_all_gains_as_estimated() {
    // ev_1du_rwd has no driver section: every driver gain is a literature default → all reported.
    let l = loader();
    let a = load_vehicle("ev_1du_rwd/vehicle.yaml", &l, &LoadOptions::default()).unwrap();
    assert!(a.spec.driver.is_none(), "no driver section given");
    let n = a
        .report
        .estimated
        .iter()
        .filter(|e| e.pointer.starts_with("/driver/"))
        .count();
    assert_eq!(n, 10, "all ten driver gains surfaced as estimated");
    // Record-only: the hash matches an in-memory re-resolve (no spec mutation from the driver
    // estimation), so a vehicle with no driver section is unchanged by the feature.
    let b = resolve_vehicle(&a.spec, &Overrides::default(), &l, &LoadOptions::default()).unwrap();
    assert_eq!(a.report.resolved_hash, b.report.resolved_hash);
}

#[test]
fn negative_driver_preview_time_is_a_semantic_error() {
    let l = loader();
    let mut spec = load_vehicle("ev_1du_rwd/vehicle.yaml", &l, &LoadOptions::default())
        .unwrap()
        .spec;
    spec.driver = Some(outlap_schema::vehicle::Driver {
        preview_time_s: Some(-0.5),
        ..Default::default()
    });
    let err = resolve_vehicle(&spec, &Overrides::default(), &l, &LoadOptions::default())
        .expect_err("negative preview time must be rejected");
    match err {
        outlap_schema::error::SchemaError::Semantic { message, .. } => {
            assert!(message.contains("preview_time_s"), "message: {message}");
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
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
fn brush_tyres_load_and_round_trip() {
    let l = loader();
    // Brush-only (tyr/1.1): structural keys + brush block, no MF6.1 force core → no warnings.
    let (brush, warnings) = load_tyr("tyr/brush_only.tyr.yaml", &l).unwrap();
    assert!(
        warnings.is_empty(),
        "brush-only should be clean: {warnings:?}"
    );
    assert!(brush.brush.is_some(), "brush block should parse");
    let back: outlap_schema::tyr::Tyr =
        serde_json::from_str(&serde_json::to_string(&brush).unwrap()).unwrap();
    assert_eq!(brush, back);

    // Full MF6.1 core + brush block → also clean (both models available).
    let (both, warnings) = load_tyr("tyr/brush_plus_mf61.tyr.yaml", &l).unwrap();
    assert!(
        warnings.is_empty(),
        "mf61+brush should be clean: {warnings:?}"
    );
    assert!(both.brush.is_some() && both.mf61.0.contains_key("PDX1"));
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

#[test]
fn dotted_override_indexes_into_sequences() {
    use outlap_schema::load::Origin;
    let l = loader();
    let base = load_vehicle("ev_1du_rwd/vehicle.yaml", &l, &LoadOptions::default()).unwrap();
    // A what-if drive-unit swap: numeric segments index EXISTING sequence elements.
    let overrides =
        Overrides::new().with("drivetrain.units.0.source", "ptm/front_drive_unit.ptm.yaml");
    let r = resolve_vehicle(&base.spec, &overrides, &l, &LoadOptions::default()).unwrap();
    assert_eq!(
        r.spec.drivetrain.units[0].source.as_str(),
        "ptm/front_drive_unit.ptm.yaml"
    );
    assert!(matches!(
        r.provenance.get("/drivetrain/units/0/source"),
        Some(Origin::DottedOverride { .. })
    ));

    // Out-of-bounds index and non-numeric segment on a sequence are loud errors.
    let oob = Overrides::new().with("drivetrain.units.3.source", "x");
    let e = resolve_vehicle(&base.spec, &oob, &l, &LoadOptions::default()).unwrap_err();
    assert!(e.to_string().contains("out of bounds"), "{e}");
    let non_numeric = Overrides::new().with("drivetrain.units.first.source", "x");
    let e = resolve_vehicle(&base.spec, &non_numeric, &l, &LoadOptions::default()).unwrap_err();
    assert!(e.to_string().contains("numeric index"), "{e}");
}
