// SPDX-License-Identifier: AGPL-3.0-only
//! Golden-diagnostic tests: known-bad inputs must produce the right typed error, a helpful
//! message, and a span pointing at the offending token. This is the #43 contract under test.

use outlap_schema::error::SchemaError;
use outlap_schema::io::{FsLoader, MemLoader};
use outlap_schema::load::load_tyr;
use outlap_schema::{load_vehicle, LoadOptions};

fn loader() -> FsLoader {
    FsLoader::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
}

fn load_err(path: &str) -> SchemaError {
    load_vehicle(path, &loader(), &LoadOptions::default())
        .expect_err(&format!("{path} should have failed to load"))
}

/// The byte offset of the first occurrence of `needle` in a fixture's content.
fn offset_of(path: &str, needle: &str) -> usize {
    let full = format!("{}/tests/fixtures/{path}", env!("CARGO_MANIFEST_DIR"));
    let content = std::fs::read_to_string(full).unwrap();
    content.find(needle).expect("needle present")
}

#[test]
fn unknown_key_is_caught_with_did_you_mean_and_span() {
    let err = load_err("bad/unknown_key.yaml");
    match err {
        SchemaError::UnknownField {
            field, help, span, ..
        } => {
            assert_eq!(field, "chasis");
            let help = help.expect("a suggestion");
            assert!(
                help.contains("chassis"),
                "help should suggest chassis: {help}"
            );
            // Span points at the `chasis` key token (not the word in the comment above).
            assert_eq!(span.offset(), offset_of("bad/unknown_key.yaml", "chasis:"));
        }
        other => panic!("expected UnknownField, got {other:?}"),
    }
}

#[test]
fn yaml_anchor_is_rejected_at_parse() {
    let err = load_err("bad/anchor.yaml");
    assert!(
        matches!(err, SchemaError::Parse { .. }),
        "anchors must fail at parse, got {err:?}"
    );
}

#[test]
fn lsd_without_preload_is_a_semantic_error() {
    let err = load_err("bad/lsd_no_preload/vehicle.yaml");
    match err {
        SchemaError::Semantic { message, .. } => {
            assert!(message.contains("preload_nm"), "message: {message}");
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

#[test]
fn drive_unit_behind_gearbox_is_a_topology_error() {
    let err = load_err("bad/drive_unit_gearbox/vehicle.yaml");
    match err {
        SchemaError::Topology {
            message, labels, ..
        } => {
            assert!(message.contains("drive_unit"), "message: {message}");
            assert!(!labels.is_empty(), "topology error should carry spans");
        }
        other => panic!("expected Topology, got {other:?}"),
    }
}

#[test]
fn bad_soc_window_is_a_semantic_error() {
    let err = load_err("bad/bad_soc/vehicle.yaml");
    match err {
        SchemaError::Semantic { message, .. } => {
            assert!(message.contains("soc_window"), "message: {message}");
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

#[test]
fn unknown_mf61_coefficient_is_a_warning_not_an_error() {
    let (_, warnings) = load_tyr("bad/unknown_coeff.tyr.yaml", &loader())
        .expect("tyr with an unknown coeff still loads");
    assert!(
        warnings.iter().any(|w| w.detail.contains("PDX9")),
        "expected an unknown-coefficient warning: {warnings:?}"
    );
}

/// A `.tyr` document with the given schema string, MF6.1 body, and optional brush block. The
/// thermal/wear/provenance blocks are fixed boilerplate (the brush/force logic is what varies).
fn tyr_doc(schema: &str, mf61_body: &str, brush_block: &str) -> String {
    format!(
        "schema: {schema}\nmf61:\n{mf61_body}{brush_block}thermal:\n  c_s: 8000.0\n  c_c: 22000.0\n  \
         c_g: 1500.0\n  g_sc: 90.0\n  g_cg: 40.0\n  g_road: 250.0\n  h0: 15.0\n  h1: 5.5\n  \
         p_t: 0.65\n  t_opt: 95.0\n  c_t: 2.2\n  k_c: 0.0015\n  t_c_ref: 80.0\n  p_cold: 138.0\n  \
         t_cold: 20.0\nwear:\n  k_w: 0.0009\n  w_max: 8.0\n  w_c: 2.0\n  tau_d: 600.0\n  \
         t_deg: 120.0\n  delta_t_ref: 20.0\n  beta: 2.0\n  delta_c: 0.25\n  s_w: 0.5\n  \
         delta_d: 0.30\nprovenance:\n  citation: \"x\"\n  source: \"y\"\n  synthetic: true\n"
    )
}

const BRUSH_BLOCK: &str = "brush:\n  c_kappa_n: 150000.0\n  c_alpha_n_per_rad: 120000.0\n  \
                           mu0: 1.2\n  patch_half_length_m: 0.1\n";

#[test]
fn brush_under_tyr_1_0_warns() {
    // A brush block is a tyr/1.1 feature; declaring tyr/1.0 is a warning, not an error.
    let yaml = tyr_doc(
        "tyr/1.0",
        "  FNOMIN: 4000.0\n  UNLOADED_RADIUS: 0.33\n",
        BRUSH_BLOCK,
    );
    let l = MemLoader::new().with("t.tyr.yaml", yaml);
    let (_, warnings) = load_tyr("t.tyr.yaml", &l).expect("brush under 1.0 still loads");
    assert!(
        warnings
            .iter()
            .any(|w| w.detail.contains("requires schema `tyr/1.1`")),
        "expected a brush-minor warning: {warnings:?}"
    );
}

#[test]
fn partial_force_core_with_brush_warns() {
    // A brush block plus a partial (incomplete) MF6.1 force set → warning, and the brush is used.
    let yaml = tyr_doc(
        "tyr/1.1",
        "  FNOMIN: 4000.0\n  UNLOADED_RADIUS: 0.33\n  PCX1: 1.6\n  PDX1: 1.3\n",
        BRUSH_BLOCK,
    );
    let l = MemLoader::new().with("t.tyr.yaml", yaml);
    let (_, warnings) = load_tyr("t.tyr.yaml", &l).expect("partial force + brush still loads");
    assert!(
        warnings
            .iter()
            .any(|w| w.detail.contains("partial MF6.1 force")),
        "expected a partial-force warning: {warnings:?}"
    );
}

#[test]
fn partial_force_core_without_brush_is_an_error() {
    // The same partial force set WITHOUT a brush block is a hard semantic error.
    let yaml = tyr_doc(
        "tyr/1.1",
        "  FNOMIN: 4000.0\n  UNLOADED_RADIUS: 0.33\n  PCX1: 1.6\n  PDX1: 1.3\n",
        "",
    );
    let l = MemLoader::new().with("t.tyr.yaml", yaml);
    match load_tyr("t.tyr.yaml", &l).unwrap_err() {
        SchemaError::Semantic { message, .. } => {
            assert!(
                message.contains("PEX1") || message.contains("PCY1"),
                "message: {message}"
            );
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

#[test]
fn unknown_key_in_newer_minor_hints_at_schema_version() {
    // An unknown top-level key in a file that declares a newer MINOR than this build supports:
    // the error hint should point at the newer schema version rather than a bogus did-you-mean.
    let yaml = "schema: tyr/1.9\nwibble: 3\nmf61:\n  FNOMIN: 4000.0\n";
    let l = MemLoader::new().with("t.tyr.yaml", yaml);
    match load_tyr("t.tyr.yaml", &l).unwrap_err() {
        SchemaError::UnknownField { field, help, .. } => {
            assert_eq!(field, "wibble");
            let help = help.expect("a hint");
            assert!(help.contains("newer schema"), "help: {help}");
        }
        other => panic!("expected UnknownField, got {other:?}"),
    }
}

#[test]
fn type_mismatch_reports_path_and_span() {
    // mass_kg is a string where a number is required.
    let yaml = "\
schema: vehicle/1.0
name: bad types
chassis:
  mass_kg: \"heavy\"
  cg: [1.0, 0.0, 0.4]
  inertia: [500.0, 2000.0, 2200.0]
  wheelbase_m: 2.8
  track_m: [1.6, 1.6]
";
    let l = MemLoader::new().with("v.yaml", yaml);
    let err = load_vehicle("v.yaml", &l, &LoadOptions::default()).expect_err("should fail");
    match err {
        SchemaError::Deserialize { path, .. } => {
            assert!(
                path.contains("mass_kg"),
                "path should point at mass_kg: {path}"
            );
        }
        other => panic!("expected Deserialize, got {other:?}"),
    }
}

#[test]
fn same_major_minor_is_accepted_but_new_major_is_rejected() {
    let base = |schema: &str| {
        format!(
            "schema: {schema}\nname: v\nchassis:\n  mass_kg: 1000.0\n  cg: [1.0,0.0,0.4]\n  \
             inertia: [1.0,1.0,1.0]\n  wheelbase_m: 2.5\n  track_m: [1.5,1.5]\n"
        )
    };
    // A newer MINOR under the same MAJOR is accepted (it then fails later for missing fields,
    // NOT with a version error).
    let l = MemLoader::new().with("v.yaml", base("vehicle/1.9"));
    let err = load_vehicle("v.yaml", &l, &LoadOptions::default()).unwrap_err();
    assert!(
        !matches!(err, SchemaError::SchemaVersionMismatch { .. }),
        "1.9 should pass the gate"
    );

    // A new MAJOR is rejected at the version gate.
    let l = MemLoader::new().with("v.yaml", base("vehicle/2.0"));
    let err = load_vehicle("v.yaml", &l, &LoadOptions::default()).unwrap_err();
    assert!(
        matches!(err, SchemaError::SchemaVersionMismatch { .. }),
        "2.0 must be rejected"
    );

    // Wrong document kind is rejected.
    let l = MemLoader::new().with("v.yaml", base("ptm/1.0"));
    let err = load_vehicle("v.yaml", &l, &LoadOptions::default()).unwrap_err();
    assert!(
        matches!(err, SchemaError::SchemaVersionMismatch { .. }),
        "wrong kind must be rejected"
    );
}

#[test]
fn all_six_reference_topologies_resolve() {
    let l = loader();
    for path in [
        "ev_1du_rwd/vehicle.yaml",
        "ev_2du_awd/vehicle.yaml",
        "ev_4du_tv/vehicle.yaml",
        "fwd_hatch/vehicle.yaml",
        "gt_hybrid/vehicle.yaml",
        "f1_2026/vehicle.yaml",
    ] {
        load_vehicle(path, &l, &LoadOptions::default())
            .unwrap_or_else(|e| panic!("{path} topology should resolve: {e:?}"));
    }
}
