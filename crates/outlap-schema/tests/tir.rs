// SPDX-License-Identifier: AGPL-3.0-only
//! `.tir` codec tests: load/convert the synthetic fixture, the diagnostic surface (non-SI units,
//! duplicate key, malformed line, unknown section), byte-stable write∘parse idempotence, exact
//! coefficient round-trips, and property tests (random key subsets + parser panic-freedom).
// Coefficient values are exact decimals chosen to parse without rounding, so `==` is intentional.
#![allow(clippy::float_cmp, clippy::unreadable_literal)]

use std::collections::BTreeMap;

use outlap_schema::error::SchemaError;
use outlap_schema::io::FsLoader;
use outlap_schema::tir::{parse_tir, tir_to_tyr, tyr_to_tir, write_tir, TirToTyrOptions, TirValue};
use outlap_schema::tyr::{Mf61Coeffs, Tyr, TyrProvenance, TyrThermal, TyrWear, KNOWN_MF61_KEYS};
use outlap_schema::{load_tir, schema_name, SchemaVersion, SCHEMA_MAJOR};

use proptest::prelude::*;

fn loader() -> FsLoader {
    FsLoader::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures"))
}

fn read_fixture(rel: &str) -> String {
    std::fs::read_to_string(format!(
        "{}/tests/fixtures/{rel}",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap()
}

// --- Loading + conversion ---------------------------------------------------------------------

#[test]
fn synthetic_slick_loads_to_a_tyr() {
    let (tyr, warnings) = load_tir(
        "tir/synthetic_slick.tir",
        &loader(),
        &TirToTyrOptions::default(),
    )
    .unwrap();

    // Coefficients flatten out of their sections; metadata ([MODEL]/[UNITS]) does not.
    assert_eq!(tyr.mf61.0.get("PDX1"), Some(&1.30));
    assert_eq!(tyr.mf61.0.get("FNOMIN"), Some(&4000.0));
    assert_eq!(tyr.mf61.0.get("NOMPRES"), Some(&165000.0));
    assert_eq!(tyr.mf61.0.get("LONGITUDINAL_STIFFNESS"), Some(&300000.0));
    assert!(!tyr.mf61.0.contains_key("FITTYP"), "FITTYP is metadata");
    assert!(!tyr.mf61.0.contains_key("TYRESIDE"), "TYRESIDE is metadata");
    assert!(!tyr.mf61.0.contains_key("LENGTH"), "units are metadata");

    // `.tir` has no thermal/wear: they are synthesised and provenance is marked synthetic.
    assert!(tyr.provenance.synthetic);
    assert!(tyr.brush.is_none());
    assert!(
        warnings
            .iter()
            .any(|w| w.pointer == "/thermal" && w.detail.contains("synthesised")),
        "expected a thermal/wear synthesis note: {warnings:?}"
    );
}

#[test]
fn from_donor_policy_uses_donor_blocks_and_is_not_synthetic() {
    let (doc, _) = parse_tir("t", &read_fixture("tir/synthetic_slick.tir")).unwrap();
    let donor = sample_tyr(BTreeMap::new());
    let opts = TirToTyrOptions {
        thermal_wear: outlap_schema::tir::ThermalWearPolicy::FromDonor {
            thermal: Box::new(donor.thermal.clone()),
            wear: Box::new(donor.wear.clone()),
        },
    };
    let (tyr, _) = tir_to_tyr(&doc, &opts).unwrap();
    assert_eq!(tyr.thermal, donor.thermal);
    assert!(!tyr.provenance.synthetic, "donor data is not synthetic");
}

#[test]
fn none_policy_refuses_to_invent_thermal_wear() {
    let (doc, _) = parse_tir("t", &read_fixture("tir/synthetic_slick.tir")).unwrap();
    let opts = TirToTyrOptions {
        thermal_wear: outlap_schema::tir::ThermalWearPolicy::None,
    };
    match tir_to_tyr(&doc, &opts).unwrap_err() {
        SchemaError::Semantic { message, .. } => assert!(message.contains("thermal/wear")),
        other => panic!("expected Semantic, got {other:?}"),
    }
}

// --- Diagnostics ------------------------------------------------------------------------------

#[test]
fn non_si_units_is_a_hard_semantic_error_with_a_span() {
    let content = read_fixture("bad/non_si_units.tir");
    match parse_tir("bad/non_si_units.tir", &content).unwrap_err() {
        SchemaError::Semantic { message, span, .. } => {
            assert!(
                message.contains("LENGTH") && message.contains("meter"),
                "{message}"
            );
            // Span points at the offending `'mm'` token.
            assert_eq!(span.offset(), content.find("'mm'").unwrap());
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

#[test]
fn malformed_line_is_a_hard_parse_error() {
    let content = read_fixture("bad/malformed_line.tir");
    match parse_tir("bad/malformed_line.tir", &content).unwrap_err() {
        SchemaError::Parse { message, span, .. } => {
            assert!(message.contains("malformed"), "{message}");
            assert_eq!(span.offset(), content.find("this line").unwrap());
        }
        other => panic!("expected Parse, got {other:?}"),
    }
}

#[test]
fn duplicate_key_warns_and_keeps_the_last_value() {
    let content = read_fixture("bad/duplicate_key.tir");
    let (doc, warnings) = parse_tir("bad/duplicate_key.tir", &content).unwrap();
    assert!(
        warnings
            .iter()
            .any(|w| w.detail.contains("duplicate key `PDX1`")),
        "expected a duplicate-key warning: {warnings:?}"
    );
    // Last-wins: 1.45 not 1.30.
    let (tyr, _) = tir_to_tyr(&doc, &TirToTyrOptions::default()).unwrap();
    assert_eq!(tyr.mf61.0.get("PDX1"), Some(&1.45));
}

#[test]
fn unknown_section_warns_with_did_you_mean() {
    let content = read_fixture("bad/unknown_section.tir");
    let (_, warnings) = parse_tir("bad/unknown_section.tir", &content).unwrap();
    assert!(
        warnings
            .iter()
            .any(|w| w.detail.contains("unknown `.tir` section")
                && w.detail.contains("LONGITUDINAL_COEFFICIENTS")),
        "expected an unknown-section did-you-mean: {warnings:?}"
    );
}

#[test]
fn missing_required_force_core_is_an_error() {
    let text = "[VERTICAL]\nFNOMIN = 4000\n[DIMENSION]\nUNLOADED_RADIUS = 0.33\n";
    let loader = outlap_schema::io::MemLoader::new().with("t.tir", text);
    match load_tir("t.tir", &loader, &TirToTyrOptions::default()).unwrap_err() {
        SchemaError::Semantic { message, .. } => assert!(message.contains("PCX1")),
        other => panic!("expected Semantic, got {other:?}"),
    }
}

// --- Tolerances -------------------------------------------------------------------------------

#[test]
fn bom_and_crlf_are_tolerated() {
    let text = "\u{FEFF}[DIMENSION]\r\nUNLOADED_RADIUS = 0.33\r\n";
    let (doc, _) = parse_tir("t", text).unwrap();
    assert_eq!(
        doc.section("DIMENSION").unwrap().entries[0].value,
        TirValue::Number(0.33)
    );
}

// --- Round-trips ------------------------------------------------------------------------------

#[test]
fn write_then_parse_is_byte_stable() {
    // tir → doc → tir is byte-stable once through the canonical writer (comments/whitespace/order
    // are normalised on the first pass; every pass thereafter is identical).
    let content = read_fixture("tir/synthetic_slick.tir");
    let (doc, _) = parse_tir("t", &content).unwrap();
    let once = write_tir(&doc);
    let (doc2, _) = parse_tir("t", &once).unwrap();
    let twice = write_tir(&doc2);
    assert_eq!(once, twice, "canonical form is not a fixed point");
}

#[test]
fn tir_to_tyr_to_tir_is_numeric_exact_over_coefficients() {
    let content = read_fixture("tir/synthetic_slick.tir");
    let (doc, _) = parse_tir("t", &content).unwrap();
    let (tyr, _) = tir_to_tyr(&doc, &TirToTyrOptions::default()).unwrap();
    let doc2 = tyr_to_tir(&tyr);
    let (tyr2, _) = tir_to_tyr(&doc2, &TirToTyrOptions::default()).unwrap();
    assert_eq!(
        tyr.mf61.0, tyr2.mf61.0,
        "coefficient map changed on round-trip"
    );
}

#[test]
fn unknown_coefficient_round_trips_through_the_overflow_section() {
    // A coefficient not placed by the mapping table is emitted in `[USER_COEFFICIENTS]`,
    // canonicalised to uppercase, and re-parses to the same numeric value with no unknown-section
    // warning (the writer's own overflow section is recognised).
    let tyr = sample_tyr(BTreeMap::from([("customx".to_owned(), 3.5)]));
    let text = write_tir(&tyr_to_tir(&tyr));
    assert!(text.contains("[USER_COEFFICIENTS]"), "{text}");
    assert!(text.contains("CUSTOMX = 3.5"), "{text}");
    let (doc, warnings) = parse_tir("t", &text).unwrap();
    assert!(
        !warnings
            .iter()
            .any(|w| w.detail.contains("unknown `.tir` section")),
        "overflow section should re-parse cleanly: {warnings:?}"
    );
    let (back, _) = tir_to_tyr(&doc, &TirToTyrOptions::default()).unwrap();
    assert_eq!(back.mf61.0.get("CUSTOMX"), Some(&3.5));
    // Fixed point.
    assert_eq!(text, write_tir(&tyr_to_tir(&back)));
}

#[test]
fn parser_tolerates_multibyte_utf8() {
    // Non-ASCII bytes in comments and values must not panic or mis-slice.
    let text = "$ café — °C ½\n[DIMENSION]\nUNLOADED_RADIUS = 0.33 $ münchen\nNOTE = 'café'\n";
    let (doc, _) = parse_tir("t", text).unwrap();
    assert_eq!(
        doc.section("DIMENSION").unwrap().entries[0].value,
        TirValue::Number(0.33)
    );
}

// --- Property tests ---------------------------------------------------------------------------

fn sample_tyr(map: BTreeMap<String, f64>) -> Tyr {
    Tyr {
        schema: SchemaVersion::new(schema_name::TYR, SCHEMA_MAJOR, 0),
        mf61: Mf61Coeffs(map),
        brush: None,
        thermal: TyrThermal {
            c_s: 8000.0,
            c_c: 22000.0,
            c_g: 1500.0,
            g_sc: 90.0,
            g_cg: 40.0,
            g_road: 250.0,
            h0: 15.0,
            h1: 5.5,
            p_t: 0.65,
            t_opt: 95.0,
            c_t: 2.2,
            k_c: 0.0015,
            t_c_ref: 80.0,
            p_cold: 138.0,
            t_cold: 20.0,
        },
        wear: TyrWear {
            k_w: 0.0009,
            w_max: 8.0,
            w_c: 2.0,
            tau_d: 600.0,
            t_deg: 120.0,
            delta_t_ref: 20.0,
            beta: 2.0,
            delta_c: 0.25,
            s_w: 0.5,
            delta_d: 0.30,
        },
        provenance: TyrProvenance {
            citation: "x".into(),
            source: "y".into(),
            synthetic: true,
        },
    }
}

proptest! {
    /// A random subset of known coefficients, with arbitrary finite values, survives
    /// tyr → tir → text → tir → tyr numerically exactly, and the writer's text is a fixed point.
    #[test]
    fn coefficient_map_round_trips(
        pick in prop::collection::vec(any::<bool>(), KNOWN_MF61_KEYS.len()),
        vals in prop::collection::vec(
            any::<f64>().prop_filter("finite", |x| x.is_finite()),
            KNOWN_MF61_KEYS.len()),
    ) {
        let mut map = BTreeMap::new();
        for (i, key) in KNOWN_MF61_KEYS.iter().enumerate() {
            if pick[i] {
                map.insert((*key).to_owned(), vals[i]);
            }
        }
        let tyr = sample_tyr(map);
        let text = write_tir(&tyr_to_tir(&tyr));
        let (doc, _) = parse_tir("p", &text).unwrap();
        let (back, _) = tir_to_tyr(&doc, &TirToTyrOptions::default()).unwrap();
        prop_assert_eq!(&tyr.mf61.0, &back.mf61.0);
        // The writer's output is a fixed point.
        prop_assert_eq!(&text, &write_tir(&tyr_to_tir(&back)));
    }

    /// The parser never panics on arbitrary input drawn from the `.tir` character set, plus a
    /// sampling of multi-byte UTF-8 (guards against non-char-boundary slicing).
    #[test]
    fn parser_never_panics(s in r"[\[\]=A-Za-z0-9_.+\-'$! \r\né°½µ√]{0,400}") {
        let _ = parse_tir("fuzz", &s);
    }
}
