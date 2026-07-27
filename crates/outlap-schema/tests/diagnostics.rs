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

/// Load a fixture that is expected to resolve cleanly.
fn resolved(path: &str) -> outlap_schema::ResolvedVehicle {
    load_vehicle(path, &loader(), &LoadOptions::default())
        .unwrap_or_else(|e| panic!("{path} should have resolved: {e}"))
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
fn a_gearbox_on_a_units_private_path_is_rejected() {
    // A unit's `path` carries its differential ONLY — a gearbox belongs on the shared graph
    // (`drivetrain.couplers`), where every source below it shares its ratios. The rule is
    // structural and deliberately blind to what the `.ptm` calls itself (D-M6-13 Layer 3).
    let err = load_err("bad/drive_unit_gearbox/vehicle.yaml");
    match err {
        SchemaError::Semantic { message, help, .. } => {
            assert!(message.contains("gearbox"), "message: {message}");
            assert!(
                help.is_some_and(|h| h.contains("couplers")),
                "the fix names the shared graph"
            );
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

#[test]
fn a_fixed_ratio_on_a_units_private_path_is_rejected() {
    // There is exactly ONE way to declare a unit's internal reduction: the `fixed_ratio:` field.
    let err = load_err("bad/path_fixed_ratio/vehicle.yaml");
    match err {
        SchemaError::Semantic { message, help, .. } => {
            assert!(message.contains("fixed_ratio"), "message: {message}");
            assert!(
                help.is_some_and(|h| h.contains("`fixed_ratio:` field")),
                "the fix names the field"
            );
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

#[test]
fn a_units_fixed_ratio_is_folded_into_its_reduction_at_load() {
    // `fixed_ratio` declares that a unit's `.ptm` map is referenced to the MACHINE's own shaft
    // rather than the shaft it outputs onto, and supplies the ratio between them. The loader folds
    // it into the unit's private reduction once, at load — closest to the machine, ahead of the
    // diff — so every tier downstream sees one consistent output-shaft frame and no per-step code
    // ever knows the ratio existed.
    use outlap_schema::vehicle::Coupler;

    let sugar = resolved("fixed_ratio_sugar/vehicle.yaml");
    let unit = &sugar.spec.drivetrain.units[0];
    assert!(
        unit.fixed_ratio.is_none(),
        "the field is consumed by the fold, not left dangling"
    );
    assert!(
        matches!(unit.path.first(), Some(Coupler::FixedRatio(r)) if (*r - 3.5).abs() < 1e-12),
        "the ratio is folded in closest to the machine: {:?}",
        unit.path
    );
    assert!(
        matches!(unit.path.get(1), Some(Coupler::Diff(_))),
        "the declared diff follows it: {:?}",
        unit.path
    );
}

#[test]
fn a_non_positive_fixed_ratio_is_a_semantic_error() {
    let err = load_err("bad/fixed_ratio_zero/vehicle.yaml");
    match err {
        SchemaError::Semantic { message, .. } => {
            assert!(message.contains("fixed_ratio"), "message: {message}");
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

#[test]
fn bad_soc_window_is_a_semantic_error() {
    // The bad window now lives on the referenced pack document (D-M6-13 single source of truth).
    let err = load_err("bad/bad_soc/vehicle.yaml");
    match err {
        SchemaError::Semantic { message, .. } => {
            assert!(message.contains("soc_window"), "message: {message}");
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

/// A pre-2.0 (`ers:` / singular `battery:`) layout hard-fails with the curated legacy diagnostic
/// (there is no `outlap migrate`) — the D-M6-13 legacy sniff.
#[test]
fn a_legacy_ers_layout_is_rejected() {
    let err = load_err("bad/legacy_ers/vehicle.yaml");
    match err {
        SchemaError::LegacyDrivetrainFormat { message, help, .. } => {
            assert!(message.contains("ers"), "names the legacy key: {message}");
            let help = help.expect("a pointer to the new layout");
            assert!(
                help.contains("policy") && help.contains("batteries"),
                "help points at the new shape: {help}"
            );
        }
        other => panic!("expected LegacyDrivetrainFormat, got {other:?}"),
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
schema: vehicle/2.0
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
    // A newer MINOR under the same MAJOR (vehicle is on MAJOR 2 now) is accepted (it then fails
    // later for missing fields, NOT with a version error).
    let l = MemLoader::new().with("v.yaml", base("vehicle/2.9"));
    let err = load_vehicle("v.yaml", &l, &LoadOptions::default()).unwrap_err();
    assert!(
        !matches!(err, SchemaError::SchemaVersionMismatch { .. }),
        "2.9 should pass the gate"
    );

    // The old MAJOR (1.x) is rejected at the version gate (D-M6-13 bumped vehicle to 2).
    let l = MemLoader::new().with("v.yaml", base("vehicle/1.9"));
    let err = load_vehicle("v.yaml", &l, &LoadOptions::default()).unwrap_err();
    assert!(
        matches!(err, SchemaError::SchemaVersionMismatch { .. }),
        "1.9 must be rejected"
    );

    // A future MAJOR is rejected too.
    let l = MemLoader::new().with("v.yaml", base("vehicle/3.0"));
    let err = load_vehicle("v.yaml", &l, &LoadOptions::default()).unwrap_err();
    assert!(
        matches!(err, SchemaError::SchemaVersionMismatch { .. }),
        "3.0 must be rejected"
    );

    // Wrong document kind is rejected.
    let l = MemLoader::new().with("v.yaml", base("ptm/2.0"));
    let err = load_vehicle("v.yaml", &l, &LoadOptions::default()).unwrap_err();
    assert!(
        matches!(err, SchemaError::SchemaVersionMismatch { .. }),
        "wrong kind must be rejected"
    );
}

/// A `ptm/1.x` file hits the version gate (ptm/2.0 renamed the kinds — the regen-honesty break),
/// the ptm twin of the vehicle 1.x witness above: a version error, never a bare deserialize
/// failure from whatever field changed underneath.
#[test]
fn a_ptm_1x_file_is_rejected_at_the_version_gate() {
    for schema in ["ptm/1.0", "ptm/1.2"] {
        let ptm = format!(
            "schema: {schema}\nkind: electric\n\
             axes: {{speed_rpm: [0.0, 8000.0], load_axis: {{torque_nm: [0.0, 300.0]}}, torque_nm: [0.0, 300.0]}}\n\
             tables: {{file: x.parquet}}\n\
             limits: {{max_torque_nm_vs_speed: {{speed_rpm: [0.0, 8000.0], torque_nm: [300.0, 300.0]}}}}\n\
             inertia_kgm2: 0.1\nmass_kg: 80.0\n"
        );
        let loader = MemLoader::new().with("u.ptm.yaml", &ptm);
        let err = outlap_schema::load::load_ptm("u.ptm.yaml", &loader)
            .expect_err("a ptm/1.x file must be rejected at the version gate");
        assert!(
            matches!(err, SchemaError::SchemaVersionMismatch { .. }),
            "{schema} must fail the version gate, got: {err}"
        );
    }
}

/// ptm/2.0 has NO aliases for the retired kind names — auto-mapping `drive_unit` to either side is
/// exactly how two petrol engines silently inherited a regen envelope. The rejection must name
/// both live candidates so the author knows which honest label to pick.
#[test]
fn old_ptm_kind_names_are_rejected_naming_both_candidates() {
    for old in ["drive_unit", "ice", "electric_machine"] {
        let ptm = format!(
            "schema: ptm/2.0\nkind: {old}\n\
             axes: {{speed_rpm: [0.0, 8000.0], load_axis: {{torque_nm: [0.0, 300.0]}}, torque_nm: [0.0, 300.0]}}\n\
             tables: {{file: x.parquet}}\n\
             limits: {{max_torque_nm_vs_speed: {{speed_rpm: [0.0, 8000.0], torque_nm: [300.0, 300.0]}}}}\n\
             inertia_kgm2: 0.1\nmass_kg: 80.0\n"
        );
        let loader = MemLoader::new().with("u.ptm.yaml", &ptm);
        let err = outlap_schema::load::load_ptm("u.ptm.yaml", &loader)
            .expect_err("an old kind name must be rejected");
        let msg = format!("{err}");
        assert!(
            msg.contains(old) && msg.contains("combustion") && msg.contains("electric"),
            "`kind: {old}` must be rejected naming both candidates: {msg}"
        );
    }
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

// --- ptm/1.2 regen envelope + battery/1.1 charge derate (series regen blend) ---------------------

/// The regen envelope is a *positive magnitude*. Signing it negative is the obvious authoring
/// mistake, so the message says exactly what to do rather than just "invalid".
#[test]
fn a_negative_regen_envelope_is_rejected_with_a_plain_language_fix() {
    let ptm = "schema: ptm/2.0\nkind: electric\n\
        axes: {speed_rpm: [0.0, 8000.0], load_axis: {torque_nm: [-300.0, 300.0]}, torque_nm: [-300.0, 300.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 8000.0], torque_nm: [300.0, 300.0]}, max_regen_torque_nm_vs_speed: {speed_rpm: [0.0, 8000.0], torque_nm: [-120.0, -120.0]}}\n\
        inertia_kgm2: 0.1\nmass_kg: 80.0\n";
    let loader = MemLoader::new().with("u.ptm.yaml", ptm);
    let err = outlap_schema::load::load_ptm("u.ptm.yaml", &loader)
        .expect_err("a negative regen envelope must be rejected");
    let msg = format!("{err}");
    assert!(
        msg.contains("positive-magnitude") && msg.contains("minus sign"),
        "the message must tell the author what to do: {msg}"
    );
}

/// `kind: combustion` has no regenerative quadrant (ptm/2.0: the kind IS the energy source), so a
/// declared regen envelope on one is always meaningless — a hard error naming the contradiction
/// (Decision #40), never a silent discard.
#[test]
fn a_combustion_map_with_a_declared_regen_envelope_is_rejected() {
    let ptm = "schema: ptm/2.0\nkind: combustion\n\
        axes: {speed_rpm: [1000.0, 8000.0], load_axis: {torque_nm: [0.0, 300.0]}, torque_nm: [0.0, 300.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [1000.0, 8000.0], torque_nm: [280.0, 300.0]}, max_regen_torque_nm_vs_speed: {speed_rpm: [1000.0, 8000.0], torque_nm: [120.0, 120.0]}}\n\
        inertia_kgm2: 0.1\nmass_kg: 120.0\n";
    let loader = MemLoader::new().with("u.ptm.yaml", ptm);
    let err = outlap_schema::load::load_ptm("u.ptm.yaml", &loader)
        .expect_err("a combustion map declaring a regen envelope must be rejected");
    let msg = format!("{err}");
    assert!(
        msg.contains("regenerative quadrant"),
        "the message names the contradiction: {msg}"
    );
}

/// A derate factor scales the declared ceiling, so it may not exceed 1 — that would silently *raise*
/// the pack's charge acceptance above what the vehicle file declares.
#[test]
fn a_charge_derate_factor_above_one_is_rejected() {
    let doc = battery_doc("temp_c: [0.0, 25.0]\n    factor: [0.0, 1.5]");
    let loader = MemLoader::new().with("p.battery.yaml", &doc);
    let err = outlap_schema::load_battery("p.battery.yaml", &loader)
        .expect_err("a derate factor > 1 must be rejected");
    assert!(
        format!("{err}").contains("[0, 1]"),
        "the bound is named: {err}"
    );
}

/// Temperature breakpoints must ascend, or the interpolant is meaningless.
#[test]
fn a_charge_derate_with_descending_temperatures_is_rejected() {
    let doc = battery_doc("temp_c: [25.0, 0.0]\n    factor: [1.0, 0.0]");
    let loader = MemLoader::new().with("p.battery.yaml", &doc);
    let err = outlap_schema::load_battery("p.battery.yaml", &loader)
        .expect_err("descending temp_c must be rejected");
    assert!(
        format!("{err}").contains("ascending"),
        "the message names the ordering requirement: {err}"
    );
}

/// A well-formed derate curve loads cleanly (the happy path this validation must not break).
#[test]
fn a_well_formed_charge_derate_loads() {
    let doc = battery_doc("temp_c: [-20.0, 0.0, 25.0]\n    factor: [0.0, 0.0, 1.0]");
    let loader = MemLoader::new().with("p.battery.yaml", &doc);
    let b = outlap_schema::load_battery("p.battery.yaml", &loader).expect("valid derate curve");
    let d = b.limits.regen_derate_vs_temp.expect("curve present");
    assert_eq!(d.factor, vec![0.0, 0.0, 1.0]);
}

/// A minimal `battery/1.1` document with the given `regen_derate_vs_temp` body spliced in.
fn battery_doc(derate: &str) -> String {
    format!(
        "schema: battery/1.1\nmodel: rc_pairs\n\
         topology: {{ns: 100, np: 1}}\n\
         capacity: {{q_pack_ah: 50.0, e_pack_wh: 18000.0}}\n\
         soc_window: [0.05, 0.95]\n\
         ecm:\n  rc_pairs: 1\n  axes: {{soc: [0.0, 1.0], temp_c: [0.0, 45.0]}}\n  \
         tables: {{file: t.parquet, level: cell}}\n\
         limits:\n  \
         peak_discharge_power_w_vs_soc: {{soc: [0.0, 1.0], power_w: [1000.0, 2000.0]}}\n  \
         peak_regen_power_w_vs_soc: {{soc: [0.0, 1.0], power_w: [1000.0, 1000.0]}}\n  \
         regen_derate_vs_temp:\n    {derate}\n  \
         cell_v_min: 2.8\n  cell_v_max: 4.2\n  max_c_rate: 3.0\n\
         thermal: {{mass_kg: 400.0, cp_j_per_kgk: 900.0, thermal_resistance_k_per_w: 0.02, coolant_temp_c: 25.0}}\n"
    )
}

// --- vehicle/1.7 ERS block validation (M6/PR1: the consumer lands the checks) --------------------

/// A rising `power_frac` grants MORE power at higher speed than the declared limit — always
/// meaningless for a de-rate curve, so it is rejected (the `SpeedTaper` doc-comment promise,
/// enforced since the rulebook consumes the taper).
#[test]
fn a_rising_taper_power_frac_is_rejected() {
    use outlap_schema::load::{resolve_vehicle, Overrides};
    let l = loader();
    let base = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves")
        .spec;
    let mut broken = base;
    // Drop the first knot below the second so the taper RISES out of the plateau. Mutating in
    // place keeps the arity the fixture happens to declare, so this stays about monotonicity
    // rather than tripping the equal-length check when the taper gains a breakpoint.
    broken
        .policy
        .as_mut()
        .expect("f1_2026 has policy")
        .deployment
        .taper_vs_speed
        .power_frac[0] = 0.5;
    let err = resolve_vehicle(&broken, &Overrides::default(), &l, &LoadOptions::default())
        .expect_err("a rising taper must be rejected");
    assert!(
        format!("{err}").contains("monotone non-increasing"),
        "message should name the property: {err}"
    );
}

/// A `recharge_target_soc` outside the usable `soc_window` can never be reached — rejected with
/// the window bounds in the message.
#[test]
fn a_recharge_target_outside_the_soc_window_is_rejected() {
    use outlap_schema::load::{resolve_vehicle, Overrides};
    let l = loader();
    let base = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves")
        .spec;
    let mut broken = base;
    broken.policy.as_mut().unwrap().recovery.recharge_target_soc = Some(0.95); // window is [0.2, 0.9]
    let err = resolve_vehicle(&broken, &Overrides::default(), &l, &LoadOptions::default())
        .expect_err("a target outside the window must be rejected");
    assert!(
        format!("{err}").contains("recharge_target_soc"),
        "message should name the field: {err}"
    );
}

/// `per_lap_deploy_mj` is never estimated (M6/PR1, D-M6-5): an absent value means "unbounded"
/// — the old `= capacity_mj` heuristic would become a phantom cap once budgets are enforced.
#[test]
fn per_lap_deploy_budget_is_not_estimated() {
    let l = loader();
    let resolved = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves");
    assert!(
        resolved
            .spec
            .policy
            .as_ref()
            .expect("f1_2026 has policy")
            .deployment
            .per_lap_deploy_mj
            .is_none(),
        "an absent deploy budget must stay absent"
    );
    assert!(
        !resolved
            .report
            .estimated
            .iter()
            .any(|e| e.pointer.contains("per_lap_deploy")),
        "no estimation row for the deploy budget"
    );
}

/// The new optional vehicle/1.7 ERS fields load, validate, and round-trip.
#[test]
fn the_new_ers_recovery_fields_load() {
    use outlap_schema::load::{resolve_vehicle, Overrides};
    let l = loader();
    let base = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves")
        .spec;
    let mut spec = base;
    {
        let policy = spec.policy.as_mut().unwrap();
        policy.elec_mech_factor = Some(0.97);
        policy.recovery.recharge_target_soc = Some(0.55);
        policy.recovery.ramp_initial_step_kw = Some(150.0);
        policy.recovery.ramp_rate_kw_per_s = Some(50.0);
        policy.recovery.ramp_total_kw = Some(700.0);
    }
    let resolved = resolve_vehicle(&spec, &Overrides::default(), &l, &LoadOptions::default())
        .expect("the new optional fields validate");
    let policy = resolved.spec.policy.expect("policy present");
    assert_eq!(policy.recovery.recharge_target_soc, Some(0.55));
    assert_eq!(policy.elec_mech_factor, Some(0.97));
}

// --- M6 PR2: the ers↔battery integrity contract ----------------------------------------------

/// A `MemLoader` carrying every side file the `gt_hybrid` fixture references, with its
/// `vehicle.yaml` swapped for `vehicle` and the battery document optionally omitted or replaced.
fn gt_loader(vehicle: &str, battery: Option<&str>) -> MemLoader {
    let slick = include_str!("fixtures/tyr/slick.tyr.yaml");
    let mgu = include_str!("fixtures/ptm/mgu_k.ptm.yaml");
    let ice = include_str!("fixtures/ptm/ice_v6.ptm.yaml");
    let mut l = MemLoader::new()
        .with("vehicle.yaml", vehicle)
        .with("tyr/slick.tyr.yaml", slick)
        .with("ptm/mgu_k.ptm.yaml", mgu)
        .with("ptm/ice_v6.ptm.yaml", ice);
    if let Some(b) = battery {
        l = l.with("battery/gt_es.yaml", b);
    }
    l
}

const GT_VEHICLE: &str = include_str!("fixtures/gt_hybrid/vehicle.yaml");
const GT_BATTERY: &str = include_str!("fixtures/battery/gt_es.yaml");

/// The energy manager schedules the pack: an `ers:`-bearing vehicle whose battery document is
/// missing must fail loudly — unless `allow_degraded` downgrades it to a RECORDED degradation.
#[test]
fn an_ers_vehicle_with_a_missing_battery_file_is_gated() {
    let l = gt_loader(GT_VEHICLE, None);
    let err = load_vehicle("vehicle.yaml", &l, &LoadOptions::default())
        .expect_err("missing battery on an ers car must be a hard error");
    let msg = format!("{err}");
    assert!(
        msg.contains("not found") && msg.contains("pack"),
        "the message explains the contract: {msg}"
    );

    let resolved = load_vehicle(
        "vehicle.yaml",
        &l,
        &LoadOptions {
            allow_degraded: true,
        },
    )
    .expect("allow_degraded downgrades the missing battery to a recorded degradation");
    assert!(
        resolved
            .report
            .degraded
            .iter()
            .any(|e| e.pointer.contains("batteries")),
        "the degradation is recorded, nothing silent"
    );
}

/// A `policy:` overlay governing a unit whose `battery:` pack has no `batteries:` entry is a
/// contract violation — the governed pack must exist (caught at the semantic id resolution).
#[test]
fn an_ers_vehicle_without_a_battery_block_is_gated() {
    let stripped = GT_VEHICLE.replace(
        "batteries:\n  gt_es:\n    model: rc_pairs\n    params: battery/gt_es.yaml\n",
        "",
    );
    assert!(!stripped.contains("batteries:"), "block stripped");
    let l = gt_loader(&stripped, Some(GT_BATTERY));
    let err = load_vehicle("vehicle.yaml", &l, &LoadOptions::default())
        .expect_err("a governed unit with no batteries map must be a hard error");
    assert!(
        format!("{err}").contains("battery"),
        "the missing pack is named: {err}"
    );
    // An unresolved in-document `battery:` id is a semantic contract error, not a degradable
    // missing file — `allow_degraded` does not paper over it (the id simply doesn't resolve).
    let err = load_vehicle(
        "vehicle.yaml",
        &l,
        &LoadOptions {
            allow_degraded: true,
        },
    )
    .expect_err("an unresolved battery id stays a hard error");
    assert!(format!("{err}").contains("battery"));
}

/// `policy.regulatory_window_mj` is the FIA C5.2.9 on-track swing limit; it must FIT WITHIN the battery's
/// physical usable-window energy `(window span) × e_pack_wh` — a swing that draws more than the
/// store physically holds is a vehicle-level declaration error.
#[test]
fn an_ers_swing_limit_over_the_physical_window_is_rejected() {
    let bad = GT_VEHICLE.replace("regulatory_window_mj: 2.0", "regulatory_window_mj: 2.5");
    let l = gt_loader(&bad, Some(GT_BATTERY));
    let err = load_vehicle("vehicle.yaml", &l, &LoadOptions::default())
        .expect_err("a swing limit above the physical window must be rejected");
    let msg = format!("{err}");
    assert!(
        msg.contains("regulatory_window_mj") && msg.contains("e_pack_wh"),
        "both sides of the disagreement are named: {msg}"
    );
}

/// The C5.2.9 swing limit is INDEPENDENT of the physical window: a reg limit SMALLER than the
/// pack's usable-window energy is valid (the swing is then clipped below the physical edge). The
/// old `= window` heuristic would have rejected this.
#[test]
fn a_swing_limit_below_the_physical_window_is_accepted() {
    let smaller = GT_VEHICLE.replace("regulatory_window_mj: 2.0", "regulatory_window_mj: 1.5");
    let l = gt_loader(&smaller, Some(GT_BATTERY));
    let resolved = load_vehicle("vehicle.yaml", &l, &LoadOptions::default())
        .expect("a reg swing limit below the physical window is valid (independent of it)");
    assert!((resolved.spec.policy.as_ref().unwrap().regulatory_window_mj - 1.5).abs() < 1e-9);
}

// (The pre-2.0 ers.es↔battery `soc_window` reconciliation test is gone: under D-M6-13 the pack
// document is the SINGLE source of truth, so there is no duplicate window to disagree.)

/// A NON-ers vehicle whose battery file is absent stays CLEAN (no `degraded` entry without
/// `allow_degraded`) — the electro coupling is simply inert and the binding notes it. A
/// `degraded` entry is the mark of an opted-into fallback run, never a silent side effect.
#[test]
fn a_non_ers_vehicle_with_a_missing_battery_stays_clean() {
    // Strip the `ers:` block from the gt_hybrid fixture → a plain battery car (no manager),
    // and point the battery at a missing file. It must load clean (no degraded entry).
    let no_policy = strip_policy_block(GT_VEHICLE)
        .replace("params: battery/gt_es.yaml", "params: battery/missing.yaml");
    assert!(!no_policy.contains("policy:"), "policy block stripped");
    let l = gt_loader(&no_policy, None); // no battery doc supplied ⇒ NotFound
    let resolved = load_vehicle("vehicle.yaml", &l, &LoadOptions::default())
        .expect("a non-ers car with an absent battery still resolves");
    assert!(resolved.spec.policy.is_none(), "control: no policy block");
    assert!(
        resolved.report.degraded.is_empty(),
        "a non-ers car with a missing battery must NOT record a degradation without \
         allow_degraded: {:?}",
        resolved.report.degraded
    );
}

// --- D-M6-13 Layer-1 hardening: policy/graph/pack validation guards -------------------------------

/// `policy.governs` must name at least one unit — a policy that governs nothing would build an
/// energy manager that can never deploy or harvest (and freezes an EV's state of charge), so it
/// is rejected.
#[test]
fn an_empty_governs_is_rejected() {
    use outlap_schema::load::{resolve_vehicle, Overrides};
    let l = loader();
    let mut broken = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves")
        .spec;
    broken.policy.as_mut().expect("f1 has policy").governs = vec![];
    let err = resolve_vehicle(&broken, &Overrides::default(), &l, &LoadOptions::default())
        .expect_err("an empty governs must be rejected");
    match err {
        SchemaError::Semantic { message, .. } => {
            assert!(message.contains("governs"), "message: {message}");
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

/// This build evaluates a single governed machine; a second `governs` entry would be dropped from
/// the mechanical ceilings yet never force-added (silently losing a machine), so it is rejected.
#[test]
fn a_multi_unit_governs_is_rejected() {
    use outlap_schema::load::{resolve_vehicle, Overrides};
    use outlap_schema::refs::UnitId;
    let l = loader();
    let mut broken = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves")
        .spec;
    broken.policy.as_mut().unwrap().governs = vec![UnitId::from("mguk"), UnitId::from("ice")];
    let err = resolve_vehicle(&broken, &Overrides::default(), &l, &LoadOptions::default())
        .expect_err("a multi-unit governs must be rejected");
    match err {
        SchemaError::Semantic { message, .. } => {
            assert!(
                message.contains("single governed machine"),
                "message: {message}"
            );
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

/// Governing a NON-electric unit (one with no `battery:`, e.g. the ICE) is rejected — only an
/// electric machine that references a pack can be governed. Re-establishes the pre-2.0 "the ERS
/// needs a store" hard error the typed `ers.mgu_k` block enforced by shape.
#[test]
fn governing_a_non_electric_unit_is_rejected() {
    use outlap_schema::load::{resolve_vehicle, Overrides};
    use outlap_schema::refs::UnitId;
    let l = loader();
    let mut broken = load_vehicle("f1_2026/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves")
        .spec;
    broken.policy.as_mut().unwrap().governs = vec![UnitId::from("ice")]; // the ICE has no battery
    let err = resolve_vehicle(&broken, &Overrides::default(), &l, &LoadOptions::default())
        .expect_err("governing a non-electric unit must be rejected");
    match err {
        SchemaError::Semantic { message, .. } => {
            assert!(
                message.contains("battery") && message.contains("electric"),
                "message: {message}"
            );
        }
        other => panic!("expected Semantic, got {other:?}"),
    }
}

/// A branching (fan-out) drivetrain graph — one node feeding more than one coupler — would have its
/// branch ratios multiplied in series and applied to every wheel, so it is rejected at topology.
#[test]
fn a_fan_out_drivetrain_graph_is_rejected() {
    use outlap_schema::load::{resolve_vehicle, Overrides};
    use outlap_schema::vehicle::{Coupler, CouplerEdge, Wheel};
    let l = loader();
    let mut broken = load_vehicle("gt_hybrid/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves")
        .spec;
    // Add a SECOND coupler leaving the shared `crank` node (it already feeds the gearbox), so the
    // crank now has out-degree 2 — a fan-out.
    let crank = broken
        .drivetrain
        .units
        .iter()
        .find_map(|u| u.output.clone())
        .expect("gt_hybrid outputs onto a shared node");
    broken.drivetrain.couplers.push(CouplerEdge {
        coupler: Coupler::FixedRatio(2.0),
        from: crank,
        to: None,
        wheels: vec![Wheel::Fl, Wheel::Fr],
    });
    let err = resolve_vehicle(&broken, &Overrides::default(), &l, &LoadOptions::default())
        .expect_err("a fan-out graph must be rejected");
    match err {
        SchemaError::Topology { message, .. } => {
            assert!(message.contains("fan-out"), "message: {message}");
        }
        other => panic!("expected Topology, got {other:?}"),
    }
}

/// An unknown key INSIDE a `batteries` map entry must be a hard error (the id-keyed map values are
/// walked through `additionalProperties` now, so typos in a pack entry are not silently dropped).
#[test]
fn an_unknown_key_inside_a_battery_entry_is_rejected() {
    let with_typo = GT_VEHICLE.replace(
        "    model: rc_pairs\n    params: battery/gt_es.yaml",
        "    model: rc_pairs\n    bogus_key: 1\n    params: battery/gt_es.yaml",
    );
    assert!(with_typo.contains("bogus_key"), "typo spliced in");
    let l = gt_loader(&with_typo, Some(GT_BATTERY));
    match load_vehicle("vehicle.yaml", &l, &LoadOptions::default())
        .expect_err("an unknown key inside a pack entry must be rejected")
    {
        SchemaError::UnknownField { field, .. } => assert_eq!(field, "bogus_key"),
        other => panic!("expected UnknownField, got {other:?}"),
    }
}

/// The solver marches a SINGLE pack, so a car whose units reference more than one distinct pack is
/// gated (a hard error) — unless `allow_degraded` records the single-pack collapse.
#[test]
fn a_multi_pack_car_is_gated() {
    use outlap_schema::load::{resolve_vehicle, Overrides};
    use outlap_schema::refs::BatteryId;
    let l = loader();
    let base = load_vehicle("gt_hybrid/vehicle.yaml", &l, &LoadOptions::default())
        .expect("fixture resolves")
        .spec;
    let mut broken = base;
    // Add a second pack (reusing the same on-disk params so it is loadable) and point the ICE at
    // it, so two distinct packs are referenced.
    let pack = broken.batteries.values().next().cloned().expect("a pack");
    broken.batteries.insert(BatteryId::from("gt_es2"), pack);
    broken.drivetrain.units[0].battery = Some(BatteryId::from("gt_es2"));
    let err = resolve_vehicle(&broken, &Overrides::default(), &l, &LoadOptions::default())
        .expect_err("distinct packs per unit must be gated");
    assert!(
        format!("{err}").contains("distinct battery packs"),
        "message: {err}"
    );
    // allow_degraded runs it collapsed-to-primary, with the collapse recorded (nothing silent).
    let resolved = resolve_vehicle(
        &broken,
        &Overrides::default(),
        &l,
        &LoadOptions {
            allow_degraded: true,
        },
    )
    .expect("allow_degraded collapses to the primary pack");
    assert!(
        resolved
            .report
            .degraded
            .iter()
            .any(|e| e.pointer.contains("batteries")),
        "the collapse is recorded: {:?}",
        resolved.report.degraded
    );
}

/// A legacy `ers:` block inherited through an `extends:` PARENT (not present in the root tree) must
/// still yield the curated legacy diagnostic — the merged-tree re-sniff, not a generic error.
#[test]
fn a_legacy_block_inherited_through_extends_is_rejected() {
    let child = "schema: vehicle/2.0\nname: child\nextends: parent.yaml\n";
    let parent = "schema: vehicle/2.0\nname: parent\ners:\n  mgu_k: {}\n";
    let l = MemLoader::new()
        .with("child.yaml", child)
        .with("parent.yaml", parent);
    match load_vehicle("child.yaml", &l, &LoadOptions::default())
        .expect_err("a legacy block in an extends parent must be rejected")
    {
        SchemaError::LegacyDrivetrainFormat { message, .. } => {
            assert!(message.contains("ers"), "names the legacy key: {message}");
        }
        other => panic!("expected LegacyDrivetrainFormat, got {other:?}"),
    }
}

/// Remove the top-level `ers:` block (up to the next top-level `battery:` key) from a vehicle
/// YAML string — a test helper for the no-manager battery cases.
fn strip_policy_block(vehicle: &str) -> String {
    let mut out = String::new();
    let mut skipping = false;
    for line in vehicle.lines() {
        if line.starts_with("policy:") {
            skipping = true;
            continue;
        }
        // A new top-level key (no leading whitespace, not a comment) ends the ers block.
        if skipping && !line.is_empty() && !line.starts_with(char::is_whitespace) {
            skipping = false;
        }
        if !skipping {
            out.push_str(line);
            out.push('\n');
        }
    }
    out
}
