// SPDX-License-Identifier: AGPL-3.0-only
//! Rulebook tests: the piecewise-linear tapers against the CLOSED-FORM FIA C5.2.8 formulas at
//! random interior speeds (this is exactly the test a Hermite-through-breakpoints fails), knee
//! exactness, override dominance, and the `gt_hybrid` Option paths (D-M6-12).
// Bit-exactness IS the assertion in several tests (knee exactness, edge clamps); the closed-form
// helpers mirror the regulation text verbatim rather than clippy's preferred clamp spelling.
#![allow(
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::manual_clamp,
    clippy::manual_midpoint,
    clippy::cast_possible_truncation
)]

mod common;

use common::{f1_ers, gt_ers, TestRng};
use outlap_powertrain::{ErsRulebook, RulebookError};

const KPH_TO_MPS: f64 = 1.0 / 3.6;

/// C5.2.8(i): min(350 kW, 1800 − 5·v, 6900 − 20·v for v ≥ 340), zero at ≥ 345 (kW over kph).
fn deploy_closed_form_w(v_kph: f64) -> f64 {
    if v_kph >= 345.0 {
        return 0.0;
    }
    let curve = if v_kph < 340.0 {
        1800.0 - 5.0 * v_kph
    } else {
        6900.0 - 20.0 * v_kph
    };
    curve.min(350.0).max(0.0) * 1e3
}

/// C5.2.8(ii): min(350 kW, 7100 − 20·v), zero at ≥ 355 (full power to exactly 337.5 kph).
fn override_closed_form_w(v_kph: f64) -> f64 {
    if v_kph >= 355.0 {
        return 0.0;
    }
    (7100.0 - 20.0 * v_kph).min(350.0).max(0.0) * 1e3
}

/// The D-M6-5 breakpoints reproduce the C5.2.8 closed forms at random INTERIOR speeds — not just
/// at the knots. The shared Hermite bows the [290, 340] segment up to +78 kW at 315 kph (the flat
/// 0–290 plateau forces a zero tangent at 290), which is why PR1's piecewise-LINEAR evaluation is
/// mandatory, not cosmetic.
#[test]
fn taper_matches_the_closed_form_at_interior_speeds() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let mut rng = TestRng::new(0x5EED_0001);
    for _ in 0..10_000 {
        let v_kph = rng.range(0.0, 400.0);
        let v = v_kph * KPH_TO_MPS;
        let expect = deploy_closed_form_w(v_kph);
        let got = rb.deploy_cap_electrical_w(v, false);
        assert!(
            (got - expect).abs() <= 1e-6 * 350e3,
            "deploy taper diverges from C5.2.8(i) at {v_kph:.3} kph: got {got:.1} W, reg {expect:.1} W"
        );
        let expect_ov = override_closed_form_w(v_kph);
        let got_ov = rb.deploy_cap_electrical_w(v, true);
        assert!(
            (got_ov - expect_ov).abs() <= 1e-6 * 350e3,
            "override taper diverges from C5.2.8(ii) at {v_kph:.3} kph: got {got_ov:.1} W, reg {expect_ov:.1} W"
        );
    }
}

/// The knee is EXACTLY 100 kW at 340 km/h — 350·(2/7), the fraction, never a truncated decimal.
#[test]
fn knee_is_exactly_two_sevenths() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let at_knee = rb.deploy_cap_electrical_w(340.0 * KPH_TO_MPS, false);
    assert_eq!(at_knee, 350e3 * (2.0 / 7.0), "knee must be exact");
    assert!(
        (at_knee - 100e3).abs() < 1e-6,
        "2/7 of 350 kW is 100 kW to rounding, got {at_knee}"
    );
    // Full power holds to exactly 290 kph (the min(cap, curve) plateau made explicit).
    assert_eq!(rb.deploy_cap_electrical_w(290.0 * KPH_TO_MPS, false), 350e3);
    // Zero at and beyond the 345 kph cut.
    assert_eq!(rb.deploy_cap_electrical_w(345.0 * KPH_TO_MPS, false), 0.0);
    assert_eq!(rb.deploy_cap_electrical_w(360.0 * KPH_TO_MPS, false), 0.0);
}

/// Override grants at least the normal envelope at every speed (C5.2.8(ii) holds full power to a
/// higher speed; it never reduces it).
#[test]
fn override_dominates_normal_deployment() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    let mut rng = TestRng::new(0x5EED_0002);
    for _ in 0..10_000 {
        let v = rng.range(0.0, 120.0);
        assert!(
            rb.deploy_cap_electrical_w(v, true) >= rb.deploy_cap_electrical_w(v, false),
            "override must never grant less than the base envelope at v={v} m/s"
        );
    }
}

/// The override harvest bonus is HARVEST allowance (C5.2.10(iii)) — 9.0 MJ with the flag, 8.5
/// without; and a car without an override mode gets the base envelope for override queries.
#[test]
fn harvest_budget_carries_the_override_bonus() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    assert_eq!(rb.harvest_budget_j(false), 8.5e6);
    assert_eq!(rb.harvest_budget_j(true), 9.0e6);
}

/// Torque cap: min-composed on the mechanical side; identity without a `.ptm` envelope.
#[test]
fn torque_cap_binds_at_low_shaft_speed() {
    // A flat 500 Nm crank envelope (C5.2.11) over 0..2400 rad/s.
    let omega = [0.0, 2400.0];
    let torque = [500.0, 500.0];
    let rb: ErsRulebook<f64> =
        ErsRulebook::from_schema(&f1_ers(), Some((&omega, &torque))).unwrap();
    // At 200 rad/s the torque cap allows 100 kW mechanical — it binds before the power cap.
    assert!((rb.torque_capped_mech_w(339.5e3, 200.0) - 100e3).abs() < 1e-6);
    // At 1200 rad/s the envelope allows 600 kW — the power argument passes through.
    assert!((rb.torque_capped_mech_w(339.5e3, 1200.0) - 339.5e3).abs() < 1e-6);
    // No envelope → identity.
    let rb_none: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    assert_eq!(rb_none.torque_capped_mech_w(339.5e3, 200.0), 339.5e3);
}

/// The single conversion seam: deploy mech = elec × 0.97, harvest elec = mech × 0.97, and the
/// mechanical power absorbed at the 350 kW electrical harvest cap is ≈ 360.8 kW (C5.2.21).
#[test]
fn conversion_seam_is_the_c5_2_14_factor() {
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&f1_ers(), None).unwrap();
    assert_eq!(rb.mech_deploy_w(350e3), 350e3 * 0.97);
    assert_eq!(rb.elec_harvest_w(100e3), 97e3);
    let mech_at_cap = rb.mech_harvest_w(350e3);
    assert!(
        (mech_at_cap - 360.825e3).abs() < 50.0,
        "mechanical absorption at the electrical cap should be ≈ 360.8 kW, got {mech_at_cap}"
    );
}

/// gt_hybrid (D-M6-12): the Option paths — no override mode (override queries fall back to the
/// base envelope; harvest bonus is zero), no recharge fields (reg defaults fill), a decreasing
/// mid-knot taper evaluated linearly, 120 kW / 3 MJ budgets.
#[test]
fn gt_hybrid_option_paths() {
    let ers = gt_ers();
    assert!(ers.override_mode.is_none(), "fixture has no override");
    let rb: ErsRulebook<f64> = ErsRulebook::from_schema(&ers, None).unwrap();

    // Override queries on an override-less car fall back to the base envelope + budget.
    let mut rng = TestRng::new(0x5EED_0003);
    for _ in 0..1_000 {
        let v = rng.range(0.0, 100.0);
        assert_eq!(
            rb.deploy_cap_electrical_w(v, true),
            rb.deploy_cap_electrical_w(v, false)
        );
    }
    assert_eq!(rb.harvest_budget_j(true), rb.harvest_budget_j(false));
    assert_eq!(rb.harvest_budget_j(false), 3.0e6);

    // The decreasing mid-knot taper `[0, 250, 320] / [1.0, 0.8, 0.0]` interpolates LINEARLY.
    let v = 285.0 * KPH_TO_MPS; // midpoint of [250, 320]
    assert!((rb.deploy_cap_electrical_w(v, false) - 120e3 * 0.4).abs() < 1e-6);

    // Reg defaults fill the absent recharge fields; recharge phases stay off, and the recharge
    // target defaults to the TOP of the usable window (recharge toward the max the pack allows).
    assert!(!rb.recharge_phases());
    assert_eq!(rb.recharge_target_soc(), 0.85);
    assert_eq!(rb.ramp_allowed_reduction_w(0.0, 0.1), 150e3);
    assert_eq!(rb.ramp_allowed_reduction_w(150e3, 0.1), 50e3 * 0.1);
}

/// A rising power fraction is rejected with the typed error (defense in depth over `check_taper`).
#[test]
fn a_rising_taper_is_rejected() {
    let mut ers = f1_ers();
    ers.deployment.taper_vs_speed.power_frac = vec![0.5, 1.0, 0.2, 0.0];
    assert!(matches!(
        ErsRulebook::<f64>::from_schema(&ers, None),
        Err(RulebookError::TaperNotMonotone {
            table: "deployment"
        })
    ));
}

/// f32 and f64 rulebooks agree to single precision across the speed range.
#[test]
fn f32_f64_parity() {
    let ers = f1_ers();
    let rb64: ErsRulebook<f64> = ErsRulebook::from_schema(&ers, None).unwrap();
    let rb32: ErsRulebook<f32> = ErsRulebook::from_schema(&ers, None).unwrap();
    let mut rng = TestRng::new(0x5EED_0004);
    for _ in 0..2_000 {
        let v = rng.range(0.0, 110.0);
        let p64 = rb64.deploy_cap_electrical_w(v, false);
        let p32 = f64::from(rb32.deploy_cap_electrical_w(v as f32, false));
        assert!(
            (p64 - p32).abs() <= 1e-3 * 350e3,
            "f32/f64 taper divergence at v={v}: {p64} vs {p32}"
        );
    }
}
