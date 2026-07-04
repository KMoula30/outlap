// SPDX-License-Identifier: AGPL-3.0-only
//! Unit tests for the MF6.1 steady-state model: fixture sanity, sign contract, closed-form
//! peak cross-checks, defaults regression, and build-error taxonomy.
#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    // Exact zero is the contract for the airborne short-circuit and absent families.
    clippy::float_cmp
)]

use std::collections::BTreeMap;

use outlap_schema::io::MemLoader;
use outlap_schema::load::load_tyr;
use outlap_tire::{peak_mu_x, peak_mu_y, Mf61, Mf61BuildError, Mf61Params, SlipState};

const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

/// Nominal-load slick conditions: dpi ≡ 0 (no NOMPRES), 200 kPa placeholder pressure.
const FZ: f64 = 4000.0;
const P: f64 = 200_000.0;
const VX: f64 = 16.7;

fn slick_model() -> Mf61<f64> {
    let loader = MemLoader::new().with("slick.tyr.yaml", SLICK);
    let (tyr, _) = load_tyr("slick.tyr.yaml", &loader).unwrap();
    let (model, _) = Mf61::from_tyr(&tyr).unwrap();
    model
}

fn map(pairs: &[(&str, f64)]) -> BTreeMap<String, f64> {
    pairs.iter().map(|(k, v)| ((*k).to_owned(), *v)).collect()
}

fn state(kappa: f64, alpha: f64) -> SlipState<f64> {
    SlipState::new(kappa, alpha, 0.0, FZ, P, VX)
}

#[test]
fn slick_fixture_evaluates_finite_and_sane() {
    let m = slick_model();
    let f = m.forces(&state(0.05, -0.05));
    assert!(f.fx.is_finite() && f.fy.is_finite() && f.mz.is_finite());
    assert!(f.mx.is_finite() && f.my.is_finite());
    // The fixture has no Mz/Mx/My families: those channels degrade to exactly zero.
    assert_eq!(f.mz, 0.0);
    assert_eq!(f.mx, 0.0);
    assert_eq!(f.my, 0.0);
    // Forces are meaningfully nonzero (the PKY4=2/PKY2=1 defaults regression: a naive
    // all-zero default table collapses Kyα ≡ 0 on this required-minimum fixture).
    assert!(f.fx > 1000.0, "Fx {:.1}", f.fx);
    assert!(f.fy > 1000.0, "Fy {:.1}", f.fy);
}

#[test]
fn cornering_stiffness_matches_closed_form() {
    // At Fz = FNOMIN, dpi = 0, γ = 0 and PKY2 = 1 (default): the load curve's sine hits
    // sin(PKY4·atan(1)) = sin(π/2) = 1, so Kyα = PKY1·Fz0' = −20·4000 = −80 kN/rad.
    let m = slick_model();
    let d = 1e-6;
    let slope = (m.forces(&state(0.0, d)).fy - m.forces(&state(0.0, -d)).fy) / (2.0 * d);
    let expected = -20.0 * 4000.0;
    assert!(
        (slope - expected).abs() < 0.01 * expected.abs(),
        "Kyα {slope:.0} vs {expected:.0}"
    );
}

#[test]
fn sign_contract_iso_w() {
    let m = slick_model();
    // Driving slip → positive Fx (Kxκ > 0).
    assert!(m.forces(&state(0.05, 0.0)).fx > 0.0);
    assert!(m.forces(&state(-0.05, 0.0)).fx < 0.0);
    // Positive slip angle → negative Fy (PKY1 < 0 in ISO-W).
    assert!(m.forces(&state(0.0, 0.05)).fy < 0.0);
    assert!(m.forces(&state(0.0, -0.05)).fy > 0.0);
}

#[test]
fn peak_mu_matches_closed_form_when_c_above_one() {
    // Slick: Cx = 1.65 > 1, Cy = 1.40 > 1, no shifts, dpi = 0 ⇒ peak = D/Fz = PD·LMU exactly.
    let m = slick_model();
    let mux = peak_mu_x(&m, FZ, P);
    let muy = peak_mu_y(&m, FZ, P);
    assert!((mux - 1.30).abs() < 1e-7, "μx {mux}");
    assert!((muy - 1.25).abs() < 1e-7, "μy {muy}");
}

#[test]
fn airborne_wheel_is_exactly_zero() {
    let m = slick_model();
    for fz in [0.0, -100.0, -1e6] {
        let f = m.forces(&SlipState::new(0.1, 0.1, 0.02, fz, P, VX));
        assert_eq!((f.fx, f.fy, f.mz, f.mx, f.my), (0.0, 0.0, 0.0, 0.0, 0.0));
    }
}

#[test]
fn finite_at_extreme_inputs() {
    let m = slick_model();
    for &(k, a, g, fz, p, vx) in &[
        (2.0, 1.5, 0.3, 3.0 * FZ, 3.0 * P, 100.0),
        (-1.0, -1.5, -0.3, 1.0, 0.0, -50.0),
        (0.0, std::f64::consts::FRAC_PI_2, 0.0, FZ, P, 0.0),
        (-2.0, 0.0, 0.3, 0.5, P, 1e-9),
    ] {
        let f = m.forces(&SlipState::new(k, a, g, fz, p, vx));
        assert!(
            f.fx.is_finite()
                && f.fy.is_finite()
                && f.mz.is_finite()
                && f.mx.is_finite()
                && f.my.is_finite(),
            "non-finite at k={k} a={a} g={g} fz={fz} p={p} vx={vx}"
        );
    }
}

#[test]
fn param_notes_report_degradations() {
    let loader = MemLoader::new().with("slick.tyr.yaml", SLICK);
    let (tyr, _) = load_tyr("slick.tyr.yaml", &loader).unwrap();
    let (_, notes) = Mf61Params::<f64>::from_tyr(&tyr).unwrap();
    // Notes are ReportEntry values pointed at `/mf61/<family>` so they merge into the
    // loaded-model report unchanged.
    let ptrs: Vec<&str> = notes.iter().map(|n| n.pointer.as_str()).collect();
    for expected in [
        "/mf61/NOMPRES",
        "/mf61/QDZ*",
        "/mf61/QSX*",
        "/mf61/QSY*",
        "/mf61/RBX*",
        "/mf61/RBY*",
    ] {
        assert!(ptrs.contains(&expected), "missing {expected} in {notes:?}");
    }
}

#[test]
fn finite_under_exp_overflow_load() {
    // PKX3 > 0 with an extreme load drives PKX3·dfz past the exp overflow threshold; the clamp
    // must keep every channel finite (the "finite for all finite inputs" contract).
    let mut c = map(&[
        ("FNOMIN", 4000.0),
        ("UNLOADED_RADIUS", 0.33),
        ("PCX1", 1.65),
        ("PDX1", 1.30),
        ("PEX1", 0.10),
        ("PKX1", 22.0),
        ("PCY1", 1.40),
        ("PDY1", 1.25),
        ("PEY1", -1.0),
        ("PKY1", -20.0),
    ]);
    c.insert("PKX3".to_owned(), 0.5);
    let (p, _) = Mf61Params::<f64>::from_coeffs(&c).unwrap();
    let m = Mf61::new(p);
    // dfz = (Fz − Fz0)/Fz0 = (8e6 − 4000)/4000 ≈ 2000 ⇒ PKX3·dfz ≈ 1000 ≫ 709.
    let f = m.forces(&SlipState::new(0.1, 0.05, 0.0, 8_000_000.0, P, VX));
    assert!(
        f.fx.is_finite() && f.fy.is_finite() && f.mz.is_finite(),
        "Fx {} Fy {} Mz {}",
        f.fx,
        f.fy,
        f.mz
    );
}

#[test]
fn build_errors() {
    // Missing structural keys.
    let err = Mf61Params::<f64>::from_coeffs(&map(&[("PDX1", 1.0)])).unwrap_err();
    assert!(matches!(
        err,
        Mf61BuildError::MissingCoefficient { key: "FNOMIN" }
    ));
    // Non-finite value.
    let err =
        Mf61Params::<f64>::from_coeffs(&map(&[("FNOMIN", f64::NAN), ("UNLOADED_RADIUS", 0.3)]))
            .unwrap_err();
    assert!(matches!(err, Mf61BuildError::NonFinite { .. }));
    // Non-positive nominal load.
    let err = Mf61Params::<f64>::from_coeffs(&map(&[("FNOMIN", -1.0), ("UNLOADED_RADIUS", 0.3)]))
        .unwrap_err();
    assert!(matches!(
        err,
        Mf61BuildError::NonPositive { key: "FNOMIN", .. }
    ));
}

#[test]
fn works_in_f32() {
    let loader = MemLoader::new().with("slick.tyr.yaml", SLICK);
    let (tyr, _) = load_tyr("slick.tyr.yaml", &loader).unwrap();
    let (p, _) = Mf61Params::<f32>::from_tyr(&tyr).unwrap();
    let m = Mf61::new(p);
    let mux = peak_mu_x(&m, 4000.0_f32, 200_000.0);
    assert!((mux - 1.30).abs() < 1e-3, "f32 μx {mux}");
}

#[test]
fn mz_is_restoring_with_aligning_family() {
    // Slick core + a plausible aligning-moment family: Mz > 0 for α > 0 (trail on a negative
    // Fy is restoring — the sign chain of eq. 4.E31/4.E71 with ISO-W conventions).
    let mut c = map(&[
        ("FNOMIN", 4000.0),
        ("UNLOADED_RADIUS", 0.33),
        ("PCX1", 1.65),
        ("PDX1", 1.30),
        ("PEX1", 0.10),
        ("PKX1", 22.0),
        ("PCY1", 1.40),
        ("PDY1", 1.25),
        ("PEY1", -1.0),
        ("PKY1", -20.0),
    ]);
    for (k, v) in [
        ("QBZ1", 8.0),
        ("QCZ1", 1.1),
        ("QDZ1", 0.09),
        ("QEZ1", -1.0),
        ("QBZ9", 15.0),
        ("QDZ6", 0.0),
    ] {
        c.insert(k.to_owned(), v);
    }
    let (p, _) = Mf61Params::<f64>::from_coeffs(&c).unwrap();
    let m = Mf61::new(p);
    let f = m.forces(&state(0.0, 0.06));
    assert!(f.fy < 0.0);
    assert!(f.mz > 0.0, "Mz {:.2} should restore", f.mz);
    // Odd in α with this shift-free set.
    let g = m.forces(&state(0.0, -0.06));
    assert!((f.mz + g.mz).abs() < 1e-9 * f.mz.abs().max(1.0));
}

#[test]
fn my_opposes_rolling_direction() {
    let mut c = map(&[
        ("FNOMIN", 4000.0),
        ("UNLOADED_RADIUS", 0.33),
        ("PCX1", 1.65),
        ("PDX1", 1.30),
        ("PEX1", 0.10),
        ("PKX1", 22.0),
        ("PCY1", 1.40),
        ("PDY1", 1.25),
        ("PEY1", -1.0),
        ("PKY1", -20.0),
    ]);
    c.insert("QSY1".to_owned(), 0.01);
    let (p, _) = Mf61Params::<f64>::from_coeffs(&c).unwrap();
    let m = Mf61::new(p);
    let fwd = m.forces(&SlipState::new(0.0, 0.0, 0.0, FZ, P, 30.0));
    let rev = m.forces(&SlipState::new(0.0, 0.0, 0.0, FZ, P, -30.0));
    assert!(fwd.my < 0.0, "forward rolling: My {:.3}", fwd.my);
    assert!(rev.my > 0.0, "reverse rolling: My {:.3}", rev.my);
}
