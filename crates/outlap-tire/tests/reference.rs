// SPDX-License-Identifier: AGPL-3.0-only
//! Integration test over the committed reference `.tyr` data (`data/tires/`).
//!
//! Loads every committed reference tyre through the real file loader and asserts it (a) parses
//! with no unknown-coefficient warnings, (b) evaluates to physically-sane forces with the right
//! sign conventions and a class-plausible grip band, and (c) round-trips through the `.tir`
//! codec numerically exactly. This is the reference-data sanity gate; the tight `≤ 0.5%`
//! cross-check against the teasit Magic-Formula oracle lives in `golden.rs`.
#![allow(clippy::float_cmp, clippy::doc_markdown)]

use outlap_schema::load::load_tyr;
use outlap_schema::tir::{parse_tir, tir_to_tyr, tyr_to_tir, write_tir, TirToTyrOptions};
use outlap_tire::{peak_mu_x, peak_mu_y, SlipState};

mod common;
use common::{all_reference_tyres, data_loader, pacejka_model, roborace_model, PACEJKA_TYR};

const FZ: f64 = 4000.0; // FNOMIN of the reference tyre.
const P: f64 = 220_000.0; // 2.2 bar.
const VX: f64 = 16.67;

#[test]
fn pacejka_book_tyre_loads_without_warnings() {
    let (_tyr, warnings) = load_tyr(PACEJKA_TYR, &data_loader()).unwrap();
    assert!(
        warnings.is_empty(),
        "reference tyre produced load warnings: {warnings:?}"
    );
}

#[test]
fn pacejka_book_tyre_evaluates_physically() {
    let m = pacejka_model();

    // Peak grip: μx ≈ PDX1 = 1.21, μy ≈ |PDY1| = 0.99 (both C > 1 ⇒ peak ≈ D/Fz at nominal).
    let mux = peak_mu_x(&m, FZ, P);
    let muy = peak_mu_y(&m, FZ, P);
    assert!((1.15..1.30).contains(&mux), "μx {mux}");
    assert!((0.90..1.05).contains(&muy), "μy {muy}");
    // Longitudinal grip exceeds lateral for this road tyre.
    assert!(mux > muy, "expected μx > μy, got {mux} vs {muy}");

    // ISO-W sign conventions on the real coefficient set (negative PDY1/PKY1 and all).
    let drive = m.forces(&SlipState::new(0.08, 0.0, 0.0, FZ, P, VX));
    let brake = m.forces(&SlipState::new(-0.08, 0.0, 0.0, FZ, P, VX));
    assert!(drive.fx > 0.0 && brake.fx < 0.0, "Fx sign");

    let right = m.forces(&SlipState::new(0.0, 0.06, 0.0, FZ, P, VX));
    let left = m.forces(&SlipState::new(0.0, -0.06, 0.0, FZ, P, VX));
    assert!(right.fy < 0.0 && left.fy > 0.0, "Fy sign");
    // Aligning moment restores (Mz > 0 for α > 0, from the trail on a negative Fy).
    assert!(right.mz > 0.0, "Mz restoring, got {}", right.mz);

    // This tyre models Mx ≡ 0 (QSX* = 0); rolling resistance My opposes forward rolling.
    assert_eq!(right.mx, 0.0, "Mx should be zero for this tyre");
    assert!(
        right.my < 0.0,
        "My should oppose forward rolling, got {}",
        right.my
    );
}

#[test]
fn all_reference_tyres_load_clean_and_roundtrip_tir() {
    // Every committed dataset (globbed): zero load warnings of any kind, and the coefficient map
    // survives .tyr → .tir text → .tyr numerically exactly (the PR5 codec contract).
    let l = data_loader();
    for path in all_reference_tyres() {
        let (tyr, warnings) =
            load_tyr(&path, &l).unwrap_or_else(|e| panic!("{path} failed to load: {e:?}"));
        assert!(
            warnings.is_empty(),
            "{path} produced load warnings: {warnings:?}"
        );

        let text = write_tir(&tyr_to_tir(&tyr));
        let (doc, tir_warnings) = parse_tir(&path, &text).unwrap();
        assert!(
            tir_warnings.is_empty(),
            "{path} canonical .tir re-parsed with warnings: {tir_warnings:?}"
        );
        let (back, _) = tir_to_tyr(&doc, &TirToTyrOptions::default()).unwrap();
        assert_eq!(
            tyr.mf61.0, back.mf61.0,
            "{path} coefficient map changed through the .tir round-trip"
        );
    }
}

// Roborace tyre operating point: FNOMIN = 3000 N; the set carries no pressure model (NOMPRES
// absent ⇒ dpi ≡ 0 for any pressure argument), so pass 0.0 to make that explicit.
const RB_FZ: f64 = 3000.0;
const RB_P: f64 = 0.0;

#[test]
fn roborace_tyre_evaluates_physically() {
    let m = roborace_model();

    // Peak grip: μx ≈ PDX1·LMUX = 1.5·0.97 ≈ 1.46, μy ≈ PDY1·LMUY = 1.2·0.97 ≈ 1.16 —
    // the "sport focused road tire" band, between the passenger-car set and a slick.
    let mux = peak_mu_x(&m, RB_FZ, RB_P);
    let muy = peak_mu_y(&m, RB_FZ, RB_P);
    assert!((1.35..1.55).contains(&mux), "μx {mux}");
    assert!((1.05..1.30).contains(&muy), "μy {muy}");
    assert!(mux > muy, "expected μx > μy, got {mux} vs {muy}");

    // ISO-W sign conventions (negative PKY1 set).
    let drive = m.forces(&SlipState::new(0.08, 0.0, 0.0, RB_FZ, RB_P, VX));
    let brake = m.forces(&SlipState::new(-0.08, 0.0, 0.0, RB_FZ, RB_P, VX));
    assert!(drive.fx > 0.0 && brake.fx < 0.0, "Fx sign");
    let right = m.forces(&SlipState::new(0.0, 0.06, 0.0, RB_FZ, RB_P, VX));
    let left = m.forces(&SlipState::new(0.0, -0.06, 0.0, RB_FZ, RB_P, VX));
    assert!(right.fy < 0.0 && left.fy > 0.0, "Fy sign");

    // The source set has no Mz/Mx coefficients: both moments are exactly zero; rolling
    // resistance (QSY1 = 0.025 mapped) opposes forward rolling.
    assert_eq!(right.mz, 0.0, "Mz should be zero for this tyre");
    assert_eq!(right.mx, 0.0, "Mx should be zero for this tyre");
    assert!(right.my < 0.0, "My opposes rolling, got {}", right.my);
}

#[test]
fn roborace_camber_remap_matches_mf52_sensitivity_at_fnomin() {
    // The PKY6 = -1.0651 fold-in (README §camber) must reproduce the MF5.2 small-camber Fy
    // sensitivity at FNOMIN: dFy/dγ = Kyα·PHY3 + FNOMIN·PVY3·LMUY ≈ -3195 N/rad, i.e. a camber
    // increment of ≈ -55.8 N at γ = 1°. Measured as a difference to cancel the static
    // PHY1/PVY1 offsets.
    let m = roborace_model();
    let gamma = 1.0_f64.to_radians();
    let at = |g: f64| m.forces(&SlipState::new(0.0, 0.0, g, RB_FZ, RB_P, VX)).fy;
    let d_fy = at(gamma) - at(0.0);
    assert!(
        (-75.0..-40.0).contains(&d_fy),
        "camber Fy increment at 1° should be ≈ -56 N, got {d_fy}"
    );
}

#[test]
fn pacejka_book_tyre_combined_contained_and_finite() {
    let m = pacejka_model();

    // Sweep the full slip/load box: every channel stays finite.
    for &kappa in &[-0.3, -0.1, 0.0, 0.1, 0.3] {
        for &alpha in &[-0.2, -0.05, 0.0, 0.05, 0.2] {
            for &gamma in &[-0.05, 0.0, 0.05] {
                for &fz in &[2000.0, 4000.0, 7000.0] {
                    let f = m.forces(&SlipState::new(kappa, alpha, gamma, fz, P, VX));
                    assert!(
                        f.fx.is_finite()
                            && f.fy.is_finite()
                            && f.mz.is_finite()
                            && f.mx.is_finite()
                            && f.my.is_finite(),
                        "non-finite at κ={kappa} α={alpha} γ={gamma} Fz={fz}"
                    );
                }
            }
        }
    }
}
