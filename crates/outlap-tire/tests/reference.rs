// SPDX-License-Identifier: AGPL-3.0-only
//! Integration test over the committed reference `.tyr` data (`data/tires/`).
//!
//! Loads the Pacejka book reference tyre through the real file loader and asserts it (a) parses
//! with no unknown-coefficient warnings and (b) evaluates to physically-sane forces with the
//! right sign conventions and grip band. This is the reference-data sanity gate; the tight
//! `≤ 0.5%` cross-check against the teasit Magic-Formula oracle lives in `golden.rs`.
#![allow(clippy::float_cmp, clippy::doc_markdown)]

use outlap_schema::load::load_tyr;
use outlap_tire::{peak_mu_x, peak_mu_y, SlipState};

mod common;
use common::{data_loader, pacejka_model, PACEJKA_TYR};

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
