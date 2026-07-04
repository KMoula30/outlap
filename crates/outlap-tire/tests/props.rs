// SPDX-License-Identifier: AGPL-3.0-only
//! Property tests for the MF6.1 steady-state model (CLAUDE.md: new physics ⇒ property tests).
//!
//! Strategies jitter physically-plausible coefficient sets. Symmetry properties use *symmetric
//! subsets* (shift and asymmetry coefficients zeroed) — with `RHX1 ≠ 0` etc. the containment
//! and oddness claims are false in general, by design of the MF shift terms.
#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    // Exact zero is the contract for the airborne short-circuit.
    clippy::float_cmp
)]

use std::collections::BTreeMap;

use outlap_tire::{peak_mu_x, Mf61, Mf61Params, SlipState};
use proptest::prelude::*;

const P0: f64 = 220_000.0;
const VX: f64 = 20.0;

fn insert(map: &mut BTreeMap<String, f64>, k: &str, v: f64) {
    map.insert(k.to_owned(), v);
}

/// A symmetric (shift-free, camber-free-at-γ=0) coefficient set with jittered magnitudes.
#[allow(clippy::too_many_arguments)]
fn symmetric_map(
    fnomin: f64,
    pcx1: f64,
    pdx1: f64,
    pex1: f64,
    pkx1: f64,
    pcy1: f64,
    pdy1: f64,
    pey1: f64,
    pky1: f64,
) -> BTreeMap<String, f64> {
    let mut m = BTreeMap::new();
    insert(&mut m, "FNOMIN", fnomin);
    insert(&mut m, "UNLOADED_RADIUS", 0.33);
    insert(&mut m, "NOMPRES", P0);
    insert(&mut m, "PCX1", pcx1);
    insert(&mut m, "PDX1", pdx1);
    insert(&mut m, "PEX1", pex1);
    insert(&mut m, "PKX1", pkx1);
    insert(&mut m, "PCY1", pcy1);
    insert(&mut m, "PDY1", pdy1);
    insert(&mut m, "PEY1", pey1);
    insert(&mut m, "PKY1", pky1);
    // Aligning-moment family without conicity/ply-steer terms (QDZ6/7, QHZ* stay 0).
    insert(&mut m, "QBZ1", 8.0);
    insert(&mut m, "QCZ1", 1.1);
    insert(&mut m, "QDZ1", 0.08);
    insert(&mut m, "QEZ1", -1.0);
    insert(&mut m, "QBZ9", 12.0);
    m
}

/// Extends the symmetric map with combined-slip weighting (still shift-free).
fn combined_map(base: &BTreeMap<String, f64>, rbx1: f64, rby1: f64) -> BTreeMap<String, f64> {
    let mut m = base.clone();
    insert(&mut m, "RBX1", rbx1);
    insert(&mut m, "RBX2", rbx1 * 0.6);
    insert(&mut m, "RCX1", 1.0);
    insert(&mut m, "RBY1", rby1);
    insert(&mut m, "RBY2", rby1 * 0.5);
    insert(&mut m, "RCY1", 1.0);
    m
}

fn model(map: &BTreeMap<String, f64>) -> Mf61<f64> {
    let (p, _) = Mf61Params::from_coeffs(map).unwrap();
    Mf61::new(p)
}

prop_compose! {
    fn arb_symmetric()(
        fnomin in 2000.0..8000.0,
        pcx1 in 1.3..2.0,
        pdx1 in 0.8..1.6,
        pex1 in -1.0..0.5,
        pkx1 in 15.0..30.0,
        pcy1 in 1.2..1.8,
        pdy1 in 0.8..1.5,
        pey1 in -2.0..0.3,
        pky1 in -35.0..-10.0,
    ) -> BTreeMap<String, f64> {
        symmetric_map(fnomin, pcx1, pdx1, pex1, pkx1, pcy1, pdy1, pey1, pky1)
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    /// All five channels stay finite over a hostile-but-finite input box.
    #[test]
    fn outputs_finite_everywhere(
        map in arb_symmetric(),
        kappa in -2.0..2.0,
        alpha in -1.5..1.5,
        gamma in -0.3..0.3,
        fz in -1000.0..24_000.0,
        p in 0.5 * P0..1.5 * P0,
        vx in -100.0..100.0,
    ) {
        let m = model(&map);
        let f = m.forces(&SlipState::new(kappa, alpha, gamma, fz, p, vx));
        prop_assert!(f.fx.is_finite() && f.fy.is_finite() && f.mz.is_finite()
            && f.mx.is_finite() && f.my.is_finite());
    }

    /// Fz ≤ 0 short-circuits to exactly zero on every channel.
    #[test]
    fn airborne_is_zero(map in arb_symmetric(), fz in -5000.0..0.0, kappa in -1.0..1.0) {
        let m = model(&map);
        let f = m.forces(&SlipState::new(kappa, 0.2, 0.05, fz, P0, VX));
        prop_assert_eq!((f.fx, f.fy, f.mz, f.mx, f.my), (0.0, 0.0, 0.0, 0.0, 0.0));
    }

    /// On the symmetric subset: Fx odd in κ, Fy and Mz odd in α (γ = 0).
    #[test]
    fn odd_symmetry(map in arb_symmetric(), kappa in 0.0..1.5, alpha in 0.0..1.2, fz in 2000.0..12_000.0) {
        let m = model(&map);
        let a = m.forces(&SlipState::new(kappa, 0.0, 0.0, fz, P0, VX));
        let b = m.forces(&SlipState::new(-kappa, 0.0, 0.0, fz, P0, VX));
        prop_assert!((a.fx + b.fx).abs() <= 1e-9 * a.fx.abs().max(1.0));

        let c = m.forces(&SlipState::new(0.0, alpha, 0.0, fz, P0, VX));
        let d = m.forces(&SlipState::new(0.0, -alpha, 0.0, fz, P0, VX));
        prop_assert!((c.fy + d.fy).abs() <= 1e-9 * c.fy.abs().max(1.0));
        prop_assert!((c.mz + d.mz).abs() <= 1e-9 * c.mz.abs().max(1.0));
    }

    /// ISO-W sign pins: Kxκ > 0 and sgn(Kyα) = sgn(PKY1) < 0; Mz restoring.
    #[test]
    fn sign_pins(map in arb_symmetric(), fz in 2000.0..12_000.0) {
        let m = model(&map);
        let d = 1e-6;
        let kxk = (m.forces(&SlipState::new(d, 0.0, 0.0, fz, P0, VX)).fx
            - m.forces(&SlipState::new(-d, 0.0, 0.0, fz, P0, VX)).fx) / (2.0 * d);
        prop_assert!(kxk > 0.0);
        let kya = (m.forces(&SlipState::new(0.0, d, 0.0, fz, P0, VX)).fy
            - m.forces(&SlipState::new(0.0, -d, 0.0, fz, P0, VX)).fy) / (2.0 * d);
        prop_assert!(kya < 0.0);
        let f = m.forces(&SlipState::new(0.0, 0.08, 0.0, fz, P0, VX));
        prop_assert!(f.fy < 0.0 && f.mz > 0.0);
    }

    /// Value continuity across κ = 0, α = 0, and the Vx = 0⁺ approach.
    #[test]
    fn value_continuity(map in arb_symmetric(), fz in 2000.0..12_000.0) {
        let m = model(&map);
        let d = 1e-9;
        let scale = fz;

        let a = m.forces(&SlipState::new(d, 0.03, 0.01, fz, P0, VX));
        let b = m.forces(&SlipState::new(-d, 0.03, 0.01, fz, P0, VX));
        prop_assert!((a.fx - b.fx).abs() <= 1e-3 * scale);
        prop_assert!((a.fy - b.fy).abs() <= 1e-3 * scale);

        let c = m.forces(&SlipState::new(0.03, d, 0.01, fz, P0, VX));
        let e = m.forces(&SlipState::new(0.03, -d, 0.01, fz, P0, VX));
        prop_assert!((c.fx - e.fx).abs() <= 1e-3 * scale);
        prop_assert!((c.fy - e.fy).abs() <= 1e-3 * scale);

        let f0 = m.forces(&SlipState::new(0.03, 0.02, 0.0, fz, P0, 0.0));
        let f1 = m.forces(&SlipState::new(0.03, 0.02, 0.0, fz, P0, 1e-9));
        prop_assert!((f0.fx - f1.fx).abs() <= 1e-3 * scale);
        prop_assert!((f0.fy - f1.fy).abs() <= 1e-3 * scale);
    }

    /// Cosine-weighting containment on the shift-free subset with C = 1:
    /// combined |Fx| ≤ pure |Fx|, combined |Fy| ≤ pure |Fy| (G ∈ (0, 1]).
    #[test]
    fn combined_containment(
        map in arb_symmetric(),
        rbx1 in 5.0..25.0,
        rby1 in 5.0..20.0,
        kappa in -0.5..0.5,
        alpha in -0.4..0.4,
        fz in 2000.0..12_000.0,
    ) {
        let m = model(&combined_map(&map, rbx1, rby1));
        let both = m.forces(&SlipState::new(kappa, alpha, 0.0, fz, P0, VX));
        let pure_x = m.forces(&SlipState::new(kappa, 0.0, 0.0, fz, P0, VX));
        let pure_y = m.forces(&SlipState::new(0.0, alpha, 0.0, fz, P0, VX));
        prop_assert!(both.fx.abs() <= pure_x.fx.abs() + 1e-9);
        prop_assert!(both.fy.abs() <= pure_y.fy.abs() + 1e-9);
    }

    /// Peak linearity in LMUX (V-shifts zero, C > 1): peak(λ·L) = λ·peak(L).
    #[test]
    fn peak_scaling_linearity(map in arb_symmetric(), lam in 0.5..1.2) {
        let base = model(&map);
        let mut scaled_map = map;
        insert(&mut scaled_map, "LMUX", lam);
        let scaled = model(&scaled_map);
        let mu1 = peak_mu_x(&base, 4000.0, P0);
        let mu2 = peak_mu_x(&scaled, 4000.0, P0);
        prop_assert!((mu2 - lam * mu1).abs() <= 1e-6 * mu1);
    }

    /// The scan agrees with the closed-form peak D/Fz = PDX1 at nominal conditions (C > 1,
    /// shift-free, dfz = dpi = 0).
    #[test]
    fn peak_matches_closed_form(map in arb_symmetric()) {
        let fnomin = map["FNOMIN"];
        let pdx1 = map["PDX1"];
        let m = model(&map);
        let mu = peak_mu_x(&m, fnomin, P0);
        prop_assert!((mu - pdx1).abs() <= 1e-7 * pdx1, "μ {mu} vs PDX1 {pdx1}");
    }
}
