// SPDX-License-Identifier: AGPL-3.0-only
//! Property + unit tests for the brush model and the relaxation helper (new physics ⇒ properties).
#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::float_cmp
)]

use std::collections::BTreeMap;

use outlap_tire::{relax_step, Brush, Mf61Params, Relaxation, SlipState};
use proptest::prelude::*;

const P0: f64 = 220_000.0;
const VX: f64 = 20.0;

fn brush(c_kappa: f64, c_alpha: f64, mu0: f64, a: f64) -> Brush<f64> {
    Brush::new(c_kappa, c_alpha, mu0, a)
}

prop_compose! {
    fn arb_brush()(
        c_kappa in 2.0e4..3.0e5,
        c_alpha in 2.0e4..2.0e5,
        mu0 in 0.5..2.0,
        a in 0.05..0.20,
    ) -> Brush<f64> {
        brush(c_kappa, c_alpha, mu0, a)
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// Every channel stays finite across a hostile-but-finite input box (κ near the −1 pole too).
    #[test]
    fn brush_finite_everywhere(
        b in arb_brush(),
        kappa in -1.5..2.0,
        alpha in -1.4..1.4,
        gamma in -0.3..0.3,
        fz in -1000.0..20_000.0,
        p in 0.5 * P0..1.5 * P0,
        vx in -80.0..80.0,
    ) {
        let f = b.forces(&SlipState::new(kappa, alpha, gamma, fz, p, vx));
        prop_assert!(f.fx.is_finite() && f.fy.is_finite() && f.mz.is_finite()
            && f.mx.is_finite() && f.my.is_finite());
        // Mx = My ≡ 0 at the brush tier.
        prop_assert_eq!((f.mx, f.my), (0.0, 0.0));
    }

    /// Airborne (Fz ≤ 0) short-circuits to exactly zero on every channel.
    #[test]
    fn brush_airborne_is_zero(b in arb_brush(), fz in -5000.0..0.0, kappa in -0.9..1.0) {
        let f = b.forces(&SlipState::new(kappa, 0.2, 0.05, fz, P0, VX));
        prop_assert_eq!((f.fx, f.fy, f.mz, f.mx, f.my), (0.0, 0.0, 0.0, 0.0, 0.0));
    }

    /// The friction ellipse is respected: |F| ≤ μ0·Fz for every slip.
    #[test]
    fn brush_respects_friction_bound(
        b in arb_brush(),
        kappa in -0.9..1.5,
        alpha in -1.0..1.0,
        fz in 500.0..12_000.0,
    ) {
        let f = b.forces(&SlipState::new(kappa, alpha, 0.0, fz, P0, VX));
        let mag = (f.fx * f.fx + f.fy * f.fy).sqrt();
        prop_assert!(mag <= b.mu0() * fz + 1e-6 * fz, "|F| {mag} exceeds μ0·Fz {}", b.mu0() * fz);
    }

    /// Origin slopes pin the sign contract: ∂Fx/∂κ|₀ = +C_κ and ∂Fy/∂α|₀ = −C_α.
    #[test]
    fn brush_origin_slopes(
        c_kappa in 2.0e4..3.0e5,
        c_alpha in 2.0e4..2.0e5,
        mu0 in 0.8..2.0,
        a in 0.05..0.20,
        fz in 3000.0..9000.0,
    ) {
        let b = brush(c_kappa, c_alpha, mu0, a);
        let d = 1e-6;
        let dfx = (b.forces(&SlipState::new(d, 0.0, 0.0, fz, P0, VX)).fx
            - b.forces(&SlipState::new(-d, 0.0, 0.0, fz, P0, VX)).fx) / (2.0 * d);
        prop_assert!((dfx - c_kappa).abs() <= 1e-3 * c_kappa, "∂Fx/∂κ {dfx} vs {c_kappa}");
        let dfy = (b.forces(&SlipState::new(0.0, d, 0.0, fz, P0, VX)).fy
            - b.forces(&SlipState::new(0.0, -d, 0.0, fz, P0, VX)).fy) / (2.0 * d);
        prop_assert!((dfy + c_alpha).abs() <= 1e-3 * c_alpha, "∂Fy/∂α {dfy} vs {}", -c_alpha);
    }

    /// Sign pins at a small working point (guaranteed sub-sliding, ψ ≪ 1): driving (κ>0) pushes
    /// Fx>0, slip (α>0) pushes Fy<0, and the aligning moment is restoring (Mz>0).
    #[test]
    fn brush_sign_pins(b in arb_brush(), fz in 4000.0..9000.0) {
        // κ = α = 0.004 keeps ψ ≤ ~0.25 across the whole arb_brush box (a/3 trail non-zero).
        let f = b.forces(&SlipState::new(0.004, 0.004, 0.0, fz, P0, VX));
        prop_assert!(f.fx > 0.0, "driving should give Fx>0");
        prop_assert!(f.fy < 0.0, "positive slip should give Fy<0");
        prop_assert!(f.mz > 0.0, "aligning moment should be restoring, got {}", f.mz);
    }
}

/// At full sliding a pure-longitudinal brush tire delivers exactly μ0·Fz and zero trail.
#[test]
fn brush_saturates_to_mu0_fz() {
    let b = brush(1.5e5, 1.2e5, 1.3, 0.10);
    let fz = 4000.0;
    // κ = 5 with these stiffnesses guarantees ψ ≫ 1 (full sliding).
    let f = b.forces(&SlipState::new(5.0, 0.0, 0.0, fz, P0, VX));
    assert!(
        (f.fx - 1.3 * fz).abs() <= 1e-6 * fz,
        "Fx {} vs μ0·Fz {}",
        f.fx,
        1.3 * fz
    );
    assert!(f.fy.abs() <= 1e-9);
    assert!(
        f.mz.abs() <= 1e-9,
        "trail must vanish at full sliding, got Mz {}",
        f.mz
    );
}

/// `mu_scale_*` scale μ0 per axis: a pure-x saturated force scales by `mu_scale_x`.
#[test]
fn brush_mu_scale_scales_peak() {
    let b = brush(1.5e5, 1.2e5, 1.3, 0.10);
    let fz = 4000.0;
    let mut s = SlipState::new(5.0, 0.0, 0.0, fz, P0, VX);
    s.mu_scale_x = 0.8;
    assert!((b.forces(&s).fx - 0.8 * 1.3 * fz).abs() <= 1e-6 * fz);
}

// --- Relaxation --------------------------------------------------------------------------------

fn relax_map() -> BTreeMap<String, f64> {
    // A tyre with PT* transient coefficients AND carcass stiffness present.
    let pairs: &[(&str, f64)] = &[
        ("FNOMIN", 4000.0),
        ("UNLOADED_RADIUS", 0.33),
        ("PTX1", 2.3),
        ("PTX2", 1.9),
        ("PTX3", 0.24),
        ("PTY1", 2.1),
        ("PTY2", 2.0),
        ("PKX1", 22.0),
        ("PKY1", -20.0),
        ("PKY2", 1.8),
        ("LONGITUDINAL_STIFFNESS", 3.5e5),
        ("LATERAL_STIFFNESS", 1.6e5),
    ];
    pairs.iter().map(|(k, v)| ((*k).to_owned(), *v)).collect()
}

fn params(map: &BTreeMap<String, f64>) -> Mf61Params<f64> {
    Mf61Params::from_coeffs(map).unwrap().0
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// The exact-exponential step contracts toward x_ss for every dt ≥ 0 (|error| never grows).
    #[test]
    fn relax_contracts(
        x in -1.0f64..1.0, x_ss in -1.0..1.0, v in 0.0..90.0, dt in 0.0..0.1, sigma in 0.01..1.0,
    ) {
        let y = relax_step(x, x_ss, v, dt, sigma);
        prop_assert!((y - x_ss).abs() <= (x - x_ss).abs() + 1e-12);
    }

    /// The step reproduces the analytic ratio (y − x_ss) = (x − x_ss)·exp(−v·dt/σ).
    #[test]
    fn relax_exact_ratio(
        x in -1.0f64..1.0, x_ss in -1.0..1.0, v in 0.1..90.0, dt in 0.0..0.1, sigma in 0.02..1.0,
    ) {
        let y = relax_step(x, x_ss, v, dt, sigma);
        let expect = x_ss + (x - x_ss) * (-v * dt / sigma).exp();
        prop_assert!((y - expect).abs() <= 1e-12);
    }

    /// Two half-steps equal one full step (the exact update composes exactly).
    #[test]
    fn relax_half_step_composition(
        x in -1.0f64..1.0, x_ss in -1.0..1.0, v in 0.1..90.0, dt in 0.0..0.1, sigma in 0.02..1.0,
    ) {
        let one = relax_step(x, x_ss, v, dt, sigma);
        let half = relax_step(x, x_ss, v, dt / 2.0, sigma);
        let two = relax_step(half, x_ss, v, dt / 2.0, sigma);
        prop_assert!((one - two).abs() <= 1e-12, "{one} vs {two}");
    }

    /// Relaxation lengths are finite and floored at `SIGMA_FLOOR_M` over a wide load/camber range.
    #[test]
    fn relax_lengths_are_positive(fz in 500.0..14_000.0, gamma in -0.2..0.2) {
        let (r, _notes) = Relaxation::from_params(&params(&relax_map()));
        let sk = r.sigma_kappa(fz);
        let sa = r.sigma_alpha(fz, gamma);
        prop_assert!(sk.is_finite() && sk >= 1e-3, "σκ {sk}");
        prop_assert!(sa.is_finite() && sa >= 1e-3, "σα {sa}");
    }
}

/// A tyre with no PT* coefficients and no carcass stiffness falls back to 0.5·R0 with a loud note.
#[test]
fn relax_last_resort_note() {
    let mut map = BTreeMap::new();
    for (k, v) in [("FNOMIN", 4000.0), ("UNLOADED_RADIUS", 0.33)] {
        map.insert(k.to_owned(), v);
    }
    let (r, notes) = Relaxation::from_params(&params(&map));
    assert!((r.sigma_kappa(4000.0) - 0.5 * 0.33).abs() <= 1e-9);
    assert!((r.sigma_alpha(4000.0, 0.0) - 0.5 * 0.33).abs() <= 1e-9);
    assert!(
        notes.iter().any(|n| n.detail.contains("0.5·R0")),
        "expected a loud last-resort note"
    );
}
