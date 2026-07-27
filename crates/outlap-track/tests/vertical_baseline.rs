// SPDX-License-Identifier: AGPL-3.0-only
//! The vertical finite-difference baseline as a recorded simulation setting (`sim.vertical_baseline_m`).
//!
//! `Track` estimates grade and vertical curvature by central differences over a physical baseline
//! (30 m by default — the DEM-noise low-pass pinned by `elevation.rs`). This suite pins the MT
//! promotion of that constant to a per-run override: the default is bit-identical to the historical
//! constant, a shorter baseline resolves elevation detail the 30 m one filters out (the expected
//! direction: a central difference over half-step `h` attenuates a wavelength-`λ` bump by
//! `2·(1 − cos(2πh/λ))/(2πh/λ)²`, monotone in `h`), and an offset line inherits its parent's value.
#![allow(
    // Bit-identity is the contract under test: exact f64 comparison is the point.
    clippy::float_cmp,
    // Geometry helpers use single-letter symbols (s, x, y) by convention (Decision #33).
    clippy::many_single_char_names,
    clippy::doc_markdown
)]

use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_track::{offset_track, Track};

/// A straight 600 m line with a short-wavelength (12 m) sinusoidal elevation ripple — well below
/// what a 30 m baseline can resolve, well within reach of a 6 m one.
fn bumpy_line() -> Track {
    let (amp, lambda) = (0.1, 12.0);
    let rows: Vec<CenterlineRow> = (0..=600)
        .map(|i| {
            let s = f64::from(i);
            CenterlineRow {
                s_m: s,
                x_m: s,
                y_m: 0.0,
                z_m: amp * (2.0 * std::f64::consts::PI * s / lambda).sin(),
                banking_deg: 0.0,
                width_left_m: 5.0,
                width_right_m: 5.0,
                grip_scale: 1.0,
            }
        })
        .collect();
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "bumpy".into(),
        closed: false,
        centerline: CenterlineRef("mem".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

/// Max |κ_v| and max |grade| over an interior sample grid (the ends are clamped-query territory).
fn peaks(t: &Track) -> (f64, f64) {
    let (mut kv, mut g) = (0.0_f64, 0.0_f64);
    for i in 0..500 {
        let s = 50.0 + f64::from(i);
        kv = kv.max(t.curvature_v(s).abs());
        g = g.max(t.grade(s).abs());
    }
    (kv, g)
}

#[test]
fn default_baseline_is_the_historical_constant_bit_for_bit() {
    let t = bumpy_line();
    assert_eq!(t.vertical_baseline_m(), 30.0, "the default must stay 30 m");

    // Explicitly setting the default value must not move a single bit.
    let same = t.clone().with_vertical_baseline_m(30.0);
    for i in 0..600 {
        let s = f64::from(i);
        assert_eq!(t.curvature_v(s).to_bits(), same.curvature_v(s).to_bits());
        assert_eq!(t.grade(s).to_bits(), same.grade(s).to_bits());
    }
}

#[test]
fn shorter_baseline_resolves_the_ripple() {
    let t30 = bumpy_line();
    let t6 = bumpy_line().with_vertical_baseline_m(6.0);
    assert_eq!(t6.vertical_baseline_m(), 6.0);

    let (kv30, g30) = peaks(&t30);
    let (kv6, g6) = peaks(&t6);
    // The 30 m baseline spans 2.5 ripple wavelengths: it filters the bump almost entirely. The 6 m
    // baseline (half a wavelength) transmits ~81% of the true κ_v — an order of magnitude more.
    assert!(
        kv6 > 5.0 * kv30,
        "6 m baseline must resolve the 12 m ripple the 30 m one filters: |κ_v| {kv6:.5} vs {kv30:.5}"
    );
    assert!(
        g6 > 2.0 * g30,
        "grade must sharpen under the short baseline: {g6:.5} vs {g30:.5}"
    );
    // Sanity: the resolved κ_v approaches the analytic peak A·k² ≈ 0.0274 (attenuation ~0.81).
    let true_kv = 0.1 * (2.0 * std::f64::consts::PI / 12.0).powi(2);
    assert!(
        kv6 > 0.5 * true_kv && kv6 < 1.2 * true_kv,
        "resolved |κ_v| {kv6:.5} should sit near the attenuated analytic peak {true_kv:.5}"
    );
}

#[test]
fn offset_line_inherits_the_parent_baseline() {
    let parent = bumpy_line().with_vertical_baseline_m(6.0);
    let s: Vec<f64> = (0..=120).map(|i| 5.0 * f64::from(i)).collect();
    let n = vec![0.0; s.len()];
    let line = offset_track(&parent, &s, &n, "offset").unwrap();
    assert_eq!(
        line.vertical_baseline_m(),
        6.0,
        "a generated line must keep the recorded override of the track it offsets"
    );
}
