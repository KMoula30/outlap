// SPDX-License-Identifier: AGPL-3.0-only
//! Analytic geometry property tests — closed-form curvature/grade/banking, no real-world data.
//!
//! Every fixture is generated in-process from a known shape so the spline pipeline can be checked
//! against exact values: a flat circle (κ = 1/R), a banked oval (κ transitions + frame tilt), and a
//! crested straight (known vertical curvature and grade).
#![allow(clippy::many_single_char_names)] // geometry fixtures use r, l, s, x, y, z, a, c

use std::f64::consts::PI;

use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{BankingKeypoint, TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_track::Track;

/// A minimal centerline row with flat, unit-grip, 5 m half-width defaults.
fn row(s: f64, x: f64, y: f64, z: f64, banking_deg: f64) -> CenterlineRow {
    CenterlineRow {
        s_m: s,
        x_m: x,
        y_m: y,
        z_m: z,
        banking_deg,
        width_left_m: 5.0,
        width_right_m: 5.0,
        grip_scale: 1.0,
    }
}

fn doc(closed: bool, keypoints: Vec<BankingKeypoint>) -> TrackDoc {
    TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "analytic".into(),
        closed,
        centerline: CenterlineRef("mem.csv".into()),
        banking_keypoints: keypoints,
        meta: TrackMeta::default(),
    }
}

fn track(closed: bool, rows: Vec<CenterlineRow>) -> Track {
    Track::from_doc(&doc(closed, vec![]), &Centerline { rows }).expect("track builds")
}

// ---------------------------------------------------------------------------------------------

#[test]
fn flat_circle_has_constant_curvature() {
    let r = 50.0;
    let n = 120;
    // Counter-clockwise (θ increasing) → a left turn → positive curvature.
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let theta = 2.0 * PI * f64::from(i) / f64::from(n);
            row(r * theta, r * theta.cos(), r * theta.sin(), 0.0, 0.0)
        })
        .collect();
    let t = track(true, rows);

    // Perimeter ≈ 2πR.
    assert!((t.length() - 2.0 * PI * r).abs() / (2.0 * PI * r) < 1e-3);

    // κ ≈ 1/R everywhere; grade and vertical curvature ≈ 0.
    let mut s = 0.0;
    while s < t.length() {
        assert!(
            (t.curvature_h(s) - 1.0 / r).abs() < 1e-3,
            "κ_h at s={s} was {} (want {})",
            t.curvature_h(s),
            1.0 / r
        );
        assert!(t.curvature_v(s).abs() < 1e-4, "κ_v nonzero at s={s}");
        assert!(t.grade(s).abs() < 1e-4, "grade nonzero at s={s}");
        s += t.length() / 50.0;
    }
}

#[test]
fn banked_oval_curvature_transitions_and_frame_tilt() {
    // Stadium oval: two straights (length l) joined by two semicircles (radius r), banked at φ.
    let l = 200.0;
    let r = 40.0;
    let phi_deg = 10.0;
    let ds = 2.0;

    let mut rows = Vec::new();
    let mut s = 0.0;
    let push = |rows: &mut Vec<CenterlineRow>, s: f64, x: f64, y: f64| {
        rows.push(row(s, x, y, 0.0, phi_deg));
    };
    // Bottom straight: (−l/2,−r) → (l/2,−r).
    let mut d = 0.0;
    while d < l {
        push(&mut rows, s, -l / 2.0 + d, -r);
        d += ds;
        s += ds;
    }
    // Right semicircle: center (l/2,0), θ from −90° to +90°.
    let mut a = -PI / 2.0;
    while a < PI / 2.0 {
        push(&mut rows, s, l / 2.0 + r * a.cos(), r * a.sin());
        a += ds / r;
        s += ds;
    }
    // Top straight: (l/2,r) → (−l/2,r).
    d = 0.0;
    while d < l {
        push(&mut rows, s, l / 2.0 - d, r);
        d += ds;
        s += ds;
    }
    // Left semicircle: center (−l/2,0), θ from +90° to +270°.
    a = PI / 2.0;
    while a < 3.0 * PI / 2.0 {
        push(&mut rows, s, -l / 2.0 + r * a.cos(), r * a.sin());
        a += ds / r;
        s += ds;
    }
    let t = track(true, rows);

    // Mid-bottom-straight: curvature ≈ 0.
    assert!(t.curvature_h(l / 2.0).abs() < 5e-3, "straight not flat");
    // Mid right-semicircle (≈ bottom straight length + quarter circle): κ ≈ 1/r, left turn.
    let s_curve = l + PI * r / 2.0;
    assert!(
        (t.curvature_h(s_curve) - 1.0 / r).abs() < 5e-3,
        "curve κ was {} (want {})",
        t.curvature_h(s_curve),
        1.0 / r
    );

    // Banking is incorporated in the road frame: the surface normal tilts φ off vertical.
    let phi = phi_deg.to_radians();
    let frame = t.road_frame(s_curve);
    assert!((frame.banking - phi).abs() < 1e-6);
    let normal_dot_up = frame.normal[2];
    assert!(
        (normal_dot_up - phi.cos()).abs() < 1e-3,
        "normal·ẑ was {normal_dot_up} (want cos φ = {})",
        phi.cos()
    );
}

#[test]
fn crested_straight_matches_closed_form_vertical_curvature() {
    // A parabolic crest z(x) = z0 − ½c(x−xa)² over a straight along +x. With s ≡ x this gives
    // z'' = −c, so κ_v = −c / (1+z'²)^{3/2} and grade = atan(z').
    let c = 2.0e-3;
    let xa = 100.0;
    let z0 = 5.0;
    let rows: Vec<CenterlineRow> = (0..=200)
        .map(|i| {
            let x = f64::from(i);
            let z = z0 - 0.5 * c * (x - xa).powi(2);
            row(x, x, 0.0, z, 0.0)
        })
        .collect();
    let t = track(false, rows);

    for &x in &[40.0, 80.0, 100.0, 130.0, 170.0] {
        let zp = -c * (x - xa);
        let want_kv = -c / (1.0 + zp * zp).powf(1.5);
        let want_grade = zp.atan();
        assert!(
            (t.curvature_v(x) - want_kv).abs() < 1e-6,
            "κ_v at x={x}: {} vs {want_kv}",
            t.curvature_v(x)
        );
        assert!(
            (t.grade(x) - want_grade).abs() < 1e-6,
            "grade at x={x}: {} vs {want_grade}",
            t.grade(x)
        );
    }
    // At the apex the crest curvature is exactly −c and the grade is zero.
    assert!((t.curvature_v(xa) + c).abs() < 1e-6);
    assert!(t.grade(xa).abs() < 1e-9);
}

#[test]
fn closed_track_wraps_and_channels_are_continuous() {
    let r = 30.0;
    let n = 90;
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let theta = 2.0 * PI * f64::from(i) / f64::from(n);
            // Vary grip slightly so the channel is non-trivial, but keep it continuous at the seam.
            let grip = 1.0 + 0.05 * theta.sin();
            CenterlineRow {
                grip_scale: grip,
                ..row(r * theta, r * theta.cos(), r * theta.sin(), 0.0, 0.0)
            }
        })
        .collect();
    let t = track(true, rows);

    // Position is continuous across the wrap.
    let before = t.position(t.length() - 1e-4);
    let after = t.position(1e-4);
    let start = t.position(0.0);
    assert!((before[0] - start[0]).abs() < 0.1 && (after[0] - start[0]).abs() < 0.1);

    // Grip channel matches across the seam (first ≈ last value).
    let g_end = t.grip_scale(t.length() - 1e-4);
    let g_start = t.grip_scale(0.0);
    assert!((g_end - g_start).abs() < 1e-2, "grip discontinuous at seam");

    // Widths stay positive everywhere.
    let samples = t.sample_uniform(1.0);
    assert!(samples.width_left.iter().all(|&w| w > 0.0));
    assert!(samples.width_right.iter().all(|&w| w > 0.0));
}

#[test]
fn banking_keypoints_override_column() {
    // Column banking is zero, but keypoints declare a ramp; the keypoints must win.
    let rows: Vec<CenterlineRow> = (0..=10)
        .map(|i| row(f64::from(i) * 10.0, f64::from(i) * 10.0, 0.0, 0.0, 0.0))
        .collect();
    let keypoints = vec![
        BankingKeypoint {
            s_m: 0.0,
            banking_deg: 0.0,
        },
        BankingKeypoint {
            s_m: 100.0,
            banking_deg: 8.0,
        },
    ];
    let t = Track::from_doc(&doc(false, keypoints), &Centerline { rows }).unwrap();
    // Midpoint banking is between 0 and 8°, not the column's 0.
    let mid = t.banking(50.0).to_degrees();
    assert!(
        mid > 1.0 && mid < 8.0,
        "keypoint banking not applied: {mid}°"
    );
    assert!((t.banking(100.0).to_degrees() - 8.0).abs() < 1e-6);
}

#[test]
fn rejects_mislabelled_closed_track() {
    // An open arc (endpoints far apart) declared closed → a clear diagnostic, not a garbage fit.
    let rows: Vec<CenterlineRow> = (0..=20)
        .map(|i| {
            let x = f64::from(i) * 10.0;
            row(x, x, 0.0, 0.0, 0.0)
        })
        .collect();
    let err = Track::from_doc(&doc(true, vec![]), &Centerline { rows }).unwrap_err();
    assert!(
        matches!(err, outlap_track::TrackError::NotClosed { .. }),
        "got {err:?}"
    );
}

#[test]
fn pathological_vertical_curvature_is_clamped_to_a_physical_bound() {
    // A crest far tighter than any real road (`z'' = −c`, c = 0.1 ⇒ κ_v = −0.1 at the apex, a 10 m
    // vertical radius). The physical backstop clamps it to −0.05 (a 20 m radius) so a solver's
    // κ_v·v² normal-load term can never be driven to a divergent value by bad elevation data.
    let c = 0.1;
    let xa = 100.0;
    let rows: Vec<CenterlineRow> = (0..=200)
        .map(|i| {
            let x = f64::from(i);
            row(x, x, 0.0, 5.0 - 0.5 * c * (x - xa).powi(2), 0.0)
        })
        .collect();
    let t = track(false, rows);
    // Unclamped this would be −0.1; the guard holds it at the −0.05 bound.
    assert!(
        (t.curvature_v(xa) + 0.05).abs() < 1e-9,
        "apex κ_v {} not clamped to −0.05",
        t.curvature_v(xa)
    );
    for i in 0..=200 {
        let kv = t.curvature_v(f64::from(i));
        assert!(
            kv.abs() <= 0.05 + 1e-12,
            "κ_v {kv} exceeds the clamp at x={i}"
        );
    }
}
