// SPDX-License-Identifier: AGPL-3.0-only
//! Tests for `offset_track` — a laterally-offset line is itself a valid, correctly-shaped `Track`.
#![allow(clippy::many_single_char_names, clippy::cast_precision_loss)]

use std::f64::consts::PI;

use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_track::{offset_track, Track, TrackError};

fn ccw_circle(r: f64, n: usize) -> Track {
    // Counter-clockwise (θ increasing) flat circle → left turn, lateral points inward.
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let theta = 2.0 * PI * i as f64 / n as f64;
            CenterlineRow {
                s_m: r * theta,
                x_m: r * theta.cos(),
                y_m: r * theta.sin(),
                z_m: 0.0,
                banking_deg: 0.0,
                width_left_m: 8.0,
                width_right_m: 8.0,
                grip_scale: 1.0,
            }
        })
        .collect();
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "circle".into(),
        closed: true,
        centerline: CenterlineRef("mem".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

/// Sample the parent stations without the wrap-duplicate endpoint.
fn stations(track: &Track, ds: f64) -> Vec<f64> {
    let s = track.sample_uniform(ds).s;
    s[..s.len() - 1].to_vec()
}

#[test]
fn constant_inward_offset_shrinks_a_circle() {
    let r = 50.0;
    let parent = ccw_circle(r, 160);
    let s = stations(&parent, 2.0);
    // +n is road-left; on a CCW circle that is inward → radius r − n.
    let off = 10.0;
    let n = vec![off; s.len()];
    let line = offset_track(&parent, &s, &n, "inner").unwrap();

    assert!(line.is_closed());
    let r_line = r - off;
    assert!(
        (line.length() - 2.0 * PI * r_line).abs() / (2.0 * PI * r_line) < 2e-3,
        "length {} vs want {}",
        line.length(),
        2.0 * PI * r_line
    );
    let mut u = 0.0;
    while u < line.length() {
        assert!(
            (line.curvature_h(u) - 1.0 / r_line).abs() < 5e-3,
            "κ {} vs want {} at s={u}",
            line.curvature_h(u),
            1.0 / r_line
        );
        u += line.length() / 40.0;
    }
}

#[test]
fn outward_offset_grows_a_circle_and_shifts_widths() {
    let r = 40.0;
    let parent = ccw_circle(r, 140);
    let s = stations(&parent, 2.0);
    let n = vec![-6.0; s.len()]; // road-right → outward → radius r + 6
    let line = offset_track(&parent, &s, &n, "outer").unwrap();
    assert!((line.length() - 2.0 * PI * (r + 6.0)).abs() / (2.0 * PI * (r + 6.0)) < 2e-3);
    // Residual widths: moved 6 m right, so left width grows, right width shrinks.
    let (wl, wr) = line.width(10.0);
    assert!((wl - 14.0).abs() < 0.2, "left width {wl}"); // 8 − (−6)
    assert!((wr - 2.0).abs() < 0.2, "right width {wr}"); // 8 + (−6)
}

#[test]
fn straight_offset_stays_straight_and_parallel() {
    // Open straight along +x; offsetting by a constant keeps it straight, shifted in y.
    let rows: Vec<CenterlineRow> = (0..=20)
        .map(|i| CenterlineRow {
            s_m: f64::from(i) * 10.0,
            x_m: f64::from(i) * 10.0,
            y_m: 0.0,
            z_m: 0.0,
            banking_deg: 0.0,
            width_left_m: 5.0,
            width_right_m: 5.0,
            grip_scale: 1.0,
        })
        .collect();
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "straight".into(),
        closed: false,
        centerline: CenterlineRef("mem".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    let parent = Track::from_doc(&doc, &Centerline { rows }).unwrap();
    let s: Vec<f64> = (0..=20).map(|i| f64::from(i) * 10.0).collect();
    let n = vec![2.0; s.len()];
    let line = offset_track(&parent, &s, &n, "shifted").unwrap();
    for &u in &[15.0, 55.0, 120.0, 180.0] {
        assert!(line.curvature_h(u).abs() < 1e-6, "not straight at {u}");
        let p = line.position(u);
        assert!((p[1] - 2.0).abs() < 1e-6, "y offset wrong: {}", p[1]); // +left = +y
    }
    assert!((line.length() - 200.0).abs() < 1e-6);
}

#[test]
fn rejects_mismatched_lengths() {
    let parent = ccw_circle(30.0, 80);
    let err = offset_track(&parent, &[0.0, 1.0, 2.0, 3.0], &[0.0, 0.0], "x").unwrap_err();
    assert!(matches!(err, TrackError::MismatchedOffsets { s: 4, n: 2 }));
}

#[test]
fn rejects_too_few_points() {
    let parent = ccw_circle(30.0, 80);
    let err = offset_track(&parent, &[0.0, 1.0], &[0.0, 0.0], "x").unwrap_err();
    assert!(matches!(err, TrackError::TooFewPoints { got: 2, min: 4 }));
}
