// SPDX-License-Identifier: AGPL-3.0-only
//! Min-curvature generator tests on synthetic tracks.
#![allow(
    clippy::many_single_char_names,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::doc_markdown
)]

use std::f64::consts::PI;

use outlap_raceline::{min_curvature_line, read_raceline_csv, write_raceline_csv, RacelineOptions};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_track::Track;

fn circle(r: f64, width: f64, n: usize) -> Track {
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let th = 2.0 * PI * i as f64 / n as f64;
            CenterlineRow {
                s_m: r * th,
                x_m: r * th.cos(),
                y_m: r * th.sin(),
                z_m: 0.0,
                banking_deg: 0.0,
                width_left_m: width,
                width_right_m: width,
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

/// ∫κ² ≈ Σ κ_h(s)²·ds over a track.
fn curvature_energy(track: &Track, ds: f64) -> f64 {
    let n = (track.length() / ds).round() as usize;
    let ds = track.length() / n as f64;
    (0..n)
        .map(|i| track.curvature_h(i as f64 * ds).powi(2) * ds)
        .sum()
}

#[test]
fn circle_line_hugs_the_outside_edge() {
    // A CCW circle: +n is road-left (inside); the min-curvature (max-radius) line offsets OUTWARD,
    // i.e. to the negative (right) bound, giving radius R + (width − half_width).
    let r = 100.0;
    let width = 10.0;
    let half_width = 0.5;
    let track = circle(r, width, 240);
    let opts = RacelineOptions {
        margin_m: 0.0,
        ..RacelineOptions::default()
    };
    let line = min_curvature_line(&track, half_width, &opts).unwrap();

    let n_lo = -(width - half_width); // −9.5
                                      // Every offset sits at the outer bound (within a few cm).
    for &ni in &line.n {
        assert!(
            (ni - n_lo).abs() < 0.1,
            "offset {ni} not at outer bound {n_lo}"
        );
    }
    // The line's radius (and curvature) match the outer circle.
    let r_line = r + (width - half_width);
    let mut u = 0.0;
    while u < line.line.length() {
        assert!(
            (line.line.curvature_h(u) - 1.0 / r_line).abs() < 5e-3,
            "line κ {} vs {}",
            line.line.curvature_h(u),
            1.0 / r_line
        );
        u += line.line.length() / 20.0;
    }
    // And it reduces the curvature energy vs the centerline.
    assert!(curvature_energy(&line.line, 2.0) < curvature_energy(&track, 2.0));
}

#[test]
fn respects_the_corridor_bounds() {
    let width = 6.0;
    let half_width = 0.5;
    let track = circle(60.0, width, 200);
    let line = min_curvature_line(&track, half_width, &RacelineOptions::default()).unwrap();
    let hi = width - half_width;
    let lo = -(width - half_width);
    for &ni in &line.n {
        assert!(
            ni >= lo - 1e-3 && ni <= hi + 1e-3,
            "offset {ni} out of [{lo}, {hi}]"
        );
    }
}

#[test]
fn csv_round_trips() {
    let track = circle(80.0, 8.0, 160);
    let line = min_curvature_line(&track, 0.5, &RacelineOptions::default()).unwrap();
    let text = write_raceline_csv(&line.s, &line.n);
    let back = read_raceline_csv(&text, &track).unwrap();
    assert_eq!(back.n.len(), line.n.len());
    for (a, b) in line.n.iter().zip(&back.n) {
        assert!((a - b).abs() < 1e-3);
    }
    // The re-read line is a usable track of the same length class.
    assert!((back.line.length() - line.line.length()).abs() < 1.0);
}
