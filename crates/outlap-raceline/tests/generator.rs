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

use outlap_raceline::{
    min_curvature_line, min_curvature_line_weighted, raceline_stations, read_raceline_csv,
    write_raceline_csv, RacelineOptions,
};
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

/// A closed ellipse (`a≠b`): curvature varies around the lap, so the corridor is worth exercising
/// with a non-uniform weight (unlike a circle, where every station is symmetric).
fn ellipse(a: f64, b: f64, width: f64, n: usize) -> Track {
    // Cumulative arc length by fine sampling, then place the n stations on it.
    let m = 20 * n;
    let mut xs = Vec::with_capacity(n);
    let mut ys = Vec::with_capacity(n);
    let mut ss = Vec::with_capacity(n);
    let mut s_acc = 0.0;
    let (mut px, mut py) = (a, 0.0);
    let mut next = 0usize;
    for k in 0..=m {
        let th = 2.0 * PI * k as f64 / m as f64;
        let (x, y) = (a * th.cos(), b * th.sin());
        if k > 0 {
            s_acc += ((x - px).powi(2) + (y - py).powi(2)).sqrt();
        }
        // Emit a station every ~(perimeter/n) of accumulated length.
        while next < n && s_acc >= next as f64 * (perimeter(a, b) / n as f64) {
            let phi = 2.0 * PI * next as f64 / n as f64;
            xs.push(a * phi.cos());
            ys.push(b * phi.sin());
            ss.push(next as f64 * (perimeter(a, b) / n as f64));
            next += 1;
        }
        px = x;
        py = y;
    }
    let rows: Vec<CenterlineRow> = (0..xs.len())
        .map(|i| CenterlineRow {
            s_m: ss[i],
            x_m: xs[i],
            y_m: ys[i],
            z_m: 0.0,
            banking_deg: 0.0,
            width_left_m: width,
            width_right_m: width,
            grip_scale: 1.0,
        })
        .collect();
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "ellipse".into(),
        closed: true,
        centerline: CenterlineRef("mem".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

/// Ramanujan's approximation to the ellipse perimeter (good to ~1e-5 for moderate eccentricity).
fn perimeter(a: f64, b: f64) -> f64 {
    let h = ((a - b) / (a + b)).powi(2);
    PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()))
}

/// The weighted, linearised curvature cost `Σ wᵢ·κ_new,i²` the QP actually minimises (Heilmeier §3.1):
/// `κ_new,i = κ_r,i + (n_{i-1} − 2n_i + n_{i+1})/Δs² + κ_r,i²·n_i`, closed-loop wrap. This is the
/// objective the P/q matrices encode — a wrong assembly would not minimise it.
fn linearized_weighted_cost(kappa: &[f64], off: &[f64], w: &[f64], ds: f64) -> f64 {
    let n = kappa.len();
    let inv2 = 1.0 / (ds * ds);
    (0..n)
        .map(|i| {
            let im = (i + n - 1) % n;
            let ip = (i + 1) % n;
            let curv =
                kappa[i] + (off[im] - 2.0 * off[i] + off[ip]) * inv2 + kappa[i] * kappa[i] * off[i];
            w[i] * curv * curv
        })
        .sum()
}

#[test]
fn weighting_is_scale_invariant() {
    // Scaling every weight by a constant does not change the argmin: the time-weighted line with a
    // flat weight must match the plain min-curvature line.
    let track = ellipse(120.0, 70.0, 6.0, 260);
    let opts = RacelineOptions::default();
    let base = min_curvature_line(&track, 0.6, &opts).unwrap();
    let stations = raceline_stations(&track, opts.ds_m);
    let flat = vec![3.7_f64; stations.len()];
    let weighted = min_curvature_line_weighted(&track, 0.6, &flat, &opts).unwrap();
    assert_eq!(base.n.len(), weighted.n.len());
    for (a, b) in base.n.iter().zip(&weighted.n) {
        assert!(
            (a - b).abs() < 5e-3,
            "flat-weight offset {a} vs min-curv {b}"
        );
    }
}

#[test]
fn corner_weighting_lowers_the_weighted_cost() {
    // Up-weighting the high-curvature stations (where a slow car spends time) must yield a line with
    // a lower *weighted* curvature cost than the flat min-curvature line — the whole point of the
    // Δt-weighted QP (Decision #10).
    let track = ellipse(140.0, 60.0, 7.0, 300);
    let opts = RacelineOptions::default();
    let stations = raceline_stations(&track, opts.ds_m);
    // Weight ∝ |κ_r| (a monotone proxy for Δt in a corner), floored so straights keep a small say.
    let w: Vec<f64> = stations
        .iter()
        .map(|&s| track.curvature_h(s).abs() + 1e-3)
        .collect();
    let kappa: Vec<f64> = stations.iter().map(|&s| track.curvature_h(s)).collect();
    let ds = track.length() / stations.len() as f64; // closed loop: n_seg == n
    let flat_line = min_curvature_line(&track, 0.6, &opts).unwrap();
    let weighted_line = min_curvature_line_weighted(&track, 0.6, &w, &opts).unwrap();
    let flat_cost = linearized_weighted_cost(&kappa, &flat_line.n, &w, ds);
    let weighted_cost_opt = linearized_weighted_cost(&kappa, &weighted_line.n, &w, ds);
    assert!(
        weighted_cost_opt <= flat_cost + 1e-9,
        "weighted line cost {weighted_cost_opt} not ≤ flat line cost {flat_cost}"
    );
    // The weighting must actually change the line (it is not a no-op).
    let moved: f64 = flat_line
        .n
        .iter()
        .zip(&weighted_line.n)
        .map(|(a, b)| (a - b).abs())
        .sum();
    assert!(moved > 1e-3, "weighted line identical to flat line");
    // And it stays inside the corridor.
    let hi = 7.0 - 0.6;
    for &ni in &weighted_line.n {
        assert!(
            ni >= -hi - 1e-3 && ni <= hi + 1e-3,
            "offset {ni} out of bounds"
        );
    }
}

#[test]
fn weight_length_mismatch_is_an_error() {
    let track = ellipse(100.0, 80.0, 6.0, 200);
    let opts = RacelineOptions::default();
    let bad = vec![1.0; 3];
    assert!(min_curvature_line_weighted(&track, 0.6, &bad, &opts).is_err());
}
