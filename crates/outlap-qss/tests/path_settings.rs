// SPDX-License-Identifier: AGPL-3.0-only
//! The path curvature-smoothing window as a recorded simulation setting (`sim.path_curvature_smooth_m`).
//!
//! MT promotes the T0 path sampler's fixed noise-rejection boxcar (6 stations half-width — ~25 m at
//! the default 2 m step) to a per-run sim numeric. This suite pins the contract: the `None` default
//! reproduces the legacy sampler bit-for-bit at ANY step (so no existing run moves), an explicit
//! window rounds to whole stations (25 m at the default step IS the legacy radius), `0.0` disables
//! smoothing (a synthetic hairpin's apex sharpens — the expected direction), and the path records
//! the window it actually applied.
#![allow(
    // Bit-identity is the contract under test: exact f64 comparison is the point.
    clippy::float_cmp,
    // Geometry helpers use single-letter symbols (s, x, y, r) by convention (Decision #33).
    clippy::many_single_char_names,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]

use outlap_qss::path::{T0Path, T0PathOptions};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_track::Track;

/// An open hairpin: 40 m straight in, a 180° left arc of radius 6 m (arc length ~18.8 m — narrower
/// than the 25 m default window, so smoothing measurably rounds the apex), 40 m straight out.
/// Parameterised by arc length at 0.5 m spacing.
fn hairpin(r: f64) -> Track {
    let straight = 40.0;
    let arc = std::f64::consts::PI * r;
    let length = 2.0 * straight + arc;
    let n = (length / 0.5).ceil() as usize;
    let rows: Vec<CenterlineRow> = (0..=n)
        .map(|i| {
            let s = length * i as f64 / n as f64;
            let (x, y) = if s <= straight {
                (s, 0.0)
            } else if s <= straight + arc {
                let phi = (s - straight) / r;
                (straight + r * phi.sin(), r - r * phi.cos())
            } else {
                (straight - (s - straight - arc), 2.0 * r)
            };
            CenterlineRow {
                s_m: s,
                x_m: x,
                y_m: y,
                z_m: 0.0,
                banking_deg: 0.0,
                width_left_m: 5.0,
                width_right_m: 5.0,
                grip_scale: 1.0,
            }
        })
        .collect();
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "hairpin".into(),
        closed: false,
        centerline: CenterlineRef("mem".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0_f64, |m, x| m.max(x.abs()))
}

fn with_window(track: &Track, ds: f64, window_m: Option<f64>) -> T0Path {
    T0Path::from_track_with(
        track,
        ds,
        T0PathOptions {
            flat: false,
            curvature_smooth_m: window_m,
        },
    )
}

#[test]
fn zero_window_sharpens_the_apex() {
    let t = hairpin(6.0);
    let raw = with_window(&t, 2.0, Some(0.0));
    let smoothed = with_window(&t, 2.0, Some(25.0));

    let k_raw = max_abs(&raw.kappa_l);
    let k_smooth = max_abs(&smoothed.kappa_l);
    // Unsmoothed apex κ sits near the geometric 1/r; the 25 m boxcar (wider than the 18.8 m arc)
    // necessarily averages straight stations into the apex and rounds it off.
    assert!(
        (k_raw * 6.0 - 1.0).abs() < 0.3,
        "raw apex κ {k_raw:.4} should sit near the geometric 1/6 = 0.1667"
    );
    assert!(
        k_raw > 1.05 * k_smooth,
        "disabling smoothing must sharpen the apex: raw {k_raw:.4} vs smoothed {k_smooth:.4}"
    );

    // The applied window is recorded: 0 for the disabled boxcar, the station-rounded span otherwise.
    assert_eq!(raw.curv_smooth_m, 0.0);
    assert!(
        smoothed.curv_smooth_m > 0.0,
        "a nonzero window must record the span it applied"
    );
    let radius = (0.5 * 25.0 / smoothed.ds).round();
    assert_eq!(smoothed.curv_smooth_m, 2.0 * radius * smoothed.ds);
}

#[test]
fn none_reproduces_the_legacy_sampler_at_any_step() {
    let t = hairpin(6.0);
    // The per-consumer default (None) must be bit-identical to the historical constructors at the
    // default step AND at a non-default one (the legacy half-width is stations, not metres).
    for ds in [2.0, 5.0] {
        let legacy = T0Path::from_track(&t, ds);
        let defaulted = with_window(&t, ds, None);
        assert_eq!(legacy.kappa_l, defaulted.kappa_l, "ds {ds}");
        assert_eq!(legacy.kappa_n, defaulted.kappa_n, "ds {ds}");
        assert_eq!(legacy.s, defaulted.s, "ds {ds}");

        let legacy_flat = T0Path::from_track_flat(&t, ds);
        let defaulted_flat = T0Path::from_track_with(
            &t,
            ds,
            T0PathOptions {
                flat: true,
                curvature_smooth_m: None,
            },
        );
        assert_eq!(legacy_flat.kappa_l, defaulted_flat.kappa_l, "ds {ds}");
        assert_eq!(legacy_flat.sin_g, defaulted_flat.sin_g, "ds {ds}");
    }
}

#[test]
fn the_documented_default_window_is_the_legacy_radius_at_the_default_step() {
    let t = hairpin(6.0);
    // 25 m at the ~2 m default step rounds to the legacy 6-station half-width, tying the documented
    // metre default to the historical behavior exactly.
    let legacy = T0Path::from_track(&t, 2.0);
    let explicit = with_window(&t, 2.0, Some(25.0));
    assert_eq!(legacy.kappa_l, explicit.kappa_l);
    assert_eq!(legacy.kappa_n, explicit.kappa_n);
    // And the legacy default records the same applied span as the explicit 25 m window.
    assert_eq!(legacy.curv_smooth_m, explicit.curv_smooth_m);
    assert_eq!(legacy.curv_smooth_m, 12.0 * legacy.ds);
}

#[test]
fn an_oversized_window_is_a_recorded_no_op() {
    let t = hairpin(6.0);
    // Wider than the whole path: smooth() cannot form a full window, applies nothing, records 0.
    let p = with_window(&t, 2.0, Some(1.0e6));
    let raw = with_window(&t, 2.0, Some(0.0));
    assert_eq!(p.kappa_l, raw.kappa_l);
    assert_eq!(p.curv_smooth_m, 0.0);
}
