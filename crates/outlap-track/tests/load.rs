// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end load of the synthetic track fixture through a `SourceLoader` (exercises the
//! `track.yaml` + `centerline.csv` path, not just in-memory `from_doc`).

use std::f64::consts::PI;

use outlap_schema::io::MemLoader;
use outlap_track::Track;

const TRACK_YAML: &str =
    include_str!("../../outlap-schema/tests/fixtures/track/synthetic_oval.track.yaml");
const CENTERLINE_CSV: &str =
    include_str!("../../outlap-schema/tests/fixtures/track/synthetic_oval.centerline.csv");

#[test]
fn loads_fixture_through_loader() {
    let loader = MemLoader::new()
        .with("track.yaml", TRACK_YAML)
        .with("synthetic_oval.centerline.csv", CENTERLINE_CSV);

    let t = Track::load("track.yaml", &loader).expect("track loads");
    assert_eq!(t.name(), "Synthetic Oval");
    assert!(t.is_closed());

    // r = 100 m circle → perimeter ≈ 2πR, curvature ≈ 1/R.
    let r = 100.0;
    assert!((t.length() - 2.0 * PI * r).abs() / (2.0 * PI * r) < 1e-3);
    let mut s = 0.0;
    while s < t.length() {
        assert!((t.curvature_h(s) - 1.0 / r).abs() < 5e-3, "κ at s={s}");
        s += t.length() / 20.0;
    }

    // The resample is dense and width-positive.
    let samples = t.sample_uniform(5.0);
    assert!(samples.s.len() > 100);
    assert!(samples.width_left.iter().all(|&w| (w - 6.0).abs() < 1e-6));
}

/// Cross-check the real OSM+DEM Catalunya import (if it has been generated locally) through the
/// authoritative Rust geometry. `#[ignore]`d so CI never depends on real-world data (Decision #23 /
/// §13): run with `cargo test -p outlap-track -- --ignored`.
#[test]
#[ignore = "requires a local `outlap import` of Catalunya under data/tracks/"]
fn loads_real_catalunya_if_present() {
    use outlap_schema::io::FsLoader;

    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/tracks/catalunya");
    if !std::path::Path::new(dir).join("track.yaml").exists() {
        eprintln!("skipping: {dir}/track.yaml not present");
        return;
    }
    let t = Track::load("track.yaml", &FsLoader::new(dir)).expect("real Catalunya loads");
    assert!(t.is_closed());
    // The GP layout is ~4.65 km.
    assert!(
        (t.length() - 4650.0).abs() < 250.0,
        "length {} m off expected ~4650 m",
        t.length()
    );
    // Sanity: curvature is finite and bounded everywhere; some corners are genuinely tight.
    let samples = t.sample_uniform(5.0);
    assert!(samples.kappa_h.iter().all(|k| k.is_finite()));
    let max_kappa = samples.kappa_h.iter().fold(0.0f64, |m, &k| m.max(k.abs()));
    assert!(
        max_kappa > 0.02,
        "no real corners found (max |κ| = {max_kappa})"
    );
    assert!(
        max_kappa < 1.0,
        "implausible curvature spike (max |κ| = {max_kappa})"
    );
}

#[test]
fn missing_centerline_is_a_clean_error() {
    let loader = MemLoader::new().with("track.yaml", TRACK_YAML); // no csv
    let err = Track::load("track.yaml", &loader).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("centerline") || msg.contains("not found"),
        "{msg}"
    );
}
