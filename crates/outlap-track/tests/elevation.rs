// SPDX-License-Identifier: AGPL-3.0-only
//! Regression: DEM-derived elevation must produce **physical** grade and vertical curvature.
//!
//! `catalunya_osm` fuses an OSM centreline with the `eudem25m` DEM (25 m ground resolution). Taking
//! grade and vertical curvature from the elevation spline's *analytic* derivatives lets the
//! interpolating spline's second-derivative ringing masquerade as road geometry — on this circuit
//! that produced a 34-degree grade and a 3 m vertical radius, which drove a transient lap's
//! normal-load term to ~250 g and diverged the closed loop. Estimating both over a physical baseline
//! (just above the DEM resolution) recovers the real profile. This test pins that: the old
//! analytic-derivative code fails it, the baseline-difference code passes.

use outlap_schema::io::FsLoader;
use outlap_track::Track;

fn catalunya_osm() -> Track {
    let dir = format!(
        "{}/../../data/tracks/catalunya_osm",
        env!("CARGO_MANIFEST_DIR")
    );
    Track::load("track.yaml", &FsLoader::new(&dir)).expect("catalunya_osm loads")
}

#[test]
fn dem_grade_and_vertical_curvature_are_physical() {
    let t = catalunya_osm();
    let n = 2000;
    let (mut max_kv, mut max_grade) = (0.0_f64, 0.0_f64);
    for i in 0..n {
        #[allow(clippy::cast_precision_loss)]
        let s = t.length() * f64::from(i) / f64::from(n);
        max_kv = max_kv.max(t.curvature_v(s).abs());
        max_grade = max_grade.max(t.grade(s).abs());
    }
    // A real circuit's tightest vertical radius is tens of metres; grade a handful of degrees. The
    // analytic-spline code gave κ_v ≈ 0.32 (a 3 m radius) and a 34° grade — both far outside these.
    assert!(
        max_kv < 0.05,
        "max |κ_v| = {max_kv:.4} (radius {:.0} m) is not physical — elevation ringing?",
        1.0 / max_kv
    );
    assert!(
        max_grade < 15.0_f64.to_radians(),
        "max grade = {:.1}° is not physical for a race circuit",
        max_grade.to_degrees()
    );
}
