// SPDX-License-Identifier: AGPL-3.0-only
//! Sanity checks for the committed `spa_osm` 3-D import (OSM centreline + `eudem25m` DEM).
//!
//! Spa-Francorchamps is the M4 3-D showcase: OSM splits it into corner-named `highway=raceway` ways
//! plus pit/kart ways, so the importer assembles the main loop by pruning spurs to the 2-core and
//! resolving the pit-bypass theta junction to the two-longest-path cycle (`osm_track._assemble_circuit`).
//! This pins that the committed asset is the real circuit: closed, right length, with Spa's famous
//! ~100 m of elevation change captured as *physical* grade and vertical curvature (not spline ringing).

use outlap_schema::io::FsLoader;
use outlap_track::Track;

fn spa_osm() -> Track {
    let dir = format!("{}/../../data/tracks/spa_osm", env!("CARGO_MANIFEST_DIR"));
    Track::load("track.yaml", &FsLoader::new(&dir)).expect("spa_osm loads")
}

#[test]
fn spa_osm_is_a_sane_closed_circuit() {
    let t = spa_osm();
    assert!(t.is_closed(), "the assembled Spa lap must be a closed circuit");
    // Official GP layout is 7004 m; the OSM centreline reproduces it to well within 1%.
    let len = t.length();
    assert!(
        (6800.0..7200.0).contains(&len),
        "Spa length {len:.0} m is not the ~7004 m GP layout — bad way assembly?"
    );
}

#[test]
fn spa_osm_elevation_and_geometry_are_physical() {
    let t = spa_osm();
    let n = 3000;
    let (mut max_kv, mut max_grade, mut max_kh) = (0.0_f64, 0.0_f64, 0.0_f64);
    let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for i in 0..n {
        #[allow(clippy::cast_precision_loss)]
        let s = t.length() * f64::from(i) / f64::from(n);
        let z = t.position(s)[2];
        zmin = zmin.min(z);
        zmax = zmax.max(z);
        max_kv = max_kv.max(t.curvature_v(s).abs());
        max_grade = max_grade.max(t.grade(s).abs());
        max_kh = max_kh.max(t.curvature_h(s).abs());
    }
    // Spa climbs ~100 m from Eau Rouge to Les Combes; anything under a few tens of metres means the
    // DEM fusion silently failed.
    assert!(
        zmax - zmin > 60.0,
        "Spa elevation span {:.0} m is too small — DEM fusion failed?",
        zmax - zmin
    );
    // Physical bounds (same regression the catalunya_osm test guards): tens-of-metres vertical radii,
    // a handful of degrees of grade, a metres-scale tightest corner (La Source hairpin).
    assert!(max_kv < 0.05, "max |κ_v| = {max_kv:.4} is not physical");
    assert!(
        max_grade < 15.0_f64.to_radians(),
        "max grade = {:.1}° is not physical",
        max_grade.to_degrees()
    );
    // Plan-view curvature comes straight from the raw OSM x/y (widths defaulted, no smoothing), so
    // the tight hairpin (La Source) carries some noding noise; only guard against *degenerate*
    // sub-metre spikes that would signal a duplicate-node / bad-assembly artifact.
    assert!(
        max_kh.is_finite() && 1.0 / max_kh > 2.0,
        "tightest corner radius {:.1} m is degenerate — bad assembly?",
        1.0 / max_kh
    );
}
