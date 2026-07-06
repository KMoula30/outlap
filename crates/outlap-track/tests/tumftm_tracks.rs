// SPDX-License-Identifier: AGPL-3.0-only
//! Every vendored TUMFTM `racetrack-database` circuit (LGPL-3.0 data, `data/tracks/<name>/`) loads
//! through the authoritative Rust geometry: closed loop, a hand-anchored length, sane curvature and
//! measured-width band. This is committed data (not real-world *input* — the conversion already ran
//! offline via `outlap.importers.tumftm_track`), so the test loads it directly and runs in CI.
//!
//! Length anchors are the loaded spline period; the well-known circuits are cross-checked to their
//! published lengths in the comments. Note the TUMFTM set is frozen ~2021, so a few layouts are the
//! *era* geometry (Yas Marina pre-2021 ≈ 5.55 km; Zandvoort pre-2020 ≈ 4.31 km) — hence the anchor
//! is the vendored value, not today's published figure.

use outlap_schema::io::FsLoader;
use outlap_track::Track;

/// `(directory, expected loaded length in metres)`. Cross-checks to published circuit lengths:
/// Catalunya 4.655 km, Monza 5.793 km, Spa 7.004 km, Silverstone 5.891 km, Suzuka 5.807 km,
/// **Nürburgring GP 5.148 km (the GP-Strecke, NOT the 20.8 km Nordschleife)**, IMS 4.023 km
/// (the 2.5-mile oval). All within the ±25 m band below.
const TRACKS: &[(&str, f64)] = &[
    ("austin", 5507.5),
    ("brands_hatch", 3904.5),
    ("budapest", 4376.9),
    ("catalunya", 4649.8),
    ("hockenheim", 4569.2),
    ("ims", 4022.3),
    ("melbourne", 5298.7),
    ("mexico_city", 4297.2),
    ("montreal", 4357.5),
    ("monza", 5790.2),
    ("moscow_raceway", 4063.3),
    ("norisring", 2295.8),
    ("nuerburgring", 5144.1),
    ("oschersleben", 3692.3),
    ("sakhir", 5405.8),
    ("sao_paulo", 4304.6),
    ("sepang", 5537.4),
    ("shanghai", 5445.3),
    ("silverstone", 5886.8),
    ("sochi", 5841.1),
    ("spa", 7000.1),
    ("spielberg", 4315.5),
    ("suzuka", 5802.9),
    ("yas_marina", 5546.6),
    ("zandvoort", 4316.5),
];

/// Absolute length tolerance around each anchor (m). Loose enough for spline-vs-chord variance,
/// tight enough to catch a wrong layout, unit error, or dropped/duplicated segment.
const LENGTH_TOL_M: f64 = 25.0;

#[test]
fn all_tumftm_tracks_load_sane() {
    assert_eq!(TRACKS.len(), 25, "the racetrack-database ships 25 circuits");

    for &(name, expected_len) in TRACKS {
        let dir = format!("{}/../../data/tracks/{name}", env!("CARGO_MANIFEST_DIR"));
        let t = Track::load("track.yaml", &FsLoader::new(&dir))
            .unwrap_or_else(|e| panic!("{name} failed to load: {e}"));

        // Closed loop (the importer leaves the seam ~1 sample open; the loader closes the chord).
        assert!(t.is_closed(), "{name} should be a closed loop");

        // Hand-anchored length.
        let len = t.length();
        assert!(
            (len - expected_len).abs() < LENGTH_TOL_M,
            "{name} length {len:.1} m off anchor {expected_len:.1} m (tol {LENGTH_TOL_M} m)"
        );

        // Dense resample: curvature finite + spike-free, widths measured-positive.
        let s = t.sample_uniform(5.0);
        assert!(s.s.len() > 100, "{name} resample too sparse");
        assert!(
            s.kappa_h.iter().all(|k| k.is_finite()),
            "{name} has non-finite curvature"
        );
        let max_k = s.kappa_h.iter().fold(0.0f64, |m, &k| m.max(k.abs()));
        // The tightest real hairpin (Spa's La Source, Yas Marina) sits near κ ≈ 0.17 (R ≈ 6 m); a
        // value beyond 0.30 would be an interpolation spike, not a corner.
        assert!(
            (0.004..0.30).contains(&max_k),
            "{name} max|κ| = {max_k:.4} outside the sane corner band"
        );
        // Corridor half-widths are measured (satellite imagery), strictly positive, never absurd.
        for (wl, wr) in s.width_left.iter().zip(s.width_right.iter()) {
            assert!(
                (1.0..20.0).contains(wl) && (1.0..20.0).contains(wr),
                "{name} implausible width (L={wl:.2}, R={wr:.2}) m"
            );
        }
    }
}

/// Catalunya is the Limebeer cross-check geometry (era-consistent, measured widths): confirm it has
/// genuine corners and the expected ISO-8855 handedness surface (finite, signed curvature).
#[test]
fn catalunya_has_real_corners() {
    let dir = format!("{}/../../data/tracks/catalunya", env!("CARGO_MANIFEST_DIR"));
    let t = Track::load("track.yaml", &FsLoader::new(&dir)).expect("catalunya loads");
    let s = t.sample_uniform(5.0);
    let max_k = s.kappa_h.iter().fold(0.0f64, |m, &k| m.max(k.abs()));
    assert!(
        max_k > 0.05,
        "catalunya should have tight corners (max|κ| = {max_k:.4})"
    );
    // A left/right corridor swap would not change |κ|, but genuine geometry has both signs.
    assert!(
        s.kappa_h.iter().any(|&k| k > 0.02) && s.kappa_h.iter().any(|&k| k < -0.02),
        "catalunya should turn both left and right"
    );
}
