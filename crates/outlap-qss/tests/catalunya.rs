// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end: the f1_2026 reference vehicle on the real Catalunya track — a lap-time sanity band
//! (§18 Day 5: magnitude, not parity) and the < 50 ms performance target (§11.2).
#![allow(clippy::doc_markdown)]

use std::time::Instant;

use outlap_qss::path::T0Path;
use outlap_qss::solver::{solve_into, solve_lap};
use outlap_qss::{LineDescriptor, T0Options, T0Vehicle, T0Workspace, DEFAULT_DS_M};
use outlap_schema::io::FsLoader;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

fn f1_on_catalunya() -> (T0Vehicle, Track) {
    let veh_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/vehicles/f1_2026");
    let track_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/tracks/catalunya");
    let vl = FsLoader::new(veh_dir);
    let rv = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default()).expect("f1_2026 loads");
    let veh = T0Vehicle::assemble(&rv, &Conditions::default(), &vl, &T0Options::default())
        .expect("assembles");
    let track = Track::load("track.yaml", &FsLoader::new(track_dir)).expect("catalunya loads");
    (veh, track)
}

#[test]
fn f1_lap_time_is_in_a_sane_band() {
    let (veh, track) = f1_on_catalunya();
    let path = T0Path::from_track(&track, DEFAULT_DS_M);
    let lap = solve_lap(
        &veh,
        &path,
        LineDescriptor::Centerline,
        String::new(),
        vec![],
    )
    .unwrap();

    eprintln!(
        "Catalunya T0 lap: {:.2} s, top speed {:.1} m/s ({:.0} km/h), {} stations",
        lap.lap_time_s,
        lap.v.iter().copied().fold(0.0, f64::max),
        lap.v.iter().copied().fold(0.0, f64::max) * 3.6,
        lap.v.len()
    );

    // Real F1 laps Catalunya in ~72–80 s; a point-mass T0 should land in a wide sanity band.
    assert!(
        (60.0..120.0).contains(&lap.lap_time_s),
        "lap {:.2} s outside the sanity band",
        lap.lap_time_s
    );
    // All finite, positive.
    assert!(lap.v.iter().all(|v| v.is_finite() && *v > 0.0));
}

#[test]
fn solve_is_under_50ms() {
    let (veh, track) = f1_on_catalunya();
    let path = T0Path::from_track(&track, DEFAULT_DS_M);
    let mut ws = T0Workspace::for_path(&path);
    // Warm, then time the median of several solves.
    solve_into(&veh, &path, &mut ws).unwrap();
    let mut times = Vec::new();
    for _ in 0..11 {
        let t = Instant::now();
        solve_into(&veh, &path, &mut ws).unwrap();
        times.push(t.elapsed().as_secs_f64());
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = times[times.len() / 2];
    eprintln!("Catalunya solve_into median: {:.3} ms", median * 1e3);
    assert!(
        median < 0.050,
        "solve took {:.1} ms (> 50 ms)",
        median * 1e3
    );
}

/// The §11.5 QSS-lap perf target for the PRODUCTION path (T0-on-envelope, Decision #31): the
/// velocity-profile solve on a pre-generated envelope stays under 50 ms. Envelope generation is
/// the documented cold assembly step and is excluded (user decision, M3 PR8+9). Release only —
/// the target is a production number, not a debug-build property. (The instruction-count
/// iai-callgrind trend gate is deferred until a valgrind-equipped runner exists; the dhat
/// zero-alloc gate covers the same kernels in `tests/alloc.rs`.)
#[cfg(not(debug_assertions))]
#[test]
fn ggv_solve_is_under_50ms() {
    use outlap_qss::solver::solve_into_ggv;
    use outlap_qss::{GgvEnvelope, T1Vehicle};
    use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling};

    let veh_dir = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/vehicles/limebeer_2014_f1"
    );
    let vl = FsLoader::new(veh_dir);
    let rv = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default()).expect("loads");
    let conditions =
        outlap_schema::load_conditions("conditions.yaml", &vl).expect("conditions load");
    let t1 = T1Vehicle::assemble(&rv, &conditions, &vl, false).expect("t1 assembles");
    // Test-speed grid: the SOLVE cost is grid-independent (interpolated queries), so a coarse
    // envelope times the same hot path the production 40x25x7 grid does.
    let res = EnvelopeRes {
        v_points: 10,
        ax_points: 7,
        g_normal_points: 2,
    };
    let env = GgvEnvelope::generate(&t1, &res, FzCoupling::OneStepLag).expect("envelope");
    let veh = {
        let opts = outlap_qss::T0Options::default();
        T0Vehicle::assemble(&rv, &conditions, &vl, &opts).expect("t0 assembles")
    };
    let track_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/tracks/catalunya");
    let track = Track::load("track.yaml", &FsLoader::new(track_dir)).expect("track loads");
    let path = T0Path::from_track_flat(&track, DEFAULT_DS_M);
    let mut ws = T0Workspace::for_path(&path);
    solve_into_ggv(&veh, &env, &path, &mut ws).unwrap(); // warm

    let mut times = Vec::new();
    for _ in 0..11 {
        let t = Instant::now();
        solve_into_ggv(&veh, &env, &path, &mut ws).unwrap();
        times.push(t.elapsed().as_secs_f64());
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = times[times.len() / 2];
    eprintln!("Catalunya solve_into_ggv median: {:.3} ms", median * 1e3);
    assert!(
        median < 0.050,
        "ggv solve took {:.1} ms (> 50 ms)",
        median * 1e3
    );
}
