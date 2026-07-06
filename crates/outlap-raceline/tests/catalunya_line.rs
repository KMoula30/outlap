// SPDX-License-Identifier: AGPL-3.0-only
//! The min-curvature line beats the centerline on the real Catalunya track (end-to-end with T0).
#![allow(clippy::doc_markdown)]

use outlap_qss::path::T0Path;
use outlap_qss::solver::solve_lap;
use outlap_qss::{LineDescriptor, T0Options, T0Vehicle, DEFAULT_DS_M};
use outlap_raceline::{min_curvature_line, RacelineOptions};
use outlap_schema::io::FsLoader;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

fn lap_time(veh: &T0Vehicle, track: &Track) -> f64 {
    let path = T0Path::from_track(track, DEFAULT_DS_M);
    solve_lap(
        veh,
        &path,
        LineDescriptor::Centerline,
        String::new(),
        vec![],
    )
    .unwrap()
    .lap_time_s
}

#[test]
fn min_curvature_line_lowers_the_catalunya_lap() {
    let veh_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/vehicles/f1_2026");
    let track_dir = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/tracks/catalunya_osm"
    );
    let vl = FsLoader::new(veh_dir);
    let rv = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default()).unwrap();
    let veh = T0Vehicle::assemble(&rv, &Conditions::default(), &vl, &T0Options::default()).unwrap();
    let track = Track::load("track.yaml", &FsLoader::new(track_dir)).unwrap();

    // Car half-width from chassis.track_m (1.65 m front) + a margin.
    let half_width = 1.65 / 2.0 + 0.3;
    let line = min_curvature_line(&track, half_width, &RacelineOptions::default()).unwrap();

    let centerline_lap = lap_time(&veh, &track);
    let raceline_lap = lap_time(&veh, &line.line);
    eprintln!("centerline {centerline_lap:.2} s → min-curvature {raceline_lap:.2} s");

    assert!(
        raceline_lap < centerline_lap,
        "raceline {raceline_lap:.2} not faster than centerline {centerline_lap:.2}"
    );
    // The offsets stay inside the corridor.
    assert!(line.n.iter().all(|n| n.abs() < 10.0));
}
