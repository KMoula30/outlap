// SPDX-License-Identifier: AGPL-3.0-only
//! Compare a T0 lap on the Catalunya centerline vs the generated min-curvature line.
//!
//! ```text
//! cargo run -p outlap-raceline --example catalunya_line
//! ```
#![allow(clippy::doc_markdown)]

use std::error::Error;
use std::fmt::Write as _;
use std::path::PathBuf;

use outlap_qss::{
    solve_lap, LapResult, LineDescriptor, T0Options, T0Path, T0Vehicle, DEFAULT_DS_M,
};
use outlap_raceline::{min_curvature_line, write_raceline_csv, RacelineOptions};
use outlap_schema::io::FsLoader;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

fn lap(veh: &T0Vehicle, track: &Track, line: LineDescriptor) -> LapResult {
    let path = T0Path::from_track(track, DEFAULT_DS_M);
    solve_lap(veh, &path, line, String::new(), vec![]).unwrap()
}

/// Write `s_m,x_m,y_m,v_mps` for a lap on `track`.
fn write_lap(path: &PathBuf, track: &Track, lap: &LapResult) -> Result<(), Box<dyn Error>> {
    let mut csv = String::from("s_m,x_m,y_m,v_mps\n");
    for i in 0..lap.s.len() {
        let p = track.position(lap.s[i]);
        writeln!(
            csv,
            "{:.3},{:.3},{:.3},{:.4}",
            lap.s[i], p[0], p[1], lap.v[i]
        )?;
    }
    std::fs::write(path, csv)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let vl = FsLoader::new(data("vehicles/f1_2026"));
    let rv = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default())?;
    let veh = T0Vehicle::assemble(&rv, &Conditions::default(), &vl, &T0Options::default())?;
    let track = Track::load("track.yaml", &FsLoader::new(data("tracks/catalunya")))?;

    let half_width = rv.spec.chassis.track_m[0] / 2.0 + 0.3;
    let opts = RacelineOptions::default();
    let raceline = min_curvature_line(&track, half_width, &opts)?;

    let center_lap = lap(&veh, &track, LineDescriptor::Centerline);
    let line_lap = lap(
        &veh,
        &raceline.line,
        LineDescriptor::MinCurvature {
            ds_m: opts.ds_m,
            iterations: 1,
        },
    );

    println!("Centerline lap:      {:.3} s", center_lap.lap_time_s);
    println!("Min-curvature lap:   {:.3} s", line_lap.lap_time_s);
    println!(
        "Improvement:         {:.3} s ({:.1}%)",
        center_lap.lap_time_s - line_lap.lap_time_s,
        100.0 * (center_lap.lap_time_s - line_lap.lap_time_s) / center_lap.lap_time_s
    );

    let out = data("../python/examples/output");
    std::fs::create_dir_all(&out)?;
    write_lap(&out.join("catalunya_centerline.csv"), &track, &center_lap)?;
    write_lap(
        &out.join("catalunya_raceline.csv"),
        &raceline.line,
        &line_lap,
    )?;
    std::fs::write(
        out.join("catalunya_raceline_offsets.csv"),
        write_raceline_csv(&raceline.s, &raceline.n),
    )?;
    println!("Wrote comparison CSVs to {}", out.display());
    Ok(())
}
