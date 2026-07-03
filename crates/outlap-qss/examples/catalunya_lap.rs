// SPDX-License-Identifier: AGPL-3.0-only
//! First lap time on the real Catalunya track (§18 Day 5).
//!
//! Loads a vehicle + the imported Catalunya track, runs the T0 point-mass solver on the centerline,
//! prints the lap time and top speed, and writes a CSV for `plot_lap.py`.
//!
//! ```text
//! cargo run -p outlap-qss --example catalunya_lap [-- --vehicle <dir>] [--out <csv>]
//! ```
//! `--vehicle` is a directory holding a `vehicle.yaml` (and its referenced `.ptm`/`.tyr` files),
//! defaulting to the bundled `data/vehicles/f1_2026`.
#![allow(clippy::doc_markdown)]

use std::error::Error;
use std::fmt::Write as _;
use std::path::PathBuf;

use outlap_qss::{solve_lap, LineDescriptor, T0Options, T0Path, T0Vehicle, DEFAULT_DS_M};
use outlap_schema::io::FsLoader;
use outlap_schema::{load_conditions, load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

/// Parse `--vehicle <dir>` / `--out <csv>` flags (both optional).
fn parse_args() -> (PathBuf, Option<PathBuf>) {
    let mut veh_dir = data("vehicles/f1_2026");
    let mut out = None;
    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        match flag.as_str() {
            "--vehicle" => {
                if let Some(v) = args.next() {
                    veh_dir = PathBuf::from(v);
                }
            }
            "--out" => out = args.next().map(PathBuf::from),
            other => out = Some(PathBuf::from(other)), // bare positional = output csv
        }
    }
    (veh_dir, out)
}

fn main() -> Result<(), Box<dyn Error>> {
    let (veh_dir, out_arg) = parse_args();
    let track_dir = data("tracks/catalunya");

    let vl = FsLoader::new(&veh_dir);
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default())?;
    // Conditions are optional (ISA defaults); use a file if one is present next to the vehicle.
    let conditions =
        load_conditions("conditions.yaml", &vl).unwrap_or_else(|_| Conditions::default());

    let opts = T0Options::default();
    let veh = T0Vehicle::assemble(&resolved, &conditions, &vl, &opts)?;
    let track = Track::load("track.yaml", &FsLoader::new(&track_dir))?;

    let path = T0Path::from_track(&track, DEFAULT_DS_M);
    let notes = veh.notes().to_vec();
    let lap = solve_lap(
        &veh,
        &path,
        LineDescriptor::Centerline,
        resolved.report.resolved_hash.clone(),
        notes,
    )?;

    let top = lap.v.iter().copied().fold(0.0_f64, f64::max);
    println!(
        "Track:      {} ({:.0} m, centerline)",
        track.name(),
        track.length()
    );
    println!("Lap time:   {:.3} s", lap.lap_time_s);
    println!("Top speed:  {:.1} m/s ({:.0} km/h)", top, top * 3.6);
    println!("Stations:   {} at ds = {:.2} m", lap.s.len(), path.ds);
    println!(
        "Note: point-mass T0 on the noisy imported centerline; the min-curvature line and higher"
    );
    println!("      tiers go faster. This is a sanity magnitude, not a parity result (§18).");
    for n in &lap.notes {
        println!("  - {n}");
    }

    let out = out_arg.unwrap_or_else(|| data("../python/examples/output/catalunya_t0.csv"));
    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut csv = String::from("s_m,x_m,y_m,v_mps,ax_mps2,ay_mps2,t_s\n");
    for i in 0..lap.s.len() {
        let p = track.position(lap.s[i]);
        writeln!(
            csv,
            "{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4}",
            lap.s[i], p[0], p[1], lap.v[i], lap.ax[i], lap.ay[i], lap.t[i]
        )?;
    }
    std::fs::write(&out, csv)?;
    println!("Wrote {}", out.display());
    Ok(())
}
