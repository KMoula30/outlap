// SPDX-License-Identifier: AGPL-3.0-only
//! The HANDOFF §13 / Decision #48 Limebeer cross-check lap, driven by the REAL model.
//!
//! Loads the `limebeer_2014_f1` reference car (Perantoni & Limebeer 2014, VSD 52(5), Tables 3–4),
//! generates the production-resolution g-g-g-v envelope, and solves the flat-track T0 lap of the
//! Catalunya import on its min-curvature line — the configuration the validation page and the
//! `plot_limebeer.py` comparison figure use. Emits the lap channels as CSV.
//!
//! ```text
//! cargo run --release -p outlap-qss [--features parallel] --example limebeer_lap [-- --out <csv>]
//! ```
#![allow(clippy::doc_markdown)]

use std::error::Error;
use std::fmt::Write as _;
use std::path::PathBuf;

use outlap_qss::path::T0Path;
use outlap_qss::{
    solve_t0, GgvEnvelope, LineDescriptor, T0Options, T0Vehicle, T1Vehicle, DEFAULT_DS_M,
};
use outlap_raceline::{min_curvature_line, RacelineOptions};
use outlap_schema::io::FsLoader;
use outlap_schema::sim::Sim;
use outlap_schema::{load_conditions, load_vehicle, LoadOptions};
use outlap_track::Track;

/// PL2014's published optimal lap time on the 2 m grid, seconds (§4.3 of the manuscript).
const ORACLE_S: f64 = 82.43;
/// Car half-width for the min-curvature corridor, metres.
const CAR_HALF_WIDTH_M: f64 = 1.1;

fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

fn main() -> Result<(), Box<dyn Error>> {
    let out = std::env::args()
        .skip_while(|a| a != "--out")
        .nth(1)
        .map_or_else(
            || data("../python/examples/output/limebeer_catalunya.csv"),
            PathBuf::from,
        );

    let vl = FsLoader::new(data("vehicles/limebeer_2014_f1"));
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default())?;
    let conditions = load_conditions("conditions.yaml", &vl)?; // pins rho = 1.2 kg/m^3 (Table 4)
    let track = Track::load("track.yaml", &FsLoader::new(data("tracks/catalunya")))?;

    // The min-curvature line on the corridor, then the FLAT path (Decision #48 / sim.flat_track:
    // PL2014 is a 2-D study, so grade/banking/vertical curvature are zeroed).
    let rl = min_curvature_line(&track, CAR_HALF_WIDTH_M, &RacelineOptions::default())?;
    let path = T0Path::from_track_flat(&rl.line, DEFAULT_DS_M);

    let sim = Sim::default(); // production 40x25x7 envelope grid
    let t1 = T1Vehicle::assemble(&resolved, &conditions, &vl, false)?;
    let env = GgvEnvelope::generate(&t1, &sim.envelope, sim.fz_coupling)?;
    let t0 = T0Vehicle::assemble(&resolved, &conditions, &vl, &T0Options::default())?;

    let lap = solve_t0(
        &t0,
        env,
        None,
        &path,
        LineDescriptor::MinCurvature {
            ds_m: RacelineOptions::default().ds_m,
            iterations: 1,
        },
        resolved.report.resolved_hash.clone(),
        t0.notes().to_vec(),
        sim.fz_coupling,
        true,
    )?;

    let r = &lap.lap;
    println!(
        "Limebeer flat-track lap: {:.2} s (PL2014 oracle {ORACLE_S:.2} s, {:+.2}%)",
        r.lap_time_s,
        100.0 * (r.lap_time_s - ORACLE_S) / ORACLE_S
    );
    println!(
        "Top speed: {:.1} m/s (PL2014 Fig. 8: ~88 m/s)",
        r.v.iter().copied().fold(0.0_f64, f64::max)
    );

    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut csv = String::from("s_m,v_mps,ax_mps2,ay_mps2,t_s\n");
    for i in 0..r.s.len() {
        writeln!(
            csv,
            "{:.3},{:.4},{:.4},{:.4},{:.4}",
            r.s[i], r.v[i], r.ax[i], r.ay[i], r.t[i]
        )?;
    }
    std::fs::write(&out, csv)?;
    println!("Wrote {}", out.display());
    Ok(())
}
