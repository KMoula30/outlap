// SPDX-License-Identifier: AGPL-3.0-only
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::needless_range_loop,
    clippy::too_many_lines,
    clippy::format_push_string
)]
//! **Warn-only QSS↔T2 parity report** (PR5 onward). Runs a QSS T0 point-mass lap and a closed-loop
//! T2 transient lap on the *same* racing line, and **logs** the lap-time / top-speed / worst-apex
//! deltas without asserting on them — so PR10's hard gate-flip (|T2−T0| lap ≤ 0.3 %, apex ≤ 1 %) is
//! not a discovery event. The measured deltas are recorded in the PR description. The test *does*
//! assert the transient lap **closes** (completes without spinning) — that is PR5's deliverable.
//!
//! Two harness choices are documented conservatisms of the PR5 (pre-torque-vectoring) driver:
//! * the lap is seeded at the **straightest** station (`start_s`), because a cold transient — zero
//!   relaxation, running straight — seeded *at* a corner is unphysical (a real lap arrives moving);
//! * the driver tracks a fraction ([`SPEED_MARGIN`]) of the QSS profile. The point-mass profile
//!   spends the whole grip envelope; the transient rear-drive car, with only front-steer + a minimal
//!   yaw-rate stabiliser (no active torque vectoring yet — that is PR6), needs a grip margin to lap a
//!   real circuit. PR6's TV controller closes the margin; the residual T2−T0 gap is the parity signal.
//!
//! Scope note: the T0 profile is built here directly via `solve_t0` (the deliberate test-assembly
//! path; the Python dispatch that assembles all artifacts arrives in PR7). Only `limebeer_2014_f1`
//! carries an *inline* peak-torque envelope, so it runs in this pure-Rust harness; `f1_2026` and
//! `tesla_model3_rwd` reference parquet powertrain-map sidecars whose decode is the native/Python
//! edge, so they join this report through the Python boundary in PR7 and the full three-car gate in
//! PR10. The heavy T2 lap runs in **release** only (a debug `cargo test` prints a skip note).

mod common;

use outlap_core::bus::ChannelInterner;
use outlap_qss::path::T0Path;
use outlap_qss::{
    solve_t0, GgvEnvelope, LineDescriptor, T0Options, T0Vehicle, T1Vehicle, DEFAULT_DS_M,
};
use outlap_raceline::{min_curvature_line, RacelineOptions};
use outlap_schema::io::FsLoader;
use outlap_schema::sim::{FzCoupling, Sim};
use outlap_schema::{load_conditions, load_vehicle, LoadOptions};
use outlap_track::Track;
use outlap_transient::{LineSamples, LineTable, SimConfig, TransientSolver};

const CAR_HALF_WIDTH_M: f64 = 1.1;
/// Fraction of the QSS speed profile the PR5 ideal driver tracks (grip margin pending PR6's TV).
const SPEED_MARGIN: f64 = 0.85;

/// Build a flat T2 [`LineTable`] from a racing line + the T0 path/profile. The chassis + driver
/// curvature comes from the T0 path's **own smoothed** curvature (`kappa_l`), so `κ_ref` aligns with
/// the `v_ref` the point-mass solver derived from it — feeding the driver the raw (unsmoothed) line
/// curvature instead makes it try to corner harder than the profile ever braked for. Grade/banking/
/// vertical curvature are zeroed (flat, matching `from_track_flat`); the world trajectory comes from
/// the actual racing line.
fn line_from_track(line: &Track, path: &T0Path, v_ref: &[f64]) -> LineTable<f64> {
    let s = &path.s;
    let n = s.len();
    let mut samples = LineSamples {
        s: s.clone(),
        kappa_h: path.kappa_l.clone(),
        grade: vec![0.0; n],
        banking: vec![0.0; n],
        kappa_v: vec![0.0; n],
        n_ref: vec![0.0; n],
        kappa_ref: path.kappa_l.clone(),
        v_ref: v_ref.to_vec(),
        x_ref: Vec::with_capacity(n),
        y_ref: Vec::with_capacity(n),
        z_ref: vec![0.0; n],
        lat_x: Vec::with_capacity(n),
        lat_y: Vec::with_capacity(n),
        lat_z: vec![0.0; n],
        closed: path.closed,
    };
    for &si in s {
        let f = line.road_frame(si);
        samples.x_ref.push(f.origin[0]);
        samples.y_ref.push(f.origin[1]);
        samples.lat_x.push(f.lateral[0]);
        samples.lat_y.push(f.lateral[1]);
    }
    LineTable::new(&samples).expect("valid parity line table")
}

/// The nearest-sample speed at arc length `s_query` (the T2 trace is dense at 1 ms).
fn speed_at(s: &[f64], v: &[f64], s_query: f64) -> f64 {
    let mut best = 0;
    let mut best_d = f64::INFINITY;
    for i in 0..s.len() {
        let d = (s[i] - s_query).abs();
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    v[best]
}

/// Index of the straightest station (min |κ|) — where the cold lap is seeded.
fn straightest(kappa: &[f64]) -> usize {
    (0..kappa.len())
        .min_by(|&a, &b| kappa[a].abs().partial_cmp(&kappa[b].abs()).unwrap())
        .unwrap()
}

#[test]
fn qss_t2_parity_report_limebeer_catalunya() {
    if cfg!(debug_assertions) {
        eprintln!(
            "[parity] skipped in debug — run `cargo test -p outlap-transient --release \
             --test parity_report -- --nocapture` for the full lap"
        );
        return;
    }

    // --- QSS T0 lap on the flat racing line (the reference the driver tracks). ---
    let vl = FsLoader::new(common::data("vehicles/limebeer_2014_f1"));
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default()).unwrap();
    let conditions = load_conditions("conditions.yaml", &vl).unwrap();
    let track = Track::load(
        "track.yaml",
        &FsLoader::new(common::data("tracks/catalunya_osm")),
    )
    .unwrap();
    let rl = min_curvature_line(&track, CAR_HALF_WIDTH_M, &RacelineOptions::default()).unwrap();
    let path = T0Path::from_track_flat(&rl.line, DEFAULT_DS_M);

    let sim = Sim::default();
    let t1 = T1Vehicle::assemble(&resolved, &conditions, &vl, false).unwrap();
    let env = GgvEnvelope::generate(&t1, &sim.envelope, sim.resolved_fz_coupling()).unwrap();
    let t0v = T0Vehicle::assemble(&resolved, &conditions, &vl, &T0Options::default()).unwrap();
    let t0 = solve_t0(
        &t0v,
        env,
        None,
        &path,
        LineDescriptor::MinCurvature {
            ds_m: RacelineOptions::default().ds_m,
            iterations: 1,
        },
        resolved.report.resolved_hash.clone(),
        t0v.notes().to_vec(),
        sim.resolved_fz_coupling(),
        true,
    )
    .unwrap();
    let t0r = &t0.lap;

    // --- T2 transient lap: seeded at the straightest station, tracking SPEED_MARGIN × the profile. ---
    let mut it = ChannelInterner::new();
    let blocks = common::build_blocks(&t1, &mut it);
    let v_target: Vec<f64> = t0r.v.iter().map(|v| v * SPEED_MARGIN).collect();
    let line = line_from_track(&rl.line, &path, &v_target);
    let start_i = straightest(&path.kappa_l);
    let start_s = path.s[start_i];
    let length = *t0r.s.last().unwrap();
    let cfg = SimConfig {
        fz_coupling: FzCoupling::FixedPoint, // the T2 default
        start_s,
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(blocks, line, &it, cfg);
    let t2 = solver.run(start_s + length, 400_000);

    // Optional CSV dump for the PR figures (T0 profile + T2 traces), gated by an env var.
    if let Ok(dir) = std::env::var("OUTLAP_PARITY_CSV") {
        let mut t0_csv = String::from("s,v\n");
        for i in 0..t0r.s.len() {
            t0_csv.push_str(&format!("{},{}\n", t0r.s[i], t0r.v[i]));
        }
        std::fs::write(format!("{dir}/t0_catalunya.csv"), t0_csv).ok();
        let mut t2_csv = String::from("s,vx,n,steer,beta\n");
        for i in 0..t2.len() {
            let beta = t2.vy[i].atan2(t2.vx[i].max(0.5));
            t2_csv.push_str(&format!(
                "{},{},{},{},{}\n",
                t2.s[i].rem_euclid(length),
                t2.vx[i],
                t2.n[i],
                t2.steer[i],
                beta
            ));
        }
        std::fs::write(format!("{dir}/t2_catalunya.csv"), t2_csv).ok();
    }

    // --- Did the lap close? (asserted) + how big is the T2−T0 delta? (warn-only) ---
    let max_beta = (0..t2.len())
        .map(|i| t2.vy[i].atan2(t2.vx[i].max(0.5)).abs())
        .fold(0.0_f64, f64::max);
    let s_reached = t2.s.last().copied().unwrap_or(0.0);
    let closed = !max_beta.is_nan() && max_beta < 0.30 && s_reached >= start_s + length - 5.0;

    let t0_time = t0r.lap_time_s;
    let t2_time = t2.lap_time_s;
    let lap_pct = 100.0 * (t2_time - t0_time) / t0_time;
    let t0_top = t0r.v.iter().copied().fold(0.0_f64, f64::max);
    let t2_top = t2.vx.iter().copied().fold(0.0_f64, f64::max);
    let top_pct = 100.0 * (t2_top - t0_top) / t0_top;

    // Worst apex (local-minimum) speed delta vs the RAW T0 (the PR10 reference), so the margin shows.
    // The T2 lap runs `s ∈ [start_s, start_s+length]`, so wrap it back onto `[0, length]` to compare
    // at the same station as the T0 apex.
    let t2_s_wrapped: Vec<f64> = t2.s.iter().map(|s| s.rem_euclid(length)).collect();
    let mut worst_apex_pct = 0.0_f64;
    let mut n_apex = 0;
    for i in 3..t0r.v.len() - 3 {
        let v = t0r.v[i];
        if !(t0r.v[i - 3] > v && t0r.v[i + 3] > v && v < 0.9 * t0_top) {
            continue;
        }
        n_apex += 1;
        let t2v = speed_at(&t2_s_wrapped, &t2.vx, t0r.s[i]);
        worst_apex_pct = worst_apex_pct.max(100.0 * (t2v - v).abs() / v.max(1.0));
    }

    eprintln!("======== QSS↔T2 parity (WARN-ONLY) — limebeer_2014_f1 / catalunya_osm ========");
    eprintln!(
        "  lap closed = {closed}  (max|β| = {max_beta:.3} rad, seeded at s = {start_s:.0} m)"
    );
    eprintln!("  T0 lap = {t0_time:7.2} s    T2 lap = {t2_time:7.2} s    Δlap = {lap_pct:+.1}%   (PR10 gate ≤ 0.3%)");
    eprintln!("  T0 top = {t0_top:7.2} m/s  T2 top = {t2_top:7.2} m/s  Δtop = {top_pct:+.1}%");
    eprintln!("  worst apex Δ = {worst_apex_pct:.1}%  over {n_apex} apexes   (PR10 gate ≤ 1%)");
    eprintln!(
        "  note: T2 tracks {:.0}% of the QSS profile (PR5 grip margin); PR6's torque vectoring",
        SPEED_MARGIN * 100.0
    );
    eprintln!("        removes the margin. f1_2026 + tesla_model3_rwd join via Python in PR7.");
    eprintln!("=============================================================================");

    // PR5 deliverable (asserted): the closed-loop lap completes without spinning. The parity DELTAS
    // above are warn-only — PR10 turns them into the hard |T2−T0| gate.
    assert!(t2.len() > 100, "T2 recorded a full lap");
    assert!(
        closed,
        "T2 lap did not close: max|β| = {max_beta:.3}, s = {s_reached:.0}"
    );
}
