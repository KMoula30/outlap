// SPDX-License-Identifier: AGPL-3.0-only
//! M5 PR5 demo: the QSS tyre-thermal slow-state march on a real lap.
//!
//! Loads the `limebeer_2014_f1` reference car + the Catalunya import, builds the g-g-g-v envelope
//! **with tyre-state axes** (`generate_with_tire_state`), and solves the flat-track T0 lap twice:
//! once frozen (`solve_t0(.., tire = None)`) and once with a cold-seeded [`TireThermalMarch`] wired
//! in, so the tyres warm into the grip window over the lap and the re-solve sees the evolving
//! `(T_tire, wear)`. Dumps two CSVs for the PR figures:
//!
//! * `tire_march_lap.csv` — per-station lap channels (frozen vs tyre-thermal speed, `T_s/T_c/T_g`,
//!   wear, grip multiplier, friction utilisation).
//! * `tire_march_axes.csv` — the envelope's peak lateral grip sampled across the `T_tire` and `wear`
//!   axes at a reference `(v, g_normal)`, i.e. the tyre-state grip surface the QSS lap indexes.
//!
//! ```text
//! cargo run --release -p outlap-qss --features parallel --example tire_march_lap
//! ```
#![allow(
    clippy::doc_markdown,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use std::error::Error;
use std::fmt::Write as _;
use std::path::PathBuf;

use outlap_qss::path::T0Path;
use outlap_qss::{
    solve_t0, GgvEnvelope, LineDescriptor, T0Options, T0Vehicle, T1Vehicle, TireStateRes,
    TireThermalMarch, DEFAULT_DS_M, G,
};
use outlap_raceline::{min_curvature_line, RacelineOptions};
use outlap_schema::io::FsLoader;
use outlap_schema::load::load_tyr;
use outlap_schema::sim::Sim;
use outlap_schema::{load_conditions, load_vehicle, LoadOptions};
use outlap_tire::TireThermalRing;
use outlap_track::Track;

const CAR_HALF_WIDTH_M: f64 = 1.1;

/// An illustrative Archard wear coefficient for the demo figure (mm-per-unit-sliding-energy scale).
/// The reference `.tyr` files ship a synthetic placeholder `k_w` that is orders too large (wear
/// saturates in a single QSS segment); the true value comes from the FastF1 inverse calibration
/// (M5 PR7/PR8). This is a plot-only override so the wear channel is legible; it does NOT touch the
/// shipped physics or the envelope's wear axis (which keys off the cliff position `w_c`, not `k_w`).
const DEMO_K_W: f64 = 1.5e-7;

fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

fn main() -> Result<(), Box<dyn Error>> {
    let out_dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../scratch_figs"));
    std::fs::create_dir_all(&out_dir).ok();

    let vl = FsLoader::new(data("vehicles/limebeer_2014_f1"));
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default())?;
    let conditions = load_conditions("conditions.yaml", &vl)?;
    let track = Track::load("track.yaml", &FsLoader::new(data("tracks/catalunya_osm")))?;

    let rl = min_curvature_line(&track, CAR_HALF_WIDTH_M, &RacelineOptions::default())?;
    let path = T0Path::from_track_flat(&rl.line, DEFAULT_DS_M);

    // A moderate envelope grid (the demo does not need the production 40×25×7; the tyre-state re-solve
    // multiplies the build by the axis product).
    let mut sim = Sim::default();
    sim.envelope.v_points = 24;
    sim.envelope.ax_points = 15;
    sim.envelope.g_normal_points = 5;
    let coupling = sim.resolved_fz_coupling();

    let t1 = T1Vehicle::assemble(&resolved, &conditions, &vl, false)?;
    let t0 = T0Vehicle::assemble(&resolved, &conditions, &vl, &T0Options::default())?;
    let env = GgvEnvelope::generate_with_tire_state(
        &t1,
        &sim.envelope,
        coupling,
        TireStateRes::default(),
    )?;

    // The tyre-thermal march. Seeded WARM at the grip optimum (the parity-safe default): the lap
    // starts bit-identical to the frozen envelope, then the surface drifts within the grip window and
    // the tread wears under load, so the re-solve sees a physical, bounded degradation (the synthetic
    // .tyr params are pre-calibration — a cold seed collapses grip, which is why the tier default is
    // opt-in until the FastF1 calibration in PR7/PR8).
    let (front, _) = load_tyr(resolved.spec.tires.front.as_str(), &vl)?;
    let c = &front.mf61.0;
    // Build the march ring from the schema, but with the illustrative demo wear rate (see DEMO_K_W).
    let mut wear = front.wear.clone();
    wear.k_w = DEMO_K_W;
    let demo_ring = TireThermalRing::<f64>::from_schema_with_wear(&front.thermal, &wear);
    let march = TireThermalMarch::new(
        demo_ring,
        c.get("UNLOADED_RADIUS").copied(),
        c.get("WIDTH").copied(),
        c.get("VERTICAL_STIFFNESS").copied(),
        front.thermal.t_opt,
        front.thermal.t_cold,
        conditions.air.temperature_c,
        conditions.track_surface_c,
    );

    let hash = resolved.report.resolved_hash.clone();
    let frozen = solve_t0(
        &t0,
        env.clone(),
        None,
        None,
        &path,
        LineDescriptor::Centerline,
        hash.clone(),
        Vec::new(),
        coupling,
        true,
    )?;
    let warm = solve_t0(
        &t0,
        env.clone(),
        None,
        Some(&march),
        &path,
        LineDescriptor::Centerline,
        hash,
        Vec::new(),
        coupling,
        true,
    )?;

    println!(
        "frozen lap  {:.3} s\ntyre lap    {:.3} s  (Δ {:+.3} s from cold-start warm-up + wear)",
        frozen.lap.lap_time_s,
        warm.lap.lap_time_s,
        warm.lap.lap_time_s - frozen.lap.lap_time_s
    );

    // --- CSV 1: the lap channels ---
    let tire = warm
        .tire
        .as_ref()
        .expect("tyre-thermal march produced channels");
    let n = path.len();
    let mut csv =
        String::from("s,v_frozen,v_tire,t_surface_c,t_carcass_c,t_gas_c,wear_mm,grip,util\n");
    for i in 0..n {
        let v = warm.lap.v[i];
        let u = v.max(1e-3) * v.max(1e-3);
        let ay = path.kappa_l[i] * u + G * path.sin_b_cos_g[i];
        let gn = G * path.cos_b_cos_g[i] + path.kappa_n[i] * u;
        let fx = t0.mass_kg * (warm.lap.ax[i] + env.drag_accel(v) + G * path.sin_g[i]);
        let fy = t0.mass_kg * ay;
        let f_tire = fx.hypot(fy);
        let f_cap = (t0.mass_kg
            * env.ay_boundary_at(
                v.max(1e-3),
                0.0,
                gn,
                tire_surface_k(tire, i),
                tire.wear_mm[i],
            ))
        .max(1.0);
        let util = (f_tire / f_cap).clamp(0.0, 1.0);
        writeln!(
            csv,
            "{:.3},{:.4},{:.4},{:.3},{:.3},{:.3},{:.5},{:.5},{:.4}",
            path.s[i],
            frozen.lap.v[i],
            warm.lap.v[i],
            tire.surface_temp_c[i],
            tire.carcass_temp_c[i],
            tire.gas_temp_c[i],
            tire.wear_mm[i],
            tire.grip_scale[i],
            util
        )?;
    }
    let lap_path = out_dir.join("tire_march_lap.csv");
    std::fs::write(&lap_path, csv)?;
    println!("wrote {}", lap_path.display());

    // --- CSV 2: the tyre-state grip surface the lap indexes (peak lateral grip vs T_tire, wear) ---
    let [(t_lo, t_hi), (w_lo, w_hi)] = env.tire_state_domain().expect("tyre-state axes present");
    let v_ref = 60.0; // a representative mid-corner speed, m/s
    let gn_ref = G; // 1 g road-normal
    let mut axes = String::from("t_tire_c,wear_mm,ay_peak\n");
    let (nt, nw) = (21usize, 21usize);
    for it in 0..nt {
        let t = t_lo + (t_hi - t_lo) * it as f64 / (nt as f64 - 1.0);
        for iw in 0..nw {
            let w = w_lo + (w_hi - w_lo) * iw as f64 / (nw as f64 - 1.0);
            let ay = env.ay_boundary_at(v_ref, 0.0, gn_ref, t, w);
            writeln!(axes, "{:.2},{:.4},{:.4}", t - 273.15, w, ay)?;
        }
    }
    let axes_path = out_dir.join("tire_march_axes.csv");
    std::fs::write(&axes_path, axes)?;
    println!("wrote {}", axes_path.display());

    Ok(())
}

/// The station's surface temperature back in kelvin (the march logs it in °C).
fn tire_surface_k(tire: &outlap_qss::TireSlowLog, i: usize) -> f64 {
    tire.surface_temp_c[i] + 273.15
}
