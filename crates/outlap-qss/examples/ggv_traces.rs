// SPDX-License-Identifier: AGPL-3.0-only
//! Emit g-g-g-v envelope traces from the **real** [`GgvEnvelope`](outlap_qss::GgvEnvelope) generator
//! as CSV on stdout. `python/tools/plot_ggv_envelope.py` runs this and plots the output, so the
//! theory figure is driven by the actual model (T1 trim boundary search + Decision #31 corrections),
//! not a re-implementation.
//!
//! Five sections, one CSV `section,x,a,b,c`:
//! - `gg`: g-g sections a_y(a_x) at three speeds (flat ground). `x`=a_x, `a`=a_y, `b`=speed.
//! - `gnv`: the apparent-gravity axis — pure-lateral grip vs `g_normal` at three speeds.
//! - `speed`: pure-lateral grip vs speed (downforce raises grip with v-squared).
//! - `corr`: the Decision #31 correction to +15% grip (`a`) vs a full re-solve at that grip (`b`).
//! - `gggv`: the 4-D funnel — g-g boundary rings stacked by speed, for two apparent-gravity levels.
//!   `x`=speed, `a`=a_x, `b`=a_y (upper half), `c`=a_z (the apparent gravity `g_normal`).
#![allow(clippy::doc_markdown)]

use outlap_qss::{GgvEnvelope, T1Vehicle};
use outlap_schema::io::MemLoader;
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling};
use outlap_schema::{load_vehicle, Conditions, LoadOptions};

const G: f64 = 9.806_65;
const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

/// A rear-driven downforce car (rev-limited to a realistic top speed), assembled from an in-memory
/// fixture so the example is self-contained.
fn build_car() -> T1Vehicle {
    let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
        axes: {speed_rpm: [0.0, 12000.0], load_axis: {torque_nm: [0.0, 800.0]}, torque_nm: [0.0, 800.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 12000.0], torque_nm: [800.0, 800.0]}}\n\
        inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
    let veh = "schema: vehicle/1.0\nname: ggv_demo\n\
        chassis: {mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}\n\
        aero: {map: a.parquet, axes: [], constant: {cx_a_m2: 1.0, cz_front_a_m2: 1.5, cz_rear_a_m2: 3.0}}\n\
        suspension: {model: lumped_kc, front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}, rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}\n\
        tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}\n\
        drivetrain: {units: [{source: ptm/u.ptm.yaml, path: [{fixed_ratio: 9.0}], wheels: [RL, RR]}]}\n\
        brakes: {balance_bar: 0.6, disc: {front: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}, rear: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}}\n";
    let loader = MemLoader::new()
        .with("vehicle.yaml", veh)
        .with("ptm/u.ptm.yaml", ptm)
        .with("tyr/slick.tyr.yaml", SLICK);
    let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap()
}

fn main() {
    let car = build_car();
    let res = EnvelopeRes {
        v_points: 28,
        ax_points: 21,
        g_normal_points: 7,
    };
    let env = GgvEnvelope::generate(&car, &res, FzCoupling::OneStepLag).unwrap();
    // Full re-solve at μ_tire +15% (the correction truth).
    let mu = 1.15;
    let env_mu =
        GgvEnvelope::generate(&car.with_mu_scale(mu), &res, FzCoupling::OneStepLag).unwrap();

    let [(v_lo, v_hi), _, _] = env.domain();
    println!("# ggv envelope traces (real GgvEnvelope generator)");
    println!("section,x,a,b,c");

    // (gg) g-g sections at three speeds, flat ground.
    for &v in &[0.35_f64, 0.6, 0.9] {
        let v = v_lo + v * (v_hi - v_lo);
        let brake = env.brake_limit(v, G);
        let accel = env.accel_limit(v, G);
        for i in 0..=80 {
            let ax = -brake + (accel + brake) * f64::from(i) / 80.0;
            println!("gg,{ax:.4},{:.4},{v:.2},0", env.ay_boundary(v, ax, G));
        }
    }

    // (gnv) apparent-gravity axis: pure-lateral grip vs g_normal at three speeds.
    for &v in &[0.3_f64, 0.6, 0.9] {
        let v = v_lo + v * (v_hi - v_lo);
        for i in 0..=60 {
            let gn = 0.5 * G + 1.5 * G * f64::from(i) / 60.0;
            println!(
                "gnv,{:.4},{:.4},{v:.2},0",
                gn / G,
                env.ay_boundary(v, 0.0, gn)
            );
        }
    }

    // (speed) pure-lateral grip vs speed (downforce growing grip).
    for i in 0..=60 {
        let v = v_lo + (v_hi - v_lo) * f64::from(i) / 60.0;
        println!("speed,{v:.3},{:.4},0,0", env.ay_boundary(v, 0.0, G));
    }

    // (corr) Decision #31 correction to μ+15% vs the full re-solve truth, pure lateral, flat ground.
    for i in 0..=60 {
        let v = v_lo + (v_hi - v_lo) * f64::from(i) / 60.0;
        let corrected = env.ay_boundary_corrected(v, 0.0, G, mu, car.mass_kg, 1.0);
        let truth = env_mu.ay_boundary(v, 0.0, G);
        println!("corr,{v:.3},{corrected:.4},{truth:.4},0");
    }

    // (gggv) the 4-D funnel: the upper-half g-g boundary ring at a set of speeds, for two
    // apparent-gravity (a_z = g_normal) levels — a mild crest (8 m/s²) and a compression (15 m/s²).
    // Each ring sweeps the normalised longitudinal axis so it spans the full braking→acceleration
    // extent at that (speed, a_z); Python mirrors the upper half to close the ring.
    for &az in &[8.0_f64, 15.0] {
        for iv in 0..=7 {
            let v = v_lo + (v_hi - v_lo) * f64::from(iv) / 7.0;
            for k in 0..=40 {
                let axn = -1.0 + 2.0 * f64::from(k) / 40.0;
                let cap = if axn >= 0.0 {
                    env.accel_limit(v, az)
                } else {
                    env.brake_limit(v, az)
                };
                let ax = axn * cap;
                println!(
                    "gggv,{v:.3},{ax:.4},{:.4},{az:.1}",
                    env.ay_boundary(v, ax, az)
                );
            }
        }
    }
}
