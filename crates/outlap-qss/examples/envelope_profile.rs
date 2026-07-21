// SPDX-License-Identifier: AGPL-3.0-only
//! TEMPORARY profiling harness for the g-g-g-v envelope generation cost. Not committed.
#![allow(
    clippy::cast_possible_truncation,
    clippy::as_conversions,
    clippy::doc_markdown,
    missing_docs,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::uninlined_format_args,
    clippy::similar_names
)]

use std::time::Instant;

use outlap_qss::{GgvEnvelope, T1Vehicle, TrimInput};
use outlap_schema::io::{FsLoader, MemLoader};
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling};
use outlap_schema::{load_vehicle, Conditions, LoadOptions};

const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

fn data(rel: &str) -> std::path::PathBuf {
    std::path::PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

/// The self-contained synthetic demo car from ggv_traces (rev-limited ~45 m/s).
fn demo_car() -> T1Vehicle {
    let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
        axes: {speed_rpm: [0.0, 12000.0], load_axis: {torque_nm: [0.0, 800.0]}, torque_nm: [0.0, 800.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 12000.0], torque_nm: [800.0, 800.0]}}\n\
        inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
    let veh = "schema: vehicle/2.0\nname: ggv_demo\n\
        chassis: {mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}\n\
        aero: {map: a.parquet, axes: [], constant: {cx_a_m2: 1.0, cz_front_a_m2: 1.5, cz_rear_a_m2: 3.0}}\n\
        suspension: {model: lumped_kc, front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}, rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}\n\
        tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}\n\
        drivetrain: {units: [{id: u0, source: ptm/u.ptm.yaml, path: [{fixed_ratio: 9.0}], wheels: [RL, RR]}]}\n\
        brakes: {balance_bar: 0.6, disc: {front: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}, rear: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}}\n";
    let loader = MemLoader::new()
        .with("vehicle.yaml", veh)
        .with("ptm/u.ptm.yaml", ptm)
        .with("tyr/slick.tyr.yaml", SLICK);
    let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap()
}

fn f1_car() -> T1Vehicle {
    let vl = FsLoader::new(data("vehicles/f1_2026"));
    let rv = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default()).unwrap();
    T1Vehicle::assemble(&rv, &Conditions::default(), &vl, false).unwrap()
}

fn time_trims(name: &str, car: &T1Vehicle, v: f64, ay: f64, ax: f64) {
    let inp = TrimInput::flat(v, ay, ax);
    // cold (full trim incl. continuation fallback)
    let t = Instant::now();
    let n = 200;
    let mut feas = 0;
    let mut iters = 0usize;
    for _ in 0..n {
        let out = car.trim(&inp);
        match out {
            outlap_qss::TrimOutcome::Converged(s) => {
                feas += 1;
                iters += s.iterations;
            }
            outlap_qss::TrimOutcome::Infeasible { iterations, .. } => iters += iterations,
        }
    }
    let dt = t.elapsed().as_secs_f64() / n as f64;
    println!(
        "{name:36} v={v:5.1} ay={ay:5.1} ax={ax:5.1} -> {:7.3} ms/solve  feasible={} avg_iters={:.1}",
        dt * 1e3,
        feas > 0,
        iters as f64 / n as f64,
    );
}

fn time_env(name: &str, car: &T1Vehicle, grid: (u32, u32, u32)) {
    let res = EnvelopeRes {
        v_points: grid.0,
        ax_points: grid.1,
        g_normal_points: grid.2,
    };
    let t = Instant::now();
    let env = GgvEnvelope::generate(car, &res, FzCoupling::OneStepLag).unwrap();
    let dt = t.elapsed().as_secs_f64();
    let nodes = f64::from(grid.0 * grid.1 * grid.2);
    // Determinism checksum: sample the boundary densely and fold the bits (serial vs parallel
    // builds must print identical values).
    let [(vlo, vhi), _, (glo, ghi)] = env.domain();
    let mut sum: u64 = 0;
    for i in 0..40u32 {
        for j in 0..10u32 {
            let v = vlo + (vhi - vlo) * f64::from(i) / 39.0;
            let gn = glo + (ghi - glo) * f64::from(j) / 9.0;
            sum = sum.wrapping_mul(0x0100_0000_01b3).wrapping_add(
                env.ay_boundary(v, 0.3 * f64::from(j % 3) - 0.3, gn)
                    .to_bits(),
            );
        }
    }
    println!("    checksum: {sum:016x}");
    println!(
        "{name:20} grid {}x{}x{} = {:6.0} nodes -> {:8.2} s   ({:6.2} ms/node)  vmax={:.0}",
        grid.0,
        grid.1,
        grid.2,
        nodes,
        dt,
        dt * 1e3 / nodes,
        env.domain()[0].1
    );
}

fn main() {
    let demo = demo_car();
    let f1 = f1_car();
    println!("== single trim solves (200x each, release) ==");
    // interior feasible point
    time_trims("demo interior (v=30, ay=10)", &demo, 30.0, 10.0, 0.0);
    time_trims("f1   interior (v=30, ay=10)", &f1, 30.0, 10.0, 0.0);
    time_trims("f1   interior (v=80, ay=20)", &f1, 80.0, 20.0, 0.0);
    // near/over the boundary (infeasible probe — the bisection's upper half)
    time_trims("demo infeasible (v=30, ay=60)", &demo, 30.0, 60.0, 0.0);
    time_trims("f1   infeasible (v=30, ay=60)", &f1, 30.0, 60.0, 0.0);
    time_trims("f1   infeasible (v=80, ay=80)", &f1, 80.0, 80.0, 0.0);
    // combined-slip corner (ax too)
    time_trims("f1   combined  (v=50, ay=25, ax=8)", &f1, 50.0, 25.0, 8.0);

    println!("\n== envelope generation ==");
    time_env("demo", &demo, (8, 7, 3));
    time_env("f1", &f1, (8, 7, 3));
    time_env("demo", &demo, (16, 11, 5));
    time_env("f1", &f1, (16, 11, 5));
    if std::env::args().any(|a| a == "--full") {
        time_env("f1 FULL DEFAULT", &f1, (40, 25, 7));
    }
}
