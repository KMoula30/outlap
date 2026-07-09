// SPDX-License-Identifier: AGPL-3.0-only
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_lines,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::cast_lossless,
    clippy::needless_range_loop
)]
//! Open-loop steady-state cornering probe: Limebeer car, straight synthetic road, constant steer
//! feed-forward (`kappa_ref`), speed held by the placeholder driver. Prints the yaw-rate / velocity
//! transient, which should approach a bounded steady yaw `≈ v·δ/L` with the correct sign — the
//! sanity check behind the step-steer property test.
//!
//! ```text
//! cargo run --release -p outlap-transient --example diag -- <speed m/s> <kappa_ref 1/m>
//! ```

use outlap_core::bus::{ChannelInterner, WHEELS};
use outlap_qss::t1::LoadTransferGeometry;
use outlap_qss::T1Vehicle;
use outlap_schema::io::FsLoader;
use outlap_schema::sim::FzCoupling;
use outlap_schema::{load_conditions, load_vehicle, LoadOptions};
use outlap_transient::{LineSamples, LineTable, SimConfig, T2Blocks, TransientSolver};
use outlap_vehicle::{
    Aero, Chassis, ChassisParams, Driver, LoadTransfer, Powertrain, RelaxProvider, RoadChannels,
    Tire,
};
use std::path::PathBuf;

fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let vl = FsLoader::new(data("vehicles/limebeer_2014_f1"));
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default())?;
    let conditions = load_conditions("conditions.yaml", &vl)?;
    let t1 = T1Vehicle::assemble(&resolved, &conditions, &vl, false)?;

    let mut interner = ChannelInterner::new();
    let road = RoadChannels::intern(&mut interner);
    let (a, b) = (t1.a_f, t1.b_r);
    let (tf, tr) = (t1.t_f, t1.t_r);
    let x = [a, a, -b, -b];
    let y = [tf * 0.5, -tf * 0.5, tr * 0.5, -tr * 0.5];
    let front = [true, true, false, false];
    let r_f = t1.tire_front.unloaded_radius(0.33);
    let r_r = t1.tire_rear.unloaded_radius(0.33);
    let radius = [r_f, r_f, r_r, r_r];
    let params = ChassisParams::from_f64(t1.mass_kg, t1.izz, x, y, front, radius, [1.1; WHEELS]);
    let geom = LoadTransferGeometry {
        mass_kg: t1.mass_kg,
        wheelbase_m: t1.wheelbase_m,
        a_f: t1.a_f,
        b_r: t1.b_r,
        t_f: t1.t_f,
        t_r: t1.t_r,
        h_cg: t1.h_cg,
        h_ra: t1.h_ra,
        rc_f: t1.rc_f,
        rc_r: t1.rc_r,
        roll_share_f: t1.roll_share_f,
        roll_share_r: t1.roll_share_r,
    };
    let wheels = params.wheels;
    let target_v: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(40.0);
    let kappa_ref: f64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    let blocks = T2Blocks {
        chassis: Chassis::new(params, road),
        tire: Tire {
            front: t1.tire_front.clone(),
            rear: t1.tire_rear.clone(),
            p_front: t1.p_front,
            p_rear: t1.p_rear,
            mu_scale: 1.0,
            relax_front: RelaxProvider::from_model(&t1.tire_front, 0.5 * r_f),
            relax_rear: RelaxProvider::from_model(&t1.tire_rear, 0.5 * r_r),
            wheels,
        },
        aero: Aero {
            qx: t1.qx,
            qz_f: t1.qz_f,
            qz_r: t1.qz_r,
        },
        load: LoadTransfer {
            geom,
            g_normal: outlap_vehicle::G,
            speed: 0.0,
            ax: 0.0,
            ay: 0.0,
            qz_f: t1.qz_f,
            qz_r: t1.qz_r,
        },
        // Zero path gains → constant steer = L*kappa_ref; hold speed with k_speed.
        driver: Driver {
            wheelbase: t1.wheelbase_m,
            k_offset: 0.0,
            k_heading: 0.0,
            k_yaw_rate: 0.0,
            k_speed: 0.3,
            max_steer: 0.5,
            road,
        },
        powertrain: Powertrain {
            max_drive_torque: t1.max_tractive_force(50.0) * r_r,
            max_brake_torque: 6000.0,
            brake_front_bias: t1.brake_front_bias,
            driven: t1.driven,
        },
        road,
    };

    // Straight synthetic road at constant kappa_ref (steer feedforward) and constant v_ref.
    let n = 200usize;
    let len = 4000.0;
    let mk = |val: f64| vec![val; n];
    let s: Vec<f64> = (0..n).map(|i| i as f64 * len / (n as f64 - 1.0)).collect();
    let samples = LineSamples {
        s,
        kappa_h: mk(0.0),
        grade: mk(0.0),
        banking: mk(0.0),
        kappa_v: mk(0.0),
        n_ref: mk(0.0),
        kappa_ref: mk(kappa_ref),
        v_ref: mk(target_v),
        x_ref: (0..n).map(|i| i as f64 * len / (n as f64 - 1.0)).collect(),
        y_ref: mk(0.0),
        z_ref: mk(0.0),
        lat_x: mk(0.0),
        lat_y: mk(1.0),
        lat_z: mk(0.0),
        closed: false,
    };
    let line = LineTable::new(&samples)?;
    let cfg = SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(blocks, line, &interner, cfg);
    println!(
        "target_v={target_v} kappa_ref={kappa_ref} steer_ff={:.4}",
        t1.wheelbase_m * kappa_ref
    );
    for step in 0..2000 {
        solver.step();
        if step % 100 == 99 {
            let fs = solver.fast_state();
            println!(
                "t={:.3} vx={:.3} vy={:.4} r={:.5} n={:.4}",
                (step + 1) as f64 * 0.001,
                fs[3],
                fs[4],
                fs[5],
                fs[1]
            );
        }
    }
    Ok(())
}
