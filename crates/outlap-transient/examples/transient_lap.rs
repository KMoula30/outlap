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
//! **T2 skeleton demo scenarios** (PR3/PR4) on the real `limebeer_2014_f1` car. Emits CSV traces the
//! PR figures are drawn from — deliberately *clean, analytically checkable* maneuvers, since the
//! driver/powertrain are the placeholder blocks (real `MacAdam` driver = PR5, shift FSM + torque
//! vectoring = PR6) and the per-vehicle Python assembly + real-circuit racing lap are PR7/PR12:
//!
//! * **skidpad** — a closed-loop lap of a constant-radius circle: the placeholder driver tracks the
//!   reference line, so `n` stays bounded and the cornering reaches a clean steady state;
//! * **coastdown** — straight, no drive/brake: `v_x(t)` decays under aero drag (checked vs analytic);
//! * **step-steer** — straight road, constant steer: the yaw rate builds to `≈ v·δ/L` with the right
//!   sign, and the outer wheels gain load (lateral transfer).
//!
//! ```text
//! cargo run --release -p outlap-transient --example transient_lap [-- --out <dir>]
//! ```

use std::error::Error;
use std::path::PathBuf;

use outlap_core::bus::{ChannelInterner, WHEELS};
use outlap_qss::t1::LoadTransferGeometry;
use outlap_qss::T1Vehicle;
use outlap_schema::io::FsLoader;
use outlap_schema::sim::FzCoupling;
use outlap_schema::{load_conditions, load_vehicle, LoadOptions};
use outlap_transient::{
    LineSamples, LineTable, SimConfig, T2Blocks, TransientLap, TransientSolver,
};
use outlap_vehicle::{
    Aero, Chassis, ChassisParams, Driver, LoadTransfer, Powertrain, RelaxProvider, RoadChannels,
    Tire,
};

const WHEEL_INERTIA_KGM2: f64 = 1.1;
const FALLBACK_RADIUS_M: f64 = 0.33;

fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

/// Driver gains `(k_offset, k_heading, k_yaw_rate, k_speed)`.
type Gains = (f64, f64, f64, f64);

/// Build the T2 block set from an assembled T1 vehicle (the PR7 Python path mirrors this).
fn build_blocks(t1: &T1Vehicle, interner: &mut ChannelInterner, gains: Gains) -> T2Blocks<f64> {
    let road = RoadChannels::intern(interner);
    let (a, b) = (t1.a_f, t1.b_r);
    let (tf, tr) = (t1.t_f, t1.t_r);
    let x = [a, a, -b, -b];
    let y = [tf * 0.5, -tf * 0.5, tr * 0.5, -tr * 0.5];
    let front = [true, true, false, false];
    let r_f = t1.tire_front.unloaded_radius(FALLBACK_RADIUS_M);
    let r_r = t1.tire_rear.unloaded_radius(FALLBACK_RADIUS_M);
    let radius = [r_f, r_f, r_r, r_r];
    let params = ChassisParams::from_f64(
        t1.mass_kg,
        t1.izz,
        x,
        y,
        front,
        radius,
        [WHEEL_INERTIA_KGM2; WHEELS],
    );
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
    let (k_offset, k_heading, k_yaw_rate, k_speed) = gains;
    T2Blocks {
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
        driver: Driver {
            wheelbase: t1.wheelbase_m,
            k_offset,
            k_heading,
            k_yaw_rate,
            k_speed,
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
    }
}

/// A synthetic line: a circle of `circle_radius` (closed) or a straight (open) along +x. `steer_kappa`
/// is the constant `kappa_ref` steer feed-forward; `road_kappa` is the actual road curvature the
/// chassis frame follows (equal to `1/R` for the circle, 0 for the straight).
#[allow(clippy::too_many_arguments)]
fn synthetic_line(
    length: f64,
    stations: usize,
    closed: bool,
    road_kappa: f64,
    steer_kappa: f64,
    v_ref: f64,
    circle_radius: Option<f64>,
) -> LineTable<f64> {
    let s: Vec<f64> = (0..stations)
        .map(|i| i as f64 * length / (stations as f64 - 1.0))
        .collect();
    let mk = |val: f64| vec![val; stations];
    let (mut xr, mut yr, mut lx, mut ly) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for &si in &s {
        if let Some(r) = circle_radius {
            let th = si / r;
            xr.push(r * th.sin());
            yr.push(r * (1.0 - th.cos()));
            lx.push(-th.sin());
            ly.push(th.cos());
        } else {
            xr.push(si);
            yr.push(0.0);
            lx.push(0.0);
            ly.push(1.0);
        }
    }
    let samples = LineSamples {
        s,
        kappa_h: mk(road_kappa),
        grade: mk(0.0),
        banking: mk(0.0),
        kappa_v: mk(0.0),
        n_ref: mk(0.0),
        kappa_ref: mk(steer_kappa),
        v_ref: mk(v_ref),
        x_ref: xr,
        y_ref: yr,
        z_ref: mk(0.0),
        lat_x: lx,
        lat_y: ly,
        lat_z: mk(0.0),
        closed,
    };
    LineTable::new(&samples).expect("valid synthetic line")
}

fn write_csv(lap: &TransientLap<f64>, header: &str, row: impl Fn(usize) -> String, path: &PathBuf) {
    let mut out = String::from(header);
    out.push('\n');
    for i in 0..lap.len() {
        out.push_str(&row(i));
        out.push('\n');
    }
    std::fs::write(path, out).expect("write csv");
    println!("wrote {}", path.display());
}

fn main() -> Result<(), Box<dyn Error>> {
    let out_dir = std::env::args()
        .skip_while(|a| a != "--out")
        .nth(1)
        .map_or_else(|| data("../debug_plots/t2"), PathBuf::from);
    std::fs::create_dir_all(&out_dir)?;

    let vl = FsLoader::new(data("vehicles/limebeer_2014_f1"));
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default())?;
    let conditions = load_conditions("conditions.yaml", &vl)?;
    let t1 = T1Vehicle::assemble(&resolved, &conditions, &vl, false)?;
    let cfg = SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    };

    // Scenario 1 — closed-loop skidpad (R = 60 m, v = 30 m/s ≈ 1.5 g).
    {
        let radius = 60.0;
        let v = 30.0;
        let length = 2.0 * std::f64::consts::PI * radius;
        let mut it = ChannelInterner::new();
        let blocks = build_blocks(&t1, &mut it, (0.08, 0.7, 0.2, 0.4));
        let line = synthetic_line(
            length,
            400,
            true,
            1.0 / radius,
            1.0 / radius,
            v,
            Some(radius),
        );
        let mut solver = TransientSolver::new(blocks, line, &it, cfg);
        let lap = solver.run(length, 60_000);
        println!(
            "skidpad: {} steps, {:.2} s, |n|max={:.3} m",
            lap.len(),
            lap.lap_time_s,
            lap.n.iter().fold(0.0_f64, |m, &n| m.max(n.abs()))
        );
        write_csv(
            &lap,
            "t,s,n,psi_rel,vx,vy,r,ay,steer,x,y,fz_fl,fz_fr,fz_rl,fz_rr",
            |i| {
                format!(
                    "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                    lap.t[i],
                    lap.s[i],
                    lap.n[i],
                    lap.psi_rel[i],
                    lap.vx[i],
                    lap.vy[i],
                    lap.yaw_rate[i],
                    lap.ay[i],
                    lap.steer[i],
                    lap.x[i],
                    lap.y[i],
                    lap.fz[i][0],
                    lap.fz[i][1],
                    lap.fz[i][2],
                    lap.fz[i][3]
                )
            },
            &out_dir.join("skidpad.csv"),
        );
    }

    // Scenario 2 — coastdown from 80 m/s (all gains 0 → no steer, no throttle; drag only).
    {
        let v0 = 80.0;
        let mut it = ChannelInterner::new();
        let blocks = build_blocks(&t1, &mut it, (0.0, 0.0, 0.0, 0.0));
        let line = synthetic_line(20_000.0, 200, false, 0.0, 0.0, v0, None);
        let mut solver = TransientSolver::new(blocks, line, &it, cfg);
        let lap = solver.run(19_000.0, 40_000);
        println!(
            "coastdown: {} steps from {:.0} m/s to {:.1} m/s",
            lap.len(),
            v0,
            lap.vx[lap.len() - 1]
        );
        write_csv(
            &lap,
            "t,vx,ax",
            |i| format!("{},{},{}", lap.t[i], lap.vx[i], lap.ax[i]),
            &out_dir.join("coastdown.csv"),
        );
    }

    // Scenario 3 — step-steer (straight road, constant steer feed-forward, hold 40 m/s).
    {
        let v = 40.0;
        let kappa = 0.006; // steer_ff = L·kappa ≈ 0.02 rad
        let mut it = ChannelInterner::new();
        let blocks = build_blocks(&t1, &mut it, (0.0, 0.0, 0.0, 0.4));
        let line = synthetic_line(8000.0, 200, false, 0.0, kappa, v, None);
        let mut solver = TransientSolver::new(blocks, line, &it, cfg);
        let lap = solver.run(7900.0, 1500);
        println!(
            "step-steer: {} steps, steady r={:.4} rad/s (v·δ/L≈{:.4})",
            lap.len(),
            lap.yaw_rate[lap.len() - 1],
            v * t1.wheelbase_m * kappa / t1.wheelbase_m
        );
        write_csv(
            &lap,
            "t,vx,vy,r,ay,steer,slip_alpha_fl,fz_fl,fz_fr,fz_rl,fz_rr",
            |i| {
                format!(
                    "{},{},{},{},{},{},{},{},{},{},{}",
                    lap.t[i],
                    lap.vx[i],
                    lap.vy[i],
                    lap.yaw_rate[i],
                    lap.ay[i],
                    lap.steer[i],
                    lap.slip_alpha[i][0],
                    lap.fz[i][0],
                    lap.fz[i][1],
                    lap.fz[i][2],
                    lap.fz[i][3]
                )
            },
            &out_dir.join("step_steer.csv"),
        );
    }

    Ok(())
}
