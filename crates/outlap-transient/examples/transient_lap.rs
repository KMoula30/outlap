// SPDX-License-Identifier: AGPL-3.0-only
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::needless_range_loop,
    clippy::too_many_lines
)]
//! **T2 closed-loop demo scenarios** on the real `limebeer_2014_f1` car, driven by the ideal
//! MacAdam-preview + PI driver (PR5). Emits CSV traces the PR figures are drawn from:
//!
//! * **skidpad** — a closed constant-radius lap: the driver tracks the reference line (`n` stays
//!   small) and settles into a clean steady corner;
//! * **coastdown** — straight, longitudinal loop silenced: `v_x(t)` decays under aero drag;
//! * **step-steer** — straight road, constant curvature feed-forward (path feedback off): the yaw
//!   rate builds to `≈ v·κ` with the right sign and the outside wheels gain load;
//! * **speed-tracking** — straight road, a triangular `v_ref` profile the PI + preview follows.
//!
//! ```text
//! cargo run --release -p outlap-transient --example transient_lap [-- --out <dir>]
//! ```

#[path = "../tests/common/mod.rs"]
mod common;

use std::error::Error;
use std::path::PathBuf;

use outlap_core::bus::ChannelInterner;
use outlap_schema::sim::FzCoupling;
use outlap_transient::{LineSamples, LineTable, SimConfig, TransientLap, TransientSolver};

fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
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

    let (t1, spec) = common::limebeer();
    let cfg = SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    };

    // Scenario 1 — closed-loop skidpad (R = 60 m, v = 30 m/s ≈ 1.5 g), full ideal driver.
    {
        let radius = 60.0;
        let v = 30.0;
        let length = 2.0 * std::f64::consts::PI * radius;
        let mut it = ChannelInterner::new();
        let blocks = common::build_blocks(&t1, &spec, &mut it);
        let line = common::line(
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

    // Scenario 2 — coastdown from 80 m/s (longitudinal loop silenced; drag only).
    {
        let v0 = 80.0;
        let mut it = ChannelInterner::new();
        let mut blocks = common::build_blocks(&t1, &spec, &mut it);
        blocks.driver.speed_kp = 0.0;
        blocks.driver.speed_ki = 0.0;
        blocks.driver.ff_accel_scale = f64::INFINITY; // no throttle / brake
        let line = common::line(20_000.0, 200, false, 0.0, 0.0, v0, None);
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

    // Scenario 3 — step-steer (straight road, constant curvature FF, path feedback off, hold 40).
    {
        let v = 40.0;
        let kappa = 0.006;
        let mut it = ChannelInterner::new();
        let mut blocks = common::build_blocks(&t1, &spec, &mut it);
        blocks.driver.preview_gain = 0.0;
        blocks.driver.heading_gain = 0.0;
        blocks.driver.yaw_damping = 0.0;
        let line = common::line(8000.0, 200, false, 0.0, kappa, v, None);
        let mut solver = TransientSolver::new(blocks, line, &it, cfg);
        let lap = solver.run(7900.0, 1500);
        println!(
            "step-steer: {} steps, steady r={:.4} rad/s (v·κ≈{:.4})",
            lap.len(),
            lap.yaw_rate[lap.len() - 1],
            v * kappa
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

    // Scenario 4 — speed tracking: straight road, a triangular v_ref profile 35 → 60 → 35 the PI +
    // preview feed-forward follows (the QSS-profile-tracking mechanism, on a synthetic profile).
    {
        let len = 3000.0;
        let stations = 300;
        let s: Vec<f64> = (0..stations)
            .map(|i| i as f64 * len / (stations as f64 - 1.0))
            .collect();
        let vref: Vec<f64> = s
            .iter()
            .map(|&si| {
                let f = si / len;
                if f < 0.5 {
                    35.0 + 50.0 * f
                } else {
                    60.0 - 50.0 * (f - 0.5)
                }
            })
            .collect();
        let mk = |v: f64| vec![v; stations];
        let line = LineTable::new(&LineSamples {
            s: s.clone(),
            kappa_h: mk(0.0),
            grade: mk(0.0),
            banking: mk(0.0),
            kappa_v: mk(0.0),
            n_ref: mk(0.0),
            kappa_ref: mk(0.0),
            v_ref: vref,
            x_ref: s.clone(),
            y_ref: mk(0.0),
            z_ref: mk(0.0),
            lat_x: mk(0.0),
            lat_y: mk(1.0),
            lat_z: mk(0.0),
            closed: false,
        })?;
        let mut it = ChannelInterner::new();
        let blocks = common::build_blocks(&t1, &spec, &mut it);
        let mut solver = TransientSolver::new(blocks, line, &it, cfg);
        let lap = solver.run(len - 50.0, 200_000);
        println!(
            "speed-track: {} steps, final v={:.1} m/s",
            lap.len(),
            lap.vx[lap.len() - 1]
        );
        write_csv(
            &lap,
            "t,s,vx,throttle,brake",
            |i| {
                format!(
                    "{},{},{},{},{}",
                    lap.t[i], lap.s[i], lap.vx[i], lap.throttle[i], lap.brake[i]
                )
            },
            &out_dir.join("speed_track.csv"),
        );
    }

    Ok(())
}
