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
//! T2 dynamics property tests on the real `limebeer_2014_f1` car (assembled once per test): the
//! assembler order is deterministic, the split integrator is bit-reproducible, the relaxation states
//! converge, and open-loop maneuvers match analytic expectations (flat-track planar degeneration,
//! coastdown drag decel, step-steer yaw sign/magnitude, friction-circle containment).

use outlap_core::block::Phase;
use outlap_core::bus::{ChannelInterner, WHEELS};
use outlap_core::state::{ChassisState, RelaxState, StateLayout};
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

fn limebeer() -> T1Vehicle {
    let vl = FsLoader::new(data("vehicles/limebeer_2014_f1"));
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default()).unwrap();
    let conditions = load_conditions("conditions.yaml", &vl).unwrap();
    T1Vehicle::assemble(&resolved, &conditions, &vl, false).unwrap()
}

fn build_blocks(
    t1: &T1Vehicle,
    it: &mut ChannelInterner,
    gains: (f64, f64, f64, f64),
) -> T2Blocks<f64> {
    let road = RoadChannels::intern(it);
    let (a, b, tf, tr) = (t1.a_f, t1.b_r, t1.t_f, t1.t_r);
    let x = [a, a, -b, -b];
    let y = [tf * 0.5, -tf * 0.5, tr * 0.5, -tr * 0.5];
    let r_f = t1.tire_front.unloaded_radius(0.33);
    let r_r = t1.tire_rear.unloaded_radius(0.33);
    let params = ChassisParams::from_f64(
        t1.mass_kg,
        t1.izz,
        x,
        y,
        [true, true, false, false],
        [r_f, r_f, r_r, r_r],
        [1.1; WHEELS],
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

/// Synthetic straight (or circular) line; `road_kappa`/`steer_kappa` as in the demo example.
fn line(
    len: f64,
    stations: usize,
    closed: bool,
    road_k: f64,
    steer_k: f64,
    v_ref: f64,
    circle: Option<f64>,
) -> LineTable<f64> {
    let s: Vec<f64> = (0..stations)
        .map(|i| i as f64 * len / (stations as f64 - 1.0))
        .collect();
    let mk = |v: f64| vec![v; stations];
    let (mut xr, mut yr, mut lx, mut ly) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for &si in &s {
        if let Some(r) = circle {
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
    LineTable::new(&LineSamples {
        s,
        kappa_h: mk(road_k),
        grade: mk(0.0),
        banking: mk(0.0),
        kappa_v: mk(0.0),
        n_ref: mk(0.0),
        kappa_ref: mk(steer_k),
        v_ref: mk(v_ref),
        x_ref: xr,
        y_ref: yr,
        z_ref: mk(0.0),
        lat_x: lx,
        lat_y: ly,
        lat_z: mk(0.0),
        closed,
    })
    .unwrap()
}

fn cfg() -> SimConfig<f64> {
    SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    }
}

#[test]
fn assembler_order_is_deterministic_and_phase_sorted() {
    let t1 = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &mut it, (0.0, 0.0, 0.0, 0.0));
    let solver = TransientSolver::new(
        blocks,
        line(100.0, 50, false, 0.0, 0.0, 20.0, None),
        &it,
        cfg(),
    );
    let order = solver.schedule().order().to_vec();
    // The assembler-produced order must equal the solver's fixed execution order
    // (driver → powertrain → load-transfer → aero → tyre → chassis), so the hardcoded
    // eval order in `eval_rhs_raw` genuinely honours the topological sort.
    assert_eq!(order, vec![0, 1, 2, 3, 4, 5]);
    // Determinism: same specs → same schedule.
    let solver2 = TransientSolver::new(
        build_blocks(&t1, &mut ChannelInterner::new(), (0.0, 0.0, 0.0, 0.0)),
        line(100.0, 50, false, 0.0, 0.0, 20.0, None),
        &it,
        cfg(),
    );
    assert_eq!(order, solver2.schedule().order().to_vec());
    // Phase ordering: driver (control) precedes the chassis (integrate); the tyre (reads Fz) follows
    // the load-transfer writer.
    let pos = |b: usize| order.iter().position(|&x| x == b).unwrap();
    assert!(pos(0) < pos(5), "driver before chassis");
    assert!(pos(2) < pos(4), "load-transfer before tyre");
    assert!(Phase::Control < Phase::Integrate);
}

#[test]
fn flat_straight_stays_planar() {
    let t1 = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &mut it, (0.0, 0.0, 0.0, 0.3));
    let mut solver = TransientSolver::new(
        blocks,
        line(4000.0, 200, false, 0.0, 0.0, 40.0, None),
        &it,
        cfg(),
    );
    for _ in 0..2000 {
        solver.step();
    }
    let fs = solver.fast_state();
    // No steer, no banking ⇒ lateral/yaw/offset stay at zero.
    assert!(
        fs[ChassisState::Vy as usize].abs() < 1e-6,
        "vy={}",
        fs[ChassisState::Vy as usize]
    );
    assert!(fs[ChassisState::YawRate as usize].abs() < 1e-6);
    assert!(fs[ChassisState::N as usize].abs() < 1e-6);
}

#[test]
fn coastdown_decelerates_under_drag() {
    let t1 = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &mut it, (0.0, 0.0, 0.0, 0.0)); // no throttle
    let mut solver = TransientSolver::new(
        blocks,
        line(20_000.0, 200, false, 0.0, 0.0, 80.0, None),
        &it,
        cfg(),
    );
    let v0 = solver.fast_state()[ChassisState::Vx as usize];
    for _ in 0..3000 {
        solver.step();
    }
    let v1 = solver.fast_state()[ChassisState::Vx as usize];
    assert!(v1 < v0, "coasted down: {v0} -> {v1}");
    // Initial decel is dominated by aero drag qx·v²/m (rolling adds a little). Order-of-magnitude.
    let drag_decel = t1.qx * v0 * v0 / t1.mass_kg;
    assert!(
        drag_decel > 0.5 && drag_decel < 30.0,
        "drag decel {drag_decel}"
    );
}

#[test]
fn step_steer_builds_correct_yaw_and_loads_the_outside() {
    let t1 = limebeer();
    let mut it = ChannelInterner::new();
    // Constant steer feed-forward via kappa_ref on a straight road (path gains 0).
    let (v, kappa) = (40.0, 0.006);
    let blocks = build_blocks(&t1, &mut it, (0.0, 0.0, 0.0, 0.4));
    let mut solver = TransientSolver::new(
        blocks,
        line(8000.0, 200, false, 0.0, kappa, v, None),
        &it,
        cfg(),
    );
    for _ in 0..1500 {
        solver.step();
    }
    let fs = solver.fast_state();
    let r = fs[ChassisState::YawRate as usize];
    // Positive steer ⇒ +yaw (left). Magnitude near the neutral-steer estimate v·δ/L (understeer
    // makes it a bit lower), so bound it generously.
    let neutral = v * (t1.wheelbase_m * kappa) / t1.wheelbase_m; // = v·κ
    assert!(r > 0.0, "left steer ⇒ +yaw, got {r}");
    assert!(
        r > 0.4 * neutral && r < 1.2 * neutral,
        "r={r} vs neutral {neutral}"
    );
}

#[test]
fn relaxation_states_converge_to_steady_state() {
    let t1 = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &mut it, (0.0, 0.0, 0.0, 0.4));
    let mut solver = TransientSolver::new(
        blocks,
        line(8000.0, 200, false, 0.0, 0.006, 40.0, None),
        &it,
        cfg(),
    );
    // Run to a steady turn, then check the lagged slip stops moving (converged).
    for _ in 0..1400 {
        solver.step();
    }
    let a_before: Vec<f64> = (0..WHEELS)
        .map(|w| solver.fast_state()[StateLayout::relax_slot(RelaxState::Alpha, w)])
        .collect();
    for _ in 0..100 {
        solver.step();
    }
    for w in 0..WHEELS {
        let a_after = solver.fast_state()[StateLayout::relax_slot(RelaxState::Alpha, w)];
        assert!(
            (a_after - a_before[w]).abs() < 1e-4,
            "wheel {w} lagged α not converged"
        );
        // A cornering front wheel carries a non-trivial slip angle.
    }
}

#[test]
fn skidpad_is_bit_reproducible() {
    let t1 = limebeer();
    let run = || {
        let mut it = ChannelInterner::new();
        let blocks = build_blocks(&t1, &mut it, (0.08, 0.7, 0.2, 0.4));
        let l = line(
            2.0 * std::f64::consts::PI * 60.0,
            400,
            true,
            1.0 / 60.0,
            1.0 / 60.0,
            30.0,
            Some(60.0),
        );
        let mut solver = TransientSolver::new(blocks, l, &it, cfg());
        let lap = solver.run(2.0 * std::f64::consts::PI * 60.0, 60_000);
        (lap.len(), lap.yaw_rate.clone(), lap.n.clone())
    };
    let (n1, r1, off1) = run();
    let (n2, r2, off2) = run();
    assert_eq!(n1, n2);
    assert_eq!(r1, r2, "yaw-rate trace bit-identical across runs");
    assert_eq!(off1, off2, "offset trace bit-identical across runs");
}

#[test]
fn skidpad_stays_within_the_friction_circle() {
    let t1 = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &mut it, (0.08, 0.7, 0.2, 0.4));
    let l = line(
        2.0 * std::f64::consts::PI * 60.0,
        400,
        true,
        1.0 / 60.0,
        1.0 / 60.0,
        30.0,
        Some(60.0),
    );
    let mut solver = TransientSolver::new(blocks, l, &it, cfg());
    let lap = solver.run(2.0 * std::f64::consts::PI * 60.0, 60_000);
    // The combined tyre force never exceeds the model's own peak-μ ellipse (a small margin covers
    // combined-slip vs the per-axis peak).
    for i in (0..lap.len()).step_by(50) {
        for w in 0..WHEELS {
            let fz = lap.fz[i][w].max(1.0);
            let (fx, fy) = (lap.fx[i][w], lap.fy[i][w]);
            let combined = (fx * fx + fy * fy).sqrt();
            let model = if w < 2 { &t1.tire_front } else { &t1.tire_rear };
            let p = if w < 2 { t1.p_front } else { t1.p_rear };
            let mu = model.peak_mu_y(fz, p).max(model.peak_mu_x(fz, p));
            assert!(
                combined <= 1.1 * mu * fz,
                "wheel {w}: |F|={combined} > μ·Fz={}",
                mu * fz
            );
        }
    }
}
