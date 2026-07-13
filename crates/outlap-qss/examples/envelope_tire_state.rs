// SPDX-License-Identifier: AGPL-3.0-only
//! Emit g-g-g-v **tyre-state** traces from the real
//! [`GgvEnvelope`](outlap_qss::GgvEnvelope) generator as CSV on stdout — the M5 amendment to
//! Decision #31 (T_tire / wear grip axes, D-M5-2). `python/tools/plot_envelope_tire_state.py` runs
//! this and plots it, so the theory figure is driven by the actual boundary re-solve, not a
//! re-implementation.
//!
//! One CSV `section,a,b,c,d`; the sections:
//! - `win`: the two grip couplings the axes carry — thermal window `λ_μ(T_s)` vs surface temp (°C)
//!   (`a`=T_c, `b`=λ_μ) and the wear factor vs tread wear mm (`c`=w, `d`=factor), overlaid.
//! - `temp`: peak lateral grip a_y vs T_tire (°C) at fixed v, zero wear (`a`=T_c, `b`=a_y), plus the
//!   frozen (tyre-state-blind) reference as `c`.
//! - `wear`: peak lateral grip a_y vs tread wear (mm) at T_opt, fixed v (`a`=w, `b`=a_y).
//! - `gg`: g-g sections a_y(a_x) at three tyre states — cold, optimum, worn — showing the whole
//!   envelope breathe (`a`=a_x, `b`=a_y, `c`=state id 0/1/2).
//! - `heat`: the 2-D grip surface a_y(T_tire, wear) at fixed v, pure lateral (`a`=T_c, `b`=w, `c`=a_y).
//! - `ident`: the reference slice (T_opt, 0) vs the frozen envelope, a g-g section — they must overlie
//!   (`a`=a_x, `b`=a_y frozen, `c`=a_y at reference tyre state).
#![allow(clippy::doc_markdown)]

use outlap_qss::{GgvEnvelope, T1Vehicle, TireStateRes};
use outlap_schema::io::MemLoader;
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling};
use outlap_schema::{load_vehicle, Conditions, LoadOptions};

const G: f64 = 9.806_65;
const CELSIUS_K: f64 = 273.15;
const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

/// A rear-driven downforce car (rev-limited to a realistic top speed), assembled from an in-memory
/// fixture so the example is self-contained — the same car `ggv_traces` uses.
fn build_car() -> T1Vehicle {
    let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
        axes: {speed_rpm: [0.0, 12000.0], load_axis: {torque_nm: [0.0, 800.0]}, torque_nm: [0.0, 800.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 12000.0], torque_nm: [800.0, 800.0]}}\n\
        inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
    let veh = "schema: vehicle/1.0\nname: tyre_state_demo\n\
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
    // The figure slices at a single reference speed on flat ground, so v/g_normal stay coarse; â_x
    // and the tyre-state axes carry the resolution the plot needs (smooth g-g rings + grip surface).
    let res = EnvelopeRes {
        v_points: 8,
        ax_points: 13,
        g_normal_points: 3,
    };
    let tire_res = TireStateRes {
        t_points: 9,
        w_points: 7,
    };
    let env = GgvEnvelope::generate_with_tire_state(&car, &res, FzCoupling::OneStepLag, tire_res)
        .unwrap();

    let ring = car.tire_thermal();
    let t_opt_k = ring.t_opt_k();
    let [(t_lo_k, t_hi_k), (_w_lo, w_hi)] = env.tire_state_domain().unwrap();
    let [(v_lo, v_hi), _, _] = env.domain();
    let v_ref = v_lo + 0.6 * (v_hi - v_lo); // a representative mid-high speed for the slices

    // Echo the parameters the plot annotates (Farroni window + Archard cliff), as `#`-comment lines.
    println!(
        "# tyre-state envelope traces (real GgvEnvelope::generate_with_tire_state); \
         t_opt_c={:.1} w_c={:.2} w_hi={:.2} v_ref={:.1}",
        t_opt_k - CELSIUS_K,
        ring.w_c_mm(),
        w_hi,
        v_ref,
    );
    for note in env.notes() {
        if note.starts_with("tyre-state axes") {
            println!("# {note}");
        }
    }
    println!("section,a,b,c,d");

    // (win) the two grip couplings the axes carry: thermal window and wear cliff (both normalised).
    for i in 0..=80 {
        let t_c = (t_lo_k - CELSIUS_K) + (t_hi_k - t_lo_k) * f64::from(i) / 80.0;
        let lam = ring.grip_window(t_c + CELSIUS_K) / ring.grip_window(t_opt_k);
        let w = w_hi * f64::from(i) / 80.0;
        let wf = ring.wear_grip_scale(w) / ring.wear_grip_scale(0.0);
        println!("win,{t_c:.3},{lam:.5},{w:.4},{wf:.5}");
    }

    // (temp) peak lateral grip vs T_tire at zero wear, plus the frozen (blind) reference line.
    let frozen_ref = env.ay_boundary(v_ref, 0.0, G);
    for i in 0..=60 {
        let t_c = (t_lo_k - CELSIUS_K) + (t_hi_k - t_lo_k) * f64::from(i) / 60.0;
        let ay = env.ay_boundary_at(v_ref, 0.0, G, t_c + CELSIUS_K, 0.0);
        println!("temp,{t_c:.3},{ay:.4},{frozen_ref:.4},0");
    }

    // (wear) peak lateral grip vs tread wear at the optimum temperature.
    for i in 0..=60 {
        let w = w_hi * f64::from(i) / 60.0;
        let ay = env.ay_boundary_at(v_ref, 0.0, G, t_opt_k, w);
        println!("wear,{w:.4},{ay:.4},0,0");
    }

    // (gg) g-g sections at three tyre states: cold (t_lo, new), optimum (t_opt, new), worn (t_opt,
    // w_hi). The braking/acceleration extent uses each state's own straight-line capability.
    let states = [(t_lo_k, 0.0, 0u8), (t_opt_k, 0.0, 1), (t_opt_k, w_hi, 2)];
    for &(t_k, w, id) in &states {
        let brake = env.brake_limit_at(v_ref, G, t_k, w);
        let accel = env.accel_limit_at(v_ref, G, t_k, w);
        for i in 0..=80 {
            let ax = -brake + (accel + brake) * f64::from(i) / 80.0;
            println!(
                "gg,{ax:.4},{:.4},{id},0",
                env.ay_boundary_at(v_ref, ax, G, t_k, w)
            );
        }
    }

    // (heat) the 2-D grip surface a_y(T_tire, wear) at pure lateral — the tyre-state map.
    for it in 0..=24 {
        let t_c = (t_lo_k - CELSIUS_K) + (t_hi_k - t_lo_k) * f64::from(it) / 24.0;
        for iw in 0..=24 {
            let w = w_hi * f64::from(iw) / 24.0;
            let ay = env.ay_boundary_at(v_ref, 0.0, G, t_c + CELSIUS_K, w);
            println!("heat,{t_c:.3},{w:.4},{ay:.4},0");
        }
    }

    // (ident) the reference slice (T_opt, 0) laid over the frozen envelope — the bit-identity invariant.
    let brake = env.brake_limit(v_ref, G);
    let accel = env.accel_limit(v_ref, G);
    for i in 0..=80 {
        let ax = -brake + (accel + brake) * f64::from(i) / 80.0;
        let frozen = env.ay_boundary(v_ref, ax, G);
        let at_ref = env.ay_boundary_at(v_ref, ax, G, t_opt_k, 0.0);
        println!("ident,{ax:.4},{frozen:.4},{at_ref:.4},0");
    }
}
