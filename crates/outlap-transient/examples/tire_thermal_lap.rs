// SPDX-License-Identifier: AGPL-3.0-only
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::many_single_char_names,
    clippy::similar_names
)]
//! **T2 tire-thermal lap demo** (M5 PR3): the reduced Farroni-TRT ring + Archard wear, wired into the
//! transient solver on the decimated slow clock, driven over closed skidpad laps of the real
//! `limebeer_2014_f1` car. Emits the CSV the PR figure (`python/tools/plot_tire_thermal_lap.py`) is
//! drawn from — so the figure comes from the actual `TransientSolver`, not a re-implementation.
//!
//! Scenarios (one CSV, `scenario` column):
//! - `warmup`: cold-seeded tyres warming into the grip window over a skidpad — per-wheel surface
//!   temperature (outer vs inner) and the total grip multiplier the force call uses.
//! - `stint`: warm-seeded, high-wear long run — tread wear crossing the cliff and grip falling.
//! - `window`: the static grip window `λ_μ(T_s)` (the phase-plot background the warm-up climbs).
//!
//! ```text
//! cargo run --release -p outlap-transient --example tire_thermal_lap
//! ```

#[path = "../tests/common/mod.rs"]
mod common;

use std::f64::consts::PI;

use common::{build_blocks, limebeer, line};
use outlap_core::bus::ChannelInterner;
use outlap_schema::sim::FzCoupling;
use outlap_schema::tyr::{TyrThermal, TyrWear};
use outlap_tire::{TireThermalRing, TireThermalState};
use outlap_transient::{AxleGeometry, SimConfig, TireThermalStack, TransientSolver};

const CELSIUS_K: f64 = 273.15;
/// Outer / inner front wheel on a left-turn skidpad (`[FL, FR, RL, RR]`): the right wheels are
/// loaded, so `FR` runs the hotter, harder-worked contact patch.
const OUTER: usize = 1;
const INNER: usize = 0;

/// A synthetic F1-slick thermal block with light cooling, so a skidpad warms the tyres into the
/// window inside the demo — uncalibrated (calibration is PR7/PR8); the point is the *response*.
fn thermal() -> TyrThermal {
    TyrThermal {
        c_s: 800.0,
        c_c: 2000.0,
        c_g: 600.0,
        g_sc: 180.0,
        g_cg: 70.0,
        g_road: 80.0,
        h0: 3.0,
        h1: 0.9,
        p_t: 0.65,
        t_opt: 95.0,
        c_t: 2.2,
        k_c: 0.0015,
        t_c_ref: 80.0,
        p_cold: 138.0,
        t_cold: 20.0,
    }
}

/// A synthetic wear block whose `k_w` (the ring's `mm/(W·s/m²)` Archard scale is ~1e-7) reaches the
/// cliff (`w_c = 2 mm`) over a minutes-long stint — gradual, so warm-up is *thermal* not wear-driven.
fn wear() -> TyrWear {
    TyrWear {
        k_w: 3.0e-7,
        w_max: 8.0,
        w_c: 2.0,
        tau_d: 500.0,
        t_deg: 112.0,
        delta_t_ref: 15.0,
        beta: 2.0,
        delta_c: 0.35,
        s_w: 0.35,
        delta_d: 0.25,
    }
}

fn geom() -> AxleGeometry {
    AxleGeometry::new(0.33, Some(0.30), Some(250_000.0))
}

fn stack(seed_c: f64) -> TireThermalStack<f64> {
    let (th, wr, g) = (thermal(), wear(), geom());
    let mut s = TireThermalStack::new(&th, &wr, &th, &wr, g, g, 25.0, 35.0);
    s.seed_uniform(seed_c);
    s
}

fn skidpad(seed_c: f64, r: f64, v: f64) -> TransientSolver<f64> {
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let ln = line(2.0 * PI * r, 400, true, 1.0 / r, 1.0 / r, v, Some(r));
    let cfg = SimConfig {
        fz_coupling: FzCoupling::FixedPoint,
        ..SimConfig::default()
    };
    TransientSolver::new(blocks, ln, &it, cfg).with_tire_thermal(stack(seed_c))
}

fn main() {
    let th = thermal();
    println!(
        "# t_opt_c={} window_lo={} window_hi={} w_c={}",
        th.t_opt,
        th.t_opt - 12.0,
        th.t_opt + 12.0,
        wear().w_c
    );
    println!("scenario,x,a,b,c,d");

    // --- warmup: cold tyres (60 °C) heating toward the window over five skidpad laps. --------------
    let r = 100.0;
    let mut solver = skidpad(60.0, r, 30.0);
    let lap = solver.run(5.0 * 2.0 * PI * r, 220_000);
    let n = lap.len();
    for k in (0..n).step_by(120) {
        println!(
            "warmup,{:.2},{:.3},{:.3},{:.4},{:.4}",
            lap.t[k].max(0.0),
            lap.tire_surface_c[k][OUTER],
            lap.tire_surface_c[k][INNER],
            lap.tire_grip[k][OUTER],
            lap.tire_grip[k][INNER],
        );
    }

    // --- stint: warm tyres (95 °C) on a long high-wear run — wear crosses the cliff, grip falls. ---
    let mut solver = skidpad(95.0, r, 28.0);
    let lap = solver.run(1.0e9, 200_000);
    let n = lap.len();
    for k in (0..n).step_by(200) {
        println!(
            "stint,{:.2},{:.4},{:.4},{:.4},",
            lap.t[k].max(0.0),
            lap.tire_wear_mm[k][OUTER],
            lap.tire_damage[k][OUTER],
            lap.tire_grip[k][OUTER],
        );
    }

    // --- window: the static grip window λ_μ(T_s) (the phase-plot background). ----------------------
    let ring = TireThermalRing::<f64>::from_schema_with_wear(&th, &wear());
    let mut temp_c = 40.0;
    while temp_c <= 150.0 {
        let node = TireThermalState::uniform(CELSIUS_K + temp_c);
        println!("window,{:.2},{:.5},,,", temp_c, ring.mu_scale(&node));
        temp_c += 2.0;
    }
}
