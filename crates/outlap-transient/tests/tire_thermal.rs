// SPDX-License-Identifier: AGPL-3.0-only
//! T2 tire-thermal integration (M5 PR3): the ring + wear stack, wired on the decimated slow clock,
//! makes a transient lap respond to tyre temperature and wear.
//!
//! These are behaviour tests over the *wired* stack (the isolated ring physics is covered by the
//! `outlap-tire` property tests). They assert: cold tyres warm into the grip window and grip rises; a
//! long stint drives wear across the cliff and grip falls; the accumulate→window-average→step
//! pipeline closes energy exactly; and the wired lap is bit-deterministic.
#![allow(
    clippy::many_single_char_names,
    clippy::cast_precision_loss,
    clippy::float_cmp // determinism asserts bit-identical floats on purpose
)]

mod common;

use std::f64::consts::PI;

use common::{build_blocks, limebeer, line};
use outlap_core::bus::ChannelInterner;
use outlap_schema::sim::FzCoupling;
use outlap_schema::tyr::{TyrThermal, TyrWear};
use outlap_tire::{ThermalDrivers, TireThermalRing, TireThermalState};
use outlap_transient::{AxleGeometry, SimConfig, TireThermalStack, TransientSolver};

const CELSIUS_K: f64 = 273.15;

/// A synthetic F1-slick thermal block with light cooling + small capacities so warm-up is fast within
/// a test lap (uncalibrated — the point is the wiring's *response*, not a fitted number; calibration
/// is PR7/8).
fn thermal_block() -> TyrThermal {
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
        k_c: 0.001,
        t_c_ref: 80.0,
        p_cold: 138.0,
        t_cold: 20.0,
    }
}

/// A synthetic wear block. `k_w` is the ring's Archard scale (~1e-7): a *low* value keeps wear
/// negligible (isolating the thermal warm-up), a *high* one drives the cliff quickly for the stint.
fn wear_block(k_w: f64) -> TyrWear {
    TyrWear {
        k_w,
        w_max: 8.0,
        w_c: 2.0,
        tau_d: 500.0,
        t_deg: 110.0,
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

/// A stack over the synthetic blocks, seeded at `seed_c` (°C) uniform (cold-start warm-up), with the
/// given Archard `k_w`.
fn stack_seeded(seed_c: f64, k_w: f64) -> TireThermalStack<f64> {
    let th = thermal_block();
    let wr = wear_block(k_w);
    let g = geom();
    let mut s = TireThermalStack::new(&th, &wr, &th, &wr, g, g, 30.0, 40.0);
    s.seed_uniform(seed_c);
    s
}

/// Build a closed-skidpad T2 solver with the tyre-thermal stack attached, seeded at `seed_c` with
/// Archard `k_w`.
fn skidpad_solver(seed_c: f64, k_w: f64, r: f64, v: f64) -> TransientSolver<f64> {
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let ln = line(2.0 * PI * r, 400, true, 1.0 / r, 1.0 / r, v, Some(r));
    let cfg = SimConfig {
        fz_coupling: FzCoupling::FixedPoint,
        ..SimConfig::default()
    };
    TransientSolver::new(blocks, ln, &it, cfg).with_tire_thermal(stack_seeded(seed_c, k_w))
}

/// Mean of a per-wheel channel over a `[lo, hi)` slice of the recorded rows.
fn mean_slice(rows: &[[f64; 4]], lo: usize, hi: usize) -> f64 {
    let hi = hi.min(rows.len());
    let mut sum = 0.0;
    let mut n = 0i32;
    for row in &rows[lo..hi] {
        for &v in row {
            sum += v;
            n += 1;
        }
    }
    sum / f64::from(n.max(1))
}

#[test]
fn cold_tyres_warm_into_the_window_and_grip_rises() {
    // Seed cold (60 °C) below the optimum (T_opt = 95 °C) with negligible wear (low k_w), so the grip
    // change is purely thermal: the tyres heat under the skidpad load, the surface climbs toward the
    // window, and the total grip multiplier rises from cold toward warm — the first lap-level effect.
    let mut solver = skidpad_solver(60.0, 1.0e-7, 100.0, 30.0);
    let lap = solver.run(2.0 * 2.0 * PI * 100.0, 90_000);

    assert!(lap.len() > 1000, "the skidpad ran a real trace");
    assert!(
        !lap.tire_surface_c.is_empty(),
        "tire-thermal channels were recorded"
    );

    // Wear stays negligible over the warm-up (low k_w), so grip tracks temperature, not the cliff.
    let final_wear = mean_slice(&lap.tire_wear_mm, lap.len().saturating_sub(200), lap.len());
    assert!(
        final_wear < 0.5,
        "wear is negligible over the warm-up (isolating the thermal effect): {final_wear:.3} mm"
    );

    let early_ts = mean_slice(&lap.tire_surface_c, 0, 200);
    let late_ts = mean_slice(
        &lap.tire_surface_c,
        lap.len().saturating_sub(200),
        lap.len(),
    );
    assert!(
        late_ts > early_ts + 3.0,
        "surface warmed from cold: {early_ts:.1} °C -> {late_ts:.1} °C"
    );

    // Grip rises with temperature (no wear cliff to reverse it here — pure thermal window climb).
    let early_grip = mean_slice(&lap.tire_grip, 0, 200);
    let late_grip = mean_slice(&lap.tire_grip, lap.len().saturating_sub(200), lap.len());
    assert!(
        late_grip > early_grip + 0.02,
        "grip rose as the tyres warmed toward the window: {early_grip:.3} -> {late_grip:.3}"
    );
    assert!(
        (0.0..=1.0 + 1e-9).contains(&late_grip),
        "grip multiplier stays a physical fraction: {late_grip:.3}"
    );
}

#[test]
fn a_long_stint_drives_wear_across_the_cliff_and_grip_falls() {
    // Seed warm (in the window) so grip starts high; a long high-k_w stint pushes wear *gradually*
    // past w_c and the C¹ cliff pulls grip back down — grip peaks, then falls, wear grows monotone.
    let mut solver = skidpad_solver(95.0, 2.0e-6, 100.0, 28.0);
    let lap = solver.run(1.0e9, 90_000); // far past one lap: keep looping the skidpad

    assert!(lap.len() > 5000, "the stint ran long");
    // Wear is monotone non-decreasing per wheel.
    for w in 0..4 {
        for k in 1..lap.tire_wear_mm.len() {
            assert!(
                lap.tire_wear_mm[k][w] + 1e-9 >= lap.tire_wear_mm[k - 1][w],
                "wear never decreases (wheel {w}, step {k})"
            );
        }
    }
    let final_wear = mean_slice(&lap.tire_wear_mm, lap.len().saturating_sub(50), lap.len());
    assert!(
        final_wear > 2.0,
        "the stint wore past the cliff onset w_c = 2 mm: {final_wear:.2} mm"
    );

    // Grip peaks (warm, low wear) then falls (cliff). The last grip is below the early-stint peak.
    let peak_grip = lap
        .tire_grip
        .iter()
        .map(|r| r.iter().sum::<f64>() / 4.0)
        .fold(0.0_f64, f64::max);
    let final_grip = mean_slice(&lap.tire_grip, lap.len().saturating_sub(50), lap.len());
    assert!(
        final_grip < peak_grip - 0.05,
        "grip fell across the wear cliff: peak {peak_grip:.3} -> final {final_grip:.3}"
    );
}

#[test]
fn the_window_average_pipeline_closes_energy_exactly() {
    // Accumulating a constant per-wheel heat over N fast steps and advancing once must land the ring
    // in exactly the state a single direct ring step over the same window at the window-averaged
    // drivers reaches — i.e. Σ(P·dt)/W = avg with no loss. Isolate one wheel (front-left).
    let th = thermal_block();
    let wr = wear_block(5.0e-7);
    let g = geom();
    let mut stack = TireThermalStack::new(&th, &wr, &th, &wr, g, g, 30.0, 40.0);
    stack.seed_uniform(80.0);

    let n = 20usize;
    let dt = 0.001;
    let window = n as f64 * dt;
    let (p_slide, fz, omega, speed) = (9000.0, 4200.0, 170.0, 60.0);
    for _ in 0..n {
        stack.accumulate(&[p_slide; 4], &[fz; 4], &[omega; 4], dt);
    }
    // The ring's own hot pressure at the (pre-advance) seed state sets the contact fraction, matching
    // what `advance` samples internally on this first window.
    let ring = TireThermalRing::<f64>::from_schema_with_wear(&th, &wr);
    let mut direct = TireThermalState::uniform(80.0 + CELSIUS_K);
    let pressure = ring.pressure_pa(&direct).max(1.0);
    let ext_area = g.ext_area_m2;
    let contact_fraction = (fz / (pressure * ext_area)).clamp(0.0, 1.0);
    // Carcass driver the stack forms: c_h · Fz · (Fz/kz) · |Ω|, c_h = 0.10.
    let carcass = 0.10 * fz * (fz / g.k_vertical_n_per_m) * omega.abs();
    let drivers = ThermalDrivers {
        slip_power_w: p_slide,
        carcass_loss_w: carcass,
        speed_mps: speed,
        contact_fraction,
        ext_area_m2: ext_area,
        t_air_k: 30.0 + CELSIUS_K,
        t_road_k: 40.0 + CELSIUS_K,
    };
    ring.step(&mut direct, &drivers, window);

    let grip = stack.advance(speed).expect("a non-empty window advances");
    let st = stack.state(0);
    assert!(
        (st.t_s_k - direct.t_s_k).abs() < 1e-9,
        "surface node closes: stack {} vs direct {}",
        st.t_s_k,
        direct.t_s_k
    );
    assert!(
        (st.t_c_k - direct.t_c_k).abs() < 1e-9,
        "carcass node closes"
    );
    assert!((st.t_g_k - direct.t_g_k).abs() < 1e-9, "gas node closes");
    assert!((st.wear_mm - direct.wear_mm).abs() < 1e-12, "wear closes");
    let direct_grip = ring.couplings(&direct).mu_scale_total;
    assert!(
        (grip.mu_x[0] - direct_grip).abs() < 1e-12,
        "the returned grip matches the ring couplings"
    );
}

#[test]
fn the_wired_lap_is_deterministic() {
    let run = || {
        let mut solver = skidpad_solver(60.0, 1.0e-7, 100.0, 28.0);
        let lap = solver.run(2.0 * PI * 100.0, 30_000);
        (lap.lap_time_s, lap.vx, lap.tire_surface_c, lap.tire_wear_mm)
    };
    let a = run();
    let b = run();
    assert_eq!(a.0, b.0, "lap time is bit-reproducible");
    assert_eq!(a.1, b.1, "vx trace is bit-reproducible");
    assert_eq!(a.2, b.2, "tyre surface trace is bit-reproducible");
    assert_eq!(a.3, b.3, "tyre wear trace is bit-reproducible");
}

#[test]
fn the_frozen_default_is_unchanged_by_the_thermal_seam() {
    // A solver *without* a stack attached records no tyre-thermal channels and runs the frozen-tyre
    // path — the parity/alloc/throughput baselines depend on this being byte-identical to pre-M5.
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let ln = line(2.0 * PI * 100.0, 400, true, 0.01, 0.01, 28.0, Some(100.0));
    let mut solver = TransientSolver::new(blocks, ln, &it, SimConfig::default());
    let lap = solver.run(2.0 * PI * 100.0, 10_000);
    assert!(
        lap.tire_surface_c.is_empty(),
        "no tyre channels without a stack"
    );
    assert!(lap.tire_grip.is_empty(), "no grip channel without a stack");
    assert!(lap.len() > 100, "the frozen lap still ran");
}
