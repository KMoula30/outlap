// SPDX-License-Identifier: AGPL-3.0-only
//! Emit tire-thermal-ring traces from the **real** `outlap-tire` integrator as CSV on stdout.
//! `python/tools/plot_tire_thermal.py` runs this and plots the output, so the theory figure is
//! driven by the actual `TireThermalRing`, not a re-implementation.
//!
//! Three scenarios, one CSV with a `scenario` column (`scenario,x,a,b,c`):
//! - `warm`: cold-start warm-up under a constant hard-cornering load; `x` = time [s],
//!   `a/b/c` = surface / carcass / gas temperature [°C]. Shows the three node time constants.
//! - `couple`: the force-model couplings swept over temperature; `x` = temperature [°C],
//!   `a` = grip window `λ_μ(T_s)`, `b` = carcass stiffness factor `(1−k_c ΔT_c)`, `c` = hot
//!   pressure `p(T_g)` [kPa]. Shows what temperature does to grip, stiffness, and pressure.
//! - `steady`: steady surface temperature vs sliding-power load at two speeds; `x` = load [kW],
//!   `a` = steady `T_s` [°C] at 40 m/s, `b` at 70 m/s. Shows the load↔cooling energy balance.

use outlap_schema::tyr::TyrThermal;
use outlap_tire::{ThermalDrivers, TireThermalRing, TireThermalState};

const CELSIUS_K: f64 = 273.15;

/// An F1-slick-representative synthetic thermal block (~100 °C optimum). Uncalibrated placeholder —
/// calibration is M5 PR7/PR8; the point here is the model's *shape*, not a fitted number.
fn f1_thermal() -> TyrThermal {
    TyrThermal {
        c_s: 8000.0,
        c_c: 25000.0,
        c_g: 1500.0,
        g_sc: 150.0,
        g_cg: 60.0,
        g_road: 400.0,
        h0: 20.0,
        h1: 8.0,
        p_t: 0.65,
        t_opt: 100.0,
        c_t: 1.5,
        k_c: 0.001,
        t_c_ref: 80.0,
        p_cold: 138.0,
        t_cold: 25.0,
    }
}

fn drivers(slip_power_w: f64, speed_mps: f64) -> ThermalDrivers<f64> {
    ThermalDrivers {
        slip_power_w,
        carcass_loss_w: 1500.0,
        speed_mps,
        contact_fraction: 0.05,
        ext_area_m2: 0.4,
        t_air_k: CELSIUS_K + 30.0,
        t_road_k: CELSIUS_K + 45.0,
    }
}

/// Integrate to (near) steady state and return the surface temperature in °C.
fn steady_surface_c(ring: &TireThermalRing<f64>, d: &ThermalDrivers<f64>) -> f64 {
    let mut st = TireThermalState::uniform(CELSIUS_K + 30.0);
    for _ in 0..120_000 {
        ring.step(&mut st, d, 0.05);
    }
    st.t_s_k - CELSIUS_K
}

fn main() {
    let th = f1_thermal();
    let ring = TireThermalRing::<f64>::from_schema(&th);
    // Working-window band (±12 °C around the optimum — a plotting convenience, not a model output).
    println!(
        "# t_opt_c={} window_lo={} window_hi={}",
        th.t_opt,
        th.t_opt - 12.0,
        th.t_opt + 12.0
    );
    println!("scenario,x,a,b,c");

    // --- (a) warm: cold-start warm-up under a constant hard-cornering load. ----------------------
    let d = drivers(9200.0, 60.0);
    let mut st = TireThermalState::uniform(CELSIUS_K + 30.0);
    let dt = 0.05;
    let n = 12_000; // 600 s
    for k in 0..=n {
        if k % 20 == 0 {
            println!(
                "warm,{:.2},{:.3},{:.3},{:.3}",
                f64::from(k) * dt,
                st.t_s_k - CELSIUS_K,
                st.t_c_k - CELSIUS_K,
                st.t_g_k - CELSIUS_K,
            );
        }
        ring.step(&mut st, &d, dt);
    }

    // --- (b) couple: the three force-model couplings swept over node temperature. -----------------
    let mut temp_c = 20.0;
    while temp_c <= 170.0 {
        let node = TireThermalState::uniform(CELSIUS_K + temp_c);
        println!(
            "couple,{:.2},{:.5},{:.5},{:.3}",
            temp_c,
            ring.mu_scale(&node),             // λ_μ as if T_s = temp_c
            ring.stiffness_scale(&node),      // stiffness factor as if T_c = temp_c
            ring.pressure_pa(&node) / 1000.0, // hot pressure [kPa] as if T_g = temp_c
        );
        temp_c += 2.0;
    }

    // --- (c) steady: steady surface temperature vs sliding-power load, at two speeds. -------------
    let mut kw = 2.0;
    while kw <= 16.0 {
        let ts_40 = steady_surface_c(&ring, &drivers(kw * 1000.0, 40.0));
        let ts_70 = steady_surface_c(&ring, &drivers(kw * 1000.0, 70.0));
        println!("steady,{kw:.2},{ts_40:.3},{ts_70:.3},");
        kw += 0.5;
    }
}
