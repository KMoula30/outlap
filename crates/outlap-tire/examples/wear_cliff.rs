// SPDX-License-Identifier: AGPL-3.0-only
//! Emit tire wear / thermal-damage traces from the **real** `outlap-tire` integrator as CSV on
//! stdout. `python/tools/plot_tire_wear.py` runs this and plots the output, so the §7.3 theory figure
//! is driven by the actual `TireThermalRing`, not a re-implementation.
//!
//! Three scenarios, one CSV with a `scenario` column (`scenario,x,a,b,c`):
//! - `stint`: a hard cornering stint from new tires; `x` = time [s], `a` = wear `w` [mm],
//!   `b` = thermal damage `D` [0..1], `c` = surface temperature `T_s` [°C]. Shows the Archard wear
//!   growth and the damage onset once the carcass runs hot.
//! - `grip`: the grip factors swept over wear at the optimum surface temperature; `x` = wear [mm],
//!   `a` = total grip `λ_μ,total`, `b` = the cliff factor `(1−Δ_c σ)`, `c` = the damage factor at a
//!   representative `D`. Shows the C¹ cliff.
//! - `feedback`: fresh vs worn surface temperature under a corner/straight load oscillation;
//!   `x` = time [s], `a` = fresh `T_s` [°C], `b` = worn `T_s` [°C]. Shows the `C_s(w)` positive
//!   feedback — the worn tire, with less surface capacity, peaks hotter.

use outlap_schema::tyr::{TyrThermal, TyrWear};
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

/// A racing-slick-band synthetic wear block, scaled so a hard stint reaches the cliff and the damage
/// threshold within the plotted window. Uncalibrated placeholder (M5 PR7/PR8).
fn f1_wear() -> TyrWear {
    TyrWear {
        k_w: 1.0e-8,
        w_max: 8.0,
        w_c: 3.0,
        tau_d: 500.0,
        t_deg: 108.0,
        delta_t_ref: 18.0,
        beta: 2.0,
        delta_c: 0.30,
        s_w: 0.4,
        delta_d: 0.25,
    }
}

fn drivers(slip_power_w: f64, carcass_loss_w: f64, speed_mps: f64) -> ThermalDrivers<f64> {
    ThermalDrivers {
        slip_power_w,
        carcass_loss_w,
        speed_mps,
        contact_fraction: 0.05,
        ext_area_m2: 0.4,
        t_air_k: CELSIUS_K + 30.0,
        t_road_k: CELSIUS_K + 45.0,
    }
}

fn main() {
    let th = f1_thermal();
    let wr = f1_wear();
    let ring = TireThermalRing::<f64>::from_schema_with_wear(&th, &wr);
    println!(
        "# t_opt_c={} w_c={} w_max={} delta_c={} t_deg_c={}",
        th.t_opt, wr.w_c, wr.w_max, wr.delta_c, wr.t_deg
    );
    println!("scenario,x,a,b,c");

    // --- (a) stint: a hard cornering stint from new tires. --------------------------------------
    let d = drivers(11_000.0, 2_500.0, 55.0);
    let mut st = TireThermalState::uniform(CELSIUS_K + 80.0);
    let dt = 0.05;
    let n = 40_000; // 2000 s
    for k in 0..=n {
        if k % 100 == 0 {
            println!(
                "stint,{:.1},{:.4},{:.4},{:.3}",
                f64::from(k) * dt,
                st.wear_mm,
                st.damage,
                st.t_s_k - CELSIUS_K,
            );
        }
        ring.step(&mut st, &d, dt);
    }

    // --- (b) grip: the grip factors swept over wear at the optimum surface temperature. ----------
    // Evaluated at T_s = T_opt (so λ_μ(T_s)=1) and a representative damage D=0.4 for the damage line.
    let mut w = 0.0;
    while w <= wr.w_max {
        let node = TireThermalState::with_wear(
            CELSIUS_K + th.t_opt,
            CELSIUS_K + 80.0,
            CELSIUS_K + 60.0,
            w,
            0.4,
        );
        let c = ring.couplings(&node);
        println!(
            "grip,{:.3},{:.5},{:.5},{:.5}",
            w, c.mu_scale_total, c.wear_grip_scale, c.damage_grip_scale,
        );
        w += 0.05;
    }

    // --- (c) feedback: fresh vs worn T_s under a corner/straight load oscillation. ----------------
    // Both tires' wear is frozen at its initial value so the comparison isolates the C_s(w) effect.
    // 40 s of warm-up (discarded) so the surface reaches its periodic steady oscillation, then 24 s
    // of the steady cycle: the worn tire, with less surface capacity, swings wider — higher peaks in
    // the corners and lower troughs on the straights. The higher peaks are what tip a worn tire out
    // of its grip window and into the cliff.
    let warmup = 8_000; // 400 s — long enough that both tires reach their periodic steady orbit
    let feedback = |w0: f64| -> Vec<(f64, f64)> {
        let mut st = TireThermalState::with_wear(
            CELSIUS_K + 100.0,
            CELSIUS_K + 100.0,
            CELSIUS_K + 100.0,
            w0,
            0.0,
        );
        let mut out = Vec::new();
        for k in 0..(warmup + 480) {
            let corner = (k / 40) % 2 == 0; // 40 steps @ 0.05 s = 2 s
            let load = if corner { 16_000.0 } else { 1_000.0 };
            let speed = if corner { 45.0 } else { 75.0 };
            let mut frozen = st; // freeze wear so the comparison isolates the C_s(w) effect
            ring.step(&mut frozen, &drivers(load, 1500.0, speed), dt);
            st.t_s_k = frozen.t_s_k;
            st.t_c_k = frozen.t_c_k;
            st.t_g_k = frozen.t_g_k;
            if k >= warmup {
                out.push((f64::from(k - warmup) * dt, st.t_s_k - CELSIUS_K));
            }
        }
        out
    };
    let fresh = feedback(0.0);
    let worn = feedback(wr.w_max * 0.9);
    for ((t, ts_fresh), (_, ts_worn)) in fresh.iter().zip(worn.iter()) {
        println!("feedback,{t:.2},{ts_fresh:.3},{ts_worn:.3},");
    }
}
