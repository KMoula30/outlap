// SPDX-License-Identifier: AGPL-3.0-only
//! Property + validation tests for the tire thermal ring (CLAUDE.md: new physics ⇒ property tests).
//!
//! Coverage (HANDOFF §13/§14):
//! - the discrete fixed point equals the closed-form steady state (integrator correctness),
//! - steady-state **energy closure** (heat in = convection + road rejection),
//! - warm-up time constant + steady surface temperature land in a broadcast-consistent band for an
//!   F1-representative parameter set,
//! - the grip window `λ_μ ≤ 1` peaks at `T_opt`; convection monotone in speed; gas law calibrated;
//!   carcass softening reduces stiffness,
//! - determinism (bit-identical on re-run) and f32/f64 parity.
#![allow(clippy::float_cmp)]

use outlap_schema::tyr::TyrThermal;
use outlap_tire::{ThermalDrivers, TireThermalRing, TireThermalState};
use proptest::prelude::*;

const CELSIUS_K: f64 = 273.15;

/// The shipped passenger-car synthetic thermal block (`data/tires/pacejka_2006_205_60r15`).
fn passenger_thermal() -> TyrThermal {
    TyrThermal {
        c_s: 6000.0,
        c_c: 18000.0,
        c_g: 1300.0,
        g_sc: 80.0,
        g_cg: 35.0,
        g_road: 220.0,
        h0: 15.0,
        h1: 5.5,
        p_t: 0.65,
        t_opt: 75.0,
        c_t: 2.0,
        k_c: 0.0015,
        t_c_ref: 60.0,
        p_cold: 220.0,
        t_cold: 20.0,
    }
}

/// An F1-slick-representative thermal block (larger capacities, ~100 °C optimum) — used to check the
/// steady temperature and warm-up land in the broadcast operating band. Physically plausible, not a
/// fitted/proprietary set (calibration is PR7/PR8).
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

fn c_to_k(c: f64) -> f64 {
    c + CELSIUS_K
}

/// A steady operating point: hard cornering at `speed`, patch fraction `a_cp`, on a warm track.
fn drivers(slip_power_w: f64, carcass_loss_w: f64, speed_mps: f64) -> ThermalDrivers<f64> {
    ThermalDrivers {
        slip_power_w,
        carcass_loss_w,
        speed_mps,
        contact_fraction: 0.05,
        ext_area_m2: 0.4,
        t_air_k: c_to_k(30.0),
        t_road_k: c_to_k(45.0),
    }
}

/// Closed-form steady state of the ring under constant `d` (see the module docs of `thermal.rs`):
/// `T_g* = T_c* = T_s* + Q_hyst/g_sc`, and `T_s*` closes the surface energy balance.
fn closed_form_steady(th: &TyrThermal, d: &ThermalDrivers<f64>) -> (f64, f64, f64) {
    let a_cp = d.contact_fraction;
    let g_conv = (th.h0 + th.h1 * d.speed_mps.powf(0.8)) * d.ext_area_m2;
    let g_air = g_conv * (1.0 - a_cp);
    let g_rd = th.g_road * a_cp;
    let q_fric = th.p_t * d.slip_power_w;
    let q_in = q_fric + d.carcass_loss_w;
    let t_s = (q_in + g_air * d.t_air_k + g_rd * d.t_road_k) / (g_air + g_rd);
    let t_c = t_s + d.carcass_loss_w / th.g_sc;
    (t_s, t_c, t_c)
}

/// Integrate from a uniform cold start to (near) steady state.
fn run_to_steady(
    th: &TyrThermal,
    d: &ThermalDrivers<f64>,
    t0_k: f64,
    dt: f64,
    n: usize,
) -> TireThermalState<f64> {
    let ring = TireThermalRing::<f64>::from_schema(th);
    let mut st = TireThermalState::uniform(t0_k);
    for _ in 0..n {
        ring.step(&mut st, d, dt);
    }
    st
}

#[test]
fn discrete_fixed_point_matches_closed_form() {
    let th = passenger_thermal();
    let d = drivers(9000.0, 1500.0, 45.0);
    let (ts, tc, tg) = closed_form_steady(&th, &d);
    let st = run_to_steady(&th, &d, c_to_k(20.0), 0.02, 400_000);
    assert!((st.t_s_k - ts).abs() < 1e-3, "T_s {} vs {}", st.t_s_k, ts);
    assert!((st.t_c_k - tc).abs() < 1e-3, "T_c {} vs {}", st.t_c_k, tc);
    assert!((st.t_g_k - tg).abs() < 1e-3, "T_g {} vs {}", st.t_g_k, tg);
    // Started at the fixed point ⇒ stays there (it is a genuine fixed point of the scheme).
    let ring = TireThermalRing::<f64>::from_schema(&th);
    let mut at_fp = TireThermalState::new(ts, tc, tg);
    ring.step(&mut at_fp, &d, 0.02);
    assert!((at_fp.t_s_k - ts).abs() < 1e-9);
    assert!((at_fp.t_c_k - tc).abs() < 1e-9);
    assert!((at_fp.t_g_k - tg).abs() < 1e-9);
}

#[test]
fn steady_state_energy_closes() {
    // All heat in (Q_fric + Q_hyst) must leave through the surface (convection + road) at steady
    // state — the gas carries no external loss path, so the balance is on the surface node.
    let th = passenger_thermal();
    let d = drivers(9000.0, 1500.0, 45.0);
    let st = run_to_steady(&th, &d, c_to_k(20.0), 0.02, 400_000);
    let a_cp = d.contact_fraction;
    let g_conv = (th.h0 + th.h1 * d.speed_mps.powf(0.8)) * d.ext_area_m2;
    let g_air = g_conv * (1.0 - a_cp);
    let g_rd = th.g_road * a_cp;
    let q_in = th.p_t * d.slip_power_w + d.carcass_loss_w;
    let q_out = g_air * (st.t_s_k - d.t_air_k) + g_rd * (st.t_s_k - d.t_road_k);
    assert!(
        (q_in - q_out).abs() <= 1e-3 * q_in,
        "energy not closed: in {q_in} W vs out {q_out} W"
    );
}

#[test]
fn warmup_and_steady_land_in_broadcast_band() {
    // F1-representative params + a hard-cornering heat load. Three checks against the published
    // motorsport-tyre picture (Farroni TRT / F1 broadcast):
    //  1. the closed-form steady surface temperature sits in the ~85–115 °C operating band,
    //  2. the surface node's thermal time constant τ_s = C_s / G_s is tens of seconds (the fast
    //     node — Farroni's surface layer responds in seconds, not minutes),
    //  3. from cold the surface warms monotonically into the working window on a lap scale.
    let th = f1_thermal();
    let d = drivers(9200.0, 1500.0, 60.0);
    let (ts_star, _, _) = closed_form_steady(&th, &d);
    let steady_c = ts_star - CELSIUS_K;
    assert!(
        (85.0..=115.0).contains(&steady_c),
        "steady surface temp {steady_c:.1} °C outside the broadcast band"
    );

    // Surface time constant at the operating point (G_s = g_sc + g_air + g_road·a_cp).
    let g_conv = (th.h0 + th.h1 * d.speed_mps.powf(0.8)) * d.ext_area_m2;
    let g_s = th.g_sc + g_conv * (1.0 - d.contact_fraction) + th.g_road * d.contact_fraction;
    let tau_s = th.c_s / g_s;
    assert!(
        (10.0..=60.0).contains(&tau_s),
        "surface time constant {tau_s:.1} s is not the tens-of-seconds Farroni band"
    );

    // From a realistic tyre-blanket start (~70 °C), the surface warms monotonically into the
    // working window on a lap scale (real F1 slicks leave the blankets and switch on over a few laps).
    let ring = TireThermalRing::<f64>::from_schema(&th);
    let mut st = TireThermalState::uniform(c_to_k(70.0));
    let dt = 0.02;
    let mut prev = st.t_s_k;
    let mut t_to_window = f64::INFINITY;
    for k in 0..200_000 {
        ring.step(&mut st, &d, dt);
        assert!(
            st.t_s_k >= prev - 1e-9,
            "surface must warm monotonically from a blanket start"
        );
        prev = st.t_s_k;
        if t_to_window.is_infinite() && (st.t_s_k - CELSIUS_K) >= 85.0 {
            t_to_window = f64::from(k) * dt;
        }
    }
    // Not instant, not never — a few laps into the window (~211 s for this synthetic set).
    assert!(
        (30.0..=300.0).contains(&t_to_window),
        "warm-up time into the working window was {t_to_window} s"
    );
}

#[test]
fn mu_scale_peaks_at_topt_and_is_bounded() {
    let th = passenger_thermal();
    let ring = TireThermalRing::<f64>::from_schema(&th);
    let at_opt = TireThermalState::uniform(c_to_k(th.t_opt));
    assert!(
        (ring.mu_scale(&at_opt) - 1.0).abs() < 1e-12,
        "λ_μ peaks at 1 at T_opt"
    );
    for ts_c in [-10.0, 20.0, 50.0, 75.0, 100.0, 150.0, 200.0] {
        let st = TireThermalState::uniform(c_to_k(ts_c));
        let lam = ring.mu_scale(&st);
        assert!(
            lam > 0.0 && lam <= 1.0 + 1e-12,
            "λ_μ({ts_c}) = {lam} out of (0,1]"
        );
    }
    // Symmetric in the °C deviation about T_opt.
    let lo = ring.mu_scale(&TireThermalState::uniform(c_to_k(th.t_opt - 20.0)));
    let hi = ring.mu_scale(&TireThermalState::uniform(c_to_k(th.t_opt + 20.0)));
    assert!(
        (lo - hi).abs() < 1e-12,
        "grip window not symmetric in °C: {lo} vs {hi}"
    );
}

#[test]
fn convection_monotone_in_speed() {
    let ring = TireThermalRing::<f64>::from_schema(&passenger_thermal());
    let mut prev = ring.conv_conductance(0.0, 0.4);
    for v in [1.0, 5.0, 20.0, 40.0, 60.0, 90.0] {
        let g = ring.conv_conductance(v, 0.4);
        assert!(g > prev, "g_conv must increase with speed at v={v}");
        prev = g;
    }
}

#[test]
fn gas_law_monotone_and_calibrated() {
    let th = passenger_thermal();
    let ring = TireThermalRing::<f64>::from_schema(&th);
    // At the cold reference temperature the pressure is exactly p_cold (kPa → Pa).
    let cold = TireThermalState::uniform(c_to_k(th.t_cold));
    assert!((ring.pressure_pa(&cold) - th.p_cold * 1000.0).abs() < 1e-6);
    // Hotter gas ⇒ higher pressure (ideal-gas ratio in absolute temperature).
    let warm = TireThermalState::uniform(c_to_k(th.t_cold + 60.0));
    assert!(ring.pressure_pa(&warm) > ring.pressure_pa(&cold));
}

#[test]
fn carcass_softening_reduces_stiffness() {
    let th = passenger_thermal();
    let ring = TireThermalRing::<f64>::from_schema(&th);
    let at_ref = ring.stiffness_scale(&TireThermalState::uniform(c_to_k(th.t_c_ref)));
    assert!(
        (at_ref - 1.0).abs() < 1e-12,
        "stiffness is 1 at the reference temperature"
    );
    let hot = ring.stiffness_scale(&TireThermalState::uniform(c_to_k(th.t_c_ref + 40.0)));
    assert!(
        hot < at_ref && hot > 0.0,
        "hot carcass must be softer (0 < {hot} < 1)"
    );
}

#[test]
fn determinism_bit_identical() {
    let th = passenger_thermal();
    let d = drivers(7000.0, 1200.0, 50.0);
    let a = run_to_steady(&th, &d, c_to_k(15.0), 0.05, 5000);
    let b = run_to_steady(&th, &d, c_to_k(15.0), 0.05, 5000);
    assert_eq!(a.t_s_k.to_bits(), b.t_s_k.to_bits());
    assert_eq!(a.t_c_k.to_bits(), b.t_c_k.to_bits());
    assert_eq!(a.t_g_k.to_bits(), b.t_g_k.to_bits());
}

#[test]
#[allow(clippy::cast_possible_truncation)] // the f32 downcast is the whole point of this parity test.
fn f32_f64_parity() {
    let th = passenger_thermal();
    let d64 = drivers(7000.0, 1200.0, 50.0);
    let d32 = ThermalDrivers::<f32> {
        slip_power_w: d64.slip_power_w as f32,
        carcass_loss_w: d64.carcass_loss_w as f32,
        speed_mps: d64.speed_mps as f32,
        contact_fraction: d64.contact_fraction as f32,
        ext_area_m2: d64.ext_area_m2 as f32,
        t_air_k: d64.t_air_k as f32,
        t_road_k: d64.t_road_k as f32,
    };
    let r64 = TireThermalRing::<f64>::from_schema(&th);
    let r32 = TireThermalRing::<f32>::from_schema(&th);
    let mut s64 = TireThermalState::uniform(c_to_k(15.0));
    let mut s32 = TireThermalState::uniform(c_to_k(15.0) as f32);
    for _ in 0..2000 {
        r64.step(&mut s64, &d64, 0.05);
        r32.step(&mut s32, &d32, 0.05);
    }
    assert!(
        (f64::from(s32.t_s_k) - s64.t_s_k).abs() < 0.5,
        "f32/f64 surface diverged"
    );
    assert!(
        (f64::from(s32.t_c_k) - s64.t_c_k).abs() < 0.5,
        "f32/f64 carcass diverged"
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// Over a hostile-but-finite driver box the ring stays finite and physical (temps positive K).
    #[test]
    fn outputs_finite_and_physical(
        slip in 0.0..40_000.0,
        hyst in 0.0..6000.0,
        speed in 0.0..100.0,
        a_cp in 0.0..0.3,
        t0_c in -20.0..60.0,
    ) {
        let th = passenger_thermal();
        let ring = TireThermalRing::<f64>::from_schema(&th);
        let mut d = drivers(slip, hyst, speed);
        d.contact_fraction = a_cp;
        let mut st = TireThermalState::uniform(c_to_k(t0_c));
        for _ in 0..2000 {
            let c = ring.step(&mut st, &d, 0.05);
            prop_assert!(st.t_s_k.is_finite() && st.t_c_k.is_finite() && st.t_g_k.is_finite());
            prop_assert!(st.t_s_k > 0.0 && st.t_c_k > 0.0 && st.t_g_k > 0.0);
            prop_assert!(c.mu_scale > 0.0 && c.mu_scale <= 1.0 + 1e-12);
            prop_assert!(c.stiffness_scale > 0.0);
            prop_assert!(c.pressure_pa > 0.0);
        }
    }

    /// From a cold uniform start under a positive heat load, all three nodes warm up monotonically
    /// (semi-implicit Euler cannot overshoot the moving target).
    #[test]
    fn cold_start_warms_monotonically(
        slip in 2000.0..20_000.0,
        hyst in 200.0..4000.0,
        speed in 5.0..90.0,
    ) {
        let th = passenger_thermal();
        let ring = TireThermalRing::<f64>::from_schema(&th);
        let d = drivers(slip, hyst, speed);
        // Cold start below the eventual steady state, so every node is heating.
        let mut st = TireThermalState::uniform(c_to_k(10.0));
        let (mut ps, mut pc, mut pg) = (st.t_s_k, st.t_c_k, st.t_g_k);
        for _ in 0..3000 {
            ring.step(&mut st, &d, 0.05);
            prop_assert!(st.t_s_k >= ps - 1e-9);
            prop_assert!(st.t_c_k >= pc - 1e-9);
            prop_assert!(st.t_g_k >= pg - 1e-9);
            ps = st.t_s_k; pc = st.t_c_k; pg = st.t_g_k;
        }
    }

    /// Wear-free grip window: strictly decreasing away from T_opt on either side.
    #[test]
    fn grip_window_falls_off_optimum(delta in 5.0..80.0) {
        let th = passenger_thermal();
        let ring = TireThermalRing::<f64>::from_schema(&th);
        let peak = ring.mu_scale(&TireThermalState::uniform(c_to_k(th.t_opt)));
        let below = ring.mu_scale(&TireThermalState::uniform(c_to_k(th.t_opt - delta)));
        let above = ring.mu_scale(&TireThermalState::uniform(c_to_k(th.t_opt + delta)));
        prop_assert!(below < peak && above < peak);
    }
}
