// SPDX-License-Identifier: AGPL-3.0-only
//! Property + validation tests for the §7.3 tire wear / thermal-damage law layered on the thermal
//! ring (CLAUDE.md: new physics ⇒ property tests).
//!
//! Coverage (HANDOFF §13/§14):
//! - **wear is monotone in sliding energy** (§14): `w` only ever grows, and more sliding power (or
//!   more time) removes more tread,
//! - **damage is monotone non-decreasing and irreversible**: `D` grows only while `T_c > T_deg` and
//!   never falls, even when the tire cools,
//! - the total grip `λ_μ,total ∈ [0,1]` and is **C¹ across the cliff** (finite-difference derivative
//!   continuity through `w_c`),
//! - the **`C_s(w)` positive feedback** — a worn tire, with less surface capacity, runs hotter peaks
//!   under an oscillating (corner/straight) load than a fresh tire,
//! - wear accelerates with surface temperature (Grosch hardness falling with `T_s`),
//! - the thermal-only `from_schema` path is inert (no wear, no damage, grip factors = 1),
//! - determinism (bit-identical) and f32/f64 parity of the wear/damage advance.
#![allow(clippy::float_cmp)]

use outlap_schema::tyr::{TyrThermal, TyrWear};
use outlap_tire::{ThermalDrivers, TireThermalRing, TireThermalState};
use proptest::prelude::*;

const CELSIUS_K: f64 = 273.15;

fn c_to_k(c: f64) -> f64 {
    c + CELSIUS_K
}

/// An F1-slick-representative thermal block (~100 °C optimum) — the same shape used in the thermal
/// tests. Physically plausible, not a fitted set (calibration is PR7/PR8).
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

/// A racing-slick-band synthetic wear block, chosen so a hard-cornering load actually reaches the
/// cliff and the damage threshold on a testable timescale (uncalibrated — PR7/PR8).
fn f1_wear() -> TyrWear {
    TyrWear {
        k_w: 1.5e-8,
        w_max: 8.0,
        w_c: 3.0,
        tau_d: 400.0,
        t_deg: 115.0,
        delta_t_ref: 20.0,
        beta: 2.0,
        delta_c: 0.30,
        s_w: 0.4,
        delta_d: 0.25,
    }
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

fn ring() -> TireThermalRing<f64> {
    TireThermalRing::<f64>::from_schema_with_wear(&f1_thermal(), &f1_wear())
}

/// Integrate a fresh cold tire for `n` steps under a constant load and return the terminal state.
fn run(d: &ThermalDrivers<f64>, dt: f64, n: usize) -> TireThermalState<f64> {
    let r = ring();
    let mut st = TireThermalState::uniform(c_to_k(80.0));
    for _ in 0..n {
        r.step(&mut st, d, dt);
    }
    st
}

#[test]
fn wear_monotone_and_grows_with_sliding_energy() {
    let r = ring();
    // Within a single run the wear only ever increases.
    let d = drivers(12_000.0, 1500.0, 60.0);
    let mut st = TireThermalState::uniform(c_to_k(80.0));
    let mut prev = st.wear_mm;
    for _ in 0..20_000 {
        r.step(&mut st, &d, 0.05);
        assert!(
            st.wear_mm >= prev - 1e-15,
            "wear must be monotone non-decreasing"
        );
        prev = st.wear_mm;
    }
    assert!(st.wear_mm > 0.0, "a loaded tire must wear");

    // More sliding power over the same time removes more tread (monotone in sliding energy, §14).
    // Partial-wear runs (~150 s) so neither has hit the bald limit and the comparison is genuine.
    let low = run(&drivers(6_000.0, 1500.0, 60.0), 0.05, 3_000).wear_mm;
    let high = run(&drivers(18_000.0, 1500.0, 60.0), 0.05, 3_000).wear_mm;
    assert!(
        high < 8.0 && high > low,
        "more sliding power ⇒ more wear: {high} !> {low}"
    );
    // Zero sliding power ⇒ zero wear (Archard is driven purely by frictional sliding energy).
    let none = run(&drivers(0.0, 1500.0, 60.0), 0.05, 3_000).wear_mm;
    assert!(none.abs() < 1e-12, "no sliding ⇒ no wear, got {none}");
}

#[test]
fn wear_accelerates_with_surface_temperature() {
    // Grosch: hotter rubber is softer and wears faster. Hold everything but the surface temperature
    // fixed and compare the instantaneous wear rate.
    let r = ring();
    let cold = r.wear_rate(c_to_k(80.0), 10_000.0, 0.02);
    let hot = r.wear_rate(c_to_k(120.0), 10_000.0, 0.02);
    assert!(
        hot > cold,
        "wear rate must rise with surface temperature: {hot} !> {cold}"
    );
    assert!(cold > 0.0, "positive sliding load ⇒ positive wear rate");
}

#[test]
fn damage_monotone_irreversible_and_thresholded() {
    let r = ring();
    // Below T_deg the carcass takes no damage.
    let wr = f1_wear();
    let cool = drivers(3_000.0, 400.0, 70.0); // light load, carcass stays under T_deg
    let mut st = TireThermalState::uniform(c_to_k(70.0));
    for _ in 0..10_000 {
        r.step(&mut st, &cool, 0.05);
    }
    assert!(
        (st.t_c_k - CELSIUS_K) < wr.t_deg,
        "sanity: this load should keep the carcass under T_deg"
    );
    assert!(
        st.damage.abs() < 1e-12,
        "no damage below T_deg, got {}",
        st.damage
    );

    // A punishing load drives the carcass over T_deg and accumulates damage monotonically.
    let hard = drivers(22_000.0, 6_000.0, 55.0);
    let mut st = TireThermalState::uniform(c_to_k(90.0));
    let mut prev = st.damage;
    for _ in 0..40_000 {
        r.step(&mut st, &hard, 0.05);
        assert!(
            st.damage >= prev - 1e-15,
            "damage must be monotone non-decreasing"
        );
        assert!(
            (0.0..=1.0).contains(&st.damage),
            "damage out of [0,1]: {}",
            st.damage
        );
        prev = st.damage;
    }
    let damaged = st.damage;
    assert!(
        damaged > 0.0,
        "an overheated carcass must accumulate damage"
    );

    // Now cool it right down for a long time: damage is irreversible, it must not recover.
    let cooldown = drivers(0.0, 0.0, 90.0);
    for _ in 0..40_000 {
        r.step(&mut st, &cooldown, 0.05);
    }
    assert!(
        st.damage >= damaged - 1e-15,
        "damage recovered on cooldown ({} < {damaged}) — it must be irreversible",
        st.damage
    );
}

#[test]
fn total_grip_bounded_and_c1_across_the_cliff() {
    let r = ring();
    let wr = f1_wear();
    // λ_μ,total ∈ [0,1] over the whole wear range, at the grip optimum (isolate the wear factors).
    let at_opt = f1_thermal().t_opt;
    let sample = |w: f64| {
        let st = TireThermalState::with_wear(c_to_k(at_opt), c_to_k(80.0), c_to_k(60.0), w, 0.0);
        r.couplings(&st).mu_scale_total
    };
    let mut w = 0.0;
    while w <= wr.w_max {
        let g = sample(w);
        assert!(
            (0.0..=1.0 + 1e-12).contains(&g),
            "λ_μ,total({w}) = {g} out of [0,1]"
        );
        w += 0.05;
    }
    // Grip is monotone-decreasing in wear (the sigmoid cliff is monotone everywhere).
    assert!(
        sample(0.0) > sample(wr.w_c),
        "grip must fall as the tire wears"
    );
    assert!(
        sample(wr.w_c) > sample(wr.w_max),
        "grip must keep falling through the cliff"
    );

    // C¹ continuity across the cliff: the numeric derivative of grip w.r.t. w has no jump at w_c.
    let h = 1e-4;
    let deriv = |w: f64| (sample(w + h) - sample(w - h)) / (2.0 * h);
    let just_below = deriv(wr.w_c - 5.0 * h);
    let at_cliff = deriv(wr.w_c);
    let just_above = deriv(wr.w_c + 5.0 * h);
    assert!(
        (at_cliff - just_below).abs() < 1e-2 && (just_above - at_cliff).abs() < 1e-2,
        "grip derivative is discontinuous across the cliff: {just_below} / {at_cliff} / {just_above}"
    );
}

#[test]
fn worn_tire_runs_hotter_under_oscillating_load() {
    // The C_s(w) feedback: a worn tire has less surface capacity, so its surface tracks the load
    // peaks more closely — under a corner/straight oscillation it reaches higher peak temperatures
    // than a fresh tire. (Under a *constant* load the steady T_s is capacity-independent; the
    // feedback bites on the transient peaks, which is exactly where the cliff lives — see the doc.)
    let r = ring();
    let fresh_cap = r.c_s_effective(0.0);
    let worn_cap = r.c_s_effective(f1_wear().w_max * 0.9);
    assert!(
        worn_cap < fresh_cap,
        "a worn tire must have less surface capacity"
    );

    let peak = |w0: f64| {
        let mut st = TireThermalState::with_wear(c_to_k(95.0), c_to_k(95.0), c_to_k(95.0), w0, 0.0);
        let mut hot = f64::NEG_INFINITY;
        // 40 s of a 2 s corner / 2 s straight square wave.
        for k in 0..800 {
            let corner = (k / 40) % 2 == 0; // 40 steps @ 0.05 s = 2 s
            let load = if corner { 16_000.0 } else { 1_000.0 };
            let mut st_frozen = st; // keep wear from evolving so we compare capacity only
            let d = drivers(load, 1500.0, if corner { 45.0 } else { 75.0 });
            r.step(&mut st_frozen, &d, 0.05);
            st.t_s_k = st_frozen.t_s_k;
            st.t_c_k = st_frozen.t_c_k;
            st.t_g_k = st_frozen.t_g_k;
            hot = hot.max(st.t_s_k);
        }
        hot
    };
    let fresh_peak = peak(0.0);
    let worn_peak = peak(f1_wear().w_max * 0.9);
    assert!(
        worn_peak > fresh_peak + 1e-6,
        "worn tire should peak hotter under load oscillation: worn {worn_peak} vs fresh {fresh_peak}"
    );
}

#[test]
fn thermal_only_ring_is_inert_to_wear() {
    // The `from_schema` (thermal-only) path must not wear or damage, and its grip factors are 1 —
    // that is what keeps the isolated-ring behaviour bit-identical to the pre-wear PR.
    let r = TireThermalRing::<f64>::from_schema(&f1_thermal());
    let d = drivers(20_000.0, 5_000.0, 55.0);
    let mut st = TireThermalState::uniform(c_to_k(90.0));
    for _ in 0..40_000 {
        let c = r.step(&mut st, &d, 0.05);
        assert_eq!(c.wear_grip_scale, 1.0, "inert ring must not lose wear grip");
        assert_eq!(c.damage_grip_scale, 1.0, "inert ring must not take damage");
        assert_eq!(
            c.mu_scale_total, c.mu_scale,
            "inert total grip is the thermal window"
        );
    }
    assert_eq!(st.wear_mm, 0.0, "inert ring must not wear");
    assert_eq!(st.damage, 0.0, "inert ring must not accumulate damage");
}

#[test]
fn determinism_bit_identical() {
    let d = drivers(14_000.0, 3_000.0, 55.0);
    let a = run(&d, 0.05, 8000);
    let b = run(&d, 0.05, 8000);
    assert_eq!(a.wear_mm.to_bits(), b.wear_mm.to_bits());
    assert_eq!(a.damage.to_bits(), b.damage.to_bits());
    assert_eq!(a.t_s_k.to_bits(), b.t_s_k.to_bits());
}

#[test]
#[allow(clippy::cast_possible_truncation)] // the f32 downcast is the point of this parity test.
fn f32_f64_parity() {
    let th = f1_thermal();
    let wr = f1_wear();
    let d64 = drivers(14_000.0, 3_000.0, 55.0);
    let d32 = ThermalDrivers::<f32> {
        slip_power_w: d64.slip_power_w as f32,
        carcass_loss_w: d64.carcass_loss_w as f32,
        speed_mps: d64.speed_mps as f32,
        contact_fraction: d64.contact_fraction as f32,
        ext_area_m2: d64.ext_area_m2 as f32,
        t_air_k: d64.t_air_k as f32,
        t_road_k: d64.t_road_k as f32,
    };
    let r64 = TireThermalRing::<f64>::from_schema_with_wear(&th, &wr);
    let r32 = TireThermalRing::<f32>::from_schema_with_wear(&th, &wr);
    let mut s64 = TireThermalState::uniform(c_to_k(80.0));
    let mut s32 = TireThermalState::uniform(c_to_k(80.0) as f32);
    for _ in 0..8000 {
        r64.step(&mut s64, &d64, 0.05);
        r32.step(&mut s32, &d32, 0.05);
    }
    assert!(
        (f64::from(s32.wear_mm) - s64.wear_mm).abs() < 1e-2,
        "f32/f64 wear diverged"
    );
    assert!(
        (f64::from(s32.damage) - s64.damage).abs() < 1e-2,
        "f32/f64 damage diverged"
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// Over a hostile-but-finite driver box the wear/damage states stay finite and physical, and
    /// both accumulators are monotone across the whole run.
    #[test]
    fn wear_and_damage_finite_and_monotone(
        slip in 0.0..40_000.0,
        hyst in 0.0..8000.0,
        speed in 0.0..100.0,
        a_cp in 0.0..0.3,
    ) {
        let r = ring();
        let mut d = drivers(slip, hyst, speed);
        d.contact_fraction = a_cp;
        let mut st = TireThermalState::uniform(c_to_k(60.0));
        let (mut pw, mut pd) = (st.wear_mm, st.damage);
        for _ in 0..4000 {
            let c = r.step(&mut st, &d, 0.05);
            prop_assert!(st.wear_mm.is_finite() && st.damage.is_finite());
            prop_assert!(st.wear_mm >= pw - 1e-12 && st.wear_mm <= 8.0 + 1e-9);
            prop_assert!(st.damage >= pd - 1e-12 && (0.0..=1.0 + 1e-12).contains(&st.damage));
            prop_assert!((0.0..=1.0 + 1e-9).contains(&c.mu_scale_total));
            pw = st.wear_mm;
            pd = st.damage;
        }
    }
}
