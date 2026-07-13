// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the MF6.1 evaluation path (CLAUDE.md: allocs/step is CI-enforced).
//!
//! Construction (map → dense params) may allocate; `Mf61::forces` must not. dhat's testing
//! profiler counts heap blocks; we assert the count is unchanged across warmed evaluations —
//! the same pattern as `outlap-qss/tests/alloc.rs`.

use std::collections::BTreeMap;

use outlap_schema::tyr::{TyrThermal, TyrWear};
use outlap_tire::{
    relax_step, Brush, Mf61, Mf61Params, Relaxation, SlipState, ThermalDrivers, TireThermalRing,
    TireThermalState,
};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

fn full_map() -> BTreeMap<String, f64> {
    let pairs: &[(&str, f64)] = &[
        ("FNOMIN", 4000.0),
        ("UNLOADED_RADIUS", 0.33),
        ("NOMPRES", 220_000.0),
        ("LONGVL", 16.7),
        ("PCX1", 1.65),
        ("PDX1", 1.30),
        ("PDX2", -0.05),
        ("PEX1", 0.10),
        ("PKX1", 22.0),
        ("PHX1", 0.002),
        ("PVX1", 0.01),
        ("PPX1", -0.3),
        ("PPX3", -0.1),
        ("RBX1", 13.0),
        ("RBX2", 10.0),
        ("RCX1", 1.0),
        ("PCY1", 1.40),
        ("PDY1", 1.25),
        ("PDY3", -1.0),
        ("PEY1", -1.0),
        ("PKY1", -20.0),
        ("PKY2", 1.8),
        ("PKY4", 2.0),
        ("PKY6", -1.0),
        ("PHY1", 0.003),
        ("PVY1", 0.02),
        ("PVY3", -0.2),
        ("PPY1", -0.5),
        ("RBY1", 11.0),
        ("RBY2", 8.0),
        ("RCY1", 1.0),
        ("RVY5", 1.9),
        ("RVY6", -10.0),
        ("QBZ1", 8.0),
        ("QCZ1", 1.1),
        ("QDZ1", 0.09),
        ("QDZ6", 0.002),
        ("QEZ1", -1.0),
        ("QHZ1", 0.002),
        ("QBZ9", 15.0),
        ("SSZ1", 0.02),
        ("QSX1", 0.005),
        ("QSX2", 1.0),
        ("QSX3", 0.05),
        ("QSY1", 0.01),
        ("QSY7", 0.85),
    ];
    pairs.iter().map(|(k, v)| ((*k).to_owned(), *v)).collect()
}

/// All hot paths share ONE dhat profiler: the testing profiler is process-global, so separate
/// `#[test]`s would race under the parallel test runner. Each path is measured in its own
/// before/after window inside the single profiler.
#[test]
#[allow(clippy::too_many_lines)] // one profiler window over several hot paths; splitting races it.
fn hot_paths_do_not_allocate() {
    let _profiler = dhat::Profiler::builder().testing().build();

    // --- MF6.1 forces ---
    let (p, _notes) = Mf61Params::<f64>::from_coeffs(&full_map()).unwrap();
    let mf61 = Mf61::new(p);
    let mut sink = mf61
        .forces(&SlipState::new(0.05, -0.03, 0.01, 4200.0, 210_000.0, 40.0))
        .fx;
    let before = dhat::HeapStats::get().total_blocks;
    for i in 0..16 {
        #[allow(clippy::cast_precision_loss)]
        let t = f64::from(i) / 16.0;
        let s = SlipState::new(
            -0.2 + 0.4 * t,
            0.15 - 0.3 * t,
            0.02 * t,
            3000.0 + 2000.0 * t,
            200_000.0 + 40_000.0 * t,
            5.0 + 60.0 * t,
        );
        let f = mf61.forces(&s);
        sink += f.fx + f.fy + f.mz + f.mx + f.my;
    }
    assert_eq!(
        before,
        dhat::HeapStats::get().total_blocks,
        "Mf61::forces allocated on the heap"
    );

    // --- Brush forces ---
    let brush = Brush::<f64>::new(1.5e5, 1.2e5, 1.3, 0.10);
    sink += brush
        .forces(&SlipState::new(0.05, -0.03, 0.01, 4200.0, 210_000.0, 40.0))
        .fx;
    let before = dhat::HeapStats::get().total_blocks;
    for i in 0..16 {
        #[allow(clippy::cast_precision_loss)]
        let t = f64::from(i) / 16.0;
        let s = SlipState::new(
            -0.2 + 0.4 * t,
            0.15 - 0.3 * t,
            0.02 * t,
            3000.0 + 2000.0 * t,
            210_000.0,
            5.0 + 60.0 * t,
        );
        let f = brush.forces(&s);
        sink += f.fx + f.fy + f.mz;
    }
    assert_eq!(
        before,
        dhat::HeapStats::get().total_blocks,
        "Brush::forces allocated on the heap"
    );

    // --- Relaxation: sigma queries + relax_step ---
    let relax_map: BTreeMap<String, f64> = [
        ("FNOMIN", 4000.0),
        ("UNLOADED_RADIUS", 0.33),
        ("PTX1", 2.3),
        ("PTX2", 1.9),
        ("PTX3", 0.24),
        ("PTY1", 2.1),
        ("PTY2", 2.0),
    ]
    .iter()
    .map(|(k, v)| ((*k).to_owned(), *v))
    .collect();
    let (rparams, _) = Mf61Params::<f64>::from_coeffs(&relax_map).unwrap();
    let (relax, _) = Relaxation::from_params(&rparams);
    let mut x = relax_step(0.0, 0.1, 30.0, 1e-3, relax.sigma_kappa(4000.0));
    let before = dhat::HeapStats::get().total_blocks;
    for i in 0..16 {
        #[allow(clippy::cast_precision_loss)]
        let t = f64::from(i) / 16.0;
        let fz = 3000.0 + 2000.0 * t;
        let sigma = relax.sigma_alpha(fz, 0.02 * t);
        x = relax_step(x, 0.1, 30.0, 1e-3, sigma);
    }
    assert_eq!(
        before,
        dhat::HeapStats::get().total_blocks,
        "relax_step / sigma queries allocated on the heap"
    );

    // --- Tire thermal ring: step + couplings ---
    let thermal = TyrThermal {
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
    };
    let ring = TireThermalRing::<f64>::from_schema(&thermal);
    let mut state = TireThermalState::uniform(293.15);
    let mut drv = ThermalDrivers {
        slip_power_w: 8000.0,
        carcass_loss_w: 1400.0,
        speed_mps: 40.0,
        contact_fraction: 0.05,
        ext_area_m2: 0.4,
        t_air_k: 303.15,
        t_road_k: 318.15,
    };
    let before = dhat::HeapStats::get().total_blocks;
    for i in 0..16 {
        #[allow(clippy::cast_precision_loss)]
        let frac = f64::from(i) / 16.0;
        drv.speed_mps = 5.0 + 60.0 * frac;
        drv.slip_power_w = 2000.0 + 20_000.0 * frac;
        let cpl = ring.step(&mut state, &drv, 0.05);
        sink += cpl.pressure_pa + cpl.mu_scale + cpl.stiffness_scale;
    }
    assert_eq!(
        before,
        dhat::HeapStats::get().total_blocks,
        "TireThermalRing::step allocated on the heap"
    );

    // --- Tire wear/damage ring: step advancing temps + wear + damage together ---
    let wear = TyrWear {
        k_w: 1.0e-8,
        w_max: 8.0,
        w_c: 3.0,
        tau_d: 400.0,
        t_deg: 110.0,
        delta_t_ref: 20.0,
        beta: 2.0,
        delta_c: 0.25,
        s_w: 0.5,
        delta_d: 0.25,
    };
    let wear_ring = TireThermalRing::<f64>::from_schema_with_wear(&thermal, &wear);
    let mut wear_state = TireThermalState::uniform(363.15);
    let before = dhat::HeapStats::get().total_blocks;
    for i in 0..16 {
        #[allow(clippy::cast_precision_loss)]
        let frac = f64::from(i) / 16.0;
        drv.speed_mps = 5.0 + 60.0 * frac;
        drv.slip_power_w = 8000.0 + 14_000.0 * frac;
        let cpl = wear_ring.step(&mut wear_state, &drv, 0.05);
        sink += cpl.mu_scale_total + cpl.wear_grip_scale + cpl.damage_grip_scale;
    }
    assert_eq!(
        before,
        dhat::HeapStats::get().total_blocks,
        "wear-capable TireThermalRing::step allocated on the heap"
    );

    assert!(sink.is_finite() && x.is_finite() && wear_state.wear_mm.is_finite());
}
