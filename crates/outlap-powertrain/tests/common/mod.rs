// SPDX-License-Identifier: AGPL-3.0-only
//! Shared fixtures for the outlap-powertrain tests: the D-M6-5 f1 2026 `ers:` block (the verified
//! FIA Issue-19 figures), the gt_hybrid fixture block loaded through the real pipeline
//! (D-M6-12), and a deterministic counter-based test RNG.
#![allow(dead_code)] // each test binary uses a different subset of these helpers
#![allow(clippy::doc_markdown, clippy::cast_precision_loss)] // fixture names; 53-bit RNG mantissa

use outlap_schema::io::FsLoader;
use outlap_schema::vehicle::{
    Activation, Deployment, EnergyStore, Ers, OverrideMode, Recovery, SpeedTaper,
};
use outlap_schema::{load_vehicle, LoadOptions};

/// The verified FIA 2026 Issue-19 `ers:` block (D-M6-5): two-segment deployment taper with the
/// knee at EXACTLY 100/350 = 2/7 (a truncated decimal fails the knee-exactness test), override
/// per C5.2.8(ii), 8.5 MJ harvest + 0.5 MJ override bonus, NO per-lap deploy budget.
pub fn f1_ers() -> Ers {
    Ers {
        mgu_k: "ptm/mgu_k.ptm.yaml".into(),
        es: EnergyStore {
            capacity_mj: 4.0,
            soc_window: [0.2, 0.9],
        },
        deployment: Deployment {
            power_limit_kw: 350.0,
            taper_vs_speed: SpeedTaper {
                speed_kph: vec![0.0, 290.0, 340.0, 345.0],
                power_frac: vec![1.0, 1.0, 2.0 / 7.0, 0.0],
            },
            per_lap_deploy_mj: None,
        },
        override_mode: Some(OverrideMode {
            power_limit_kw: 350.0,
            taper_vs_speed: SpeedTaper {
                speed_kph: vec![0.0, 337.5, 355.0],
                power_frac: vec![1.0, 1.0, 0.0],
            },
            extra_energy_per_lap_mj: Some(0.5),
            activation: Activation::Strategy,
        }),
        recovery: Recovery {
            braking_power_limit_kw: 350.0,
            per_lap_harvest_mj: 8.5,
            recharge_phases: true,
            recharge_target_soc: None,
            ramp_initial_step_kw: None,
            ramp_rate_kw_per_s: None,
            ramp_total_kw: None,
        },
        elec_mech_factor: None,
    }
}

/// The gt_hybrid fixture's `ers:` block, loaded through the real schema pipeline (D-M6-12): no
/// override mode, no recharge phases, a decreasing mid-knot taper, 120 kW / 3 MJ budgets — the
/// Option-handling paths must never ship untested.
pub fn gt_ers() -> Ers {
    let loader = FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ));
    let resolved = load_vehicle("gt_hybrid/vehicle.yaml", &loader, &LoadOptions::default())
        .expect("gt_hybrid fixture resolves");
    resolved.spec.ers.expect("gt_hybrid has an ers block")
}

/// A deterministic counter-based test RNG (splitmix64) — fixed seeds, no clock, no rand dep.
pub struct TestRng(u64);

impl TestRng {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in `[lo, hi)`.
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }
}
