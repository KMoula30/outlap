// SPDX-License-Identifier: AGPL-3.0-only
//! M6 PR5 — the T2 fuel mass/CG fan-out (`T2Blocks::apply_mass_state`, D-M6-4c). The conservation
//! property test the plan calls out (PR5e): after a slow-clock mass/CG update EVERY block-resident
//! copy of the mass and wheel geometry agrees, the static normal loads sum to `m·g`, and the
//! front/rear split balances the pitch moment about the NEW CG — the silent-split failure mode no
//! lap-time gate would catch.
#![allow(clippy::float_cmp)]

mod common;

use common::{build_blocks, limebeer, line};
use outlap_core::bus::ChannelInterner;
use outlap_qss::t1::load_transfer;

const WHEELS: usize = 4;
const G: f64 = 9.81;

/// After `apply_mass_state`, the load geometry, the chassis inertia block, and the tyre block all
/// carry the SAME mass and wheel-x geometry; the static `F_z` sums to `m·g` and the front axle
/// carries exactly `b_r/L` of it (pitch balance about the migrated CG).
#[test]
fn apply_mass_state_conserves_load_and_balances_pitch() {
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let mut blocks = build_blocks(&t1, &spec, &mut it);

    // Burn to a lighter mass with the CG migrated rearward (a_f grows) and lowered.
    let wheelbase = blocks.load.geom.wheelbase_m;
    let a_f0 = blocks.load.geom.a_f;
    let new_mass = blocks.load.geom.mass_kg - 40.0;
    let new_a_f = a_f0 + 0.05;
    let new_h_cg = blocks.load.geom.h_cg - 0.01;
    blocks.apply_mass_state(new_mass, new_a_f, new_h_cg);

    // (1) Every block-visible copy agrees — no copy missed.
    let g = blocks.load.geom;
    assert_eq!(g.mass_kg, new_mass);
    assert_eq!(g.a_f, new_a_f);
    assert_eq!(g.b_r, wheelbase - new_a_f);
    assert_eq!(g.h_cg, new_h_cg);
    assert_eq!(blocks.chassis.params.mass, new_mass);
    // h_ra is recomputed from the migrated a_f (mirrors T1Vehicle::with_cg).
    let want_h_ra = g.rc_f + (g.rc_r - g.rc_f) * (new_a_f / wheelbase);
    assert!((g.h_ra - want_h_ra).abs() < 1e-12);
    // Both WheelGeometry copies (chassis inertia + tyre slip) carry the same x rel the moved CG.
    for i in 0..WHEELS {
        let want_x = if blocks.chassis.params.wheels.front[i] {
            new_a_f
        } else {
            -(wheelbase - new_a_f)
        };
        assert!((blocks.chassis.params.wheels.x[i] - want_x).abs() < 1e-12);
        assert_eq!(
            blocks.chassis.params.wheels.x[i], blocks.tire.wheels.x[i],
            "chassis and tyre wheel-x copies diverged at wheel {i}"
        );
    }

    // (2) Static normal-load conservation: at rest (v = 0, a_x = a_y = 0) ΣF_z == m·g, and the front
    // axle carries b_r/L of it (moment balance about the new CG). qz_f/qz_r vanish at v = 0.
    let fz = load_transfer(&g, 0.0, G, 0.0, 0.0, blocks.load.qz_f, blocks.load.qz_r);
    let sum: f64 = fz.iter().sum();
    assert!(
        (sum - new_mass * G).abs() < 1e-6,
        "ΣF_z {sum} must equal m·g {}",
        new_mass * G
    );
    let front: f64 = (0..WHEELS)
        .filter(|&i| blocks.chassis.params.wheels.front[i])
        .map(|i| fz[i])
        .sum();
    let want_front = new_mass * G * (g.b_r / wheelbase);
    assert!(
        (front - want_front).abs() < 1e-6,
        "front-axle load {front} must balance the pitch moment about the new CG ({want_front})"
    );
}

use outlap_schema::sim::FzCoupling;
use outlap_transient::{FuelSlow, SimConfig, TransientSolver};

/// A T2 lap with a fuel slow state burns fuel and updates the block mass: on a straight the ICE
/// covers the drive, the tank drains, and on the slow clock `apply_mass_state` fans the lighter mass
/// out to the blocks — the live D-M6-4 slow-state path in the transient tier.
#[test]
fn t2_lap_burns_fuel_and_lightens_the_blocks() {
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let full_mass = blocks.load.geom.mass_kg;
    let a_f0 = blocks.load.geom.a_f;
    let h_cg0 = blocks.load.geom.h_cg;
    let initial_fuel = 30.0_f64;
    // Dry mass so the full-tank state matches the assembled blocks (no jump at the first fire).
    let fuel = FuelSlow {
        dry_mass_kg: full_mass - initial_fuel,
        tank_kg: 60.0,
        a_f_dry: a_f0,
        h_cg_dry: h_cg0,
        a_f_tank: a_f0 - 0.20, // tank 0.20 m rearward of the dry CG
        h_cg_tank: h_cg0 - 0.05,
        lhv_j_per_kg: 43.0e6,
        ice_thermal_eff: 0.33,
        fuel_kg: 0.0,
        burn_accum_kg: 0.0,
    };
    let cfg = SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(
        blocks,
        line(6000.0, 300, false, 0.0, 0.0, 70.0, None),
        &it,
        cfg,
    )
    .with_fuel(fuel, initial_fuel);
    // Accelerate from the seeded speed toward v_ref = 70 m/s on a long straight: throttle > 0 ⇒ burn.
    for _ in 0..4000 {
        solver.step();
    }
    let remaining = solver.fuel_remaining_kg().expect("fuel state present");
    assert!(
        remaining < initial_fuel && remaining > 0.0,
        "fuel burned over the lap ({remaining} kg < {initial_fuel} kg, tank not dry)"
    );
    // The slow clock fanned the lighter mass out to the blocks (apply_mass_state fired).
    let mass_now = solver.blocks().load.geom.mass_kg;
    assert!(
        mass_now < full_mass,
        "the blocks got lighter as fuel burned ({mass_now} < {full_mass})"
    );
    // Mass consistency after the live burn: chassis inertia block agrees with the load geometry.
    assert!((solver.blocks().chassis.params.mass - mass_now).abs() < 1e-9);
}
