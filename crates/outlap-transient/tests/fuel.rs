// SPDX-License-Identifier: AGPL-3.0-only
//! M6 PR5 — the T2 fuel mass/CG fan-out (`T2Blocks::apply_mass_state`, D-M6-4c). The conservation
//! property test the plan calls out (PR5e): after a slow-clock mass/CG update EVERY block-resident
//! copy of the mass and wheel geometry agrees, the static normal loads sum to `m·g`, and the
//! front/rear split balances the pitch moment about the NEW CG — the silent-split failure mode no
//! lap-time gate would catch.
#![allow(clippy::float_cmp)]

mod common;

use common::{build_blocks, limebeer};
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
