// SPDX-License-Identifier: AGPL-3.0-only
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_lossless
)]
//! Open-loop steady-state cornering probe: Limebeer car, straight synthetic road, constant steer
//! feed-forward (`kappa_ref`), speed held by the ideal driver's PI loop with the path feedback off.
//! Prints the yaw-rate / velocity transient, which should approach a bounded steady yaw `≈ v·κ` with
//! the correct sign — the sanity check behind the step-steer property test.
//!
//! ```text
//! cargo run --release -p outlap-transient --example diag -- <speed m/s> <kappa_ref 1/m>
//! ```

#[path = "../tests/common/mod.rs"]
mod common;

use outlap_core::bus::ChannelInterner;
use outlap_schema::sim::FzCoupling;
use outlap_transient::{SimConfig, TransientSolver};

fn main() {
    let target_v: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(40.0);
    let kappa_ref: f64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);

    let (t1, spec) = common::limebeer();
    let mut interner = ChannelInterner::new();
    let mut blocks = common::build_blocks(&t1, &spec, &mut interner);
    // Path feedback off → constant steer = the curvature feed-forward; PI holds v_ref.
    blocks.driver.preview_gain = 0.0;
    blocks.driver.heading_gain = 0.0;
    blocks.driver.yaw_damping = 0.0;

    let line = common::line(4000.0, 200, false, 0.0, kappa_ref, target_v, None);
    let cfg = SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(blocks, line, &interner, cfg);
    println!(
        "target_v={target_v} kappa_ref={kappa_ref} steer_ff={:.4}",
        t1.wheelbase_m * kappa_ref
    );
    for step in 0..2000 {
        solver.step();
        if step % 100 == 99 {
            let fs = solver.fast_state();
            println!(
                "t={:.3} vx={:.3} vy={:.4} r={:.5} n={:.4}",
                (step + 1) as f64 * 0.001,
                fs[3],
                fs[4],
                fs[5],
                fs[1]
            );
        }
    }
}
