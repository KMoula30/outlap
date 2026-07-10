// SPDX-License-Identifier: AGPL-3.0-only
//! Property tests for the PR6 torque-vectoring allocator ([`allocate_yaw_moment`], HANDOFF §8.0).
//!
//! The allocator's contract is four invariants, each checked here over randomised *physically
//! consistent* wheel states (a lateral force inside the friction circle, a longitudinal force inside
//! the remaining ellipse headroom):
//!
//! 1. **Friction-ellipse containment** — no wheel is pushed past `f_x,max = √((μ·F_z)² − F_y²)`.
//! 2. **Sign convention** — the realised moment never opposes the demand, and never exceeds it
//!    (ISO 8855: `M_z` is +CCW, a force `Δf_x` at lateral arm `y_i` contributes `−y_i·Δf_x`).
//! 3. **Moment exactness** — the per-wheel deltas realise *exactly* the reported moment, i.e. the
//!    allocation is a genuine per-wheel split and never a lumped couple injected on the side.
//! 4. **Drive capability** — a wheel with no machine on it may only ever brake.
//!
//! Energy closure for the regen blend needs a populated bus, so it lives with the block-level
//! integration tests (`outlap-transient/tests/control.rs`).
#![allow(clippy::cast_precision_loss)] // small loop counters → f64 grid coordinates.
#![allow(clippy::float_cmp)] // the no-op cases assert an *exact* zero; a tolerance would void them.

use outlap_core::bus::WHEELS;
use outlap_vehicle::allocate_yaw_moment;
use proptest::prelude::*;

/// Longitudinal grip headroom radius `f_x,max = √((μ·F_z)² − F_y²)`, 0 at the lateral limit.
fn fx_max(mu: f64, fz: f64, fy: f64) -> f64 {
    ((mu * fz) * (mu * fz) - fy * fy).max(0.0).sqrt()
}

/// A randomised, physically consistent allocator input.
#[derive(Clone, Debug)]
struct Case {
    mu: f64,
    fz: [f64; WHEELS],
    fy: [f64; WHEELS],
    fx: [f64; WHEELS],
    y: [f64; WHEELS],
    demand: f64,
    drive_capable: [bool; WHEELS],
}

prop_compose! {
    fn case()(
        mu in 0.6f64..2.0,
        fz in prop::array::uniform4(500.0f64..8000.0),
        // Strictly inside the circle/ellipse: a wheel exactly at its limit has zero headroom, which
        // the dedicated saturation test covers exactly rather than approximately.
        fy_frac in prop::array::uniform4(-0.98f64..0.98),
        fx_frac in prop::array::uniform4(-0.98f64..0.98),
        half_track in 0.6f64..1.0,
        demand in -6000.0f64..6000.0,
        drive_capable in prop::array::uniform4(any::<bool>()),
    ) -> Case {
        let mut fy = [0.0; WHEELS];
        let mut fx = [0.0; WHEELS];
        for i in 0..WHEELS {
            fy[i] = fy_frac[i] * mu * fz[i];
            fx[i] = fx_frac[i] * fx_max(mu, fz[i], fy[i]);
        }
        // [FL, FR, RL, RR], +y to the left (ISO 8855).
        let y = [half_track, -half_track, half_track, -half_track];
        Case { mu, fz, fy, fx, y, demand, drive_capable }
    }
}

/// Absolute tolerance scaled to a magnitude — the allocator is a few multiplies deep, so a relative
/// 1e-9 is a tight bound, not a fudge.
fn tol(magnitude: f64) -> f64 {
    1e-9 * magnitude.abs().max(1.0)
}

proptest! {
    /// Invariant 1: every wheel stays inside its friction ellipse after the delta is applied.
    #[test]
    fn allocation_never_leaves_the_friction_ellipse(c in case()) {
        let alloc = allocate_yaw_moment(
            c.demand, &c.y, &c.fx, &c.fy, &c.fz, c.mu, &c.drive_capable,
        );
        for i in 0..WHEELS {
            let limit = fx_max(c.mu, c.fz[i], c.fy[i]);
            let after = c.fx[i] + alloc.delta_fx[i];
            prop_assert!(
                after.abs() <= limit + tol(limit),
                "wheel {i}: |fx {} + Δ {}| = {} exceeds f_x,max {limit}",
                c.fx[i], alloc.delta_fx[i], after.abs(),
            );
        }
    }

    /// Invariant 2: the realised moment never opposes the demand and never overshoots it.
    #[test]
    fn realised_moment_agrees_in_sign_and_never_exceeds_the_demand(c in case()) {
        let alloc = allocate_yaw_moment(
            c.demand, &c.y, &c.fx, &c.fy, &c.fz, c.mu, &c.drive_capable,
        );
        prop_assert!(
            alloc.moment_nm * c.demand >= -tol(c.demand),
            "realised {} opposes demand {}", alloc.moment_nm, c.demand,
        );
        prop_assert!(
            alloc.moment_nm.abs() <= c.demand.abs() + tol(c.demand),
            "realised {} overshoots demand {}", alloc.moment_nm, c.demand,
        );
    }

    /// Invariant 3: the deltas realise exactly the reported moment, `Σ −y_i·Δf_x,i = ΔM_z`. This is
    /// what separates a per-wheel split from a lumped couple: the telemetry channel cannot claim a
    /// moment the tyres were never asked to produce.
    #[test]
    fn reported_moment_equals_the_moment_the_deltas_produce(c in case()) {
        let alloc = allocate_yaw_moment(
            c.demand, &c.y, &c.fx, &c.fy, &c.fz, c.mu, &c.drive_capable,
        );
        let mut realised = 0.0;
        for i in 0..WHEELS {
            realised += -c.y[i] * alloc.delta_fx[i];
        }
        prop_assert!(
            (realised - alloc.moment_nm).abs() <= tol(alloc.moment_nm),
            "deltas produce {realised} but the block reports {}", alloc.moment_nm,
        );
    }

    /// Invariant 4: a wheel with no machine on it can only ever brake (non-positive delta).
    #[test]
    fn drive_incapable_wheels_only_brake(c in case()) {
        let alloc = allocate_yaw_moment(
            c.demand, &c.y, &c.fx, &c.fy, &c.fz, c.mu, &c.drive_capable,
        );
        for i in 0..WHEELS {
            prop_assert!(
                c.drive_capable[i] || alloc.delta_fx[i] <= tol(c.fz[i]),
                "wheel {i} is drive-incapable but got a drive delta {}", alloc.delta_fx[i],
            );
        }
    }
}

/// A zero demand must be a byte-exact no-op: a car that never asks for yaw moment is identical to
/// one with the allocator absent (the `enabled == false` byte-identity claim on [`TorqueVectoring`]).
#[test]
fn zero_demand_allocates_nothing() {
    let fz = [4000.0; WHEELS];
    let alloc = allocate_yaw_moment(
        0.0,
        &[0.8, -0.8, 0.8, -0.8],
        &[0.0; WHEELS],
        &[0.0; WHEELS],
        &fz,
        1.5,
        &[true; WHEELS],
    );
    assert_eq!(alloc.delta_fx, [0.0; WHEELS], "zero demand moved a wheel");
    assert_eq!(alloc.moment_nm, 0.0, "zero demand realised a moment");
}

/// Wheels saturated laterally (`|F_y| = μ·F_z`) have zero longitudinal headroom, so no moment is
/// feasible however large the demand — the allocator must report the truth (0), not the demand.
#[test]
fn lateral_saturation_leaves_no_headroom() {
    let mu = 1.4;
    let fz = [4000.0; WHEELS];
    let fy = [mu * 4000.0, -mu * 4000.0, mu * 4000.0, -mu * 4000.0];
    let alloc = allocate_yaw_moment(
        5000.0,
        &[0.8, -0.8, 0.8, -0.8],
        &[0.0; WHEELS],
        &fy,
        &fz,
        mu,
        &[true; WHEELS],
    );
    assert_eq!(
        alloc.delta_fx, [0.0; WHEELS],
        "no headroom but wheels moved"
    );
    assert_eq!(alloc.moment_nm, 0.0, "no headroom but a moment was claimed");
}
