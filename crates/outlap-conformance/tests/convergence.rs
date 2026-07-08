// SPDX-License-Identifier: AGPL-3.0-only
//! Convergence + determinism gate for the fixed-step RK stepper (HANDOFF §11.2).
//!
//! The production stepper must converge to a trusted reference at its formal order. We integrate a
//! stiff, nonlinear scalar ODE with the production [`SimArena`] stepper and compare against a
//! `diffsol` **BDF** solution (the CI verification integrator, run to a tight tolerance):
//!
//! * Heun (RK2) shows **O(dt²)** global error — the halving-dt error ratio is ≈ 4;
//! * RK4 is dramatically more accurate at the same step (selectable for convergence studies);
//! * two identical runs are **bit-for-bit** equal (determinism).

// Step counts derive from positive, integral (T/dt) ratios — the truncation/sign casts are safe.
#![allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]

use diffsol::{NalgebraLU, NalgebraMat, OdeBuilder, OdeSolverMethod};
use outlap_core::{RkMethod, SimArena};

/// Stiffness of the test ODE (`|∂f/∂y| ≈ K`); large enough to be a genuine stiff problem for the
/// implicit reference, small enough that explicit Heun stays stable at the tested steps.
const K: f64 = 50.0;
/// Integration horizon, s.
const T_FINAL: f64 = 1.0;

/// The test ODE: `y' = −K·y + K·cos(t) − y²`, `y(0) = 0`. Nonlinear (the `−y²` term) and stiff.
fn rhs(y: f64, t: f64) -> f64 {
    -K * y + K * t.cos() - y * y
}

/// Integrate to `T_FINAL` with the production stepper `method` at fixed step `dt`; return `y(T)`.
fn integrate(method: RkMethod, dt: f64) -> f64 {
    let mut arena = SimArena::for_method(method, 1);
    let mut x = [0.0f64];
    let steps = (T_FINAL / dt).round() as usize;
    let mut t = 0.0;
    for _ in 0..steps {
        arena.step(&mut x, t, dt, |ti, xs, dx| dx[0] = rhs(xs[0], ti));
        t += dt;
    }
    x[0]
}

/// The trusted `y(T_FINAL)` from a tight `diffsol` BDF solve (the CI reference integrator).
fn reference() -> f64 {
    type M = NalgebraMat<f64>;
    type Ls = NalgebraLU<f64>;
    let problem = OdeBuilder::<M>::new()
        .rtol(1e-10)
        .atol([1e-12])
        .p([K])
        .rhs_implicit(
            |x, p, t, y| y[0] = -p[0] * x[0] + p[0] * t.cos() - x[0] * x[0],
            |x, p, _t, v, y| y[0] = (-p[0] - 2.0 * x[0]) * v[0],
        )
        .init(|_p, _t, y| y[0] = 0.0, 1)
        .build()
        .expect("diffsol problem builds");
    let mut solver = problem.bdf::<Ls>().expect("bdf solver");
    while solver.state().t < T_FINAL {
        solver.step().expect("bdf step");
    }
    solver.interpolate(T_FINAL).expect("interpolate")[0]
}

#[test]
fn heun_is_second_order_vs_diffsol_bdf() {
    let y_ref = reference();
    let dts = [4e-3, 2e-3, 1e-3];
    let errs: Vec<f64> = dts
        .iter()
        .map(|&dt| (integrate(RkMethod::Heun, dt) - y_ref).abs())
        .collect();

    // Errors shrink monotonically and stay well above f64 round-off (a clean order window).
    assert!(errs[0] > errs[1] && errs[1] > errs[2], "errors: {errs:?}");
    assert!(errs[2] > 1e-12, "error saturated at round-off: {errs:?}");

    // Observed order between successive halvings ≈ 2.
    for w in errs.windows(2) {
        let order = (w[0] / w[1]).log2();
        assert!(
            (1.8..=2.2).contains(&order),
            "observed Heun order {order} outside [1.8, 2.2] (errs {errs:?})"
        );
    }
}

#[test]
fn rk4_beats_heun_at_equal_step() {
    let y_ref = reference();
    let dt = 4e-3;
    let e_heun = (integrate(RkMethod::Heun, dt) - y_ref).abs();
    let e_rk4 = (integrate(RkMethod::Rk4, dt) - y_ref).abs();
    // RK4 is orders of magnitude tighter than Heun on the same smooth problem.
    assert!(
        e_rk4 * 100.0 < e_heun,
        "RK4 ({e_rk4:e}) not >> Heun ({e_heun:e})"
    );
}

#[test]
fn stepping_is_bit_deterministic() {
    let a = integrate(RkMethod::Heun, 1e-3);
    let b = integrate(RkMethod::Heun, 1e-3);
    assert_eq!(a.to_bits(), b.to_bits(), "identical runs diverged");
}
