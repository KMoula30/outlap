// SPDX-License-Identifier: AGPL-3.0-only
//! Property tests for the T1 quasi-steady-state trim solver (PR2).
//!
//! Sign conventions (ISO 8855): a left corner ⇒ `a_y > 0`, positive steer `δ > 0`, positive yaw
//! rate `r > 0`, and lateral load transfer to the outside (right) wheels. Physics invariants:
//! per-wheel friction-circle containment, ΣF_z = weight + downforce, left/right symmetry for a
//! symmetric car at ±a_y, and Newton convergence over a dense feasible (v, a_y, a_x) grid.
#![allow(clippy::doc_markdown)] // physics symbols (a_y, F_z, …) read better without backticks here

use outlap_qss::{T1Vehicle, TrimInput, TrimOutcome};
use outlap_schema::io::FsLoader;
use outlap_schema::sim::FzCoupling;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

fn assemble(name: &str) -> T1Vehicle {
    let loader = fixtures();
    let rv = load_vehicle(name, &loader, &LoadOptions::default()).unwrap();
    T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap()
}

fn state(outcome: &TrimOutcome) -> outlap_qss::TrimState {
    *outcome.state().expect("trim should be feasible")
}

#[test]
fn assembles_and_trims_f1() {
    let car = assemble("f1_2026/vehicle.yaml");
    let out = car.trim(&TrimInput::flat(60.0, 12.0, 0.0));
    let s = state(&out);
    assert!(s.residual_norm < 1e-9, "residual {}", s.residual_norm);
    // A left corner: steer left (δ>0), yaw left (r>0).
    assert!(
        s.delta > 0.0,
        "left corner should steer left, δ={}",
        s.delta
    );
    assert!(
        s.yaw_rate > 0.0,
        "left corner should yaw left, r={}",
        s.yaw_rate
    );
}

#[test]
fn sum_fz_equals_weight_plus_downforce() {
    let car = assemble("f1_2026/vehicle.yaml");
    let g = outlap_qss::G;
    for &(v, ay, ax) in &[(30.0, 0.0, 0.0), (60.0, 15.0, 0.0), (50.0, 8.0, -6.0)] {
        let s = state(&car.trim(&TrimInput::flat(v, ay, ax)));
        let sum_fz: f64 = s.fz.iter().sum();
        let weight = car.mass_kg * g;
        let downforce = (car.qz_f + car.qz_r) * v * v;
        let expected = weight + downforce;
        assert!(
            (sum_fz - expected).abs() < 1e-6 * expected,
            "ΣFz {sum_fz} vs weight+downforce {expected} at (v={v}, ay={ay}, ax={ax})"
        );
    }
}

#[test]
fn lateral_load_transfer_to_outside_wheels() {
    let car = assemble("f1_2026/vehicle.yaml");
    // Left corner (ay>0): outside = right (FR, RR) must carry more than inside (FL, RL).
    let s = state(&car.trim(&TrimInput::flat(55.0, 14.0, 0.0)));
    assert!(
        s.fz[1] > s.fz[0],
        "FR ({}) should exceed FL ({})",
        s.fz[1],
        s.fz[0]
    );
    assert!(
        s.fz[3] > s.fz[2],
        "RR ({}) should exceed RL ({})",
        s.fz[3],
        s.fz[2]
    );
}

#[test]
fn longitudinal_load_transfer_under_braking_and_accel() {
    let car = assemble("f1_2026/vehicle.yaml");
    // Braking (ax<0): weight to the front.
    let brake = state(&car.trim(&TrimInput::flat(50.0, 0.0, -8.0)));
    let front_b = brake.fz[0] + brake.fz[1];
    let rear_b = brake.fz[2] + brake.fz[3];
    // Acceleration (ax>0): weight to the rear.
    let accel = state(&car.trim(&TrimInput::flat(50.0, 0.0, 5.0)));
    let front_a = accel.fz[0] + accel.fz[1];
    let rear_a = accel.fz[2] + accel.fz[3];
    assert!(
        front_b > front_a,
        "braking should load the front more than accelerating"
    );
    assert!(
        rear_a > rear_b,
        "accelerating should load the rear more than braking"
    );
}

#[test]
fn friction_circle_containment_per_wheel() {
    let car = assemble("f1_2026/vehicle.yaml");
    // μ from the tyre peak at the nominal load (a loose upper bound is fine for containment).
    let mu = 1.9; // slick peak is < ~1.9; the true grip circle is tighter than μ·Fz
    for &(v, ay, ax) in &[(60.0, 18.0, 0.0), (45.0, 10.0, -8.0), (70.0, 8.0, 6.0)] {
        let s = state(&car.trim(&TrimInput::flat(v, ay, ax)));
        for i in 0..4 {
            let f_horiz = (s.fx[i] * s.fx[i] + s.fy[i] * s.fy[i]).sqrt();
            let cap = mu * s.fz[i].max(0.0);
            assert!(
                f_horiz <= cap + 1.0,
                "wheel {i} horizontal force {f_horiz} exceeds μ·Fz {cap} at (v={v},ay={ay},ax={ax})"
            );
        }
    }
}

#[test]
fn left_right_symmetry_for_symmetric_car() {
    // Use a symmetric car (equal front/rear track is not required; L/R mirror is what matters).
    let car = assemble("f1_2026/vehicle.yaml");
    let v = 55.0;
    let ay = 12.0;
    let left = state(&car.trim(&TrimInput::flat(v, ay, 0.0)));
    let right = state(&car.trim(&TrimInput::flat(v, -ay, 0.0)));
    // Steer and yaw rate flip sign; magnitudes match.
    assert!(
        (left.delta + right.delta).abs() < 1e-6,
        "δ should be antisymmetric in ay"
    );
    assert!(
        (left.yaw_rate + right.yaw_rate).abs() < 1e-9,
        "r should be antisymmetric in ay"
    );
    // Loads mirror: FL(+ay) == FR(−ay), RL(+ay) == RR(−ay).
    assert!(
        (left.fz[0] - right.fz[1]).abs() < 1e-6,
        "FL(+ay) vs FR(−ay)"
    );
    assert!(
        (left.fz[1] - right.fz[0]).abs() < 1e-6,
        "FR(+ay) vs FL(−ay)"
    );
    assert!(
        (left.fz[2] - right.fz[3]).abs() < 1e-6,
        "RL(+ay) vs RR(−ay)"
    );
    assert!(
        (left.fz[3] - right.fz[2]).abs() < 1e-6,
        "RR(+ay) vs RL(−ay)"
    );
}

#[test]
fn newton_converges_over_modest_feasible_grid() {
    // Modest operating points inside the friction envelope of both reference cars (an F1 with
    // downforce and a FWD road car with slight lift), spanning low race-corner speeds (8 m/s ≈
    // 29 km/h) up to 60 m/s — the homotopy continuation keeps every one converging. Every one trims.
    for name in ["f1_2026/vehicle.yaml", "fwd_hatch/vehicle.yaml"] {
        let car = assemble(name);
        for vi in 0..7 {
            let v = 8.0 + 8.0 * f64::from(vi); // 8 … 56 m/s
            for ai in 0..7 {
                let ay = -6.0 + 2.0 * f64::from(ai); // −6 … 6 m/s²
                for xi in 0..5 {
                    let ax = -3.0 + 1.5 * f64::from(xi); // −3 … 3 m/s²
                    match car.trim(&TrimInput::flat(v, ay, ax)) {
                        TrimOutcome::Converged(s) => assert!(s.residual_norm <= 1e-10),
                        TrimOutcome::Infeasible { residual_norm, .. } => {
                            panic!("{name}: (v={v}, ay={ay}, ax={ax}) should trim (rn={residual_norm:.2e})")
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn low_speed_tight_corners_converge_via_continuation() {
    // Regression: a plain Newton stalled at tight low-speed geometry; homotopy continuation trims
    // it. These are feasible hairpin-scale corners (down to ~6 m radius) at 8–15 m/s.
    let car = assemble("f1_2026/vehicle.yaml");
    for &(v, ay) in &[(8.0, 9.0), (10.0, 10.0), (12.0, 10.0), (15.0, 12.0)] {
        let out = car.trim(&TrimInput::flat(v, ay, 0.0));
        let s = out
            .state()
            .unwrap_or_else(|| panic!("(v={v}, ay={ay}) should trim"));
        assert!(s.residual_norm <= 1e-10);
        // Physical: converged loads are non-negative and steer is within the road-wheel range.
        assert!(s.fz.iter().all(|&f| f >= 0.0));
        assert!(s.delta.abs() < 0.75);
    }
}

#[test]
fn solver_is_robust_and_flags_infeasibility() {
    // Over an aggressive grid (including 90 m/s + hard accel that a FWD road car cannot deliver),
    // every outcome is clean: converged points hit tolerance, infeasible points return a finite
    // residual — never a panic or NaN. And extreme demand is correctly infeasible.
    for name in ["f1_2026/vehicle.yaml", "fwd_hatch/vehicle.yaml"] {
        let car = assemble(name);
        for vi in 0..8 {
            let v = 20.0 + 10.0 * f64::from(vi);
            for ai in 0..9 {
                let ay = -8.0 + 2.0 * f64::from(ai);
                for xi in 0..5 {
                    let ax = -4.0 + 2.0 * f64::from(xi);
                    match car.trim(&TrimInput::flat(v, ay, ax)) {
                        TrimOutcome::Converged(s) => {
                            assert!(s.residual_norm.is_finite() && s.residual_norm <= 1e-10);
                            assert!(s.delta.is_finite() && s.beta.is_finite());
                        }
                        TrimOutcome::Infeasible { residual_norm, .. } => {
                            assert!(residual_norm.is_finite() && residual_norm > 1e-10);
                        }
                    }
                }
            }
        }
        // A demand far beyond any tyre: 45 m/s² lateral is unreachable → infeasible, no panic.
        assert!(!car.trim(&TrimInput::flat(40.0, 45.0, 0.0)).is_feasible());
    }
}

#[test]
fn fz_coupling_modes_agree_at_convergence() {
    let car = assemble("f1_2026/vehicle.yaml");
    let base = TrimInput::flat(55.0, 12.0, -5.0);
    let lag = state(&car.trim(&TrimInput {
        coupling: FzCoupling::OneStepLag,
        ..base
    }));
    let fixed = state(&car.trim(&TrimInput {
        coupling: FzCoupling::FixedPoint,
        ..base
    }));
    // Both closures converge to the same trim (at convergence ΣFy = m·ay, so they coincide).
    for i in 0..4 {
        assert!(
            (lag.fz[i] - fixed.fz[i]).abs() < 1e-3,
            "wheel {i} Fz differs across coupling modes"
        );
    }
    assert!((lag.delta - fixed.delta).abs() < 1e-6);
}

#[test]
fn wheel_lift_stays_physical() {
    // Adversarial review regression: near the friction limit the inside wheel can lift. Its load
    // must floor at 0 and its partner must not exceed the whole axle load — otherwise the g-g
    // boundary is optimistic. ΣFz must still equal weight + downforce.
    let car = assemble("f1_2026/vehicle.yaml");
    let g = outlap_qss::G;
    for &(v, ay, ax) in &[(45.0, 22.0, 0.0), (40.0, 24.0, -4.0), (60.0, 26.0, 0.0)] {
        if let Some(s) = car.trim(&TrimInput::flat(v, ay, ax)).state() {
            let front_axle = s.fz[0] + s.fz[1];
            let rear_axle = s.fz[2] + s.fz[3];
            for i in 0..4 {
                assert!(
                    s.fz[i] >= 0.0,
                    "wheel {i} Fz negative ({}) at (v={v},ay={ay})",
                    s.fz[i]
                );
            }
            // No single wheel carries more than its whole axle.
            assert!(s.fz[0] <= front_axle + 1e-6 && s.fz[1] <= front_axle + 1e-6);
            assert!(s.fz[2] <= rear_axle + 1e-6 && s.fz[3] <= rear_axle + 1e-6);
            let sum: f64 = s.fz.iter().sum();
            let expected = car.mass_kg * g + (car.qz_f + car.qz_r) * v * v;
            assert!(
                (sum - expected).abs() < 1e-6 * expected,
                "ΣFz drifted at wheel-lift"
            );
        }
    }
}

#[test]
fn rejects_nonpositive_or_nan_speed() {
    // Adversarial review regression: the QSS kinematics divide by v, so a stationary/NaN speed has
    // no well-posed trim — it must return Infeasible cleanly, never a NaN residual or panic.
    let car = assemble("f1_2026/vehicle.yaml");
    for &v in &[0.0, -5.0, f64::NAN, 0.1] {
        let out = car.trim(&TrimInput::flat(v, 5.0, 0.0));
        assert!(!out.is_feasible(), "v={v} should be infeasible");
    }
    // Just above the floor is fine.
    assert!(car.trim(&TrimInput::flat(5.0, 3.0, 0.0)).is_feasible());
}

#[test]
fn setup_metrics_are_finite() {
    let car = assemble("fwd_hatch/vehicle.yaml");
    let k = car
        .understeer_gradient(30.0, outlap_qss::G)
        .expect("gradient probes feasible");
    assert!(k.is_finite());
    let balance = car.aero_front_downforce_share();
    assert!((0.0..=1.0).contains(&balance));
}
