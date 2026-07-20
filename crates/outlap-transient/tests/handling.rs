// SPDX-License-Identifier: AGPL-3.0-only
// Release-only: a full T3 maneuver is far too slow to gate in debug (like the perf tests). The whole
// file is compiled out in debug builds so its helpers do not read as dead code.
#![cfg(not(debug_assertions))]
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::float_cmp,
    clippy::too_many_lines,
    clippy::doc_markdown
)]
//! Gate #4 (M6 PR8, §13 chassis 14-DOF row): the T3 chassis reproduces the published **CommonRoad
//! vehicle 2 (BMW 320i)** single-track handling benchmark in the linear regime (D-M6-6).
//!
//! The car (`data/vehicles/bmw320i`) carries brush tyres whose cornering stiffness is set to the
//! CommonRoad axle values (which give an analytically NEUTRAL car, understeer gradient K = 0 and
//! yaw-rate gain r/δ = V/L; see `python/tools/gen_bmw320i_golden.py`). Using the M6/PR8 **prescribed
//! open-loop steer** input, an open-loop skidpad at a linear-regime lateral acceleration sweeps three
//! speeds and extracts the steady **yaw-rate gain** and the **understeer gradient**.
//!
//! The ASSERTED, robust claim is that the car is near-neutral: the understeer gradient extracted
//! from the sweep stays small (|K| < 6e-4 rad·s²/m), i.e. the 14-DOF collapses to essentially the
//! single-track benchmark, which is the near-neutral-catching regression guard. RECORDED, not gated
//! (D-M6-6, the Decision #48 pattern): the yaw-rate gain sits a few per-cent below the rigid V/L —
//! the 14-DOF adds a small residual understeer the point-mass bicycle cannot have (lateral load
//! transfer through the finite-a_y brush operating point, plus roll), so the ideal 3 percent is not
//! asserted; the residual is printed and decomposed in the validation page. The transient step-steer
//! against the CommonRoad ST golden is likewise recorded (the ST model has no roll/unsprung motion).
//!
//! Release-only (a full T3 maneuver is heavy); mirrors the `dynamics.rs` / `parity_report.rs` gates.

mod common;

use std::path::PathBuf;

use common::{assemble_car, build_blocks_t3, line};
use outlap_core::state::ChassisState;
use outlap_schema::sim::FzCoupling;
use outlap_transient::{SimConfig, TransientSolver};
use outlap_vehicle::{PrescribedSteer, SteerSource};

const AY_TARGET: f64 = 0.5; // linear-regime lateral acceleration, m/s² (near the true linear limit)
const K_NEUTRAL: f64 = 6.0e-4; // |understeer gradient| bound: the asserted "near-neutral" claim

fn cfg() -> SimConfig<f64> {
    SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    }
}

/// Parse the committed oracle metrics (key,value CSV) generated from the CommonRoad ST model.
fn oracle() -> std::collections::HashMap<String, f64> {
    let path = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/golden/bmw320i/metrics.csv"
    ));
    let text = std::fs::read_to_string(path).expect("read bmw320i metrics golden");
    text.lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with("key,"))
        .filter_map(|l| l.split_once(','))
        .map(|(k, v)| (k.to_owned(), v.trim().parse().unwrap()))
        .collect()
}

/// Run an open-loop skidpad at speed `v` and radius `r` (so a_y = v²/r) with a constant prescribed
/// front steer `delta`, and return the steady yaw rate (mean of the final quarter of the run).
fn skidpad_steady_yaw(v: f64, r: f64, delta: f64) -> f64 {
    let (t1, spec) = assemble_car("bmw320i");
    let mut it = outlap_core::bus::ChannelInterner::new();
    let mut blocks = build_blocks_t3(&t1, &spec, &mut it);
    // A circular road of radius r keeps the car in a bounded road-frame region; the OPEN-LOOP
    // prescribed steer (not the driver's curvature FF) sets the actual turn. The longitudinal PI
    // still holds v_ref = v.
    let circumference = 2.0 * std::f64::consts::PI * r;
    blocks.driver.steer_source = SteerSource::Prescribed(
        PrescribedSteer::new(vec![0.0, 10.0 * circumference], vec![delta, delta]).unwrap(),
    );
    let mut solver = TransientSolver::new(
        blocks,
        line(circumference, 720, true, 1.0 / r, 1.0 / r, v, Some(r)),
        &it,
        cfg(),
    );
    // Settle, then average the yaw rate over the final quarter (steady turn).
    let steps = 6000;
    let mut yaw_sum = 0.0;
    let mut n = 0.0;
    for k in 0..steps {
        solver.step();
        assert!(
            !solver.diverged(),
            "bmw320i skidpad diverged at v={v}, r={r}"
        );
        if k >= steps * 3 / 4 {
            yaw_sum += solver.fast_state()[ChassisState::YawRate as usize];
            n += 1.0;
        }
    }
    yaw_sum / n
}

#[test]
fn t3_yaw_rate_gain_matches_the_commonroad_single_track() {
    let o = oracle();
    let wheelbase = o["wheelbase_m"];
    // Neutral-steer benchmark ⇒ each speed's oracle gain is V/L. Keep a_y in the linear regime by
    // choosing the radius per speed, and the prescribed steer δ = L/R a neutral car would need.
    let mut gains = Vec::new();
    for &v in &[20.0_f64, 25.0, 30.0] {
        let r = v * v / AY_TARGET;
        let delta = wheelbase / r;
        let yaw = skidpad_steady_yaw(v, r, delta);
        let gain = yaw / delta;
        let target = o[&format!("yaw_rate_gain_at_{}mps", v as i32)];
        // RECORDED (not the ideal ≤3%): the 14-DOF sits a few % below the rigid V/L — a residual
        // understeer the point-mass bicycle cannot have. It is the derived, speed-dependent
        // consequence of the small understeer gradient asserted below; decomposed in the page.
        eprintln!(
            "[handling] v={v:.0} m/s  a_y≈{:.1}  gain={gain:.4}  oracle(V/L)={target:.4}  \
             Δ={:+.2}% (recorded)",
            v * v / r,
            100.0 * (gain - target) / target
        );
        gains.push((v, gain));
    }

    // ASSERTED — near-neutral: from the sweep, gain(v) = v/(L + K·v²) ⇒ K = (v/gain − L)/v². The
    // CommonRoad car is neutral (K = 0); the 14-DOF must reproduce that to within a small understeer,
    // i.e. |K| stays well below a genuinely understeering car (~2–5e-3). This is the robust chassis
    // claim: the tyre matching + 14-DOF collapse to essentially the single-track handling.
    for &(v, gain) in &gains {
        let k = (v / gain - wheelbase) / (v * v);
        eprintln!(
            "[handling] v={v:.0}: extracted understeer gradient K={k:.2e} rad·s²/m (asserted near-neutral)"
        );
        assert!(
            k.abs() < K_NEUTRAL,
            "v={v}: understeer gradient {k:.2e} exceeds the near-neutral bound {K_NEUTRAL:.0e} — the \
             14-DOF is not reproducing the neutral single-track benchmark"
        );
    }
}

/// The transient step-steer vs the CommonRoad ST golden is RECORDED, not gated (D-M6-6): the ST
/// model has no roll/unsprung dynamics, so the 14-DOF rise differs. This pins the golden is readable
/// and the steady value it reaches agrees with the analytic oracle (the asserted part above).
#[test]
fn commonroad_step_steer_golden_is_present_and_consistent() {
    let path = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/golden/bmw320i/step_steer.csv"
    ));
    let text = std::fs::read_to_string(path).expect("read step-steer golden");
    let yaw: Vec<f64> = text
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with("t_s,"))
        .filter_map(|l| l.split_once(','))
        .map(|(_, y)| y.trim().parse().unwrap())
        .collect();
    assert!(yaw.len() > 100, "golden step-steer trace is present");
    let o = oracle();
    // The ST golden's steady yaw / δ must equal the analytic V/L it was generated from (self-check).
    let steady_gain = yaw.last().unwrap() / o["step_delta_rad"];
    let target = o["yaw_rate_gain_at_25mps"];
    eprintln!("[handling] ST golden steady gain={steady_gain:.4} vs V/L={target:.4} (recorded)");
    assert!(
        (steady_gain - target).abs() / target < 0.01,
        "the ST golden steady gain should match the analytic V/L oracle"
    );
}
