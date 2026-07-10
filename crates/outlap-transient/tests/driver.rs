// SPDX-License-Identifier: AGPL-3.0-only
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::needless_range_loop
)]
//! PR5 ideal-driver property tests on `limebeer_2014_f1`: the MacAdam-preview + PI driver keeps the
//! car on a smooth line, tracks a QSS-style speed profile with the PI + preview feed-forward, and is
//! bit-reproducible over a full closed lap (the augmented-ODE speed integral is deterministic).

mod common;

use common::{build_blocks, limebeer, line};
use outlap_core::bus::ChannelInterner;
use outlap_schema::sim::FzCoupling;
use outlap_transient::{LineSamples, LineTable, SimConfig, TransientSolver};

fn cfg() -> SimConfig<f64> {
    SimConfig {
        fz_coupling: FzCoupling::OneStepLag,
        ..SimConfig::default()
    }
}

#[test]
fn line_tracking_error_stays_bounded_on_a_smooth_corner() {
    // A gentle constant-radius corner (R = 150 m, v = 40 m/s ≈ 1.1 g — comfortably inside grip): the
    // preview steer + curvature feed-forward should hold the car on the reference line (n_ref = 0).
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);
    let radius = 150.0;
    let v = 40.0;
    let len = 2.0 * std::f64::consts::PI * radius;
    let l = line(len, 400, true, 1.0 / radius, 1.0 / radius, v, Some(radius));
    let mut solver = TransientSolver::new(blocks, l, &it, cfg());
    let lap = solver.run(len, 60_000);
    // Skip the first quarter-lap (steer/relaxation transient), then bound the tracking error.
    let start = lap.len() / 4;
    let max_n = lap.n[start..].iter().fold(0.0_f64, |m, &n| m.max(n.abs()));
    assert!(
        max_n < 0.5,
        "steady-state line error too large: |n|max = {max_n:.3} m"
    );
}

#[test]
fn tracks_a_ramping_speed_profile() {
    // Straight road, no steer; v_ref accelerates 35 → 60 then decelerates back to 35 over 3 km. The
    // preview feed-forward + PI should track it within a few m/s once past the initial lag.
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);

    let len = 3000.0;
    let stations = 300;
    let s: Vec<f64> = (0..stations)
        .map(|i| i as f64 * len / (stations as f64 - 1.0))
        .collect();
    // Triangular speed profile 35 → 60 → 35.
    let vref: Vec<f64> = s
        .iter()
        .map(|&si| {
            let frac = si / len;
            if frac < 0.5 {
                35.0 + (60.0 - 35.0) * (frac / 0.5)
            } else {
                60.0 - (60.0 - 35.0) * ((frac - 0.5) / 0.5)
            }
        })
        .collect();
    let mk = |v: f64| vec![v; stations];
    let samples = LineSamples {
        s: s.clone(),
        kappa_h: mk(0.0),
        grade: mk(0.0),
        banking: mk(0.0),
        kappa_v: mk(0.0),
        n_ref: mk(0.0),
        kappa_ref: mk(0.0),
        v_ref: vref.clone(),
        x_ref: s.clone(),
        y_ref: mk(0.0),
        z_ref: mk(0.0),
        lat_x: mk(0.0),
        lat_y: mk(1.0),
        lat_z: mk(0.0),
        closed: false,
    };
    let l = LineTable::new(&samples).unwrap();
    let mut solver = TransientSolver::new(blocks, l, &it, cfg());
    let lap = solver.run(len - 50.0, 200_000);

    // Compare v_x to the profile at the recorded stations, skipping the first 300 m of lag.
    let mut worst = 0.0_f64;
    for i in 0..lap.len() {
        if lap.s[i] < 300.0 || lap.s[i] > len - 300.0 {
            continue;
        }
        // Profile value at this arc length.
        let frac = lap.s[i] / len;
        let vr = if frac < 0.5 {
            35.0 + 25.0 * (frac / 0.5)
        } else {
            60.0 - 25.0 * ((frac - 0.5) / 0.5)
        };
        worst = worst.max((lap.vx[i] - vr).abs());
    }
    assert!(
        worst < 3.0,
        "speed tracking error too large: {worst:.2} m/s"
    );
}

#[test]
fn full_lap_is_bit_reproducible_with_the_pi_integral() {
    // A closed lap with a non-trivial speed profile exercises the augmented-ODE speed integral; two
    // runs must be bit-identical (determinism, HANDOFF §11.2).
    let (t1, spec) = limebeer();
    let run = || {
        let mut it = ChannelInterner::new();
        let blocks = build_blocks(&t1, &spec, &mut it);
        let radius = 120.0;
        let len = 2.0 * std::f64::consts::PI * radius;
        // Speed varies around the loop (v_ref dips mid-lap) to keep the PI integral working.
        let stations = 400;
        let s: Vec<f64> = (0..stations)
            .map(|i| i as f64 * len / (stations as f64 - 1.0))
            .collect();
        let vref: Vec<f64> = s
            .iter()
            .map(|&si| 32.0 + 6.0 * (2.0 * std::f64::consts::PI * si / len).sin())
            .collect();
        let (mut xr, mut yr, mut lx, mut ly) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for &si in &s {
            let th = si / radius;
            xr.push(radius * th.sin());
            yr.push(radius * (1.0 - th.cos()));
            lx.push(-th.sin());
            ly.push(th.cos());
        }
        let mk = |v: f64| vec![v; stations];
        let l = LineTable::new(&LineSamples {
            s: s.clone(),
            kappa_h: mk(1.0 / radius),
            grade: mk(0.0),
            banking: mk(0.0),
            kappa_v: mk(0.0),
            n_ref: mk(0.0),
            kappa_ref: mk(1.0 / radius),
            v_ref: vref,
            x_ref: xr,
            y_ref: yr,
            z_ref: mk(0.0),
            lat_x: lx,
            lat_y: ly,
            lat_z: mk(0.0),
            closed: true,
        })
        .unwrap();
        let mut solver = TransientSolver::new(blocks, l, &it, cfg());
        let lap = solver.run(len, 80_000);
        (
            lap.len(),
            lap.vx.clone(),
            lap.n.clone(),
            lap.throttle.clone(),
        )
    };
    let (n1, v1, off1, thr1) = run();
    let (n2, v2, off2, thr2) = run();
    assert_eq!(n1, n2);
    assert_eq!(v1, v2, "speed trace bit-identical (integral determinism)");
    assert_eq!(off1, off2, "offset trace bit-identical");
    assert_eq!(thr1, thr2, "throttle trace bit-identical");
    // Sanity: the car actually completed a meaningful lap.
    assert!(n1 > 100, "recorded a full lap");
}
