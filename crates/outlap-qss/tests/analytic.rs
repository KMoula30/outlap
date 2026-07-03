// SPDX-License-Identifier: AGPL-3.0-only
//! Analytic checks of the T0 solver against closed-form values on synthetic tracks.
#![allow(
    clippy::many_single_char_names,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::f64::consts::PI;

use outlap_qss::path::T0Path;
use outlap_qss::solver::solve_lap;
use outlap_qss::{LineDescriptor, T0Options, T0Vehicle, DEFAULT_DS_M, G};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::MemLoader;
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");
// μ from slick: μx = PDX1 = 1.30, μy = PDY1 = 1.25.
const MU_Y: f64 = 1.25;

/// A flat-torque drive unit (constant tractive force below the rev limit).
fn flat_ptm(torque: f64) -> String {
    format!(
        "schema: ptm/1.0\nkind: drive_unit\n\
         axes: {{speed_rpm: [0.0, 30000.0], load_axis: {{torque_nm: [0.0, {torque}]}}, torque_nm: [0.0, {torque}]}}\n\
         tables: {{file: x.parquet}}\n\
         limits: {{max_torque_nm_vs_speed: {{speed_rpm: [0.0, 30000.0], torque_nm: [{torque}, {torque}]}}}}\n\
         inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {{upstream_ratio_applied: false}}\n"
    )
}

/// Build a 1000 kg, single-fixed-ratio (4:1) EV with the given constant aero and drive unit.
fn vehicle(cx: f64, cz: f64, ptm: &str) -> T0Vehicle {
    let veh = format!(
        "schema: vehicle/1.0\nname: t\n\
         chassis: {{mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}}\n\
         aero: {{map: a.parquet, axes: [], constant: {{cx_a_m2: {cx}, cz_front_a_m2: 0.0, cz_rear_a_m2: {cz}}}}}\n\
         suspension: {{model: lumped_kc, front: {{ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}, rear: {{ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}}}\n\
         tires: {{front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}}\n\
         drivetrain: {{units: [{{source: ptm/u.ptm.yaml, path: [{{fixed_ratio: 4.0}}], wheels: [RL, RR]}}]}}\n\
         brakes: {{balance_bar: 0.6, disc: {{front: {{thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}, rear: {{thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}}}}}\n"
    );
    let loader = MemLoader::new()
        .with("vehicle.yaml", veh)
        .with("ptm/u.ptm.yaml", ptm.to_owned())
        .with("tyr/slick.tyr.yaml", SLICK);
    let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    T0Vehicle::assemble(&rv, &Conditions::default(), &loader, &T0Options::default()).unwrap()
}

fn track_from(rows: Vec<CenterlineRow>, closed: bool, name: &str) -> Track {
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: name.into(),
        closed,
        centerline: CenterlineRef("mem".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

fn solve(veh: &T0Vehicle, track: &Track) -> outlap_qss::LapResult {
    let path = T0Path::from_track(track, DEFAULT_DS_M);
    solve_lap(
        veh,
        &path,
        LineDescriptor::Centerline,
        String::new(),
        vec![],
    )
    .unwrap()
}

#[test]
fn flat_circle_runs_at_lateral_limit() {
    let r = 100.0;
    let n = 400usize;
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let th = 2.0 * PI * i as f64 / n as f64;
            row(r * th, r * th.cos(), r * th.sin(), 0.0, 0.0)
        })
        .collect();
    let track = track_from(rows, true, "circle");
    let veh = vehicle(0.0, 0.0, &flat_ptm(400.0)); // no aero
    let lap = solve(&veh, &track);

    let v_expect = (MU_Y * G * r).sqrt(); // √(μ·g·R)
    let lap_expect = 2.0 * PI * r / v_expect;
    assert!(
        (lap.lap_time_s - lap_expect).abs() / lap_expect < 5e-3,
        "lap {} vs {lap_expect}",
        lap.lap_time_s
    );
    for &v in &lap.v {
        assert!(
            (v - v_expect).abs() / v_expect < 1e-2,
            "v {v} vs {v_expect}"
        );
    }
}

#[test]
fn banked_circle_matches_closed_form() {
    // CCW (left) circle with banking that raises the OUTSIDE (right) edge → negative in our
    // convention (positive banking assists a right turn) → banking helps, higher limit.
    let r = 100.0;
    let n = 400usize;
    let phi_deg = -15.0;
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let th = 2.0 * PI * i as f64 / n as f64;
            row(r * th, r * th.cos(), r * th.sin(), 0.0, phi_deg)
        })
        .collect();
    let track = track_from(rows, true, "banked");
    let veh = vehicle(0.0, 0.0, &flat_ptm(400.0));
    let lap = solve(&veh, &track);

    // v² = gR(μ·cosφ + sinφ)/(cosφ − μ·sinφ), φ = |banking|.
    let phi = phi_deg.abs().to_radians();
    let v2 = G * r * (MU_Y * phi.cos() + phi.sin()) / (phi.cos() - MU_Y * phi.sin());
    let v_expect = v2.sqrt();
    assert!(v_expect > (MU_Y * G * r).sqrt(), "banking should help");
    for &v in &lap.v {
        assert!(
            (v - v_expect).abs() / v_expect < 1.5e-2,
            "v {v} vs {v_expect}"
        );
    }
}

#[test]
fn crest_respects_the_flight_limit() {
    // A localized cosine crest z = ½A(1+cos(πξ/w)) over |ξ| < w (flat elsewhere), apex at xa. At
    // the apex κ_v = −A·π²/(2w²), so the airborne-speed limit is v = √(g/|κ_v|). A flat run-up lets
    // the car reach it. This is the property test for the N ≥ 0 flight guard.
    let a = 10.0;
    let w = 100.0;
    let xa = 300.0;
    let z = |x: f64| {
        let xi = x - xa;
        if xi.abs() < w {
            0.5 * a * (1.0 + (PI * xi / w).cos())
        } else {
            0.0
        }
    };
    let rows: Vec<CenterlineRow> = (0..=600)
        .map(|i| row(f64::from(i), f64::from(i), 0.0, z(f64::from(i)), 0.0))
        .collect();
    let track = track_from(rows, false, "crest");
    let veh = vehicle(0.0, 0.0, &flat_ptm(1200.0)); // strong: grip-limited run-up

    let kappa_v = a * PI * PI / (2.0 * w * w);
    let v_flight = (G / kappa_v).sqrt();
    let lap = solve(&veh, &track);
    let apex = (xa / lap_ds(&lap)).round() as usize;
    assert!(v_flight < veh.v_cap, "flight limit must bind below v_cap");
    assert!(
        lap.v[apex] <= v_flight * 1.03,
        "apex {} exceeds flight {v_flight}",
        lap.v[apex]
    );
    assert!(
        lap.v[apex] > 0.85 * v_flight,
        "apex {} should reach ~flight {v_flight}",
        lap.v[apex]
    );
}

/// The uniform station step actually used (length / segments).
fn lap_ds(lap: &outlap_qss::LapResult) -> f64 {
    lap.s[1] - lap.s[0]
}

#[test]
fn constant_force_straight_matches_kinematics() {
    // Flat straight, no aero, traction-limited constant force F → v(s) = √(2·F/m·s).
    let rows: Vec<CenterlineRow> = (0..=300)
        .map(|i| row(f64::from(i) * 2.0, f64::from(i) * 2.0, 0.0, 0.0, 0.0))
        .collect();
    let track = track_from(rows, false, "straight");
    let veh = vehicle(0.0, 0.0, &flat_ptm(400.0));
    let lap = solve(&veh, &track);

    let f = veh.tractive_force(5.0); // constant below the rev limit; < grip so traction-limited
    let m = veh.mass_kg;
    for i in [20usize, 60, 120, 200] {
        let s = lap.s[i];
        let v_expect = (2.0 * f / m * s).sqrt();
        assert!(
            (lap.v[i] - v_expect).abs() / v_expect < 1e-2,
            "v({s}) = {} vs {v_expect}",
            lap.v[i]
        );
    }
}

fn row(s: f64, x: f64, y: f64, z: f64, banking_deg: f64) -> CenterlineRow {
    CenterlineRow {
        s_m: s,
        x_m: x,
        y_m: y,
        z_m: z,
        banking_deg,
        width_left_m: 6.0,
        width_right_m: 6.0,
        grip_scale: 1.0,
    }
}
