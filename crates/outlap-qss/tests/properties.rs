// SPDX-License-Identifier: AGPL-3.0-only
//! Property tests: the solved profile stays inside the friction ellipse and below the curvature
//! limit across random tracks and vehicles, and the lap time converges under Δs refinement.
#![allow(clippy::many_single_char_names, clippy::cast_precision_loss)]

use std::f64::consts::PI;

use outlap_qss::path::T0Path;
use outlap_qss::solver::{solve_into, solve_lap};
use outlap_qss::{LineDescriptor, T0Options, T0Vehicle, T0Workspace, G};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::MemLoader;
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;
use proptest::prelude::*;

const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

fn build_vehicle(mass: f64, cx: f64, cz: f64) -> T0Vehicle {
    let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
        axes: {speed_rpm: [0.0, 30000.0], load_axis: {torque_nm: [0.0, 1500.0]}, torque_nm: [0.0, 1500.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 30000.0], torque_nm: [1500.0, 1500.0]}}\n\
        inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
    let veh = format!(
        "schema: vehicle/1.0\nname: t\n\
         chassis: {{mass_kg: {mass}, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}}\n\
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

/// A flat circle of radius `r` with a constant banking channel `phi_deg`.
fn banked_circle(r: f64, phi_deg: f64, n: usize) -> Track {
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let th = 2.0 * PI * i as f64 / n as f64;
            CenterlineRow {
                s_m: r * th,
                x_m: r * th.cos(),
                y_m: r * th.sin(),
                z_m: 0.0,
                banking_deg: phi_deg,
                width_left_m: 6.0,
                width_right_m: 6.0,
                grip_scale: 1.0,
            }
        })
        .collect();
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "c".into(),
        closed: true,
        centerline: CenterlineRef("mem".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

/// Friction-ellipse containment at station `i`, recomputing the tyre forces from the profile.
fn ellipse_ok(veh: &T0Vehicle, p: &T0Path, v: &[f64], i: usize) -> bool {
    let m = veh.mass_kg;
    let u = v[i] * v[i];
    let n = m * (G * p.cos_b_cos_g[i] + p.kappa_n[i] * u) + veh.qz * u;
    if n <= 1e-6 {
        return true; // flight is a separate invariant (v ≤ v_lim covers it)
    }
    let gamma = p.grip[i];
    let fy = m * (p.kappa_l[i] * u + G * p.sin_b_cos_g[i]);
    // Longitudinal tyre force from the segment-average acceleration.
    let j = (i + 1) % p.len();
    let ax = (v[j] * v[j] - v[i] * v[i]) / (2.0 * p.ds);
    let ft = m * ax + veh.qx * u + m * G * p.sin_g[i];
    let ex = ft / (veh.mu_x * gamma * n);
    let ey = fy / (veh.mu_y * gamma * n);
    ex * ex + ey * ey <= 1.0 + 0.05 // 5% slack for discretisation
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    #[test]
    fn stays_inside_the_envelope(
        r in 40.0f64..250.0,
        phi_deg in -10.0f64..10.0,
        mass in 500.0f64..2000.0,
        cx in 0.0f64..2.0,
        cz in 0.0f64..6.0,
    ) {
        let veh = build_vehicle(mass, cx, cz);
        let track = banked_circle(r, phi_deg, 360);
        let path = T0Path::from_track(&track, 2.0);
        let mut ws = T0Workspace::for_path(&path);
        solve_into(&veh, &path, &mut ws).unwrap();
        for i in 0..path.len() {
            prop_assert!(ws.v[i].is_finite() && ws.v[i] >= 0.0);
            prop_assert!(ws.v[i] <= ws.v_lim[i] + 1e-6, "v {} > v_lim {}", ws.v[i], ws.v_lim[i]);
            prop_assert!(ellipse_ok(&veh, &path, &ws.v, i), "ellipse violated at {i}");
        }
    }

    #[test]
    fn passes_are_idempotent(r in 40.0f64..250.0, mass in 500.0f64..2000.0) {
        let veh = build_vehicle(mass, 1.0, 3.0);
        let track = banked_circle(r, 0.0, 300);
        let path = T0Path::from_track(&track, 2.0);
        let mut a = T0Workspace::for_path(&path);
        let t1 = solve_into(&veh, &path, &mut a).unwrap();
        let mut b = T0Workspace::for_path(&path);
        let t2 = solve_into(&veh, &path, &mut b).unwrap();
        prop_assert!((t1 - t2).abs() < 1e-9);
    }
}

#[test]
fn lap_time_converges_under_refinement() {
    // The circle lap time is analytic, so refinement should shrink the discretisation error.
    let veh = build_vehicle(900.0, 1.0, 2.0);
    let track = banked_circle(120.0, 0.0, 720);
    let coarse = solve_lap(
        &veh,
        &T0Path::from_track(&track, 4.0),
        LineDescriptor::Centerline,
        String::new(),
        vec![],
    )
    .unwrap();
    let fine = solve_lap(
        &veh,
        &T0Path::from_track(&track, 1.0),
        LineDescriptor::Centerline,
        String::new(),
        vec![],
    )
    .unwrap();
    assert!(
        (coarse.lap_time_s - fine.lap_time_s).abs() / fine.lap_time_s < 5e-3,
        "coarse {} vs fine {}",
        coarse.lap_time_s,
        fine.lap_time_s
    );
}
