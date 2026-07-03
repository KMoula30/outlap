// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the T0 solve kernel (CLAUDE.md: allocs/step is CI-enforced).
//!
//! `solve_into` runs on a pre-allocated workspace and must not allocate. dhat's testing profiler
//! counts heap blocks; we assert the count is unchanged across a warmed solve. This is the template
//! for the hot-loop zero-alloc discipline.
#![allow(clippy::many_single_char_names)]

use std::f64::consts::PI;

use outlap_qss::path::T0Path;
use outlap_qss::solver::solve_into;
use outlap_qss::{T0Options, T0Vehicle, T0Workspace};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::MemLoader;
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

fn setup() -> (T0Vehicle, T0Path) {
    let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
        axes: {speed_rpm: [0.0, 30000.0], load_axis: {torque_nm: [0.0, 800.0]}, torque_nm: [0.0, 800.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 30000.0], torque_nm: [800.0, 800.0]}}\n\
        inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
    let veh = "schema: vehicle/1.0\nname: t\n\
        chassis: {mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}\n\
        aero: {map: a.parquet, axes: [], constant: {cx_a_m2: 1.0, cz_front_a_m2: 0.0, cz_rear_a_m2: 3.0}}\n\
        suspension: {model: lumped_kc, front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}, rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}\n\
        tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}\n\
        drivetrain: {units: [{source: ptm/u.ptm.yaml, path: [{fixed_ratio: 4.0}], wheels: [RL, RR]}]}\n\
        brakes: {balance_bar: 0.6, disc: {front: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}, rear: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}}\n";
    let loader = MemLoader::new()
        .with("vehicle.yaml", veh)
        .with("ptm/u.ptm.yaml", ptm)
        .with("tyr/slick.tyr.yaml", SLICK);
    let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let veh =
        T0Vehicle::assemble(&rv, &Conditions::default(), &loader, &T0Options::default()).unwrap();

    let r = 120.0;
    let n = 400;
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let th = 2.0 * PI * f64::from(i) / f64::from(n);
            CenterlineRow {
                s_m: r * th,
                x_m: r * th.cos(),
                y_m: r * th.sin(),
                z_m: 0.0,
                banking_deg: 0.0,
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
        centerline: CenterlineRef("m".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    let track = Track::from_doc(&doc, &Centerline { rows }).unwrap();
    let path = T0Path::from_track(&track, 2.0);
    (veh, path)
}

#[test]
fn solve_into_is_zero_alloc() {
    let _profiler = dhat::Profiler::builder().testing().build();
    let (veh, path) = setup();
    let mut ws = T0Workspace::for_path(&path);

    // Warm (first call may touch lazily-initialised statics elsewhere).
    solve_into(&veh, &path, &mut ws).unwrap();

    let before = dhat::HeapStats::get();
    for _ in 0..16 {
        solve_into(&veh, &path, &mut ws).unwrap();
    }
    let after = dhat::HeapStats::get();

    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "solve_into allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );
}
