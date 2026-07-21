// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end check of the T0-on-envelope velocity-profile solve (PR7): the constant-μ friction
//! ellipse (T0's degenerate no-envelope path) and the T1-derived g-g-g-v envelope both solve a lap
//! on the same flat circuit and agree to within a documented band. They are *different* grip models
//! — the ellipse is an axle-mean constant-μ point mass; the envelope carries per-axle load transfer,
//! downforce, and the double-track trim — so an exact match is not expected, but both must produce a
//! plausible, finite lap and land in the same ballpark.
#![allow(clippy::many_single_char_names)]

use std::f64::consts::PI;

use outlap_qss::path::T0Path;
use outlap_qss::solver::{solve_into, solve_into_ggv};
use outlap_qss::{GgvEnvelope, T0Options, T0Vehicle, T0Workspace, T1Vehicle};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::MemLoader;
use outlap_schema::refs::CenterlineRef;
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling};
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");
/// Standard gravity, m/s².
const G: f64 = 9.806_65;

/// Build the resolved vehicle + a source loader from an in-memory fixture (a rear-driven downforce
/// car, rev-limited to a realistic top speed).
fn fixture() -> (T0Vehicle, T1Vehicle) {
    let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
        axes: {speed_rpm: [0.0, 12000.0], load_axis: {torque_nm: [0.0, 800.0]}, torque_nm: [0.0, 800.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 12000.0], torque_nm: [800.0, 800.0]}}\n\
        inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
    let veh = "schema: vehicle/2.0\nname: t\n\
        chassis: {mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}\n\
        aero: {map: a.parquet, axes: [], constant: {cx_a_m2: 1.0, cz_front_a_m2: 1.5, cz_rear_a_m2: 3.0}}\n\
        suspension: {model: lumped_kc, front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}, rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}\n\
        tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}\n\
        drivetrain: {units: [{id: u0, source: ptm/u.ptm.yaml, path: [{fixed_ratio: 9.0}], wheels: [RL, RR]}]}\n\
        brakes: {balance_bar: 0.6, disc: {front: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}, rear: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}}\n";
    let loader = MemLoader::new()
        .with("vehicle.yaml", veh)
        .with("ptm/u.ptm.yaml", ptm)
        .with("tyr/slick.tyr.yaml", SLICK);
    let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let t0 =
        T0Vehicle::assemble(&rv, &Conditions::default(), &loader, &T0Options::default()).unwrap();
    let t1 = T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap();
    (t0, t1)
}

/// A flat closed circle of radius `r` metres, sampled every ≈`ds` metres.
fn circle(r: f64, n: usize) -> T0Path {
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let th = 2.0 * PI * f64::from(u32::try_from(i).unwrap())
                / f64::from(u32::try_from(n).unwrap());
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
    T0Path::from_track(&track, 2.0)
}

/// One envelope generate feeds both the ellipse↔envelope lap comparison and the grip
/// self-consistency check (generation is the expensive part — do it once).
#[test]
fn t0_on_envelope_lap() {
    let (t0, t1) = fixture();
    let env = GgvEnvelope::generate(
        &t1,
        &EnvelopeRes {
            v_points: 8,
            ax_points: 9,
            g_normal_points: 3,
        },
        FzCoupling::OneStepLag,
    )
    .unwrap();

    let path = circle(60.0, 200);
    let mut ws = T0Workspace::for_path(&path);

    let lap_ellipse = solve_into(&t0, &path, &mut ws).unwrap();
    let lap_ggv = solve_into_ggv(&t0, &env, &path, &mut ws).unwrap();

    // Both models produce a plausible, finite lap.
    assert!(
        lap_ellipse.is_finite() && lap_ellipse > 0.0,
        "ellipse lap: {lap_ellipse}"
    );
    assert!(lap_ggv.is_finite() && lap_ggv > 0.0, "ggv lap: {lap_ggv}");

    // They use genuinely different grip models (axle-mean constant-μ point mass vs per-axle
    // load-transfer double-track trim with downforce), so a band is expected (realised ≈5% on this
    // fixture); ±20% keeps a healthy margin while still catching a substantial systematic regression
    // in the envelope-consuming solve.
    let rel = (lap_ggv - lap_ellipse).abs() / lap_ellipse;
    assert!(
        rel < 0.20,
        "T0-on-envelope lap {lap_ggv:.2}s vs ellipse {lap_ellipse:.2}s differ by {:.1}% (>20%)",
        rel * 100.0
    );

    // The envelope-solved speed profile is self-consistent: at every station the required lateral
    // acceleration does not exceed the envelope's pure-lateral boundary (small interp tolerance). The
    // solve left `ws.v` holding the envelope profile.
    for i in 0..path.len() {
        let v = ws.v[i];
        let u = v * v;
        let ay = (path.kappa_l[i] * u + G * path.sin_b_cos_g[i]).abs();
        let gn = G * path.cos_b_cos_g[i] + path.kappa_n[i] * u;
        let cap = env.ay_boundary(v, 0.0, gn);
        assert!(
            ay <= cap * 1.05 + 0.2,
            "station {i}: lateral demand {ay:.2} exceeds grip {cap:.2} at v={v:.1}"
        );
    }
}
