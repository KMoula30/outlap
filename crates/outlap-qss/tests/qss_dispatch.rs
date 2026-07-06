// SPDX-License-Identifier: AGPL-3.0-only
//! PR8 — tier dispatch, the per-wheel result surface, and the QSS slow-state coupling, end to end
//! on a synthetic full electrified stack (Vdc-mapped drive unit + Thevenin pack + `.emotor` LPTN).
//!
//! Covers: the closed coupling loop (SoC discharges monotonically, the winding heats, the run is
//! deterministic, and the caps can only slow the lap), the inert path (no installed maps ⇒ the
//! coupled solve is bit-identical to the uncoupled one and reports no slow channels), the `t1`
//! per-wheel/setup surface, the transient-tier typed error, and the flat-track path collapse.
#![allow(clippy::doc_markdown, clippy::similar_names)]

use std::f64::consts::PI;

use outlap_core::GriddedTable;
use outlap_qss::path::T0Path;
use outlap_qss::{
    solve_t0, solve_t1, tier_not_implemented, GgvEnvelope, LineDescriptor, MachineThermal, Pack,
    SlowCoupling, T0Options, T0Vehicle, T1Powertrain, T1Vehicle,
};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::{FsLoader, SourceLoader};
use outlap_schema::load::load_emotor;
use outlap_schema::refs::CenterlineRef;
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling, Tier};
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_battery, load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

/// A closed circular test track. `banked` adds elevation + banking so the flat-track collapse has
/// something to flatten.
fn circle_track(banked: bool) -> Track {
    let r = 150.0;
    let n = 300;
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let th = 2.0 * PI * f64::from(i) / f64::from(n);
            CenterlineRow {
                s_m: r * th,
                x_m: r * th.cos(),
                y_m: r * th.sin(),
                z_m: if banked { 3.0 * (2.0 * th).sin() } else { 0.0 },
                banking_deg: if banked { 6.0 } else { 0.0 },
                width_left_m: 6.0,
                width_right_m: 6.0,
                grip_scale: 1.0,
            }
        })
        .collect();
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "circle".into(),
        closed: true,
        centerline: CenterlineRef("m".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

/// The synthetic full stack: the `pdt_du_rwd` fixture with the Vdc-stacked efficiency/loss maps
/// installed, the synthetic Thevenin pack, and the detailed `.emotor` thermal network.
fn full_stack(
    install_maps: bool,
) -> (
    T1Vehicle,
    T0Vehicle,
    Pack,
    outlap_qss::PackState,
    MachineThermal,
) {
    let loader = fixtures();
    let rv = load_vehicle("pdt_du_rwd/vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let mut t1 = T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap();
    if install_maps {
        let bytes = loader.load_bytes("pdt_synth_du_vdc.maps.parquet").unwrap();
        let table: GriddedTable<f64> =
            read_gridded_table(&bytes, &T1Powertrain::map_axis_names_vdc()).unwrap();
        t1.install_powertrain_maps(0, &table).unwrap();
    }
    let t0 =
        T0Vehicle::assemble(&rv, &Conditions::default(), &loader, &T0Options::default()).unwrap();

    let doc = load_battery("battery/synth_pack.battery.yaml", &loader).unwrap();
    let ecm_bytes = loader
        .load_bytes("battery/synth_pack.tables.parquet")
        .unwrap();
    let ecm: GriddedTable<f64> = read_gridded_table(&ecm_bytes, &Pack::ecm_axis_names()).unwrap();
    let (pack, state) = Pack::assemble(&doc, &ecm, None).unwrap();

    let em = load_emotor("emotor/pdt_synth.emotor.yaml", &loader).unwrap();
    let thermal = MachineThermal::assemble(&em, &Conditions::default(), 45.0).unwrap();
    (t1, t0, pack, state, thermal)
}

/// A small envelope grid (test-speed; the physics gates run at full resolution elsewhere).
fn small_env(t1: &T1Vehicle) -> GgvEnvelope {
    let res = EnvelopeRes {
        v_points: 6,
        ax_points: 5,
        g_normal_points: 2,
    };
    GgvEnvelope::generate(t1, &res, FzCoupling::OneStepLag).unwrap()
}

#[test]
fn coupled_lap_discharges_heats_and_is_deterministic() {
    let (t1, t0, pack, state, thermal) = full_stack(true);
    let env = small_env(&t1);
    let track = circle_track(false);
    let path = T0Path::from_track(&track, 5.0);

    let solve = |coupling: Option<&SlowCoupling<'_>>| {
        solve_t0(
            &t0,
            env.clone(),
            coupling,
            &path,
            LineDescriptor::Centerline,
            String::new(),
            vec![],
            FzCoupling::OneStepLag,
            false,
        )
        .unwrap()
    };

    let uncoupled = solve(None);
    assert!(uncoupled.slow.is_none(), "no coupling ⇒ no slow channels");

    let coupling = SlowCoupling {
        vehicle: &t1,
        thermal,
        pack,
        pack_state: state,
    };
    let a = solve(Some(&coupling));
    let b = solve(Some(&coupling));

    // Deterministic: bit-identical repeat.
    assert_eq!(a.lap.lap_time_s.to_bits(), b.lap.lap_time_s.to_bits());
    assert_eq!(a.lap.v, b.lap.v);

    // The slow channels are present and physical.
    let slow = a.slow.as_ref().expect("coupled stack ⇒ slow channels");
    assert_eq!(slow.state_of_charge.len(), path.len());
    let soc = &slow.state_of_charge;
    assert!(
        soc.windows(2).all(|w| w[1] <= w[0] + 1e-12),
        "SoC must be monotone non-increasing over a lap (no regen in the QSS coupling)"
    );
    assert!(
        soc[path.len() - 1] < soc[0],
        "a driven lap must draw charge: {} -> {}",
        soc[0],
        soc[path.len() - 1]
    );
    let temp = &slow.machine_temp_c;
    assert!(
        temp.iter().copied().fold(f64::MIN, f64::max) > temp[0] + 1e-6,
        "the winding must heat under traction loss"
    );

    // The caps only remove traction: the coupled lap can never be faster.
    assert!(
        a.lap.lap_time_s >= uncoupled.lap.lap_time_s - 1e-9,
        "coupled {:.4} s vs uncoupled {:.4} s",
        a.lap.lap_time_s,
        uncoupled.lap.lap_time_s
    );
    // Recorded metadata.
    assert_eq!(a.tier, Tier::T0);
    assert!(a.envelope.is_some());
}

#[test]
fn coupling_without_installed_maps_is_inert() {
    let (t1, t0, pack, state, thermal) = full_stack(false); // no efficiency maps
    let env = small_env(&t1);
    let track = circle_track(false);
    let path = T0Path::from_track(&track, 5.0);

    let uncoupled = solve_t0(
        &t0,
        env.clone(),
        None,
        &path,
        LineDescriptor::Centerline,
        String::new(),
        vec![],
        FzCoupling::OneStepLag,
        false,
    )
    .unwrap();
    let coupling = SlowCoupling {
        vehicle: &t1,
        thermal,
        pack,
        pack_state: state,
    };
    let coupled = solve_t0(
        &t0,
        env,
        Some(&coupling),
        &path,
        LineDescriptor::Centerline,
        String::new(),
        vec![],
        FzCoupling::OneStepLag,
        false,
    )
    .unwrap();

    // `traction_energy` has no maps to evaluate ⇒ the scale stays 1 and nothing advances.
    assert!(
        coupled.slow.is_none(),
        "inert coupling must report no slow channels"
    );
    assert_eq!(
        coupled.lap.v, uncoupled.lap.v,
        "inert coupling must not move the profile"
    );
    assert_eq!(
        coupled.lap.lap_time_s.to_bits(),
        uncoupled.lap.lap_time_s.to_bits()
    );
}

#[test]
fn t1_emits_per_wheel_and_setup_channels() {
    let (t1, t0, ..) = full_stack(true);
    let env = small_env(&t1);
    let track = circle_track(false);
    let path = T0Path::from_track(&track, 5.0);

    let lap = solve_t1(
        &t1,
        &t0,
        env,
        None,
        &path,
        LineDescriptor::Centerline,
        String::new(),
        vec![],
        FzCoupling::OneStepLag,
        false,
    )
    .unwrap();

    assert_eq!(lap.tier, Tier::T1);
    let wheels = lap.wheels.as_ref().expect("t1 emits per-wheel channels");
    let setup = lap.setup.as_ref().expect("t1 emits setup metrics");
    let n = path.len();
    assert_eq!(wheels.vertical_load_n.len(), n);
    assert_eq!(setup.understeer_gradient.len(), n);

    // At converged stations the wheel loads are positive and sum near weight + downforce; the
    // channel is NaN only where the re-trim was infeasible (allowed, but not everywhere).
    let mut converged = 0;
    for (i, fz) in wheels.vertical_load_n.iter().enumerate() {
        if fz.iter().all(|f| f.is_finite()) {
            converged += 1;
            let total: f64 = fz.iter().sum();
            assert!(
                total > 0.5 * 420.0 * 9.81,
                "station {i}: ΣFz {total:.0} N implausibly low"
            );
        }
    }
    assert!(
        converged > n / 2,
        "most stations must re-trim ({converged}/{n} converged)"
    );
}

#[test]
fn transient_tiers_are_typed_errors() {
    let e2 = tier_not_implemented(Tier::T2).to_string();
    let e3 = tier_not_implemented(Tier::T3).to_string();
    assert!(e2.contains("t2") && e2.contains("M4"), "{e2}");
    assert!(e3.contains("t3") && e3.contains("M6"), "{e3}");
}

#[test]
fn flat_track_collapses_the_ribbon() {
    let track = circle_track(true); // banked + undulating
    let full = T0Path::from_track(&track, 5.0);
    let flat = T0Path::from_track_flat(&track, 5.0);

    // The 3-D ribbon genuinely carries banking/grade…
    assert!(full.sin_g.iter().any(|&s| s.abs() > 1e-3));
    assert!(full.sin_b_cos_g.iter().any(|&s| s.abs() > 1e-3));
    // …and the flat sampler zeroes every projection: g_normal ≡ g, no lateral/longitudinal
    // gravity component, no vertical curvature.
    assert!(flat.sin_g.iter().all(|&s| s == 0.0));
    assert!(flat.sin_b_cos_g.iter().all(|&s| s == 0.0));
    assert!(flat.cos_b_cos_g.iter().all(|&c| (c - 1.0).abs() < 1e-15));
    assert!(flat.kappa_n.iter().all(|&k| k == 0.0));
    // The horizontal geometry is preserved (same station count, similar plan curvature).
    assert_eq!(flat.len(), full.len());
    let k_flat: f64 = flat.kappa_l.iter().map(|k| k.abs()).sum();
    assert!(k_flat > 0.0, "plan curvature survives the collapse");
}
