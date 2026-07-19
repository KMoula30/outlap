// SPDX-License-Identifier: AGPL-3.0-only
//! M6 PR3 — QSS braking **regen** for a battery + electric machine, independent of the ERS manager.
//! Any mapped EV recovers braking energy through its machine (the same `blend_regen` ceilings the
//! transient tier already models, collapsed to the QSS point mass): the SoC falls under traction and
//! RISES under braking, bounded by the machine envelope, the blend authority, and — decisively near a
//! full pack — the charge acceptance. A car with no `regen_blend` block stays discharge-only.
#![allow(
    clippy::doc_markdown,
    clippy::similar_names,
    clippy::many_single_char_names
)]

use std::path::Path;

use outlap_core::GriddedTable;
use outlap_qss::path::T0Path;
use outlap_qss::{
    solve_t0, Couplings, GgvEnvelope, LapRequest, LineDescriptor, Pack, PackState, SlowCoupling,
    T0Options, T0Vehicle, T1Powertrain, T1Vehicle,
};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::{FsLoader, SourceLoader};
use outlap_schema::load::{load_battery, load_ptm};
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling};
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_vehicle, Conditions, LoadOptions, ResolvedVehicle};
use outlap_track::Track;

/// The committed Tesla Model 3 RWD (HV variant): a real mapped EV — Vdc-stacked drive unit + 800 V
/// pack + a regen curve — the one reference car exercising the non-manager regen path end-to-end.
fn model3() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/vehicles/tesla_model3_rwd"
    ))
}

/// A closed stadium: two straights + two half-circles, so a lap has real braking zones to harvest.
fn stadium_track() -> Track {
    let (r, straight, n_str, n_arc) = (60.0_f64, 700.0_f64, 140, 60);
    let mut rows: Vec<CenterlineRow> = Vec::new();
    let mut s = 0.0;
    let push = |s: f64, x: f64, y: f64| CenterlineRow {
        s_m: s,
        x_m: x,
        y_m: y,
        z_m: 0.0,
        banking_deg: 0.0,
        width_left_m: 6.0,
        width_right_m: 6.0,
        grip_scale: 1.0,
    };
    for i in 0..n_str {
        rows.push(push(s, straight * f64::from(i) / f64::from(n_str), -r));
        s += straight / f64::from(n_str);
    }
    for i in 0..n_arc {
        let th =
            -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * f64::from(i) / f64::from(n_arc);
        rows.push(push(s, straight + r * th.cos(), r * th.sin()));
        s += std::f64::consts::PI * r / f64::from(n_arc);
    }
    for i in 0..n_str {
        rows.push(push(
            s,
            straight * (1.0 - f64::from(i) / f64::from(n_str)),
            r,
        ));
        s += straight / f64::from(n_str);
    }
    for i in 0..n_arc {
        let th =
            std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * f64::from(i) / f64::from(n_arc);
        rows.push(push(s, r * th.cos(), r * th.sin()));
        s += std::f64::consts::PI * r / f64::from(n_arc);
    }
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "stadium".into(),
        closed: true,
        centerline: outlap_schema::refs::CenterlineRef("m".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

struct Ev {
    t1: T1Vehicle,
    t0: T0Vehicle,
    env: GgvEnvelope,
    pack: Pack,
    state: PackState,
}

/// Resolve a sidecar `file` referenced by a `.ptm`/battery YAML at `src`: next to it, then the root.
fn sidecar_path(src: &str, file: &str) -> String {
    Path::new(src).parent().map_or_else(
        || file.to_owned(),
        |d| d.join(file).to_string_lossy().into_owned(),
    )
}

/// Assemble the Model 3 with its powertrain map + pack installed (mirroring the binding's
/// `install_sidecars`). `regen` toggles the `regen_blend` block (the discharge-only control zeroes
/// it) and `soc0` seeds the pack.
fn ev(regen: bool, soc0: f64) -> Ev {
    let loader = model3();
    let mut resolved: ResolvedVehicle =
        load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).expect("model3 resolves");
    if !regen {
        resolved.spec.brakes.regen_blend = None;
    }
    let mut t1 = T1Vehicle::assemble(&resolved, &Conditions::default(), &loader, true).unwrap();

    // Install the drive unit's Vdc-stacked efficiency/loss + regen map (the source of the machine's
    // traction and regen envelopes).
    let src = resolved.spec.drivetrain.units[0].source.as_str().to_owned();
    let ptm = load_ptm(&src, &loader).unwrap();
    let table_path = sidecar_path(&src, ptm.tables.file.as_str());
    let bytes = loader.load_bytes(&table_path).unwrap();
    // The vdc-stacked and plain maps have different axis arities, so decode + install per-branch.
    if ptm.axes.vdc_v.is_some() {
        let table: GriddedTable<f64> =
            read_gridded_table(&bytes, &T1Powertrain::map_axis_names_vdc()).unwrap();
        t1.install_powertrain_maps(0, &table).unwrap();
    } else {
        let table: GriddedTable<f64> =
            read_gridded_table(&bytes, &T1Powertrain::map_axis_names()).unwrap();
        t1.install_powertrain_maps(0, &table).unwrap();
    }

    let res = EnvelopeRes {
        v_points: 8,
        ax_points: 7,
        g_normal_points: 2,
    };
    let env = GgvEnvelope::generate(&t1, &res, FzCoupling::OneStepLag).unwrap();
    let t0 = T0Vehicle::assemble(
        &resolved,
        &Conditions::default(),
        &loader,
        &T0Options::default(),
    )
    .unwrap();

    // The 800 V pack.
    let batt_path = resolved
        .spec
        .battery
        .as_ref()
        .unwrap()
        .params
        .as_str()
        .to_owned();
    let doc = load_battery(&batt_path, &loader).unwrap();
    let ecm_path = sidecar_path(&batt_path, doc.ecm.tables.file.as_str());
    let ecm: GriddedTable<f64> = read_gridded_table(
        &loader.load_bytes(&ecm_path).unwrap(),
        &Pack::ecm_axis_names(),
    )
    .unwrap();
    let (pack, mut state) = Pack::assemble(&doc, &ecm, None).unwrap();
    state.soc = soc0;
    Ev {
        t1,
        t0,
        env,
        pack,
        state,
    }
}

fn march(e: &Ev) -> Vec<f64> {
    let path = T0Path::from_track(&stadium_track(), 5.0);
    let electro = SlowCoupling {
        vehicle: &e.t1,
        thermal: None,
        pack: e.pack.clone(),
        pack_state: e.state,
        active: e.t1.has_energy_maps(),
    };
    let lap = solve_t0(
        &e.t0,
        e.env.clone(),
        &Couplings {
            electro: Some(&electro),
            tire: None,
            ers: None,
            fuel: None,
        },
        &path,
        LapRequest {
            line: LineDescriptor::Centerline,
            resolved_hash: String::new(),
            notes: vec![],
            fz_coupling: FzCoupling::OneStepLag,
            flat_track: false,
        },
    )
    .unwrap();
    lap.slow
        .expect("active EV stack has slow channels")
        .state_of_charge
}

/// A mapped EV recovers braking energy: SoC RISES under braking (regen) and still net-declines
/// (traction draw exceeds recovery). A `regen_blend`-less control of the SAME car stays monotone.
#[test]
fn ev_harvests_under_braking_only_with_a_regen_blend() {
    let soc = march(&ev(true, 0.6));
    let rises = soc.windows(2).any(|w| w[1] > w[0] + 1e-9);
    assert!(
        rises,
        "a mapped EV must recover charge under braking (regen)"
    );
    assert!(
        soc.last().unwrap() < soc.first().unwrap(),
        "a driven lap still draws NET charge (consumption > regen)"
    );

    // Control: strip the regen_blend block → discharge-only, SoC monotone non-increasing (the
    // pre-PR3 behaviour, proving the harvest is gated on the vehicle's regen policy).
    let none = march(&ev(false, 0.6));
    assert!(
        none.windows(2).all(|w| w[1] <= w[0] + 1e-12),
        "no `regen_blend` ⇒ no braking harvest (monotone discharge)"
    );
}

/// Charge acceptance decides the recovery: a nearly-FULL pack (top of window) accepts almost no
/// regen (the CV taper throttles it), so it recovers far less than a mid-charge pack over the same
/// braking. Physical, and the reason a hot-lap EV near 100% barely regens.
#[test]
fn regen_is_throttled_by_charge_acceptance_near_a_full_pack() {
    let full = march(&ev(true, 0.95));
    let mid = march(&ev(true, 0.55));
    let recovered = |soc: &[f64]| {
        soc.windows(2)
            .filter(|w| w[1] > w[0])
            .map(|w| w[1] - w[0])
            .sum::<f64>()
    };
    assert!(
        recovered(&full) < recovered(&mid),
        "a near-full pack accepts less regen ({:.2e}) than a mid-charge pack ({:.2e})",
        recovered(&full),
        recovered(&mid)
    );
}
