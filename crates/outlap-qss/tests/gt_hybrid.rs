// SPDX-License-Identifier: AGPL-3.0-only
//! The promoted `gt_hybrid` reference car laps on the real Catalunya track (D-M6-12).
//!
//! Two things are being pinned, and both are promotion contracts rather than physics:
//!
//! 1. The car under `data/vehicles/` loads with `allow_degraded: false`. Its schema-fixture twin
//!    cannot — the fixture's `aero/gt.parquet` does not exist, so every test that uses the fixture
//!    passes `true`. If the shipped car ever regresses to a missing sidecar, this test fails.
//! 2. The hybrid actually hybridises: the managed march both deploys and harvests real energy. A
//!    plain `solve_lap` cannot show that (its `LapResult` has no energy channels), so this runs the
//!    coupled `solve_t0` path the ERS march uses.
//!
//! The lap-time band brackets the measured pace rather than asserting a target: this car carries
//! the f1_2026 synthetic ICE, MGU-K and slick, so its pace is not a GT-class prediction. The band
//! exists to catch a regression in the car's data, not to validate the number itself.
#![allow(clippy::doc_markdown)]

use outlap_core::GriddedTable;
use outlap_powertrain::DeployPolicy;
use outlap_qss::path::T0Path;
use outlap_qss::{
    solve_t0, Couplings, ErsCoupling, GgvEnvelope, LapRequest, LineDescriptor, Pack, SlowCoupling,
    T0Options, T0Vehicle, T1Vehicle,
};
use outlap_schema::io::{FsLoader, SourceLoader};
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling};
use outlap_schema::{load_battery, load_vehicle, Conditions, LoadOptions, ResolvedVehicle};
use outlap_track::Track;

fn vehicle_loader() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/vehicles/gt_hybrid"
    ))
}

/// Install the powertrain efficiency/loss maps so the march can account energy (without them the
/// deploy path has no efficiency to charge against).
fn install_maps(t1: &mut T1Vehicle, resolved: &ResolvedVehicle, loader: &FsLoader) {
    for (idx, unit) in resolved.spec.drivetrain.units.iter().enumerate() {
        let Ok(ptm) = outlap_schema::load::load_ptm(unit.source.as_str(), loader) else {
            continue;
        };
        let sidecar = match unit.source.as_str().rsplit_once('/') {
            Some((parent, _)) => format!("{parent}/{}", ptm.tables.file.as_str()),
            None => ptm.tables.file.as_str().to_owned(),
        };
        let Ok(bytes) = loader.load_bytes(&sidecar) else {
            continue;
        };
        let table = if ptm.axes.vdc_v.is_some() {
            read_gridded_table(&bytes, &outlap_qss::T1Powertrain::map_axis_names_vdc())
        } else {
            read_gridded_table(&bytes, &outlap_qss::T1Powertrain::map_axis_names())
        }
        .expect("the ptm sidecar parses");
        t1.install_powertrain_maps(idx, &table)
            .expect("maps install");
    }
}

/// The assembled stack for the promoted car — everything a managed lap needs.
struct Stack {
    resolved: ResolvedVehicle,
    t1: T1Vehicle,
    t0: T0Vehicle,
    env: GgvEnvelope,
    pack: Pack,
    state: outlap_qss::PackState,
}

/// Assemble `data/vehicles/gt_hybrid` with `allow_degraded: false` throughout — that strictness is
/// the promotion contract this file exists to pin.
fn assemble_gt() -> Stack {
    let loader = vehicle_loader();
    let resolved = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default())
        .expect("the promoted gt_hybrid resolves through the real pipeline");

    let mut t1 = T1Vehicle::assemble(&resolved, &Conditions::default(), &loader, false)
        .expect("T1 assembles without allow_degraded");
    install_maps(&mut t1, &resolved, &loader);

    // A coarse envelope: this runs in the debug workspace suite, and the envelope build — not the
    // march — is what costs time there.
    let env = GgvEnvelope::generate(
        &t1,
        &EnvelopeRes {
            v_points: 6,
            ax_points: 5,
            g_normal_points: 2,
        },
        FzCoupling::OneStepLag,
    )
    .expect("envelope generates");

    let t0 = T0Vehicle::assemble(
        &resolved,
        &Conditions::default(),
        &loader,
        &T0Options::default(),
    )
    .expect("T0 assembles without allow_degraded");

    let batt_path = resolved
        .spec
        .batteries
        .values()
        .next()
        .expect("gt_hybrid declares a pack")
        .params
        .as_str();
    let doc = load_battery(batt_path, &loader).expect("gt_es loads");
    let ecm_bytes = loader
        .load_bytes(&format!("battery/{}", doc.ecm.tables.file.as_str()))
        .expect("gt_es ECM sidecar ships");
    let ecm: GriddedTable<f64> =
        read_gridded_table(&ecm_bytes, &Pack::ecm_axis_names()).expect("ECM table parses");
    let (pack, mut state) = Pack::assemble(&doc, &ecm, None).expect("pack assembles");
    let [lo, hi] = pack.soc_window();
    state.soc = 0.5 * (lo + hi);

    Stack {
        resolved,
        t1,
        t0,
        env,
        pack,
        state,
    }
}

#[test]
fn gt_hybrid_loads_clean_and_hybridises_over_a_lap() {
    let Stack {
        resolved,
        t1,
        t0,
        env,
        pack,
        state,
    } = assemble_gt();

    // The promotion contract. `assemble` above already refuses a degradation waiver (that is what
    // `allow_degraded: false` buys), so asserting `report.degraded.is_empty()` here would be
    // vacuous — the loader only fills that list when the waiver was granted. Assert instead on
    // what this car actually promises: its estimated entries are the documented suspension and
    // driver substitutions, and nothing has silently joined them.
    let estimated: Vec<&str> = resolved
        .report
        .estimated
        .iter()
        .map(|e| e.pointer.as_str())
        .collect();
    assert!(
        estimated
            .iter()
            .all(|p| p.starts_with("/suspension/") || p.starts_with("/driver/")),
        "a shipped reference car gained an undocumented estimated value: {estimated:?}"
    );

    let track = Track::load(
        "track.yaml",
        &FsLoader::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/tracks/catalunya_osm"
        )),
    )
    .expect("catalunya loads");
    let path = T0Path::from_track(&track, 5.0);

    let electro = SlowCoupling::single(&t1, None, pack.clone(), state, t1.has_energy_maps());
    let ers = ErsCoupling::assemble(
        &resolved.spec,
        &t0,
        pack.soc_window(),
        DeployPolicy::RuleBased,
        false,
    )
    .expect("ers coupling assembles")
    .expect("gt_hybrid has a policy block");

    let lap = solve_t0(
        &t0,
        env,
        &Couplings {
            electro: Some(&electro),
            tire: None,
            ers: Some(&ers),
            fuel: None,
        },
        &path,
        LapRequest {
            line: LineDescriptor::Centerline,
            resolved_hash: String::new(),
            notes: vec![],
            fz_coupling: FzCoupling::OneStepLag,
            flat_track: false,
            path_curvature_smooth_m: path.curv_smooth_m,
            vertical_baseline_m: 30.0,
        },
    )
    .expect("the managed lap solves");

    // Banded around the measured 111.7 s. Wide enough to absorb solver noise and a coarse
    // envelope, tight enough that losing the aero block, the ERS or a chunk of grip fails here.
    let t = lap.lap.lap_time_s;
    assert!(
        t.is_finite() && (100.0..125.0).contains(&t),
        "gt_hybrid Catalunya lap time out of band: {t:.2} s"
    );

    // The hybrid must actually hybridise: energy moves in both directions over a lap.
    let ers_log = lap
        .slow
        .as_ref()
        .expect("a managed lap reports slow channels")
        .ers
        .as_ref()
        .expect("the ERS ledger is present");
    assert!(
        ers_log.ledger_deploy_j > 0.0,
        "no energy deployed over the lap"
    );
    assert!(
        ers_log.ledger_harvest_j > 0.0,
        "no energy harvested over the lap"
    );
    // …and it hybridises within the limits the car itself declares: this policy allows 3.0 MJ of
    // harvest per lap, so a regression that ignored the budget would read as "hybridising" under
    // a bare > 0 check.
    let harvest_budget_j = resolved
        .spec
        .policy
        .as_ref()
        .expect("gt_hybrid declares a policy")
        .recovery
        .per_lap_harvest_mj
        * 1e6;
    assert!(
        ers_log.ledger_harvest_j <= harvest_budget_j + 1.0,
        "harvested {:.3} MJ against a declared {:.3} MJ per-lap allowance",
        ers_log.ledger_harvest_j * 1e-6,
        harvest_budget_j * 1e-6
    );
    eprintln!(
        "gt_hybrid Catalunya T0: {t:.2} s, deployed {:.3} MJ, harvested {:.3} MJ",
        ers_log.ledger_deploy_j * 1e-6,
        ers_log.ledger_harvest_j * 1e-6
    );
}

/// The promoted car's powertrain maps and tyre are hand copies of the f1_2026 ones, and no
/// generator writes the gt side — so the next time the f1 surfaces are recalibrated (it has
/// already happened twice this milestone) the two cars would silently diverge while both files
/// still claim identical numbers. This fails when that happens.
#[test]
fn gt_hybrid_shares_the_f1_powertrain_and_tyre_surfaces() {
    let data = concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/vehicles");
    for rel in [
        "ptm/tables/ice_v6.parquet",
        "ptm/tables/mgu_k.parquet",
        "tyr/slick.tyr.yaml",
        "ptm/ice_v6.ptm.yaml",
        "ptm/mgu_k.ptm.yaml",
    ] {
        let f1 = std::fs::read(format!("{data}/f1_2026/{rel}")).expect("f1 source exists");
        let gt = std::fs::read(format!("{data}/gt_hybrid/{rel}")).expect("gt copy exists");
        if rel.ends_with(".parquet") {
            assert_eq!(f1, gt, "{rel} drifted between f1_2026 and gt_hybrid");
        } else {
            // The YAML copies differ only in their leading provenance comments by design.
            let strip = |b: Vec<u8>| {
                String::from_utf8(b)
                    .expect("utf8")
                    .lines()
                    .filter(|l| !l.trim_start().starts_with('#'))
                    .collect::<Vec<_>>()
                    .join("\n")
            };
            assert_eq!(
                strip(f1),
                strip(gt),
                "{rel} drifted between f1_2026 and gt_hybrid (ignoring comments)"
            );
        }
    }
}
