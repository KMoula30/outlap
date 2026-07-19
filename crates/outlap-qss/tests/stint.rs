// SPDX-License-Identifier: AGPL-3.0-only
//! M6 PR3 — the QSS **stint** lap-boundary carry (the author's named acceptance check): a multi-lap
//! QSS run carries the FULL electro slow stack (pack SoC / RC voltage / temperature) across each lap
//! boundary, so SoC falls with net consumption and rises with regeneration lap-over-lap, with only
//! the per-lap ERS budget ledger resetting at the start/finish. These pin the pushed-down
//! `outlap_qss::solve_stint` directly (cargo-testable, no Python), which the binding just aggregates.
#![allow(
    clippy::doc_markdown,
    clippy::similar_names,
    clippy::many_single_char_names
)]
// Bit-exact station-0 continuity IS the assertion.
#![allow(clippy::float_cmp)]

use outlap_core::GriddedTable;
use outlap_powertrain::Policy;
use outlap_qss::path::T0Path;
use outlap_qss::{
    solve_stint, ErsCoupling, GgvEnvelope, LapRequest, LineDescriptor, Pack, PackState,
    StintElectro, StintPlan, StintSeeds, T0Options, T0Vehicle, T1Vehicle,
};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::{FsLoader, SourceLoader};
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling, Tier};
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_battery, load_vehicle, Conditions, LoadOptions, ResolvedVehicle};
use outlap_track::Track;

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

/// A closed "stadium": two straights + two half-circles → real braking (harvest), corner exits
/// (part-throttle harvest), and straights (deploy / super-clip) — an honest per-lap energy cycle.
fn stadium_track() -> Track {
    let r = 60.0;
    let straight = 700.0;
    let n_str = 140;
    let n_arc = 60;
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
        let x = straight * f64::from(i) / f64::from(n_str);
        rows.push(push(s, x, -r));
        s += straight / f64::from(n_str);
    }
    for i in 0..n_arc {
        let th =
            -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * f64::from(i) / f64::from(n_arc);
        rows.push(push(s, straight + r * th.cos(), r * th.sin()));
        s += std::f64::consts::PI * r / f64::from(n_arc);
    }
    for i in 0..n_str {
        let x = straight * (1.0 - f64::from(i) / f64::from(n_str));
        rows.push(push(s, x, r));
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

struct Hybrid {
    resolved: ResolvedVehicle,
    t1: T1Vehicle,
    t0: T0Vehicle,
    env: GgvEnvelope,
    pack: Pack,
    state: PackState,
}

fn hybrid(dir: &str, initial_soc: Option<f64>) -> Hybrid {
    let loader = fixtures();
    let resolved = load_vehicle(
        &format!("{dir}/vehicle.yaml"),
        &loader,
        &LoadOptions::default(),
    )
    .expect("fixture hybrid resolves");
    let t1 = T1Vehicle::assemble(&resolved, &Conditions::default(), &loader, true).unwrap();
    let res = EnvelopeRes {
        v_points: 6,
        ax_points: 5,
        g_normal_points: 2,
    };
    let env = GgvEnvelope::generate(&t1, &res, FzCoupling::OneStepLag).unwrap();
    let t0_opts = T0Options {
        allow_degraded: true,
        ..T0Options::default()
    };
    let t0 = T0Vehicle::assemble(&resolved, &Conditions::default(), &loader, &t0_opts).unwrap();
    let batt_path = resolved.spec.battery.as_ref().unwrap().params.as_str();
    let doc = load_battery(batt_path, &loader).unwrap();
    let sidecar = format!("battery/{}", doc.ecm.tables.file.as_str());
    let ecm_bytes = loader.load_bytes(&sidecar).unwrap();
    let ecm: GriddedTable<f64> = read_gridded_table(&ecm_bytes, &Pack::ecm_axis_names()).unwrap();
    let (pack, mut state) = Pack::assemble(&doc, &ecm, None).unwrap();
    let [lo, hi] = pack.soc_window();
    state.soc = initial_soc.unwrap_or(0.5 * (lo + hi));
    Hybrid {
        resolved,
        t1,
        t0,
        env,
        pack,
        state,
    }
}

/// Build the stint plan for a fixture hybrid at the given tier + override state.
fn plan<'a>(h: &'a Hybrid, ers: &'a ErsCoupling, path: &'a T0Path) -> StintPlan<'a> {
    StintPlan {
        tier: Tier::T0,
        t0: &h.t0,
        t1: &h.t1,
        env: &h.env,
        path,
        electro: Some(StintElectro {
            vehicle: &h.t1,
            pack: &h.pack,
            thermal: None,
            pack_state: h.state,
            active: h.t1.has_energy_maps(),
        }),
        ers: Some(ers),
        base_march: None,
        fuel: None,
        request: LapRequest {
            line: LineDescriptor::Centerline,
            resolved_hash: String::new(),
            notes: vec![],
            fz_coupling: FzCoupling::OneStepLag,
            flat_track: false,
        },
    }
}

/// The headline acceptance check: over a multi-lap f1_2026 stint the pack SoC is CONTINUOUS across
/// the lap boundary — lap N+1 enters at exactly lap N's terminal SoC, NOT re-seeded to mid-window —
/// and it genuinely moves lap-to-lap with the car's consumption + regeneration.
#[test]
fn f1_qss_stint_carries_soc_across_lap_boundaries() {
    let h = hybrid("f1_2026", None);
    let path = T0Path::from_track(&stadium_track(), 5.0);
    let ers = ErsCoupling::assemble(&h.resolved.spec, &h.t0, Policy::RuleBased, false)
        .unwrap()
        .unwrap();
    let n_laps = 5;
    let result = solve_stint(&plan(&h, &ers, &path), n_laps, StintSeeds::default()).unwrap();
    assert_eq!(result.laps.len(), n_laps);

    let seed = h.state.soc;
    for i in 0..n_laps {
        let lap = &result.laps[i];
        let slow = lap.slow.as_ref().expect("f1 stint lap has slow channels");
        lap.terminal
            .pack
            .expect("f1 stint lap carries a pack terminal");

        // Station 0 logs the ENTRY SoC — exactly the state this lap seeded from. Continuity: lap N+1
        // enters at lap N's terminal, NOT re-seeded (the whole point of the stint).
        let entry = slow.state_of_charge[0];
        if i == 0 {
            assert_eq!(entry, seed, "lap 1 must start from the assembled seed");
        } else {
            let prev_terminal = result.laps[i - 1].terminal.pack.unwrap().soc;
            assert_eq!(
                entry,
                prev_terminal,
                "lap {} must ENTER at lap {}'s terminal SoC (no reset)",
                i + 1,
                i
            );
        }
    }

    // Lap 1 from the mid-window seed genuinely moves the pack (deploy/harvest happened) — and the
    // stint is multi-lap-stateful: a later lap enters at a SoC well away from the assembled seed
    // (the f1 pack recharges toward the top of its window over the run, then charge-sustains there).
    let lap0 = &result.laps[0];
    assert!(
        (lap0.terminal.pack.unwrap().soc - lap0.slow.as_ref().unwrap().state_of_charge[0]).abs()
            > 1e-6,
        "lap 1 SoC must move from the seed"
    );
    let carried = (1..n_laps)
        .any(|i| (result.laps[i].slow.as_ref().unwrap().state_of_charge[0] - seed).abs() > 1e-6);
    assert!(
        carried,
        "a carried stint must diverge from the mid-window seed"
    );
}

/// `initial_soc` seeds the pack; the whole stint shifts with it (a lower start stays lower).
#[test]
fn f1_qss_stint_respects_the_initial_soc_seed() {
    let path = T0Path::from_track(&stadium_track(), 5.0);
    let high = hybrid("f1_2026", Some(0.85));
    let low = hybrid("f1_2026", Some(0.45));
    let ers_h = ErsCoupling::assemble(&high.resolved.spec, &high.t0, Policy::RuleBased, false)
        .unwrap()
        .unwrap();
    let ers_l = ErsCoupling::assemble(&low.resolved.spec, &low.t0, Policy::RuleBased, false)
        .unwrap()
        .unwrap();
    let r_hi = solve_stint(&plan(&high, &ers_h, &path), 3, StintSeeds::default()).unwrap();
    let r_lo = solve_stint(&plan(&low, &ers_l, &path), 3, StintSeeds::default()).unwrap();
    assert_eq!(r_hi.laps[0].slow.as_ref().unwrap().state_of_charge[0], 0.85);
    assert_eq!(r_lo.laps[0].slow.as_ref().unwrap().state_of_charge[0], 0.45);
}

/// Determinism: the same plan run twice is bit-identical (counter-based state, fixed reductions).
#[test]
fn f1_qss_stint_is_deterministic() {
    let h = hybrid("f1_2026", None);
    let path = T0Path::from_track(&stadium_track(), 5.0);
    let ers = ErsCoupling::assemble(&h.resolved.spec, &h.t0, Policy::RuleBased, false)
        .unwrap()
        .unwrap();
    let a = solve_stint(&plan(&h, &ers, &path), 4, StintSeeds::default()).unwrap();
    let b = solve_stint(&plan(&h, &ers, &path), 4, StintSeeds::default()).unwrap();
    for (la, lb) in a.laps.iter().zip(&b.laps) {
        assert_eq!(la.lap_time_s, lb.lap_time_s);
        assert_eq!(la.terminal.pack.unwrap().soc, lb.terminal.pack.unwrap().soc);
        assert_eq!(
            la.slow.as_ref().unwrap().state_of_charge,
            lb.slow.as_ref().unwrap().state_of_charge
        );
    }
}
