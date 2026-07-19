// SPDX-License-Identifier: AGPL-3.0-only
//! M6 PR2 — the QSS energy-manager march: budget-enforced deployment, the five-ceiling harvest
//! chain, recharge phases, the lap ledger, and the no-ers bit-identity contract.
//!
//! The fixture vehicles run through the REAL load pipeline (including the new ers↔battery
//! cross-check): `f1_2026` (350 kW / 8.5 MJ, override + recharge phases) and `gt_hybrid`
//! (120 kW / 3 MJ, absent override / absent recharge fields — the D-M6-12 Option paths).
#![allow(
    clippy::doc_markdown,
    clippy::similar_names,
    clippy::many_single_char_names,
    // Physics kernels index parallel SoA arrays by station (the crate convention).
    clippy::needless_range_loop
)]
// Bit-exactness IS the assertion in the A/B and determinism tests.
#![allow(clippy::float_cmp)]

use std::f64::consts::PI;

use outlap_core::GriddedTable;
use outlap_powertrain::{DecideInput, ErsRulebook, LapEnergyLedger, Policy};
use outlap_qss::path::T0Path;
use outlap_qss::solver::solve_into_ggv_coupled;
use outlap_qss::{
    solve_t0, Couplings, ErsCoupling, GgvEnvelope, LapRequest, LineDescriptor, Pack, PackState,
    QssError, QssLap, SlowCoupling, T0Options, T0Vehicle, T0Workspace, T1Vehicle,
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

/// A closed "stadium" test track: two straights joined by two half-circles, so a lap has real
/// braking zones (harvest), corner exits (part throttle), and straights (deploy / super-clip).
fn stadium_track() -> Track {
    let r = 60.0;
    let straight = 700.0;
    let n_str = 140; // stations per straight
    let n_arc = 60; // stations per half-circle
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
    // Bottom straight, left → right.
    for i in 0..n_str {
        let x = straight * f64::from(i) / f64::from(n_str);
        rows.push(push(s, x, -r));
        s += straight / f64::from(n_str);
    }
    // Right half-circle, bottom → top.
    for i in 0..n_arc {
        let th = -PI / 2.0 + PI * f64::from(i) / f64::from(n_arc);
        rows.push(push(s, straight + r * th.cos(), r * th.sin()));
        s += PI * r / f64::from(n_arc);
    }
    // Top straight, right → left.
    for i in 0..n_str {
        let x = straight * (1.0 - f64::from(i) / f64::from(n_str));
        rows.push(push(s, x, r));
        s += straight / f64::from(n_str);
    }
    // Left half-circle, top → bottom.
    for i in 0..n_arc {
        let th = PI / 2.0 + PI * f64::from(i) / f64::from(n_arc);
        rows.push(push(s, r * th.cos(), r * th.sin()));
        s += PI * r / f64::from(n_arc);
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

/// Assemble a fixture hybrid end-to-end: resolved vehicle (through the REAL pipeline, so the
/// ers↔battery cross-check runs), T1 + small envelope + T0, the pack, and the manager coupling.
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
    .expect("fixture hybrid resolves (incl. the ers↔battery cross-check)");
    // allow_degraded: the gt_hybrid fixture has no constant-aero block (zero-aero fallback).
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

fn plain_request() -> LapRequest {
    LapRequest {
        line: LineDescriptor::Centerline,
        resolved_hash: String::new(),
        notes: vec![],
        fz_coupling: FzCoupling::OneStepLag,
        flat_track: false,
    }
}

/// Solve a managed lap on the stadium for a fixture hybrid.
fn managed_lap(h: &Hybrid, override_active: bool) -> (QssLap, T0Path) {
    let track = stadium_track();
    let path = T0Path::from_track(&track, 5.0);
    let electro = SlowCoupling {
        vehicle: &h.t1,
        thermal: None,
        pack: h.pack.clone(),
        pack_state: h.state,
        active: h.t1.has_energy_maps(),
    };
    let ers = ErsCoupling::assemble(&h.resolved.spec, &h.t0, Policy::RuleBased, override_active)
        .unwrap()
        .expect("ers block present");
    let lap = solve_t0(
        &h.t0,
        h.env.clone(),
        &Couplings {
            electro: Some(&electro),
            tire: None,
            ers: Some(&ers),
            fuel: None,
        },
        &path,
        plain_request(),
    )
    .unwrap();
    (lap, path)
}

/// Per-segment dt on the solved profile — the march's own formula (`2·ds/(v_i+v_j)`).
fn segment_dts(path: &T0Path, v: &[f64]) -> Vec<f64> {
    let n = path.len();
    (0..path.segments())
        .map(|i| {
            let j = if path.closed { (i + 1) % n } else { i + 1 };
            2.0 * path.ds / (v[i] + v[j]).max(1e-6)
        })
        .collect()
}

// ---------------------------------------------------------------------------------------------
// The physics of the managed lap.
// ---------------------------------------------------------------------------------------------

#[test]
fn f1_lap_deploys_under_the_curve_and_harvests_under_braking() {
    let h = hybrid("f1_2026", None);
    let (lap, path) = managed_lap(&h, false);

    let slow = lap.slow.as_ref().expect("managed lap has slow channels");
    let ers = slow.ers.as_ref().expect("managed lap has ers channels");
    let rb: ErsRulebook<f64> =
        ErsRulebook::from_schema(h.resolved.spec.ers.as_ref().unwrap(), None).unwrap();

    // Deployment: happens, and never exceeds the regulation curve at the station speed.
    let total_deploy: f64 = ers.deploy_power_w.iter().sum();
    assert!(total_deploy > 0.0, "an f1 lap must deploy");
    for (i, (&p, &v)) in ers.deploy_power_w.iter().zip(&lap.lap.v).enumerate() {
        let cap = rb.deploy_cap_electrical_w(v, false);
        assert!(
            p <= cap + 1e-6,
            "station {i}: deploy {p:.0} W exceeds the C5.2.8 curve {cap:.0} W at v={v:.1}"
        );
    }

    // Harvest: happens under braking, is bounded by the per-lap Recharge budget, and charges the
    // pack (SoC rises across a harvesting segment).
    assert!(
        ers.ledger_harvest_j > 0.0,
        "an f1 lap with braking zones must harvest"
    );
    assert!(
        ers.ledger_harvest_j <= rb.harvest_budget_j(false) + 1e-6,
        "harvest {:.3} MJ exceeds the Recharge budget",
        ers.ledger_harvest_j * 1e-6
    );
    let soc = &slow.state_of_charge;
    let charged = (0..path.segments().min(soc.len() - 1))
        .any(|i| ers.harvest_power_w[i] > 0.0 && soc[i + 1] > soc[i]);
    assert!(charged, "a harvesting segment must raise the entry SoC");

    // The SoC trace is physical, stays inside the usable window (the C5.2.9 clamp), and the
    // recorded swing brackets the visible channel. The stats see every post-step state; the
    // channel logs ENTRY states, so on a closed lap it misses the final segment — the stats
    // therefore bracket the channel min/max, never sit inside it.
    let [win_lo, win_hi] = h.pack.soc_window();
    assert!(soc
        .iter()
        .all(|s| (win_lo - 1e-9..=win_hi + 1e-9).contains(s)));
    let (lo, hi) = soc
        .iter()
        .fold((f64::MAX, f64::MIN), |(lo, hi), &s| (lo.min(s), hi.max(s)));
    assert!(
        ers.soc_min <= lo + 1e-12 && ers.soc_max >= hi - 1e-9,
        "recorded swing [{:.4}, {:.4}] must bracket the channel [{lo:.4}, {hi:.4}]",
        ers.soc_min,
        ers.soc_max
    );
    // The on-track SoC swing is bounded by the usable window span — for f1 the FIA C5.2.9 ≤ 4 MJ
    // recharge window, now clamped rather than only recorded (per the recharge-to-top default).
    assert!(
        ers.soc_max - ers.soc_min <= (win_hi - win_lo) + 1e-9,
        "on-track swing {:.4} exceeds the usable window span {:.4}",
        ers.soc_max - ers.soc_min,
        win_hi - win_lo
    );
}

#[test]
fn the_c5_2_9_swing_clip_is_independent_of_the_physical_window() {
    // The FIA C5.2.9 on-track swing limit is a REGULATION, enforced independently of the pack's
    // PHYSICAL usable window. Give the f1 pack (physical window [0.2, 0.9] = 4 MJ) a TIGHTER reg
    // swing limit of 2 MJ, then drain it: the swing must clip at the 2 MJ reg band (≈ 0.35 SoC),
    // NOT at the physical 0.7 span — the pack stops discharging well ABOVE its physical floor.
    let mut h = hybrid("f1_2026", None);
    let e_total = h.pack.total_energy_j();
    let swing_mj = 2.0;
    h.resolved.spec.ers.as_mut().unwrap().es.capacity_mj = swing_mj; // 2 MJ < the 4 MJ window

    let spec = {
        let mut s = h.resolved.spec.clone();
        s.ers.as_mut().unwrap().recovery.recharge_phases = false;
        s
    };
    let ers = ErsCoupling::assemble(&spec, &h.t0, Policy::RuleBased, true)
        .unwrap()
        .unwrap();
    assert!((ers.swing_limit_j - swing_mj * 1e6).abs() < 1.0);
    let path = T0Path::from_track(&stadium_track(), 5.0);
    let electro = SlowCoupling {
        vehicle: &h.t1,
        thermal: None,
        pack: h.pack.clone(),
        pack_state: PackState {
            soc: 0.9,
            ..h.state
        },
        active: h.t1.has_energy_maps(),
    };
    let lap = solve_t0(
        &h.t0,
        h.env.clone(),
        &Couplings {
            electro: Some(&electro),
            tire: None,
            ers: Some(&ers),
            fuel: None,
        },
        &path,
        plain_request(),
    )
    .unwrap();
    let ers_log = lap.slow.unwrap().ers.unwrap();
    let swing_soc = swing_mj * 1e6 / e_total;
    // The recorded swing respects the reg limit (a hair of discretisation tolerance)...
    assert!(
        ers_log.soc_max - ers_log.soc_min <= swing_soc + 2e-3,
        "swing {:.4} exceeds the {swing_mj} MJ reg band {swing_soc:.4}",
        ers_log.soc_max - ers_log.soc_min
    );
    // ...and it is genuinely the REG limit biting, not the physical window: the pack drains from
    // 0.9 but stops far above the physical floor 0.2 (≈ 0.9 − 0.35 = 0.55).
    let [phys_lo, _] = h.pack.soc_window();
    assert!(
        ers_log.soc_min > phys_lo + 0.2,
        "reg clip must stop the drain ABOVE the physical floor {phys_lo}, got soc_min {:.4}",
        ers_log.soc_min
    );
}

#[test]
fn pack_soc_closes_against_the_ledger_over_the_lap() {
    // The spec's energy-closure gate ('ES out − ES in == Σ deploy − Σ harvest'), in two parts:
    //   (1) an independent re-march reproduces the PRODUCTION ledger — so the reported ledger is
    //       the algebra we think it is; and
    //   (2) the PRODUCTION reported SoC trace closes against the PRODUCTION ledger on a
    //       net-draining lap (below) — this reads the actual solve's channels, so a pack↔ledger
    //       sign/scale divergence (mutation-checked with a step-sign flip) breaks it.
    let h = hybrid("f1_2026", None);
    let (lap, path) = managed_lap(&h, false);
    let ers = lap.slow.as_ref().unwrap().ers.as_ref().unwrap();

    // Independently re-run the manager march reading the pack terminal state.
    let e = ErsCoupling::assemble(&h.resolved.spec, &h.t0, Policy::RuleBased, false)
        .unwrap()
        .unwrap();
    let mut st = h.state;
    let mut ledger = LapEnergyLedger::<f64>::new();
    let mut ax = vec![0.0; path.len()];
    derive_ax_like(&path, &lap.lap.v, &mut ax);
    let m = h.t1.mass_kg;
    let mut prev_k = 0.0;
    let mut ramp = 0.0;
    let soc0 = st.soc;
    for i in 0..path.segments() {
        let j = (i + 1) % path.len();
        let vi = lap.lap.v[i];
        let dt = 2.0 * path.ds / (vi + lap.lap.v[j]).max(1e-6);
        let f_req = m * (ax[i] + h.env.drag_accel(vi) + outlap_qss::G * path.sin_g[i]);
        let (dem, surplus, brake) = if f_req > 0.0 {
            let f_avail = h.t0.tractive_force(vi).max(1e-9);
            let raw = (f_req / f_avail).clamp(0.0, 1.0);
            let pmech = h.t0.mech_tractive_force(vi) * vi;
            if raw >= 0.98 {
                (1.0, pmech, 0.0)
            } else {
                (raw, (pmech - f_req * vi).max(0.0), 0.0)
            }
        } else {
            (
                0.0,
                0.0,
                e.max_regen_frac * e.regen_axle_share * (-f_req * vi).max(0.0),
            )
        };
        let inp = DecideInput {
            v: vi,
            driver_demand: dem,
            brake_demand_w: brake,
            mech_regen_envelope_w: e.p_mech_max_w * ErsCoupling::fade(vi),
            ice_surplus_w: surplus,
            soc: st.soc,
            override_active: false,
            prev_k_power_w: prev_k,
            ramp_reduced_w: ramp,
            dt,
            station: i,
        };
        let cmd = e.manager.decide(&inp, &ledger);
        // Realize against the pack exactly as the march does.
        let (dw, hw) = if cmd.deploy_w > 0.0 {
            let p = cmd.deploy_w.min(h.pack.discharge_power_limit_w(&st));
            let (_pm, pe) = h.t0.ers_realized_deploy_w(p);
            h.pack.step_power(&mut st, pe, dt);
            (pe, 0.0)
        } else if cmd.harvest_w > 0.0 {
            let p = cmd.harvest_w.min(h.pack.regen_power_limit_w(&st));
            h.pack.step_power(&mut st, -p, dt);
            (0.0, p)
        } else {
            h.pack.step_power(&mut st, 0.0, dt);
            (0.0, 0.0)
        };
        let realized = outlap_powertrain::ErsCommand {
            deploy_w: dw,
            harvest_w: hw,
            mode: cmd.mode,
        };
        ledger.record(&realized, dt);
        let k = dw - hw;
        if k < prev_k {
            ramp += prev_k - k;
        } else {
            ramp = 0.0;
        }
        prev_k = k;
    }
    // This independent march reproduces the reported ledger bit-for-close (same rules, same
    // profile) — so the PRODUCTION ledger is the algebra we think it is, not a coincidence.
    let _ = soc0;
    assert!(
        (ledger.deploy_j() - ers.ledger_deploy_j).abs() < 1.0,
        "independent deploy ledger {:.1} J vs reported {:.1} J",
        ledger.deploy_j(),
        ers.ledger_deploy_j
    );

    // (2) The PRODUCTION SoC trace closes against the PRODUCTION ledger on a net-draining lap.
    production_soc_closes_on_a_draining_lap(&h);
}

/// The well-conditioned half of the closure gate: a NET-DRAINING lap. A charge-sustaining lap
/// returns the pack near its seed, so `net = deploy − harvest` is a small difference of two large
/// ledgers dominated by loss/OCV-slope noise (a useless ratio). Seeding near the window TOP with
/// the recharge phases OFF and Override ON drains the pack monotonically, so the reported SoC
/// drift and the reported net ledger are both large; a wrong sign/scale in the pack stepping
/// breaks the band (mutation-checked with a step-sign flip). Reads the actual solve's channels.
fn production_soc_closes_on_a_draining_lap(h: &Hybrid) {
    let mut spec = h.resolved.spec.clone();
    spec.ers.as_mut().unwrap().recovery.recharge_phases = false;
    let drain = ErsCoupling::assemble(&spec, &h.t0, Policy::RuleBased, true)
        .unwrap()
        .unwrap();
    let dpath = T0Path::from_track(&stadium_track(), 5.0);
    let electro = SlowCoupling {
        vehicle: &h.t1,
        thermal: None,
        pack: h.pack.clone(),
        pack_state: PackState {
            soc: 0.9,
            ..h.state
        },
        active: h.t1.has_energy_maps(),
    };
    let dlap = solve_t0(
        &h.t0,
        h.env.clone(),
        &Couplings {
            electro: Some(&electro),
            tire: None,
            ers: Some(&drain),
            fuel: None,
        },
        &dpath,
        plain_request(),
    )
    .unwrap();
    let dslow = dlap.slow.unwrap();
    let ders = dslow.ers.unwrap();
    let e_total = h.pack.total_energy_j();
    // Net SoC drift over the lap (entry states; the un-logged final segment is negligible on a
    // multi-hundred-station lap that drains ~0.5 of the window). `+` = the store was drained.
    let soc = &dslow.state_of_charge;
    let d_store_j = (soc[0] - soc[soc.len() - 1]) * e_total;
    let net_elec_j = ders.ledger_deploy_j - ders.ledger_harvest_j;
    assert!(
        net_elec_j > 0.5e6,
        "the drain scenario must net-discharge for a well-conditioned check (net {:.3} MJ)",
        net_elec_j * 1e-6
    );
    // Discharging: the store loses AT LEAST the delivered electrical energy (I²R losses add to the
    // draw) and no more than ~2× it — a sign flip or a 2× scale error in the pack stepping breaks
    // this. Nominal `ΔSoC·e_pack_wh` vs exact `∫V·I dt` is absorbed by the generous upper bound.
    assert!(
        d_store_j >= net_elec_j - 1.0,
        "the pack's SoC fell {d_store_j:.0} J but the ledger drew a NET {net_elec_j:.0} J — the \
         reported SoC trace and the ledger disagree in sign/scale (a pack↔ledger divergence)"
    );
    assert!(
        d_store_j <= net_elec_j * 2.0,
        "the pack's SoC fell {d_store_j:.0} J, implausibly more than 2× the {net_elec_j:.0} J net draw"
    );
}

#[test]
fn the_ledger_closes_over_the_realized_channels() {
    let h = hybrid("f1_2026", None);
    let (lap, path) = managed_lap(&h, false);
    let slow = lap.slow.unwrap();
    let ers = slow.ers.unwrap();
    let dts = segment_dts(&path, &lap.lap.v);

    // Σ realized·dt over the stations reproduces the ledger integrals bit-for-bit (the ledger is
    // the only writer and accumulates in the same fixed order).
    let (mut deploy_j, mut harvest_j) = (0.0_f64, 0.0_f64);
    for (i, &dt) in dts.iter().enumerate() {
        deploy_j += ers.deploy_power_w[i] * dt;
        harvest_j += ers.harvest_power_w[i] * dt;
    }
    assert_eq!(deploy_j.to_bits(), ers.ledger_deploy_j.to_bits());
    assert_eq!(harvest_j.to_bits(), ers.ledger_harvest_j.to_bits());
}

#[test]
fn recharge_phases_only_harvest_below_the_target() {
    // Seed LOW so the automated Recharge paths (part-throttle / super-clip) are hungry.
    let h = hybrid("f1_2026", Some(0.30));
    let (lap, path) = managed_lap(&h, false);
    let slow = lap.slow.unwrap();
    let ers = slow.ers.unwrap();
    let rb: ErsRulebook<f64> =
        ErsRulebook::from_schema(h.resolved.spec.ers.as_ref().unwrap(), None).unwrap();
    let target = rb.recharge_target_soc();

    // Reconstruct drive stations from the solved profile (the march's own classification input).
    let m = h.t1.mass_kg;
    let mut ax = vec![0.0; path.len()];
    // Central-difference ax, matching the march.
    for i in 0..path.segments() {
        let j = (i + 1) % path.len();
        ax[i] = (lap.lap.v[j] * lap.lap.v[j] - lap.lap.v[i] * lap.lap.v[i]) / (2.0 * path.ds);
    }
    let mut drive_harvest = 0;
    for i in 0..path.segments() {
        let f_req = m
            * (ax[i] + h.env.drag_accel(lap.lap.v[i]) + outlap_qss::G * 0.0/* flat stadium: sin_g = 0 */);
        if f_req > 0.0 && ers.harvest_power_w[i] > 0.0 {
            drive_harvest += 1;
            assert!(
                slow.state_of_charge[i] < target,
                "station {i}: recharge-phase harvest at SoC {:.3} ≥ target {target:.3}",
                slow.state_of_charge[i]
            );
        }
    }
    assert!(
        drive_harvest > 0,
        "a SoC-poor lap with recharge phases must harvest on drive stations"
    );
}

#[test]
fn override_extends_the_envelope_and_the_harvest_bonus() {
    // Disable the recharge phases for the comparison: the charge-sustain policy would otherwise
    // confound it (more override deploy → SoC below target sooner → more super-clip "power
    // limited" running → a legitimately SLOWER lap, the real cost of Overtake overuse).
    let h = hybrid("f1_2026", Some(0.85));
    let mut spec = h.resolved.spec.clone();
    spec.ers.as_mut().unwrap().recovery.recharge_phases = false;
    let track = stadium_track();
    let path = T0Path::from_track(&track, 5.0);
    let solve = |override_active: bool| {
        let electro = SlowCoupling {
            vehicle: &h.t1,
            thermal: None,
            pack: h.pack.clone(),
            pack_state: h.state,
            active: h.t1.has_energy_maps(),
        };
        let ers = ErsCoupling::assemble(&spec, &h.t0, Policy::RuleBased, override_active)
            .unwrap()
            .unwrap();
        solve_t0(
            &h.t0,
            h.env.clone(),
            &Couplings {
                electro: Some(&electro),
                tire: None,
                ers: Some(&ers),
                fuel: None,
            },
            &path,
            plain_request(),
        )
        .unwrap()
    };
    let normal = solve(false);
    let over = solve(true);
    // The override envelope holds full power to a higher speed: the lap can only get faster.
    assert!(
        over.lap.lap_time_s <= normal.lap.lap_time_s + 1e-9,
        "override lap {:.4} s slower than normal {:.4} s",
        over.lap.lap_time_s,
        normal.lap.lap_time_s
    );
    // And the Recharge ledger may use the +0.5 MJ bonus (C5.2.10iii) — never exceeded.
    let rb: ErsRulebook<f64> =
        ErsRulebook::from_schema(h.resolved.spec.ers.as_ref().unwrap(), None).unwrap();
    let ers = over.slow.unwrap().ers.unwrap();
    assert!(ers.ledger_harvest_j <= rb.harvest_budget_j(true) + 1e-6);
}

#[test]
fn gt_hybrid_option_paths_wire_through_the_march() {
    // Absent override, absent recharge fields, decreasing mid-knot taper, 120 kW / 3 MJ budgets
    // (D-M6-12) — through the REAL pipeline (cross-check included) into a managed lap.
    let h = hybrid("gt_hybrid", None);
    let (lap, _) = managed_lap(&h, false);
    let slow = lap.slow.unwrap();
    let ers = slow.ers.unwrap();
    let rb: ErsRulebook<f64> =
        ErsRulebook::from_schema(h.resolved.spec.ers.as_ref().unwrap(), None).unwrap();

    for (&p, &v) in ers.deploy_power_w.iter().zip(&lap.lap.v) {
        assert!(p <= rb.deploy_cap_electrical_w(v, false) + 1e-6);
        assert!(p <= 120e3 + 1e-6, "deploy exceeds the 120 kW GT cap");
    }
    assert!(ers.ledger_harvest_j <= 3.0e6 + 1e-6, "3 MJ Recharge budget");
    // The override flag on a car WITHOUT an override block falls back to the base envelope.
    let (over, _) = managed_lap(&h, true);
    let ers_over = over.slow.unwrap().ers.unwrap();
    for (&p, &v) in ers_over.deploy_power_w.iter().zip(&over.lap.v) {
        assert!(p <= rb.deploy_cap_electrical_w(v, false) + 1e-6);
    }
}

// ---------------------------------------------------------------------------------------------
// The T0 pedal availability: piecewise-linear regulation curve through the 0.97 seam.
// ---------------------------------------------------------------------------------------------

#[test]
fn t0_deploy_force_is_piecewise_linear_between_the_knots() {
    let h = hybrid("f1_2026", None);
    let ers_block = h.resolved.spec.ers.as_ref().unwrap();
    let taper = &ers_block.deployment.taper_vs_speed;
    let cap_w = ers_block.deployment.power_limit_kw * 1e3;
    let eta = h.t0.ers_eta();
    let p_mech_max = h.t0.ers_p_mech_max_w();
    let factor = ers_block.elec_mech_factor.unwrap_or(0.97);

    // Random interior speeds across the taper domain (splitmix-style counter, no clock/rand).
    let mut z = 0x9E37_79B9_7F4A_7C15_u64;
    for _ in 0..2000 {
        z = z.wrapping_mul(0x2545_F491_4F6C_DD1D).rotate_left(17) ^ 0xDEAD_BEEF;
        let vmax_kph = *taper.speed_kph.last().unwrap();
        #[allow(clippy::cast_precision_loss)]
        let v_kph = (z % 10_000) as f64 / 10_000.0 * vmax_kph;
        let v = v_kph / 3.6;
        // Closed-form piecewise-LINEAR interpolation of the schema breakpoints.
        let frac = {
            let xs = &taper.speed_kph;
            let ys = &taper.power_frac;
            let k = xs.partition_point(|&x| x <= v_kph).clamp(1, xs.len() - 1);
            let t = (v_kph - xs[k - 1]) / (xs[k] - xs[k - 1]);
            (ys[k - 1] + t * (ys[k] - ys[k - 1])).clamp(0.0, 1.0)
        };
        let p_mech = (cap_w * frac * factor).min(p_mech_max).max(0.0);
        let expect = eta * p_mech / v.max(1.0);
        let got = h.t0.tractive_force(v) - h.t0.mech_tractive_force(v);
        assert!(
            (got - expect).abs() <= 1e-9 * expect.max(1.0),
            "at {v_kph:.2} km/h: ERS force {got:.3} N != closed-form {expect:.3} N \
             (a Hermite bows the flat-plateau breakpoints — Decision #30 exception)"
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Contracts: no-ers bit-identity, determinism, typed errors, convergence stability.
// ---------------------------------------------------------------------------------------------

/// The pre-PR2 electro march, inlined as the A/B oracle: full-draw attribution via
/// `traction_energy`, machine derate ∧ battery discharge cap into a scale, `step_power` on the
/// source power. A no-ers vehicle must reproduce it BIT-FOR-BIT through the new march.
#[allow(clippy::too_many_arguments)]
fn oracle_march(
    t1: &T1Vehicle,
    pack: &Pack,
    state: PackState,
    thermal: &outlap_qss::MachineThermal,
    env: &GgvEnvelope,
    path: &T0Path,
    v: &[f64],
    ax: &[f64],
    scale: &mut [f64],
    soc: &mut [f64],
) {
    let mut thermal = thermal.clone();
    let mut st = state;
    let pt = t1.powertrain();
    let m = t1.mass_kg;
    let n = path.len();
    scale.fill(1.0);
    for seg in 0..path.segments() {
        let i = seg;
        let j = if path.closed { (seg + 1) % n } else { seg + 1 };
        let vi = v[i];
        let dt = 2.0 * path.ds / (v[i] + v[j]).max(1e-6);
        soc[i] = st.soc;
        let f_drive = (m * (ax[i] + env.drag_accel(vi) + outlap_qss::G * path.sin_g[i])).max(0.0);
        let vdc = pack.terminal_voltage_v(&st);
        if let Some(te) = pt.traction_energy(vi, f_drive, Some(vdc)) {
            let derate = thermal
                .step(te.loss_w, |_| None, te.omega_rad_s, dt)
                .unwrap_or(1.0);
            let p_cap = pack.discharge_power_limit_w(&st);
            let batt_scale = if te.source_w > p_cap && te.source_w > 0.0 {
                (p_cap / te.source_w).clamp(0.0, 1.0)
            } else {
                1.0
            };
            let _ = pack.step_power(&mut st, te.source_w, dt);
            scale[i] = (derate.min(batt_scale)).clamp(0.0, 1.0);
        }
    }
    if !path.closed && n > 0 {
        soc[n - 1] = st.soc;
    }
}

#[test]
fn no_ers_ev_stack_matches_the_pre_pr2_march_bit_for_bit() {
    use outlap_qss::{MachineThermal, T1Powertrain};
    use outlap_schema::load::load_emotor;

    // The mapped-EV full stack (the pdt_du_rwd fixture) — the exact pre-PR2 coupled path.
    let loader = fixtures();
    let rv = load_vehicle("pdt_du_rwd/vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let mut t1 = T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap();
    let bytes = loader.load_bytes("pdt_synth_du_vdc.maps.parquet").unwrap();
    let table: GriddedTable<f64> =
        read_gridded_table(&bytes, &T1Powertrain::map_axis_names_vdc()).unwrap();
    t1.install_powertrain_maps(0, &table).unwrap();
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

    let res = EnvelopeRes {
        v_points: 6,
        ax_points: 5,
        g_normal_points: 2,
    };
    let env = GgvEnvelope::generate(&t1, &res, FzCoupling::OneStepLag).unwrap();
    let track = stadium_track();
    let path = T0Path::from_track(&track, 5.0);

    // A: the production march through the new Couplings surface.
    let coupling = SlowCoupling {
        vehicle: &t1,
        thermal: Some(thermal.clone()),
        pack: pack.clone(),
        pack_state: state,
        active: t1.has_energy_maps(),
    };
    let a = solve_t0(
        &t0,
        env.clone(),
        &Couplings {
            electro: Some(&coupling),
            ..Couplings::default()
        },
        &path,
        plain_request(),
    )
    .unwrap();

    // B: the pre-PR2 algorithm, inlined: iterate solve ↔ oracle-march exactly as solve_profile
    // does (iteration 0 uncoupled, OUTER_ITERS coupled re-solves, final reporting march).
    let n = path.len();
    let mut ws = T0Workspace::for_path(&path);
    let mut ax = vec![0.0; n];
    let mut scale = vec![1.0; n];
    let mut soc = vec![0.0; n];
    let mut lap_time = outlap_qss::solver::solve_into_ggv(&t0, &env, &path, &mut ws).unwrap();
    for _ in 0..outlap_qss::qss::OUTER_ITERS {
        derive_ax_like(&path, &ws.v, &mut ax);
        oracle_march(
            &t1, &pack, state, &thermal, &env, &path, &ws.v, &ax, &mut scale, &mut soc,
        );
        lap_time = solve_into_ggv_coupled(
            &t0,
            &env,
            Some(&scale),
            None,
            None,
            None,
            None,
            &path,
            &mut ws,
        )
        .unwrap();
    }
    derive_ax_like(&path, &ws.v, &mut ax);
    oracle_march(
        &t1, &pack, state, &thermal, &env, &path, &ws.v, &ax, &mut scale, &mut soc,
    );

    assert_eq!(a.lap.lap_time_s.to_bits(), lap_time.to_bits());
    assert_eq!(a.lap.v, ws.v);
    let slow = a.slow.expect("active EV stack reports slow channels");
    for (i, (&got, &want)) in slow.state_of_charge.iter().zip(&soc).enumerate() {
        assert_eq!(got.to_bits(), want.to_bits(), "SoC diverged at station {i}");
    }
    assert!(slow.ers.is_none(), "no manager ⇒ no ers channels");
}

/// The central-difference ax the march consumes (mirrors the private `derive_ax`).
fn derive_ax_like(path: &T0Path, v: &[f64], ax_out: &mut [f64]) {
    let n = v.len();
    for seg in 0..path.segments() {
        let i = seg;
        let j = if path.closed { (seg + 1) % n } else { seg + 1 };
        ax_out[i] = (v[j] * v[j] - v[i] * v[i]) / (2.0 * path.ds);
    }
}

#[test]
fn managed_lap_is_deterministic() {
    let h = hybrid("f1_2026", None);
    let (a, _) = managed_lap(&h, false);
    let (b, _) = managed_lap(&h, false);
    assert_eq!(a.lap.lap_time_s.to_bits(), b.lap.lap_time_s.to_bits());
    assert_eq!(a.lap.v, b.lap.v);
    let (sa, sb) = (a.slow.unwrap(), b.slow.unwrap());
    assert_eq!(sa.state_of_charge, sb.state_of_charge);
    let (ea, eb) = (sa.ers.unwrap(), sb.ers.unwrap());
    assert_eq!(ea.deploy_power_w, eb.deploy_power_w);
    assert_eq!(ea.harvest_power_w, eb.harvest_power_w);
    assert_eq!(ea.ledger_deploy_j.to_bits(), eb.ledger_deploy_j.to_bits());
}

#[test]
fn ers_coupling_without_a_pack_is_a_typed_error() {
    let h = hybrid("f1_2026", None);
    let track = stadium_track();
    let path = T0Path::from_track(&track, 5.0);
    let ers = ErsCoupling::assemble(&h.resolved.spec, &h.t0, Policy::RuleBased, false)
        .unwrap()
        .unwrap();
    let e = solve_t0(
        &h.t0,
        h.env.clone(),
        &Couplings {
            electro: None,
            tire: None,
            ers: Some(&ers),
            fuel: None,
        },
        &path,
        plain_request(),
    )
    .unwrap_err();
    assert!(matches!(e, QssError::ErsCouplingWithoutPack), "{e}");
}

#[test]
fn soc_starved_and_budget_exhausted_laps_stay_stable() {
    // SoC-starved: seed at the window floor — deployment must clamp to the pack's (zero)
    // discharge ceiling, the lap must complete, and the outer iteration must not limit-cycle.
    let h = hybrid("f1_2026", Some(0.2));
    let (lap, _) = managed_lap(&h, false);
    let slow = lap.slow.as_ref().unwrap();
    assert!(
        slow.convergence.dlap_s < 0.15,
        "SoC-starved lap oscillates: |Δlap| {:.3} s between the last two passes",
        slow.convergence.dlap_s
    );
    assert!(lap.lap.v.iter().all(|v| v.is_finite() && *v > 0.0));

    // Budget-exhausting: shrink the per-lap Recharge budget to a sliver — the ledger must clamp
    // exactly and the lap stays stable.
    let mut spec = h.resolved.spec.clone();
    spec.ers.as_mut().unwrap().recovery.per_lap_harvest_mj = 0.05;
    let track = stadium_track();
    let path = T0Path::from_track(&track, 5.0);
    let electro = SlowCoupling {
        vehicle: &h.t1,
        thermal: None,
        pack: h.pack.clone(),
        pack_state: h.state,
        active: h.t1.has_energy_maps(),
    };
    let ers = ErsCoupling::assemble(&spec, &h.t0, Policy::RuleBased, false)
        .unwrap()
        .unwrap();
    let lap = solve_t0(
        &h.t0,
        h.env.clone(),
        &Couplings {
            electro: Some(&electro),
            tire: None,
            ers: Some(&ers),
            fuel: None,
        },
        &path,
        plain_request(),
    )
    .unwrap();
    let slow = lap.slow.unwrap();
    let ers_log = slow.ers.unwrap();
    assert!(
        ers_log.ledger_harvest_j <= 0.05e6 + 1e-6,
        "harvest {:.4} MJ exceeds the 0.05 MJ budget",
        ers_log.ledger_harvest_j * 1e-6
    );
    assert!(slow.convergence.dlap_s < 0.15);
}

#[test]
fn managed_lap_records_the_manager_notes() {
    let h = hybrid("f1_2026", None);
    let (lap, _) = managed_lap(&h, false);
    let joined = lap.lap.notes.join("\n");
    assert!(
        joined.contains("2026 ERS energy manager active"),
        "missing the manager note: {joined}"
    );
    assert!(
        joined.contains("recorded per FIA C5.2.9"),
        "missing the SoC-swing record: {joined}"
    );
    assert!(
        joined.contains("outer-iteration convergence"),
        "missing the convergence record: {joined}"
    );
    assert_eq!(lap.tier, Tier::T0);
}
