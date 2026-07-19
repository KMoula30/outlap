// SPDX-License-Identifier: AGPL-3.0-only
//! M6 PR5 — the QSS fuel-mass slow-state core (§8.1, D-M6-4). Two levels, cargo-testable:
//!
//! 1. **Assembly folds fuel at the full-tank reference m₀** — a `fuel:` block makes the T1/T0
//!    assembly mass `chassis.mass_kg + initial_kg` at the mass-weighted full-tank CG, so the
//!    envelope is built at m₀ (the correction is 1.0 at lap start, D-M6-4b). No block ⇒ unchanged.
//! 2. **The per-station mass/CG coupling makes a lighter car lap faster** — the point-mass solve
//!    fed a decreasing mass slice returns a faster lap than the full-tank solve (the "stint gets
//!    faster as the tank drains" physics, driven through the real solver + envelope corrections).
#![allow(clippy::float_cmp)]

use outlap_core::GriddedTable;
use outlap_qss::fuel::{FuelCoupling, FuelModel};
use outlap_qss::path::T0Path;
use outlap_qss::solver::{solve_into_ggv, solve_into_ggv_coupled};
use outlap_qss::{
    solve_stint, GgvEnvelope, LapRequest, LineDescriptor, StintPlan, StintSeeds, T0Options,
    T0Vehicle, T1Vehicle,
};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::FsLoader;
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling, Tier};
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::vehicle::Fuel;
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_vehicle, Conditions, LoadOptions, ResolvedVehicle};
use outlap_track::Track;

/// A synthetic ICE brake-thermal efficiency map over the `ice_v6` grid (constant 0.33 — the ~33 %
/// pump-fuel figure) so the fuel burn is live in a cargo test (the shipped `.parquet` sidecar is not
/// committed). Axes `speed_rpm × torque_nm`, one `efficiency` value column.
fn ice_eff_table() -> GriddedTable<f64> {
    let speeds = [1000.0, 4000.0, 8000.0, 12000.0, 15000.0];
    let torques = [0.0, 100.0, 200.0, 300.0, 400.0];
    let mut speed_col = Vec::new();
    let mut torque_col = Vec::new();
    let mut eff_col = Vec::new();
    for &s in &speeds {
        for &t in &torques {
            speed_col.push(s);
            torque_col.push(t);
            eff_col.push(0.33);
        }
    }
    GriddedTable::from_long(
        &[
            ("speed_rpm".to_owned(), speed_col),
            ("torque_nm".to_owned(), torque_col),
            ("efficiency".to_owned(), eff_col),
        ],
        &["speed_rpm", "torque_nm"],
    )
    .unwrap()
}

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

fn oval() -> Track {
    let r = 70.0;
    let straight = 500.0;
    let n_str = 100;
    let n_arc = 50;
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
        name: "oval".into(),
        closed: true,
        centerline: outlap_schema::refs::CenterlineRef("m".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows }).unwrap()
}

fn resolved_f1() -> ResolvedVehicle {
    load_vehicle("f1_2026/vehicle.yaml", &fixtures(), &LoadOptions::default())
        .expect("f1_2026 fixture resolves")
}

fn a_fuel_block() -> Fuel {
    Fuel {
        initial_kg: 80.0,
        tank_kg: 110.0,
        cg_offset_m: Some([-0.25, -0.05]),
        lhv_j_per_kg: 43.0e6,
        flow_limit: None,
    }
}

/// Assembly builds the T1 (and T0) vehicle at the FULL-TANK reference m₀ = `chassis.mass_kg` +
/// `initial_kg`, at the mass-weighted full-tank CG; without a `fuel:` block the mass/CG are the raw
/// chassis values (byte-identical to pre-M6).
#[test]
fn assembly_folds_fuel_at_full_tank_reference() {
    let loader = fixtures();
    let dry = resolved_f1();
    let dry_mass = dry.spec.chassis.mass_kg;
    let dry_a_f = dry.spec.chassis.cg[0];

    let t1_dry = T1Vehicle::assemble(&dry, &Conditions::default(), &loader, true).unwrap();
    assert_eq!(t1_dry.mass_kg, dry_mass, "no fuel ⇒ raw chassis mass");
    assert_eq!(t1_dry.a_f, dry_a_f, "no fuel ⇒ raw chassis CG");

    let mut fueled = resolved_f1();
    fueled.spec.fuel = Some(a_fuel_block());
    let t1_wet = T1Vehicle::assemble(&fueled, &Conditions::default(), &loader, true).unwrap();
    assert!(
        (t1_wet.mass_kg - (dry_mass + 80.0)).abs() < 1e-9,
        "full-tank mass m₀ = dry + initial ({} vs {})",
        t1_wet.mass_kg,
        dry_mass + 80.0
    );
    // Tank is rearward of the dry CG (offset −x) ⇒ the full-tank CG sits further back (a_f grows).
    assert!(
        t1_wet.a_f > dry_a_f,
        "full-tank CG shifts rearward toward the tank ({} vs dry {})",
        t1_wet.a_f,
        dry_a_f
    );

    // The T0 point-mass reference mass is m₀ too (so F/m starts heavy).
    let opts = T0Options {
        allow_degraded: true,
        ..T0Options::default()
    };
    let t0_wet = T0Vehicle::assemble(&fueled, &Conditions::default(), &loader, &opts).unwrap();
    assert!((t0_wet.mass_kg - (dry_mass + 80.0)).abs() < 1e-9);
}

/// The per-station mass/CG coupling: a car carrying LESS fuel (a lighter per-station mass slice)
/// laps FASTER than the same car at the full-tank reference — the D-M6-4 "stint gets faster as the
/// tank drains" physics, driven through the real g-g-g-v solver + the Decision-#31 corrections. And
/// the full-tank slice reproduces the uncoupled reference lap bit-for-bit (correction ≡ 1.0 at m₀).
#[test]
fn lighter_mass_slice_laps_faster_and_full_tank_is_identity() {
    let loader = fixtures();
    let mut fueled = resolved_f1();
    fueled.spec.fuel = Some(a_fuel_block());
    let t1 = T1Vehicle::assemble(&fueled, &Conditions::default(), &loader, true).unwrap();
    let res = EnvelopeRes {
        v_points: 6,
        ax_points: 5,
        g_normal_points: 2,
    };
    let env = GgvEnvelope::generate(&t1, &res, FzCoupling::OneStepLag).unwrap();
    let opts = T0Options {
        allow_degraded: true,
        ..T0Options::default()
    };
    let t0 = T0Vehicle::assemble(&fueled, &Conditions::default(), &loader, &opts).unwrap();
    let path = T0Path::from_track(&oval(), 5.0);
    let n = path.len();

    // Reference (uncoupled) lap at the full-tank m₀.
    let mut ws_ref = outlap_qss::result::T0Workspace::for_path(&path);
    let lt_ref = solve_into_ggv(&t0, &env, &path, &mut ws_ref).unwrap();

    // Full-tank mass/CG slice (mass = m₀, CG = the envelope's reference) — must be bit-identical.
    let m0 = env.mass_ref();
    let a_f0 = env.a_f_ref();
    let h0 = env.h_cg_ref();
    let mass_full = vec![m0; n];
    let af_full = vec![a_f0; n];
    let hcg_full = vec![h0; n];
    let mut ws_full = outlap_qss::result::T0Workspace::for_path(&path);
    let lt_full = solve_into_ggv_coupled(
        &t0,
        &env,
        None,
        None,
        None,
        Some((&mass_full, &af_full, &hcg_full)),
        None,
        &path,
        &mut ws_full,
    )
    .unwrap();
    assert_eq!(
        lt_ref.to_bits(),
        lt_full.to_bits(),
        "full-tank slice is bit-identical to the uncoupled reference (correction ≡ 1.0 at m₀)"
    );

    // A lighter car (30 kg of fuel burned) at the migrated CG ⇒ a faster lap.
    let fm = outlap_qss::fuel::FuelModel::from_spec(&fueled.spec).unwrap();
    let light_fuel = 50.0; // 30 kg lighter than the 80 kg full tank
    let mass_light = vec![fm.mass_at(light_fuel); n];
    let (af_l, hcg_l) = fm.cg_at(light_fuel);
    let af_light = vec![af_l; n];
    let hcg_light = vec![hcg_l; n];
    let mut ws_light = outlap_qss::result::T0Workspace::for_path(&path);
    let lt_light = solve_into_ggv_coupled(
        &t0,
        &env,
        None,
        None,
        None,
        Some((&mass_light, &af_light, &hcg_light)),
        None,
        &path,
        &mut ws_light,
    )
    .unwrap();
    assert!(
        lt_light < lt_full,
        "a 30 kg-lighter car laps faster ({lt_light} vs full-tank {lt_full})"
    );
}

/// The live burn end-to-end (§8.1, D-M6-4): with a synthetic ICE efficiency map installed, a
/// multi-lap f1 stint burns fuel — the tank mass falls monotonically lap-over-lap — and the lap
/// time drops as the car gets lighter (the "stint starts heavy, gets faster" acceptance). No ERS /
/// electro stack here: the ICE covers all traction, so the whole drive force burns fuel.
#[test]
fn f1_stint_burns_fuel_and_gets_faster() {
    let loader = fixtures();
    let mut fueled = resolved_f1();
    fueled.spec.fuel = Some(a_fuel_block());
    let mut t1 = T1Vehicle::assemble(&fueled, &Conditions::default(), &loader, true).unwrap();
    // Install the synthetic ICE efficiency map on the ICE drive unit (index 0) so the burn is live.
    t1.install_powertrain_maps(0, &ice_eff_table()).unwrap();
    let res = EnvelopeRes {
        v_points: 6,
        ax_points: 5,
        g_normal_points: 2,
    };
    let env = GgvEnvelope::generate(&t1, &res, FzCoupling::OneStepLag).unwrap();
    let opts = T0Options {
        allow_degraded: true,
        ..T0Options::default()
    };
    let t0 = T0Vehicle::assemble(&fueled, &Conditions::default(), &loader, &opts).unwrap();
    let path = T0Path::from_track(&oval(), 5.0);
    let fm = FuelModel::from_spec(&fueled.spec).unwrap();
    let fuel = FuelCoupling {
        model: fm,
        vehicle: &t1,
    };
    let plan = StintPlan {
        tier: Tier::T0,
        t0: &t0,
        t1: &t1,
        env: &env,
        path: &path,
        electro: None,
        ers: None,
        base_march: None,
        fuel: Some(&fuel),
        request: LapRequest {
            line: LineDescriptor::Centerline,
            resolved_hash: String::new(),
            notes: vec![],
            fz_coupling: FzCoupling::OneStepLag,
            flat_track: false,
        },
    };
    let n_laps = 5;
    let result = solve_stint(&plan, n_laps, StintSeeds::default()).unwrap();
    assert_eq!(result.laps.len(), n_laps);

    // Terminal fuel falls every lap (monotone burn), starting below the 80 kg full tank.
    let mut prev_fuel = fm.initial_kg;
    for (i, lap) in result.laps.iter().enumerate() {
        let term = lap
            .terminal
            .fuel_kg
            .expect("a fuel coupling surfaces terminal fuel");
        assert!(
            term < prev_fuel,
            "lap {i}: terminal fuel {term} must fall below the entry {prev_fuel}"
        );
        assert!(
            term > 0.0,
            "lap {i}: the tank does not run dry in this stint"
        );
        prev_fuel = term;
    }

    // The car gets lighter, so the last lap is faster than the first (strictly).
    let first = result.laps.first().unwrap().lap_time_s;
    let last = result.laps.last().unwrap().lap_time_s;
    assert!(
        last < first,
        "the stint gets faster as the tank drains (lap {n_laps} {last} < lap 1 {first})"
    );
}

/// The lower heating value the fuel block declares, J/kg.
const LHV: f64 = 43.0e6;

/// Solve one f1 lap with the given fuel flow limit and return `(lap_time, fuel_burned_kg,
/// max_fuel_energy_rate_w)` — the peak per-segment fuel-energy rate `ṁ·LHV` reconstructed from the
/// per-station fuel-mass channel and the speed profile.
fn solve_one_lap(flow: Option<outlap_schema::vehicle::FuelFlowLimit>) -> (f64, f64, f64) {
    let loader = fixtures();
    let mut fueled = resolved_f1();
    let mut fb = a_fuel_block();
    fb.flow_limit = flow;
    fueled.spec.fuel = Some(fb);
    let mut t1 = T1Vehicle::assemble(&fueled, &Conditions::default(), &loader, true).unwrap();
    t1.install_powertrain_maps(0, &ice_eff_table()).unwrap();
    let res = EnvelopeRes {
        v_points: 6,
        ax_points: 5,
        g_normal_points: 2,
    };
    let env = GgvEnvelope::generate(&t1, &res, FzCoupling::OneStepLag).unwrap();
    let opts = T0Options {
        allow_degraded: true,
        ..T0Options::default()
    };
    let t0 = T0Vehicle::assemble(&fueled, &Conditions::default(), &loader, &opts).unwrap();
    let path = T0Path::from_track(&oval(), 5.0);
    let fm = FuelModel::from_spec(&fueled.spec).unwrap();
    let fuel = FuelCoupling {
        model: fm,
        vehicle: &t1,
    };
    let plan = StintPlan {
        tier: Tier::T0,
        t0: &t0,
        t1: &t1,
        env: &env,
        path: &path,
        electro: None,
        ers: None,
        base_march: None,
        fuel: Some(&fuel),
        request: LapRequest {
            line: LineDescriptor::Centerline,
            resolved_hash: String::new(),
            notes: vec![],
            fz_coupling: FzCoupling::OneStepLag,
            flat_track: false,
        },
    };
    let result = solve_stint(&plan, 1, StintSeeds::default()).unwrap();
    let lap = &result.laps[0];
    let burned = fm.initial_kg - lap.terminal.fuel_kg.unwrap();
    // Peak per-segment fuel-energy rate: ṁ_seg = Δfuel / dt, dt = 2·ds/(v_i+v_j).
    let f = &lap.fuel.as_ref().unwrap().fuel_mass_kg;
    let v = &lap.v;
    let n = v.len();
    let mut max_rate = 0.0_f64;
    for i in 0..path.segments() {
        let j = if path.closed { (i + 1) % n } else { i + 1 };
        let dt = 2.0 * path.ds / (v[i] + v[j]).max(1e-6);
        let mdot = ((f[i] - f[j]) / dt).max(0.0);
        max_rate = max_rate.max(mdot * LHV);
    }
    (lap.lap_time_s, burned, max_rate)
}

/// The FIA fuel-flow ceiling (§8.1, D-M6-5) BINDS ON POWER: a tight flat limit shrinks the ICE
/// traction envelope, so the lap is slower and burns less fuel than the un-limited car — and the
/// honest per-station `ṁ` accounting stays at/below the imposed energy ceiling (the §14 closure the
/// plan calls out: the envelope is shrunk, the `ṁ` bookkeeping is NEVER clamped).
#[test]
fn flow_limit_binds_on_power_and_the_burn_respects_the_ceiling() {
    let (t_free, burn_free, _rate_free) = solve_one_lap(None);
    // A tight 1500 MJ/h flat ceiling: η·EF ≈ 0.33·417 kW ≈ 138 kW of crank power, well below the
    // f1's high-speed mechanical ceiling, so it binds hard on the straights.
    let ef_limit_w = 1500.0e6 / 3600.0;
    let (t_lim, burn_lim, rate_lim) = solve_one_lap(Some(outlap_schema::vehicle::FuelFlowLimit {
        mj_per_h: 1500.0,
        rpm_line: None,
    }));
    // (1) Binds on power: the shrunk envelope makes the lap strictly slower.
    assert!(
        t_lim > t_free * 1.02,
        "the flow limit shrinks the envelope: capped lap {t_lim} s vs free {t_free} s"
    );
    // (2) Less work ⇒ less fuel (the car cannot draw the power it otherwise would).
    assert!(
        burn_lim < burn_free,
        "the capped car burns less fuel: {burn_lim} kg vs {burn_free} kg"
    );
    // (3) §14 closure: the honest ṁ accounting stays within the imposed ceiling — the flow limit is
    // realised by shrinking the traction envelope, NOT by clamping the burn. A small tolerance
    // absorbs the central-vs-forward ax reconstruction and the representative-vs-map η spread.
    assert!(
        rate_lim <= ef_limit_w * 1.10,
        "peak fuel-energy rate {rate_lim} W must respect the {ef_limit_w} W ceiling"
    );
}
