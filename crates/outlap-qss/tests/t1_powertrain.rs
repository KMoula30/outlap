// SPDX-License-Identifier: AGPL-3.0-only
//! PR4 — topology powertrain in the T1 traction limit (§8.0–8.2, §10.5, §13).
//!
//! Covers the drivetrain-graph contract end to end: coupler torque conservation, the differential
//! torque split (open = equal torque, locked = equal speed, LSD in between), the split fractions
//! summing to one, energy closure through the efficiency/loss maps, the PDT round-trip gate
//! (importer-emitted `.ptm` + parquet reproduces spot efficiencies to 1e-6 through the real
//! `GriddedMapN`), the diff genuinely shaping the trim's per-wheel slip, and the traction ceiling.
#![allow(clippy::doc_markdown, clippy::similar_names, clippy::float_cmp)]

use outlap_core::GriddedTable;
use outlap_qss::{DiffModel, T1Powertrain, T1Vehicle, TrimInput};
use outlap_schema::io::{FsLoader, MemLoader, SourceLoader};
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::vehicle::DiffKind;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};

const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

fn assemble(name: &str) -> T1Vehicle {
    let loader = fixtures();
    let rv = load_vehicle(name, &loader, &LoadOptions::default()).unwrap();
    T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap()
}

/// Decode a `.ptm` sidecar parquet through the shared sidecar reader and install it onto a unit.
fn install_maps(car: &mut T1Vehicle, unit: usize, parquet_path: &str) {
    let bytes = fixtures().load_bytes(parquet_path).unwrap();
    let names = T1Powertrain::map_axis_names();
    let table: GriddedTable<f64> = read_gridded_table(&bytes, &names).unwrap();
    car.install_powertrain_maps(unit, &table).unwrap();
}

fn diff(kind: DiffKind, preload_nm: f64, ramp: f64) -> DiffModel {
    DiffModel {
        kind,
        preload_nm,
        ramp_accel: ramp,
        ramp_decel: ramp,
    }
}

// --- Differential torque split (§8.2) -----------------------------------------------------------

#[test]
fn diff_split_open_is_equal_torque_locked_is_grip_proportional() {
    // Open: both output shafts carry equal torque regardless of grip (the lesser-grip wheel caps the
    // deliverable axle torque). Locked/solid: grip-proportional, summing to the axle torque.
    let (t, cl, cr) = (200.0, 60.0, 140.0);
    let (ol, or) = diff(DiffKind::Open, 0.0, 0.0).split(t, cl, cr);
    assert!((ol - or).abs() < 1e-12, "open ⇒ equal torque");
    assert!((ol + or - t).abs() < 1e-12, "split sums to the axle torque");

    for kind in [DiffKind::Locked, DiffKind::Solid] {
        let (l, r) = diff(kind, 0.0, 0.0).split(t, cl, cr);
        assert!(
            (l - t * cl / (cl + cr)).abs() < 1e-12,
            "{kind:?} grip-proportional left"
        );
        assert!(
            (r - t * cr / (cl + cr)).abs() < 1e-12,
            "{kind:?} grip-proportional right"
        );
        assert!(
            (l + r - t).abs() < 1e-12,
            "{kind:?} sums to the axle torque"
        );
    }
}

#[test]
fn diff_split_lsd_sits_between_open_and_locked() {
    // LSD: grip-proportional but with the side-to-side difference clamped to preload + ramp·|t|.
    let (t, cl, cr) = (200.0, 40.0, 160.0);
    let open = diff(DiffKind::Open, 0.0, 0.0).split(t, cl, cr); // (100, 100)
    let locked = diff(DiffKind::Locked, 0.0, 0.0).split(t, cl, cr); // (40, 160)
    let lsd = diff(DiffKind::Lsd, 20.0, 0.30).split(t, cl, cr); // bias cap = 20 + 0.30·200 = 80
    assert!(
        (lsd.0 + lsd.1 - t).abs() < 1e-12,
        "LSD sums to the axle torque"
    );
    assert!(
        (lsd.0 - lsd.1).abs() <= 80.0 + 1e-9,
        "LSD difference clamped to the bias cap"
    );
    // The low-grip left wheel gets more than locked (bias pulls toward even) but less than open.
    assert!(
        locked.0 < lsd.0 && lsd.0 < open.0,
        "LSD left share between locked and open"
    );
}

#[test]
fn diff_max_torque_bias_limits_are_correct() {
    assert_eq!(
        diff(DiffKind::Open, 99.0, 0.9).max_torque_bias(500.0, false),
        0.0
    );
    assert!(diff(DiffKind::Locked, 0.0, 0.0)
        .max_torque_bias(500.0, false)
        .is_infinite());
    // LSD drive vs brake ramps: bias = preload + ramp·|t|.
    let d = DiffModel {
        kind: DiffKind::Lsd,
        preload_nm: 30.0,
        ramp_accel: 0.4,
        ramp_decel: 0.6,
    };
    assert!((d.max_torque_bias(100.0, false) - (30.0 + 0.4 * 100.0)).abs() < 1e-12);
    assert!((d.max_torque_bias(100.0, true) - (30.0 + 0.6 * 100.0)).abs() < 1e-12);
}

// --- Coupler torque conservation + split fractions ----------------------------------------------

#[test]
fn wheel_torque_is_conserved_through_couplers() {
    // Στ_out = τ_in · ratio · η. The FWD hatch: ICE → gearbox(ratios, final_drive 4.06, η 0.96).
    let car = assemble("fwd_hatch/vehicle.yaml");
    let pt = car.powertrain();
    // First gear: total ratio = 3.5 · 4.06, mechanical efficiency 0.96.
    let expected = 100.0 * 3.5 * 4.06 * 0.96;
    let w0 = pt.wheel_torque(0, 0, 100.0).expect("unit 0, gear 0");
    assert!(
        (w0 - expected).abs() < 1e-9,
        "gear-0 wheel torque {w0} vs {expected}"
    );
    // Linear in the source torque (a coupler is a linear gain).
    assert!((pt.wheel_torque(0, 0, 250.0).unwrap() - 2.5 * w0).abs() < 1e-9);
    // A taller gear (index 5, ratio 0.72) delivers less wheel torque than first.
    assert!(pt.wheel_torque(0, 5, 100.0).unwrap() < w0);
}

#[test]
fn split_fractions_sum_to_one() {
    for name in [
        "f1_2026/vehicle.yaml",
        "fwd_hatch/vehicle.yaml",
        "pdt_du_rwd/vehicle.yaml",
    ] {
        let car = assemble(name);
        let pt = car.powertrain();
        let (af, ar) = pt.axle_split();
        let (sl, sr) = pt.side_split();
        assert!(
            (af + ar - 1.0).abs() < 1e-12,
            "{name}: axle split sums to 1"
        );
        assert!(
            (sl + sr - 1.0).abs() < 1e-12,
            "{name}: side split sums to 1"
        );
    }
    // And the differential output torques sum to the axle torque for every kind.
    for kind in [
        DiffKind::Open,
        DiffKind::Locked,
        DiffKind::Lsd,
        DiffKind::Solid,
    ] {
        let (l, r) = diff(kind, 25.0, 0.35).split(180.0, 70.0, 110.0);
        assert!(
            (l + r - 180.0).abs() < 1e-9,
            "{kind:?} outputs sum to axle torque"
        );
    }
}

// --- Energy accounting (efficiency/loss maps) ---------------------------------------------------

#[test]
fn energy_closes_source_equals_mech_plus_loss() {
    // With the installed efficiency/loss maps, source = mechanical + loss holds exactly at the grid
    // nodes (the importer emits a consistent pair: drive loss = mech·(1/η − 1)).
    let mut car = assemble("pdt_du_rwd/vehicle.yaml");
    install_maps(&mut car, 0, "d.ptm.maps.parquet");
    let pt = car.powertrain();
    let mut checked = 0;
    for &rpm in &[81.8, 153.6, 225.4, 297.2] {
        for &tau in &[42.0, 84.0, 126.0, 168.0] {
            let e = pt.energy_at_shaft(0, rpm, tau).expect("map installed");
            let closure = (e.source_w - (e.mech_w + e.loss_w)).abs();
            assert!(
                closure < 1e-6 * e.source_w.abs().max(1.0),
                "energy closure at ({rpm},{tau}): source {} vs mech+loss {}",
                e.source_w,
                e.mech_w + e.loss_w
            );
            assert!(e.efficiency > 0.0 && e.efficiency <= 1.0);
            assert!(
                e.source_w >= e.mech_w,
                "drive: source ≥ mechanical (losses ≥ 0)"
            );
            assert_eq!(e.fuel_kg_per_s, 0.0, "an electric DU burns no fuel");
            checked += 1;
        }
    }
    assert!(checked >= 3);

    // The spin point (τ = 0) still closes: no mechanical power, but the idle draw is accounted, so
    // source = loss (the earlier η-only path returned source = 0 while loss > 0 — a closure gap).
    let idle = pt.energy_at_shaft(0, 297.2, 0.0).expect("map installed");
    assert_eq!(idle.mech_w, 0.0);
    assert!(idle.loss_w > 0.0, "there is an idle draw at the spin point");
    assert!(
        (idle.source_w - idle.loss_w).abs() < 1e-9,
        "idle: source {} should equal loss {}",
        idle.source_w,
        idle.loss_w
    );
}

#[test]
fn ice_accounts_fuel_mass_from_thermal_efficiency() {
    // The ICE map's efficiency is brake thermal efficiency, so a positive drive point burns fuel.
    let mut car = assemble("fwd_hatch/vehicle.yaml");
    install_maps(&mut car, 0, "tables/ice_v6.parquet");
    let pt = car.powertrain();
    let e = pt
        .energy_at_shaft(0, 8000.0, 300.0)
        .expect("ICE map installed");
    assert!(e.fuel_kg_per_s > 0.0, "an ICE under load burns fuel");
    // Fuel chemical power = source power; fuel_rate = P_src / LHV (43 MJ/kg reference).
    assert!((e.fuel_kg_per_s - e.source_w / 43.0e6).abs() < 1e-12);
    assert!((e.source_w - (e.mech_w + e.loss_w)).abs() < 1e-6 * e.source_w);
}

// --- PDT round-trip gate (§10.5 / §13) ----------------------------------------------------------

/// The drive-unit source efficiency — a faithful mirror of `gen_ptm_maps.py::_drive_unit_eta`, i.e.
/// the "source array" the importer wrote into the parquet. The round-trip reproduces it through the
/// Rust `GriddedMapN`.
fn source_du_efficiency(speed: f64, tau: f64) -> f64 {
    if tau == 0.0 {
        0.0
    } else {
        (0.95 - 0.10 * tau.abs() / 168.0 - 5.0e-5 * speed).clamp(0.30, 0.97)
    }
}

#[test]
fn pdt_round_trip_reproduces_spot_efficiencies_to_1e6() {
    // §10.5/§13: load the importer-emitted `.ptm` (ptm/pdt_synth_du.ptm.yaml) + its parquet, and
    // reproduce ≥3 spot efficiencies from the source arrays to 1e-6 through the real GriddedMapN.
    let mut car = assemble("pdt_du_rwd/vehicle.yaml");
    install_maps(&mut car, 0, "d.ptm.maps.parquet");
    let pt = car.powertrain();
    // Interior drive-quadrant grid nodes (away from the τ=0 spin kink and the envelope edge).
    let spots = [(81.8, 126.0), (153.6, 84.0), (225.4, 168.0), (297.2, 42.0)];
    let mut checked = 0;
    for &(rpm, tau) in &spots {
        let got = pt.efficiency(0, rpm, tau).expect("map installed");
        let src = source_du_efficiency(rpm, tau);
        assert!(
            (got - src).abs() < 1e-6,
            "spot ({rpm},{tau}) efficiency {got} vs source {src}"
        );
        checked += 1;
    }
    assert!(
        checked >= 3,
        "reproduced {checked} spot efficiencies (need ≥3)"
    );
}

// --- The differential inside the trim + the traction ceiling ------------------------------------

#[test]
fn open_diff_splits_slip_while_locked_keeps_it_equal() {
    // Open front diff (FWD hatch): equal torque ⇒ the two driven wheels take UNEQUAL slip under a
    // combined corner + accel (the inner wheel slips more to match the outer wheel's torque).
    let fwd = assemble("fwd_hatch/vehicle.yaml");
    let s = fwd
        .trim(&TrimInput::flat(30.0, 5.0, 2.0))
        .state()
        .copied()
        .expect("feasible");
    assert!(
        (s.kappa[0] - s.kappa[1]).abs() > 1e-4,
        "open diff ⇒ unequal driven-wheel slip (FL {} vs FR {})",
        s.kappa[0],
        s.kappa[1]
    );
    assert!(
        s.kappa[2].abs() < 1e-12 && s.kappa[3].abs() < 1e-12,
        "rear is undriven"
    );

    // Locked-in-the-trim (F1 LSD): equal speed ⇒ the two driven wheels share the SAME slip.
    let f1 = assemble("f1_2026/vehicle.yaml");
    let s = f1
        .trim(&TrimInput::flat(60.0, 8.0, 3.0))
        .state()
        .copied()
        .expect("feasible");
    assert!(
        (s.kappa[2] - s.kappa[3]).abs() < 1e-9,
        "locked/LSD ⇒ equal driven-wheel slip (RL {} vs RR {})",
        s.kappa[2],
        s.kappa[3]
    );
    assert!(
        s.kappa[0].abs() < 1e-12 && s.kappa[1].abs() < 1e-12,
        "front is undriven"
    );
}

#[test]
fn traction_ceiling_is_positive_and_falls_with_speed() {
    // The powertrain traction ceiling (best-gear peak envelope through the graph) is positive and,
    // for a geared ICE, falls from low to high speed (top gear trades torque for speed).
    let car = assemble("f1_2026/vehicle.yaml");
    let lo = car.max_tractive_force(20.0);
    let hi = car.max_tractive_force(90.0);
    assert!(lo > 0.0 && hi > 0.0, "traction ceiling is positive");
    assert!(lo > hi, "traction ceiling falls with speed: {lo} vs {hi}");
    assert!(car.max_tractive_accel(20.0) > 0.0);
    // A lumped drive unit (single gear) still puts down a finite force.
    assert!(assemble("pdt_du_rwd/vehicle.yaml").max_tractive_force(10.0) > 0.0);
}

#[test]
fn gearbox_map_efficiency_assembles_for_t1() {
    // T0 errors on a gearbox map efficiency (UnsupportedEfficiencyMap); T1 accepts it — a constant
    // proxy carries the traction force until the map is installed — retiring that error for T1.
    let ptm = "schema: ptm/1.0\nkind: ice\n\
        axes: {speed_rpm: [1000.0, 8000.0], load_axis: {torque_nm: [0.0, 300.0]}, torque_nm: [0.0, 300.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [1000.0, 8000.0], torque_nm: [280.0, 300.0]}}\n\
        inertia_kgm2: 0.1\nmass_kg: 120.0\n";
    let veh = "schema: vehicle/1.0\nname: t\n\
        chassis: {mass_kg: 1200.0, cg: [1.2, 0.0, 0.4], inertia: [120.0, 500.0, 550.0], wheelbase_m: 2.6, track_m: [1.5, 1.5]}\n\
        aero: {map: a.parquet, axes: [], constant: {cx_a_m2: 0.7, cz_front_a_m2: 0.0, cz_rear_a_m2: 0.0}}\n\
        suspension: {model: lumped_kc, front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}, rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}\n\
        tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}\n\
        drivetrain: {units: [{source: ptm/u.ptm.yaml, path: [{gearbox: {ratios: [3.2, 1.1], final_drive: 3.5, shift_time_s: 0.2, efficiency: {map: gb.parquet}}}, {diff: {type: open}}], wheels: [FL, FR]}]}\n\
        brakes: {balance_bar: 0.6, disc: {front: {thermal_capacity_j_per_k: 30000.0, cooling_area_m2: 0.06}, rear: {thermal_capacity_j_per_k: 20000.0, cooling_area_m2: 0.04}}}\n";
    let loader = MemLoader::new()
        .with("vehicle.yaml", veh)
        .with("ptm/u.ptm.yaml", ptm)
        .with("tyr/slick.tyr.yaml", SLICK);
    let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let car = T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false)
        .expect("T1 assembles with a gearbox map efficiency");
    assert!(
        car.notes().iter().any(|n| n.contains("map efficiency")),
        "the map-efficiency proxy is surfaced in the notes"
    );
    assert!(car.max_tractive_force(30.0) > 0.0);
}
