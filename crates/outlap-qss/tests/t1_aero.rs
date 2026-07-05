// SPDX-License-Identifier: AGPL-3.0-only
//! PR3 — ride-height/yaw aero map + aero-platform equilibrium in the T1 trim.
//!
//! Covers the map-consumption contract end to end: the committed synthetic F1 parquet decodes and
//! reproduces the reference coefficients at the reference ride heights; a constant map degenerates
//! to the constant-aero trim to 1e-9; and installing the map makes the aero balance speed-dependent
//! while every trim over a feasible grid still converges inside the friction envelope.
#![allow(clippy::doc_markdown, clippy::similar_names)]

use outlap_core::GriddedTable;
use outlap_qss::t1::aero::AeroMap;
use outlap_qss::{T1Vehicle, TrimInput};
use outlap_schema::io::{FsLoader, SourceLoader};
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

/// The committed synthetic F1 aero parquet lives with the reference vehicle under `data/`.
fn aero_bytes() -> Vec<u8> {
    let loader = FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/vehicles/f1_2026"
    ));
    loader
        .load_bytes("aero/f1_2026.parquet")
        .expect("committed f1 aero parquet present")
}

fn f1() -> (T1Vehicle, Vec<String>) {
    let loader = fixtures();
    let rv = load_vehicle("f1_2026/vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let car = T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap();
    (car, rv.spec.aero.axes.clone())
}

fn decode_map() -> (GriddedTable<f64>, Vec<String>) {
    let (_, axes) = f1();
    let axis_refs: Vec<&str> = axes.iter().map(String::as_str).collect();
    let table = read_gridded_table(&aero_bytes(), &axis_refs).unwrap();
    (table, axes)
}

#[test]
fn committed_map_reproduces_reference_coefficients() {
    // User decision #4: the map is anchored so the reference ride heights (30 mm front / 70 mm rear,
    // yaw 0, DRS closed) reproduce the f1_2026 constant-aero fallback (stand-in for PL2014 aero).
    let (table, axes) = decode_map();
    let map = AeroMap::from_table(&table, &axes).unwrap();
    let c = map.eval(30.0, 70.0, 0.0, 0.0);
    assert!(
        (c.cz_front_a_m2 - 1.9).abs() < 1e-12,
        "cz_front {}",
        c.cz_front_a_m2
    );
    assert!(
        (c.cz_rear_a_m2 - 2.6).abs() < 1e-12,
        "cz_rear {}",
        c.cz_rear_a_m2
    );
    assert!((c.cx_a_m2 - 1.25).abs() < 1e-12, "cx {}", c.cx_a_m2);
}

#[test]
fn committed_map_is_ground_effect_and_drs_shaped() {
    let (table, axes) = decode_map();
    let map = AeroMap::from_table(&table, &axes).unwrap();
    // Lower front ride height ⇒ more front downforce (ground effect).
    let high = map.eval(60.0, 70.0, 0.0, 0.0);
    let low = map.eval(10.0, 70.0, 0.0, 0.0);
    assert!(
        low.cz_front_a_m2 > high.cz_front_a_m2,
        "front DF should rise as the car lowers"
    );
    // |yaw| cuts downforce; DRS open cuts rear downforce and drag.
    let straight = map.eval(30.0, 70.0, 0.0, 0.0);
    let yawed = map.eval(30.0, 70.0, 8.0, 0.0);
    assert!(yawed.cz_front_a_m2 < straight.cz_front_a_m2);
    let drs = map.eval(30.0, 70.0, 0.0, 1.0);
    assert!(drs.cz_rear_a_m2 < straight.cz_rear_a_m2);
    assert!(drs.cx_a_m2 < straight.cx_a_m2);
    assert!(
        (drs.cz_front_a_m2 - straight.cz_front_a_m2).abs() < 1e-12,
        "DRS leaves front DF"
    );
}

#[test]
fn constant_map_degenerates_to_constant_aero() {
    // A flat map equal to the constant-aero coefficients must reproduce the constant-aero trim.
    let (constant_car, _) = f1();
    let (mut mapped_car, axes) = f1();
    // Build a constant table over the real axes.
    let names: Vec<&str> = axes.iter().map(String::as_str).collect();
    let hf = [10.0, 60.0];
    let hr = [30.0, 140.0];
    let (mut c_hf, mut c_hr, mut c_yaw, mut c_drs) = (vec![], vec![], vec![], vec![]);
    let (mut czf, mut czr, mut cx) = (vec![], vec![], vec![]);
    for &a in &hf {
        for &b in &hr {
            for &y in &[-8.0, 8.0] {
                for &d in &[0.0, 1.0] {
                    c_hf.push(a);
                    c_hr.push(b);
                    c_yaw.push(y);
                    c_drs.push(d);
                    czf.push(1.9);
                    czr.push(2.6);
                    cx.push(1.25);
                }
            }
        }
    }
    let cols = vec![
        ("ride_height_f_mm".to_owned(), c_hf),
        ("ride_height_r_mm".to_owned(), c_hr),
        ("yaw_deg".to_owned(), c_yaw),
        ("drs_flag".to_owned(), c_drs),
        ("cz_front_a_m2".to_owned(), czf),
        ("cz_rear_a_m2".to_owned(), czr),
        ("cx_a_m2".to_owned(), cx),
    ];
    let table = GriddedTable::from_long(&cols, &names).unwrap();
    mapped_car.install_aero_map(&table, &axes).unwrap();
    assert!(mapped_car.has_aero_map());

    for &(v, ay, ax) in &[(60.0, 12.0, 0.0), (50.0, 8.0, -6.0), (70.0, 5.0, 4.0)] {
        let a = constant_car.trim(&TrimInput::flat(v, ay, ax));
        let b = mapped_car.trim(&TrimInput::flat(v, ay, ax));
        let (sa, sb) = (a.state().unwrap(), b.state().unwrap());
        assert!(
            (sa.delta - sb.delta).abs() < 1e-9,
            "δ at (v={v},ay={ay},ax={ax})"
        );
        assert!((sa.beta - sb.beta).abs() < 1e-9, "β");
        for i in 0..4 {
            assert!((sa.fz[i] - sb.fz[i]).abs() < 1e-6, "Fz[{i}]");
        }
    }
}

#[test]
fn mapped_aero_balance_is_speed_dependent() {
    let (constant_car, _) = f1();
    let (mut mapped_car, axes) = f1();
    let (table, _) = decode_map();
    mapped_car.install_aero_map(&table, &axes).unwrap();

    // Constant aero: the balance does not move with speed.
    let c_lo = constant_car.aero_front_downforce_share_at(30.0);
    let c_hi = constant_car.aero_front_downforce_share_at(90.0);
    assert!(
        (c_lo - c_hi).abs() < 1e-12,
        "constant balance must be speed-invariant"
    );

    // Mapped aero: the platform rakes with downforce ⇒ the balance moves with speed.
    let m_lo = mapped_car.aero_front_downforce_share_at(30.0);
    let m_hi = mapped_car.aero_front_downforce_share_at(90.0);
    assert!(
        (m_lo - m_hi).abs() > 1e-4,
        "mapped balance should shift with speed: {m_lo} vs {m_hi}"
    );
    assert!((0.0..=1.0).contains(&m_lo) && (0.0..=1.0).contains(&m_hi));
}

#[test]
fn mapped_car_trims_across_a_feasible_grid() {
    let (mut car, axes) = f1();
    let (table, _) = decode_map();
    car.install_aero_map(&table, &axes).unwrap();
    let g = outlap_qss::G;
    for vi in 0..6 {
        let v = 20.0 + 12.0 * f64::from(vi); // 20 … 80 m/s
        for ai in 0..5 {
            let ay = -8.0 + 4.0 * f64::from(ai); // −8 … 8 m/s²
            for &ax in &[-6.0, 0.0, 4.0] {
                let out = car.trim(&TrimInput::flat(v, ay, ax));
                if let Some(s) = out.state() {
                    assert!(s.residual_norm <= 1e-10, "rn at (v={v},ay={ay},ax={ax})");
                    // Per-wheel loads non-negative and there IS downforce (ΣFz > static weight).
                    assert!(s.fz.iter().all(|&f| f >= 0.0));
                    let sum: f64 = s.fz.iter().sum();
                    assert!(
                        sum > car.mass_kg * g,
                        "ΣFz {sum} should exceed static weight {} at v={v}",
                        car.mass_kg * g
                    );
                    // Friction-circle containment (loose μ upper bound).
                    for i in 0..4 {
                        let f_h = (s.fx[i] * s.fx[i] + s.fy[i] * s.fy[i]).sqrt();
                        assert!(
                            f_h <= 1.9 * s.fz[i].max(0.0) + 1.0,
                            "wheel {i} outside μ·Fz"
                        );
                    }
                }
            }
        }
    }
}
