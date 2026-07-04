// SPDX-License-Identifier: AGPL-3.0-only
//! Decoder tests for the parquet map sidecar (`outlap-schema/src/sidecar.rs`, `parquet` feature).
//!
//! Reads a committed synthetic fixture that mirrors the PDT importer's emission byte-for-byte
//! (SNAPPY-compressed `DOUBLE` columns `speed_rpm, torque_nm, efficiency, loss_w`, one NaN cell) and
//! checks: column extraction, rectilinear pivot + node-exact interpolation through the real
//! `GriddedMapN`, NaN masking, and the loader path.

#![cfg(feature = "parquet")]

use outlap_core::OutOfDomain;
use outlap_schema::io::MemLoader;
use outlap_schema::sidecar::{
    load_gridded_map, read_columns, read_gridded_map, read_gridded_table,
};

const FIXTURE: &[u8] = include_bytes!("fixtures/gridmap_2d.parquet");

fn clamp2() -> Vec<OutOfDomain> {
    vec![OutOfDomain::Clamp, OutOfDomain::Clamp]
}

#[test]
fn reads_all_columns_in_order() {
    let cols = read_columns(FIXTURE).unwrap();
    let names: Vec<&str> = cols.iter().map(|(n, _)| n.as_str()).collect();
    assert_eq!(names, ["speed_rpm", "torque_nm", "efficiency", "loss_w"]);
    for (name, v) in &cols {
        assert_eq!(v.len(), 12, "column {name} row count");
    }
    // The single masked cell (speed=1000, torque=200) is NaN in the value columns.
    let (_, eff) = cols.iter().find(|(n, _)| n == "efficiency").unwrap();
    assert_eq!(eff.iter().filter(|v| v.is_nan()).count(), 1);
}

#[test]
fn pivots_and_interpolates_node_exact() {
    let cols = read_columns(FIXTURE).unwrap();
    let speed = &cols.iter().find(|(n, _)| n == "speed_rpm").unwrap().1;
    let torque = &cols.iter().find(|(n, _)| n == "torque_nm").unwrap().1;
    let eff = &cols.iter().find(|(n, _)| n == "efficiency").unwrap().1;

    let table = read_gridded_table(FIXTURE, &["speed_rpm", "torque_nm"]).unwrap();
    assert_eq!(table.axis_names(), &["speed_rpm", "torque_nm"]);
    let map = table.map("efficiency", clamp2()).unwrap();
    assert_eq!(map.shape(), &[3, 4]);

    // Every non-NaN long-form row is reproduced exactly at its node.
    for i in 0..speed.len() {
        if eff[i].is_nan() {
            // Masked cell: filled (finite) and flagged out-of-hull.
            let (v, flags) = map.eval_flagged(&[speed[i], torque[i]]);
            assert!(v.is_finite(), "masked cell should be filled, not NaN");
            assert!(flags.out_of_hull, "masked cell should flag out_of_hull");
        } else {
            let got = map.eval(&[speed[i], torque[i]]);
            assert!(
                (got - eff[i]).abs() < 1e-12,
                "node ({}, {}) eff {got} vs {}",
                speed[i],
                torque[i],
                eff[i]
            );
        }
    }
}

#[test]
fn one_step_map_and_loader_path_agree() {
    let direct =
        read_gridded_map(FIXTURE, &["speed_rpm", "torque_nm"], "loss_w", clamp2()).unwrap();
    let loader = MemLoader::new().with_bytes("maps.parquet", FIXTURE.to_vec());
    let via_loader = load_gridded_map(
        &loader,
        "maps.parquet",
        &["speed_rpm", "torque_nm"],
        "loss_w",
        clamp2(),
    )
    .unwrap();
    for &(s, t) in &[(1500.0, -50.0), (2500.0, 50.0), (2000.0, 0.0)] {
        // Same bytes through both entry points must be bit-identical (determinism, not "close").
        #[allow(clippy::float_cmp)]
        {
            assert_eq!(direct.eval(&[s, t]), via_loader.eval(&[s, t]));
        }
    }
}

#[test]
fn missing_value_column_errors() {
    let err = read_gridded_map(FIXTURE, &["speed_rpm", "torque_nm"], "nope", clamp2());
    assert!(err.is_err());
}
