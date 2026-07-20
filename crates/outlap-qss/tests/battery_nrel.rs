// SPDX-License-Identifier: AGPL-3.0-only
//! Gate #1 (M6 PR8, §13 battery row): the outlap Thevenin pack reproduces the **NREL `thevenin`**
//! reference terminal-voltage response to within **RMS ≤ 1 %** on identical current pulses — charge
//! and discharge, ±1C and ±2C, 10 s on / 60 s rest, two temperatures, and both RC counts (D-M6-3).
//!
//! The reference traces live in `tests/golden/battery_nrel/*.csv`, generated ONCE by the opt-in tool
//! `python/tools/gen_battery_nrel_golden.py` (the NREL package is BSD-3 and consumed as DATA only,
//! never a runtime dep — the MFeval / tire-golden pattern). Each CSV header carries the exact ECM
//! parameters the oracle used, so this test builds an identical [`Pack`] from a single source of
//! truth and drives it with the CSV's own current column. The pass is machine-tight (both integrate
//! the same published Thevenin ODE), well under the ≤1 % gate — a genuine cross-implementation check
//! of outlap's exact-exponential RC advance + Coulomb SoC + terminal-voltage assembly.
#![allow(
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::path::PathBuf;

use outlap_core::GriddedTable;
use outlap_qss::{Pack, PackState};
use outlap_schema::battery::{
    BatteryDoc, BatteryMeta, BatteryModelKind, BatteryTables, Ecm, EcmAxes, PackCapacity,
    PackLimits, PackThermal, PackTopology, PowerVsSoc, TableLevel,
};
use outlap_schema::version::SchemaVersion;

/// The four committed reference cases: {cold 25 °C, warm 40 °C} × {1, 2} RC pairs.
const CASES: [&str; 4] = ["cold25_rc1", "cold25_rc2", "warm40_rc1", "warm40_rc2"];

fn golden_dir() -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/golden/battery_nrel"
    ))
}

/// The ECM parameters carried in a golden header (the single source of truth for the pack build).
struct Params {
    ocv: f64,
    r0: f64,
    r1: f64,
    tau1: f64,
    r2: f64,
    tau2: f64,
    rc_pairs: u32,
    capacity_ah: f64,
    soc0: f64,
}

/// A parsed golden: the ECM parameters plus the `(t, I, V_ref)` pulse trace.
struct Golden {
    p: Params,
    t: Vec<f64>,
    current: Vec<f64>,
    v_ref: Vec<f64>,
    has_provenance: bool,
}

fn parse_golden(name: &str) -> Golden {
    let text = std::fs::read_to_string(golden_dir().join(format!("{name}.csv")))
        .unwrap_or_else(|e| panic!("read {name}.csv: {e}"));
    let mut kv = std::collections::HashMap::<String, f64>::new();
    let (mut t, mut current, mut v_ref) = (Vec::new(), Vec::new(), Vec::new());
    let mut has_provenance = false;
    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("# param ") {
            let (key, val) = rest.split_once(':').expect("`# param key: value`");
            kv.insert(key.trim().to_owned(), val.trim().parse().unwrap());
        } else if line.contains("NREL thevenin") {
            has_provenance = true;
        } else if line.starts_with('#') || line.starts_with("t_s,") {
            // other header / column line
        } else if !line.trim().is_empty() {
            let mut it = line.split(',').map(|f| f.parse::<f64>().unwrap());
            t.push(it.next().unwrap());
            current.push(it.next().unwrap());
            v_ref.push(it.next().unwrap());
        }
    }
    let g = |k: &str| {
        *kv.get(k)
            .unwrap_or_else(|| panic!("golden {name} missing param {k}"))
    };
    Golden {
        p: Params {
            ocv: g("ocv_v"),
            r0: g("r0_ohm"),
            r1: g("r1_ohm"),
            tau1: g("tau1_s"),
            r2: g("r2_ohm"),
            tau2: g("tau2_s"),
            rc_pairs: g("rc_pairs") as u32,
            capacity_ah: g("capacity_ah"),
            soc0: g("soc0"),
        },
        t,
        current,
        v_ref,
        has_provenance,
    }
}

/// Build a single-cell constant-ECM [`Pack`] from the golden's parameters (no Ns×Np scaling, so the
/// terminal voltage IS the cell voltage the oracle reports).
fn pack_from(p: &Params) -> (Pack, PackState) {
    let axis = |v: f64| vec![v, v, v, v]; // constant over the (soc, temp_c) grid corners
    let mut cols = vec![
        ("soc".to_owned(), vec![0.0, 0.0, 1.0, 1.0]),
        ("temp_c".to_owned(), vec![0.0, 50.0, 0.0, 50.0]),
        ("ocv_v".to_owned(), axis(p.ocv)),
        ("r0_ohm".to_owned(), axis(p.r0)),
        ("r1_ohm".to_owned(), axis(p.r1)),
        ("tau1_s".to_owned(), axis(p.tau1)),
        ("dudt_v_per_k".to_owned(), axis(0.0)),
    ];
    if p.rc_pairs == 2 {
        cols.push(("r2_ohm".to_owned(), axis(p.r2)));
        cols.push(("tau2_s".to_owned(), axis(p.tau2)));
    }
    let table = GriddedTable::from_long(&cols, &["soc", "temp_c"]).unwrap();
    let doc = BatteryDoc {
        schema: SchemaVersion::new("battery", 1, 1 + u16::from(p.rc_pairs == 2)),
        model: BatteryModelKind::RcPairs,
        topology: PackTopology { ns: 1, np: 1 },
        capacity: PackCapacity {
            q_pack_ah: p.capacity_ah,
            e_pack_wh: 1.0,
        },
        soc_window: [0.0, 1.0],
        ecm: Ecm {
            rc_pairs: p.rc_pairs,
            axes: EcmAxes {
                soc: vec![0.0, 1.0],
                temp_c: vec![0.0, 50.0],
            },
            tables: BatteryTables {
                file: "x.parquet".into(),
                level: TableLevel::Cell,
            },
        },
        limits: PackLimits {
            peak_discharge_power_w_vs_soc: PowerVsSoc {
                soc: vec![0.0, 1.0],
                power_w: vec![1.0e9, 1.0e9],
            },
            peak_regen_power_w_vs_soc: PowerVsSoc {
                soc: vec![0.0, 1.0],
                power_w: vec![1.0e9, 1.0e9],
            },
            regen_derate_vs_temp: None,
            cell_v_min: 0.0,
            cell_v_max: 100.0,
            max_c_rate: 100.0,
        },
        thermal: PackThermal {
            mass_kg: 1.0,
            cp_j_per_kgk: 1000.0,
            thermal_resistance_k_per_w: 0.0, // temperature does not enter the constant ECM
            coolant_temp_c: 25.0,
        },
        meta: BatteryMeta::default(),
    };
    Pack::assemble(&doc, &table, Some(p.soc0)).unwrap()
}

#[test]
fn terminal_voltage_matches_nrel_thevenin_within_one_percent() {
    for name in CASES {
        let g = parse_golden(name);
        let (pack, mut st) = pack_from(&g.p);
        // Replay the reference current at the reference cadence. dt = 0 at t=0 and at every pulse
        // boundary (the oracle records the instant the current switches) — a valid no-op that just
        // re-evaluates the terminal voltage with the new current, matching the oracle row-for-row.
        let mut sum_sq = 0.0;
        let mut sum_abs = 0.0;
        let mut prev_t = g.t[0];
        for k in 0..g.t.len() {
            let dt = g.t[k] - prev_t;
            prev_t = g.t[k];
            let out = pack.step_current(&mut st, g.current[k], dt);
            sum_sq += (out.terminal_v - g.v_ref[k]).powi(2);
            sum_abs += g.v_ref[k].abs();
        }
        let n = g.t.len() as f64;
        let rms = (sum_sq / n).sqrt();
        let mean_abs = sum_abs / n;
        let rms_pct = 100.0 * rms / mean_abs;
        assert!(
            rms_pct <= 1.0,
            "{name}: terminal-voltage RMS {rms_pct:.4}% exceeds the 1% NREL gate (rms {rms:.6} V)"
        );
        // Sanity: this is a machine-tight cross-check, not a marginal 0.99% pass.
        assert!(
            rms_pct < 0.1,
            "{name}: RMS {rms_pct:.4}% unexpectedly large — the ECM parameterisation may have drifted"
        );
    }
}

#[test]
fn goldens_carry_nrel_provenance_headers() {
    for name in CASES {
        assert!(
            parse_golden(name).has_provenance,
            "{name}.csv is missing its NREL thevenin provenance header"
        );
    }
}
