// SPDX-License-Identifier: AGPL-3.0-only
//! Emit battery + Vdc–SoC-coupling validation traces from the **real** `outlap-qss` battery model
//! (`Pack`) and powertrain (`T1Powertrain`) as CSV on stdout. `python/tools/plot_battery_coupling.py`
//! runs this and plots the output, so the theory figure is driven by the actual model, not a
//! re-implementation.
//!
//! Three scenarios, one CSV with a `scenario` column:
//! - `pulse`: a constant-current discharge pulse on a constant-parameter pack; `x` = time [s],
//!   `a` = terminal voltage [V]. Python overlays the closed-form Thevenin `OCV − I·R0 − I·R1(1 −
//!   e^{−t/τ})`.
//! - `sweep`: a pack discharged down its SoC window; `x` = SoC, `a` = pack terminal voltage [V],
//!   `b` = the drive-unit efficiency the map returns at that coupled voltage (the Vdc–SoC coupling).
//!   Uses the committed `synth_pack.battery.yaml`, or a battery YAML passed as `argv[1]` — the latter
//!   is the local real-pack validation path (derived PDT artifacts stay untracked, decision #7).
//! - `extrap`: the drive-unit efficiency vs DC-link voltage at a fixed operating point; `x` =
//!   vdc [V], `a` = efficiency. Python shades the in-grid band (730–850 V) so the below/above-grid
//!   linear extrapolation is visible.
#![allow(clippy::doc_markdown)]

use outlap_core::GriddedTable;
use outlap_qss::{Pack, PackState, T1Powertrain, T1Vehicle};
use outlap_schema::battery::{
    BatteryDoc, BatteryMeta, BatteryModelKind, BatteryTables, Ecm, EcmAxes, PackCapacity,
    PackLimits, PackThermal, PackTopology, PowerVsSoc, TableLevel,
};
use outlap_schema::io::{FsLoader, SourceLoader};
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_battery, load_vehicle, Conditions, LoadOptions};

/// Pulse scenario parameters (Python recomputes the analytic curve from these — keep in sync).
const P_OCV: f64 = 720.0;
const P_R0: f64 = 0.22;
const P_R1: f64 = 0.09;
const P_TAU: f64 = 18.0;
const P_I: f64 = 260.0;
const P_DT: f64 = 0.5;

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

/// A constant-parameter pack so the pulse trace can be checked against the closed form.
fn constant_pack() -> (Pack, PackState) {
    let axis = |name: &str, v: f64| (name.to_owned(), vec![v, v, v, v]);
    let cols = vec![
        ("soc".to_owned(), vec![0.0, 0.0, 1.0, 1.0]),
        ("temp_c".to_owned(), vec![0.0, 50.0, 0.0, 50.0]),
        axis("ocv_v", P_OCV),
        axis("r0_ohm", P_R0),
        axis("r1_ohm", P_R1),
        axis("tau1_s", P_TAU),
        axis("dudt_v_per_k", 0.0),
    ];
    let table = GriddedTable::from_long(&cols, &["soc", "temp_c"]).unwrap();
    let doc = BatteryDoc {
        schema: SchemaVersion::new("battery", 1, 0),
        model: BatteryModelKind::RcPairs,
        topology: PackTopology { ns: 1, np: 1 },
        capacity: PackCapacity {
            q_pack_ah: 1.0e9,
            e_pack_wh: 1.0,
        },
        soc_window: [0.0, 1.0],
        ecm: Ecm {
            rc_pairs: 1,
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
                power_w: vec![1.0e12, 1.0e12],
            },
            peak_regen_power_w_vs_soc: PowerVsSoc {
                soc: vec![0.0, 1.0],
                power_w: vec![1.0e12, 1.0e12],
            },
            cell_v_min: 0.0,
            cell_v_max: 1000.0,
            max_c_rate: 100.0,
        },
        thermal: PackThermal {
            mass_kg: 1.0,
            cp_j_per_kgk: 1000.0,
            thermal_resistance_k_per_w: 0.0,
            coolant_temp_c: 25.0,
        },
        meta: BatteryMeta::default(),
    };
    Pack::assemble(&doc, &table, Some(0.5)).unwrap()
}

fn main() {
    let loader = fixtures();
    println!(
        "# ocv={P_OCV} r0={P_R0} r1={P_R1} tau={P_TAU} i={P_I} dt={P_DT} vdc_lo=730 vdc_hi=850"
    );
    println!("scenario,x,a,b");

    // --- (a) Pulse response through the real Pack integrator (sampled after each applied step, so
    // every point reflects the loaded terminal voltage the closed form describes). -----------------
    let (pack, mut st) = constant_pack();
    for k in 0..200 {
        let out = pack.step_current(&mut st, P_I, P_DT);
        println!(
            "pulse,{:.2},{:.4},",
            f64::from(k + 1) * P_DT,
            out.terminal_v
        );
    }

    // --- Load the pack (the committed fixture, or a battery YAML passed as argv[1] — the latter is
    // the local real-pack validation path: derived PDT artifacts stay untracked, decision #7). -----
    let (fpack, mut fst) = if let Some(path) = std::env::args().nth(1) {
        let p = std::path::Path::new(&path);
        let dir = p.parent().unwrap().to_str().unwrap();
        let local = FsLoader::new(dir);
        let doc = load_battery(p.file_name().unwrap().to_str().unwrap(), &local).unwrap();
        let bytes = local.load_bytes(doc.ecm.tables.file.as_str()).unwrap();
        let ecm: GriddedTable<f64> = read_gridded_table(&bytes, &Pack::ecm_axis_names()).unwrap();
        Pack::assemble(&doc, &ecm, None).unwrap()
    } else {
        let doc = load_battery("battery/synth_pack.battery.yaml", &loader).unwrap();
        let bytes = loader
            .load_bytes("battery/synth_pack.tables.parquet")
            .unwrap();
        let ecm: GriddedTable<f64> = read_gridded_table(&bytes, &Pack::ecm_axis_names()).unwrap();
        Pack::assemble(&doc, &ecm, None).unwrap()
    };

    let rv = load_vehicle("pdt_du_rwd/vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let mut car = T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap();
    let vbytes = loader.load_bytes("pdt_synth_du_vdc.maps.parquet").unwrap();
    let vtable: GriddedTable<f64> =
        read_gridded_table(&vbytes, &T1Powertrain::map_axis_names_vdc()).unwrap();
    car.install_powertrain_maps(0, &vtable).unwrap();
    let pt = car.powertrain();
    let (rpm, tau) = (153.6, 84.0);

    // --- (b) SoC sweep: terminal voltage and the coupled drive-unit efficiency. ------------------
    for _ in 0..900 {
        let v = fpack.terminal_voltage_v(&fst);
        let eta = pt.efficiency_vdc(0, rpm, tau, v).unwrap();
        println!("sweep,{:.5},{v:.3},{eta:.5}", fst.soc);
        fpack.step_power(&mut fst, 140_000.0, 1.0);
    }

    // --- (c) Efficiency vs Vdc: below / in / above the 730–850 V grid (linear extrapolation). ----
    for k in 0..=60 {
        let vdc = 600.0 + 5.0 * f64::from(k); // 600 … 900 V
        let eta = pt.efficiency_vdc(0, rpm, tau, vdc).unwrap();
        println!("extrap,{vdc:.1},{eta:.5},");
    }
}
