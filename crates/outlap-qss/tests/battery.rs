// SPDX-License-Identifier: AGPL-3.0-only
//! PR6 — battery Thevenin model, the SoC / temperature slow states, and the Vdc–SoC coupling
//! (§8.4, §13 battery row). Synthetic fixtures only (firewall §1).
//!
//! Covers: the pulse-response validation vs the closed-form Thevenin (≤1% RMS); SoC monotone under
//! discharge; determinism of the slow-state advance; the Vdc-stacked drive-unit map evaluated at a
//! coupled terminal voltage with in-grid / below-grid / above-grid (linear-extrapolated) behaviour;
//! the coupling presence matrix (a map with vs without a Vdc axis); and the coupled loss lookup that
//! feeds PR5's machine-thermal model.
#![allow(clippy::float_cmp, clippy::doc_markdown)]

use outlap_core::GriddedTable;
use outlap_qss::{Pack, T1Powertrain, T1Vehicle};
use outlap_schema::battery::{
    BatteryDoc, BatteryMeta, BatteryModelKind, BatteryTables, Ecm, EcmAxes, PackCapacity,
    PackLimits, PackThermal, PackTopology, PowerVsSoc, TableLevel,
};
use outlap_schema::io::{FsLoader, SourceLoader};
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_battery, load_vehicle, Conditions, LoadOptions};

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

/// Load the committed synthetic pack fixture (YAML + ECM parquet) into a runnable [`Pack`].
fn fixture_pack() -> (Pack, outlap_qss::PackState) {
    let loader = fixtures();
    let doc = load_battery("battery/synth_pack.battery.yaml", &loader).unwrap();
    let bytes = loader
        .load_bytes("battery/synth_pack.tables.parquet")
        .unwrap();
    let table: GriddedTable<f64> = read_gridded_table(&bytes, &Pack::ecm_axis_names()).unwrap();
    Pack::assemble(&doc, &table, None).unwrap()
}

// --- Thevenin pulse-response validation (§13 battery row) ----------------------------------------

/// A constant-parameter pack so the discrete integrator can be checked against the analytic
/// closed-form Thevenin response. 1 cell (pack==cell), an enormous capacity (SoC ≈ frozen over the
/// pulse), and no cooling path (temperature pinned) isolate the RC dynamics.
fn constant_pack(ocv: f64, r0: f64, r1: f64, tau: f64) -> (Pack, outlap_qss::PackState) {
    let axis = |name: &str, v: f64| (name.to_owned(), vec![v, v, v, v]);
    let cols = vec![
        ("soc".to_owned(), vec![0.0, 0.0, 1.0, 1.0]),
        ("temp_c".to_owned(), vec![0.0, 50.0, 0.0, 50.0]),
        axis("ocv_v", ocv),
        axis("r0_ohm", r0),
        axis("r1_ohm", r1),
        axis("tau1_s", tau),
        axis("dudt_v_per_k", 0.0),
    ];
    let table = GriddedTable::from_long(&cols, &["soc", "temp_c"]).unwrap();
    let doc = BatteryDoc {
        schema: SchemaVersion::new("battery", 1, 0),
        model: BatteryModelKind::RcPairs,
        topology: PackTopology { ns: 1, np: 1 },
        capacity: PackCapacity {
            q_pack_ah: 1.0e9, // effectively frozen SoC over a short pulse
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
            thermal_resistance_k_per_w: 0.0, // pin temperature (isolate the RC dynamics)
            coolant_temp_c: 25.0,
        },
        meta: BatteryMeta::default(),
    };
    Pack::assemble(&doc, &table, Some(0.5)).unwrap()
}

// --- Charge acceptance: SoC ∧ temperature (series regen blend, §7.6) -----------------------------

/// Absolute zero offset, K (the crate's private `CELSIUS_K`).
const CELSIUS_K: f64 = 273.15;

/// A single-cell pack whose `R0` is temperature-graded (cold ⇒ high) and whose OCV is flat, so the
/// charge-acceptance ceilings can be isolated one at a time. `ns = 1`, so `v_max_pack = cell_v_max`.
fn charge_pack(
    ocv: f64,
    r0_cold: f64,
    r0_warm: f64,
    cell_v_max: f64,
    peak_regen_w: f64,
    derate: Option<outlap_schema::battery::DerateVsTemp>,
) -> (Pack, outlap_qss::PackState) {
    // Grid corners: (soc, temp_c) ∈ {0,1} × {-20, 40}. R0 grades with temperature only.
    let cols = vec![
        ("soc".to_owned(), vec![0.0, 0.0, 1.0, 1.0]),
        ("temp_c".to_owned(), vec![-20.0, 40.0, -20.0, 40.0]),
        ("ocv_v".to_owned(), vec![ocv; 4]),
        (
            "r0_ohm".to_owned(),
            vec![r0_cold, r0_warm, r0_cold, r0_warm],
        ),
        ("r1_ohm".to_owned(), vec![1.0e-6; 4]),
        ("tau1_s".to_owned(), vec![1.0; 4]),
        ("dudt_v_per_k".to_owned(), vec![0.0; 4]),
    ];
    let table = GriddedTable::from_long(&cols, &["soc", "temp_c"]).unwrap();
    let doc = BatteryDoc {
        schema: SchemaVersion::new("battery", 1, 1),
        model: BatteryModelKind::RcPairs,
        topology: PackTopology { ns: 1, np: 1 },
        capacity: PackCapacity {
            q_pack_ah: 1.0e9,
            e_pack_wh: 1.0,
        },
        soc_window: [0.0, 0.9],
        ecm: Ecm {
            rc_pairs: 1,
            axes: EcmAxes {
                soc: vec![0.0, 1.0],
                temp_c: vec![-20.0, 40.0],
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
                power_w: vec![peak_regen_w, peak_regen_w],
            },
            regen_derate_vs_temp: derate,
            cell_v_min: 0.0,
            cell_v_max,
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

/// A BMS-style charge derate: nothing below 0 °C, full acceptance by 25 °C.
fn plating_derate() -> outlap_schema::battery::DerateVsTemp {
    outlap_schema::battery::DerateVsTemp {
        temp_c: vec![-20.0, 0.0, 10.0, 25.0, 45.0],
        factor: vec![0.0, 0.0, 0.35, 1.0, 1.0],
    }
}

/// A resistance low enough that the voltage ceiling sits far above the design curve, so the kinetic
/// derate is the only thing that can bind. (At a realistic 0.02 Ω a *single cell* only accepts ~126 W
/// — the voltage ceiling would mask the derate entirely. Packs escape this via `ns` in series.)
const R0_NEGLIGIBLE: f64 = 1.0e-6;

/// A cold pack must not accept the charge a warm one does — the kinetic (lithium-plating) derate.
/// The design curve and the voltage headroom are identical at both temperatures here, so the *only*
/// thing that can move the ceiling is temperature.
#[test]
fn cold_pack_accepts_less_charge_than_warm() {
    let (pack, st) = charge_pack(
        3.6,
        R0_NEGLIGIBLE,
        R0_NEGLIGIBLE,
        4.2,
        50_000.0,
        Some(plating_derate()),
    );
    let cold = outlap_qss::PackState {
        temp_k: -5.0 + CELSIUS_K,
        ..st
    };
    let warm = outlap_qss::PackState {
        temp_k: 30.0 + CELSIUS_K,
        ..st
    };
    let (p_cold, p_warm) = (
        pack.regen_power_limit_w(&cold),
        pack.regen_power_limit_w(&warm),
    );
    assert!(
        p_cold < p_warm,
        "cold pack must accept less charge: cold={p_cold} warm={p_warm}"
    );
    assert_eq!(
        p_cold, 0.0,
        "below 0 °C the BMS accepts no charge: {p_cold}"
    );
    assert!(p_warm > 0.0, "a warm pack accepts charge: {p_warm}");
}

/// The derate is a *scaling* of the design curve, not a replacement: at a partial-derate temperature
/// the ceiling is exactly `factor · peak_regen(SoC)` (the voltage ceiling is far away here).
#[test]
fn derate_scales_the_design_curve() {
    let peak = 50_000.0;
    let (pack, st) = charge_pack(
        3.6,
        R0_NEGLIGIBLE,
        R0_NEGLIGIBLE,
        4.2,
        peak,
        Some(plating_derate()),
    );
    let at10 = outlap_qss::PackState {
        temp_k: 10.0 + CELSIUS_K,
        ..st
    };
    let limit = pack.regen_power_limit_w(&at10);
    assert!(
        (limit - 0.35 * peak).abs() < 1e-6 * peak,
        "expected 0.35·{peak}, got {limit}"
    );
}

/// With no derate curve declared, temperature must not gate acceptance (backward compatibility with
/// `battery/1.0` documents) — only the SoC curve and the voltage ceiling bind.
#[test]
fn absent_derate_curve_leaves_acceptance_temperature_independent() {
    let (pack, st) = charge_pack(3.6, R0_NEGLIGIBLE, R0_NEGLIGIBLE, 4.2, 50_000.0, None);
    let cold = outlap_qss::PackState {
        temp_k: -15.0 + CELSIUS_K,
        ..st
    };
    let warm = outlap_qss::PackState {
        temp_k: 35.0 + CELSIUS_K,
        ..st
    };
    assert_eq!(pack.regen_derate_factor(&cold), 1.0);
    let (p_cold, p_warm) = (
        pack.regen_power_limit_w(&cold),
        pack.regen_power_limit_w(&warm),
    );
    // The design curve binds at both ends; R0 is flat in temperature, so only interpolation
    // round-off can separate them.
    assert!(
        (p_cold - p_warm).abs() <= 1e-9 * p_warm,
        "no derate curve ⇒ the ceiling is temperature-independent: cold={p_cold} warm={p_warm}"
    );
}

/// A nearly-full pack tapers on the **voltage** ceiling even when the design curve and the derate
/// both say "take it all": charging pushes the terminal voltage above the EMF, and it may not pass
/// `ns · cell_v_max`. This is the constant-voltage taper, and it is what stops a full battery from
/// swallowing a whole braking event.
#[test]
fn nearly_full_pack_tapers_on_the_voltage_ceiling() {
    // OCV 4.19 V sits 10 mV under the 4.2 V ceiling ⇒ tiny headroom ⇒ tiny charge power.
    let (pack, st) = charge_pack(4.19, 0.02, 0.02, 4.2, 1.0e9, None);
    let warm = outlap_qss::PackState {
        temp_k: 25.0 + CELSIUS_K,
        ..st
    };
    let limit = pack.regen_power_limit_w(&warm);
    let expected = 4.2 * (4.2 - 4.19) / 0.02; // V_max · (V_max − emf) / R0
    assert!(
        (limit - expected).abs() < 1e-9 * expected,
        "voltage-limited ceiling: expected {expected}, got {limit}"
    );
    assert!(limit < 1.0e9, "the 1 GW design curve must not win");

    // At the ceiling exactly, nothing is accepted.
    let (full, st_full) = charge_pack(4.2, 0.02, 0.02, 4.2, 1.0e9, None);
    let at_ceiling = outlap_qss::PackState {
        temp_k: 25.0 + CELSIUS_K,
        ..st_full
    };
    assert_eq!(full.regen_power_limit_w(&at_ceiling), 0.0);
}

/// The voltage ceiling itself tightens when cold, because `R0(SoC, T)` rises: the same voltage
/// headroom admits less current. This is the *ohmic* half of the cold story, independent of the
/// declared kinetic derate.
#[test]
fn voltage_ceiling_tightens_when_cold() {
    // No derate curve, so only the ohmic term can differ; R0 is 5× higher cold.
    let (pack, st) = charge_pack(4.19, 0.10, 0.02, 4.2, 1.0e9, None);
    let cold = outlap_qss::PackState {
        temp_k: -20.0 + CELSIUS_K,
        ..st
    };
    let warm = outlap_qss::PackState {
        temp_k: 40.0 + CELSIUS_K,
        ..st
    };
    let (p_cold, p_warm) = (
        pack.voltage_limited_charge_power_w(&cold),
        pack.voltage_limited_charge_power_w(&warm),
    );
    assert!(
        (p_cold * 5.0 - p_warm).abs() < 1e-6 * p_warm,
        "5× the resistance ⇒ 1/5 the voltage-limited power: cold={p_cold} warm={p_warm}"
    );
    assert!(p_cold < p_warm);
}

/// Above the usable SoC window nothing is accepted, whatever the temperature says.
#[test]
fn full_pack_accepts_nothing_above_the_soc_window() {
    let (pack, st) = charge_pack(3.6, 0.02, 0.02, 4.2, 50_000.0, None);
    let full = outlap_qss::PackState {
        soc: 0.95, // window top is 0.90
        temp_k: 25.0 + CELSIUS_K,
        ..st
    };
    assert_eq!(pack.regen_power_limit_w(&full), 0.0);
}

/// An absent derate curve is an *estimate*, and estimates are surfaced in the loaded-model report —
/// never applied silently (#41). A declared curve is data, so it earns no note.
#[test]
fn an_absent_charge_derate_is_surfaced_as_estimated() {
    let (bare, _) = charge_pack(3.6, R0_NEGLIGIBLE, R0_NEGLIGIBLE, 4.2, 50_000.0, None);
    assert!(!bare.regen_derate_declared());
    assert!(
        bare.notes()
            .iter()
            .any(|n| n.contains("temperature-independent")),
        "the assumption is surfaced: {:?}",
        bare.notes()
    );

    let (declared, _) = charge_pack(
        3.6,
        R0_NEGLIGIBLE,
        R0_NEGLIGIBLE,
        4.2,
        50_000.0,
        Some(plating_derate()),
    );
    assert!(declared.regen_derate_declared());
    assert!(
        declared.notes().is_empty(),
        "a declared curve is data, not an estimate: {:?}",
        declared.notes()
    );
}

/// The ceiling is a positive magnitude at every reachable state — never negative, never NaN.
#[test]
fn charge_acceptance_is_never_negative() {
    let (pack, st) = charge_pack(3.6, 0.10, 0.001, 4.2, 50_000.0, Some(plating_derate()));
    for &soc in &[0.0, 0.25, 0.5, 0.75, 0.89, 0.9, 1.0] {
        for &t in &[-30.0, -20.0, 0.0, 25.0, 40.0, 60.0] {
            let s = outlap_qss::PackState {
                soc,
                temp_k: t + CELSIUS_K,
                ..st
            };
            let p = pack.regen_power_limit_w(&s);
            assert!(p >= 0.0 && p.is_finite(), "soc={soc} T={t} ⇒ {p}");
        }
    }
}

#[test]
fn pulse_response_matches_closed_form_thevenin() {
    // A constant-current discharge pulse from rest: V(t) = OCV − I·R0 − I·R1·(1 − e^{−t/τ}). The
    // exact-exponential RC advance reproduces it at every step, so the RMS error is machine-zero
    // (well under the §13 ≤1% gate).
    let (ocv, r0, r1, tau, i, dt) = (4.0, 0.02, 0.01, 15.0, 50.0, 0.5);
    let (pack, mut st) = constant_pack(ocv, r0, r1, tau);
    let mut sum_sq = 0.0;
    let mut n = 0.0;
    for k in 1..=200 {
        let out = pack.step_current(&mut st, i, dt);
        let t = f64::from(k) * dt;
        let closed = ocv - i * r0 - i * r1 * (1.0 - (-t / tau).exp());
        sum_sq += (out.terminal_v - closed).powi(2);
        n += 1.0;
    }
    let rms = (sum_sq / n).sqrt();
    assert!(
        rms < 1.0e-9 * ocv,
        "pulse RMS {rms} exceeds the closed form"
    );
    // The RC overpotential tracks the closed-form charge state at the elapsed time (t = 200·dt).
    let elapsed = 200.0 * dt;
    assert!((st.v_rc_v - i * r1 * (1.0 - (-elapsed / tau).exp())).abs() < 1.0e-9);
}

#[test]
fn regen_pulse_raises_terminal_voltage() {
    // A charge (regen) current is negative: the terminal sits ABOVE the OCV by I·(R0+R1).
    let (pack, mut st) = constant_pack(4.0, 0.02, 0.01, 15.0);
    let out = pack.step_current(&mut st, -40.0, 300.0); // step ≫ τ → RC fully relaxed
    assert!(
        out.terminal_v > 4.0,
        "charging lifts the terminal above OCV"
    );
    assert!((out.terminal_v - (4.0 + 40.0 * 0.03)).abs() < 1.0e-6);
}

// --- Slow-state behaviour ------------------------------------------------------------------------

#[test]
fn soc_decreases_monotonically_under_discharge() {
    let (pack, mut st) = fixture_pack();
    let mut last = st.soc;
    for _ in 0..400 {
        let out = pack.step_power(&mut st, 120_000.0, 1.0); // 120 kW discharge segments
        assert!(out.soc <= last + 1e-12, "SoC must not rise under discharge");
        last = out.soc;
    }
    assert!(st.soc < 0.98, "the pack drained appreciably over the run");
}

#[test]
fn slow_state_advance_is_deterministic() {
    // Same inputs ⇒ bit-identical trajectory (determinism of the per-segment advance).
    let run = || {
        let (pack, mut st) = fixture_pack();
        let mut trace = Vec::new();
        for k in 0..300 {
            let p = 80_000.0 + 400.0 * f64::from(k);
            let out = pack.step_power(&mut st, p, 0.5);
            trace.push((out.soc, out.terminal_v, out.temp_c));
        }
        trace
    };
    assert_eq!(run(), run(), "the slow-state advance is deterministic");
}

#[test]
fn terminal_voltage_drops_with_soc_and_stays_in_band() {
    // Sweep the fixture pack down its SoC window: OCV (hence the terminal) falls, and the pack
    // terminal voltage stays in the ~620–810 V band that partially overlaps the 730–850 V DU grid.
    let (pack, mut st) = fixture_pack();
    let v_full = pack.terminal_voltage_v(&st);
    for _ in 0..600 {
        pack.step_power(&mut st, 150_000.0, 1.0);
    }
    let v_low = pack.terminal_voltage_v(&st);
    assert!(v_low < v_full, "terminal voltage drops as SoC falls");
    assert!(
        (600.0..=820.0).contains(&v_full) && (600.0..=820.0).contains(&v_low),
        "pack terminal stays in the 620–808 V band ({v_low}..{v_full})"
    );
}

#[test]
fn discharge_is_power_limited_below_the_soc_window() {
    // Draining to the bottom of the SoC window clips the discharge power to zero (the dynamic
    // battery cap that composes with PR5's thermal derate on the traction boundary).
    let (pack, mut st) = fixture_pack();
    for _ in 0..5000 {
        pack.step_power(&mut st, 260_000.0, 1.0);
    }
    assert!(st.soc <= 0.05 + 1e-6, "drained to the SoC-window floor");
    let out = pack.step_power(&mut st, 260_000.0, 1.0);
    assert!(
        out.power_limited,
        "at the floor the discharge demand is clipped"
    );
    assert_eq!(out.current_a, 0.0, "no current is drawn below the window");
}

// --- Vdc–SoC coupling + linear extrapolation -----------------------------------------------------

fn du_with_vdc_map() -> T1Vehicle {
    let loader = fixtures();
    let rv = load_vehicle("pdt_du_rwd/vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let mut car = T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap();
    // Install the 3-D Vdc-stacked table (decoded with the vdc axis name) onto unit 0.
    let bytes = loader.load_bytes("pdt_synth_du_vdc.maps.parquet").unwrap();
    let table: GriddedTable<f64> =
        read_gridded_table(&bytes, &T1Powertrain::map_axis_names_vdc()).unwrap();
    car.install_powertrain_maps(0, &table).unwrap();
    car
}

/// The synthetic map's analytic (unclamped) efficiency — linear in Vdc — mirroring
/// `gen_ptm_maps.py::_drive_unit_eta_vdc` on the interior where no clip is active.
fn analytic_eta_vdc(speed: f64, tau: f64, vdc: f64) -> f64 {
    let base = 0.95 - 0.10 * tau.abs() / 168.0 - 5.0e-5 * speed;
    base + 3.0e-4 * (vdc - 790.0)
}

#[test]
fn vdc_map_reproduces_in_grid_and_extrapolates_below_and_above() {
    let car = du_with_vdc_map();
    let pt = car.powertrain();
    assert!(pt.has_vdc_axis(0), "a Vdc-stacked map is installed");
    let (rpm, tau) = (153.6, 84.0);
    // In-grid nodes: reproduced to interpolation accuracy (the field is linear in Vdc).
    for &v in &[730.0, 790.0, 850.0] {
        let got = pt.efficiency_vdc(0, rpm, tau, v).unwrap();
        assert!(
            (got - analytic_eta_vdc(rpm, tau, v)).abs() < 1e-6,
            "in-grid η at vdc={v}: {got}"
        );
    }
    // Below the grid (a low-SoC terminal voltage) and above it: exact-linear extrapolation.
    for &v in &[620.0, 700.0, 900.0] {
        let got = pt.efficiency_vdc(0, rpm, tau, v).unwrap();
        assert!(
            (got - analytic_eta_vdc(rpm, tau, v)).abs() < 1e-6,
            "extrapolated η at vdc={v}: {got} vs {}",
            analytic_eta_vdc(rpm, tau, v)
        );
    }
    // Monotone in Vdc (higher bus voltage → better efficiency, by construction).
    assert!(
        pt.efficiency_vdc(0, rpm, tau, 620.0).unwrap()
            < pt.efficiency_vdc(0, rpm, tau, 900.0).unwrap()
    );
}

#[test]
fn coupling_presence_matrix_selects_single_or_multi_voltage() {
    // Vdc axis present ⇒ the efficiency (and the machine-heating loss) genuinely track the coupled
    // terminal voltage; a low-SoC (low-voltage) point shifts BOTH traction and heating.
    let car = du_with_vdc_map();
    let pt = car.powertrain();
    let e_lo = pt.energy_at_shaft_vdc(0, 153.6, 84.0, 620.0).unwrap();
    let e_hi = pt.energy_at_shaft_vdc(0, 153.6, 84.0, 900.0).unwrap();
    assert!(e_lo.efficiency < e_hi.efficiency, "η tracks Vdc");
    assert!(
        e_lo.loss_w != e_hi.loss_w,
        "the machine-heating loss tracks Vdc too (feeds PR5's thermal model)"
    );

    // No Vdc axis (the plain 2-D map) ⇒ single-voltage: the Vdc argument is ignored.
    let loader = fixtures();
    let rv = load_vehicle("pdt_du_rwd/vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let mut plain = T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap();
    let bytes = loader.load_bytes("d.ptm.maps.parquet").unwrap();
    let table: GriddedTable<f64> =
        read_gridded_table(&bytes, &T1Powertrain::map_axis_names()).unwrap();
    plain.install_powertrain_maps(0, &table).unwrap();
    let pt = plain.powertrain();
    assert!(!pt.has_vdc_axis(0), "no Vdc axis on the 2-D map");
    let a = pt.efficiency_vdc(0, 153.6, 84.0, 500.0).unwrap();
    let b = pt.efficiency_vdc(0, 153.6, 84.0, 999.0).unwrap();
    assert_eq!(a, b, "single-voltage map ignores the Vdc argument");
    assert_eq!(a, pt.efficiency(0, 153.6, 84.0).unwrap());
}

#[test]
fn pack_terminal_voltage_drives_the_coupled_map() {
    // End-to-end: the pack's SoC-dependent terminal voltage is the value the drive-unit map is
    // evaluated at (the coupling). A full pack and a near-empty pack give different efficiencies.
    let (pack, mut st) = fixture_pack();
    let v_full = pack.terminal_voltage_v(&st);
    let car = du_with_vdc_map();
    let pt = car.powertrain();
    let eta_full = pt.efficiency_vdc(0, 153.6, 84.0, v_full).unwrap();
    for _ in 0..600 {
        pack.step_power(&mut st, 150_000.0, 1.0);
    }
    let v_low = pack.terminal_voltage_v(&st);
    let eta_low = pt.efficiency_vdc(0, 153.6, 84.0, v_low).unwrap();
    assert!(v_low < v_full);
    assert!(
        eta_low < eta_full,
        "the drained pack couples to a lower efficiency"
    );
    assert!(
        (0.3..=0.985).contains(&eta_low),
        "efficiency stays physical"
    );
}
