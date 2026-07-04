// SPDX-License-Identifier: AGPL-3.0-only
//! Assembly tests for `T0Vehicle`: friction/aero derivation, gear folding, ERS, degraded aero.

use outlap_qss::{T0Error, T0Options, T0Vehicle};
use outlap_schema::io::{FsLoader, MemLoader};
use outlap_schema::{load_vehicle, Conditions, LoadOptions};

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn assembles_f1_2026() {
    let loader = fixtures();
    let v = load_vehicle("f1_2026/vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let t0 =
        T0Vehicle::assemble(&v, &Conditions::default(), &loader, &T0Options::default()).unwrap();

    assert!(close(t0.mass_kg, 768.0, 1e-9));
    // μ now comes from the MF6.1 pure-slip peak @ FNOMIN/p_cold (not raw PD*·LMU*). For this
    // synthetic slick the values are unchanged: no NOMPRES ⇒ dpi = 0, Fz = FNOMIN ⇒ dfz = 0, and
    // Cx = 1.65 / Cy = 1.40 > 1 ⇒ the curve peak equals D exactly (PDX1 = 1.30, PDY1 = 1.25).
    assert!(close(t0.mu_x, 1.30, 1e-9), "mu_x {}", t0.mu_x);
    assert!(close(t0.mu_y, 1.25, 1e-9), "mu_y {}", t0.mu_y);
    // ISA default conditions → ρ ≈ 1.2039 kg/m³; CxA 1.25, CzA 4.5.
    let rho = 100.0 * 1013.25 / (287.05 * 293.15);
    assert!(close(t0.qx, 0.5 * rho * 1.25, 1e-6), "qx {}", t0.qx);
    assert!(close(t0.qz, 0.5 * rho * 4.5, 1e-6), "qz {}", t0.qz);

    // Tractive force is positive through the speed range, and ERS dominates at very low speed
    // (power ÷ speed), so the launch force is large (the friction ellipse caps it in the solver).
    assert!(t0.tractive_force(50.0) > 0.0);
    assert!(
        t0.tractive_force(2.0) > 50_000.0,
        "ERS not contributing: {}",
        t0.tractive_force(2.0)
    );
    assert!(!t0.notes().is_empty());
}

// A minimal single-fixed-ratio EV so the tractive force is exactly τ·ratio·η / r_wheel. The tyre
// is reused from the schema fixtures (all MF6.1 required keys); only the vehicle + a flat ptm are
// authored here.
const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

const FLAT_PTM: &str = "\
schema: ptm/1.0
kind: drive_unit
axes:
  speed_rpm: [0.0, 12000.0]
  load_axis: {torque_nm: [0.0, 300.0]}
  torque_nm: [0.0, 300.0]
tables: {file: x.parquet}
limits:
  max_torque_nm_vs_speed: {speed_rpm: [0.0, 12000.0], torque_nm: [300.0, 300.0]}
inertia_kgm2: 0.05
mass_kg: 60.0
meta: {upstream_ratio_applied: false}
";

const EV_VEHICLE: &str = "\
schema: vehicle/1.0
name: minimal ev
chassis: {mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}
aero:
  map: a.parquet
  axes: []
  constant: {cx_a_m2: 0.7, cz_front_a_m2: 0.0, cz_rear_a_m2: 0.0}
suspension:
  model: lumped_kc
  front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}
  rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}
tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}
drivetrain:
  units:
    - source: ptm/flat.ptm.yaml
      path: [{fixed_ratio: 8.0}]
      wheels: [RL, RR]
brakes:
  balance_bar: 0.6
  disc:
    front: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}
    rear: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}
";

#[test]
fn single_fixed_ratio_force_is_exact() {
    let loader = MemLoader::new()
        .with("vehicle.yaml", EV_VEHICLE)
        .with("ptm/flat.ptm.yaml", FLAT_PTM)
        .with("tyr/slick.tyr.yaml", SLICK);
    let v = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let t0 =
        T0Vehicle::assemble(&v, &Conditions::default(), &loader, &T0Options::default()).unwrap();

    // F = τ·ratio·η / r_wheel = 300 · 8 · 1 / 0.33 (slick UNLOADED_RADIUS), below the rev limit.
    let expect = 300.0 * 8.0 / 0.33;
    // v where shaft ω hits 12000 rpm: (12000·π/30)/(8/0.33) ≈ 51.8 m/s. Check below that.
    for &vv in &[5.0, 20.0, 40.0] {
        assert!(
            close(t0.tractive_force(vv), expect, 1.0),
            "F({vv}) = {} vs {expect}",
            t0.tractive_force(vv)
        );
    }
    // Past the rev limit the only gear drops out → no tractive force.
    assert!(
        t0.tractive_force(60.0) < 1e-9,
        "expected rev-limited at 60 m/s"
    );
}

// A pressure-sensitive slick: NOMPRES (200 kPa) ≠ p_cold (138 kPa) ⇒ dpi ≠ 0, and PPX3/PPY3
// scale the peak μ with pressure. The MF6.1-peak derivation must therefore land BELOW the raw
// PDX1·LMUX = 1.30 / PDY1·LMUY = 1.25 — proving μ is read off the curve, not multiplied.
const PRESSURE_SENSITIVE_TYR: &str = "\
# SPDX-License-Identifier: CC-BY-SA-4.0
schema: tyr/1.0
mf61:
  FNOMIN: 4000.0
  UNLOADED_RADIUS: 0.33
  NOMPRES: 200000.0
  PCX1: 1.65
  PDX1: 1.30
  PEX1: 0.10
  PKX1: 22.0
  PPX3: 0.5
  PCY1: 1.40
  PDY1: 1.25
  PEY1: -1.0
  PKY1: -20.0
  PPY3: 0.5
  LMUX: 1.0
  LMUY: 1.0
thermal:
  c_s: 8000.0
  c_c: 22000.0
  c_g: 1500.0
  g_sc: 90.0
  g_cg: 40.0
  g_road: 250.0
  h0: 15.0
  h1: 5.5
  p_t: 0.65
  t_opt: 95.0
  c_t: 2.2
  k_c: 0.0015
  t_c_ref: 80.0
  p_cold: 138.0
  t_cold: 20.0
wear:
  k_w: 0.0009
  w_max: 8.0
  w_c: 2.0
  tau_d: 600.0
  t_deg: 120.0
  delta_t_ref: 20.0
  beta: 2.0
  delta_c: 0.25
  s_w: 0.5
  delta_d: 0.30
provenance:
  citation: \"Pacejka, Tire and Vehicle Dynamics, 3rd ed. (2012)\"
  source: \"synthetic — pressure-sensitive slick\"
  synthetic: true
";

#[test]
fn pressure_sensitive_tyre_mu_differs_from_pdx1() {
    let vehicle = EV_VEHICLE.replace("tyr/slick.tyr.yaml", "tyr/psens.tyr.yaml");
    let loader = MemLoader::new()
        .with("vehicle.yaml", &vehicle)
        .with("ptm/flat.ptm.yaml", FLAT_PTM)
        .with("tyr/psens.tyr.yaml", PRESSURE_SENSITIVE_TYR);
    let v = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let t0 =
        T0Vehicle::assemble(&v, &Conditions::default(), &loader, &T0Options::default()).unwrap();

    // dpi = (138000 − 200000) / 200000 = −0.31; μx scale = 1 + PPX3·dpi = 0.845 ⇒ μx ≈ 1.099.
    let expect_mu_x = 1.30 * (1.0 + 0.5 * ((138_000.0 - 200_000.0) / 200_000.0));
    let expect_mu_y = 1.25 * (1.0 + 0.5 * ((138_000.0 - 200_000.0) / 200_000.0));
    assert!(
        t0.mu_x < 1.29,
        "μx should be pressure-derated, got {}",
        t0.mu_x
    );
    assert!(
        t0.mu_y < 1.24,
        "μy should be pressure-derated, got {}",
        t0.mu_y
    );
    assert!(close(t0.mu_x, expect_mu_x, 1e-6), "μx {}", t0.mu_x);
    assert!(close(t0.mu_y, expect_mu_y, 1e-6), "μy {}", t0.mu_y);
}

#[test]
fn missing_constant_aero_is_degraded_or_error() {
    let loader = fixtures();
    // ev_1du_rwd is map-only (no aero.constant).
    let v = load_vehicle("ev_1du_rwd/vehicle.yaml", &loader, &LoadOptions::default()).unwrap();

    // Strict: hard error.
    let strict = T0Vehicle::assemble(&v, &Conditions::default(), &loader, &T0Options::default());
    assert!(matches!(strict, Err(T0Error::NoConstantAero)));

    // Degraded: zero aero + a recorded note.
    let opts = T0Options {
        allow_degraded: true,
        ..T0Options::default()
    };
    let t0 = T0Vehicle::assemble(&v, &Conditions::default(), &loader, &opts).unwrap();
    assert!(close(t0.qx, 0.0, 1e-12) && close(t0.qz, 0.0, 1e-12));
    assert!(t0.notes().iter().any(|n| n.contains("zero aero")));
}
