// SPDX-License-Identifier: AGPL-3.0-only
//! Fuel: on-board fuel mass, its centre-of-gravity offset, and the optional energy-flow limit
//! (§8.1). Fuel is an OPTIONAL vehicle subsystem (`fuel: Option<Fuel>`): absent ⇒ mass is the
//! all-inclusive [`crate::vehicle::Chassis::mass_kg`] and the tier results reproduce v0.3.0
//! byte-identically (D-M6-4b).
//!
//! Mass semantics (D-M6-4a): `chassis.mass_kg` is the ONE all-inclusive car+driver number;
//! `fuel.initial_kg` ADDS on top, so the full-tank reference mass is
//! `m₀ = chassis.mass_kg + fuel.initial_kg`. The QSS grip envelope is built at `m₀`, so the
//! mass/CG correction is exactly 1.0 at lap start and drifts as the tank drains.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Default lower heating value of the fuel, J/kg (≈ pump gasoline / F1 E-fuel, 43 MJ/kg).
#[must_use]
pub fn default_lhv_j_per_kg() -> f64 {
    43.0e6
}

/// On-board fuel: initial race load, tank capacity, optional CG offset, LHV, and flow limit.
///
/// The tank centroid CG offset is expressed relative to the DRY-mass (empty-tank) centre of
/// gravity in the ISO 8855 body frame (x forward, z up). The full-tank CG is the mass-weighted
/// blend of the dry CG and the tank centroid; as fuel burns the CG migrates linearly back toward
/// the dry CG (D-M6-4d).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Fuel {
    /// Initial (race-start) fuel mass, kg. Typical F1 race load 70–80 kg. ADDS on top of the
    /// all-inclusive `chassis.mass_kg`: `m₀ = chassis.mass_kg + initial_kg`.
    pub initial_kg: f64,
    /// Tank capacity, kg (`initial_kg ≤ tank_kg`). Fuel mass is clamped to `[0, tank_kg]`.
    pub tank_kg: f64,
    /// Fuel-tank centroid `[x, z]` offset (m) from the DRY-mass CG, ISO 8855 (+x forward, +z up).
    /// Absent ⇒ CG does not migrate (mass-only feedback).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cg_offset_m: Option<[f64; 2]>,
    /// Lower heating value, J/kg (defaults to 43.0e6). The config home of the fuel↔energy
    /// conversion; ṁ_fuel = P_chem / lhv.
    #[serde(default = "default_lhv_j_per_kg")]
    pub lhv_j_per_kg: f64,
    /// Optional fuel-flow limit (F1-style, §8.1 / FIA C5.2.3–C5.2.5). Energy-only form (D-M6-5):
    /// a constraint on available ICE power, never a clamp on the ṁ accounting.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flow_limit: Option<FuelFlowLimit>,
}

/// The fuel-flow limit as an ENERGY constraint (MJ/h). The kg/h ↔ MJ/h equivalence goes through
/// [`Fuel::lhv_j_per_kg`]; §8.1's "ṁ_max" is recorded as satisfied-by-energy-equivalence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct FuelFlowLimit {
    /// Absolute energy-flow cap, MJ/h (the FIA C5.2.4 100 kg/h ≈ 4300 MJ/h at 43 MJ/kg).
    pub mj_per_h: f64,
    /// Optional low-rpm line (FIA C5.2.5): below `below_rpm`, `EF(MJ/h) = slope·N + intercept`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rpm_line: Option<RpmFlowLine>,
}

/// The FIA C5.2.5 low-rpm fuel-flow line: `EF(MJ/h) = slope·N + intercept` for `N < below_rpm`.
/// F1 2026 values are `below_rpm = 10500`, `slope = 0.27`, `intercept = 165`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct RpmFlowLine {
    /// Crank speed (rpm) below which the linear line applies (at/above it, the flat `mj_per_h`).
    pub below_rpm: f64,
    /// Slope of the line, MJ/h per rpm.
    pub slope_mj_per_h_per_rpm: f64,
    /// Intercept of the line, MJ/h.
    pub intercept_mj_per_h: f64,
}
