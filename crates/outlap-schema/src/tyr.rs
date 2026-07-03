// SPDX-License-Identifier: AGPL-3.0-only
//! The `.tyr` tire schema (§9.4) — an MF6.1 coefficient block plus thermal (§7.2), wear (§7.3),
//! and provenance.
//!
//! HANDOFF gives no worked `.tyr` example, so the modelling here is a deliberate contract choice:
//! the raw MF6.1 coefficients are a validated map keyed by their standard `.tir` names
//! ([`Mf61Coeffs`], checked against [`KNOWN_MF61_KEYS`] with a required-core subset), while the
//! `thermal` and `wear` physics parameters are fully-structured named-field blocks. This keeps the
//! wire contract meaningful and typo-catching without inventing ~100 named coefficient fields.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::version::SchemaVersion;

/// A tire model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Tyr {
    /// Schema version, e.g. `tyr/1.0`.
    pub schema: SchemaVersion,
    /// MF6.1 coefficient block (superset of `.tir`, round-trippable).
    pub mf61: Mf61Coeffs,
    /// Thermal-ring parameters (§7.2).
    pub thermal: TyrThermal,
    /// Wear/cliff parameters (§7.3).
    pub wear: TyrWear,
    /// Provenance / citation.
    pub provenance: TyrProvenance,
}

/// MF6.1 (Pacejka 2012) coefficients keyed by their standard `.tir` names, e.g. `PCX1`, `PKY1`.
///
/// Validated in the semantic stage: unknown coefficient names produce a did-you-mean warning
/// (the spec is silent on the full set, so unknown keys are non-fatal), and any missing member of
/// [`REQUIRED_MF61_KEYS`] is a semantic error.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct Mf61Coeffs(pub BTreeMap<String, f64>);

/// Two-node (tread surface / carcass, plus inflation gas) thermal-ring parameters (§7.2).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TyrThermal {
    /// Tread-surface node thermal capacity, J/K.
    pub c_s: f64,
    /// Carcass/bulk node thermal capacity, J/K.
    pub c_c: f64,
    /// Inflation-gas node thermal capacity, J/K.
    pub c_g: f64,
    /// Surface↔carcass conductance, W/K.
    pub g_sc: f64,
    /// Carcass↔gas conductance, W/K.
    pub g_cg: f64,
    /// Carcass↔road conductance, W/K.
    pub g_road: f64,
    /// Convection intercept `h0` in `h(v) = h0 + h1·v^0.8`, W/(m²·K).
    pub h0: f64,
    /// Convection slope `h1`, W/(m²·K) per (m/s)^0.8.
    pub h1: f64,
    /// Friction-power partition into the surface node, 0..1 (≈0.6–0.7).
    pub p_t: f64,
    /// Optimal grip temperature `T_opt`, °C.
    pub t_opt: f64,
    /// Grip-window width coefficient `c_T`.
    pub c_t: f64,
    /// Carcass-softening coefficient `k_c`.
    pub k_c: f64,
    /// Carcass-softening reference temperature `T_c,ref`, °C.
    pub t_c_ref: f64,
    /// Cold inflation pressure `p_cold`, kPa.
    pub p_cold: f64,
    /// Cold reference temperature `T_cold`, °C.
    pub t_cold: f64,
}

/// Wear and thermal-damage (cliff) parameters (§7.3).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TyrWear {
    /// Wear coefficient `k_w`.
    pub k_w: f64,
    /// New-tread depth `w_max`, mm.
    pub w_max: f64,
    /// Cliff onset tread depth `w_c`, mm.
    pub w_c: f64,
    /// Thermal-damage time constant `τ_D`, s.
    pub tau_d: f64,
    /// Damage-onset temperature `T_deg`, °C.
    pub t_deg: f64,
    /// Damage reference temperature step `ΔT_ref`, K.
    pub delta_t_ref: f64,
    /// Damage exponent `β`.
    pub beta: f64,
    /// Cliff grip drop `Δ_c`, fraction.
    pub delta_c: f64,
    /// Cliff sharpness `s_w`, mm.
    pub s_w: f64,
    /// Thermal-damage grip drop `Δ_D`, fraction.
    pub delta_d: f64,
}

/// Provenance / citation for a tire dataset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TyrProvenance {
    /// Literature citation the coefficients derive from.
    pub citation: String,
    /// Human-readable source note.
    pub source: String,
    /// Whether the dataset is synthetic (physically plausible but not measured).
    #[serde(default)]
    pub synthetic: bool,
}

/// The required-core MF6.1 coefficients: a document missing any of these is a semantic error.
///
/// These are the minimal Pacejka pure-slip shape/peak/stiffness coefficients plus the nominal
/// operating point without which no force can be evaluated.
pub const REQUIRED_MF61_KEYS: &[&str] = &[
    "FNOMIN",
    "UNLOADED_RADIUS",
    // Longitudinal pure slip (Fx0).
    "PCX1",
    "PDX1",
    "PEX1",
    "PKX1",
    // Lateral pure slip (Fy0).
    "PCY1",
    "PDY1",
    "PEY1",
    "PKY1",
];

/// The full set of MF6.1 coefficient names recognized by this loader (Pacejka 2012 / `.tir`).
///
/// Coefficients outside this set are accepted but flagged with a did-you-mean warning, since the
/// spec does not fix the complete list. This set covers the standard structural, longitudinal,
/// lateral, aligning-moment, overturning, rolling-resistance, and scaling families.
pub const KNOWN_MF61_KEYS: &[&str] = &[
    // Structural / nominal.
    "FNOMIN",
    "UNLOADED_RADIUS",
    "WIDTH",
    "RIM_RADIUS",
    "VERTICAL_STIFFNESS",
    "NOMPRES",
    "LONGVL",
    // Longitudinal force, pure slip (Fx0).
    "PCX1",
    "PDX1",
    "PDX2",
    "PDX3",
    "PEX1",
    "PEX2",
    "PEX3",
    "PEX4",
    "PKX1",
    "PKX2",
    "PKX3",
    "PHX1",
    "PHX2",
    "PVX1",
    "PVX2",
    "PPX1",
    "PPX2",
    "PPX3",
    "PPX4",
    // Longitudinal force, combined slip.
    "RBX1",
    "RBX2",
    "RBX3",
    "RCX1",
    "REX1",
    "REX2",
    "RHX1",
    // Lateral force, pure slip (Fy0).
    "PCY1",
    "PDY1",
    "PDY2",
    "PDY3",
    "PEY1",
    "PEY2",
    "PEY3",
    "PEY4",
    "PEY5",
    "PKY1",
    "PKY2",
    "PKY3",
    "PKY4",
    "PKY5",
    "PKY6",
    "PKY7",
    "PHY1",
    "PHY2",
    "PVY1",
    "PVY2",
    "PVY3",
    "PVY4",
    "PPY1",
    "PPY2",
    "PPY3",
    "PPY4",
    "PPY5",
    // Lateral force, combined slip.
    "RBY1",
    "RBY2",
    "RBY3",
    "RBY4",
    "RCY1",
    "REY1",
    "REY2",
    "RHY1",
    "RHY2",
    "RVY1",
    "RVY2",
    "RVY3",
    "RVY4",
    "RVY5",
    "RVY6",
    // Aligning moment (Mz).
    "QBZ1",
    "QBZ2",
    "QBZ3",
    "QBZ4",
    "QBZ5",
    "QBZ6",
    "QBZ9",
    "QBZ10",
    "QCZ1",
    "QDZ1",
    "QDZ2",
    "QDZ3",
    "QDZ4",
    "QDZ6",
    "QDZ7",
    "QDZ8",
    "QDZ9",
    "QDZ10",
    "QDZ11",
    "QEZ1",
    "QEZ2",
    "QEZ3",
    "QEZ4",
    "QEZ5",
    "QHZ1",
    "QHZ2",
    "QHZ3",
    "QHZ4",
    "PPZ1",
    "PPZ2",
    "SSZ1",
    "SSZ2",
    "SSZ3",
    "SSZ4",
    // Overturning moment (Mx).
    "QSX1",
    "QSX2",
    "QSX3",
    "QSX4",
    "QSX5",
    "QSX6",
    "QSX7",
    "QSX8",
    "QSX9",
    "QSX10",
    "QSX11",
    // Rolling resistance (My).
    "QSY1",
    "QSY2",
    "QSY3",
    "QSY4",
    "QSY5",
    "QSY6",
    "QSY7",
    "QSY8",
    // Scaling factors.
    "LFZO",
    "LCX",
    "LMUX",
    "LEX",
    "LKX",
    "LHX",
    "LVX",
    "LCY",
    "LMUY",
    "LEY",
    "LKY",
    "LKYC",
    "LKZC",
    "LHY",
    "LVY",
    "LTR",
    "LRES",
    "LXAL",
    "LYKA",
    "LVYKA",
    "LS",
    "LMX",
    "LMY",
    "LVMX",
    "LGYR",
];
