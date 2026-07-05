// SPDX-License-Identifier: AGPL-3.0-only
//! Suspension: lumped kinematics & compliance (K&C) per axle.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::MapRef;

/// Suspension block: a model selector plus front/rear axle K&C.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Suspension {
    /// Suspension model (v1: lumped K&C only).
    pub model: SuspensionModel,
    /// Front axle K&C.
    pub front: AxleKc,
    /// Rear axle K&C.
    pub rear: AxleKc,
}

/// Suspension model family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionModel {
    /// Lumped kinematics & compliance (ride/roll rates + geometry).
    LumpedKc,
}

/// Per-axle kinematics & compliance.
///
/// `static_ride_height_m`, `anti_dive`, `anti_squat`, `camber_map`, and `toe_map` are **estimable**:
/// when omitted, the estimation stage fills them from documented heuristics (ride height →
/// axle-nominal, kinematic angles → 0, maps → identity) and records a line in the loaded-model
/// report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct AxleKc {
    /// Vertical ride rate at the wheel, N/m.
    pub ride_rate_n_per_m: f64,
    /// Static (design) ride height at the wheel with the car at rest, m (estimable → axle-nominal).
    ///
    /// The reference platform the T1 aero-platform equilibrium (§7.4) compresses under downforce:
    /// `h = static − ΔF_spring / (2·ride_rate)`. Only consumed by the ride-height aero map; the
    /// constant-aero degenerate path ignores it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub static_ride_height_m: Option<f64>,
    /// Fraction of total roll stiffness carried by this axle, 0..1.
    pub roll_stiffness_share: f64,
    /// Roll-centre height, m.
    pub roll_center_height_m: f64,
    /// Anti-dive fraction (estimable).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub anti_dive: Option<f64>,
    /// Anti-squat fraction (estimable).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub anti_squat: Option<f64>,
    /// Camber-vs-travel map reference (estimable → zero).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camber_map: Option<MapRef>,
    /// Toe-vs-travel map reference (estimable → zero).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub toe_map: Option<MapRef>,
}
