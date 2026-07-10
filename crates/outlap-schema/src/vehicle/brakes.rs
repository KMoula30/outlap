// SPDX-License-Identifier: AGPL-3.0-only
//! Brakes: balance, per-axle discs, ABS, and optional regen blending.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::MapRef;

/// Brake system.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Brakes {
    /// Brake-balance bar: front bias fraction, 0..1.
    pub balance_bar: f64,
    /// Front/rear brake discs.
    pub disc: AxlePair<BrakeDisc>,
    /// Whether ABS is fitted.
    #[serde(default)]
    pub abs: bool,
    /// Optional regen/friction blending (present when an ERS/battery can recover braking energy).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub regen_blend: Option<RegenBlend>,
}

/// A single brake disc's thermal properties.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct BrakeDisc {
    /// Lumped thermal capacity, J/K.
    pub thermal_capacity_j_per_k: f64,
    /// Convective cooling area, m².
    pub cooling_area_m2: f64,
    /// Optional pad friction-vs-temperature map reference.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pad_mu_vs_temp: Option<MapRef>,
}

/// Regen/friction blending policy for recovered braking.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct RegenBlend {
    /// Maximum fraction of the **driven axle's** brake torque provided by regen, 0..1 (the machine can
    /// only ever brake its own axle). The calipers supply the deficit, so the axle's total brake torque
    /// — and the trajectory — is unchanged: this is series/blended braking, not added deceleration.
    pub max_regen_frac: f64,
    /// Optional front-axle regen bias fraction, 0..1 (defaults to the friction balance if omitted).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub front_bias: Option<f64>,
}

/// A generic front/rear pair.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct AxlePair<T> {
    /// Front-axle value.
    pub front: T,
    /// Rear-axle value.
    pub rear: T,
}
