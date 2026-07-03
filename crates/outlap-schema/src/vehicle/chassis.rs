// SPDX-License-Identifier: AGPL-3.0-only
//! Chassis mass / geometry / inertia.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Chassis bulk properties in the ISO 8855 body frame (x forward, y left, z up).
///
/// `inertia` is a diagonal `[Ixx, Iyy, Izz]` for v1 (7/14-DOF tiers); products of inertia are an
/// additive MINOR field later.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Chassis {
    /// Total sprung + unsprung mass, kg.
    pub mass_kg: f64,
    /// Centre of gravity `[x, y, z]`, m, in the body frame.
    pub cg: [f64; 3],
    /// Diagonal moments of inertia `[Ixx, Iyy, Izz]`, kg·m².
    pub inertia: [f64; 3],
    /// Wheelbase, m.
    pub wheelbase_m: f64,
    /// Track width `[front, rear]`, m.
    pub track_m: [f64; 2],
}
