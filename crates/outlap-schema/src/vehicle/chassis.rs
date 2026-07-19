// SPDX-License-Identifier: AGPL-3.0-only
//! Chassis mass / geometry / inertia.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Chassis bulk properties in the ISO 8855 body frame (x forward, y left, z up).
///
/// `inertia` is a diagonal `[Ixx, Iyy, Izz]` for v1 (7/14-DOF tiers); products of inertia are an
/// additive MINOR field later.
///
/// **Inertia convention (D-M6, user-locked Option A).** For the T3 14-DOF tier, `inertia[0]` (roll,
/// `Ixx`) and `inertia[1]` (pitch, `Iyy`) are the **sprung-mass** inertias about the sprung CG — the
/// exact quantities the roll/pitch EOM consume, and how these are measured (a CAD sprung model / K&C
/// rig). `inertia[2]` (yaw, `Izz`) is the **whole-car** yaw inertia (the value the T2 tier uses
/// unchanged, as measured on a spin rig). Roll/pitch are motions of the sprung mass alone, so their
/// resisting inertia is a sprung-mass property; yaw is a whole-car motion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Chassis {
    /// Total sprung + unsprung mass, kg. (At T3 the per-axle `suspension.*.unsprung_mass_kg` split
    /// out of this total gives the sprung mass; the rest is unsprung.)
    pub mass_kg: f64,
    /// Centre of gravity `[x, y, z]`, m, in the body frame.
    pub cg: [f64; 3],
    /// Diagonal moments of inertia `[Ixx, Iyy, Izz]`, kg·m². Roll/pitch (`Ixx`/`Iyy`) are
    /// sprung-mass inertias about the sprung CG; yaw (`Izz`) is whole-car (see the type docs).
    pub inertia: [f64; 3],
    /// Wheelbase, m.
    pub wheelbase_m: f64,
    /// Track width `[front, rear]`, m.
    pub track_m: [f64; 2],
}
