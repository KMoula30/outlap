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
///
/// The **T3 fields** (`unsprung_mass_kg`, `damper_bump_n_s_per_m`, `damper_rebound_n_s_per_m`,
/// `arb_stiffness_n_m_per_rad`, `bumpstop`) are optional (a T2 vehicle omits them) and are applied
/// **L/R-symmetrically** across the axle by the 14-DOF chassis. Unlike the estimable fields above,
/// they are **not** estimated: a `tier: t3` vehicle that omits them fails at assembly with a
/// plain-language missing-fields list (they are load-bearing equilibrium/dynamic inputs, so an
/// invented value would be a phantom just like the `per_lap_deploy_mj` trap — the estimation stage
/// must never back-fill them).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct AxleKc {
    /// Vertical ride rate at the wheel, N/m.
    pub ride_rate_n_per_m: f64,
    /// Static (design) ride height at the wheel with the car at rest, m (estimable → axle-nominal).
    ///
    /// The reference platform the T1 aero-platform equilibrium (§7.4) compresses under downforce:
    /// `h = static − ΔF_spring / (2·ride_rate)`. Consumed by the ride-height aero map (T1/T2); the
    /// constant-aero degenerate path ignores it. At **T3** it becomes a suspension equilibrium input
    /// (the static compression carrying the sprung corner load), so a T3 assembly refuses an
    /// *estimated* ride height (`allow_degraded` marks it) rather than silently using the nominal.
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
    /// **T3**: per-corner unsprung mass, kg (rim + tyre + hub-side, L/R-symmetric). The sprung mass
    /// is `chassis.mass_kg − Σ(2·unsprung_mass_kg)`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unsprung_mass_kg: Option<f64>,
    /// **T3**: bump (compression) damping coefficient at the wheel, N·s/m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub damper_bump_n_s_per_m: Option<f64>,
    /// **T3**: rebound (extension) damping coefficient at the wheel, N·s/m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub damper_rebound_n_s_per_m: Option<f64>,
    /// **T3**: absolute anti-roll-bar roll stiffness for this axle, N·m/rad (distinct from the
    /// relative `roll_stiffness_share`, which the T1/T2 algebraic load transfer uses).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arb_stiffness_n_m_per_rad: Option<f64>,
    /// **T3**: progressive bumpstop (C¹-engaged) at this axle's wheels.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bumpstop: Option<Bumpstop>,
}

/// **T3** progressive bumpstop: an extra spring rate that engages once the suspension compresses
/// past `gap_m`, smoothed C¹ (quadratic-to-linear knee) so the RK path never sees a discontinuous
/// force or stiffness at contact.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Bumpstop {
    /// Compression before the bumpstop engages, m (≥ 0).
    pub gap_m: f64,
    /// Bumpstop spring rate once engaged, N/m (> 0).
    pub rate_n_per_m: f64,
}
