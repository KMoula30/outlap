// SPDX-License-Identifier: AGPL-3.0-only
//! Energy-management policy overlay (§8.3, D-M6-13) — the generic rulebook that governs one or more
//! electric `drivetrain.units[]`.
//!
//! Formerly the singleton `ers:` block that both *described* the MGU-K machine and *ruled* it. Under
//! the D-M6-13 restructure the machine is a first-class `drivetrain.units[]` entry (with its own
//! `.ptm` source and `battery` id); this overlay carries only the *rules* — the deploy taper, the
//! optional override envelope, the harvest/recharge budgets, the regulatory swing window — and names
//! the unit ids it `governs`. Absent ⇒ the electric units run purely as force-adders with no manager
//! (a plain EV). Non-F1 hybrids are the same overlay with different data.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::UnitId;

/// Energy-management policy overlay: the rules governing one or more electric drive units.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Policy {
    /// The `drivetrain.units[]` ids this policy governs (the machines it deploys/harvests through).
    /// Each id must resolve to a declared unit; the governed units are excluded from the mechanical
    /// traction/regen ceilings and driven by the manager's force-adder instead.
    pub governs: Vec<UnitId>,
    /// The regulatory usable-window energy, MJ — the max−min SoC swing allowed on track (FIA 2026
    /// C5.2.9 caps it at 4 MJ). This is a *swing limit*, not a pack capacity; the physical pack
    /// window lives on the governed unit's referenced battery document (`soc_window`).
    pub regulatory_window_mj: f64,
    /// Deployment rules.
    pub deployment: Deployment,
    /// Optional manual override mode (2026 regs).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub override_mode: Option<OverrideMode>,
    /// Recovery rules.
    pub recovery: Recovery,
    /// Fixed electrical→mechanical conversion factor between the CU-K DC bus (where the
    /// regulatory power caps and energy ledgers live) and the crank (FIA 2026 C5.2.14; the
    /// harvest direction uses its inverse per C5.2.21). Default 0.97.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub elec_mech_factor: Option<f64>,
}

/// A power-vs-speed taper table (paired equal-length arrays; monotone non-increasing power).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct SpeedTaper {
    /// Speed breakpoints, km/h (ascending).
    pub speed_kph: Vec<f64>,
    /// Power fraction at each breakpoint, 0..1.
    pub power_frac: Vec<f64>,
}

/// Deployment (discharge) rules.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Deployment {
    /// Peak deployment power, kW — ELECTRICAL DC power at the CU-K bus (FIA 2026 C5.2.7), not
    /// mechanical crank power.
    pub power_limit_kw: f64,
    /// Power-vs-speed taper (evaluated piecewise-linearly — the regulatory closed-form curves of
    /// C5.2.8 are the breakpoints, not samples of a smooth map).
    pub taper_vs_speed: SpeedTaper,
    /// Optional per-lap deployment budget, MJ (electrical). Generic config for non-F1 rule sets —
    /// the 2026 F1 regulations impose NO per-lap deployment budget (deployment is bounded only by
    /// the power curves and the SoC window); leave it absent for a 2026 car. Never estimated.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub per_lap_deploy_mj: Option<f64>,
}

/// Manual override mode: a separate deployment envelope with extra per-lap energy.
///
/// A distinct type (not [`Deployment`]) because it carries `extra_energy_per_lap_mj` and an
/// `activation` policy that base deployment lacks.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct OverrideMode {
    /// Peak override power, kW.
    pub power_limit_kw: f64,
    /// Override power-vs-speed taper (typically a higher-speed taper than base deployment).
    pub taper_vs_speed: SpeedTaper,
    /// Extra per-lap HARVEST allowance while override is active, MJ (estimable; defaults to 0).
    /// FIA 2026 C5.2.10iii: the +0.5 MJ granted with Override is additional *Recharge* (harvest)
    /// allowance — it is NOT a deployment budget.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub extra_energy_per_lap_mj: Option<f64>,
    /// How override is activated.
    #[serde(default)]
    pub activation: Activation,
}

/// Override activation policy.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Activation {
    /// Activated by the energy-management strategy layer (default).
    #[default]
    Strategy,
    /// Activated manually by the driver.
    Manual,
    /// Always available where the regulations permit.
    Automatic,
}

/// Recovery (regen/harvest) rules.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Recovery {
    /// Peak braking-recovery power, kW — ELECTRICAL DC power at the CU-K bus (FIA 2026 C5.2.7
    /// caps both directions at the same bus).
    pub braking_power_limit_kw: f64,
    /// Per-lap harvest ("Recharge") budget, MJ (electrical; FIA 2026 C5.2.10). ALL harvest paths
    /// — braking, part-throttle, ICE-driven — count against this one budget.
    pub per_lap_harvest_mj: f64,
    /// Whether dedicated recharge phases (part-throttle harvest and full-throttle ICE-driven
    /// back-drive on straights) are modelled.
    #[serde(default)]
    pub recharge_phases: bool,
    /// Target pack SoC the automated Recharge paths steer toward (the 2026 ECU's selectable
    /// "Recharge target"). Must lie inside the governed pack's `soc_window`. Default: mid-window.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub recharge_target_soc: Option<f64>,
    /// Recharge-phase ramp: maximum initial power-demand step, kW (FIA 2026 C5.12.4 "power
    /// limited" ramp-down, simplified). Default 150.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ramp_initial_step_kw: Option<f64>,
    /// Recharge-phase ramp: maximum demand-reduction rate after the initial step, kW per second
    /// (C5.12.5, the conservative bound of the reg's 50–100 range). Default 50.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ramp_rate_kw_per_s: Option<f64>,
    /// Recharge-phase ramp: maximum total demand reduction, kW (C5.12.6). Default 700.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ramp_total_kw: Option<f64>,
}
