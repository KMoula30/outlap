// SPDX-License-Identifier: AGPL-3.0-only
//! Energy-recovery system (§8.3) — MGU-K only (MGU-H removed per the 2026 F1 regulations).
//!
//! Non-F1 hybrids are the same block with different data (an LMDh single rear-axle MGU, a road
//! PHEV's P2 machine, or a pure EV where the MGU-K *is* the powertrain).

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::PtmRef;

/// Energy-recovery system.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Ers {
    /// The MGU-K machine (`.ptm`, bidirectional torque/speed/efficiency).
    pub mgu_k: PtmRef,
    /// Energy store sizing/window (battery physics lives in the battery block).
    pub es: EnergyStore,
    /// Deployment rules.
    pub deployment: Deployment,
    /// Optional manual override mode (2026 regs).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub override_mode: Option<OverrideMode>,
    /// Recovery rules.
    pub recovery: Recovery,
}

/// Energy-store sizing and usable state-of-charge window.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct EnergyStore {
    /// Usable capacity, MJ.
    pub capacity_mj: f64,
    /// Usable SOC window `[min, max]`, each in 0..1, ascending.
    pub soc_window: [f64; 2],
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
    /// Peak deployment power, kW.
    pub power_limit_kw: f64,
    /// Power-vs-speed taper.
    pub taper_vs_speed: SpeedTaper,
    /// Optional per-lap deployment budget, MJ (estimable).
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
    /// Extra energy allowance per lap in override, MJ (estimable).
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
    /// Peak braking-recovery power, kW.
    pub braking_power_limit_kw: f64,
    /// Per-lap harvest budget, MJ.
    pub per_lap_harvest_mj: f64,
    /// Whether dedicated recharge phases (off-throttle harvesting) are modelled.
    #[serde(default)]
    pub recharge_phases: bool,
}
