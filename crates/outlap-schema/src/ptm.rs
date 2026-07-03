// SPDX-License-Identifier: AGPL-3.0-only
//! The `.ptm` neutral powertrain-map schema (§9.2) — **the firewall**.
//!
//! Powertrains are consumed as `.ptm` map files only: a torque/efficiency map over a speed axis
//! and a load axis, plus limit envelopes and scalar inertia/mass. outlap never models machines,
//! inverters, or gearboxes internally. Numeric tables live in a sidecar (parquet) referenced by
//! path; only the contract metadata is modelled here.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::MapRef;
use crate::version::SchemaVersion;

/// A neutral powertrain map.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Ptm {
    /// Schema version, e.g. `ptm/1.0`.
    pub schema: SchemaVersion,
    /// Source kind — determines how the topology consumes this map.
    pub kind: PtmKind,
    /// Map axes.
    pub axes: PtmAxes,
    /// Sidecar table references and column semantics.
    pub tables: PtmTables,
    /// Limit envelopes (only `max_torque_nm_vs_speed` is required).
    pub limits: PtmLimits,
    /// Rotational inertia referred to this map's shaft, kg·m².
    pub inertia_kgm2: f64,
    /// Mass attributed to this unit, kg.
    pub mass_kg: f64,
    /// Free-form provenance/metadata.
    #[serde(default)]
    pub meta: PtmMeta,
}

/// Source kind of a powertrain map.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum PtmKind {
    /// A bare electric machine (torque available at its own shaft; a downstream ratio may apply).
    ElectricMachine,
    /// An internal-combustion engine.
    Ice,
    /// A machine+inverter+gearbox lumped at the wheel-side shaft. The topology must **not** apply
    /// another gear ratio unless [`PtmMeta::upstream_ratio_applied`] is `false`.
    DriveUnit,
}

/// The axes of a powertrain map.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PtmAxes {
    /// Shaft-speed axis, rpm (monotonically increasing).
    pub speed_rpm: Vec<f64>,
    /// Load-axis kind — either an absolute torque axis or a normalized load fraction.
    pub load_axis: LoadAxis,
    /// Torque axis, N·m (the sampled torque grid, paired with `speed_rpm`).
    pub torque_nm: Vec<f64>,
}

/// The load-axis declaration: absolute torque or a normalized fraction.
///
/// Externally tagged, so the wire form is `{torque_nm: [...]}` **or** `{load_fraction: [...]}`.
/// A load fraction is in −1..1 where negative values are the regen quadrant.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum LoadAxis {
    /// Absolute torque breakpoints, N·m.
    TorqueNm(Vec<f64>),
    /// Normalized load-fraction breakpoints, −1..1 (negative = regen).
    LoadFraction(Vec<f64>),
}

/// Sidecar table reference plus the column semantics it must carry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PtmTables {
    /// Reference to the sidecar table (parquet/CSV).
    pub file: MapRef,
    /// Efficiency column present (0..1, covering drive and regen quadrants).
    #[serde(default = "default_true")]
    pub efficiency: bool,
    /// Whether a `loss_w` column is present (must be consistent with `efficiency` if both given).
    #[serde(default)]
    pub loss_w: bool,
}

fn default_true() -> bool {
    true
}

/// Limit envelopes.
///
/// Only `max_torque_nm_vs_speed` (the peak envelope) is required. The others, when present, serve
/// as **validation references**, not the derating mechanism — thermal capability is computed by the
/// `.emotor` model from the loss tables (Decision #25).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PtmLimits {
    /// Peak torque vs speed, N·m (REQUIRED).
    pub max_torque_nm_vs_speed: TorqueCurve,
    /// Continuous torque vs speed, N·m (optional validation reference).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cont_torque_nm_vs_speed: Option<TorqueCurve>,
    /// Overload envelope by duration (optional validation reference).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub overload: Option<Overload>,
    /// Drag/braking torque vs speed, N·m (optional).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drag_torque_nm_vs_speed: Option<TorqueCurve>,
}

/// A torque-vs-speed curve: paired equal-length speed/torque arrays.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TorqueCurve {
    /// Speed breakpoints, rpm (ascending).
    pub speed_rpm: Vec<f64>,
    /// Torque at each breakpoint, N·m.
    pub torque_nm: Vec<f64>,
}

/// Time-limited overload envelope: torque curves valid for each listed duration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Overload {
    /// Durations for which the paired curves are valid, s.
    pub durations_s: Vec<f64>,
    /// Torque curves, one per duration.
    pub torque_nm_vs_speed: Vec<TorqueCurve>,
}

/// Free-form provenance/metadata for a powertrain map.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PtmMeta {
    /// Human-readable source/provenance note.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    /// DC bus voltage the map was measured at, V (electric machines).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_voltage_v: Option<f64>,
    /// For `drive_unit` maps: whether the internal gear ratio is already applied (default `true`).
    /// When `false`, the topology may apply one downstream ratio.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub upstream_ratio_applied: Option<bool>,
}
