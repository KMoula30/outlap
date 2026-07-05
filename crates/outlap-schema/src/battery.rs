// SPDX-License-Identifier: AGPL-3.0-only
//! The `battery/1.0` equivalent-circuit parameter document (§8.4).
//!
//! A battery pack is consumed as a Thevenin equivalent-circuit model: OCV / series resistance R0 /
//! one RC pair (R1, τ1) / entropic coefficient dU/dT tabulated on a `(soc, temp)` grid, plus the
//! `ns × np` pack topology, the SoC window, power limits, and a lumped thermal node. The numeric
//! tables live in a long/tidy parquet sidecar (`soc, temp_c, ocv_v, r0_ohm, r1_ohm, tau1_s,
//! dudt_v_per_k`) referenced by [`BatteryTables::file`]; only the contract metadata is modelled here.
//!
//! The layout mirrors what the PDT BatteryPack importer emits
//! (`python/src/outlap/importers/pdt_h5/battery.py`, §10.4) so a real pack imports without a
//! translation step. The runtime (Thevenin evaluation, SoC / temperature slow states, the Vdc–SoC
//! coupling) lives in `outlap-qss`; this crate is the wire contract only.
//!
//! Clean-room: the equivalent-circuit form and its state equations follow the published NREL
//! `thevenin` model (BSD-3) and the ECM literature it cites — never derived from other simulators.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::MapRef;
use crate::version::SchemaVersion;

/// A battery equivalent-circuit parameter document.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct BatteryDoc {
    /// Schema version, e.g. `battery/1.0`.
    pub schema: SchemaVersion,
    /// The equivalent-circuit family (currently only RC-pair Thevenin).
    pub model: BatteryModelKind,
    /// Series/parallel cell counts.
    pub topology: PackTopology,
    /// Pack capacity (charge and energy).
    pub capacity: PackCapacity,
    /// Usable state-of-charge window `[min, max]`, ascending within `[0, 1]`.
    pub soc_window: [f64; 2],
    /// The equivalent-circuit model: RC-pair count, grid axes, and the table sidecar.
    pub ecm: Ecm,
    /// Operating limits (power vs SoC, cell voltage bounds, C-rate).
    pub limits: PackLimits,
    /// Lumped thermal node (mass·cp, jacket resistance, coolant temperature).
    pub thermal: PackThermal,
    /// Provenance/metadata.
    #[serde(default)]
    pub meta: BatteryMeta,
}

/// The equivalent-circuit model family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum BatteryModelKind {
    /// Thevenin equivalent circuit with one or more series RC pairs.
    RcPairs,
}

/// Series/parallel pack topology (`ns` cells in series × `np` strings in parallel).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PackTopology {
    /// Number of cells in series (sets the pack voltage `V_pack = ns · V_cell`).
    pub ns: u32,
    /// Number of parallel strings (splits the pack current `I_cell = I_pack / np`).
    pub np: u32,
}

/// Pack capacity, as reported by the source pack.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PackCapacity {
    /// Pack charge capacity, A·h (Coulomb counting divides by this).
    pub q_pack_ah: f64,
    /// Pack energy capacity, W·h (nominal; informational).
    pub e_pack_wh: f64,
}

/// The equivalent-circuit model: RC-pair count, table axes, and the sidecar reference.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Ecm {
    /// Number of series RC pairs (currently 1 — the tables carry `r1`/`tau1`).
    pub rc_pairs: u32,
    /// The `(soc, temp_c)` grid axes the sidecar tables are sampled on.
    pub axes: EcmAxes,
    /// The parameter-table sidecar reference and its level (cell vs pack).
    pub tables: BatteryTables,
}

/// The ECM grid axes (both strictly ascending).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct EcmAxes {
    /// State-of-charge breakpoints, 0..1 (ascending).
    pub soc: Vec<f64>,
    /// Temperature breakpoints, °C (ascending).
    pub temp_c: Vec<f64>,
}

/// The parameter-table sidecar reference plus the level its values are stated at.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct BatteryTables {
    /// Reference to the long/tidy parquet sidecar
    /// (`soc, temp_c, ocv_v, r0_ohm, r1_ohm, tau1_s, dudt_v_per_k`).
    pub file: MapRef,
    /// Whether the OCV/resistance values are stated per cell or per pack.
    pub level: TableLevel,
}

/// The level a parameter table's values are stated at.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum TableLevel {
    /// Values are per cell; the pack scales them by `ns`/`np` (voltage ×ns, resistance ×ns/np).
    Cell,
    /// Values are already stated at the pack level (no `ns`/`np` scaling applied).
    Pack,
}

/// Operating limits: peak power vs SoC, cell-voltage bounds, and the maximum C-rate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PackLimits {
    /// Peak discharge power vs SoC, W (a positive-power ceiling).
    pub peak_discharge_power_w_vs_soc: PowerVsSoc,
    /// Peak regen (charge) power vs SoC, W (a positive-magnitude ceiling on charging).
    pub peak_regen_power_w_vs_soc: PowerVsSoc,
    /// Minimum cell terminal voltage, V.
    pub cell_v_min: f64,
    /// Maximum cell terminal voltage, V.
    pub cell_v_max: f64,
    /// Maximum continuous C-rate (informational; power limits bind first).
    pub max_c_rate: f64,
}

/// A power-vs-SoC curve: paired equal-length SoC / power arrays.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PowerVsSoc {
    /// SoC breakpoints, 0..1 (ascending).
    pub soc: Vec<f64>,
    /// Power at each breakpoint, W.
    pub power_w: Vec<f64>,
}

/// The lumped-node thermal parameters of the pack.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PackThermal {
    /// Pack thermal mass, kg (heat capacity `C = mass · cp`).
    pub mass_kg: f64,
    /// Specific heat, J/(kg·K).
    pub cp_j_per_kgk: f64,
    /// Jacket thermal resistance to the coolant, K/W (heat leaves as `(T − T_coolant) / R`).
    pub thermal_resistance_k_per_w: f64,
    /// Coolant/ambient sink temperature, °C.
    pub coolant_temp_c: f64,
}

/// Provenance/metadata for a battery document.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct BatteryMeta {
    /// Human-readable source/provenance note.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    /// Cell name/chemistry note.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cell: Option<String>,
}
