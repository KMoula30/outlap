// SPDX-License-Identifier: AGPL-3.0-only
//! The `.emotor` electric-machine thermal schema (§9.5) — a 2-node lumped network.
//!
//! A winding node and a case node, coupled to each other and to a coolant/ambient sink, driven by
//! the `.ptm` loss maps. Deliberately not PDT-grade: community users need only peak envelope plus
//! losses; a PDT importer *distills* its detailed thermal model into these few parameters. Every
//! field has a documented mass-based default, so a minimal file is node masses + coolant temp +
//! winding `t_max` (the estimation stage fills the rest, logged in the report).

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::version::SchemaVersion;

/// A 2-node electric-machine thermal model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Emotor {
    /// Schema version, e.g. `emotor/1.0`.
    pub schema: SchemaVersion,
    /// The winding and case nodes.
    pub nodes: EmotorNodes,
    /// Inter-node and node-to-sink conductances.
    pub coupling: Coupling,
    /// Cooling sink (liquid or air).
    pub cooling: Cooling,
    /// Loss routing between nodes.
    pub loss_routing: LossRouting,
    /// Provenance/metadata.
    #[serde(default)]
    pub meta: EmotorMeta,
}

/// The two thermal nodes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct EmotorNodes {
    /// Winding node.
    pub winding: ThermalNode,
    /// Case node.
    pub case: ThermalNode,
}

/// A single thermal node: capacity and temperature limits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct ThermalNode {
    /// Thermal capacity, J/K.
    pub c_j_per_k: f64,
    /// Maximum allowable temperature (derating cutoff), °C.
    pub t_max_c: f64,
    /// Warning temperature, °C.
    pub t_warn_c: f64,
}

/// Inter-node and node-to-sink conductances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Coupling {
    /// Winding↔case conductance, W/K.
    pub g_wc_w_per_k: f64,
    /// Case↔coolant/ambient conductance, W/K.
    pub g_cool_w_per_k: f64,
}

/// Cooling sink.
///
/// Externally tagged, so the wire form is `{liquid: {coolant_temp_c: 65}}` or
/// `{air: {ambient_c: 25}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Cooling {
    /// Liquid cooling at a fixed coolant temperature.
    Liquid {
        /// Coolant temperature, °C.
        coolant_temp_c: f64,
    },
    /// Air cooling at a fixed ambient temperature.
    Air {
        /// Ambient temperature, °C.
        ambient_c: f64,
    },
}

/// How machine losses are routed into the thermal nodes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct LossRouting {
    /// Fraction of total loss deposited in the winding node, 0..1.
    /// Ignored if the `.ptm` carries a loss breakdown (then computed per operating point).
    pub winding_split: f64,
    /// Optional copper resistance-rise coefficient `α`, per K (omit to disable the feedback).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub copper_alpha_per_k: Option<f64>,
}

/// Provenance/metadata for an emotor thermal model.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct EmotorMeta {
    /// Where the parameters came from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<EmotorSource>,
    /// Free-form notes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

/// Source/provenance category for emotor parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum EmotorSource {
    /// From a manufacturer datasheet.
    Datasheet,
    /// Estimated from mass-based heuristics.
    Estimated,
    /// Distilled from a detailed PDT thermal model.
    PdtDistilled,
}
