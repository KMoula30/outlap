// SPDX-License-Identifier: AGPL-3.0-only
//! Battery equivalent-circuit model reference (§8.4).
//!
//! The detailed RC-pair parameter file (OCV/R0/R1/τ1 vs SOC & T, entropic heating, pack scaling)
//! stays a path-reference this milestone — it is modelled fully in a later increment.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::BatteryRef;

/// Battery block: an equivalent-circuit model selector plus its parameter file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Battery {
    /// The equivalent-circuit model family.
    pub model: BatteryModel,
    /// Reference to the parameter file (Thevenin RC-pair parameters).
    pub params: BatteryRef,
}

/// Battery model family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum BatteryModel {
    /// Thevenin equivalent circuit with one or more RC pairs.
    RcPairs,
}
