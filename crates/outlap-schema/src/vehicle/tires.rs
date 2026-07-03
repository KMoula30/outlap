// SPDX-License-Identifier: AGPL-3.0-only
//! Tire references (front/rear `.tyr`).

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::TyrRef;

/// Front/rear tire references. Both are required (load-bearing, no sane default).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Tires {
    /// Front-axle tire (`.tyr`).
    pub front: TyrRef,
    /// Rear-axle tire (`.tyr`).
    pub rear: TyrRef,
}
