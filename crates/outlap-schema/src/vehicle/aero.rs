// SPDX-License-Identifier: AGPL-3.0-only
//! Aerodynamics: a gridded map reference plus an optional constant degenerate case.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::MapRef;

/// Aerodynamics block.
///
/// The primary representation is a gridded map (sidecar parquet/CSV referenced by `map`) whose
/// axes are named in `axes` (e.g. `["ride_height_f_mm", "ride_height_r_mm", "yaw_deg"]`).
/// A passenger car degenerates to constant `CdA`/`ClA`, given via `constant`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Aero {
    /// Reference to the gridded aero map (parquet/CSV sidecar).
    pub map: MapRef,
    /// Ordered names of the map's input axes.
    pub axes: Vec<String>,
    /// Optional constant coefficients (the passenger-car degenerate case, §7.4).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub constant: Option<AeroConstant>,
}

/// Constant aerodynamic coefficients × frontal area (the degenerate, non-mapped case).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct AeroConstant {
    /// Drag area `C_x·A`, m².
    pub cx_a_m2: f64,
    /// Front downforce area `C_z,front·A`, m².
    pub cz_front_a_m2: f64,
    /// Rear downforce area `C_z,rear·A`, m².
    pub cz_rear_a_m2: f64,
}
