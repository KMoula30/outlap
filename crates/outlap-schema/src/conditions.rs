// SPDX-License-Identifier: AGPL-3.0-only
//! The `conditions.yaml` schema (§9.6) — the fourth input of the quartet (Locked Decision #46).
//!
//! Same track, different day: air state (→ density for aero), a constant wind vector (v1), the
//! track-surface temperature (the tire thermal boundary `T_road`, §7.2), and a thermal ambient.
//! Every field carries a full ISA default (20 °C, 1013.25 hPa, still air), so the whole document is
//! optional — an absent `conditions.yaml` resolves to [`Conditions::default`].

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::version::SchemaVersion;

/// Standard sea-level ISA air temperature, °C.
const ISA_TEMPERATURE_C: f64 = 20.0;
/// Standard sea-level ISA pressure, hPa.
const ISA_PRESSURE_HPA: f64 = 1013.25;

/// Session conditions: air, wind, and thermal boundaries for one run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Conditions {
    /// Schema version, e.g. `conditions/1.0`.
    pub schema: SchemaVersion,
    /// Air state (drives air density for the aero model).
    #[serde(default)]
    pub air: Air,
    /// Wind vector (constant in v1).
    #[serde(default)]
    pub wind: Wind,
    /// Track-surface temperature (tire thermal boundary `T_road`, §7.2), °C.
    #[serde(default = "default_track_surface_c")]
    pub track_surface_c: f64,
    /// Thermal-model ambient / pre-radiator coolant proxy, °C.
    #[serde(default = "default_ambient_c")]
    pub ambient_c: f64,
}

impl Default for Conditions {
    fn default() -> Self {
        Self {
            schema: SchemaVersion::new(crate::schema_name::CONDITIONS, crate::SCHEMA_MAJOR, 0),
            air: Air::default(),
            wind: Wind::default(),
            track_surface_c: default_track_surface_c(),
            ambient_c: default_ambient_c(),
        }
    }
}

/// Air state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Air {
    /// Air temperature, °C.
    #[serde(default = "default_air_temperature_c")]
    pub temperature_c: f64,
    /// Absolute air pressure, hPa.
    #[serde(default = "default_air_pressure_hpa")]
    pub pressure_hpa: f64,
}

impl Default for Air {
    fn default() -> Self {
        Self {
            temperature_c: default_air_temperature_c(),
            pressure_hpa: default_air_pressure_hpa(),
        }
    }
}

/// A constant wind vector (v1): speed and the compass direction it blows *from*.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Wind {
    /// Wind speed, m/s (default still air).
    #[serde(default)]
    pub speed_mps: f64,
    /// Meteorological direction the wind comes from, degrees (0 = North, 90 = East).
    #[serde(default)]
    pub direction_deg: f64,
}

fn default_air_temperature_c() -> f64 {
    ISA_TEMPERATURE_C
}
fn default_air_pressure_hpa() -> f64 {
    ISA_PRESSURE_HPA
}
fn default_track_surface_c() -> f64 {
    ISA_TEMPERATURE_C
}
fn default_ambient_c() -> f64 {
    ISA_TEMPERATURE_C
}
