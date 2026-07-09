// SPDX-License-Identifier: AGPL-3.0-only
//! Ideal-driver preview/tracking gains (HANDOFF §7.7; Decision #21 ideal-deterministic driver).
//!
//! The transient tiers steer with a MacAdam-style preview law + curvature feed-forward and track the
//! QSS speed profile with a PI loop (`docs/theory/driver.md`). Every gain here is optional: absent
//! fields fall back to literature defaults tuned once on `limebeer_2014_f1` (Decision #8), surfaced
//! as estimated in the loaded-model report. The understeer gradient `K_us` in the curvature
//! feed-forward is deliberately **not** a field — it is derived per-vehicle from the vehicle's own
//! understeer gradient at block assembly, so the same driver data transfers across cars.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Ideal-driver control gains: MacAdam-style preview steering + PI speed tracking (§7.7).
///
/// Optional throughout; unset gains take the `DEFAULT_*` literature values (tuned on
/// `limebeer_2014_f1`) and are reported as estimated. Numbers are SI (rad, m, s); pedal gains are
/// dimensionless-pedal `[0, 1]` per error unit.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Driver {
    /// MacAdam preview time, s (look-ahead distance `L_p = preview_time · v`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preview_time_s: Option<f64>,
    /// Preview lateral-offset-error steer gain, rad/m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preview_gain: Option<f64>,
    /// Heading-error steer gain `k_ψ`, rad/rad.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_gain: Option<f64>,
    /// Yaw-rate damping steer gain `k_r`, rad/(rad/s).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub yaw_damping: Option<f64>,
    /// Maximum road-wheel steer, rad.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_steer_rad: Option<f64>,
    /// Speed-loop proportional gain `k_p`, pedal per (m/s).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_kp: Option<f64>,
    /// Speed-loop integral gain `k_i`, pedal per (m/s·s).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_ki: Option<f64>,
    /// Feed-forward normalising acceleration — the friction-circle radius (the gg-headroom usable
    /// accel that maps a demanded longitudinal acceleration to a `[−1, 1]` pedal command), m/s².
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ff_accel_scale_mps2: Option<f64>,
    /// Sideslip magnitude at which the stability throttle-cut begins, rad.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stability_slip_limit_rad: Option<f64>,
    /// Throttle-cut rate per rad of sideslip past the limit, 1/rad (a lift-when-loose stability aid).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stability_slip_gain: Option<f64>,
}

impl Driver {
    /// Literature default MacAdam preview time, s.
    pub const DEFAULT_PREVIEW_TIME_S: f64 = 0.6;
    /// Literature default preview lateral-error steer gain, rad/m.
    pub const DEFAULT_PREVIEW_GAIN: f64 = 0.2;
    /// Literature default heading-error steer gain, rad/rad.
    pub const DEFAULT_HEADING_GAIN: f64 = 1.0;
    /// Literature default yaw-rate damping steer gain, rad/(rad/s).
    pub const DEFAULT_YAW_DAMPING: f64 = 0.3;
    /// Literature default maximum road-wheel steer, rad (~28.6°).
    pub const DEFAULT_MAX_STEER_RAD: f64 = 0.5;
    /// Literature default speed-loop proportional gain, pedal per (m/s).
    pub const DEFAULT_SPEED_KP: f64 = 0.2;
    /// Literature default speed-loop integral gain, pedal per (m/s·s).
    pub const DEFAULT_SPEED_KI: f64 = 0.05;
    /// Literature default feed-forward normalising acceleration, m/s² (≈ 1.5 g usable).
    pub const DEFAULT_FF_ACCEL_SCALE_MPS2: f64 = 15.0;
    /// Literature default sideslip stability-cut threshold, rad (~2.9°).
    pub const DEFAULT_STABILITY_SLIP_LIMIT_RAD: f64 = 0.05;
    /// Literature default sideslip stability-cut rate, 1/rad.
    pub const DEFAULT_STABILITY_SLIP_GAIN: f64 = 8.0;

    /// The resolved preview time, s (field or literature default).
    #[must_use]
    pub fn preview_time_s(&self) -> f64 {
        self.preview_time_s.unwrap_or(Self::DEFAULT_PREVIEW_TIME_S)
    }
    /// The resolved preview steer gain, rad/m.
    #[must_use]
    pub fn preview_gain(&self) -> f64 {
        self.preview_gain.unwrap_or(Self::DEFAULT_PREVIEW_GAIN)
    }
    /// The resolved heading-error steer gain, rad/rad.
    #[must_use]
    pub fn heading_gain(&self) -> f64 {
        self.heading_gain.unwrap_or(Self::DEFAULT_HEADING_GAIN)
    }
    /// The resolved yaw-rate damping steer gain, rad/(rad/s).
    #[must_use]
    pub fn yaw_damping(&self) -> f64 {
        self.yaw_damping.unwrap_or(Self::DEFAULT_YAW_DAMPING)
    }
    /// The resolved maximum road-wheel steer, rad.
    #[must_use]
    pub fn max_steer_rad(&self) -> f64 {
        self.max_steer_rad.unwrap_or(Self::DEFAULT_MAX_STEER_RAD)
    }
    /// The resolved speed-loop proportional gain, pedal per (m/s).
    #[must_use]
    pub fn speed_kp(&self) -> f64 {
        self.speed_kp.unwrap_or(Self::DEFAULT_SPEED_KP)
    }
    /// The resolved speed-loop integral gain, pedal per (m/s·s).
    #[must_use]
    pub fn speed_ki(&self) -> f64 {
        self.speed_ki.unwrap_or(Self::DEFAULT_SPEED_KI)
    }
    /// The resolved feed-forward normalising acceleration, m/s².
    #[must_use]
    pub fn ff_accel_scale_mps2(&self) -> f64 {
        self.ff_accel_scale_mps2
            .unwrap_or(Self::DEFAULT_FF_ACCEL_SCALE_MPS2)
    }
    /// The resolved sideslip stability-cut threshold, rad.
    #[must_use]
    pub fn stability_slip_limit_rad(&self) -> f64 {
        self.stability_slip_limit_rad
            .unwrap_or(Self::DEFAULT_STABILITY_SLIP_LIMIT_RAD)
    }
    /// The resolved sideslip stability-cut rate, 1/rad.
    #[must_use]
    pub fn stability_slip_gain(&self) -> f64 {
        self.stability_slip_gain
            .unwrap_or(Self::DEFAULT_STABILITY_SLIP_GAIN)
    }
}
