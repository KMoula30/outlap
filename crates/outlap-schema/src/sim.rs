// SPDX-License-Identifier: AGPL-3.0-only
//! The `sim.yaml` schema (§9.7) — simulation settings (Locked Decision #42).
//!
//! Optional; every field defaulted. CLI/API values override the file, and the *resolved* settings
//! embed in every result artifact. The `fz_coupling` mode is a recorded simulation setting
//! (Decision #29); `allow_degraded` is the single documented fallback escape hatch (Decision #40).

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::version::SchemaVersion;

/// Simulation settings for one run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Sim {
    /// Schema version, e.g. `sim/1.0`.
    pub schema: SchemaVersion,
    /// Solver tier.
    #[serde(default)]
    pub tier: Tier,
    /// Fixed integration step, seconds (transient tiers).
    #[serde(default = "default_dt_s")]
    pub dt_s: f64,
    /// Normal-load algebraic-loop coupling mode (Decision #29). `None` (the default) resolves to a
    /// tier-dependent choice at assembly — `one_step_lag` for the QSS tiers (T0/T1), `fixed_point`
    /// for the transient tiers (T2/T3) — via [`Sim::resolved_fz_coupling`]. The *resolved* value is
    /// recorded in every result artifact.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fz_coupling: Option<FzCoupling>,
    /// Fixed-step integrator (transient tiers).
    #[serde(default)]
    pub integrator: Integrator,
    /// Slow-clock decimation: the split integrator advances the slow states (temperatures, wear,
    /// SOC, fuel) once every `slow_decimation` fast steps (transient tiers). Slow states evolve on
    /// 10–100 s timescales, so resolving them at the 1 ms fast step is wasteful — the default 20
    /// gives a 20 ms slow substep at `dt_s = 0.001`. Must be ≥ 1.
    #[serde(default = "default_slow_decimation")]
    pub slow_decimation: u32,
    /// Damped fixed-point iteration knobs for `fz_coupling: fixed_point` (transient tiers). Ignored
    /// under `one_step_lag`.
    #[serde(default)]
    pub fixed_point: FixedPointSettings,
    /// g-g-g-v envelope resolution (QSS tiers).
    #[serde(default)]
    pub envelope: Envelope,
    /// Racing-line source.
    #[serde(default)]
    pub raceline: Raceline,
    /// Allow documented-fallback (degraded) combinations, recorded in the result metadata
    /// (Decision #40).
    #[serde(default)]
    pub allow_degraded: bool,
    /// Flat-track analysis mode: zero the track grade, banking, and vertical curvature so the
    /// g-g-g-v envelope collapses to a flat g-g (the 2-D oracle-comparison mode, e.g. the Limebeer
    /// cross-check). A recorded simulation setting; the physical track file is left untouched.
    #[serde(default)]
    pub flat_track: bool,
}

impl Default for Sim {
    fn default() -> Self {
        Self {
            schema: SchemaVersion::new(crate::schema_name::SIM, crate::SCHEMA_MAJOR, 2),
            tier: Tier::default(),
            dt_s: default_dt_s(),
            fz_coupling: None,
            integrator: Integrator::default(),
            slow_decimation: default_slow_decimation(),
            fixed_point: FixedPointSettings::default(),
            envelope: Envelope::default(),
            raceline: Raceline::default(),
            allow_degraded: false,
            flat_track: false,
        }
    }
}

impl Sim {
    /// The concrete normal-load coupling mode, resolving the tier-dependent default when
    /// `fz_coupling` is `None`: `fixed_point` for the transient tiers (T2/T3), `one_step_lag` for the
    /// QSS tiers (T0/T1). An explicit `fz_coupling` is always honoured as-is (Decision #29).
    #[must_use]
    pub fn resolved_fz_coupling(&self) -> FzCoupling {
        self.fz_coupling.unwrap_or(match self.tier {
            Tier::T2 | Tier::T3 => FzCoupling::FixedPoint,
            Tier::T0 | Tier::T1 => FzCoupling::OneStepLag,
        })
    }
}

/// Solver tier (T0 point-mass … T3 full transient). The same vehicle description drives all tiers
/// (Hard rule #4); this only selects fidelity.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Tier {
    /// Point-mass with constant-μ and a power cap.
    T0,
    /// Quasi-steady-state g-g-g-v lap (the default lap solver).
    #[default]
    T1,
    /// Transient double-track.
    T2,
    /// Full transient with driver model.
    T3,
}

/// Normal-load (Fz) algebraic-loop coupling mode (Decision #29). A recorded simulation setting.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum FzCoupling {
    /// Use the previous step's normal loads (default).
    #[default]
    OneStepLag,
    /// Damped fixed-point iteration to convergence within the step.
    FixedPoint,
}

/// Damped fixed-point iteration settings for the normal-load algebraic loop under
/// `fz_coupling: fixed_point` (HANDOFF §11.2). Each transient step re-solves Fz→forces→accel a few
/// times, blending the new loads with the previous iterate by `damping` (a relaxation factor in
/// `(0, 1]`), stopping when the relative load change falls below `tol` or after `max_iter` sweeps.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct FixedPointSettings {
    /// Relaxation factor blending the new load estimate with the previous iterate, `(0, 1]`
    /// (1.0 = undamped). Lower values trade convergence speed for robustness near the grip limit.
    pub damping: f64,
    /// Relative-change convergence tolerance on the normal loads; the sweep stops once every wheel's
    /// `|ΔF_z| / F_z` falls below this.
    pub tol: f64,
    /// Maximum fixed-point sweeps per step (2–3 is typical; a hard cap keeps the step bounded).
    pub max_iter: u32,
}

impl Default for FixedPointSettings {
    fn default() -> Self {
        Self {
            damping: 0.5,
            tol: 1e-4,
            max_iter: 3,
        }
    }
}

/// Fixed-step integrator (Decision #30 mandates fixed-step in production paths).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Integrator {
    /// Heun (explicit trapezoidal, 2nd order) — the default.
    #[default]
    Heun,
    /// Classical Runge–Kutta, 4th order.
    Rk4,
}

/// g-g-g-v envelope sampling resolution.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Envelope {
    /// Number of speed samples.
    pub v_points: u32,
    /// Number of longitudinal-acceleration samples.
    pub ax_points: u32,
    /// Number of normal-g (banking/grade) samples.
    pub g_normal_points: u32,
}

impl Default for Envelope {
    fn default() -> Self {
        Self {
            v_points: 40,
            ax_points: 25,
            g_normal_points: 7,
        }
    }
}

/// Racing-line source: a generator (v1: min-curvature, Decision #14) or a user-supplied CSV.
///
/// Exactly one of `generator` / `file` may be set; the semantic check enforces it. The default is
/// the min-curvature generator.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Raceline {
    /// Line generator to run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub generator: Option<RacelineGenerator>,
    /// Path to a user-supplied `raceline.csv` (same s-based format), instead of a generator.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub file: Option<String>,
}

impl Default for Raceline {
    fn default() -> Self {
        Self {
            generator: Some(RacelineGenerator::MinCurvature),
            file: None,
        }
    }
}

/// A racing-line generator (v1 ships only the min-curvature QP, §6.3).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RacelineGenerator {
    /// Minimum-curvature line: QP over lateral offset minimizing ∫κ² on the 3D ribbon.
    #[default]
    MinCurvature,
}

fn default_dt_s() -> f64 {
    0.001
}

fn default_slow_decimation() -> u32 {
    20
}
