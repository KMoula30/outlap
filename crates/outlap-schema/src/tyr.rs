// SPDX-License-Identifier: AGPL-3.0-only
//! The `.tyr` tire schema (§9.4) — an MF6.1 coefficient block plus thermal (§7.2), wear (§7.3),
//! and provenance.
//!
//! HANDOFF gives no worked `.tyr` example, so the modelling here is a deliberate contract choice:
//! the raw MF6.1 coefficients are a validated map keyed by their standard `.tir` names
//! ([`Mf61Coeffs`], checked against [`KNOWN_MF61_KEYS`] with a required-core subset), while the
//! `thermal` and `wear` physics parameters are fully-structured named-field blocks. This keeps the
//! wire contract meaningful and typo-catching without inventing ~100 named coefficient fields.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::version::SchemaVersion;

/// A tire model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Tyr {
    /// Schema version, e.g. `tyr/1.0` (the `brush` block below requires `tyr/1.1`).
    pub schema: SchemaVersion,
    /// MF6.1 coefficient block (superset of `.tir`, round-trippable).
    pub mf61: Mf61Coeffs,
    /// Optional physical brush-model block (`tyr/1.1`, §7.1). When present, T0 may build a brush
    /// tire from it; when the MF6.1 force core is also complete, both models are available and the
    /// tier picks MF6.1. A `brush` block with only a partial MF6.1 force set is a warning, not an
    /// error — see [`crate::load::semantic::check_tyr`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub brush: Option<TyrBrush>,
    /// Optional structured tyre **vertical** dynamics (`tyr/1.2`, T3): the carcass vertical
    /// spring/damper the 14-DOF chassis uses to produce per-wheel `F_z` from the unsprung
    /// deflection. This is the preferred home for the vertical stiffness over the raw
    /// `VERTICAL_STIFFNESS` MF6.1 map key (which stays supported as a fallback for back-compat);
    /// both tiers' carcass-heat model already read the stiffness with a 250 kN/m fallback.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vertical: Option<TyrVertical>,
    /// Thermal-ring parameters (§7.2).
    pub thermal: TyrThermal,
    /// Wear/cliff parameters (§7.3).
    pub wear: TyrWear,
    /// Provenance / citation.
    pub provenance: TyrProvenance,
}

/// Physical brush-model parameters (Pacejka 2012, ch. 3): a first-principles alternative to the
/// empirical MF6.1 force core, parameterised by tread stiffnesses, a base friction, and the
/// contact half-length. Introduced at `tyr/1.1`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TyrBrush {
    /// Longitudinal tread stiffness `C_κ`, N (force per unit slip-ratio at the origin).
    pub c_kappa_n: f64,
    /// Lateral (cornering) tread stiffness `C_α`, N/rad (force per unit slip-angle at the origin).
    pub c_alpha_n_per_rad: f64,
    /// Base sliding friction coefficient `μ0` (dimensionless; scaled at runtime by `mu_scale_*`).
    pub mu0: f64,
    /// Contact-patch half-length `a`, m (sets the closed-form pneumatic trail `t(0) = a/3`).
    pub patch_half_length_m: f64,
    /// Contact-pressure profile along the patch. Only the classic parabolic profile is modelled.
    #[serde(default)]
    pub pressure_profile: BrushPressureProfile,
}

/// The contact-pressure distribution assumed by the brush model along the contact length.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum BrushPressureProfile {
    /// Parabolic pressure `p(x) ∝ 1 − (x/a)²` — the classic brush assumption (the only option).
    #[default]
    Parabolic,
}

/// Structured tyre **vertical** dynamics (`tyr/1.2`, T3; Pacejka 2012 ch. 1). The tyre carcass acts
/// as a vertical spring — and optionally a light damper — between the unsprung mass and the road,
/// producing the per-wheel normal load `F_z` at T3. Preferred over the raw `VERTICAL_STIFFNESS`
/// MF6.1 coefficient (which stays supported as a fallback so existing `.tyr` files keep working).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TyrVertical {
    /// Tyre vertical stiffness `k_z`, N/m (> 0). Racing-slick-representative ≈ 250 kN/m.
    pub stiffness_n_per_m: f64,
    /// Tyre vertical damping `c_z`, N·s/m (≥ 0). Optional; small — tyres are lightly damped.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub damping_n_s_per_m: Option<f64>,
}

/// MF6.1 (Pacejka 2012) coefficients keyed by their standard `.tir` names, e.g. `PCX1`, `PKY1`.
///
/// Validated in the semantic stage: unknown coefficient names produce a did-you-mean warning
/// (the spec is silent on the full set, so unknown keys are non-fatal); the [`REQUIRED_STRUCTURAL_KEYS`]
/// are always mandatory, and the [`REQUIRED_FORCE_KEYS`] are mandatory unless a [`TyrBrush`] block
/// supplies the force model instead.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct Mf61Coeffs(pub BTreeMap<String, f64>);

/// Two-node (tread surface / carcass, plus inflation gas) thermal-ring parameters (§7.2).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TyrThermal {
    /// Tread-surface node thermal capacity, J/K.
    pub c_s: f64,
    /// Carcass/bulk node thermal capacity, J/K.
    pub c_c: f64,
    /// Inflation-gas node thermal capacity, J/K.
    pub c_g: f64,
    /// Surface↔carcass conductance, W/K.
    pub g_sc: f64,
    /// Carcass↔gas conductance, W/K.
    pub g_cg: f64,
    /// Carcass↔road conductance, W/K.
    pub g_road: f64,
    /// Convection intercept `h0` in `h(v) = h0 + h1·v^0.8`, W/(m²·K).
    pub h0: f64,
    /// Convection slope `h1`, W/(m²·K) per (m/s)^0.8.
    pub h1: f64,
    /// Friction-power partition into the surface node, 0..1 (≈0.6–0.7).
    pub p_t: f64,
    /// Optimal grip temperature `T_opt`, °C.
    pub t_opt: f64,
    /// Grip-window width coefficient `c_T`.
    pub c_t: f64,
    /// Carcass-softening coefficient `k_c`.
    pub k_c: f64,
    /// Carcass-softening reference temperature `T_c,ref`, °C.
    pub t_c_ref: f64,
    /// Cold inflation pressure `p_cold`, kPa.
    pub p_cold: f64,
    /// Cold reference temperature `T_cold`, °C.
    pub t_cold: f64,
}

/// Wear and thermal-damage (cliff) parameters (§7.3).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TyrWear {
    /// Wear coefficient `k_w`.
    pub k_w: f64,
    /// New-tread depth `w_max`, mm.
    pub w_max: f64,
    /// Cliff onset tread depth `w_c`, mm.
    pub w_c: f64,
    /// Thermal-damage time constant `τ_D`, s.
    pub tau_d: f64,
    /// Damage-onset temperature `T_deg`, °C.
    pub t_deg: f64,
    /// Damage reference temperature step `ΔT_ref`, K.
    pub delta_t_ref: f64,
    /// Damage exponent `β`.
    pub beta: f64,
    /// Cliff grip drop `Δ_c`, fraction.
    pub delta_c: f64,
    /// Cliff sharpness `s_w`, mm.
    pub s_w: f64,
    /// Thermal-damage grip drop `Δ_D`, fraction.
    pub delta_d: f64,
}

/// Provenance / citation for a tire dataset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TyrProvenance {
    /// Literature citation the coefficients derive from.
    pub citation: String,
    /// Human-readable source note.
    pub source: String,
    /// Whether the dataset is synthetic (physically plausible but not measured).
    #[serde(default)]
    pub synthetic: bool,
}

/// The `tyr` MINOR version at which the optional [`TyrBrush`] block was introduced. A file that
/// carries a `brush` block but declares an older MINOR (e.g. `tyr/1.0`) gets a warning.
pub const TYR_MINOR_BRUSH: u16 = 1;

/// Structural coefficients required of *every* `.tyr` document — the nominal load and radius that
/// both the MF6.1 and brush models need. A document missing either is a semantic error.
pub const REQUIRED_STRUCTURAL_KEYS: &[&str] = &["FNOMIN", "UNLOADED_RADIUS"];

/// The MF6.1 pure-slip force core (shape/peak/curvature/stiffness for `Fx0` and `Fy0`). Required
/// only when no [`TyrBrush`] block is present: a brush-only tire evaluates forces without them.
/// A file that carries a `brush` block *and* a partial (non-empty, incomplete) force set is a
/// warning — the brush model is used and the stray coefficients are ignored.
pub const REQUIRED_FORCE_KEYS: &[&str] = &[
    // Longitudinal pure slip (Fx0).
    "PCX1", "PDX1", "PEX1", "PKX1", // Lateral pure slip (Fy0).
    "PCY1", "PDY1", "PEY1", "PKY1",
];

/// The full set of MF6.1 coefficient names recognized by this loader (Pacejka 2012 / `.tir`).
///
/// Coefficients outside this set are accepted but flagged with a did-you-mean warning, since the
/// spec does not fix the complete list. This set covers the MF6.1 model coefficient families:
/// structural/nominal, longitudinal, lateral, aligning-moment, overturning, rolling-resistance,
/// first-order relaxation, carcass stiffness, and scaling. The non-coefficient `.tir` housekeeping
/// keys (dimensions, inertia, vertical, operating ranges) are the `.tir` codec's concern and are
/// recognized there, not here.
pub const KNOWN_MF61_KEYS: &[&str] = &[
    // Structural / nominal.
    "FNOMIN",
    "UNLOADED_RADIUS",
    "WIDTH",
    "RIM_RADIUS",
    "VERTICAL_STIFFNESS",
    "NOMPRES",
    "LONGVL",
    "VXLOW",
    // Longitudinal force, pure slip (Fx0).
    "PCX1",
    "PDX1",
    "PDX2",
    "PDX3",
    "PEX1",
    "PEX2",
    "PEX3",
    "PEX4",
    "PKX1",
    "PKX2",
    "PKX3",
    "PHX1",
    "PHX2",
    "PVX1",
    "PVX2",
    "PPX1",
    "PPX2",
    "PPX3",
    "PPX4",
    // Longitudinal force, combined slip.
    "RBX1",
    "RBX2",
    "RBX3",
    "RCX1",
    "REX1",
    "REX2",
    "RHX1",
    // Lateral force, pure slip (Fy0).
    "PCY1",
    "PDY1",
    "PDY2",
    "PDY3",
    "PEY1",
    "PEY2",
    "PEY3",
    "PEY4",
    "PEY5",
    "PKY1",
    "PKY2",
    "PKY3",
    "PKY4",
    "PKY5",
    "PKY6",
    "PKY7",
    "PHY1",
    "PHY2",
    "PVY1",
    "PVY2",
    "PVY3",
    "PVY4",
    "PPY1",
    "PPY2",
    "PPY3",
    "PPY4",
    "PPY5",
    // Lateral force, combined slip.
    "RBY1",
    "RBY2",
    "RBY3",
    "RBY4",
    "RCY1",
    "REY1",
    "REY2",
    "RHY1",
    "RHY2",
    "RVY1",
    "RVY2",
    "RVY3",
    "RVY4",
    "RVY5",
    "RVY6",
    // Aligning moment (Mz).
    "QBZ1",
    "QBZ2",
    "QBZ3",
    "QBZ4",
    "QBZ5",
    "QBZ6",
    "QBZ9",
    "QBZ10",
    "QCZ1",
    "QDZ1",
    "QDZ2",
    "QDZ3",
    "QDZ4",
    "QDZ6",
    "QDZ7",
    "QDZ8",
    "QDZ9",
    "QDZ10",
    "QDZ11",
    "QEZ1",
    "QEZ2",
    "QEZ3",
    "QEZ4",
    "QEZ5",
    "QHZ1",
    "QHZ2",
    "QHZ3",
    "QHZ4",
    "PPZ1",
    "PPZ2",
    "SSZ1",
    "SSZ2",
    "SSZ3",
    "SSZ4",
    // Overturning moment (Mx).
    "QSX1",
    "QSX2",
    "QSX3",
    "QSX4",
    "QSX5",
    "QSX6",
    "QSX7",
    "QSX8",
    "QSX9",
    "QSX10",
    "QSX11",
    "PPMX1",
    // Rolling resistance (My).
    "QSY1",
    "QSY2",
    "QSY3",
    "QSY4",
    "QSY5",
    "QSY6",
    "QSY7",
    "QSY8",
    // First-order relaxation lengths (σ_κ / σ_α transients, §7.1).
    "PTX1",
    "PTX2",
    "PTX3",
    "PTY1",
    "PTY2",
    // Carcass / structural stiffness (`.tir [STRUCTURAL]`; σ fallback route).
    "LONGITUDINAL_STIFFNESS",
    "LATERAL_STIFFNESS",
    "PCFX1",
    "PCFX2",
    "PCFX3",
    "PCFY1",
    "PCFY2",
    "PCFY3",
    // Scaling factors.
    "LFZO",
    "LCX",
    "LMUX",
    "LEX",
    "LKX",
    "LHX",
    "LVX",
    "LCY",
    "LMUY",
    "LEY",
    "LKY",
    "LKYC",
    "LKZC",
    "LHY",
    "LVY",
    "LTR",
    "LRES",
    "LXAL",
    "LYKA",
    "LVYKA",
    "LS",
    "LMX",
    "LMY",
    "LVMX",
    "LGYR",
    "LSGKP",
    "LSGAL",
];
