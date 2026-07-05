// SPDX-License-Identifier: AGPL-3.0-only
//! The `.emotor` machine-thermal schema (§9.5) — a data-declared N-node lumped-parameter thermal
//! network (LPTN).
//!
//! One `.emotor` document describes a machine's thermal network at any resolution:
//!
//! * **Hand-authored / lumped** — a handful of role-tagged nodes (winding, stator iron, rotor,
//!   housing, coolant, ambient) with heat capacities and *constant* pairwise conductances. Omitted
//!   capacities/conductances are filled from documented mass heuristics at assembly time and flagged
//!   as estimates. Losses come from the `.ptm` total-loss map, split across nodes (whatever is not
//!   routed goes to the winding node).
//! * **Imported / detailed** — the full FEA-resolved node set (as a PDT importer emits it): explicit
//!   capacities, the constant conduction/contact edges, plus **convection edges** whose conductance
//!   is rebuilt each segment from heat-transfer correlations at the shaft speed and temperatures.
//!   Losses come from the per-component `.ptm` loss maps routed to their nodes.
//!
//! The runtime lives in `outlap-thermal`; assembly of this document into a network lives in
//! `outlap-qss`. **Firewall (Locked Decision #25, amended 2026-07-05):** outlap now *builds* the
//! conductance operator from machine internals for the detailed path — a deliberate, author-authorized
//! amendment for the (open-sourced) thermal model.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::version::SchemaVersion;

/// A data-declared N-node machine-thermal network.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Emotor {
    /// Schema version, e.g. `emotor/1.1`.
    pub schema: SchemaVersion,
    /// Thermal nodes (at least two, one of which is the ambient boundary named in [`Cooling`]).
    pub nodes: Vec<ThermalNode>,
    /// Constant conductance edges (the conduction/contact skeleton).
    pub conductances: Vec<Conductance>,
    /// Speed/temperature-dependent convection edges (detailed path; empty for a lumped model).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub convection: Vec<ConvectionEdge>,
    /// Loss-component→node routing. Empty ⇒ all of the `.ptm` total loss goes to the winding node;
    /// any total-loss fraction not routed also lands on the winding node.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub loss_routing: Vec<LossRoute>,
    /// The ambient boundary node and the optional coolant node.
    pub cooling: Cooling,
    /// Optional DC copper-resistance temperature feedback.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cu_feedback: Option<CuFeedback>,
    /// Optional initial node temperatures (default: every node at its sink temperature).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub initial_temp: Option<InitialTemp>,
    /// Provenance/metadata.
    #[serde(default)]
    pub meta: EmotorMeta,
}

/// One thermal node.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct ThermalNode {
    /// Node name (unique), referenced by conductances, loss routing, and the boundary.
    pub name: String,
    /// Physical role — drives the mass-based capacity/conductance heuristics and identifies the
    /// winding node (the required loss target). Optional for the detailed path, where capacities and
    /// conductances are all explicit.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<NodeRole>,
    /// Heat capacity, J/K. Omit on a lumped node to have it estimated from the machine mass and role.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub c_j_per_k: Option<f64>,
    /// Warning temperature °C — where linear derating begins. Set with `t_max_c` (or neither).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t_warn_c: Option<f64>,
    /// Maximum temperature °C — where the commanded torque limit reaches zero.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t_max_c: Option<f64>,
}

/// The physical role of a node, used for heuristics and for identifying the boundary/winding nodes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum NodeRole {
    /// Copper winding (the binding thermal limit; the default loss target).
    Winding,
    /// Stator core/iron.
    StatorIron,
    /// Rotor / magnet lump (carries the magnet limit for PM machines).
    Rotor,
    /// Housing / case / frame.
    Housing,
    /// Liquid-jacket coolant.
    Coolant,
    /// Ambient boundary.
    Ambient,
    /// An FEA-resolved node with no lumped role (detailed path).
    Other,
}

/// A constant conductance edge `g = 1/R` (W/K) between two nodes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Conductance {
    /// The two node names this conductance connects (order irrelevant; must differ).
    pub between: (String, String),
    /// Conductance `g = 1/R`, W/K (> 0). Omit on a lumped edge to have it estimated from the machine
    /// mass and the two nodes' roles.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub w_per_k: Option<f64>,
}

/// A convection edge whose conductance is recomputed each segment from a heat-transfer correlation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct ConvectionEdge {
    /// The two node names (by convention solid/stator first, rotor/fluid second for the air-gap film).
    pub between: (String, String),
    /// Interface area, m².
    pub area_m2: f64,
    /// The correlation used.
    pub model: ConvModel,
}

/// A convection correlation and its geometry parameters. Externally tagged, e.g.
/// `{air_gap: {r_gap_m: 0.05, gap0_m: 5.0e-4}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ConvModel {
    /// Air-gap film (Becker–Kaye).
    AirGap {
        /// Mean air-gap radius, m.
        r_gap_m: f64,
        /// Cold radial gap, m.
        gap0_m: f64,
        /// Iron linear thermal-expansion coefficient, 1/K (defaults to electrical steel).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        kappa_fe: Option<f64>,
    },
    /// Rotor-driven cavity/end-winding convection.
    RotorAir {
        /// Rotor radius setting the peripheral speed `u = |ω|·r`, m.
        r_rotor_m: f64,
        /// Which Kylander form.
        law: RotorAirLaw,
    },
    /// Rotating-shaft external convection to ambient (Etemad).
    ShaftExternal {
        /// Shaft diameter, m.
        d_shaft_m: f64,
    },
    /// Liquid-cooled channel (Gnielinski/laminar); pump-driven, speed-independent.
    LiquidChannel {
        /// Hydraulic diameter, m.
        hydraulic_diameter_m: f64,
        /// Mean coolant velocity, m/s.
        velocity_mps: f64,
        /// Coolant properties at the film temperature.
        fluid: FluidProps,
    },
    /// Free convection plus linearized radiation to ambient on a cylinder.
    FreeConvection {
        /// Characteristic length, m.
        char_length_m: f64,
        /// Cylinder orientation.
        orientation: Orientation,
        /// Surface emissivity for the radiation term.
        emissivity: f64,
    },
}

/// The rotor-driven convection law.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RotorAirLaw {
    /// End-winding to internal cavity air.
    EndWinding,
    /// Internal cavity air to housing inner.
    InternalAir,
}

/// Cylinder orientation for the free-convection correlation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Orientation {
    /// Horizontal cylinder (characteristic length = diameter).
    Horizontal,
    /// Vertical cylinder (characteristic length = axial length).
    Vertical,
}

/// Liquid-coolant properties at the film temperature.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct FluidProps {
    /// Thermal conductivity λ, W/(m·K).
    pub lam: f64,
    /// Kinematic viscosity ν, m²/s.
    pub nu: f64,
    /// Prandtl number, dimensionless.
    pub pr: f64,
}

/// A loss-component→node routing entry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct LossRoute {
    /// The `.ptm` loss-map column to draw from. Omit for the total-loss column (`loss_w`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub component: Option<String>,
    /// The node the loss is deposited into.
    pub node: String,
    /// Fraction of the component routed to this node, 0..1 (default 1).
    #[serde(default = "default_one")]
    pub fraction: f64,
}

fn default_one() -> f64 {
    1.0
}

/// The ambient boundary and optional coolant node.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Cooling {
    /// Name of the pinned ambient node (its heat capacity is ignored).
    pub ambient_node: String,
    /// Fixed ambient temperature override, °C. When omitted, `conditions.yaml` `ambient_c` is used.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ambient_fixed_c: Option<f64>,
    /// Low-level coolant node (explicit `ρ·c_p·ṁ`) — the escape hatch. Prefer `jacket` for a
    /// liquid jacket, which derives this from raw settings.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub coolant: Option<CoolantSpec>,
    /// A liquid-jacket cooling loop declared by raw settings. The assembly derives the coolant node's
    /// `ρ·c_p·ṁ` and a `housing↔coolant` channel-convection edge (velocity, hydraulic diameter, area).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jacket: Option<JacketSpec>,
    /// An air-gap film coupling declared by raw geometry. The assembly derives an air-gap convection
    /// edge between the two named nodes (interface area from the rotor radius and stack length).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub air_gap: Option<AirGapSpec>,
}

/// A liquid-jacket coolant node: `T_coolant = inlet + Q_in / (2·ρ·c_p·ṁ)`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct CoolantSpec {
    /// Name of the coolant node.
    pub node: String,
    /// Coolant inlet temperature, °C.
    pub inlet_c: f64,
    /// Thermal mass-flow capacity `ρ·c_p·ṁ`, W/K (> 0).
    pub rho_cp_mdot_w_per_k: f64,
}

/// A liquid-jacket cooling loop, described by raw settings a user or importer can read directly
/// (channel geometry + flow + coolant). The assembly derives the coolant thermal-capacity rate
/// `ρ·c_p·ṁ` and the `housing↔coolant` channel-convection edge (`velocity = Q/(n·w·h)`,
/// `D_h = 2wh/(w+h)`, `g = h(velocity, D_h, fluid)·A_wetted`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct JacketSpec {
    /// The (housing-side) node the jacket cools.
    pub housing_node: String,
    /// The coolant node the jacket feeds (closed by the quasi-static balance).
    pub coolant_node: String,
    /// Coolant inlet temperature, °C.
    pub inlet_c: f64,
    /// Volumetric coolant flow, litres/s.
    pub flow_rate_lps: f64,
    /// Number of parallel channels.
    pub channel_count: u32,
    /// Channel width, mm.
    pub channel_width_mm: f64,
    /// Channel height, mm.
    pub channel_height_mm: f64,
    /// Total wetted inner area, m².
    pub wetted_area_m2: f64,
    /// The coolant fluid (a named preset or explicit properties).
    pub fluid: FluidSpec,
}

/// An air-gap film coupling, described by raw rotor geometry. The assembly derives the interface
/// area `A = 2π·r_gap·L` and evaluates the Becker–Kaye film per segment.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct AirGapSpec {
    /// The two nodes the air gap couples (stator side first, rotor side second).
    pub between: (String, String),
    /// Rotor outer radius, mm.
    pub rotor_outer_radius_mm: f64,
    /// Radial air-gap thickness, mm.
    pub gap_mm: f64,
    /// Active (stack) axial length, mm.
    pub stack_length_mm: f64,
}

/// A coolant fluid: a named preset (resolved against a built-in table) or explicit properties.
/// Externally tagged: `{named: ethylene_glycol_50}` or `{props: {rho, cp, lam, nu, pr}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum FluidSpec {
    /// A named preset (e.g. `water`, `ethylene_glycol_50`, `oil`).
    Named(String),
    /// Explicit fluid properties.
    Props(CoolantProps),
}

/// Explicit coolant-fluid properties at the film temperature.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct CoolantProps {
    /// Density ρ, kg/m³.
    pub rho: f64,
    /// Specific heat `c_p`, J/(kg·K).
    pub cp: f64,
    /// Thermal conductivity λ, W/(m·K).
    pub lam: f64,
    /// Kinematic viscosity ν, m²/s.
    pub nu: f64,
    /// Prandtl number, dimensionless.
    pub pr: f64,
}

/// DC copper-resistance feedback: the loss injected at the listed winding nodes is rescaled by
/// `1 + α·(T − T_ref)` each step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct CuFeedback {
    /// Winding node names whose loss is temperature-scaled.
    pub nodes: Vec<String>,
    /// Reference temperature the loss maps were computed at, °C.
    pub t_ref_c: f64,
    /// Resistance-rise coefficient α, per K.
    pub alpha_per_k: f64,
}

/// Optional initial node temperatures. Externally tagged: `{uniform_c: 25}` or
/// `{per_node_c: [{node: winding, temp_c: 90}]}`. Absent ⇒ each node starts at its sink temperature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum InitialTemp {
    /// Every node starts at a uniform temperature, °C.
    UniformC(f64),
    /// Named per-node starting temperatures, °C; unlisted nodes start at their sink temperature.
    PerNodeC(Vec<NodeTemp>),
}

/// A named node temperature, °C.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct NodeTemp {
    /// Node name.
    pub node: String,
    /// Temperature, °C.
    pub temp_c: f64,
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
    /// Imported from a PDT detailed thermal model (network integrated as-is).
    PdtImported,
}
