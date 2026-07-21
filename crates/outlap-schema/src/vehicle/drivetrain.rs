// SPDX-License-Identifier: AGPL-3.0-only
//! Drivetrain topology graph (§8.0) — the versatility surface.
//!
//! The powertrain is a directed graph, not a fixed layout: torque **sources** (`.ptm` files:
//! ICE, electric machines, or lumped drive units) connect to wheel **sinks** through ordered
//! **coupler** elements (gearbox, differential, fixed ratio). Any four-wheeled concept is a
//! topology plus data. The load-time topology-graph check validates reachability and conflicts.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::{BatteryId, EmotorRef, MapRef, NodeId, PtmRef, UnitId};

/// The drivetrain: one or more drive units plus the control layer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Drivetrain {
    /// Torque sources and the coupler paths from each to its wheels.
    pub units: Vec<DriveUnit>,
    /// Shared couplers on the drivetrain graph: elements that join a named node (a source's
    /// `output`) to another node or to wheels (§8.0, D-M6-13). Absent ⇒ every unit drives its own
    /// private `wheels` chain (the `wheels:` sugar) ⇒ byte-identical to the pre-2.0 layout.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub couplers: Vec<CouplerEdge>,
    /// Static splits and torque-vectoring control (defaulted).
    #[serde(default)]
    pub control: DriveControl,
    /// Optional named up-shift maps selectable per station by a `u(s)` `shift_map_id` schedule
    /// (§8.3, D-M6-9). The DERIVED schedule (from the gear force curves) is the implicit default
    /// map (id 0); a `shift_maps` entry with `name == "default"` overrides that default. Absent ⇒
    /// only the derived default exists ⇒ byte-identical to pre-1.8.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub shift_maps: Vec<ShiftMap>,
}

/// A named up-shift map: either explicit per-gear crossover speeds or a scalar factor on the
/// derived schedule (D-M6-9). Selected per station by a `u(s)` `shift_map_id` schedule.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct ShiftMap {
    /// Map name — must be unique across `shift_maps`. `"default"` overrides the derived default.
    pub name: String,
    /// How this map defines its up-shift speeds.
    #[serde(flatten)]
    pub kind: ShiftMapKind,
}

/// The two ways a [`ShiftMap`] can define its up-shift crossover speeds.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ShiftMapKind {
    /// Explicit per-gear up-shift crossover speeds, m/s (length must equal the up-shift count, i.e.
    /// one fewer than the gear count). Index 0 = the 1→2 up-shift speed.
    UpshiftSpeedsMps(Vec<f64>),
    /// A single positive multiplier applied to every derived up-shift speed (`> 1` shifts later,
    /// `< 1` shifts earlier). A factor of exactly `1.0` reproduces the derived default.
    Factor(f64),
}

/// A single torque source and the coupler path from it to its terminus.
///
/// The terminus is **exactly one of** (semantic XOR, checked at load): a non-empty `wheels` list
/// (the private-chain sugar — the source drives those wheels straight through its `path`), **or**
/// an `output` node id (the source joins a shared drivetrain node, and top-level
/// [`Drivetrain::couplers`] carry the torque onward).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct DriveUnit {
    /// Unique in-document id for this source (§8.0, D-M6-13). Targeted by `policy.governs` and the
    /// `.ptm` sidecar-install order; disjoint from node ids.
    pub id: UnitId,
    /// The `.ptm` map for this source (ICE, electric machine, or lumped drive unit).
    pub source: PtmRef,
    /// Optional id of the `batteries` map entry this source draws from / harvests into (electric
    /// machines only). Absent for the ICE and for purely-mechanical units.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub battery: Option<BatteryId>,
    /// Optional `.emotor` thermal model — electric machines only (§9.5).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thermal: Option<EmotorRef>,
    /// The source's private series reduction toward its terminus (present only for an actual
    /// step-up/down). Empty when the source outputs directly onto a shared node.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub path: Vec<Coupler>,
    /// The wheels this unit ultimately drives (the private-chain terminus). Empty when the source
    /// joins a shared node via `output` instead.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub wheels: Vec<Wheel>,
    /// The shared node this source outputs onto (the shared-graph terminus). Mutually exclusive
    /// with a non-empty `wheels`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub output: Option<NodeId>,
}

/// A coupler on the shared drivetrain graph: a [`Coupler`] joining a source node (`from`) to
/// another node (`to`) or terminating at `wheels` (§8.0, D-M6-13).
///
/// The downstream terminus is **exactly one of** `{to, wheels}` (semantic XOR, checked at load),
/// mirroring [`DriveUnit`]'s terminus rule. Reuses the [`Coupler`]/[`Gearbox`]/[`Diff`] shapes
/// verbatim under a `coupler:` key (the same enum-tagged form `units[].path` uses), so the wire
/// form is `{coupler: {gearbox: {…}}, from: crank, to: gearbox_out}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct CouplerEdge {
    /// The coupler element (gearbox / diff / fixed ratio) carried on this edge.
    pub coupler: Coupler,
    /// The upstream node this coupler takes torque from.
    pub from: NodeId,
    /// The downstream node this coupler feeds (mutually exclusive with a non-empty `wheels`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub to: Option<NodeId>,
    /// The wheels this coupler terminates at (mutually exclusive with `to`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub wheels: Vec<Wheel>,
}

/// A coupler element on a drive path.
///
/// Externally tagged (serde default), so the wire forms are `{gearbox: {...}}`, `{diff: {...}}`,
/// and `{fixed_ratio: 2.4}`. A standalone clutch coupler is deferred; shift/clutch dynamics live
/// inside [`Gearbox`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Coupler {
    /// A multi-ratio gearbox with a final drive.
    Gearbox(Gearbox),
    /// A differential (open/locked/LSD/solid).
    Diff(Diff),
    /// A single fixed reduction ratio.
    FixedRatio(f64),
}

/// A gearbox: ordered ratios, final drive, shift time, and efficiency.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Gearbox {
    /// Forward gear ratios (index 0 = first gear).
    pub ratios: Vec<f64>,
    /// Final-drive ratio.
    pub final_drive: f64,
    /// Shift time, s.
    pub shift_time_s: f64,
    /// Mechanical efficiency (constant or map). Defaults to a constant 0.985.
    #[serde(default = "Efficiency::default_985")]
    pub efficiency: Efficiency,
}

/// Drivetrain efficiency: a single constant or a gridded map reference.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum Efficiency {
    /// A single constant efficiency, e.g. `0.985`.
    Constant(f64),
    /// A gridded efficiency map (parquet/CSV sidecar), e.g. `{map: eff.parquet}`.
    Map {
        /// The map reference.
        map: MapRef,
    },
}

impl Efficiency {
    /// Default gearbox efficiency (constant 0.985).
    pub fn default_985() -> Self {
        Efficiency::Constant(0.985)
    }
}

/// A differential.
///
/// `preload_nm` is **conditionally required**: the semantic stage requires it for
/// [`DiffKind::Lsd`] and [`DiffKind::Locked`]. `ramp` (`[accel, decel]`) applies to LSDs only.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Diff {
    /// The differential kind. On the wire this is the key `type`.
    #[serde(rename = "type")]
    pub kind: DiffKind,
    /// Preload torque, N·m (required for `lsd`/`locked`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preload_nm: Option<f64>,
    /// LSD ramp angles/fractions `[accel, decel]` (LSD only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ramp: Option<[f64; 2]>,
}

/// Differential kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum DiffKind {
    /// Free differential.
    Open,
    /// Fully locked differential.
    Locked,
    /// Limited-slip differential (preload + ramp).
    Lsd,
    /// Solid axle (locked-diff limit case; day-1 support for karts/live axles).
    Solid,
}

/// A wheel identifier. Serialized uppercase (`FL`, `FR`, `RL`, `RR`).
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema,
)]
#[serde(rename_all = "UPPERCASE")]
pub enum Wheel {
    /// Front-left.
    Fl,
    /// Front-right.
    Fr,
    /// Rear-left.
    Rl,
    /// Rear-right.
    Rr,
}

impl Wheel {
    /// All four wheels, in canonical order.
    pub const ALL: [Wheel; 4] = [Wheel::Fl, Wheel::Fr, Wheel::Rl, Wheel::Rr];

    /// Whether this wheel is on the front axle.
    pub fn is_front(self) -> bool {
        matches!(self, Wheel::Fl | Wheel::Fr)
    }

    /// The axle label (`front`/`rear`) for messages.
    pub fn axle(self) -> &'static str {
        if self.is_front() {
            "front"
        } else {
            "rear"
        }
    }
}

impl std::fmt::Display for Wheel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Wheel::Fl => "FL",
            Wheel::Fr => "FR",
            Wheel::Rl => "RL",
            Wheel::Rr => "RR",
        };
        f.write_str(s)
    }
}

/// The rule-based control layer: static splits + torque vectoring.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct DriveControl {
    /// Static torque splits.
    #[serde(default)]
    pub split: Split,
    /// Yaw-moment torque vectoring.
    #[serde(default)]
    pub torque_vectoring: TorqueVectoring,
}

/// Static torque splits. `front` is the front-axle share; `left` is the left-side share.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Split {
    /// Front-axle torque share, 0..1 (omit for single-axle cars).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub front: Option<f64>,
    /// Left-side torque share, 0..1 (omit unless per-side allocation applies).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub left: Option<f64>,
}

/// Yaw-moment-proportional torque vectoring: `ΔM_z = k_yaw · (r_target − r)`, with the demanded
/// moment physically allocated across the driven wheels within their friction-ellipse and
/// machine-envelope limits (HANDOFF §8.0; the allocator interface is shaped so a QP replaces the
/// rule-based split post-v1, Decision #2).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TorqueVectoring {
    /// Whether torque vectoring is enabled.
    #[serde(default)]
    pub enabled: bool,
    /// Yaw-rate feedback gain `k_yaw` (N·m per rad/s).
    #[serde(default)]
    pub k_yaw: f64,
    /// Optional hard cap on the commanded yaw moment `|ΔM_z|`, N·m (a machine-envelope proxy). When
    /// omitted, the friction-ellipse per-wheel limits alone bound the allocation.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_yaw_moment_nm: Option<f64>,
}
