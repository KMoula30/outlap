// SPDX-License-Identifier: AGPL-3.0-only
//! The `vehicle.yaml` schema — car identity (chassis, aero, suspension, tires, drivetrain
//! topology, ERS, battery, brakes). This is the input quartet's centerpiece.

mod aero;
mod battery;
mod brakes;
mod chassis;
mod driver;
mod drivetrain;
mod fuel;
mod policy;
mod suspension;
mod tires;

pub use aero::{Aero, AeroConstant};
pub use battery::{Battery, BatteryModel};
pub use brakes::{AxlePair, BrakeDisc, Brakes, RegenBlend};
pub use chassis::Chassis;
pub use driver::Driver;
pub use drivetrain::{
    Coupler, CouplerEdge, Diff, DiffKind, DriveControl, DriveUnit, Drivetrain, Efficiency, Gearbox,
    ShiftMap, ShiftMapKind, Split, TorqueVectoring, Wheel,
};
pub use fuel::{default_lhv_j_per_kg, Fuel, FuelFlowLimit, RpmFlowLine};
pub use policy::{Activation, Deployment, OverrideMode, Policy, Recovery, SpeedTaper};
pub use suspension::{AxleKc, Bumpstop, Suspension, SuspensionModel};
pub use tires::Tires;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use std::collections::BTreeMap;

use crate::refs::{BatteryId, Extensions, PresetRef};
use crate::version::SchemaVersion;

/// A complete vehicle description.
///
/// `extends` and `extensions` are the two contract-level affordances: inheritance is a single
/// `extends:` parent chain (resolved to `None` in the loaded model), and `x-*` vendor keys are
/// gathered into `extensions`. Optional subsystems (`policy`, `fuel`) are `Option` and packs are an
/// id-keyed map (`batteries`); see the crate docs for the required-vs-optional rule.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Vehicle {
    /// Schema version, e.g. `vehicle/1.0`.
    pub schema: SchemaVersion,
    /// Optional single preset this document inherits from (resolved away in the loaded model).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub extends: Option<PresetRef>,
    /// Human-readable car name.
    pub name: String,
    /// Chassis mass / geometry / inertia.
    pub chassis: Chassis,
    /// Aerodynamics (map reference + optional constant degenerate coefficients).
    pub aero: Aero,
    /// Suspension kinematics & compliance (lumped K&C).
    pub suspension: Suspension,
    /// Tire references (front/rear `.tyr`).
    pub tires: Tires,
    /// Drivetrain topology graph (sources → couplers → wheels) plus the control layer.
    pub drivetrain: Drivetrain,
    /// Optional energy-management policy overlay governing one or more electric drive units
    /// (deploy taper, override, harvest/recharge budgets). Absent ⇒ electric units run as plain
    /// force-adders with no manager (a pure EV) (§8.3, D-M6-13).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub policy: Option<Policy>,
    /// Battery equivalent-circuit packs, keyed by in-document id and referenced by
    /// `drivetrain.units[].battery`. A single-pack car is a length-1 map (§8.4, D-M6-13).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub batteries: BTreeMap<BatteryId, Battery>,
    /// Brakes (balance, discs, ABS, regen blending).
    pub brakes: Brakes,
    /// On-board fuel (mass, CG offset, flow limit), if the car burns fuel. Absent ⇒ mass is the
    /// all-inclusive `chassis.mass_kg` and results reproduce v0.3.0 byte-identically (§8.1, D-M6-4).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fuel: Option<Fuel>,
    /// Ideal-driver preview/tracking gains for the transient tiers (defaulted; MacAdam preview + PI
    /// speed tracking, §7.7). Absent ⇒ literature defaults, surfaced as estimated.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub driver: Option<Driver>,
    /// `x-*` extension keys (carried through, not interpreted).
    #[serde(default, skip_serializing_if = "Extensions::is_empty")]
    pub extensions: Extensions,
}
