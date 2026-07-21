// SPDX-License-Identifier: AGPL-3.0-only
//! The `vehicle.yaml` schema — car identity (chassis, aero, suspension, tires, drivetrain
//! topology, ERS, battery, brakes). This is the input quartet's centerpiece.

mod aero;
mod battery;
mod brakes;
mod chassis;
mod driver;
mod drivetrain;
mod ers;
mod fuel;
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
pub use ers::{Activation, Deployment, EnergyStore, Ers, OverrideMode, Recovery, SpeedTaper};
pub use fuel::{default_lhv_j_per_kg, Fuel, FuelFlowLimit, RpmFlowLine};
pub use suspension::{AxleKc, Bumpstop, Suspension, SuspensionModel};
pub use tires::Tires;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::{Extensions, PresetRef};
use crate::version::SchemaVersion;

/// A complete vehicle description.
///
/// `extends` and `extensions` are the two contract-level affordances: inheritance is a single
/// `extends:` parent chain (resolved to `None` in the loaded model), and `x-*` vendor keys are
/// gathered into `extensions`. Whole optional subsystems (`ers`, `battery`) are `Option`; see the
/// crate docs for the required-vs-optional rule.
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
    /// Energy-recovery system (MGU-K + energy store), if the car has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ers: Option<Ers>,
    /// Battery equivalent-circuit model reference, if the car has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub battery: Option<Battery>,
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
