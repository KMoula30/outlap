// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-vehicle` — the transient (T2) physics blocks: the curvilinear 3-D-road-frame chassis RHS
//! and the tyre / aero / load-transfer / control blocks that feed it (HANDOFF §6.1, §6.2).
//!
//! Every block implements the shared [`outlap_core::block::Block`] trait (pure, generic over
//! `f32`/`f64`, allocation-free) and exchanges data over the flat signal [`outlap_core::bus::Bus`].
//! The chassis is the `integrate`-phase block that writes the chassis-DOF derivatives; the driver
//! (a `control`-phase block) additionally writes its augmented-ODE speed-integral derivative, so the
//! RK sweep advances both. The tyre, aero, load-transfer, driver and powertrain blocks run in
//! `sense`/`control`/`actuate` and publish onto the bus. The concrete hot-loop dispatch and the
//! split-integrator orchestration live in `outlap-transient`; this crate is wasm-clean (no
//! filesystem/threads/clock).
//!
//! The 7-DOF chassis EOM is symbolically verified against a `SymPy` `KanesMethod` derivation to
//! 1e-12 (`docs/derivations/t2_chassis_kane.ipynb`; Decision #32) — see the theory page
//! `docs/theory/transient_chassis.md`.
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::module_name_repetitions,
    // Math kernels use single-letter paper symbols by convention (CLAUDE.md / Decision #33).
    clippy::many_single_char_names,
    clippy::similar_names
)]

pub mod chassis;
pub mod control;
pub mod forces;
pub mod params;

pub use chassis::Chassis;
pub use control::{
    allocate_yaw_moment, drive_weights, preview_distance, Driver, Powertrain, RegenParams,
    TorqueVectoring, YawAllocation, PREVIEW_FLOOR_M,
};
pub use forces::{relax_wheel, Aero, LoadTransfer, RelaxProvider, RelaxTargets, Tire};
pub use params::{ActuationChannels, ChassisParams, RoadChannels, WheelGeometry, G};
