// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-transient` — the T2 lap-orchestration skeleton (HANDOFF §11.2, PR4).
//!
//! Assembles the [`outlap_vehicle`] blocks, runs the fixed-step split integrator
//! ([`outlap_core::integrator`]), and produces a time-indexed [`TransientLap`]. The entry point
//! **receives** the QSS artifacts — the envelope-derived target speed, the racing line, and the road
//! geometry — sampled into a [`LineTable`]; it never computes or caches them, so the crate stays
//! wasm-clean (no filesystem/threads/clock).
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::module_name_repetitions,
    clippy::many_single_char_names,
    clippy::similar_names
)]

pub mod control;
pub mod lap;
pub mod line_table;
pub mod result;
pub mod tire_thermal;

// The solver's public API takes an interned channel table, so callers need the type without
// depending on `outlap-core` directly (the Python extension module is itself named `outlap_core`).
pub use outlap_core::bus::ChannelInterner;

pub use control::{
    ErsGovernor, ErsStepInput, ErsStepOut, LiftSchedule, ShiftEvent, ShiftSchedule, Shifter,
    SlowStack, DOWNSHIFT_HYSTERESIS, SHIFT_CUT_FRACTION,
};
pub use lap::{
    FuelSlow, Provenance, SimConfig, SuspensionSample, T2Blocks, T3Blocks, TierBlocks,
    TransientSolver,
};
pub use line_table::{LineSamples, LineTable, PreviewSample, RoadSample};
pub use result::{TransientLap, Wheels};
pub use tire_thermal::{AxleGeometry, TireThermalStack};
// Re-export the force-block grip override so callers wiring the ring see one import surface.
pub use outlap_vehicle::ThermalGrip;
