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

// The solver's public API takes an interned channel table, so callers need the type without
// depending on `outlap-core` directly (the Python extension module is itself named `outlap_core`).
pub use outlap_core::bus::ChannelInterner;

pub use control::{ShiftEvent, Shifter, SlowStack, DOWNSHIFT_HYSTERESIS, SHIFT_CUT_FRACTION};
pub use lap::{Provenance, SimConfig, T2Blocks, TransientSolver};
pub use line_table::{LineSamples, LineTable, PreviewSample, RoadSample};
pub use result::{TransientLap, Wheels};
