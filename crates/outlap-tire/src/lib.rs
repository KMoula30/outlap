// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-tire` — the tire force backbone (HANDOFF §7.1).
//!
//! Steady-state MF6.1 (Pacejka 2012, 3rd ed.) force/moment evaluation: pure and combined slip
//! `Fx`, `Fy`, aligning moment `Mz`, overturning moment `Mx`, and rolling-resistance moment `My`,
//! including the Besselink inflation-pressure terms. Turn-slip is omitted in v1 (all ζ factors
//! are unity; see [`mf61`]). Implemented clean-room from the published book only.
//!
//! Conventions (CLAUDE.md): SI units internally (rad, N, N·m, Pa, m/s), ISO 8855 axes
//! (x forward, y left, z up) with the ISO-W sign set modern `.tir` files use — see [`slip`] for
//! the full sign contract. Kernels are pure, panic-free, allocation-free, and generic over
//! `f32`/`f64`; all fallible work (coefficient extraction, validation) happens at construction.
//!
//! The crate is wasm-clean: no filesystem, threads, or clock access. Parsing/IO lives in
//! `outlap-schema`; this crate consumes the already-loaded [`outlap_schema::tyr::Tyr`].

#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::module_name_repetitions,
    // Paper symbols inside math kernels (Pacejka 2012 notation).
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::doc_markdown
)]

pub mod brush;
pub mod mf61;
pub mod model;
pub mod relax;
pub mod slip;
pub mod thermal;

pub use brush::Brush;
pub use mf61::params::{Mf61BuildError, Mf61Params};
pub use mf61::peak::{peak_mu_x, peak_mu_y};
pub use mf61::Mf61;
pub use model::{TireBuildError, TireModel};
pub use relax::{relax_step, Relaxation};
pub use slip::{SlipState, TireForces};
pub use thermal::{ThermalCouplings, ThermalDrivers, TireThermalRing, TireThermalState};

// Re-export the brush schema block so `outlap_tire::TyrBrush` is a one-stop force-model import.
pub use outlap_schema::tyr::TyrBrush;

/// The loaded-model report note type (re-exported from `outlap-schema`): parameter-extraction
/// degradations are reported as these so they merge into the loaded-model report unchanged.
pub use outlap_schema::load::report::ReportEntry;
