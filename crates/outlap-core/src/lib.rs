// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-core` — shared numerics and primitives for the outlap solvers.
//!
//! This crate is wasm-clean (no filesystem, threads, or clock) and dependency-light; it holds the
//! math shared across the file-format layer and the T0–T3 tiers. Its first inhabitant is the one
//! shared gridded-map interpolant ([`interp`], Decision #30).
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

pub mod gridmap;
pub mod interp;
pub mod spline;

pub use gridmap::{EvalFlags, GridMapError, GriddedMapN, GriddedTable, OutOfDomain, MAX_DIMS};
pub use interp::{InterpError, MonotoneCubic};
pub use spline::{CubicSpline, SplineError};
