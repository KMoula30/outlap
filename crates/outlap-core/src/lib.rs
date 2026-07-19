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

pub mod assembler;
pub mod block;
pub mod bus;
pub mod gridmap;
pub mod integrator;
pub mod interp;
pub mod relax;
pub mod spline;
pub mod state;

pub use assembler::{assemble, AssemblyError, BlockSpec, Schedule};
pub use block::{Block, Phase, Ports};
pub use bus::{
    core_channel_count, Bus, ChannelId, ChannelInterner, CoreSignal, WheelSignal, WHEELS,
};
pub use gridmap::{EvalFlags, GridMapError, GriddedMapN, GriddedTable, OutOfDomain, MAX_DIMS};
pub use integrator::{
    back_interpolate, ButcherTableau, EventQueue, RkMethod, ScheduledEvent, SimArena,
};
pub use interp::{InterpError, MonotoneCubic, PiecewiseLinear};
pub use relax::{exact_exponential, semi_implicit_decay, SlowClock};
pub use spline::{CubicSpline, SplineError};
pub use state::{
    fast_slot_count, ChassisState, ControllerState, DerivView, RelaxState, SlowDerivView,
    SlowStateView, StateLayout, StateView,
};
