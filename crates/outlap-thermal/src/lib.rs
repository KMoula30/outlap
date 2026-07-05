// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-thermal` — the machine lumped-parameter thermal network (LPTN).
//!
//! A machine's thermal state is a small set of node temperatures advanced over the QSS solution. A
//! network is `N` nodes, each an isothermal lump with a heat capacity `C_i` (J/K); nodes exchange
//! heat through conductances `g_ij = 1/R_ij` (W/K); machine losses are injected as a per-node source
//! `P_i` (W). One node is a pinned ambient boundary; an optional coolant node is closed each step by
//! a quasi-static jacket energy balance. The state advances with a Crank–Nicolson (trapezoidal) step
//! that is unconditionally stable, so a coarse per-segment step over a lap stays well-behaved.
//!
//! **Two ways a network is built (§8.5, Decision #25 as amended 2026-07-05):**
//!
//! * **Lumped / hand-authored** — a handful of nodes with heat capacities and *constant* pairwise
//!   conductances (given, or filled from mass heuristics). No geometry; `G` is speed-independent.
//! * **Detailed / imported** — the full FEA-resolved node set with the conductance graph rebuilt
//!   **each segment** from geometry and heat-transfer correlations at the segment's shaft speed and
//!   current temperatures (air-gap film, end-cavity convection, liquid-jacket channel, …). This is
//!   the ported PDT thermal model: outlap now *builds* the operator from machine internals, a
//!   deliberate amendment of the powertrain firewall for the (author-owned) thermal model.
//!
//! The correlations ([`correlations`]) are standard published forms (Becker–Kaye/Taylor air-gap,
//! Kylander end-cavity, Etemad shaft, Churchill–Chu free convection, Gnielinski channel); each is
//! cited at its definition. This crate is pure math: the mapping from an `.emotor` document to a
//! [`Network`] lives in `outlap-qss`.
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::module_name_repetitions,
    // Heat-transfer kernels use single-letter symbols from the source correlations (Decision #33).
    clippy::many_single_char_names,
    clippy::similar_names,
    // The Crank–Nicolson assembly and Gaussian elimination read most clearly with explicit i/j/k
    // matrix indices, not iterator adapters.
    clippy::needless_range_loop
)]

pub mod correlations;
pub mod network;

pub use correlations::{AirProps, FluidProps, Orientation};
pub use network::{
    ConvEdge, ConvKind, Coolant, CuFeedback, Edge, Network, RotorAirLaw, ThermalError,
    ThermalState, MAX_NODES,
};
