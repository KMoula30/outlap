// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-powertrain` — the tier-agnostic ERS rulebook and energy manager (HANDOFF §8.3, M6).
//!
//! This crate is the single home of the 2026-Formula-1-style ERS *rules*: deployment speed
//! tapers, the override ("Overtake") envelope, per-lap energy budgets, the recharge-phase
//! power-ramp bounds, and the electrical↔mechanical conversion seam. Both solver families (the
//! QSS T0/T1 march and the transient T2/T3 loop) consume the SAME implementation, so the tier
//! parity gate compares physics — never two hand-written copies of the regulations (D-M6-2).
//!
//! It is a **clean-room flagship model**: implemented from the FIA 2026 Formula 1 Regulations,
//! Section C \[Technical\] Issue 19 (2026-06-25) and Section B \[Sporting\] Issue 07 (2026-06-25)
//! — no other project was consulted. Article numbers are cited at each rule; the companion theory
//! page is `docs/theory/ers-energy-manager.md`. Non-F1 hybrids are the same rulebook with
//! different data (a GT hybrid's 120 kW MGU with a 3 MJ harvest budget is just another `ers:`
//! block).
//!
//! # Design
//!
//! * [`ErsRulebook`] — the regulations as pure data + queries, built once from the loaded
//!   `ers:` schema block (SI internally: W, J, m/s; kph/kW/MJ converted at construction ONLY).
//!   Regulatory tapers are evaluated **piecewise-linearly** ([`outlap_core::PiecewiseLinear`], the
//!   recorded Decision #30 exception) — the C5.2.8 curves are closed-form regulation lines, and a
//!   Hermite through their breakpoints bows up to +78 kW above the rulebook at 315 km/h.
//! * [`LapEnergyLedger`] — the per-lap deploy/harvest integrals on the ELECTRICAL side (the CU-K
//!   DC bus, where C5.2.7/C5.2.10 place every cap). The caller owns the clock and the lap
//!   boundaries: it calls [`LapEnergyLedger::record`] each step and [`LapEnergyLedger::reset`] at
//!   the start line.
//! * [`EnergyManager`] — a pure `decide(inputs, ledger) → ErsCommand` control-phase policy
//!   (§6.2b: sense → control → actuate → integrate; mode changes are step-boundary events,
//!   §11.2). The policy is an enum (D-M6-9): [`DeployPolicy::RuleBased`] (greedy feed-forward
//!   deploy, D-M6-8, plus the automated Recharge paths) or [`DeployPolicy::Schedule`] (a
//!   data-driven `u(s)` control vector, the stage-2 strategy surface). Named `DeployPolicy` to
//!   avoid colliding with the schema's `vehicle::Policy` overlay (D-M6-13).
//!
//! Everything is generic over `f32`/`f64`, allocation-free after construction, and wasm-clean
//! (no filesystem, threads, or clock).

#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::module_name_repetitions,
    clippy::doc_markdown
)]

mod ledger;
mod manager;
mod rulebook;
mod schedule;

pub use ledger::LapEnergyLedger;
pub use manager::{DecideInput, DeployPolicy, EnergyManager, ErsCommand, ErsMode};
pub use rulebook::{ErsRulebook, RulebookError};
pub use schedule::{ScheduleError, UsSchedule};
