// SPDX-License-Identifier: AGPL-3.0-only
//! The T1 quasi-steady-state double-track tier: per-axle tyres, quasi-static load transfer, and a
//! damped-Newton trim per operating point.
//!
//! [`T1Vehicle`] is the cold assembly (allocations allowed); [`T1Vehicle::trim`] is the
//! zero-allocation, panic-free trim solve consumed by the g-g-g-v envelope generator (PR7). The
//! theory is documented in `docs/theory/t1-trim.md` with citations.

pub mod aero;
pub mod battery;
pub mod powertrain;
pub mod thermal;
pub mod trim;
pub mod vehicle;

pub use aero::{AeroCoeffs, AeroLumped, AeroMap};
pub use battery::{Pack, PackState, StepOut};
pub use powertrain::{DiffModel, EnergyPoint, PrimaryDiff, T1Powertrain};
pub use thermal::MachineThermal;
pub use trim::{TrimInput, TrimOutcome, TrimState};
pub use vehicle::T1Vehicle;
