// SPDX-License-Identifier: AGPL-3.0-only
//! The [`Block`] abstraction (HANDOFF §6.2): a block is *a declaration + an eval contract*.
//!
//! A block is `(immutable parameters, states, typed ports on the [`Bus`](crate::bus::Bus))`. It
//! exposes three pure, `f32`/`f64`-generic evaluations — an algebraic [`equilibrium`](Block::equilibrium)
//! (T0/T1 trim), an ODE [`derivatives`](Block::derivatives) (T2/T3 fast RHS), and a
//! [`slow_derivatives`](Block::slow_derivatives) (thermal/wear/SOC on the decimated slow clock) —
//! plus a static [`ports`](Block::ports)/[`phase`](Block::phase) declaration the assembler sorts on.
//!
//! Blocks run per lane: the caller binds the `SoA` views to a lane and passes the same `lane` for the
//! bus accessors. **Dispatch is static** (Decision #26 — never `dyn` in the hot path): each tier owns
//! a concrete block set with named fields (e.g. `outlap-transient`'s `T2Blocks` holds `Chassis`,
//! `Tire`, `Aero`, … and the T3 tier holds `ChassisT3` + suspension) and runs them in a fixed,
//! assembly-computed schedule order — there is no per-step enum match and no trait object. The
//! external plugin trait is deferred (Decision #38); the built-in controllers and physics blocks are
//! concrete types the tier structs compose directly.

use num_traits::Float;

use crate::bus::Bus;
use crate::state::{DerivView, SlowDerivView, SlowStateView, StateView};

/// The step phase a block runs in. The scheduler orders `sense → control → actuate → integrate`
/// (HANDOFF §6.2b); within a phase, data dependencies and registration order decide the sequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Phase {
    /// Read sensors / propagate the current state into observable bus signals.
    Sense = 0,
    /// Controllers (driver, torque vectoring, shift logic, regen blending).
    Control = 1,
    /// Turn control demands into physical forces/torques (tires, aero, powertrain, brakes).
    Actuate = 2,
    /// Assemble the RHS / advance the integrator.
    Integrate = 3,
}

/// A block's static port declaration: the bus channels it reads and writes. Used once by the
/// assembler to topologically sort the schedule (HANDOFF §6.2). Channels are flat bus indices
/// (fixed-core discriminants or interned [`ChannelId`](crate::bus::ChannelId) indices).
#[derive(Clone, Debug, Default)]
pub struct Ports {
    /// Bus channels this block reads.
    pub reads: Vec<usize>,
    /// Bus channels this block writes.
    pub writes: Vec<usize>,
}

impl Ports {
    /// A block that reads nothing and writes nothing.
    #[must_use]
    pub fn none() -> Self {
        Self::default()
    }

    /// Declare the reads and writes.
    #[must_use]
    pub fn new(reads: Vec<usize>, writes: Vec<usize>) -> Self {
        Self { reads, writes }
    }
}

/// A model block: pure, generic over the real type, with statically declared ports.
///
/// The three evaluations correspond to the tier being run; a block implements whichever apply and
/// leaves the rest as no-ops (the default methods). All are allocation-free and touch only the views
/// and the bus lane they are handed.
pub trait Block<T: Float> {
    /// The phase this block runs in.
    fn phase(&self) -> Phase;

    /// The block's static read/write ports (queried once at assembly).
    fn ports(&self) -> Ports;

    /// T0/T1: publish the algebraic equilibrium contribution at a trim point. Default: no-op.
    fn equilibrium(&self, bus: &mut Bus<T>, slow: &SlowStateView<T>, lane: usize) {
        let _ = (bus, slow, lane);
    }

    /// T2/T3: accumulate this block's fast-state RHS. Default: no-op.
    fn derivatives(&self, x: &StateView<T>, bus: &mut Bus<T>, dx: &mut DerivView<T>, lane: usize) {
        let _ = (x, bus, dx, lane);
    }

    /// Both tiers: accumulate slow-state derivatives on the decimated clock. Default: no-op.
    fn slow_derivatives(&self, bus: &Bus<T>, dslow: &mut SlowDerivView<T>, lane: usize) {
        let _ = (bus, dslow, lane);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phases_order_sense_before_integrate() {
        // The scheduler sorts on the phase ordinal; the integrate-phase RHS runs last.
        assert!(Phase::Sense < Phase::Control);
        assert!(Phase::Control < Phase::Actuate);
        assert!(Phase::Actuate < Phase::Integrate);
    }

    #[test]
    fn empty_ports_read_and_write_nothing() {
        let p = Ports::none();
        assert!(p.reads.is_empty() && p.writes.is_empty());
    }
}
