// SPDX-License-Identifier: AGPL-3.0-only
//! The [`Block`] abstraction and the [`CoreBlock`] dispatch enum (HANDOFF §6.2).
//!
//! A block is `(immutable parameters, states, typed ports on the [`Bus`](crate::bus::Bus))`. It
//! exposes three pure, `f32`/`f64`-generic evaluations — an algebraic [`equilibrium`](Block::equilibrium)
//! (T0/T1 trim), an ODE [`derivatives`](Block::derivatives) (T2/T3 fast RHS), and a
//! [`slow_derivatives`](Block::slow_derivatives) (thermal/wear/SOC on the decimated slow clock) —
//! plus a static [`ports`](Block::ports)/[`phase`](Block::phase) declaration the assembler sorts on.
//!
//! Blocks run per lane: the caller binds the `SoA` views to a lane and passes the same `lane` for the
//! bus accessors. In the hot loop, dispatch is via the [`CoreBlock`] enum — **never `dyn`**
//! (Decision #26). The external plugin trait is deferred (Decision #38); the built-in controllers
//! and physics blocks are core enum variants.

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

/// The **stubbed suspension interface** (T3 groundwork, Decision #3). In M4 it is a no-op that owns
/// no state: it reserves the block slot and the port surface so the T3 lumped-K&C model drops in
/// without a scaffolding break. It declares no ports and contributes nothing to any evaluation.
#[derive(Clone, Copy, Debug, Default)]
pub struct SuspensionStub;

impl<T: Float> Block<T> for SuspensionStub {
    fn phase(&self) -> Phase {
        Phase::Actuate
    }

    fn ports(&self) -> Ports {
        Ports::none()
    }
}

/// The hot-loop block dispatch (enum, not `dyn`; Decision #26). Physics and controller blocks are
/// added as variants in later PRs (Chassis, Tire×4, Aero, Driver, …); M4 ships the suspension stub.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum CoreBlock {
    /// The T3-groundwork suspension stub.
    Suspension(SuspensionStub),
}

impl CoreBlock {
    /// The phase this block runs in.
    #[must_use]
    pub fn phase(&self) -> Phase {
        match self {
            CoreBlock::Suspension(b) => Block::<f64>::phase(b),
        }
    }

    /// The block's static ports.
    #[must_use]
    pub fn ports(&self) -> Ports {
        match self {
            CoreBlock::Suspension(b) => Block::<f64>::ports(b),
        }
    }

    /// Dispatch [`Block::derivatives`] to the concrete variant.
    pub fn derivatives<T: Float>(
        &self,
        x: &StateView<T>,
        bus: &mut Bus<T>,
        dx: &mut DerivView<T>,
        lane: usize,
    ) {
        match self {
            CoreBlock::Suspension(b) => b.derivatives(x, bus, dx, lane),
        }
    }

    /// Dispatch [`Block::equilibrium`] to the concrete variant.
    pub fn equilibrium<T: Float>(&self, bus: &mut Bus<T>, slow: &SlowStateView<T>, lane: usize) {
        match self {
            CoreBlock::Suspension(b) => b.equilibrium(bus, slow, lane),
        }
    }

    /// Dispatch [`Block::slow_derivatives`] to the concrete variant.
    pub fn slow_derivatives<T: Float>(
        &self,
        bus: &Bus<T>,
        dslow: &mut SlowDerivView<T>,
        lane: usize,
    ) {
        match self {
            CoreBlock::Suspension(b) => b.slow_derivatives(bus, dslow, lane),
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;
    use crate::bus::{core_channel_count, Bus};
    use crate::state::{fast_slot_count, DerivView, StateView};

    #[test]
    fn suspension_stub_is_an_inert_actuate_block() {
        let stub = SuspensionStub;
        assert_eq!(Block::<f64>::phase(&stub), Phase::Actuate);
        let ports = Block::<f64>::ports(&stub);
        assert!(ports.reads.is_empty() && ports.writes.is_empty());
    }

    #[test]
    fn core_block_dispatch_is_a_no_op_for_the_stub() {
        // Wrapping in the dispatch enum and running derivatives leaves the derivative buffer at zero.
        let block = CoreBlock::Suspension(SuspensionStub);
        assert_eq!(block.phase(), Phase::Actuate);
        let fast = vec![0.0f64; fast_slot_count()];
        let mut dfast = vec![7.0f64; fast_slot_count()]; // pre-filled sentinel
        let mut bus = Bus::<f64>::new(core_channel_count(), 1);
        let x = StateView::new(&fast, 1, 0);
        let mut dx = DerivView::new(&mut dfast, 1, 0);
        block.derivatives(&x, &mut bus, &mut dx, 0);
        // The stub wrote nothing; the sentinel is untouched.
        assert!(dfast.iter().all(|&v| v == 7.0));
    }

    #[test]
    fn core_block_dispatch_is_generic_over_f32() {
        let block = CoreBlock::Suspension(SuspensionStub);
        let bus = Bus::<f32>::new(core_channel_count(), 1);
        let mut dslow = [0.0f32; 2];
        let mut d = crate::state::SlowDerivView::new(&mut dslow, 1, 0);
        block.slow_derivatives(&bus, &mut d, 0);
        assert_eq!(dslow, [0.0, 0.0]);
    }
}
