// SPDX-License-Identifier: AGPL-3.0-only
//! The per-lap energy ledger — deploy/harvest integrals on the ELECTRICAL side.
//!
//! The FIA 2026 budgets are written at the CU-K DC bus (C5.2.10: the Recharge budget counts
//! *electrical* energy into the ES; a deployment budget, where a rule set has one, is the same
//! side), so the ledger integrates the electrical command powers — never the mechanical axle
//! powers. The caller owns the clock and the lap boundaries: [`record`](LapEnergyLedger::record)
//! each step, [`reset`](LapEnergyLedger::reset) at the start line (a lap boundary resets the
//! LEDGER, never the pack — the store carries over).

use num_traits::Float;

use crate::manager::ErsCommand;

/// Per-lap deploy/harvest energy integrals, J electrical.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LapEnergyLedger<T> {
    /// Electrical energy deployed this lap, J (≥ 0).
    deploy_j: T,
    /// Electrical energy harvested this lap, J (≥ 0). ALL harvest paths — braking,
    /// part-throttle, ICE-driven back-drive — count against the same integral (C5.2.10).
    harvest_j: T,
}

impl<T: Float> LapEnergyLedger<T> {
    /// A fresh ledger (both integrals zero).
    pub fn new() -> Self {
        Self {
            deploy_j: T::zero(),
            harvest_j: T::zero(),
        }
    }

    /// Integrate one step's command: `deploy_j += deploy_w·dt`, `harvest_j += harvest_w·dt`.
    /// Exact accumulation of what was commanded — the closure property `Σ cmd·dt == ledger`
    /// holds bit-for-bit because this is the only writer.
    pub fn record(&mut self, cmd: &ErsCommand<T>, dt: T) {
        self.deploy_j = self.deploy_j + cmd.deploy_w * dt;
        self.harvest_j = self.harvest_j + cmd.harvest_w * dt;
    }

    /// Reset both integrals at a lap boundary (per-lap budgets; the pack state is NOT reset).
    pub fn reset(&mut self) {
        self.deploy_j = T::zero();
        self.harvest_j = T::zero();
    }

    /// Electrical energy deployed this lap, J.
    pub fn deploy_j(&self) -> T {
        self.deploy_j
    }

    /// Electrical energy harvested this lap, J.
    pub fn harvest_j(&self) -> T {
        self.harvest_j
    }
}
