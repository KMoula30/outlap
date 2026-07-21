// SPDX-License-Identifier: AGPL-3.0-only
//! Shared drivetrain-graph helpers for the QSS assembly (§8.0, D-M6-13).
//!
//! Two operations the T0 and T1 reductions share:
//! * [`governed_unit_ids`] — the unit ids a `policy:` overlay governs (excluded from the mechanical
//!   traction/regen ceilings; driven instead by the energy-manager force-adder), and
//! * [`flatten_chain`] — a source's ordered coupler sequence + terminal wheels, formed from its
//!   private `path` followed by the shared graph couplers from its `output` node down to wheels.
//!
//! A `wheels:`-sugar source (no `output`, no top-level couplers) flattens to exactly its private
//! `(path, wheels)`, so its reduction is byte-identical to the pre-2.0 layout — the central Layer-1
//! invariant.

use std::collections::{BTreeSet, HashSet};

use outlap_schema::refs::UnitId;
use outlap_schema::vehicle::{Coupler, DriveUnit, Drivetrain, Wheel};
use outlap_schema::Vehicle;

/// The set of `drivetrain.units[]` ids a `policy:` overlay governs (empty when there is no policy).
pub(crate) fn governed_unit_ids(spec: &Vehicle) -> HashSet<&str> {
    spec.policy
        .iter()
        .flat_map(|p| p.governs.iter())
        .map(UnitId::as_str)
        .collect()
}

/// Flatten a source's drive chain: the ordered couplers from the source shaft to the wheels it
/// drives — its private `path` followed by the shared couplers from its `output` node down to
/// wheels, in graph order — plus those terminal wheels. Couplers are cloned (cold assembly path).
///
/// For a `wheels:`-sugar unit this returns `(unit.path.clone(), unit.wheels.clone())`, so
/// [`fold_path`](crate::vehicle) sees the exact same sequence it did before D-M6-13.
pub(crate) fn flatten_chain(dt: &Drivetrain, unit: &DriveUnit) -> (Vec<Coupler>, Vec<Wheel>) {
    let mut chain: Vec<Coupler> = unit.path.clone();
    let mut wheels: Vec<Wheel> = Vec::new();
    match &unit.output {
        None => wheels.extend_from_slice(&unit.wheels),
        Some(start) => {
            // Walk the shared graph forward from the output node, appending couplers (in file
            // declaration order per node) and collecting terminal wheels. The visited guard keeps
            // it finite; the topology stage has already rejected cycles.
            let mut visited: BTreeSet<&str> = BTreeSet::new();
            let mut queue: Vec<&str> = vec![start.as_str()];
            while let Some(node) = queue.pop() {
                if !visited.insert(node) {
                    continue;
                }
                for edge in dt.couplers.iter().filter(|e| e.from.as_str() == node) {
                    chain.push(edge.coupler.clone());
                    wheels.extend_from_slice(&edge.wheels);
                    if let Some(to) = &edge.to {
                        queue.push(to.as_str());
                    }
                }
            }
        }
    }
    (chain, wheels)
}
