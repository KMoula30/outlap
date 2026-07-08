// SPDX-License-Identifier: AGPL-3.0-only
//! The **assembler**: turns a set of block port declarations into a deterministic execution
//! [`Schedule`] (HANDOFF §6.2, §6.2b).
//!
//! Blocks declare the bus channels they read and write ([`Ports`]) and the phase they run in
//! ([`Phase`]). The assembler runs **once at load** (allocation is fine here — never in the loop):
//! it fixes the global phase order `sense → control → actuate → integrate`, then within each phase
//! topologically sorts blocks so every intra-phase writer precedes its readers. A cyclic intra-phase
//! data dependency is a hard error ([`AssemblyError::CyclicDependency`]) — a genuine algebraic loop
//! that must be broken by moving a block to a later phase or by the one-step-lag `fz_coupling` path.
//!
//! Cross-phase dependencies pointing *backwards* (a `sense`-phase reader of an `integrate`-phase
//! writer) are **one-step-lag** by design: they use the previous step's value and impose no ordering
//! constraint. Ties are broken by registration index, so the schedule is bit-deterministic.

use thiserror::Error;

use crate::block::{Phase, Ports};

/// One block's contribution to the assembly: its phase and its port declaration.
#[derive(Clone, Debug)]
pub struct BlockSpec {
    /// The phase the block runs in.
    pub phase: Phase,
    /// The bus channels the block reads and writes.
    pub ports: Ports,
}

impl BlockSpec {
    /// A spec from a phase and ports.
    #[must_use]
    pub fn new(phase: Phase, ports: Ports) -> Self {
        Self { phase, ports }
    }
}

/// Errors raised while assembling a schedule.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum AssemblyError {
    /// A set of blocks in one phase form a write→read cycle that cannot be linearised.
    #[error(
        "cyclic intra-phase data dependency among blocks {blocks:?} in phase {phase:?}; \
         break it with a phase change or the one-step-lag fz_coupling path"
    )]
    CyclicDependency {
        /// The block indices left unscheduled by the cycle.
        blocks: Vec<usize>,
        /// The phase the cycle occurs in.
        phase: Phase,
    },
}

/// A frozen, deterministic execution order: the block indices to run, in order, with the phase
/// boundaries recorded. Produced by [`assemble`]; consumed by the stepper (no config logic remains).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Schedule {
    order: Vec<usize>,
}

impl Schedule {
    /// The block indices in execution order.
    #[must_use]
    pub fn order(&self) -> &[usize] {
        &self.order
    }

    /// The number of scheduled blocks.
    #[must_use]
    pub fn len(&self) -> usize {
        self.order.len()
    }

    /// Whether the schedule is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }
}

/// The four phases in execution order.
const PHASES: [Phase; 4] = [
    Phase::Sense,
    Phase::Control,
    Phase::Actuate,
    Phase::Integrate,
];

/// Assemble `specs` into a deterministic [`Schedule`].
///
/// Blocks are grouped by phase (fixed order), then each phase is topologically sorted on its
/// intra-phase write→read edges via Kahn's algorithm with a registration-index tie-break.
///
/// # Errors
/// [`AssemblyError::CyclicDependency`] if any phase contains an unbreakable data cycle.
pub fn assemble(specs: &[BlockSpec]) -> Result<Schedule, AssemblyError> {
    let mut order = Vec::with_capacity(specs.len());
    for &phase in &PHASES {
        let members: Vec<usize> = (0..specs.len())
            .filter(|&i| specs[i].phase == phase)
            .collect();
        toposort_phase(specs, &members, phase, &mut order)?;
    }
    Ok(Schedule { order })
}

/// Topologically sort one phase's `members`, appending the result to `order`.
fn toposort_phase(
    specs: &[BlockSpec],
    members: &[usize],
    phase: Phase,
    order: &mut Vec<usize>,
) -> Result<(), AssemblyError> {
    let n = members.len();
    // Edge u -> v (both indices into `members`) when block u writes a channel that block v reads.
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut indeg: Vec<usize> = vec![0; n];
    for (u, &bu) in members.iter().enumerate() {
        for (v, &bv) in members.iter().enumerate() {
            if u == v {
                continue;
            }
            let writes_a_read = specs[bu]
                .ports
                .writes
                .iter()
                .any(|w| specs[bv].ports.reads.contains(w));
            if writes_a_read {
                adj[u].push(v);
                indeg[v] += 1;
            }
        }
    }

    // Kahn's algorithm; among ready nodes always take the lowest registration index (determinism).
    let mut ready: Vec<usize> = (0..n).filter(|&i| indeg[i] == 0).collect();
    let mut emitted = 0usize;
    while !ready.is_empty() {
        // Pick the ready node with the smallest original block index.
        let pick_pos = ready
            .iter()
            .enumerate()
            .min_by_key(|(_, &r)| members[r])
            .map(|(pos, _)| pos)
            .expect("ready is non-empty");
        let u = ready.swap_remove(pick_pos);
        order.push(members[u]);
        emitted += 1;
        for &v in &adj[u] {
            indeg[v] -= 1;
            if indeg[v] == 0 {
                ready.push(v);
            }
        }
    }

    if emitted != n {
        let blocks: Vec<usize> = (0..n)
            .filter(|&i| indeg[i] > 0)
            .map(|i| members[i])
            .collect();
        return Err(AssemblyError::CyclicDependency { blocks, phase });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::Phase;

    fn spec(phase: Phase, reads: &[usize], writes: &[usize]) -> BlockSpec {
        BlockSpec::new(phase, Ports::new(reads.to_vec(), writes.to_vec()))
    }

    #[test]
    fn writer_precedes_reader_within_a_phase() {
        // block0 reads chan 7 (written by block1); block1 writes 7. Expect 1 before 0.
        let specs = [
            spec(Phase::Actuate, &[7], &[]),
            spec(Phase::Actuate, &[], &[7]),
        ];
        let sched = assemble(&specs).unwrap();
        assert_eq!(sched.order(), &[1, 0]);
    }

    #[test]
    fn phases_run_in_fixed_order_regardless_of_registration() {
        // Registered integrate-first, but sense must come out first.
        let specs = [
            spec(Phase::Integrate, &[], &[]),
            spec(Phase::Sense, &[], &[]),
            spec(Phase::Control, &[], &[]),
            spec(Phase::Actuate, &[], &[]),
        ];
        let sched = assemble(&specs).unwrap();
        assert_eq!(sched.order(), &[1, 2, 3, 0]);
    }

    #[test]
    fn independent_blocks_break_ties_by_registration_index() {
        let specs = [
            spec(Phase::Sense, &[], &[]),
            spec(Phase::Sense, &[], &[]),
            spec(Phase::Sense, &[], &[]),
        ];
        // Deterministic: ascending registration index.
        assert_eq!(assemble(&specs).unwrap().order(), &[0, 1, 2]);
        // Reproducible across calls.
        assert_eq!(assemble(&specs).unwrap(), assemble(&specs).unwrap());
    }

    #[test]
    fn intra_phase_cycle_is_rejected() {
        // 0 writes A/reads B, 1 writes B/reads A — a write→read cycle in one phase.
        let specs = [
            spec(Phase::Control, &[2], &[1]),
            spec(Phase::Control, &[1], &[2]),
        ];
        match assemble(&specs) {
            Err(AssemblyError::CyclicDependency { phase, blocks }) => {
                assert_eq!(phase, Phase::Control);
                assert_eq!(blocks.len(), 2);
            }
            other => panic!("expected a cycle error, got {other:?}"),
        }
    }

    #[test]
    fn backward_cross_phase_dependency_is_one_step_lag_not_a_cycle() {
        // A sense-phase reader of an integrate-phase writer uses the previous step's value — legal.
        let specs = [
            spec(Phase::Sense, &[9], &[]),
            spec(Phase::Integrate, &[], &[9]),
        ];
        let sched = assemble(&specs).unwrap();
        assert_eq!(sched.order(), &[0, 1]); // sense before integrate, no cycle
    }
}
