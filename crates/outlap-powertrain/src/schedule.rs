// SPDX-License-Identifier: AGPL-3.0-only
//! The `u(s)` control-vector schedule — §8.3's per-station strategy input (D-M6-9).
//!
//! `u(s) = [deploy/regen ∈ [−1,1], override_flag, lift_point, shift_map_id]`, accepted as a
//! data-driven schedule over the track's station grid. It is an API input, not a vehicle-schema
//! document (control input ≠ car identity; stage 2 formalizes the file format). The energy
//! manager executes the deploy/regen fraction and the override flag; the lift point and shift-map
//! id are carried per station for the driver and gearbox wiring (M6 PR4).

use num_traits::Float;

/// Error building a [`UsSchedule`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ScheduleError {
    /// The component arrays have different lengths.
    #[error(
        "u(s) component arrays must be equal length: deploy_regen {deploy_regen}, \
         override_flag {override_flag}, lift_point {lift_point}, shift_map_id {shift_map_id}"
    )]
    LengthMismatch {
        /// Length of `deploy_regen`.
        deploy_regen: usize,
        /// Length of `override_flag`.
        override_flag: usize,
        /// Length of `lift_point`.
        lift_point: usize,
        /// Length of `shift_map_id`.
        shift_map_id: usize,
    },
    /// The schedule is empty (no stations).
    #[error("u(s) schedule must have at least one station")]
    Empty,
    /// A deploy/regen fraction lies outside `[−1, 1]`.
    #[error("deploy_regen[{index}] must lie in [-1, 1]")]
    FracOutOfRange {
        /// The offending station index.
        index: usize,
    },
}

/// A per-station `u(s)` control schedule.
#[derive(Clone, Debug)]
pub struct UsSchedule<T> {
    /// Deploy (+) / regen (−) fraction per station, in `[−1, 1]`.
    deploy_regen: Vec<T>,
    /// Override ("Overtake") flag per station.
    override_flag: Vec<bool>,
    /// Lift-and-coast point per station (interpretation is the driver hook's, PR4).
    lift_point: Vec<T>,
    /// Named-shift-map selector per station (PR4 wires the FSM selection).
    shift_map_id: Vec<u32>,
}

impl<T: Float> UsSchedule<T> {
    /// Build a schedule from its four component arrays (equal length, ≥ 1 station,
    /// `deploy_regen ∈ [−1, 1]`).
    ///
    /// # Errors
    /// [`ScheduleError`] on length mismatch, empty arrays, or an out-of-range fraction.
    pub fn new(
        deploy_regen: Vec<T>,
        override_flag: Vec<bool>,
        lift_point: Vec<T>,
        shift_map_id: Vec<u32>,
    ) -> Result<Self, ScheduleError> {
        let n = deploy_regen.len();
        if override_flag.len() != n || lift_point.len() != n || shift_map_id.len() != n {
            return Err(ScheduleError::LengthMismatch {
                deploy_regen: n,
                override_flag: override_flag.len(),
                lift_point: lift_point.len(),
                shift_map_id: shift_map_id.len(),
            });
        }
        if n == 0 {
            return Err(ScheduleError::Empty);
        }
        for (i, &u) in deploy_regen.iter().enumerate() {
            if !(u >= -T::one() && u <= T::one()) {
                return Err(ScheduleError::FracOutOfRange { index: i });
            }
        }
        Ok(Self {
            deploy_regen,
            override_flag,
            lift_point,
            shift_map_id,
        })
    }

    /// Number of stations.
    pub fn len(&self) -> usize {
        self.deploy_regen.len()
    }

    /// Whether the schedule is empty (construction forbids it; kept for the `len` pairing).
    pub fn is_empty(&self) -> bool {
        self.deploy_regen.is_empty()
    }

    /// The deploy (+) / regen (−) fraction at `station` (clamped to the last station past the
    /// end — mirrors the table edge-clamp convention; a mis-sized schedule is a construction
    /// error, not a query error).
    pub fn deploy_regen(&self, station: usize) -> T {
        self.deploy_regen[station.min(self.deploy_regen.len() - 1)]
    }

    /// The override flag at `station` (edge-clamped like [`deploy_regen`](Self::deploy_regen)).
    pub fn override_flag(&self, station: usize) -> bool {
        self.override_flag[station.min(self.override_flag.len() - 1)]
    }

    /// The lift point at `station` (edge-clamped; consumed by the PR4 driver hook).
    pub fn lift_point(&self, station: usize) -> T {
        self.lift_point[station.min(self.lift_point.len() - 1)]
    }

    /// The shift-map id at `station` (edge-clamped; consumed by the PR4 gearbox FSM).
    pub fn shift_map_id(&self, station: usize) -> u32 {
        self.shift_map_id[station.min(self.shift_map_id.len() - 1)]
    }
}
