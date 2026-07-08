// SPDX-License-Identifier: AGPL-3.0-only
//! The **state registry** and the `SoA` state views (HANDOFF §6.1, §6.2).
//!
//! outlap splits the model state into a **fast** buffer (chassis velocities, wheel speeds, tire
//! relaxation) advanced every step by the split integrator, and a **slow** buffer (temperatures,
//! wear, SOC, fuel) advanced on a decimated clock. Both are struct-of-arrays with an explicit batch
//! dimension: slot `i`, lane `b` lives at `i * batch + b`.
//!
//! # Frozen fast-state layout
//!
//! The fast buffer is `[chassis | relaxation]`. The chassis region reserves the full **14-DOF**
//! ([`ChassisState`]) footprint so the T3 groundwork is laid without a layout break: T2 integrates
//! only the first ten slots ([`ChassisState::T2_DOF`]); the heave/pitch/roll + four-unsprung slots
//! sit reserved and read as zero until T3. The relaxation region holds a lagged `κ` and `α` per
//! wheel ([`WHEELS`]). This layout is a frozen contract (see the PR layout note); downstream code
//! addresses states by the [`ChassisState`] / [`RelaxState`] enums, never by bare integers.

use num_traits::Float;

use crate::bus::WHEELS;

/// Frozen chassis fast-state slots in the curvilinear 3-D road frame (HANDOFF §6.1). The first ten
/// ([`ChassisState::T2_DOF`]) are the T2 7-DOF set `[s, n, ψ_rel, vx, vy, r, ω₁..₄]`; the remainder
/// are **reserved for T3** (sprung heave/pitch/roll + rates, four unsprung verticals + rates) and
/// are not integrated in M4.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum ChassisState {
    /// Distance along the track centre-line `s` (m).
    S = 0,
    /// Lateral offset from the reference line `n` (m, +left).
    N,
    /// Heading relative to the road tangent `ψ_rel` (rad).
    PsiRel,
    /// Body-frame longitudinal velocity `v_x` (m/s).
    Vx,
    /// Body-frame lateral velocity `v_y` (m/s, +left).
    Vy,
    /// Yaw rate `r` (rad/s, +CCW).
    YawRate,
    /// Front-left wheel speed `ω` (rad/s).
    OmegaFl,
    /// Front-right wheel speed `ω` (rad/s).
    OmegaFr,
    /// Rear-left wheel speed `ω` (rad/s).
    OmegaRl,
    /// Rear-right wheel speed `ω` (rad/s).
    OmegaRr,
    // --- T3-reserved (not integrated in M4) -------------------------------------------------
    /// Sprung-mass heave `z` (m). Reserved for T3.
    Heave,
    /// Sprung-mass pitch `θ` (rad). Reserved for T3.
    Pitch,
    /// Sprung-mass roll `φ` (rad). Reserved for T3.
    Roll,
    /// Heave rate `ż` (m/s). Reserved for T3.
    HeaveRate,
    /// Pitch rate `θ̇` (rad/s). Reserved for T3.
    PitchRate,
    /// Roll rate `φ̇` (rad/s). Reserved for T3.
    RollRate,
    /// Front-left unsprung vertical position (m). Reserved for T3.
    ZuFl,
    /// Front-right unsprung vertical position (m). Reserved for T3.
    ZuFr,
    /// Rear-left unsprung vertical position (m). Reserved for T3.
    ZuRl,
    /// Rear-right unsprung vertical position (m). Reserved for T3.
    ZuRr,
    /// Front-left unsprung vertical velocity (m/s). Reserved for T3.
    ZuRateFl,
    /// Front-right unsprung vertical velocity (m/s). Reserved for T3.
    ZuRateFr,
    /// Rear-left unsprung vertical velocity (m/s). Reserved for T3.
    ZuRateRl,
    /// Rear-right unsprung vertical velocity (m/s). Reserved for T3.
    ZuRateRr,
    /// Number of chassis slots (full 14-DOF footprint). Keep last.
    COUNT,
}

impl ChassisState {
    /// The T2 degrees of freedom actually integrated in M4 — the first ten slots
    /// `[s, n, ψ_rel, vx, vy, r, ω₁..₄]`.
    pub const T2_DOF: usize = 10;
}

/// Per-wheel tire relaxation (lagged-slip) states, two per wheel (HANDOFF §11.2). PR4 populates
/// these via the exact-exponential channel; the region is reserved here so the layout is frozen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum RelaxState {
    /// Lagged longitudinal slip `κ`.
    Kappa = 0,
    /// Lagged slip angle `α` (rad).
    Alpha,
    /// Number of relaxation states per wheel. Keep last.
    COUNT,
}

/// Offset (in slots) where the relaxation region begins.
const RELAX_BASE: usize = ChassisState::COUNT as usize;
/// Total fast-state slots: chassis (14-DOF footprint) + relaxation (2 × [`WHEELS`]).
const FAST_SLOTS: usize = RELAX_BASE + (RelaxState::COUNT as usize) * WHEELS;

/// The number of fast-state slots reserved per lane (chassis + relaxation).
#[must_use]
pub const fn fast_slot_count() -> usize {
    FAST_SLOTS
}

/// Immutable description of a model's state buffers, computed once at assembly. Records the fast
/// footprint (frozen) and the dynamic slow-state count, plus how many DOF the active tier integrates.
#[derive(Clone, Debug)]
pub struct StateLayout {
    fast_slots: usize,
    slow_slots: usize,
    integrated_dof: usize,
}

impl StateLayout {
    /// Build a layout for `slow_slots` slow states, integrating `integrated_dof` chassis DOF
    /// (`ChassisState::T2_DOF` for T2). The fast footprint is the frozen [`fast_slot_count`].
    ///
    /// # Panics
    /// Panics if `integrated_dof` exceeds the reserved chassis footprint.
    #[must_use]
    pub fn new(slow_slots: usize, integrated_dof: usize) -> Self {
        assert!(
            integrated_dof <= ChassisState::COUNT as usize,
            "integrated DOF exceeds the reserved chassis footprint"
        );
        Self {
            fast_slots: FAST_SLOTS,
            slow_slots,
            integrated_dof,
        }
    }

    /// Fast-state slots per lane.
    #[must_use]
    pub fn fast_slots(&self) -> usize {
        self.fast_slots
    }

    /// Slow-state slots per lane.
    #[must_use]
    pub fn slow_slots(&self) -> usize {
        self.slow_slots
    }

    /// Chassis DOF the active tier integrates (the rest of the fast buffer stays reserved/lagged).
    #[must_use]
    pub fn integrated_dof(&self) -> usize {
        self.integrated_dof
    }

    /// Flat index of the relaxation state `which` for `wheel` in the fast buffer.
    #[must_use]
    pub fn relax_slot(which: RelaxState, wheel: usize) -> usize {
        RELAX_BASE + (which as usize) * WHEELS + wheel
    }
}

/// Read-only view of one lane's **fast** state over an `SoA` buffer. Access is allocation-free.
#[derive(Clone, Copy, Debug)]
pub struct StateView<'a, T> {
    data: &'a [T],
    batch: usize,
    lane: usize,
}

impl<'a, T: Float> StateView<'a, T> {
    /// Bind a view to `lane` of a fast `SoA` buffer (`data.len() == slots * batch`).
    #[must_use]
    pub fn new(data: &'a [T], batch: usize, lane: usize) -> Self {
        Self { data, batch, lane }
    }

    /// Read a chassis state.
    #[inline]
    #[must_use]
    pub fn chassis(&self, s: ChassisState) -> T {
        self.data[(s as usize) * self.batch + self.lane]
    }

    /// Read a per-wheel relaxation state.
    #[inline]
    #[must_use]
    pub fn relax(&self, which: RelaxState, wheel: usize) -> T {
        self.data[StateLayout::relax_slot(which, wheel) * self.batch + self.lane]
    }

    /// Read a raw fast slot by flat index (escape hatch for generic code).
    #[inline]
    #[must_use]
    pub fn slot(&self, slot: usize) -> T {
        self.data[slot * self.batch + self.lane]
    }
}

/// Write view of one lane's **fast** derivative buffer (`dx/dt`).
#[derive(Debug)]
pub struct DerivView<'a, T> {
    data: &'a mut [T],
    batch: usize,
    lane: usize,
}

impl<'a, T: Float> DerivView<'a, T> {
    /// Bind a derivative view to `lane` of a fast `SoA` buffer.
    #[must_use]
    pub fn new(data: &'a mut [T], batch: usize, lane: usize) -> Self {
        Self { data, batch, lane }
    }

    /// Set a chassis-state derivative.
    #[inline]
    pub fn set_chassis(&mut self, s: ChassisState, value: T) {
        let i = (s as usize) * self.batch + self.lane;
        self.data[i] = value;
    }

    /// Set a raw fast-slot derivative by flat index.
    #[inline]
    pub fn set_slot(&mut self, slot: usize, value: T) {
        let i = slot * self.batch + self.lane;
        self.data[i] = value;
    }
}

/// Read-only view of one lane's **slow** state.
#[derive(Clone, Copy, Debug)]
pub struct SlowStateView<'a, T> {
    data: &'a [T],
    batch: usize,
    lane: usize,
}

impl<'a, T: Float> SlowStateView<'a, T> {
    /// Bind a slow-state view to `lane`.
    #[must_use]
    pub fn new(data: &'a [T], batch: usize, lane: usize) -> Self {
        Self { data, batch, lane }
    }

    /// Read slow slot `slot`.
    #[inline]
    #[must_use]
    pub fn get(&self, slot: usize) -> T {
        self.data[slot * self.batch + self.lane]
    }
}

/// Write view of one lane's **slow** derivative buffer (`dslow/dt`).
#[derive(Debug)]
pub struct SlowDerivView<'a, T> {
    data: &'a mut [T],
    batch: usize,
    lane: usize,
}

impl<'a, T: Float> SlowDerivView<'a, T> {
    /// Bind a slow-derivative view to `lane`.
    #[must_use]
    pub fn new(data: &'a mut [T], batch: usize, lane: usize) -> Self {
        Self { data, batch, lane }
    }

    /// Set slow-slot derivative `slot`.
    #[inline]
    pub fn set(&mut self, slot: usize, value: T) {
        let i = slot * self.batch + self.lane;
        self.data[i] = value;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn fast_footprint_is_chassis_plus_relaxation() {
        assert_eq!(
            fast_slot_count(),
            ChassisState::COUNT as usize + (RelaxState::COUNT as usize) * WHEELS
        );
        // The T2 tier integrates only the first ten chassis slots; the rest are reserved for T3.
        assert_eq!(ChassisState::T2_DOF, 10);
        assert!(ChassisState::T2_DOF < ChassisState::COUNT as usize);
    }

    #[test]
    fn relax_slots_are_contiguous_past_the_chassis_region() {
        let base = ChassisState::COUNT as usize;
        for wheel in 0..WHEELS {
            assert_eq!(
                StateLayout::relax_slot(RelaxState::Kappa, wheel),
                base + wheel
            );
            assert_eq!(
                StateLayout::relax_slot(RelaxState::Alpha, wheel),
                base + WHEELS + wheel
            );
        }
    }

    #[test]
    fn views_read_and_write_a_lane_and_reserved_slots_read_zero() {
        let batch = 2;
        let mut fast = vec![0.0f64; fast_slot_count() * batch];
        {
            let mut dx = DerivView::new(&mut fast, batch, 1);
            dx.set_chassis(ChassisState::Vx, 42.0);
            dx.set_slot(StateLayout::relax_slot(RelaxState::Kappa, 3), 0.1);
        }
        let x = StateView::new(&fast, batch, 1);
        assert_eq!(x.chassis(ChassisState::Vx), 42.0);
        assert_eq!(x.relax(RelaxState::Kappa, 3), 0.1);
        // T3-reserved slot reads zero; lane 0 untouched.
        assert_eq!(x.chassis(ChassisState::Heave), 0.0);
        assert_eq!(
            StateView::new(&fast, batch, 0).chassis(ChassisState::Vx),
            0.0
        );
    }

    #[test]
    fn layout_records_slow_and_integrated_dof() {
        let layout = StateLayout::new(5, ChassisState::T2_DOF);
        assert_eq!(layout.fast_slots(), fast_slot_count());
        assert_eq!(layout.slow_slots(), 5);
        assert_eq!(layout.integrated_dof(), ChassisState::T2_DOF);
    }

    #[test]
    fn views_are_generic_over_f32() {
        let mut fast = vec![0.0f32; fast_slot_count()];
        DerivView::new(&mut fast, 1, 0).set_chassis(ChassisState::YawRate, 1.5);
        assert_eq!(
            StateView::new(&fast, 1, 0).chassis(ChassisState::YawRate),
            1.5
        );
    }

    #[test]
    #[should_panic(expected = "reserved chassis footprint")]
    fn layout_rejects_dof_beyond_the_footprint() {
        let _ = StateLayout::new(0, ChassisState::COUNT as usize + 1);
    }
}
