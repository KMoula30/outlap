// SPDX-License-Identifier: AGPL-3.0-only
//! The signal [`Bus`] — the flat struct-of-arrays board that blocks read and write each step
//! (HANDOFF §6.2, Decision #39).
//!
//! A block never talks to another block directly: it publishes to and consumes from a shared bus of
//! typed scalar channels. The bus has two regions:
//!
//! * a **fixed core set** with compile-time indices ([`CoreSignal`] scalars + [`WheelSignal`]
//!   per-wheel groups) — the signals every built-in T2 block exchanges (forces, slips, controls);
//! * an **interned dynamic region** for plugin/custom named channels ([`ChannelInterner`]). Interning
//!   happens once at assembly; the hot loop touches only integer indices — never a string or a hash.
//!
//! Every channel carries an explicit **batch dimension** (`SoA`, state-major): channel `c`, lane `b`
//! lives at `c * batch + b`, so one channel is contiguous across the batch (GPU-transposable,
//! HANDOFF §11.3). Access is allocation-free (CI-gated); construction may allocate.

use num_traits::Float;

/// Number of tire/wheel corners (ISO 8855 order: FL, FR, RL, RR).
pub const WHEELS: usize = 4;

/// A fixed **scalar** bus channel with a compile-time index. Discriminants are the channel offsets
/// into the scalar region; [`CoreSignal::COUNT`] is the region width and MUST stay last.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum CoreSignal {
    /// Front-axle road-wheel steer angle `δ` (rad, ISO 8855: +left).
    Steer = 0,
    /// Normalised throttle demand, `0..=1`.
    Throttle,
    /// Normalised brake demand, `0..=1`.
    Brake,
    /// Total drive torque delivered to the driven shaft (N·m).
    DriveTorque,
    /// Yaw-moment demand from torque vectoring, `ΔM_z` (N·m, +CCW).
    YawMomentDemand,
    /// Aerodynamic drag force on the platform (N, opposes `+x`).
    AeroDrag,
    /// Aerodynamic vertical load on the front axle (N, +down).
    AeroFzFront,
    /// Aerodynamic vertical load on the rear axle (N, +down).
    AeroFzRear,
    /// Number of scalar channels — region width. Keep last.
    COUNT,
}

/// A fixed **per-wheel** bus channel group; each group spans [`WHEELS`] consecutive slots.
/// [`WheelSignal::COUNT`] is the number of groups and MUST stay last.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum WheelSignal {
    /// Longitudinal tire force in the wheel frame (N).
    TireFx = 0,
    /// Lateral tire force in the wheel frame (N).
    TireFy,
    /// Vertical (normal) tire load `F_z` (N, +down).
    TireFz,
    /// Aligning moment `M_z` (N·m).
    TireMz,
    /// Lagged (relaxation) longitudinal slip `κ` fed into the force model.
    SlipKappa,
    /// Lagged (relaxation) slip angle `α` (rad) fed into the force model.
    SlipAlpha,
    /// Steady-state longitudinal-slip target `κ_ss` (relaxation input).
    SlipKappaSs,
    /// Steady-state slip-angle target `α_ss` (rad, relaxation input).
    SlipAlphaSs,
    /// Drive torque applied at this wheel (N·m).
    WheelDriveTorque,
    /// Brake torque applied at this wheel (N·m, ≥ 0).
    WheelBrakeTorque,
    /// Number of per-wheel groups. Keep last.
    COUNT,
}

/// Byte offset (in channels) where the per-wheel region starts.
const WHEEL_BASE: usize = CoreSignal::COUNT as usize;
/// Number of fixed core channels (scalar + per-wheel).
const CORE_CHANNELS: usize = WHEEL_BASE + (WheelSignal::COUNT as usize) * WHEELS;

/// The number of fixed core channels reserved on every bus (scalar + per-wheel).
#[must_use]
pub const fn core_channel_count() -> usize {
    CORE_CHANNELS
}

/// Interns dynamic named channels to integer indices **at assembly time** (Decision #39). Names are
/// resolved once here; the hot loop only ever sees the returned [`ChannelId`].
#[derive(Clone, Debug, Default)]
pub struct ChannelInterner {
    names: Vec<String>,
}

/// A resolved dynamic-channel handle (an index past the fixed core region).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ChannelId(usize);

impl ChannelId {
    /// The flat bus index this channel occupies (past the core region).
    #[must_use]
    pub fn index(self) -> usize {
        self.0
    }
}

impl ChannelInterner {
    /// A fresh interner with no dynamic channels.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Intern `name`, returning a stable [`ChannelId`]. Idempotent: the same name always maps to the
    /// same id, so blocks that share a channel by name land on the same slot.
    pub fn intern(&mut self, name: &str) -> ChannelId {
        if let Some(pos) = self.names.iter().position(|n| n == name) {
            return ChannelId(CORE_CHANNELS + pos);
        }
        self.names.push(name.to_owned());
        ChannelId(CORE_CHANNELS + self.names.len() - 1)
    }

    /// Look up an already-interned channel without inserting.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<ChannelId> {
        self.names
            .iter()
            .position(|n| n == name)
            .map(|pos| ChannelId(CORE_CHANNELS + pos))
    }

    /// Number of interned dynamic channels.
    #[must_use]
    pub fn len(&self) -> usize {
        self.names.len()
    }

    /// Whether no dynamic channels have been interned.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.names.is_empty()
    }

    /// Total bus width (core + dynamic) needed to hold every interned channel.
    #[must_use]
    pub fn total_channels(&self) -> usize {
        CORE_CHANNELS + self.names.len()
    }
}

/// The flat `SoA` signal board. `data.len() == channels * batch`; channel `c`, lane `b` is at
/// `c * batch + b`. Allocated once (via [`Bus::new`]); all accessors are allocation-free.
#[derive(Clone, Debug)]
pub struct Bus<T> {
    data: Vec<T>,
    channels: usize,
    batch: usize,
}

impl<T: Float> Bus<T> {
    /// Allocate a zeroed bus with `channels` total channels over `batch` lanes.
    ///
    /// # Panics
    /// Panics if `channels < CORE_CHANNELS` (the fixed core region must always fit) or `batch == 0`.
    #[must_use]
    pub fn new(channels: usize, batch: usize) -> Self {
        assert!(channels >= CORE_CHANNELS, "bus below the fixed core width");
        assert!(batch > 0, "batch dimension must be non-zero");
        Self {
            data: vec![T::zero(); channels * batch],
            channels,
            batch,
        }
    }

    /// Sized to hold the fixed core region plus every channel `interner` has interned.
    #[must_use]
    pub fn with_interner(interner: &ChannelInterner, batch: usize) -> Self {
        Self::new(interner.total_channels(), batch)
    }

    /// The batch (lane) count.
    #[must_use]
    pub fn batch(&self) -> usize {
        self.batch
    }

    /// The total channel count (core + dynamic).
    #[must_use]
    pub fn channels(&self) -> usize {
        self.channels
    }

    /// Zero every channel on every lane. Called at the top of **every RHS evaluation** — once per
    /// RK stage and once per fixed-point coupling iteration, not once per step — so any value that
    /// must persist across an eval (the road channels, and every boundary-controller channel such
    /// as `torque_scale` / `regen_limit_w` / the ERS command) is re-published immediately after the
    /// clear, on every eval. A boundary controller that publishes only once per step would see its
    /// value zeroed on the 2nd..nth stage and reach the blocks at a fraction of its intended value.
    pub fn clear(&mut self) {
        for v in &mut self.data {
            *v = T::zero();
        }
    }

    #[inline]
    fn flat(&self, channel: usize, lane: usize) -> usize {
        channel * self.batch + lane
    }

    /// Read a scalar core signal on `lane`.
    #[inline]
    #[must_use]
    pub fn get(&self, sig: CoreSignal, lane: usize) -> T {
        self.data[self.flat(sig as usize, lane)]
    }

    /// Write a scalar core signal on `lane`.
    #[inline]
    pub fn set(&mut self, sig: CoreSignal, lane: usize, value: T) {
        let i = self.flat(sig as usize, lane);
        self.data[i] = value;
    }

    /// Read a per-wheel core signal for `wheel` (`0..WHEELS`) on `lane`.
    #[inline]
    #[must_use]
    pub fn get_wheel(&self, sig: WheelSignal, wheel: usize, lane: usize) -> T {
        let channel = WHEEL_BASE + (sig as usize) * WHEELS + wheel;
        self.data[self.flat(channel, lane)]
    }

    /// Write a per-wheel core signal for `wheel` (`0..WHEELS`) on `lane`.
    #[inline]
    pub fn set_wheel(&mut self, sig: WheelSignal, wheel: usize, lane: usize, value: T) {
        let channel = WHEEL_BASE + (sig as usize) * WHEELS + wheel;
        let i = self.flat(channel, lane);
        self.data[i] = value;
    }

    /// Read a dynamic (interned) channel on `lane`.
    #[inline]
    #[must_use]
    pub fn get_channel(&self, id: ChannelId, lane: usize) -> T {
        self.data[self.flat(id.index(), lane)]
    }

    /// Write a dynamic (interned) channel on `lane`.
    #[inline]
    pub fn set_channel(&mut self, id: ChannelId, lane: usize, value: T) {
        let i = self.flat(id.index(), lane);
        self.data[i] = value;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn interning_is_idempotent_and_distinct() {
        let mut it = ChannelInterner::new();
        assert!(it.is_empty());
        let a = it.intern("tv.torque_fl");
        let b = it.intern("tv.torque_fr");
        let a2 = it.intern("tv.torque_fl");
        assert_eq!(a, a2, "same name → same id");
        assert_ne!(a, b, "distinct names → distinct ids");
        assert_eq!(it.len(), 2);
        assert_eq!(it.get("tv.torque_fr"), Some(b));
        assert_eq!(it.get("absent"), None);
        // Dynamic ids start past the fixed core region.
        assert!(a.index() >= core_channel_count());
        assert_eq!(it.total_channels(), core_channel_count() + 2);
    }

    #[test]
    fn core_and_wheel_channels_round_trip_per_lane() {
        let mut bus = Bus::<f64>::new(core_channel_count(), 3);
        bus.set(CoreSignal::Steer, 1, 0.25);
        bus.set_wheel(WheelSignal::TireFz, 2, 1, 4200.0);
        assert_eq!(bus.get(CoreSignal::Steer, 1), 0.25);
        assert_eq!(bus.get_wheel(WheelSignal::TireFz, 2, 1), 4200.0);
        // Other lanes are untouched (batch isolation).
        assert_eq!(bus.get(CoreSignal::Steer, 0), 0.0);
        assert_eq!(bus.get(CoreSignal::Steer, 2), 0.0);
        bus.clear();
        assert_eq!(bus.get(CoreSignal::Steer, 1), 0.0);
    }

    #[test]
    fn dynamic_channels_read_back() {
        let mut it = ChannelInterner::new();
        let id = it.intern("plugin.custom");
        let mut bus = Bus::<f32>::with_interner(&it, 2);
        bus.set_channel(id, 0, 1.5);
        assert_eq!(bus.get_channel(id, 0), 1.5);
        assert_eq!(bus.get_channel(id, 1), 0.0);
        assert_eq!(bus.channels(), it.total_channels());
        assert_eq!(bus.batch(), 2);
    }

    #[test]
    #[should_panic(expected = "fixed core width")]
    fn bus_below_core_width_panics() {
        let _ = Bus::<f64>::new(core_channel_count() - 1, 1);
    }
}
