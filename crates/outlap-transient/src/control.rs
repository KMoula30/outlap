// SPDX-License-Identifier: AGPL-3.0-only
//! The transient **rule-based control layer's discrete state** (PR6, HANDOFF §8.2/§8.4): the
//! gear-shift finite state machine and the slow-state (battery) stack interface.
//!
//! These carry the *discrete* / *slow* state the continuous `SoA` fast buffer deliberately does not
//! ([`outlap_core::state`] frozen-layout note): the engaged gear index and shift timers live on the
//! step-boundary [`EventQueue`]; the pack charge advances on the
//! decimated slow clock. The orchestrator ([`crate::lap`]) owns one of each and publishes their
//! outputs onto the bus every step, frozen across the RK sweep exactly like the relaxation and
//! load-transfer coupling.

use num_traits::Float;

use outlap_core::integrator::{back_interpolate, EventQueue, ScheduledEvent};

/// Fraction of the total shift time spent in the pure **torque-cut** window before the ratio swap;
/// the remainder is the **clutch re-engagement ramp** (HANDOFF §8.2). A literature-typical split for
/// a seamless-shift race gearbox (documented in `docs/theory/transient_control.md`); the total
/// duration is the vehicle's own `Gearbox.shift_time_s`.
pub const SHIFT_CUT_FRACTION: f64 = 0.35;

/// Down-shift speed hysteresis: down-shift only once the speed falls below `HYSTERESIS ·` the gear's
/// up-shift threshold, so a car hovering at a shift point does not chatter between gears.
pub const DOWNSHIFT_HYSTERESIS: f64 = 0.93;

/// A per-station **named-shift-map selector** (§8.3, D-M6-9): the `u(s)` `shift_map_id` schedule
/// resampled onto the shifter as the resolved-map index active at each arc-length station. Empty ⇒
/// every station uses map 0 (the derived default), so the FSM is byte-identical to the pre-1.8 path.
///
/// Self-contained plain data fed PRE-SAMPLED by the assembly layer (the solver crate never reaches
/// for `outlap-powertrain`'s `UsSchedule`, exactly as [`crate::FuelSlow`] receives its inertial model
/// rather than the QSS `FuelModel`). The raw cumulative arc-length the solver hands in is wrapped
/// into one lap so the schedule repeats every lap — the line table and the ERS schedule wrap the same
/// way — and the resulting id is clamped to the resolved-map count (a construction-time
/// [`crate::ScheduleError::UnknownShiftMap`] already rejects an out-of-range id, so the clamp is a
/// panic-safety belt, not a policy).
#[derive(Clone, Debug, Default)]
pub struct ShiftSchedule {
    /// Ascending arc-length breakpoints, m (the last entry is one lap length). Empty ⇒ always map 0.
    stations_s: Vec<f64>,
    /// The resolved-map index selected at each breakpoint (parallel to `stations_s`).
    map_id: Vec<u32>,
}

impl ShiftSchedule {
    /// Build a selector from the arc-length grid and its per-station map ids (parallel arrays). A
    /// mismatched length or an empty grid degrades to "always map 0" (the caller validates the ids).
    #[must_use]
    pub fn new(stations_s: Vec<f64>, map_id: Vec<u32>) -> Self {
        if stations_s.is_empty() || stations_s.len() != map_id.len() {
            return Self::default();
        }
        Self { stations_s, map_id }
    }

    /// The resolved-map index active at arc-length `s`, clamped to `[0, n_maps)`. The raw cumulative
    /// `s` is wrapped into one lap first (nearest breakpoint at/above it), mirroring the ERS
    /// schedule's `station` wrap so a stint's laps 2..n select the same maps as lap 1.
    #[must_use]
    fn map_index(&self, s: f64, n_maps: usize) -> usize {
        let Some(&length) = self.stations_s.last() else {
            return 0;
        };
        let s = if length > 0.0 {
            s.rem_euclid(length)
        } else {
            0.0
        };
        let station = match self
            .stations_s
            .binary_search_by(|x| x.partial_cmp(&s).unwrap_or(std::cmp::Ordering::Less))
        {
            Ok(i) => i,
            Err(i) => i.min(self.stations_s.len() - 1),
        };
        (self.map_id[station] as usize).min(n_maps.saturating_sub(1))
    }
}

/// A per-station **lift-and-coast selector** (§8.3, D-M6-9): the `u(s)` `lift_point` schedule
/// resampled onto the solver as the speed the driver's tracked reference is capped to at each
/// arc-length station. A non-finite / very large lift point ⇒ no cap (the un-lifted reference), so an
/// absent schedule is byte-identical to the pre-lift path.
///
/// The lift lowers the reference the *closed-loop* driver already tracks smoothly — no new
/// discontinuity beyond the braking zones the profile already imposes — so the car lifts off the
/// throttle early and coasts into the braking zone while the ERS banks the freed energy. Self-
/// contained plain data fed PRE-SAMPLED by the assembly layer (mirrors [`ShiftSchedule`]); the raw
/// cumulative `s` is wrapped into one lap so the schedule repeats every lap.
#[derive(Clone, Debug, Default)]
pub struct LiftSchedule {
    /// Ascending arc-length breakpoints, m (the last entry is one lap length). Empty ⇒ no lift.
    stations_s: Vec<f64>,
    /// The lift-and-coast speed cap at each breakpoint, m/s (`+∞` ⇒ no cap). Parallel to `stations_s`.
    lift_mps: Vec<f64>,
}

impl LiftSchedule {
    /// Build a selector from the arc-length grid + its per-station lift speeds. A mismatched length or
    /// an empty grid degrades to "no lift" (every cap `+∞`).
    #[must_use]
    pub fn new(stations_s: Vec<f64>, lift_mps: Vec<f64>) -> Self {
        if stations_s.is_empty() || stations_s.len() != lift_mps.len() {
            return Self::default();
        }
        Self {
            stations_s,
            lift_mps,
        }
    }

    /// Whether the schedule ever caps (some finite lift point) — lets the assembly skip attaching an
    /// all-`+∞` schedule, keeping the no-lift path provably byte-identical.
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.lift_mps.iter().any(|v| v.is_finite())
    }

    /// The lift speed cap active at arc-length `s`, m/s (`+∞` ⇒ no cap). The raw cumulative `s` is
    /// wrapped into one lap (nearest breakpoint at/above it), mirroring [`ShiftSchedule::map_index`].
    #[must_use]
    pub(crate) fn cap_at(&self, s: f64) -> f64 {
        let Some(&length) = self.stations_s.last() else {
            return f64::INFINITY;
        };
        let s = if length > 0.0 {
            s.rem_euclid(length)
        } else {
            0.0
        };
        let station = match self
            .stations_s
            .binary_search_by(|x| x.partial_cmp(&s).unwrap_or(std::cmp::Ordering::Less))
        {
            Ok(i) => i,
            Err(i) => i.min(self.stations_s.len() - 1),
        };
        self.lift_mps[station]
    }
}

/// A discrete gear-shift transition, fired at a step boundary off the [`EventQueue`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShiftEvent {
    /// The ratio swap: the engaged gear becomes the target gear (mid-shift, after the torque cut).
    Engage(usize),
    /// The clutch is fully re-engaged: the shift is complete, drive torque is restored.
    Complete,
}

/// The gear-shift finite state machine (HANDOFF §8.2). It watches the vehicle speed against a set of
/// up-shift thresholds (the traction crossover speeds, supplied by the assembly pipeline), and on a
/// crossing runs a **torque-cut → ratio-swap → clutch-ramp** shift consuming the gearbox's
/// `shift_time_s`. The discrete transitions fire off a time-ordered [`EventQueue`] with one linear
/// back-interpolation of the crossing (no root-finding, §11.2); the continuous drive-torque scale is
/// a plain function of the elapsed shift time, published each step for the powertrain to apply.
#[derive(Clone, Debug)]
pub struct Shifter<T> {
    /// Currently engaged gear index (0-based).
    gear: usize,
    /// Number of selectable gears (`≥ 1`).
    n_gears: usize,
    /// Resolved absolute up-shift-speed maps (§8.3, D-M6-9); `maps[0]` is the default (the derived
    /// schedule, or a `shift_maps` entry named `"default"`). Each is ascending with length
    /// `max(0, n_gears − 1)`; a `Factor` map is pre-multiplied at assembly. A single-entry `maps`
    /// with an empty [`ShiftSchedule`] is the pre-1.8 single-map FSM.
    maps: Vec<Vec<T>>,
    /// The per-station map selector (empty ⇒ always `maps[0]`).
    schedule: ShiftSchedule,
    /// The map index selected at the current step boundary (updated by [`Self::select_map`]).
    active_map: usize,
    /// Total shift duration, s (`Gearbox.shift_time_s`; `0` ⇒ instantaneous ideal shift).
    shift_time: T,
    /// Pending discrete shift transitions.
    events: EventQueue<T, ShiftEvent>,
    /// Active shift, if any: `(start_time, from_gear, to_gear)`.
    active: Option<(T, usize, usize)>,
    /// Last speed seen (for the back-interpolated threshold crossing).
    v_prev: T,
    /// Whether `v_prev` has been seeded.
    seeded: bool,
}

impl<T: Float> Shifter<T> {
    /// Build a shift FSM starting in gear 0 with a single up-shift map. `upshift_speeds` must be
    /// ascending with length `max(0, n_gears − 1)`; an empty list (single-speed transmission) never
    /// shifts. This is the pre-1.8 constructor — [`Self::with_maps`] adds the named-map set.
    #[must_use]
    pub fn new(n_gears: usize, upshift_speeds: Vec<T>, shift_time: T) -> Self {
        let n_gears = n_gears.max(1);
        let mut events = EventQueue::new();
        events.reserve(4);
        Self {
            gear: 0,
            n_gears,
            maps: vec![upshift_speeds],
            schedule: ShiftSchedule::default(),
            active_map: 0,
            shift_time,
            events,
            active: None,
            v_prev: T::zero(),
            seeded: false,
        }
    }

    /// Install the resolved named-map set + the per-station selector (§8.3, D-M6-9, consuming). `maps`
    /// is index-addressed by the `u(s)` `shift_map_id` (id 0 = the derived/`"default"` map); an empty
    /// `maps` degrades to the single map already held. Absent (or an empty `schedule`) ⇒ every station
    /// selects `maps[0]`, byte-identical to [`Self::new`].
    #[must_use]
    pub fn with_maps(mut self, maps: Vec<Vec<T>>, schedule: ShiftSchedule) -> Self {
        if !maps.is_empty() {
            self.maps = maps;
        }
        self.schedule = schedule;
        self.active_map = self.active_map.min(self.maps.len() - 1);
        self
    }

    /// Select the up-shift map active for arc-length `s` from the schedule (called at the step
    /// boundary before [`Self::update`]). A no-op when the schedule is empty (map 0 stays selected).
    /// The selection only changes *when the next shift triggers* — an in-progress shift is unaffected
    /// (the threshold test runs only while idle), so switching maps mid-corner is well-posed.
    pub fn select_map(&mut self, s: T) {
        if self.schedule.stations_s.is_empty() {
            return;
        }
        self.active_map = self
            .schedule
            .map_index(s.to_f64().unwrap_or(0.0), self.maps.len());
    }

    /// The up-shift speeds of the currently selected map.
    #[inline]
    fn active_speeds(&self) -> &[T] {
        &self.maps[self.active_map]
    }

    /// The engaged gear index (telemetry).
    #[must_use]
    pub fn gear(&self) -> usize {
        self.gear
    }

    /// Whether a shift is in progress.
    #[must_use]
    pub fn is_shifting(&self) -> bool {
        self.active.is_some()
    }

    /// Advance the FSM to the step boundary at time `t` (the step just integrated used step size
    /// `dt` and reached speed `v`), and return the drive-torque scale `∈ [0, 1]` to apply this step:
    /// `0` through the torque-cut window, ramping to `1` over the clutch re-engagement, `1` when
    /// engaged. Draining due events swaps the gear and completes the shift.
    ///
    /// **Drain order vs. map selection (D-M6-9).** The due events are drained *first*, in the
    /// [`EventQueue`]'s deterministic time-then-insertion (LIFO on equal-time ties) order, and only
    /// then — while idle — is the threshold tested against the map [`Self::select_map`] chose at this
    /// boundary. So a `shift_map_id` switch never races an in-flight ratio swap or completion: it can
    /// only change *which threshold the next shift decision uses*, keeping the discrete timeline and
    /// the per-station map selection cleanly separated.
    pub fn update(&mut self, t: T, dt: T, v: T) -> T {
        // Drain due discrete transitions (ratio swap, completion) at this boundary.
        while let Some(ev) = self.events.pop_due(t) {
            match ev.payload {
                ShiftEvent::Engage(to) => self.gear = to,
                ShiftEvent::Complete => self.active = None,
            }
        }
        // If idle, test the speed thresholds and possibly begin a shift.
        if self.active.is_none() && self.shift_time > T::zero() {
            self.maybe_begin_shift(t, dt, v);
        }
        self.v_prev = v;
        self.seeded = true;
        self.torque_scale(t)
    }

    /// Begin a shift if the speed has crossed an up-shift threshold (rising) or fallen below the
    /// hysteresis-scaled threshold of the gear below (falling). The crossing time is recovered by a
    /// single linear back-interpolation across the step just taken.
    fn maybe_begin_shift(&mut self, t: T, dt: T, v: T) {
        if !self.seeded {
            return; // need a previous speed to detect a crossing
        }
        // Up-shift: v crossed above the threshold out of the current gear.
        if self.gear + 1 < self.n_gears {
            let thr = self.active_speeds()[self.gear];
            if self.v_prev < thr && v >= thr {
                let theta = back_interpolate(self.v_prev - thr, v - thr);
                self.start_shift(t - dt + theta * dt, self.gear + 1);
                return;
            }
        }
        // Down-shift: v fell below the hysteresis-scaled up-shift threshold into the gear below.
        if self.gear > 0 {
            let hyst = T::from(DOWNSHIFT_HYSTERESIS).unwrap_or_else(T::one);
            let thr = self.active_speeds()[self.gear - 1] * hyst;
            if self.v_prev >= thr && v < thr {
                let theta = back_interpolate(thr - self.v_prev, thr - v);
                self.start_shift(t - dt + theta * dt, self.gear - 1);
            }
        }
    }

    /// Schedule the ratio-swap and completion events for a shift starting at `t_start` toward `to`.
    fn start_shift(&mut self, t_start: T, to: usize) {
        let from = self.gear;
        self.active = Some((t_start, from, to));
        let cut = T::from(SHIFT_CUT_FRACTION).unwrap_or_else(T::zero) * self.shift_time;
        self.events.schedule(ScheduledEvent {
            time: t_start + cut,
            payload: ShiftEvent::Engage(to),
        });
        self.events.schedule(ScheduledEvent {
            time: t_start + self.shift_time,
            payload: ShiftEvent::Complete,
        });
    }

    /// The drive-torque scale at time `t` for the active shift (or `1` when engaged).
    fn torque_scale(&self, t: T) -> T {
        let (one, zero) = (T::one(), T::zero());
        let Some((start, _, _)) = self.active else {
            return one;
        };
        let cut = T::from(SHIFT_CUT_FRACTION).unwrap_or_else(T::zero) * self.shift_time;
        let elapsed = t - start;
        if elapsed <= zero || self.shift_time <= zero {
            one
        } else if elapsed < cut {
            zero // torque cut
        } else if elapsed < self.shift_time {
            // Clutch re-engagement ramp 0 → 1 over the remaining window.
            (elapsed - cut) / (self.shift_time - cut)
        } else {
            one
        }
    }
}

/// The **slow-state stack** the transient orchestrator advances on the decimated slow clock (Decision
/// #6): the battery pack (and, later, the machine-thermal network). It is *received* by the solver
/// as a boxed artifact (the concrete implementation lives at the Python boundary, wrapping the QSS
/// `outlap_qss::t1::Pack` primitive) so the wasm-clean transient crate never depends on the
/// QSS trim/envelope machinery — mirroring how the line table and envelope are handed in (§11.1).
///
/// It is touched only once every `slow_decimation` fast steps (never the hot RK path), so the single
/// dynamic dispatch is off the hot loop; implementations must still be allocation-free per call.
pub trait SlowStack {
    /// Advance the slow states by `dt_s`, Coulomb-counting `net_charge_power_w` into the pack over the
    /// interval — the recovered regen power **minus** the electrical traction draw. Positive charges
    /// the pack (braking dominates the window), negative discharges it (drive dominates), so the state
    /// of charge moves both ways over a lap.
    fn on_slow_step(&mut self, dt_s: f64, net_charge_power_w: f64);
    /// The current battery regen (charge) power ceiling, W (0 at full charge / no battery).
    fn regen_power_limit_w(&self) -> f64;
    /// The current battery **discharge** (draw) power ceiling, W — the mirror of
    /// [`Self::regen_power_limit_w`], refreshed on the same slow clock and consumed by the ERS
    /// deploy realization (`0` below the `SoC`-window floor, so a `SoC`-starved MGU-K stops deploying).
    /// Defaults to "no cap" so the energy-double test mock and any pack-free stack keep compiling.
    fn discharge_power_limit_w(&self) -> f64 {
        f64::INFINITY
    }
    /// The current pack state of charge, 0..1.
    fn soc(&self) -> f64;
    /// The current pack temperature, °C.
    fn temp_c(&self) -> f64;
}

/// The per-step state a boundary [`ErsGovernor`] decides on. Assembled by the solver once per step
/// at the boundary from the current fast state and the slow-clock-refreshed pack limits.
#[derive(Clone, Copy, Debug, Default)]
pub struct ErsStepInput {
    /// Arc-length station of the car this step, m — the `u(s)` schedule (if any) is indexed by it.
    pub s: f64,
    /// Vehicle speed, m/s.
    pub v: f64,
    /// Driver throttle demand this step, `0..1`.
    pub throttle: f64,
    /// Driver brake demand this step, `0..1`.
    pub brake: f64,
    /// Step length, s (turns per-lap MJ budgets into per-step power ceilings).
    pub dt: f64,
    /// Pack state of charge, 0..1 (slow-clock refreshed) — the recharge-target gate reads it.
    pub soc: f64,
    /// Pack discharge (draw) ceiling, W (slow-clock refreshed) — caps the realized deploy; `0`
    /// below the SoC-window floor, so a SoC-starved MGU-K stops deploying.
    pub discharge_limit_w: f64,
    /// Pack charge-acceptance ceiling, W (slow-clock refreshed) — caps the realized harvest.
    pub regen_limit_w: f64,
    /// Full-throttle mechanical drive power available at this speed, W — the ICE surplus the K may
    /// back-drive against for the part-throttle / super-clip recharge phases.
    pub mech_drive_power_w: f64,
}

/// What a boundary [`ErsGovernor`] publishes for the powertrain block and the per-lap ledger.
#[derive(Clone, Copy, Debug, Default)]
pub struct ErsStepOut {
    /// The MGU-K deploy wheel force, N (`+` deploy under power / `−` super-clip back-drive).
    pub deploy_force_n: f64,
    /// The realized electrical deploy power drawn from the pack, W (≥ 0).
    pub deploy_power_w: f64,
    /// The realized electrical harvest banked into the pack, W (≥ 0).
    pub harvest_power_w: f64,
}

/// The **2026 ERS energy manager** as the transient orchestrator's step-boundary controller
/// (sense → control → actuate → integrate, §6.2b): it decides the MGU-K deploy/harvest ONCE per
/// step from the boundary state and the slow-clock pack limits, publishing frozen bus channels the
/// pure powertrain block consumes every RHS evaluation (the `torque_scale` two-layer pattern).
///
/// Like [`SlowStack`], the concrete implementation lives at the Python boundary — it wraps the
/// `outlap-powertrain` [`EnergyManager`](https://docs.rs/outlap-powertrain) and the QSS mechanical
/// facts, so the wasm-clean transient crate never depends on the manager/QSS machinery. Both tiers
/// therefore drive the SAME rulebook (parity gate #4 compares physics, not two rule copies).
pub trait ErsGovernor {
    /// Decide this step's deploy/harvest from the boundary state (allocation-free per call).
    fn decide(&mut self, inp: &ErsStepInput) -> ErsStepOut;
    /// Reset the per-lap energy ledger at the start/finish line (the pack `SoC` is NOT reset).
    fn reset_lap(&mut self);
    /// The electrical deploy energy banked this lap so far, J (for the result channels).
    fn deploy_j(&self) -> f64;
    /// The electrical harvest energy banked this lap so far, J.
    fn harvest_j(&self) -> f64;
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn single_speed_transmission_never_shifts() {
        let mut sh = Shifter::<f64>::new(1, vec![], 0.05);
        for k in 0..100 {
            let t = f64::from(k) * 0.01;
            let scale = sh.update(t, 0.01, 10.0 + f64::from(k));
            assert_eq!(scale, 1.0);
            assert_eq!(sh.gear(), 0);
        }
    }

    #[test]
    fn upshift_cuts_torque_then_ramps_and_swaps_gear() {
        // Two gears, up-shift at 30 m/s, 100 ms shift. Accelerate through the threshold.
        let mut sh = Shifter::<f64>::new(2, vec![30.0], 0.1);
        let dt = 0.01;
        let mut min_scale = 1.0;
        let mut swapped_at = None;
        for k in 0..40 {
            let t = f64::from(k) * dt;
            let v = 25.0 + 0.5 * f64::from(k); // crosses 30 at k=10
            let scale = sh.update(t, dt, v);
            min_scale = f64::min(min_scale, scale);
            if sh.gear() == 1 && swapped_at.is_none() {
                swapped_at = Some(t);
            }
        }
        assert!(min_scale < 1e-9, "torque must be fully cut mid-shift");
        assert_eq!(sh.gear(), 1, "gear engaged after the shift");
        assert!(!sh.is_shifting(), "shift completes");
        // The ratio swap happens after the cut window (~35 ms into the shift near t≈0.1 s).
        let swapped_at = swapped_at.expect("gear swapped");
        assert!(
            swapped_at > 0.10,
            "swap after the torque-cut window: {swapped_at}"
        );
    }

    /// The first speed at which the FSM begins a shift, running `sched`/`maps` over a rising ramp.
    fn first_shift_speed(maps: Vec<Vec<f64>>, sched: ShiftSchedule) -> Option<f64> {
        let mut sh = Shifter::<f64>::new(2, vec![30.0], 0.1).with_maps(maps, sched);
        let dt = 0.01;
        for k in 0..60 {
            let t = f64::from(k) * dt;
            let v = 15.0 + 0.5 * f64::from(k); // crosses 20 at k=10, 30 at k=30
            let s = 10.0 * v; // arbitrary monotone arc-length for the selector
            let was_shifting = sh.is_shifting();
            sh.select_map(s);
            sh.update(t, dt, v);
            if !was_shifting && sh.is_shifting() {
                return Some(v);
            }
        }
        None
    }

    #[test]
    fn shift_map_id_selects_a_different_map() {
        // Two maps: default (id 0) up-shifts at 30 m/s, id 1 up-shifts earlier at 20 m/s.
        let maps = || vec![vec![30.0], vec![20.0]];
        // A schedule pinning map 1 everywhere begins the shift ~10 m/s earlier than the default.
        let v_default = first_shift_speed(maps(), ShiftSchedule::new(vec![0.0, 500.0], vec![0, 0]))
            .expect("default map shifts");
        let v_map1 = first_shift_speed(maps(), ShiftSchedule::new(vec![0.0, 500.0], vec![1, 1]))
            .expect("map 1 shifts");
        assert!(
            v_map1 < v_default - 5.0,
            "the earlier map begins the shift sooner: map1 {v_map1} vs default {v_default}"
        );
    }

    #[test]
    fn empty_schedule_is_byte_identical_to_the_single_map_fsm() {
        // A `with_maps` install carrying the same single map + an empty schedule must reproduce the
        // pre-1.8 `new`-only FSM exactly (the named-map path is inert without a schedule).
        let run = |sh: &mut Shifter<f64>| {
            let mut trace = Vec::new();
            for k in 0..120 {
                let t = f64::from(k) * 0.01;
                let v = 10.0 + 0.35 * f64::from(k);
                sh.select_map(1000.0); // no schedule ⇒ ignored
                trace.push((sh.update(t, 0.01, v).to_bits(), sh.gear()));
            }
            trace
        };
        let mut plain = Shifter::<f64>::new(3, vec![20.0, 40.0], 0.08);
        let mut mapped = Shifter::<f64>::new(3, vec![20.0, 40.0], 0.08)
            .with_maps(vec![vec![20.0, 40.0]], ShiftSchedule::default());
        assert_eq!(
            run(&mut plain),
            run(&mut mapped),
            "named-map path inert without a schedule"
        );
    }

    #[test]
    fn shift_schedule_wraps_and_clamps() {
        let sched = ShiftSchedule::new(vec![0.0, 100.0, 200.0], vec![0, 1, 2]);
        assert_eq!(sched.map_index(50.0, 3), 1, "nearest breakpoint at/above s");
        assert_eq!(sched.map_index(150.0, 3), 2);
        assert_eq!(
            sched.map_index(250.0, 3),
            1,
            "wraps a second lap: 250 → 50 → id 1"
        );
        assert_eq!(
            sched.map_index(150.0, 2),
            1,
            "clamps an id past the resolved-map count"
        );
        assert_eq!(
            ShiftSchedule::default().map_index(123.0, 4),
            0,
            "empty ⇒ map 0"
        );
    }

    #[test]
    fn shift_timeline_is_deterministic() {
        let run = || {
            let mut sh = Shifter::<f64>::new(3, vec![20.0, 40.0], 0.08);
            let mut trace = Vec::new();
            for k in 0..120 {
                let t = f64::from(k) * 0.01;
                let v = 10.0 + 0.35 * f64::from(k);
                trace.push((sh.update(t, 0.01, v), sh.gear()));
            }
            trace
        };
        assert_eq!(run(), run(), "the shift schedule is bit-reproducible");
    }
}
