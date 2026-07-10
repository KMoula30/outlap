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
    /// Up-shift speed threshold out of gear `g` (length `n_gears − 1`), m/s.
    upshift_speeds: Vec<T>,
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
    /// Build a shift FSM starting in gear 0. `upshift_speeds` must be ascending with length
    /// `max(0, n_gears − 1)`; an empty list (single-speed transmission) never shifts.
    #[must_use]
    pub fn new(n_gears: usize, upshift_speeds: Vec<T>, shift_time: T) -> Self {
        let n_gears = n_gears.max(1);
        let mut events = EventQueue::new();
        events.reserve(4);
        Self {
            gear: 0,
            n_gears,
            upshift_speeds,
            shift_time,
            events,
            active: None,
            v_prev: T::zero(),
            seeded: false,
        }
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
            let thr = self.upshift_speeds[self.gear];
            if self.v_prev < thr && v >= thr {
                let theta = back_interpolate(self.v_prev - thr, v - thr);
                self.start_shift(t - dt + theta * dt, self.gear + 1);
                return;
            }
        }
        // Down-shift: v fell below the hysteresis-scaled up-shift threshold into the gear below.
        if self.gear > 0 {
            let hyst = T::from(DOWNSHIFT_HYSTERESIS).unwrap_or_else(T::one);
            let thr = self.upshift_speeds[self.gear - 1] * hyst;
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
    /// The current pack state of charge, 0..1.
    fn soc(&self) -> f64;
    /// The current pack temperature, °C.
    fn temp_c(&self) -> f64;
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
