// SPDX-License-Identifier: AGPL-3.0-only
//! The **fixed-step split integrator** (HANDOFF §11.2, Decision #30).
//!
//! Production transient runs use a *fixed*-step, *split* scheme — never an adaptive solver:
//!
//! * the smooth chassis/driveline states advance with a **Butcher-tableau-generic explicit
//!   Runge–Kutta** step ([`SimArena::step`] over a [`ButcherTableau`]) — Heun/RK2 by default, RK4
//!   selectable for convergence studies;
//! * the stiff tire-relaxation states advance with the **exact-exponential** update
//!   ([`crate::relax::exact_exponential`]);
//! * the slow states advance with **semi-implicit Euler** on a decimated clock
//!   ([`crate::relax::SlowClock`]);
//! * discrete transitions (gear shifts, mode changes) fire at **step boundaries** via a time-ordered
//!   [`EventQueue`] with a single linear back-interpolation of the crossing — no root-finding in the
//!   hot loop.
//!
//! All step scratch lives in a preallocated [`SimArena`], so stepping is allocation-free (CI-gated).
//! The stepper is generic over `f32`/`f64` and holds no clock/IO — wasm-clean.

use num_traits::Float;

/// A fixed-step explicit Runge–Kutta method (mirrors the `sim.yaml` `integrator` selector, but
/// core-local so this crate stays free of the schema).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RkMethod {
    /// Heun / explicit trapezoidal, 2nd order — the production default.
    Heun,
    /// Classical Runge–Kutta, 4th order — for convergence studies.
    Rk4,
}

impl RkMethod {
    /// The Butcher tableau for this method, materialised in `T`.
    #[must_use]
    pub fn tableau<T: Float>(self) -> ButcherTableau<T> {
        match self {
            RkMethod::Heun => ButcherTableau::heun(),
            RkMethod::Rk4 => ButcherTableau::rk4(),
        }
    }

    /// The method's global order of accuracy (Heun: 2, RK4: 4).
    #[must_use]
    pub fn order(self) -> u32 {
        match self {
            RkMethod::Heun => 2,
            RkMethod::Rk4 => 4,
        }
    }
}

/// A Butcher tableau for an explicit Runge–Kutta method. `a` is strictly lower-triangular
/// (`a[i].len() == i`); `b` are the weights; `c` the nodes. Built once (allocation at setup is fine);
/// consumed read-only in the loop.
#[derive(Clone, Debug)]
pub struct ButcherTableau<T> {
    a: Vec<Vec<T>>,
    b: Vec<T>,
    c: Vec<T>,
}

impl<T: Float> ButcherTableau<T> {
    /// Construct from coefficients, checking the explicit lower-triangular shape.
    ///
    /// # Panics
    /// Panics if the shapes are inconsistent (`a[i].len() != i`, or `b`/`c` lengths disagree).
    #[must_use]
    pub fn new(a: Vec<Vec<T>>, b: Vec<T>, c: Vec<T>) -> Self {
        let s = b.len();
        assert_eq!(c.len(), s, "c must have one node per stage");
        assert_eq!(a.len(), s, "a must have one row per stage");
        for (i, row) in a.iter().enumerate() {
            assert_eq!(row.len(), i, "explicit RK: row i must have i entries");
        }
        Self { a, b, c }
    }

    /// Heun (explicit trapezoidal, RK2): `c = [0, 1]`, `b = [½, ½]`.
    #[must_use]
    pub fn heun() -> Self {
        let half = T::from(0.5).expect("0.5 representable");
        let one = T::one();
        let zero = T::zero();
        Self::new(vec![vec![], vec![one]], vec![half, half], vec![zero, one])
    }

    /// Classical RK4: `c = [0, ½, ½, 1]`, `b = [⅙, ⅓, ⅓, ⅙]`.
    #[must_use]
    pub fn rk4() -> Self {
        let zero = T::zero();
        let one = T::one();
        let half = T::from(0.5).expect("0.5 representable");
        let sixth = T::from(1.0 / 6.0).expect("1/6 representable");
        let third = T::from(1.0 / 3.0).expect("1/3 representable");
        Self::new(
            vec![vec![], vec![half], vec![zero, half], vec![zero, zero, one]],
            vec![sixth, third, third, sixth],
            vec![zero, half, half, one],
        )
    }

    /// Number of stages.
    #[must_use]
    pub fn stages(&self) -> usize {
        self.b.len()
    }
}

/// Preallocated scratch for a fixed-step RK step over an `n`-dimensional state. Sized once; the step
/// itself never allocates.
#[derive(Clone, Debug)]
pub struct SimArena<T> {
    tableau: ButcherTableau<T>,
    /// Stage derivatives, flattened `stages × n`.
    k: Vec<T>,
    /// Stage-state scratch, length `n`.
    x_stage: Vec<T>,
    n: usize,
}

impl<T: Float> SimArena<T> {
    /// Allocate scratch for an `n`-state system stepped by `tableau`.
    #[must_use]
    pub fn new(tableau: ButcherTableau<T>, n: usize) -> Self {
        let stages = tableau.stages();
        Self {
            tableau,
            k: vec![T::zero(); stages * n],
            x_stage: vec![T::zero(); n],
            n,
        }
    }

    /// Convenience: allocate for `method` over `n` states.
    #[must_use]
    pub fn for_method(method: RkMethod, n: usize) -> Self {
        Self::new(method.tableau(), n)
    }

    /// State dimension.
    #[must_use]
    pub fn dim(&self) -> usize {
        self.n
    }

    /// The RK tableau in use.
    #[must_use]
    pub fn tableau(&self) -> &ButcherTableau<T> {
        &self.tableau
    }

    /// Advance `x` by one fixed step `dt` from time `t`, using the RHS `f(t, x, dxdt)`.
    ///
    /// `f` writes `dx/dt` for state `x` at the given time into its third argument. The step is a
    /// standard explicit RK sweep; no heap allocation occurs. Determinism: the stage and weight
    /// reductions run in fixed (ascending) order.
    ///
    /// # Panics
    /// Panics if `x.len() != self.dim()`.
    pub fn step<F>(&mut self, x: &mut [T], t: T, dt: T, mut f: F)
    where
        F: FnMut(T, &[T], &mut [T]),
    {
        assert_eq!(x.len(), self.n, "state length mismatches the arena");
        let stages = self.tableau.stages();
        for i in 0..stages {
            // x_stage = x + dt · Σ_{j<i} a[i][j] · k[j]
            self.x_stage.copy_from_slice(x);
            for j in 0..i {
                let aij = self.tableau.a[i][j];
                if aij != T::zero() {
                    let scale = dt * aij;
                    let kj = &self.k[j * self.n..(j + 1) * self.n];
                    for (xs, &kv) in self.x_stage.iter_mut().zip(kj) {
                        *xs = *xs + scale * kv;
                    }
                }
            }
            let ti = t + self.tableau.c[i] * dt;
            let (before, rest) = self.k.split_at_mut(i * self.n);
            let _ = before;
            let ki = &mut rest[..self.n];
            f(ti, &self.x_stage, ki);
        }
        // x += dt · Σ_i b[i] · k[i]
        for i in 0..stages {
            let scale = dt * self.tableau.b[i];
            let ki = &self.k[i * self.n..(i + 1) * self.n];
            for (xv, &kv) in x.iter_mut().zip(ki) {
                *xv = *xv + scale * kv;
            }
        }
    }
}

/// A discrete event scheduled at a boundary time (gear shift, ERS mode change, …). `payload` is a
/// caller-defined tag identifying the transition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScheduledEvent<T, E> {
    /// The (fast-clock) time the event is due.
    pub time: T,
    /// The transition to apply.
    pub payload: E,
}

/// A time-ordered queue of discrete events processed at step boundaries (HANDOFF §11.2). Events are
/// applied at the boundary at or after their due time; a single linear [`back_interpolate`] recovers
/// the sub-step crossing fraction where a caller needs it. No root-finding.
///
/// Populated at assembly / on the fly (allocation there is fine); draining in the loop pops from the
/// tail of a descending-sorted buffer, which does not allocate.
#[derive(Clone, Debug)]
pub struct EventQueue<T, E> {
    /// Sorted **descending** by time, so the soonest event is at the end (O(1) pop).
    pending: Vec<ScheduledEvent<T, E>>,
}

impl<T: Float, E> Default for EventQueue<T, E> {
    fn default() -> Self {
        Self {
            pending: Vec::new(),
        }
    }
}

impl<T: Float, E> EventQueue<T, E> {
    /// An empty queue.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Reserve capacity for `n` events so later scheduling need not reallocate.
    pub fn reserve(&mut self, n: usize) {
        self.pending.reserve(n);
    }

    /// Schedule `event`, keeping the queue sorted (soonest last).
    pub fn schedule(&mut self, event: ScheduledEvent<T, E>) {
        // Insertion sort by descending time (events are few and near-sorted in practice).
        let mut i = self.pending.len();
        self.pending.push(event);
        while i > 0 && self.pending[i - 1].time < self.pending[i].time {
            self.pending.swap(i - 1, i);
            i -= 1;
        }
    }

    /// Whether any events remain.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pending.is_empty()
    }

    /// Number of pending events.
    #[must_use]
    pub fn len(&self) -> usize {
        self.pending.len()
    }

    /// Time of the soonest pending event, if any.
    #[must_use]
    pub fn peek_time(&self) -> Option<T> {
        self.pending.last().map(|e| e.time)
    }

    /// Pop the soonest event if it is due at or before `t_now`; returns `None` otherwise. Call in a
    /// loop each boundary to drain every due event. Allocation-free.
    pub fn pop_due(&mut self, t_now: T) -> Option<ScheduledEvent<T, E>> {
        match self.pending.last() {
            Some(e) if e.time <= t_now => self.pending.pop(),
            _ => None,
        }
    }
}

/// Linear back-interpolation of a threshold crossing across one step: given a monitored quantity
/// `g_prev` at the step start and `g_now` at the step end that changed sign, returns the fraction
/// `θ ∈ [0, 1]` of the step at which `g` crossed zero (`t_cross = t_prev + θ·dt`). Returns `0` if the
/// endpoints do not straddle zero (no crossing to interpolate).
#[inline]
#[must_use]
pub fn back_interpolate<T: Float>(g_prev: T, g_now: T) -> T {
    let denom = g_prev - g_now;
    if denom == T::zero() {
        return T::zero();
    }
    let theta = g_prev / denom;
    // A genuine crossing lands θ in [0, 1]; same-sign endpoints (no crossing this step) put θ
    // outside it — return 0 so callers can treat 0 as the "no crossing" sentinel.
    if theta >= T::zero() && theta <= T::one() {
        theta
    } else {
        T::zero()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn tableaux_have_the_expected_shape() {
        assert_eq!(ButcherTableau::<f64>::heun().stages(), 2);
        assert_eq!(ButcherTableau::<f64>::rk4().stages(), 4);
        assert_eq!(RkMethod::Heun.order(), 2);
        assert_eq!(RkMethod::Rk4.order(), 4);
    }

    #[test]
    fn rk_integrates_a_quadratic_exactly() {
        // y' = 2t, y(0)=0 → y(1)=1. Both RK2 and RK4 integrate polynomials of low degree exactly.
        for method in [RkMethod::Heun, RkMethod::Rk4] {
            let mut arena = SimArena::for_method(method, 1);
            let mut x = [0.0f64];
            let dt = 0.1;
            let mut t = 0.0;
            for _ in 0..10 {
                arena.step(&mut x, t, dt, |ti, _x, dx| dx[0] = 2.0 * ti);
                t += dt;
            }
            assert!((x[0] - 1.0).abs() < 1e-12, "{method:?} gave {}", x[0]);
        }
    }

    #[test]
    fn rk_step_is_generic_over_f32() {
        let mut arena = SimArena::for_method(RkMethod::Heun, 1);
        let mut x = [0.0f32];
        arena.step(&mut x, 0.0, 1.0, |_t, _x, dx| dx[0] = 3.0);
        assert!((x[0] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn back_interpolation_finds_the_crossing_fraction() {
        assert!((back_interpolate(1.0, -1.0) - 0.5).abs() < 1e-12);
        assert!((back_interpolate(3.0, -1.0) - 0.75).abs() < 1e-12);
        assert!((back_interpolate(-2.0, 6.0) - 0.25).abs() < 1e-12); // opposite-direction crossing
                                                                     // No sign change → 0 (the "no crossing" sentinel), in BOTH directions of |g_prev| vs |g_now|.
        assert_eq!(back_interpolate(1.0, 2.0), 0.0); // θ would be < 0
        assert_eq!(back_interpolate(2.0, 1.0), 0.0); // θ would be > 1 (previously mis-clamped to 1)
        assert_eq!(back_interpolate(-2.0, -1.0), 0.0);
        assert_eq!(back_interpolate(1.0, 1.0), 0.0); // flat → 0 (no crossing)
    }

    #[test]
    fn event_queue_drains_in_time_order() {
        let mut q: EventQueue<f64, u32> = EventQueue::new();
        assert!(q.is_empty());
        q.schedule(ScheduledEvent {
            time: 0.30,
            payload: 3,
        });
        q.schedule(ScheduledEvent {
            time: 0.10,
            payload: 1,
        });
        q.schedule(ScheduledEvent {
            time: 0.20,
            payload: 2,
        });
        assert_eq!(q.len(), 3);
        assert_eq!(q.peek_time(), Some(0.10));
        // Nothing due before 0.10.
        assert!(q.pop_due(0.05).is_none());
        // At t=0.25, events 1 and 2 are due (in order); 3 is not.
        assert_eq!(q.pop_due(0.25).map(|e| e.payload), Some(1));
        assert_eq!(q.pop_due(0.25).map(|e| e.payload), Some(2));
        assert!(q.pop_due(0.25).is_none());
        assert_eq!(q.pop_due(1.0).map(|e| e.payload), Some(3));
        assert!(q.is_empty());
    }
}
