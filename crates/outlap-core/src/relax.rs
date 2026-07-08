// SPDX-License-Identifier: AGPL-3.0-only
//! Split-integrator sub-steppers for the **stiff** channels (HANDOFF §11.2).
//!
//! The chassis/driveline states integrate with an explicit Runge–Kutta stepper ([`crate::integrator`]),
//! but two families of state are stiff and get closed-form sub-steps instead, so the 1 ms step stays
//! stable without an implicit solve:
//!
//! * **Tire relaxation** — a first-order slip lag `σ·ẋ + |V|·x = |V|·x_ss`. The
//!   [`exact_exponential`] update `x ← x_ss + (x − x_ss)·exp(−|V|·dt/σ)` is the analytic solution over
//!   a step at frozen `x_ss`, unconditionally stable at every speed. `outlap_tire::relax_step` floors
//!   `σ` and calls this primitive, so there is a single implementation.
//! * **Slow states** (temperatures, wear, SOC, fuel) — advanced with [`semi_implicit_decay`], a
//!   semi-implicit Euler step that is implicit in the diagonal decay term (A-stable for decay ≥ 0),
//!   fired on a decimated clock ([`SlowClock`]).

use num_traits::Float;

/// Exact-exponential update of one first-order relaxation state over a step `dt` at a frozen target.
///
/// Solves `σ·ẋ + |V|·x = |V|·x_ss` in closed form: `x ← x_ss + (x − x_ss)·exp(−|V|·dt/σ)`. The decay
/// factor lies in `(0, 1]` for `|V|, dt ≥ 0`, so the update is a pure contraction toward `x_ss` —
/// unconditionally stable, and exact (two half-steps equal one full step). `sigma` must be strictly
/// positive; callers that may see a zero length (e.g. `outlap_tire`) floor it first.
#[inline]
pub fn exact_exponential<T: Float>(x: T, x_ss: T, v_abs: T, dt: T, sigma: T) -> T {
    let zero = T::zero();
    let decay = (-(v_abs.max(zero) * dt.max(zero)) / sigma).exp();
    x_ss + (x - x_ss) * decay
}

/// Semi-implicit Euler step for a slow state governed by `ẋ = source − decay·x` (decay ≥ 0).
///
/// The decay term is taken implicitly, giving `x ← (x + dt·source) / (1 + dt·decay)` — A-stable, so a
/// large decimated slow step cannot ring or overshoot. With `decay == 0` this reduces to explicit
/// Euler `x ← x + dt·source`.
#[inline]
pub fn semi_implicit_decay<T: Float>(x: T, decay: T, source: T, dt: T) -> T {
    let one = T::one();
    (x + dt * source) / (one + dt * decay.max(T::zero()))
}

/// The decimated **slow clock**: the split integrator advances the slow states once every
/// `decimation` fast steps (HANDOFF §11.2 / §6.1). Slow states move on 10–100 s timescales, so
/// resolving them at the 1 ms fast step is wasteful; decimation trades that away deterministically.
#[derive(Clone, Copy, Debug)]
pub struct SlowClock {
    decimation: u32,
    counter: u32,
}

impl SlowClock {
    /// A slow clock that fires every `decimation` fast steps.
    ///
    /// # Panics
    /// Panics if `decimation == 0`.
    #[must_use]
    pub fn new(decimation: u32) -> Self {
        assert!(decimation > 0, "slow-clock decimation must be non-zero");
        Self {
            decimation,
            counter: 0,
        }
    }

    /// The decimation factor.
    #[must_use]
    pub fn decimation(&self) -> u32 {
        self.decimation
    }

    /// Advance the clock by one fast step; returns `true` on the steps where the slow substep fires
    /// (every `decimation`-th call, starting with the `decimation`-th).
    pub fn tick(&mut self) -> bool {
        self.counter += 1;
        if self.counter >= self.decimation {
            self.counter = 0;
            true
        } else {
            false
        }
    }

    /// The slow-substep step size `decimation · fast_dt`.
    #[must_use]
    pub fn slow_dt<T: Float>(&self, fast_dt: T) -> T {
        fast_dt * T::from(self.decimation).unwrap_or_else(T::one)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn exact_exponential_contracts_toward_the_target() {
        let (x0, xss, v, sigma) = (0.0, 1.0, 30.0, 0.3);
        let x1 = exact_exponential(x0, xss, v, 0.05, sigma);
        assert!(x1 > x0 && x1 < xss, "must move toward xss: {x1}");
        // A large step drives it essentially to steady state.
        let x_big = exact_exponential(x0, xss, v, 100.0, sigma);
        assert!((x_big - xss).abs() < 1e-9);
    }

    #[test]
    fn exact_exponential_is_exact_under_a_half_step_split() {
        let (x0, xss, v, sigma, dt) = (0.2, -0.4, 25.0, 0.25, 0.02);
        let one_full = exact_exponential(x0, xss, v, dt, sigma);
        let half = exact_exponential(x0, xss, v, dt / 2.0, sigma);
        let two_half = exact_exponential(half, xss, v, dt / 2.0, sigma);
        assert!((one_full - two_half).abs() < 1e-14);
    }

    #[test]
    fn zero_speed_freezes_the_relaxation_state() {
        // exp(0) = 1 → x is returned unchanged (modulo the reassociation round-off of x_ss+(x-x_ss)).
        assert!((exact_exponential(0.3, 1.0, 0.0, 0.01, 0.2) - 0.3).abs() < 1e-12);
    }

    #[test]
    fn semi_implicit_decay_is_stable_and_reduces_to_euler() {
        // decay = 0 → explicit Euler.
        assert!((semi_implicit_decay(1.0, 0.0, 2.0, 0.5) - 2.0).abs() < 1e-12);
        // A huge decay·dt cannot overshoot: x stays between 0 and the steady state source/decay.
        let x = semi_implicit_decay(0.0, 1000.0, 500.0, 10.0);
        assert!((0.0..=0.5).contains(&x), "overshoot: {x}");
        // Fixed point x* = source/decay is reproduced when started there.
        let x_star = 5.0 / 2.0;
        assert!((semi_implicit_decay(x_star, 2.0, 5.0, 0.1) - x_star).abs() < 1e-12);
    }

    #[test]
    fn slow_clock_fires_on_the_decimation_boundary() {
        let mut clock = SlowClock::new(3);
        assert_eq!(clock.decimation(), 3);
        assert_eq!(
            [
                clock.tick(),
                clock.tick(),
                clock.tick(),
                clock.tick(),
                clock.tick(),
                clock.tick()
            ],
            [false, false, true, false, false, true]
        );
        assert!((clock.slow_dt(0.001) - 0.003).abs() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "decimation must be non-zero")]
    fn slow_clock_rejects_zero_decimation() {
        let _ = SlowClock::new(0);
    }
}
