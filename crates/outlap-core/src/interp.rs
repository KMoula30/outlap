// SPDX-License-Identifier: AGPL-3.0-only
//! Monotone cubic Hermite interpolation — the one shared gridded-map interpolant (Decision #30).
//!
//! Piecewise cubic Hermite with Fritsch–Carlson tangent limiting (F. N. Fritsch & R. E. Carlson,
//! "Monotone Piecewise Cubic Interpolation", *SIAM J. Numer. Anal.* 17(2), 1980, pp. 238–246). The
//! result is C¹ and shape-preserving: it never overshoots the data and it is monotone on every
//! interval where the samples are monotone. Every gridded lookup in outlap (engine/aero maps, the
//! track's per-`s` data channels, arc-length inversion) shares this implementation.
//!
//! The interpolant exposes an [analytic derivative](MonotoneCubic::deriv) (Decision #30 requires it
//! for the Newton solvers in the transient tiers).
//!
//! # Symbols
//!
//! Following the paper: `Δ_k` are the secant slopes, `m_k` the knot tangents; the Hermite basis is
//! `h00,h10,h01,h11` on the unit interval `t = (x − x_k) / h_k`.

use num_traits::Float;

/// Error building a [`MonotoneCubic`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum InterpError {
    /// Fewer than two knots were supplied (nothing to interpolate between).
    #[error("need at least two knots, got {got}")]
    TooFewKnots {
        /// The number of knots supplied.
        got: usize,
    },
    /// `xs` and `ys` had different lengths.
    #[error("xs has {xs} knots but ys has {ys}")]
    LengthMismatch {
        /// Length of the `xs` grid.
        xs: usize,
        /// Length of the `ys` values.
        ys: usize,
    },
    /// The `xs` grid was not strictly increasing at the given index.
    #[error("xs must be strictly increasing; xs[{index}] >= xs[{}]", index + 1)]
    NotIncreasing {
        /// The index `k` where `xs[k] >= xs[k+1]`.
        index: usize,
    },
}

/// A C¹ monotone piecewise-cubic Hermite interpolant over a strictly increasing grid.
///
/// Construct with [`MonotoneCubic::new`]; evaluate with [`MonotoneCubic::eval`] and
/// [`MonotoneCubic::deriv`]. Queries outside `[xs[0], xs[last]]` clamp to the nearest endpoint
/// value (derivative zero), matching the "saturate at the table edge" convention for lookup maps.
#[derive(Clone, Debug)]
pub struct MonotoneCubic<T> {
    xs: Vec<T>,
    ys: Vec<T>,
    /// Fritsch–Carlson-limited tangents, one per knot.
    m: Vec<T>,
}

impl<T: Float> MonotoneCubic<T> {
    /// Build a monotone cubic interpolant from a strictly increasing grid `xs` and values `ys`.
    ///
    /// # Errors
    /// Returns [`InterpError`] if there are fewer than two knots, the lengths differ, or `xs` is
    /// not strictly increasing.
    pub fn new(xs: Vec<T>, ys: Vec<T>) -> Result<Self, InterpError> {
        if xs.len() != ys.len() {
            return Err(InterpError::LengthMismatch {
                xs: xs.len(),
                ys: ys.len(),
            });
        }
        if xs.len() < 2 {
            return Err(InterpError::TooFewKnots { got: xs.len() });
        }
        for k in 0..xs.len() - 1 {
            if xs[k + 1] <= xs[k] {
                return Err(InterpError::NotIncreasing { index: k });
            }
        }
        let m = fritsch_carlson_tangents(&xs, &ys);
        Ok(Self { xs, ys, m })
    }

    /// The interpolated value at `x` (clamped to the endpoint value outside the grid).
    pub fn eval(&self, x: T) -> T {
        let (k, t, h) = self.locate(x);
        let (h00, h10, h01, h11) = hermite_basis(t);
        h00 * self.ys[k] + h10 * h * self.m[k] + h01 * self.ys[k + 1] + h11 * h * self.m[k + 1]
    }

    /// The analytic derivative `dy/dx` at `x` (zero outside the grid; Decision #30).
    pub fn deriv(&self, x: T) -> T {
        // Outside the grid we clamp to a constant, so the derivative is zero.
        if x <= self.xs[0] || x >= self.xs[self.xs.len() - 1] {
            return T::zero();
        }
        let (k, t, h) = self.locate(x);
        let (d00, d10, d01, d11) = hermite_basis_deriv(t);
        // d/dx = (1/h) d/dt for the position-basis terms; the tangent terms carry an explicit h.
        d00 / h * self.ys[k] + d10 * self.m[k] + d01 / h * self.ys[k + 1] + d11 * self.m[k + 1]
    }

    /// The grid span `[first, last]` knot abscissae.
    pub fn domain(&self) -> (T, T) {
        (self.xs[0], self.xs[self.xs.len() - 1])
    }

    /// Locate the interval containing `x`: returns `(k, t, h)` with `t ∈ [0,1]` the local parameter
    /// on `[xs[k], xs[k+1]]` and `h = xs[k+1] − xs[k]`. Clamps to the end intervals outside the grid.
    fn locate(&self, x: T) -> (usize, T, T) {
        let last = self.xs.len() - 1;
        if x <= self.xs[0] {
            return (0, T::zero(), self.xs[1] - self.xs[0]);
        }
        if x >= self.xs[last] {
            let h = self.xs[last] - self.xs[last - 1];
            return (last - 1, T::one(), h);
        }
        // First index k with xs[k+1] > x. `partition_point` is a branchless binary search.
        let k = self.xs.partition_point(|&xk| xk <= x) - 1;
        let h = self.xs[k + 1] - self.xs[k];
        let t = (x - self.xs[k]) / h;
        (k, t, h)
    }
}

/// Fritsch–Carlson (1980) monotone tangent computation.
fn fritsch_carlson_tangents<T: Float>(xs: &[T], ys: &[T]) -> Vec<T> {
    let n = xs.len();
    // Secant slopes Δ_k on each interval.
    let mut delta = Vec::with_capacity(n - 1);
    for k in 0..n - 1 {
        delta.push((ys[k + 1] - ys[k]) / (xs[k + 1] - xs[k]));
    }

    // Initial tangents: one-sided at the ends, arithmetic mean of neighbouring secants inside.
    let two = T::from(2).unwrap();
    let mut m = Vec::with_capacity(n);
    m.push(delta[0]);
    for k in 1..n - 1 {
        m.push((delta[k - 1] + delta[k]) / two);
    }
    m.push(delta[n - 2]);

    // Limit tangents onto the monotone region (the α²+β² ≤ 9 circle of Fritsch–Carlson).
    let three = T::from(3).unwrap();
    let nine = T::from(9).unwrap();
    for k in 0..n - 1 {
        if delta[k] == T::zero() {
            // Flat segment: force both tangents to zero to kill any overshoot.
            m[k] = T::zero();
            m[k + 1] = T::zero();
            continue;
        }
        let alpha = m[k] / delta[k];
        let beta = m[k + 1] / delta[k];
        // Negative α/β would introduce a non-monotone wiggle; clamp to zero.
        if alpha < T::zero() {
            m[k] = T::zero();
        }
        if beta < T::zero() {
            m[k + 1] = T::zero();
        }
        let s = alpha * alpha + beta * beta;
        if s > nine {
            let tau = three / s.sqrt();
            m[k] = tau * alpha * delta[k];
            m[k + 1] = tau * beta * delta[k];
        }
    }
    m
}

/// Cubic Hermite basis on the unit interval: `(h00, h10, h01, h11)`.
fn hermite_basis<T: Float>(t: T) -> (T, T, T, T) {
    let t2 = t * t;
    let t3 = t2 * t;
    let two = T::from(2).unwrap();
    let three = T::from(3).unwrap();
    let h00 = two * t3 - three * t2 + T::one();
    let h10 = t3 - two * t2 + t;
    let h01 = -two * t3 + three * t2;
    let h11 = t3 - t2;
    (h00, h10, h01, h11)
}

/// Derivatives w.r.t. `t` of the cubic Hermite basis: `(h00', h10', h01', h11')`.
fn hermite_basis_deriv<T: Float>(t: T) -> (T, T, T, T) {
    let t2 = t * t;
    let two = T::from(2).unwrap();
    let three = T::from(3).unwrap();
    let four = T::from(4).unwrap();
    let six = T::from(6).unwrap();
    let d00 = six * t2 - six * t;
    let d10 = three * t2 - four * t + T::one();
    let d01 = -six * t2 + six * t;
    let d11 = three * t2 - two * t;
    (d00, d10, d01, d11)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn passes_through_knots() {
        let xs = vec![0.0, 1.0, 2.0, 3.0];
        let ys = vec![0.0, 1.0, 4.0, 9.0];
        let f = MonotoneCubic::new(xs.clone(), ys.clone()).unwrap();
        for (&x, &y) in xs.iter().zip(&ys) {
            assert!(
                approx(f.eval(x), y, 1e-12),
                "knot ({x},{y}) not interpolated"
            );
        }
    }

    #[test]
    fn clamps_outside_domain() {
        let f = MonotoneCubic::new(vec![0.0, 1.0, 2.0], vec![10.0, 20.0, 30.0]).unwrap();
        assert!(approx(f.eval(-5.0), 10.0, 1e-12));
        assert!(approx(f.eval(99.0), 30.0, 1e-12));
        assert!(f.deriv(-5.0).abs() < 1e-12);
        assert!(f.deriv(99.0).abs() < 1e-12);
    }

    #[test]
    fn monotone_data_stays_monotone() {
        // A step-like monotone dataset that a plain cubic spline would overshoot on.
        let xs = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let ys = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let f = MonotoneCubic::new(xs, ys).unwrap();
        let mut prev = f.eval(0.0);
        let mut x = 0.0;
        while x <= 5.0 {
            let v = f.eval(x);
            assert!(
                v >= prev - 1e-12,
                "overshoot/non-monotone at x={x}: {v} < {prev}"
            );
            assert!(
                (-1e-12..=1.0 + 1e-12).contains(&v),
                "out of data range at x={x}: {v}"
            );
            prev = v;
            x += 0.01;
        }
    }

    #[test]
    fn analytic_derivative_matches_finite_difference() {
        let xs = vec![0.0, 1.0, 2.5, 4.0, 5.0];
        let ys = vec![0.0, 2.0, 3.0, 3.5, 10.0];
        let f = MonotoneCubic::new(xs, ys).unwrap();
        let h = 1e-6;
        for &x in &[0.3, 0.9, 1.5, 2.5, 3.2, 4.5] {
            let fd = (f.eval(x + h) - f.eval(x - h)) / (2.0 * h);
            assert!(
                approx(f.deriv(x), fd, 1e-4),
                "deriv mismatch at x={x}: analytic {} vs fd {fd}",
                f.deriv(x)
            );
        }
    }

    #[test]
    fn works_in_f32() {
        let f = MonotoneCubic::new(vec![0.0f32, 1.0, 2.0], vec![0.0f32, 1.0, 2.0]).unwrap();
        assert!((f.eval(0.5) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn rejects_bad_grids() {
        assert!(matches!(
            MonotoneCubic::new(vec![0.0], vec![0.0]),
            Err(InterpError::TooFewKnots { got: 1 })
        ));
        assert!(matches!(
            MonotoneCubic::new(vec![0.0, 1.0], vec![0.0]),
            Err(InterpError::LengthMismatch { xs: 2, ys: 1 })
        ));
        assert!(matches!(
            MonotoneCubic::new(vec![0.0, 1.0, 1.0], vec![0.0, 1.0, 2.0]),
            Err(InterpError::NotIncreasing { index: 1 })
        ));
    }
}
