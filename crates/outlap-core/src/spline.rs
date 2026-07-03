// SPDX-License-Identifier: AGPL-3.0-only
//! C² cubic splines for parametric geometry (the track ribbon; Decision #13).
//!
//! Unlike the shape-preserving [`interp`](crate::interp) map interpolant (C¹, Decision #30), a
//! *geometry* channel `x(t)`, `y(t)`, `z(t)` must be **C²** so that curvature — which depends on
//! the second derivative — is continuous. This module fits the classic moment-form cubic spline:
//!
//! * [`CubicSpline::not_a_knot`] / [`CubicSpline::natural`] for open (point-to-point) parameters;
//! * [`CubicSpline::periodic`] for closed loops, where value, slope, and curvature all match across
//!   the wrap (a cyclic-tridiagonal moment system solved with Sherman–Morrison).
//!
//! # Symbols
//!
//! `M_k = S''(x_k)` are the *moments*; `h_k = x_{k+1} − x_k` the knot spacings. On each interval the
//! spline is `S(x) = y_k + b_k ξ + (M_k/2) ξ² + ((M_{k+1}−M_k)/6h_k) ξ³`, `ξ = x − x_k`.

use num_traits::Float;

/// Error building a [`CubicSpline`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SplineError {
    /// Not enough knots for the requested end condition.
    #[error("need at least {need} knots, got {got}")]
    TooFewKnots {
        /// Minimum knots required.
        need: usize,
        /// Knots supplied.
        got: usize,
    },
    /// `xs` and `ys` had different lengths.
    #[error("xs has {xs} knots but ys has {ys}")]
    LengthMismatch {
        /// Length of the parameter grid.
        xs: usize,
        /// Length of the values.
        ys: usize,
    },
    /// The parameter grid was not strictly increasing at the given index.
    #[error("xs must be strictly increasing; xs[{index}] >= xs[{}]", index + 1)]
    NotIncreasing {
        /// Index `k` where `xs[k] >= xs[k+1]`.
        index: usize,
    },
    /// A periodic fit was requested but the period does not exceed the knot span.
    #[error("period {period} must exceed the knot span (last knot is beyond it)")]
    BadPeriod {
        /// The offending period.
        period: f64,
    },
}

/// A cubic spline over a strictly increasing parameter grid, evaluable with continuous value,
/// first, and second derivatives.
#[derive(Clone, Debug)]
pub struct CubicSpline<T> {
    xs: Vec<T>,
    ys: Vec<T>,
    /// Moments `M_k = S''(x_k)`, one per knot (`m[n-1] == m[0]` for a periodic fit).
    m: Vec<T>,
    /// For a periodic spline, the wrap period; `None` for open splines.
    period: Option<T>,
    /// Closing-interval length `h_{n-1}` for a periodic spline (`period − (x_last − x_first)`).
    close_h: T,
}

impl<T: Float> CubicSpline<T> {
    /// Fit a not-a-knot cubic spline (the default for open parameters — no artificial end
    /// curvature). Needs at least 3 knots.
    ///
    /// # Errors
    /// [`SplineError`] on grid/length problems or too few knots.
    pub fn not_a_knot(xs: Vec<T>, ys: Vec<T>) -> Result<Self, SplineError> {
        validate_grid(&xs, &ys, 3)?;
        let m = solve_open(&xs, &ys, EndCondition::NotAKnot);
        Ok(Self {
            xs,
            ys,
            m,
            period: None,
            close_h: T::zero(),
        })
    }

    /// Fit a natural cubic spline (`S'' = 0` at both ends). Needs at least 2 knots.
    ///
    /// # Errors
    /// [`SplineError`] on grid/length problems or too few knots.
    pub fn natural(xs: Vec<T>, ys: Vec<T>) -> Result<Self, SplineError> {
        validate_grid(&xs, &ys, 2)?;
        let m = solve_open(&xs, &ys, EndCondition::Natural);
        Ok(Self {
            xs,
            ys,
            m,
            period: None,
            close_h: T::zero(),
        })
    }

    /// Fit a periodic cubic spline over knots `xs` (all strictly inside one `period`), with the
    /// curve wrapping `x_0 + period ≡ x_0`. Value, slope, and curvature are continuous across the
    /// seam. The knots are the *distinct* samples around the loop (do not repeat the first point).
    ///
    /// # Errors
    /// [`SplineError`] on grid/length problems, fewer than 3 knots, or a period not exceeding the
    /// knot span.
    pub fn periodic(xs: Vec<T>, ys: Vec<T>, period: T) -> Result<Self, SplineError> {
        validate_grid(&xs, &ys, 3)?;
        let span = xs[xs.len() - 1] - xs[0];
        if period <= span {
            return Err(SplineError::BadPeriod {
                period: period.to_f64().unwrap_or(f64::NAN),
            });
        }
        let close_h = period - span; // h_{n-1}: last knot → wrapped first knot
        let m = solve_periodic(&xs, &ys, close_h);
        Ok(Self {
            xs,
            ys,
            m,
            period: Some(period),
            close_h,
        })
    }

    /// Whether this spline wraps periodically.
    pub fn is_periodic(&self) -> bool {
        self.period.is_some()
    }

    /// Value `S(x)`.
    pub fn eval(&self, x: T) -> T {
        let (k, xi, h) = self.locate(x);
        let (mk, mk1) = (self.m[k], self.moment_next(k));
        let yk = self.ys[k];
        let yk1 = self.value_next(k);
        let two = T::from(2).unwrap();
        let six = T::from(6).unwrap();
        let bk = (yk1 - yk) / h - h * (two * mk + mk1) / six;
        yk + bk * xi + (mk / two) * xi * xi + ((mk1 - mk) / (six * h)) * xi * xi * xi
    }

    /// First derivative `S'(x)`.
    pub fn deriv(&self, x: T) -> T {
        let (k, xi, h) = self.locate(x);
        let (mk, mk1) = (self.m[k], self.moment_next(k));
        let yk = self.ys[k];
        let yk1 = self.value_next(k);
        let two = T::from(2).unwrap();
        let six = T::from(6).unwrap();
        let bk = (yk1 - yk) / h - h * (two * mk + mk1) / six;
        bk + mk * xi + ((mk1 - mk) / (two * h)) * xi * xi
    }

    /// Second derivative `S''(x)` (continuous — this is why geometry uses this spline, not the
    /// C¹ Hermite).
    pub fn deriv2(&self, x: T) -> T {
        let (k, xi, h) = self.locate(x);
        let (mk, mk1) = (self.m[k], self.moment_next(k));
        mk + ((mk1 - mk) / h) * xi
    }

    /// Moment at the knot following `k` (wraps for a periodic spline).
    fn moment_next(&self, k: usize) -> T {
        if k + 1 < self.m.len() {
            self.m[k + 1]
        } else {
            self.m[0]
        }
    }

    /// Value at the knot following `k` (wraps for a periodic spline).
    fn value_next(&self, k: usize) -> T {
        if k + 1 < self.ys.len() {
            self.ys[k + 1]
        } else {
            self.ys[0]
        }
    }

    /// Locate the interval containing `x`, returning `(k, ξ, h)`. For a periodic spline `x` is
    /// reduced modulo the period; for an open spline it clamps to the end intervals.
    fn locate(&self, x: T) -> (usize, T, T) {
        let n = self.xs.len();
        if let Some(period) = self.period {
            // Reduce x into [x_0, x_0 + period).
            let x0 = self.xs[0];
            let mut u = (x - x0) % period;
            if u < T::zero() {
                u = u + period;
            }
            let xr = x0 + u;
            // Closing interval [x_{n-1}, x_0 + period).
            if xr >= self.xs[n - 1] {
                return (n - 1, xr - self.xs[n - 1], self.close_h);
            }
            let k = self.xs.partition_point(|&xk| xk <= xr) - 1;
            return (k, xr - self.xs[k], self.xs[k + 1] - self.xs[k]);
        }
        // Open: clamp.
        if x <= self.xs[0] {
            return (0, x - self.xs[0], self.xs[1] - self.xs[0]);
        }
        if x >= self.xs[n - 1] {
            let h = self.xs[n - 1] - self.xs[n - 2];
            return (n - 2, x - self.xs[n - 2], h);
        }
        let k = self.xs.partition_point(|&xk| xk <= x) - 1;
        (k, x - self.xs[k], self.xs[k + 1] - self.xs[k])
    }
}

/// End condition for an open cubic spline.
#[derive(Clone, Copy)]
enum EndCondition {
    Natural,
    NotAKnot,
}

fn validate_grid<T: Float>(xs: &[T], ys: &[T], need: usize) -> Result<(), SplineError> {
    if xs.len() != ys.len() {
        return Err(SplineError::LengthMismatch {
            xs: xs.len(),
            ys: ys.len(),
        });
    }
    if xs.len() < need {
        return Err(SplineError::TooFewKnots {
            need,
            got: xs.len(),
        });
    }
    for k in 0..xs.len() - 1 {
        if xs[k + 1] <= xs[k] {
            return Err(SplineError::NotIncreasing { index: k });
        }
    }
    Ok(())
}

/// Solve for the moments of an open spline with the given end condition.
fn solve_open<T: Float>(xs: &[T], ys: &[T], end: EndCondition) -> Vec<T> {
    match end {
        EndCondition::NotAKnot => return solve_not_a_knot(xs, ys),
        EndCondition::Natural => {}
    }
    let n = xs.len();
    let two = T::from(2).unwrap();
    let six = T::from(6).unwrap();
    let h: Vec<T> = (0..n - 1).map(|k| xs[k + 1] - xs[k]).collect();
    let slope = |k: usize| (ys[k + 1] - ys[k]) / h[k];

    // Tridiagonal system A·M = rhs (size n); natural ends pin M_0 = M_{n-1} = 0.
    let mut lower = vec![T::zero(); n];
    let mut diag = vec![T::zero(); n];
    let mut upper = vec![T::zero(); n];
    let mut rhs = vec![T::zero(); n];
    for k in 1..n - 1 {
        lower[k] = h[k - 1];
        diag[k] = two * (h[k - 1] + h[k]);
        upper[k] = h[k];
        rhs[k] = six * (slope(k) - slope(k - 1));
    }
    diag[0] = T::one();
    diag[n - 1] = T::one();
    thomas(&lower, &diag, &upper, &rhs)
}

/// Not-a-knot spline moments via the standard tridiagonal system (Boor). Kept separate because its
/// first/last rows couple three moments; we eliminate the ghost column analytically.
fn solve_not_a_knot<T: Float>(xs: &[T], ys: &[T]) -> Vec<T> {
    let n = xs.len();
    // With only three knots the not-a-knot end substitutions become mutually circular (both ends
    // reference the single interior moment); a natural fit is the sensible C² fallback there.
    if n < 4 {
        return solve_open(xs, ys, EndCondition::Natural);
    }
    let two = T::from(2).unwrap();
    let six = T::from(6).unwrap();
    let h: Vec<T> = (0..n - 1).map(|k| xs[k + 1] - xs[k]).collect();
    let slope = |k: usize| (ys[k + 1] - ys[k]) / h[k];

    let mut lower = vec![T::zero(); n];
    let mut diag = vec![T::zero(); n];
    let mut upper = vec![T::zero(); n];
    let mut rhs = vec![T::zero(); n];

    for k in 1..n - 1 {
        lower[k] = h[k - 1];
        diag[k] = two * (h[k - 1] + h[k]);
        upper[k] = h[k];
        rhs[k] = six * (slope(k) - slope(k - 1));
    }

    // Not-a-knot end rows eliminate M_0 via M_0 = M_1 + (h_0/h_1)(M_1 − M_2), substituted into the
    // k=1 equation; symmetric at the far end. This yields modified first/last active rows.
    // Row 1 after substitution:
    let c0 = h[0] / h[1];
    diag[1] = diag[1] + h[0] * (T::one() + c0);
    upper[1] = upper[1] - h[0] * c0;
    // Row n-2 after substitution:
    let cn = h[n - 2] / h[n - 3];
    diag[n - 2] = diag[n - 2] + h[n - 2] * (T::one() + cn);
    lower[n - 2] = lower[n - 2] - h[n - 2] * cn;

    // Solve the interior system (indices 1..n-2) with Thomas, then recover M_0, M_{n-1}.
    let interior_lower = lower[1..n - 1].to_vec();
    let interior_diag = diag[1..n - 1].to_vec();
    let interior_upper = upper[1..n - 1].to_vec();
    let interior_rhs = rhs[1..n - 1].to_vec();
    let interior = thomas(
        &interior_lower,
        &interior_diag,
        &interior_upper,
        &interior_rhs,
    );

    let mut m = vec![T::zero(); n];
    m[1..n - 1].copy_from_slice(&interior);
    // M_0 = M_1 + c0 (M_1 − M_2); M_{n-1} = M_{n-2} + cn (M_{n-2} − M_{n-3}).
    m[0] = m[1] + c0 * (m[1] - m[2]);
    m[n - 1] = m[n - 2] + cn * (m[n - 2] - m[n - 3]);
    m
}

/// Solve for the moments of a periodic spline. `close_h` is the closing interval length
/// `h_{n-1} = x_0 + period − x_{n-1}`.
fn solve_periodic<T: Float>(xs: &[T], ys: &[T], close_h: T) -> Vec<T> {
    let n = xs.len();
    let two = T::from(2).unwrap();
    let six = T::from(6).unwrap();
    // Interval lengths, with the closing interval appended as h[n-1].
    let mut h: Vec<T> = (0..n - 1).map(|k| xs[k + 1] - xs[k]).collect();
    h.push(close_h);
    // Periodic slope: value after knot k, wrapping.
    let slope = |k: usize| {
        let yk1 = if k + 1 < n { ys[k + 1] } else { ys[0] };
        (yk1 - ys[k]) / h[k]
    };

    // Cyclic tridiagonal system A·M = rhs, size n, indices mod n.
    let mut lower = vec![T::zero(); n]; // sub-diagonal a_k (coeff of M_{k-1})
    let mut diag = vec![T::zero(); n];
    let mut upper = vec![T::zero(); n]; // super-diagonal c_k (coeff of M_{k+1})
    let mut rhs = vec![T::zero(); n];
    for k in 0..n {
        let hkm1 = h[(k + n - 1) % n];
        let hk = h[k];
        lower[k] = hkm1;
        diag[k] = two * (hkm1 + hk);
        upper[k] = hk;
        let sk = slope(k);
        let skm1 = slope((k + n - 1) % n);
        rhs[k] = six * (sk - skm1);
    }
    // Corner terms: a_0 (couples M_{-1}=M_{n-1}) and c_{n-1} (couples M_n=M_0).
    let alpha = upper[n - 1]; // coeff on M_0 in the last equation
    let beta = lower[0]; // coeff on M_{n-1} in the first equation
    sherman_morrison_cyclic(&lower, &diag, &upper, &rhs, alpha, beta)
}

/// Thomas algorithm for a tridiagonal system (`lower[0]` and `upper[n-1]` are ignored).
fn thomas<T: Float>(lower: &[T], diag: &[T], upper: &[T], rhs: &[T]) -> Vec<T> {
    let n = diag.len();
    let mut c = vec![T::zero(); n];
    let mut d = vec![T::zero(); n];
    c[0] = upper[0] / diag[0];
    d[0] = rhs[0] / diag[0];
    for i in 1..n {
        let denom = diag[i] - lower[i] * c[i - 1];
        c[i] = upper[i] / denom;
        d[i] = (rhs[i] - lower[i] * d[i - 1]) / denom;
    }
    let mut x = vec![T::zero(); n];
    x[n - 1] = d[n - 1];
    for i in (0..n - 1).rev() {
        x[i] = d[i] - c[i] * x[i + 1];
    }
    x
}

/// Solve a cyclic tridiagonal system (tridiagonal plus the two corner couplings `alpha`, `beta`)
/// via the Sherman–Morrison formula (Numerical Recipes §2.7, `cyclic`).
fn sherman_morrison_cyclic<T: Float>(
    lower: &[T],
    diag: &[T],
    upper: &[T],
    rhs: &[T],
    alpha: T, // A[n-1][0]
    beta: T,  // A[0][n-1]
) -> Vec<T> {
    let n = diag.len();
    // Choose gamma = −diag[0]; perturb the corners away and correct with Sherman–Morrison.
    let gamma = -diag[0];
    let mut d = diag.to_vec();
    d[0] = diag[0] - gamma;
    d[n - 1] = diag[n - 1] - alpha * beta / gamma;

    // Solve A'·y = rhs.
    let y = thomas(lower, &d, upper, rhs);
    // Solve A'·z = u, where u = (gamma, 0, …, 0, alpha).
    let mut u = vec![T::zero(); n];
    u[0] = gamma;
    u[n - 1] = alpha;
    let z = thomas(lower, &d, upper, &u);

    // fact = (v·y) / (1 + v·z), with v = (1, 0, …, 0, beta/gamma).
    let vg = beta / gamma;
    let fact = (y[0] + vg * y[n - 1]) / (T::one() + z[0] + vg * z[n - 1]);
    (0..n).map(|i| y[i] - fact * z[i]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    fn max_err(f: impl Fn(f64) -> f64, s: &CubicSpline<f64>, a: f64, b: f64) -> f64 {
        let mut e: f64 = 0.0;
        let mut x = a;
        while x <= b {
            e = e.max((s.eval(x) - f(x)).abs());
            x += (b - a) / 500.0;
        }
        e
    }

    #[test]
    fn not_a_knot_reproduces_cubic_exactly() {
        // A cubic polynomial is reproduced to round-off by a not-a-knot spline.
        let f = |x: f64| 2.0 - x + 0.5 * x * x - 0.3 * x * x * x;
        let xs: Vec<f64> = (0..8).map(|i| f64::from(i) * 0.7).collect();
        let ys: Vec<f64> = xs.iter().map(|&x| f(x)).collect();
        let s = CubicSpline::not_a_knot(xs.clone(), ys).unwrap();
        assert!(max_err(f, &s, xs[0], xs[xs.len() - 1]) < 1e-9);
    }

    #[test]
    fn second_derivative_is_continuous() {
        let f = |x: f64| (x * 0.8).sin();
        let xs: Vec<f64> = (0..12).map(f64::from).collect();
        let ys: Vec<f64> = xs.iter().map(|&x| f(x)).collect();
        let s = CubicSpline::not_a_knot(xs, ys).unwrap();
        // Sample S'' just left and right of an interior knot; they must agree.
        let k = 5.0;
        let left = s.deriv2(k - 1e-7);
        let right = s.deriv2(k + 1e-7);
        assert!(
            (left - right).abs() < 1e-4,
            "S'' jumps at knot: {left} vs {right}"
        );
    }

    #[test]
    fn periodic_matches_across_seam() {
        // Points sampled from cos over one full period; the periodic spline must be C² at the wrap.
        let n = 16;
        let period = 2.0 * PI;
        let xs: Vec<f64> = (0..n)
            .map(|i| period * f64::from(i) / f64::from(n))
            .collect();
        let ys: Vec<f64> = xs.iter().map(|&x| x.cos()).collect();
        let s = CubicSpline::periodic(xs, ys, period).unwrap();
        // Value, slope, curvature continuous across x = 0 ≡ period.
        let eps = 1e-6;
        for (lo, hi) in [
            (s.eval(period - eps), s.eval(eps)),
            (s.deriv(period - eps), s.deriv(eps)),
            (s.deriv2(period - eps), s.deriv2(eps)),
        ] {
            assert!((lo - hi).abs() < 1e-3, "seam discontinuity: {lo} vs {hi}");
        }
        // And it should approximate cos well.
        assert!(max_err(f64::cos, &s, 0.0, period) < 1e-3);
    }

    #[test]
    fn periodic_wraps_query() {
        let period = 10.0;
        let xs = vec![0.0, 2.5, 5.0, 7.5];
        let ys = vec![1.0, 0.0, -1.0, 0.0];
        let s = CubicSpline::periodic(xs, ys, period).unwrap();
        // Querying past the period wraps back.
        assert!((s.eval(0.3) - s.eval(period + 0.3)).abs() < 1e-9);
        assert!((s.eval(-0.4) - s.eval(period - 0.4)).abs() < 1e-9);
    }

    #[test]
    fn natural_has_zero_end_curvature() {
        let xs = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let ys = vec![0.0, 1.0, 0.0, 1.0, 0.0];
        let s = CubicSpline::natural(xs, ys).unwrap();
        assert!(s.deriv2(0.0).abs() < 1e-9);
        assert!(s.deriv2(4.0).abs() < 1e-9);
    }
}
