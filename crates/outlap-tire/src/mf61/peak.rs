// SPDX-License-Identifier: AGPL-3.0-only
//! Peak-friction extraction from the pure-slip curves (T0 assembly, HANDOFF §6.1).
//!
//! `μ_x = max_κ |Fx(κ)| / Fz` and `μ_y = max_α |Fy(α)| / Fz` at a fixed operating point
//! (γ = 0, `V_cx = LONGVL`, caller-chosen `Fz` and pressure), scanning **both** slip signs —
//! real tires are asymmetric through their shift terms, and the maximum of the two branches is
//! the documented choice for a symmetric point-mass envelope.
//!
//! A dense grid scan followed by golden-section refinement is used instead of the closed-form
//! peak `D/Fz`: the scan is robust to `E`-clamp edge cases and to shifted curves where the peak
//! is not at the origin. It reports the maximum over the *physical* slip window (κ ∈ [−1, 1],
//! α ∈ [−0.5, 0.5]) — which is exactly what a point-mass friction envelope wants, since grip
//! only reached at unbounded slip is unusable. For a soft `C ≤ 1` curve (monotone, supremum
//! `D·sin(Cπ/2)` approached only as slip → ∞) this window maximum is below that analytic
//! asymptote by construction; that is the intended envelope value, not the asymptote. Cold path:
//! runs once per tire at assembly; still allocation-free.

use num_traits::Float;

use super::Mf61;
use crate::slip::SlipState;

/// Grid points of the coarse scan.
const GRID: usize = 256;
/// Golden-section refinement iterations (interval shrinks by ~0.618 each).
const REFINE: usize = 48;

/// Inverse golden ratio `(√5 − 1)/2`.
fn inv_phi<T: Float>() -> T {
    T::from(0.618_033_988_749_894_9).unwrap_or_else(T::zero)
}

/// Maximize `f` on `[a, b]`: coarse grid, then golden-section around the best cell.
fn scan_max<T: Float>(f: impl Fn(T) -> T, a: T, b: T) -> T {
    let n = T::from(GRID).unwrap_or_else(T::one);
    let step = (b - a) / n;

    let mut best_x = a;
    let mut best_f = f(a);
    let mut x = a;
    for _ in 0..=GRID {
        let fx = f(x);
        if fx > best_f {
            best_f = fx;
            best_x = x;
        }
        x = x + step;
    }

    // Golden-section refine on the bracketing cells.
    let mut lo = (best_x - step).max(a);
    let mut hi = (best_x + step).min(b);
    let ip = inv_phi::<T>();
    let mut c = hi - ip * (hi - lo);
    let mut d = lo + ip * (hi - lo);
    let mut fc = f(c);
    let mut fd = f(d);
    for _ in 0..REFINE {
        if fc > fd {
            hi = d;
            d = c;
            fd = fc;
            c = hi - ip * (hi - lo);
            fc = f(c);
        } else {
            lo = c;
            c = d;
            fc = fd;
            d = lo + ip * (hi - lo);
            fd = f(d);
        }
    }
    let mid = (lo + hi) / (T::one() + T::one());
    f(mid).max(best_f)
}

/// Peak longitudinal friction `μ_x` at load `fz` (N) and inflation pressure `p` (Pa).
///
/// Returns 0 for `fz ≤ 0`.
pub fn peak_mu_x<T: Float>(model: &Mf61<T>, fz: T, p: T) -> T {
    if fz <= T::zero() {
        return T::zero();
    }
    let vx = model.params().longvl;
    let f = |kappa: T| {
        model
            .forces(&SlipState::new(kappa, T::zero(), T::zero(), fz, p, vx))
            .fx
            .abs()
    };
    scan_max(f, -T::one(), T::one()) / fz
}

/// Peak lateral friction `μ_y` at load `fz` (N) and inflation pressure `p` (Pa).
///
/// Returns 0 for `fz ≤ 0`. The α scan range is ±0.5 rad (≈ ±28.6°), comfortably past any
/// physical lateral-force peak.
pub fn peak_mu_y<T: Float>(model: &Mf61<T>, fz: T, p: T) -> T {
    if fz <= T::zero() {
        return T::zero();
    }
    let vx = model.params().longvl;
    let half = T::from(0.5).unwrap_or_else(T::one);
    let f = |alpha: T| {
        model
            .forces(&SlipState::new(T::zero(), alpha, T::zero(), fz, p, vx))
            .fy
            .abs()
    };
    scan_max(f, -half, half) / fz
}
