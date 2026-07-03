// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-raceline` — minimum-curvature racing line generator (§6.3, Locked Decision #14).
//!
//! Minimises `∫κ²` over the lateral offset `n(s)` within the track bounds, on the 3D ribbon. With
//! the offset path `r_i = c_i + n_i·l̂_i` (centerline point plus offset along the road-plane lateral)
//! and the discrete curvature `κ⃗_i ≈ (r_{i-1} − 2r_i + r_{i+1})/Δs²`, the cost is `‖A·n + b‖²` — a
//! convex QP with box bounds `n_min ≤ n ≤ n_max`, solved with clarabel. The resulting line is
//! returned as a first-class [`Track`] (via [`outlap_track::offset_track`]) so downstream tiers
//! query its own `κ(s)` through the identical API.
//!
//! Re-implemented from the published formulation (F. Braghin et al., *Race driver model*, Computers
//! & Structures 86, 2008; A. Heilmeier et al., *Vehicle System Dynamics* 58(10), 2020 §3.1–3.2) —
//! never from the LGPL TUM source.
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::doc_markdown,
    clippy::result_large_err,
    clippy::field_reassign_with_default,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]

use clarabel::algebra::CscMatrix;
use clarabel::solver::{DefaultSettings, DefaultSolver, IPSolver, SolverStatus, SupportedConeT};
use outlap_track::{offset_track, Track, TrackError};

mod csv;
pub use csv::{read_raceline_csv, write_raceline_csv, RacelineCsvError};

/// The minimum number of stations for a well-posed QP.
const MIN_STATIONS: usize = 8;

/// Options for the min-curvature generator.
#[derive(Clone, Debug)]
pub struct RacelineOptions {
    /// Arc-length sampling step, metres.
    pub ds_m: f64,
    /// Safety margin added to the car half-width when computing the corridor bounds, metres.
    pub margin_m: f64,
    /// Tikhonov regularisation (relative to the max diagonal) for a unique solution on straights.
    pub epsilon: f64,
}

impl Default for RacelineOptions {
    fn default() -> Self {
        Self {
            ds_m: 2.0,
            margin_m: 0.3,
            epsilon: 1e-8,
        }
    }
}

/// A generated racing line: the offsets and the drivable line as a [`Track`].
#[derive(Clone, Debug)]
pub struct Raceline {
    /// Parent-centerline arc-length stations, metres.
    pub s: Vec<f64>,
    /// Signed lateral offset at each station (`+` road-left), metres.
    pub n: Vec<f64>,
    /// The line as a first-class track (its own κ(s), grade, road frame).
    pub line: Track,
}

/// An error generating a racing line.
#[derive(Debug, thiserror::Error)]
pub enum RacelineError {
    /// The track has too few stations for a stable QP.
    #[error("track sampled to {got} stations; need at least {min}")]
    TooFewStations {
        /// Stations sampled.
        got: usize,
        /// Minimum required.
        min: usize,
    },
    /// The QP solver did not converge.
    #[error("min-curvature QP did not solve (status: {0:?})")]
    QpFailed(SolverStatus),
    /// Building the offset line as a track failed.
    #[error(transparent)]
    Track(#[from] TrackError),
}

/// Generate the minimum-curvature line for `track`, keeping the car (half-width `half_width_m`,
/// which the caller derives from `chassis.track_m` plus a margin) inside the track.
pub fn min_curvature_line(
    track: &Track,
    half_width_m: f64,
    opts: &RacelineOptions,
) -> Result<Raceline, RacelineError> {
    let length = track.length();
    let closed = track.is_closed();
    let n_seg = ((length / opts.ds_m).round() as usize).max(MIN_STATIONS);
    let ds = length / n_seg as f64;
    let n = if closed { n_seg } else { n_seg + 1 };
    if n < MIN_STATIONS {
        return Err(RacelineError::TooFewStations {
            got: n,
            min: MIN_STATIONS,
        });
    }

    // Sample the centerline: signed plan-view curvature κ_r and the corridor bounds.
    let mut s = Vec::with_capacity(n);
    let mut kappa = Vec::with_capacity(n);
    let mut n_lo = Vec::with_capacity(n);
    let mut n_hi = Vec::with_capacity(n);
    for i in 0..n {
        let si = i as f64 * ds;
        let (wl, wr) = track.width(si);
        let hi = wl - half_width_m;
        let lo = -(wr - half_width_m);
        // If the corridor collapses (car wider than track), pin to the centerline.
        let (lo, hi) = if hi < lo { (0.0, 0.0) } else { (lo, hi) };
        s.push(si);
        kappa.push(track.curvature_h(si));
        n_lo.push(lo);
        n_hi.push(hi);
    }

    let n_opt = solve_qp(&kappa, ds, closed, &n_lo, &n_hi, opts.epsilon)?;
    let line = offset_track(track, &s, &n_opt, "min-curvature line")?;
    Ok(Raceline { s, n: n_opt, line })
}

/// Neighbour index `k + delta`, wrapping for closed, `None` past an open end.
fn neighbor(k: usize, delta: isize, n: usize, closed: bool) -> Option<usize> {
    let j = k as isize + delta;
    if closed {
        Some(j.rem_euclid(n as isize) as usize)
    } else if j < 0 || j >= n as isize {
        None
    } else {
        Some(j as usize)
    }
}

/// Assemble and solve the box-constrained min-curvature QP, returning the optimal offsets.
///
/// Scalar linearisation (Heilmeier et al. 2020 §3.1): the offset-path curvature is
/// `κ_new,i = κ_r,i + (n_{i-1} − 2n_i + n_{i+1})/Δs² + κ_r,i²·n_i`, i.e. `M·n + κ_r` with `M`
/// tridiagonal (`M_{i,i} = −2/Δs² + κ_r,i²`, `M_{i,i±1} = 1/Δs²`). Minimising `‖M·n + κ_r‖²` gives
/// `P = 2·MᵀM` (pentadiagonal + wrap) and `q = 2·Mᵀκ_r`. The `κ_r²·n` metric term is what makes an
/// inward offset correctly *increase* curvature.
fn solve_qp(
    kappa: &[f64],
    ds: f64,
    closed: bool,
    n_lo: &[f64],
    n_hi: &[f64],
    epsilon: f64,
) -> Result<Vec<f64>, RacelineError> {
    let n = kappa.len();
    let ds2 = ds * ds;
    let ds4 = ds2 * ds2;
    let inv2 = 1.0 / ds2;
    // Tridiagonal M's diagonal: d_i = −2/Δs² + κ_i².
    let d: Vec<f64> = (0..n).map(|i| -2.0 * inv2 + kappa[i] * kappa[i]).collect();

    // q = 2·Mᵀκ_r; (Mᵀκ_r)_i = (κ_{i-1} + κ_{i+1})/Δs² + d_i·κ_i.
    let mut q = vec![0.0; n];
    for i in 0..n {
        let km = neighbor(i, -1, n, closed).map_or(0.0, |j| kappa[j]);
        let kp = neighbor(i, 1, n, closed).map_or(0.0, |j| kappa[j]);
        q[i] = 2.0 * ((km + kp) * inv2 + d[i] * kappa[i]);
    }

    // P = 2·MᵀM (upper triangle): diag 2(2/Δs⁴ + d_i²) + ε, ±1: 2(d_i+d_j)/Δs², ±2: 2/Δs⁴.
    let max_diag = 2.0 * (2.0 / ds4 + d.iter().fold(0.0_f64, |m, &v| m.max(v * v)));
    let eps = epsilon * max_diag;
    let mut entries: Vec<(usize, usize, f64)> = Vec::with_capacity(3 * n);
    for i in 0..n {
        entries.push((i, i, 2.0 * (2.0 / ds4 + d[i] * d[i]) + eps));
        if let Some(j) = neighbor(i, 1, n, closed) {
            if j != i {
                entries.push((i.min(j), i.max(j), 2.0 * (d[i] + d[j]) * inv2));
            }
        }
        if let Some(j) = neighbor(i, 2, n, closed) {
            if j != i {
                entries.push((i.min(j), i.max(j), 2.0 / ds4));
            }
        }
    }
    let p = upper_csc(&mut entries, n);

    // Box constraints [I; -I]·n ≤ [n_hi; -n_lo], NonnegativeCone(2n).
    let a = box_csc(n);
    let mut rhs = n_hi.to_vec();
    rhs.extend(n_lo.iter().map(|v| -v));
    let cones = [SupportedConeT::NonnegativeConeT(2 * n)];

    let mut settings = DefaultSettings::default();
    settings.verbose = false;
    settings.max_iter = 200;
    let mut solver = DefaultSolver::new(&p, &q, &a, &rhs, &cones, settings)
        .map_err(|_| RacelineError::QpFailed(SolverStatus::Unsolved))?;
    solver.solve();
    match solver.solution.status {
        SolverStatus::Solved | SolverStatus::AlmostSolved => Ok(solver.solution.x.clone()),
        other => Err(RacelineError::QpFailed(other)),
    }
}

/// Build a symmetric matrix's upper-triangular CSC from `(row ≤ col, val)` entries.
fn upper_csc(entries: &mut [(usize, usize, f64)], n: usize) -> CscMatrix {
    entries.sort_by_key(|&(r, c, _)| (c, r));
    let mut colptr = vec![0usize; n + 1];
    let mut rowval = Vec::with_capacity(entries.len());
    let mut nzval = Vec::with_capacity(entries.len());
    for &(r, col, v) in entries.iter() {
        rowval.push(r);
        nzval.push(v);
        colptr[col + 1] += 1;
    }
    for c in 0..n {
        colptr[c + 1] += colptr[c];
    }
    CscMatrix::new(n, n, colptr, rowval, nzval)
}

/// The `[I; -I]` constraint matrix (2n × n) in CSC.
fn box_csc(n: usize) -> CscMatrix {
    let mut colptr = vec![0usize; n + 1];
    let mut rowval = Vec::with_capacity(2 * n);
    let mut nzval = Vec::with_capacity(2 * n);
    for j in 0..n {
        rowval.push(j);
        nzval.push(1.0);
        rowval.push(n + j);
        nzval.push(-1.0);
        colptr[j + 1] = colptr[j] + 2;
    }
    CscMatrix::new(2 * n, n, colptr, rowval, nzval)
}
