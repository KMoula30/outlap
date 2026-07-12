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
    clippy::cast_sign_loss,
    clippy::too_many_lines
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
    /// A time-weight profile of the wrong length was supplied.
    #[error("weights has {got} entries; the QP samples {want} stations at this ds")]
    WeightLenMismatch {
        /// Weights supplied.
        got: usize,
        /// Stations the QP samples (see `raceline_stations`).
        want: usize,
    },
    /// Building the offset line as a track failed.
    #[error(transparent)]
    Track(#[from] TrackError),
}

/// The station count and step the QP samples `track` at for the given `ds_m`.
///
/// Deterministic in `(length, closed, ds_m)` — [`raceline_stations`] and both generators agree, so a
/// caller can precompute a per-station weight profile (e.g. Δt from a speed pre-pass) that lines up
/// exactly with [`min_curvature_line_weighted`].
fn station_grid(track: &Track, ds_m: f64) -> (usize, f64, bool) {
    let length = track.length();
    let closed = track.is_closed();
    let n_seg = ((length / ds_m).round() as usize).max(MIN_STATIONS);
    let ds = length / n_seg as f64;
    let n = if closed { n_seg } else { n_seg + 1 };
    (n, ds, closed)
}

/// The centerline arc-length stations (metres) the QP will sample for `ds_m`.
///
/// Use this to sample a per-station weight profile (`weights[i]` at station `stations[i]`) before
/// calling [`min_curvature_line_weighted`]; the indices align by construction.
pub fn raceline_stations(track: &Track, ds_m: f64) -> Vec<f64> {
    let (n, ds, _) = station_grid(track, ds_m);
    (0..n).map(|i| i as f64 * ds).collect()
}

/// Sample the centerline into `(s, κ_r, n_lo, n_hi)` over the QP grid.
fn sample_corridor(
    track: &Track,
    half_width_m: f64,
    ds: f64,
    n: usize,
) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
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
    (s, kappa, n_lo, n_hi)
}

/// Generate the minimum-curvature line for `track`, keeping the car (half-width `half_width_m`,
/// which the caller derives from `chassis.track_m` plus a margin) inside the track.
pub fn min_curvature_line(
    track: &Track,
    half_width_m: f64,
    opts: &RacelineOptions,
) -> Result<Raceline, RacelineError> {
    let (n, ds, closed) = station_grid(track, opts.ds_m);
    if n < MIN_STATIONS {
        return Err(RacelineError::TooFewStations {
            got: n,
            min: MIN_STATIONS,
        });
    }
    let (s, kappa, n_lo, n_hi) = sample_corridor(track, half_width_m, ds, n);
    let n_opt = solve_qp(&kappa, ds, closed, &n_lo, &n_hi, opts.epsilon, None)?;
    let line = offset_track(track, &s, &n_opt, "min-curvature line")?;
    Ok(Raceline { s, n: n_opt, line })
}

/// Generate a **time-weighted** minimum-curvature line: the same QP with a per-station weight
/// `w_i ≥ 0` on the curvature cost, minimising `Σ w_i·κ_new,i²` instead of the flat `Σ κ_new,i²`
/// (Locked Decision #10). Feeding `w_i = Δt_i = Δs_i / v_i` from a speed pre-pass spends the
/// curvature budget where the car is slow (corners), trading a little straight-line curvature for a
/// faster driven line. `weights.len()` must equal [`raceline_stations`]`(track, opts.ds_m).len()`.
///
/// The outer reweight loop (re-run the speed pre-pass on the new line, recompute `w`, repeat) lives
/// at the orchestration layer that owns the speed model — this crate stays wasm-clean and does the
/// one weighted solve.
pub fn min_curvature_line_weighted(
    track: &Track,
    half_width_m: f64,
    weights: &[f64],
    opts: &RacelineOptions,
) -> Result<Raceline, RacelineError> {
    let (n, ds, closed) = station_grid(track, opts.ds_m);
    if n < MIN_STATIONS {
        return Err(RacelineError::TooFewStations {
            got: n,
            min: MIN_STATIONS,
        });
    }
    if weights.len() != n {
        return Err(RacelineError::WeightLenMismatch {
            got: weights.len(),
            want: n,
        });
    }
    let (s, kappa, n_lo, n_hi) = sample_corridor(track, half_width_m, ds, n);
    let n_opt = solve_qp(
        &kappa,
        ds,
        closed,
        &n_lo,
        &n_hi,
        opts.epsilon,
        Some(weights),
    )?;
    let line = offset_track(track, &s, &n_opt, "time-weighted line")?;
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
/// tridiagonal (`M_{i,i} = −2/Δs² + κ_r,i²`, `M_{i,i±1} = 1/Δs²`). Minimising the weighted residual
/// `Σ w_i·κ_new,i² = (M·n + κ_r)ᵀW(M·n + κ_r)` gives `P = 2·MᵀWM` (pentadiagonal + wrap) and
/// `q = 2·MᵀWκ_r`. The `κ_r²·n` metric term is what makes an inward offset correctly *increase*
/// curvature. `weights = None` is the flat `W = I` (min-curvature); `Some(w)` is the time-weighted
/// variant (Decision #10) — the flat path is assembled bit-identically to preserve provenance.
fn solve_qp(
    kappa: &[f64],
    ds: f64,
    closed: bool,
    n_lo: &[f64],
    n_hi: &[f64],
    epsilon: f64,
    weights: Option<&[f64]>,
) -> Result<Vec<f64>, RacelineError> {
    let n = kappa.len();
    let ds2 = ds * ds;
    let ds4 = ds2 * ds2;
    let inv2 = 1.0 / ds2;
    let inv4 = 1.0 / ds4;
    // Tridiagonal M's diagonal: d_i = −2/Δs² + κ_i².
    let d: Vec<f64> = (0..n).map(|i| -2.0 * inv2 + kappa[i] * kappa[i]).collect();

    let (q, mut entries, max_offdiag);
    if let Some(w) = weights {
        // Row i of M weighted by w_i: (MᵀWκ_r)_j = Σ_i w_i M_{ij} κ_i over rows i ∈ {j-1, j, j+1}.
        let wof = |i: usize| w[i];
        let mut qw = vec![0.0; n];
        for j in 0..n {
            let mut acc = w[j] * d[j] * kappa[j];
            if let Some(im) = neighbor(j, -1, n, closed) {
                acc += wof(im) * inv2 * kappa[im]; // row j-1 hits column j via M_{j-1,j}=inv2
            }
            if let Some(ip) = neighbor(j, 1, n, closed) {
                acc += wof(ip) * inv2 * kappa[ip]; // row j+1 hits column j via M_{j+1,j}=inv2
            }
            qw[j] = 2.0 * acc;
        }
        // (MᵀWM)_{jk} = Σ_i w_i M_{ij} M_{ik}. Diagonal, +1 and +2 bands (upper triangle).
        let mut ent: Vec<(usize, usize, f64)> = Vec::with_capacity(3 * n);
        let mut peak = 0.0_f64;
        for j in 0..n {
            let mut diag = w[j] * d[j] * d[j];
            if let Some(im) = neighbor(j, -1, n, closed) {
                diag += wof(im) * inv2 * inv2; // row j-1: M_{j-1,j}²
            }
            if let Some(ip) = neighbor(j, 1, n, closed) {
                diag += wof(ip) * inv2 * inv2; // row j+1: M_{j+1,j}²
            }
            ent.push((j, j, 2.0 * diag));
            // Traverse only +1/+2 from each j so every undirected band edge (including the closed
            // wrap edge) is emitted exactly once; place it in the upper triangle by (min, max).
            if let Some(k) = neighbor(j, 1, n, closed) {
                if k != j {
                    // rows i=j (d_j·inv2) and i=k (inv2·d_k)
                    let off = w[j] * d[j] * inv2 + w[k] * inv2 * d[k];
                    ent.push((j.min(k), j.max(k), 2.0 * off));
                    peak = peak.max((2.0 * off).abs());
                }
            }
            if let Some(k) = neighbor(j, 2, n, closed) {
                if k != j {
                    // only the row between them (M·inv2 on both sides), weighted by that row's w
                    let mid = neighbor(j, 1, n, closed).expect("interior neighbor exists");
                    let off = wof(mid) * inv2 * inv2;
                    ent.push((j.min(k), j.max(k), 2.0 * off));
                    peak = peak.max((2.0 * off).abs());
                }
            }
        }
        q = qw;
        entries = ent;
        max_offdiag = peak;
    } else {
        // Flat W = I: the original assembly, byte-for-byte (min-curvature provenance is stable).
        let mut qf = vec![0.0; n];
        for i in 0..n {
            let km = neighbor(i, -1, n, closed).map_or(0.0, |j| kappa[j]);
            let kp = neighbor(i, 1, n, closed).map_or(0.0, |j| kappa[j]);
            qf[i] = 2.0 * ((km + kp) * inv2 + d[i] * kappa[i]);
        }
        let mut ent: Vec<(usize, usize, f64)> = Vec::with_capacity(3 * n);
        for i in 0..n {
            ent.push((i, i, 2.0 * (2.0 / ds4 + d[i] * d[i])));
            if let Some(j) = neighbor(i, 1, n, closed) {
                if j != i {
                    ent.push((i.min(j), i.max(j), 2.0 * (d[i] + d[j]) * inv2));
                }
            }
            if let Some(j) = neighbor(i, 2, n, closed) {
                if j != i {
                    ent.push((i.min(j), i.max(j), 2.0 / ds4));
                }
            }
        }
        q = qf;
        entries = ent;
        max_offdiag = 2.0 * inv4;
    }

    // Tikhonov ε on the diagonal for a unique solution on straights (relative to the biggest term).
    let max_diag = entries
        .iter()
        .filter(|&&(r, c, _)| r == c)
        .fold(0.0_f64, |m, &(_, _, v)| m.max(v.abs()))
        .max(max_offdiag);
    let eps = epsilon * max_diag;
    for e in &mut entries {
        if e.0 == e.1 {
            e.2 += eps;
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
