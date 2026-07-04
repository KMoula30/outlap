// SPDX-License-Identifier: AGPL-3.0-only
//! N-dimensional gridded maps: rectilinear tensor-product monotone cubic Hermite (Decision #30).
//!
//! [`GriddedMapN`] interpolates a value defined on a rectilinear N-D grid (engine/aero/battery
//! maps). It is the multivariate sibling of [`MonotoneCubic`](crate::MonotoneCubic) and is built on
//! the **same** Fritsch–Carlson tangent limiter and cubic-Hermite basis — Decision #30 mandates one
//! monotone-cubic implementation for every gridded lookup.
//!
//! # Construction
//!
//! At assembly time (never in the hot loop) the map precomputes, at every grid node, the full set of
//! mixed partial derivatives of orders `{0,1}` along each axis (`2ⁿ` values per node). These are
//! obtained by applying the 1-D tangent limiter successively along each axis in index order:
//! `∂^{|d|}/∏∂xᵢ^{dᵢ}` for the derivative pattern `d ∈ {0,1}ⁿ` is the axis-`imax` tangent field of
//! the pattern with `imax` cleared, where `imax` is the highest set bit of `d`. The tensor-product
//! cubic Hermite is then evaluated locally from the `2ⁿ` cell corners and their precomputed
//! partials.
//!
//! # Guarantees
//!
//! * **Exact node interpolation** — the interpolant reproduces the tabulated value at every node.
//! * **C¹** — node partials are single-valued, so value and gradient are continuous everywhere.
//! * **Axis-aligned monotonicity** — restricted to a grid-aligned fibre (all but one coordinate at
//!   grid nodes) the map coincides *exactly* with [`MonotoneCubic`](crate::MonotoneCubic) on that
//!   fibre and therefore inherits its shape-preserving/no-overshoot property.
//! * **Analytic gradient** — [`GriddedMapN::grad_into`] returns `∂f/∂xⱼ` without finite differences
//!   (the transient/trim Newton solvers need it — Decision #30).
//!
//! # Out-of-domain policy
//!
//! Each axis carries an [`OutOfDomain`] mode. [`OutOfDomain::Clamp`] (the default) saturates at the
//! table edge (constant value, zero slope) matching [`MonotoneCubic`]. [`OutOfDomain::Linear`]
//! extrapolates along the boundary tangent — C¹-continuous with the interior (PR6 uses this on the
//! Vdc axis so drive-unit maps stay usable below their voltage grid). Every evaluation reports
//! whether it left the grid or touched a NaN-filled cell via [`EvalFlags`].
//!
//! # NaN cells
//!
//! PDT-derived maps carry ~1.5 % NaN cells beyond the reachable torque envelope. Construction fills
//! them by nearest-valid multi-source breadth-first search over the grid (deterministic: FIFO from
//! index-ordered valid cells, fixed axis/sign neighbour order) so the interpolant is total and C¹.
//! The original NaN mask is retained; an evaluation is flagged out-of-hull
//! ([`EvalFlags::out_of_hull`]) whenever its value's *domain of dependence* — the cell corners
//! **and** the `±1` fibre neighbours the Fritsch–Carlson tangents reach — touches a filled cell, so
//! any result influenced by synthetic fill surfaces in the loaded-model report.

use std::collections::VecDeque;

use num_traits::Float;

use crate::interp::{fritsch_carlson_tangents, hermite_basis, hermite_basis_deriv};

/// Maximum number of grid axes a [`GriddedMapN`] may have.
///
/// Bounds the stack scratch used by the zero-allocation evaluator. Six covers every M3 map (a
/// Vdc-stacked drive unit is 3-D; a ride-height/yaw/DRS aero map is 4-D).
pub const MAX_DIMS: usize = 6;

/// Per-axis behaviour for queries outside the tabulated range.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum OutOfDomain {
    /// Saturate at the nearest edge value (constant beyond the grid, zero slope). The default,
    /// matching [`MonotoneCubic`](crate::MonotoneCubic).
    #[default]
    Clamp,
    /// Extrapolate linearly along the boundary tangent (C¹-continuous with the interior).
    Linear,
}

/// Error building a [`GriddedMapN`] or [`GriddedTable`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum GridMapError {
    /// No axes were supplied.
    #[error("a gridded map needs at least one axis")]
    NoAxes,
    /// More than [`MAX_DIMS`] axes were supplied.
    #[error("too many axes: {got} (max {MAX_DIMS})")]
    TooManyDims {
        /// The number of axes requested.
        got: usize,
    },
    /// An axis had fewer than two breakpoints.
    #[error("axis {axis} needs at least two breakpoints, got {got}")]
    AxisTooShort {
        /// The offending axis index.
        axis: usize,
        /// The breakpoint count supplied.
        got: usize,
    },
    /// An axis grid was not strictly increasing.
    #[error(
        "axis {axis} must be strictly increasing; breakpoint {index} is not below its successor"
    )]
    AxisNotIncreasing {
        /// The offending axis index.
        axis: usize,
        /// The breakpoint index `k` where `xs[k] >= xs[k+1]`.
        index: usize,
    },
    /// The values tensor length did not equal the product of the axis lengths.
    #[error("values tensor has {got} entries but the axes imply {expected}")]
    ValuesLen {
        /// The product of the axis lengths.
        expected: usize,
        /// The number of values supplied.
        got: usize,
    },
    /// The out-of-domain modes count did not match the number of axes.
    #[error("expected {expected} out-of-domain modes (one per axis), got {got}")]
    ModesLen {
        /// The number of axes.
        expected: usize,
        /// The number of modes supplied.
        got: usize,
    },
    /// The query point dimensionality did not match the map.
    #[error("query has {got} coordinates but the map is {expected}-dimensional")]
    QueryDim {
        /// The map dimensionality.
        expected: usize,
        /// The query length supplied.
        got: usize,
    },
    /// Every cell was NaN — there is nothing to interpolate.
    #[error("every cell is NaN; no valid data to interpolate")]
    AllNan,
    /// A long-form column referenced an axis name not present in the table.
    #[error("axis column `{name}` is not present in the table")]
    UnknownAxisColumn {
        /// The missing axis-column name.
        name: String,
    },
    /// The long-form columns had differing lengths.
    #[error("column `{name}` has {got} rows but the table has {expected}")]
    ColumnLen {
        /// The mismatched column name.
        name: String,
        /// The table's row count.
        expected: usize,
        /// The column's row count.
        got: usize,
    },
    /// An axis column held a non-finite (NaN/±∞) coordinate — it cannot be a grid breakpoint.
    #[error("axis {axis} has a non-finite (NaN/inf) coordinate; axis breakpoints must be finite")]
    NonFiniteAxis {
        /// The offending axis index.
        axis: usize,
    },
    /// The long-form samples did not cover every grid cell — the data is not a full rectilinear grid
    /// (a ragged/sheared table whose axes were inferred from per-row coordinates leaves holes).
    #[error(
        "long-form data is not a full rectilinear grid: {covered} of {expected} cells covered"
    )]
    IncompleteGrid {
        /// The number of grid cells that received a sample.
        covered: usize,
        /// The number of grid cells the discovered axes imply.
        expected: usize,
    },
    /// A requested value column was not present in the table.
    #[error("value column `{name}` is not present in the table")]
    MissingValueColumn {
        /// The missing value-column name.
        name: String,
    },
}

/// Flags recording how an evaluation related to the tabulated domain.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EvalFlags {
    /// The query fell outside the grid on at least one axis (extrapolated or clamped).
    pub extrapolated: bool,
    /// The active interpolation stencil touched a cell that was originally NaN (out of the valid
    /// data hull; the value came from the nearest-valid fill).
    pub out_of_hull: bool,
}

/// A rectilinear N-D tensor-product monotone cubic Hermite interpolant.
///
/// Build with [`GriddedMapN::from_gridded`] (a row-major values tensor) or via
/// [`GriddedTable`] (long/tidy columns). Evaluate with [`GriddedMapN::eval`] /
/// [`GriddedMapN::eval_flagged`] and differentiate with [`GriddedMapN::grad_into`]. All queries are
/// zero-allocation.
#[derive(Clone, Debug)]
pub struct GriddedMapN<T> {
    ndim: usize,
    axes: Vec<Vec<T>>,
    modes: Vec<OutOfDomain>,
    shape: Vec<usize>,
    strides: Vec<usize>,
    /// `n_nodes * 2ⁿ` mixed partials, node-major: `partials[node * 2ⁿ + d]` is `∂^{|d|}f` for the
    /// derivative pattern `d` (bit `i` set ⇒ one derivative along axis `i`) at `node`.
    partials: Vec<T>,
    /// Per-node validity of the *original* data (false ⇒ NaN-filled).
    hull: Vec<bool>,
    /// Whether any node was NaN-filled (fast path: a clean map never reports out-of-hull).
    has_fill: bool,
    n_nodes: usize,
}

impl<T: Float> GriddedMapN<T> {
    /// Build a map from explicit axes and a row-major values tensor.
    ///
    /// `axes[i]` are the strictly-increasing breakpoints of axis `i` (outermost/slowest-varying
    /// first); `values` is the C-order (row-major) tensor of length `∏ axes[i].len()`. Non-finite
    /// (NaN) cells are permitted and filled by nearest-valid BFS. `modes` gives the per-axis
    /// out-of-domain behaviour.
    ///
    /// # Errors
    /// [`GridMapError`] if the axes/values/modes are inconsistent, an axis is too short or
    /// non-increasing, too many axes are given, or every cell is NaN.
    pub fn from_gridded(
        axes: Vec<Vec<T>>,
        values: Vec<T>,
        modes: Vec<OutOfDomain>,
    ) -> Result<Self, GridMapError> {
        let ndim = axes.len();
        if ndim == 0 {
            return Err(GridMapError::NoAxes);
        }
        if ndim > MAX_DIMS {
            return Err(GridMapError::TooManyDims { got: ndim });
        }
        if modes.len() != ndim {
            return Err(GridMapError::ModesLen {
                expected: ndim,
                got: modes.len(),
            });
        }
        let mut shape = Vec::with_capacity(ndim);
        for (axis, xs) in axes.iter().enumerate() {
            if xs.len() < 2 {
                return Err(GridMapError::AxisTooShort {
                    axis,
                    got: xs.len(),
                });
            }
            for k in 0..xs.len() - 1 {
                if xs[k + 1] <= xs[k] {
                    return Err(GridMapError::AxisNotIncreasing { axis, index: k });
                }
            }
            shape.push(xs.len());
        }
        let n_nodes: usize = shape.iter().product();
        if values.len() != n_nodes {
            return Err(GridMapError::ValuesLen {
                expected: n_nodes,
                got: values.len(),
            });
        }
        // Row-major strides: the last axis is contiguous.
        let mut strides = vec![0usize; ndim];
        strides[ndim - 1] = 1;
        for i in (0..ndim - 1).rev() {
            strides[i] = strides[i + 1] * shape[i + 1];
        }

        let (filled, hull) = fill_nan(&shape, &strides, values)?;
        let partials = precompute_partials(&shape, &strides, &axes, &filled);
        let has_fill = hull.iter().any(|&valid| !valid);

        Ok(Self {
            ndim,
            axes,
            modes,
            shape,
            strides,
            partials,
            hull,
            has_fill,
            n_nodes,
        })
    }

    /// The number of grid axes.
    pub fn ndim(&self) -> usize {
        self.ndim
    }

    /// The per-axis breakpoint counts (the tensor shape).
    pub fn shape(&self) -> &[usize] {
        &self.shape
    }

    /// The total number of grid nodes.
    pub fn n_nodes(&self) -> usize {
        self.n_nodes
    }

    /// The `[first, last]` breakpoint of each axis.
    pub fn domain(&self) -> Vec<(T, T)> {
        self.axes
            .iter()
            .map(|xs| (xs[0], xs[xs.len() - 1]))
            .collect()
    }

    /// Interpolate at `x` (length must equal [`ndim`](Self::ndim)); out-of-domain per the axis modes.
    ///
    /// # Panics
    /// Panics if `x.len() != self.ndim()`. Use [`GriddedMapN::try_eval`] for a checked variant.
    pub fn eval(&self, x: &[T]) -> T {
        self.eval_flagged(x).0
    }

    /// Like [`GriddedMapN::eval`] but returns the value and the [`EvalFlags`] for the query.
    ///
    /// # Panics
    /// Panics if `x.len() != self.ndim()`.
    pub fn eval_flagged(&self, x: &[T]) -> (T, EvalFlags) {
        assert_eq!(x.len(), self.ndim, "query dimensionality mismatch");
        let mut w = AxisScratch::default();
        let flags = self.compute_weights(x, &mut w);
        let value = self.accumulate(&w, None);
        (value, flags)
    }

    /// Checked [`GriddedMapN::eval`]: returns an error instead of panicking on a dimension mismatch.
    ///
    /// # Errors
    /// [`GridMapError::QueryDim`] if `x.len() != self.ndim()`.
    pub fn try_eval(&self, x: &[T]) -> Result<T, GridMapError> {
        if x.len() != self.ndim {
            return Err(GridMapError::QueryDim {
                expected: self.ndim,
                got: x.len(),
            });
        }
        Ok(self.eval(x))
    }

    /// Write the analytic gradient `∂f/∂xⱼ` (length `ndim`) into `out`.
    ///
    /// # Panics
    /// Panics if `x.len() != self.ndim()` or `out.len() != self.ndim()`.
    pub fn grad_into(&self, x: &[T], out: &mut [T]) {
        assert_eq!(x.len(), self.ndim, "query dimensionality mismatch");
        assert_eq!(out.len(), self.ndim, "gradient buffer length mismatch");
        let mut w = AxisScratch::default();
        let _ = self.compute_weights(x, &mut w);
        for (j, slot) in out.iter_mut().enumerate() {
            *slot = self.accumulate(&w, Some(j));
        }
    }

    /// Compute per-axis interval, value weights, tangent weights, and their derivatives; return the
    /// domain flags. Zero-allocation (fixed stack scratch).
    #[allow(clippy::needless_range_loop)] // `i` indexes axes/modes/scratch in lockstep with x.
    fn compute_weights(&self, x: &[T], w: &mut AxisScratch<T>) -> EvalFlags {
        let mut flags = EvalFlags::default();
        for i in 0..self.ndim {
            let xs = &self.axes[i];
            let n = xs.len();
            let lo = xs[0];
            let hi = xs[n - 1];
            let xi = x[i];
            if xi < lo {
                flags.extrapolated = true;
                let dx = xi - lo;
                w.k[i] = 0;
                match self.modes[i] {
                    OutOfDomain::Clamp => {
                        w.wv[i] = [T::one(), T::zero()];
                        w.wt[i] = [T::zero(), T::zero()];
                        w.dwv[i] = [T::zero(), T::zero()];
                        w.dwt[i] = [T::zero(), T::zero()];
                    }
                    OutOfDomain::Linear => {
                        w.wv[i] = [T::one(), T::zero()];
                        w.wt[i] = [dx, T::zero()];
                        w.dwv[i] = [T::zero(), T::zero()];
                        w.dwt[i] = [T::one(), T::zero()];
                    }
                }
            } else if xi > hi {
                flags.extrapolated = true;
                let dx = xi - hi;
                w.k[i] = n - 2;
                match self.modes[i] {
                    OutOfDomain::Clamp => {
                        w.wv[i] = [T::zero(), T::one()];
                        w.wt[i] = [T::zero(), T::zero()];
                        w.dwv[i] = [T::zero(), T::zero()];
                        w.dwt[i] = [T::zero(), T::zero()];
                    }
                    OutOfDomain::Linear => {
                        w.wv[i] = [T::zero(), T::one()];
                        w.wt[i] = [T::zero(), dx];
                        w.dwv[i] = [T::zero(), T::zero()];
                        w.dwt[i] = [T::zero(), T::one()];
                    }
                }
            } else {
                // In-domain: locate the interval and build the four Hermite weights.
                let k = if xi >= hi {
                    n - 2
                } else {
                    xs.partition_point(|&xk| xk <= xi).saturating_sub(1)
                };
                let h = xs[k + 1] - xs[k];
                let t = (xi - xs[k]) / h;
                let (h00, h10, h01, h11) = hermite_basis(t);
                let (d00, d10, d01, d11) = hermite_basis_deriv(t);
                w.k[i] = k;
                w.wv[i] = [h00, h01];
                w.wt[i] = [h10 * h, h11 * h];
                w.dwv[i] = [d00 / h, d01 / h];
                w.dwt[i] = [d10, d11];
            }
        }
        // Out-of-hull: does the value's domain of dependence touch a NaN-filled node? A clean map
        // never can, so skip the scan entirely.
        if self.has_fill && self.stencil_touches_fill(&w.k) {
            flags.out_of_hull = true;
        }
        flags
    }

    /// Whether the located cell's *domain of dependence* touches an originally-NaN (filled) node.
    ///
    /// The cell value uses the `2ⁿ` corner values **and** their Fritsch–Carlson tangents/mixed
    /// partials, and a tangent at node `k` is built from its `k±1` fibre neighbours. So along each
    /// axis the value can depend on nodes `k-1 .. k+2` (clamped to the grid), and the full N-D
    /// dependence is the tensor product of those ranges. Checking that box flags exactly the queries
    /// whose value can be influenced by fill data (never under-reports; may over-report only where a
    /// tangent weight vanishes, which is safe for a degraded-data advisory).
    fn stencil_touches_fill(&self, k: &[usize; MAX_DIMS]) -> bool {
        let mut lo = [0usize; MAX_DIMS];
        let mut span = [0usize; MAX_DIMS];
        let mut total = 1usize;
        for i in 0..self.ndim {
            let l = k[i].saturating_sub(1);
            let h = (k[i] + 2).min(self.shape[i] - 1);
            lo[i] = l;
            span[i] = h - l + 1;
            total *= span[i];
        }
        for m in 0..total {
            let mut rem = m;
            let mut node = 0usize;
            for i in 0..self.ndim {
                let idx = lo[i] + rem % span[i];
                rem /= span[i];
                node += idx * self.strides[i];
            }
            if !self.hull[node] {
                return true;
            }
        }
        false
    }

    /// Accumulate the tensor-product Hermite sum. `deriv_axis = Some(j)` differentiates w.r.t. axis
    /// `j` (uses the derivative weight set on that axis, value weights elsewhere).
    fn accumulate(&self, w: &AxisScratch<T>, deriv_axis: Option<usize>) -> T {
        let patterns = 1usize << self.ndim;
        let mut acc = T::zero();
        for c in 0..patterns {
            // Corner node index; skip corners with an all-zero (inactive) weight on any axis.
            let mut node = 0usize;
            let mut active = true;
            for i in 0..self.ndim {
                let ci = (c >> i) & 1;
                let (vw, tw) = axis_pair(w, i, ci, deriv_axis);
                if vw == T::zero() && tw == T::zero() {
                    active = false;
                    break;
                }
                node += (w.k[i] + ci) * self.strides[i];
            }
            if !active {
                continue;
            }
            let base = node * patterns;
            for d in 0..patterns {
                let mut weight = T::one();
                for i in 0..self.ndim {
                    let ci = (c >> i) & 1;
                    let di = (d >> i) & 1;
                    let (vw, tw) = axis_pair(w, i, ci, deriv_axis);
                    weight = weight * if di == 0 { vw } else { tw };
                    if weight == T::zero() {
                        break;
                    }
                }
                if weight != T::zero() {
                    acc = acc + weight * self.partials[base + d];
                }
            }
        }
        acc
    }
}

/// The `(value_weight, tangent_weight)` for axis `i`, corner `ci`, using the derivative weight set
/// iff this axis is the one being differentiated.
#[inline]
fn axis_pair<T: Float>(
    w: &AxisScratch<T>,
    i: usize,
    ci: usize,
    deriv_axis: Option<usize>,
) -> (T, T) {
    if deriv_axis == Some(i) {
        (w.dwv[i][ci], w.dwt[i][ci])
    } else {
        (w.wv[i][ci], w.wt[i][ci])
    }
}

/// Fixed stack scratch for a single evaluation (keeps [`GriddedMapN::eval`] zero-allocation).
struct AxisScratch<T> {
    k: [usize; MAX_DIMS],
    wv: [[T; 2]; MAX_DIMS],
    wt: [[T; 2]; MAX_DIMS],
    dwv: [[T; 2]; MAX_DIMS],
    dwt: [[T; 2]; MAX_DIMS],
}

impl<T: Float> Default for AxisScratch<T> {
    fn default() -> Self {
        let z2 = [T::zero(); 2];
        Self {
            k: [0; MAX_DIMS],
            wv: [z2; MAX_DIMS],
            wt: [z2; MAX_DIMS],
            dwv: [z2; MAX_DIMS],
            dwt: [z2; MAX_DIMS],
        }
    }
}

/// Multi-source BFS nearest-valid fill of NaN cells; returns `(filled, hull)` where `hull[node]` is
/// true iff the original cell was finite.
fn fill_nan<T: Float>(
    shape: &[usize],
    strides: &[usize],
    values: Vec<T>,
) -> Result<(Vec<T>, Vec<bool>), GridMapError> {
    let ndim = shape.len();
    let hull: Vec<bool> = values.iter().map(|v| v.is_finite()).collect();
    if !hull.iter().any(|&f| f) {
        return Err(GridMapError::AllNan);
    }
    let mut filled = values;
    let mut visited = hull.clone();
    let mut queue: VecDeque<usize> = hull
        .iter()
        .enumerate()
        .filter_map(|(i, &f)| f.then_some(i))
        .collect();
    while let Some(node) = queue.pop_front() {
        for i in 0..ndim {
            let idx_i = (node / strides[i]) % shape[i];
            // Lower and upper grid neighbours along axis `i` (fixed order: −1 then +1).
            let mut neighbors = [None; 2];
            if idx_i > 0 {
                neighbors[0] = Some(node - strides[i]);
            }
            if idx_i + 1 < shape[i] {
                neighbors[1] = Some(node + strides[i]);
            }
            for neighbor in neighbors.into_iter().flatten() {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    filled[neighbor] = filled[node];
                    queue.push_back(neighbor);
                }
            }
        }
    }
    Ok((filled, hull))
}

/// Precompute the `2ⁿ` mixed partials at every node (node-major layout).
fn precompute_partials<T: Float>(
    shape: &[usize],
    strides: &[usize],
    axes: &[Vec<T>],
    filled: &[T],
) -> Vec<T> {
    let ndim = shape.len();
    let patterns = 1usize << ndim;
    let n_nodes = filled.len();
    let mut part = vec![T::zero(); n_nodes * patterns];
    // Pattern 0: the values themselves.
    for (node, &v) in filled.iter().enumerate() {
        part[node * patterns] = v;
    }
    // Higher patterns: apply the 1-D tangent limiter along the highest set axis of the parent.
    let mut fiber_ys: Vec<T> = Vec::new();
    for d in 1..patterns {
        let imax = highest_set_bit(d);
        let parent = d & !(1 << imax);
        let len = shape[imax];
        let stride = strides[imax];
        fiber_ys.resize(len, T::zero());
        for node in 0..n_nodes {
            // Fibre starts: nodes whose axis-imax index is zero.
            if !(node / stride).is_multiple_of(len) {
                continue;
            }
            for j in 0..len {
                fiber_ys[j] = part[(node + j * stride) * patterns + parent];
            }
            let tangents = fritsch_carlson_tangents(&axes[imax], &fiber_ys);
            for (j, &m) in tangents.iter().enumerate() {
                part[(node + j * stride) * patterns + d] = m;
            }
        }
    }
    part
}

/// Index of the highest set bit of a non-zero `d`.
#[inline]
fn highest_set_bit(d: usize) -> usize {
    (usize::BITS - 1 - d.leading_zeros()) as usize
}

/// A decoded rectilinear table: the shared axes plus one or more named value columns.
///
/// This is the wasm-clean, in-memory representation of a long/tidy sidecar (a parquet reader lives
/// behind a feature flag in `outlap-schema` and produces one of these). Turn a value column into an
/// interpolant with [`GriddedTable::map`].
#[derive(Clone, Debug)]
pub struct GriddedTable<T> {
    axis_names: Vec<String>,
    axes: Vec<Vec<T>>,
    columns: Vec<(String, Vec<T>)>,
}

impl<T: Float> GriddedTable<T> {
    /// Pivot long/tidy columns onto a rectilinear grid.
    ///
    /// `columns` are equal-length named columns (one row per sample). `axis_names` selects, in
    /// tensor order (outermost first), which columns are the grid axes; every other column becomes a
    /// value column. Axis breakpoints are the sorted distinct coordinates. Rows may arrive in any
    /// order, and duplicate rows for a cell are allowed (last wins). The samples must cover **every**
    /// grid cell (a full rectilinear product) — this is exactly what the importers emit, with any
    /// unreachable cell carried as a NaN in the *value* column (masked/filled by the interpolant),
    /// not as a missing row. A ragged table that leaves cells uncovered is a config error.
    ///
    /// # Errors
    /// [`GridMapError`] if an axis name is unknown, columns differ in length, too many axes are
    /// given, an axis coordinate is non-finite ([`GridMapError::NonFiniteAxis`]), or the samples do
    /// not cover a full rectilinear grid ([`GridMapError::IncompleteGrid`]).
    #[allow(clippy::too_many_lines)] // one linear pivot procedure; splitting it hurts clarity.
    pub fn from_long(
        columns: &[(String, Vec<T>)],
        axis_names: &[&str],
    ) -> Result<Self, GridMapError> {
        let ndim = axis_names.len();
        if ndim == 0 {
            return Err(GridMapError::NoAxes);
        }
        if ndim > MAX_DIMS {
            return Err(GridMapError::TooManyDims { got: ndim });
        }
        let n_rows = columns.first().map_or(0, |(_, v)| v.len());
        for (name, col) in columns {
            if col.len() != n_rows {
                return Err(GridMapError::ColumnLen {
                    name: name.clone(),
                    expected: n_rows,
                    got: col.len(),
                });
            }
        }
        // Locate each axis column and each value column.
        let mut axis_cols: Vec<&[T]> = Vec::with_capacity(ndim);
        for name in axis_names {
            let col = columns.iter().find(|(n, _)| n == name).ok_or_else(|| {
                GridMapError::UnknownAxisColumn {
                    name: (*name).to_owned(),
                }
            })?;
            axis_cols.push(&col.1);
        }
        // Distinct sorted breakpoints per axis. Axis coordinates must be finite: a NaN would make
        // `partial_cmp` partial (panic) and cannot be a grid breakpoint. (A value column *may* carry
        // NaN — that is the map's own unreachable-cell mask, handled by the interpolant's fill.)
        let mut axes: Vec<Vec<T>> = Vec::with_capacity(ndim);
        for (axis, col) in axis_cols.iter().enumerate() {
            if col.iter().any(|v| !v.is_finite()) {
                return Err(GridMapError::NonFiniteAxis { axis });
            }
            let mut vals: Vec<T> = col.to_vec();
            // Safe: every coordinate is finite, so `partial_cmp` is a total order.
            vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            vals.dedup();
            if vals.len() < 2 {
                return Err(GridMapError::AxisTooShort {
                    axis,
                    got: vals.len(),
                });
            }
            axes.push(vals);
        }
        let shape: Vec<usize> = axes.iter().map(Vec::len).collect();
        let n_nodes: usize = shape.iter().product();
        let mut strides = vec![0usize; ndim];
        strides[ndim - 1] = 1;
        for i in (0..ndim - 1).rev() {
            strides[i] = strides[i + 1] * shape[i + 1];
        }
        // Value columns: everything not named as an axis. Scatter into row-major tensors.
        let value_names: Vec<String> = columns
            .iter()
            .filter(|(n, _)| !axis_names.contains(&n.as_str()))
            .map(|(n, _)| n.clone())
            .collect();
        // Scatter each value column into its tensor (NaN for unfilled cells).
        let nan = T::nan();
        let mut out_columns: Vec<(String, Vec<T>)> = value_names
            .iter()
            .map(|n| (n.clone(), vec![nan; n_nodes]))
            .collect();
        let value_src: Vec<&[T]> = value_names
            .iter()
            .map(|n| {
                columns
                    .iter()
                    .find(|(cn, _)| cn == n)
                    .map(|(_, v)| v.as_slice())
                    .expect("value column present by construction")
            })
            .collect();
        // Every node must be covered by exactly one sample so the pivot is a genuine full
        // rectilinear grid. Missing cells (a ragged/sheared table where axes were inferred from
        // per-row coordinates) are a config error, not silent NaN holes.
        let mut covered = vec![false; n_nodes];
        for row in 0..n_rows {
            let mut node = 0usize;
            for axis in 0..ndim {
                let coord = axis_cols[axis][row];
                // The coordinate came from this axis column, so binary_search always finds it (axis
                // coordinates were validated finite above). On the impossible miss, leave the cell
                // uncovered; the coverage check below turns that into a typed error, never a panic.
                if let Ok(idx) = axes[axis].binary_search_by(|probe| {
                    probe
                        .partial_cmp(&coord)
                        .unwrap_or(core::cmp::Ordering::Equal)
                }) {
                    node += idx * strides[axis];
                } else {
                    node = usize::MAX;
                    break;
                }
            }
            if node == usize::MAX {
                continue;
            }
            covered[node] = true;
            for (col_i, src) in value_src.iter().enumerate() {
                out_columns[col_i].1[node] = src[row];
            }
        }
        let filled = covered.iter().filter(|&&c| c).count();
        if filled != n_nodes {
            return Err(GridMapError::IncompleteGrid {
                covered: filled,
                expected: n_nodes,
            });
        }
        let _ = shape; // consumed only to derive strides/n_nodes above.
        Ok(Self {
            axis_names: axis_names.iter().map(|s| (*s).to_owned()).collect(),
            axes,
            columns: out_columns,
        })
    }

    /// The axis names in tensor order.
    pub fn axis_names(&self) -> &[String] {
        &self.axis_names
    }

    /// The value-column names.
    pub fn value_names(&self) -> impl Iterator<Item = &str> {
        self.columns.iter().map(|(n, _)| n.as_str())
    }

    /// Build an interpolant for value column `value` with the given per-axis out-of-domain `modes`.
    ///
    /// # Errors
    /// [`GridMapError`] if the value column is missing or the axes/modes are inconsistent.
    pub fn map(
        &self,
        value: &str,
        modes: Vec<OutOfDomain>,
    ) -> Result<GriddedMapN<T>, GridMapError> {
        let col = self
            .columns
            .iter()
            .find(|(n, _)| n == value)
            .ok_or_else(|| GridMapError::MissingValueColumn {
                name: value.to_owned(),
            })?;
        GriddedMapN::from_gridded(self.axes.clone(), col.1.clone(), modes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MonotoneCubic;

    fn clamp_modes(n: usize) -> Vec<OutOfDomain> {
        vec![OutOfDomain::Clamp; n]
    }

    /// Build a 2-D row-major tensor `values[i*ny + j] = f(xs[i], ys[j])`.
    fn tensor2(xs: &[f64], ys: &[f64], f: impl Fn(f64, f64) -> f64) -> Vec<f64> {
        let mut v = Vec::with_capacity(xs.len() * ys.len());
        for &x in xs {
            for &y in ys {
                v.push(f(x, y));
            }
        }
        v
    }

    #[test]
    fn one_d_matches_monotone_cubic() {
        let xs = vec![0.0, 1.0, 2.5, 4.0, 5.0];
        let ys = vec![0.0, 2.0, 3.0, 3.5, 10.0];
        let map = GriddedMapN::from_gridded(vec![xs.clone()], ys.clone(), clamp_modes(1)).unwrap();
        let mc = MonotoneCubic::new(xs, ys).unwrap();
        let mut g = [0.0];
        for i in 0..=200 {
            let x = -1.0 + 7.0 * f64::from(i) / 200.0;
            assert!(
                (map.eval(&[x]) - mc.eval(x)).abs() < 1e-12,
                "value mismatch at x={x}"
            );
            map.grad_into(&[x], &mut g);
            assert!(
                (g[0] - mc.deriv(x)).abs() < 1e-12,
                "deriv mismatch at x={x}: {} vs {}",
                g[0],
                mc.deriv(x)
            );
        }
    }

    #[test]
    fn fibers_match_monotone_cubic_2d() {
        let xs = vec![0.0, 1.0, 2.0, 3.5, 5.0];
        let ys = vec![-2.0, 0.0, 1.0, 4.0];
        // A non-separable, monotone-ish field.
        let f = |x: f64, y: f64| 0.5 * x * x + y - 0.1 * x * y + 3.0;
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            tensor2(&xs, &ys, f),
            clamp_modes(2),
        )
        .unwrap();
        // For each grid node in y, the map along x must equal MonotoneCubic on that fibre.
        for (j, &y0) in ys.iter().enumerate() {
            let fiber: Vec<f64> = xs.iter().map(|&x| f(x, y0)).collect();
            let mc = MonotoneCubic::new(xs.clone(), fiber).unwrap();
            let mut g = [0.0, 0.0];
            for i in 0..=100 {
                let x = xs[0] + (xs[4] - xs[0]) * f64::from(i) / 100.0;
                assert!(
                    (map.eval(&[x, y0]) - mc.eval(x)).abs() < 1e-12,
                    "2d fibre value mismatch at (x={x}, y={y0})"
                );
                map.grad_into(&[x, y0], &mut g);
                // MonotoneCubic::deriv clamps to 0 at the exact outer endpoints; the tensor map
                // reports the (C¹-from-inside) interior slope there. Compare on the open interval.
                if x > xs[0] && x < xs[4] {
                    assert!(
                        (g[0] - mc.deriv(x)).abs() < 1e-12,
                        "2d fibre d/dx mismatch at (x={x}, y={y0}, j={j})"
                    );
                }
            }
        }
        // And symmetrically along y at each grid node in x.
        for &x0 in &xs {
            let fiber: Vec<f64> = ys.iter().map(|&y| f(x0, y)).collect();
            let mc = MonotoneCubic::new(ys.clone(), fiber).unwrap();
            let mut g = [0.0, 0.0];
            for i in 0..=100 {
                let y = ys[0] + (ys[3] - ys[0]) * f64::from(i) / 100.0;
                assert!((map.eval(&[x0, y]) - mc.eval(y)).abs() < 1e-12);
                map.grad_into(&[x0, y], &mut g);
                if y > ys[0] && y < ys[3] {
                    assert!((g[1] - mc.deriv(y)).abs() < 1e-12);
                }
            }
        }
    }

    #[test]
    fn fibers_match_monotone_cubic_3d() {
        let xs = vec![0.0, 1.0, 2.0, 3.0];
        let ys = vec![0.0, 0.5, 2.0];
        let zs = vec![10.0, 20.0, 30.0, 45.0];
        let f = |x: f64, y: f64, z: f64| x * x + 2.0 * y - 0.01 * z * z + 0.3 * x * y * z;
        let mut vals = Vec::new();
        for &x in &xs {
            for &y in &ys {
                for &z in &zs {
                    vals.push(f(x, y, z));
                }
            }
        }
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone(), zs.clone()],
            vals,
            clamp_modes(3),
        )
        .unwrap();
        // Vary z on the grid-aligned fibre through node (x=xs[1], y=ys[2]).
        let x0 = xs[1];
        let y0 = ys[2];
        let fiber: Vec<f64> = zs.iter().map(|&z| f(x0, y0, z)).collect();
        let mc = MonotoneCubic::new(zs.clone(), fiber).unwrap();
        let mut g = [0.0; 3];
        for i in 0..=100 {
            let z = zs[0] + (zs[3] - zs[0]) * f64::from(i) / 100.0;
            assert!((map.eval(&[x0, y0, z]) - mc.eval(z)).abs() < 1e-12);
            map.grad_into(&[x0, y0, z], &mut g);
            if z > zs[0] && z < zs[3] {
                assert!((g[2] - mc.deriv(z)).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn bilinear_reproduced_everywhere() {
        // A bilinear field f = a + b·x + c·y + d·x·y lies in the tensor-Hermite space (degree ≤1 per
        // axis), so the map must reproduce it EXACTLY at off-node points too — a strong check that
        // the value, first-partial, and mixed-partial precompute are all correct.
        let xs = vec![0.0, 1.0, 2.5, 4.0];
        let ys = vec![-1.0, 0.5, 2.0, 3.0, 5.0];
        let f = |x: f64, y: f64| 2.0 + 3.0 * x - 1.5 * y + 0.7 * x * y;
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            tensor2(&xs, &ys, f),
            clamp_modes(2),
        )
        .unwrap();
        let mut g = [0.0, 0.0];
        for i in 0..=37 {
            for j in 0..=41 {
                let x = xs[0] + (xs[3] - xs[0]) * f64::from(i) / 37.0;
                let y = ys[0] + (ys[4] - ys[0]) * f64::from(j) / 41.0;
                assert!(
                    (map.eval(&[x, y]) - f(x, y)).abs() < 1e-11,
                    "bilinear value off at ({x},{y})"
                );
                // Analytic gradient of the bilinear: (b + d·y, c + d·x).
                map.grad_into(&[x, y], &mut g);
                assert!(
                    (g[0] - (3.0 + 0.7 * y)).abs() < 1e-10,
                    "d/dx off at ({x},{y})"
                );
                assert!(
                    (g[1] - (-1.5 + 0.7 * x)).abs() < 1e-10,
                    "d/dy off at ({x},{y})"
                );
            }
        }
    }

    #[test]
    fn exact_node_interpolation() {
        let xs = vec![0.0, 1.0, 3.0];
        let ys = vec![0.0, 2.0, 5.0, 6.0];
        let f = |x: f64, y: f64| (x + 1.0) * (y - 2.0) + 0.2 * x * x;
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            tensor2(&xs, &ys, f),
            clamp_modes(2),
        )
        .unwrap();
        for &x in &xs {
            for &y in &ys {
                assert!(
                    (map.eval(&[x, y]) - f(x, y)).abs() < 1e-12,
                    "node ({x},{y}) not reproduced"
                );
            }
        }
    }

    #[test]
    fn analytic_grad_matches_finite_difference() {
        let xs = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let ys = vec![0.0, 1.0, 2.5, 4.0];
        let f = |x: f64, y: f64| (0.3 * x * x - 0.5 * y + 0.1 * x * y + 2.0).sin() + x;
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            tensor2(&xs, &ys, f),
            clamp_modes(2),
        )
        .unwrap();
        let h = 1e-6;
        let mut g = [0.0, 0.0];
        for &(x, y) in &[(0.7, 0.4), (1.3, 1.8), (2.2, 3.1), (3.6, 2.2)] {
            map.grad_into(&[x, y], &mut g);
            let fdx = (map.eval(&[x + h, y]) - map.eval(&[x - h, y])) / (2.0 * h);
            let fdy = (map.eval(&[x, y + h]) - map.eval(&[x, y - h])) / (2.0 * h);
            assert!(
                (g[0] - fdx).abs() < 1e-4,
                "d/dx at ({x},{y}): {} vs {fdx}",
                g[0]
            );
            assert!(
                (g[1] - fdy).abs() < 1e-4,
                "d/dy at ({x},{y}): {} vs {fdy}",
                g[1]
            );
        }
    }

    #[test]
    fn c1_continuity_across_cell_boundary() {
        let xs = vec![0.0, 1.0, 2.0, 3.0];
        let ys = vec![0.0, 1.0, 2.0];
        let f = |x: f64, y: f64| x * x * x - y * y + x * y;
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            tensor2(&xs, &ys, f),
            clamp_modes(2),
        )
        .unwrap();
        // Approach the interior grid line x=2 from both sides; value and gradient must agree.
        let eps = 1e-7;
        let mut gl = [0.0, 0.0];
        let mut gr = [0.0, 0.0];
        for &y in &[0.3, 1.4] {
            let vl = map.eval(&[2.0 - eps, y]);
            let vr = map.eval(&[2.0 + eps, y]);
            assert!((vl - vr).abs() < 1e-5, "value jump at x=2, y={y}");
            map.grad_into(&[2.0 - eps, y], &mut gl);
            map.grad_into(&[2.0 + eps, y], &mut gr);
            assert!((gl[0] - gr[0]).abs() < 1e-4, "slope jump at x=2, y={y}");
        }
    }

    #[test]
    fn clamp_saturates_linear_extrapolates() {
        let xs = vec![0.0, 1.0, 2.0, 3.0];
        let ys: Vec<f64> = xs.iter().map(|&x| 1.0 + 2.0 * x).collect(); // exactly linear
        let clamp =
            GriddedMapN::from_gridded(vec![xs.clone()], ys.clone(), vec![OutOfDomain::Clamp])
                .unwrap();
        let lin =
            GriddedMapN::from_gridded(vec![xs.clone()], ys.clone(), vec![OutOfDomain::Linear])
                .unwrap();
        // Clamp: constant beyond the grid.
        assert!((clamp.eval(&[-5.0]) - 1.0).abs() < 1e-12);
        assert!((clamp.eval(&[9.0]) - 7.0).abs() < 1e-12);
        // Linear: exact-linear extrapolation (this data is linear, tangents are the slope 2).
        assert!(
            (lin.eval(&[-5.0]) - (1.0 + 2.0 * -5.0)).abs() < 1e-9,
            "below-grid linear"
        );
        assert!(
            (lin.eval(&[9.0]) - (1.0 + 2.0 * 9.0)).abs() < 1e-9,
            "above-grid linear"
        );
        // Derivative continuity at the boundary for Linear.
        let mut gin = [0.0];
        let mut gout = [0.0];
        lin.grad_into(&[3.0 - 1e-7], &mut gin);
        lin.grad_into(&[3.0 + 1e-7], &mut gout);
        assert!(
            (gin[0] - gout[0]).abs() < 1e-4,
            "linear slope jump at upper boundary"
        );
        assert!((gout[0] - 2.0).abs() < 1e-6, "linear extrapolation slope");
        // Flags: extrapolated set out of domain, clear inside.
        assert!(lin.eval_flagged(&[9.0]).1.extrapolated);
        assert!(!lin.eval_flagged(&[1.5]).1.extrapolated);
    }

    #[test]
    fn nan_cells_filled_and_flagged() {
        // A 6x6 grid with one NaN node at (i=1, j=1). The 6x6 size leaves cells (e.g. [4,4]) far
        // enough that even the tangent domain of dependence (k-1..k+2) misses the filled node.
        let xs: Vec<f64> = (0..6).map(f64::from).collect();
        let ys: Vec<f64> = (0..6).map(f64::from).collect();
        let mut vals = tensor2(&xs, &ys, |x, y| x + y);
        vals[6 + 1] = f64::NAN; // node (i=1, j=1) in the 6x6 row-major tensor
        let map =
            GriddedMapN::from_gridded(vec![xs.clone(), ys.clone()], vals, clamp_modes(2)).unwrap();
        // Everything stays finite.
        for i in 0..=50 {
            for j in 0..=50 {
                let x = 5.0 * f64::from(i) / 50.0;
                let y = 5.0 * f64::from(j) / 50.0;
                assert!(map.eval(&[x, y]).is_finite());
            }
        }
        // The NaN node reproduces its filled (nearest-valid) value, not NaN.
        assert!(map.eval(&[1.0, 1.0]).is_finite());
        // A query in a cell touching the filled node is flagged out-of-hull; a far cell is not.
        assert!(map.eval_flagged(&[1.1, 1.1]).1.out_of_hull);
        assert!(!map.eval_flagged(&[4.5, 4.5]).1.out_of_hull);
    }

    #[test]
    fn out_of_hull_covers_tangent_leakage() {
        // Regression (adversarial review): a query two cells from a NaN node still depends on it
        // through the Fritsch–Carlson tangent stencil, so it MUST be flagged out-of-hull even though
        // none of its own cell corners is filled.
        let xs: Vec<f64> = (0..6).map(f64::from).collect();
        let ys: Vec<f64> = (0..6).map(f64::from).collect();
        let f = |x: f64, y: f64| x + 2.0 * y + 0.3 * x * y;
        let mut vals = tensor2(&xs, &ys, f);
        vals[6 + 1] = f64::NAN; // node (1,1)
        let nan_map =
            GriddedMapN::from_gridded(vec![xs.clone(), ys.clone()], vals, clamp_modes(2)).unwrap();
        let clean_map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            tensor2(&xs, &ys, f),
            clamp_modes(2),
        )
        .unwrap();
        // Cell k=[2,1] (query 2.5,1.4): corners (2,1),(3,1),(2,2),(3,2) are all hull-clean, yet the
        // value differs from clean data (tangent at corner (2,1) used the filled node (1,1)).
        let (v, flags) = nan_map.eval_flagged(&[2.5, 1.4]);
        assert!(flags.out_of_hull, "tangent leakage must flag out_of_hull");
        assert!(
            (v - clean_map.eval(&[2.5, 1.4])).abs() > 1e-6,
            "the value really does depend on the filled node"
        );
    }

    #[test]
    fn all_nan_is_an_error() {
        let xs = vec![0.0, 1.0];
        let vals = vec![f64::NAN, f64::NAN];
        assert!(matches!(
            GriddedMapN::from_gridded(vec![xs], vals, clamp_modes(1)),
            Err(GridMapError::AllNan)
        ));
    }

    #[test]
    fn from_long_pivots_unordered_rows() {
        // Long/tidy rows in scrambled order over a 3x2 grid.
        let speed = vec![
            10.0, 20.0, 10.0, 30.0, 20.0, 30.0, 10.0, 20.0, 30.0, 30.0, 10.0, 20.0,
        ];
        let torque = vec![0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0]; // 3x2 -> 6 unique pairs, duplicated arbitrarily
                                                                                       // Value = speed + 100*torque for the first occurrence of each (dedup handled by pivot: last write wins, but all consistent here).
        let eff: Vec<f64> = speed
            .iter()
            .zip(&torque)
            .map(|(&s, &t)| s + 100.0 * t)
            .collect();
        let cols = vec![
            ("speed_rpm".to_owned(), speed),
            ("torque_nm".to_owned(), torque),
            ("efficiency".to_owned(), eff),
        ];
        let table = GriddedTable::from_long(&cols, &["speed_rpm", "torque_nm"]).unwrap();
        assert_eq!(table.axis_names(), &["speed_rpm", "torque_nm"]);
        let map = table.map("efficiency", clamp_modes(2)).unwrap();
        assert_eq!(map.shape(), &[3, 2]);
        for &s in &[10.0, 20.0, 30.0] {
            for &t in &[0.0, 1.0] {
                assert!((map.eval(&[s, t]) - (s + 100.0 * t)).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn from_long_rejects_nan_axis_without_panicking() {
        // A NaN in an axis column (e.g. a parquet NULL selected as an axis) is a typed error, not a
        // panic (adversarial review): a Result-returning public constructor must stay panic-free.
        let speed = vec![10.0, 20.0, f64::NAN, 20.0];
        let torque = vec![0.0, 0.0, 1.0, 1.0];
        let val = vec![1.0, 2.0, 3.0, 4.0];
        let cols = vec![
            ("speed_rpm".to_owned(), speed),
            ("torque_nm".to_owned(), torque),
            ("v".to_owned(), val),
        ];
        assert!(matches!(
            GriddedTable::from_long(&cols, &["speed_rpm", "torque_nm"]),
            Err(GridMapError::NonFiniteAxis { axis: 0 })
        ));
    }

    #[test]
    fn from_long_rejects_incomplete_grid() {
        // A ragged/sheared table (per-x-varying y) leaves cells uncovered — a config error, not
        // silent NaN holes (adversarial review). Axes x=[0,1], y=[10,20,30,40]; 4 rows cover 4 of 8.
        let x = vec![0.0, 0.0, 1.0, 1.0];
        let y = vec![10.0, 20.0, 30.0, 40.0];
        let v = vec![1.0, 2.0, 3.0, 4.0];
        let cols = vec![
            ("x".to_owned(), x),
            ("y".to_owned(), y),
            ("v".to_owned(), v),
        ];
        assert!(matches!(
            GriddedTable::from_long(&cols, &["x", "y"]),
            Err(GridMapError::IncompleteGrid {
                covered: 4,
                expected: 8
            })
        ));
    }

    #[test]
    fn rejects_bad_construction() {
        assert!(matches!(
            GriddedMapN::<f64>::from_gridded(vec![], vec![], vec![]),
            Err(GridMapError::NoAxes)
        ));
        assert!(matches!(
            GriddedMapN::from_gridded(vec![vec![0.0]], vec![1.0], clamp_modes(1)),
            Err(GridMapError::AxisTooShort { axis: 0, got: 1 })
        ));
        assert!(matches!(
            GriddedMapN::from_gridded(vec![vec![0.0, 0.0]], vec![1.0, 2.0], clamp_modes(1)),
            Err(GridMapError::AxisNotIncreasing { axis: 0, index: 0 })
        ));
        assert!(matches!(
            GriddedMapN::from_gridded(vec![vec![0.0, 1.0]], vec![1.0], clamp_modes(1)),
            Err(GridMapError::ValuesLen {
                expected: 2,
                got: 1
            })
        ));
        assert!(matches!(
            GriddedMapN::from_gridded(vec![vec![0.0, 1.0]], vec![1.0, 2.0], vec![]),
            Err(GridMapError::ModesLen {
                expected: 1,
                got: 0
            })
        ));
        let too_many: Vec<Vec<f64>> = (0..=MAX_DIMS).map(|_| vec![0.0, 1.0]).collect();
        let n = too_many.len();
        assert!(matches!(
            GriddedMapN::from_gridded(too_many, vec![0.0; 1 << (MAX_DIMS + 1)], clamp_modes(n)),
            Err(GridMapError::TooManyDims { .. })
        ));
    }

    #[test]
    fn works_in_f32() {
        let xs = vec![0.0f32, 1.0, 2.0];
        let ys = vec![0.0f32, 1.0];
        let vals = vec![0.0f32, 1.0, 1.0, 2.0, 2.0, 3.0];
        let map = GriddedMapN::from_gridded(vec![xs, ys], vals, clamp_modes(2)).unwrap();
        assert!((map.eval(&[0.5, 0.5]) - 1.0).abs() < 1e-6);
    }
}
