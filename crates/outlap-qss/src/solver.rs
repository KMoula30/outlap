// SPDX-License-Identifier: AGPL-3.0-only
//! The T0 forward/backward velocity-profile solver (§6.1, §11.2).
//!
//! Not an ODE integration: a curvature-limited speed per station followed by a traction-limited
//! forward sweep and a braking-limited backward sweep, taking the pointwise minimum. Re-implemented
//! from the TUM `calc_vel_profile` formulation (Heilmeier et al., *Vehicle System Dynamics* 58(10),
//! 2020) on the 3D ribbon (Perantoni & Limebeer 2015; Lovato & Massaro 2022). The point-mass
//! equations:
//!
//! * `N(s,v)  = m·(g·cosθ_b·cosθ_g + κ_n·v²) + q_z·v²`  (normal load; crest unloads, dip loads)
//! * `F_y(s,v) = m·(κ_l·v² + g·sinθ_b·cosθ_g)`           (lateral demand; banking assists)
//! * `m·v̇ = F_t − q_x·v² − m·g·sinθ_g`                   (longitudinal)
//! * friction ellipse `(F_t/(μ_x γ N))² + (F_y/(μ_y γ N))² ≤ 1`
//!
//! # Two grip models, one sweep (PR7)
//!
//! The per-station grip query is abstracted behind [`GripModel`], monomorphised into the shared
//! [`solve_generic`] passes (no per-station dynamic dispatch):
//!
//! * [`EllipseGrip`] — the constant-μ friction ellipse above (T0's degenerate no-envelope path,
//!   unchanged).
//! * [`GgvGrip`] — the T1-derived g-g-g-v [`GgvEnvelope`]: the tyre-grip boundary comes from the
//!   envelope while the powertrain ceiling (`tractive_force`) and grade are applied by the solver,
//!   exactly as the ellipse path applies them. This is the production T0↔T1 coupling (Decision #31);
//!   `sim.tier` dispatch and the Python surface that select it land in PR8.
//!
//! `[`solve_into`]` / [`solve_into_ggv`] are the zero-allocation kernels (they write into a
//! caller-owned [`T0Workspace`]); [`solve_lap`] is the owning convenience wrapper.

use crate::error::T0Error;
use crate::path::T0Path;
use crate::result::{LapResult, LineDescriptor, T0Workspace};
use crate::t1::GgvEnvelope;
use crate::vehicle::T0Vehicle;
use crate::G;

/// Slope/denominator guard for the closed-form comparisons.
const EPS: f64 = 1e-9;
/// Iteration cap for the closed-lap pass fixed point (a divergence backstop).
const MAX_PASS_ITERS: usize = 8;
/// Relative tolerance for closed-lap seed convergence.
const SEED_TOL: f64 = 1e-6;
/// Bisection iterations for the envelope cornering-speed limit (`v_cap`/2²⁴ ≈ sub-mm/s).
const V_LIMIT_ITERS: usize = 24;
/// Bisection iterations for inverting the envelope's `a_x` boundary at a lateral demand.
const AX_INV_ITERS: usize = 16;

/// The per-station grip model the velocity-profile passes query. Implementations provide the
/// cornering-speed limit and the forward/backward longitudinal steps; [`solve_generic`] owns the
/// sweep logic and is generic (monomorphised) over the model.
trait GripModel {
    /// Curvature-limited (cornering) speed at station `i`, m/s.
    fn v_limit(&self, p: &T0Path, i: usize) -> f64;
    /// Forward (traction-limited) step from station `i` at `v_i` over one `ds`: candidate `v²` next.
    fn forward_v2(&self, p: &T0Path, i: usize, v_i: f64) -> f64;
    /// Backward (braking-limited) step into station `ip1` at `v_ip1`: candidate `v²` earlier.
    fn backward_v2(&self, p: &T0Path, ip1: usize, v_ip1: f64) -> f64;
}

// --- Constant-μ friction ellipse (the degenerate no-envelope path) ------------------------------

/// Curvature-limited speed at station `i`, m/s (closed form: both sides of the lateral friction
/// constraint are affine in `u = v²`).
fn ellipse_v_limit(veh: &T0Vehicle, p: &T0Path, i: usize) -> f64 {
    let m = veh.mass_kg;
    let gamma = p.grip[i];
    // |a·u + b| ≤ c·u + d, where c·u + d = μ_y·γ·N ≥ 0.
    let a = m * p.kappa_l[i];
    let b = m * G * p.sin_b_cos_g[i];
    let c = veh.mu_y * gamma * (m * p.kappa_n[i] + veh.qz);
    let d = veh.mu_y * gamma * m * G * p.cos_b_cos_g[i];

    let mut u = veh.v_cap * veh.v_cap;
    // Upper branch a·u+b ≥ 0: (a−c)u ≤ d−b bounds u only when demand outgrows capability (a > c).
    if a - c > EPS {
        u = u.min((d - b) / (a - c));
    }
    // Lower branch a·u+b < 0: (a+c)u ≥ −(d+b) bounds u when a+c < 0.
    if a + c < -EPS {
        u = u.min((d + b) / -(a + c));
    }
    // Flight guard: N ≥ 0.
    let kn = m * p.kappa_n[i] + veh.qz;
    if kn < -EPS {
        u = u.min(-(m * G * p.cos_b_cos_g[i]) / kn);
    }
    u.max(0.0).sqrt()
}

/// Longitudinal grip remaining after the lateral demand at station `i`, speed `v` (N). Zero if the
/// normal load is non-positive (flight) or the lateral demand already saturates the ellipse.
fn long_grip_remaining(veh: &T0Vehicle, p: &T0Path, i: usize, v: f64) -> f64 {
    let m = veh.mass_kg;
    let u = v * v;
    let n = m * (G * p.cos_b_cos_g[i] + p.kappa_n[i] * u) + veh.qz * u;
    if n <= 0.0 {
        return 0.0;
    }
    let gamma = p.grip[i];
    let cap_x = veh.mu_x * gamma * n;
    let cap_y = veh.mu_y * gamma * n;
    let fy = m * (p.kappa_l[i] * u + G * p.sin_b_cos_g[i]);
    let ratio2 = (fy / cap_y) * (fy / cap_y);
    if ratio2 >= 1.0 {
        0.0
    } else {
        cap_x * (1.0 - ratio2).sqrt()
    }
}

/// Forward (traction-limited) step from station `i` at `v_i` over one `ds`: candidate `v²` next.
fn ellipse_forward_v2(veh: &T0Vehicle, p: &T0Path, i: usize, v_i: f64) -> f64 {
    let f_rem = long_grip_remaining(veh, p, i, v_i);
    let f_t = veh.tractive_force(v_i).min(f_rem);
    let u = v_i * v_i;
    let accel = (f_t - veh.qx * u - veh.mass_kg * G * p.sin_g[i]) / veh.mass_kg;
    (u + 2.0 * p.ds * accel).max(0.0)
}

/// Backward (braking-limited) step from station `ip1` at `v_ip1` over one `ds`: candidate `v²` at
/// the earlier station.
fn ellipse_backward_v2(veh: &T0Vehicle, p: &T0Path, ip1: usize, v_ip1: f64) -> f64 {
    let f_rem = long_grip_remaining(veh, p, ip1, v_ip1);
    let u = v_ip1 * v_ip1;
    // Drag and (uphill) gravity add to the braking budget.
    let a_dec = (f_rem + veh.qx * u + veh.mass_kg * G * p.sin_g[ip1]) / veh.mass_kg;
    (u + 2.0 * p.ds * a_dec).max(0.0)
}

/// The T0 constant-μ friction-ellipse grip model.
struct EllipseGrip<'a> {
    veh: &'a T0Vehicle,
}

impl GripModel for EllipseGrip<'_> {
    fn v_limit(&self, p: &T0Path, i: usize) -> f64 {
        ellipse_v_limit(self.veh, p, i)
    }
    fn forward_v2(&self, p: &T0Path, i: usize, v_i: f64) -> f64 {
        ellipse_forward_v2(self.veh, p, i, v_i)
    }
    fn backward_v2(&self, p: &T0Path, ip1: usize, v_ip1: f64) -> f64 {
        ellipse_backward_v2(self.veh, p, ip1, v_ip1)
    }
}

// --- T1-derived g-g-g-v envelope grip -----------------------------------------------------------

/// The g-g-g-v envelope grip model: the tyre-grip boundary comes from a T1-derived [`GgvEnvelope`],
/// while the powertrain ceiling, aero drag, and grade are applied by the solver (as the ellipse path
/// does). The envelope is queried at its reference state (the T0 vehicle *is* that reference, so the
/// Decision #31 corrections are identity); off-reference sweeps compose in PR8.
struct GgvGrip<'a> {
    veh: &'a T0Vehicle,
    env: &'a GgvEnvelope,
    /// Optional per-station **traction scale** in `[0, 1]` (`None` ⇒ uncoupled, scale ≡ 1). The QSS
    /// slow-state coupling fills this from the machine-thermal derate ∧ battery peak-power cap
    /// marched along the previous pass; it multiplies the powertrain traction ceiling in the forward
    /// step. Braking is unaffected (it draws no drive power). See [`crate::qss`].
    traction_scale: Option<&'a [f64]>,
}

impl<'a> GgvGrip<'a> {
    fn new(veh: &'a T0Vehicle, env: &'a GgvEnvelope) -> Self {
        Self {
            veh,
            env,
            traction_scale: None,
        }
    }

    /// As [`Self::new`] but with a per-station traction scale (the slow-state coupling).
    fn with_scale(veh: &'a T0Vehicle, env: &'a GgvEnvelope, scale: &'a [f64]) -> Self {
        Self {
            veh,
            env,
            traction_scale: Some(scale),
        }
    }

    /// The traction scale at station `i` (`1.0` when uncoupled).
    fn scale(&self, i: usize) -> f64 {
        self.traction_scale.map_or(1.0, |s| s[i])
    }

    /// The envelope's lateral-acceleration boundary at `(v, a_x, g_normal)`, scaled by the local
    /// track **grip scale** `grip` (`T0Path::grip`, the surface `grip_scale`). Friction scales the
    /// boundary linearly, exactly as the constant-μ ellipse scales its `μ` by `γ(s)` — so g-g-g-v laps
    /// honour grip maps like the ellipse path does.
    fn ay(&self, v: f64, ax: f64, gn: f64, grip: f64) -> f64 {
        self.env.ay_boundary(v, ax, gn) * grip
    }

    /// The maximum feasible **positive** longitudinal acceleration (net of the envelope's embedded
    /// drag) at which the tyre grip still meets the lateral demand `ay` (magnitude), m/s². Bisected
    /// on the envelope's `a_x` boundary (which decreases as `a_x` grows).
    fn ax_forward(&self, v: f64, gn: f64, ay: f64, grip: f64) -> f64 {
        if self.ay(v, 0.0, gn, grip) < ay {
            return 0.0; // the corner is already at/over the lateral limit — no accel budget
        }
        let hi0 = self.env.accel_limit(v, gn);
        if self.ay(v, hi0, gn, grip) >= ay {
            return hi0; // grip is not the limit here (the powertrain min caps it)
        }
        let mut lo = 0.0;
        let mut hi = hi0;
        for _ in 0..AX_INV_ITERS {
            let mid = 0.5 * (lo + hi);
            if self.ay(v, mid, gn, grip) >= ay {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// The most-negative feasible longitudinal acceleration (braking, net of embedded drag) at which
    /// the tyre grip still meets the lateral demand `ay` (magnitude), m/s² (≤ 0).
    fn ax_backward(&self, v: f64, gn: f64, ay: f64, grip: f64) -> f64 {
        if self.ay(v, 0.0, gn, grip) < ay {
            return 0.0;
        }
        let lo0 = -self.env.brake_limit(v, gn);
        if self.ay(v, lo0, gn, grip) >= ay {
            return lo0; // full braking grip available
        }
        let mut lo = lo0; // infeasible (most negative)
        let mut hi = 0.0; // feasible
        for _ in 0..AX_INV_ITERS {
            let mid = 0.5 * (lo + hi);
            if self.ay(v, mid, gn, grip) >= ay {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        hi
    }

    /// Whether the tyres carry positive normal load at `(v, g_normal)`. The road-normal specific
    /// gravity `g_normal` (gravity + vertical-curvature) alone can go non-positive over a high-speed
    /// crest, but **aero downforce** `q_z·v²/m` still plants a downforce car — so the flight guard
    /// keys off the *total* normal specific force, exactly matching the constant-μ ellipse's `N ≤ 0`
    /// test (`N = m·g_normal + q_z·v²`). (The envelope's grip already includes the aero at speed `v`;
    /// this only rules out genuine flight. Between the envelope's lowest `g_normal` sample — `0.5 g`,
    /// a strong crest just short of flight — and 0 the boundary query clamps `g_normal`, so the
    /// gravity contribution to grip is slightly over-predicted there; the error is bounded (`≈ μ·0.5g`)
    /// and dominated by aero at the high speeds where vertical-curvature crests matter.)
    fn planted(&self, v: f64, gn: f64) -> bool {
        gn + self.veh.qz * v * v / self.veh.mass_kg > EPS
    }

    /// Whether speed `v` at station `i` is within the cornering grip limit (and the tyres are loaded).
    fn corner_feasible(&self, p: &T0Path, i: usize, v: f64) -> bool {
        let (ay_dem, gn) = demand_and_gn(p, i, v);
        self.planted(v, gn) && ay_dem.abs() <= self.ay(v, 0.0, gn, p.grip[i])
    }
}

impl GripModel for GgvGrip<'_> {
    fn v_limit(&self, p: &T0Path, i: usize) -> f64 {
        let mut hi = self.veh.v_cap;
        if self.corner_feasible(p, i, hi) {
            return hi;
        }
        let mut lo = 0.0;
        if !self.corner_feasible(p, i, lo) {
            return 0.0; // even crawling is over the limit (very tight/steeply-banked)
        }
        for _ in 0..V_LIMIT_ITERS {
            let mid = 0.5 * (lo + hi);
            if self.corner_feasible(p, i, mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    fn forward_v2(&self, p: &T0Path, i: usize, v_i: f64) -> f64 {
        let u = v_i * v_i;
        let (ay_dem, gn) = demand_and_gn(p, i, v_i);
        let accel = if self.planted(v_i, gn) {
            let ax_grip = self.ax_forward(v_i, gn, ay_dem.abs(), p.grip[i]);
            // Slow-state coupling: the machine-thermal derate ∧ battery peak-power cap scale the
            // powertrain traction ceiling (drag is unaffected). Uncoupled ⇒ scale ≡ 1.
            let pt_net = self.veh.tractive_force(v_i) * self.scale(i) / self.veh.mass_kg
                - self.env.drag_accel(v_i);
            ax_grip.min(pt_net) - G * p.sin_g[i]
        } else {
            // Airborne (crest unloading beyond aero downforce): no traction, just drag + grade coast.
            -self.env.drag_accel(v_i) - G * p.sin_g[i]
        };
        (u + 2.0 * p.ds * accel).max(0.0)
    }

    fn backward_v2(&self, p: &T0Path, ip1: usize, v_ip1: f64) -> f64 {
        let u = v_ip1 * v_ip1;
        let (ay_dem, gn) = demand_and_gn(p, ip1, v_ip1);
        let a_dec = if self.planted(v_ip1, gn) {
            let ax_brake = self.ax_backward(v_ip1, gn, ay_dem.abs(), p.grip[ip1]); // ≤ 0, net of drag
            -ax_brake + G * p.sin_g[ip1]
        } else {
            self.env.drag_accel(v_ip1) + G * p.sin_g[ip1]
        };
        (u + 2.0 * p.ds * a_dec).max(0.0)
    }
}

// --- Shared sweep -------------------------------------------------------------------------------

/// The velocity-frame lateral demand `a_y` (signed; banking assists) and road-normal specific
/// gravity `g_normal` (gravity's normal component + vertical-curvature `κ_n·v²`; aero load is already
/// inside the envelope's speed dependence) at station `i`, speed `v`.
fn demand_and_gn(p: &T0Path, i: usize, v: f64) -> (f64, f64) {
    let u = v * v;
    let ay_dem = p.kappa_l[i] * u + G * p.sin_b_cos_g[i];
    let gn = G * p.cos_b_cos_g[i] + p.kappa_n[i] * u;
    (ay_dem, gn)
}

/// Index of the minimum curvature-limited speed.
fn argmin(v: &[f64]) -> usize {
    let mut best = 0;
    for i in 1..v.len() {
        if v[i] < v[best] {
            best = i;
        }
    }
    best
}

/// Run the forward/backward velocity passes for a grip model into a caller-owned workspace, returning
/// the lap time (seconds). Zero-allocation: touches only the workspace and path slices and the grip
/// model's allocation-free queries.
fn solve_generic<G: GripModel>(g: &G, path: &T0Path, ws: &mut T0Workspace) -> Result<f64, T0Error> {
    let n = path.len();
    if ws.len() != n {
        return Err(T0Error::WorkspaceMismatch {
            workspace: ws.len(),
            path: n,
        });
    }

    for i in 0..n {
        ws.v_lim[i] = g.v_limit(path, i);
        ws.v[i] = ws.v_lim[i];
    }

    if path.closed {
        let i0 = argmin(&ws.v_lim);
        let mut converged = false;
        for _ in 0..MAX_PASS_ITERS {
            let before = ws.v[i0];
            // Forward sweep, one full wrap from the slowest point.
            let mut i = i0;
            for _ in 0..n {
                let j = (i + 1) % n;
                let cand = g.forward_v2(path, i, ws.v[i]).sqrt();
                if cand < ws.v[j] {
                    ws.v[j] = cand;
                }
                i = j;
            }
            // Backward sweep, one full wrap ending at the slowest point.
            let mut i = i0;
            for _ in 0..n {
                let j = (i + n - 1) % n;
                let cand = g.backward_v2(path, i, ws.v[i]).sqrt();
                if cand < ws.v[j] {
                    ws.v[j] = cand;
                }
                i = j;
            }
            if (ws.v[i0] - before).abs() <= SEED_TOL * before.max(1.0) {
                converged = true;
                break;
            }
        }
        if !converged {
            return Err(T0Error::PassesDiverged {
                iterations: MAX_PASS_ITERS,
            });
        }
    } else {
        // Open path: standing start, single forward then backward sweep.
        ws.v[0] = 0.0;
        for i in 0..n - 1 {
            let cand = g.forward_v2(path, i, ws.v[i]).sqrt();
            if cand < ws.v[i + 1] {
                ws.v[i + 1] = cand;
            }
        }
        for i in (1..n).rev() {
            let cand = g.backward_v2(path, i, ws.v[i]).sqrt();
            if cand < ws.v[i - 1] {
                ws.v[i - 1] = cand;
            }
        }
    }

    Ok(lap_time(path, &ws.v))
}

/// Run the T0 passes on the constant-μ friction ellipse into a caller-owned workspace, returning the
/// lap time (seconds). Zero-allocation.
///
/// # Errors
/// [`T0Error::WorkspaceMismatch`] if `ws` is sized differently from `path`, or
/// [`T0Error::PassesDiverged`] if a closed lap fails to reach a fixed point.
pub fn solve_into(veh: &T0Vehicle, path: &T0Path, ws: &mut T0Workspace) -> Result<f64, T0Error> {
    solve_generic(&EllipseGrip { veh }, path, ws)
}

/// Run the T0 passes on a T1-derived g-g-g-v envelope into a caller-owned workspace, returning the
/// lap time (seconds). The envelope supplies the tyre-grip boundary; the vehicle supplies mass, the
/// powertrain ceiling, drag, and the speed cap. Zero-allocation.
///
/// # Errors
/// As [`solve_into`].
pub fn solve_into_ggv(
    veh: &T0Vehicle,
    env: &GgvEnvelope,
    path: &T0Path,
    ws: &mut T0Workspace,
) -> Result<f64, T0Error> {
    solve_generic(&GgvGrip::new(veh, env), path, ws)
}

/// As [`solve_into_ggv`] but with a per-station **traction scale** (`scale.len() == path.len()`),
/// the machine-thermal derate ∧ battery peak-power cap the QSS slow-state coupling marches along the
/// previous pass. `scale[i] ∈ [0, 1]` multiplies the powertrain traction ceiling in the forward
/// step; braking is unaffected. Zero-allocation.
///
/// # Errors
/// As [`solve_into`]; also [`T0Error::WorkspaceMismatch`] if `scale` is not `path.len()` long.
pub fn solve_into_ggv_scaled(
    veh: &T0Vehicle,
    env: &GgvEnvelope,
    scale: &[f64],
    path: &T0Path,
    ws: &mut T0Workspace,
) -> Result<f64, T0Error> {
    if scale.len() != path.len() {
        return Err(T0Error::WorkspaceMismatch {
            workspace: scale.len(),
            path: path.len(),
        });
    }
    solve_generic(&GgvGrip::with_scale(veh, env, scale), path, ws)
}

/// Central segment longitudinal acceleration `(v_{i+1}² − v_i²)/2ds` at each station into `ax_out`
/// (`ax_out.len() == path.len()`; the last open-path station keeps its prior value, 0 by default).
/// Shared by the owning wrappers and the slow-state coupling march.
pub(crate) fn derive_ax(path: &T0Path, v: &[f64], ax_out: &mut [f64]) {
    let n = v.len();
    for seg in 0..path.segments() {
        let i = seg;
        let j = if path.closed { (seg + 1) % n } else { seg + 1 };
        ax_out[i] = (v[j] * v[j] - v[i] * v[i]) / (2.0 * path.ds);
    }
}

/// Lap time from a solved speed profile: `Σ 2·ds/(v_i + v_{i+1})` (fixed-order).
fn lap_time(path: &T0Path, v: &[f64]) -> f64 {
    let n = v.len();
    let mut t = 0.0;
    for seg in 0..path.segments() {
        let i = seg;
        let j = if path.closed { (seg + 1) % n } else { seg + 1 };
        t += 2.0 * path.ds / (v[i] + v[j]).max(1e-6);
    }
    t
}

/// Solve a lap on the constant-μ ellipse, returning an owned [`LapResult`] with the SoA channels
/// (allocates).
///
/// # Errors
/// As [`solve_into`].
pub fn solve_lap(
    veh: &T0Vehicle,
    path: &T0Path,
    line: LineDescriptor,
    resolved_hash: String,
    notes: Vec<String>,
) -> Result<LapResult, T0Error> {
    let mut ws = T0Workspace::for_path(path);
    let lap_time_s = solve_into(veh, path, &mut ws)?;
    Ok(lap_result_from_ws(
        path,
        &ws,
        lap_time_s,
        line,
        resolved_hash,
        notes,
    ))
}

/// Solve a lap on a T1-derived g-g-g-v envelope, returning an owned [`LapResult`] with the SoA
/// point-mass channels (allocates). The owning convenience wrapper around [`solve_into_ggv`] — the
/// T0-on-envelope production path (`sim.tier = t0`).
///
/// # Errors
/// As [`solve_into`].
pub fn solve_lap_ggv(
    veh: &T0Vehicle,
    env: &GgvEnvelope,
    path: &T0Path,
    line: LineDescriptor,
    resolved_hash: String,
    notes: Vec<String>,
) -> Result<LapResult, T0Error> {
    let mut ws = T0Workspace::for_path(path);
    let lap_time_s = solve_into_ggv(veh, env, path, &mut ws)?;
    Ok(lap_result_from_ws(
        path,
        &ws,
        lap_time_s,
        line,
        resolved_hash,
        notes,
    ))
}

/// Derive the point-mass SoA channels (`ax`, `ay`, `t`) from a solved speed profile and pack them
/// into an owned [`LapResult`]. Shared by every T0-flavoured owning wrapper (ellipse and envelope).
/// `ay` is the velocity-frame lateral demand `κ_l·v² + g·sinθ_b·cosθ_g`; `ax` is the central
/// segment acceleration `(v_{i+1}² − v_i²)/2ds`.
pub(crate) fn lap_result_from_ws(
    path: &T0Path,
    ws: &T0Workspace,
    lap_time_s: f64,
    line: LineDescriptor,
    resolved_hash: String,
    notes: Vec<String>,
) -> LapResult {
    let n = path.len();
    let mut ax = vec![0.0; n];
    let mut ay = vec![0.0; n];
    let mut t = vec![0.0; n];
    for i in 0..n {
        let u = ws.v[i] * ws.v[i];
        ay[i] = path.kappa_l[i] * u + G * path.sin_b_cos_g[i];
    }
    derive_ax(path, &ws.v, &mut ax);
    for i in 1..n {
        t[i] = t[i - 1] + 2.0 * path.ds / (ws.v[i - 1] + ws.v[i]).max(1e-6);
    }
    LapResult {
        s: path.s.clone(),
        v: ws.v.clone(),
        ax,
        ay,
        t,
        lap_time_s,
        line,
        resolved_hash,
        notes,
    }
}
