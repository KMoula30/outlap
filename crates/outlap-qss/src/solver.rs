// SPDX-License-Identifier: AGPL-3.0-only
//! The T0 forward/backward velocity-profile solver (§6.1, §11.2).
//!
//! Not an ODE integration: a curvature-limited speed per station (closed form) followed by a
//! traction-limited forward sweep and a braking-limited backward sweep over a constant-μ friction
//! ellipse, taking the pointwise minimum. Re-implemented from the TUM `calc_vel_profile`
//! formulation (Heilmeier et al., *Vehicle System Dynamics* 58(10), 2020) on the 3D ribbon
//! (Perantoni & Limebeer 2015; Lovato & Massaro 2022). The point-mass equations:
//!
//! * `N(s,v)  = m·(g·cosθ_b·cosθ_g + κ_n·v²) + q_z·v²`  (normal load; crest unloads, dip loads)
//! * `F_y(s,v) = m·(κ_l·v² + g·sinθ_b·cosθ_g)`           (lateral demand; banking assists)
//! * `m·v̇ = F_t − q_x·v² − m·g·sinθ_g`                   (longitudinal)
//! * friction ellipse `(F_t/(μ_x γ N))² + (F_y/(μ_y γ N))² ≤ 1`
//!
//! `[`solve_into`]` is the zero-allocation kernel (it writes into a caller-owned [`T0Workspace`]);
//! [`solve_lap`] is the owning convenience wrapper.

use crate::error::T0Error;
use crate::path::T0Path;
use crate::result::{LapResult, LineDescriptor, T0Workspace};
use crate::vehicle::T0Vehicle;
use crate::G;

/// Slope/denominator guard for the closed-form comparisons.
const EPS: f64 = 1e-9;
/// Iteration cap for the closed-lap pass fixed point (a divergence backstop).
const MAX_PASS_ITERS: usize = 8;
/// Relative tolerance for closed-lap seed convergence.
const SEED_TOL: f64 = 1e-6;

/// Curvature-limited speed at station `i`, m/s (closed form: both sides of the lateral friction
/// constraint are affine in `u = v²`).
fn v_limit(veh: &T0Vehicle, p: &T0Path, i: usize) -> f64 {
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
fn forward_v2(veh: &T0Vehicle, p: &T0Path, i: usize, v_i: f64) -> f64 {
    let f_rem = long_grip_remaining(veh, p, i, v_i);
    let f_t = veh.tractive_force(v_i).min(f_rem);
    let u = v_i * v_i;
    let accel = (f_t - veh.qx * u - veh.mass_kg * G * p.sin_g[i]) / veh.mass_kg;
    (u + 2.0 * p.ds * accel).max(0.0)
}

/// Backward (braking-limited) step from station `ip1` at `v_ip1` over one `ds`: candidate `v²` at
/// the earlier station.
fn backward_v2(veh: &T0Vehicle, p: &T0Path, ip1: usize, v_ip1: f64) -> f64 {
    let f_rem = long_grip_remaining(veh, p, ip1, v_ip1);
    let u = v_ip1 * v_ip1;
    // Drag and (uphill) gravity add to the braking budget.
    let a_dec = (f_rem + veh.qx * u + veh.mass_kg * G * p.sin_g[ip1]) / veh.mass_kg;
    (u + 2.0 * p.ds * a_dec).max(0.0)
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

/// Run the T0 passes into a caller-owned workspace, returning the lap time (seconds).
///
/// Zero-allocation: touches only the workspace slices, the path slices, and the vehicle's
/// allocation-free [`T0Vehicle::tractive_force`].
///
/// # Errors
/// [`T0Error::WorkspaceMismatch`] if `ws` is sized differently from `path`, or
/// [`T0Error::PassesDiverged`] if a closed lap fails to reach a fixed point.
pub fn solve_into(veh: &T0Vehicle, path: &T0Path, ws: &mut T0Workspace) -> Result<f64, T0Error> {
    let n = path.len();
    if ws.len() != n {
        return Err(T0Error::WorkspaceMismatch {
            workspace: ws.len(),
            path: n,
        });
    }

    for i in 0..n {
        ws.v_lim[i] = v_limit(veh, path, i);
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
                let cand = forward_v2(veh, path, i, ws.v[i]).sqrt();
                if cand < ws.v[j] {
                    ws.v[j] = cand;
                }
                i = j;
            }
            // Backward sweep, one full wrap ending at the slowest point.
            let mut i = i0;
            for _ in 0..n {
                let j = (i + n - 1) % n;
                let cand = backward_v2(veh, path, i, ws.v[i]).sqrt();
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
            let cand = forward_v2(veh, path, i, ws.v[i]).sqrt();
            if cand < ws.v[i + 1] {
                ws.v[i + 1] = cand;
            }
        }
        for i in (1..n).rev() {
            let cand = backward_v2(veh, path, i, ws.v[i]).sqrt();
            if cand < ws.v[i - 1] {
                ws.v[i - 1] = cand;
            }
        }
    }

    Ok(lap_time(path, &ws.v))
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

/// Solve a lap, returning an owned [`LapResult`] with the SoA channels (allocates).
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

    let n = path.len();
    let mut ax = vec![0.0; n];
    let mut ay = vec![0.0; n];
    let mut t = vec![0.0; n];
    for i in 0..n {
        let u = ws.v[i] * ws.v[i];
        ay[i] = path.kappa_l[i] * u + G * path.sin_b_cos_g[i];
    }
    for seg in 0..path.segments() {
        let i = seg;
        let j = if path.closed { (seg + 1) % n } else { seg + 1 };
        ax[i] = (ws.v[j] * ws.v[j] - ws.v[i] * ws.v[i]) / (2.0 * path.ds);
    }
    for i in 1..n {
        t[i] = t[i - 1] + 2.0 * path.ds / (ws.v[i - 1] + ws.v[i]).max(1e-6);
    }

    Ok(LapResult {
        s: path.s.clone(),
        v: ws.v.clone(),
        ax,
        ay,
        t,
        lap_time_s,
        line,
        resolved_hash,
        notes,
    })
}
