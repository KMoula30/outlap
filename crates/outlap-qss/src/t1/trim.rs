// SPDX-License-Identifier: AGPL-3.0-only
//! T1 quasi-steady-state double-track **trim** solver (§6.1; Perantoni & Limebeer 2014;
//! Lovato & Massaro 2022 for the g-g framing; Pacejka 2012 / Guiggiani 2018 for the load transfer).
//!
//! Given a commanded operating point `(v, a_y, a_x)` the trim finds the steady-state chassis state
//! that produces exactly those CG accelerations in planar force/moment balance, with per-wheel tyre
//! forces from the shared [`TireModel`](outlap_tire::TireModel). It is a damped Newton solve of a
//! 9-unknown, 9-residual algebraic system — zero-allocation (fixed stack arrays), panic-free
//! ([`TrimOutcome::Infeasible`] flags an unreachable operating point, consumed by the envelope
//! generator as a boundary, never a panic).
//!
//! # Unknowns `z`  (ISO 8855: x forward, y left, z up; SI)
//!
//! | index | symbol | meaning |
//! |-------|--------|---------|
//! | 0 | `δ`   | front road-wheel steer angle, rad |
//! | 1 | `β`   | body-slip angle (velocity vs body x), rad |
//! | 2 | `r`   | yaw rate, rad/s |
//! | 3 | `s`   | longitudinal-slip control (drive if > 0, brake if < 0) |
//! | 4 | `w`   | driven-axle slip split: `κ_left = s + w`, `κ_right = s − w` (PR4 diff) |
//! | 5–8 | `F_z,i` | per-wheel normal loads `[FL, FR, RL, RR]`, N |
//!
//! # Residuals `R`
//!
//! - `ΣF_x = m·a_x` — longitudinal balance (tyre + aero drag).
//! - `ΣF_y = m·a_y` — lateral balance.
//! - `ΣM_z = 0` — yaw-moment balance (steady state ⇒ ṙ = 0).
//! - `r·v = a_y·cos β − a_x·sin β` — steady-state kinematic closure (the yaw rate that makes the
//!   body-frame CG acceleration equal the commanded `(a_x, a_y)`).
//! - **differential law** (the 9th residual, §8.2): **open** ⇒ equal torque on the driven axle's
//!   two wheels (`F_{x,left} = F_{x,right}`, `w` free); **locked/solid/LSD** ⇒ equal speed (`w = 0`;
//!   a well-preloaded LSD locks at the traction limit, so LSD reduces to the locked constraint here
//!   and its preload/ramp bound the reported split). Under braking the diff is inactive (`w = 0`).
//! - `F_z,i = F_z,i^pred` (×4) — quasi-static load transfer: static + downforce + longitudinal pitch
//!   transfer + per-axle lateral transfer via roll-centre geometry and roll-stiffness distribution.

use outlap_schema::sim::FzCoupling;
use outlap_schema::vehicle::DiffKind;
use outlap_tire::{SlipState, TireModel};

use crate::t1::vehicle::T1Vehicle;

/// Number of trim unknowns/residuals.
const N: usize = 9;
/// Index of the first per-wheel normal-load unknown/residual (`F_z` occupies `FZ0..FZ0+4`).
const FZ0: usize = 5;
/// Maximum Newton iterations before an operating point is declared infeasible.
const MAX_NEWTON: usize = 80;
/// Maximum backtracking-line-search halvings per Newton step.
const MAX_LINE_SEARCH: usize = 40;
/// Scaled-residual convergence tolerance (dimensionless; residuals normalised by `m·g` etc.).
const TOL: f64 = 1e-10;
/// Iteration window for the infeasibility stall test (see `solve_lm`).
const STALL_WINDOW: usize = 6;
/// Required residual-reduction factor per stall window: a converging LM drops `‖R‖` by orders of
/// magnitude every few iterations, while an *infeasible* target parks the solver at a nonzero
/// least-squares minimum where each step shaves only a microscopic strictly-positive amount. If a
/// whole window passes without at least this reduction (and the residual is still far from
/// tolerance), the solve is stalled at that minimum — the commanded point is infeasible; stop
/// immediately instead of burning the remaining Newton budget (the boundary bisection makes ~half
/// its probes infeasible by construction, so this is the envelope generator's dominant cost).
const STALL_FACTOR: f64 = 0.7;
/// Residual floor for the stall test: only declare a stall well above the convergence tolerance so
/// a near-root solve with a slow tail can never be cut short.
const STALL_MIN_RN: f64 = 1e3 * TOL;
/// Relative step for the finite-difference Jacobian.
const FD_H: f64 = 1e-7;
/// Minimum speed for a meaningful QSS trim, m/s. The steady-cornering kinematics divide by `v`
/// (yaw rate `r = a_y/v`), so a stationary or crawling car has no well-posed g-g trim; the envelope
/// generator samples speeds above this floor.
const V_MIN: f64 = 0.5;
/// Initial homotopy-continuation step in the `[0, 1]` ramp parameter (attempt the whole ramp first).
const CONTINUATION_DT0: f64 = 1.0;
/// Growth factor applied to the continuation step after a successful sub-solve.
const CONTINUATION_GROW: f64 = 1.6;
/// Maximum continuation step.
const CONTINUATION_DT_MAX: f64 = 0.5;
/// Smallest continuation step; shrinking below this means the target is past the friction boundary.
const CONTINUATION_DT_MIN: f64 = 1e-3;
/// Backstop on continuation sub-solves (bounds worst-case work; never hit on feasible targets).
const MAX_CONTINUATION_STEPS: usize = 400;

/// A commanded quasi-steady-state operating point.
#[derive(Clone, Copy, Debug)]
pub struct TrimInput {
    /// Speed (magnitude of the CG velocity), m/s. Must be positive.
    pub v: f64,
    /// Lateral CG acceleration (ISO 8855, `+` left), m/s².
    pub ay: f64,
    /// Longitudinal CG acceleration (`+` forward), m/s².
    pub ax: f64,
    /// Normal gravity component (banking/grade; the envelope's `g_normal` axis), m/s².
    pub g_normal: f64,
    /// Normal-load coupling mode (Decision #29), recorded in the result.
    pub coupling: FzCoupling,
}

impl TrimInput {
    /// A flat-ground operating point at standard gravity with the default coupling.
    pub fn flat(v: f64, ay: f64, ax: f64) -> Self {
        Self {
            v,
            ay,
            ax,
            g_normal: crate::G,
            coupling: FzCoupling::OneStepLag,
        }
    }
}

/// A converged trim state (per-wheel channels are in `[FL, FR, RL, RR]` order, ISO 8855 body frame).
#[derive(Clone, Copy, Debug)]
pub struct TrimState {
    /// Front road-wheel steer angle `δ`, rad.
    pub delta: f64,
    /// Body-slip angle `β`, rad.
    pub beta: f64,
    /// Yaw rate `r`, rad/s.
    pub yaw_rate: f64,
    /// Longitudinal-slip control `s` (drive > 0, brake < 0).
    pub long_ctrl: f64,
    /// Per-wheel normal load, N.
    pub fz: [f64; 4],
    /// Per-wheel longitudinal slip `κ`.
    pub kappa: [f64; 4],
    /// Per-wheel slip angle `α`, rad.
    pub alpha: [f64; 4],
    /// Per-wheel longitudinal force in the body frame, N.
    pub fx: [f64; 4],
    /// Per-wheel lateral force in the body frame, N.
    pub fy: [f64; 4],
    /// Scaled residual norm at convergence.
    pub residual_norm: f64,
    /// Newton iterations taken.
    pub iterations: usize,
    /// The coupling mode used (recorded, Decision #29).
    pub coupling: FzCoupling,
}

/// The outcome of a trim solve.
#[derive(Clone, Copy, Debug)]
pub enum TrimOutcome {
    /// The operating point is feasible; here is the converged state.
    Converged(TrimState),
    /// No steady state produces `(v, a_y, a_x)` within the friction envelope (a boundary point).
    Infeasible {
        /// Scaled residual norm reached before giving up.
        residual_norm: f64,
        /// Newton iterations taken.
        iterations: usize,
    },
}

impl TrimOutcome {
    /// The converged state, or `None` if the point was infeasible.
    pub fn state(&self) -> Option<&TrimState> {
        match self {
            TrimOutcome::Converged(s) => Some(s),
            TrimOutcome::Infeasible { .. } => None,
        }
    }

    /// Whether the operating point was feasible.
    pub fn is_feasible(&self) -> bool {
        matches!(self, TrimOutcome::Converged(_))
    }
}

/// Per-wheel body-frame geometry, precomputed for a [`T1Vehicle`] (`[FL, FR, RL, RR]`).
struct Geom {
    /// Longitudinal position relative to the CG (`+` forward), m.
    x: [f64; 4],
    /// Lateral position relative to the CG (`+` left), m.
    y: [f64; 4],
    /// Whether each wheel is on the front (steered) axle.
    front: [bool; 4],
}

impl Geom {
    fn new(car: &T1Vehicle) -> Self {
        Self {
            x: [car.a_f, car.a_f, -car.b_r, -car.b_r],
            y: [car.t_f * 0.5, -car.t_f * 0.5, car.t_r * 0.5, -car.t_r * 0.5],
            front: [true, true, false, false],
        }
    }
}

impl T1Vehicle {
    /// Solve the trim for a commanded operating point. Zero-allocation and panic-free.
    ///
    /// Fast path: a direct Levenberg–Marquardt solve from the point-mass warm start — succeeds for
    /// the vast majority of operating points. Robust fallback: **homotopy continuation** from the
    /// trivial straight-line trim `(a_y, a_x) = (0, 0)`, ramping the target accelerations with an
    /// adaptive step, warm-starting each sub-solve from the previous one. Continuation converges at
    /// arbitrarily tight low-speed corners and, when the ramp cannot reach the target, returns
    /// [`TrimOutcome::Infeasible`] — the last feasible point on the ramp is the friction boundary.
    pub fn trim(&self, inp: &TrimInput) -> TrimOutcome {
        // The QSS kinematics divide by `v`; a stationary/crawling car (or a NaN/negative speed or
        // gravity) has no well-posed trim.
        let bad_speed = !inp.v.is_finite() || inp.v <= V_MIN;
        let bad_gravity = !inp.g_normal.is_finite() || inp.g_normal <= 0.0;
        if bad_speed || bad_gravity {
            return TrimOutcome::Infeasible {
                residual_norm: f64::INFINITY,
                iterations: 0,
            };
        }
        let geom = Geom::new(self);

        // Fast path: direct solve from the physics warm start.
        let z0 = self.initial_guess(inp);
        let (z, rn, iters) = self.solve_lm(inp, &geom, z0);
        if rn <= TOL {
            return TrimOutcome::Converged(self.finalize(inp, &geom, &z, rn, iters));
        }

        // Robust path: homotopy continuation from the straight-line base state.
        self.trim_continuation(inp, &geom, iters)
    }

    /// A single direct trim solve seeded from an optional warm state, **without** the homotopy
    /// continuation fallback. This is the boundary-search primitive the g-g-g-v envelope generator
    /// marches: walking `a_y` upward from a feasible state and warm-starting each probe from the
    /// previous solution keeps every solve in the LM fast path, so feasible points converge and
    /// infeasible ones (past the friction boundary) fail fast — orders of magnitude cheaper than
    /// running the full continuation on the many infeasible probes a bisection makes. Zero-allocation,
    /// panic-free.
    pub(crate) fn trim_warm(&self, inp: &TrimInput, warm: Option<&TrimState>) -> TrimOutcome {
        let bad_speed = !inp.v.is_finite() || inp.v <= V_MIN;
        let bad_gravity = !inp.g_normal.is_finite() || inp.g_normal <= 0.0;
        if bad_speed || bad_gravity {
            return TrimOutcome::Infeasible {
                residual_norm: f64::INFINITY,
                iterations: 0,
            };
        }
        let geom = Geom::new(self);
        let z0 = warm.map_or_else(|| self.initial_guess(inp), |w| self.warm_z(inp, w));
        let (z, rn, iters) = self.solve_lm(inp, &geom, z0);
        if rn <= TOL {
            TrimOutcome::Converged(self.finalize(inp, &geom, &z, rn, iters))
        } else {
            TrimOutcome::Infeasible {
                residual_norm: rn,
                iterations: iters,
            }
        }
    }

    /// Reconstruct an unknown vector `z` from a converged [`TrimState`] to warm-start a nearby solve
    /// (the `F_z` components are re-non-dimensionalised by `m·g_normal`; the driven-axle split `w`
    /// starts even and the LM refines it).
    fn warm_z(&self, inp: &TrimInput, w: &TrimState) -> [f64; N] {
        let mg = self.mass_kg * inp.g_normal;
        [
            w.delta,
            w.beta,
            w.yaw_rate,
            w.long_ctrl,
            0.0,
            w.fz[0] / mg,
            w.fz[1] / mg,
            w.fz[2] / mg,
            w.fz[3] / mg,
        ]
    }

    /// Homotopy continuation of the trim: solve the easy straight-line state, then ramp the target
    /// `(a_y, a_x)` with an adaptive continuation parameter, re-solving from the previous solution.
    fn trim_continuation(&self, inp: &TrimInput, geom: &Geom, mut iters: usize) -> TrimOutcome {
        // Base state: straight running at this speed (a_y = a_x = 0) is near-linear and always trims.
        let base_input = TrimInput {
            ay: 0.0,
            ax: 0.0,
            ..*inp
        };
        let base_guess = self.initial_guess(&base_input);
        let (mut z, base_rn, base_iters) = self.solve_lm(&base_input, geom, base_guess);
        iters += base_iters;
        if base_rn > TOL {
            return TrimOutcome::Infeasible {
                residual_norm: base_rn,
                iterations: iters,
            };
        }

        // Walk the continuation parameter `t` from 0 to 1, scaling the target accelerations.
        let mut t = 0.0;
        let mut dt = CONTINUATION_DT0;
        let mut steps = 0;
        while t < 1.0 {
            steps += 1;
            if steps > MAX_CONTINUATION_STEPS {
                return TrimOutcome::Infeasible {
                    residual_norm: f64::INFINITY,
                    iterations: iters,
                };
            }
            let t_try = (t + dt).min(1.0);
            let sub = TrimInput {
                ay: inp.ay * t_try,
                ax: inp.ax * t_try,
                ..*inp
            };
            let (z_new, rn, sub_iters) = self.solve_lm(&sub, geom, z);
            iters += sub_iters;
            if rn <= TOL {
                t = t_try;
                z = z_new;
                dt = (dt * CONTINUATION_GROW).min(CONTINUATION_DT_MAX); // accelerate on success
            } else {
                dt *= 0.5; // shrink and retry the increment
                if dt < CONTINUATION_DT_MIN {
                    // Cannot advance the ramp → the target is past the friction boundary.
                    return TrimOutcome::Infeasible {
                        residual_norm: rn,
                        iterations: iters,
                    };
                }
            }
        }
        // `t == 1`: `z` is the converged trim at the full target.
        let mut r = [0.0; N];
        self.residual(inp, geom, &z, &mut r, &mut None);
        TrimOutcome::Converged(self.finalize(inp, geom, &z, norm(&r), iters))
    }

    /// One Levenberg–Marquardt solve from an initial guess `z`. Returns `(z, ‖R‖, iterations)`.
    ///
    /// LM (Marquardt diagonal scaling) interpolates between Gauss–Newton near the root and gradient
    /// descent when far; the trial state is clamped to physical bounds so the search cannot wander
    /// into the periodic-`β` aliases that trap a plain Newton. Zero-allocation (fixed stack arrays).
    fn solve_lm(&self, inp: &TrimInput, geom: &Geom, mut z: [f64; N]) -> ([f64; N], f64, usize) {
        clamp_state(&mut z);
        // Aero-platform warm-start slot, threaded through every residual evaluation of this solve.
        let mut aero_h: Option<(f64, f64)> = None;
        let mut r = [0.0; N];
        self.residual(inp, geom, &z, &mut r, &mut aero_h);
        let mut rn = norm(&r);
        let mut mu = 1e-3;
        let mut iterations = 0;
        // Infeasibility stall test state: the residual at the start of the current window.
        let mut rn_window = rn;
        let mut window_iters = 0;
        while iterations < MAX_NEWTON && rn > TOL {
            iterations += 1;
            // Finite-difference Jacobian J (column j = ∂R/∂z_j).
            let mut jac = [[0.0; N]; N];
            let mut zp = z;
            let mut rp = [0.0; N];
            for j in 0..N {
                let h = FD_H * z[j].abs().max(1.0);
                zp[j] = z[j] + h;
                self.residual(inp, geom, &zp, &mut rp, &mut aero_h);
                for i in 0..N {
                    jac[i][j] = (rp[i] - r[i]) / h;
                }
                zp[j] = z[j];
            }
            // Normal equations: A = JᵀJ, g = Jᵀr.
            let mut a = [[0.0; N]; N];
            let mut g = [0.0; N];
            for i in 0..N {
                for j in 0..N {
                    let mut acc = 0.0;
                    for k in 0..N {
                        acc += jac[k][i] * jac[k][j];
                    }
                    a[i][j] = acc;
                }
                let mut gi = 0.0;
                for k in 0..N {
                    gi += jac[k][i] * r[k];
                }
                g[i] = gi;
            }
            // Inner loop: grow `μ` until the damped step reduces the residual (or give up).
            let mut accepted = false;
            for _ in 0..MAX_LINE_SEARCH {
                let mut damped = a;
                for i in 0..N {
                    damped[i][i] += mu * a[i][i].max(1e-12); // Marquardt scaling
                }
                let mut rhs = [0.0; N];
                for i in 0..N {
                    rhs[i] = -g[i];
                }
                let Some(dz) = solve_linear(&mut damped, &rhs) else {
                    mu *= 4.0;
                    if mu > 1e14 {
                        break;
                    }
                    continue;
                };
                let mut ztry = z;
                for k in 0..N {
                    ztry[k] += dz[k];
                }
                clamp_state(&mut ztry); // keep the search physical (no periodic-β aliases, Fz ≥ 0)
                let mut rtry = [0.0; N];
                self.residual(inp, geom, &ztry, &mut rtry, &mut aero_h);
                let rntry = norm(&rtry);
                if rntry < rn {
                    z = ztry;
                    r = rtry;
                    rn = rntry;
                    mu = (mu * 0.3).max(1e-12); // trust the model more next step
                    accepted = true;
                    break;
                }
                mu *= 4.0; // damp harder and retry
                if mu > 1e14 {
                    break;
                }
            }
            if !accepted {
                break; // cannot reduce the residual further
            }
            // Infeasibility stall test: strict decrease alone is not progress — an infeasible
            // target converges to a nonzero-residual least-squares minimum where every LM step
            // still reduces `‖R‖` by a vanishing amount. Cut the solve as soon as a full window
            // passes without a real reduction (never near the tolerance, where a slow tail on a
            // genuinely feasible point must be allowed to finish).
            window_iters += 1;
            if window_iters >= STALL_WINDOW {
                if rn > STALL_MIN_RN && rn > STALL_FACTOR * rn_window {
                    break; // stalled at a nonzero-residual minimum ⇒ infeasible
                }
                rn_window = rn;
                window_iters = 0;
            }
        }
        (z, rn, iterations)
    }

    /// Understeer gradient `K = dδ/da_y − L/v²` at speed `v` (rad per m/s²): `> 0` understeer,
    /// `< 0` oversteer. Central-differenced from two small-`a_y` trims at zero longitudinal accel.
    ///
    /// Returns `None` if either probe trim is infeasible.
    pub fn understeer_gradient(&self, v: f64, g_normal: f64) -> Option<f64> {
        let ay = 1.0; // a small, linear-regime lateral acceleration, m/s²
        let probe = |a: f64| {
            self.trim(&TrimInput {
                v,
                ay: a,
                ax: 0.0,
                g_normal,
                coupling: FzCoupling::OneStepLag,
            })
            .state()
            .map(|s| s.delta)
        };
        let dp = probe(ay)?;
        let dm = probe(-ay)?;
        Some((dp - dm) / (2.0 * ay) - self.wheelbase_m / (v * v))
    }

    /// Aero balance: the front axle's share of total downforce, 0..1, at the **reference** platform
    /// (constant terms). With a ride-height map installed this is speed-invariant; use
    /// [`Self::aero_front_downforce_share_at`] for the speed-dependent balance the map produces.
    pub fn aero_front_downforce_share(&self) -> f64 {
        share(self.qz_f, self.qz_r)
    }

    /// Aero balance at speed `v` (m/s): the front axle's share of total downforce, 0..1, at the
    /// aero-platform equilibrium (straight running, `a_x = 0`). Speed-dependent when a ride-height
    /// map is installed (the platform rakes with downforce); equals the reference share otherwise.
    pub fn aero_front_downforce_share_at(&self, v: f64) -> f64 {
        let aero = self.aero_lumped(v, 0.0, 0.0, 0.0);
        share(aero.qz_f, aero.qz_r)
    }

    /// The residual vector `R(z)` for a commanded operating point (scaled dimensionless).
    ///
    /// `aero_h` is the warm-start slot for the aero-platform fixed point: it carries the converged
    /// ride heights from the previous residual evaluation of the same solve (β moves little between
    /// evaluations, so the warm fixed point converges in ~1–2 iterations instead of ~20; the tight
    /// `AERO_TOL_M` makes the result physically identical either way).
    fn residual(
        &self,
        inp: &TrimInput,
        geom: &Geom,
        z: &[f64; N],
        out: &mut [f64; N],
        aero_h: &mut Option<(f64, f64)>,
    ) {
        let (delta, beta, yaw_rate, s, w) = (z[0], z[1], z[2], z[3], z[4]);
        let v = inp.v;
        // The four F_z unknowns are non-dimensionalised by `m·g` (all unknowns then O(1), so the
        // finite-difference Jacobian is well-conditioned despite mixing rad and newtons).
        let mg = self.mass_kg * inp.g_normal;
        let (mut sum_fx, mut sum_fy, mut sum_mz) = (0.0, 0.0, 0.0);
        // Wheel-frame longitudinal forces of the primary driven axle's two wheels (for the diff law).
        let (mut fxw_left, mut fxw_right) = (0.0, 0.0);
        // Per-wheel tyre forces in the body frame.
        for i in 0..4 {
            let steer = if geom.front[i] { delta } else { 0.0 };
            let (cs, sn) = (steer.cos(), steer.sin());
            // Contact-point velocity in the body frame: V + ω × r_i.
            let vx_b = v * beta.cos() - yaw_rate * geom.y[i];
            let vy_b = v * beta.sin() + yaw_rate * geom.x[i];
            // Rotate into the wheel frame.
            let vxw = vx_b * cs + vy_b * sn;
            let vyw = -vx_b * sn + vy_b * cs;
            let alpha = vyw.atan2(vxw.abs()); // tan α = V_sy / |V_cx|  (ISO-W sign contract)
            let kappa = self.wheel_slip(s, w, i);
            let (model, p) = if geom.front[i] {
                (&self.tire_front, self.p_front)
            } else {
                (&self.tire_rear, self.p_rear)
            };
            let f = tire_forces(model, kappa, alpha, z[FZ0 + i] * mg, p, vxw, self.mu_scale);
            // Capture the driven-axle wheel-frame longitudinal forces for the diff residual.
            if let Some(pd) = &self.primary_diff {
                if i == pd.left {
                    fxw_left = f.fx;
                } else if i == pd.right {
                    fxw_right = f.fx;
                }
            }
            // Rotate the wheel-frame force back into the body frame.
            let fx_b = f.fx * cs - f.fy * sn;
            let fy_b = f.fx * sn + f.fy * cs;
            sum_fx += fx_b;
            sum_fy += fy_b;
            sum_mz += geom.x[i] * fy_b - geom.y[i] * fx_b + f.mz;
        }
        // Aero: constant, or the ride-height map's platform equilibrium at this operating point.
        // The aerodynamic yaw is the body-slip angle β (evaluated in degrees); DRS is closed in the
        // trim (its activation is a controller concern). Keeping the evaluation inside the residual
        // lets the finite-difference Jacobian pick up ∂(downforce)/∂β for the mid-corner asymmetry.
        let aero = self.aero_lumped_warm(v, inp.ax, beta.to_degrees(), 0.0, *aero_h);
        *aero_h = Some((aero.h_f_m, aero.h_r_m));
        let drag = aero.qx * v * v;
        sum_fx -= drag;

        // Load-transfer prediction of the four normal loads.
        let (ax_lt, ay_lt) = match inp.coupling {
            FzCoupling::OneStepLag => (inp.ax, inp.ay),
            FzCoupling::FixedPoint => (sum_fx / self.mass_kg, sum_fy / self.mass_kg),
        };
        let fz_pred = self.load_transfer(inp, ax_lt, ay_lt, aero.qz_f, aero.qz_r);

        // Scales: forces by m·g, moment by m·g·L, kinematic by g.
        let g = inp.g_normal;
        let f_scale = 1.0 / (self.mass_kg * g);
        let m_scale = f_scale / self.wheelbase_m;
        let k_scale = 1.0 / g;

        out[0] = (sum_fx - self.mass_kg * inp.ax) * f_scale;
        out[1] = (sum_fy - self.mass_kg * inp.ay) * f_scale;
        out[2] = sum_mz * m_scale;
        out[3] = (yaw_rate * v - (inp.ay * beta.cos() - inp.ax * beta.sin())) * k_scale;
        // Differential law (§8.2): open ⇒ equal driven-wheel torque (equal wheel-frame Fx); locked/
        // solid/LSD ⇒ equal speed (w = 0). Under braking the diff is inactive (the balance bar splits
        // brake torque), so w is pinned to 0. Absent a driven axle pair the split is 0 (baseline).
        out[4] = self.diff_residual(s, w, fxw_left, fxw_right, f_scale);
        // F_z residuals in the same non-dimensional units as the unknowns (Fz/(m·g)).
        for i in 0..4 {
            out[FZ0 + i] = z[FZ0 + i] - fz_pred[i] * f_scale;
        }
    }

    /// The differential-law residual (§8.2). See the residual list in the module header.
    fn diff_residual(&self, s: f64, w: f64, fxw_left: f64, fxw_right: f64, f_scale: f64) -> f64 {
        match &self.primary_diff {
            // Open diff under drive: the two driven wheels carry equal torque ⇒ equal wheel-frame
            // longitudinal force (equal rolling radius). Non-dimensionalised by m·g like the forces.
            Some(pd) if s >= 0.0 && matches!(pd.diff.kind, DiffKind::Open) => {
                (fxw_left - fxw_right) * f_scale
            }
            // Locked / solid / LSD (or braking, or no driven pair): equal speed ⇒ zero slip split.
            _ => w,
        }
    }

    /// Quasi-static normal-load prediction `[FL, FR, RL, RR]` from the load-transfer model, given the
    /// effective per-axle downforce terms `qz_f`/`qz_r` (`½·ρ·C_z·A`) at this operating point.
    fn load_transfer(&self, inp: &TrimInput, ax: f64, ay: f64, qz_f: f64, qz_r: f64) -> [f64; 4] {
        let m = self.mass_kg;
        let l = self.wheelbase_m;
        let g = inp.g_normal;
        let v2 = inp.v * inp.v;
        // Static + downforce, per axle.
        let front_total = m * g * (self.b_r / l) + qz_f * v2;
        let rear_total = m * g * (self.a_f / l) + qz_r * v2;
        // Longitudinal (pitch) transfer: rear gains under forward acceleration.
        let dfz_x = m * ax * self.h_cg / l;
        // Lateral transfer per axle: geometric (roll-centre) + elastic (roll-stiffness share).
        let big_h = self.h_cg - self.h_ra; // sprung CG height above the roll axis
        let m_roll = m * ay * big_h;
        let dfz_y_f = (m * ay * (self.b_r / l) * self.rc_f) / self.t_f
            + self.roll_share_f * m_roll / self.t_f;
        let dfz_y_r = (m * ay * (self.a_f / l) * self.rc_r) / self.t_r
            + self.roll_share_r * m_roll / self.t_r;
        // Axle vertical load after pitch transfer (rear gains under +a_x).
        let front_axle = front_total - dfz_x;
        let rear_axle = rear_total + dfz_x;
        // Split each axle left/right by its lateral transfer (a_y > 0 loads the outside/right
        // wheel). At wheel-lift the lifted wheel floors at 0 and the grounded wheel carries the
        // whole axle — never more than the axle can bear (keeps the g-g boundary from becoming
        // optimistic; ΣF_z stays weight + downforce).
        let (fl, fr) = split_axle(front_axle, dfz_y_f);
        let (rl, rr) = split_axle(rear_axle, dfz_y_r);
        [fl, fr, rl, rr]
    }

    /// Per-wheel longitudinal slip `κ_i` from the controls `(s, w)`. Under drive (`s ≥ 0`) the driven
    /// wheels slip; the primary axle's left/right wheels split by the differential unknown `w`
    /// (`κ_left = s + w`, `κ_right = s − w`) so open vs locked behaviour is realised, and any other
    /// driven wheels (AWD) share the common slip `s`. Under braking (`s < 0`) all wheels brake, split
    /// by the balance bar, and `w` is inactive.
    fn wheel_slip(&self, s: f64, w: f64, i: usize) -> f64 {
        if s >= 0.0 {
            if !self.driven[i] {
                return 0.0;
            }
            if let Some(pd) = &self.primary_diff {
                if i == pd.left {
                    return s + w;
                }
                if i == pd.right {
                    return s - w;
                }
            }
            s
        } else {
            let front = i < 2;
            let bias = if front {
                2.0 * self.brake_front_bias
            } else {
                2.0 * (1.0 - self.brake_front_bias)
            };
            s * bias
        }
    }

    /// A warm initial guess from point-mass kinematics + the direct load-transfer prediction (the
    /// four F_z components are non-dimensionalised by `m·g`, matching the residual's unknowns).
    fn initial_guess(&self, inp: &TrimInput) -> [f64; N] {
        let v = inp.v;
        let delta = (self.wheelbase_m * inp.ay / (v * v)).clamp(-0.5, 0.5);
        let beta = 0.0;
        let yaw_rate = inp.ay / v;
        let s = 0.03 * inp.ax.signum() * f64::from(u8::from(inp.ax != 0.0));
        let w = 0.0; // start from an even (equal-speed) driven-axle split
        let mg = self.mass_kg * inp.g_normal;
        // Warm-start aero at zero yaw (β ≈ 0); the residual refines it against the true β.
        let aero = self.aero_lumped(inp.v, inp.ax, 0.0, 0.0);
        let fz = self.load_transfer(inp, inp.ax, inp.ay, aero.qz_f, aero.qz_r);
        [
            delta,
            beta,
            yaw_rate,
            s,
            w,
            fz[0] / mg,
            fz[1] / mg,
            fz[2] / mg,
            fz[3] / mg,
        ]
    }

    /// Recompute the per-wheel diagnostics for a converged unknown vector.
    fn finalize(
        &self,
        inp: &TrimInput,
        geom: &Geom,
        z: &[f64; N],
        residual_norm: f64,
        iterations: usize,
    ) -> TrimState {
        let (delta, beta, yaw_rate, s, w) = (z[0], z[1], z[2], z[3], z[4]);
        let v = inp.v;
        let mg = self.mass_kg * inp.g_normal;
        let mut st = TrimState {
            delta,
            beta,
            yaw_rate,
            long_ctrl: s,
            fz: [
                z[FZ0] * mg,
                z[FZ0 + 1] * mg,
                z[FZ0 + 2] * mg,
                z[FZ0 + 3] * mg,
            ],
            kappa: [0.0; 4],
            alpha: [0.0; 4],
            fx: [0.0; 4],
            fy: [0.0; 4],
            residual_norm,
            iterations,
            coupling: inp.coupling,
        };
        for i in 0..4 {
            let steer = if geom.front[i] { delta } else { 0.0 };
            let (cs, sn) = (steer.cos(), steer.sin());
            let vx_b = v * beta.cos() - yaw_rate * geom.y[i];
            let vy_b = v * beta.sin() + yaw_rate * geom.x[i];
            let vxw = vx_b * cs + vy_b * sn;
            let vyw = -vx_b * sn + vy_b * cs;
            let alpha = vyw.atan2(vxw.abs());
            let kappa = self.wheel_slip(s, w, i);
            let (model, p) = if geom.front[i] {
                (&self.tire_front, self.p_front)
            } else {
                (&self.tire_rear, self.p_rear)
            };
            let f = tire_forces(model, kappa, alpha, z[FZ0 + i] * mg, p, vxw, self.mu_scale);
            st.kappa[i] = kappa;
            st.alpha[i] = alpha;
            st.fx[i] = f.fx * cs - f.fy * sn;
            st.fy[i] = f.fx * sn + f.fy * cs;
        }
        st
    }
}

/// Evaluate the tyre force model at a contact-patch state (`γ = 0`). `mu_scale` uniformly scales the
/// per-axis friction (1.0 at the reference state; the envelope generator perturbs it for the
/// Decision #31 ∂/∂μ_tire sensitivity).
#[inline]
fn tire_forces(
    model: &TireModel<f64>,
    kappa: f64,
    alpha: f64,
    fz: f64,
    p: f64,
    vx: f64,
    mu_scale: f64,
) -> outlap_tire::TireForces<f64> {
    let mut slip = SlipState::new(kappa, alpha, 0.0, fz, p, vx);
    slip.mu_scale_x = mu_scale;
    slip.mu_scale_y = mu_scale;
    model.forces(&slip)
}

/// Clamp a trim state to physically-generous bounds so the solver cannot wander into unphysical
/// regions (large body slip aliases the periodic trig; the true trim is well interior). Steer/body
/// slip/control are bounded in rad; the `m·g`-normalised loads are floored at 0 and capped generously
/// (high-downforce cars can exceed 1·mg per wheel).
fn clamp_state(z: &mut [f64; N]) {
    z[0] = z[0].clamp(-0.7, 0.7); // δ
    z[1] = z[1].clamp(-0.5, 0.5); // β
    z[2] = z[2].clamp(-8.0, 8.0); // r
    z[3] = z[3].clamp(-0.6, 0.6); // s
    z[4] = z[4].clamp(-0.6, 0.6); // w (driven-axle slip split; an open diff can spin the inside wheel)
    for zi in z.iter_mut().skip(FZ0) {
        *zi = zi.clamp(0.0, 6.0); // Fz/(m·g)
    }
}

/// Split an axle's vertical load `total` left/right by the lateral transfer `transfer` (`+` ⇒ the
/// right wheel gains). Floors each wheel at 0 and never exceeds the axle load: at wheel-lift all of
/// the (non-negative) axle load goes to the grounded wheel. Returns `(left, right)`.
fn split_axle(total: f64, transfer: f64) -> (f64, f64) {
    let total = total.max(0.0);
    let left = total * 0.5 - transfer;
    let right = total * 0.5 + transfer;
    if left < 0.0 {
        (0.0, total)
    } else if right < 0.0 {
        (total, 0.0)
    } else {
        (left, right)
    }
}

/// The front axle's share of total downforce (0.5 when there is no downforce).
fn share(qz_f: f64, qz_r: f64) -> f64 {
    let total = qz_f + qz_r;
    if total > 0.0 {
        qz_f / total
    } else {
        0.5
    }
}

/// Scaled-residual L2 norm.
fn norm(r: &[f64; N]) -> f64 {
    r.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// Solve the dense `N×N` system `a·x = b` by Gaussian elimination with partial pivoting.
/// Returns `None` if the matrix is (numerically) singular. Zero-allocation.
fn solve_linear(a: &mut [[f64; N]; N], b: &[f64; N]) -> Option<[f64; N]> {
    let mut x = *b;
    for col in 0..N {
        // Partial pivot: largest magnitude in this column at/below the diagonal.
        let mut pivot = col;
        let mut best = a[col][col].abs();
        for row in (col + 1)..N {
            let v = a[row][col].abs();
            if v > best {
                best = v;
                pivot = row;
            }
        }
        if best < 1e-14 {
            return None;
        }
        if pivot != col {
            a.swap(col, pivot);
            x.swap(col, pivot);
        }
        // Eliminate below.
        let diag = a[col][col];
        for row in (col + 1)..N {
            let factor = a[row][col] / diag;
            if factor != 0.0 {
                for k in col..N {
                    a[row][k] -= factor * a[col][k];
                }
                x[row] -= factor * x[col];
            }
        }
    }
    // Back-substitution.
    for col in (0..N).rev() {
        let mut acc = x[col];
        for k in (col + 1)..N {
            acc -= a[col][k] * x[k];
        }
        x[col] = acc / a[col][col];
    }
    Some(x)
}
