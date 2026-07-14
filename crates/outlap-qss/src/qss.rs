// SPDX-License-Identifier: AGPL-3.0-only
//! Tier dispatch, the per-wheel/slow-state result surface, and the QSS slow-state coupling (PR8).
//!
//! `sim.tier` selects the lap solver over one shared vehicle description (Hard rule #4):
//!
//! * **`t0`** — the point-mass velocity profile on the corrected g-g-g-v envelope
//!   ([`solve_lap_ggv`](crate::solver::solve_lap_ggv)); point-mass channels only.
//! * **`t1`** — the same envelope velocity profile, then a per-station **re-trim**
//!   ([`T1Vehicle::trim`]) at the solved `(v, a_x, a_y, g_normal)` to emit per-wheel loads/slips/
//!   forces and the setup metrics.
//! * **`t2` / `t3`** — a typed [`QssError::TierNotImplemented`] (they arrive in M4 / M6).
//!
//! # Slow-state coupling (closed loop)
//!
//! The static PR7 envelope is thermal/SoC-neutral. When a full electrified stack is present (an
//! `.emotor` thermal network + a battery pack + Vdc-mapped drive units) the machine-thermal derate
//! and the battery peak-power ceiling **compose** as `min` caps on the powertrain traction ceiling,
//! and the machine temperatures + pack SoC advance segment-to-segment (§6.1 slow states, §8.5). The
//! coupling is resolved by a bounded, deterministic **outer march**: solve the profile, march the
//! slow states forward along it to build a per-station traction scale, re-solve — repeated
//! [`OUTER_ITERS`] times. Over a single flying lap the states barely move, so it converges at once
//! and stays neutral for the flat-track Limebeer gate. The march reuses PR5/PR6's zero-allocation
//! per-segment `step` primitives verbatim; when no mapped stack is supplied the scale stays `≡ 1`
//! and the result is bit-identical to the uncoupled solve.
//!
//! Regenerative harvest is not fed back into the pack in this QSS coupling — SoC is a discharge-only
//! bound this milestone (monotone non-increasing over a lap); recovery phases arrive with the ERS
//! energy manager in M6.

use outlap_schema::sim::{FzCoupling, Tier};

use crate::error::{T0Error, T1Error};
use crate::path::T0Path;
use crate::result::{LapResult, LineDescriptor, T0Workspace};
use crate::solver::{derive_ax, lap_result_from_ws, solve_into_ggv, solve_into_ggv_coupled};
use crate::t1::{GgvEnvelope, MachineThermal, Pack, PackState, T1Vehicle, TrimInput, TrimOutcome};
use crate::tire::{tire_slow_log, TireSlowLog, TireThermalMarch};
use crate::vehicle::T0Vehicle;
use crate::G;

/// Fixed number of solve → march → re-solve outer iterations for the slow-state coupling. Two is
/// ample: a single flying lap moves the slow states little, so the traction scale is essentially
/// converged after the first correction; the count is fixed (not tolerance-driven) for determinism.
pub const OUTER_ITERS: usize = 2;

/// The wheel-channel order (`[FL, FR, RL, RR]`) the per-wheel logs and the Python `wheel` dim use.
pub const WHEEL_ORDER: [&str; 4] = ["FL", "FR", "RL", "RR"];

/// Per-wheel T1 channels over stations, each `[FL, FR, RL, RR]`.
#[derive(Clone, Debug)]
pub struct WheelLog {
    /// Vertical (normal) load `F_z`, N.
    pub vertical_load_n: Vec<[f64; 4]>,
    /// Longitudinal slip ratio `κ` (dimensionless).
    pub slip_ratio: Vec<[f64; 4]>,
    /// Slip angle `α`, rad.
    pub slip_angle_rad: Vec<[f64; 4]>,
    /// Longitudinal tyre force (body frame) `F_x`, N.
    pub force_long_n: Vec<[f64; 4]>,
    /// Lateral tyre force (body frame) `F_y`, N.
    pub force_lat_n: Vec<[f64; 4]>,
}

impl WheelLog {
    fn with_capacity(n: usize) -> Self {
        Self {
            vertical_load_n: Vec::with_capacity(n),
            slip_ratio: Vec::with_capacity(n),
            slip_angle_rad: Vec::with_capacity(n),
            force_long_n: Vec::with_capacity(n),
            force_lat_n: Vec::with_capacity(n),
        }
    }

    fn push(&mut self, fz: [f64; 4], kappa: [f64; 4], alpha: [f64; 4], fx: [f64; 4], fy: [f64; 4]) {
        self.vertical_load_n.push(fz);
        self.slip_ratio.push(kappa);
        self.slip_angle_rad.push(alpha);
        self.force_long_n.push(fx);
        self.force_lat_n.push(fy);
    }

    fn push_nan(&mut self) {
        let nan = [f64::NAN; 4];
        self.push(nan, nan, nan, nan, nan);
    }
}

/// Per-station T1 setup metrics.
#[derive(Clone, Debug)]
pub struct SetupLog {
    /// Understeer gradient `K = dδ/da_y − L/v²` at the station's `(v, g_normal)` (`NaN` if a probe
    /// was infeasible), rad·s²/m.
    pub understeer_gradient: Vec<f64>,
    /// Front axle's share of total aerodynamic downforce at the station speed (0..1).
    pub aero_front_share: Vec<f64>,
}

/// Per-station slow-state channels (present only when a coupled electrified stack was supplied).
#[derive(Clone, Debug)]
pub struct SlowLog {
    /// Pack state of charge (0..1).
    pub state_of_charge: Vec<f64>,
    /// Machine winding temperature, °C (the representative rated node).
    pub machine_temp_c: Vec<f64>,
}

/// A solved QSS lap: the point-mass core plus the resolved tier, the recorded numerics, and — for
/// `t1` — the per-wheel / setup / slow-state channels and the returnable envelope.
#[derive(Clone, Debug)]
pub struct QssLap {
    /// The point-mass SoA channels + lap time + provenance (always present).
    pub lap: LapResult,
    /// The resolved solver tier (recorded in every artifact).
    pub tier: Tier,
    /// The recorded normal-load coupling mode (Decision #29).
    pub fz_coupling: FzCoupling,
    /// Whether the lap ran in flat-track analysis mode (`sim.flat_track`).
    pub flat_track: bool,
    /// Per-wheel channels (`t1` only).
    pub wheels: Option<WheelLog>,
    /// Setup metrics (`t1` only).
    pub setup: Option<SetupLog>,
    /// Slow-state channels (present iff a coupled stack was supplied and active).
    pub slow: Option<SlowLog>,
    /// Tyre-thermal slow-state channels (present iff a [`TireThermalMarch`] was supplied — the
    /// representative front tyre's `T_s/T_c/T_g`, wear, damage, and grip multiplier).
    pub tire: Option<TireSlowLog>,
    /// The g-g-g-v envelope the lap ran on (the returnable `lap.envelope`; `None` for the degenerate
    /// no-envelope path).
    pub envelope: Option<GgvEnvelope>,
}

/// The electrified slow-state stack the coupling marches: the T1 vehicle (its powertrain maps + the
/// envelope's drag), the machine thermal network, and the battery pack + its initial state. Built at
/// the native/host edge (parquet decode) and handed to [`solve_t0`] / [`solve_t1`].
#[derive(Clone, Debug)]
pub struct SlowCoupling<'a> {
    /// The T1 vehicle carrying the installed Vdc-mapped drive units (traction energy) and mass.
    pub vehicle: &'a T1Vehicle,
    /// The machine thermal network (advanced from its assembled state each march).
    pub thermal: MachineThermal,
    /// The battery pack (immutable maps + limits).
    pub pack: Pack,
    /// The pack's initial per-segment state (full charge / coolant temperature by default).
    pub pack_state: PackState,
}

/// Errors from the tier dispatch and the QSS lap solve.
#[derive(Debug, thiserror::Error)]
pub enum QssError {
    /// A T0 velocity-profile failure (workspace mismatch, diverged passes).
    #[error(transparent)]
    T0(#[from] T0Error),
    /// A T1 trim / envelope failure.
    #[error(transparent)]
    T1(#[from] T1Error),
    /// A transient tier was requested — it is not implemented in this milestone.
    #[error(
        "solver tier `{tier}` is not implemented yet (the transient tiers arrive in milestone \
         {milestone}); select tier `t0` (point-mass on the g-g-g-v envelope) or `t1` (full QSS \
         with per-wheel outputs)"
    )]
    TierNotImplemented {
        /// The requested tier as written in `sim.yaml` (`t2` / `t3`).
        tier: &'static str,
        /// The milestone that ships it (`M4` for T2, `M6` for T3).
        milestone: &'static str,
    },
}

/// The velocity-frame lateral demand `a_y` and the road-normal specific gravity `g_normal` at
/// station `i`, speed `v` (the same projection the solver's grip model uses).
fn demand_and_gn(path: &T0Path, i: usize, v: f64) -> (f64, f64) {
    let u = v * v;
    let ay = path.kappa_l[i] * u + G * path.sin_b_cos_g[i];
    let gn = G * path.cos_b_cos_g[i] + path.kappa_n[i] * u;
    (ay, gn)
}

/// March the slow states forward along a solved profile `(v, ax)`, filling the per-station traction
/// `scale ∈ [0, 1]` (derate ∧ battery-power cap) and the `soc` / `temp_c` logs. Resets the thermal
/// network and pack state to their assembled values first, so every outer iteration marches the
/// whole lap from the reference state (deterministic). Zero heap allocation (the per-segment `step`
/// primitives use stack arrays).
#[allow(clippy::too_many_arguments)]
fn march_slow_states(
    c: &SlowCoupling<'_>,
    env: &GgvEnvelope,
    path: &T0Path,
    v: &[f64],
    ax: &[f64],
    scale: &mut [f64],
    soc: &mut [f64],
    temp_c: &mut [f64],
) {
    let mut thermal = c.thermal.clone();
    let mut st = c.pack_state;
    let pt = c.vehicle.powertrain();
    let m = c.vehicle.mass_kg;
    let n = path.len();
    scale.fill(1.0);
    for seg in 0..path.segments() {
        let i = seg;
        let j = if path.closed { (seg + 1) % n } else { seg + 1 };
        let vi = v[i];
        let dt = 2.0 * path.ds / (v[i] + v[j]).max(1e-6);
        // Log the ENTRY state at station i (the state the car carries INTO segment i): station 0
        // reports the initial SoC/temperature, and the channels line up with the stations rather
        // than leading the car by one segment.
        soc[i] = st.soc;
        temp_c[i] = thermal.winding_temp_c();
        // Wheel drive force actually demanded (positive part): F_t = m·(a_x + drag_accel + g·sinθ_g).
        let f_drive = (m * (ax[i] + env.drag_accel(vi) + G * path.sin_g[i])).max(0.0);
        let vdc = c.pack.terminal_voltage_v(&st);
        if let Some(te) = pt.traction_energy(vi, f_drive, Some(vdc)) {
            // Machine thermal → derate. A thermal-integrator error leaves the derate at 1 (no cap).
            let derate = thermal
                .step(te.loss_w, |_| None, te.omega_rad_s, dt)
                .unwrap_or(1.0);
            // Battery peak-power cap, evaluated before the step advances SoC.
            let p_cap = c.pack.discharge_power_limit_w(&st);
            let batt_scale = if te.source_w > p_cap && te.source_w > 0.0 {
                (p_cap / te.source_w).clamp(0.0, 1.0)
            } else {
                1.0
            };
            let out = c.pack.step_power(&mut st, te.source_w, dt);
            scale[i] = (derate.min(batt_scale)).clamp(0.0, 1.0);
            debug_assert!(out.soc <= 1.0 && out.soc >= 0.0);
        }
    }
    // An open path's final station is not a segment start: it carries the end-of-lap state.
    if !path.closed && n > 0 {
        soc[n - 1] = st.soc;
        temp_c[n - 1] = thermal.winding_temp_c();
    }
}

/// Run the coupled (or uncoupled) velocity profile into `ws`, returning the lap time and — when a
/// coupling is active — the filled slow-state logs (the electrified `SlowLog` and/or the tyre-thermal
/// `TireSlowLog`; each `None` when its coupling was absent).
///
/// The electrified stack (`coupling`) and the tyre-thermal march (`tire`) both march along the
/// previous pass and re-solve, composed into one outer iteration: each solve indexes the envelope on
/// the marched `(T_tire, wear)` **and** scales the powertrain ceiling by the marched traction scale.
fn solve_profile(
    t0: &T0Vehicle,
    env: &GgvEnvelope,
    coupling: Option<&SlowCoupling<'_>>,
    tire: Option<&TireThermalMarch>,
    path: &T0Path,
    ws: &mut T0Workspace,
) -> Result<(f64, Option<SlowLog>, Option<TireSlowLog>), T0Error> {
    let n = path.len();
    if coupling.is_none() && tire.is_none() {
        let lap_time = solve_into_ggv(t0, env, path, ws)?;
        return Ok((lap_time, None, None));
    }
    let mut ax = vec![0.0; n];
    // Electrified slow-state buffers (traction scale + SoC/temperature logs).
    let mut scale = vec![1.0; n];
    let mut soc = vec![0.0; n];
    let mut temp_c = vec![0.0; n];
    // Tyre-thermal buffers: the envelope index `(T_tire, wear)` + the surfaced channels.
    let mut t_tire_k = vec![tire.map_or(0.0, TireThermalMarch::seed_surface_k); n];
    let mut wear_mm = vec![0.0; n];
    let mut tire_log = tire_slow_log(n);

    // Iteration 0: the uncoupled profile seeds the marches.
    let mut lap_time = solve_into_ggv(t0, env, path, ws)?;
    for _ in 0..OUTER_ITERS {
        derive_ax(path, &ws.v, &mut ax);
        if let Some(c) = coupling {
            march_slow_states(c, env, path, &ws.v, &ax, &mut scale, &mut soc, &mut temp_c);
        }
        if let Some(tm) = tire {
            tm.march(
                t0,
                env,
                path,
                &ws.v,
                &ax,
                &mut t_tire_k,
                &mut wear_mm,
                &mut tire_log,
            );
        }
        let scale_ref = coupling.map(|_| scale.as_slice());
        let tire_ref = tire.map(|_| (t_tire_k.as_slice(), wear_mm.as_slice()));
        lap_time = solve_into_ggv_coupled(t0, env, scale_ref, tire_ref, path, ws)?;
    }
    // Final marches against the converged profile so the reported channels match it.
    derive_ax(path, &ws.v, &mut ax);
    let slow = coupling.and_then(|c| {
        march_slow_states(c, env, path, &ws.v, &ax, &mut scale, &mut soc, &mut temp_c);
        // A coupling with no mapped units (`traction_energy` always `None`) leaves the states pinned —
        // report it only when it actually did something (SoC moved / winding heated / a scale applied).
        let active = soc.iter().any(|&s| (s - c.pack_state.soc).abs() > 0.0)
            || temp_c.iter().any(|&t| (t - temp_c[0]).abs() > 0.0)
            || scale.iter().any(|&s| s < 1.0);
        active.then(|| SlowLog {
            state_of_charge: std::mem::take(&mut soc),
            machine_temp_c: std::mem::take(&mut temp_c),
        })
    });
    let tire_slow = tire.map(|tm| {
        tm.march(
            t0,
            env,
            path,
            &ws.v,
            &ax,
            &mut t_tire_k,
            &mut wear_mm,
            &mut tire_log,
        );
        tire_log
    });
    Ok((lap_time, slow, tire_slow))
}

/// Solve a `t0` lap: the point-mass velocity profile on the envelope, with the slow-state coupling
/// when `coupling` is supplied. Point-mass channels only.
///
/// # Errors
/// [`QssError::T0`] on a velocity-profile failure.
#[allow(clippy::too_many_arguments)]
pub fn solve_t0(
    t0: &T0Vehicle,
    env: GgvEnvelope,
    coupling: Option<&SlowCoupling<'_>>,
    tire: Option<&TireThermalMarch>,
    path: &T0Path,
    line: LineDescriptor,
    resolved_hash: String,
    notes: Vec<String>,
    fz_coupling: FzCoupling,
    flat_track: bool,
) -> Result<QssLap, QssError> {
    let mut ws = T0Workspace::for_path(path);
    let (lap_time_s, slow, tire_slow) = solve_profile(t0, &env, coupling, tire, path, &mut ws)?;
    let lap = lap_result_from_ws(path, &ws, lap_time_s, line, resolved_hash, notes);
    Ok(QssLap {
        lap,
        tier: Tier::T0,
        fz_coupling,
        flat_track,
        wheels: None,
        setup: None,
        slow,
        tire: tire_slow,
        envelope: Some(env),
    })
}

/// Solve a `t1` lap: the envelope velocity profile (coupled when `coupling` is supplied) plus a
/// per-station re-trim for the per-wheel channels and setup metrics.
///
/// # Errors
/// [`QssError::T0`] on a velocity-profile failure.
#[allow(clippy::too_many_arguments)]
pub fn solve_t1(
    t1: &T1Vehicle,
    t0: &T0Vehicle,
    env: GgvEnvelope,
    coupling: Option<&SlowCoupling<'_>>,
    tire: Option<&TireThermalMarch>,
    path: &T0Path,
    line: LineDescriptor,
    resolved_hash: String,
    notes: Vec<String>,
    fz_coupling: FzCoupling,
    flat_track: bool,
) -> Result<QssLap, QssError> {
    let mut ws = T0Workspace::for_path(path);
    let (lap_time_s, slow, tire_slow) = solve_profile(t0, &env, coupling, tire, path, &mut ws)?;

    // Re-trim at each solved station for the per-wheel channels + setup metrics.
    let n = path.len();
    let mut ax = vec![0.0; n];
    derive_ax(path, &ws.v, &mut ax);
    let mut wheels = WheelLog::with_capacity(n);
    let mut understeer_gradient = Vec::with_capacity(n);
    let mut aero_front_share = Vec::with_capacity(n);
    for i in 0..n {
        let v = ws.v[i];
        let (ay, gn) = demand_and_gn(path, i, v);
        let inp = TrimInput {
            v: v.max(1e-3),
            ay,
            ax: ax[i],
            g_normal: gn,
            coupling: fz_coupling,
        };
        match t1.trim(&inp) {
            TrimOutcome::Converged(s) => wheels.push(s.fz, s.kappa, s.alpha, s.fx, s.fy),
            TrimOutcome::Infeasible { .. } => wheels.push_nan(),
        }
        understeer_gradient.push(t1.understeer_gradient(v.max(1e-3), gn).unwrap_or(f64::NAN));
        aero_front_share.push(t1.aero_front_downforce_share_at(v.max(1e-3)));
    }

    let lap = lap_result_from_ws(path, &ws, lap_time_s, line, resolved_hash, notes);
    Ok(QssLap {
        lap,
        tier: Tier::T1,
        fz_coupling,
        flat_track,
        wheels: Some(wheels),
        setup: Some(SetupLog {
            understeer_gradient,
            aero_front_share,
        }),
        slow,
        tire: tire_slow,
        envelope: Some(env),
    })
}

/// The typed "not implemented" error for the transient tiers (`t2` / `t3`).
///
/// # Errors
/// Always [`QssError::TierNotImplemented`] — this only constructs the error for the dispatch site.
pub fn tier_not_implemented(tier: Tier) -> QssError {
    match tier {
        Tier::T2 => QssError::TierNotImplemented {
            tier: "t2",
            milestone: "M4",
        },
        _ => QssError::TierNotImplemented {
            tier: "t3",
            milestone: "M6",
        },
    }
}
