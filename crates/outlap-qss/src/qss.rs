// SPDX-License-Identifier: AGPL-3.0-only
//! Tier dispatch, the per-wheel/slow-state result surface, and the QSS slow-state coupling.
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
//! The static PR7 envelope is thermal/SoC-neutral. When an electro slow stack is present (a
//! battery pack, optionally a machine-thermal network and Vdc-mapped drive units) the machine
//! derate and the battery ceilings **compose** as caps on the powertrain traction, and the pack
//! state + machine temperatures advance segment-to-segment (§6.1 slow states, §8.5). The coupling
//! is resolved by a bounded, deterministic **outer march**: solve the profile, march the slow
//! states forward along it to build the per-station traction scale + ERS deploy-force slice,
//! re-solve — repeated [`OUTER_ITERS`] times, with the convergence metric of the last two
//! iterations recorded in [`SlowLog::convergence`]. The march reuses the zero-allocation
//! per-segment `step` primitives verbatim; when no active stack is supplied the result is
//! bit-identical to the uncoupled solve.
//!
//! # The 2026 ERS energy manager (M6 PR2)
//!
//! With an [`ErsCoupling`] the march is governed by the shared
//! [`EnergyManager`](outlap_powertrain::EnergyManager) — the SAME rulebook implementation every
//! tier consumes (D-M6-2). Per segment the manager decides deploy / brake harvest /
//! part-throttle harvest / super-clip back-drive (§8.3's rule-based policy, or a `u(s)`
//! schedule); the pack then has the final word (discharge ceiling on deploy, charge acceptance
//! on harvest), the realized command is banked in the per-lap
//! [`LapEnergyLedger`](outlap_powertrain::LapEnergyLedger) (reset at `s = 0`), and the electric
//! wheel-force share enters the next profile solve as an ADDITIVE per-station slice — the
//! machine/battery caps scale the electric share only, never the ICE (D-M6-10). Braking harvest
//! composes the same five ceilings as T2's `blend_regen` (see [`crate::ers`]) and never touches
//! the trajectory; super-clip back-drive on full-throttle straights reduces net force through
//! the C5.12 ramp — the regulation's "power limited" periods.

use outlap_powertrain::{DecideInput, ErsCommand, ErsMode, LapEnergyLedger};
use outlap_schema::sim::{FzCoupling, Tier};
use outlap_tire::TireThermalState;

use crate::error::{T0Error, T1Error};
use crate::ers::ErsCoupling;
use crate::fuel::FuelCoupling;
use crate::path::T0Path;
use crate::result::{LapResult, LineDescriptor, T0Workspace};
use crate::solver::{derive_ax, lap_result_from_ws, solve_into_ggv, solve_into_ggv_coupled};
use crate::t1::{GgvEnvelope, MachineThermal, Pack, PackState, T1Vehicle, TrimInput, TrimOutcome};
use crate::tire::{tire_slow_log, TireSlowLog, TireThermalMarch};
use crate::vehicle::T0Vehicle;
use crate::G;

/// Fixed number of solve → march → re-solve outer iterations for the slow-state coupling. Two is
/// ample for the thermal/discharge marches: a single flying lap moves the slow states little, so
/// the traction scale is essentially converged after the first correction; the count is fixed
/// (not tolerance-driven) for determinism, and the last two iterations' residual is recorded in
/// [`SlowLog::convergence`].
pub const OUTER_ITERS: usize = 2;

/// Fixed outer-iteration count when an ENERGY MANAGER governs the march ([`Couplings::ers`]).
/// The manager closes a stronger loop than the derate marches — the deploy/harvest schedule
/// reshapes the very profile it was decided on — and a SoC-starved lap needs several passes for
/// the deploy pattern to settle. With the deploy-slice under-relaxation ([`DEPLOY_RELAX`]) the
/// charge-sustain bistability damps out well before this count; still fixed (never tolerance-
/// driven) for determinism, and no-ers couplings keep [`OUTER_ITERS`], so every pre-M6 path is
/// bit-identical.
pub const OUTER_ITERS_ERS: usize = 8;

/// Under-relaxation factor for the solver-fed ERS deploy-force slice across outer iterations
/// (`ω = 0.5`). Damps the deploy↔harvest bistability at the charge-sustain equilibrium — a
/// straight station where SoC hovers at the Recharge target — into a converged fixed point.
/// Applied only to the slice the profile solve consumes; the marched pack state, the ledger, and
/// the reported channels always come from a fresh march on the converged profile.
const DEPLOY_RELAX: f64 = 0.5;

/// Driver-demand fraction at/above which a drive station counts as FULL throttle for the energy
/// manager's recharge-phase classification (the C5.12 super-clip path); below it the station is
/// part throttle (the ICE covers the demand gap directly). The demand is measured against the
/// pedal availability `tractive_force(v)` reconstructed from the solved profile, so a strict
/// `== 1` would be numerically fragile.
const FULL_THROTTLE_DEMAND: f64 = 0.98;

/// Mechanical→electrical regen recovery efficiency (machine + inverter) for the non-manager
/// mapped-EV braking harvest — a documented constant proxy matching the transient tier's
/// `RegenParams` default (`outlap-vehicle`), so QSS and T2 recover the same electrical energy from a
/// given braking capture. The ERS manager uses the FIA 0.97 electrical↔mechanical factor instead.
const REGEN_EFFICIENCY: f64 = 0.9;

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

/// Per-station ERS energy-manager channels + the per-lap ledger totals (present iff an
/// [`ErsCoupling`] governed the march). Powers are ELECTRICAL (the CU-K DC bus) and REALIZED —
/// post pack-acceptance/machine-ceiling clips, exactly what the ledger banked.
#[derive(Clone, Debug)]
pub struct ErsSlowLog {
    /// Realized electrical deployment power per station, W (≥ 0).
    pub deploy_power_w: Vec<f64>,
    /// Realized electrical harvest power per station, W (≥ 0) — all harvest paths (braking,
    /// part-throttle, super-clip back-drive).
    pub harvest_power_w: Vec<f64>,
    /// The lap ledger's deployment integral, J electrical.
    pub ledger_deploy_j: f64,
    /// The lap ledger's harvest ("Recharge") integral, J electrical (C5.2.10).
    pub ledger_harvest_j: f64,
    /// Minimum on-track SoC over the lap (with `soc_max`, the recorded FIA C5.2.9 max−min ≤ 4 MJ
    /// compliance channel — record, not clamp; D-M6-3).
    pub soc_min: f64,
    /// Maximum on-track SoC over the lap.
    pub soc_max: f64,
}

/// The outer-iteration convergence residual between the LAST TWO solve → march passes (recorded,
/// not asserted — promote [`OUTER_ITERS`] to a sim setting only if a limit cycle is observed).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MarchConvergence {
    /// Max per-station |Δ traction scale| between the last two marches.
    pub max_dscale: f64,
    /// Max per-station |Δ deploy-force slice| between the last two marches, N.
    pub max_ddeploy_n: f64,
    /// |Δ lap time| between the last two profile solves, s.
    pub dlap_s: f64,
}

/// Per-station slow-state channels (present only when an ACTIVE coupled electro stack was
/// supplied — the assembly-time [`SlowCoupling::active`] flag, not an output heuristic).
#[derive(Clone, Debug)]
pub struct SlowLog {
    /// Pack state of charge (0..1).
    pub state_of_charge: Vec<f64>,
    /// Machine winding temperature, °C (the representative rated node) — `None` when the pack
    /// marches without a machine-thermal network (the relaxed M6 PR2 stack).
    pub machine_temp_c: Option<Vec<f64>>,
    /// ERS energy-manager channels (present iff an [`ErsCoupling`] governed the march).
    pub ers: Option<ErsSlowLog>,
    /// The recorded outer-iteration convergence residual.
    pub convergence: MarchConvergence,
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
    /// The **terminal** slow state at the end of the lap — the representative-tyre state, the battery
    /// pack state, and the machine-thermal network — bundled so a stint carries ONE object into the
    /// next lap's seed and the slow states continue across the lap boundary with no reset.
    pub slow_terminal: SlowSnapshot,
    /// The g-g-g-v envelope the lap ran on (the returnable `lap.envelope`; `None` for the degenerate
    /// no-envelope path).
    pub envelope: Option<GgvEnvelope>,
}

/// The terminal slow state at the end of a QSS lap: the representative-tyre thermal ring + wear
/// state, the battery pack state (SoC / RC voltage / temperature), and the machine-thermal network.
/// Bundled into ONE object (used as both the [`QssLap`] terminal and, for the state a stint carries,
/// the next lap's seed) rather than parallel `Option`s. Each field is `None` when its subsystem did
/// not march. (The fuel slow state joins this snapshot in M6 PR5.)
///
/// Carry policy across a stint lap boundary ([`solve_stint`]): the tyre state and the pack SoC +
/// temperature carry; the pack's within-lap transients (RC voltage / last current) reset; the
/// machine-thermal network is a per-lap **diagnostic only** (surfaced, not carried) — seeding a
/// near-limit winding temperature into the quasi-steady distance march destabilises it.
#[derive(Clone, Debug, Default)]
pub struct SlowSnapshot {
    /// Terminal representative-tyre state (present iff a [`TireThermalMarch`] ran).
    pub tire: Option<TireThermalState<f64>>,
    /// Terminal battery pack state (present iff an electro stack was coupled).
    pub pack: Option<PackState>,
    /// Terminal machine-thermal network (present iff a drive unit declared an `.emotor` and it was
    /// marched — never under an energy manager, D-M6-10). A per-lap diagnostic; not carried.
    pub machine: Option<MachineThermal>,
    /// Terminal fuel mass, kg (present iff a [`FuelCoupling`] marched). Carried across a stint lap
    /// boundary so lap N+1 starts lighter (D-M6-4).
    pub fuel_kg: Option<f64>,
}

/// The electro slow-state stack the coupling marches: the T1 vehicle (its powertrain maps + the
/// envelope's drag), the battery pack + its initial state, and — when a drive unit declares an
/// `.emotor` — the machine thermal network. Built at the native/host edge (parquet decode) and
/// handed to [`solve_t0`] / [`solve_t1`] through [`Couplings`].
#[derive(Clone, Debug)]
pub struct SlowCoupling<'a> {
    /// The T1 vehicle carrying the installed Vdc-mapped drive units (traction energy) and mass.
    pub vehicle: &'a T1Vehicle,
    /// The machine thermal network (advanced from its assembled state each march) — `None` when
    /// no drive unit declares an `.emotor` (the pack still marches; M6 PR2 relaxed rule).
    pub thermal: Option<MachineThermal>,
    /// The battery pack (immutable maps + limits).
    pub pack: Pack,
    /// The pack's initial per-segment state.
    pub pack_state: PackState,
    /// Whether this stack can actually move a slow state — set at ASSEMBLY (installed energy
    /// maps on some drive unit), not inferred from outputs. An [`ErsCoupling`] activates the
    /// stack regardless (the manager drives the pack directly).
    pub active: bool,
}

/// The optional per-lap couplings, bundled (they were heading past 10 positional parameters).
#[derive(Clone, Copy, Default)]
pub struct Couplings<'a> {
    /// The electro slow stack (battery pack + optional machine thermal + the T1 vehicle).
    pub electro: Option<&'a SlowCoupling<'a>>,
    /// The tyre-thermal march.
    pub tire: Option<&'a TireThermalMarch>,
    /// The 2026 ERS energy manager (requires `electro` — the manager schedules the pack).
    pub ers: Option<&'a ErsCoupling>,
    /// The fuel-mass slow state (§8.1, D-M6-4): the tank drains as the ICE burns fuel and the
    /// shrinking mass + migrating CG feed the point-mass equations and the #31 envelope corrections.
    /// `None` ⇒ mass is the assembly constant (byte-identical to pre-M6).
    pub fuel: Option<&'a FuelCoupling<'a>>,
}

/// The per-lap request metadata threaded through the solve into the result artifact.
#[derive(Clone, Debug)]
pub struct LapRequest {
    /// The sampled target line descriptor.
    pub line: LineDescriptor,
    /// The resolved-vehicle hash (provenance).
    pub resolved_hash: String,
    /// Assembly/loading notes to carry into the lap result (nothing silent).
    pub notes: Vec<String>,
    /// The recorded normal-load coupling mode (Decision #29).
    pub fz_coupling: FzCoupling,
    /// Whether the lap runs in flat-track analysis mode.
    pub flat_track: bool,
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
    /// An energy-manager coupling was supplied without the electro slow stack it schedules.
    #[error(
        "an ERS energy-manager coupling requires the electro slow stack (a battery pack) — \
         nothing to bank deployment/harvest into"
    )]
    ErsCouplingWithoutPack,
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

/// The SoA per-station buffers the slow-state marches fill (one struct, not six positional
/// slices). Allocated once per profile solve; the march kernels stay zero-allocation.
#[derive(Clone, Debug)]
pub(crate) struct SlowMarchBuffers {
    /// Traction scale ∈ [0, 1] on the MECHANICAL share (machine derate ∧ battery cap for a
    /// mapped EV stack; ≡ 1 under an energy manager — the caps move to the electric share).
    pub scale: Vec<f64>,
    /// Pack SoC at station entry.
    pub soc: Vec<f64>,
    /// Machine winding temperature at station entry, °C (untouched when no thermal network).
    pub temp_c: Vec<f64>,
    /// ADDITIVE ERS deploy-force slice, N (signed; negative = super-clip back-drive).
    pub deploy_force_n: Vec<f64>,
    /// Realized electrical deployment power, W.
    pub deploy_w: Vec<f64>,
    /// Realized electrical harvest power, W.
    pub harvest_w: Vec<f64>,
}

impl SlowMarchBuffers {
    fn new(n: usize) -> Self {
        Self {
            scale: vec![1.0; n],
            soc: vec![0.0; n],
            temp_c: vec![0.0; n],
            deploy_force_n: vec![0.0; n],
            deploy_w: vec![0.0; n],
            harvest_w: vec![0.0; n],
        }
    }
}

/// The per-march scalar outcomes (ledger totals + the SoC swing record).
#[derive(Clone, Copy, Debug, Default)]
struct MarchStats {
    ledger_deploy_j: f64,
    ledger_harvest_j: f64,
    soc_min: f64,
    soc_max: f64,
}

/// The full outcome of one slow-state march: the scalar [`MarchStats`] plus the **terminal** pack
/// and machine-thermal state at the end of the lap (what a stint carries into the next lap's seed).
struct MarchOutcome {
    stats: MarchStats,
    pack_terminal: PackState,
    machine_terminal: Option<MachineThermal>,
}

/// March the slow states forward along a solved profile `(v, ax)`, filling the per-station
/// buffers. Resets the thermal network, pack state, and lap ledger to their assembled values
/// first, so every outer iteration marches the whole lap from the reference state
/// (deterministic; the ledger reset IS the `s = 0` lap boundary). Zero heap allocation (the
/// per-segment `step` primitives use stack arrays).
///
/// Two regimes:
///
/// * **No manager** (`ers: None`) — the pre-M6 mapped-EV path, unchanged: the full traction draw
///   feeds the pack (exact for a pure-electric drive), machine derate ∧ battery discharge cap
///   compose into `scale`.
/// * **Manager** (`ers: Some`) — D-M6-10: the pack exchanges ONLY the manager's electrical
///   deploy/harvest power (the ICE covers the rest of traction; `scale ≡ 1` on the mechanical
///   share), the realized command lands in `deploy_force_n`/`deploy_w`/`harvest_w`, and the
///   per-lap ledger enforces the Recharge budget. Machine-thermal is not marched under a manager
///   (no shipped vehicle pairs an `.emotor` with `ers:`; the MGU-K has no thermal-network schema
///   home — recorded in the loaded-model notes).
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn march_slow_states(
    t0: &T0Vehicle,
    c: &SlowCoupling<'_>,
    ers: Option<&ErsCoupling>,
    env: &GgvEnvelope,
    path: &T0Path,
    v: &[f64],
    ax: &[f64],
    bufs: &mut SlowMarchBuffers,
) -> MarchOutcome {
    let mut thermal = c.thermal.clone();
    let mut st = c.pack_state;
    let pt = c.vehicle.powertrain();
    let m = c.vehicle.mass_kg;
    let n = path.len();
    bufs.scale.fill(1.0);
    bufs.deploy_force_n.fill(0.0);
    bufs.deploy_w.fill(0.0);
    bufs.harvest_w.fill(0.0);
    let mut ledger = LapEnergyLedger::new();
    // Caller-owned C5.12 ramp episode state (the manager is pure): the previous step's signed
    // electrical K power and the cumulative reduction taken in the active episode.
    let mut prev_k_power_w = 0.0_f64;
    let mut ramp_reduced_w = 0.0_f64;
    let mut stats = MarchStats {
        soc_min: st.soc,
        soc_max: st.soc,
        ..MarchStats::default()
    };
    for seg in 0..path.segments() {
        let i = seg;
        let j = if path.closed { (seg + 1) % n } else { seg + 1 };
        let vi = v[i];
        let dt = 2.0 * path.ds / (v[i] + v[j]).max(1e-6);
        // Log the ENTRY state at station i (the state the car carries INTO segment i): station 0
        // reports the initial SoC/temperature, and the channels line up with the stations rather
        // than leading the car by one segment.
        bufs.soc[i] = st.soc;
        if let Some(th) = &thermal {
            bufs.temp_c[i] = th.winding_temp_c();
        }
        // Signed wheel force demanded: F = m·(a_x + drag_accel + g·sinθ_g); > 0 drive, < 0 brake.
        let f_req = m * (ax[i] + env.drag_accel(vi) + G * path.sin_g[i]);
        if let Some(e) = ers {
            let cmd = ers_decide(
                t0,
                e,
                &st,
                f_req,
                vi,
                dt,
                i,
                prev_k_power_w,
                ramp_reduced_w,
                &ledger,
            );
            // The C5.2.9 running-band clip is built from the running SoC min/max SEEN so far this
            // lap (`stats`, which already folds in the entry state carried into this station).
            let band = SwingBand {
                seen_lo: stats.soc_min,
                seen_hi: stats.soc_max,
                swing_soc: e.swing_limit_j / c.pack.total_energy_j(),
            };
            let realized = ers_realize(t0, e, c, &mut st, &cmd, f_req, vi, dt, band, bufs, i);
            ledger.record(&realized, dt);
            // Ramp episode accounting (the manager_trace idiom): reductions accumulate while the
            // signed K power falls, and the episode resets the moment it rises.
            let k_now = realized.deploy_w - realized.harvest_w;
            if k_now < prev_k_power_w {
                ramp_reduced_w += prev_k_power_w - k_now;
            } else {
                ramp_reduced_w = 0.0;
            }
            prev_k_power_w = k_now;
            debug_assert!(st.soc <= 1.0 && st.soc >= 0.0);
        } else {
            // Mapped-EV march. The drive/idle DISCHARGE path is the pre-M6 algorithm verbatim (a
            // braking segment requests `f_drive = 0`, so it draws only the map's no-load power) — so
            // a car that cannot regen stays byte-identical. Braking REGEN (M6 PR3) is then added as a
            // NET pack term: any battery + electric machine recovers braking energy through the
            // machine, independent of the ERS manager, so the pack exchanges `draw − recovered` this
            // segment. The recovery uses the SAME ceiling chain as the transient tier's `blend_regen`
            // collapsed to the point mass — blend authority × the driven axle's brake demand, capped
            // by the machine's regen envelope × low-speed fade, converted at [`REGEN_EFFICIENCY`],
            // clipped by the pack charge acceptance (CV taper ∧ kinetic derate). The braking force at
            // the wheels is untouched (the calipers supply the rest), so the trajectory is unchanged.
            let f_drive = f_req.max(0.0);
            let vdc = c.pack.terminal_voltage_v(&st);
            // Braking regen (zero on drive segments and for a car with no `regen_blend` / no regen
            // curve — hence byte-identical to the pre-M6 discharge-only march there). Evaluated from
            // the ENTRY state, before the pack step.
            let regen_w = if f_req < 0.0 {
                let braking_power = -f_req * vi;
                let demand_w =
                    c.vehicle.regen_max_frac() * c.vehicle.regen_axle_share() * braking_power;
                let envelope_w = c.vehicle.regen_mech_power_max_w(vi) * ErsCoupling::fade(vi);
                let mech_w = demand_w.min(envelope_w).max(0.0);
                let e = (mech_w * REGEN_EFFICIENCY).min(c.pack.regen_power_limit_w(&st));
                bufs.harvest_w[i] = e;
                e
            } else {
                0.0
            };
            if let Some(te) = pt.traction_energy(vi, f_drive, Some(vdc)) {
                // Machine thermal → derate. A thermal-integrator error leaves the derate at 1.
                let derate = thermal.as_mut().map_or(1.0, |th| {
                    th.step(te.loss_w, |_| None, te.omega_rad_s, dt)
                        .unwrap_or(1.0)
                });
                // Battery peak-power cap, evaluated before the step advances SoC.
                let p_cap = c.pack.discharge_power_limit_w(&st);
                let batt_scale = if te.source_w > p_cap && te.source_w > 0.0 {
                    (p_cap / te.source_w).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                // The pack exchanges the drive/idle draw NET of any recovered regen this segment.
                let out = c.pack.step_power(&mut st, te.source_w - regen_w, dt);
                bufs.scale[i] = (derate.min(batt_scale)).clamp(0.0, 1.0);
                debug_assert!(out.soc <= 1.0 && out.soc >= 0.0);
            } else if regen_w > 0.0 {
                // No mapped drive result (e.g. above the map) but the machine still regenerates.
                let out = c.pack.step_power(&mut st, -regen_w, dt);
                debug_assert!(out.soc <= 1.0 && out.soc >= 0.0);
            }
        }
        stats.soc_min = stats.soc_min.min(st.soc);
        stats.soc_max = stats.soc_max.max(st.soc);
    }
    // An open path's final station is not a segment start: it carries the end-of-lap state.
    if !path.closed && n > 0 {
        bufs.soc[n - 1] = st.soc;
        if let Some(th) = &thermal {
            bufs.temp_c[n - 1] = th.winding_temp_c();
        }
    }
    stats.ledger_deploy_j = ledger.deploy_j();
    stats.ledger_harvest_j = ledger.harvest_j();
    MarchOutcome {
        stats,
        pack_terminal: st,
        machine_terminal: thermal,
    }
}

/// Build the manager's per-segment inputs and decide the command (electrical, unclipped by the
/// pack — "the pack has the final word" happens in [`ers_realize`]).
#[allow(clippy::too_many_arguments)]
fn ers_decide(
    t0: &T0Vehicle,
    e: &ErsCoupling,
    st: &PackState,
    f_req: f64,
    vi: f64,
    dt: f64,
    station: usize,
    prev_k_power_w: f64,
    ramp_reduced_w: f64,
    ledger: &LapEnergyLedger<f64>,
) -> ErsCommand<f64> {
    let (driver_demand, ice_surplus_w, brake_demand_w) = if f_req > 0.0 {
        // Pedal availability: mechanical units + the greedy (budget-free) ERS curve.
        let f_avail = t0.tractive_force(vi).max(1e-9);
        let raw_demand = (f_req / f_avail).clamp(0.0, 1.0);
        let p_mech_avail = t0.mech_tractive_force(vi) * vi;
        // The manager splits part-throttle harvest from the C5.12 super-clip on `demand < 1`
        // (`manager.rs`), and only the super-clip (full-throttle) path cuts the wheel force via a
        // negative slice. The march classifies full throttle at `>= FULL_THROTTLE_DEMAND`, so the
        // demand handed to `decide()` must be SNAPPED to exactly 1.0 there — otherwise a station
        // in `[0.98, 1)` reports the FULL ICE power as harvestable "surplus" yet routes through
        // the trajectory-invariant part-throttle branch, banking harvest with no mechanical source
        // and no force cut. Snapping keeps the surplus value and the manager's branch consistent.
        let (demand, surplus) = if raw_demand >= FULL_THROTTLE_DEMAND {
            // Wide-open throttle: per C5.12.7 the K may back-drive against the full ICE power
            // (the "power limited" straight — net force drops while the store recharges).
            (1.0, p_mech_avail)
        } else {
            // Part throttle: the ICE covers the demand gap directly; only the headroom above the
            // driver's demand is harvestable without touching the trajectory.
            (raw_demand, (p_mech_avail - f_req * vi).max(0.0))
        };
        (demand, surplus, 0.0)
    } else {
        // Braking: harvest ceilings 4 (blend authority) + 5 (per-axle split) fold into the
        // demanded power at the driven axle(s), exactly as the manager's input docs specify.
        let braking_power = -f_req * vi;
        (
            0.0,
            0.0,
            e.max_regen_frac * e.regen_axle_share * braking_power.max(0.0),
        )
    };
    // Harvest ceilings 1 (machine envelope, symmetric-machine proxy) + 2 (low-speed fade).
    let mech_regen_envelope_w = e.p_mech_max_w * ErsCoupling::fade(vi);
    let inp = DecideInput {
        v: vi,
        driver_demand,
        brake_demand_w,
        mech_regen_envelope_w,
        ice_surplus_w,
        soc: st.soc,
        override_active: e.override_active,
        prev_k_power_w,
        ramp_reduced_w,
        dt,
        station,
    };
    e.manager.decide(&inp, ledger)
}

/// Apply the downstream ceilings the tier owns to a manager command — the pack has the final
/// word — advance the pack, and fill the station's buffers. Returns the REALIZED command (what
/// the ledger banks).
#[allow(clippy::too_many_arguments)]
fn ers_realize(
    t0: &T0Vehicle,
    e: &ErsCoupling,
    c: &SlowCoupling<'_>,
    st: &mut PackState,
    cmd: &ErsCommand<f64>,
    f_req: f64,
    vi: f64,
    dt: f64,
    band: SwingBand,
    bufs: &mut SlowMarchBuffers,
    i: usize,
) -> ErsCommand<f64> {
    let mut realized = ErsCommand {
        deploy_w: 0.0,
        harvest_w: 0.0,
        mode: cmd.mode,
    };
    // The SoC floor/ceiling this step must respect: the PHYSICAL usable window intersected with the
    // REGULATORY C5.2.9 swing band (running-band clip — see `SwingBand`). The two are independent:
    // for a pack sized to the reg they coincide; for a physically larger pack the reg band bites
    // first. The `Pack` power ceilings + the post-step clamp enforce the PHYSICAL edge exactly as
    // before; the reg edge adds a power cap (from the SoC headroom) that fires ONLY where the reg
    // band is STRICTLY inside the physical window, so the pack stops delivering/accepting at the
    // regulatory limit (ledger-consistent) — and a pack sized to the reg (`reg == physical`, the
    // f1 case) is untouched by this branch.
    let [phys_lo, phys_hi] = c.pack.soc_window();
    let (soc_floor, soc_ceil) = band.bounds([phys_lo, phys_hi]);
    let e_total = c.pack.total_energy_j();
    if cmd.deploy_w > 0.0 {
        // Pack discharge ceiling on the ELECTRIC share (D-M6-10; the ICE is untouched), then — only
        // if the reg floor sits ABOVE the physical floor — the reg headroom, then the machine's
        // mechanical ceiling (the pack never pays for power the machine cannot convert).
        let mut p_elec = cmd.deploy_w.min(c.pack.discharge_power_limit_w(st));
        if soc_floor > phys_lo {
            p_elec = p_elec.min((st.soc - soc_floor).max(0.0) * e_total / dt);
        }
        let (_p_mech, p_elec_real) = t0.ers_realized_deploy_w(p_elec);
        c.pack.step_power(st, p_elec_real, dt);
        bufs.deploy_force_n[i] = t0.ers_deploy_force_n(vi, p_elec_real);
        realized.deploy_w = p_elec_real;
    } else if cmd.harvest_w > 0.0 {
        // Harvest ceiling 3: pack charge acceptance (design curve × kinetic derate ∧ CV taper),
        // then — only if the reg ceiling sits BELOW the physical ceiling — the reg headroom.
        let mut p_elec = cmd.harvest_w.min(c.pack.regen_power_limit_w(st));
        if soc_ceil < phys_hi {
            p_elec = p_elec.min((soc_ceil - st.soc).max(0.0) * e_total / dt);
        }
        c.pack.step_power(st, -p_elec, dt);
        realized.harvest_w = p_elec;
        if cmd.mode == ErsMode::HarvestStraight && f_req > 0.0 {
            // Super-clip back-drive: the K absorbs mechanical power at the crank while the ICE
            // stays wide open — net wheel force drops by the absorbed share (driveline η is
            // skipped on the harvest side, the T2 regen convention).
            let p_mech_abs = e.manager.rulebook().mech_harvest_w(p_elec);
            bufs.deploy_force_n[i] = -p_mech_abs / vi.max(1.0);
        }
        // Brake / part-throttle harvest never touches the trajectory: braking force is unchanged
        // (the calipers supply the deficit) and the ICE covers the part-throttle gap.
    } else {
        // Idle: the pack still relaxes (RC decay + thermal node) over the segment.
        c.pack.step_power(st, 0.0, dt);
    }
    // Belt-and-suspenders: `step_power` clamps SoC to [0, 1] only, so a segment that begins just
    // inside an edge can overshoot by one step. Clamp to the physical ∩ regulatory band so the
    // on-track swing is bounded exactly.
    st.soc = st.soc.clamp(soc_floor, soc_ceil);
    bufs.deploy_w[i] = realized.deploy_w;
    bufs.harvest_w[i] = realized.harvest_w;
    realized
}

/// The FIA C5.2.9 regulatory swing band for the current step: the running SoC min/max seen so far
/// this lap, plus the swing limit in SoC units. Bounds `max − min ≤ swing` causally — a step may
/// not raise SoC more than `swing` above the lap's lowest point so far (`seen_lo + swing`), nor
/// lower it more than `swing` below the highest (`seen_hi − swing`) — with no knowledge of the
/// future minimum. Independent of the pack's physical window; [`SwingBand::bounds`] intersects the
/// two.
#[derive(Clone, Copy)]
struct SwingBand {
    seen_lo: f64,
    seen_hi: f64,
    swing_soc: f64,
}

impl SwingBand {
    /// The physical usable window intersected with the regulatory swing band → `(floor, ceiling)`.
    fn bounds(self, [phys_lo, phys_hi]: [f64; 2]) -> (f64, f64) {
        let reg_lo = self.seen_hi - self.swing_soc;
        let reg_hi = self.seen_lo + self.swing_soc;
        (phys_lo.max(reg_lo), phys_hi.min(reg_hi))
    }
}

/// March the fuel-mass slow state over the current profile, filling the per-station `mass`/`a_f`/
/// `h_cg` slices (station-ENTRY state, matching the SoC/temperature convention) and returning the
/// terminal fuel mass (kg). The ICE burns fuel for the traction it covers — the drive force NET of
/// the realized electric deploy (`deploy_w[i]/v`, the hybrid ICE-share, §8.1) — at the ICE map's
/// brake-thermal efficiency. Zero-allocation (all buffers caller-owned).
#[allow(clippy::too_many_arguments)] // model + envelope + profile + the three output slices.
fn march_fuel(
    fc: &FuelCoupling<'_>,
    env: &GgvEnvelope,
    path: &T0Path,
    v: &[f64],
    ax: &[f64],
    deploy_w: &[f64],
    fuel_start_kg: f64,
    mass: &mut [f64],
    a_f: &mut [f64],
    h_cg: &mut [f64],
) -> f64 {
    let fm = &fc.model;
    let pt = fc.vehicle.powertrain();
    let n = path.len();
    let mut fuel = fuel_start_kg;
    for seg in 0..path.segments() {
        let i = seg;
        let j = if path.closed { (seg + 1) % n } else { seg + 1 };
        let vi = v[i];
        let dt = 2.0 * path.ds / (v[i] + v[j]).max(1e-6);
        // Station-entry mass/CG (the state the car carries INTO segment i).
        mass[i] = fm.mass_at(fuel);
        let (af_i, hcg_i) = fm.cg_at(fuel);
        a_f[i] = af_i;
        h_cg[i] = hcg_i;
        // Drive force the ICE must cover = total drive demand net of the electric deploy share.
        let f_req = mass[i] * (ax[i] + env.drag_accel(vi) + G * path.sin_g[i]);
        let f_drive = f_req.max(0.0);
        let electric_force = if vi > 1e-6 { deploy_w[i] / vi } else { 0.0 };
        let ice_force = (f_drive - electric_force).max(0.0);
        let mdot = pt.ice_fuel_rate_kg_per_s(vi, ice_force);
        fuel = (fuel - mdot * dt).max(0.0);
    }
    // An open path's final station carries the end-of-lap state (no segment starts there).
    if !path.closed && n > 0 {
        mass[n - 1] = fm.mass_at(fuel);
        let (af_i, hcg_i) = fm.cg_at(fuel);
        a_f[n - 1] = af_i;
        h_cg[n - 1] = hcg_i;
    }
    fuel
}

/// Run the coupled (or uncoupled) velocity profile into `ws`, returning the lap time and — when a
/// coupling is active — the filled slow-state logs (the electro `SlowLog` and/or the tyre-thermal
/// `TireSlowLog`; each `None` when its coupling was absent).
///
/// The electro stack (`couplings.electro`), the energy manager (`couplings.ers`), and the
/// tyre-thermal march (`couplings.tire`) all march along the previous pass and re-solve, composed
/// into one outer iteration: each solve indexes the envelope on the marched `(T_tire, wear)`,
/// scales the powertrain's mechanical ceiling by the marched traction scale, and adds the
/// manager's per-station deploy-force slice.
// The 4-tuple return is the internal profile-solve contract (lap time + the two slow-state logs +
// the terminal slow snapshot a stint carries); a struct would just rename the fields.
#[allow(clippy::type_complexity, clippy::too_many_lines)]
fn solve_profile(
    t0: &T0Vehicle,
    env: &GgvEnvelope,
    couplings: &Couplings<'_>,
    path: &T0Path,
    ws: &mut T0Workspace,
) -> Result<(f64, Option<SlowLog>, Option<TireSlowLog>, SlowSnapshot), T0Error> {
    let n = path.len();
    let ers = couplings.ers;
    // Assembly-time activity: an inactive stack (no energy maps, no manager) cannot move a slow
    // state — skip it entirely (bit-identical to the uncoupled solve by construction).
    let coupling = couplings.electro.filter(|c| c.active || ers.is_some());
    let tire = couplings.tire;
    let fuel = couplings.fuel;
    if coupling.is_none() && tire.is_none() && fuel.is_none() {
        let lap_time = solve_into_ggv(t0, env, path, ws)?;
        return Ok((lap_time, None, None, SlowSnapshot::default()));
    }
    // Fuel-mass slow-state buffers (station-entry mass/CG marched along the previous profile). Seeded
    // at the lap-start fuel (the model's initial, overridden per stint lap by the carry).
    let mut fuel_mass = vec![0.0; n];
    let mut fuel_a_f = vec![0.0; n];
    let mut fuel_h_cg = vec![0.0; n];
    let mut fuel_kg = vec![0.0; n];
    let fuel_start = fuel.map(|f| f.model.initial_kg);
    let mut ax = vec![0.0; n];
    let mut bufs = SlowMarchBuffers::new(n);
    // Previous-iteration copies for the recorded convergence residual (PR2c).
    let mut prev_scale = vec![1.0; n];
    let mut prev_deploy = vec![0.0; n];
    // Under-relaxed deploy-force slice fed to the solver. The deploy/harvest schedule reshapes the
    // very profile it was decided on, and at the charge-sustain equilibrium (SoC hovering at the
    // Recharge target) a straight station is bistable between greedy deploy and super-clip harvest;
    // feeding the raw marched slice back would limit-cycle. Damping the SOLVER-fed slice (never the
    // marched pack state or the reported channels) converges the fixed point deterministically. The
    // final reported channels come from a fresh march on the converged profile.
    let mut relaxed_deploy = vec![0.0; n];
    let mut convergence = MarchConvergence::default();
    // Tyre-thermal buffers: the envelope index `(T_tire, wear)` + the surfaced channels.
    let mut t_tire_k = vec![tire.map_or(0.0, TireThermalMarch::seed_surface_k); n];
    let mut wear_mm = vec![0.0; n];
    let mut tire_log = tire_slow_log(n);

    // Iteration 0: the uncoupled profile seeds the marches.
    let mut lap_time = solve_into_ggv(t0, env, path, ws)?;
    let mut prev_lap_time = lap_time;
    let iters = if ers.is_some() {
        OUTER_ITERS_ERS
    } else {
        OUTER_ITERS
    };
    for it in 0..iters {
        derive_ax(path, &ws.v, &mut ax);
        if let Some(c) = coupling {
            march_slow_states(t0, c, ers, env, path, &ws.v, &ax, &mut bufs);
            convergence.max_dscale = max_abs_diff(&bufs.scale, &prev_scale);
            convergence.max_ddeploy_n = max_abs_diff(&bufs.deploy_force_n, &prev_deploy);
            prev_scale.copy_from_slice(&bufs.scale);
            prev_deploy.copy_from_slice(&bufs.deploy_force_n);
            // Under-relax the solver-fed deploy slice (first pass = the raw march, no history).
            if it == 0 {
                relaxed_deploy.copy_from_slice(&bufs.deploy_force_n);
            } else {
                for (r, &d) in relaxed_deploy.iter_mut().zip(&bufs.deploy_force_n) {
                    *r = (1.0 - DEPLOY_RELAX) * *r + DEPLOY_RELAX * d;
                }
            }
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
        // Fuel burns along the current profile: the ICE-share fuel rate drains the tank and the
        // shrinking mass + migrating CG feed the next solve (D-M6-4). The deploy slice is the
        // realized electric share (all-zero without an ERS manager).
        if let (Some(fc), Some(f0)) = (fuel, fuel_start) {
            march_fuel(
                fc,
                env,
                path,
                &ws.v,
                &ax,
                &bufs.deploy_w,
                f0,
                &mut fuel_mass,
                &mut fuel_a_f,
                &mut fuel_h_cg,
            );
        }
        let scale_ref = coupling.map(|_| bufs.scale.as_slice());
        let deploy_ref = ers.and(coupling).map(|_| relaxed_deploy.as_slice());
        let tire_ref = tire.map(|_| (t_tire_k.as_slice(), wear_mm.as_slice()));
        let mass_cg_ref = fuel.map(|_| {
            (
                fuel_mass.as_slice(),
                fuel_a_f.as_slice(),
                fuel_h_cg.as_slice(),
            )
        });
        lap_time = solve_into_ggv_coupled(
            t0,
            env,
            scale_ref,
            deploy_ref,
            tire_ref,
            mass_cg_ref,
            path,
            ws,
        )?;
        convergence.dlap_s = (lap_time - prev_lap_time).abs();
        prev_lap_time = lap_time;
    }
    // Final marches against the converged profile so the reported channels match it.
    derive_ax(path, &ws.v, &mut ax);
    // The terminal pack + machine state the final march ends on (captured out of the closure so it
    // can seed the next stint lap). `None` here means no electro stack marched.
    let mut pack_terminal: Option<PackState> = None;
    let mut machine_terminal: Option<MachineThermal> = None;
    let slow = coupling.map(|c| {
        let outcome = march_slow_states(t0, c, ers, env, path, &ws.v, &ax, &mut bufs);
        pack_terminal = Some(outcome.pack_terminal);
        machine_terminal = outcome.machine_terminal;
        let stats = outcome.stats;
        SlowLog {
            state_of_charge: std::mem::take(&mut bufs.soc),
            // The machine-thermal network is NOT marched under an energy manager (D-M6-10: the
            // caps apply to the electric share, and no shipped `ers:` car pairs an `.emotor`), so
            // a manager-governed lap must not surface a frozen winding channel — it stays `None`
            // and `finish_notes` records the skip.
            machine_temp_c: (ers.is_none() && c.thermal.is_some())
                .then(|| std::mem::take(&mut bufs.temp_c)),
            ers: ers.map(|_| ErsSlowLog {
                deploy_power_w: std::mem::take(&mut bufs.deploy_w),
                harvest_power_w: std::mem::take(&mut bufs.harvest_w),
                ledger_deploy_j: stats.ledger_deploy_j,
                ledger_harvest_j: stats.ledger_harvest_j,
                soc_min: stats.soc_min,
                soc_max: stats.soc_max,
            }),
            convergence,
        }
    });
    let mut tire_terminal = None;
    let tire_slow = tire.map(|tm| {
        tire_terminal = Some(tm.march(
            t0,
            env,
            path,
            &ws.v,
            &ax,
            &mut t_tire_k,
            &mut wear_mm,
            &mut tire_log,
        ));
        tire_log
    });
    // Final fuel march on the converged profile so the terminal (carried) fuel matches it. The
    // per-station fuel-mass channel `fuel_kg[i] = mass[i] − dry_mass` is filled for the reported log.
    let fuel_terminal = match (fuel, fuel_start) {
        (Some(fc), Some(f0)) => {
            let term = march_fuel(
                fc,
                env,
                path,
                &ws.v,
                &ax,
                &bufs.deploy_w,
                f0,
                &mut fuel_mass,
                &mut fuel_a_f,
                &mut fuel_h_cg,
            );
            for (dst, &m) in fuel_kg.iter_mut().zip(fuel_mass.iter()) {
                *dst = m - fc.model.dry_mass_kg;
            }
            Some(term)
        }
        _ => None,
    };
    let terminal = SlowSnapshot {
        tire: tire_terminal,
        pack: pack_terminal,
        machine: machine_terminal,
        fuel_kg: fuel_terminal,
    };
    Ok((lap_time, slow, tire_slow, terminal))
}

/// Max elementwise |a − b| (equal lengths).
fn max_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// Validate the coupling bundle (the manager requires a pack to schedule).
fn check_couplings(couplings: &Couplings<'_>) -> Result<(), QssError> {
    if couplings.ers.is_some() && couplings.electro.is_none() {
        return Err(QssError::ErsCouplingWithoutPack);
    }
    Ok(())
}

/// Solve a `t0` lap: the point-mass velocity profile on the envelope, with the slow-state
/// couplings when supplied. Point-mass channels only.
///
/// # Errors
/// [`QssError::T0`] on a velocity-profile failure; [`QssError::ErsCouplingWithoutPack`] for a
/// manager coupling without the electro stack.
pub fn solve_t0(
    t0: &T0Vehicle,
    env: GgvEnvelope,
    couplings: &Couplings<'_>,
    path: &T0Path,
    req: LapRequest,
) -> Result<QssLap, QssError> {
    check_couplings(couplings)?;
    let mut ws = T0Workspace::for_path(path);
    let (lap_time_s, slow, tire_slow, slow_terminal) =
        solve_profile(t0, &env, couplings, path, &mut ws)?;
    let notes = finish_notes(req.notes, couplings, slow.as_ref());
    let lap = lap_result_from_ws(path, &ws, lap_time_s, req.line, req.resolved_hash, notes);
    Ok(QssLap {
        lap,
        tier: Tier::T0,
        fz_coupling: req.fz_coupling,
        flat_track: req.flat_track,
        wheels: None,
        setup: None,
        slow,
        tire: tire_slow,
        slow_terminal,
        envelope: Some(env),
    })
}

/// Solve a `t1` lap: the envelope velocity profile (coupled when couplings are supplied) plus a
/// per-station re-trim for the per-wheel channels and setup metrics.
///
/// # Errors
/// As [`solve_t0`].
pub fn solve_t1(
    t1: &T1Vehicle,
    t0: &T0Vehicle,
    env: GgvEnvelope,
    couplings: &Couplings<'_>,
    path: &T0Path,
    req: LapRequest,
) -> Result<QssLap, QssError> {
    check_couplings(couplings)?;
    let mut ws = T0Workspace::for_path(path);
    let (lap_time_s, slow, tire_slow, slow_terminal) =
        solve_profile(t0, &env, couplings, path, &mut ws)?;

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
            coupling: req.fz_coupling,
        };
        match t1.trim(&inp) {
            TrimOutcome::Converged(s) => wheels.push(s.fz, s.kappa, s.alpha, s.fx, s.fy),
            TrimOutcome::Infeasible { .. } => wheels.push_nan(),
        }
        understeer_gradient.push(t1.understeer_gradient(v.max(1e-3), gn).unwrap_or(f64::NAN));
        aero_front_share.push(t1.aero_front_downforce_share_at(v.max(1e-3)));
    }

    let notes = finish_notes(req.notes, couplings, slow.as_ref());
    let lap = lap_result_from_ws(path, &ws, lap_time_s, req.line, req.resolved_hash, notes);
    Ok(QssLap {
        lap,
        tier: Tier::T1,
        fz_coupling: req.fz_coupling,
        flat_track: req.flat_track,
        wheels: Some(wheels),
        setup: Some(SetupLog {
            understeer_gradient,
            aero_front_share,
        }),
        slow,
        tire: tire_slow,
        slow_terminal,
        envelope: Some(env),
    })
}

// ---------------------------------------------------------------------------------------------
// QSS stint: the multi-lap loop, pushed down from the Python binding so the lap-boundary carry
// semantics (the M6 acceptance check — SoC falls with consumption, rises with regeneration, across
// laps) are cargo-testable and the binding stops duplicating the solve_lap prologue (PR3a).
// ---------------------------------------------------------------------------------------------

/// The electro slow-stack ingredients a stint reuses across laps: the pack + optional machine
/// thermal network are cloned into a fresh [`SlowCoupling`] each lap, seeded with the CARRIED
/// pack/machine state (not the assembled one — that is the whole point of a stint).
#[derive(Clone, Copy)]
pub struct StintElectro<'a> {
    /// The T1 vehicle carrying the installed drive units + mass (as in [`SlowCoupling`]).
    pub vehicle: &'a T1Vehicle,
    /// The battery pack (immutable maps + limits), cloned per lap.
    pub pack: &'a Pack,
    /// The assembled machine-thermal network, if a drive unit declared an `.emotor` — the lap-1
    /// seed; later laps carry the terminal network forward (never marched under a manager).
    pub thermal: Option<&'a MachineThermal>,
    /// The lap-1 pack state (`initial_soc` already applied by assembly).
    pub pack_state: PackState,
    /// Assembly-time activity flag (installed energy maps), copied into each lap's coupling.
    pub active: bool,
}

/// The borrowed, assembled artifacts a QSS stint runs its laps on (built once, reused every lap):
/// the tier + vehicles + envelope + path, the optional electro stack / energy manager / tyre
/// march, and the per-lap request template. Passed to [`solve_stint`] by value.
pub struct StintPlan<'a> {
    /// The resolved solver tier (`T0` / `T1`).
    pub tier: Tier,
    /// The point-mass vehicle (drives both `t0` and `t1` laps).
    pub t0: &'a T0Vehicle,
    /// The T1 vehicle (per-wheel channels + the slow-stack mass/maps).
    pub t1: &'a T1Vehicle,
    /// The g-g-g-v envelope (built once — cloned into each lap's solve).
    pub env: &'a GgvEnvelope,
    /// The shared arc-length path all laps run on.
    pub path: &'a T0Path,
    /// The electro slow stack, when the car carries a runnable battery. `None` → no SoC to carry.
    pub electro: Option<StintElectro<'a>>,
    /// The 2026 ERS energy manager (requires `electro`).
    pub ers: Option<&'a ErsCoupling>,
    /// The representative-tyre thermal march (built once; re-seeded per lap). `None` → frozen tyre.
    pub base_march: Option<&'a TireThermalMarch>,
    /// The fuel-mass slow state (§8.1, D-M6-4). `None` → constant mass. Its `model.initial_kg` is the
    /// LAP-1 fuel load; later laps re-seed from the carried terminal fuel (lap N+1 starts lighter).
    pub fuel: Option<&'a FuelCoupling<'a>>,
    /// The per-lap request template (line / hash / fz-coupling / flat-track); cloned per lap with
    /// the notes reset (the per-lap manager notes are surfaced separately).
    pub request: LapRequest,
}

/// A single stint lap's lean result (no per-lap envelope — the stint reuses ONE): the lap time, the
/// point-mass speed trace, the slow-state + tyre channels, and the terminal snapshot the next lap
/// seeds from.
#[derive(Clone, Debug)]
pub struct StintLap {
    /// Lap time, s.
    pub lap_time_s: f64,
    /// Per-station speed, m/s.
    pub v: Vec<f64>,
    /// Slow-state channels (SoC trace, machine temp, ERS ledger) — `None` when no active stack.
    pub slow: Option<SlowLog>,
    /// Tyre-thermal channels — `None` when the tyre march was off.
    pub tire: Option<TireSlowLog>,
    /// The terminal slow state at the end of this lap (the next lap's seed).
    pub terminal: SlowSnapshot,
    /// The lap's energy-manager + convergence notes (`finish_notes` output).
    pub notes: Vec<String>,
}

/// The lap-1 seeds a stint starts from (beyond the pack state, which rides in [`StintElectro`]).
#[derive(Clone, Debug, Default)]
pub struct StintSeeds {
    /// The lap-1 representative-tyre state (`None` → the march's warm-at-optimum default).
    pub tire: Option<TireThermalState<f64>>,
}

/// A solved QSS stint: the shared stations, the resolved tier, and the per-lap lean results.
pub struct QssStintResult {
    /// The shared arc-length stations (from lap 1).
    pub s: Vec<f64>,
    /// The resolved tier (`T0` / `T1`).
    pub tier: Tier,
    /// The recorded normal-load coupling mode.
    pub fz_coupling: FzCoupling,
    /// Per-lap results, in lap order.
    pub laps: Vec<StintLap>,
}

/// Run a QSS stint: `n_laps` laps on the shared [`StintPlan`] artifacts, carrying the FULL slow
/// stack — representative-tyre state AND the battery pack (SoC / RC voltage / temperature) AND the
/// machine-thermal network — across every lap boundary with no reset. Lap N+1 seeds from lap N's
/// terminal [`SlowSnapshot`]; only the per-lap ERS ledger resets (the `s = 0` lap boundary).
///
/// # Errors
/// Propagates any per-lap [`solve_t0`] / [`solve_t1`] failure.
#[allow(clippy::too_many_lines, clippy::needless_pass_by_value)]
pub fn solve_stint(
    plan: &StintPlan<'_>,
    n_laps: usize,
    seeds: StintSeeds,
) -> Result<QssStintResult, QssError> {
    let mut laps = Vec::with_capacity(n_laps);
    let mut s_grid = Vec::new();
    // The carried seeds — updated from each lap's terminal snapshot (the stint's whole point).
    let mut tire_seed = seeds.tire;
    let mut pack_seed = plan.electro.map(|e| e.pack_state);
    // The carried fuel mass (kg): lap-1 uses the model's initial load, later laps the prior
    // terminal (a lighter car ⇒ a faster lap, the D-M6-4 acceptance check).
    let mut fuel_seed = plan.fuel.map(|f| f.model.initial_kg);
    // The machine-thermal network re-seeds from the assembled state each lap (see the boundary note
    // below): a constant lap-1 seed, never carried, so it is not `mut`.
    let machine_seed = plan.electro.and_then(|e| e.thermal.cloned());

    for lap_idx in 0..n_laps {
        let march_lap = plan.base_march.map(|bm| {
            bm.clone()
                .with_state(tire_seed.expect("tyre seed set when a march is supplied"))
        });
        let electro = plan.electro.map(|e| SlowCoupling {
            vehicle: e.vehicle,
            thermal: machine_seed.clone(),
            pack: e.pack.clone(),
            pack_state: pack_seed.expect("pack seed set when an electro stack is supplied"),
            active: e.active,
        });
        // Re-seed the fuel model with this lap's starting mass (carried from the prior lap's terminal).
        let fuel_lap = plan.fuel.map(|f| {
            let mut model = f.model;
            model.initial_kg = fuel_seed.expect("fuel seed set when a fuel coupling is supplied");
            FuelCoupling {
                model,
                vehicle: f.vehicle,
            }
        });
        let couplings = Couplings {
            electro: electro.as_ref(),
            tire: march_lap.as_ref(),
            ers: plan.ers,
            fuel: fuel_lap.as_ref(),
        };
        let mut req = plan.request.clone();
        req.notes = Vec::new();
        let qss = if plan.tier == Tier::T0 {
            solve_t0(plan.t0, plan.env.clone(), &couplings, plan.path, req)?
        } else {
            solve_t1(
                plan.t1,
                plan.t0,
                plan.env.clone(),
                &couplings,
                plan.path,
                req,
            )?
        };
        if lap_idx == 0 {
            s_grid.clone_from(&qss.lap.s);
        }
        // Carry the terminal slow state into the next lap's seed BEFORE moving the lap result. A
        // `None` terminal (subsystem absent this lap) keeps the prior seed rather than resetting it.
        tire_seed = qss.slow_terminal.tire.or(tire_seed);
        if let Some(p) = qss.slow_terminal.pack {
            // Carry the SLOW pack states — SoC and temperature — across the lap boundary (the
            // headline acceptance check). The RC overpotential `v_rc_v` and the last-solved
            // terminal `current_a` are within-lap transients (τ ~ seconds), re-established in the
            // new lap's first segments; carrying them would feed a stale end-of-straight
            // high-current terminal-voltage estimate into station 0. Reset them to match the
            // single-lap entry, which starts from a rested pack.
            pack_seed = Some(PackState {
                v_rc_v: 0.0,
                v_rc2_v: 0.0,
                current_a: 0.0,
                ..p
            });
        }
        // Carry the terminal fuel mass into the next lap (a lighter car ⇒ a faster lap, D-M6-4). A
        // `None` terminal keeps the prior seed rather than resetting to the full tank.
        fuel_seed = qss.slow_terminal.fuel_kg.or(fuel_seed);
        // The machine-thermal network is NOT carried across the QSS lap boundary — it re-seeds each
        // lap (the terminal is still surfaced as an end-of-lap diagnostic). Coupling a near-limit
        // winding temperature into the quasi-steady DISTANCE march creates a derate↔slowdown
        // positive feedback (a slower lap integrates MORE heating over its longer time, so the
        // winding gets hotter, derates harder, slows further) with no inter-lap cooling to arrest
        // it — an artifact of the QSS march, not real thermal behaviour. Inter-lap machine-thermal
        // continuity is the transient tier's job (T2, with real-time cooling); this QSS EV-stint
        // asymmetry is recorded (§13 validation). Under an energy manager the machine is not marched
        // at all (D-M6-10), so this only affects mapped-EV stints.
        laps.push(StintLap {
            lap_time_s: qss.lap.lap_time_s,
            v: qss.lap.v,
            slow: qss.slow,
            tire: qss.tire,
            terminal: qss.slow_terminal,
            notes: qss.lap.notes,
        });
    }

    Ok(QssStintResult {
        s: s_grid,
        tier: plan.tier,
        fz_coupling: plan.request.fz_coupling,
        laps,
    })
}

/// Append the energy-manager + convergence notes to the request's assembly notes.
fn finish_notes(
    mut notes: Vec<String>,
    couplings: &Couplings<'_>,
    slow: Option<&SlowLog>,
) -> Vec<String> {
    if let Some(e) = couplings.ers {
        let recharge = if e.manager.rulebook().recharge_phases() {
            "on"
        } else {
            "off"
        };
        notes.push(format!(
            "2026 ERS energy manager active: deploy curve + per-lap Recharge budget enforced, \
             recharge phases {recharge}, override {}",
            if e.override_active { "active" } else { "off" }
        ));
        if let Some(log) = slow.and_then(|s| s.ers.as_ref()) {
            // The on-track SoC swing in MJ (D-M6-3, "record not clamp"): the FIA C5.2.9 window is
            // max − min SoC ≤ 4 MJ. The pack owns the total energy the normalized SoC spans.
            let swing = log.soc_max - log.soc_min;
            let swing_mj = couplings
                .electro
                .map(|c| swing * c.pack.total_energy_j() * 1e-6);
            let swing_note = match swing_mj {
                Some(mj) => format!("{swing:.3} ≈ {mj:.2} MJ"),
                None => format!("{swing:.3}"),
            };
            notes.push(format!(
                "ERS lap energy: deployed {:.3} MJ, harvested {:.3} MJ (electrical); on-track \
                 SoC swing {swing_note} (max − min, recorded per FIA C5.2.9 ≤ 4 MJ window)",
                log.ledger_deploy_j * 1e-6,
                log.ledger_harvest_j * 1e-6,
            ));
        }
        // A manager-governed lap does NOT march the machine-thermal network (D-M6-10); if a car
        // paired an `.emotor` with `ers:`, say so rather than leave a frozen channel unexplained.
        if couplings.electro.is_some_and(|c| c.thermal.is_some()) {
            notes.push(
                "machine-thermal network present but NOT marched under the energy manager \
                 (M6 PR2: the caps apply to the electric share) — no winding-temperature channel \
                 is reported for this lap"
                    .to_owned(),
            );
        }
        if let Some(s) = slow {
            notes.push(format!(
                "QSS outer-iteration convergence (last two passes): max |Δscale| {:.2e}, \
                 max |Δdeploy force| {:.2e} N, |Δlap time| {:.2e} s",
                s.convergence.max_dscale, s.convergence.max_ddeploy_n, s.convergence.dlap_s,
            ));
        }
    }
    notes
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
