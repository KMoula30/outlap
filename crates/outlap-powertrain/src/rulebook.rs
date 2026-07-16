// SPDX-License-Identifier: AGPL-3.0-only
//! The ERS rulebook — the FIA 2026 energy-recovery regulations as pure data + queries.
//!
//! Built once from the loaded `ers:` vehicle-schema block. All regulatory caps and budgets live
//! on the ELECTRICAL side — the CU-K DC bus, where C5.2.7 (±350 kW both directions) and C5.2.10
//! (the per-lap Recharge budget) are written — with ONE conversion seam to the mechanical crank
//! side: the fixed electrical→mechanical factor of C5.2.14 (deploy) and its inverse per C5.2.21
//! (harvest). Unit conversions (km/h → m/s, kW → W, MJ → J) happen HERE and nowhere else.
//!
//! # Symbols / articles
//!
//! * Deployment taper — C5.2.8(i): `P(kW) = 1800 − 5·v(kph)` for `v < 340`, `6900 − 20·v` for
//!   `340 ≤ v < 345`, `0` at `≥ 345`, always min-composed with the C5.2.7 350 kW cap. The
//!   breakpoint form (`[0, 290, 340, 345] / [1, 1, 2/7, 0]`) reproduces `min(cap, curve)`
//!   exactly under piecewise-LINEAR evaluation — which is why the taper is a
//!   [`PiecewiseLinear`], the recorded Decision #30 exception (closed-form regulation lines,
//!   not gridded maps).
//! * Override taper — C5.2.8(ii): `P = 7100 − 20·v`, zero at ≥ 355 km/h.
//! * Per-lap harvest budget — C5.2.10; the +0.5 MJ granted with Override active is extra
//!   HARVEST allowance (C5.2.10(iii)), not a deployment budget.
//! * Per-lap deployment budget — NONE in the 2026 regulations (C5.2, absence verified);
//!   `per_lap_deploy_mj` is generic config for non-F1 rule sets and is never estimated.
//! * Recharge-phase ramp — C5.12.4–C5.12.7, simplified to three bounds: initial demand step
//!   ≤ `ramp_initial_step_kw` (150), thereafter ≤ `ramp_rate_kw_per_s` (50), total reduction
//!   ≤ `ramp_total_kw` (700). The ≥ 1 s hold, the two-regime rate rule, and the 210 km/h /
//!   gearshift carve-outs are recorded as not-modelled in the theory page.
//! * MGU-K torque cap — C5.2.11 (500 Nm crank-referenced): carried by the MGU-K `.ptm` torque
//!   envelope, min-composed on the mechanical side by [`ErsRulebook::torque_capped_mech_w`].

use num_traits::Float;
use outlap_core::{InterpError, MonotoneCubic, PiecewiseLinear};
use outlap_schema::vehicle::Ers;

/// km/h → m/s.
const KPH_TO_MPS: f64 = 1.0 / 3.6;
/// The C5.2.14 fixed electrical→mechanical conversion factor (default when the schema is silent).
const DEFAULT_ELEC_MECH_FACTOR: f64 = 0.97;
/// C5.12.4 default maximum initial power-demand step, W.
const DEFAULT_RAMP_INITIAL_STEP_W: f64 = 150e3;
/// C5.12.5 default maximum demand-reduction rate, W/s (the conservative bound of 50–100 kW/s).
const DEFAULT_RAMP_RATE_W_PER_S: f64 = 50e3;
/// C5.12.6 default maximum total demand reduction, W.
const DEFAULT_RAMP_TOTAL_W: f64 = 700e3;

/// Error building an [`ErsRulebook`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum RulebookError {
    /// A taper table failed to build (too few knots, length mismatch, non-ascending speeds).
    #[error("taper table: {0}")]
    Taper(#[from] InterpError),
    /// A taper's power fraction rises with speed — meaningless for a de-rate curve. The semantic
    /// validation stage rejects this with a spanned error; this is the typed defense in depth for
    /// rulebooks built from in-memory blocks.
    #[error("`{table}` power_frac must be monotone non-increasing with speed")]
    TaperNotMonotone {
        /// Which taper table (`deployment` or `override_mode`).
        table: &'static str,
    },
}

/// The FIA-2026-style ERS regulations for one car, in SI units, generic over `f32`/`f64`.
///
/// Construct with [`ErsRulebook::from_schema`]; query with the methods below. Everything is
/// immutable after construction and allocation-free to query.
#[derive(Clone, Debug)]
pub struct ErsRulebook<T> {
    /// Peak deployment power at the CU-K DC bus, W (C5.2.7).
    p_deploy_cap_w: T,
    /// Deployment power-fraction taper vs speed (m/s), piecewise-linear (C5.2.8(i)).
    deploy_taper: PiecewiseLinear<T>,
    /// Override envelope: (peak power W, taper), when the car has an override mode (C5.2.8(ii)).
    override_env: Option<(T, PiecewiseLinear<T>)>,
    /// Optional per-lap deployment budget, J electrical. `None` = unbounded (the 2026 rules).
    per_lap_deploy_j: Option<T>,
    /// Peak harvest power at the CU-K DC bus, W (C5.2.7 — both directions).
    p_harvest_cap_w: T,
    /// Per-lap harvest ("Recharge") budget, J electrical (C5.2.10).
    per_lap_harvest_j: T,
    /// Extra per-lap HARVEST allowance with Override active, J (C5.2.10(iii)).
    override_extra_harvest_j: T,
    /// Fixed electrical→mechanical conversion factor (C5.2.14; inverse direction per C5.2.21).
    elec_mech_factor: T,
    /// Whether the automated recharge phases (part-throttle / full-throttle back-drive) run.
    recharge_phases: bool,
    /// Target pack SoC the recharge paths steer toward (default: mid `soc_window`).
    recharge_target_soc: T,
    /// C5.12.4 maximum initial power-demand step, W.
    ramp_initial_step_w: T,
    /// C5.12.5 maximum demand-reduction rate after the initial step, W/s.
    ramp_rate_w_per_s: T,
    /// C5.12.6 maximum total demand reduction, W.
    ramp_total_w: T,
    /// MGU-K mechanical torque envelope τ(ω), Nm over rad/s — a gridded map, so it stays on the
    /// shared Hermite (Decision #30 proper). Carries the C5.2.11 crank torque cap for f1-class
    /// data. `None` when the caller has no `.ptm` limits at hand (pure rulebook tests).
    torque_env: Option<MonotoneCubic<T>>,
}

/// Cast an f64 schema value into the rulebook scalar type (f32/f64: always representable).
fn c<T: Float>(x: f64) -> T {
    T::from(x).expect("f64 → Float cast")
}

impl<T: Float> ErsRulebook<T> {
    /// Build the rulebook from a resolved `ers:` schema block plus (optionally) the MGU-K `.ptm`
    /// torque envelope `(ω rad/s, τ Nm)` — SI, already converted from RPM at the loading boundary.
    ///
    /// # Errors
    /// [`RulebookError`] if a taper table is malformed or its power fraction rises with speed.
    pub fn from_schema(
        ers: &Ers,
        mgu_k_torque: Option<(&[f64], &[f64])>,
    ) -> Result<Self, RulebookError> {
        let deploy_taper = build_taper::<T>(&ers.deployment.taper_vs_speed, "deployment")?;
        let override_env = ers
            .override_mode
            .as_ref()
            .map(|om| {
                Ok::<_, RulebookError>((
                    c::<T>(om.power_limit_kw * 1e3),
                    build_taper::<T>(&om.taper_vs_speed, "override_mode")?,
                ))
            })
            .transpose()?;
        let torque_env = mgu_k_torque.map(|(omega, torque)| {
            let xs: Vec<T> = omega.iter().map(|&w| c(w)).collect();
            let ys: Vec<T> = torque.iter().map(|&t| c(t)).collect();
            MonotoneCubic::new(xs, ys).map_err(RulebookError::from)
        });
        let torque_env = torque_env.transpose()?;

        let [soc_lo, soc_hi] = ers.es.soc_window;
        let two = c::<T>(2.0);
        Ok(Self {
            p_deploy_cap_w: c(ers.deployment.power_limit_kw * 1e3),
            deploy_taper,
            override_env,
            per_lap_deploy_j: ers.deployment.per_lap_deploy_mj.map(|mj| c(mj * 1e6)),
            p_harvest_cap_w: c(ers.recovery.braking_power_limit_kw * 1e3),
            per_lap_harvest_j: c(ers.recovery.per_lap_harvest_mj * 1e6),
            override_extra_harvest_j: c(ers
                .override_mode
                .as_ref()
                .and_then(|om| om.extra_energy_per_lap_mj)
                .unwrap_or(0.0)
                * 1e6),
            elec_mech_factor: c(ers.elec_mech_factor.unwrap_or(DEFAULT_ELEC_MECH_FACTOR)),
            recharge_phases: ers.recovery.recharge_phases,
            recharge_target_soc: ers
                .recovery
                .recharge_target_soc
                .map_or_else(|| (c::<T>(soc_lo) + c::<T>(soc_hi)) / two, c),
            ramp_initial_step_w: ers
                .recovery
                .ramp_initial_step_kw
                .map_or(c(DEFAULT_RAMP_INITIAL_STEP_W), |kw| c(kw * 1e3)),
            ramp_rate_w_per_s: ers
                .recovery
                .ramp_rate_kw_per_s
                .map_or(c(DEFAULT_RAMP_RATE_W_PER_S), |kw| c(kw * 1e3)),
            ramp_total_w: ers
                .recovery
                .ramp_total_kw
                .map_or(c(DEFAULT_RAMP_TOTAL_W), |kw| c(kw * 1e3)),
            torque_env,
        })
    }

    /// The electrical deployment cap at vehicle speed `v` (m/s), W — `min(cap, cap·taper(v))`,
    /// using the override envelope (C5.2.8(ii)) when `override_active` and the car has one,
    /// else the base envelope (C5.2.8(i)). Never negative.
    pub fn deploy_cap_electrical_w(&self, v: T, override_active: bool) -> T {
        let (cap, taper) = match (&self.override_env, override_active) {
            (Some((cap, taper)), true) => (*cap, taper),
            _ => (self.p_deploy_cap_w, &self.deploy_taper),
        };
        let frac = taper.eval(v).max(T::zero()).min(T::one());
        (cap * frac).max(T::zero())
    }

    /// The electrical harvest cap, W (C5.2.7 / `recovery.braking_power_limit_kw`).
    pub fn harvest_cap_electrical_w(&self) -> T {
        self.p_harvest_cap_w
    }

    /// The per-lap harvest budget, J electrical — `per_lap_harvest_mj` plus the C5.2.10(iii)
    /// override bonus on laps where the override flag is set.
    pub fn harvest_budget_j(&self, override_active: bool) -> T {
        if override_active {
            self.per_lap_harvest_j + self.override_extra_harvest_j
        } else {
            self.per_lap_harvest_j
        }
    }

    /// The optional per-lap deployment budget, J electrical. `None` = unbounded (2026 F1).
    pub fn per_lap_deploy_j(&self) -> Option<T> {
        self.per_lap_deploy_j
    }

    /// Mechanical crank power delivered for an electrical deploy power (C5.2.14): `p · 0.97`.
    pub fn mech_deploy_w(&self, p_electrical: T) -> T {
        p_electrical * self.elec_mech_factor
    }

    /// Electrical DC power banked for a mechanical harvest power (C5.2.21, the inverse
    /// direction): `p · 0.97` — at the 350 kW electrical cap the axle may absorb ≈ 360.8 kW.
    pub fn elec_harvest_w(&self, p_mechanical: T) -> T {
        p_mechanical * self.elec_mech_factor
    }

    /// Mechanical harvest power the axle must absorb for an electrical target: `p / 0.97`.
    pub fn mech_harvest_w(&self, p_electrical: T) -> T {
        p_electrical / self.elec_mech_factor
    }

    /// Min-compose a mechanical power with the MGU-K torque envelope at shaft speed `omega`
    /// (rad/s): `min(p, τ(ω)·ω)`. Identity when no envelope was supplied. This is where the
    /// C5.2.11 crank torque cap binds (before the power cap at low speed).
    pub fn torque_capped_mech_w(&self, p_mech_w: T, omega: T) -> T {
        match &self.torque_env {
            Some(env) => p_mech_w.min(env.eval(omega).max(T::zero()) * omega.max(T::zero())),
            None => p_mech_w,
        }
    }

    /// Whether the automated recharge phases (part-throttle harvest, full-throttle back-drive)
    /// are enabled for this car.
    pub fn recharge_phases(&self) -> bool {
        self.recharge_phases
    }

    /// The pack SoC the automated Recharge paths steer toward.
    pub fn recharge_target_soc(&self) -> T {
        self.recharge_target_soc
    }

    /// The K-power demand reduction allowed by the C5.12 ramp this step, W, given the cumulative
    /// reduction already taken in the active ramp episode (`reduced_so_far_w`; the caller resets
    /// it to zero when demand rises and the episode ends).
    ///
    /// Simplified per D-M6-5: the first reduction of an episode may step by up to
    /// `ramp_initial_step_kw` instantly; each subsequent step is rate-limited to
    /// `ramp_rate_kw_per_s · dt`; the episode total is capped at `ramp_total_kw`.
    pub fn ramp_allowed_reduction_w(&self, reduced_so_far_w: T, dt: T) -> T {
        let remaining = (self.ramp_total_w - reduced_so_far_w).max(T::zero());
        let step = if reduced_so_far_w <= T::zero() {
            self.ramp_initial_step_w
        } else {
            self.ramp_rate_w_per_s * dt
        };
        step.min(remaining)
    }
}

/// Build a piecewise-linear taper `frac(v m/s)` from a schema `SpeedTaper` (km/h → m/s at this
/// boundary), enforcing the monotone non-increasing contract.
fn build_taper<T: Float>(
    taper: &outlap_schema::vehicle::SpeedTaper,
    table: &'static str,
) -> Result<PiecewiseLinear<T>, RulebookError> {
    let xs: Vec<T> = taper.speed_kph.iter().map(|&s| c(s * KPH_TO_MPS)).collect();
    let ys: Vec<T> = taper.power_frac.iter().map(|&f| c(f)).collect();
    let pl = PiecewiseLinear::new(xs, ys)?;
    if !pl.is_non_increasing() {
        return Err(RulebookError::TaperNotMonotone { table });
    }
    Ok(pl)
}
