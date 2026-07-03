// SPDX-License-Identifier: AGPL-3.0-only
//! Assembly stage: reduce a [`ResolvedVehicle`] + `conditions` into a compact [`T0Vehicle`].
//!
//! This is a cold path (allocations allowed): it re-loads the referenced `.ptm`/`.tyr` files,
//! derives the constant friction coefficients from the tyre MF6.1 peak factors, lumps the aero
//! coefficients with air density, and folds each drive unit's coupler path into a set of gears so
//! the hot [`T0Vehicle::tractive_force`] query is allocation-free.

use outlap_core::MonotoneCubic;
use outlap_schema::conditions::Conditions;
use outlap_schema::io::SourceLoader;
use outlap_schema::load::{load_ptm, load_tyr};
use outlap_schema::ptm::TorqueCurve;
use outlap_schema::tyr::Tyr;
use outlap_schema::vehicle::{Coupler, Efficiency, Gearbox};
use outlap_schema::ResolvedVehicle;

use crate::error::T0Error;
use crate::DEFAULT_DS_M;

/// Revolutions per minute → radians per second.
const RPM_TO_RAD_PER_S: f64 = std::f64::consts::PI / 30.0;
/// Kilometres per hour → metres per second.
const KPH_TO_MPS: f64 = 1000.0 / 3600.0;
/// Specific gas constant for dry air, J/(kg·K).
const DRY_AIR_R: f64 = 287.05;
/// Speed floor for the ERS power→force conversion (the friction ellipse caps launch force anyway).
const ERS_V_FLOOR_MPS: f64 = 1.0;

/// Options controlling T0 assembly and solving.
#[derive(Clone, Debug)]
pub struct T0Options {
    /// Arc-length step for the passes, metres (default [`DEFAULT_DS_M`]).
    pub ds_m: f64,
    /// Upper speed bound for the curvature-limited speed, m/s (safety cap; default 150).
    pub v_cap: f64,
    /// Max forward/backward pass iterations before declaring divergence on a closed lap (default 4).
    pub max_pass_iterations: usize,
    /// Allow documented-degraded combinations (e.g. missing constant aero → zero aero), recorded in
    /// the notes (Decision #40).
    pub allow_degraded: bool,
}

impl Default for T0Options {
    fn default() -> Self {
        Self {
            ds_m: DEFAULT_DS_M,
            v_cap: 150.0,
            max_pass_iterations: 4,
            allow_degraded: false,
        }
    }
}

/// A point-mass reduction of a vehicle for the T0 tier.
#[derive(Clone, Debug)]
pub struct T0Vehicle {
    /// Total mass, kg.
    pub mass_kg: f64,
    /// Longitudinal friction coefficient (MF6.1 `PDX1·LMUX`, mean of axles).
    pub mu_x: f64,
    /// Lateral friction coefficient (MF6.1 `PDY1·LMUY`, mean of axles).
    pub mu_y: f64,
    /// Lumped drag term `½·ρ·CxA` (N per (m/s)²).
    pub qx: f64,
    /// Lumped downforce term `½·ρ·CzA` (N per (m/s)²).
    pub qz: f64,
    /// Curvature-limited-speed cap, m/s.
    pub v_cap: f64,
    units: Vec<T0Unit>,
    ers: Option<T0Ers>,
    notes: Vec<String>,
}

/// One drive unit reduced to a peak-torque envelope and a set of gears.
#[derive(Clone, Debug)]
struct T0Unit {
    /// Peak torque vs shaft speed [rad/s → N·m].
    torque_env: MonotoneCubic<f64>,
    /// Highest shaft speed the envelope covers, rad/s (gears past this are rev-limited out).
    omega_max: f64,
    gears: Vec<T0Gear>,
}

/// A single selectable gear, precomputed for the point-mass force query.
#[derive(Clone, Copy, Debug)]
struct T0Gear {
    /// Shaft speed per unit vehicle speed, (rad/s)/(m/s) = total ratio / wheel radius.
    omega_per_v: f64,
    /// Wheel force per unit shaft torque, N/(N·m) = total ratio · efficiency / wheel radius.
    force_per_torque: f64,
}

/// An ERS reduced to a power-capped tractive-force contribution.
#[derive(Clone, Debug)]
struct T0Ers {
    /// Peak deployment power, W.
    p_deploy_w: f64,
    /// Machine mechanical power ceiling `max(τ·ω)` over the map, W (ratio-invariant).
    p_mech_max_w: f64,
    /// Driveline efficiency applied to the ERS force.
    eta: f64,
    /// Power fraction vs vehicle speed [m/s → 0..1].
    taper: MonotoneCubic<f64>,
}

impl T0Vehicle {
    /// Assemble a [`T0Vehicle`] from a resolved vehicle, session conditions, and a source loader.
    pub fn assemble(
        vehicle: &ResolvedVehicle,
        conditions: &Conditions,
        loader: &dyn SourceLoader,
        opts: &T0Options,
    ) -> Result<Self, T0Error> {
        let spec = &vehicle.spec;
        let mut notes = Vec::new();

        // --- Tyres: friction coefficients (mean of axles) + driven-axle radii ---
        let (front, _) = load_tyr(spec.tires.front.as_str(), loader)?;
        let (rear, _) = load_tyr(spec.tires.rear.as_str(), loader)?;
        let mu_x = 0.5 * (mu(&front, "PDX1", "LMUX") + mu(&rear, "PDX1", "LMUX"));
        let mu_y = 0.5 * (mu(&front, "PDY1", "LMUY") + mu(&rear, "PDY1", "LMUY"));
        let r_front = coeff(&front, "UNLOADED_RADIUS", 0.33);
        let r_rear = coeff(&rear, "UNLOADED_RADIUS", 0.33);

        // --- Aero + air density (ideal gas) ---
        let rho = 100.0 * conditions.air.pressure_hpa
            / (DRY_AIR_R * (conditions.air.temperature_c + 273.15));
        let (qx, qz) = match &spec.aero.constant {
            Some(c) => (
                0.5 * rho * c.cx_a_m2,
                0.5 * rho * (c.cz_front_a_m2 + c.cz_rear_a_m2),
            ),
            None if opts.allow_degraded => {
                notes.push(
                    "aero.constant absent — T0 ran with zero aero (ride-height map arrives with T1)"
                        .to_owned(),
                );
                (0.0, 0.0)
            }
            None => return Err(T0Error::NoConstantAero),
        };

        // --- Drive units ---
        let mut units = Vec::with_capacity(spec.drivetrain.units.len());
        for (i, unit) in spec.drivetrain.units.iter().enumerate() {
            let ptm = load_ptm(unit.source.as_str(), loader)?;
            let r_wheel = driven_radius(unit, r_front, r_rear, i, &mut notes);
            let (base_ratio, base_eff, gearbox) = fold_path(&unit.path, i, &mut notes)?;
            let torque_env = torque_env(&ptm.limits.max_torque_nm_vs_speed)?;
            let omega_max = torque_env.domain().1;
            let gears = build_gears(base_ratio, base_eff, gearbox, r_wheel);
            units.push(T0Unit {
                torque_env,
                omega_max,
                gears,
            });
        }

        // --- ERS (power-capped force; schema gives the MGU-K no path/ratio) ---
        let ers = match &spec.ers {
            Some(e) => {
                let ptm = load_ptm(e.mgu_k.as_str(), loader)?;
                let curve = &ptm.limits.max_torque_nm_vs_speed;
                let p_mech_max_w = curve
                    .speed_rpm
                    .iter()
                    .zip(&curve.torque_nm)
                    .map(|(rpm, t)| (rpm * RPM_TO_RAD_PER_S) * t.abs())
                    .fold(0.0_f64, f64::max);
                let taper = taper_env(&e.deployment.taper_vs_speed)?;
                let eta = single_gearbox_eff(spec).unwrap_or(1.0);
                notes.push(
                    "ERS modelled as a power cap; per-lap deploy/harvest budgets and override mode \
                     are not enforced at T0"
                        .to_owned(),
                );
                Some(T0Ers {
                    p_deploy_w: e.deployment.power_limit_kw * 1000.0,
                    p_mech_max_w,
                    eta,
                    taper,
                })
            }
            None => None,
        };

        if units.is_empty() && ers.is_none() {
            return Err(T0Error::NoDrive);
        }

        notes.push(
            "μ from tyre MF6.1 PD*·LMU* (mean of front/rear); braking is friction-limited only at T0"
                .to_owned(),
        );

        Ok(Self {
            mass_kg: spec.chassis.mass_kg,
            mu_x,
            mu_y,
            qx,
            qz,
            v_cap: opts.v_cap,
            units,
            ers,
            notes,
        })
    }

    /// Total tractive force available at vehicle speed `v` (m/s), N — the sum over drive units of
    /// the best gear's wheel force plus the power-capped ERS contribution. Allocation-free.
    pub fn tractive_force(&self, v: f64) -> f64 {
        let mut force = 0.0;
        for unit in &self.units {
            let mut best = 0.0;
            for g in &unit.gears {
                let omega = g.omega_per_v * v;
                if omega <= unit.omega_max {
                    let f = unit.torque_env.eval(omega) * g.force_per_torque;
                    if f > best {
                        best = f;
                    }
                }
            }
            force += best;
        }
        if let Some(e) = &self.ers {
            let frac = e.taper.eval(v).clamp(0.0, 1.0);
            let power = (e.p_deploy_w * frac).min(e.p_mech_max_w).max(0.0);
            force += e.eta * power / v.max(ERS_V_FLOOR_MPS);
        }
        force
    }

    /// Human-readable notes on T0 simplifications and any degradations (nothing silent).
    pub fn notes(&self) -> &[String] {
        &self.notes
    }
}

/// A friction coefficient from a peak factor × its scaling factor (scaling defaults to 1.0).
fn mu(tyr: &Tyr, peak: &str, scale: &str) -> f64 {
    coeff(tyr, peak, 0.0) * coeff(tyr, scale, 1.0)
}

/// An MF6.1 coefficient, or `default` if absent.
fn coeff(tyr: &Tyr, key: &str, default: f64) -> f64 {
    tyr.mf61.0.get(key).copied().unwrap_or(default)
}

/// The driven-axle wheel radius for a unit (mean if it spans both axles).
fn driven_radius(
    unit: &outlap_schema::vehicle::DriveUnit,
    r_front: f64,
    r_rear: f64,
    index: usize,
    notes: &mut Vec<String>,
) -> f64 {
    let front = unit.wheels.iter().any(|w| w.is_front());
    let rear = unit.wheels.iter().any(|w| !w.is_front());
    match (front, rear) {
        (true, false) => r_front,
        (false, _) => r_rear, // rear-only, or no wheels declared → rear radius
        (true, true) => {
            if (r_front - r_rear).abs() > 1e-3 {
                notes.push(format!(
                    "drive unit {index} spans both axles; using the mean tyre radius"
                ));
            }
            0.5 * (r_front + r_rear)
        }
    }
}

/// Fold a coupler path into `(base_ratio, base_efficiency, optional gearbox)`. The gearbox (if any)
/// supplies the selectable gear ratios; fixed ratios multiply into `base_ratio`; diffs are 1:1.
fn fold_path<'a>(
    path: &'a [Coupler],
    unit: usize,
    notes: &mut Vec<String>,
) -> Result<(f64, f64, Option<&'a Gearbox>), T0Error> {
    let mut base_ratio = 1.0;
    let mut base_eff = 1.0;
    let mut gearbox: Option<&Gearbox> = None;
    for coupler in path {
        match coupler {
            Coupler::FixedRatio(r) => base_ratio *= r,
            Coupler::Diff(_) => {} // 1:1, unity efficiency at the point-mass level
            Coupler::Gearbox(g) => {
                base_eff *= efficiency_constant(&g.efficiency, unit)?;
                if gearbox.is_none() {
                    gearbox = Some(g);
                } else {
                    // A second gearbox in one path is unusual; fold it as a fixed reduction.
                    base_ratio *= g.final_drive * g.ratios.first().copied().unwrap_or(1.0);
                    notes.push(format!(
                        "drive unit {unit} has multiple gearboxes; only the first provides gears"
                    ));
                }
            }
        }
    }
    Ok((base_ratio, base_eff, gearbox))
}

/// A constant efficiency, or an error if it is a (not-yet-readable) gridded map.
fn efficiency_constant(eff: &Efficiency, unit: usize) -> Result<f64, T0Error> {
    match eff {
        Efficiency::Constant(e) => Ok(*e),
        Efficiency::Map { .. } => Err(T0Error::UnsupportedEfficiencyMap { unit }),
    }
}

/// Expand a folded path into gears (one per gearbox ratio, or a single direct-drive gear).
fn build_gears(
    base_ratio: f64,
    base_eff: f64,
    gearbox: Option<&Gearbox>,
    r_wheel: f64,
) -> Vec<T0Gear> {
    match gearbox {
        Some(g) => g
            .ratios
            .iter()
            .map(|&rk| {
                let ratio = base_ratio * rk * g.final_drive;
                T0Gear {
                    omega_per_v: ratio / r_wheel,
                    force_per_torque: ratio * base_eff / r_wheel,
                }
            })
            .collect(),
        None => vec![T0Gear {
            omega_per_v: base_ratio / r_wheel,
            force_per_torque: base_ratio * base_eff / r_wheel,
        }],
    }
}

/// Fit a peak-torque envelope `τ(ω)` from a speed/torque curve (rpm → rad/s at the boundary).
fn torque_env(curve: &TorqueCurve) -> Result<MonotoneCubic<f64>, T0Error> {
    let omega: Vec<f64> = curve
        .speed_rpm
        .iter()
        .map(|r| r * RPM_TO_RAD_PER_S)
        .collect();
    MonotoneCubic::new(omega, curve.torque_nm.clone()).map_err(T0Error::from)
}

/// Fit an ERS taper envelope `frac(v)` from a speed/fraction taper (kph → m/s at the boundary).
fn taper_env(taper: &outlap_schema::vehicle::SpeedTaper) -> Result<MonotoneCubic<f64>, T0Error> {
    let v: Vec<f64> = taper.speed_kph.iter().map(|s| s * KPH_TO_MPS).collect();
    MonotoneCubic::new(v, taper.power_frac.clone()).map_err(T0Error::from)
}

/// If the whole drivetrain has exactly one gearbox with a constant efficiency, return it.
fn single_gearbox_eff(spec: &outlap_schema::Vehicle) -> Option<f64> {
    let mut found: Option<f64> = None;
    for unit in &spec.drivetrain.units {
        for coupler in &unit.path {
            if let Coupler::Gearbox(g) = coupler {
                if found.is_some() {
                    return None; // ambiguous
                }
                match g.efficiency {
                    Efficiency::Constant(e) => found = Some(e),
                    Efficiency::Map { .. } => return None,
                }
            }
        }
    }
    found
}
