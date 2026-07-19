// SPDX-License-Identifier: AGPL-3.0-only
//! T1 topology powertrain: the drivetrain graph reduced to a traction/braking limit, the
//! differential torque split, and energy accounting for the quasi-steady-state trim (§8.0–8.2).
//!
//! The powertrain is a directed graph (§8.0): torque **sources** (`.ptm` maps: ICE, electric
//! machines, or lumped drive units) reach wheel **sinks** through ordered **couplers** (gearbox,
//! fixed ratio, differential). This module folds each unit's coupler path into a set of gears — the
//! shaft-speed/force convention is the T0 one (`ω = ratio/r · v`, `F = ratio·η/r · τ`) — reads the
//! peak-torque envelope from the `.ptm`, and models the differential that splits an axle's torque
//! left/right.
//!
//! Two limits enter the QSS solve:
//!
//! * The **traction ceiling** [`T1Powertrain::max_drive_force`] — the largest wheel force the
//!   sources can put down at a given speed through their best gear. PR7's g-g-g-v envelope caps the
//!   acceleration boundary with it; the tyre-grip limit is enforced by the trim itself.
//! * The **differential torque split** [`T1Powertrain::primary_diff`] — open (equal torque) vs
//!   locked/solid (equal speed) vs LSD (bounded torque bias) — which the trim's 9th unknown/residual
//!   consumes so the diff genuinely shapes the mid-corner per-wheel forces and yaw moment.
//!
//! The dense efficiency/loss tables are installed separately (parquet decode is a native-edge step,
//! like the aero map): [`T1Powertrain::install_maps`] consumes an already-decoded [`GriddedTable`].
//! They drive energy accounting only — the mechanical traction force uses the peak envelope and the
//! (constant or mapped) gearbox efficiency, never the machine/thermal efficiency map.
//!
//! Clean-room from published literature: Perantoni & Limebeer, VSD 52(5), 2014 (the reference F1
//! driveline); Guiggiani, *The Science of Vehicle Dynamics*, 2018 §3 (driveline torque balance);
//! Milliken & Milliken, *Race Car Vehicle Dynamics*, 1995 ch.20 (differential torque-bias models).

use outlap_core::{GriddedMapN, GriddedTable, MonotoneCubic, OutOfDomain};
use outlap_schema::io::SourceLoader;
use outlap_schema::load::load_ptm;
use outlap_schema::ptm::{PtmKind, TorqueCurve};
use outlap_schema::vehicle::{Coupler, DiffKind, Efficiency, Gearbox, Wheel};
use outlap_schema::ResolvedVehicle;

use crate::error::T1Error;
use crate::t1::vehicle::wheel_index;

/// Revolutions per minute → radians per second.
const RPM_TO_RAD_PER_S: f64 = std::f64::consts::PI / 30.0;
/// Canonical `.ptm` sidecar column: machine/thermal efficiency (0..1, drive and regen quadrants).
const COL_EFFICIENCY: &str = "efficiency";
/// Canonical `.ptm` sidecar column: total power loss at the operating point, W.
const COL_LOSS_W: &str = "loss_w";
/// Canonical `.ptm` sidecar axis: shaft speed, rpm.
const AXIS_SPEED: &str = "speed_rpm";
/// Canonical `.ptm` sidecar axis: shaft torque, N·m.
const AXIS_TORQUE: &str = "torque_nm";
/// Canonical `.ptm` sidecar axis: DC-link voltage, V (`ptm/1.1`, the Vdc–SoC coupling axis).
const AXIS_VDC: &str = "vdc_v";
/// Lower heating value of the reference fuel, J/kg (petrol ≈ 43 MJ/kg) — used only to turn an ICE
/// map's thermal efficiency into a fuel-mass rate for accounting. Config-overridable later.
const FUEL_LHV_J_PER_KG: f64 = 43.0e6;

/// The differential on a driven axle: kind plus the LSD preload/ramp data (§8.2).
///
/// Sign/units: `preload_nm` is the always-present locking torque (N·m); `ramp_accel`/`ramp_decel`
/// are the LSD lock **fractions** (0..1) applied to the axle drive/brake torque. Raw schema values
/// above 1 are read as percentages (÷100) — the schema comment calls them "angles/fractions", so the
/// QSS model fixes the interpretation as a percent lock-up and documents it (theory page).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiffModel {
    /// The differential kind.
    pub kind: DiffKind,
    /// Preload (always-locked) torque, N·m.
    pub preload_nm: f64,
    /// LSD lock fraction under acceleration, 0..1.
    pub ramp_accel: f64,
    /// LSD lock fraction under braking, 0..1.
    pub ramp_decel: f64,
}

impl DiffModel {
    /// The maximum torque **difference** the differential can sustain between its two output shafts,
    /// N·m, at an axle drive torque `t_axle_nm` (its absolute value is used).
    ///
    /// * open — `0` (both shafts always carry equal torque),
    /// * locked/solid — `+∞` (any difference; the housing reacts it),
    /// * LSD — `preload + ramp · |t_axle|` (drive uses `ramp_accel`, brake `ramp_decel`).
    #[must_use]
    pub fn max_torque_bias(&self, t_axle_nm: f64, braking: bool) -> f64 {
        match self.kind {
            DiffKind::Open => 0.0,
            DiffKind::Locked | DiffKind::Solid => f64::INFINITY,
            DiffKind::Lsd => {
                let ramp = if braking {
                    self.ramp_decel
                } else {
                    self.ramp_accel
                };
                self.preload_nm + ramp * t_axle_nm.abs()
            }
        }
    }

    /// Split a **drive** axle torque `t_axle_nm` (clamped ≥ 0) into `(left, right)` output-shaft
    /// torques given each side's *available* longitudinal grip torque `cap_left`/`cap_right` (N·m,
    /// the tyre `μ·F_z·r` at that wheel). This is the standalone reference used by the diff property
    /// test and the loaded-model report; the trim couples the split into its force balance directly.
    /// LSD uses the drive (accel) ramp; the braking split is a brake-balance concern, not the diff's.
    ///
    /// * open — equal torque `t/2` each (the lesser-grip side caps the deliverable axle torque),
    /// * locked/solid — grip-proportional (each side takes as much as it can, summing to `t`),
    /// * LSD — grip-proportional but with the side-to-side difference clamped to the drive
    ///   [`max_torque_bias`](Self::max_torque_bias).
    #[must_use]
    pub fn split(&self, t_axle_nm: f64, cap_left: f64, cap_right: f64) -> (f64, f64) {
        let t = t_axle_nm.max(0.0);
        match self.kind {
            DiffKind::Open => (0.5 * t, 0.5 * t),
            DiffKind::Locked | DiffKind::Solid => grip_proportional(t, cap_left, cap_right),
            DiffKind::Lsd => {
                let (l, r) = grip_proportional(t, cap_left, cap_right);
                let bias = self.max_torque_bias(t, false);
                let half = 0.5 * (t - (l - r).clamp(-bias, bias)); // enforce |l−r| ≤ bias, keep sum = t
                (t - half, half)
            }
        }
    }
}

/// Grip-proportional left/right split of an axle torque, summing to `t` and never exceeding either
/// side's grip cap where feasible (a locked diff can over-drive one wheel; that surplus is reported
/// by the trim's friction-circle containment, not clamped here).
fn grip_proportional(t: f64, cap_left: f64, cap_right: f64) -> (f64, f64) {
    let total_cap = cap_left + cap_right;
    if total_cap <= 0.0 {
        return (0.5 * t, 0.5 * t);
    }
    (t * cap_left / total_cap, t * cap_right / total_cap)
}

/// The primary driven axle's differential and geometry, consumed by the trim's diff residual.
#[derive(Clone, Copy, Debug)]
pub struct PrimaryDiff {
    /// The differential model.
    pub diff: DiffModel,
    /// Left driven-wheel index (`[FL, FR, RL, RR]` order).
    pub left: usize,
    /// Right driven-wheel index.
    pub right: usize,
    /// Driven-wheel rolling radius, m (force↔torque at the contact patch).
    pub r_wheel: f64,
}

/// A single selectable gear: total ratio (source shaft → wheel) and constant mechanical efficiency.
#[derive(Clone, Copy, Debug)]
struct Gear {
    /// Total ratio source→wheel (fixed ratios × gearbox ratio × final drive).
    ratio: f64,
    /// Constant mechanical efficiency folded along the path (gearbox / driveline).
    eff: f64,
}

/// One drive unit reduced for the T1 traction limit and energy accounting.
#[derive(Clone, Debug)]
struct PtUnit {
    /// Source kind (ICE / electric machine / lumped drive unit).
    kind: PtmKind,
    /// Peak torque vs shaft speed [rad/s → N·m] (source shaft).
    peak_env: MonotoneCubic<f64>,
    /// Peak **regenerative braking** torque vs shaft speed [rad/s → N·m, positive magnitude] on the
    /// source shaft (`ptm/1.2` `max_regen_torque_nm_vs_speed`, or a copy of `peak_env` when the map
    /// declares no regen envelope — the symmetric-machine assumption).
    regen_env: MonotoneCubic<f64>,
    /// Highest shaft speed the regen envelope covers, rad/s.
    regen_omega_max: f64,
    /// Highest shaft speed the envelope covers, rad/s.
    omega_max: f64,
    /// Driven-wheel rolling radius, m.
    r_wheel: f64,
    /// Selectable gears (one per gearbox ratio, or a single direct gear).
    gears: Vec<Gear>,
    /// Gearbox shift duration, s (`0` for a direct-drive / single-speed unit) — the transient shift
    /// FSM charges this per gear change.
    shift_time_s: f64,
    /// The differential on this unit's axle.
    diff: DiffModel,
    /// Which wheels this unit drives (and therefore can regen-brake) `[FL, FR, RL, RR]`.
    driven: [bool; 4],
    /// Left/right driven-wheel indices when the unit drives exactly one axle's pair.
    axle_pair: Option<(usize, usize)>,
    /// Efficiency map η(rpm, τ[, vdc]) — installed from the sidecar (energy accounting only).
    eff_map: Option<GriddedMapN<f64>>,
    /// Loss map loss_w(rpm, τ[, vdc]), W — installed from the sidecar (energy accounting only).
    loss_map: Option<GriddedMapN<f64>>,
    /// Reference DC-link voltage the map was measured at, V (`.ptm` `meta.dc_voltage_v`). Used to
    /// fill the Vdc query when a caller does not supply a coupled voltage.
    ref_vdc: f64,
    /// Whether the installed maps carry a Vdc axis (3-D) — the Vdc–SoC coupling is live.
    has_vdc: bool,
}

impl PtUnit {
    /// Wheel drive force in a specific `gear` at vehicle speed `v` (m/s), N — `0` above the gear's
    /// rev limit. The gears are the source-shaft ratios folded to the wheel.
    fn gear_force(&self, gear: usize, v: f64) -> f64 {
        let g = self.gears[gear];
        let omega = g.ratio / self.r_wheel * v;
        if omega <= self.omega_max {
            self.peak_env.eval(omega) * g.ratio * g.eff / self.r_wheel
        } else {
            0.0
        }
    }

    /// Ascending up-shift speeds (m/s), one per adjacent gear pair — the speeds at which the *next*
    /// gear's wheel-force curve overtakes the current one (or the current gear reaches its rev limit).
    /// Empty for a single-speed unit. These are exactly the crossover speeds baked into the best-gear
    /// [`Self::max_wheel_force`] envelope, so the shift FSM's torque interruption lands where the
    /// traction ceiling already switches gears — no change to the delivered force, only the shift cut.
    fn upshift_speeds(&self) -> Vec<f64> {
        let n = self.gears.len();
        if n < 2 {
            return Vec::new();
        }
        // Gears sorted by descending ratio (low gear first): the best gear rises with speed.
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| {
            self.gears[b]
                .ratio
                .partial_cmp(&self.gears[a].ratio)
                .unwrap()
        });
        // The unit's top speed: the highest gear (lowest ratio) at the rev limit.
        let min_ratio = self
            .gears
            .iter()
            .map(|g| g.ratio)
            .fold(f64::INFINITY, f64::min);
        let v_top = self.omega_max * self.r_wheel / min_ratio;
        let steps = 2000;
        let mut speeds = Vec::with_capacity(n - 1);
        for pair in 0..n - 1 {
            let (lo, hi) = (order[pair], order[pair + 1]);
            // Smallest speed where the higher gear's force meets or beats the lower gear's.
            let mut cross = v_top; // fall back to the top speed if they never cross
            for i in 1..=steps {
                let v = v_top * f64::from(i) / f64::from(steps);
                if self.gear_force(hi, v) >= self.gear_force(lo, v) {
                    cross = v;
                    break;
                }
            }
            speeds.push(cross);
        }
        // Enforce strict ascension (a degenerate ratio set could produce ties); nudge duplicates up.
        for i in 1..speeds.len() {
            if speeds[i] <= speeds[i - 1] {
                speeds[i] = speeds[i - 1] + 1e-3;
            }
        }
        speeds
    }

    /// Best-gear maximum wheel drive force at vehicle speed `v` (m/s), N. Zero-allocation.
    fn max_wheel_force(&self, v: f64) -> f64 {
        let mut best = 0.0;
        for g in &self.gears {
            let omega = g.ratio / self.r_wheel * v;
            if omega <= self.omega_max {
                let f = self.peak_env.eval(omega) * g.ratio * g.eff / self.r_wheel;
                if f > best {
                    best = f;
                }
            }
        }
        best
    }

    /// Best-gear maximum **regen braking** wheel force at vehicle speed `v` (m/s), N (positive
    /// magnitude). Zero-allocation.
    ///
    /// Driveline efficiency is deliberately *not* applied here. In the drive direction a loss shrinks
    /// the force reaching the wheel (`τ·ratio·η/r`); under regen the power flows the other way, so a
    /// loss would *add* braking at the wheel while shrinking what the machine recovers. Taking
    /// `F = τ_regen·ratio/r` and charging the efficiency once, against the recovered *power*, keeps
    /// the ledger honest and understates rather than overstates the machine's braking authority.
    fn max_regen_wheel_force(&self, v: f64) -> f64 {
        let mut best = 0.0;
        for g in &self.gears {
            let omega = g.ratio / self.r_wheel * v;
            if omega <= self.regen_omega_max {
                let f = self.regen_env.eval(omega) * g.ratio / self.r_wheel;
                if f > best {
                    best = f;
                }
            }
        }
        best
    }

    /// The machine's representative drive-quadrant efficiency at vehicle speed `v`, `0..1` — the
    /// installed `.ptm` efficiency map sampled at the best-gear peak-torque operating point. `None`
    /// for an ICE (fuel, not charge), a unit with no efficiency map, or a speed with no on-envelope
    /// gear. Zero-allocation.
    fn machine_efficiency(&self, v: f64) -> Option<f64> {
        if self.kind == PtmKind::Ice {
            return None;
        }
        let eff = self.eff_map.as_ref()?;
        let (rpm, torque) = self.source_op(v, self.max_wheel_force(v))?;
        Some(self.eval_map(eff, rpm, torque, None).clamp(1e-3, 1.0))
    }

    /// Evaluate an installed value map at `(rpm, torque)` and, for a 3-D Vdc-stacked map, `vdc`
    /// (falling back to the reference Vdc when the caller passes `None`). Zero-allocation.
    fn eval_map(&self, map: &GriddedMapN<f64>, rpm: f64, torque: f64, vdc: Option<f64>) -> f64 {
        if map.ndim() == 3 {
            map.eval(&[rpm, torque, vdc.unwrap_or(self.ref_vdc)])
        } else {
            map.eval(&[rpm, torque])
        }
    }

    /// Source-shaft operating point `(rpm, torque)` for a commanded wheel force at speed `v`, using
    /// the gear that maximises available force (the gear the traction ceiling assumes). Returns
    /// `None` if no gear is on-envelope at this speed.
    fn source_op(&self, v: f64, wheel_force: f64) -> Option<(f64, f64)> {
        let mut best: Option<(f64, f64, f64)> = None; // (available force, rpm, torque)
        for g in &self.gears {
            let omega = g.ratio / self.r_wheel * v;
            if omega > self.omega_max {
                continue;
            }
            let avail = self.peak_env.eval(omega) * g.ratio * g.eff / self.r_wheel;
            if best.is_none_or(|(b, ..)| avail > b) {
                // Source torque to make `wheel_force`: τ = F·r / (ratio·η).
                let tau = wheel_force * self.r_wheel / (g.ratio * g.eff);
                best = Some((avail, omega / RPM_TO_RAD_PER_S, tau));
            }
        }
        best.map(|(_, rpm, tau)| (rpm, tau))
    }
}

/// The drivetrain graph reduced for the T1 quasi-steady-state trim.
#[derive(Clone, Debug)]
pub struct T1Powertrain {
    units: Vec<PtUnit>,
    /// Static front-axle torque share (from `control.split.front`), if declared.
    split_front: Option<f64>,
    /// Static left-side torque share (from `control.split.left`), if declared.
    split_left: Option<f64>,
    /// Assembly notes / simplifications (nothing silent).
    notes: Vec<String>,
}

impl T1Powertrain {
    /// Assemble the powertrain graph from the resolved vehicle. Reads each drive unit's `.ptm`
    /// (peak envelope, kind) and folds its coupler path into gears and a differential. The
    /// efficiency/loss maps are **not** read here — install them with [`Self::install_maps`].
    ///
    /// `r_front`/`r_rear` are the driven-axle rolling radii (from the tyre `UNLOADED_RADIUS`).
    ///
    /// # Errors
    /// [`T1Error`] if a referenced `.ptm` fails to load/validate or a peak envelope cannot be fitted.
    pub fn assemble(
        vehicle: &ResolvedVehicle,
        loader: &dyn SourceLoader,
        r_front: f64,
        r_rear: f64,
    ) -> Result<Self, T1Error> {
        let spec = &vehicle.spec;
        let mut units = Vec::with_capacity(spec.drivetrain.units.len());
        let mut notes = Vec::new();
        for (i, unit) in spec.drivetrain.units.iter().enumerate() {
            let ptm = load_ptm(unit.source.as_str(), loader)?;
            let mut driven = [false; 4];
            for w in &unit.wheels {
                driven[wheel_index(*w)] = true;
            }
            let r_wheel = driven_radius(driven, r_front, r_rear);
            let (base_ratio, base_eff, gearbox, has_eff_map) = fold_path(&unit.path);
            if has_eff_map {
                notes.push(format!(
                    "drive unit {i} gearbox uses a map efficiency; a constant proxy carries the \
                     traction force until the efficiency map is installed"
                ));
            }
            let gears = build_gears(base_ratio, base_eff, gearbox, r_wheel);
            let shift_time_s = gearbox.map_or(0.0, |g| g.shift_time_s);
            let peak_env = torque_env(&ptm.limits.max_torque_nm_vs_speed)?;
            let omega_max = peak_env.domain().1;
            // The regen (negative-quadrant) envelope.
            //
            // An internal-combustion engine cannot regenerate: it has no negative quadrant to
            // command. Its overrun braking is parasitic drag (`drag_torque_nm_vs_speed`), not
            // recoverable energy, so its regen envelope is identically zero.
            //
            // An electric machine that declares no envelope is assumed **symmetric** with its drive
            // envelope — the usual first-order truth when inverter current sets the limit. That is an
            // estimate, so it is surfaced, never silent (#41).
            let regen_env = match (&ptm.kind, &ptm.limits.max_regen_torque_nm_vs_speed) {
                (PtmKind::Ice, _) => {
                    notes.push(format!(
                        "drive unit {i} is an internal-combustion engine: it recovers no braking \
                         energy (regen envelope zero; overrun drag is not commandable regen)"
                    ));
                    zero_env(&peak_env)?
                }
                (_, Some(curve)) => torque_env(curve)?,
                (_, None) => {
                    notes.push(format!(
                        "drive unit {i} declares no `max_regen_torque_nm_vs_speed`; its regen \
                         braking envelope is assumed symmetric with the drive envelope (estimated)"
                    ));
                    peak_env.clone()
                }
            };
            let regen_omega_max = regen_env.domain().1;
            let diff = diff_model(&unit.path);
            units.push(PtUnit {
                kind: ptm.kind,
                peak_env,
                regen_env,
                regen_omega_max,
                omega_max,
                r_wheel,
                gears,
                shift_time_s,
                diff,
                driven,
                axle_pair: axle_pair(driven),
                eff_map: None,
                loss_map: None,
                ref_vdc: ptm.meta.dc_voltage_v.unwrap_or(0.0),
                has_vdc: false,
            });
        }
        let split_front = spec.drivetrain.control.split.front;
        let split_left = spec.drivetrain.control.split.left;
        notes.push(
            "T1 powertrain: traction ceiling = best-gear peak envelope × ratio × mechanical \
             efficiency / wheel radius; the differential torque split enters the trim; efficiency/\
             loss maps (when installed) drive energy accounting only."
                .to_owned(),
        );
        if spec.ers.is_some() {
            notes.push(
                "ERS/MGU-K deployment is NOT folded into the T1 traction ceiling (it is a separate \
                 rule-based deployment mechanism, §8.3); the ceiling is the drivetrain units only."
                    .to_owned(),
            );
        }
        Ok(Self {
            units,
            split_front,
            split_left,
            notes,
        })
    }

    /// Install the decoded efficiency/loss tables for drive unit `unit_idx` from a `.ptm` sidecar.
    ///
    /// The axes are `(speed_rpm, torque_nm)` for a single-voltage map, or `(speed_rpm, torque_nm,
    /// vdc_v)` for a Vdc-stacked `ptm/1.1` map (decode it with [`Self::map_axis_names_vdc`]). The Vdc
    /// axis uses **linear** out-of-domain extrapolation from the boundary slice (Decision #30 /
    /// `OutOfDomain::Linear`) so a low-SoC terminal voltage below the map grid stays usable; the
    /// speed/torque axes clamp. The parquet decode is a native-edge step; this crate stays wasm-clean
    /// by consuming the decoded table.
    ///
    /// # Errors
    /// [`T1Error::PowertrainMap`] if a required column is missing or the grid is not rectilinear;
    /// [`T1Error::UnknownDriveUnit`] if `unit_idx` is out of range.
    pub fn install_maps(
        &mut self,
        unit_idx: usize,
        table: &GriddedTable<f64>,
    ) -> Result<(), T1Error> {
        let unit = self
            .units
            .get_mut(unit_idx)
            .ok_or(T1Error::UnknownDriveUnit { unit: unit_idx })?;
        // A Vdc axis (ptm/1.1) makes the maps 3-D and enables the coupling; extrapolate on Vdc only.
        // The Linear mode and `eval_map` both attach positionally to axis index 2, so the decode must
        // put Vdc last (it does — decode with `map_axis_names_vdc`); assert that contract.
        let has_vdc = table.axis_names().iter().any(|n| n == AXIS_VDC);
        debug_assert!(
            !has_vdc || table.axis_names().last().map(String::as_str) == Some(AXIS_VDC),
            "a Vdc-stacked table must be decoded with the Vdc axis last (map_axis_names_vdc)"
        );
        let modes = || {
            if has_vdc {
                vec![OutOfDomain::Clamp, OutOfDomain::Clamp, OutOfDomain::Linear]
            } else {
                vec![OutOfDomain::Clamp, OutOfDomain::Clamp]
            }
        };
        unit.eff_map = Some(
            table
                .map(COL_EFFICIENCY, modes())
                .map_err(T1Error::PowertrainMap)?,
        );
        if table.value_names().any(|n| n == COL_LOSS_W) {
            unit.loss_map = Some(
                table
                    .map(COL_LOSS_W, modes())
                    .map_err(T1Error::PowertrainMap)?,
            );
        }
        unit.has_vdc = has_vdc;
        // If the map is Vdc-stacked but the `.ptm` recorded no reference voltage, fall back to the
        // middle of the Vdc grid rather than 0 V (which would extrapolate far below the grid) for the
        // non-coupled `efficiency`/`energy_at_shaft` queries. The coupled path always passes a Vdc.
        if has_vdc && unit.ref_vdc <= 0.0 {
            if let Some(eff) = unit.eff_map.as_ref() {
                let (lo, hi) = eff.domain()[2];
                unit.ref_vdc = 0.5 * (lo + hi);
            }
        }
        self.notes.push(format!(
            "drive unit {unit_idx}: efficiency/loss map installed — energy accounting is live{}",
            if has_vdc {
                " (Vdc-coupled; linear extrapolation below/above the voltage grid)"
            } else {
                ""
            }
        ));
        Ok(())
    }

    /// The canonical `.ptm` sidecar axis names for a single-voltage map, in tensor order.
    #[must_use]
    pub fn map_axis_names() -> [&'static str; 2] {
        [AXIS_SPEED, AXIS_TORQUE]
    }

    /// The canonical `.ptm` sidecar axis names for a Vdc-stacked (`ptm/1.1`) map, in tensor order.
    #[must_use]
    pub fn map_axis_names_vdc() -> [&'static str; 3] {
        [AXIS_SPEED, AXIS_TORQUE, AXIS_VDC]
    }

    /// Whether drive unit `unit_idx` has a Vdc-stacked map installed (the coupling is live).
    #[must_use]
    pub fn has_vdc_axis(&self, unit_idx: usize) -> bool {
        self.units.get(unit_idx).is_some_and(|u| u.has_vdc)
    }

    /// Whether ANY drive unit has an installed efficiency map — the assembly-time fact that the
    /// mapped-EV slow-state march can actually move a state ([`Self::traction_energy`] returns
    /// `Some`). The QSS coupling's activity flag reads this instead of diffing outputs.
    #[must_use]
    pub fn has_energy_maps(&self) -> bool {
        self.units.iter().any(|u| u.eff_map.is_some())
    }

    /// The maximum wheel **drive** force available at vehicle speed `v` (m/s), N — summed over drive
    /// units at each unit's best gear. This is the powertrain traction ceiling (PR7 caps the g-g-g-v
    /// acceleration boundary with it). Allocation-free.
    #[must_use]
    pub fn max_drive_force(&self, v: f64) -> f64 {
        self.units.iter().map(|u| u.max_wheel_force(v)).sum()
    }

    /// Ascending up-shift speeds (m/s) for the **primary geared unit** — the drive unit with the most
    /// selectable gears (the gearbox the transient shift FSM models). Empty when every unit is
    /// single-speed (a direct-drive EV never shifts). See [`PtUnit::upshift_speeds`].
    #[must_use]
    pub fn upshift_speeds(&self) -> Vec<f64> {
        self.units
            .iter()
            .max_by_key(|u| u.gears.len())
            .map(PtUnit::upshift_speeds)
            .unwrap_or_default()
    }

    /// The number of selectable gears on the primary geared unit (`1` for direct drive).
    #[must_use]
    pub fn primary_gear_count(&self) -> usize {
        self.units
            .iter()
            .map(|u| u.gears.len())
            .max()
            .unwrap_or(1)
            .max(1)
    }

    /// The gearbox shift duration of the primary geared unit, s (`0` for a direct-drive car).
    #[must_use]
    pub fn primary_shift_time_s(&self) -> f64 {
        self.units
            .iter()
            .max_by_key(|u| u.gears.len())
            .map_or(0.0, |u| u.shift_time_s)
    }

    /// The peak **regen braking** wheel force each axle's machines can produce at vehicle speed `v`,
    /// N (positive magnitude), as `[front, rear]`.
    ///
    /// Each machine can only ever brake the wheels it drives, so a unit's capability is attributed to
    /// the axle(s) in its driven set — split by driven-wheel count when one unit spans both axles
    /// (a single motor through a centre differential contributes half to each). An axle with no
    /// machine returns `0` and is braked by friction alone.
    #[must_use]
    pub fn max_regen_force_by_axle(&self, v: f64) -> [f64; 2] {
        let mut axle = [0.0, 0.0];
        for u in &self.units {
            let force = u.max_regen_wheel_force(v);
            let front = u64::from(u.driven[0]) + u64::from(u.driven[1]);
            let rear = u64::from(u.driven[2]) + u64::from(u.driven[3]);
            let total = front + rear;
            if total == 0 {
                continue;
            }
            #[allow(clippy::cast_precision_loss)] // ≤ 4 driven wheels
            let (fw, rw, tw) = (front as f64, rear as f64, total as f64);
            axle[0] += force * fw / tw;
            axle[1] += force * rw / tw;
        }
        axle
    }

    /// The representative machine efficiency `[front, rear]` at vehicle speed `v` — the mean over the
    /// machines driving each axle, sampled from their `.ptm` efficiency maps at the peak-torque
    /// operating point (§8.4). `None` for an axle with no efficiency-mapped machine, so the caller
    /// keeps its documented constant fallback (the T2 series-regen blend and electric traction draw
    /// then run a speed-varying efficiency instead of the flat proxy). Zero-allocation.
    #[must_use]
    pub fn machine_efficiency_by_axle(&self, v: f64) -> [Option<f64>; 2] {
        let mut sum = [0.0_f64, 0.0];
        let mut cnt = [0_u32, 0];
        for u in &self.units {
            let Some(eta) = u.machine_efficiency(v) else {
                continue;
            };
            if u.driven[0] || u.driven[1] {
                sum[0] += eta;
                cnt[0] += 1;
            }
            if u.driven[2] || u.driven[3] {
                sum[1] += eta;
                cnt[1] += 1;
            }
        }
        [
            (cnt[0] > 0).then(|| sum[0] / f64::from(cnt[0])),
            (cnt[1] > 0).then(|| sum[1] / f64::from(cnt[1])),
        ]
    }

    /// The primary driven axle's differential + geometry for the trim's diff residual: the first
    /// unit that drives exactly one axle's left/right pair. `None` when no unit drives a clean pair
    /// (single-wheel drive, or a hand-built vehicle with no drivetrain) — the trim then falls back to
    /// the equal-speed (locked) baseline.
    #[must_use]
    pub fn primary_diff(&self) -> Option<PrimaryDiff> {
        self.units.iter().find_map(|u| {
            u.axle_pair.map(|(left, right)| PrimaryDiff {
                diff: u.diff,
                left,
                right,
                r_wheel: u.r_wheel,
            })
        })
    }

    /// Front/rear axle torque split fractions `(front, rear)` — the static `control.split.front`
    /// (default: all torque to whichever axle the units drive). Always sums to 1.
    #[must_use]
    pub fn axle_split(&self) -> (f64, f64) {
        match self.split_front {
            Some(f) => {
                let f = f.clamp(0.0, 1.0);
                (f, 1.0 - f)
            }
            None => (0.0, 1.0),
        }
    }

    /// Left/right side torque split fractions `(left, right)` — the static `control.split.left`
    /// (default: even). Always sums to 1.
    #[must_use]
    pub fn side_split(&self) -> (f64, f64) {
        match self.split_left {
            Some(l) => {
                let l = l.clamp(0.0, 1.0);
                (l, 1.0 - l)
            }
            None => (0.5, 0.5),
        }
    }

    /// Energy accounting at a source **shaft** operating point `(rpm, torque_nm)`: the source
    /// (electrical/fuel) power drawn, the shaft mechanical power, the loss, and — for an ICE — the
    /// fuel-mass rate. Requires an installed efficiency map. Zero-allocation.
    ///
    /// With a loss map present the loss is taken as measured and energy closes **exactly** —
    /// `source = mech + loss` — in every quadrant, including the `τ = 0` spin point where the
    /// efficiency sentinel `η = 0` is not a real efficiency but the idle draw *is* real. Without a
    /// loss map the loss is derived from the efficiency (drive: `mech/η − mech`; regen:
    /// `mech − mech·η`), closing to interpolation accuracy between the importer's grid nodes.
    #[must_use]
    pub fn energy_at_shaft(
        &self,
        unit_idx: usize,
        rpm: f64,
        torque_nm: f64,
    ) -> Option<EnergyPoint> {
        self.energy_at_shaft_inner(unit_idx, rpm, torque_nm, None)
    }

    /// Vdc-coupled energy accounting at a source-shaft operating point: evaluate the drive unit's
    /// Vdc-stacked efficiency/loss map at the pack's terminal voltage `vdc` (the Vdc–SoC coupling,
    /// §8.4). The loss it returns is the machine-heating loss PR5's thermal model consumes — looked
    /// up at the coupled voltage, so a low-SoC (low-voltage) point shifts both traction and heating.
    /// For a single-voltage map `vdc` is ignored and this equals [`Self::energy_at_shaft`].
    #[must_use]
    pub fn energy_at_shaft_vdc(
        &self,
        unit_idx: usize,
        rpm: f64,
        torque_nm: f64,
        vdc: f64,
    ) -> Option<EnergyPoint> {
        self.energy_at_shaft_inner(unit_idx, rpm, torque_nm, Some(vdc))
    }

    fn energy_at_shaft_inner(
        &self,
        unit_idx: usize,
        rpm: f64,
        torque_nm: f64,
        vdc: Option<f64>,
    ) -> Option<EnergyPoint> {
        let unit = self.units.get(unit_idx)?;
        let eff = unit.eff_map.as_ref()?;
        let eta = unit.eval_map(eff, rpm, torque_nm, vdc).clamp(1e-3, 1.0);
        let p_mech = torque_nm * (rpm * RPM_TO_RAD_PER_S); // source-shaft mechanical power, W
                                                           // With a measured loss map, close energy by construction (source = mech + loss), idle
                                                           // included. Without one, derive the source from the efficiency and the loss from the balance.
        let (source_w, loss_w) = if let Some(m) = unit.loss_map.as_ref() {
            let loss = unit.eval_map(m, rpm, torque_nm, vdc);
            (p_mech + loss, loss)
        } else {
            let source = if p_mech >= 0.0 {
                p_mech / eta
            } else {
                p_mech * eta
            };
            (source, (source - p_mech).abs())
        };
        Some(EnergyPoint {
            source_w,
            mech_w: p_mech,
            loss_w,
            // An ICE burns fuel whenever it draws chemical power (drive or idle); motoring does not.
            fuel_kg_per_s: if unit.kind == PtmKind::Ice && source_w > 0.0 {
                source_w / FUEL_LHV_J_PER_KG
            } else {
                0.0
            },
            efficiency: eta,
        })
    }

    /// Energy accounting for delivering `wheel_force` (N) at speed `v` (m/s) through drive unit
    /// `unit_idx`: resolves the best-gear source operating point and evaluates
    /// [`Self::energy_at_shaft`]. Returns `None` if the unit has no installed efficiency map or no
    /// gear is on-envelope at this speed.
    #[must_use]
    pub fn source_and_loss_power(
        &self,
        unit_idx: usize,
        v: f64,
        wheel_force: f64,
    ) -> Option<EnergyPoint> {
        let (rpm, tau) = self.units.get(unit_idx)?.source_op(v, wheel_force)?;
        self.energy_at_shaft(unit_idx, rpm, tau)
    }

    /// Aggregate source power drawn, machine-heating loss, and a representative machine shaft speed
    /// for delivering a **positive** wheel drive force `wheel_force_n` (N) at speed `v` (m/s),
    /// evaluating the Vdc-stacked maps at the pack terminal voltage `vdc` (`None` ⇒ each unit's
    /// reference Vdc). The requested force is distributed across drive units in proportion to their
    /// best-gear capacity at this speed; each unit's `(rpm, τ)` operating point is looked up through
    /// its Vdc-coupled efficiency/loss map. The representative `omega_rad_s` is the fastest
    /// contributing unit's shaft speed (the air-gap-cooling driver for the thermal step).
    ///
    /// This is the per-segment lookup the QSS slow-state coupling uses: `loss_w` feeds the machine
    /// thermal network, `source_w` feeds the battery Coulomb count / peak-power cap. Returns `None`
    /// when no unit has an installed efficiency map (⇒ the coupling is inert — a fuel/constant-Vdc
    /// drive). Zero-allocation.
    ///
    /// QSS simplification: with a hybrid ICE + electric drive this attributes the full traction draw
    /// to the mapped units, so a distinct ICE source is treated as battery-fed; the coupling is exact
    /// for a pure-electric drive (the case the CI fixture exercises) and conservative otherwise —
    /// documented in `docs/theory/qss-powertrain.md`.
    #[must_use]
    pub fn traction_energy(
        &self,
        v: f64,
        wheel_force_n: f64,
        vdc: Option<f64>,
    ) -> Option<TractionEnergy> {
        // Total mapped capacity at this speed sets each unit's force share.
        let mut total_cap = 0.0;
        let mut any_map = false;
        for u in &self.units {
            if u.eff_map.is_some() {
                any_map = true;
                total_cap += u.max_wheel_force(v);
            }
        }
        if !any_map || total_cap <= 0.0 {
            return None;
        }
        let mut source_w = 0.0f64;
        let mut loss_w = 0.0f64;
        let mut omega_rad_s = 0.0f64;
        for (idx, u) in self.units.iter().enumerate() {
            if u.eff_map.is_none() {
                continue;
            }
            let share = u.max_wheel_force(v) / total_cap;
            let unit_force = wheel_force_n * share;
            let Some((rpm, tau)) = u.source_op(v, unit_force) else {
                continue;
            };
            if let Some(pt) = self.energy_at_shaft_inner(idx, rpm, tau, vdc) {
                source_w += pt.source_w;
                loss_w += pt.loss_w;
                omega_rad_s = omega_rad_s.max(rpm * RPM_TO_RAD_PER_S);
            }
        }
        Some(TractionEnergy {
            source_w,
            loss_w,
            omega_rad_s,
        })
    }

    /// The ICE fuel-burn rate, kg/s, delivering a **positive** ICE wheel force `ice_wheel_force_n`
    /// (N) at speed `v` (m/s) — the fuel half of the §8.1 slow state (D-M6-4). Sums each ICE unit's
    /// `EnergyPoint::fuel_kg_per_s` (chemical power `P_mech/η_thermal` ÷ LHV) at its share of the ICE
    /// force; non-ICE units contribute nothing. Zero on a braking/coast segment (force ≤ 0) and for
    /// a car with no ICE unit (a pure EV). This attributes the ICE SHARE of hybrid traction (the
    /// electric deploy is subtracted upstream), the correct fuel accounting the QSS core wires (the
    /// note at [`Self::traction_energy`]). Zero-allocation.
    #[must_use]
    pub fn ice_fuel_rate_kg_per_s(&self, v: f64, ice_wheel_force_n: f64) -> f64 {
        if ice_wheel_force_n <= 0.0 {
            return 0.0;
        }
        // ICE force share ∝ each ICE unit's best-gear capacity at this speed (usually one unit).
        let mut total_cap = 0.0;
        for u in &self.units {
            if u.kind == PtmKind::Ice && u.eff_map.is_some() {
                total_cap += u.max_wheel_force(v);
            }
        }
        if total_cap <= 0.0 {
            return 0.0;
        }
        let mut kg_per_s = 0.0;
        for (idx, u) in self.units.iter().enumerate() {
            if u.kind != PtmKind::Ice || u.eff_map.is_none() {
                continue;
            }
            let share = u.max_wheel_force(v) / total_cap;
            if let Some(pt) = self.source_and_loss_power(idx, v, ice_wheel_force_n * share) {
                kg_per_s += pt.fuel_kg_per_s;
            }
        }
        kg_per_s
    }

    /// A representative ICE brake-thermal efficiency (0..1) for the T2 tier's scalar fuel burn
    /// (§8.1, D-M6-4): sampled from the first ICE unit's efficiency map at a mid-high-load operating
    /// point (clamped into the grid). `None` when the car has no mapped ICE unit — the caller falls
    /// back to a literature default. The QSS tier uses the full per-operating-point map; this scalar
    /// is the documented tier simplification (parity gate recorded, Decision #48).
    #[must_use]
    pub fn representative_ice_efficiency(&self) -> Option<f64> {
        for (idx, u) in self.units.iter().enumerate() {
            if u.kind == PtmKind::Ice && u.eff_map.is_some() {
                return self.efficiency(idx, 10_000.0, 200.0);
            }
        }
        None
    }

    /// The best-gear crank speed (rpm) of the fastest ICE unit at speed `v` (m/s), for the FIA
    /// C5.2.5 low-rpm fuel-flow line. `None` when the car has no mapped ICE unit. Zero-allocation.
    #[must_use]
    pub fn ice_crank_rpm(&self, v: f64, wheel_force_n: f64) -> Option<f64> {
        let mut rpm_out = None::<f64>;
        for u in &self.units {
            if u.kind != PtmKind::Ice {
                continue;
            }
            if let Some((rpm, _)) = u.source_op(v, wheel_force_n.max(0.0)) {
                rpm_out = Some(rpm_out.map_or(rpm, |r: f64| r.max(rpm)));
            }
        }
        rpm_out
    }

    /// The installed machine/thermal efficiency at a source-shaft operating point `(rpm, torque_nm)`
    /// — the raw interpolated map value (0..1), unclamped, for round-trip verification against the
    /// importer's source arrays. `None` if `unit_idx` has no installed efficiency map.
    #[must_use]
    pub fn efficiency(&self, unit_idx: usize, rpm: f64, torque_nm: f64) -> Option<f64> {
        let unit = self.units.get(unit_idx)?;
        let eff = unit.eff_map.as_ref()?;
        Some(unit.eval_map(eff, rpm, torque_nm, None))
    }

    /// The installed machine/thermal efficiency at `(rpm, torque_nm)` evaluated at DC-link voltage
    /// `vdc` (the Vdc-coupled lookup; the Vdc axis extrapolates linearly out of grid). For a
    /// single-voltage map `vdc` is ignored. `None` if `unit_idx` has no installed efficiency map.
    #[must_use]
    pub fn efficiency_vdc(
        &self,
        unit_idx: usize,
        rpm: f64,
        torque_nm: f64,
        vdc: f64,
    ) -> Option<f64> {
        let unit = self.units.get(unit_idx)?;
        let eff = unit.eff_map.as_ref()?;
        Some(unit.eval_map(eff, rpm, torque_nm, Some(vdc)))
    }

    /// Wheel torque delivered through a unit's gear for a source-shaft torque `tau_source` (N·m):
    /// `Σ τ_wheel = τ_source · ratio · η`. The coupler-conservation identity the property test uses.
    #[must_use]
    pub fn wheel_torque(&self, unit_idx: usize, gear: usize, tau_source: f64) -> Option<f64> {
        let g = self.units.get(unit_idx)?.gears.get(gear)?;
        Some(tau_source * g.ratio * g.eff)
    }

    /// Assembly notes / simplifications (nothing silent).
    #[must_use]
    pub fn notes(&self) -> &[String] {
        &self.notes
    }
}

/// Aggregate traction energy for the per-segment slow-state coupling: the electrical/chemical source
/// power drawn, the machine-heating loss routed to the thermal network, and the fastest contributing
/// machine's shaft speed. Produced by [`T1Powertrain::traction_energy`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TractionEnergy {
    /// Source (electrical / fuel-chemical) power drawn to deliver the requested wheel force, W.
    pub source_w: f64,
    /// Machine-heating loss at the operating point (for the `.emotor` thermal network), W.
    pub loss_w: f64,
    /// Representative machine shaft speed (fastest contributing unit), rad/s.
    pub omega_rad_s: f64,
}

/// An energy-accounting sample at one source operating point (all powers in W).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnergyPoint {
    /// Source (electrical / fuel-chemical) power drawn, W.
    pub source_w: f64,
    /// Source-shaft mechanical power, W.
    pub mech_w: f64,
    /// Power loss (from the loss map, or `|source − mech|`), W.
    pub loss_w: f64,
    /// Fuel-mass rate for an ICE source (0 for electric), kg/s.
    pub fuel_kg_per_s: f64,
    /// Machine/thermal efficiency at the operating point, 0..1.
    pub efficiency: f64,
}

/// The driven-axle rolling radius from the per-wheel driven mask (mean if it spans both axles).
fn driven_radius(driven: [bool; 4], r_front: f64, r_rear: f64) -> f64 {
    let front = driven[0] || driven[1];
    let rear = driven[2] || driven[3];
    match (front, rear) {
        (true, false) => r_front,
        (true, true) => 0.5 * (r_front + r_rear),
        _ => r_rear, // rear-only, or nothing declared → rear radius
    }
}

/// The left/right driven-wheel index pair when a unit drives exactly one axle's two wheels.
fn axle_pair(driven: [bool; 4]) -> Option<(usize, usize)> {
    let front = driven[0] && driven[1] && !driven[2] && !driven[3];
    let rear = driven[2] && driven[3] && !driven[0] && !driven[1];
    if front {
        Some((wheel_index(Wheel::Fl), wheel_index(Wheel::Fr)))
    } else if rear {
        Some((wheel_index(Wheel::Rl), wheel_index(Wheel::Rr)))
    } else {
        None
    }
}

/// The differential on a coupler path (first `Diff` coupler), or a locked default (a rigid two-wheel
/// drive with no diff is a solid axle — the locked limit).
fn diff_model(path: &[Coupler]) -> DiffModel {
    for c in path {
        if let Coupler::Diff(d) = c {
            let (ramp_accel, ramp_decel) = d
                .ramp
                .map_or((0.0, 0.0), |[a, b]| (lock_fraction(a), lock_fraction(b)));
            return DiffModel {
                kind: d.kind,
                preload_nm: d.preload_nm.unwrap_or(0.0),
                ramp_accel,
                ramp_decel,
            };
        }
    }
    DiffModel {
        kind: DiffKind::Locked,
        preload_nm: 0.0,
        ramp_accel: 0.0,
        ramp_decel: 0.0,
    }
}

/// Interpret a schema LSD ramp value as a lock fraction (0..1). Values above 1 are read as a
/// percentage (÷100); everything is clamped to `[0, 1]`. Documented in the theory page.
fn lock_fraction(raw: f64) -> f64 {
    let f = if raw > 1.0 { raw / 100.0 } else { raw };
    f.clamp(0.0, 1.0)
}

/// Fold a coupler path into `(base_ratio, base_efficiency, gearbox, has_efficiency_map)`. Fixed
/// ratios multiply into `base_ratio`; a gearbox supplies the selectable ratios; diffs are 1:1 at the
/// power level. A gearbox map efficiency contributes a conservative constant proxy (0.95) until the
/// map is installed, and flags `has_efficiency_map` so the caller records it.
fn fold_path(path: &[Coupler]) -> (f64, f64, Option<&Gearbox>, bool) {
    let mut base_ratio = 1.0;
    let mut base_eff = 1.0;
    let mut gearbox: Option<&Gearbox> = None;
    let mut has_map = false;
    for coupler in path {
        match coupler {
            Coupler::FixedRatio(r) => base_ratio *= r,
            Coupler::Diff(_) => {}
            Coupler::Gearbox(g) => {
                match &g.efficiency {
                    Efficiency::Constant(e) => base_eff *= e,
                    Efficiency::Map { .. } => {
                        base_eff *= 0.95;
                        has_map = true;
                    }
                }
                if gearbox.is_none() {
                    gearbox = Some(g);
                } else {
                    base_ratio *= g.final_drive * g.ratios.first().copied().unwrap_or(1.0);
                }
            }
        }
    }
    (base_ratio, base_eff, gearbox, has_map)
}

/// Expand a folded path into gears (one per gearbox ratio, or one direct gear).
fn build_gears(
    base_ratio: f64,
    base_eff: f64,
    gearbox: Option<&Gearbox>,
    r_wheel: f64,
) -> Vec<Gear> {
    let _ = r_wheel; // radius applied at force-query time; kept for signature symmetry with T0
    match gearbox {
        Some(g) => g
            .ratios
            .iter()
            .map(|&rk| Gear {
                ratio: base_ratio * rk * g.final_drive,
                eff: base_eff,
            })
            .collect(),
        None => vec![Gear {
            ratio: base_ratio,
            eff: base_eff,
        }],
    }
}

/// Fit a peak-torque envelope `τ(ω)` from a speed/torque curve (rpm → rad/s at the boundary).
fn torque_env(curve: &TorqueCurve) -> Result<MonotoneCubic<f64>, T1Error> {
    let omega: Vec<f64> = curve
        .speed_rpm
        .iter()
        .map(|r| r * RPM_TO_RAD_PER_S)
        .collect();
    MonotoneCubic::new(omega, curve.torque_nm.clone()).map_err(T1Error::Envelope)
}

/// An identically-zero torque envelope spanning the same shaft-speed domain as `like` — the regen
/// capability of a unit that has no negative quadrant to command (an ICE).
fn zero_env(like: &MonotoneCubic<f64>) -> Result<MonotoneCubic<f64>, T1Error> {
    let (lo, hi) = like.domain();
    MonotoneCubic::new(vec![lo, hi], vec![0.0, 0.0]).map_err(T1Error::Envelope)
}
