// SPDX-License-Identifier: AGPL-3.0-only
//! T1 assembly: reduce a [`ResolvedVehicle`] + `conditions` into a [`T1Vehicle`] for the trim solver.
//!
//! Unlike the T0 point-mass reduction, T1 keeps the per-axle tyre force models, the chassis mass /
//! CG / yaw inertia, the suspension roll geometry, and the aero coefficients — everything the
//! quasi-steady-state double-track trim ([`crate::t1::trim`]) needs, including the ride-height/yaw
//! aero map (installed via [`T1Vehicle::install_aero_map`]). Powertrain traction is still a
//! simplified longitudinal-slip control here (no drivetrain graph); PR4 refines the traction limit.

use outlap_core::GriddedTable;
use outlap_schema::conditions::Conditions;
use outlap_schema::io::SourceLoader;
use outlap_schema::load::load_tyr;
use outlap_schema::tyr::Tyr;
use outlap_schema::vehicle::Wheel;
use outlap_schema::ResolvedVehicle;
use outlap_tire::TireModel;

use crate::error::T1Error;
use crate::t1::aero::{AeroLumped, AeroMap, AeroPlatform};
use crate::t1::powertrain::{PrimaryDiff, T1Powertrain};

/// Specific gas constant for dry air, J/(kg·K).
const DRY_AIR_R: f64 = 287.05;

/// A double-track reduction of a vehicle for the T1 quasi-steady-state trim.
///
/// All geometry is in the ISO 8855 body frame (x forward, y left, z up) with the origin at the CG:
/// the front axle is at `x = +a_f`, the rear at `x = −b_r`; the left wheels at `y = +t/2`.
#[derive(Clone, Debug)]
pub struct T1Vehicle {
    /// Total mass, kg.
    pub mass_kg: f64,
    /// Yaw moment of inertia `I_zz`, kg·m² (currently informational — steady-state trim has ṙ = 0).
    pub izz: f64,
    /// Longitudinal distance from the CG to the front axle, m.
    pub a_f: f64,
    /// Longitudinal distance from the CG to the rear axle, m.
    pub b_r: f64,
    /// Wheelbase `L = a_f + b_r`, m.
    pub wheelbase_m: f64,
    /// Front track width, m.
    pub t_f: f64,
    /// Rear track width, m.
    pub t_r: f64,
    /// CG height above the ground, m.
    pub h_cg: f64,
    /// Roll-axis height directly under the CG, m (interpolated from the roll-centre heights).
    pub h_ra: f64,
    /// Front axle share of total roll stiffness, 0..1.
    pub roll_share_f: f64,
    /// Rear axle share of total roll stiffness, 0..1.
    pub roll_share_r: f64,
    /// Front roll-centre height, m.
    pub rc_f: f64,
    /// Rear roll-centre height, m.
    pub rc_r: f64,
    /// Front-axle tyre force model.
    pub tire_front: TireModel<f64>,
    /// Rear-axle tyre force model.
    pub tire_rear: TireModel<f64>,
    /// Front tyre cold inflation pressure, Pa.
    pub p_front: f64,
    /// Rear tyre cold inflation pressure, Pa.
    pub p_rear: f64,
    /// Lumped drag term `½·ρ·C_xA`, N per (m/s)² — the constant/reference value (the ride-height
    /// map, when installed, supersedes it at each operating point).
    pub qx: f64,
    /// Lumped front-downforce term `½·ρ·C_z,fA`, N per (m/s)² (constant/reference; see [`Self::qx`]).
    pub qz_f: f64,
    /// Lumped rear-downforce term `½·ρ·C_z,rA`, N per (m/s)² (constant/reference; see [`Self::qx`]).
    pub qz_r: f64,
    /// Air density, kg/m³ (for the ride-height aero-map platform equilibrium).
    pub rho: f64,
    /// Front ride rate at the wheel, N/m.
    pub k_ride_f: f64,
    /// Rear ride rate at the wheel, N/m.
    pub k_ride_r: f64,
    /// Static (design) front ride height, m.
    pub h_ref_f_m: f64,
    /// Static (design) rear ride height, m.
    pub h_ref_r_m: f64,
    /// Front anti-dive fraction, 0..1.
    pub anti_dive: f64,
    /// Rear anti-squat fraction, 0..1.
    pub anti_squat: f64,
    /// Optional ride-height/yaw aero map (§7.4). When present it supersedes the constant aero at
    /// each operating point via the [`AeroPlatform`] equilibrium; when absent the constant terms
    /// (`qx`/`qz_f`/`qz_r`) carry the aero.
    pub(crate) aero_map: Option<AeroMap>,
    /// Which wheels are driven, in `[FL, FR, RL, RR]` order.
    pub driven: [bool; 4],
    /// Brake-balance bar: front bias fraction, 0..1.
    pub brake_front_bias: f64,
    /// The topology powertrain: traction ceiling + differential torque split + energy accounting.
    pub(crate) powertrain: T1Powertrain,
    /// The primary driven axle's differential (cached from the powertrain for the trim's diff
    /// residual). `None` ⇒ the trim uses the equal-speed (locked) baseline.
    pub(crate) primary_diff: Option<PrimaryDiff>,
    /// Assembly notes / simplifications (nothing silent).
    notes: Vec<String>,
}

impl T1Vehicle {
    /// Assemble a [`T1Vehicle`] from a resolved vehicle, session conditions, and a source loader.
    ///
    /// The constant/reference aero is set here; install a ride-height/yaw map afterwards with
    /// [`T1Vehicle::install_aero_map`] to supersede it (the parquet decode is a native-edge step).
    ///
    /// # Errors
    /// [`T1Error`] if a referenced `.tyr` fails to load/validate, the tyre force model cannot be
    /// built, or the aero block has no `constant` coefficients and no map is installed.
    #[allow(clippy::too_many_lines)] // one linear assembly procedure; splitting it hurts clarity.
    pub fn assemble(
        vehicle: &ResolvedVehicle,
        conditions: &Conditions,
        loader: &dyn SourceLoader,
        allow_degraded: bool,
    ) -> Result<Self, T1Error> {
        let spec = &vehicle.spec;
        let mut notes = Vec::new();

        // --- Tyres: per-axle force models + cold pressures ---
        let (front_doc, _) = load_tyr(spec.tires.front.as_str(), loader)?;
        let (rear_doc, _) = load_tyr(spec.tires.rear.as_str(), loader)?;
        let (tire_front, front_notes) = TireModel::<f64>::from_tyr(&front_doc)?;
        let (tire_rear, rear_notes) = TireModel::<f64>::from_tyr(&rear_doc)?;
        for n in front_notes.iter().chain(&rear_notes) {
            if !notes.contains(&n.detail) {
                notes.push(n.detail.clone());
            }
        }
        let p_front = 1000.0 * front_doc.thermal.p_cold;
        let p_rear = 1000.0 * rear_doc.thermal.p_cold;
        let r_front = tyr_radius(&front_doc);
        let r_rear = tyr_radius(&rear_doc);

        // --- Chassis geometry (CG measured from the front axle, §6.1) ---
        let ch = &spec.chassis;
        let a_f = ch.cg[0];
        let wheelbase_m = ch.wheelbase_m;
        let b_r = wheelbase_m - a_f;
        let h_cg = ch.cg[2];
        let t_f = ch.track_m[0];
        let t_r = ch.track_m[1];

        // --- Suspension roll geometry ---
        let sus = &spec.suspension;
        let rc_f = sus.front.roll_center_height_m;
        let rc_r = sus.rear.roll_center_height_m;
        // Roll-axis height under the CG: interpolate the two roll-centre heights along the wheelbase.
        let h_ra = rc_f + (rc_r - rc_f) * (a_f / wheelbase_m);
        let roll_share_f = sus.front.roll_stiffness_share;
        let roll_share_r = sus.rear.roll_stiffness_share;
        let k_ride_f = sus.front.ride_rate_n_per_m;
        let k_ride_r = sus.rear.ride_rate_n_per_m;
        // Static ride heights + anti-effects are estimable (filled by the load pipeline's estimate
        // stage). Fall back to the axle nominals with a note if a hand-built ResolvedVehicle skipped
        // it, so the platform equilibrium always has a reference height.
        let h_ref_f_m =
            ride_height_or_default(sus.front.static_ride_height_m, 0.030, "front", &mut notes);
        let h_ref_r_m =
            ride_height_or_default(sus.rear.static_ride_height_m, 0.050, "rear", &mut notes);
        let anti_dive = sus.front.anti_dive.unwrap_or(0.0);
        let anti_squat = sus.rear.anti_squat.unwrap_or(0.0);

        // --- Aero: constant/reference terms (the ride-height map, installed via
        // `install_aero_map`, supersedes these at each operating point) ---
        let rho = 100.0 * conditions.air.pressure_hpa
            / (DRY_AIR_R * (conditions.air.temperature_c + 273.15));
        let (qx, qz_f, qz_r) = constant_aero(spec, rho, allow_degraded, &mut notes)?;

        // --- Driven wheels (which wheels can produce drive slip) ---
        let mut driven = [false; 4];
        for unit in &spec.drivetrain.units {
            for w in &unit.wheels {
                driven[wheel_index(*w)] = true;
            }
        }
        if !driven.iter().any(|&d| d) {
            notes
                .push("no drive units declare wheels; T1 traction uses all four wheels".to_owned());
            driven = [true; 4];
        }

        let brake_front_bias = spec.brakes.balance_bar;

        // --- Topology powertrain (traction ceiling + differential torque split) ---
        let powertrain = T1Powertrain::assemble(vehicle, loader, r_front, r_rear)?;
        let primary_diff = powertrain.primary_diff();
        for n in powertrain.notes() {
            if !notes.contains(n) {
                notes.push(n.clone());
            }
        }

        notes.push(
            "T1 load transfer: total longitudinal transfer + per-axle lateral transfer (roll-centre \
             geometry + roll-stiffness distribution). With a ride-height aero map installed, \
             anti-dive/anti-squat modulate the aero-platform heave; without one they do not affect \
             steady-state Fz. The differential torque split (open/locked/LSD/solid) enters the trim; \
             the powertrain torque envelope is the traction ceiling. Camber = 0 (camber maps land \
             later)."
                .to_owned(),
        );

        Ok(Self {
            mass_kg: ch.mass_kg,
            izz: ch.inertia[2],
            a_f,
            b_r,
            wheelbase_m,
            t_f,
            t_r,
            h_cg,
            h_ra,
            roll_share_f,
            roll_share_r,
            rc_f,
            rc_r,
            tire_front,
            tire_rear,
            p_front,
            p_rear,
            qx,
            qz_f,
            qz_r,
            rho,
            k_ride_f,
            k_ride_r,
            h_ref_f_m,
            h_ref_r_m,
            anti_dive,
            anti_squat,
            aero_map: None,
            driven,
            brake_front_bias,
            powertrain,
            primary_diff,
            notes,
        })
    }

    /// Install a decoded ride-height/yaw aero map (§7.4), superseding the constant aero.
    ///
    /// The parquet decode happens on the native/host edge (behind `outlap-schema`'s `parquet`
    /// feature); this crate stays wasm-clean by consuming the already-decoded [`GriddedTable`].
    /// `axis_names` are the vehicle's `aero.axes`.
    ///
    /// # Errors
    /// [`T1Error::UnknownAeroAxis`] or [`T1Error::AeroMap`] if the map cannot be built.
    pub fn install_aero_map(
        &mut self,
        table: &GriddedTable<f64>,
        axis_names: &[String],
    ) -> Result<(), T1Error> {
        let map = AeroMap::from_table(table, axis_names)?;
        self.notes.push(
            "aero: ride-height/yaw map consumed — downforce & drag from the aero-platform \
             equilibrium (ride heights from ride rates + downforce, iterated against the map)"
                .to_owned(),
        );
        self.aero_map = Some(map);
        Ok(())
    }

    /// The aero-platform parameters for the ride-height equilibrium.
    pub(crate) fn platform(&self) -> AeroPlatform {
        AeroPlatform {
            rho: self.rho,
            h_ref_f_m: self.h_ref_f_m,
            h_ref_r_m: self.h_ref_r_m,
            k_ride_f: self.k_ride_f,
            k_ride_r: self.k_ride_r,
            anti_dive: self.anti_dive,
            anti_squat: self.anti_squat,
            mass_kg: self.mass_kg,
            h_cg: self.h_cg,
            wheelbase_m: self.wheelbase_m,
        }
    }

    /// The effective lumped aero terms at an operating point `(v, ax, β)`: the ride-height-map
    /// platform equilibrium when a map is installed, else the constant/reference terms.
    ///
    /// `yaw_deg` is the aerodynamic yaw (vehicle sideslip β) in degrees; `drs` the DRS flag.
    pub(crate) fn aero_lumped(&self, v: f64, ax: f64, yaw_deg: f64, drs: f64) -> AeroLumped {
        match &self.aero_map {
            Some(map) => self.platform().equilibrium(map, v, ax, yaw_deg, drs),
            None => AeroLumped {
                qx: self.qx,
                qz_f: self.qz_f,
                qz_r: self.qz_r,
                h_f_m: self.h_ref_f_m,
                h_r_m: self.h_ref_r_m,
                converged: true,
            },
        }
    }

    /// Whether a ride-height/yaw aero map is installed.
    pub fn has_aero_map(&self) -> bool {
        self.aero_map.is_some()
    }

    /// Install a decoded `.ptm` efficiency/loss table onto drive unit `unit_idx` (energy
    /// accounting). Decode the sidecar with the axis names from [`T1Powertrain::map_axis_names`].
    ///
    /// # Errors
    /// [`T1Error::PowertrainMap`] / [`T1Error::UnknownDriveUnit`] if the table or index is invalid.
    pub fn install_powertrain_maps(
        &mut self,
        unit_idx: usize,
        table: &GriddedTable<f64>,
    ) -> Result<(), T1Error> {
        self.powertrain.install_maps(unit_idx, table)
    }

    /// The topology powertrain (traction ceiling, differential split, energy accounting).
    pub fn powertrain(&self) -> &T1Powertrain {
        &self.powertrain
    }

    /// The maximum wheel **drive** force the powertrain can put down at vehicle speed `v` (m/s), N —
    /// the traction ceiling PR7's g-g-g-v envelope caps the acceleration boundary with. The
    /// tyre-grip limit is enforced separately by the trim. Allocation-free.
    pub fn max_tractive_force(&self, v: f64) -> f64 {
        self.powertrain.max_drive_force(v)
    }

    /// The maximum powertrain-limited longitudinal acceleration at speed `v` (m/s), m/s² — the
    /// traction ceiling divided by mass (aero drag is applied by the caller/envelope generator).
    pub fn max_tractive_accel(&self, v: f64) -> f64 {
        self.max_tractive_force(v) / self.mass_kg
    }

    /// Front-axle static weight fraction `b_r / L` (share of vertical load carried by the front axle
    /// with no aero and no acceleration).
    pub fn front_weight_fraction(&self) -> f64 {
        self.b_r / self.wheelbase_m
    }

    /// Assembly notes / simplifications (nothing silent).
    pub fn notes(&self) -> &[String] {
        &self.notes
    }
}

/// The constant/reference lumped aero terms `(qx, qz_f, qz_r)` (`½·ρ·C·A`) from the aero block's
/// `constant:` coefficients, or a recorded zero-aero fallback under `allow_degraded`.
fn constant_aero(
    spec: &outlap_schema::Vehicle,
    rho: f64,
    allow_degraded: bool,
    notes: &mut Vec<String>,
) -> Result<(f64, f64, f64), T1Error> {
    match &spec.aero.constant {
        Some(c) => Ok((
            0.5 * rho * c.cx_a_m2,
            0.5 * rho * c.cz_front_a_m2,
            0.5 * rho * c.cz_rear_a_m2,
        )),
        // With no constant block a ride-height map must be installed to supply aero; otherwise
        // degrade to zero aero (recorded) or error.
        None if allow_degraded => {
            notes.push(
                "aero.constant absent — install a ride-height map or T1 runs with zero aero"
                    .to_owned(),
            );
            Ok((0.0, 0.0, 0.0))
        }
        None => Err(T1Error::NoConstantAero),
    }
}

/// The static ride height, or an axle nominal (recorded) when a hand-built vehicle skipped the
/// load pipeline's estimate stage. Committed vehicles always carry an estimated or explicit value.
fn ride_height_or_default(
    value: Option<f64>,
    default_m: f64,
    axle: &str,
    notes: &mut Vec<String>,
) -> f64 {
    value.unwrap_or_else(|| {
        notes.push(format!(
            "{axle} static ride height missing — assumed {} mm (only used by the ride-height aero map)",
            default_m * 1000.0
        ));
        default_m
    })
}

/// The tyre unloaded rolling radius, m (MF6.1 `UNLOADED_RADIUS`; 0.33 m fallback), for the
/// shaft-speed/force conversion in the powertrain traction limit.
fn tyr_radius(tyr: &Tyr) -> f64 {
    tyr.mf61.0.get("UNLOADED_RADIUS").copied().unwrap_or(0.33)
}

/// Map a [`Wheel`] to its index in the canonical `[FL, FR, RL, RR]` order.
pub(crate) fn wheel_index(w: Wheel) -> usize {
    match w {
        Wheel::Fl => 0,
        Wheel::Fr => 1,
        Wheel::Rl => 2,
        Wheel::Rr => 3,
    }
}
