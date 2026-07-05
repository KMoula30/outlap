// SPDX-License-Identifier: AGPL-3.0-only
//! T1 assembly: reduce a [`ResolvedVehicle`] + `conditions` into a [`T1Vehicle`] for the trim solver.
//!
//! Unlike the T0 point-mass reduction, T1 keeps the per-axle tyre force models, the chassis mass /
//! CG / yaw inertia, the suspension roll geometry, and the aero coefficients — everything the
//! quasi-steady-state double-track trim ([`crate::t1::trim`]) needs. Powertrain traction is still a
//! simplified longitudinal-slip control here (constant aero, no drivetrain graph); PR4/PR3 refine
//! the traction limit and the ride-height aero map.

use outlap_schema::conditions::Conditions;
use outlap_schema::io::SourceLoader;
use outlap_schema::load::load_tyr;
use outlap_schema::vehicle::Wheel;
use outlap_schema::ResolvedVehicle;
use outlap_tire::TireModel;

use crate::error::T1Error;

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
    /// Lumped drag term `½·ρ·C_xA`, N per (m/s)².
    pub qx: f64,
    /// Lumped front-downforce term `½·ρ·C_z,fA`, N per (m/s)².
    pub qz_f: f64,
    /// Lumped rear-downforce term `½·ρ·C_z,rA`, N per (m/s)².
    pub qz_r: f64,
    /// Which wheels are driven, in `[FL, FR, RL, RR]` order.
    pub driven: [bool; 4],
    /// Brake-balance bar: front bias fraction, 0..1.
    pub brake_front_bias: f64,
    /// Assembly notes / simplifications (nothing silent).
    notes: Vec<String>,
}

impl T1Vehicle {
    /// Assemble a [`T1Vehicle`] from a resolved vehicle, session conditions, and a source loader.
    ///
    /// # Errors
    /// [`T1Error`] if a referenced `.tyr` fails to load/validate, the tyre force model cannot be
    /// built, or the aero block has no `constant` coefficients (the ride-height map is PR3).
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

        // --- Aero (constant only in T1/PR2; the ride-height map arrives in PR3) ---
        let rho = 100.0 * conditions.air.pressure_hpa
            / (DRY_AIR_R * (conditions.air.temperature_c + 273.15));
        let (qx, qz_f, qz_r) = match &spec.aero.constant {
            Some(c) => (
                0.5 * rho * c.cx_a_m2,
                0.5 * rho * c.cz_front_a_m2,
                0.5 * rho * c.cz_rear_a_m2,
            ),
            None if allow_degraded => {
                notes.push(
                    "aero.constant absent — T1 ran with zero aero (ride-height map arrives in PR3)"
                        .to_owned(),
                );
                (0.0, 0.0, 0.0)
            }
            None => return Err(T1Error::NoConstantAero),
        };

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

        notes.push(
            "T1 load transfer: total longitudinal transfer + per-axle lateral transfer (roll-centre \
             geometry + roll-stiffness distribution); anti-dive/anti-squat affect ride height (PR3), \
             not steady-state Fz. Constant aero; simplified longitudinal-slip traction (PR4 adds the \
             drivetrain graph). Camber = 0 (camber maps land later)."
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
            driven,
            brake_front_bias,
            notes,
        })
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

/// Map a [`Wheel`] to its index in the canonical `[FL, FR, RL, RR]` order.
pub(crate) fn wheel_index(w: Wheel) -> usize {
    match w {
        Wheel::Fl => 0,
        Wheel::Fr => 1,
        Wheel::Rl => 2,
        Wheel::Rr => 3,
    }
}
