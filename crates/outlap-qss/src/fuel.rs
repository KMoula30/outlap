// SPDX-License-Identifier: AGPL-3.0-only
//! Fuel mass as a QSS slow state (§8.1, D-M6-4). The tank drains as the ICE burns fuel, and the
//! shrinking mass + migrating centre of gravity feed the point-mass longitudinal equations and the
//! Decision-#31 mass/CG envelope corrections through the [`crate::solver`] per-station coupling.
//!
//! Mass semantics (D-M6-4a): `chassis.mass_kg` is the DRY car+driver mass and `chassis.cg` the dry
//! CG; `fuel.initial_kg` ADDS on top at the tank centroid, so the full-tank reference is
//! `m₀ = dry_mass + initial_kg` at the mass-weighted blend of the dry CG and the tank centroid. The
//! g-g-g-v envelope is built at that full-tank reference (D-M6-4b), so the correction is exactly 1.0
//! at lap start and drifts as the tank empties. No `fuel:` block ⇒ this module never runs ⇒
//! byte-identical to v0.3.0.

use outlap_schema::vehicle::{Fuel, Vehicle};

use crate::t1::T1Vehicle;

/// The parsed fuel model: the dry inertial reference, the tank centroid, the heating value, and the
/// optional energy-flow limit. Cheap `Copy` value (no maps) so it rides on the coupling.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FuelModel {
    /// Dry (empty-tank) mass, kg — the all-inclusive `chassis.mass_kg` (car + driver, no fuel).
    pub dry_mass_kg: f64,
    /// Initial (race-start) fuel mass, kg.
    pub initial_kg: f64,
    /// Tank capacity, kg (the fuel mass is clamped to `[0, tank_kg]`).
    pub tank_kg: f64,
    /// Lower heating value, J/kg.
    pub lhv_j_per_kg: f64,
    /// Dry-CG longitudinal split `a_f` (front-axle → CG), m.
    pub a_f_dry: f64,
    /// Dry-CG height `h_cg`, m.
    pub h_cg_dry: f64,
    /// Fuel-tank centroid `a_f`, m (`a_f_dry − offset_x`: a tank ahead of the dry CG sits closer to
    /// the front axle, ISO 8855 +x forward).
    pub a_f_tank: f64,
    /// Fuel-tank centroid height, m (`h_cg_dry + offset_z`).
    pub h_cg_tank: f64,
    /// Optional energy-flow cap, MJ/h (the flat FIA C5.2.4 ceiling).
    pub flow_mj_per_h: Option<f64>,
    /// Optional low-rpm flow line `(below_rpm, slope MJ/h/rpm, intercept MJ/h)` (FIA C5.2.5).
    pub flow_rpm_line: Option<(f64, f64, f64)>,
}

impl FuelModel {
    /// Build the fuel model from a resolved [`Vehicle`], or `None` when the car carries no `fuel:`
    /// block (⇒ mass is the assembly constant, byte-identical to pre-M6).
    #[must_use]
    pub fn from_spec(spec: &Vehicle) -> Option<Self> {
        let fuel = spec.fuel.as_ref()?;
        Some(Self::from_fuel(
            fuel,
            &spec.chassis.cg,
            spec.chassis.mass_kg,
        ))
    }

    /// Build from a [`Fuel`] block plus the dry CG `[x, _, z]` and dry mass.
    #[must_use]
    pub fn from_fuel(fuel: &Fuel, dry_cg: &[f64; 3], dry_mass_kg: f64) -> Self {
        let [ox, oz] = fuel.cg_offset_m.unwrap_or([0.0, 0.0]);
        Self {
            dry_mass_kg,
            initial_kg: fuel.initial_kg,
            tank_kg: fuel.tank_kg,
            lhv_j_per_kg: fuel.lhv_j_per_kg,
            a_f_dry: dry_cg[0],
            h_cg_dry: dry_cg[2],
            a_f_tank: dry_cg[0] - ox,
            h_cg_tank: dry_cg[2] + oz,
            flow_mj_per_h: fuel.flow_limit.as_ref().map(|f| f.mj_per_h),
            flow_rpm_line: fuel.flow_limit.as_ref().and_then(|f| {
                f.rpm_line
                    .as_ref()
                    .map(|l| (l.below_rpm, l.slope_mj_per_h_per_rpm, l.intercept_mj_per_h))
            }),
        }
    }

    /// Full-tank reference mass `m₀ = dry + initial`, kg (the envelope-build mass, D-M6-4b).
    #[must_use]
    pub fn full_mass_kg(&self) -> f64 {
        self.dry_mass_kg + self.initial_kg
    }

    /// Total mass carrying `fuel_kg` of fuel, kg.
    #[must_use]
    pub fn mass_at(&self, fuel_kg: f64) -> f64 {
        self.dry_mass_kg + fuel_kg.clamp(0.0, self.tank_kg)
    }

    /// The mass-weighted CG `(a_f, h_cg)` carrying `fuel_kg` of fuel, m. At `fuel_kg = 0` it is the
    /// dry CG; at `initial_kg` the full-tank reference CG. Linear in the fuel mass.
    #[must_use]
    pub fn cg_at(&self, fuel_kg: f64) -> (f64, f64) {
        let f = fuel_kg.clamp(0.0, self.tank_kg);
        let m = self.dry_mass_kg + f;
        if m <= 0.0 {
            return (self.a_f_dry, self.h_cg_dry);
        }
        let a_f = (self.dry_mass_kg * self.a_f_dry + f * self.a_f_tank) / m;
        let h_cg = (self.dry_mass_kg * self.h_cg_dry + f * self.h_cg_tank) / m;
        (a_f, h_cg)
    }

    /// The full-tank reference CG `(a_f, h_cg)`, m (the geometry the envelope is built at).
    #[must_use]
    pub fn full_cg(&self) -> (f64, f64) {
        self.cg_at(self.initial_kg)
    }

    /// The **ICE mechanical-power ceiling** the flat energy-flow cap imposes, W, or `None` when the
    /// car has no flat flow limit (§8.1, D-M6-5). A fuel-energy rate `EF` (MJ/h) burns
    /// `P_mech = η·EF` of crank power, so the FIA flat fuel-flow ceiling caps the ICE mechanical power
    /// at `η · flow_mj_per_h · 1e6/3600` — a CONSTRAINT ON AVAILABLE POWER that shrinks the traction
    /// envelope (the car does less work and burns proportionally less), never a clamp on the `ṁ`
    /// accounting (that would break the §14 energy closure). The C5.2.5 low-rpm line governs only
    /// sub-`below_rpm` operation, where the cap is far above any binding force, so the envelope uses
    /// the flat ceiling alone.
    #[must_use]
    pub fn ice_power_cap_w(&self, ice_thermal_eff: f64) -> Option<f64> {
        self.flow_mj_per_h
            .map(|mj| ice_thermal_eff * mj * 1.0e6 / 3600.0)
    }

    /// The energy-flow cap at crank speed `rpm`, MJ/h, or `None` when the car has no flow limit.
    /// Below the C5.2.5 rpm line the linear `slope·N + intercept` applies; at/above it the flat
    /// `flow_mj_per_h`. The lower of the two binds where both are present.
    #[must_use]
    pub fn flow_cap_mj_per_h(&self, rpm: f64) -> Option<f64> {
        let flat = self.flow_mj_per_h;
        let line = self.flow_rpm_line.map(|(below, slope, intercept)| {
            if rpm < below {
                slope * rpm + intercept
            } else {
                f64::INFINITY
            }
        });
        match (flat, line) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> FuelModel {
        // Dry car 730 kg, CG a_f = 1.6 m, h_cg = 0.30 m; 80 kg tank 0.30 m BEHIND the dry CG and
        // 0.10 m below it (offset [-0.30, -0.10] ISO 8855: −x = rearward, −z = lower).
        let fuel = Fuel {
            initial_kg: 80.0,
            tank_kg: 110.0,
            cg_offset_m: Some([-0.30, -0.10]),
            lhv_j_per_kg: 43.0e6,
            flow_limit: None,
        };
        FuelModel::from_fuel(&fuel, &[1.6, 0.0, 0.30], 730.0)
    }

    #[test]
    fn mass_is_dry_plus_fuel_and_full_is_m0() {
        let m = model();
        assert!((m.mass_at(0.0) - 730.0).abs() < 1e-12);
        assert!((m.mass_at(80.0) - 810.0).abs() < 1e-12);
        assert!((m.full_mass_kg() - 810.0).abs() < 1e-12);
        // Clamp: negative / over-tank fuel is bounded.
        assert!((m.mass_at(-5.0) - 730.0).abs() < 1e-12);
        assert!((m.mass_at(200.0) - (730.0 + 110.0)).abs() < 1e-12);
    }

    #[test]
    fn cg_blends_between_dry_and_tank() {
        let m = model();
        // Empty tank → dry CG exactly.
        let (a0, h0) = m.cg_at(0.0);
        assert!((a0 - 1.6).abs() < 1e-12 && (h0 - 0.30).abs() < 1e-12);
        // Tank centroid is rearward (a_f_tank = 1.6 − (−0.30) = 1.9) and lower (0.20). Full-tank CG
        // shifts toward it: a_f increases (CG moves back), h_cg drops.
        assert!((m.a_f_tank - 1.9).abs() < 1e-12);
        assert!((m.h_cg_tank - 0.20).abs() < 1e-12);
        let (af, hf) = m.full_cg();
        assert!(af > 1.6 && af < 1.9, "full a_f {af} between dry and tank");
        assert!(
            hf < 0.30 && hf > 0.20,
            "full h_cg {hf} between dry and tank"
        );
        // Mass-weighted mean: a_f_full = (730·1.6 + 80·1.9)/810.
        let want = (730.0 * 1.6 + 80.0 * 1.9) / 810.0;
        assert!((af - want).abs() < 1e-12);
    }

    #[test]
    fn cg_migration_is_monotone_as_fuel_burns() {
        let m = model();
        // As fuel falls 80 → 0, a_f decreases monotonically toward the dry 1.6.
        let mut prev = m.cg_at(80.0).0;
        for k in (0..80).rev() {
            let a = m.cg_at(f64::from(k)).0;
            assert!(a <= prev + 1e-12, "a_f monotone decreasing as fuel burns");
            prev = a;
        }
        assert!((m.cg_at(0.0).0 - 1.6).abs() < 1e-12);
    }

    #[test]
    fn no_offset_means_no_cg_migration() {
        let fuel = Fuel {
            initial_kg: 80.0,
            tank_kg: 110.0,
            cg_offset_m: None,
            lhv_j_per_kg: 43.0e6,
            flow_limit: None,
        };
        let m = FuelModel::from_fuel(&fuel, &[1.6, 0.0, 0.30], 730.0);
        assert!((m.cg_at(80.0).0 - 1.6).abs() < 1e-12);
        assert!((m.cg_at(0.0).0 - 1.6).abs() < 1e-12);
        assert!((m.full_cg().1 - 0.30).abs() < 1e-12);
    }

    #[test]
    fn flow_cap_takes_the_low_rpm_line_then_the_flat() {
        // Flat 4300 MJ/h; below 10500 rpm EF = 0.27·N + 165.
        let fuel = Fuel {
            initial_kg: 80.0,
            tank_kg: 110.0,
            cg_offset_m: None,
            lhv_j_per_kg: 43.0e6,
            flow_limit: Some(outlap_schema::vehicle::FuelFlowLimit {
                mj_per_h: 4300.0,
                rpm_line: Some(outlap_schema::vehicle::RpmFlowLine {
                    below_rpm: 10_500.0,
                    slope_mj_per_h_per_rpm: 0.27,
                    intercept_mj_per_h: 165.0,
                }),
            }),
        };
        let m = FuelModel::from_fuel(&fuel, &[1.6, 0.0, 0.30], 730.0);
        // At 8000 rpm the line binds: 0.27·8000 + 165 = 2325 MJ/h < 4300.
        assert!((m.flow_cap_mj_per_h(8_000.0).unwrap() - 2325.0).abs() < 1e-9);
        // At 12000 rpm the line is above (INF), so the flat 4300 binds.
        assert!((m.flow_cap_mj_per_h(12_000.0).unwrap() - 4300.0).abs() < 1e-9);
    }
}

/// A fuel coupling handed to the solve: the [`FuelModel`] plus the [`T1Vehicle`] whose powertrain
/// yields the per-station ICE fuel rate. Separate from the electro stack so a car can burn fuel
/// without a battery (though the shipped F1 pairs both).
#[derive(Clone, Copy, Debug)]
pub struct FuelCoupling<'a> {
    /// The fuel inertial + flow model.
    pub model: FuelModel,
    /// The T1 vehicle carrying the installed powertrain maps (the ICE fuel-rate source).
    pub vehicle: &'a T1Vehicle,
}
