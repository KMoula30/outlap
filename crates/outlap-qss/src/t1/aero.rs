// SPDX-License-Identifier: AGPL-3.0-only
//! T1 ride-height / yaw aero map (§7.4) and the aero-platform equilibrium.
//!
//! The primary aero representation is a gridded map `{C_z,front·A, C_z,rear·A, C_x·A} = f(h_front,
//! h_rear, yaw [, DRS])` (Perantoni & Limebeer 2014's speed-dependent aero, generalised to explicit
//! ride heights — the first open ride-height aero-map representation, §5.5). The *effective* lumped
//! coefficients at an operating point come from the **aero-platform equilibrium**: the platform
//! sinks under its own downforce (`h = static − ΔF_spring / (2·ride_rate)`) which changes the
//! coefficients, which change the downforce — a fixed point ([`AeroPlatform::equilibrium`]).
//!
//! Construction (parquet decode) happens on the native/host edge; this module consumes the already
//! decoded, wasm-clean [`GriddedTable`] (Decision, PR1). Evaluation is zero-allocation.
//!
//! Clean-room from published literature: Perantoni & Limebeer, VSD 52(5), 2014 (the reference car
//! and its speed-dependent aero); Katz, *Race Car Aerodynamics*, 1995 (ground-effect ride-height
//! sensitivity, rake); the platform-equilibrium fixed point is a standard quasi-static heave balance.

use outlap_core::{GriddedMapN, GriddedTable, OutOfDomain, MAX_DIMS};

use crate::error::T1Error;

/// Canonical value-column name for the front-axle downforce coefficient × area, m².
const COL_CZ_FRONT: &str = "cz_front_a_m2";
/// Canonical value-column name for the rear-axle downforce coefficient × area, m².
const COL_CZ_REAR: &str = "cz_rear_a_m2";
/// Canonical value-column name for the drag coefficient × area, m².
const COL_CX: &str = "cx_a_m2";

/// Which physical quantity a map axis carries (resolved from the axis name).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AxisRole {
    /// Front-axle ride height, mm.
    RideFront,
    /// Rear-axle ride height, mm.
    RideRear,
    /// Aerodynamic yaw (vehicle sideslip) angle, degrees.
    Yaw,
    /// DRS flag (0 closed, 1 open).
    Drs,
}

impl AxisRole {
    fn from_name(name: &str) -> Result<Self, T1Error> {
        match name {
            "ride_height_f_mm" => Ok(Self::RideFront),
            "ride_height_r_mm" => Ok(Self::RideRear),
            "yaw_deg" => Ok(Self::Yaw),
            "drs_flag" => Ok(Self::Drs),
            other => Err(T1Error::UnknownAeroAxis {
                name: other.to_owned(),
            }),
        }
    }
}

/// The three aero coefficients × area at one operating point (m²).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AeroCoeffs {
    /// Front-axle downforce coefficient × area `C_z,front·A`, m².
    pub cz_front_a_m2: f64,
    /// Rear-axle downforce coefficient × area `C_z,rear·A`, m².
    pub cz_rear_a_m2: f64,
    /// Drag coefficient × area `C_x·A`, m².
    pub cx_a_m2: f64,
}

/// A decoded ride-height/yaw aero map: one interpolant per coefficient over shared axes.
///
/// Build with [`AeroMap::from_table`] from a decoded [`GriddedTable`]; evaluate with
/// [`AeroMap::eval`]. Out-of-domain queries **clamp** on every axis (the equilibrium can push ride
/// heights below the tabulated grid; clamping keeps the coefficients at their edge values rather
/// than extrapolating a ground-effect curve past its validity).
#[derive(Clone, Debug)]
pub struct AeroMap {
    cz_front: GriddedMapN<f64>,
    cz_rear: GriddedMapN<f64>,
    cx: GriddedMapN<f64>,
    /// Axis roles in the map's axis order.
    roles: Vec<AxisRole>,
    ndim: usize,
}

impl AeroMap {
    /// Build an [`AeroMap`] from a decoded table and the vehicle's ordered axis names.
    ///
    /// `axis_names` are `aero.axes` from the vehicle document (e.g.
    /// `["ride_height_f_mm", "ride_height_r_mm", "yaw_deg", "drs_flag"]`). The value columns
    /// `cz_front_a_m2`, `cz_rear_a_m2`, `cx_a_m2` must all be present.
    ///
    /// # Errors
    /// [`T1Error::UnknownAeroAxis`] for an unrecognised axis name, or [`T1Error::AeroMap`] if a
    /// value column is missing or the grid is not rectilinear.
    pub fn from_table(table: &GriddedTable<f64>, axis_names: &[String]) -> Result<Self, T1Error> {
        let roles: Vec<AxisRole> = axis_names
            .iter()
            .map(|n| AxisRole::from_name(n))
            .collect::<Result<_, _>>()?;
        let ndim = roles.len();
        let modes = vec![OutOfDomain::Clamp; ndim];
        let cz_front = table.map(COL_CZ_FRONT, modes.clone())?;
        let cz_rear = table.map(COL_CZ_REAR, modes.clone())?;
        let cx = table.map(COL_CX, modes)?;
        Ok(Self {
            cz_front,
            cz_rear,
            cx,
            roles,
            ndim,
        })
    }

    /// Evaluate the three coefficients at a ride-height / yaw / DRS operating point.
    ///
    /// `h_f_mm`/`h_r_mm` are the front/rear ride heights (mm), `yaw_deg` the aerodynamic yaw angle
    /// (degrees), `drs` the DRS flag (0/1). Axes absent from the map are simply not queried.
    /// Zero-allocation.
    #[must_use]
    pub fn eval(&self, h_f_mm: f64, h_r_mm: f64, yaw_deg: f64, drs: f64) -> AeroCoeffs {
        let mut coord = [0.0_f64; MAX_DIMS];
        for (i, role) in self.roles.iter().enumerate() {
            coord[i] = match role {
                AxisRole::RideFront => h_f_mm,
                AxisRole::RideRear => h_r_mm,
                AxisRole::Yaw => yaw_deg,
                AxisRole::Drs => drs,
            };
        }
        let x = &coord[..self.ndim];
        AeroCoeffs {
            cz_front_a_m2: self.cz_front.eval(x),
            cz_rear_a_m2: self.cz_rear.eval(x),
            cx_a_m2: self.cx.eval(x),
        }
    }
}

/// Suspension platform parameters for the aero-platform equilibrium (per axle, SI).
#[derive(Clone, Copy, Debug)]
pub struct AeroPlatform {
    /// Air density, kg/m³.
    pub rho: f64,
    /// Static (design) front ride height, m.
    pub h_ref_f_m: f64,
    /// Static (design) rear ride height, m.
    pub h_ref_r_m: f64,
    /// Front ride rate at the wheel, N/m (axle rate is `2×` this).
    pub k_ride_f: f64,
    /// Rear ride rate at the wheel, N/m.
    pub k_ride_r: f64,
    /// Front anti-dive fraction (0..1).
    pub anti_dive: f64,
    /// Rear anti-squat fraction (0..1).
    pub anti_squat: f64,
    /// Total mass, kg (for the longitudinal-transfer heave term).
    pub mass_kg: f64,
    /// CG height, m.
    pub h_cg: f64,
    /// Wheelbase, m.
    pub wheelbase_m: f64,
}

/// The lumped `½·ρ·C·A` aero terms (N per (m/s)²) at the equilibrium platform, plus the converged
/// ride heights (for reporting/diagnostics).
#[derive(Clone, Copy, Debug)]
pub struct AeroLumped {
    /// Lumped drag term `½·ρ·C_x·A`, N per (m/s)².
    pub qx: f64,
    /// Lumped front-downforce term `½·ρ·C_z,front·A`, N per (m/s)².
    pub qz_f: f64,
    /// Lumped rear-downforce term `½·ρ·C_z,rear·A`, N per (m/s)².
    pub qz_r: f64,
    /// Converged front ride height, m.
    pub h_f_m: f64,
    /// Converged rear ride height, m.
    pub h_r_m: f64,
    /// Whether the fixed point met its tolerance within the iteration cap.
    pub converged: bool,
}

/// Maximum aero-platform fixed-point iterations (ample headroom for the damped contraction to reach
/// the tolerance from any start).
const MAX_AERO_ITERS: usize = 60;
/// Ride-height convergence tolerance, m (0.1 nm). Deliberately far below the trim's residual
/// tolerance (1e-10 scaled) so the converged coefficients are effectively a smooth function of the
/// chassis state — the nested fixed point never injects an iteration-count discontinuity into the
/// outer Newton's finite-difference Jacobian.
const AERO_TOL_M: f64 = 1e-10;
/// Under-relaxation on the ride-height update (keeps the fixed point stable near soft platforms).
const AERO_DAMP: f64 = 0.6;

impl AeroPlatform {
    /// Solve the aero-platform equilibrium at `(v, ax, yaw_deg, drs)` and return the effective lumped
    /// coefficients. Damped fixed point on the ride heights against `map`, deterministic and
    /// zero-allocation.
    ///
    /// The platform sinks under downforce and the longitudinal load transfer (`m·ax·h_cg/L`,
    /// reacted geometrically by anti-dive under braking / anti-squat under acceleration): for each
    /// axle `h = h_static − (F_downforce + F_lt) / (2·ride_rate)`, clamped at `0` (planked). The
    /// downforce is re-evaluated from the map at the new heights until the heights stop moving.
    #[must_use]
    pub fn equilibrium(
        &self,
        map: &AeroMap,
        v: f64,
        ax: f64,
        yaw_deg: f64,
        drs: f64,
    ) -> AeroLumped {
        let qdyn = 0.5 * self.rho * v * v; // dynamic pressure factor: F = qdyn · (C·A)
                                           // Longitudinal load transfer moved onto the springs (signed per axle; + compresses).
        let t = self.mass_kg * ax * self.h_cg / self.wheelbase_m; // + under acceleration
        let (front_lt, rear_lt) = if ax >= 0.0 {
            // Acceleration: rear squats (reduced by anti-squat), front lifts.
            (-t, (1.0 - self.anti_squat) * t)
        } else {
            // Braking (t < 0): front dives (reduced by anti-dive), rear lifts.
            ((1.0 - self.anti_dive) * (-t), t)
        };

        let mut h_f = self.h_ref_f_m;
        let mut h_r = self.h_ref_r_m;
        let mut coeffs = map.eval(h_f * 1000.0, h_r * 1000.0, yaw_deg, drs);
        let mut converged = false;
        for _ in 0..MAX_AERO_ITERS {
            let f_dz_f = qdyn * coeffs.cz_front_a_m2;
            let f_dz_r = qdyn * coeffs.cz_rear_a_m2;
            let target_f = (self.h_ref_f_m - (f_dz_f + front_lt) / (2.0 * self.k_ride_f)).max(0.0);
            let target_r = (self.h_ref_r_m - (f_dz_r + rear_lt) / (2.0 * self.k_ride_r)).max(0.0);
            let new_f = h_f + AERO_DAMP * (target_f - h_f);
            let new_r = h_r + AERO_DAMP * (target_r - h_r);
            let moved = (new_f - h_f).abs().max((new_r - h_r).abs());
            h_f = new_f;
            h_r = new_r;
            coeffs = map.eval(h_f * 1000.0, h_r * 1000.0, yaw_deg, drs);
            if moved < AERO_TOL_M {
                converged = true;
                break;
            }
        }
        AeroLumped {
            qx: 0.5 * self.rho * coeffs.cx_a_m2,
            qz_f: 0.5 * self.rho * coeffs.cz_front_a_m2,
            qz_r: 0.5 * self.rho * coeffs.cz_rear_a_m2,
            h_f_m: h_f,
            h_r_m: h_r,
            converged,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 4-axis F1-style grid used by the fixtures (ride_f, ride_r, yaw, drs).
    const RF: [f64; 5] = [10.0, 20.0, 30.0, 40.0, 60.0];
    const RR: [f64; 5] = [30.0, 50.0, 70.0, 100.0, 140.0];
    const YAW: [f64; 5] = [-8.0, -4.0, 0.0, 4.0, 8.0];
    const DRS: [f64; 2] = [0.0, 1.0];

    /// Build a 4-axis [`AeroMap`] whose coefficients come from a closure `(hf,hr,yaw,drs)`.
    fn build_map(f: impl Fn(f64, f64, f64, f64) -> AeroCoeffs) -> AeroMap {
        let names = [
            "ride_height_f_mm",
            "ride_height_r_mm",
            "yaw_deg",
            "drs_flag",
        ];
        let (mut c_hf, mut c_hr, mut c_yaw, mut c_drs) = (vec![], vec![], vec![], vec![]);
        let (mut czf, mut czr, mut cx) = (vec![], vec![], vec![]);
        for &hf in &RF {
            for &hr in &RR {
                for &yaw in &YAW {
                    for &drs in &DRS {
                        let c = f(hf, hr, yaw, drs);
                        c_hf.push(hf);
                        c_hr.push(hr);
                        c_yaw.push(yaw);
                        c_drs.push(drs);
                        czf.push(c.cz_front_a_m2);
                        czr.push(c.cz_rear_a_m2);
                        cx.push(c.cx_a_m2);
                    }
                }
            }
        }
        let columns = vec![
            ("ride_height_f_mm".to_owned(), c_hf),
            ("ride_height_r_mm".to_owned(), c_hr),
            ("yaw_deg".to_owned(), c_yaw),
            ("drs_flag".to_owned(), c_drs),
            ("cz_front_a_m2".to_owned(), czf),
            ("cz_rear_a_m2".to_owned(), czr),
            ("cx_a_m2".to_owned(), cx),
        ];
        let table = GriddedTable::from_long(&columns, &names).unwrap();
        let names_owned: Vec<String> = names.iter().map(|s| (*s).to_owned()).collect();
        AeroMap::from_table(&table, &names_owned).unwrap()
    }

    /// A constant map (every node the same coefficients).
    fn constant_map(czf: f64, czr: f64, cx: f64) -> AeroMap {
        build_map(|_, _, _, _| AeroCoeffs {
            cz_front_a_m2: czf,
            cz_rear_a_m2: czr,
            cx_a_m2: cx,
        })
    }

    fn platform() -> AeroPlatform {
        AeroPlatform {
            rho: 1.2,
            h_ref_f_m: 0.040,
            h_ref_r_m: 0.090,
            k_ride_f: 220_000.0,
            k_ride_r: 240_000.0,
            anti_dive: 0.4,
            anti_squat: 0.3,
            mass_kg: 768.0,
            h_cg: 0.30,
            wheelbase_m: 3.40,
        }
    }

    #[test]
    fn unknown_axis_is_rejected() {
        let table = GriddedTable::from_long(
            &[
                ("bogus".to_owned(), vec![0.0, 1.0, 0.0, 1.0]),
                ("drs_flag".to_owned(), vec![0.0, 0.0, 1.0, 1.0]),
                ("cz_front_a_m2".to_owned(), vec![1.0, 1.0, 1.0, 1.0]),
                ("cz_rear_a_m2".to_owned(), vec![1.0, 1.0, 1.0, 1.0]),
                ("cx_a_m2".to_owned(), vec![1.0, 1.0, 1.0, 1.0]),
            ],
            &["bogus", "drs_flag"],
        )
        .unwrap();
        let axes = vec!["bogus".to_owned(), "drs_flag".to_owned()];
        assert!(matches!(
            AeroMap::from_table(&table, &axes),
            Err(T1Error::UnknownAeroAxis { .. })
        ));
    }

    #[test]
    fn missing_value_column_errors() {
        // A table with the axes but no `cx_a_m2` column.
        let names = ["ride_height_f_mm", "drs_flag"];
        let table = GriddedTable::from_long(
            &[
                ("ride_height_f_mm".to_owned(), vec![10.0, 10.0, 20.0, 20.0]),
                ("drs_flag".to_owned(), vec![0.0, 1.0, 0.0, 1.0]),
                ("cz_front_a_m2".to_owned(), vec![1.0; 4]),
                ("cz_rear_a_m2".to_owned(), vec![1.0; 4]),
            ],
            &names,
        )
        .unwrap();
        let axes: Vec<String> = names.iter().map(|s| (*s).to_owned()).collect();
        assert!(matches!(
            AeroMap::from_table(&table, &axes),
            Err(T1Error::AeroMap(_))
        ));
    }

    #[test]
    fn reference_node_reproduced_exactly() {
        let map = constant_map(1.9, 2.6, 1.25);
        let c = map.eval(30.0, 70.0, 0.0, 0.0);
        assert!((c.cz_front_a_m2 - 1.9).abs() < 1e-12);
        assert!((c.cz_rear_a_m2 - 2.6).abs() < 1e-12);
        assert!((c.cx_a_m2 - 1.25).abs() < 1e-12);
    }

    #[test]
    fn equilibrium_converges_and_sinks_under_downforce() {
        // Ground-effect map: front/rear downforce rise as ride height drops (mm).
        let map = build_map(|hf, hr, yaw, _drs| {
            let yaw_f = 1.0 - 0.08 * (yaw / 10.0).powi(2);
            AeroCoeffs {
                cz_front_a_m2: 1.9 * (1.0 + 0.35 * (30.0 - hf) / 30.0) * yaw_f,
                cz_rear_a_m2: 2.6 * (1.0 + 0.30 * (70.0 - hr) / 70.0) * yaw_f,
                cx_a_m2: 1.25,
            }
        });
        let p = platform();
        // Sweep speeds: every solve converges, and the platform sinks monotonically with speed
        // while the effective downforce coefficient rises (ground effect).
        let mut last_hf = p.h_ref_f_m + 1.0;
        let mut last_hr = p.h_ref_r_m + 1.0;
        let mut last_qzf = 0.0;
        for i in 0..12 {
            let v = 10.0 + 8.0 * f64::from(i);
            let a = p.equilibrium(&map, v, 0.0, 0.0, 0.0);
            assert!(a.converged, "aero equilibrium did not converge at v={v}");
            assert!(a.h_f_m > 0.0 && a.h_f_m <= p.h_ref_f_m + 1e-9);
            assert!(a.h_r_m > 0.0 && a.h_r_m <= p.h_ref_r_m + 1e-9);
            assert!(a.h_f_m <= last_hf + 1e-9, "front should sink with speed");
            assert!(a.h_r_m <= last_hr + 1e-9, "rear should sink with speed");
            assert!(
                a.qz_f >= last_qzf - 1e-9,
                "front downforce should grow with speed"
            );
            last_hf = a.h_f_m;
            last_hr = a.h_r_m;
            last_qzf = a.qz_f;
        }
    }

    #[test]
    fn constant_map_is_speed_and_yaw_invariant() {
        let map = constant_map(1.9, 2.6, 1.25);
        let p = platform();
        let ref_ = p.equilibrium(&map, 30.0, 0.0, 0.0, 0.0);
        for &(v, yaw) in &[(60.0, 4.0), (80.0, -8.0), (20.0, 0.0)] {
            let a = p.equilibrium(&map, v, 0.0, yaw, 0.0);
            assert!((a.qz_f - ref_.qz_f).abs() < 1e-12);
            assert!((a.qz_r - ref_.qz_r).abs() < 1e-12);
            assert!((a.qx - ref_.qx).abs() < 1e-12);
        }
    }

    #[test]
    fn yaw_reduces_downforce_only_when_map_depends_on_yaw() {
        let p = platform();
        // Yaw-flat map vs yaw-sensitive map at the same yawed operating point.
        let flat = constant_map(1.9, 2.6, 1.25);
        let yawed = build_map(|_hf, _hr, yaw, _drs| {
            let yaw_f = 1.0 - 0.08 * (yaw / 10.0).powi(2);
            AeroCoeffs {
                cz_front_a_m2: 1.9 * yaw_f,
                cz_rear_a_m2: 2.6 * yaw_f,
                cx_a_m2: 1.25,
            }
        });
        let a_flat = p.equilibrium(&flat, 70.0, 0.0, 6.0, 0.0);
        let a_yaw = p.equilibrium(&yawed, 70.0, 0.0, 6.0, 0.0);
        // Flat map: yaw does nothing. Yaw-sensitive map: downforce drops at |yaw|>0.
        assert!((a_flat.qz_f - 0.5 * p.rho * 1.9).abs() < 1e-12);
        assert!(
            a_yaw.qz_f < a_flat.qz_f,
            "yaw sensitivity must cut downforce"
        );
        assert!(a_yaw.qz_r < a_flat.qz_r);
        // At zero yaw the sensitive map matches the flat one.
        let a_yaw0 = p.equilibrium(&yawed, 70.0, 0.0, 0.0, 0.0);
        assert!((a_yaw0.qz_f - a_flat.qz_f).abs() < 1e-12);
    }

    #[test]
    fn drs_open_cuts_rear_downforce_and_drag() {
        let map = build_map(|_hf, _hr, _yaw, drs| {
            let (rear_f, drag_f) = if drs > 0.5 { (0.7, 0.82) } else { (1.0, 1.0) };
            AeroCoeffs {
                cz_front_a_m2: 1.9,
                cz_rear_a_m2: 2.6 * rear_f,
                cx_a_m2: 1.25 * drag_f,
            }
        });
        let closed = map.eval(30.0, 70.0, 0.0, 0.0);
        let open = map.eval(30.0, 70.0, 0.0, 1.0);
        assert!((open.cz_front_a_m2 - closed.cz_front_a_m2).abs() < 1e-12);
        assert!(open.cz_rear_a_m2 < closed.cz_rear_a_m2);
        assert!(open.cx_a_m2 < closed.cx_a_m2);
    }
}
