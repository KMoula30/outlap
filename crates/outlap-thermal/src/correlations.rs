// SPDX-License-Identifier: AGPL-3.0-only
//! Heat-transfer correlations and fluid-property lookups for the machine LPTN.
//!
//! All temperatures are in kelvin, all lengths in metres, SI throughout. Each correlation is a
//! standard published form; the citation is on the function. These are the temperature/speed
//! dependence behind the detailed (imported) conductance graph — the constant conduction/contact
//! skeleton needs none of them.
//!
//! References:
//! * Becker, K. M. & Kaye, J. (1962), *J. Heat Transfer* 84(2) — Taylor-number air-gap regimes.
//! * Kylander, G. (1995), doctoral thesis, Chalmers — end-winding / internal-air convection.
//! * Etemad, G. A. (1955), *Trans. ASME* 77 — rotating-shaft external convection.
//! * Churchill, S. W. & Chu, H. H. S. (1975), *Int. J. Heat Mass Transfer* 18 — free convection.
//! * Gnielinski, V. (1976), *Int. Chem. Eng.* 16 — turbulent pipe flow.
//! * Staton, D. & Cavagnino, A. (2008), *IEEE Trans. Ind. Electron.* 55(10) — TEFC channels.

/// Stefan–Boltzmann constant, W/(m²·K⁴).
pub const STEFAN_BOLTZMANN: f64 = 5.670_374_419e-8;
/// Standard atmospheric pressure, Pa.
pub const P_ATM: f64 = 101_325.0;
/// Specific gas constant for dry air, J/(kg·K).
pub const R_AIR: f64 = 287.05;
/// Gravitational acceleration, m/s².
const G_ACCEL: f64 = 9.81;

/// Air properties at a temperature, evaluated for the film of a convective interface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirProps {
    /// Thermal conductivity λ, W/(m·K).
    pub lam: f64,
    /// Dynamic viscosity μ, Pa·s.
    pub mu: f64,
    /// Kinematic viscosity ν, m²/s.
    pub nu: f64,
    /// Density ρ, kg/m³.
    pub rho: f64,
    /// Specific heat `c_p`, J/(kg·K).
    pub cp: f64,
    /// Prandtl number, dimensionless.
    pub pr: f64,
    /// Volumetric thermal expansion β, 1/K.
    pub beta: f64,
}

/// Air properties at absolute temperature `t_k` (kelvin) at 1 atm.
///
/// Polynomial/power fits valid over roughly 250–500 K; ideal-gas density and `β = 1/T`.
#[must_use]
pub fn air_properties(t_k: f64) -> AirProps {
    let lam = 0.024_16 + 7.7e-5 * (t_k - 273.15);
    let mu = 1.716e-5 * (t_k / 273.15).powf(0.7);
    let rho = P_ATM / (R_AIR * t_k);
    let nu = mu / rho;
    let cp = 1006.0;
    let pr = cp * mu / lam;
    let beta = 1.0 / t_k;
    AirProps {
        lam,
        mu,
        nu,
        rho,
        cp,
        pr,
        beta,
    }
}

/// Properties of a liquid coolant at its film temperature (supplied — not recomputed here).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FluidProps {
    /// Thermal conductivity λ, W/(m·K).
    pub lam: f64,
    /// Kinematic viscosity ν, m²/s.
    pub nu: f64,
    /// Prandtl number, dimensionless.
    pub pr: f64,
}

/// Result of the air-gap film correlation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirGapFilm {
    /// Effective (Nu-enhanced) conductivity across the gap, W/(m·K).
    pub lam_eff: f64,
    /// Hot air-gap radial thickness δ, m (after rotor thermal expansion).
    pub delta: f64,
    /// Modified Taylor number, dimensionless.
    pub ta_m: f64,
    /// Air-gap Nusselt number, dimensionless.
    pub nu: f64,
}

/// Air-gap heat transfer with a rotor thermal-expansion correction (Becker–Kaye modified-Taylor
/// regimes). `omega` rotor speed (rad/s), `r_gap` mean gap radius (m), `gap0` cold radial gap (m).
/// `kappa_fe` is the iron linear-expansion coefficient (1/K).
#[must_use]
pub fn airgap_film(
    omega: f64,
    r_gap: f64,
    gap0: f64,
    t_rotor_k: f64,
    t_stator_k: f64,
    t_amb_k: f64,
    kappa_fe: f64,
) -> AirGapFilm {
    let delta = (gap0 - kappa_fe * r_gap * (t_rotor_k - t_amb_k)).max(1e-6);
    let t_film = 0.5 * (t_rotor_k + t_stator_k);
    let p = air_properties(t_film);
    let ta_m = omega.powi(2) * r_gap * delta.powi(3) / p.nu.powi(2);
    let nu = if ta_m < 1700.0 {
        2.0
    } else if ta_m < 1.0e4 {
        0.128 * ta_m.powf(0.367)
    } else {
        0.409 * ta_m.powf(0.241)
    };
    AirGapFilm {
        lam_eff: nu * p.lam,
        delta,
        ta_m,
        nu,
    }
}

/// The default iron linear thermal-expansion coefficient, 1/K (electrical steel).
pub const KAPPA_FE: f64 = 10.4e-6;

/// End-winding to internal-cavity-air heat-transfer coefficient, W/(m²·K) (Kylander).
/// `u_rotor` is the rotor peripheral speed, m/s.
#[must_use]
pub fn endwinding_h(u_rotor: f64) -> f64 {
    6.5 + 5.25 * u_rotor.max(0.0).powf(0.6)
}

/// Internal cavity air to housing-inner heat-transfer coefficient, W/(m²·K) (Kylander).
/// `u_rotor` is the rotor peripheral speed, m/s.
#[must_use]
pub fn internal_air_h(u_rotor: f64) -> f64 {
    15.0 + 6.75 * u_rotor.max(0.0).powf(0.65)
}

/// Rotating-shaft to ambient-air heat-transfer coefficient, W/(m²·K) (Etemad).
/// Below the correlation's Reynolds range it returns a free-convection baseline.
#[must_use]
pub fn shaft_external_h(omega: f64, d_shaft: f64, t_air_k: f64) -> f64 {
    let u_shaft = omega.abs() * d_shaft / 2.0;
    let p = air_properties(t_air_k);
    let re_d = u_shaft * d_shaft / p.nu;
    if re_d < 1.0 {
        return 5.0;
    }
    let nu_d = 0.076 * re_d.powf(0.7);
    nu_d * p.lam / d_shaft
}

/// Cylinder orientation for the Churchill–Chu free-convection correlation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Orientation {
    /// Horizontal cylinder (characteristic length = diameter).
    Horizontal,
    /// Vertical cylinder (characteristic length = axial length).
    Vertical,
}

/// Free-convection coefficient on a horizontal or vertical cylinder, W/(m²·K) (Churchill–Chu 1975).
/// Returns 0 when the wall is at or below ambient (no buoyant drive).
#[must_use]
pub fn churchill_chu_h(
    t_wall_k: f64,
    t_amb_k: f64,
    char_length: f64,
    orientation: Orientation,
) -> f64 {
    if t_wall_k <= t_amb_k + 1e-6 {
        return 0.0;
    }
    let t_film = 0.5 * (t_wall_k + t_amb_k);
    let p = air_properties(t_film);
    let ra = G_ACCEL * p.beta * (t_wall_k - t_amb_k) * char_length.powi(3) / p.nu.powi(2) * p.pr;
    let ra16 = ra.powf(1.0 / 6.0);
    let nu = match orientation {
        Orientation::Horizontal => {
            let d = (1.0 + (0.559 / p.pr).powf(9.0 / 16.0)).powf(8.0 / 27.0);
            (0.60 + 0.387 * ra16 / d).powi(2)
        }
        Orientation::Vertical => {
            let d = (1.0 + (0.492 / p.pr).powf(9.0 / 16.0)).powf(8.0 / 27.0);
            (0.825 + 0.387 * ra16 / d).powi(2)
        }
    };
    nu * p.lam / char_length
}

/// Linearized radiation coefficient to ambient, W/(m²·K), such that `q = h·(T_wall − T_amb)`.
#[must_use]
pub fn radiation_h(t_wall_k: f64, t_amb_k: f64, emissivity: f64) -> f64 {
    emissivity * STEFAN_BOLTZMANN * (t_wall_k.powi(2) + t_amb_k.powi(2)) * (t_wall_k + t_amb_k)
}

/// Average turbulent forced-convection coefficient over a flat plate, W/(m²·K).
/// `velocity` free-stream speed (m/s), `char_length` plate length in the flow direction (m).
#[must_use]
pub fn forced_flat_plate_h(velocity: f64, char_length: f64, t_film_k: f64) -> f64 {
    if velocity <= 0.0 {
        return 0.0;
    }
    let p = air_properties(t_film_k);
    let re = velocity * char_length / p.nu;
    let nu = 0.037 * re.powf(0.8) * p.pr.powf(1.0 / 3.0);
    nu * p.lam / char_length
}

/// Straight-fin efficiency `η = tanh(mL)/(mL)`, dimensionless in (0, 1].
#[must_use]
pub fn fin_efficiency(h_base: f64, lambda_fin: f64, t_fin: f64, fin_height: f64) -> f64 {
    if h_base <= 0.0 || fin_height <= 0.0 {
        return 1.0;
    }
    let m = (2.0 * h_base / (lambda_fin * t_fin)).sqrt();
    let ml = m * fin_height;
    if ml < 1e-6 {
        return 1.0;
    }
    ml.tanh() / ml
}

/// Heiles closed-form coefficient for fan-cooled semi-open axial fin channels (TEFC), W/(m²·K).
/// The 1.7 turbulence factor of Staton & Cavagnino is folded in.
#[must_use]
pub fn heiles_h(velocity: f64, hydraulic_diameter: f64, channel_length: f64, t_film_k: f64) -> f64 {
    if velocity <= 0.0 || channel_length <= 0.0 || hydraulic_diameter <= 0.0 {
        return 0.0;
    }
    let p = air_properties(t_film_k);
    let m = 0.1448 * channel_length.powf(0.946) / hydraulic_diameter.powf(1.16)
        * (p.lam / (p.rho * p.cp * velocity)).powf(0.214);
    let h =
        p.rho * p.cp * hydraulic_diameter * velocity / (4.0 * channel_length) * (1.0 - (-m).exp());
    1.7 * h
}

/// Gnielinski Nusselt number for fully-developed turbulent pipe flow (valid 3000 < Re < 5e6,
/// 0.5 < Pr < 2000).
#[must_use]
pub fn gnielinski_nu(re: f64, pr: f64) -> f64 {
    let f = (0.79 * re.ln() - 1.64).powi(-2);
    (f / 8.0) * (re - 1000.0) * pr / (1.0 + 12.7 * (f / 8.0).sqrt() * (pr.powf(2.0 / 3.0) - 1.0))
}

/// Heat-transfer coefficient inside a liquid-cooled channel, W/(m²·K). Laminar `Nu = 4.36` below
/// `Re = 2300`, Gnielinski above `Re = 3000`, linearly blended between for numerical continuity.
#[must_use]
pub fn channel_h(velocity: f64, hydraulic_diameter: f64, fluid: FluidProps) -> f64 {
    if velocity <= 0.0 || hydraulic_diameter <= 0.0 {
        return 0.0;
    }
    let re = velocity * hydraulic_diameter / fluid.nu;
    let nu_lam = 4.36;
    let nu = if re < 2300.0 {
        nu_lam
    } else if re > 3000.0 {
        gnielinski_nu(re, fluid.pr)
    } else {
        let w = (re - 2300.0) / 700.0;
        (1.0 - w) * nu_lam + w * gnielinski_nu(3000.0, fluid.pr)
    };
    nu * fluid.lam / hydraulic_diameter
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, rel: f64) -> bool {
        (a - b).abs() <= rel * b.abs().max(1e-9)
    }

    #[test]
    fn air_properties_at_300k() {
        let p = air_properties(300.0);
        // Sanity bands for air near room temperature.
        assert!(close(p.lam, 0.0263, 0.1), "lam={}", p.lam);
        assert!(close(p.rho, 1.177, 0.05), "rho={}", p.rho);
        assert!((p.pr - 0.7).abs() < 0.05, "pr={}", p.pr);
        assert!(p.beta > 0.0 && p.nu > 0.0);
    }

    #[test]
    fn airgap_film_regimes_are_monotone_in_speed() {
        // Higher speed ⇒ higher Taylor number ⇒ more air-gap mixing (never below molecular Nu=2).
        let f0 = airgap_film(0.0, 0.05, 5e-4, 350.0, 340.0, 300.0, KAPPA_FE);
        let f1 = airgap_film(500.0, 0.05, 5e-4, 350.0, 340.0, 300.0, KAPPA_FE);
        assert!(f0.nu >= 2.0 - 1e-12);
        assert!(f1.nu >= f0.nu);
        assert!(f1.lam_eff >= f0.lam_eff);
    }

    #[test]
    fn convection_coeffs_increase_with_speed() {
        assert!(endwinding_h(20.0) > endwinding_h(0.0));
        assert!(internal_air_h(20.0) > internal_air_h(0.0));
        assert!(shaft_external_h(500.0, 0.04, 300.0) > shaft_external_h(1.0, 0.04, 300.0));
    }

    #[test]
    fn churchill_chu_zero_below_ambient_positive_above() {
        assert_eq!(
            churchill_chu_h(300.0, 320.0, 0.2, Orientation::Horizontal),
            0.0
        );
        assert!(churchill_chu_h(360.0, 300.0, 0.2, Orientation::Horizontal) > 0.0);
    }

    #[test]
    fn channel_h_turbulent_exceeds_laminar_floor() {
        let fp = FluidProps {
            lam: 0.401,
            nu: 1.74e-6,
            pr: 15.6,
        };
        let d_h = 8.89e-3;
        let lam_floor = 4.36 * fp.lam / d_h;
        // A brisk coolant velocity is turbulent and beats the laminar floor.
        assert!(channel_h(3.0, d_h, fp) > lam_floor);
        // Zero flow ⇒ no convective coefficient.
        assert_eq!(channel_h(0.0, d_h, fp), 0.0);
    }

    #[test]
    fn gnielinski_positive_in_range() {
        assert!(gnielinski_nu(1.0e4, 15.6) > 0.0);
    }

    #[test]
    fn radiation_h_grows_with_temperature() {
        assert!(radiation_h(400.0, 300.0, 0.3) > radiation_h(320.0, 300.0, 0.3));
    }
}
