// SPDX-License-Identifier: AGPL-3.0-only
//! The **T2 tire-thermal slow-state subsystem** (M5 PR3): the per-wheel Farroni-TRT ring + wear law
//! (`outlap_tire::TireThermalRing`) advanced on the decimated slow clock, feeding its grip window +
//! gas-law pressure back into the per-step tyre force call.
//!
//! This is the third hand-rolled slow subsystem (D-M5-1), stepped exactly like the battery pack: the
//! solver accumulates each fast step's frictional + carcass heat into per-wheel energy counters, and
//! on a slow-clock fire the ring advances one step over the window and refreshes the per-wheel grip
//! multiplier `λ_μ,total` and hot inflation pressure `p`. Those held values ([`Tire::thermal`]) drive
//! every intervening fast step, so the ring's single, decimated step never touches the hot RK path
//! (the battery / shift-FSM idiom).
//!
//! # Driver formation (§7.2 exogenous inputs, from the T2 force solution)
//!
//! The ring is a pure function of `(state, params, drivers, dt)`; this module forms the §7.2 drivers
//! from the fast-loop force solution each window:
//!
//! - **`Q_fric` (surface heat)** — the frictional sliding power `P_slide = |F_x·V_sx| + |F_y·V_sy|`,
//!   accumulated per fast step (`Tire::wheel_slip_powers`) into a per-wheel energy and averaged over
//!   the window, so the heat the ring deposits closes to the frictional energy the patch actually
//!   dissipated (energy closure across the slow-clock window).
//! - **`Q_hyst` (carcass heat)** — the rolling-deformation loss `Q_hyst = c_h·F_z·δ·Ω` (§7.2), with
//!   the tyre deflection `δ = F_z/k_z` (vertical stiffness) and `Ω = v/R` the wheel spin rate. This is
//!   the standard load-squared rolling-hysteresis power; `c_h` is a documented modelling constant
//!   ([`HYSTERESIS_LOSS_FACTOR`], the fraction of deformation energy dissipated as heat per
//!   revolution). Also accumulated as energy over the window.
//! - **contact fraction `a_cp = A_cp/A_ext`** — the patch area `A_cp = F_z/p` (load over inflation
//!   pressure, the standard contact-patch estimate) over the external tread area `A_ext = 2π·R·W`;
//!   sampled at the window boundary. The patch conducts to the road, the rest of the tread convects.
//! - **`speed`, `T_air`, `T_road`** — the road speed sets the forced-convection conductance; the air
//!   and track-surface temperatures (`conditions.yaml`) are the convection/conduction boundaries.
//!
//! # Seeding (parity-safe default)
//!
//! A default T2 lap seeds every node **warm at the grip optimum** `T_s = T_c = T_opt` with the gas at
//! the cold reference `T_g = T_cold` and zero wear/damage. At that state `λ_μ(T_opt) = 1` exactly and
//! `p = p_cold·T_cold/T_cold = p_cold` exactly, so the wired ring reproduces the frozen-tire forces
//! **bit-for-bit at the first step** — the QSS↔T2 hull-containment parity gate stays valid — and then
//! drifts physically as the surface leaves the window under load and the gas heats. A cold seed
//! (`seed_uniform`) reproduces the warm-up transient for the property tests.

use num_traits::Float;

use outlap_core::bus::WHEELS;
use outlap_schema::tyr::{TyrThermal, TyrWear};
use outlap_tire::{ThermalDrivers, TireThermalRing, TireThermalState};
use outlap_vehicle::ThermalGrip;

/// Zero of the Celsius scale, in kelvin (schema temperatures are °C; the ring state is kelvin).
const CELSIUS_K: f64 = 273.15;

/// Rolling-hysteresis loss factor `c_h` in `Q_hyst = c_h·F_z·δ·Ω` (§7.2) — the dimensionless fraction
/// of the per-revolution tyre deformation energy dissipated as carcass heat (rubber loss angle,
/// Grosch/§7.2 range ≈ 0.05–0.15). A modelling constant, not a `.tyr` field: the thermal block's
/// `g_cg`/`c_c` set the carcass balance the calibration (PR7) tunes; this sets only the load↔heat
/// scale, documented in `docs/theory/tire-thermal.md`.
const HYSTERESIS_LOSS_FACTOR: f64 = 0.10;

/// Fallback tyre vertical stiffness `k_z`, N/m, when a `.tyr` omits `VERTICAL_STIFFNESS` (a
/// racing-slick-representative ~250 kN/m). Only sets the carcass-heat deflection scale, which
/// calibration absorbs.
const VERTICAL_STIFFNESS_FALLBACK: f64 = 250_000.0;

/// Fallback tread width `W`, m, when a `.tyr` omits `WIDTH` (a wide-slick default). Sets the external
/// tread area `A_ext = 2π·R·W`, i.e. the convection/contact-fraction geometry scale.
const WIDTH_FALLBACK: f64 = 0.30;

/// Smallest window (s) the ring will average power over (guards the first, empty flush).
const WINDOW_FLOOR_S: f64 = 1e-9;

/// Per-axle tyre geometry the driver formation needs but the thermal block does not carry: the
/// external convecting tread area and the deflection/spin geometry for the carcass-heat driver.
#[derive(Clone, Copy, Debug)]
pub struct AxleGeometry {
    /// External (convecting) tread area `A_ext = 2π·R·W`, m².
    pub ext_area_m2: f64,
    /// Vertical stiffness `k_z`, N/m (for the deflection `δ = F_z/k_z`).
    pub k_vertical_n_per_m: f64,
    /// Loaded rolling radius `R`, m (for the spin rate `Ω = v/R`).
    pub radius_m: f64,
}

impl AxleGeometry {
    /// Build from the tyre's radius, width, and vertical stiffness, applying the documented fallbacks
    /// for the two optional coefficients. `radius_m` is the required `UNLOADED_RADIUS`.
    #[must_use]
    pub fn new(radius_m: f64, width_m: Option<f64>, k_vertical: Option<f64>) -> Self {
        let w = width_m.unwrap_or(WIDTH_FALLBACK).max(1e-3);
        let r = radius_m.max(1e-3);
        Self {
            ext_area_m2: std::f64::consts::TAU * r * w,
            k_vertical_n_per_m: k_vertical.unwrap_or(VERTICAL_STIFFNESS_FALLBACK).max(1.0),
            radius_m: r,
        }
    }
}

/// Which axle a wheel belongs to (`[FL, FR, RL, RR]` ⇒ front, front, rear, rear).
#[inline]
fn axle_of(wheel: usize) -> usize {
    usize::from(wheel >= 2)
}

/// The per-wheel tire-thermal ring + wear stack the T2 solver advances on the slow clock.
///
/// Holds the two per-axle ring models (front/rear tyres differ in compound + pressure), the four
/// integrated states, the per-wheel heat accumulators, and the environment boundaries. Cloneable and
/// allocation-free per step.
#[derive(Clone, Debug)]
pub struct TireThermalStack<T> {
    /// Per-axle ring parameters (`[front, rear]`).
    rings: [TireThermalRing<T>; 2],
    /// Per-axle geometry (`[front, rear]`).
    geom: [AxleGeometry; 2],
    /// Per-wheel integrated state `[FL, FR, RL, RR]`.
    state: [TireThermalState<T>; WHEELS],
    /// Per-wheel frictional (surface) heat energy accumulated this window, J.
    slip_energy_j: [T; WHEELS],
    /// Per-wheel carcass (hysteresis) heat energy accumulated this window, J.
    carcass_energy_j: [T; WHEELS],
    /// Per-wheel latest normal load (for the window-boundary contact fraction), N.
    fz_last_n: [T; WHEELS],
    /// Window length accumulated since the last [`advance`](Self::advance), s.
    window_s: T,
    /// Ambient air temperature `T_air`, K.
    t_air_k: T,
    /// Road-surface temperature `T_road`, K.
    t_road_k: T,
    /// Rolling-hysteresis loss factor `c_h` (typed once).
    hysteresis_factor: T,
}

impl<T: Float> TireThermalStack<T> {
    /// Build a stack from the per-axle thermal + wear blocks, per-axle geometry, and the session
    /// air / track-surface temperatures (°C). The four states are seeded **warm** at the grip optimum
    /// with the gas at the cold reference (the parity-safe default; see the module docs).
    #[must_use]
    #[allow(clippy::too_many_arguments)] // a linear per-axle constructor (thermal+wear+geometry ×2).
    pub fn new(
        front_thermal: &TyrThermal,
        front_wear: &TyrWear,
        rear_thermal: &TyrThermal,
        rear_wear: &TyrWear,
        front_geom: AxleGeometry,
        rear_geom: AxleGeometry,
        air_temp_c: f64,
        track_surface_c: f64,
    ) -> Self {
        let rings = [
            TireThermalRing::from_schema_with_wear(front_thermal, front_wear),
            TireThermalRing::from_schema_with_wear(rear_thermal, rear_wear),
        ];
        let cvt = |x: f64| T::from(x).unwrap_or_else(T::zero);
        let mut stack = Self {
            rings,
            geom: [front_geom, rear_geom],
            state: [TireThermalState::uniform(T::zero()); WHEELS],
            slip_energy_j: [T::zero(); WHEELS],
            carcass_energy_j: [T::zero(); WHEELS],
            fz_last_n: [T::zero(); WHEELS],
            window_s: T::zero(),
            t_air_k: cvt(air_temp_c + CELSIUS_K),
            t_road_k: cvt(track_surface_c + CELSIUS_K),
            hysteresis_factor: cvt(HYSTERESIS_LOSS_FACTOR),
        };
        // Warm, parity-safe seed: T_s = T_c = T_opt (grip window = 1), T_g = T_cold (p = p_cold).
        for wheel in 0..WHEELS {
            let axle = axle_of(wheel);
            let (t_opt_c, t_cold_c) = if axle == 0 {
                (front_thermal.t_opt, front_thermal.t_cold)
            } else {
                (rear_thermal.t_opt, rear_thermal.t_cold)
            };
            stack.state[wheel] = TireThermalState::new(
                cvt(t_opt_c + CELSIUS_K),
                cvt(t_opt_c + CELSIUS_K),
                cvt(t_cold_c + CELSIUS_K),
            );
        }
        stack
    }

    /// Overwrite every wheel's state with a uniform cold start at `temp_c` (°C), zero wear/damage —
    /// the warm-up transient the property tests exercise.
    pub fn seed_uniform(&mut self, temp_c: f64) {
        let t_k = T::from(temp_c + CELSIUS_K).unwrap_or_else(T::zero);
        for st in &mut self.state {
            *st = TireThermalState::uniform(t_k);
        }
        self.reset_accumulators();
    }

    /// Seed one wheel's full state explicitly (temps in °C, wear mm, damage 0..1) — stint continuity
    /// (carry lap N's terminal state into lap N+1) and partly-worn test setups.
    pub fn seed_wheel(
        &mut self,
        wheel: usize,
        t_s_c: f64,
        t_c_c: f64,
        t_g_c: f64,
        wear_mm: f64,
        damage: f64,
    ) {
        let cvt = |x: f64| T::from(x).unwrap_or_else(T::zero);
        self.state[wheel] = TireThermalState::with_wear(
            cvt(t_s_c + CELSIUS_K),
            cvt(t_c_c + CELSIUS_K),
            cvt(t_g_c + CELSIUS_K),
            cvt(wear_mm),
            cvt(damage),
        );
    }

    fn reset_accumulators(&mut self) {
        self.slip_energy_j = [T::zero(); WHEELS];
        self.carcass_energy_j = [T::zero(); WHEELS];
        self.window_s = T::zero();
    }

    /// The per-wheel carcass hysteresis power `Q_hyst = c_h·F_z·(F_z/k_z)·(v/R)`, W (§7.2), at load
    /// `fz` and wheel spin `omega` on the wheel's axle geometry. Non-negative.
    #[inline]
    fn carcass_loss_w(&self, wheel: usize, fz: T, omega: T) -> T {
        let g = &self.geom[axle_of(wheel)];
        let k_z = T::from(g.k_vertical_n_per_m).unwrap_or_else(T::one);
        let fz = fz.max(T::zero());
        let deflection = fz / k_z;
        self.hysteresis_factor * fz * deflection * omega.abs()
    }

    /// Accumulate one fast step's per-wheel heat into the window energies (mirrors the battery's
    /// net-charge energy count). `slip_power` is `Tire::wheel_slip_powers` (surface heat); the carcass
    /// heat is formed here from the load `fz` and wheel spin `omega`. `fz` is also stashed for the
    /// window-boundary contact fraction. Allocation-free.
    pub fn accumulate(
        &mut self,
        slip_power: &[T; WHEELS],
        fz: &[T; WHEELS],
        omega: &[T; WHEELS],
        dt: T,
    ) {
        for wheel in 0..WHEELS {
            self.slip_energy_j[wheel] =
                self.slip_energy_j[wheel] + slip_power[wheel].max(T::zero()) * dt;
            let q_hyst = self.carcass_loss_w(wheel, fz[wheel], omega[wheel]);
            self.carcass_energy_j[wheel] = self.carcass_energy_j[wheel] + q_hyst * dt;
            self.fz_last_n[wheel] = fz[wheel];
        }
        self.window_s = self.window_s + dt;
    }

    /// Advance every wheel's ring by the accumulated window under the window-averaged heat, and return
    /// the refreshed per-wheel grip + pressure override for the `Tire` force block. Resets the
    /// accumulators. Returns `None` when no time has accumulated (nothing to flush).
    ///
    /// `speed_mps` is the current road speed (forced-convection driver, ~constant over the ~20 ms
    /// window). The contact fraction uses the ring's own current hot pressure, so it tracks the
    /// gas-law feedback.
    #[must_use]
    pub fn advance(&mut self, speed_mps: T) -> Option<ThermalGrip<T>> {
        let floor = T::from(WINDOW_FLOOR_S).unwrap_or_else(T::zero);
        if self.window_s <= floor {
            return None;
        }
        let window = self.window_s;
        let mut grip = ThermalGrip {
            mu_x: [T::one(); WHEELS],
            mu_y: [T::one(); WHEELS],
            p: [T::zero(); WHEELS],
        };
        for wheel in 0..WHEELS {
            let axle = axle_of(wheel);
            let ring = &self.rings[axle];
            let g = &self.geom[axle];
            let ext_area = T::from(g.ext_area_m2).unwrap_or_else(T::one);
            let avg_slip = self.slip_energy_j[wheel] / window;
            let avg_carcass = self.carcass_energy_j[wheel] / window;
            // Contact patch A_cp = Fz/p over the external tread area, using the ring's current hot
            // pressure (the gas-law feedback). Clamp the fraction to [0, 1].
            let pressure = ring.pressure_pa(&self.state[wheel]).max(T::one());
            let contact_fraction = (self.fz_last_n[wheel].max(T::zero()) / (pressure * ext_area))
                .max(T::zero())
                .min(T::one());
            let drivers = ThermalDrivers {
                slip_power_w: avg_slip,
                carcass_loss_w: avg_carcass,
                speed_mps: speed_mps.max(T::zero()),
                contact_fraction,
                ext_area_m2: ext_area,
                t_air_k: self.t_air_k,
                t_road_k: self.t_road_k,
            };
            let couplings = ring.step(&mut self.state[wheel], &drivers, window);
            grip.mu_x[wheel] = couplings.mu_scale_total;
            grip.mu_y[wheel] = couplings.mu_scale_total;
            grip.p[wheel] = couplings.pressure_pa;
        }
        self.reset_accumulators();
        Some(grip)
    }

    /// The current grip + pressure couplings at the held state, without advancing — used to install
    /// the initial (seeded) override before the first step so the seeded forces already carry the
    /// warm-tire grip/pressure (bit-identical to the frozen path at the parity-safe seed).
    #[must_use]
    pub fn current_grip(&self) -> ThermalGrip<T> {
        let mut grip = ThermalGrip {
            mu_x: [T::one(); WHEELS],
            mu_y: [T::one(); WHEELS],
            p: [T::zero(); WHEELS],
        };
        for wheel in 0..WHEELS {
            let c = self.rings[axle_of(wheel)].couplings(&self.state[wheel]);
            grip.mu_x[wheel] = c.mu_scale_total;
            grip.mu_y[wheel] = c.mu_scale_total;
            grip.p[wheel] = c.pressure_pa;
        }
        grip
    }

    /// A read-only view of one wheel's state (K), for the result surface / telemetry.
    #[must_use]
    pub fn state(&self, wheel: usize) -> &TireThermalState<T> {
        &self.state[wheel]
    }

    /// One wheel's total grip multiplier `λ_μ,total` at the held state (telemetry).
    #[must_use]
    pub fn grip(&self, wheel: usize) -> T {
        self.rings[axle_of(wheel)]
            .couplings(&self.state[wheel])
            .mu_scale_total
    }
}
