// SPDX-License-Identifier: AGPL-3.0-only
//! The **QSS tire-thermal slow-state march** (M5 PR5): the §7.2 reduced Farroni-TRT ring + §7.3
//! Archard wear ([`outlap_tire::TireThermalRing`]) advanced **segment-to-segment** along the
//! quasi-static velocity profile (§6.1 explicit Euler over the QSS solution), producing the per-station
//! `(T_tire, wear)` the [`GgvEnvelope`](crate::t1::GgvEnvelope) tyre-state axes index. This is what
//! makes the T0/T1 tier **stint-capable**.
//!
//! # Where this differs from the T2 march
//!
//! The T2 transient tier ([`outlap-transient`]'s `TireThermalStack`) carries a **per-wheel** ring +
//! wear state, accumulating each fast RK step's frictional/carcass energy into a window and flushing
//! it on the decimated slow clock. The QSS tier has no fast loop and no per-wheel slip solution — the
//! velocity profile is a point-mass sweep — so this march:
//!
//! 1. Advances a **single representative tyre** state (the front-tyre ring, the same
//!    [`T1Vehicle::tire_thermal`](crate::t1::T1Vehicle::tire_thermal) the envelope's tyre-state axes
//!    are built from), because the envelope's `T_tire` / `wear` axes are **scalar** — one representative
//!    state indexes them. (Per-wheel resolution is the T2 tier's differentiator; the *tyre-state axes*
//!    are this tier's, per §6.1.)
//! 2. Forms the §7.2 drivers from the **point-mass forces at each station** (no slip solution): the
//!    normal load `N = m·g_normal + q_z·v²`, the tyre force `F = √(F_x² + F_y²)` demanded to hold the
//!    station's `(a_x, a_y)`, and a **reduced-slip closure** for the frictional sliding power (below).
//! 3. Steps the ring **once per segment** over the segment time `dt = 2·ds/(v_i + v_{i+1})` — the ring's
//!    temperatures are semi-implicit (A-stable, so a coarse segment step cannot ring), wear/damage are
//!    monotone-clamped explicit Euler, so the coarse QSS step is safe.
//!
//! # Reduced-slip frictional-power closure (§7.2 `Q_fric`, QSS form)
//!
//! `Q_fric = p_t·(|F_x·V_sx| + |F_y·V_sy|)` needs the contact-patch **sliding velocities** `V_s`, which
//! the T2 tier reads from the tyre model but the point-mass QSS solve does not resolve. We close it with
//! the standard reduced form `V_s = κ_ref·v·ρ`, where `ρ = F/F_cap ∈ [0, 1]` is the friction-circle
//! **utilisation** (tyre force over the envelope's local grip capacity `F_cap = m·a_y,boundary`) and
//! `κ_ref` ([`SLIP_REFERENCE`]) is a reference slip at the grip limit. The frictional power then reads
//! `P_slide = F·V_s = κ_ref·v·F²/F_cap` — rising with speed, force, and utilisation, zero when coasting
//! straight, and closing to the mechanical work the patch must dissipate. `κ_ref` is a documented
//! modelling constant (like the carcass [`HYSTERESIS_LOSS_FACTOR`]); the absolute heat magnitude is set
//! by the FastF1 inverse calibration (M5 PR7/PR8).
//!
//! # Seeding (parity-safe default)
//!
//! The march seeds **warm at the grip optimum** `T_s = T_c = T_opt`, gas at the cold reference
//! `T_g = T_cold`, zero wear/damage — exactly the T2 parity-safe seed. At that state
//! `λ_μ(T_opt) = 1` and the envelope's reference `(T_opt, wear = 0)` slice is bit-identical to the
//! frozen-tyre envelope (the PR4 invariant), so a march that never leaves the reference reproduces the
//! pre-M5 lap. Under load the surface drifts off the window and wear accumulates, and the re-solve sees
//! the degradation. A cold seed ([`TireThermalMarch::with_seed`]) reproduces the warm-up transient.

use outlap_tire::{ThermalDrivers, TireThermalRing, TireThermalState};

use crate::path::T0Path;
use crate::solver::demand_and_gn;
use crate::t1::GgvEnvelope;
use crate::vehicle::T0Vehicle;
use crate::G;

/// Zero of the Celsius scale, in kelvin (schema temperatures are °C; the ring state is kelvin).
const CELSIUS_K: f64 = 273.15;

/// Reference contact-patch slip at the grip limit `κ_ref` in the QSS frictional-power closure
/// `V_s = κ_ref·v·ρ` (§7.2, QSS form). A dimensionless modelling constant — the slip fraction a fully
/// utilised tyre runs at — mirroring the T2 [`HYSTERESIS_LOSS_FACTOR`]; the absolute frictional-heat
/// scale is set by the FastF1 inverse calibration (M5 PR7/PR8). Documented in
/// `docs/theory/tire-thermal.md`.
const SLIP_REFERENCE: f64 = 0.08;

/// Rolling-hysteresis loss factor `c_h` in `Q_hyst = c_h·F_z·δ·Ω` (§7.2) — the fraction of the
/// per-revolution tyre deformation energy dissipated as carcass heat. The same modelling constant the
/// T2 tier uses (`outlap-transient`'s `HYSTERESIS_LOSS_FACTOR`), kept in sync so both tiers deposit the
/// same carcass heat for a given load/speed.
const HYSTERESIS_LOSS_FACTOR: f64 = 0.10;

/// Fallback tyre vertical stiffness `k_z`, N/m, when a `.tyr` omits `VERTICAL_STIFFNESS`
/// (racing-slick-representative ~250 kN/m). Matches the T2 fallback.
const VERTICAL_STIFFNESS_FALLBACK: f64 = 250_000.0;

/// Fallback tread width `W`, m, when a `.tyr` omits `WIDTH` (a wide-slick default). Matches the T2
/// fallback. Sets the external tread area `A_ext = 2π·R·W`.
const WIDTH_FALLBACK: f64 = 0.30;

/// Fallback loaded rolling radius `R`, m, when a `.tyr` omits `UNLOADED_RADIUS`.
const RADIUS_FALLBACK: f64 = 0.33;

/// Number of tyres the point-mass normal load is shared across for the representative-tyre drivers.
const N_TYRES: f64 = 4.0;

/// Per-station tyre slow-state channels the march surfaces (the single representative front tyre;
/// present only when a [`TireThermalMarch`] was supplied). Temperatures in **°C**, wear in **mm**.
#[derive(Clone, Debug)]
pub struct TireSlowLog {
    /// Tread-surface temperature `T_s`, °C (the grip-window driver + the envelope `T_tire` axis).
    pub surface_temp_c: Vec<f64>,
    /// Tread-bulk / carcass temperature `T_c`, °C.
    pub carcass_temp_c: Vec<f64>,
    /// Inflation-gas temperature `T_g`, °C.
    pub gas_temp_c: Vec<f64>,
    /// Tread wear depth `w`, mm (monotone non-decreasing over a lap).
    pub wear_mm: Vec<f64>,
    /// Irreversible thermal damage `D` ∈ [0, 1].
    pub damage: Vec<f64>,
    /// Total grip multiplier `λ_μ,total = λ_μ(T_s)·f_w·(1 − Δ_c·σ)·(1 − Δ_D·D)` at the station state.
    pub grip_scale: Vec<f64>,
}

impl TireSlowLog {
    /// A zeroed log sized for `n` stations (the march fills it by index each outer iteration).
    fn zeros(n: usize) -> Self {
        Self {
            surface_temp_c: vec![0.0; n],
            carcass_temp_c: vec![0.0; n],
            gas_temp_c: vec![0.0; n],
            wear_mm: vec![0.0; n],
            damage: vec![0.0; n],
            grip_scale: vec![0.0; n],
        }
    }
}

/// The representative-tyre thermal ring + wear the QSS slow-state coupling marches along a solved
/// profile. Built at the native/host edge from the vehicle's front `.tyr` thermal + wear blocks and
/// geometry; handed to [`solve_t0`](crate::qss::solve_t0) / [`solve_t1`](crate::qss::solve_t1) as an
/// `Option`. Cloneable and allocation-free per segment.
#[derive(Clone, Debug)]
pub struct TireThermalMarch {
    /// The representative (front-tyre) ring parameters — the same ring the envelope tyre-state axes
    /// are generated from.
    ring: TireThermalRing<f64>,
    /// External (convecting) tread area `A_ext = 2π·R·W`, m².
    ext_area_m2: f64,
    /// Vertical stiffness `k_z`, N/m (for the deflection `δ = F_z/k_z`).
    k_vertical_n_per_m: f64,
    /// Loaded rolling radius `R`, m (for the spin rate `Ω = v/R`).
    radius_m: f64,
    /// Ambient air temperature `T_air`, K.
    t_air_k: f64,
    /// Road-surface temperature `T_road`, K.
    t_road_k: f64,
    /// The seed state every march starts from (warm at the grip optimum by default).
    seed: TireThermalState<f64>,
}

impl TireThermalMarch {
    /// Build a march from the representative front-tyre ring, its geometry, the compound's grip-optimum
    /// / cold-inflation reference temperatures (°C, for the parity-safe warm seed), and the session
    /// air / track-surface temperatures (°C). Optional geometry coefficients fall back to the documented
    /// racing-slick defaults. The seed is **warm at the grip optimum** (`T_s = T_c = T_opt`,
    /// `T_g = T_cold`).
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ring: TireThermalRing<f64>,
        radius_m: Option<f64>,
        width_m: Option<f64>,
        k_vertical: Option<f64>,
        t_opt_c: f64,
        t_cold_c: f64,
        air_temp_c: f64,
        track_surface_c: f64,
    ) -> Self {
        let r = radius_m.unwrap_or(RADIUS_FALLBACK).max(1e-3);
        let w = width_m.unwrap_or(WIDTH_FALLBACK).max(1e-3);
        let seed = TireThermalState::new(
            t_opt_c + CELSIUS_K,
            t_opt_c + CELSIUS_K,
            t_cold_c + CELSIUS_K,
        );
        Self {
            ring,
            ext_area_m2: std::f64::consts::TAU * r * w,
            k_vertical_n_per_m: k_vertical.unwrap_or(VERTICAL_STIFFNESS_FALLBACK).max(1.0),
            radius_m: r,
            t_air_k: air_temp_c + CELSIUS_K,
            t_road_k: track_surface_c + CELSIUS_K,
            seed,
        }
    }

    /// Override the seed with an explicit uniform cold start at `temp_c` (°C), zero wear/damage — the
    /// warm-up transient the property tests exercise.
    #[must_use]
    pub fn with_seed(mut self, temp_c: f64) -> Self {
        self.seed = TireThermalState::uniform(temp_c + CELSIUS_K);
        self
    }

    /// Override the seed with an explicit full state — the **stint continuity** path: seed lap N+1's
    /// march from lap N's terminal `(T_s/T_c/T_g, wear, damage)` so the tyre state carries across the
    /// lap boundary with no reset (§6.1 segment-to-segment march, extended across laps).
    #[must_use]
    pub fn with_state(mut self, state: TireThermalState<f64>) -> Self {
        self.seed = state;
        self
    }

    /// The seed state every march starts from (the state lap N+1 inherits from lap N in a stint).
    #[must_use]
    pub fn seed(&self) -> TireThermalState<f64> {
        self.seed
    }

    /// The seed surface temperature (K) — the reference `T_tire` the first (pre-march) solve indexes.
    #[must_use]
    pub fn seed_surface_k(&self) -> f64 {
        self.seed.t_s_k
    }

    /// March the representative tyre state forward along a solved profile `(v, ax)`, filling the
    /// per-station envelope index `t_tire_k` / `wear_mm` (the state the car carries **into** station
    /// `i`) and the telemetry `log`, and returning the **terminal** state (the end-of-lap
    /// `(T_s/T_c/T_g, wear, damage)` a stint carries into the next lap's seed). Restarts from the seed
    /// each call, so every outer iteration marches the whole lap from the reference state
    /// (deterministic). Zero heap allocation.
    #[allow(clippy::too_many_arguments)]
    pub fn march(
        &self,
        veh: &T0Vehicle,
        env: &GgvEnvelope,
        path: &T0Path,
        v: &[f64],
        ax: &[f64],
        t_tire_k: &mut [f64],
        wear_mm: &mut [f64],
        log: &mut TireSlowLog,
    ) -> TireThermalState<f64> {
        let mut st = self.seed;
        let m = veh.mass_kg;
        let n = path.len();
        for seg in 0..path.segments() {
            let i = seg;
            let j = if path.closed { (seg + 1) % n } else { seg + 1 };
            let vi = v[i].max(1e-3);
            let dt = 2.0 * path.ds / (v[i] + v[j]).max(1e-6);
            // Log the ENTRY state at station i — the state the car carries INTO segment i, and the
            // state the re-solve indexes the envelope with at station i.
            self.record(i, &st, t_tire_k, wear_mm, log);
            self.step_segment(veh, env, path, i, vi, ax[i], m, &mut st, dt);
        }
        // An open path's final station is not a segment start: it carries the end-of-lap state.
        if !path.closed && n > 0 {
            self.record(n - 1, &st, t_tire_k, wear_mm, log);
        }
        st
    }

    /// Record station `i`'s envelope index + telemetry from the entry state.
    fn record(
        &self,
        i: usize,
        st: &TireThermalState<f64>,
        t_tire_k: &mut [f64],
        wear_mm: &mut [f64],
        log: &mut TireSlowLog,
    ) {
        t_tire_k[i] = st.t_s_k;
        wear_mm[i] = st.wear_mm;
        log.surface_temp_c[i] = st.t_s_k - CELSIUS_K;
        log.carcass_temp_c[i] = st.t_c_k - CELSIUS_K;
        log.gas_temp_c[i] = st.t_g_k - CELSIUS_K;
        log.wear_mm[i] = st.wear_mm;
        log.damage[i] = st.damage;
        log.grip_scale[i] = self.ring.couplings(st).mu_scale_total;
    }

    /// Advance the ring one segment under the point-mass drivers at station `i`.
    #[allow(clippy::too_many_arguments)]
    fn step_segment(
        &self,
        veh: &T0Vehicle,
        env: &GgvEnvelope,
        path: &T0Path,
        i: usize,
        vi: f64,
        ax_i: f64,
        m: f64,
        st: &mut TireThermalState<f64>,
        dt: f64,
    ) {
        let (ay_dem, gn) = demand_and_gn(path, i, vi);
        // Point-mass tyre forces (total, all four tyres): the normal load carrying aero, the
        // longitudinal force to hold a_x against drag + grade, and the lateral demand.
        let n_load = (m * gn + veh.qz * vi * vi).max(0.0);
        let fx = m * (ax_i + env.drag_accel(vi) + G * path.sin_g[i]);
        let fy = m * ay_dem;
        let f_tire = fx.hypot(fy);
        // Friction-circle utilisation against the envelope's current grip capacity `F_cap = m·a_grip`
        // (indexed at the tyre's own live state, so a hot/worn tyre reports a higher utilisation).
        let a_grip = env
            .ay_boundary_at(vi, 0.0, gn, st.t_s_k, st.wear_mm)
            .max(1e-3);
        let f_cap = m * a_grip;
        let rho = (f_tire / f_cap).clamp(0.0, 1.0);
        let v_slip = SLIP_REFERENCE * vi * rho;
        // The representative single tyre carries 1/4 of the point-mass load + force.
        let fz = n_load / N_TYRES;
        let f_tire_wheel = f_tire / N_TYRES;
        let slip_power = f_tire_wheel * v_slip;
        // Carcass hysteresis: Q_hyst = c_h·Fz·(Fz/k_z)·(v/R).
        let deflection = fz / self.k_vertical_n_per_m;
        let omega = vi / self.radius_m;
        let carcass_loss = HYSTERESIS_LOSS_FACTOR * fz * deflection * omega;
        // Contact patch A_cp = Fz/p over the external tread area, using the ring's current hot pressure.
        let pressure = self.ring.pressure_pa(st).max(1.0);
        let contact_fraction = (fz / (pressure * self.ext_area_m2)).clamp(0.0, 1.0);
        let drivers = ThermalDrivers {
            slip_power_w: slip_power,
            carcass_loss_w: carcass_loss,
            speed_mps: vi,
            contact_fraction,
            ext_area_m2: self.ext_area_m2,
            t_air_k: self.t_air_k,
            t_road_k: self.t_road_k,
        };
        let _ = self.ring.step(st, &drivers, dt);
    }
}

/// Build a zeroed [`TireSlowLog`] sized for `n` stations (used by the profile solver).
pub(crate) fn tire_slow_log(n: usize) -> TireSlowLog {
    TireSlowLog::zeros(n)
}

#[cfg(test)]
mod tests {
    use std::sync::LazyLock;

    use super::*;
    use crate::result::T0Workspace;
    use crate::solver::{derive_ax, solve_into_ggv, solve_into_ggv_coupled};
    use crate::t1::envelope::tests::{sample_car, sample_t0};
    use crate::t1::TireStateRes;
    use crate::vehicle::T0Vehicle;
    use outlap_schema::sim::FzCoupling;

    /// A constant-radius closed circle: uniform hard cornering (steady lateral load → tyre heating),
    /// flat, full grip. `n` stations of radius `r` m.
    fn circle(n: usize, r: f64) -> T0Path {
        let circumference = std::f64::consts::TAU * r;
        let ds = circumference / n as f64;
        T0Path {
            s: (0..n).map(|i| ds * i as f64).collect(),
            kappa_l: vec![1.0 / r; n],
            kappa_n: vec![0.0; n],
            sin_b_cos_g: vec![0.0; n],
            cos_b_cos_g: vec![1.0; n],
            sin_g: vec![0.0; n],
            grip: vec![1.0; n],
            ds,
            closed: true,
        }
    }

    /// A minimal tyre-state resolution (3 T-nodes, 2 wear-nodes) — enough to exercise the axes while
    /// keeping the expensive re-solve cheap for CI (the reference slice is still node-exact).
    fn tiny_res() -> TireStateRes {
        TireStateRes {
            t_points: 3,
            w_points: 2,
        }
    }

    /// The tyre-state envelope built **once** (the re-solve across the axes is the expensive step) and
    /// cloned per test. Minimal resolution; `OneStepLag` coupling.
    static ENV: LazyLock<GgvEnvelope> = LazyLock::new(|| {
        GgvEnvelope::generate_with_tire_state(
            &sample_car(),
            &crate::t1::envelope::tests::small_res(),
            FzCoupling::OneStepLag,
            tiny_res(),
        )
        .unwrap()
    });

    /// The shared setup: the T0 vehicle, the (cloned) tyre-state envelope, the circle path, and a
    /// solved frozen profile `(v, ax)` on it.
    fn solved() -> (T0Vehicle, GgvEnvelope, T0Path, Vec<f64>, Vec<f64>) {
        let t0 = sample_t0();
        let env = ENV.clone();
        let path = circle(48, 45.0);
        let mut ws = T0Workspace::for_path(&path);
        solve_into_ggv(&t0, &env, &path, &mut ws).unwrap();
        let mut ax = vec![0.0; path.len()];
        derive_ax(&path, &ws.v, &mut ax);
        (t0, env, path, ws.v.clone(), ax)
    }

    /// A march for the sample tyre — grip optimum / cold reference matching the ring, geometry defaults.
    fn sample_march() -> TireThermalMarch {
        let ring = sample_car().tire_thermal().clone();
        let t_opt_c = ring.t_opt_c();
        TireThermalMarch::new(ring, None, None, None, t_opt_c, 25.0, 25.0, 30.0)
    }

    /// A cold-seeded march on a hard-cornering lap warms the surface and wears the tread monotonically.
    #[test]
    fn warm_up_and_wear_monotone() {
        let (t0, env, path, v, ax) = solved();
        let n = path.len();
        let march = sample_march().with_seed(25.0);
        let (mut t_tire, mut wear) = (vec![0.0; n], vec![0.0; n]);
        let mut log = tire_slow_log(n);
        march.march(&t0, &env, &path, &v, &ax, &mut t_tire, &mut wear, &mut log);
        // Surface warms above the 25 °C seed under sustained lateral load.
        let peak_surface = log.surface_temp_c.iter().copied().fold(f64::MIN, f64::max);
        assert!(
            peak_surface > 25.5,
            "surface did not warm from the cold seed: peak {peak_surface:.2} °C"
        );
        // Wear is monotone non-decreasing along the lap (Archard: sliding energy only adds).
        for i in 1..n {
            assert!(
                wear[i] >= wear[i - 1] - 1e-12,
                "wear fell at station {i}: {} < {}",
                wear[i],
                wear[i - 1]
            );
        }
        // Some wear actually accumulated.
        assert!(wear[n - 1] > 0.0, "no wear accumulated over the lap");
    }

    /// Reference-slice bit-identity through the solver: on a tyre-state envelope, the frozen solve
    /// (`ay_boundary`) and a coupled solve pinned at `(T_opt, wear = 0)` (`ay_boundary_at`) produce a
    /// bit-for-bit identical velocity profile — the PR4 invariant, exercised through the sweep.
    #[test]
    fn reference_slice_identity_through_solver() {
        let (t0, env, path, _v, _ax) = solved();
        let n = path.len();
        let t_opt_k = sample_car().tire_thermal().t_opt_k();
        let t_ref = vec![t_opt_k; n];
        let w_ref = vec![0.0; n];

        let mut ws_frozen = T0Workspace::for_path(&path);
        let lt_frozen = solve_into_ggv(&t0, &env, &path, &mut ws_frozen).unwrap();
        let mut ws_ref = T0Workspace::for_path(&path);
        let lt_ref = solve_into_ggv_coupled(
            &t0,
            &env,
            None,
            None,
            Some((&t_ref, &w_ref)),
            None,
            &path,
            &mut ws_ref,
        )
        .unwrap();

        assert_eq!(
            lt_frozen.to_bits(),
            lt_ref.to_bits(),
            "reference-slice lap time not bit-identical"
        );
        for i in 0..n {
            assert_eq!(
                ws_frozen.v[i].to_bits(),
                ws_ref.v[i].to_bits(),
                "reference-slice speed differs at station {i}"
            );
        }
    }

    /// A hotter/worn tyre solves a slower lap: the coupled march (leaving the reference under load)
    /// never beats the frozen reference lap, and here is measurably slower.
    #[test]
    fn degraded_tire_is_not_faster() {
        let car = sample_car();
        let (t0, env, path, _v, _ax) = solved();
        // Frozen reference lap.
        let mut ws = T0Workspace::for_path(&path);
        let lt_frozen = solve_into_ggv(&t0, &env, &path, &mut ws).unwrap();
        // A pre-worn, off-optimum tyre state pinned across the lap.
        let n = path.len();
        let hot = car.tire_thermal().t_opt_k() + 40.0; // 40 K over the window
        let worn = car.tire_thermal().w_c_mm(); // at the wear cliff
        let t_state = vec![hot; n];
        let w_state = vec![worn; n];
        let mut ws2 = T0Workspace::for_path(&path);
        let lt_deg = solve_into_ggv_coupled(
            &t0,
            &env,
            None,
            None,
            Some((&t_state, &w_state)),
            None,
            &path,
            &mut ws2,
        )
        .unwrap();
        assert!(
            lt_deg > lt_frozen,
            "a hot, worn tyre should not lap faster: {lt_deg:.4} vs {lt_frozen:.4}"
        );
    }

    /// Stint continuity (M5 PR6): the terminal state a march returns, fed into the next lap's seed
    /// via `with_state`, makes lap N+1 start **exactly** where lap N ended — no reset — and wear stays
    /// monotone non-decreasing across the lap boundary (Archard: sliding energy only ever adds; it may
    /// saturate at `w_max`, but never falls). A cold-seeded first lap also warms the surface, and that
    /// warmed state carries into lap 2's seed.
    #[test]
    fn stint_continuity_carries_state_across_laps() {
        let (t0, env, path, v, ax) = solved();
        let n = path.len();
        let march = sample_march().with_seed(25.0);
        // Lap 1 from the cold seed.
        let (mut t1, mut w1) = (vec![0.0; n], vec![0.0; n]);
        let mut log1 = tire_slow_log(n);
        let term1 = march.march(&t0, &env, &path, &v, &ax, &mut t1, &mut w1, &mut log1);
        assert!(term1.wear_mm > 0.0, "lap 1 wore the tread");
        assert!(
            term1.t_s_k > 25.0 + CELSIUS_K,
            "lap 1 warmed the surface off the 25 °C cold seed"
        );
        // Lap 2 seeded from lap 1's terminal state.
        let march2 = march.with_state(term1);
        let (mut t2, mut w2) = (vec![0.0; n], vec![0.0; n]);
        let mut log2 = tire_slow_log(n);
        let term2 = march2.march(&t0, &env, &path, &v, &ax, &mut t2, &mut w2, &mut log2);
        // Lap 2 starts exactly where lap 1 ended (the envelope-index buffers + telemetry logs).
        assert_eq!(
            w2[0].to_bits(),
            term1.wear_mm.to_bits(),
            "lap 2 wear seed = lap 1 terminal wear (no reset)"
        );
        assert_eq!(
            log2.surface_temp_c[0].to_bits(),
            (term1.t_s_k - CELSIUS_K).to_bits(),
            "lap 2 surface seed = lap 1 terminal surface (no reset)"
        );
        // Wear is monotone non-decreasing across the lap boundary (never falls; may cap at w_max).
        assert!(
            term2.wear_mm >= term1.wear_mm - 1e-12,
            "wear never falls across the lap boundary: {} -> {}",
            term1.wear_mm,
            term2.wear_mm
        );
    }

    /// The march is deterministic: same inputs twice → bit-identical state trajectories.
    #[test]
    fn march_is_deterministic() {
        let (t0, env, path, v, ax) = solved();
        let n = path.len();
        let march = sample_march().with_seed(25.0);
        let run = || {
            let (mut t, mut w) = (vec![0.0; n], vec![0.0; n]);
            let mut log = tire_slow_log(n);
            march.march(&t0, &env, &path, &v, &ax, &mut t, &mut w, &mut log);
            (t, w)
        };
        let (t1, w1) = run();
        let (t2, w2) = run();
        for i in 0..n {
            assert_eq!(
                t1[i].to_bits(),
                t2[i].to_bits(),
                "T_tire not deterministic at {i}"
            );
            assert_eq!(
                w1[i].to_bits(),
                w2[i].to_bits(),
                "wear not deterministic at {i}"
            );
        }
    }
}
