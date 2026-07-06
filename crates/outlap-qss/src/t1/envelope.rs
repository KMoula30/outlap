// SPDX-License-Identifier: AGPL-3.0-only
//! The g-g-g-v acceleration envelope (§6.1, §11.2 Locked Decision #31) generated from the T1 trim,
//! and its separable multiplicative sensitivity corrections.
//!
//! # What it is
//!
//! A g-g-g-v diagram is the classical g-g (longitudinal × lateral acceleration) limit extended with
//! two more axes: **vehicle speed** `v` (aerodynamic load grows with `v²`) and an **apparent /
//! road-normal specific gravity** `g_normal` (banking, grade, and vertical road curvature change how
//! hard the car is pressed into the road, so on a 3-D ribbon the grip envelope is a function of the
//! normal load — crests unload, dips/compressions like Eau Rouge load). outlap stores the base table
//! `a_y,corr = gg(v, a_x, g_normal)` — the maximum lateral acceleration the tyres sustain at speed
//! `v`, longitudinal acceleration `a_x`, and road-normal specific gravity `g_normal` — in a
//! [`GriddedMapN`] over the `sim.envelope` grid (40 × 25 × 7 by default, Locked Decision #10).
//!
//! **Normalised `a_x` axis.** The longitudinal capability spans a wide range across the speed/load
//! axes (light-load low-speed grip is a fraction of high-downforce high-speed grip), so a single
//! *actual*-`a_x` grid would leave the feasible window falling between nodes at low load. Following
//! the per-speed g-g construction of the reference works, the `a_x` axis is therefore **normalised**:
//! a node `â_x ∈ [−1, 1]` maps to the actual longitudinal acceleration `a_x = â_x · a_x,cap(v,
//! g_normal)` where `a_x,cap` is that operating point's own straight-line braking (`â_x<0`) or
//! acceleration (`â_x>0`) limit ([`GgvEnvelope::accel_limit`] / [`GgvEnvelope::brake_limit`]). Every
//! slice then uses the full range with a node on `â_x = 0` (pure lateral) — no holes, uniform
//! resolution. Queries take the *actual* `a_x` and normalise internally.
//!
//! Following Werner et al. (2025, §II-E, eq. 5) the stored lateral acceleration is projected into the
//! **velocity-vector frame** — `a_y,corr = a_y,body · cos β − a_x · sin β` at the converged body-slip
//! `β` — so a point-mass T0 solver, which has no slip angle as a state, can consume the boundary
//! directly. Following Werner et al. (§II-C), **powertrain torque limits are omitted from the
//! envelope**: it is a pure *tyre-force* limit; the drive-force ceiling is applied separately by the
//! T0 velocity-profile solver (`min(tractive_force, grip)`).
//!
//! # How it is generated
//!
//! For each `(v, g_normal)` the generator first brackets the straight-line braking/acceleration
//! limits `a_x,cap` (the `â_x = ±1` shoulders), then for each normalised `â_x` node bisects the
//! commanded lateral acceleration against the T1 trim's feasibility ([`T1Vehicle::trim_warm`]): a
//! converged trim is inside the friction limit, an infeasible one is past it. The largest feasible
//! `a_y` (projected to the velocity frame) is the boundary; the shoulders carry `a_y = 0` (never a
//! panic; the PR2 infeasible-trim contract). This is the quasi-steady-state acceleration-envelope
//! construction of Tremlett et al. (2014) and Lovato & Massaro (2022); the apparent-gravity axis
//! follows Rowold et al. (2023) and Werner et al. (2025).
//!
//! # Decision #31 corrections
//!
//! Regenerating the whole envelope for every off-reference vehicle state (a different tyre grip,
//! mass, or downforce level in a strategy sweep) is expensive. Instead the generator stores, per
//! node, three **relative sensitivities** from central finite differences of full-T1 boundary
//! re-solves over each parameter's correction band (μ_tire, mass, ClA):
//!
//! ```text
//!   S_μ ≈ ∂ln a_y,corr / ∂ln μ_tire ,  S_m ≈ ∂ln a_y,corr / ∂ln m ,  S_ClA ≈ ∂ln a_y,corr / ∂ln ClA
//! ```
//!
//! and evaluates the corrected boundary as a separable multiplicative form that is **identity at the
//! reference by construction**:
//!
//! ```text
//!   a_y,corr(… ; μ,m,ClA) = gg(…) · (1 + S_μ·(μ/μ₀−1)) · (1 + S_m·(m/m₀−1)) · (1 + S_ClA·(ClA/ClA₀−1))
//! ```
//!
//! clamped at `0`. The reference is `μ₀`/`ClA₀` scale = 1 (nominal grip and downforce), `m₀` the
//! vehicle's mass, cold tyres (the trim's basis), and a thermal-/SoC-neutral state (the dynamic
//! thermal derate and battery power cap compose with this static envelope at the lap level, PR6). The
//! correction is a **lateral-grip magnitude** model, accurate near the cornering peak; toward the
//! longitudinal shoulders (where the velocity-frame `−a_x·sinβ` term dominates, and where a
//! multiplicative factor cannot MOVE the shoulder) it is magnitude-clamped, not accurate. CI
//! validates it against full-T1 re-solves at sampled off-reference states in the lateral-grip region.
//!
//! # References (clean-room from published literature; no code copied — the GPL-3.0
//! TUM-AVS/GGGVDiagrams repo, the reference implementation of Werner et al. 2025, was consulted for
//! approach only and re-authored from the papers)
//!
//! * F. Werner, S. Sagmeister, M. Piccinini, J. Betz, "A Quasi-Steady-State Black Box Simulation
//!   Approach for the Generation of g-g-g-v Diagrams," 2025, arXiv:2504.10225 (virtual-inertial-force
//!   QSS method, the velocity-frame lateral correction eq. 5, the tyre-force-only envelope).
//! * M. Rowold, L. Ögretmen, U. Kasolowsky, B. Lohmann, "Online Time-Optimal Trajectory Planning on
//!   Three-Dimensional Race Tracks," 2023 IEEE Intelligent Vehicles Symposium (IV) — the 3-D
//!   apparent-gravity `g̃` axis.
//! * D. Lovato & M. Massaro, "A three-dimensional free-trajectory quasi-steady-state optimal-control
//!   method for the minimum-lap-time of race vehicles," Vehicle System Dynamics 60(5), 2022,
//!   pp. 1512–1530.
//! * A. J. Tremlett et al., "Quasi-steady-state linearisation of the racing vehicle acceleration
//!   envelope: a limited slip differential example," Vehicle System Dynamics 52(11), 2014.

use outlap_core::{EvalFlags, GriddedMapN, MonotoneCubic, OutOfDomain};
use outlap_schema::sim::{Envelope as EnvelopeRes, FzCoupling};

use crate::error::T1Error;
use crate::t1::trim::{TrimInput, TrimState};
use crate::t1::vehicle::T1Vehicle;
use crate::G;

/// Lowest speed sampled by the envelope, m/s (above the trim's `V_MIN` crawl floor; queries below it
/// clamp).
const V_ENV_LO: f64 = 5.0;
/// Hard ceiling on the sampled top speed, m/s (≈ 432 km/h — above any modelled vehicle).
const V_ENV_CAP: f64 = 120.0;
/// Lower bound of the `g_normal` axis as a factor of standard gravity (strong crest unloading).
const GN_LO_FACTOR: f64 = 0.5;
/// Upper bound of the `g_normal` axis as a factor of standard gravity (Eau-Rouge-type compression).
const GN_HI_FACTOR: f64 = 2.0;
/// Hard ceiling on the lateral / longitudinal bracket search, m/s² (~9 g; downforce cars).
const A_CAP: f64 = 90.0;
/// Initial upper guess for the max-lateral bracket, m/s².
const AY_SEED: f64 = 20.0;
/// Maximum bracket-expansion doublings.
const MAX_EXPAND: usize = 8;
/// Bisection iterations for a full boundary search (2⁻¹⁶·A_CAP ≈ 1.4e-3 m/s² — below the ≤2% gate).
const MAX_BISECT: usize = 16;
/// Bisection iterations for a narrow (hinted) boundary search around a nearby known boundary.
const MAX_BISECT_NARROW: usize = 12;
/// Half-width of the narrow bracket, as a fraction of the hinted boundary.
const NARROW_W: f64 = 0.4;
/// Central-difference relative steps for the sensitivity corrections, matched to each parameter's
/// intended correction band so the stored sensitivity is the **secant** over that band (exact at the
/// band edge for a linear response) rather than a tiny tangent a wide extrapolation would misread.
const H_MU: f64 = 0.15;
/// Central-difference step for the mass sensitivity (± the mass band).
const H_MASS: f64 = 0.10;
/// Central-difference step for the downforce (ClA) sensitivity (± the ClA band).
const H_CLA: f64 = 0.30;
/// Lateral acceleration below which corrections are suppressed (the node carries ≈ no grip to
/// correct; a relative sensitivity there would divide by ≈ 0), m/s².
const AY_FLOOR: f64 = 0.5;
/// Fraction of a fibre's peak lateral grip below which per-node sensitivities are NOT computed:
/// toward the shoulders the boundary is steep and a finite-difference sensitivity is noisy, so
/// corrections are confined to the reliable near-peak bulk.
const CORR_MIN_FRAC: f64 = 0.5;
/// Clamp on the stored relative sensitivities. A g-g boundary's grip/mass/downforce log-sensitivities
/// are O(1) (`a_y ≈ μ·(g_normal + ClA·q·v²/m)`), so |S| ≲ 1.5 in the bulk; larger is finite-difference
/// noise and is clamped.
const S_CLAMP: f64 = 2.0;
/// Lower clamp on the evaluated multiplicative correction factor (keeps an off-reference query
/// physical).
const F_MIN: f64 = 0.3;
/// Upper clamp on the evaluated multiplicative correction factor.
const F_MAX: f64 = 3.0;
/// Step for the top-speed estimate scan, m/s.
const TOP_SPEED_DV: f64 = 2.0;
/// Floor on a longitudinal capability used as a normalisation denominator, m/s² (avoids /0 in the
/// degenerate no-drive / no-brake case).
const CAP_FLOOR: f64 = 0.5;

/// A generated g-g-g-v acceleration envelope: the base velocity-frame lateral-acceleration boundary
/// `a_y,corr(v, a_x, g_normal)` (on a normalised `a_x` axis), the three Decision #31 sensitivity
/// fields, and the per-`(v, g_normal)` longitudinal capability the normalisation uses.
///
/// Build with [`GgvEnvelope::generate`]; query the reference boundary with
/// [`GgvEnvelope::ay_boundary`] or the corrected boundary with [`GgvEnvelope::ay_boundary_corrected`]
/// (both zero-allocation, taking the *actual* `a_x`). The Python surface lands in PR8.
#[derive(Clone, Debug)]
pub struct GgvEnvelope {
    /// Base boundary `a_y,corr(v, â_x, g_normal)`, m/s², on the normalised `â_x ∈ [−1, 1]` axis.
    base: GriddedMapN<f64>,
    /// Per-node relative sensitivity `∂ln a_y,corr/∂ln μ_tire` at the reference.
    s_mu: GriddedMapN<f64>,
    /// Per-node relative sensitivity `∂ln a_y,corr/∂ln m` at the reference.
    s_mass: GriddedMapN<f64>,
    /// Per-node relative sensitivity `∂ln a_y,corr/∂ln ClA` at the reference.
    s_cla: GriddedMapN<f64>,
    /// Straight-line acceleration capability `a_x,cap⁺(v, g_normal)`, m/s² (the `â_x = +1` shoulder).
    accel_cap: GriddedMapN<f64>,
    /// Straight-line braking capability magnitude `a_x,cap⁻(v, g_normal)`, m/s² (the `â_x = −1`
    /// shoulder).
    brake_cap: GriddedMapN<f64>,
    /// Reference straight-line aero drag as an acceleration `q_x(v)·v²/m` (m/s²) vs speed — the drag
    /// currency the base `a_x` boundary embeds, so the lap solver can subtract a consistent drag from
    /// the powertrain branch when taking the traction `min`.
    drag_accel: MonotoneCubic<f64>,
    /// Reference mass, kg (corrections normalise mass by this).
    mass_ref: f64,
    /// The recorded coupling mode (Decision #29).
    coupling: FzCoupling,
    /// Human-readable notes on the envelope's construction and any degradations.
    notes: Vec<String>,
}

impl GgvEnvelope {
    /// Generate the g-g-g-v envelope from a [`T1Vehicle`] at the requested resolution.
    ///
    /// The `v` axis auto-ranges to the vehicle's own speed band, `g_normal` spans `[0.5 g, 2 g]`, and
    /// the `a_x` axis is the normalised `â_x ∈ [−1, 1]` mapped per operating point to its own
    /// longitudinal capability. Cold assembly step (allocations allowed); the per-lap query is the
    /// zero-allocation hot path.
    ///
    /// # Errors
    /// [`T1Error::GgvEnvelope`] / [`T1Error::Envelope`] if an interpolant cannot be built (should not
    /// happen: the generator builds full rectilinear grids of finite values).
    #[allow(clippy::too_many_lines)] // one linear grid-sweep procedure; splitting it hurts clarity.
    pub fn generate(
        car: &T1Vehicle,
        resolution: &EnvelopeRes,
        coupling: FzCoupling,
    ) -> Result<Self, T1Error> {
        let mut notes = Vec::new();
        let nv = (resolution.v_points as usize).max(2);
        let nax = (resolution.ax_points as usize).max(3);
        let ngn = (resolution.g_normal_points as usize).max(2);

        // --- Axes ---
        let v_hi = top_speed_estimate(car);
        let v_axis = linspace(V_ENV_LO, v_hi, nv);
        let gn_axis = linspace(GN_LO_FACTOR * G, GN_HI_FACTOR * G, ngn);
        let axn_axis = linspace(-1.0, 1.0, nax); // normalised longitudinal acceleration
        let mass_ref = car.mass_kg;

        notes.push(format!(
            "g-g-g-v envelope on a {nv}×{nax}×{ngn} (v, â_x, g_normal) grid: v ∈ [{:.1}, {:.1}] m/s, \
             â_x ∈ [−1, 1] normalised to each point's straight-line longitudinal capability, \
             g_normal ∈ [{:.2}, {:.2}] m/s²",
            V_ENV_LO,
            v_hi,
            GN_LO_FACTOR * G,
            GN_HI_FACTOR * G,
        ));
        notes.push(
            "envelope = tyre-force limit only (powertrain ceiling applied separately by the lap \
             solver); lateral accel stored in the velocity-vector frame (a_y·cosβ − a_x·sinβ) for \
             point-mass consumption; boundary = the T1 trim's friction feasibility limit (not \
             filtered for open-loop stability — a T2+ concern)."
                .to_owned(),
        );
        notes.push(
            "Decision #31 sensitivities sampled at every 2nd â_x node (full v and g_normal \
             resolution) and linearly interpolated along the fibre — near-flat along â_x in the \
             near-peak bulk; accuracy is guarded by the corrected-envelope CI gate."
                .to_owned(),
        );

        // --- Sweep the grid ---
        let n_nodes = nv * nax * ngn;
        let n_vg = nv * ngn;
        let mut base = vec![0.0; n_nodes];
        let mut s_mu = vec![0.0; n_nodes];
        let mut s_mass = vec![0.0; n_nodes];
        let mut s_cla = vec![0.0; n_nodes];
        let mut accel = vec![CAP_FLOOR; n_vg];
        let mut brake = vec![CAP_FLOOR; n_vg];
        // Perturbed vehicles for the central-difference sensitivities (cold clones).
        let car_mu_p = car.with_mu_scale(1.0 + H_MU);
        let car_mu_m = car.with_mu_scale(1.0 - H_MU);
        let car_m_p = car.with_mass(mass_ref * (1.0 + H_MASS));
        let car_m_m = car.with_mass(mass_ref * (1.0 - H_MASS));
        let car_cla_p = car.with_cla_scale(1.0 + H_CLA);
        let car_cla_m = car.with_cla_scale(1.0 - H_CLA);
        let clamp_s = |s: f64| s.clamp(-S_CLAMP, S_CLAMP);
        // Index of the â_x node nearest 0 (the pure-lateral seed for the outward march).
        let i0 = (0..nax)
            .min_by(|&a, &b| axn_axis[a].abs().total_cmp(&axn_axis[b].abs()))
            .unwrap_or(0);

        for (iv, &v) in v_axis.iter().enumerate() {
            for (ign, &gn) in gn_axis.iter().enumerate() {
                // Per-point longitudinal capability (the â_x = ±1 shoulders).
                let a_cap = max_straight_ax(car, v, gn, coupling, 1.0).max(CAP_FLOOR);
                let b_cap = (-max_straight_ax(car, v, gn, coupling, -1.0)).max(CAP_FLOOR);
                accel[iv * ngn + ign] = a_cap;
                brake[iv * ngn + ign] = b_cap;
                let ax_of = |axn: f64| axn * if axn >= 0.0 { a_cap } else { b_cap };

                // Pass 1: the base boundary over the â_x fibre, marching outward from â_x = 0.
                let mut fibre: Vec<Boundary> = vec![Boundary::zero(); nax];
                fibre[i0] = max_lateral(car, v, ax_of(axn_axis[i0]), gn, coupling, None);
                for iax in (i0 + 1)..nax {
                    fibre[iax] = max_lateral(
                        car,
                        v,
                        ax_of(axn_axis[iax]),
                        gn,
                        coupling,
                        fibre[iax - 1].hint(),
                    );
                }
                for iax in (0..i0).rev() {
                    fibre[iax] = max_lateral(
                        car,
                        v,
                        ax_of(axn_axis[iax]),
                        gn,
                        coupling,
                        fibre[iax + 1].hint(),
                    );
                }
                for (iax, b) in fibre.iter().enumerate() {
                    base[(iv * nax + iax) * ngn + ign] = b.ay_corr;
                }

                // Pass 2: central-difference sensitivities in the fibre's near-peak bulk — on
                // every 2nd â_x node only (the skipped nodes are filled by linear interpolation
                // below). `v` and `g_normal` keep full resolution: the fields vary strongly with
                // speed through the downforce transition (a v-subsampled variant failed the
                // Decision #31 gate at 6%), but are near-flat along â_x in the near-peak bulk
                // where corrections apply. Each sampled node costs SIX extra boundary searches,
                // so this halves the dominant sensitivity cost; the #31 CI gate guards accuracy.
                let peak = fibre.iter().fold(0.0_f64, |m, b| m.max(b.ay_corr));
                let thresh = (CORR_MIN_FRAC * peak).max(AY_FLOOR);
                for (iax, &axn) in axn_axis.iter().enumerate() {
                    let h = fibre[iax].hint();
                    if !sens_sampled(iax, nax) || fibre[iax].ay_corr < thresh || h.is_none() {
                        continue;
                    }
                    let ax = ax_of(axn);
                    let d_mu = max_lateral(&car_mu_p, v, ax, gn, coupling, h).ay_corr
                        - max_lateral(&car_mu_m, v, ax, gn, coupling, h).ay_corr;
                    let d_m = max_lateral(&car_m_p, v, ax, gn, coupling, h).ay_corr
                        - max_lateral(&car_m_m, v, ax, gn, coupling, h).ay_corr;
                    let d_cla = max_lateral(&car_cla_p, v, ax, gn, coupling, h).ay_corr
                        - max_lateral(&car_cla_m, v, ax, gn, coupling, h).ay_corr;
                    let ay0 = fibre[iax].ay_corr;
                    let node = (iv * nax + iax) * ngn + ign;
                    s_mu[node] = clamp_s(d_mu / (2.0 * H_MU * ay0));
                    s_mass[node] = clamp_s(d_m / (2.0 * H_MASS * ay0));
                    s_cla[node] = clamp_s(d_cla / (2.0 * H_CLA * ay0));
                }
            }
        }
        fill_sensitivities(
            [&mut s_mu, &mut s_mass, &mut s_cla],
            &base,
            &axn_axis,
            [nv, nax, ngn],
        );

        // Reference straight-line drag-as-acceleration vs speed (the currency the a_x axis embeds).
        let drag_vals: Vec<f64> = v_axis
            .iter()
            .map(|&v| car.aero_lumped(v, 0.0, 0.0, 0.0).qx * v * v / mass_ref)
            .collect();
        let drag_accel =
            MonotoneCubic::new(v_axis.clone(), drag_vals).map_err(T1Error::Envelope)?;

        let axes3 = vec![v_axis.clone(), axn_axis, gn_axis.clone()];
        let modes3 = vec![OutOfDomain::Clamp; 3];
        let build3 = |values: Vec<f64>| {
            GriddedMapN::from_gridded(axes3.clone(), values, modes3.clone())
                .map_err(T1Error::GgvEnvelope)
        };
        let axes2 = vec![v_axis, gn_axis];
        let modes2 = vec![OutOfDomain::Clamp; 2];
        let build2 = |values: Vec<f64>| {
            GriddedMapN::from_gridded(axes2.clone(), values, modes2.clone())
                .map_err(T1Error::GgvEnvelope)
        };
        Ok(Self {
            base: build3(base)?,
            s_mu: build3(s_mu)?,
            s_mass: build3(s_mass)?,
            s_cla: build3(s_cla)?,
            accel_cap: build2(accel)?,
            brake_cap: build2(brake)?,
            drag_accel,
            mass_ref,
            coupling,
            notes,
        })
    }

    /// The straight-line acceleration capability `a_x,cap⁺(v, g_normal)`, m/s² (the `â_x = +1`
    /// shoulder). Zero-allocation.
    #[must_use]
    pub fn accel_limit(&self, v: f64, g_normal: f64) -> f64 {
        self.accel_cap.eval(&[v, g_normal]).max(CAP_FLOOR)
    }

    /// The straight-line braking capability magnitude `a_x,cap⁻(v, g_normal)`, m/s² (the `â_x = −1`
    /// shoulder; positive). Zero-allocation.
    #[must_use]
    pub fn brake_limit(&self, v: f64, g_normal: f64) -> f64 {
        self.brake_cap.eval(&[v, g_normal]).max(CAP_FLOOR)
    }

    /// Normalise an actual longitudinal acceleration to `â_x ∈ [−1, 1]` for this operating point.
    fn normalize_ax(&self, v: f64, ax: f64, g_normal: f64) -> f64 {
        let cap = if ax >= 0.0 {
            self.accel_limit(v, g_normal)
        } else {
            self.brake_limit(v, g_normal)
        };
        (ax / cap).clamp(-1.0, 1.0)
    }

    /// The reference-state lateral-acceleration boundary `a_y,corr(v, a_x, g_normal)`, m/s²
    /// (velocity-vector frame), at the *actual* longitudinal acceleration `a_x`. Zero-allocation.
    #[must_use]
    pub fn ay_boundary(&self, v: f64, ax: f64, g_normal: f64) -> f64 {
        self.base
            .eval(&[v, self.normalize_ax(v, ax, g_normal), g_normal])
    }

    /// The corrected lateral-acceleration boundary at an off-reference tyre grip / mass / downforce,
    /// m/s². `mu_scale` and `cla_scale` are multiples of the reference (1.0 = reference); `mass_kg` is
    /// the absolute mass. Identity at the reference state; clamped at 0. Zero-allocation.
    #[must_use]
    pub fn ay_boundary_corrected(
        &self,
        v: f64,
        ax: f64,
        g_normal: f64,
        mu_scale: f64,
        mass_kg: f64,
        cla_scale: f64,
    ) -> f64 {
        let x = [v, self.normalize_ax(v, ax, g_normal), g_normal];
        let base = self.base.eval(&x);
        let f = |s: f64, delta: f64| (1.0 + s * delta).clamp(F_MIN, F_MAX);
        let f_mu = f(self.s_mu.eval(&x), mu_scale - 1.0);
        let f_mass = f(self.s_mass.eval(&x), mass_kg / self.mass_ref - 1.0);
        let f_cla = f(self.s_cla.eval(&x), cla_scale - 1.0);
        (base * f_mu * f_mass * f_cla).max(0.0)
    }

    /// The reference boundary with the [`EvalFlags`] recording whether the query left the grid.
    /// Zero-allocation.
    #[must_use]
    pub fn ay_boundary_flagged(&self, v: f64, ax: f64, g_normal: f64) -> (f64, EvalFlags) {
        self.base
            .eval_flagged(&[v, self.normalize_ax(v, ax, g_normal), g_normal])
    }

    /// The reference straight-line aero drag as an acceleration `q_x(v)·v²/m` at speed `v` (m/s²) —
    /// the drag currency the base `a_x` boundary embeds. Below the lowest sampled speed the drag
    /// tapers as `v²` toward 0 (the interpolant would otherwise clamp to a small constant, applying a
    /// spurious drag at a standing start). Zero-allocation.
    #[must_use]
    pub fn drag_accel(&self, v: f64) -> f64 {
        if v < V_ENV_LO {
            self.drag_accel.eval(V_ENV_LO) * (v / V_ENV_LO).powi(2)
        } else {
            self.drag_accel.eval(v)
        }
    }

    /// The `[first, last]` breakpoints of the `(v, â_x, g_normal)` axes (`â_x` is normalised).
    #[must_use]
    pub fn domain(&self) -> [(f64, f64); 3] {
        let d = self.base.domain();
        [d[0], d[1], d[2]]
    }

    /// The grid shape `[n_v, n_âx, n_g_normal]`.
    #[must_use]
    pub fn shape(&self) -> [usize; 3] {
        let s = self.base.shape();
        [s[0], s[1], s[2]]
    }

    /// The reference mass, kg (corrections normalise mass by this).
    #[must_use]
    pub fn mass_ref(&self) -> f64 {
        self.mass_ref
    }

    /// The recorded normal-load coupling mode (Decision #29).
    #[must_use]
    pub fn coupling(&self) -> FzCoupling {
        self.coupling
    }

    /// Human-readable notes on the envelope's construction (nothing silent).
    #[must_use]
    pub fn notes(&self) -> &[String] {
        &self.notes
    }
}

/// A solved lateral-acceleration boundary at one `(v, a_x, g_normal)` node.
#[derive(Clone)]
struct Boundary {
    /// Velocity-frame lateral boundary `a_y,corr`, m/s² (0 if `a_x` is unreachable).
    ay_corr: f64,
    /// Body-frame boundary `a_y`, m/s² (the bisection variable).
    ay_body: f64,
    /// The converged trim at the boundary (for the sideslip projection and as a warm hint).
    state: Option<TrimState>,
}

impl Boundary {
    /// A zero (infeasible-`a_x`) boundary.
    fn zero() -> Self {
        Self {
            ay_corr: 0.0,
            ay_body: 0.0,
            state: None,
        }
    }

    /// The `(a_y, state)` warm hint for a neighbouring node, if this one converged.
    fn hint(&self) -> Option<(f64, &TrimState)> {
        self.state.as_ref().map(|s| (self.ay_body, s))
    }
}

/// The maximum feasible **velocity-frame** lateral acceleration at `(v, a_x, g_normal)`, m/s²
/// (Werner et al. 2025 eq. 5), with the body-frame boundary and its trim state for reuse. Returns a
/// zero boundary if the longitudinal acceleration is unreachable even in a straight line. A `hint` (a
/// nearby known boundary) enables a narrow, warm-started bracket; otherwise a full expand-and-bisect
/// search runs. The straight-line seed falls back to the robust continuation `trim` if the warm
/// direct solve misses a feasible hard-braking / high-load state.
fn max_lateral(
    car: &T1Vehicle,
    v: f64,
    ax: f64,
    gn: f64,
    coupling: FzCoupling,
    hint: Option<(f64, &TrimState)>,
) -> Boundary {
    let inp = |ay: f64| TrimInput {
        v,
        ay,
        ax,
        g_normal: gn,
        coupling,
    };
    let warm = hint.map(|(_, s)| s);
    let base_state = car
        .trim_warm(&inp(0.0), warm)
        .state()
        .copied()
        .or_else(|| car.trim(&inp(0.0)).state().copied());
    let Some(base_state) = base_state else {
        return Boundary::zero();
    };

    // Fast path: a hinted narrow bracket around a nearby boundary.
    if let Some((hay, hs)) = hint {
        if hay >= AY_FLOOR {
            let lo = (hay * (1.0 - NARROW_W)).max(0.0);
            let hi = hay * (1.0 + NARROW_W);
            if let Some(lo_state) = car.trim_warm(&inp(lo), Some(hs)).state().copied() {
                if car.trim_warm(&inp(hi), Some(&lo_state)).state().is_none() {
                    return bisect(car, &inp, lo, lo_state, hi, MAX_BISECT_NARROW, ax);
                }
            }
        }
    }

    // Full path: expand a_y until an infeasible upper bound is found, then bisect.
    let mut lo = 0.0;
    let mut lo_state = base_state;
    let mut hi = AY_SEED;
    let mut bracketed = false;
    for _ in 0..MAX_EXPAND {
        if let Some(st) = car.trim_warm(&inp(hi), Some(&lo_state)).state().copied() {
            lo = hi;
            lo_state = st;
            hi = (hi * 2.0).min(A_CAP);
            if lo >= A_CAP {
                break;
            }
        } else {
            bracketed = true;
            break;
        }
    }
    if bracketed {
        bisect(car, &inp, lo, lo_state, hi, MAX_BISECT, ax)
    } else {
        Boundary {
            ay_corr: velocity_frame_ay(lo, ax, &lo_state),
            ay_body: lo,
            state: Some(lo_state),
        }
    }
}

/// Bisect a feasible (`lo`, `lo_state`) / infeasible (`hi`) lateral-acceleration bracket, warm-starting
/// each probe from the last feasible state, and return the boundary.
fn bisect(
    car: &T1Vehicle,
    inp: &impl Fn(f64) -> TrimInput,
    mut lo: f64,
    mut lo_state: TrimState,
    mut hi: f64,
    iters: usize,
    ax: f64,
) -> Boundary {
    for _ in 0..iters {
        let mid = 0.5 * (lo + hi);
        match car.trim_warm(&inp(mid), Some(&lo_state)).state().copied() {
            Some(st) => {
                lo = mid;
                lo_state = st;
            }
            None => hi = mid,
        }
    }
    Boundary {
        ay_corr: velocity_frame_ay(lo, ax, &lo_state),
        ay_body: lo,
        state: Some(lo_state),
    }
}

/// Project the body-frame lateral acceleration `ay_body` into the velocity-vector frame at the
/// converged body-slip `β` (Werner et al. 2025 eq. 5): `a_y,corr = a_y·cosβ − a_x·sinβ`.
fn velocity_frame_ay(ay_body: f64, ax: f64, state: &TrimState) -> f64 {
    let (sin_b, cos_b) = state.beta.sin_cos();
    (ay_body * cos_b - ax * sin_b).max(0.0)
}

/// The maximum feasible straight-line (`a_y = 0`) longitudinal acceleration at speed `v` and
/// road-normal gravity `gn` in the direction `sign` (`+1` acceleration, `−1` braking), m/s².
fn max_straight_ax(car: &T1Vehicle, v: f64, gn: f64, coupling: FzCoupling, sign: f64) -> f64 {
    let inp = |ax: f64| TrimInput {
        v,
        ay: 0.0,
        ax,
        g_normal: gn,
        coupling,
    };
    if car.trim_warm(&inp(0.0), None).state().is_none() && car.trim(&inp(0.0)).state().is_none() {
        return 0.0;
    }
    let mut lo = 0.0; // feasible magnitude
    let mut hi = 5.0; // initial guess, m/s²
    let mut warm: Option<TrimState> = None;
    let mut bracketed = false;
    for _ in 0..MAX_EXPAND {
        if let Some(st) = car
            .trim_warm(&inp(sign * hi), warm.as_ref())
            .state()
            .copied()
        {
            lo = hi;
            warm = Some(st);
            hi = (hi * 2.0).min(A_CAP);
            if lo >= A_CAP {
                break;
            }
        } else {
            bracketed = true;
            break;
        }
    }
    if bracketed {
        for _ in 0..MAX_BISECT {
            let mid = 0.5 * (lo + hi);
            if let Some(st) = car
                .trim_warm(&inp(sign * mid), warm.as_ref())
                .state()
                .copied()
            {
                lo = mid;
                warm = Some(st);
            } else {
                hi = mid;
            }
        }
    }
    sign * lo
}

/// Whether grid index `i` of `n` is a sensitivity sample node: every 2nd index, plus the last (so
/// the fill below always interpolates between two samples, never extrapolates past the edge).
fn sens_sampled(i: usize, n: usize) -> bool {
    i.is_multiple_of(2) || i + 1 == n
}

/// Fill the unsampled `â_x` nodes of the Decision #31 sensitivity fields by linear interpolation
/// along each fibre (sensitivities are sampled at every 2nd `â_x` node; `v` and `g_normal` keep
/// full resolution). Only bulk nodes — boundary ≥ the fibre's correction threshold — are filled;
/// the rest keep the suppressed 0, matching the full-grid semantics.
fn fill_sensitivities(
    mut fields: [&mut Vec<f64>; 3],
    base: &[f64],
    axn_axis: &[f64],
    [nv, nax, ngn]: [usize; 3],
) {
    let node = |iv: usize, iax: usize, ign: usize| (iv * nax + iax) * ngn + ign;
    for iv in 0..nv {
        for ign in 0..ngn {
            let peak = (0..nax).fold(0.0_f64, |m, iax| m.max(base[node(iv, iax, ign)]));
            let thresh = (CORR_MIN_FRAC * peak).max(AY_FLOOR);
            for iax in (0..nax).filter(|&i| !sens_sampled(i, nax)) {
                if base[node(iv, iax, ign)] < thresh {
                    continue;
                }
                // Bracketing sampled neighbours (indices 0 and nax−1 are always sampled).
                let lo = (0..iax).rev().find(|&i| sens_sampled(i, nax)).unwrap_or(0);
                let hi = ((iax + 1)..nax)
                    .find(|&i| sens_sampled(i, nax))
                    .unwrap_or(nax - 1);
                let w = (axn_axis[iax] - axn_axis[lo]) / (axn_axis[hi] - axn_axis[lo]);
                for f in &mut fields {
                    let (a, b) = (f[node(iv, lo, ign)], f[node(iv, hi, ign)]);
                    f[node(iv, iax, ign)] = a + w * (b - a);
                }
            }
        }
    }
}

/// Estimate the top speed for the `v` axis: the speed at which the powertrain drive force can no
/// longer overcome the reference aero drag, m/s (clamped to `[V_ENV_LO+ε, V_ENV_CAP]`). Approximate —
/// the `v` axis only needs to bracket the used speed range; queries beyond it clamp.
fn top_speed_estimate(car: &T1Vehicle) -> f64 {
    let mut v = V_ENV_LO + TOP_SPEED_DV;
    while v < V_ENV_CAP {
        let drive = car.max_tractive_force(v);
        let drag = car.qx * v * v; // reference drag (a map, if present, is close enough for the axis)
        if drive <= drag {
            return v;
        }
        v += TOP_SPEED_DV;
    }
    V_ENV_CAP
}

/// `n` evenly spaced points from `lo` to `hi` inclusive (`n ≥ 2`). Fixed-order, deterministic.
fn linspace(lo: f64, hi: f64, n: usize) -> Vec<f64> {
    debug_assert!(n >= 2, "linspace needs at least two points");
    debug_assert!(hi > lo, "linspace needs hi > lo");
    let step = (hi - lo) / (n as f64 - 1.0);
    (0..n).map(|i| lo + step * i as f64).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use outlap_schema::io::MemLoader;
    use outlap_schema::{load_vehicle, Conditions, LoadOptions};

    const SLICK: &str = include_str!("../../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

    /// Assemble a rear-driven downforce car from an in-memory fixture (constant aero — the envelope
    /// generator does not need a ride-height map to exercise the trim boundary search).
    fn sample_car() -> T1Vehicle {
        // Rev-limited at ≈12 000 rpm with a 9:1 ratio → a realistic ≈45 m/s top speed (so the v axis
        // does not span an implausible flat-torque range).
        let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
            axes: {speed_rpm: [0.0, 12000.0], load_axis: {torque_nm: [0.0, 800.0]}, torque_nm: [0.0, 800.0]}\n\
            tables: {file: x.parquet}\n\
            limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 12000.0], torque_nm: [800.0, 800.0]}}\n\
            inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
        let veh = "schema: vehicle/1.0\nname: t\n\
            chassis: {mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}\n\
            aero: {map: a.parquet, axes: [], constant: {cx_a_m2: 1.0, cz_front_a_m2: 1.5, cz_rear_a_m2: 3.0}}\n\
            suspension: {model: lumped_kc, front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}, rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}\n\
            tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}\n\
            drivetrain: {units: [{source: ptm/u.ptm.yaml, path: [{fixed_ratio: 9.0}], wheels: [RL, RR]}]}\n\
            brakes: {balance_bar: 0.6, disc: {front: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}, rear: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}}\n";
        let loader = MemLoader::new()
            .with("vehicle.yaml", veh)
            .with("ptm/u.ptm.yaml", ptm)
            .with("tyr/slick.tyr.yaml", SLICK);
        let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
        T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap()
    }

    fn small_res() -> EnvelopeRes {
        EnvelopeRes {
            v_points: 6,
            ax_points: 7,
            g_normal_points: 3,
        }
    }

    #[test]
    fn linspace_endpoints_and_count() {
        let xs = linspace(2.0, 8.0, 4);
        assert_eq!(xs.len(), 4);
        assert!((xs[0] - 2.0).abs() < 1e-12);
        assert!((xs[3] - 8.0).abs() < 1e-12);
        assert!((xs[1] - 4.0).abs() < 1e-12);
    }

    /// One generate, many structural assertions (generation is the expensive part — do it once).
    #[test]
    fn envelope_basic_properties() {
        let car = sample_car();
        let env = GgvEnvelope::generate(&car, &small_res(), FzCoupling::OneStepLag).unwrap();
        assert_eq!(env.shape(), [6, 7, 3]);
        let [(v_lo, v_hi), (axn_lo, axn_hi), (gn_lo, gn_hi)] = env.domain();
        // The â_x axis is the normalised [−1, 1].
        assert!((axn_lo + 1.0).abs() < 1e-9 && (axn_hi - 1.0).abs() < 1e-9);
        // g_normal spans [0.5g, 2g].
        assert!((gn_lo - 0.5 * G).abs() < 1e-9 && (gn_hi - 2.0 * G).abs() < 1e-9);
        // Longitudinal capability is positive and grows with normal load.
        let v_mid = 0.5 * (v_lo + v_hi);
        assert!(env.accel_limit(v_mid, G) > 1.0 && env.brake_limit(v_mid, G) > 1.0);
        assert!(
            env.brake_limit(v_mid, 2.0 * G) > env.brake_limit(v_mid, 0.5 * G),
            "more normal load should allow harder braking"
        );

        // Pure-lateral grip is positive at every sampled (v, g_normal) — no holes.
        for &fv in &[0.1, 0.5, 0.9] {
            for &fg in &[0.0, 0.5, 1.0] {
                let v = v_lo + fv * (v_hi - v_lo);
                let gn = gn_lo + fg * (gn_hi - gn_lo);
                assert!(env.ay_boundary(v, 0.0, gn) > 1.0, "hole at v={v}, gn={gn}");
            }
        }

        // Corrections are identity at the reference state.
        for &fv in &[0.25, 0.6] {
            for &fa in &[-6.0, 0.0, 6.0] {
                for &fg in &[0.25, 0.75] {
                    let v = v_lo + fv * (v_hi - v_lo);
                    let gn = gn_lo + fg * (gn_hi - gn_lo);
                    let base = env.ay_boundary(v, fa, gn);
                    let corr = env.ay_boundary_corrected(v, fa, gn, 1.0, env.mass_ref(), 1.0);
                    assert!(
                        (base - corr).abs() < 1e-12,
                        "corrections not identity: {base} vs {corr}"
                    );
                }
            }
        }

        // Correction signs: more grip ⇒ more lateral; more mass ⇒ less lateral (per unit mass).
        let base = env.ay_boundary(v_mid, 0.0, G);
        assert!(
            env.ay_boundary_corrected(v_mid, 0.0, G, 1.1, env.mass_ref(), 1.0) > base,
            "more grip should raise the boundary"
        );
        assert!(
            env.ay_boundary_corrected(v_mid, 0.0, G, 1.0, env.mass_ref() * 1.1, 1.0) < base,
            "more mass should lower the boundary"
        );

        // g_normal monotonicity: more road-normal load ⇒ no less absolute lateral grip.
        let mut prev = -1.0;
        for i in 0..=8 {
            let gn = gn_lo + (gn_hi - gn_lo) * f64::from(i) / 8.0;
            let ay = env.ay_boundary(v_mid, 0.0, gn);
            assert!(
                ay >= prev - 1e-6,
                "lateral grip fell as load rose: {ay} < {prev}"
            );
            prev = ay;
        }

        // Concavity of the a_y(a_x) section (the feasible g-g region is convex).
        let cap = env.accel_limit(v_mid, G).min(env.brake_limit(v_mid, G));
        for i in 1..24 {
            let a = |k: f64| -0.9 * cap + 1.8 * cap * k / 24.0;
            let (y0, y1, y2) = (
                env.ay_boundary(v_mid, a(f64::from(i) - 1.0), G),
                env.ay_boundary(v_mid, a(f64::from(i)), G),
                env.ay_boundary(v_mid, a(f64::from(i) + 1.0), G),
            );
            if y0 > AY_FLOOR && y1 > AY_FLOOR && y2 > AY_FLOOR {
                assert!(
                    y0 - 2.0 * y1 + y2 <= 0.25,
                    "a_y(a_x) not concave near a_x={}",
                    a(f64::from(i))
                );
            }
        }
    }

    /// Node-exactness, the Decision #31 off-reference accuracy gate (sampled at near-peak grid nodes
    /// so it isolates the correction linearisation from base-table interpolation), and base-table
    /// interpolation accuracy in the interior. Error is a fraction of the local peak lateral grip.
    #[test]
    #[allow(clippy::too_many_lines)] // one linear accuracy-sweep procedure; splitting it hurts clarity.
    fn envelope_accuracy_and_containment() {
        let car = sample_car();
        let coupling = FzCoupling::OneStepLag;
        // Reduced CI grid (the production default is 40×25×7); the interpolation metric is
        // grid-limited and shrinks markedly on the finer grid.
        let (nv, nax, ngn) = (8usize, 9usize, 3usize);
        let res = EnvelopeRes {
            v_points: nv as u32,
            ax_points: nax as u32,
            g_normal_points: ngn as u32,
        };
        let env = GgvEnvelope::generate(&car, &res, coupling).unwrap();
        let [(v_lo, v_hi), _, (gn_lo, gn_hi)] = env.domain();
        let lerp = |lo: f64, hi: f64, f: f64| lo + f * (hi - lo);
        let peak = |v: f64, gn: f64| env.ay_boundary(v, 0.0, gn).max(1.0);
        // Grid-node coordinates. â_x nodes are linspace(−1,1); the actual a_x is â_x·capability.
        let vn = |iv: usize| lerp(v_lo, v_hi, iv as f64 / (nv as f64 - 1.0));
        let gnn = |ign: usize| lerp(gn_lo, gn_hi, ign as f64 / (ngn as f64 - 1.0));
        let axn = |iax: usize| -1.0 + 2.0 * iax as f64 / (nax as f64 - 1.0);
        let ax_of = |axn: f64, v: f64, gn: f64| {
            axn * if axn >= 0.0 {
                env.accel_limit(v, gn)
            } else {
                env.brake_limit(v, gn)
            }
        };
        let i0 = (nax - 1) / 2; // â_x = 0

        // Node exactness: the interpolant reproduces the boundary finder at grid nodes.
        let mut max_node = 0.0_f64;
        for iv in [1, 4, 6] {
            for iax in [i0 - 1, i0, i0 + 1] {
                for ign in 0..ngn {
                    let (v, gn) = (vn(iv), gnn(ign));
                    let ax = ax_of(axn(iax), v, gn);
                    let truth = max_lateral(&car, v, ax, gn, coupling, None).ay_corr;
                    max_node =
                        max_node.max((env.ay_boundary(v, ax, gn) - truth).abs() / peak(v, gn));
                }
            }
        }
        println!(
            "node-exactness max error (fraction of peak): {:.3}%",
            max_node * 100.0
        );
        assert!(
            max_node < 0.02,
            "table not node-exact vs the boundary finder: {max_node:.4}"
        );

        // Decision #31 accuracy gate: corrected envelope vs full re-solve at off-reference states,
        // sampled at near-peak (small |â_x|) grid NODES so the base-table interpolation is exact and
        // the metric isolates the correction linearisation (grid-independent). Bands: ±15% μ, ±10%
        // mass, ±30% ClA (and combined). The correction is a lateral-grip magnitude model — accurate
        // near the cornering peak, magnitude-clamped toward the longitudinal shoulders (documented).
        let cases = [
            (1.15, 1.0, 1.0),
            (0.85, 1.0, 1.0),
            (1.0, 1.10, 1.0),
            (1.0, 1.0, 1.30),
            (1.0, 1.0, 0.70),
            (1.12, 1.08, 1.22),
        ];
        let mut gate = 0.0_f64;
        for &(mu, mm, cla) in &cases {
            let pcar = car
                .with_mu_scale(mu)
                .with_mass(car.mass_kg * mm)
                .with_cla_scale(cla);
            for iv in [1, 4, 6] {
                for ign in 0..ngn {
                    let (v, gn) = (vn(iv), gnn(ign));
                    // Pure lateral (â_x = 0, a_x = 0): the correction's canonical grip-scaling case,
                    // free of the velocity-frame −a_x·sinβ projection.
                    let truth = max_lateral(&pcar, v, 0.0, gn, coupling, None).ay_corr;
                    let interp = env.ay_boundary_corrected(v, 0.0, gn, mu, car.mass_kg * mm, cla);
                    gate = gate.max((interp - truth).abs() / peak(v, gn));
                }
            }
        }
        println!(
            "Decision #31 corrected-envelope max error vs full T1 re-solve (pure lateral): {:.3}%",
            gate * 100.0
        );
        assert!(
            gate < 0.02,
            "off-reference accuracy gate {gate:.4} exceeds 2% of peak"
        );

        // At MODERATE longitudinal acceleration (|â_x| = 0.4) the velocity-frame −a_x·sinβ term makes
        // the correction less accurate, but it must stay BOUNDED — it is a documented degradation, not
        // a blow-up. (The PR7 T0 lap consumes the base table, not the corrected form; this bounds the
        // public `ay_boundary_corrected` for the PR8 off-reference sweeps that will.)
        let mut mid_gate = 0.0_f64;
        for &(mu, mm, cla) in &[(1.15, 1.0, 1.0), (1.0, 1.0, 1.30), (0.9, 0.94, 0.8)] {
            let pcar = car
                .with_mu_scale(mu)
                .with_mass(car.mass_kg * mm)
                .with_cla_scale(cla);
            for iv in [2, 5] {
                for ign in 0..ngn {
                    let (v, gn) = (vn(iv), gnn(ign));
                    for &an in &[-0.4, 0.4] {
                        let ax = an
                            * if an >= 0.0 {
                                env.accel_limit(v, gn)
                            } else {
                                env.brake_limit(v, gn)
                            };
                        let truth = max_lateral(&pcar, v, ax, gn, coupling, None).ay_corr;
                        if truth < 0.4 * peak(v, gn) {
                            continue; // below the correction's valid bulk
                        }
                        let interp =
                            env.ay_boundary_corrected(v, ax, gn, mu, car.mass_kg * mm, cla);
                        mid_gate = mid_gate.max((interp - truth).abs() / peak(v, gn));
                    }
                }
            }
        }
        println!(
            "Decision #31 correction max error at |â_x|=0.4: {:.3}%",
            mid_gate * 100.0
        );
        assert!(
            mid_gate < 0.12,
            "moderate-a_x correction error {mid_gate:.4} exceeds the documented 12% bound"
        );

        // Base-table interpolation accuracy across the whole a_x range — bulk AND the steep
        // longitudinal shoulders (|â_x| near 1) the lap solver's ax_forward/ax_backward bisect right
        // up to. Error is a fraction of the local peak grip; the shoulders (where the boundary drops
        // to 0 within a cell) carry a looser but still bounded tolerance. Grid-limited on this coarse
        // CI grid; far finer on the production 40×25×7.
        let mut max_bulk = 0.0_f64;
        let mut max_shoulder = 0.0_f64;
        for &fv in &[0.23, 0.61, 0.88] {
            for &fg in &[0.2, 0.6] {
                let (v, gn) = (lerp(v_lo, v_hi, fv), lerp(gn_lo, gn_hi, fg));
                let cap_a = env.accel_limit(v, gn);
                let cap_b = env.brake_limit(v, gn);
                for &an in &[-0.85, -0.55, -0.15, 0.2, 0.5, 0.8] {
                    let ax = an * if an >= 0.0 { cap_a } else { cap_b };
                    let truth = max_lateral(&car, v, ax, gn, coupling, None).ay_corr;
                    let err = (env.ay_boundary(v, ax, gn) - truth).abs() / peak(v, gn);
                    // Interior |â_x| ≤ 0.6 is the bulk; |â_x| ≥ 0.7 is the steep shoulder region.
                    if an.abs() <= 0.6 {
                        max_bulk = max_bulk.max(err);
                    } else {
                        max_shoulder = max_shoulder.max(err);
                    }
                }
            }
        }
        println!(
            "base-table interpolation max error (fraction of peak, {nv}×{nax}×{ngn} CI grid): bulk {:.2}%, shoulder {:.2}%",
            max_bulk * 100.0,
            max_shoulder * 100.0
        );
        assert!(
            max_bulk < 0.07,
            "bulk interpolation error {max_bulk:.4} exceeds 7% of peak"
        );
        assert!(
            max_shoulder < 0.20,
            "shoulder interpolation error {max_shoulder:.4} exceeds 20% of peak"
        );
    }
}
