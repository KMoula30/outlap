// SPDX-License-Identifier: AGPL-3.0-only
//! Tire thermal ring — a reduced Farroni-TRT lumped-node model (HANDOFF §7.2, FLAGSHIP).
//!
//! A tire's grip, pressure, and carcass stiffness all move with temperature, and temperature moves
//! over a stint — so a stint-honest simulator has to carry the tire's thermal state segment to
//! segment. This module is that state: three lumped nodes per tire, advanced by a pure, alloc-free,
//! `wasm`-clean [`TireThermalRing::step`], plus the three couplings that feed back into the force
//! model. **No tier wiring lives here** (that is PR3/PR5); the ring is proven in isolation.
//!
//! # The three nodes (§7.2)
//!
//! - **`T_s`** — the tread *surface*: fast, driven directly by the frictional sliding power and
//!   cooled by convection to the air and conduction to the road through the contact patch.
//! - **`T_c`** — the tread bulk / *carcass*: slower, fed by hysteresis (rolling deformation) loss
//!   and exchanging heat with the surface above and the inflation gas below.
//! - **`T_g`** — the inflation *gas*: the slowest, coupled only to the carcass; its temperature sets
//!   the hot inflation pressure through the ideal-gas law.
//!
//! ```text
//! C_s·dT_s/dt = Q_fric − G_sc(T_s−T_c) − g_conv(v)·(1−a_cp)·(T_s−T_air) − G_road·a_cp·(T_s−T_road)
//! C_c·dT_c/dt = Q_hyst + G_sc(T_s−T_c) − G_cg(T_c−T_g)
//! C_g·dT_g/dt = G_cg(T_c−T_g)
//! ```
//!
//! with `Q_fric = p_t · P_slide` (the surface takes a fraction `p_t ≈ 0.6–0.7` of the sliding power,
//! the rest goes into the road) and `g_conv(v) = (h0 + h1·v^0.8)·A_ext` the forced-convection
//! conductance (§7.2, Reynolds-number `v^0.8` scaling). The `§7.2` rim term `−G_gr(T_g−T_rim)` is
//! dropped: the schema carries no rim conductance or rim temperature, so this is the reduced 3-node
//! ring in which the gas equilibrates to the carcass (`§7.2` lists the rim node as optional).
//!
//! # Discretization (§11.2)
//!
//! Each node is advanced with **semi-implicit Euler**: its own out-conductance (the diagonal decay
//! term) is taken implicitly and the neighbour/boundary temperatures are held at the start-of-step
//! value (a Jacobi sweep). This is the shared [`outlap_core::relax::semi_implicit_decay`] primitive
//! the battery temperature node also uses — A-stable for any step, so the coarse slow-clock step of a
//! lap cannot ring or overshoot, and order-independent, so the update is deterministic. The discrete
//! fixed point equals the continuous one exactly, so a steady-state energy balance closes to
//! round-off (the property tests check this).
//!
//! # Couplings back to the force model (§7.2 — computed here, wired in PR3/PR5)
//!
//! 1. **Gas-law pressure** `p = p_cold · T_g/T_cold` (absolute temperatures) → `SlipState::p`.
//! 2. **Grip window** `λ_μ(T_s) = exp(−c_T·((T_s−T_opt)/T_opt)²)`, peaking at `1` at `T_opt` →
//!    scales `LMUX`/`LMUY` (isotropic). The deviation is normalised by `T_opt` **in °C**, the
//!    calibration convention the parameter is authored in; node temperatures are stored in kelvin
//!    (SI-internal) and converted at this boundary only.
//! 3. **Carcass softening** `(1 − k_c·(T_c−T_c,ref))` → scales the carcass stiffnesses `PKX1`/`PKY1`.
//!
//! # Clean-room provenance
//!
//! The reduced multi-node ring, the `v^0.8` forced-convection law, and the Gaussian grip window are
//! implemented from the published tire-thermal literature — Farroni et al.'s *Thermo Racing Tyre*
//! (TRT) and TRT-EVO papers and the standard ideal-gas inflation relation — not derived from any
//! other codebase. See `docs/theory/tire-thermal.md` for the equation map and full citations.

use num_traits::Float;
use outlap_schema::tyr::TyrThermal;

use outlap_core::relax::semi_implicit_decay;

/// Zero of the Celsius scale, in kelvin. Node state is kelvin (SI-internal); the schema authors
/// `t_opt` / `t_c_ref` / `t_cold` in °C, converted once at construction.
const CELSIUS_K: f64 = 273.15;

/// Forced-convection velocity exponent in `h(v) = h0 + h1·v^0.8` (§7.2; turbulent-plate scaling).
const CONV_EXP: f64 = 0.8;

/// Smallest thermal capacity the ring will integrate with, J/K. Capacities must be strictly
/// positive physically; the schema semantic stage does not yet enforce it, so the kernel floors to
/// stay panic-free (solver kernels are `Result`/panic-free, CLAUDE.md).
const CAP_FLOOR_J_PER_K: f64 = 1e-6;

/// Lower clamp on the carcass-softening stiffness factor. The linear `1 − k_c·ΔT` form is faithful
/// to §7.2 within the operating range; the floor only guards a non-physical sign flip at absurd
/// carcass temperatures (with typical `k_c` it never binds below ~700 °C).
const STIFFNESS_FLOOR: f64 = 0.05;

/// The three lumped node temperatures of one tire, in **kelvin** (SI-internal, CLAUDE.md).
///
/// This is the integrated state; [`TireThermalRing`] holds the (shared, per-compound) parameters and
/// advances a state through [`TireThermalRing::step`]. Separating state from parameters lets four
/// wheels share one ring model with four states (SoA-friendly).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TireThermalState<T> {
    /// Tread-surface temperature `T_s`, K.
    pub t_s_k: T,
    /// Carcass / tread-bulk temperature `T_c`, K.
    pub t_c_k: T,
    /// Inflation-gas temperature `T_g`, K.
    pub t_g_k: T,
}

impl<T: Float> TireThermalState<T> {
    /// A state with all three nodes at the same temperature (K) — the usual cold start, every node
    /// equilibrated to ambient/track before the first segment.
    pub fn uniform(t_k: T) -> Self {
        Self {
            t_s_k: t_k,
            t_c_k: t_k,
            t_g_k: t_k,
        }
    }

    /// A state with explicit per-node temperatures (K).
    pub fn new(t_s_k: T, t_c_k: T, t_g_k: T) -> Self {
        Self {
            t_s_k,
            t_c_k,
            t_g_k,
        }
    }
}

/// Per-step operating point driving the ring: the exogenous heat inputs and boundary temperatures
/// the tier supplies from the current force solution and the environment.
///
/// These are *not* tire material parameters (those live in [`TireThermalRing`]) — they change every
/// step with load, slip, and speed, so the ring stays a pure function of `(state, params, drivers,
/// dt)` and is trivially testable in isolation. Geometry (`ext_area_m2`) and load-dependent
/// contact geometry (`contact_fraction`) enter here because the ring itself carries no geometry
/// (`wasm`-clean; geometry lives with the force model).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalDrivers<T> {
    /// Frictional sliding power at the contact patch `P_slide = |Fx·v_sx| + |Fy·v_sy|`, W. The ring
    /// deposits the fraction `p_t` into the surface node (`Q_fric`); the rest heats the road.
    pub slip_power_w: T,
    /// Hysteresis / rolling-deformation loss deposited in the carcass, W (`Q_hyst`). The caller
    /// forms it from the force model (`c_h·Fz·δ·Ω`); the ring treats it as an input.
    pub carcass_loss_w: T,
    /// Road/hub forward speed `v`, m/s — sets the forced-convection conductance `g_conv(v)`.
    pub speed_mps: T,
    /// Contact-patch fraction `a_cp = A_cp/A_ext ∈ [0,1]`: the patch conducts to the road, the rest
    /// of the surface convects to the air.
    pub contact_fraction: T,
    /// External (convecting) tread area `A_ext`, m² — the geometric area behind `h(v)`.
    pub ext_area_m2: T,
    /// Ambient air temperature `T_air`, K (from `conditions.yaml`).
    pub t_air_k: T,
    /// Road-surface temperature `T_road`, K (`conditions.track_surface_C`, §7.2 boundary).
    pub t_road_k: T,
}

/// The three force-model couplings the ring exposes each step (§7.2). All dimensionless except
/// [`pressure_pa`](Self::pressure_pa).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalCouplings<T> {
    /// Hot inflation pressure `p = p_cold·T_g/T_cold`, **Pa** — feeds `SlipState::p`.
    pub pressure_pa: T,
    /// Thermal grip-window multiplier `λ_μ(T_s) ∈ (0, 1]`, peaking at `1` at `T_opt` — scales
    /// `LMUX`/`LMUY` (isotropic; the asymmetric cold/hot option is a future extension).
    pub mu_scale: T,
    /// Carcass-softening stiffness multiplier `(1 − k_c·(T_c−T_c,ref))` — scales `PKX1`/`PKY1`.
    pub stiffness_scale: T,
}

/// A tire thermal ring: the (per-compound) parameters plus the pure integrator over a
/// [`TireThermalState`]. Cheap to clone; every accessor is allocation-free.
#[derive(Clone, Debug)]
pub struct TireThermalRing<T> {
    // Capacities, J/K (floored strictly positive).
    c_s: T,
    c_c: T,
    c_g: T,
    // Solid-path conductances, W/K.
    g_sc: T,
    g_cg: T,
    g_road: T,
    // Convection h(v) = h0 + h1·v^0.8, W/(m²·K).
    h0: T,
    h1: T,
    // Friction-power partition into the surface node, 0..1.
    p_t: T,
    // Grip window: optimum (°C) and width.
    t_opt_c: T,
    c_t: T,
    // Carcass softening: coefficient and reference (°C).
    k_c: T,
    t_c_ref_c: T,
    // Gas law: cold pressure (Pa) and cold reference temperature (K).
    p_cold_pa: T,
    t_cold_k: T,
}

impl<T: Float> TireThermalRing<T> {
    /// Build a ring from a validated [`TyrThermal`] block, converting units once (°C → K for the
    /// grip/softening/gas references, kPa → Pa for the cold pressure) and flooring the capacities
    /// strictly positive.
    ///
    /// # Panics (debug only)
    /// Debug-asserts the capacities are strictly positive and `p_t ∈ [0,1]` — physics invariants
    /// (CLAUDE.md). In release the capacities are floored at [`CAP_FLOOR_J_PER_K`] so the kernel
    /// stays panic-free even on an unvalidated document.
    pub fn from_schema(th: &TyrThermal) -> Self {
        debug_assert!(
            th.c_s > 0.0 && th.c_c > 0.0 && th.c_g > 0.0,
            "thermal capacities must be strictly positive"
        );
        debug_assert!(
            (0.0..=1.0).contains(&th.p_t),
            "friction partition p_t must lie in [0,1]"
        );
        let cvt = |x: f64| T::from(x).unwrap_or_else(T::zero);
        let floor = cvt(CAP_FLOOR_J_PER_K);
        Self {
            c_s: cvt(th.c_s).max(floor),
            c_c: cvt(th.c_c).max(floor),
            c_g: cvt(th.c_g).max(floor),
            g_sc: cvt(th.g_sc).max(T::zero()),
            g_cg: cvt(th.g_cg).max(T::zero()),
            g_road: cvt(th.g_road).max(T::zero()),
            h0: cvt(th.h0).max(T::zero()),
            h1: cvt(th.h1).max(T::zero()),
            p_t: cvt(th.p_t),
            t_opt_c: cvt(th.t_opt),
            c_t: cvt(th.c_t).max(T::zero()),
            k_c: cvt(th.k_c),
            t_c_ref_c: cvt(th.t_c_ref),
            p_cold_pa: cvt(th.p_cold * 1000.0), // kPa → Pa
            t_cold_k: cvt(th.t_cold + CELSIUS_K),
        }
    }

    /// Frictional power `Q_fric = p_t·P_slide` deposited into the surface node, W.
    #[inline]
    pub fn q_fric(&self, slip_power_w: T) -> T {
        self.p_t * slip_power_w
    }

    /// Forced-convection conductance `g_conv(v) = (h0 + h1·v^0.8)·A_ext`, W/K, at speed `v` (m/s) and
    /// external area `A_ext` (m²). Monotone non-decreasing in `v`.
    #[inline]
    pub fn conv_conductance(&self, speed_mps: T, ext_area_m2: T) -> T {
        let v = speed_mps.max(T::zero());
        let exp = T::from(CONV_EXP).unwrap_or_else(T::one);
        (self.h0 + self.h1 * v.powf(exp)) * ext_area_m2
    }

    /// Advance the state by `dt` (s) under `drivers` (semi-implicit Euler, Jacobi over nodes) and
    /// return the resulting force-model [`ThermalCouplings`] evaluated at the **updated** state (the
    /// couplings feed the *next* fast-step force evaluation).
    ///
    /// Allocation-free and deterministic: all three node updates read the start-of-step neighbour
    /// temperatures, so the result is independent of node order and bit-identical on re-run.
    pub fn step(
        &self,
        st: &mut TireThermalState<T>,
        d: &ThermalDrivers<T>,
        dt: T,
    ) -> ThermalCouplings<T> {
        let one = T::one();
        let a_cp = d.contact_fraction.max(T::zero()).min(one);
        let g_conv = self.conv_conductance(d.speed_mps, d.ext_area_m2);
        let g_air = g_conv * (one - a_cp);
        let g_rd = self.g_road * a_cp;

        // Start-of-step temperatures (Jacobi: every node reads these, none reads a fresh neighbour).
        let (ts, tc, tg) = (st.t_s_k, st.t_c_k, st.t_g_k);

        // Surface: fed by Q_fric, exchanges with carcass, air, and road.
        let g_s = self.g_sc + g_air + g_rd;
        let src_s =
            self.q_fric(d.slip_power_w) + self.g_sc * tc + g_air * d.t_air_k + g_rd * d.t_road_k;
        let ts_new = semi_implicit_decay(ts, g_s / self.c_s, src_s / self.c_s, dt);

        // Carcass: fed by Q_hyst, exchanges with surface and gas.
        let g_c = self.g_sc + self.g_cg;
        let src_c = d.carcass_loss_w + self.g_sc * ts + self.g_cg * tg;
        let tc_new = semi_implicit_decay(tc, g_c / self.c_c, src_c / self.c_c, dt);

        // Gas: exchanges only with the carcass.
        let src_g = self.g_cg * tc;
        let tg_new = semi_implicit_decay(tg, self.g_cg / self.c_g, src_g / self.c_g, dt);

        st.t_s_k = ts_new;
        st.t_c_k = tc_new;
        st.t_g_k = tg_new;
        self.couplings(st)
    }

    /// The force-model couplings at a given state, without advancing it (§7.2, coupling 1–3).
    pub fn couplings(&self, st: &TireThermalState<T>) -> ThermalCouplings<T> {
        ThermalCouplings {
            pressure_pa: self.pressure_pa(st),
            mu_scale: self.mu_scale(st),
            stiffness_scale: self.stiffness_scale(st),
        }
    }

    /// Gas-law hot pressure `p = p_cold·T_g/T_cold`, Pa (absolute temperatures).
    #[inline]
    pub fn pressure_pa(&self, st: &TireThermalState<T>) -> T {
        self.p_cold_pa * (st.t_g_k / self.t_cold_k)
    }

    /// Grip-window multiplier `λ_μ(T_s) = exp(−c_T·((T_s−T_opt)/T_opt)²) ∈ (0, 1]`.
    ///
    /// The deviation is normalised by `T_opt` **in °C** (the parameter's authored scale); the state
    /// is kelvin and converted here only. Peaks at exactly `1` when `T_s = T_opt`.
    #[inline]
    pub fn mu_scale(&self, st: &TireThermalState<T>) -> T {
        let celsius = T::from(CELSIUS_K).unwrap_or_else(T::zero);
        let ts_c = st.t_s_k - celsius;
        let dev = (ts_c - self.t_opt_c) / self.t_opt_c;
        (-self.c_t * dev * dev).exp()
    }

    /// Carcass-softening stiffness multiplier `(1 − k_c·(T_c−T_c,ref))`, clamped strictly positive.
    #[inline]
    pub fn stiffness_scale(&self, st: &TireThermalState<T>) -> T {
        let celsius = T::from(CELSIUS_K).unwrap_or_else(T::zero);
        let tc_c = st.t_c_k - celsius;
        let raw = T::one() - self.k_c * (tc_c - self.t_c_ref_c);
        raw.max(T::from(STIFFNESS_FLOOR).unwrap_or_else(T::zero))
    }
}
