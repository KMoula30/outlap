// SPDX-License-Identifier: AGPL-3.0-only
//! Tire thermal ring — a reduced Farroni-TRT lumped-node model (HANDOFF §7.2, FLAGSHIP) plus the
//! §7.3 two-state wear / thermal-damage law that rides on it.
//!
//! A tire's grip, pressure, and carcass stiffness all move with temperature, and temperature moves
//! over a stint — so a stint-honest simulator has to carry the tire's thermal state segment to
//! segment. This module is that state: three lumped nodes per tire, advanced by a pure, alloc-free,
//! `wasm`-clean [`TireThermalRing::step`], plus the three couplings that feed back into the force
//! model. **No tier wiring lives here** (that is PR3/PR5); the ring is proven in isolation.
//!
//! # Wear and thermal damage (§7.3)
//!
//! Two slow states are advanced *in the same [`step`](TireThermalRing::step)* as the temperatures,
//! because they couple back into the thermal ring:
//!
//! - **`w`** — accumulated tread **wear** depth, mm, growing from `0` (new) toward `w_max` (bald) by
//!   an Archard-type sliding-energy law whose rate rises with surface temperature (Grosch: hotter
//!   rubber is softer, wears faster). Two effects: (1) it drives the grip *cliff* through a C¹
//!   sigmoid, and (2) it shrinks the surface node's thermal capacity `C_s(w)` — less tread mass — so
//!   a worn tire has less thermal inertia and its surface tracks the load peaks more closely, running
//!   *hotter* under corner loading. That is the positive-feedback cliff mechanism: worn → hotter →
//!   (off-optimum grip window, faster wear) → more worn.
//! - **`D ∈ [0,1]`** — irreversible thermal **damage**: overheated carcass rubber reverts/hardens and
//!   never recovers. It accumulates whenever `T_c` exceeds a degradation threshold `T_deg` and only
//!   ever grows.
//!
//! The total grip multiplier the ring hands the force model is the product of the thermal window and
//! the two degradation factors:
//!
//! ```text
//! λ_μ,total = λ_μ(T_s) · (1 − Δ_c·σ((w−w_c)/s_w)) · (1 − Δ_D·D)
//! ```
//!
//! **Reduction note (shipped `TyrWear` contract).** §7.3 also lists a separate linear pre-cliff term
//! `f_w = 1 − c_w1·(w/w_max)`. The shipped `.tyr` wire contract carries *no* `c_w1`: the gradual
//! pre-cliff pace loss and the cliff are unified into the single C¹ sigmoid `(1 − Δ_c·σ(·))` (which
//! is monotone-decreasing in `w` for *all* `w`, so grip erodes gradually below `w_c` and collapses
//! across it), and the irreversible component is carried by `(1 − Δ_D·D)`. This keeps the wear
//! parameter set to exactly the §7.3 headline coefficients. See `docs/theory/tire-wear.md`.
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
use outlap_schema::tyr::{TyrThermal, TyrWear};

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

/// Wear-rate temperature sensitivity `c_H`, per °C (Grosch): the inverse hardness `1/H(T_s)` rises as
/// `exp(c_H·(T_s − T_opt))`, so wear roughly e-folds per ~50 °C of surface temperature above the
/// grip optimum. Fixed here (not a `.tyr` field): the §7.3 `TyrWear` block calibrates the wear
/// *magnitude* through `k_w`; this sets only the *shape* of the temperature dependence. The optimum
/// grip temperature `T_opt` is the hardness reference — the compound is characterised at its window.
const WEAR_HARDNESS_SENS_PER_C: f64 = 0.02;

/// Upper clamp on the inverse-hardness factor `1/H(T_s)` so a pathologically hot surface cannot make
/// the wear rate blow up (keeps the kernel finite — CLAUDE.md: solver kernels panic-free/bounded).
const WEAR_INV_HARDNESS_MAX: f64 = 20.0;

/// Fraction of the fresh surface-node capacity a fully bald tire retains: `C_s(w=w_max) =
/// c_s·CS_WEAR_FLOOR`. The surface node is the tread layer, so its capacity scales with remaining
/// tread depth `(1 − w/w_max)`, but a floor keeps the belt/base contribution — the node never goes
/// mass-less (which would make the time constant vanish). A modelling constant, documented in
/// `docs/theory/tire-wear.md`.
const CS_WEAR_FLOOR: f64 = 0.4;

/// Smallest contact-patch area the wear law divides by, m² — floors `A_cp` so `Q_fric/A_cp` stays
/// finite when the patch fraction is driven to zero (kernel stays panic-free).
const CONTACT_AREA_FLOOR_M2: f64 = 1e-4;

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
    /// Accumulated tread **wear** depth `w`, mm — `0` when new, growing toward `w_max` (bald). Only
    /// ever increases (§7.3). Drives the grip cliff and shrinks the surface capacity `C_s(w)`.
    pub wear_mm: T,
    /// Irreversible thermal **damage** `D ∈ [0,1]` (§7.3): `0` undamaged, `1` fully degraded. Only
    /// ever increases; cooling never repairs it.
    pub damage: T,
}

impl<T: Float> TireThermalState<T> {
    /// A fresh (zero-wear, undamaged) state with all three nodes at the same temperature (K) — the
    /// usual cold start, every node equilibrated to ambient/track before the first segment.
    pub fn uniform(t_k: T) -> Self {
        Self {
            t_s_k: t_k,
            t_c_k: t_k,
            t_g_k: t_k,
            wear_mm: T::zero(),
            damage: T::zero(),
        }
    }

    /// A fresh (zero-wear, undamaged) state with explicit per-node temperatures (K).
    pub fn new(t_s_k: T, t_c_k: T, t_g_k: T) -> Self {
        Self {
            t_s_k,
            t_c_k,
            t_g_k,
            wear_mm: T::zero(),
            damage: T::zero(),
        }
    }

    /// A state with explicit per-node temperatures (K) and an explicit wear (mm) / damage (0..1) —
    /// e.g. seeding lap N+1 from lap N's terminal state, or testing a partly-worn tire.
    pub fn with_wear(t_s_k: T, t_c_k: T, t_g_k: T, wear_mm: T, damage: T) -> Self {
        Self {
            t_s_k,
            t_c_k,
            t_g_k,
            wear_mm,
            damage,
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
    /// Thermal grip-window multiplier `λ_μ(T_s) ∈ (0, 1]`, peaking at `1` at `T_opt` — the *thermal*
    /// factor only (no wear/damage), unchanged from the isolated-ring PR. Scales `LMUX`/`LMUY`.
    pub mu_scale: T,
    /// Carcass-softening stiffness multiplier `(1 − k_c·(T_c−T_c,ref))` — scales `PKX1`/`PKY1`.
    pub stiffness_scale: T,
    /// Wear grip factor `(1 − Δ_c·σ((w−w_c)/s_w)) ∈ [1−Δ_c, 1]` — the C¹ cliff (§7.3). `1` when new,
    /// collapsing toward `1−Δ_c` past the critical wear `w_c`.
    pub wear_grip_scale: T,
    /// Thermal-damage grip factor `(1 − Δ_D·D) ∈ [1−Δ_D, 1]` — the irreversible loss (§7.3).
    pub damage_grip_scale: T,
    /// Total grip multiplier `λ_μ,total = mu_scale · wear_grip_scale · damage_grip_scale` — the value
    /// the force model actually scales `LMUX`/`LMUY` by once wear wiring lands (PR3/PR5).
    pub mu_scale_total: T,
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
    // Wear / thermal-damage (§7.3). Inert (all-zero effects) when built via `from_schema`.
    k_w: T,
    w_max_mm: T,
    w_c_mm: T,
    s_w_mm: T,
    delta_c: T,
    tau_d_s: T,
    t_deg_c: T,
    delta_t_ref: T,
    beta: T,
    delta_d: T,
    // Precomputed constants (typed once).
    wear_sens_per_c: T,
    inv_hardness_max: T,
    cs_wear_floor: T,
    contact_area_floor: T,
}

/// An inert wear block: `k_w = 0` (no material removed), `Δ_c = Δ_D = 0` (no grip loss),
/// `τ_D → ∞`/`T_deg → ∞` (no damage). A ring built from it advances temperatures exactly as the
/// isolated §7.2 ring did — used by [`TireThermalRing::from_schema`] so the thermal-only path is
/// bit-identical to the pre-wear behaviour.
fn inert_wear() -> TyrWear {
    TyrWear {
        k_w: 0.0,
        w_max: 1.0, // avoids a divide-by-zero in C_s(w); w stays 0 so the ratio is 0 anyway.
        w_c: 1.0,
        tau_d: 1.0,
        t_deg: 1.0e9,
        delta_t_ref: 1.0,
        beta: 1.0,
        delta_c: 0.0,
        s_w: 1.0,
        delta_d: 0.0,
    }
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
        Self::from_schema_with_wear(th, &inert_wear())
    }

    /// Build a wear-capable ring from a validated [`TyrThermal`] + [`TyrWear`] block. The thermal
    /// side is identical to [`from_schema`](Self::from_schema); the wear side wires the §7.3 states.
    ///
    /// # Panics (debug only)
    /// As [`from_schema`](Self::from_schema), plus debug-asserts the wear scales `Δ_c, Δ_D ∈ [0,1]`,
    /// `w_max > 0`, and `τ_D > 0` (physics invariants). In release the kernel floors these to stay
    /// panic-free on an unvalidated document.
    pub fn from_schema_with_wear(th: &TyrThermal, wr: &TyrWear) -> Self {
        debug_assert!(
            th.c_s > 0.0 && th.c_c > 0.0 && th.c_g > 0.0,
            "thermal capacities must be strictly positive"
        );
        debug_assert!(
            (0.0..=1.0).contains(&th.p_t),
            "friction partition p_t must lie in [0,1]"
        );
        debug_assert!(
            (0.0..=1.0).contains(&wr.delta_c) && (0.0..=1.0).contains(&wr.delta_d),
            "cliff/damage grip drops Δ_c, Δ_D must lie in [0,1]"
        );
        debug_assert!(
            wr.w_max > 0.0 && wr.tau_d > 0.0,
            "w_max and τ_D must be strictly positive"
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
            k_w: cvt(wr.k_w).max(T::zero()),
            w_max_mm: cvt(wr.w_max).max(cvt(CAP_FLOOR_J_PER_K)),
            w_c_mm: cvt(wr.w_c),
            s_w_mm: cvt(wr.s_w).max(floor),
            delta_c: cvt(wr.delta_c).max(T::zero()).min(T::one()),
            tau_d_s: cvt(wr.tau_d).max(floor),
            t_deg_c: cvt(wr.t_deg),
            delta_t_ref: cvt(wr.delta_t_ref).max(floor),
            beta: cvt(wr.beta).max(T::zero()),
            delta_d: cvt(wr.delta_d).max(T::zero()).min(T::one()),
            wear_sens_per_c: cvt(WEAR_HARDNESS_SENS_PER_C),
            inv_hardness_max: cvt(WEAR_INV_HARDNESS_MAX),
            cs_wear_floor: cvt(CS_WEAR_FLOOR),
            contact_area_floor: cvt(CONTACT_AREA_FLOOR_M2),
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

    /// Effective surface-node capacity `C_s(w) = c_s·max(1 − w/w_max, floor)`, J/K (§7.3). A worn
    /// tire has less tread mass, so less surface thermal inertia — the positive-feedback mechanism.
    #[inline]
    pub fn c_s_effective(&self, wear_mm: T) -> T {
        let one = T::one();
        let ratio = (wear_mm / self.w_max_mm).max(T::zero()).min(one);
        self.c_s * (one - ratio).max(self.cs_wear_floor)
    }

    /// Inverse hardness `1/H(T_s) = min(exp(c_H·(T_s − T_opt)), cap)` (Grosch): softer, faster-wearing
    /// rubber as the surface heats above the grip optimum. Dimensionless, ≥ 0.
    #[inline]
    pub fn inv_hardness(&self, t_s_k: T) -> T {
        let celsius = T::from(CELSIUS_K).unwrap_or_else(T::zero);
        let ts_c = t_s_k - celsius;
        (self.wear_sens_per_c * (ts_c - self.t_opt_c))
            .exp()
            .min(self.inv_hardness_max)
    }

    /// Archard wear rate `dw/dt = k_w · (1/H(T_s)) · Q_fric / A_cp`, mm/s (§7.3). Non-negative, so `w`
    /// only ever grows. `A_cp = a_cp·A_ext` (floored) is the contact-patch area.
    #[inline]
    pub fn wear_rate(&self, t_s_k: T, slip_power_w: T, contact_area_m2: T) -> T {
        let a_cp = contact_area_m2.max(self.contact_area_floor);
        self.k_w * self.inv_hardness(t_s_k) * self.q_fric(slip_power_w) / a_cp
    }

    /// Thermal-damage rate `dD/dt = (1/τ_D)·⟨(T_c − T_deg)/ΔT_ref⟩₊^β`, 1/s (§7.3). Non-negative, so
    /// `D` only ever grows; zero unless the carcass exceeds the degradation threshold `T_deg`.
    #[inline]
    pub fn damage_rate(&self, t_c_k: T) -> T {
        let celsius = T::from(CELSIUS_K).unwrap_or_else(T::zero);
        let tc_c = t_c_k - celsius;
        let over = ((tc_c - self.t_deg_c) / self.delta_t_ref).max(T::zero());
        if over <= T::zero() {
            T::zero()
        } else {
            over.powf(self.beta) / self.tau_d_s
        }
    }

    /// Wear grip factor `(1 − Δ_c·σ((w−w_c)/s_w))` — the C¹ cliff (§7.3). Monotone-decreasing in `w`.
    #[inline]
    pub fn wear_grip_scale(&self, wear_mm: T) -> T {
        let z = (wear_mm - self.w_c_mm) / self.s_w_mm;
        let sigmoid = T::one() / (T::one() + (-z).exp());
        T::one() - self.delta_c * sigmoid
    }

    /// Thermal-damage grip factor `(1 − Δ_D·D)` (§7.3).
    #[inline]
    pub fn damage_grip_scale(&self, damage: T) -> T {
        T::one() - self.delta_d * damage.max(T::zero()).min(T::one())
    }

    /// Advance the state by `dt` (s) under `drivers` and return the resulting force-model
    /// [`ThermalCouplings`] evaluated at the **updated** state (the couplings feed the *next*
    /// fast-step force evaluation).
    ///
    /// One step advances the three temperatures (semi-implicit Euler, Jacobi over nodes) **and** the
    /// two §7.3 slow states — wear `w` and damage `D` — together, because the wear couples back into
    /// the surface capacity `C_s(w)`. Wear and damage are pure accumulators (no decay term), so an
    /// explicit forward Euler on them is monotone and cannot overshoot; both read the start-of-step
    /// temperatures, keeping the whole update order-independent and bit-identical on re-run.
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

        // Start-of-step state (Jacobi: every node reads these, none reads a fresh neighbour).
        let (ts, tc, tg, w) = (st.t_s_k, st.t_c_k, st.t_g_k, st.wear_mm);

        // Surface: fed by Q_fric, exchanges with carcass, air, and road. Capacity shrinks with wear.
        let c_s_eff = self.c_s_effective(w);
        let g_s = self.g_sc + g_air + g_rd;
        let src_s =
            self.q_fric(d.slip_power_w) + self.g_sc * tc + g_air * d.t_air_k + g_rd * d.t_road_k;
        let ts_new = semi_implicit_decay(ts, g_s / c_s_eff, src_s / c_s_eff, dt);

        // Carcass: fed by Q_hyst, exchanges with surface and gas.
        let g_c = self.g_sc + self.g_cg;
        let src_c = d.carcass_loss_w + self.g_sc * ts + self.g_cg * tg;
        let tc_new = semi_implicit_decay(tc, g_c / self.c_c, src_c / self.c_c, dt);

        // Gas: exchanges only with the carcass.
        let src_g = self.g_cg * tc;
        let tg_new = semi_implicit_decay(tg, self.g_cg / self.c_g, src_g / self.c_g, dt);

        // Wear + damage: explicit forward Euler on the start-of-step temperatures, clamped monotone.
        let contact_area = (d.contact_fraction.max(T::zero()) * d.ext_area_m2).max(T::zero());
        let w_new = (w + self.wear_rate(ts, d.slip_power_w, contact_area) * dt)
            .max(w)
            .min(self.w_max_mm);
        let d_new = (st.damage + self.damage_rate(tc) * dt)
            .max(st.damage)
            .min(T::one());

        st.t_s_k = ts_new;
        st.t_c_k = tc_new;
        st.t_g_k = tg_new;
        st.wear_mm = w_new;
        st.damage = d_new;
        self.couplings(st)
    }

    /// The force-model couplings at a given state, without advancing it (§7.2 couplings 1–3 plus the
    /// §7.3 wear/damage grip factors).
    pub fn couplings(&self, st: &TireThermalState<T>) -> ThermalCouplings<T> {
        let mu_scale = self.mu_scale(st);
        let wear_grip_scale = self.wear_grip_scale(st.wear_mm);
        let damage_grip_scale = self.damage_grip_scale(st.damage);
        ThermalCouplings {
            pressure_pa: self.pressure_pa(st),
            mu_scale,
            stiffness_scale: self.stiffness_scale(st),
            wear_grip_scale,
            damage_grip_scale,
            mu_scale_total: mu_scale * wear_grip_scale * damage_grip_scale,
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
