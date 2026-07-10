// SPDX-License-Identifier: AGPL-3.0-only
//! Battery Thevenin equivalent-circuit pack + its per-segment slow states (§8.4) and the Vdc–SoC
//! coupling terminal-voltage surface (§8.4, user decision 2026-07-05).
//!
//! A pack is the schema-layer [`BatteryDoc`](outlap_schema::battery::BatteryDoc) plus its decoded
//! ECM tables ([`GriddedTable`]) assembled into a runnable [`Pack`]. The live state
//! ([`PackState`]) carries three per-segment slow states advanced alongside PR5's machine
//! temperatures at the same lap-loop hook (wired in PR8):
//!
//! * **SoC** — Coulomb-counted, `ΔSoC = −I_pack·dt / (3600·Q_pack)`.
//! * **`V_RC`** — the single-RC overpotential, advanced by the *exact* exponential integrator for a
//!   current held constant over the segment (so a constant-current pulse reproduces the closed-form
//!   Thevenin response to machine precision — the §13 battery validation row).
//! * **`T_batt`** — a lumped thermal node (`C = mass·c_p`) heated by the I²R / RC dissipation and the
//!   entropic `I·T·dU/dT` term, cooled to the coolant through `R_th`; semi-implicit Euler on the
//!   decay term (A-stable, matching §11's slow-state integrator).
//!
//! **Thevenin form** (pack level, discharge current positive):
//! `V_term = OCV(SoC,T) − I·R0 − V_RC`, with `V_RC` relaxing to `I·R1` at time constant `τ1`. Cell
//! tables scale to the pack by `ns` (voltage) and `ns/np` (resistance). The QSS simplification is
//! that within one segment the current is constant; the RC state carries memory across segments.
//!
//! **Vdc–SoC coupling.** [`Pack::terminal_voltage_v`] is the SoC-dependent DC-link voltage the
//! powertrain evaluates its Vdc-stacked `.ptm` maps at (traction *and* the machine-thermal loss). A
//! low-SoC point drops the terminal voltage below the drive-unit voltage grid; the map's Vdc axis
//! extrapolates linearly there (Decision #30, `OutOfDomain::Linear`).
//!
//! Clean-room: the equivalent-circuit state equations follow the published NREL `thevenin` model
//! (BSD-3) and the ECM literature it cites (Plett, *Battery Management Systems* Vol.1, 2015,
//! ch.2–3) — never derived from another simulator's source.

use outlap_core::{GriddedMapN, GriddedTable, MonotoneCubic, OutOfDomain};
use outlap_schema::battery::{BatteryDoc, TableLevel};

use crate::error::T1Error;

/// °C → K offset (SI internally; °C only at the file/display boundary).
const CELSIUS_K: f64 = 273.15;
/// Seconds per hour (A·h → Coulomb).
const S_PER_H: f64 = 3600.0;
/// Floor on `R0` used when solving the constant-power current root (avoids a divide-by-zero at a
/// degenerate zero-resistance node; physical packs are well above this).
const R0_FLOOR: f64 = 1.0e-9;

/// The ECM sidecar columns, in the order the battery importer emits them.
const COL_OCV: &str = "ocv_v";
const COL_R0: &str = "r0_ohm";
const COL_R1: &str = "r1_ohm";
const COL_TAU1: &str = "tau1_s";
const COL_DUDT: &str = "dudt_v_per_k";

/// An assembled Thevenin battery pack: the ECM parameter maps, the pack scaling, the power/voltage
/// limits, and the lumped thermal parameters. Cold-path (allocations allowed); the runtime state is
/// [`PackState`], advanced zero-allocation by [`Pack::step_current`] / [`Pack::step_power`].
#[derive(Clone, Debug)]
pub struct Pack {
    // ECM parameter maps over the `(soc, temp_c)` grid (cell- or pack-level per `level`).
    ocv: GriddedMapN<f64>,
    r0: GriddedMapN<f64>,
    r1: GriddedMapN<f64>,
    tau1: GriddedMapN<f64>,
    dudt: GriddedMapN<f64>,
    /// Voltage scale cell→pack (`ns`, or 1 for a pack-level table).
    scale_v: f64,
    /// Resistance scale cell→pack (`ns/np`, or 1 for a pack-level table).
    scale_r: f64,
    /// Pack charge capacity, Coulomb (`q_pack_ah · 3600`).
    q_pack_coulomb: f64,
    /// Usable SoC window `[min, max]`.
    soc_window: [f64; 2],
    /// Peak discharge power vs SoC, W (positive ceiling).
    peak_discharge: MonotoneCubic<f64>,
    /// Peak regen power vs SoC, W (positive-magnitude ceiling) at the reference temperature.
    peak_regen: MonotoneCubic<f64>,
    /// Charge-acceptance derate vs pack temperature (°C → `0..1`). `None` ⇒ no kinetic derate.
    regen_derate: Option<MonotoneCubic<f64>>,
    /// Pack charge-voltage ceiling `ns · cell_v_max`, V.
    v_max_pack_v: f64,
    /// Lumped thermal capacity `C = mass·c_p`, J/K.
    c_th_j_per_k: f64,
    /// Jacket thermal resistance to the coolant, K/W (`≤ 0` ⇒ the pack is pinned to the coolant).
    r_th_k_per_w: f64,
    /// Coolant/ambient sink temperature, K.
    t_coolant_k: f64,
    /// Assembly-time notes (estimated/degraded values) for the loaded-model report — nothing silent
    /// (#41). The Python boundary threads these into the lap's `notes`.
    notes: Vec<String>,
}

/// The per-segment slow state of a [`Pack`]: SoC, the RC overpotential, the lumped temperature, and
/// the last solved terminal current (for the Vdc coupling / result channels).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PackState {
    /// State of charge, 0..1.
    pub soc: f64,
    /// Pack-level RC overpotential `V_RC`, V.
    pub v_rc_v: f64,
    /// Lumped pack temperature, K.
    pub temp_k: f64,
    /// Last solved terminal current, A (discharge positive).
    pub current_a: f64,
}

/// The result of advancing one segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepOut {
    /// Terminal (DC-link) voltage after the step, V.
    pub terminal_v: f64,
    /// Terminal current, A (discharge positive).
    pub current_a: f64,
    /// State of charge after the step.
    pub soc: f64,
    /// Pack temperature after the step, °C.
    pub temp_c: f64,
    /// Whether the requested power was clipped by a power/SoC-window limit.
    pub power_limited: bool,
}

impl Pack {
    /// The ECM sidecar axis names, in tensor order, for decoding a pack's `(soc, temp)` table.
    #[must_use]
    pub fn ecm_axis_names() -> [&'static str; 2] {
        ["soc", "temp_c"]
    }

    /// Assemble a runnable pack from a `battery/1.0` document and its decoded ECM table.
    ///
    /// `initial_soc` defaults to the top of the SoC window (full charge — the reference state PR7's
    /// static envelope assumes). The pack starts at the coolant temperature and a relaxed RC state.
    ///
    /// # Errors
    /// [`T1Error::Battery`] for an unsupported RC-pair count, a missing ECM column, or a
    /// non-rectilinear `(soc, temp)` grid.
    pub fn assemble(
        doc: &BatteryDoc,
        table: &GriddedTable<f64>,
        initial_soc: Option<f64>,
    ) -> Result<(Self, PackState), T1Error> {
        if doc.ecm.rc_pairs != 1 {
            return Err(T1Error::Battery(format!(
                "only 1 RC pair is supported (the ECM tables carry r1/tau1); got rc_pairs = {}",
                doc.ecm.rc_pairs
            )));
        }
        // Clamp out of the `(soc, temp)` grid (the ECM is only defined on its measured hull).
        let modes = || vec![OutOfDomain::Clamp; 2];
        let m = |col: &str| -> Result<GriddedMapN<f64>, T1Error> {
            table
                .map(col, modes())
                .map_err(|e| T1Error::Battery(format!("ECM column `{col}`: {e}")))
        };
        let (scale_v, scale_r) = match doc.ecm.tables.level {
            TableLevel::Cell => (
                f64::from(doc.topology.ns),
                f64::from(doc.topology.ns) / f64::from(doc.topology.np),
            ),
            TableLevel::Pack => (1.0, 1.0),
        };
        let peak_discharge =
            power_curve(&doc.limits.peak_discharge_power_w_vs_soc).map_err(T1Error::Battery)?;
        let peak_regen =
            power_curve(&doc.limits.peak_regen_power_w_vs_soc).map_err(T1Error::Battery)?;

        let regen_derate = doc
            .limits
            .regen_derate_vs_temp
            .as_ref()
            .map(derate_curve)
            .transpose()
            .map_err(T1Error::Battery)?;
        let mut notes = Vec::new();
        if regen_derate.is_none() {
            notes.push(
                "battery declares no `limits.regen_derate_vs_temp`; charge acceptance is assumed \
                 temperature-independent (estimated) — a cold pack will accept its full SoC-curve \
                 ceiling, which a real BMS would refuse"
                    .to_owned(),
            );
        }

        let pack = Self {
            ocv: m(COL_OCV)?,
            r0: m(COL_R0)?,
            r1: m(COL_R1)?,
            tau1: m(COL_TAU1)?,
            dudt: m(COL_DUDT)?,
            scale_v,
            scale_r,
            q_pack_coulomb: doc.capacity.q_pack_ah * S_PER_H,
            soc_window: doc.soc_window,
            peak_discharge,
            peak_regen,
            regen_derate,
            // `cell_v_max` is stated per cell regardless of the table level, so the pack ceiling is
            // always `ns ×` it (a pack-level ECM table does not pre-scale the voltage bounds).
            v_max_pack_v: f64::from(doc.topology.ns) * doc.limits.cell_v_max,
            c_th_j_per_k: doc.thermal.mass_kg * doc.thermal.cp_j_per_kgk,
            r_th_k_per_w: doc.thermal.thermal_resistance_k_per_w,
            t_coolant_k: doc.thermal.coolant_temp_c + CELSIUS_K,
            notes,
        };
        let soc0 = initial_soc.unwrap_or(pack.soc_window[1]);
        let state = PackState {
            soc: soc0.clamp(0.0, 1.0),
            v_rc_v: 0.0,
            temp_k: pack.t_coolant_k,
            current_a: 0.0,
        };
        Ok((pack, state))
    }

    /// Open-circuit (rest) terminal voltage at the state's SoC and temperature, V.
    #[must_use]
    pub fn open_circuit_voltage_v(&self, st: &PackState) -> f64 {
        self.scale_v * self.ocv.eval(&[st.soc, st.temp_k - CELSIUS_K])
    }

    /// The loaded terminal (DC-link) voltage at the current state — `OCV − I·R0 − V_RC` — the
    /// SoC-dependent voltage the powertrain evaluates its Vdc-stacked maps at (the Vdc–SoC coupling).
    #[must_use]
    pub fn terminal_voltage_v(&self, st: &PackState) -> f64 {
        let r0 = self.r0_pack(st);
        self.open_circuit_voltage_v(st) - st.current_a * r0 - st.v_rc_v
    }

    /// The instantaneous discharge power ceiling at the current SoC, W (0 below the SoC window). The
    /// dynamic battery cap that composes (via `min`) with PR5's thermal derate on the traction limit.
    #[must_use]
    pub fn discharge_power_limit_w(&self, st: &PackState) -> f64 {
        if st.soc <= self.soc_window[0] {
            0.0
        } else {
            self.peak_discharge.eval(st.soc).max(0.0)
        }
    }

    /// The **charge-acceptance ceiling**: the instantaneous regen power the pack will take at its
    /// current SoC *and* temperature, W (positive magnitude; `0` at or above the SoC window).
    ///
    /// Three ceilings compose by `min`, because a battery-management system enforces all three:
    ///
    /// 1. **Design/SoC ceiling** — the declared `peak_regen_power_w_vs_soc(SoC)` curve.
    /// 2. **Kinetic (cold) derate** — `regen_derate_vs_temp(T)` scales that curve. A cold cell
    ///    cannot accept a fast charge: below ~10 °C anode intercalation slows until lithium plating
    ///    competes, so a real BMS cuts charge current hard (to zero below ~0 °C). This is a kinetic
    ///    limit and does *not* emerge from the ohmic grid, so it is declared, not derived. Absent ⇒
    ///    factor 1.
    /// 3. **Voltage (CV) ceiling** — charging drives the terminal voltage *above* the open-circuit
    ///    EMF by `I·R0`, and it may not exceed `ns · cell_v_max`. With `emf = OCV(SoC,T) − V_RC` and
    ///    the pack-level `R0(SoC,T)`, the largest charge current is `(V_max − emf) / R0`, so
    ///    `P ≤ V_max · (V_max − emf) / R0`. This is the constant-voltage taper: it vanishes as the
    ///    pack fills (`emf → V_max`) and tightens when cold (`R0` rises), automatically.
    ///
    /// Ceiling 3 alone does *not* reproduce cold-charge refusal on a real pack — at mid SoC it sits
    /// far above the design curve even at −10 °C. Both terms are needed, and each binds in its own
    /// regime: (2) when cold, (3) when nearly full.
    #[must_use]
    pub fn regen_power_limit_w(&self, st: &PackState) -> f64 {
        if st.soc >= self.soc_window[1] {
            return 0.0;
        }
        let design = self.peak_regen.eval(st.soc).max(0.0);
        let derate = self.regen_derate_factor(st);
        self.voltage_limited_charge_power_w(st).min(design * derate)
    }

    /// Assembly-time notes for the loaded-model report (estimated/degraded values surfaced, #41).
    #[must_use]
    pub fn notes(&self) -> &[String] {
        &self.notes
    }

    /// The usable state-of-charge window `[min, max]`. At or above the top the pack accepts no
    /// charge, so a lap seeded there recovers nothing however hard the car brakes.
    #[must_use]
    pub fn soc_window(&self) -> [f64; 2] {
        self.soc_window
    }

    /// Whether the pack declared a `regen_derate_vs_temp` curve. When `false`, charge acceptance is
    /// temperature-independent and [`Self::notes`] says so.
    #[must_use]
    pub fn regen_derate_declared(&self) -> bool {
        self.regen_derate.is_some()
    }

    /// The temperature derate factor on charge acceptance, `0..1` (`1` when no curve is declared).
    #[must_use]
    pub fn regen_derate_factor(&self, st: &PackState) -> f64 {
        self.regen_derate
            .as_ref()
            .map_or(1.0, |c| c.eval(st.temp_k - CELSIUS_K).clamp(0.0, 1.0))
    }

    /// The charge power at which the loaded terminal voltage would reach `ns · cell_v_max`, W
    /// (positive magnitude; `0` once the EMF already sits at the ceiling). See
    /// [`Self::regen_power_limit_w`] ceiling 3.
    #[must_use]
    pub fn voltage_limited_charge_power_w(&self, st: &PackState) -> f64 {
        let emf = self.open_circuit_voltage_v(st) - st.v_rc_v;
        let headroom = self.v_max_pack_v - emf;
        if headroom <= 0.0 {
            return 0.0;
        }
        self.v_max_pack_v * headroom / self.r0_pack(st)
    }

    /// Advance one segment under a commanded **terminal current** `i_pack_a` (discharge positive)
    /// for `dt_s`. This is the current-driven path the pulse-response validation uses. Zero-alloc.
    pub fn step_current(&self, st: &mut PackState, i_pack_a: f64, dt_s: f64) -> StepOut {
        self.advance(st, i_pack_a, dt_s, false)
    }

    /// Advance one segment delivering a commanded **terminal power** `power_w` (discharge positive,
    /// regen negative) for `dt_s`. The current is solved from the constant-power Thevenin root and
    /// clipped to the SoC-dependent power limit; SoC / RC / temperature then advance. Zero-alloc.
    pub fn step_power(&self, st: &mut PackState, power_w: f64, dt_s: f64) -> StepOut {
        // Clip the demand to the instantaneous power envelope (the dynamic battery cap).
        let (clipped, limited) = if power_w >= 0.0 {
            let cap = self.discharge_power_limit_w(st);
            (power_w.min(cap), power_w > cap)
        } else {
            let cap = self.regen_power_limit_w(st);
            (power_w.max(-cap), -power_w > cap)
        };
        let i = self.current_for_power(st, clipped);
        let mut out = self.advance(st, i, dt_s, false);
        out.power_limited = limited;
        out
    }

    /// Shared advance: given a terminal current, update `V_RC` (exact exponential), SoC (Coulomb
    /// count), and temperature (semi-implicit Euler on the lumped node), then report the terminal
    /// voltage. `power_limited` is set by the caller for the power path.
    fn advance(
        &self,
        st: &mut PackState,
        i_pack_a: f64,
        dt_s: f64,
        power_limited: bool,
    ) -> StepOut {
        let r1 = self.r1_pack(st);
        let tau = self.tau1.eval(&[st.soc, st.temp_k - CELSIUS_K]).max(1.0e-6);
        // Exact exponential RC advance for a current constant over the segment.
        let decay = (-dt_s / tau).exp();
        st.v_rc_v = st.v_rc_v * decay + i_pack_a * r1 * (1.0 - decay);
        // Coulomb counting: discharge lowers SoC.
        st.soc = (st.soc - i_pack_a * dt_s / self.q_pack_coulomb).clamp(0.0, 1.0);
        // Lumped-node temperature: I²R0 + RC dissipation + entropic, cooled through R_th.
        self.advance_temperature(st, i_pack_a, dt_s);
        st.current_a = i_pack_a;
        StepOut {
            terminal_v: self.terminal_voltage_v(st),
            current_a: i_pack_a,
            soc: st.soc,
            temp_c: st.temp_k - CELSIUS_K,
            power_limited,
        }
    }

    /// Semi-implicit Euler step of the lumped thermal node (A-stable on the `−(T−T_cool)/R_th` term).
    fn advance_temperature(&self, st: &mut PackState, i_pack_a: f64, dt_s: f64) {
        if self.c_th_j_per_k <= 0.0 || self.r_th_k_per_w <= 0.0 {
            // Degenerate: no capacity or a perfect coolant coupling ⇒ pinned to the coolant.
            st.temp_k = self.t_coolant_k;
            return;
        }
        let r0 = self.r0_pack(st);
        let r1 = self.r1_pack(st);
        let dudt = self.scale_v * self.dudt.eval(&[st.soc, st.temp_k - CELSIUS_K]);
        // Irreversible ohmic (series R0 + the RC branch v²/R1) always heats; entropic can cool.
        let q_irrev = i_pack_a * i_pack_a * r0 + st.v_rc_v * st.v_rc_v / r1.max(R0_FLOOR);
        let q_entropic = i_pack_a * st.temp_k * dudt;
        let q_gen = q_irrev + q_entropic;
        let a = dt_s / self.c_th_j_per_k;
        let g = 1.0 / self.r_th_k_per_w;
        st.temp_k = (st.temp_k + a * (q_gen + g * self.t_coolant_k)) / (1.0 + a * g);
    }

    /// Pack-level R0 at the state, Ω.
    fn r0_pack(&self, st: &PackState) -> f64 {
        (self.scale_r * self.r0.eval(&[st.soc, st.temp_k - CELSIUS_K])).max(R0_FLOOR)
    }

    /// Pack-level R1 at the state, Ω.
    fn r1_pack(&self, st: &PackState) -> f64 {
        (self.scale_r * self.r1.eval(&[st.soc, st.temp_k - CELSIUS_K])).max(0.0)
    }

    /// Solve the terminal current for a demanded terminal power (discharge positive) from the
    /// constant-power Thevenin relation `P = V·I`, `V = (OCV − V_RC) − I·R0`, taking the physical
    /// (low-current) root. Falls back to `P / V_oc` if the demand exceeds the deliverable power.
    fn current_for_power(&self, st: &PackState, power_w: f64) -> f64 {
        if power_w == 0.0 {
            return 0.0;
        }
        let r0 = self.r0_pack(st);
        let emf = self.open_circuit_voltage_v(st) - st.v_rc_v; // the driving EMF behind R0
                                                               // R0·I² − emf·I + P = 0 ⇒ I = [emf − sqrt(emf² − 4·R0·P)] / (2·R0).
        let disc = emf * emf - 4.0 * r0 * power_w;
        if disc <= 0.0 {
            // Power exceeds the max deliverable (P_max = emf²/4R0); return the max-power current.
            emf / (2.0 * r0)
        } else {
            (emf - disc.sqrt()) / (2.0 * r0)
        }
    }
}

/// Fit a monotone-cubic power-vs-SoC curve from a schema `PowerVsSoc` (paired equal-length arrays).
fn power_curve(p: &outlap_schema::battery::PowerVsSoc) -> Result<MonotoneCubic<f64>, String> {
    if p.soc.len() != p.power_w.len() || p.soc.len() < 2 {
        return Err(format!(
            "power-vs-soc curve needs ≥ 2 paired points; got soc={}, power_w={}",
            p.soc.len(),
            p.power_w.len()
        ));
    }
    MonotoneCubic::new(p.soc.clone(), p.power_w.clone())
        .map_err(|e| format!("power-vs-soc curve is not monotone-fittable: {e}"))
}

/// Fit a monotone-cubic derate-vs-temperature curve from a schema `DerateVsTemp`. Evaluations off
/// the ends clamp to the terminal factors — a pack colder than the coldest breakpoint accepts the
/// coldest declared factor (typically 0), never an extrapolated negative one.
fn derate_curve(d: &outlap_schema::battery::DerateVsTemp) -> Result<MonotoneCubic<f64>, String> {
    if d.temp_c.len() != d.factor.len() || d.temp_c.len() < 2 {
        return Err(format!(
            "regen-derate curve needs ≥ 2 paired points; got temp_c={}, factor={}",
            d.temp_c.len(),
            d.factor.len()
        ));
    }
    MonotoneCubic::new(d.temp_c.clone(), d.factor.clone())
        .map_err(|e| format!("regen-derate curve is not monotone-fittable: {e}"))
}
