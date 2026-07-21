// SPDX-License-Identifier: AGPL-3.0-only
//! The energy manager — a pure control-phase policy from inputs to an [`ErsCommand`].
//!
//! `decide` is a pure function (§6.2b: the manager is a control-phase controller; §11.2: mode
//! changes are discrete events at step boundaries — the CALLER invokes `decide` once per step
//! boundary and holds the command across the step). The caller owns every piece of state: the
//! clock (`dt`), the lap ledger, and the C5.12 ramp episode accumulator; nothing here mutates.
//!
//! Command powers are ELECTRICAL (the CU-K DC bus) — the tier wiring converts to the mechanical
//! side through the rulebook's single conversion seam and then applies the machine/pack ceilings
//! it owns (torque envelope at the live shaft speed, pack charge acceptance, machine-thermal
//! derate: the pack has the final word).
//!
//! # The rule-based v1 policy (D-M6-8 / D-M6-9)
//!
//! "Deploy below taper speed, harvest under braking, recharge on designated straights" (§8.3):
//!
//! 1. **Braking** → harvest through the brake-blend path, capped by the mechanical regen
//!    envelope, the electrical harvest cap, and the remaining lap budget.
//! 2. **Drive, recharge wanted** (recharge phases on, SoC below target, ICE surplus available,
//!    budget left) → the K back-drives: at part throttle the ICE covers the demand gap
//!    (HarvestPartThrottle); at full throttle the "super-clip" transition is rate-limited by the
//!    C5.12 ramp (HarvestStraight).
//! 3. **Drive, otherwise** → greedy feed-forward deploy: the full curve `min(cap, cap·taper(v))`
//!    whenever driver demand is positive (D-M6-8 — deliberately demand-GATED, not demand-scaled;
//!    no SoC input: SoC starvation is honest physics the pack clamps downstream).
//! 4. **Neither** → idle.

use num_traits::Float;

use crate::ledger::LapEnergyLedger;
use crate::rulebook::ErsRulebook;
use crate::schedule::UsSchedule;

/// What the manager decided for this step: electrical command powers + the mode label.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ErsCommand<T> {
    /// Electrical deployment power at the CU-K DC bus, W (≥ 0).
    pub deploy_w: T,
    /// Electrical harvest power at the CU-K DC bus, W (≥ 0).
    pub harvest_w: T,
    /// Which rule produced the command (a results channel; discrete at step boundaries).
    pub mode: ErsMode,
}

impl<T: Float> ErsCommand<T> {
    /// The idle command (no deploy, no harvest).
    pub fn idle() -> Self {
        Self {
            deploy_w: T::zero(),
            harvest_w: T::zero(),
            mode: ErsMode::Idle,
        }
    }
}

/// The rule that produced a command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErsMode {
    /// No deploy, no harvest.
    Idle,
    /// Feed-forward deployment on the base C5.2.8(i) envelope.
    Deploy,
    /// Deployment on the override C5.2.8(ii) envelope ("Overtake").
    OverrideDeploy,
    /// Braking-phase harvest (the brake-blend path).
    HarvestBrake,
    /// Part-throttle harvest: the ICE covers the demand gap, the K banks the surplus.
    HarvestPartThrottle,
    /// Full-throttle straight harvest ("super-clip"): ICE back-drive through the C5.12 ramp.
    HarvestStraight,
}

/// Per-step inputs to [`EnergyManager::decide`]. All caller-owned state is explicit.
#[derive(Clone, Copy, Debug)]
pub struct DecideInput<T> {
    /// Vehicle speed, m/s.
    pub v: T,
    /// Driver traction demand in `[0, 1]` (0 = off throttle, 1 = full throttle).
    pub driver_demand: T,
    /// Mechanical braking power demanded at the driven axle(s), W (≥ 0; 0 = not braking).
    pub brake_demand_w: T,
    /// Mechanical regen power the machine can absorb at the current speed, W (the `.ptm` regen
    /// envelope sampled by the tier; the low-speed fade and pack acceptance compose downstream).
    pub mech_regen_envelope_w: T,
    /// Mechanical ICE power available beyond the driver's demand, W (≥ 0) — the surplus the K
    /// may back-drive against in the recharge phases. Zero for cars without an ICE.
    pub ice_surplus_w: T,
    /// Pack state of charge in `[0, 1]` (consumed ONLY by the recharge-target gate; deployment
    /// deliberately takes no SoC input, D-M6-8).
    pub soc: T,
    /// Whether override ("Overtake") is active this step. The per-run flag wins unconditionally
    /// over the schema `activation` hint (D-M6-5).
    pub override_active: bool,
    /// The previous step's signed electrical K power, W (+ deploy, − harvest) — the C5.12 ramp
    /// limits how fast demand may be REDUCED from this level.
    pub prev_k_power_w: T,
    /// Cumulative demand reduction already taken in the active C5.12 ramp episode, W. The caller
    /// resets it to zero when the episode ends (demand rises / the harvest phase exits).
    pub ramp_reduced_w: T,
    /// The step length, s (converts the remaining lap budgets into power ceilings).
    pub dt: T,
    /// Station index into a [`UsSchedule`] (ignored by the rule-based policy).
    pub station: usize,
}

/// The deployment policy (D-M6-9).
#[derive(Clone, Debug)]
pub enum DeployPolicy<T> {
    /// The rule-based v1 default: greedy feed-forward deploy + automated recharge paths.
    RuleBased,
    /// A data-driven `u(s)` schedule (the stage-2 strategy surface, executed as data).
    Schedule(UsSchedule<T>),
}

/// The energy manager: one rulebook + one policy.
#[derive(Clone, Debug)]
pub struct EnergyManager<T> {
    rulebook: ErsRulebook<T>,
    policy: DeployPolicy<T>,
}

impl<T: Float> EnergyManager<T> {
    /// Build a manager from a rulebook and a policy.
    pub fn new(rulebook: ErsRulebook<T>, policy: DeployPolicy<T>) -> Self {
        Self { rulebook, policy }
    }

    /// The rulebook this manager enforces.
    pub fn rulebook(&self) -> &ErsRulebook<T> {
        &self.rulebook
    }

    /// Decide this step's command. Pure: same inputs → same command, bit-for-bit. Budgets are
    /// enforced by construction — the command is clipped so `ledger + cmd·dt` can never exceed
    /// a per-lap budget.
    pub fn decide(&self, inp: &DecideInput<T>, ledger: &LapEnergyLedger<T>) -> ErsCommand<T> {
        match &self.policy {
            DeployPolicy::RuleBased => self.rule_based(inp, ledger),
            DeployPolicy::Schedule(schedule) => self.scheduled(schedule, inp, ledger),
        }
    }

    /// Remaining electrical harvest budget this lap, expressed as a power ceiling over `dt`.
    fn harvest_headroom_w(&self, inp: &DecideInput<T>, ledger: &LapEnergyLedger<T>) -> T {
        let budget = self.rulebook.harvest_budget_j(inp.override_active);
        ((budget - ledger.harvest_j()) / inp.dt).max(T::zero())
    }

    /// Remaining electrical deploy budget this lap as a power ceiling, or `None` = unbounded.
    /// A null budget stays unenforced (the 2026 rules have no per-lap deployment budget).
    fn deploy_headroom_w(&self, inp: &DecideInput<T>, ledger: &LapEnergyLedger<T>) -> Option<T> {
        self.rulebook
            .per_lap_deploy_j()
            .map(|budget| ((budget - ledger.deploy_j()) / inp.dt).max(T::zero()))
    }

    /// The full deploy command at the current speed: curve × remaining budget.
    fn deploy_command(&self, inp: &DecideInput<T>, ledger: &LapEnergyLedger<T>) -> ErsCommand<T> {
        let mut p = self
            .rulebook
            .deploy_cap_electrical_w(inp.v, inp.override_active);
        if let Some(headroom) = self.deploy_headroom_w(inp, ledger) {
            p = p.min(headroom);
        }
        let mode = if inp.override_active {
            ErsMode::OverrideDeploy
        } else {
            ErsMode::Deploy
        };
        ErsCommand {
            deploy_w: p.max(T::zero()),
            harvest_w: T::zero(),
            mode,
        }
    }

    fn rule_based(&self, inp: &DecideInput<T>, ledger: &LapEnergyLedger<T>) -> ErsCommand<T> {
        let rb = &self.rulebook;
        let harvest_headroom = self.harvest_headroom_w(inp, ledger);

        // 1. Braking → harvest (the brake-blend path). Electrical = mechanical × 0.97 (C5.2.21);
        //    capped by what the machine can absorb, what braking demands, the DC-bus cap, and the
        //    remaining lap budget.
        if inp.brake_demand_w > T::zero() {
            let mech = inp
                .mech_regen_envelope_w
                .min(inp.brake_demand_w)
                .max(T::zero());
            let elec = rb
                .elec_harvest_w(mech)
                .min(rb.harvest_cap_electrical_w())
                .min(harvest_headroom);
            return ErsCommand {
                deploy_w: T::zero(),
                harvest_w: elec,
                mode: ErsMode::HarvestBrake,
            };
        }

        if inp.driver_demand > T::zero() {
            // 2. Recharge phases (D-M6-9): SoC below target, ICE surplus available, budget left.
            let recharge_wanted = rb.recharge_phases()
                && inp.soc < rb.recharge_target_soc()
                && inp.ice_surplus_w > T::zero()
                && harvest_headroom > T::zero();
            if recharge_wanted {
                let mech = inp
                    .ice_surplus_w
                    .min(inp.mech_regen_envelope_w)
                    .max(T::zero());
                let elec_target = rb
                    .elec_harvest_w(mech)
                    .min(rb.harvest_cap_electrical_w())
                    .min(harvest_headroom);
                if inp.driver_demand < T::one() {
                    // Part-throttle harvest: the ICE covers the demand gap directly; the C5.12
                    // ramp governs "power limited" full-throttle periods, not throttle lifts.
                    return ErsCommand {
                        deploy_w: T::zero(),
                        harvest_w: elec_target,
                        mode: ErsMode::HarvestPartThrottle,
                    };
                }
                // Super-clip: at full throttle the K's demand ramps DOWN from the previous level
                // toward back-drive, rate-limited by C5.12 (initial step, rate, episode total).
                let allowed = rb.ramp_allowed_reduction_w(inp.ramp_reduced_w, inp.dt);
                let new_k = (inp.prev_k_power_w - allowed).max(-elec_target);
                return if new_k > T::zero() {
                    // Still on the deploy side of the transition: a reduced deployment, still
                    // bounded by the deploy curve + budget at the CURRENT speed.
                    let mut p = new_k.min(rb.deploy_cap_electrical_w(inp.v, inp.override_active));
                    if let Some(headroom) = self.deploy_headroom_w(inp, ledger) {
                        p = p.min(headroom);
                    }
                    ErsCommand {
                        deploy_w: p,
                        harvest_w: T::zero(),
                        mode: ErsMode::HarvestStraight,
                    }
                } else {
                    ErsCommand {
                        deploy_w: T::zero(),
                        harvest_w: (-new_k).max(T::zero()),
                        mode: ErsMode::HarvestStraight,
                    }
                };
            }
            // 3. Greedy feed-forward deploy (D-M6-8).
            return self.deploy_command(inp, ledger);
        }

        // 4. Coasting: nothing to do (lift-and-coast harvest arrives with the u(s) lift hook).
        ErsCommand::idle()
    }

    fn scheduled(
        &self,
        schedule: &UsSchedule<T>,
        inp: &DecideInput<T>,
        ledger: &LapEnergyLedger<T>,
    ) -> ErsCommand<T> {
        // The per-run override flag wins unconditionally (D-M6-5); the schedule may also raise it.
        let override_active = inp.override_active || schedule.override_flag(inp.station);
        let inp = DecideInput {
            override_active,
            ..*inp
        };
        let u = schedule.deploy_regen(inp.station);
        if u > T::zero() {
            let full = self.deploy_command(&inp, ledger);
            return ErsCommand {
                deploy_w: full.deploy_w * u,
                ..full
            };
        }
        if u < T::zero() {
            let mech = inp.mech_regen_envelope_w.max(T::zero());
            let elec = self
                .rulebook
                .elec_harvest_w(mech)
                .min(self.rulebook.harvest_cap_electrical_w())
                .min(self.harvest_headroom_w(&inp, ledger));
            let mode = if inp.brake_demand_w > T::zero() {
                ErsMode::HarvestBrake
            } else {
                ErsMode::HarvestStraight
            };
            return ErsCommand {
                deploy_w: T::zero(),
                harvest_w: elec * (-u),
                mode,
            };
        }
        ErsCommand::idle()
    }
}
