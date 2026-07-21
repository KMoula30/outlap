// SPDX-License-Identifier: AGPL-3.0-only
//! The QSS energy-manager coupling — the 2026 ERS rulebook governing the slow-state march.
//!
//! [`ErsCoupling`] carries the shared [`EnergyManager`] (built from the SAME
//! [`ErsRulebook`](outlap_powertrain::ErsRulebook) the [`T0Vehicle`] pedal availability uses —
//! parity gate #4 compares one implementation of the rules, never two) plus the tier-owned
//! mechanical facts the manager's inputs need: the driveline efficiency, the machine ceiling, and
//! the brake-blend authority/axle split of the harvest chain.
//!
//! # The five-ceiling harvest chain (D-M6-10 — parity with T2's `blend_regen`)
//!
//! QSS braking harvest composes the same ceilings the transient blend enforces, in the same
//! order, so parity gate #4 measures physics rather than modelling gaps:
//!
//! 1. **Machine envelope** — the MGU-K's ratio-invariant mechanical ceiling
//!    ([`T0Vehicle::ers_p_mech_max_w`]; the `.ptm` schema treats an absent regen curve as a
//!    symmetric machine).
//! 2. **Low-speed fade** — linear to zero below [`REGEN_FADE_SPEED_MPS`] (the same constant the
//!    transient blend uses; real controllers hand braking back to the calipers at walking pace).
//! 3. **Pack charge acceptance** — `Pack::regen_power_limit_w` (design curve × kinetic derate ∧
//!    CV taper), applied downstream of the manager: the pack has the final word.
//! 4. **Blend authority** — `brakes.regen_blend.max_regen_frac` of the commanded brake demand
//!    (no `regen_blend` block ⇒ zero braking harvest, the T2 convention).
//! 5. **Per-axle split** — a machine only ever brakes the axle(s) it drives; the balance bar
//!    apportions the commanded braking force between axles.
//!
//! Ceilings 1–2 and 4–5 fold into the manager's [`DecideInput`](outlap_powertrain::DecideInput)
//! (`mech_regen_envelope_w`, `brake_demand_w`) exactly as its field docs specify; ceiling 3 clips
//! the realized command in the march.

use outlap_powertrain::{DeployPolicy, EnergyManager};
use outlap_schema::Vehicle;

use crate::error::T0Error;
use crate::vehicle::T0Vehicle;

// Re-export the manager surface so tier consumers (the Python binding, the T2 governor) need not
// depend on `outlap-powertrain` directly to pick a policy, hand in a u(s) schedule, or drive the
// manager per step. `ErsPolicy` aliases the renamed runtime `DeployPolicy` (D-M6-13: the schema's
// `vehicle::Policy` overlay took the bare `Policy` name).
pub use outlap_powertrain::{
    DecideInput, DeployPolicy as ErsPolicy, ErsCommand, ErsMode, LapEnergyLedger, ScheduleError,
    UsSchedule,
};

/// Speed below which regen fades linearly to zero, m/s — the same constant as the transient
/// blend's `outlap_vehicle::control::REGEN_FADE_SPEED_MPS` (kept numerically identical by the
/// D-M6-10 "same rules" contract; the crates cannot share it without inverting the dependency
/// direction).
pub const REGEN_FADE_SPEED_MPS: f64 = 2.0;

/// The energy-manager coupling handed to [`solve_t0`](crate::solve_t0) /
/// [`solve_t1`](crate::solve_t1) next to the electro stack. Requires the electro stack (the
/// manager schedules the pack; without one there is nothing to bank into).
#[derive(Clone, Debug)]
pub struct ErsCoupling {
    /// The energy manager (rulebook + policy). Cloned from the vehicle's own rulebook so the
    /// pedal availability and the march enforce identical curves.
    pub manager: EnergyManager<f64>,
    /// Whether override ("Overtake") is active for this run — the per-run flag wins
    /// unconditionally over the schema `activation` hint (D-M6-5).
    pub override_active: bool,
    /// Driveline (crank→wheel) efficiency for the deploy force (distinct from the rulebook's
    /// 0.97 electrical→mechanical factor).
    pub eta: f64,
    /// MGU-K ratio-invariant mechanical power ceiling, W (deploy cap AND the symmetric-machine
    /// regen envelope of harvest ceiling 1).
    pub p_mech_max_w: f64,
    /// The driven axle(s)' share of the commanded braking force (balance bar over the axles that
    /// carry driven wheels) — harvest ceiling 5.
    pub regen_axle_share: f64,
    /// Blend authority `brakes.regen_blend.max_regen_frac` (0 without a `regen_blend` block —
    /// the T2 convention: no blend policy, no braking harvest) — harvest ceiling 4.
    pub max_regen_frac: f64,
    /// FIA C5.2.9 on-track energy-swing limit, J (`policy.regulatory_window_mj`) — the maximum
    /// `max − min` SoC energy the store may vary on track. A REGULATORY limit, enforced
    /// independently of the pack's PHYSICAL `soc_window`: the physical window is the battery's
    /// range; this caps the swing WITHIN it (they coincide only when the pack is sized exactly to
    /// the reg). Bounded causally by the running-band clip in the march (a step may not raise SoC
    /// more than this above the lap's lowest point so far, nor lower it more than this below the
    /// highest).
    pub swing_limit_j: f64,
}

impl ErsCoupling {
    /// Assemble the coupling from the resolved vehicle + its T0 reduction. Returns `None` when
    /// the car has no `ers:` block. Cold path (allocations allowed).
    ///
    /// # Errors
    /// Currently infallible past the `ers:` presence test (the rulebook was already built by the
    /// T0 assembly); typed for future policy validation.
    pub fn assemble(
        spec: &Vehicle,
        t0: &T0Vehicle,
        pack_soc_window: [f64; 2],
        policy_deploy: DeployPolicy<f64>,
        override_active: bool,
    ) -> Result<Option<Self>, T0Error> {
        let Some(policy) = &spec.policy else {
            return Ok(None);
        };
        // Built from the SPEC's `policy:` overlay + the governed pack's usable SoC window (the
        // single source of truth for `recharge_target`'s default); when `spec` is the block `t0`
        // was assembled from — the production path — the deploy curves are identical to the pedal
        // availability by construction (same `ErsRulebook::from_schema`).
        let rulebook = outlap_powertrain::ErsRulebook::from_schema(policy, pack_soc_window, None)?;
        // Driven axles come from the graph terminal wheels of the MECHANICAL sources (a
        // policy-governed machine is a force-adder, not a mechanical driver, so it is excluded).
        let governed = crate::graph::governed_unit_ids(spec);
        let (mut front_driven, mut rear_driven) = (false, false);
        for unit in spec
            .drivetrain
            .units
            .iter()
            .filter(|u| !governed.contains(u.id.as_str()))
        {
            let (_, wheels) = crate::graph::flatten_chain(&spec.drivetrain, unit);
            front_driven |= wheels.iter().any(|w| w.is_front());
            rear_driven |= wheels.iter().any(|w| !w.is_front());
        }
        let bias = spec.brakes.balance_bar;
        let regen_axle_share =
            (if front_driven { bias } else { 0.0 }) + (if rear_driven { 1.0 - bias } else { 0.0 });
        let max_regen_frac = spec
            .brakes
            .regen_blend
            .as_ref()
            .map_or(0.0, |b| b.max_regen_frac.clamp(0.0, 1.0));
        Ok(Some(Self {
            manager: EnergyManager::new(rulebook, policy_deploy),
            override_active,
            eta: t0.ers_eta(),
            p_mech_max_w: t0.ers_p_mech_max_w(),
            regen_axle_share,
            max_regen_frac,
            swing_limit_j: policy.regulatory_window_mj * 1.0e6,
        }))
    }

    /// The low-speed regen fade factor at speed `v`, `0..1` (harvest ceiling 2).
    #[must_use]
    pub fn fade(v: f64) -> f64 {
        (v / REGEN_FADE_SPEED_MPS).clamp(0.0, 1.0)
    }
}
