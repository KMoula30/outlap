// SPDX-License-Identifier: AGPL-3.0-only
//! The slow-stack PLAN — the pure eligibility/pairing rules for the QSS electro slow-state stack.
//!
//! The Python binding used to fold these rules into its IO code; the plan function makes them a
//! typed, cargo-testable decision over the resolved vehicle alone (no loader, no bytes). The
//! binding then performs exactly the IO the plan names: load the battery document + ECM sidecar,
//! and — when a thermal pairing exists — the `.emotor` network + its unit's `.ptm`.
//!
//! Rules (D-M6-13 — packs are an id-keyed `batteries:` map; the `.emotor` requirement stays RELAXED
//! so a policy-governed car's pack marches without a machine-thermal network):
//!
//! 1. No `batteries:` entry ⇒ no stack (single-voltage evaluation).
//! 2. A pack ⇒ it marches. The RELEVANT pack is the one the policy-governed machine references
//!    (or, absent a policy, the first electric unit's pack, or the sole map entry) — a single-pack
//!    car has exactly one entry, so this is byte-identical to the pre-2.0 singleton. The FIRST
//!    drive unit declaring a `thermal:` `.emotor` ref carries the machine slow state; extra
//!    declarations are dropped WITH a note. No thermal unit ⇒ the pack marches alone.

use outlap_schema::Vehicle;

/// The typed slow-stack plan for one vehicle.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SlowStackPlan {
    /// No `batteries:` entry — no electro slow stack (single-voltage evaluation).
    NoBattery,
    /// A pack marches; optionally paired with ONE machine-thermal network.
    Pack {
        /// The resolved pack's `battery.params` document reference (vehicle-root-relative path).
        battery_path: String,
        /// The machine-thermal pairing, when a drive unit declares one.
        thermal: Option<ThermalPairing>,
        /// Human-readable notes produced by the pairing rules (dropped extra declarations, the
        /// relaxed no-thermal case) — surfaced into the loaded-lap notes, nothing silent.
        notes: Vec<String>,
    },
}

/// The drive unit whose `.emotor` network carries the machine slow state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ThermalPairing {
    /// Index of the paired drive unit.
    pub unit_idx: usize,
    /// Its `thermal:` `.emotor` document reference.
    pub emotor_path: String,
    /// Its `source:` `.ptm` reference (the machine mass feeds the thermal assembly).
    pub ptm_path: String,
}

/// Decide the slow-stack plan for a resolved vehicle. Pure — no IO.
#[must_use]
pub fn plan_slow_stack(spec: &Vehicle) -> SlowStackPlan {
    // Resolve the relevant pack from the id-keyed `batteries` map: the pack the policy-governed
    // machine references, else the first electric (battery-bearing) unit's pack, else the sole map
    // entry. A single-pack car has exactly one entry, so this is the pre-2.0 singleton.
    let governed_pack_id = spec
        .policy
        .as_ref()
        .and_then(|p| p.governs.first())
        .and_then(|id| spec.drivetrain.units.iter().find(|u| u.id == *id))
        .and_then(|u| u.battery.as_ref());
    let first_unit_pack_id = spec
        .drivetrain
        .units
        .iter()
        .find_map(|u| u.battery.as_ref());
    let batt = governed_pack_id
        .or(first_unit_pack_id)
        .and_then(|id| spec.batteries.get(id))
        .or_else(|| spec.batteries.values().next());
    let Some(batt) = batt else {
        return SlowStackPlan::NoBattery;
    };
    let mut notes = Vec::new();
    let thermal_units: Vec<usize> = spec
        .drivetrain
        .units
        .iter()
        .enumerate()
        .filter_map(|(i, u)| u.thermal.as_ref().map(|_| i))
        .collect();
    let thermal = if let Some(&unit_idx) = thermal_units.first() {
        if thermal_units.len() > 1 {
            notes.push(format!(
                "{} drive units declare `.emotor` thermal models — the QSS coupling marches \
                 ONE network (unit {unit_idx}); the aggregate powertrain loss heats it and \
                 the others are not integrated this milestone",
                thermal_units.len()
            ));
        }
        let unit = &spec.drivetrain.units[unit_idx];
        Some(ThermalPairing {
            unit_idx,
            emotor_path: unit
                .thermal
                .as_ref()
                .expect("filtered on thermal")
                .as_str()
                .to_owned(),
            ptm_path: unit.source.as_str().to_owned(),
        })
    } else {
        notes.push(
            "battery present with no `.emotor` drive-unit thermal model — the pack marches \
             without a machine-thermal network (no thermal derate)"
                .to_owned(),
        );
        None
    };
    SlowStackPlan::Pack {
        battery_path: batt.params.as_str().to_owned(),
        thermal,
        notes,
    }
}
