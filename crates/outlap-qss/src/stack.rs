// SPDX-License-Identifier: AGPL-3.0-only
//! The slow-stack PLAN — the pure eligibility/pairing rules for the QSS electro slow-state stack.
//!
//! The Python binding used to fold these rules into its IO code; the plan function makes them a
//! typed, cargo-testable decision over the resolved vehicle alone (no loader, no bytes). The
//! binding then performs exactly the IO the plan names: load the battery document + ECM sidecar,
//! and — when a thermal pairing exists — the `.emotor` network + its unit's `.ptm`.
//!
//! Rules (M6 PR2 — the `.emotor` requirement is RELAXED so an `ers:` car's pack marches without
//! a machine-thermal network):
//!
//! 1. No `battery:` block ⇒ no stack (single-voltage evaluation).
//! 2. A battery ⇒ the pack marches. The FIRST drive unit declaring a `thermal:` `.emotor` ref
//!    carries the machine slow state; extra declarations are dropped WITH a note (nothing
//!    silent). No thermal unit ⇒ the pack marches alone (`thermal: None`).

use outlap_schema::Vehicle;

/// The typed slow-stack plan for one vehicle.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SlowStackPlan {
    /// No `battery:` block — no electro slow stack (single-voltage evaluation).
    NoBattery,
    /// A pack marches; optionally paired with ONE machine-thermal network.
    Pack {
        /// The `battery.params` document reference (vehicle-root-relative logical path).
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
    let Some(batt) = &spec.battery else {
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
