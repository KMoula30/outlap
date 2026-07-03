// SPDX-License-Identifier: AGPL-3.0-only
//! The loaded-model report: what was inherited, estimated, or degraded, plus a stable hash of the
//! resolved parameter set. Nothing is silent (#41); degradations mark the results (#40).

use serde::Serialize;

/// A single note in the loaded-model report.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ReportEntry {
    /// The JSON pointer the note is about.
    pub pointer: String,
    /// Human-readable explanation.
    pub detail: String,
}

impl ReportEntry {
    /// Construct a report entry.
    pub fn new(pointer: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            pointer: pointer.into(),
            detail: detail.into(),
        }
    }
}

/// A structured account of how a vehicle was resolved.
#[derive(Clone, Debug, Default, Serialize)]
pub struct LoadedModelReport {
    /// Values inherited from presets/overlays.
    pub inherited: Vec<ReportEntry>,
    /// Values filled by estimation heuristics.
    pub estimated: Vec<ReportEntry>,
    /// Degraded/fallback combinations (only present when `allow_degraded` was set).
    pub degraded: Vec<ReportEntry>,
    /// Non-fatal warnings (e.g. unknown MF6.1 coefficient keys, ignored `x-*` keys).
    pub warnings: Vec<ReportEntry>,
    /// blake3 hex hash of the canonical resolved parameter set (results record this).
    pub resolved_hash: String,
}

impl LoadedModelReport {
    /// Whether the model resolved without any degradation.
    pub fn is_clean(&self) -> bool {
        self.degraded.is_empty()
    }
}

/// Compute the canonical blake3 hash of the resolved parameter set.
///
/// The vehicle is serialized to canonical JSON (sorted keys via `serde_json::Value` → `BTreeMap`
/// ordering is not guaranteed by serde_json's Map, so we re-serialize through a sorted structure)
/// to make the hash independent of field emission order.
pub fn resolved_hash(spec: &crate::vehicle::Vehicle) -> String {
    let value = serde_json::to_value(spec).unwrap_or(serde_json::Value::Null);
    let canonical = canonicalize(&value);
    let bytes = serde_json::to_vec(&canonical).unwrap_or_default();
    blake3::hash(&bytes).to_hex().to_string()
}

/// Recursively sort object keys so the serialized form is canonical.
fn canonicalize(value: &serde_json::Value) -> serde_json::Value {
    match value {
        serde_json::Value::Object(map) => {
            let mut sorted: std::collections::BTreeMap<String, serde_json::Value> =
                std::collections::BTreeMap::new();
            for (k, v) in map {
                sorted.insert(k.clone(), canonicalize(v));
            }
            serde_json::to_value(sorted).unwrap_or(serde_json::Value::Null)
        }
        serde_json::Value::Array(items) => {
            serde_json::Value::Array(items.iter().map(canonicalize).collect())
        }
        other => other.clone(),
    }
}
