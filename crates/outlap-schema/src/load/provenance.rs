// SPDX-License-Identifier: AGPL-3.0-only
//! Per-value provenance: where each resolved scalar came from (base, preset, overlay, dotted
//! override, estimated, or default). Recorded during merge/estimation and surfaced in the report.

use std::collections::BTreeMap;

/// The origin of a resolved value, keyed by JSON pointer in [`ProvenanceMap`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Origin {
    /// Written directly in the root document.
    Base {
        /// Source file display name.
        file: String,
    },
    /// Inherited from an `extends:` preset.
    Inherited {
        /// The preset reference.
        preset: String,
        /// Source file display name.
        file: String,
    },
    /// Set by a named overlay file.
    Overlay {
        /// Source file display name.
        file: String,
    },
    /// Set by a programmatic dotted-path override (#35).
    DottedOverride {
        /// The dotted path.
        path: String,
    },
    /// Filled by a documented estimation heuristic (#41).
    Estimated {
        /// The heuristic name.
        heuristic: &'static str,
    },
    /// A schema default.
    Default,
}

impl Origin {
    /// A short label for reports.
    pub fn label(&self) -> &'static str {
        match self {
            Origin::Base { .. } => "base",
            Origin::Inherited { .. } => "inherited",
            Origin::Overlay { .. } => "overlay",
            Origin::DottedOverride { .. } => "override",
            Origin::Estimated { .. } => "estimated",
            Origin::Default => "default",
        }
    }
}

/// A map from JSON pointer (`/chassis/mass_kg`) to the [`Origin`] of that resolved value.
#[derive(Clone, Debug, Default)]
pub struct ProvenanceMap {
    /// The pointer → origin entries.
    pub entries: BTreeMap<String, Origin>,
}

impl ProvenanceMap {
    /// Record the origin of a pointer.
    pub fn set(&mut self, pointer: impl Into<String>, origin: Origin) {
        self.entries.insert(pointer.into(), origin);
    }

    /// The origin of a pointer, if known.
    pub fn get(&self, pointer: &str) -> Option<&Origin> {
        self.entries.get(pointer)
    }
}
