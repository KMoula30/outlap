// SPDX-License-Identifier: AGPL-3.0-only
//! Reference newtypes and the `x-*` extensions bag.
//!
//! Each referenced file (or sidecar table) is a distinct newtype so the type system prevents, say,
//! passing a `.tyr` path where a `.ptm` is expected. All are transparent strings on the wire.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Macro: define a `#[serde(transparent)]` string newtype with a doc comment.
macro_rules! ref_newtype {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(
            Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema,
        )]
        #[serde(transparent)]
        pub struct $name(pub String);

        impl $name {
            /// The underlying reference string.
            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl From<&str> for $name {
            fn from(s: &str) -> Self {
                Self(s.to_owned())
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }
    };
}

ref_newtype! {
    /// Reference to a preset the document `extends:`, e.g. `presets/formula_base`.
    PresetRef
}
ref_newtype! {
    /// Reference to a `.ptm` neutral powertrain-map file.
    PtmRef
}
ref_newtype! {
    /// Reference to a `.tyr` tire file.
    TyrRef
}
ref_newtype! {
    /// Reference to an `.emotor` electric-machine thermal file.
    EmotorRef
}
ref_newtype! {
    /// Reference to a numeric sidecar map table (parquet/CSV), e.g. an aero or K&C map.
    MapRef
}
ref_newtype! {
    /// Reference to a battery equivalent-circuit parameter file.
    BatteryRef
}
ref_newtype! {
    /// Reference to a `centerline.csv` track sidecar.
    CenterlineRef
}
ref_newtype! {
    /// In-document identifier of a `drivetrain.units[]` entry.
    ///
    /// Unlike the `*Ref` newtypes above this is **not** a file path: it is an intra-document symbol
    /// resolved against the set of unit ids declared in the same vehicle document (no `SourceLoader`,
    /// no IO). Targeted by `policy.governs` and the sidecar-install order.
    UnitId
}
ref_newtype! {
    /// In-document identifier of a shared drivetrain node (e.g. `crank`, `gearbox_out`).
    ///
    /// An intra-document symbol (not a file path). A node is declared *implicitly* by being
    /// referenced as a source's `output:` or a coupler's `from`/`to`; there is no separate node
    /// registry. Node ids are disjoint from unit ids.
    NodeId
}
ref_newtype! {
    /// In-document identifier of a `batteries` map entry.
    ///
    /// An intra-document symbol (not a file path — the file path is the distinct [`BatteryRef`]).
    /// Targeted by `DriveUnit.battery`; resolved against the `batteries` map keys.
    BatteryId
}

/// The `x-*` extension bag: vendor/experimental keys that are carried through but not interpreted.
///
/// Only keys beginning `x-` may appear here; the unknown-key walk (stage 4) routes `x-*` keys into
/// this bag with a warning and rejects any other unknown key as a hard error (§9 versioning rule).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct Extensions(pub BTreeMap<String, serde_json::Value>);

impl Extensions {
    /// Whether the bag is empty (used by `skip_serializing_if`).
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}
