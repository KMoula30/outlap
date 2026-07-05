// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-schema` — the outlap file-format contract.
//!
//! This crate is the single source of truth for the outlap input quartet's centerpiece —
//! the `vehicle.yaml` schema — plus the referenced-file schemas it points at (`.ptm`, `.tyr`,
//! `.emotor`). Everything downstream (the T0–T3 solvers, the powertrain firewall, the Python
//! API, the WASM surface, and the community data registry) consumes these types, so the wire
//! contract defined here is a semver boundary: additive changes bump MINOR, anything else bumps
//! MAJOR and requires a migration.
//!
//! # Design
//!
//! * **Types** ([`Vehicle`], [`ptm::Ptm`], [`tyr::Tyr`], [`emotor::Emotor`]) derive
//!   `Serialize`/`Deserialize`/`JsonSchema`; the emitted JSON Schema (draft 2020-12) is golden.
//! * **Loading** is a staged pipeline ([`load`]) — parse (span-preserving) → version gate →
//!   `extends` resolve + deep-merge (+ provenance) → unknown-key walk → single post-merge
//!   deserialize → semantic checks → topology-graph checks → estimation → loaded-model report.
//! * **Diagnostics** ([`error::SchemaError`]) carry miette source spans and did-you-mean
//!   suggestions — a bare serde error reaching the user is a bug.
//!
//! The crate is wasm-clean: no filesystem, threads, or clock. All source access goes behind the
//! [`io::SourceLoader`] trait; the filesystem loader is gated behind the `std` feature.
#![forbid(unsafe_code)]
#![deny(missing_docs)]
// clippy::pedantic is a workspace lint (warn); CI runs `-D warnings`. Curated allows for the
// pedantic lints that fight idiomatic schema/loader code (CLAUDE.md: "curated allows").
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::module_name_repetitions,
    clippy::doc_markdown,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    // SchemaError intentionally embeds source content (miette NamedSource) so diagnostics can
    // render underlined labels; it is only ever returned on the cold error path.
    clippy::result_large_err,
    // The staged pipeline / topology check read as one linear procedure; splitting hurts clarity.
    clippy::too_many_lines
)]

pub mod battery;
pub mod centerline;
pub mod conditions;
pub mod diagnostics;
pub mod emotor;
pub mod error;
pub mod io;
pub mod load;
pub mod ptm;
pub mod refs;
#[cfg(feature = "parquet")]
pub mod sidecar;
pub mod sim;
pub mod tir;
pub mod track;
pub mod tree;
pub mod tyr;
pub mod vehicle;
pub mod version;

pub use battery::BatteryDoc;
pub use centerline::{parse_centerline, Centerline};
pub use conditions::Conditions;
pub use error::SchemaError;
pub use load::{
    load_battery, load_conditions, load_sim, load_track_doc, load_vehicle, load_vehicle_with,
    resolve_vehicle, LoadOptions, Overrides, ResolvedVehicle,
};
pub use sim::Sim;
pub use tir::{
    load_tir, parse_tir, tir_to_tyr, tyr_to_tir, ThermalWearPolicy, TirDoc, TirEntry,
    TirToTyrOptions, TirValue,
};
pub use track::TrackDoc;
pub use vehicle::Vehicle;
pub use version::SchemaVersion;

/// Schema names (the `<name>` half of the `schema:` version string) understood by this crate.
pub mod schema_name {
    /// The `vehicle.yaml` document.
    pub const VEHICLE: &str = "vehicle";
    /// The `.ptm` neutral powertrain-map document.
    pub const PTM: &str = "ptm";
    /// The `.tyr` tire document.
    pub const TYR: &str = "tyr";
    /// The `.emotor` electric-machine thermal document.
    pub const EMOTOR: &str = "emotor";
    /// The `battery/1.0` equivalent-circuit parameter document.
    pub const BATTERY: &str = "battery";
    /// The `track.yaml` document.
    pub const TRACK: &str = "track";
    /// The `conditions.yaml` document.
    pub const CONDITIONS: &str = "conditions";
    /// The `sim.yaml` document.
    pub const SIM: &str = "sim";
}

/// The MAJOR version this build of the loader accepts for every schema (loaders accept same-major).
pub const SCHEMA_MAJOR: u16 = 1;
/// The MINOR version this build emits when serializing, and the highest MINOR it fully understands
/// (additive/forward-compatible within a MAJOR). Bumped to 1 for the `tyr/1.1` brush block, to
/// 2 for the `vehicle/1.2` suspension `static_ride_height_m` (ride-height aero map, §7.4), then to
/// 3 for the `ptm/1.1` optional Vdc axis (Vdc–SoC coupling, §8.4) alongside the new `battery/1.0`
/// document, then to 4 for the `sim/1.1` `flat_track` analysis flag (tier dispatch + Limebeer
/// cross-check); an unknown key in a file that declares a newer MINOR than this is flagged as
/// possibly-newer-schema.
pub const SCHEMA_MINOR: u16 = 4;
