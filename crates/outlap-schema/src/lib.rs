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

pub mod diagnostics;
pub mod emotor;
pub mod error;
pub mod io;
pub mod load;
pub mod ptm;
pub mod refs;
pub mod tree;
pub mod tyr;
pub mod vehicle;
pub mod version;

pub use error::SchemaError;
pub use load::{load_vehicle, resolve_vehicle, LoadOptions, Overrides, ResolvedVehicle};
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
}

/// The MAJOR version this build of the loader accepts for every schema (loaders accept same-major).
pub const SCHEMA_MAJOR: u16 = 1;
/// The MINOR version this build emits when serializing.
pub const SCHEMA_MINOR: u16 = 0;
