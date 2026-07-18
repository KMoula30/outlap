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

/// The highest MINOR this build fully understands for each document kind, replacing the former
/// single global counter (which conflated unrelated documents: a `tyr` bump inflated the `vehicle`
/// minor). Additive/forward-compatible within a MAJOR; an unknown key in a file that declares a
/// newer MINOR than this table is flagged as possibly-newer-schema.
///
/// Per-document history — `vehicle`: 1.2 suspension `static_ride_height_m` (§7.4), 1.5 optional
/// `driver` section (§7.7), 1.6 rule-based control layer (torque-vectoring `max_yaw_moment_nm`
/// cap §8.0 + brakes `regen_blend` §7.6), 1.7 optional ERS
/// `elec_mech_factor` + `recovery` recharge-phase fields (§8.3, M6/PR1), 1.8 optional `fuel`
/// block (§8.1 mass/CG/flow-limit, M6/PR5) + drivetrain `shift_maps` (§8.3 named up-shift maps,
/// M6/PR5); `ptm`: 1.1 optional Vdc
/// axis (§8.4), 1.2 `max_regen_torque_nm_vs_speed` (§7.6); `tyr`: 1.1 brush block; `battery`: 1.1
/// `regen_derate_vs_temp` (§7.6), 1.2 optional 2nd RC pair (`ecm.rc_pairs: 2` + `r2_ohm`/`tau2_s`
/// sidecar columns, §8.4, M6/PR4); `sim`: 1.1 `flat_track` analysis flag.
///
/// # Validation-tightening policy
///
/// Semantic validation may be TIGHTENED within a MAJOR only for values that are *always
/// meaningless* — inputs no consumer could ever have interpreted (a rising `power_frac` speed
/// taper, a non-positive capacity). Rejecting a value that some past consumer gave meaning to is
/// a behavior change and bumps MAJOR.
///
/// # Field-semantics policy
///
/// The documented *meaning* of a field may change only while the field has ZERO consumers; the
/// moment any code reads it, a semantics change is MAJOR (a migration, not a doc edit). Worked
/// example: `ers.override_mode.extra_energy_per_lap_mj` was documented as extra *deployment*
/// energy while nothing consumed it; M6/PR1 corrected it to extra *harvest* allowance (FIA 2026
/// C5.2.10iii) as a doc-only MINOR change precisely because the field was dormant. After PR1 the
/// rulebook consumes it — any further semantics change is MAJOR.
pub fn current_minor(name: &str) -> u16 {
    match name {
        schema_name::VEHICLE => 8,
        schema_name::PTM | schema_name::BATTERY => 2,
        schema_name::TYR | schema_name::SIM => 1,
        // emotor, track, conditions (and anything unknown) have had no additive change since 1.0.
        _ => 0,
    }
}
