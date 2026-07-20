// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-py` — the `outlap_core` Python extension module (HANDOFF §11.1b).
//!
//! Thin, numpy-friendly bindings over the Rust core: the MF6.1 tire model (`Tyre`), the 3D track
//! (`Track`), the min-curvature racing line, and the T0 point-mass lap solver (`Lap`). The typed,
//! documented user API lives on the Python side (`outlap.core`); this layer only converts types
//! and maps errors, never adds logic.
//!
//! This is the sanctioned FFI crate (CLAUDE.md): PyO3's macros generate `unsafe` glue, so —
//! uniquely in the workspace — `forbid(unsafe_code)` is not applied here.

#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::needless_pass_by_value,
    clippy::doc_markdown,
    // Channel names mirror the physics API (s, v, x, y, z, n — paper symbols).
    clippy::many_single_char_names,
    clippy::similar_names,
    // The 5-tuple of force arrays IS the FFI contract; a type alias would just rename it.
    clippy::type_complexity,
    // `lap_time_s` matches the Rust LapResult field name (public contract).
    clippy::struct_field_names
)]

mod artifacts;
mod assembly;
mod convert;
mod qss_entry;
mod transient_entry;

/// Shared import surface for the split modules (glob-imported as `use crate::prelude::*`).
/// Re-exports the external deps and every sibling module so cross-module refs resolve without
/// per-module import curation; glob imports do not trigger unused-import lints.
pub(crate) mod prelude {
    pub(crate) use crate::artifacts::*;
    pub(crate) use crate::assembly::*;
    pub(crate) use crate::convert::*;
    pub(crate) use crate::qss_entry::*;
    pub(crate) use crate::transient_entry::*;
    pub(crate) use numpy::{IntoPyArray, PyArray1, PyArray2, PyReadonlyArray1};
    pub(crate) use outlap_qss::{
        solve_t0, solve_t1, Couplings, ErsCoupling, GgvEnvelope, LapRequest, LineDescriptor,
        MachineThermal, Pack, PackState, QssLap, SetupLog, SlowCoupling, SlowLog, T0Options,
        T0Path, T0Vehicle, T1Vehicle, TireSlowLog, TireStateRes, TireThermalMarch,
        TireThermalState, WheelLog, DEFAULT_DS_M, WHEEL_ORDER,
    };
    pub(crate) use outlap_raceline::{
        min_curvature_line, min_curvature_line_weighted, raceline_stations, RacelineOptions,
    };
    pub(crate) use outlap_schema::io::FsLoader;
    pub(crate) use outlap_schema::load::load_tyr;
    pub(crate) use outlap_schema::load::report::ReportEntry;
    pub(crate) use outlap_schema::sim::{Sim, Tier};
    pub(crate) use outlap_schema::{
        load_conditions, load_sim, load_vehicle_with, Conditions, LoadOptions, Overrides,
        ResolvedVehicle,
    };
    pub(crate) use outlap_tire::{peak_mu_x, peak_mu_y, Mf61, SlipState};
    pub(crate) use pyo3::exceptions::{PyFileNotFoundError, PyValueError};
    pub(crate) use pyo3::prelude::*;
    pub(crate) use std::collections::HashMap;
    pub(crate) use std::path::Path;
    pub(crate) use std::sync::{LazyLock, Mutex};
}

use crate::prelude::*;

/// The `outlap_core` extension module.
#[pymodule]
fn outlap_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Tyre>()?;
    m.add_class::<Track>()?;
    m.add_class::<Raceline>()?;
    m.add_class::<Lap>()?;
    m.add_class::<TransientLap>()?;
    m.add_class::<QssStint>()?;
    m.add_class::<TransientStint>()?;
    m.add_class::<Envelope>()?;
    m.add_function(wrap_pyfunction!(min_curvature, m)?)?;
    m.add_function(wrap_pyfunction!(time_weighted, m)?)?;
    m.add_function(wrap_pyfunction!(solve_lap, m)?)?;
    m.add_function(wrap_pyfunction!(solve_transient_lap, m)?)?;
    m.add_function(wrap_pyfunction!(solve_stint, m)?)?;
    m.add_function(wrap_pyfunction!(solve_transient_stint, m)?)?;
    m.add_function(wrap_pyfunction!(vehicle_report, m)?)?;
    m.add("DEFAULT_DS_M", DEFAULT_DS_M)?;
    Ok(())
}
