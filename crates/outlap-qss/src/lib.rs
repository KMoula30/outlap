// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-qss` — the quasi-steady-state tier: the T0 point-mass lap solver (and, later, the T1
//! g-g-g-v envelope generator).
//!
//! T0 is a **forward/backward velocity-profile solver** on the 3D road ribbon (§6.1, §11.2): it is
//! not an ODE integration but a pair of arc-length sweeps over a constant-μ friction ellipse with a
//! velocity-resolved tractive-force envelope, targeting a full lap in well under 50 ms.
//!
//! This crate is split into a cold **assembly** stage ([`vehicle`], allocations allowed) that
//! reduces a [`ResolvedVehicle`](outlap_schema::ResolvedVehicle) + `conditions` into a compact
//! [`T0Vehicle`], and a zero-allocation **solve** stage (added in a following increment) that runs
//! the passes. It is wasm-clean: source access stays behind the `SourceLoader` trait.
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::module_name_repetitions,
    clippy::doc_markdown,
    // Physics kernels index parallel SoA arrays by station; a range loop is clearer than zips.
    clippy::needless_range_loop,
    // Physics kernels use single-letter symbols (v, s, m, g, μ) by convention (Decision #33).
    clippy::many_single_char_names,
    clippy::similar_names,
    // T0Error embeds the miette-annotated SchemaError (source text) on the cold error path.
    clippy::result_large_err,
    // Cold-path assembly + curve sampling: these casts are safe at drivetrain/track sizes.
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]

pub mod error;
pub mod path;
pub mod qss;
pub mod result;
pub mod solver;
pub mod t1;
pub mod vehicle;

pub use error::{T0Error, T1Error};
pub use path::T0Path;
pub use qss::{
    solve_t0, solve_t1, tier_not_implemented, QssError, QssLap, SetupLog, SlowCoupling, SlowLog,
    WheelLog, WHEEL_ORDER,
};
pub use result::{LapResult, LineDescriptor, T0Workspace};
pub use solver::{solve_into, solve_into_ggv, solve_into_ggv_scaled, solve_lap, solve_lap_ggv};
pub use t1::{
    AeroCoeffs, AeroLumped, AeroMap, DiffModel, EnergyPoint, GgvEnvelope, MachineThermal, Pack,
    PackState, PrimaryDiff, StepOut, T1Powertrain, T1Vehicle, TrimInput, TrimOutcome, TrimState,
};
pub use vehicle::{T0Options, T0Vehicle};

/// Default arc-length step for the T0 passes, metres (§11.2). Overridable via [`T0Options::ds_m`];
/// no `sim.yaml` field carries it in M1 (that would be a MINOR schema bump — deferred).
pub const DEFAULT_DS_M: f64 = 2.0;

/// Standard gravity, m/s².
pub const G: f64 = 9.806_65;
