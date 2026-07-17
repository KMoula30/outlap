// SPDX-License-Identifier: AGPL-3.0-only
//! [`T0Error`] — the typed error surface for T0 assembly and solving.

/// An error assembling a [`T0Vehicle`](crate::vehicle::T0Vehicle) or running the T0 passes.
#[derive(Debug, thiserror::Error)]
pub enum T0Error {
    /// A referenced `.ptm`/`.tyr` file failed to load or validate.
    #[error(transparent)]
    Load(#[from] outlap_schema::SchemaError),
    /// A torque/taper envelope could not be fitted (too few points or non-monotone speed axis —
    /// should not happen after schema validation; surfaced defensively).
    #[error("could not fit a T0 envelope: {0}")]
    Envelope(#[from] outlap_core::InterpError),
    /// A tyre force model could not be built from a `.tyr` document (a required force key is
    /// missing and no brush block is present — should not happen after schema validation;
    /// surfaced defensively).
    #[error("could not build the tyre force model: {0}")]
    TireBuild(#[from] outlap_tire::TireBuildError),
    /// A drive unit uses a gridded efficiency map, which the point-mass tier cannot read yet (no
    /// Rust sidecar-table reader). Use a constant `efficiency:` for T0, or run T1+ once maps land.
    #[error("drive unit {unit} uses a map efficiency; T0 needs a constant `efficiency` (no map reader yet)")]
    UnsupportedEfficiencyMap {
        /// Index of the offending drive unit.
        unit: usize,
    },
    /// The ERS rulebook could not be built from the `ers:` block (a malformed taper table or a
    /// power fraction rising with speed — should not happen after schema validation; surfaced
    /// defensively).
    #[error("could not build the ERS rulebook: {0}")]
    ErsRulebook(#[from] outlap_powertrain::RulebookError),
    /// The vehicle has no propulsion source T0 can use (no drive units and no ERS).
    #[error("vehicle has no drive units or ERS — nothing to propel the point mass")]
    NoDrive,
    /// The aero block has no `constant` coefficients and `allow_degraded` was not set. T0 needs
    /// constant CdA/CzA (the ride-height aero map is a T1 concern).
    #[error("aero has no `constant` block; T0 needs constant CdA/CzA (set `allow_degraded` to run with zero aero)")]
    NoConstantAero,
    /// The closed-lap forward/backward passes did not reach a fixed point within the iteration cap
    /// (never observed on physical tracks; a divergence backstop).
    #[error("closed-lap velocity passes did not converge within {iterations} iterations")]
    PassesDiverged {
        /// The iteration cap that was hit.
        iterations: usize,
    },
    /// The workspace was sized for a different number of stations than the path.
    #[error("workspace has {workspace} stations but the path has {path}")]
    WorkspaceMismatch {
        /// Workspace station count.
        workspace: usize,
        /// Path station count.
        path: usize,
    },
}

/// An error assembling a [`T1Vehicle`](crate::t1::T1Vehicle) for the trim solver.
#[derive(Debug, thiserror::Error)]
pub enum T1Error {
    /// A referenced `.tyr` file failed to load or validate.
    #[error(transparent)]
    Load(#[from] outlap_schema::SchemaError),
    /// A tyre force model could not be built from a `.tyr` document (a required force key is
    /// missing and no brush block is present — should not happen after schema validation).
    #[error("could not build the tyre force model: {0}")]
    TireBuild(#[from] outlap_tire::TireBuildError),
    /// The aero block has no `constant` coefficients and no ride-height map was installed, and
    /// `allow_degraded` was not set. T1 needs either a constant CdA/CzA fallback or an aero map.
    #[error("aero has no `constant` block and no ride-height map; set `allow_degraded` to run with zero aero")]
    NoConstantAero,
    /// A ride-height/yaw aero map referenced an axis name T1 does not recognise (expected one of
    /// `ride_height_f_mm`, `ride_height_r_mm`, `yaw_deg`, `drs_flag`).
    #[error("aero map axis `{name}` is not recognised (expected ride_height_f_mm | ride_height_r_mm | yaw_deg | drs_flag)")]
    UnknownAeroAxis {
        /// The unrecognised axis name.
        name: String,
    },
    /// The ride-height/yaw aero map could not be built into an interpolant (missing value column
    /// `cz_front_a_m2`/`cz_rear_a_m2`/`cx_a_m2`, a non-rectilinear grid, or a bad axis).
    #[error("could not build the aero map interpolant: {0}")]
    AeroMap(#[from] outlap_core::GridMapError),
    /// A powertrain peak-torque envelope could not be fitted from a `.ptm` (too few points or a
    /// non-monotone speed axis — should not happen after schema validation; surfaced defensively).
    #[error("could not fit a powertrain torque envelope: {0}")]
    Envelope(outlap_core::InterpError),
    /// A `.ptm` efficiency/loss sidecar table could not be built into an interpolant (missing the
    /// `efficiency` value column, a non-rectilinear grid, or a bad axis).
    #[error("could not build the powertrain efficiency/loss map: {0}")]
    PowertrainMap(outlap_core::GridMapError),
    /// A powertrain-map install referenced a drive-unit index outside the drivetrain.
    #[error("no drive unit at index {unit} to install a powertrain map onto")]
    UnknownDriveUnit {
        /// The out-of-range drive-unit index.
        unit: usize,
    },
    /// A machine `.emotor` thermal network could not be assembled (too many nodes, a missing
    /// capacity/conductance with no mass heuristic for the node roles, or a bad node reference).
    #[error("could not assemble the machine thermal network: {0}")]
    Thermal(String),
    /// A `battery/1.0` pack could not be assembled (an unsupported RC-pair count, a missing ECM
    /// table column, or a non-rectilinear `(soc, temp)` grid).
    #[error("could not assemble the battery pack: {0}")]
    Battery(String),
    /// A g-g-g-v envelope table (base boundary or a Decision #31 sensitivity field) could not be
    /// built into an interpolant — the axes/values were inconsistent (should not happen: the
    /// generator builds a full rectilinear grid with finite values).
    #[error("could not build the g-g-g-v envelope interpolant: {0}")]
    GgvEnvelope(outlap_core::GridMapError),
}
