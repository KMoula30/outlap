// SPDX-License-Identifier: AGPL-3.0-only
//! Dense MF6.1 parameter extraction from the open `.tyr` coefficient map.
//!
//! Converts [`outlap_schema::tyr::Mf61Coeffs`]'s `BTreeMap<String, f64>` into a flat, typed
//! [`Mf61Params<T>`] once, at assembly time (cold path — allocation is fine here; the hot
//! evaluation path never touches the map again).
//!
//! # Default table
//!
//! Absent coefficients take the MF convention defaults that make a *sparse* file degrade
//! gracefully instead of collapsing (Pacejka 2012 §4.3.2 usage notes):
//!
//! - **1.0**: every `L*` scaling factor, `RCX1`, `RCY1`, `QCZ1` (shape factors of the combined
//!   weighting / trail cosines), and `PKY2` (load-point of the cornering-stiffness peak — it
//!   sits inside an `atan` denominator).
//! - **2.0**: `PKY4` (cornering-stiffness sine exponent). Defaulting it to 0 would collapse
//!   `K_yα ≡ 0` — the standard value is 2.
//! - **16.7 m/s**: `LONGVL` (reference velocity `V0`); **1.0 m/s**: `VXLOW` (reserved for the
//!   low-speed/relaxation model, M2 PR4 — not consumed by the steady-state kernels).
//! - **0.0**: every other (additive/modifier) coefficient.
//!
//! An entirely absent family degrades to zero output (no `QDZ*` ⇒ `Mz ≡ 0`; no `QSX*`/`QSY*` ⇒
//! `Mx = My ≡ 0`; no `R*` ⇒ combined = pure slip) and each degradation is reported as a
//! [`ReportEntry`] so the loaded-model report stays honest — nothing silent.

use std::collections::BTreeMap;

use num_traits::Float;
use outlap_schema::load::report::ReportEntry;
use outlap_schema::tyr::Tyr;
use thiserror::Error;

/// Failure to build a parameter set from a coefficient map.
#[derive(Debug, Error)]
pub enum Mf61BuildError {
    /// A structurally required coefficient is absent.
    #[error("missing required MF6.1 coefficient `{key}`")]
    MissingCoefficient {
        /// The absent `.tir`-style coefficient name.
        key: &'static str,
    },
    /// A coefficient value is NaN or infinite.
    #[error("MF6.1 coefficient `{key}` must be finite, got {value}")]
    NonFinite {
        /// The offending coefficient name.
        key: String,
        /// The offending value.
        value: f64,
    },
    /// A coefficient that must be strictly positive is not.
    #[error("MF6.1 coefficient `{key}` must be > 0, got {value}")]
    NonPositive {
        /// The offending coefficient name.
        key: &'static str,
        /// The offending value.
        value: f64,
    },
}

/// Build a loaded-model report note for a coefficient family, keyed by a `/mf61/<key>` pointer.
///
/// Reuses [`ReportEntry`] (the loaded-model report's note type) rather than a bespoke struct so
/// tire notes serialize into the report identically to loader warnings — nothing silent (#41).
fn note(key: &str, detail: impl Into<String>) -> ReportEntry {
    ReportEntry::new(format!("/mf61/{key}"), detail)
}

/// Dense, typed MF6.1 parameter set (paper names; see the module-level default table).
///
/// Field names are the lower-cased `.tir` coefficient names. All values are converted to `T`
/// once at construction; evaluation never allocates or converts.
#[derive(Clone, Debug)]
#[allow(missing_docs)] // Field names ARE the documented .tir coefficient names (see module docs).
pub struct Mf61Params<T> {
    // Structural / nominal.
    pub fnomin: T,
    pub r0: T,
    pub nompres: T,
    pub longvl: T,
    /// Low-speed transition velocity (reserved for the low-speed/relaxation model, M2 PR4).
    pub vxlow: T,
    /// Whether `NOMPRES` was present (absent ⇒ `dpi ≡ 0` and pressure ratio ≡ 1).
    pub has_nompres: bool,

    // Longitudinal force, pure slip (Fx0).
    pub pcx1: T,
    pub pdx1: T,
    pub pdx2: T,
    pub pdx3: T,
    pub pex1: T,
    pub pex2: T,
    pub pex3: T,
    pub pex4: T,
    pub pkx1: T,
    pub pkx2: T,
    pub pkx3: T,
    pub phx1: T,
    pub phx2: T,
    pub pvx1: T,
    pub pvx2: T,
    pub ppx1: T,
    pub ppx2: T,
    pub ppx3: T,
    pub ppx4: T,

    // Longitudinal force, combined slip.
    pub rbx1: T,
    pub rbx2: T,
    pub rbx3: T,
    pub rcx1: T,
    pub rex1: T,
    pub rex2: T,
    pub rhx1: T,

    // Lateral force, pure slip (Fy0).
    pub pcy1: T,
    pub pdy1: T,
    pub pdy2: T,
    pub pdy3: T,
    pub pey1: T,
    pub pey2: T,
    pub pey3: T,
    pub pey4: T,
    pub pey5: T,
    pub pky1: T,
    pub pky2: T,
    pub pky3: T,
    pub pky4: T,
    pub pky5: T,
    pub pky6: T,
    pub pky7: T,
    pub phy1: T,
    pub phy2: T,
    pub pvy1: T,
    pub pvy2: T,
    pub pvy3: T,
    pub pvy4: T,
    pub ppy1: T,
    pub ppy2: T,
    pub ppy3: T,
    pub ppy4: T,
    pub ppy5: T,

    // Lateral force, combined slip.
    pub rby1: T,
    pub rby2: T,
    pub rby3: T,
    pub rby4: T,
    pub rcy1: T,
    pub rey1: T,
    pub rey2: T,
    pub rhy1: T,
    pub rhy2: T,
    pub rvy1: T,
    pub rvy2: T,
    pub rvy3: T,
    pub rvy4: T,
    pub rvy5: T,
    pub rvy6: T,

    // Aligning moment (Mz).
    pub qbz1: T,
    pub qbz2: T,
    pub qbz3: T,
    pub qbz4: T,
    pub qbz5: T,
    pub qbz9: T,
    pub qbz10: T,
    pub qcz1: T,
    pub qdz1: T,
    pub qdz2: T,
    pub qdz3: T,
    pub qdz4: T,
    pub qdz6: T,
    pub qdz7: T,
    pub qdz8: T,
    pub qdz9: T,
    pub qdz10: T,
    pub qdz11: T,
    pub qez1: T,
    pub qez2: T,
    pub qez3: T,
    pub qez4: T,
    pub qez5: T,
    pub qhz1: T,
    pub qhz2: T,
    pub qhz3: T,
    pub qhz4: T,
    pub ppz1: T,
    pub ppz2: T,
    pub ssz1: T,
    pub ssz2: T,
    pub ssz3: T,
    pub ssz4: T,

    // Overturning moment (Mx).
    pub qsx1: T,
    pub qsx2: T,
    pub qsx3: T,
    pub qsx4: T,
    pub qsx5: T,
    pub qsx6: T,
    pub qsx7: T,
    pub qsx8: T,
    pub qsx9: T,
    pub qsx10: T,
    pub qsx11: T,
    pub ppmx1: T,

    // Rolling resistance (My).
    pub qsy1: T,
    pub qsy2: T,
    pub qsy3: T,
    pub qsy4: T,
    pub qsy5: T,
    pub qsy6: T,
    pub qsy7: T,
    pub qsy8: T,

    // First-order relaxation lengths (consumed by the transient helper, M2 PR4).
    pub ptx1: T,
    pub ptx2: T,
    pub ptx3: T,
    pub pty1: T,
    pub pty2: T,
    /// Carcass longitudinal stiffness (`.tir [STRUCTURAL] LONGITUDINAL_STIFFNESS`), N/m.
    pub c_long: T,
    /// Carcass lateral stiffness (`.tir [STRUCTURAL] LATERAL_STIFFNESS`), N/m.
    pub c_lat: T,

    // Scaling factors (L*).
    pub lfzo: T,
    pub lcx: T,
    pub lmux: T,
    pub lex: T,
    pub lkx: T,
    pub lhx: T,
    pub lvx: T,
    pub lcy: T,
    pub lmuy: T,
    pub ley: T,
    pub lky: T,
    pub lkyc: T,
    pub lkzc: T,
    pub lhy: T,
    pub lvy: T,
    pub ltr: T,
    pub lres: T,
    pub lxal: T,
    pub lyka: T,
    pub lvyka: T,
    pub ls: T,
    pub lmx: T,
    pub lmy: T,
    pub lvmx: T,
    /// Relaxation-length scalings (reserved for the transient helper, M2 PR4).
    pub lsgkp: T,
    pub lsgal: T,
}

/// Representative keys per coefficient family, used for degradation notes.
const MZ_FAMILY: &[&str] = &[
    "QBZ1", "QBZ2", "QBZ3", "QBZ9", "QBZ10", "QCZ1", "QDZ1", "QDZ2", "QDZ3", "QDZ4", "QDZ6",
    "QDZ7", "QDZ8", "QDZ9", "QHZ1",
];
const MX_FAMILY: &[&str] = &[
    "QSX1", "QSX2", "QSX3", "QSX4", "QSX5", "QSX6", "QSX7", "QSX8", "QSX9", "QSX10", "QSX11",
];
const MY_FAMILY: &[&str] = &[
    "QSY1", "QSY2", "QSY3", "QSY4", "QSY5", "QSY6", "QSY7", "QSY8",
];
// Every coefficient that contributes to combined slip must appear here, or a file that sets only
// the omitted ones is falsely reported as "combined = pure" (the note must stay honest).
const COMBINED_X_FAMILY: &[&str] = &["RBX1", "RBX2", "RBX3", "RCX1", "REX1", "REX2", "RHX1"];
const COMBINED_Y_FAMILY: &[&str] = &[
    "RBY1", "RBY2", "RBY3", "RBY4", "RCY1", "REY1", "REY2", "RHY1", "RHY2", "RVY1", "RVY2", "RVY3",
    "RVY4", "RVY5", "RVY6",
];
const PRESSURE_FAMILY: &[&str] = &[
    "PPX1", "PPX2", "PPX3", "PPX4", "PPY1", "PPY2", "PPY3", "PPY4", "PPY5", "PPZ1", "PPZ2", "PPMX1",
];

fn family_present(map: &BTreeMap<String, f64>, keys: &[&str]) -> bool {
    keys.iter().any(|k| map.contains_key(*k))
}

/// Fetch `key` from the map, falling back to `default`; convert to `T`.
fn get<T: Float>(map: &BTreeMap<String, f64>, key: &str, default: f64) -> T {
    T::from(map.get(key).copied().unwrap_or(default)).unwrap_or_else(T::zero)
}

fn require(map: &BTreeMap<String, f64>, key: &'static str) -> Result<f64, Mf61BuildError> {
    map.get(key)
        .copied()
        .ok_or(Mf61BuildError::MissingCoefficient { key })
}

fn require_positive(map: &BTreeMap<String, f64>, key: &'static str) -> Result<f64, Mf61BuildError> {
    let v = require(map, key)?;
    if v > 0.0 {
        Ok(v)
    } else {
        Err(Mf61BuildError::NonPositive { key, value: v })
    }
}

impl<T: Float> Mf61Params<T> {
    /// Build from a loaded `.tyr` document.
    pub fn from_tyr(tyr: &Tyr) -> Result<(Self, Vec<ReportEntry>), Mf61BuildError> {
        Self::from_coeffs(&tyr.mf61.0)
    }

    /// Build from a raw coefficient map, applying the documented default table.
    ///
    /// Returns the parameter set plus the degradation/observation notes destined for the
    /// loaded-model report.
    #[allow(clippy::too_many_lines)] // One line per coefficient: the table IS the function.
    pub fn from_coeffs(
        map: &BTreeMap<String, f64>,
    ) -> Result<(Self, Vec<ReportEntry>), Mf61BuildError> {
        for (k, v) in map {
            if !v.is_finite() {
                return Err(Mf61BuildError::NonFinite {
                    key: k.clone(),
                    value: *v,
                });
            }
        }

        require_positive(map, "FNOMIN")?;
        require_positive(map, "UNLOADED_RADIUS")?;
        let has_nompres = map.contains_key("NOMPRES");
        if has_nompres {
            require_positive(map, "NOMPRES")?;
        }
        if map.contains_key("LONGVL") {
            require_positive(map, "LONGVL")?;
        }

        let mut notes = Vec::new();
        if !has_nompres {
            notes.push(note(
                "NOMPRES",
                "absent - inflation-pressure terms disabled (dpi = 0, p/p0 = 1)",
            ));
            if family_present(map, PRESSURE_FAMILY) {
                notes.push(note(
                    "NOMPRES",
                    "pressure coefficients (PP*) present but NOMPRES absent - they have no effect",
                ));
            }
        }
        if !family_present(map, MZ_FAMILY) {
            notes.push(note("QDZ*", "aligning-moment coefficients absent - Mz = 0"));
        }
        if !family_present(map, MX_FAMILY) {
            notes.push(note(
                "QSX*",
                "overturning-moment coefficients absent - Mx = 0",
            ));
        }
        if !family_present(map, MY_FAMILY) {
            notes.push(note(
                "QSY*",
                "rolling-resistance coefficients absent - My = 0",
            ));
        }
        if !family_present(map, COMBINED_X_FAMILY) {
            notes.push(note(
                "RBX*",
                "longitudinal combined-slip coefficients absent - Fx combined = pure",
            ));
        }
        if !family_present(map, COMBINED_Y_FAMILY) {
            notes.push(note(
                "RBY*",
                "lateral combined-slip coefficients absent - Fy combined = pure",
            ));
        }
        if map.contains_key("QBZ6") {
            notes.push(note(
                "QBZ6",
                "accepted but unused (the trail camber form of eq. 4.E40 uses QBZ4/QBZ5)",
            ));
        }

        let g = |key: &str, default: f64| -> T { get(map, key, default) };

        let params = Self {
            // Structural / nominal.
            fnomin: g("FNOMIN", 0.0),
            r0: g("UNLOADED_RADIUS", 0.0),
            nompres: g("NOMPRES", 0.0),
            longvl: g("LONGVL", 16.7),
            vxlow: g("VXLOW", 1.0),
            has_nompres,

            // Fx0 pure slip.
            pcx1: g("PCX1", 0.0),
            pdx1: g("PDX1", 0.0),
            pdx2: g("PDX2", 0.0),
            pdx3: g("PDX3", 0.0),
            pex1: g("PEX1", 0.0),
            pex2: g("PEX2", 0.0),
            pex3: g("PEX3", 0.0),
            pex4: g("PEX4", 0.0),
            pkx1: g("PKX1", 0.0),
            pkx2: g("PKX2", 0.0),
            pkx3: g("PKX3", 0.0),
            phx1: g("PHX1", 0.0),
            phx2: g("PHX2", 0.0),
            pvx1: g("PVX1", 0.0),
            pvx2: g("PVX2", 0.0),
            ppx1: g("PPX1", 0.0),
            ppx2: g("PPX2", 0.0),
            ppx3: g("PPX3", 0.0),
            ppx4: g("PPX4", 0.0),

            // Fx combined.
            rbx1: g("RBX1", 0.0),
            rbx2: g("RBX2", 0.0),
            rbx3: g("RBX3", 0.0),
            rcx1: g("RCX1", 1.0),
            rex1: g("REX1", 0.0),
            rex2: g("REX2", 0.0),
            rhx1: g("RHX1", 0.0),

            // Fy0 pure slip.
            pcy1: g("PCY1", 0.0),
            pdy1: g("PDY1", 0.0),
            pdy2: g("PDY2", 0.0),
            pdy3: g("PDY3", 0.0),
            pey1: g("PEY1", 0.0),
            pey2: g("PEY2", 0.0),
            pey3: g("PEY3", 0.0),
            pey4: g("PEY4", 0.0),
            pey5: g("PEY5", 0.0),
            pky1: g("PKY1", 0.0),
            pky2: g("PKY2", 1.0),
            pky3: g("PKY3", 0.0),
            pky4: g("PKY4", 2.0),
            pky5: g("PKY5", 0.0),
            pky6: g("PKY6", 0.0),
            pky7: g("PKY7", 0.0),
            phy1: g("PHY1", 0.0),
            phy2: g("PHY2", 0.0),
            pvy1: g("PVY1", 0.0),
            pvy2: g("PVY2", 0.0),
            pvy3: g("PVY3", 0.0),
            pvy4: g("PVY4", 0.0),
            ppy1: g("PPY1", 0.0),
            ppy2: g("PPY2", 0.0),
            ppy3: g("PPY3", 0.0),
            ppy4: g("PPY4", 0.0),
            ppy5: g("PPY5", 0.0),

            // Fy combined.
            rby1: g("RBY1", 0.0),
            rby2: g("RBY2", 0.0),
            rby3: g("RBY3", 0.0),
            rby4: g("RBY4", 0.0),
            rcy1: g("RCY1", 1.0),
            rey1: g("REY1", 0.0),
            rey2: g("REY2", 0.0),
            rhy1: g("RHY1", 0.0),
            rhy2: g("RHY2", 0.0),
            rvy1: g("RVY1", 0.0),
            rvy2: g("RVY2", 0.0),
            rvy3: g("RVY3", 0.0),
            rvy4: g("RVY4", 0.0),
            rvy5: g("RVY5", 0.0),
            rvy6: g("RVY6", 0.0),

            // Mz.
            qbz1: g("QBZ1", 0.0),
            qbz2: g("QBZ2", 0.0),
            qbz3: g("QBZ3", 0.0),
            qbz4: g("QBZ4", 0.0),
            qbz5: g("QBZ5", 0.0),
            qbz9: g("QBZ9", 0.0),
            qbz10: g("QBZ10", 0.0),
            qcz1: g("QCZ1", 1.0),
            qdz1: g("QDZ1", 0.0),
            qdz2: g("QDZ2", 0.0),
            qdz3: g("QDZ3", 0.0),
            qdz4: g("QDZ4", 0.0),
            qdz6: g("QDZ6", 0.0),
            qdz7: g("QDZ7", 0.0),
            qdz8: g("QDZ8", 0.0),
            qdz9: g("QDZ9", 0.0),
            qdz10: g("QDZ10", 0.0),
            qdz11: g("QDZ11", 0.0),
            qez1: g("QEZ1", 0.0),
            qez2: g("QEZ2", 0.0),
            qez3: g("QEZ3", 0.0),
            qez4: g("QEZ4", 0.0),
            qez5: g("QEZ5", 0.0),
            qhz1: g("QHZ1", 0.0),
            qhz2: g("QHZ2", 0.0),
            qhz3: g("QHZ3", 0.0),
            qhz4: g("QHZ4", 0.0),
            ppz1: g("PPZ1", 0.0),
            ppz2: g("PPZ2", 0.0),
            ssz1: g("SSZ1", 0.0),
            ssz2: g("SSZ2", 0.0),
            ssz3: g("SSZ3", 0.0),
            ssz4: g("SSZ4", 0.0),

            // Mx.
            qsx1: g("QSX1", 0.0),
            qsx2: g("QSX2", 0.0),
            qsx3: g("QSX3", 0.0),
            qsx4: g("QSX4", 0.0),
            qsx5: g("QSX5", 0.0),
            qsx6: g("QSX6", 0.0),
            qsx7: g("QSX7", 0.0),
            qsx8: g("QSX8", 0.0),
            qsx9: g("QSX9", 0.0),
            qsx10: g("QSX10", 0.0),
            qsx11: g("QSX11", 0.0),
            ppmx1: g("PPMX1", 0.0),

            // My.
            qsy1: g("QSY1", 0.0),
            qsy2: g("QSY2", 0.0),
            qsy3: g("QSY3", 0.0),
            qsy4: g("QSY4", 0.0),
            qsy5: g("QSY5", 0.0),
            qsy6: g("QSY6", 0.0),
            qsy7: g("QSY7", 0.0),
            qsy8: g("QSY8", 0.0),

            // Relaxation.
            ptx1: g("PTX1", 0.0),
            ptx2: g("PTX2", 0.0),
            ptx3: g("PTX3", 0.0),
            pty1: g("PTY1", 0.0),
            pty2: g("PTY2", 0.0),
            c_long: g("LONGITUDINAL_STIFFNESS", 0.0),
            c_lat: g("LATERAL_STIFFNESS", 0.0),

            // Scaling factors.
            lfzo: g("LFZO", 1.0),
            lcx: g("LCX", 1.0),
            lmux: g("LMUX", 1.0),
            lex: g("LEX", 1.0),
            lkx: g("LKX", 1.0),
            lhx: g("LHX", 1.0),
            lvx: g("LVX", 1.0),
            lcy: g("LCY", 1.0),
            lmuy: g("LMUY", 1.0),
            ley: g("LEY", 1.0),
            lky: g("LKY", 1.0),
            lkyc: g("LKYC", 1.0),
            lkzc: g("LKZC", 1.0),
            lhy: g("LHY", 1.0),
            lvy: g("LVY", 1.0),
            ltr: g("LTR", 1.0),
            lres: g("LRES", 1.0),
            lxal: g("LXAL", 1.0),
            lyka: g("LYKA", 1.0),
            lvyka: g("LVYKA", 1.0),
            ls: g("LS", 1.0),
            lmx: g("LMX", 1.0),
            lmy: g("LMY", 1.0),
            lvmx: g("LVMX", 1.0),
            lsgkp: g("LSGKP", 1.0),
            lsgal: g("LSGAL", 1.0),
        };

        Ok((params, notes))
    }
}
