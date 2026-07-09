// SPDX-License-Identifier: AGPL-3.0-only
//! [`TireModel`] — the static (no-`dyn`) choice between the MF6.1 and brush force models.
//!
//! One `.tyr` document can carry an MF6.1 force core, a [`crate::TyrBrush`] block, or both. The
//! tier-selection rule ([`TireModel::from_tyr`]) is: build **MF6.1** whenever the full pure-slip
//! force core is present (it is the higher-fidelity model, and a partial set never constructs one);
//! otherwise, if a brush block is present, build a **brush** tire; otherwise the document has no
//! force model at all (prevented upstream by schema validation, surfaced here defensively).

use num_traits::Float;
use outlap_schema::load::report::ReportEntry;
use outlap_schema::tyr::{Tyr, REQUIRED_FORCE_KEYS};
use thiserror::Error;

use crate::brush::Brush;
use crate::mf61::params::Mf61BuildError;
use crate::mf61::peak::{peak_mu_x, peak_mu_y};
use crate::mf61::Mf61;
use crate::slip::{SlipState, TireForces};

/// Failure to build a [`TireModel`] from a `.tyr` document.
#[derive(Debug, Error)]
pub enum TireBuildError {
    /// The MF6.1 parameter extraction failed (a required force coefficient was absent/invalid).
    #[error(transparent)]
    Mf61(#[from] Mf61BuildError),
    /// The document carries neither a complete MF6.1 force core nor a `brush` block, so no force
    /// model can be built. Schema validation rejects this earlier; this guards direct callers.
    #[error("`.tyr` has no MF6.1 force core and no `brush` block — no tire force model to build")]
    NoForceModel,
}

/// A statically-dispatched tire force model: either MF6.1 or brush.
#[derive(Clone, Debug)]
pub enum TireModel<T> {
    /// The empirical Magic Formula 6.1 model.
    Mf61(Mf61<T>),
    /// The physical brush model.
    Brush(Brush<T>),
}

impl<T: Float> TireModel<T> {
    /// Select and build a force model from a loaded `.tyr` document, plus loaded-model notes.
    ///
    /// MF6.1 is chosen iff the full [`REQUIRED_FORCE_KEYS`] core is present; else a brush tire is
    /// built from the [`crate::TyrBrush`] block; else [`TireBuildError::NoForceModel`].
    pub fn from_tyr(tyr: &Tyr) -> Result<(Self, Vec<ReportEntry>), TireBuildError> {
        let force_full = REQUIRED_FORCE_KEYS
            .iter()
            .all(|k| tyr.mf61.0.contains_key(*k));
        if force_full {
            let (mf, notes) = Mf61::from_tyr(tyr)?;
            Ok((TireModel::Mf61(mf), notes))
        } else if let Some(brush) = &tyr.brush {
            let brush = Brush::from_tyr_brush(brush);
            let notes = vec![ReportEntry::new(
                "/brush",
                "brush force model in use: Mx = My = 0, and camber/pressure are ignored (brush tier)",
            )];
            Ok((TireModel::Brush(brush), notes))
        } else {
            Err(TireBuildError::NoForceModel)
        }
    }

    /// Peak longitudinal friction `μ_x` at load `fz` (N) and pressure `p` (Pa). For the brush model
    /// this is the load/pressure-independent base friction `μ0`.
    pub fn peak_mu_x(&self, fz: T, p: T) -> T {
        match self {
            TireModel::Mf61(m) => peak_mu_x(m, fz, p),
            TireModel::Brush(b) => b.mu0(),
        }
    }

    /// Peak lateral friction `μ_y` at load `fz` (N) and pressure `p` (Pa). For the brush model this
    /// is the base friction `μ0` (isotropic).
    pub fn peak_mu_y(&self, fz: T, p: T) -> T {
        match self {
            TireModel::Mf61(m) => peak_mu_y(m, fz, p),
            TireModel::Brush(b) => b.mu0(),
        }
    }

    /// Evaluate forces/moments at the contact-patch state (dispatches to the chosen model).
    pub fn forces(&self, s: &SlipState<T>) -> TireForces<T> {
        match self {
            TireModel::Mf61(m) => m.forces(s),
            TireModel::Brush(b) => b.forces(s),
        }
    }

    /// Unloaded (free) tyre radius `R_0`, m. For the Magic-Formula model this is the `UNLOADED_RADIUS`
    /// parameter; the brush model carries no geometry, so it reports a `fallback` supplied by the
    /// caller (the transient tier passes a class-typical radius, surfaced as estimated).
    pub fn unloaded_radius(&self, fallback: T) -> T {
        match self {
            TireModel::Mf61(m) => m.params().r0,
            TireModel::Brush(_) => fallback,
        }
    }
}
