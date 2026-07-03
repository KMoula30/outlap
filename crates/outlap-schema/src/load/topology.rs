// SPDX-License-Identifier: AGPL-3.0-only
//! Stage 7 — the drivetrain topology-graph checks (§8.0).
//!
//! Builds the source → coupler → wheel graph and validates it: units must declare valid, unique
//! wheels; a lumped `drive_unit` `.ptm` must not sit behind a gearbox/fixed ratio; a rigid
//! double-drive of one wheel is a conflict; and torque vectoring cannot act across a locked/solid
//! diff on the same axle. Messages are plain-language and carry the offending spans.

use crate::diagnostics::{SourceId, Sources, SrcSpan};
use crate::error::{Result, SchemaError};
use crate::ptm::PtmKind;
use crate::tree::SpanIndex;
use crate::vehicle::{Coupler, DiffKind, Vehicle, Wheel};

/// What the pipeline learned about a unit's source `.ptm` (kind + whether an internal ratio is
/// already applied). `None` when the `.ptm` could not be loaded (the error surfaces earlier).
#[derive(Clone, Copy, Debug)]
pub struct UnitSource {
    /// The source kind.
    pub kind: PtmKind,
    /// `meta.upstream_ratio_applied` (defaults to `true` when absent).
    pub upstream_ratio_applied: bool,
}

/// Run the topology-graph checks.
pub fn check(
    spec: &Vehicle,
    unit_sources: &[Option<UnitSource>],
    index: &SpanIndex,
    sources: &Sources,
    file: SourceId,
) -> Result<()> {
    let at = |ptr: &str| index.span_for(ptr).unwrap_or_else(|| SrcSpan::blank(file));

    if spec.drivetrain.units.is_empty() {
        return Err(SchemaError::topology(
            sources,
            file,
            "drivetrain has no drive units — at least one torque source is required",
            vec![(at("/drivetrain"), "empty `units`".into())],
        ));
    }

    // Per-unit checks + build a wheel → units map.
    let mut wheel_units: std::collections::BTreeMap<Wheel, Vec<usize>> =
        std::collections::BTreeMap::new();
    for (ui, unit) in spec.drivetrain.units.iter().enumerate() {
        if unit.wheels.is_empty() {
            return Err(SchemaError::topology(
                sources,
                file,
                format!("drive unit {ui} drives no wheels"),
                vec![(
                    at(&format!("/drivetrain/units/{ui}")),
                    "no `wheels` declared".into(),
                )],
            ));
        }
        // Unique wheels within a unit.
        let mut seen = Vec::new();
        for (wi, w) in unit.wheels.iter().enumerate() {
            if seen.contains(w) {
                return Err(SchemaError::topology(
                    sources,
                    file,
                    format!("drive unit {ui} lists wheel `{w}` more than once"),
                    vec![(
                        at(&format!("/drivetrain/units/{ui}/wheels/{wi}")),
                        "duplicate wheel".into(),
                    )],
                ));
            }
            seen.push(*w);
            wheel_units.entry(*w).or_default().push(ui);
        }

        // A lumped drive_unit must not sit behind a gearbox / fixed ratio.
        if let Some(src) = unit_sources.get(ui).copied().flatten() {
            if matches!(src.kind, PtmKind::DriveUnit) && src.upstream_ratio_applied {
                for (pi, coupler) in unit.path.iter().enumerate() {
                    if matches!(coupler, Coupler::Gearbox(_) | Coupler::FixedRatio(_)) {
                        return Err(SchemaError::topology(
                            sources,
                            file,
                            format!(
                                "drive unit {ui} is a lumped `drive_unit` (ratio already applied) but its \
                                 path applies another ratio; set `meta.upstream_ratio_applied: false` in \
                                 the .ptm or remove the coupler"
                            ),
                            vec![
                                (at(&format!("/drivetrain/units/{ui}/source")), "lumped drive_unit".into()),
                                (at(&format!("/drivetrain/units/{ui}/path/{pi}")), "extra ratio here".into()),
                            ],
                        ));
                    }
                }
            }
        }
    }

    // Rigid double-drive: a wheel driven by ≥2 units where none of the driving paths contains a
    // differential is over-constrained (parallel hybrids share a diff, so they pass).
    for (wheel, units) in &wheel_units {
        if units.len() >= 2 {
            let rigid = units
                .iter()
                .all(|&ui| !path_has_diff(&spec.drivetrain.units[ui].path));
            if rigid {
                let labels = units
                    .iter()
                    .map(|&ui| {
                        (
                            at(&format!("/drivetrain/units/{ui}/wheels")),
                            format!("unit {ui} drives {wheel}"),
                        )
                    })
                    .collect();
                return Err(SchemaError::topology(
                    sources,
                    file,
                    format!(
                        "wheel `{wheel}` is rigidly driven by {} units with no differential between \
                         them — this over-constrains the wheel speed",
                        units.len()
                    ),
                    labels,
                ));
            }
        }
    }

    // Torque vectoring cannot act across a locked/solid diff on the same axle.
    if spec.drivetrain.control.torque_vectoring.enabled {
        for (ui, unit) in spec.drivetrain.units.iter().enumerate() {
            let drives_both_of_an_axle = drives_full_axle(&unit.wheels);
            let has_locking_diff = unit.path.iter().any(|c| {
                matches!(c, Coupler::Diff(d) if matches!(d.kind, DiffKind::Locked | DiffKind::Solid))
            });
            if drives_both_of_an_axle && has_locking_diff {
                return Err(SchemaError::topology(
                    sources,
                    file,
                    format!(
                        "torque vectoring is enabled but drive unit {ui} feeds a full axle through a \
                         locked/solid differential — torque cannot be vectored across it"
                    ),
                    vec![
                        (at("/drivetrain/control/torque_vectoring"), "torque vectoring enabled".into()),
                        (at(&format!("/drivetrain/units/{ui}/path")), "locked/solid diff on this axle".into()),
                    ],
                ));
            }
        }
    }

    Ok(())
}

fn path_has_diff(path: &[Coupler]) -> bool {
    path.iter().any(|c| matches!(c, Coupler::Diff(_)))
}

fn drives_full_axle(wheels: &[Wheel]) -> bool {
    let front = wheels.contains(&Wheel::Fl) && wheels.contains(&Wheel::Fr);
    let rear = wheels.contains(&Wheel::Rl) && wheels.contains(&Wheel::Rr);
    front || rear
}
