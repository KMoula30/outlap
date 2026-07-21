// SPDX-License-Identifier: AGPL-3.0-only
//! Stage 7 — the drivetrain topology-graph checks (§8.0, D-M6-13).
//!
//! Builds the source → coupler → wheel graph and validates it: each source declares exactly one
//! terminus (a private `wheels` chain **or** a shared `output` node); every source reaches at least
//! one wheel; every referenced node is produced by something; the node graph is acyclic; a lumped
//! `drive_unit` `.ptm` must not sit behind a gearbox/fixed ratio; a rigid double-drive of one wheel
//! is a conflict; and torque vectoring cannot act across a locked/solid diff on the same axle.
//!
//! A `wheels:`-only car (every source declares wheels, no top-level couplers) takes the *same*
//! flatten path it did before D-M6-13 — its per-unit coupler sequence is just `unit.path` — so its
//! validation is byte-identical.

use std::collections::{BTreeMap, BTreeSet};

use crate::diagnostics::{SourceId, Sources, SrcSpan};
use crate::error::{Result, SchemaError};
use crate::ptm::PtmKind;
use crate::tree::SpanIndex;
use crate::vehicle::{Coupler, DiffKind, DriveUnit, Vehicle, Wheel};

/// What the pipeline learned about a unit's source `.ptm` (kind + whether an internal ratio is
/// already applied). `None` when the `.ptm` could not be loaded (the error surfaces earlier).
#[derive(Clone, Copy, Debug)]
pub struct UnitSource {
    /// The source kind.
    pub kind: PtmKind,
    /// `meta.upstream_ratio_applied` (defaults to `true` when absent).
    pub upstream_ratio_applied: bool,
}

/// The flattened drive chain for one source: the ordered couplers from the source shaft to the
/// wheels it drives, plus those terminal wheels. For a `wheels:`-sugar unit this is simply
/// `(unit.path, unit.wheels)`; for a shared-node source it is `unit.path ++ (couplers from the
/// output node down to wheels)`.
struct FlatChain<'a> {
    couplers: Vec<&'a Coupler>,
    wheels: Vec<Wheel>,
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
    let dt = &spec.drivetrain;

    if dt.units.is_empty() {
        return Err(SchemaError::topology(
            sources,
            file,
            "drivetrain has no drive units — at least one torque source is required",
            vec![(at("/drivetrain"), "empty `units`".into())],
        ));
    }

    // --- Terminus XOR per unit + per coupler edge -----------------------------------------------
    for (ui, unit) in dt.units.iter().enumerate() {
        let has_wheels = !unit.wheels.is_empty();
        let has_output = unit.output.is_some();
        if has_wheels == has_output {
            let (msg, label) = if has_wheels {
                (
                    format!(
                        "drive unit `{}` declares both `wheels` and `output` — a source has exactly \
                         one terminus",
                        unit.id
                    ),
                    "both a wheel terminus and a shared-node output",
                )
            } else {
                (
                    format!(
                        "drive unit `{}` declares neither `wheels` nor `output` — a source must \
                         drive wheels or output onto a shared node",
                        unit.id
                    ),
                    "no terminus (`wheels` or `output`)",
                )
            };
            return Err(SchemaError::topology(
                sources,
                file,
                msg,
                vec![(at(&format!("/drivetrain/units/{ui}")), label.into())],
            ));
        }
    }
    for (ci, edge) in dt.couplers.iter().enumerate() {
        let has_wheels = !edge.wheels.is_empty();
        let has_to = edge.to.is_some();
        if has_wheels == has_to {
            let label = if has_wheels {
                "both a `to` node and a wheel terminus"
            } else {
                "no terminus (`to` node or `wheels`)"
            };
            return Err(SchemaError::topology(
                sources,
                file,
                format!(
                    "coupler {ci} (from `{}`) declares {} — a coupler feeds a node or terminates at \
                     wheels, not both/neither",
                    edge.from,
                    if has_wheels { "both `to` and `wheels`" } else { "neither `to` nor `wheels`" }
                ),
                vec![(at(&format!("/drivetrain/couplers/{ci}")), label.into())],
            ));
        }
    }

    // --- Node graph: produced nodes, from→to adjacency ------------------------------------------
    // A node is "produced" by being a source's `output` or a coupler's `to`.
    let mut produced: BTreeSet<&str> = BTreeSet::new();
    for unit in &dt.units {
        if let Some(n) = &unit.output {
            produced.insert(n.as_str());
        }
    }
    for edge in &dt.couplers {
        if let Some(n) = &edge.to {
            produced.insert(n.as_str());
        }
    }
    // Every coupler's `from` must be produced by something (no sourceless node).
    for (ci, edge) in dt.couplers.iter().enumerate() {
        if !produced.contains(edge.from.as_str()) {
            return Err(SchemaError::topology(
                sources,
                file,
                format!(
                    "coupler {ci} reads from node `{}`, which no source or coupler produces",
                    edge.from
                ),
                vec![(
                    at(&format!("/drivetrain/couplers/{ci}/from")),
                    "undefined / sourceless node".into(),
                )],
            ));
        }
    }

    // node → outgoing coupler indices (for reachability + cycle walks).
    let mut out_edges: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    for (ci, edge) in dt.couplers.iter().enumerate() {
        out_edges.entry(edge.from.as_str()).or_default().push(ci);
    }

    // --- Cycle detection over node→node edges (fresh 3-color DFS; not merge.rs's extends walk) ---
    detect_cycles(dt, &out_edges, &at, sources, file)?;

    // --- Flatten each source's chain + reachability ---------------------------------------------
    let mut wheel_units: BTreeMap<Wheel, Vec<usize>> = BTreeMap::new();
    let mut flats: Vec<FlatChain> = Vec::with_capacity(dt.units.len());
    for (ui, unit) in dt.units.iter().enumerate() {
        let flat = flatten_chain(unit, &dt.couplers, &out_edges);
        if flat.wheels.is_empty() {
            return Err(SchemaError::topology(
                sources,
                file,
                format!(
                    "drive unit `{}` reaches no wheel through the drivetrain graph",
                    unit.id
                ),
                vec![(
                    at(&format!("/drivetrain/units/{ui}")),
                    "source drives no wheels".into(),
                )],
            ));
        }
        // Unique wheels within a unit's terminal set.
        let mut seen = Vec::new();
        for w in &flat.wheels {
            if seen.contains(w) {
                return Err(SchemaError::topology(
                    sources,
                    file,
                    format!("drive unit `{}` drives wheel `{w}` more than once", unit.id),
                    vec![(
                        at(&format!("/drivetrain/units/{ui}")),
                        "duplicate wheel".into(),
                    )],
                ));
            }
            seen.push(*w);
            wheel_units.entry(*w).or_default().push(ui);
        }

        // A lumped drive_unit must not sit behind a gearbox / fixed ratio (anywhere on its chain).
        if let Some(src) = unit_sources.get(ui).copied().flatten() {
            if matches!(src.kind, PtmKind::DriveUnit) && src.upstream_ratio_applied {
                for coupler in &flat.couplers {
                    if matches!(coupler, Coupler::Gearbox(_) | Coupler::FixedRatio(_)) {
                        return Err(SchemaError::topology(
                            sources,
                            file,
                            format!(
                                "drive unit `{}` is a lumped `drive_unit` (ratio already applied) but its \
                                 path applies another ratio; set `meta.upstream_ratio_applied: false` in \
                                 the .ptm or remove the coupler",
                                unit.id
                            ),
                            vec![(
                                at(&format!("/drivetrain/units/{ui}/source")),
                                "lumped drive_unit with an extra ratio on its chain".into(),
                            )],
                        ));
                    }
                }
            }
        }
        flats.push(flat);
    }

    // Rigid double-drive: a wheel driven by ≥2 units where none of the connecting chains contains a
    // differential is over-constrained (parallel hybrids share a diff, so they pass).
    for (wheel, units) in &wheel_units {
        if units.len() >= 2 {
            let rigid = units.iter().all(|&ui| !chain_has_diff(&flats[ui].couplers));
            if rigid {
                let labels = units
                    .iter()
                    .map(|&ui| {
                        (
                            at(&format!("/drivetrain/units/{ui}")),
                            format!("unit `{}` drives {wheel}", dt.units[ui].id),
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
    if dt.control.torque_vectoring.enabled {
        for (ui, unit) in dt.units.iter().enumerate() {
            let drives_both_of_an_axle = drives_full_axle(&flats[ui].wheels);
            let has_locking_diff = flats[ui].couplers.iter().any(|c| {
                matches!(c, Coupler::Diff(d) if matches!(d.kind, DiffKind::Locked | DiffKind::Solid))
            });
            if drives_both_of_an_axle && has_locking_diff {
                return Err(SchemaError::topology(
                    sources,
                    file,
                    format!(
                        "torque vectoring is enabled but drive unit `{}` feeds a full axle through a \
                         locked/solid differential — torque cannot be vectored across it",
                        unit.id
                    ),
                    vec![
                        (at("/drivetrain/control/torque_vectoring"), "torque vectoring enabled".into()),
                        (at(&format!("/drivetrain/units/{ui}")), "locked/solid diff on this axle".into()),
                    ],
                ));
            }
        }
    }

    Ok(())
}

/// Build the flattened coupler chain + terminal wheels for one source, walking the shared graph
/// forward from its `output` node (or returning its private `path`/`wheels` for a sugar unit).
fn flatten_chain<'a>(
    unit: &'a DriveUnit,
    couplers: &'a [crate::vehicle::CouplerEdge],
    out_edges: &BTreeMap<&str, Vec<usize>>,
) -> FlatChain<'a> {
    let mut chain: Vec<&Coupler> = unit.path.iter().collect();
    let mut wheels: Vec<Wheel> = Vec::new();
    match &unit.output {
        None => wheels.extend_from_slice(&unit.wheels),
        Some(start) => {
            // BFS forward over coupler edges, collecting couplers + terminal wheels. `out_edges`
            // adjacency plus a visited-node guard keep this finite even before cycle detection.
            let mut visited: BTreeSet<&str> = BTreeSet::new();
            let mut queue: Vec<&str> = vec![start.as_str()];
            while let Some(node) = queue.pop() {
                if !visited.insert(node) {
                    continue;
                }
                for &ci in out_edges.get(node).map(Vec::as_slice).unwrap_or(&[]) {
                    let edge = &couplers[ci];
                    chain.push(&edge.coupler);
                    wheels.extend_from_slice(&edge.wheels);
                    if let Some(to) = &edge.to {
                        queue.push(to.as_str());
                    }
                }
            }
        }
    }
    FlatChain {
        couplers: chain,
        wheels,
    }
}

/// Fresh 3-color DFS over node→node coupler edges. A back-edge to a node on the current stack is a
/// cycle (deliberately independent of `merge.rs`'s extends-cycle walk — a different graph).
fn detect_cycles(
    dt: &crate::vehicle::Drivetrain,
    out_edges: &BTreeMap<&str, Vec<usize>>,
    at: impl Fn(&str) -> SrcSpan,
    sources: &Sources,
    file: SourceId,
) -> Result<()> {
    #[derive(Clone, Copy, PartialEq)]
    enum Color {
        White,
        Gray,
        Black,
    }
    let mut color: BTreeMap<&str, Color> = BTreeMap::new();
    // All node ids that appear anywhere in the graph.
    let mut nodes: BTreeSet<&str> = BTreeSet::new();
    for unit in &dt.units {
        if let Some(n) = &unit.output {
            nodes.insert(n.as_str());
        }
    }
    for edge in &dt.couplers {
        nodes.insert(edge.from.as_str());
        if let Some(n) = &edge.to {
            nodes.insert(n.as_str());
        }
    }
    for &n in &nodes {
        color.entry(n).or_insert(Color::White);
    }

    // Iterative DFS with an explicit stack of (node, next-edge-index).
    for &root in &nodes {
        if color[root] != Color::White {
            continue;
        }
        let mut stack: Vec<(&str, usize)> = vec![(root, 0)];
        color.insert(root, Color::Gray);
        while let Some(&mut (node, ref mut idx)) = stack.last_mut() {
            let edges = out_edges.get(node).map(Vec::as_slice).unwrap_or(&[]);
            if *idx < edges.len() {
                let ci = edges[*idx];
                *idx += 1;
                if let Some(to) = &dt.couplers[ci].to {
                    let to = to.as_str();
                    match color[to] {
                        Color::White => {
                            color.insert(to, Color::Gray);
                            stack.push((to, 0));
                        }
                        Color::Gray => {
                            return Err(SchemaError::topology(
                                sources,
                                file,
                                format!("drivetrain graph contains a cycle through node `{to}`"),
                                vec![(
                                    at(&format!("/drivetrain/couplers/{ci}")),
                                    "coupler closing the cycle".into(),
                                )],
                            ));
                        }
                        Color::Black => {}
                    }
                }
            } else {
                color.insert(node, Color::Black);
                stack.pop();
            }
        }
    }
    Ok(())
}

fn chain_has_diff(couplers: &[&Coupler]) -> bool {
    couplers.iter().any(|c| matches!(c, Coupler::Diff(_)))
}

fn drives_full_axle(wheels: &[Wheel]) -> bool {
    let front = wheels.contains(&Wheel::Fl) && wheels.contains(&Wheel::Fr);
    let rear = wheels.contains(&Wheel::Rl) && wheels.contains(&Wheel::Rr);
    front || rear
}
