// SPDX-License-Identifier: AGPL-3.0-only
//! The staged load pipeline and its public entry points.
//!
//! `load_vehicle` runs the full pipeline from a root path through a [`SourceLoader`];
//! `resolve_vehicle` runs the same stages 3–9 on an in-memory [`Vehicle`] with overrides, giving
//! identical provenance (#44). Referenced `.ptm`/`.tyr`/`.emotor` files are validated too, and
//! their `.ptm` kinds feed the topology-graph check.

mod estimate;
pub mod merge;
mod semantic;
mod topology;
mod unknown;

pub mod provenance;
pub mod report;

use serde::de::DeserializeOwned;
use serde_json::Value;

use crate::diagnostics::{SourceId, Sources, SrcSpan};
use crate::emotor::Emotor;
use crate::error::{Result, SchemaError};
use crate::io::SourceLoader;
use crate::ptm::Ptm;
use crate::tree::{self, SpanIndex, Tree};
use crate::tyr::Tyr;
use crate::vehicle::Vehicle;
use crate::schema_name;

pub use merge::Overrides;
pub use provenance::{Origin, ProvenanceMap};
pub use report::LoadedModelReport;
use report::ReportEntry;
use topology::UnitSource;

/// Options controlling a load.
#[derive(Clone, Debug, Default)]
pub struct LoadOptions {
    /// Allow documented-fallback (degraded) combinations; degradations are recorded in the report
    /// and mark the results (#40). Threaded in by the caller (typically from `sim.yaml`).
    pub allow_degraded: bool,
}

/// A fully resolved vehicle: the typed spec, per-value provenance, and the loaded-model report.
#[derive(Clone, Debug)]
pub struct ResolvedVehicle {
    /// The resolved, validated typed model (`extends` resolved away).
    pub spec: Vehicle,
    /// Where each resolved value came from.
    pub provenance: ProvenanceMap,
    /// The loaded-model report (inherited/estimated/degraded/warnings + resolved-set hash).
    pub report: LoadedModelReport,
}

/// Load and fully resolve a vehicle from `root` (a logical path understood by `loader`).
pub fn load_vehicle(
    root: &str,
    loader: &dyn SourceLoader,
    options: &LoadOptions,
) -> Result<ResolvedVehicle> {
    load_vehicle_with(root, loader, options, &Overrides::default())
}

/// Like [`load_vehicle`], but applying dotted-path [`Overrides`] (Decision #35) between the
/// extends-merge and validation stages — overridden values are schema-validated after the merge
/// and recorded in the provenance map like any other value source.
pub fn load_vehicle_with(
    root: &str,
    loader: &dyn SourceLoader,
    options: &LoadOptions,
    overrides: &Overrides,
) -> Result<ResolvedVehicle> {
    let mut sources = Sources::new();
    // Stage 0/1: load + parse the root document.
    let content = merge::load_with_fallback(root, loader)?;
    let root_id = sources.add(root, content);
    let root_tree =
        tree::parse(root_id, &sources).map_err(|e| merge::parse_to_error(e, &sources))?;
    // Stage 2: version gate (root must be a vehicle of this major).
    let version = version_gate(&root_tree, schema_name::VEHICLE, root_id, &sources)?;
    // Stage 3: resolve extends chain + merge + overrides + provenance.
    let (merged, provenance) = merge::resolve(root_tree, root_id, overrides, loader, &mut sources)?;
    finish(
        &merged,
        root_id,
        version.minor,
        provenance,
        loader,
        &mut sources,
        options,
    )
}

/// Resolve an in-memory [`Vehicle`] with dotted-path overrides, running stages 3–9. The `extends`
/// field is expected to be `None` (the in-memory object is already a concrete document).
pub fn resolve_vehicle(
    doc: &Vehicle,
    overrides: &Overrides,
    loader: &dyn SourceLoader,
    options: &LoadOptions,
) -> Result<ResolvedVehicle> {
    let mut sources = Sources::new();
    let value = serde_json::to_value(doc).map_err(|e| {
        SchemaError::deserialize(
            &sources,
            SrcSpan::blank(0),
            "",
            format!("in-memory serialize failed: {e}"),
        )
    })?;
    let root_id = sources.add("<in-memory>", value.to_string());
    let tree = merge::value_to_tree(&value, root_id);
    let (merged, provenance) = merge::resolve(tree, root_id, overrides, loader, &mut sources)?;
    // An in-memory doc carries no version string; it is by construction this build's MINOR.
    finish(
        &merged,
        root_id,
        crate::current_minor(schema_name::VEHICLE),
        provenance,
        loader,
        &mut sources,
        options,
    )
}

/// Stages 4–9: unknown-key walk → deserialize → semantic → referenced files → topology →
/// estimate → report.
fn finish(
    merged: &Tree,
    root_id: SourceId,
    root_minor: u16,
    mut provenance: ProvenanceMap,
    loader: &dyn SourceLoader,
    sources: &mut Sources,
    options: &LoadOptions,
) -> Result<ResolvedVehicle> {
    let mut report = LoadedModelReport::default();

    // Stage 4/5: value + spans, capture extensions, unknown walk, deserialize.
    let (mut value, index) = tree::to_value(merged);
    unknown::check::<Vehicle>(
        &value,
        &index,
        sources,
        root_id,
        root_minor,
        crate::current_minor(schema_name::VEHICLE),
        &mut report.warnings,
    )?;
    unknown::capture_top_level_extensions(&mut value);
    let mut spec: Vehicle = deserialize(&value, &index, sources, root_id)?;

    // Stage 6: semantic checks.
    semantic::check_vehicle(&spec, &index, sources, root_id)?;

    // Referenced files: validate each and collect .ptm kinds for topology.
    let unit_sources = load_referenced(
        &spec,
        loader,
        sources,
        &mut report,
        &index,
        root_id,
        options,
    )?;

    // Stage 7: topology graph.
    topology::check(&spec, &unit_sources, &index, sources, root_id)?;

    // Stage 8: estimation.
    estimate::estimate(&mut spec, &mut provenance, &mut report.estimated);

    // Provenance → inherited report lines.
    for (pointer, origin) in &provenance.entries {
        if let Origin::Inherited { preset, .. } = origin {
            report.inherited.push(ReportEntry::new(
                pointer,
                format!("inherited from `{preset}`"),
            ));
        }
    }

    // Stage 9: resolved-set hash.
    report.resolved_hash = report::resolved_hash(&spec);

    Ok(ResolvedVehicle {
        spec,
        provenance,
        report,
    })
}

/// Load and validate all files referenced by the vehicle, returning per-unit `.ptm` source info.
#[allow(clippy::too_many_arguments)]
fn load_referenced(
    spec: &Vehicle,
    loader: &dyn SourceLoader,
    sources: &mut Sources,
    report: &mut LoadedModelReport,
    index: &SpanIndex,
    root_id: SourceId,
    options: &LoadOptions,
) -> Result<Vec<Option<UnitSource>>> {
    // Tires.
    for tyr_ref in [&spec.tires.front, &spec.tires.rear] {
        let (tyr, id, index, _) =
            load_typed::<Tyr>(tyr_ref.as_str(), schema_name::TYR, loader, sources)?;
        semantic::check_tyr(&tyr, &index, sources, id, &mut report.warnings)?;
    }
    // Battery documents (+ the policy↔pack integrity contract, D-M6-13). The MGU-K is now a
    // `drivetrain.units[]` entry, so its `.ptm` loads through the drive-units loop below like any
    // other source — there is no longer a separate `ers.mgu_k` leg. A policy-governed unit's pack
    // must be loadable (the energy manager schedules it) — a hard error unless `allow_degraded`.
    load_battery_referenced(spec, loader, sources, report, index, root_id, options)?;
    // Drive units: source .ptm (+ optional thermal .emotor).
    let mut unit_sources = Vec::with_capacity(spec.drivetrain.units.len());
    for unit in &spec.drivetrain.units {
        let (ptm, id, index, _) =
            load_typed::<Ptm>(unit.source.as_str(), schema_name::PTM, loader, sources)?;
        semantic::check_ptm(&ptm, &index, sources, id)?;
        if let Some(thermal) = &unit.thermal {
            let (em, eid, eindex, _) =
                load_typed::<Emotor>(thermal.as_str(), schema_name::EMOTOR, loader, sources)?;
            semantic::check_emotor(&em, &eindex, sources, eid)?;
        }
        unit_sources.push(Some(UnitSource {
            kind: ptm.kind,
            upstream_ratio_applied: ptm.meta.upstream_ratio_applied.unwrap_or(true),
        }));
    }
    Ok(unit_sources)
}

/// The battery leg of [`load_referenced`]: validate each referenced `battery/1.x` pack in the
/// id-keyed `batteries` map and run the policy↔pack integrity contract (D-M6-13):
///
/// * A `policy:` overlay governs an electric unit that draws from a pack — that pack must be
///   LOADABLE (the energy manager schedules it). A missing pack is a hard error unless
///   `allow_degraded` downgrades it to a recorded degradation (results are marked). A pack that no
///   policy governs (a plain EV, or an ICE-only car) is simply inert when absent — no error.
/// * When the governed pack loads, the policy's `regulatory_window_mj` (the FIA C5.2.9 swing limit)
///   must fit within `(soc window span) × capacity.e_pack_wh`, and any `recharge_target_soc` must
///   lie inside the pack's `soc_window` — the pack ECM is the single source of truth for the window.
fn load_battery_referenced(
    spec: &Vehicle,
    loader: &dyn SourceLoader,
    sources: &mut Sources,
    report: &mut LoadedModelReport,
    index: &SpanIndex,
    root_id: SourceId,
    options: &LoadOptions,
) -> Result<()> {
    use std::collections::BTreeSet;
    let span_at = |ptr: &str| resolve_span(index, ptr, root_id);

    // Battery ids that a policy governs (load-bearing for the manager) = the packs referenced by
    // the governed units. An in-document symbol resolution (governs id → unit → unit.battery id).
    let governed_pack_ids: BTreeSet<&str> = spec
        .policy
        .iter()
        .flat_map(|p| p.governs.iter())
        .filter_map(|gid| {
            spec.drivetrain
                .units
                .iter()
                .find(|u| u.id == *gid)
                .and_then(|u| u.battery.as_ref())
                .map(crate::refs::BatteryId::as_str)
        })
        .collect();

    if spec.batteries.is_empty() {
        // No packs at all. A policy that governs a unit still needs somewhere to bank.
        if spec.policy.is_some() && !governed_pack_ids.is_empty() {
            if !options.allow_degraded {
                return Err(SchemaError::semantic(
                    sources,
                    span_at("/policy"),
                    "a `policy:` overlay governs an electric unit but the vehicle declares no \
                     `batteries:` — the energy manager needs a pack to bank into",
                    Some(
                        "add a `batteries: {<id>: {model, params}}` entry the governed unit's \
                         `battery:` names, or set `allow_degraded: true` in sim.yaml to run with an \
                         inert (budget-free, harvest-less) manager"
                            .to_owned(),
                    ),
                ));
            }
            report.degraded.push(ReportEntry::new(
                "/policy",
                "policy present without any `batteries:` — the energy manager is inert \
                 (budget-free deployment, no harvest)",
            ));
        }
        return Ok(());
    }

    // The pack the policy's window/recharge checks anchor on = the pack the FIRST governed unit
    // references (Layer-1 single-governed-pack default; multi-pack policy netting is a follow-up).
    let anchor_pack_id: Option<&str> = spec.policy.as_ref().and_then(|p| {
        p.governs.first().and_then(|gid| {
            spec.drivetrain
                .units
                .iter()
                .find(|u| u.id == *gid)
                .and_then(|u| u.battery.as_ref())
                .map(crate::refs::BatteryId::as_str)
        })
    });

    for (bid, batt) in &spec.batteries {
        let ptr = format!("/batteries/{}/params", bid.as_str());
        let parts = load_typed::<crate::battery::BatteryDoc>(
            batt.params.as_str(),
            schema_name::BATTERY,
            loader,
            sources,
        );
        let (doc, doc_id, bindex, _) = match parts {
            Ok(parts) => parts,
            Err(SchemaError::Io(crate::io::SourceError::NotFound { path })) => {
                // A pack no policy governs is inert when absent — leave the report CLEAN.
                if !governed_pack_ids.contains(bid.as_str()) {
                    continue;
                }
                if !options.allow_degraded {
                    return Err(SchemaError::semantic(
                        sources,
                        span_at(&ptr),
                        format!(
                            "battery document `{path}` (pack `{}`) not found — a policy-governed \
                             pack must be on disk (the energy manager schedules it)",
                            bid.as_str()
                        ),
                        Some(
                            "author the referenced battery/1.x document (+ its ECM parquet \
                             sidecar), or set `allow_degraded: true` in sim.yaml to run with an \
                             inert manager"
                                .to_owned(),
                        ),
                    ));
                }
                report.degraded.push(ReportEntry::new(
                    ptr,
                    format!(
                        "battery document `{path}` (pack `{}`) not present — electro slow stack inert",
                        bid.as_str()
                    ),
                ));
                continue;
            }
            Err(e) => return Err(e),
        };
        semantic::check_battery(&doc, &bindex, sources, doc_id)?;
        // Policy↔pack cross-check on the anchor pack only.
        if let (Some(policy), Some(anchor)) = (&spec.policy, anchor_pack_id) {
            if anchor == bid.as_str() {
                semantic::check_policy_pack(
                    policy,
                    &doc,
                    batt.params.as_str(),
                    index,
                    sources,
                    root_id,
                )?;
            }
        }
    }
    Ok(())
}

/// Load a referenced document of type `T`, running stages 1–5 (parse → version → unknown →
/// deserialize). Returns the value, its source id, span index, and raw JSON.
fn load_typed<T: DeserializeOwned + schemars::JsonSchema>(
    path: &str,
    expected_schema: &str,
    loader: &dyn SourceLoader,
    sources: &mut Sources,
) -> Result<(T, SourceId, SpanIndex, Value)> {
    let content = merge::load_with_fallback(path, loader)?;
    let id = sources.add(path, content);
    let tree = tree::parse(id, sources).map_err(|e| merge::parse_to_error(e, sources))?;
    let version = version_gate(&tree, expected_schema, id, sources)?;
    let (value, index) = tree::to_value(&tree);
    let mut warnings = Vec::new();
    unknown::check::<T>(
        &value,
        &index,
        sources,
        id,
        version.minor,
        crate::current_minor(expected_schema),
        &mut warnings,
    )?;
    let typed: T = deserialize(&value, &index, sources, id)?;
    Ok((typed, id, index, value))
}

/// Stage 2 — read the `schema:` version from a parsed tree and check name + major. Returns the
/// parsed version so callers can use its MINOR (e.g. to flag unknown keys as possibly-newer-schema).
fn version_gate(
    tree: &Tree,
    expected_name: &str,
    id: SourceId,
    sources: &Sources,
) -> Result<crate::version::SchemaVersion> {
    let entry = tree.get("schema").ok_or_else(|| {
        SchemaError::version(
            sources,
            SrcSpan::blank(id),
            "missing `schema:` version field",
            None,
        )
    })?;
    let span = entry.span();
    let raw = match entry {
        Tree::Scalar {
            value: Value::String(s),
            ..
        } => s.clone(),
        _ => {
            return Err(SchemaError::version(
                sources,
                span,
                "`schema:` must be a string of the form `<name>/<MAJOR>.<MINOR>`",
                None,
            ));
        }
    };
    let expected_major = crate::current_major(expected_name);
    let version: crate::version::SchemaVersion = raw.parse().map_err(|_| {
        SchemaError::version(
            sources,
            span,
            format!("malformed schema version `{raw}`"),
            Some(format!("expected `{expected_name}/{expected_major}.<MINOR>`")),
        )
    })?;
    if version.name != expected_name {
        return Err(SchemaError::version(
            sources,
            span,
            format!(
                "expected a `{expected_name}` document but found `{}`",
                version.name
            ),
            Some(format!(
                "change `schema:` to `{expected_name}/{expected_major}.0`"
            )),
        ));
    }
    if version.major != expected_major {
        return Err(SchemaError::version(
            sources,
            span,
            format!(
                "incompatible major version: file is `{}.x` but this loader is `{expected_name}/{expected_major}.x`",
                version.major
            ),
            Some("run `outlap migrate` to update the file".into()),
        ));
    }
    Ok(version)
}

/// Stage 5 — deserialize a value into `T`, mapping any error path to a source span.
fn deserialize<T: DeserializeOwned>(
    value: &Value,
    index: &SpanIndex,
    sources: &Sources,
    file: SourceId,
) -> Result<T> {
    match serde_path_to_error::deserialize::<_, T>(value) {
        Ok(t) => Ok(t),
        Err(err) => {
            let pointer = path_to_pointer(err.path());
            let span = resolve_span(index, &pointer, file);
            let message = err.inner().to_string();
            Err(SchemaError::deserialize(
                sources,
                span,
                dotted(err.path()),
                message,
            ))
        }
    }
}

/// Convert a `serde_path_to_error` path into our JSON-pointer form.
fn path_to_pointer(path: &serde_path_to_error::Path) -> String {
    use std::fmt::Write as _;

    use serde_path_to_error::Segment;
    let mut out = String::new();
    for seg in path {
        match seg {
            Segment::Seq { index } => {
                let _ = write!(out, "/{index}");
            }
            Segment::Map { key } | Segment::Enum { variant: key } => {
                out.push('/');
                out.push_str(&tree::escape_pointer(key));
            }
            Segment::Unknown => {}
        }
    }
    out
}

/// A human-friendly dotted rendering of a deserialize path.
fn dotted(path: &serde_path_to_error::Path) -> String {
    let s = path.to_string();
    if s.is_empty() {
        "<root>".into()
    } else {
        s
    }
}

/// Resolve a span for a pointer, walking up to ancestors if the exact pointer has no span.
fn resolve_span(index: &SpanIndex, pointer: &str, file: SourceId) -> SrcSpan {
    let mut p = pointer.to_owned();
    loop {
        if let Some(span) = index.span_for(&p) {
            return span;
        }
        match p.rfind('/') {
            Some(0) | None => return index.span_for("").unwrap_or_else(|| SrcSpan::blank(file)),
            Some(idx) => p.truncate(idx),
        }
    }
}

// --- Standalone referenced-file loaders (used by tests and the vehicle pipeline) -------------

/// Load and validate a standalone `.ptm` document.
pub fn load_ptm(path: &str, loader: &dyn SourceLoader) -> Result<Ptm> {
    let mut sources = Sources::new();
    let (ptm, id, index, _) = load_typed::<Ptm>(path, schema_name::PTM, loader, &mut sources)?;
    semantic::check_ptm(&ptm, &index, &sources, id)?;
    Ok(ptm)
}

/// Load and validate a standalone `.tyr` document (returns the model and any non-fatal warnings).
pub fn load_tyr(path: &str, loader: &dyn SourceLoader) -> Result<(Tyr, Vec<ReportEntry>)> {
    let mut sources = Sources::new();
    let (tyr, id, index, _) = load_typed::<Tyr>(path, schema_name::TYR, loader, &mut sources)?;
    let mut warnings = Vec::new();
    semantic::check_tyr(&tyr, &index, &sources, id, &mut warnings)?;
    Ok((tyr, warnings))
}

/// Load and validate a standalone `.emotor` document.
pub fn load_emotor(path: &str, loader: &dyn SourceLoader) -> Result<Emotor> {
    let mut sources = Sources::new();
    let (em, id, index, _) = load_typed::<Emotor>(path, schema_name::EMOTOR, loader, &mut sources)?;
    semantic::check_emotor(&em, &index, &sources, id)?;
    Ok(em)
}

/// Load and validate a standalone `battery/1.0` document.
pub fn load_battery(path: &str, loader: &dyn SourceLoader) -> Result<crate::battery::BatteryDoc> {
    let mut sources = Sources::new();
    let (b, id, index, _) =
        load_typed::<crate::battery::BatteryDoc>(path, schema_name::BATTERY, loader, &mut sources)?;
    semantic::check_battery(&b, &index, &sources, id)?;
    Ok(b)
}

/// Load and validate a standalone `track.yaml` document (the referenced `centerline.csv` is parsed
/// by the `outlap-track` crate, which owns the geometry).
pub fn load_track_doc(path: &str, loader: &dyn SourceLoader) -> Result<crate::track::TrackDoc> {
    let mut sources = Sources::new();
    let (doc, id, index, _) =
        load_typed::<crate::track::TrackDoc>(path, schema_name::TRACK, loader, &mut sources)?;
    semantic::check_track(&doc, &index, &sources, id)?;
    Ok(doc)
}

/// Load and validate a standalone `conditions.yaml` document.
pub fn load_conditions(
    path: &str,
    loader: &dyn SourceLoader,
) -> Result<crate::conditions::Conditions> {
    let mut sources = Sources::new();
    let (c, id, index, _) = load_typed::<crate::conditions::Conditions>(
        path,
        schema_name::CONDITIONS,
        loader,
        &mut sources,
    )?;
    semantic::check_conditions(&c, &index, &sources, id)?;
    Ok(c)
}

/// Load and validate a standalone `sim.yaml` document.
pub fn load_sim(path: &str, loader: &dyn SourceLoader) -> Result<crate::sim::Sim> {
    let mut sources = Sources::new();
    let (sim, id, index, _) =
        load_typed::<crate::sim::Sim>(path, schema_name::SIM, loader, &mut sources)?;
    semantic::check_sim(&sim, &index, &sources, id)?;
    Ok(sim)
}
