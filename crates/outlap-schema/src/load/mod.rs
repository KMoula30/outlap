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
use crate::{schema_name, SCHEMA_MAJOR};

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
    let mut sources = Sources::new();
    // Stage 0/1: load + parse the root document.
    let content = merge::load_with_fallback(root, loader)?;
    let root_id = sources.add(root, content);
    let root_tree =
        tree::parse(root_id, &sources).map_err(|e| merge::parse_to_error(e, &sources))?;
    // Stage 2: version gate (root must be a vehicle of this major).
    version_gate(&root_tree, schema_name::VEHICLE, root_id, &sources)?;
    // Stage 3: resolve extends chain + merge + provenance.
    let (merged, provenance) = merge::resolve(
        root_tree,
        root_id,
        &Overrides::default(),
        loader,
        &mut sources,
    )?;
    finish(&merged, root_id, provenance, loader, &mut sources, options)
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
    finish(&merged, root_id, provenance, loader, &mut sources, options)
}

/// Stages 4–9: unknown-key walk → deserialize → semantic → referenced files → topology →
/// estimate → report.
fn finish(
    merged: &Tree,
    root_id: SourceId,
    mut provenance: ProvenanceMap,
    loader: &dyn SourceLoader,
    sources: &mut Sources,
    options: &LoadOptions,
) -> Result<ResolvedVehicle> {
    let mut report = LoadedModelReport::default();

    // Stage 4/5: value + spans, capture extensions, unknown walk, deserialize.
    let (mut value, index) = tree::to_value(merged);
    unknown::check::<Vehicle>(&value, &index, sources, root_id, &mut report.warnings)?;
    unknown::capture_top_level_extensions(&mut value);
    let mut spec: Vehicle = deserialize(&value, &index, sources, root_id)?;

    // Stage 6: semantic checks.
    semantic::check_vehicle(&spec, &index, sources, root_id)?;

    // Referenced files: validate each and collect .ptm kinds for topology.
    let unit_sources = load_referenced(&spec, loader, sources, &mut report)?;

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
    let _ = options; // `allow_degraded` recorded here once degraded combos exist.

    Ok(ResolvedVehicle {
        spec,
        provenance,
        report,
    })
}

/// Load and validate all files referenced by the vehicle, returning per-unit `.ptm` source info.
fn load_referenced(
    spec: &Vehicle,
    loader: &dyn SourceLoader,
    sources: &mut Sources,
    report: &mut LoadedModelReport,
) -> Result<Vec<Option<UnitSource>>> {
    // Tires.
    for tyr_ref in [&spec.tires.front, &spec.tires.rear] {
        let (tyr, id, index, _) =
            load_typed::<Tyr>(tyr_ref.as_str(), schema_name::TYR, loader, sources)?;
        semantic::check_tyr(&tyr, &index, sources, id, &mut report.warnings)?;
    }
    // ERS machine.
    if let Some(ers) = &spec.ers {
        let (ptm, id, index, _) =
            load_typed::<Ptm>(ers.mgu_k.as_str(), schema_name::PTM, loader, sources)?;
        semantic::check_ptm(&ptm, &index, sources, id)?;
    }
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
    version_gate(&tree, expected_schema, id, sources)?;
    let (value, index) = tree::to_value(&tree);
    let mut warnings = Vec::new();
    unknown::check::<T>(&value, &index, sources, id, &mut warnings)?;
    let typed: T = deserialize(&value, &index, sources, id)?;
    Ok((typed, id, index, value))
}

/// Stage 2 — read the `schema:` version from a parsed tree and check name + major.
fn version_gate(tree: &Tree, expected_name: &str, id: SourceId, sources: &Sources) -> Result<()> {
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
    let version: crate::version::SchemaVersion = raw.parse().map_err(|_| {
        SchemaError::version(
            sources,
            span,
            format!("malformed schema version `{raw}`"),
            Some(format!("expected `{expected_name}/{SCHEMA_MAJOR}.<MINOR>`")),
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
                "change `schema:` to `{expected_name}/{SCHEMA_MAJOR}.0`"
            )),
        ));
    }
    if version.major != SCHEMA_MAJOR {
        return Err(SchemaError::version(
            sources,
            span,
            format!(
                "incompatible major version: file is `{}.x` but this loader is `{expected_name}/{SCHEMA_MAJOR}.x`",
                version.major
            ),
            Some("run `outlap migrate` to update the file".into()),
        ));
    }
    Ok(())
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
