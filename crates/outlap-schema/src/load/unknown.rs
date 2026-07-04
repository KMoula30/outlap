// SPDX-License-Identifier: AGPL-3.0-only
//! Stage 4 — the unknown-key walk.
//!
//! serde silently *ignores* unknown fields (we deliberately never use `deny_unknown_fields`,
//! because it cannot combine with `x-` passthrough and its message carries no span or suggestion).
//! So we walk the merged value against the type's JSON Schema and, for every key not known at its
//! level: route `x-*` to a warning (carried, not interpreted) and reject anything else as a hard
//! error with a did-you-mean suggestion.
//!
//! The walk is permissive around enum branches (it checks keys against the *union* of a `oneOf`'s
//! object branches) so it never false-positives; its job is to catch typos in plain structs.

use std::collections::BTreeSet;

use schemars::JsonSchema;
use serde_json::Value;

use crate::diagnostics::{SourceId, Sources, SrcSpan};
use crate::error::{Result, SchemaError};
use crate::load::report::ReportEntry;
use crate::tree::SpanIndex;

/// Walk `value` (a document of type `T`) against `T`'s JSON Schema, reporting unknown keys.
///
/// `doc_minor` is the file's declared schema MINOR; when it exceeds [`crate::SCHEMA_MINOR`] an
/// unknown-key error gains a hint that the key may be a field added in a newer schema version.
pub fn check<T: JsonSchema>(
    value: &Value,
    index: &SpanIndex,
    sources: &Sources,
    file: SourceId,
    doc_minor: u16,
    warnings: &mut Vec<ReportEntry>,
) -> Result<()> {
    let root = schemars::schema_for!(T).to_value();
    let defs = root
        .get("$defs")
        .and_then(Value::as_object)
        .cloned()
        .unwrap_or_default();
    let ctx = Ctx {
        defs,
        index,
        sources,
        file,
        doc_minor,
    };
    ctx.walk(value, &root, "")?;
    // Collect x-* warnings on a second, cheap pass at every object level.
    collect_x_warnings(value, "", warnings);
    Ok(())
}

struct Ctx<'a> {
    defs: serde_json::Map<String, Value>,
    index: &'a SpanIndex,
    sources: &'a Sources,
    file: SourceId,
    doc_minor: u16,
}

impl Ctx<'_> {
    fn at(&self, pointer: &str) -> SrcSpan {
        self.index
            .span_for(pointer)
            .unwrap_or_else(|| SrcSpan::blank(self.file))
    }

    /// A hint for the case where the file declares a newer schema MINOR than this build supports —
    /// an unknown key may simply be a field added in that newer version.
    fn newer_schema_hint(&self) -> Option<String> {
        (self.doc_minor > crate::SCHEMA_MINOR).then(|| {
            format!(
                "the file declares schema minor {} but this build supports up to minor {}; this key \
                 may be a field added in a newer schema version",
                self.doc_minor,
                crate::SCHEMA_MINOR,
            )
        })
    }

    fn resolve<'s>(&'s self, schema: &'s Value) -> &'s Value {
        let mut current = schema;
        // Follow $ref and single-member allOf a bounded number of times.
        for _ in 0..8 {
            if let Some(reference) = current.get("$ref").and_then(Value::as_str) {
                if let Some(name) = reference.rsplit('/').next() {
                    if let Some(target) = self.defs.get(name) {
                        current = target;
                        continue;
                    }
                }
            }
            break;
        }
        current
    }

    fn walk(&self, value: &Value, schema: &Value, pointer: &str) -> Result<()> {
        let schema = self.resolve(schema);
        match value {
            Value::Object(map) => {
                let allowed = self.allowed_keys(schema);
                for (key, child) in map {
                    if let Some(allowed) = &allowed {
                        if !allowed.contains(key.as_str()) && !key.starts_with("x-") {
                            let hint = crate::diagnostics::suggest(
                                key,
                                allowed.iter().map(String::as_str),
                            )
                            .map(|s| format!("did you mean `{s}`?"))
                            .or_else(|| self.newer_schema_hint());
                            let child_ptr =
                                format!("{pointer}/{}", crate::tree::escape_pointer(key));
                            return Err(SchemaError::unknown_field(
                                self.sources,
                                self.at(&child_ptr),
                                key,
                                hint,
                            ));
                        }
                    }
                    if key.starts_with("x-") {
                        continue;
                    }
                    if let Some(child_schema) = self.subschema_for_key(schema, key) {
                        let child_ptr = format!("{pointer}/{}", crate::tree::escape_pointer(key));
                        self.walk(child, &child_schema, &child_ptr)?;
                    }
                }
                Ok(())
            }
            Value::Array(items) => {
                if let Some(item_schema) = self.array_items(schema) {
                    for (i, item) in items.iter().enumerate() {
                        self.walk(item, &item_schema, &format!("{pointer}/{i}"))?;
                    }
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    /// The set of allowed keys at this schema level, or `None` if the shape is unconstrained.
    fn allowed_keys(&self, schema: &Value) -> Option<BTreeSet<String>> {
        let schema = self.resolve(schema);
        if let Some(props) = schema.get("properties").and_then(Value::as_object) {
            return Some(props.keys().cloned().collect());
        }
        for combiner in ["oneOf", "anyOf", "allOf"] {
            if let Some(branches) = schema.get(combiner).and_then(Value::as_array) {
                let mut set = BTreeSet::new();
                let mut any = false;
                for branch in branches {
                    if let Some(keys) = self.allowed_keys(branch) {
                        any = true;
                        set.extend(keys);
                    }
                }
                if any {
                    return Some(set);
                }
            }
        }
        None
    }

    /// The subschema to recurse into for `key`, if determinable.
    fn subschema_for_key(&self, schema: &Value, key: &str) -> Option<Value> {
        let schema = self.resolve(schema);
        if let Some(props) = schema.get("properties").and_then(Value::as_object) {
            if let Some(sub) = props.get(key) {
                return Some(sub.clone());
            }
        }
        for combiner in ["oneOf", "anyOf", "allOf"] {
            if let Some(branches) = schema.get(combiner).and_then(Value::as_array) {
                for branch in branches {
                    if let Some(sub) = self.subschema_for_key(branch, key) {
                        return Some(sub);
                    }
                }
            }
        }
        None
    }

    /// The element schema for an array, if determinable (`items` or the first `prefixItems`).
    fn array_items(&self, schema: &Value) -> Option<Value> {
        let schema = self.resolve(schema);
        if let Some(items) = schema.get("items") {
            if items.is_object() {
                return Some(items.clone());
            }
        }
        if let Some(prefix) = schema.get("prefixItems").and_then(Value::as_array) {
            if let Some(first) = prefix.first() {
                return Some(first.clone());
            }
        }
        None
    }
}

/// Record a warning for every `x-*` key seen (they are carried through, not interpreted).
fn collect_x_warnings(value: &Value, pointer: &str, warnings: &mut Vec<ReportEntry>) {
    match value {
        Value::Object(map) => {
            for (k, v) in map {
                let child = format!("{pointer}/{}", crate::tree::escape_pointer(k));
                if k.starts_with("x-") {
                    warnings.push(ReportEntry::new(
                        &child,
                        format!("extension key `{k}` carried through (not interpreted)"),
                    ));
                }
                collect_x_warnings(v, &child, warnings);
            }
        }
        Value::Array(items) => {
            for (i, item) in items.iter().enumerate() {
                collect_x_warnings(item, &format!("{pointer}/{i}"), warnings);
            }
        }
        _ => {}
    }
}

/// Move top-level `x-*` keys of a vehicle object into its `extensions` map so they are preserved
/// through deserialize (serde would otherwise drop them). Nested `x-*` keys are left for serde to
/// ignore.
pub fn capture_top_level_extensions(value: &mut Value) {
    let Some(obj) = value.as_object_mut() else {
        return;
    };
    let x_keys: Vec<String> = obj
        .keys()
        .filter(|k| k.starts_with("x-"))
        .cloned()
        .collect();
    if x_keys.is_empty() {
        return;
    }
    let mut ext = obj
        .remove("extensions")
        .and_then(|v| v.as_object().cloned())
        .unwrap_or_default();
    for k in x_keys {
        if let Some(v) = obj.remove(&k) {
            ext.insert(k, v);
        }
    }
    obj.insert("extensions".into(), Value::Object(ext));
}
