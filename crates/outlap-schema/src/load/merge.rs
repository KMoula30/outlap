// SPDX-License-Identifier: AGPL-3.0-only
//! Stage 3 — resolve the `extends:` chain, deep-merge presets under the root, apply dotted-path
//! overrides, and record per-value provenance.
//!
//! Inheritance is a single-parent chain (multi-parent mixins are deferred). Mappings merge
//! key-by-key; sequences and scalars replace wholesale (child wins). Every resolved leaf gets an
//! [`Origin`] so the loaded-model report can explain where each value came from.

use serde_json::Value;

use crate::diagnostics::{SourceId, Sources, SrcSpan};
use crate::error::{Result, SchemaError};
use crate::io::SourceLoader;
use crate::load::provenance::{Origin, ProvenanceMap};
use crate::tree::{self, Entry, ParseError, Tree};

/// A set of programmatic dotted-path overrides (#35), e.g. `chassis.mass_kg = 812.0`.
#[derive(Clone, Debug, Default)]
pub struct Overrides {
    /// The dotted-path → JSON value entries.
    pub entries: Vec<(String, Value)>,
}

impl Overrides {
    /// An empty override set.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a dotted-path override.
    #[must_use]
    pub fn with(mut self, path: impl Into<String>, value: impl Into<Value>) -> Self {
        self.entries.push((path.into(), value.into()));
        self
    }
}

/// Resolve `root_tree` (already parsed, source `root_id`): walk its `extends` chain, deep-merge,
/// apply `overrides`, and produce the merged tree plus provenance.
pub fn resolve(
    root_tree: Tree,
    root_id: SourceId,
    overrides: &Overrides,
    loader: &dyn SourceLoader,
    sources: &mut Sources,
) -> Result<(Tree, ProvenanceMap)> {
    // Build the ancestor chain oldest → newest (root last), following `extends`.
    let mut chain: Vec<(Tree, Origin)> = Vec::new();
    let root_origin = Origin::Base {
        file: sources.name(root_id).to_owned(),
    };
    let mut current = root_tree;
    let mut current_origin = root_origin;
    let mut seen: Vec<String> = Vec::new();

    loop {
        let extends = extends_ref(&current);
        chain.push((current, current_origin));
        let Some((preset_ref, span)) = extends else {
            break;
        };
        if seen.contains(&preset_ref) {
            return Err(SchemaError::merge(
                sources,
                span,
                format!("`extends` cycle detected: `{preset_ref}` is already in the chain"),
            ));
        }
        seen.push(preset_ref.clone());
        let (preset_tree, preset_id) = load_and_parse(&preset_ref, span, loader, sources)?;
        current = preset_tree;
        current_origin = Origin::Inherited {
            preset: preset_ref,
            file: sources.name(preset_id).to_owned(),
        };
    }

    // Fold oldest → newest so the root wins.
    chain.reverse(); // now oldest first
    let mut prov = ProvenanceMap::default();
    let mut iter = chain.into_iter();
    let (mut merged, base_origin) = iter.next().expect("chain has at least the root");
    strip_extends(&mut merged);
    mark_all(&merged, "", &base_origin, &mut prov);
    for (mut over, over_origin) in iter {
        strip_extends(&mut over);
        merged = deep_merge(merged, over, "", &over_origin, &mut prov);
    }

    apply_overrides(&mut merged, overrides, root_id, sources, &mut prov)?;
    Ok((merged, prov))
}

/// Read the `extends:` scalar (a preset ref) and its span, if present.
fn extends_ref(tree: &Tree) -> Option<(String, SrcSpan)> {
    let entry = tree.as_map()?.iter().find(|e| e.key == "extends")?;
    match &entry.value {
        Tree::Scalar {
            value: Value::String(s),
            span,
        } => Some((s.clone(), *span)),
        _ => None,
    }
}

/// Remove the `extends` key from a mapping (resolved away in the loaded model).
fn strip_extends(tree: &mut Tree) {
    if let Some(entries) = tree.as_map_mut() {
        entries.retain(|e| e.key != "extends");
    }
}

/// Load a referenced preset by ref string, with a `.yaml` extension fallback, and parse it.
fn load_and_parse(
    reference: &str,
    span: SrcSpan,
    loader: &dyn SourceLoader,
    sources: &mut Sources,
) -> Result<(Tree, SourceId)> {
    let content = load_with_fallback(reference, loader).map_err(|_| {
        SchemaError::merge(
            sources,
            span,
            format!("cannot resolve `extends` preset `{reference}`"),
        )
    })?;
    let id = sources.add(reference, content);
    let tree = tree::parse(id, sources).map_err(|e| parse_to_error(e, sources))?;
    Ok((tree, id))
}

/// Try the ref verbatim, then with a `.yaml` extension if it has none.
pub fn load_with_fallback(reference: &str, loader: &dyn SourceLoader) -> Result<String> {
    match loader.load(reference) {
        Ok(c) => Ok(c),
        Err(first) => {
            if !reference.contains('.') {
                if let Ok(c) = loader.load(&format!("{reference}.yaml")) {
                    return Ok(c);
                }
            }
            Err(SchemaError::Io(first))
        }
    }
}

/// Convert a tree [`ParseError`] into a [`SchemaError`].
pub fn parse_to_error(e: ParseError, sources: &Sources) -> SchemaError {
    match e {
        ParseError::Yaml { message, span } => SchemaError::parse(sources, span, message),
    }
}

/// Deep-merge `over` onto `base`; child (`over`) wins. Records provenance for every touched leaf.
fn deep_merge(
    base: Tree,
    over: Tree,
    pointer: &str,
    over_origin: &Origin,
    prov: &mut ProvenanceMap,
) -> Tree {
    match (base, over) {
        (
            Tree::Map {
                entries: base_entries,
                ..
            },
            Tree::Map {
                entries: over_entries,
                span,
            },
        ) => {
            let mut result: Vec<Entry> = base_entries;
            for over_entry in over_entries {
                let child_ptr = format!("{pointer}/{}", tree::escape_pointer(&over_entry.key));
                if let Some(pos) = result.iter().position(|e| e.key == over_entry.key) {
                    let base_val = std::mem::replace(
                        &mut result[pos].value,
                        Tree::Scalar {
                            value: Value::Null,
                            span: over_entry.value.span(),
                        },
                    );
                    result[pos].value =
                        deep_merge(base_val, over_entry.value, &child_ptr, over_origin, prov);
                    result[pos].key_span = over_entry.key_span;
                } else {
                    mark_all(&over_entry.value, &child_ptr, over_origin, prov);
                    result.push(over_entry);
                }
            }
            Tree::Map {
                entries: result,
                span,
            }
        }
        // Scalars and sequences replace wholesale.
        (_, over) => {
            mark_all(&over, pointer, over_origin, prov);
            over
        }
    }
}

/// Record `origin` for every leaf pointer in a subtree.
fn mark_all(tree: &Tree, pointer: &str, origin: &Origin, prov: &mut ProvenanceMap) {
    match tree {
        Tree::Scalar { .. } => prov.set(pointer, origin.clone()),
        Tree::Seq { items, .. } => {
            // Record the sequence node itself and each element.
            prov.set(pointer, origin.clone());
            for (i, item) in items.iter().enumerate() {
                mark_all(item, &format!("{pointer}/{i}"), origin, prov);
            }
        }
        Tree::Map { entries, .. } => {
            for e in entries {
                let child = format!("{pointer}/{}", tree::escape_pointer(&e.key));
                mark_all(&e.value, &child, origin, prov);
            }
        }
    }
}

/// Apply dotted-path overrides to the merged tree, recording [`Origin::DottedOverride`].
fn apply_overrides(
    tree: &mut Tree,
    overrides: &Overrides,
    root_id: SourceId,
    sources: &Sources,
    prov: &mut ProvenanceMap,
) -> Result<()> {
    for (path, value) in &overrides.entries {
        let segments: Vec<&str> = path.split('.').collect();
        set_at_path(tree, &segments, value, root_id)
            .map_err(|msg| SchemaError::merge(sources, SrcSpan::blank(root_id), msg))?;
        let pointer = format!(
            "/{}",
            segments
                .iter()
                .map(|s| tree::escape_pointer(s))
                .collect::<Vec<_>>()
                .join("/")
        );
        prov.set(pointer, Origin::DottedOverride { path: path.clone() });
    }
    Ok(())
}

/// Set a value at a dotted path in the tree, creating scalars as needed. Errors if an intermediate
/// segment targets a non-mapping node (the override has no parent to attach to).
fn set_at_path(
    tree: &mut Tree,
    segments: &[&str],
    value: &Value,
    source: SourceId,
) -> std::result::Result<(), String> {
    let Some((head, rest)) = segments.split_first() else {
        return Ok(());
    };
    let entries = tree
        .as_map_mut()
        .ok_or_else(|| format!("override target `{head}` has no mapping parent"))?;
    let span = SrcSpan::blank(source);
    if rest.is_empty() {
        let new_value = value_to_tree(value, source);
        if let Some(e) = entries.iter_mut().find(|e| e.key == *head) {
            e.value = new_value;
        } else {
            entries.push(Entry {
                key: (*head).to_owned(),
                key_span: span,
                value: new_value,
            });
        }
        Ok(())
    } else {
        if !entries.iter().any(|e| e.key == *head) {
            entries.push(Entry {
                key: (*head).to_owned(),
                key_span: span,
                value: Tree::Map {
                    entries: Vec::new(),
                    span,
                },
            });
        }
        let child = &mut entries.iter_mut().find(|e| e.key == *head).unwrap().value;
        set_at_path(child, rest, value, source)
    }
}

/// Build a span-blank [`Tree`] from a JSON value (used by overrides and the in-memory path).
pub fn value_to_tree(value: &Value, source: SourceId) -> Tree {
    let span = SrcSpan::blank(source);
    match value {
        Value::Array(items) => Tree::Seq {
            items: items.iter().map(|v| value_to_tree(v, source)).collect(),
            span,
        },
        Value::Object(map) => Tree::Map {
            entries: map
                .iter()
                .map(|(k, v)| Entry {
                    key: k.clone(),
                    key_span: span,
                    value: value_to_tree(v, source),
                })
                .collect(),
            span,
        },
        scalar => Tree::Scalar {
            value: scalar.clone(),
            span,
        },
    }
}
