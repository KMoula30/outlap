// SPDX-License-Identifier: AGPL-3.0-only
//! The span-carrying document tree — the substrate for `extends` merge, the unknown-key walk, and
//! the single post-merge deserialize.
//!
//! `marked-yaml` gives us a parsed node tree with per-node source markers. We lower it into a
//! [`Tree`] whose every node carries a resolved byte [`SrcSpan`], preserving spans across the deep
//! merge (which produces *new* nodes) so miette labels survive. Scalars are coerced to JSON values
//! at lowering time, honouring YAML quoting (`may_coerce`).

use std::collections::BTreeMap;

use marked_yaml::types::Node;
use marked_yaml::{parse_yaml_with_options, LoaderOptions};
use serde_json::Value;

use crate::diagnostics::{SourceId, Sources, SrcSpan};

/// A source document lowered to a span-carrying tree.
#[derive(Clone, Debug)]
pub enum Tree {
    /// A scalar, already coerced to a JSON value (null/bool/number/string).
    Scalar {
        /// The coerced value.
        value: Value,
        /// Span of the scalar.
        span: SrcSpan,
    },
    /// A sequence.
    Seq {
        /// The items.
        items: Vec<Tree>,
        /// Span of the whole sequence.
        span: SrcSpan,
    },
    /// A mapping (insertion-ordered).
    Map {
        /// The key/value entries.
        entries: Vec<Entry>,
        /// Span of the whole mapping.
        span: SrcSpan,
    },
}

/// A mapping entry: a key (with its own span) and a value subtree.
#[derive(Clone, Debug)]
pub struct Entry {
    /// The key text.
    pub key: String,
    /// Span of the key token.
    pub key_span: SrcSpan,
    /// The value subtree.
    pub value: Tree,
}

impl Tree {
    /// The span of this node.
    pub fn span(&self) -> SrcSpan {
        match self {
            Tree::Scalar { span, .. } | Tree::Seq { span, .. } | Tree::Map { span, .. } => *span,
        }
    }

    /// The entries if this is a mapping.
    pub fn as_map(&self) -> Option<&[Entry]> {
        match self {
            Tree::Map { entries, .. } => Some(entries),
            _ => None,
        }
    }

    /// Mutable entries if this is a mapping.
    pub fn as_map_mut(&mut self) -> Option<&mut Vec<Entry>> {
        match self {
            Tree::Map { entries, .. } => Some(entries),
            _ => None,
        }
    }

    /// The value for `key` if this is a mapping containing it.
    pub fn get(&self, key: &str) -> Option<&Tree> {
        self.as_map()?
            .iter()
            .find(|e| e.key == key)
            .map(|e| &e.value)
    }

    /// A short human label for the node kind (for error messages).
    pub fn kind(&self) -> &'static str {
        match self {
            Tree::Scalar { .. } => "scalar",
            Tree::Seq { .. } => "sequence",
            Tree::Map { .. } => "mapping",
        }
    }
}

/// Errors that can arise lowering YAML into a [`Tree`].
#[derive(Debug, Clone)]
pub enum ParseError {
    /// A YAML syntax / structural error, with the offending span.
    Yaml {
        /// Human-readable message.
        message: String,
        /// Location of the problem.
        span: SrcSpan,
    },
}

/// Parse `content` (already registered in `sources` as `id`) into a span-carrying [`Tree`].
///
/// Anchors/aliases and `<<` merge keys are rejected by `marked-yaml` itself (outlap does not
/// support them; inheritance is via `extends:` only). Duplicate keys are errors.
pub fn parse(id: SourceId, sources: &Sources) -> Result<Tree, ParseError> {
    let content = sources.content(id).to_owned();
    let opts = LoaderOptions::default().error_on_duplicate_keys(true);
    let node = parse_yaml_with_options(id, &content, opts).map_err(|e| {
        // marked-yaml's Display carries "line:col: message"; extract a span from its marker.
        let span = marker_span(&e, id, sources);
        ParseError::Yaml {
            message: e.to_string(),
            span,
        }
    })?;
    Ok(lower(&node, id, sources))
}

/// Best-effort span for a `marked-yaml` load error (its markers are private, so parse the Display).
fn marker_span(err: &marked_yaml::LoadError, id: SourceId, sources: &Sources) -> SrcSpan {
    let text = err.to_string();
    // Format is "L:C: message".
    let mut parts = text.splitn(3, ':');
    if let (Some(l), Some(c)) = (parts.next(), parts.next()) {
        if let (Ok(line), Ok(col)) = (l.trim().parse::<usize>(), c.trim().parse::<usize>()) {
            let off = crate::diagnostics::byte_offset(sources.content(id), line, col);
            return SrcSpan {
                source: id,
                offset: off,
                len: 1,
            };
        }
    }
    SrcSpan::blank(id)
}

/// Lower a `marked-yaml` node into a [`Tree`], resolving markers to byte spans.
fn lower(node: &Node, id: SourceId, sources: &Sources) -> Tree {
    let span = sources.resolve(id, node.span());
    match node {
        Node::Scalar(s) => Tree::Scalar {
            value: coerce_scalar(s),
            span,
        },
        Node::Sequence(seq) => {
            let items = seq.iter().map(|n| lower(n, id, sources)).collect();
            Tree::Seq { items, span }
        }
        Node::Mapping(map) => {
            let entries = map
                .iter()
                .map(|(k, v)| Entry {
                    key: k.as_str().to_owned(),
                    key_span: sources.resolve(id, k.span()),
                    value: lower(v, id, sources),
                })
                .collect();
            Tree::Map { entries, span }
        }
    }
}

/// Coerce a YAML scalar to a JSON value, honouring quoting (`may_coerce`) and YAML null spellings.
fn coerce_scalar(s: &marked_yaml::types::MarkedScalarNode) -> Value {
    let raw = s.as_str();
    if !s.may_coerce() {
        return Value::String(raw.to_owned());
    }
    if raw.is_empty() || matches!(raw, "null" | "Null" | "NULL" | "~") {
        return Value::Null;
    }
    if let Some(b) = s.as_bool() {
        return Value::Bool(b);
    }
    if let Some(i) = s.as_i64() {
        return Value::Number(i.into());
    }
    if let Some(f) = s.as_f64() {
        if f.is_finite() {
            if let Some(n) = serde_json::Number::from_f64(f) {
                return Value::Number(n);
            }
        }
    }
    Value::String(raw.to_owned())
}

/// A JSON-pointer → span index for the converted value, so a deserialize error path or an
/// unknown-key location can be resolved back to a source span.
#[derive(Clone, Debug, Default)]
pub struct SpanIndex {
    /// Value span for each JSON pointer (`""` for root, `/a/b/0` for nested).
    pub value: BTreeMap<String, SrcSpan>,
    /// Key-token span for each mapping-child JSON pointer (absent for the root and seq elements).
    pub key: BTreeMap<String, SrcSpan>,
}

impl SpanIndex {
    /// The best span for a pointer: the key token if known, else the value span, else `None`.
    pub fn span_for(&self, pointer: &str) -> Option<SrcSpan> {
        self.key
            .get(pointer)
            .or_else(|| self.value.get(pointer))
            .copied()
    }
}

/// Convert a [`Tree`] to a `serde_json::Value` and a parallel [`SpanIndex`].
pub fn to_value(tree: &Tree) -> (Value, SpanIndex) {
    let mut index = SpanIndex::default();
    let value = build(tree, "", &mut index);
    (value, index)
}

fn build(tree: &Tree, pointer: &str, index: &mut SpanIndex) -> Value {
    index.value.insert(pointer.to_owned(), tree.span());
    match tree {
        Tree::Scalar { value, .. } => value.clone(),
        Tree::Seq { items, .. } => {
            let mut out = Vec::with_capacity(items.len());
            for (i, item) in items.iter().enumerate() {
                let child = format!("{pointer}/{i}");
                out.push(build(item, &child, index));
            }
            Value::Array(out)
        }
        Tree::Map { entries, .. } => {
            let mut out = serde_json::Map::new();
            for e in entries {
                let child = format!("{pointer}/{}", escape_pointer(&e.key));
                index.key.insert(child.clone(), e.key_span);
                out.insert(e.key.clone(), build(&e.value, &child, index));
            }
            Value::Object(out)
        }
    }
}

/// Escape a key for use in a JSON pointer segment (RFC 6901: `~`→`~0`, `/`→`~1`).
pub fn escape_pointer(key: &str) -> String {
    key.replace('~', "~0").replace('/', "~1")
}
