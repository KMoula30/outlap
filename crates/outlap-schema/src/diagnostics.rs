// SPDX-License-Identifier: AGPL-3.0-only
//! Diagnostics plumbing: byte-offset source spans, a source registry, the `marked-yaml` →
//! `miette` span bridge, and did-you-mean suggestions.
//!
//! Config errors are a product surface (CLAUDE.md): every diagnostic points at a file + byte range
//! so miette can render an underlined label, and unknown identifiers get a Levenshtein suggestion.

use marked_yaml::Span as MarkedSpan;
use miette::{NamedSource, SourceSpan};

/// Index of a loaded source file in a [`Sources`] registry. Also the `source` id passed to
/// `marked-yaml`'s parser, so `Marker::source()` round-trips back to this index.
pub type SourceId = usize;

/// A byte-offset span into a specific loaded source file.
///
/// `marked-yaml` records *character* line/column markers; we resolve those to UTF-8 byte offsets
/// here (miette wants bytes). A blank/absent marker collapses to a zero-length span at offset 0.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SrcSpan {
    /// Which source file this span is in.
    pub source: SourceId,
    /// Byte offset of the span start.
    pub offset: usize,
    /// Byte length of the span.
    pub len: usize,
}

impl SrcSpan {
    /// A zero-length span at the start of a source (used when no marker is available).
    pub fn blank(source: SourceId) -> Self {
        Self {
            source,
            offset: 0,
            len: 0,
        }
    }

    /// Convert to a miette [`SourceSpan`] (offset + length in bytes).
    pub fn to_miette(self) -> SourceSpan {
        SourceSpan::new(self.offset.into(), self.len)
    }
}

/// A registry of loaded source files, keyed by [`SourceId`], holding each file's display name and
/// full content so diagnostics can embed the right `NamedSource`.
#[derive(Clone, Debug, Default)]
pub struct Sources {
    files: Vec<SourceFile>,
}

#[derive(Clone, Debug)]
struct SourceFile {
    name: String,
    content: String,
}

impl Sources {
    /// Create an empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a source file and return its id.
    pub fn add(&mut self, name: impl Into<String>, content: impl Into<String>) -> SourceId {
        let id = self.files.len();
        self.files.push(SourceFile {
            name: name.into(),
            content: content.into(),
        });
        id
    }

    /// The display name of a source.
    pub fn name(&self, id: SourceId) -> &str {
        self.files.get(id).map_or("<unknown>", |f| f.name.as_str())
    }

    /// The full content of a source.
    pub fn content(&self, id: SourceId) -> &str {
        self.files.get(id).map_or("", |f| f.content.as_str())
    }

    /// Build a miette [`NamedSource`] for a source id (name + owned content).
    pub fn named(&self, id: SourceId) -> NamedSource<String> {
        NamedSource::new(self.name(id), self.content(id).to_owned())
    }

    /// Resolve a `marked-yaml` span (for a node parsed from source `id`) to a byte [`SrcSpan`].
    pub fn resolve(&self, id: SourceId, span: &MarkedSpan) -> SrcSpan {
        let content = self.content(id);
        let start = span
            .start()
            .map_or(0, |m| byte_offset(content, m.line(), m.column()));
        let end = span
            .end()
            .map_or(start, |m| byte_offset(content, m.line(), m.column()));
        SrcSpan {
            source: id,
            offset: start,
            len: end.saturating_sub(start),
        }
    }
}

/// Resolve a 1-indexed (line, column) marker to a UTF-8 byte offset in `content`.
///
/// `marked-yaml` columns are 1-indexed *character* positions; we walk `column - 1` characters into
/// the target line to get a correct byte offset even with multi-byte UTF-8. Out-of-range markers
/// clamp to the end of the available text.
pub fn byte_offset(content: &str, line: usize, column: usize) -> usize {
    let mut line_start = 0usize;
    let mut current_line = 1usize;
    // Find the byte offset of the start of `line`.
    if line > 1 {
        for (idx, ch) in content.char_indices() {
            if ch == '\n' {
                current_line += 1;
                if current_line == line {
                    line_start = idx + 1;
                    break;
                }
            }
        }
        if current_line < line {
            return content.len();
        }
    }
    // Advance `column - 1` characters into the line.
    let target_chars = column.saturating_sub(1);
    for (chars_seen, (idx, ch)) in content[line_start..].char_indices().enumerate() {
        if chars_seen == target_chars || ch == '\n' {
            return line_start + idx;
        }
    }
    content.len()
}

/// Return the closest candidate to `input` within a small edit distance, for did-you-mean hints.
///
/// Uses normalized Levenshtein distance and only suggests when the match is reasonably close
/// (distance ≤ ~40% of the input length, and strictly closer than any runner-up tie is ignored).
pub fn suggest<'a>(input: &str, candidates: impl IntoIterator<Item = &'a str>) -> Option<String> {
    let max_dist = (input.chars().count() / 2).max(2);
    let mut best: Option<(usize, &str)> = None;
    for cand in candidates {
        let dist = strsim::levenshtein(input, cand);
        if dist <= max_dist && best.is_none_or(|(d, _)| dist < d) {
            best = Some((dist, cand));
        }
    }
    best.map(|(_, c)| c.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn byte_offset_handles_lines_and_utf8() {
        let content = "a: 1\nbé: 2\n";
        // line 1, col 1 -> offset 0
        assert_eq!(byte_offset(content, 1, 1), 0);
        // line 2, col 1 -> just after "a: 1\n" = 5
        assert_eq!(byte_offset(content, 2, 1), 5);
        // line 2, col 3 -> 'b'(1 byte) 'é'(2 bytes) => offset 5 + 1 + 2 = 8 (the ':')
        assert_eq!(byte_offset(content, 2, 3), 8);
    }

    #[test]
    fn suggest_finds_close_typo() {
        assert_eq!(
            suggest("chasis", ["chassis", "aero", "brakes"]),
            Some("chassis".into())
        );
        assert_eq!(suggest("zzzzzz", ["chassis", "aero"]), None);
    }
}
