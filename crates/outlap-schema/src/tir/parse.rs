// SPDX-License-Identifier: AGPL-3.0-only
//! The `.tir` text parser: `[SECTION]` headers and `KEY = value` lines into a [`TirDoc`].
//!
//! Grammar (TNO MF-Tyre `.tir` layout, implemented clean-room from the public format):
//! * Lines are `[SECTION]` headers, `KEY = value` entries, comments, or blank.
//! * Comments start at an unquoted `$` or `!` and run to end of line (inline comments allowed).
//! * Values are single/double-quoted strings or bare numbers (`0.33`, `-20`, `1.5e-3`, `2.0E5`).
//! * Leading BOM and CRLF line endings are tolerated. Keys are normalised to uppercase.
//! * A duplicate key inside a section is **last-wins** and raises a warning.
//!
//! Hard errors (typed [`SchemaError`], with a source span) are raised for a malformed line, an
//! entry before any section, an unterminated `[SECTION]` header, and a non-SI `[UNITS]` value.
//! Unknown section names are carried through with a did-you-mean warning.

use crate::diagnostics::{suggest, Sources, SrcSpan};
use crate::error::{Result, SchemaError};
use crate::load::report::ReportEntry;

use super::map::{OVERFLOW_SECTION, SECTIONS, SI_UNITS, UNITS_SECTION};
use super::{TirDoc, TirEntry, TirSection, TirValue};

/// Parse `content` (the text of a `.tir` file identified by `label` for diagnostics) into a
/// [`TirDoc`], returning any non-fatal warnings (duplicate keys, unknown sections).
pub fn parse_tir(label: &str, content: &str) -> Result<(TirDoc, Vec<ReportEntry>)> {
    let mut sources = Sources::new();
    let src = sources.add(label, content.to_owned());
    let mut warnings = Vec::new();

    let mut sections: Vec<TirSection> = Vec::new();
    let mut pos = 0usize; // byte offset of the current line's start in `content`.

    for raw_with_nl in content.split_inclusive('\n') {
        let line_start = pos;
        pos += raw_with_nl.len();
        // Drop the line terminator (\n, and a preceding \r for CRLF files).
        let raw = raw_with_nl.strip_suffix('\n').unwrap_or(raw_with_nl);
        let raw = raw.strip_suffix('\r').unwrap_or(raw);
        // Strip a UTF-8 BOM from the very first line.
        let (raw, base) = if line_start == 0 {
            match raw.strip_prefix('\u{FEFF}') {
                Some(rest) => (rest, line_start + '\u{FEFF}'.len_utf8()),
                None => (raw, line_start),
            }
        } else {
            (raw, line_start)
        };

        let code = strip_comment(raw);
        let trimmed = code.trim();
        if trimmed.is_empty() {
            continue;
        }

        if trimmed.starts_with('[') {
            let name = parse_section_header(trimmed, base, code, &sources, src)?;
            if !known_section(&name) {
                let hint = suggest(&name, SECTIONS.iter().map(|(n, _)| *n))
                    .map(|s| format!(" (did you mean `[{s}]`?)"))
                    .unwrap_or_default();
                warnings.push(ReportEntry::new(
                    format!("[{name}]"),
                    format!("unknown `.tir` section `[{name}]`{hint} — carried through"),
                ));
            }
            sections.push(TirSection {
                name,
                entries: Vec::new(),
            });
            continue;
        }

        // A `KEY = value` entry.
        let Some(eq) = code.find('=') else {
            return Err(SchemaError::parse(
                &sources,
                span_of(src, base, code, trimmed),
                "malformed `.tir` line: expected `[SECTION]`, `KEY = value`, or a comment",
            ));
        };
        let key_raw = code[..eq].trim();
        if key_raw.is_empty() {
            return Err(SchemaError::parse(
                &sources,
                span_of(src, base, code, trimmed),
                "malformed `.tir` line: empty key before `=`",
            ));
        }
        let key = key_raw.to_ascii_uppercase();

        let value_part = &code[eq + 1..];
        let value_offset = base + eq + 1 + leading_ws(value_part);
        let value_str = value_part.trim();
        if value_str.is_empty() {
            return Err(SchemaError::parse(
                &sources,
                SrcSpan {
                    source: src,
                    offset: base + eq,
                    len: 1,
                },
                format!("`{key}` has no value after `=`"),
            ));
        }
        let value_span = SrcSpan {
            source: src,
            offset: value_offset,
            len: value_str.len(),
        };
        let value = parse_value(value_str);

        let Some(section) = sections.last_mut() else {
            return Err(SchemaError::parse(
                &sources,
                span_of(src, base, code, trimmed),
                format!("`{key}` appears before any `[SECTION]` header"),
            ));
        };

        // `[UNITS]` values must be SI (hard error otherwise).
        if section.name == UNITS_SECTION {
            check_si_unit(&key, &value, value_span, &sources)?;
        }

        // Duplicate key within the section: last-wins with a warning.
        if let Some(existing) = section.entries.iter_mut().find(|e| e.key == key) {
            warnings.push(ReportEntry::new(
                format!("[{}]/{key}", section.name),
                format!(
                    "duplicate key `{key}` in `[{}]` — keeping the last value",
                    section.name
                ),
            ));
            existing.value = value;
        } else {
            section.entries.push(TirEntry { key, value });
        }
    }

    Ok((TirDoc::new(label, sections), warnings))
}

/// Parse a `[SECTION]` header line, returning the (uppercased) section name.
fn parse_section_header(
    trimmed: &str,
    base: usize,
    code: &str,
    sources: &Sources,
    src: usize,
) -> Result<String> {
    let inner = trimmed.strip_prefix('[').and_then(|s| s.strip_suffix(']'));
    match inner {
        Some(name) if !name.trim().is_empty() => Ok(name.trim().to_ascii_uppercase()),
        _ => Err(SchemaError::parse(
            sources,
            span_of(src, base, code, trimmed),
            "malformed `.tir` section header: expected `[SECTION_NAME]`",
        )),
    }
}

/// Whether `name` is a section the mapping table models (others are carried through with a warning).
/// The writer's own overflow section is recognised so round-tripped output re-parses without a warning.
fn known_section(name: &str) -> bool {
    name == OVERFLOW_SECTION || SECTIONS.iter().any(|(n, _)| *n == name)
}

/// Classify a bare value token: a quoted string, or a number, or an unquoted identifier (string).
fn parse_value(s: &str) -> TirValue {
    if let Some(inner) = unquote(s) {
        return TirValue::Text(inner);
    }
    match s.parse::<f64>() {
        Ok(n) if n.is_finite() => TirValue::Number(n),
        _ => TirValue::Text(s.to_owned()),
    }
}

/// If `s` is wrapped in matching single or double quotes, return its unquoted content.
fn unquote(s: &str) -> Option<String> {
    for q in ['\'', '"'] {
        if s.len() >= 2 && s.starts_with(q) && s.ends_with(q) {
            return Some(s[1..s.len() - 1].to_owned());
        }
    }
    None
}

/// Enforce that a `[UNITS]` entry names an SI unit; a listed dimension with any other value is a
/// hard error. Dimensions not in [`SI_UNITS`] are left to the unknown-key path (carried through).
fn check_si_unit(key: &str, value: &TirValue, span: SrcSpan, sources: &Sources) -> Result<()> {
    let Some((_, si)) = SI_UNITS.iter().find(|(dim, _)| *dim == key) else {
        return Ok(());
    };
    let ok = match value {
        TirValue::Text(t) => t.eq_ignore_ascii_case(si),
        TirValue::Number(_) => false,
    };
    if ok {
        Ok(())
    } else {
        Err(SchemaError::semantic(
            sources,
            span,
            format!("non-SI `[UNITS]` declaration: `{key}` must be `{si}`"),
            Some("outlap works in SI internally; re-express the `.tir` in SI units".into()),
        ))
    }
}

/// The number of leading ASCII/Unicode-whitespace bytes in `s`.
fn leading_ws(s: &str) -> usize {
    s.len() - s.trim_start().len()
}

/// A span covering `trimmed` (the trimmed line content) within source `src`.
fn span_of(src: usize, base: usize, code: &str, trimmed: &str) -> SrcSpan {
    let offset = base + (code.len() - code.trim_start().len());
    SrcSpan {
        source: src,
        offset,
        len: trimmed.len(),
    }
}

/// Return the code portion of a line, dropping an inline comment that starts at the first unquoted
/// `$` or `!`.
fn strip_comment(line: &str) -> &str {
    let bytes = line.as_bytes();
    let mut quote: Option<u8> = None;
    for (i, &b) in bytes.iter().enumerate() {
        match quote {
            Some(q) => {
                if b == q {
                    quote = None;
                }
            }
            None => {
                if b == b'\'' || b == b'"' {
                    quote = Some(b);
                } else if b == b'$' || b == b'!' {
                    return &line[..i];
                }
            }
        }
    }
    line
}
