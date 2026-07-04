// SPDX-License-Identifier: AGPL-3.0-only
//! The deterministic `.tir` writer.
//!
//! Output is a pure function of the [`TirDoc`]: sections are emitted in [`SECTIONS`] order, keys in
//! their declared per-section order (unlisted keys appended sorted), and every number is rendered
//! by the single canonical [`format_number`] routine. There is no clock, no filesystem access, and
//! no alignment padding — so a `.tir` the writer produces is byte-stable and PR7's Python codec can
//! reproduce it byte-for-byte.
//!
//! Known limitation (shared with the Python codec, tracked for a follow-up): text values are
//! emitted single-quoted but NOT escaped — a value containing a quote, `$`/`!`, or a newline is
//! not representable and will not survive a write→parse round trip. Coefficient data is numeric,
//! and the regenerated `[MODEL]`/`[UNITS]` metadata is fixed, so canonical documents are safe.

use super::map::SECTIONS;
use super::{TirDoc, TirEntry, TirSection, TirValue};

/// Serialise a [`TirDoc`] to canonical `.tir` text (see the module docs for the exact format).
pub fn write_tir(doc: &TirDoc) -> String {
    let mut out = String::new();
    // Canonical sections first, in table order.
    for (name, keys) in SECTIONS {
        if let Some(section) = doc.section(name) {
            if !out.is_empty() {
                out.push('\n');
            }
            write_section(&mut out, section, keys);
        }
    }
    // Any section not in the table, appended in name order (deterministic), keys sorted.
    let mut extra: Vec<&TirSection> = doc
        .sections
        .iter()
        .filter(|s| !SECTIONS.iter().any(|(n, _)| *n == s.name))
        .collect();
    extra.sort_by(|a, b| a.name.cmp(&b.name));
    for section in extra {
        if !out.is_empty() {
            out.push('\n');
        }
        write_section(&mut out, section, &[]);
    }
    out
}

/// Write one `[SECTION]` and its entries: `ordered_keys` first in order, then any remaining entries
/// sorted by key.
fn write_section(out: &mut String, section: &TirSection, ordered_keys: &[&str]) {
    out.push('[');
    out.push_str(&section.name);
    out.push_str("]\n");
    for key in ordered_keys {
        if let Some(entry) = section.entries.iter().find(|e| e.key == *key) {
            write_entry(out, entry);
        }
    }
    let mut rest: Vec<&TirEntry> = section
        .entries
        .iter()
        .filter(|e| !ordered_keys.contains(&e.key.as_str()))
        .collect();
    rest.sort_by(|a, b| a.key.cmp(&b.key));
    for entry in rest {
        write_entry(out, entry);
    }
}

/// Write one `KEY = value` line (single space either side of `=`, no column alignment).
fn write_entry(out: &mut String, entry: &TirEntry) {
    out.push_str(&entry.key);
    out.push_str(" = ");
    match &entry.value {
        TirValue::Number(n) => out.push_str(&format_number(*n)),
        TirValue::Text(t) => {
            out.push('\'');
            out.push_str(t);
            out.push('\'');
        }
    }
    out.push('\n');
}

/// The **one** canonical `f64 → text` routine. Its rules (see [`super`] module docs for the full
/// specification and the cross-language contract) are, for a finite value `x`:
///
/// 1. `±0.0` → `"0"`.
/// 2. Otherwise take the shortest round-tripping decimal significand `d₁…dₙ` (`d₁ ≠ 0`, no trailing
///    zeros) with **round-half-to-even** tie-breaking, and the exponent `E` of the leading digit,
///    i.e. `|x| = d₁.d₂…dₙ × 10^E`. The digits come from [`ryu`] (IEEE round-half-to-even), so they
///    match Python `repr` exactly — including on the ~0.02% of values with a shortest-decimal tie,
///    where Rust's own `{:e}`/`Display` rounds away-from-even and would diverge.
/// 3. If `-4 ≤ E ≤ 15`, render **plain** decimal (no exponent, no forced trailing `.0`).
/// 4. Else render **scientific** as `mantissa e ± EE`, exponent sign always present and the
///    magnitude zero-padded to at least two digits (matching Python `repr`'s scientific form).
///
/// The plain/scientific switch at `E < -4` and `E > 15` is the documented exponent threshold.
pub fn format_number(x: f64) -> String {
    if x == 0.0 {
        return "0".to_owned();
    }
    if !x.is_finite() {
        // `.tir` has no NaN/Inf concept; render a deterministic token so writing never panics.
        return if x.is_nan() {
            "nan".to_owned()
        } else if x > 0.0 {
            "inf".to_owned()
        } else {
            "-inf".to_owned()
        };
    }

    let neg = x < 0.0;
    // Ryū gives the shortest round-tripping decimal with round-half-to-even tie-breaking (matching
    // Python `repr`); we re-derive `(digits, E)` from it and apply our own plain/scientific rules.
    let mut buf = ryu::Buffer::new();
    let (digits, exp) = normalize(buf.format(x.abs()));
    let len = i32::try_from(digits.len()).expect("digit count fits in i32");

    let mut out = String::new();
    if neg {
        out.push('-');
    }
    if (-4..=15).contains(&exp) {
        render_plain(&mut out, &digits, exp, len);
    } else {
        render_scientific(&mut out, &digits, exp, len);
    }
    out
}

/// Normalise a Ryū decimal string for a positive, finite value into the shortest significant digit
/// string `d₁…dₙ` (no leading/trailing zeros) and the exponent `E` of the leading digit, so the
/// value equals `d₁.d₂…dₙ × 10^E`. Ryū emits either fixed (`"4000.0"`, `"0.0001"`) or scientific
/// (`"1e16"`, `"2.2250738585072014e-308"`) forms; both are handled.
fn normalize(ryu: &str) -> (String, i32) {
    let (mantissa, exp10) = match ryu.split_once(['e', 'E']) {
        Some((m, e)) => (m, e.parse::<i32>().expect("Ryū exponent is an integer")),
        None => (ryu, 0),
    };
    let (int_part, frac_part) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    let combined: String = int_part
        .chars()
        .chain(frac_part.chars())
        .filter(char::is_ascii_digit)
        .collect();
    // value = intval(combined) × 10^(exp10 - frac_part.len())
    let point = exp10 - i32::try_from(frac_part.len()).expect("fraction length fits in i32");
    let total = i32::try_from(combined.len()).expect("digit count fits in i32");
    // Leading significant digit index and its place value give E.
    let first = combined
        .find(|c| c != '0')
        .expect("a non-zero value has a significant digit");
    let first_i32 = i32::try_from(first).expect("index fits in i32");
    let exp = (total - 1 - first_i32) + point;
    let sig = combined[first..].trim_end_matches('0');
    let digits = if sig.is_empty() { "0" } else { sig };
    (digits.to_owned(), exp)
}

/// Render `digits × 10^(exp-(len-1))` as a plain decimal into `out` (assumes `-4 ≤ exp ≤ 15`).
fn render_plain(out: &mut String, digits: &str, exp: i32, len: i32) {
    if exp >= len - 1 {
        // Integer value: all digits, then `exp-(len-1)` trailing zeros.
        out.push_str(digits);
        for _ in 0..(exp - (len - 1)) {
            out.push('0');
        }
    } else if exp >= 0 {
        // Decimal point sits `exp+1` digits in.
        let split = usize::try_from(exp + 1).expect("non-negative split point");
        out.push_str(&digits[..split]);
        out.push('.');
        out.push_str(&digits[split..]);
    } else {
        // `0.` then `-exp-1` leading zeros, then the digits.
        out.push_str("0.");
        for _ in 0..(-exp - 1) {
            out.push('0');
        }
        out.push_str(digits);
    }
}

/// Render `digits × 10^…` in scientific form matching Python `repr` (sign + ≥2-digit exponent).
fn render_scientific(out: &mut String, digits: &str, exp: i32, len: i32) {
    out.push_str(&digits[..1]);
    if len > 1 {
        out.push('.');
        out.push_str(&digits[1..]);
    }
    out.push('e');
    out.push(if exp < 0 { '-' } else { '+' });
    let mag = exp.unsigned_abs();
    if mag < 10 {
        out.push('0');
    }
    out.push_str(&mag.to_string());
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::unreadable_literal,
    clippy::excessive_precision
)]
mod tests {
    use super::format_number;

    #[test]
    fn plain_forms() {
        // Integers keep no trailing `.0`; fractions render exactly.
        assert_eq!(format_number(0.0), "0");
        assert_eq!(format_number(-0.0), "0");
        assert_eq!(format_number(4000.0), "4000");
        assert_eq!(format_number(200000.0), "200000");
        assert_eq!(format_number(22.0), "22");
        assert_eq!(format_number(1.65), "1.65");
        assert_eq!(format_number(0.33), "0.33");
        assert_eq!(format_number(-20.0), "-20");
        assert_eq!(format_number(0.0015), "0.0015");
        // E = -4 is the last plain magnitude.
        assert_eq!(format_number(0.0009), "0.0009");
        assert_eq!(format_number(0.0001), "0.0001");
        // E = 15 is the last plain magnitude.
        assert_eq!(format_number(1e15), "1000000000000000");
    }

    #[test]
    fn scientific_thresholds() {
        // E < -4 and E > 15 switch to scientific, sign always present, exponent ≥ 2 digits.
        assert_eq!(format_number(1e-5), "1e-05");
        assert_eq!(format_number(1.5e-5), "1.5e-05");
        assert_eq!(format_number(1e16), "1e+16");
        assert_eq!(format_number(1.25e16), "1.25e+16");
        assert_eq!(format_number(-3e-9), "-3e-09");
        assert_eq!(format_number(1e100), "1e+100");
        assert_eq!(format_number(1e-300), "1e-300");
    }

    #[test]
    fn shortest_decimal_ties_break_to_even_like_python_repr() {
        // These f64 values have a shortest-decimal tie; Rust's own `{:e}`/`Display` rounds
        // away-from-even (…696.3), Python `repr` and `ryu` round to even (…696.2). The canonical
        // format must match Python for the cross-language byte-for-byte contract to hold.
        assert_eq!(format_number(686995158985696.25), "686995158985696.2");
        assert_eq!(
            format_number(f64::from_bits(0x4303868c334f5f02)),
            "686995158985696.2"
        );
        assert_eq!(format_number(17493296845004.062), "17493296845004.062");
        assert_eq!(format_number(161221913628319.62), "161221913628319.62");
    }

    #[test]
    fn non_finite_is_deterministic() {
        assert_eq!(format_number(f64::NAN), "nan");
        assert_eq!(format_number(f64::INFINITY), "inf");
        assert_eq!(format_number(f64::NEG_INFINITY), "-inf");
    }

    #[test]
    fn every_finite_round_trips_exactly() {
        for x in [
            1.0 / 3.0,
            std::f64::consts::PI,
            1.234_567_890_123_456_7e-8,
            9.876e21,
            f64::MIN_POSITIVE,
            123_456_789_012_345.0,
        ] {
            let text = format_number(x);
            let back: f64 = text.parse().expect("canonical text parses");
            assert_eq!(back, x, "`{text}` did not round-trip {x}");
        }
    }
}
