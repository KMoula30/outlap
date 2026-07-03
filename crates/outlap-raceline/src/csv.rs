// SPDX-License-Identifier: AGPL-3.0-only
//! Reader for a user-supplied `raceline.csv` (§6.3): `s_m,n_m` — arc-length station on the parent
//! centerline and signed lateral offset (`+` road-left, ISO 8855). Header required; `#` comments
//! and blank lines skipped. Consumed via [`offset_track`](outlap_track::offset_track), so a
//! user line is a first-class [`Track`] like a generated one.

use outlap_track::{offset_track, Track, TrackError};

use crate::Raceline;

/// An error reading a `raceline.csv`.
#[derive(Debug, thiserror::Error)]
pub enum RacelineCsvError {
    /// The file had no header or no data rows.
    #[error("raceline is empty (need a header and at least {min} rows)")]
    Empty {
        /// Minimum rows required.
        min: usize,
    },
    /// A required column was missing.
    #[error("raceline header is missing column `{0}` (expected `s_m,n_m`)")]
    MissingColumn(String),
    /// A field could not be parsed as a number.
    #[error("raceline line {line}: column `{column}` is not a number (`{value}`)")]
    NotANumber {
        /// 1-based source line.
        line: usize,
        /// Offending column.
        column: String,
        /// Raw text.
        value: String,
    },
    /// The offset line could not be built as a track.
    #[error(transparent)]
    Track(#[from] TrackError),
}

/// Parse a `raceline.csv` and build the line as a [`Raceline`] against `track`.
pub fn read_raceline_csv(text: &str, track: &Track) -> Result<Raceline, RacelineCsvError> {
    const MIN_ROWS: usize = 4;
    let mut lines = text
        .lines()
        .enumerate()
        .map(|(i, l)| (i + 1, l.trim()))
        .filter(|(_, l)| !l.is_empty() && !l.starts_with('#'));

    let (_, header) = lines
        .next()
        .ok_or(RacelineCsvError::Empty { min: MIN_ROWS })?;
    let cols: Vec<&str> = header.split(',').map(str::trim).collect();
    let s_idx = cols
        .iter()
        .position(|c| *c == "s_m")
        .ok_or_else(|| RacelineCsvError::MissingColumn("s_m".to_owned()))?;
    let n_idx = cols
        .iter()
        .position(|c| *c == "n_m")
        .ok_or_else(|| RacelineCsvError::MissingColumn("n_m".to_owned()))?;

    let mut s = Vec::new();
    let mut n = Vec::new();
    for (line, content) in lines {
        let fields: Vec<&str> = content.split(',').map(str::trim).collect();
        let parse = |idx: usize, name: &str| -> Result<f64, RacelineCsvError> {
            let raw = fields.get(idx).copied().unwrap_or("");
            raw.parse::<f64>()
                .map_err(|_| RacelineCsvError::NotANumber {
                    line,
                    column: name.to_owned(),
                    value: raw.to_owned(),
                })
        };
        s.push(parse(s_idx, "s_m")?);
        n.push(parse(n_idx, "n_m")?);
    }
    if s.len() < MIN_ROWS {
        return Err(RacelineCsvError::Empty { min: MIN_ROWS });
    }

    let line = offset_track(track, &s, &n, "user raceline")?;
    Ok(Raceline { s, n, line })
}

/// Serialise offsets to `raceline.csv` text (`s_m,n_m`).
pub fn write_raceline_csv(s: &[f64], n: &[f64]) -> String {
    use std::fmt::Write as _;
    let mut out = String::from("# outlap raceline (offsets on the parent centerline)\ns_m,n_m\n");
    for (si, ni) in s.iter().zip(n) {
        let _ = writeln!(out, "{si:.4},{ni:.4}");
    }
    out
}
