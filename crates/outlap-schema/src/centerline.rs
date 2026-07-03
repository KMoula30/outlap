// SPDX-License-Identifier: AGPL-3.0-only
//! The `centerline.csv` sidecar (§9.3) — the numeric substrate of the 3D track format.
//!
//! Columns (header-named, order-independent): `s_m, x_m, y_m, z_m, banking_deg, width_left_m,
//! width_right_m, grip_scale`. This is a tabular sidecar, not JSON, so it has no JSON Schema; it is
//! parsed and structurally validated here with line/column diagnostics (Decision #43). Geometry
//! (spline fit, closure) is the `outlap-track` crate's job; this layer guarantees a clean, ordered,
//! physically-sane table.

/// The eight required centerline columns, in canonical order.
pub const COLUMNS: [&str; 8] = [
    "s_m",
    "x_m",
    "y_m",
    "z_m",
    "banking_deg",
    "width_left_m",
    "width_right_m",
    "grip_scale",
];

/// One centerline sample (SI units; ISO 8855 world frame: x forward, y left, z up).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CenterlineRow {
    /// Arc-length station, metres (strictly increasing).
    pub s_m: f64,
    /// World x, metres.
    pub x_m: f64,
    /// World y, metres.
    pub y_m: f64,
    /// World z (elevation), metres.
    pub z_m: f64,
    /// Banking angle, degrees (may be overridden by `track.yaml` keypoints).
    pub banking_deg: f64,
    /// Left track half-width, metres (> 0).
    pub width_left_m: f64,
    /// Right track half-width, metres (> 0).
    pub width_right_m: f64,
    /// Grip scale multiplier (> 0; 1.0 = nominal).
    pub grip_scale: f64,
}

/// A parsed, structurally-validated centerline table.
#[derive(Clone, Debug, PartialEq)]
pub struct Centerline {
    /// The sample rows, in file order (arc-length ascending).
    pub rows: Vec<CenterlineRow>,
}

/// A `centerline.csv` parse/validation error, with a 1-based source line where applicable.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CenterlineError {
    /// The file had no header or no data rows.
    #[error("centerline is empty (need a header row and at least {min} data rows)")]
    Empty {
        /// Minimum data rows required.
        min: usize,
    },
    /// A required column was missing from the header.
    #[error("centerline header is missing column `{column}`{}", did_you_mean(.hint.as_ref()))]
    MissingColumn {
        /// The missing canonical column name.
        column: String,
        /// A did-you-mean suggestion from the actual header, if any.
        hint: Option<String>,
    },
    /// A data row had the wrong number of fields.
    #[error("centerline line {line}: expected {expected} columns, found {found}")]
    WrongFieldCount {
        /// 1-based source line.
        line: usize,
        /// Expected field count.
        expected: usize,
        /// Actual field count.
        found: usize,
    },
    /// A field could not be parsed as a number.
    #[error("centerline line {line}: column `{column}` is not a number (`{value}`)")]
    NotANumber {
        /// 1-based source line.
        line: usize,
        /// The offending column name.
        column: String,
        /// The raw text.
        value: String,
    },
    /// Arc length was not strictly increasing.
    #[error("centerline line {line}: `s_m` must be strictly increasing (got {value}, previous {previous})")]
    NonMonotoneS {
        /// 1-based source line.
        line: usize,
        /// The offending `s_m`.
        value: f64,
        /// The previous row's `s_m`.
        previous: f64,
    },
    /// A physically-invalid value (non-positive width or grip, non-finite coordinate).
    #[error("centerline line {line}: `{column}` must be {constraint} (got {value})")]
    BadValue {
        /// 1-based source line.
        line: usize,
        /// The offending column.
        column: String,
        /// The constraint description, e.g. `> 0`.
        constraint: String,
        /// The offending value.
        value: f64,
    },
}

fn did_you_mean(hint: Option<&String>) -> String {
    match hint {
        Some(h) => format!(" (did you mean `{h}`?)"),
        None => String::new(),
    }
}

/// Parse and structurally validate a `centerline.csv`. Requires at least `min_rows` data rows.
///
/// Comment lines (`#`) and blank lines are skipped. The header may list the columns in any order;
/// every canonical column of [`COLUMNS`] must be present.
///
/// # Errors
/// [`CenterlineError`] on a missing/garbled header, a bad row, non-monotone `s_m`, or a
/// physically-invalid value.
pub fn parse_centerline(text: &str, min_rows: usize) -> Result<Centerline, CenterlineError> {
    // (1-based line number, trimmed content) for every non-blank, non-comment line.
    let mut lines = text
        .lines()
        .enumerate()
        .map(|(i, l)| (i + 1, l.trim()))
        .filter(|(_, l)| !l.is_empty() && !l.starts_with('#'));

    let (_, header) = lines
        .next()
        .ok_or(CenterlineError::Empty { min: min_rows })?;
    let header_cols: Vec<String> = header.split(',').map(|c| c.trim().to_owned()).collect();

    // Resolve each canonical column to its index in the header.
    let mut col_index = [0usize; 8];
    for (slot, &name) in COLUMNS.iter().enumerate() {
        let idx = header_cols.iter().position(|c| c == name).ok_or_else(|| {
            let hint = crate::diagnostics::suggest(name, header_cols.iter().map(String::as_str));
            CenterlineError::MissingColumn {
                column: name.to_owned(),
                hint,
            }
        })?;
        col_index[slot] = idx;
    }
    let ncols = header_cols.len();

    let mut rows: Vec<CenterlineRow> = Vec::new();
    let mut prev_s: Option<f64> = None;
    for (line, content) in lines {
        let fields: Vec<&str> = content.split(',').map(str::trim).collect();
        if fields.len() != ncols {
            return Err(CenterlineError::WrongFieldCount {
                line,
                expected: ncols,
                found: fields.len(),
            });
        }
        let get = |slot: usize| -> Result<f64, CenterlineError> {
            let raw = fields[col_index[slot]];
            raw.parse::<f64>().map_err(|_| CenterlineError::NotANumber {
                line,
                column: COLUMNS[slot].to_owned(),
                value: raw.to_owned(),
            })
        };
        let row = CenterlineRow {
            s_m: get(0)?,
            x_m: get(1)?,
            y_m: get(2)?,
            z_m: get(3)?,
            banking_deg: get(4)?,
            width_left_m: get(5)?,
            width_right_m: get(6)?,
            grip_scale: get(7)?,
        };

        // Finite coordinates.
        for (slot, v) in [(1, row.x_m), (2, row.y_m), (3, row.z_m)] {
            if !v.is_finite() {
                return Err(CenterlineError::BadValue {
                    line,
                    column: COLUMNS[slot].to_owned(),
                    constraint: "finite".to_owned(),
                    value: v,
                });
            }
        }
        // Positive widths and grip.
        for (slot, v) in [
            (5, row.width_left_m),
            (6, row.width_right_m),
            (7, row.grip_scale),
        ] {
            if v <= 0.0 || !v.is_finite() {
                return Err(CenterlineError::BadValue {
                    line,
                    column: COLUMNS[slot].to_owned(),
                    constraint: "> 0".to_owned(),
                    value: v,
                });
            }
        }
        // Strictly increasing arc length. Testing `> p` positively (rather than `<= p`) also
        // rejects a NaN `s_m`, which would silently pass a `<=` check.
        if let Some(p) = prev_s {
            let strictly_increasing = row.s_m > p;
            if !strictly_increasing {
                return Err(CenterlineError::NonMonotoneS {
                    line,
                    value: row.s_m,
                    previous: p,
                });
            }
        }
        prev_s = Some(row.s_m);
        rows.push(row);
    }

    if rows.len() < min_rows {
        return Err(CenterlineError::Empty { min: min_rows });
    }
    Ok(Centerline { rows })
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOOD: &str = "\
s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale
0.0,0.0,0.0,0.0,0.0,5.0,5.0,1.0
10.0,10.0,0.0,0.5,1.0,5.0,5.0,1.0
20.0,20.0,0.0,1.0,2.0,5.0,5.0,0.98
";

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-12
    }

    #[test]
    fn parses_good_file() {
        let c = parse_centerline(GOOD, 3).unwrap();
        assert_eq!(c.rows.len(), 3);
        assert!(close(c.rows[1].x_m, 10.0));
        assert!(close(c.rows[2].grip_scale, 0.98));
    }

    #[test]
    fn column_order_independent() {
        let reordered = "\
x_m,s_m,y_m,z_m,width_left_m,banking_deg,width_right_m,grip_scale
0.0,0.0,0.0,0.0,5.0,0.0,5.0,1.0
10.0,10.0,0.0,0.5,5.0,1.0,5.0,1.0
20.0,20.0,0.0,1.0,5.0,2.0,5.0,1.0
";
        let c = parse_centerline(reordered, 3).unwrap();
        assert!(close(c.rows[1].s_m, 10.0));
        assert!(close(c.rows[1].x_m, 10.0));
    }

    #[test]
    fn rejects_missing_column_with_hint() {
        let bad =
            "s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_rite_m,grip_scale\n0,0,0,0,0,5,5,1\n";
        match parse_centerline(bad, 1) {
            Err(CenterlineError::MissingColumn { column, hint }) => {
                assert_eq!(column, "width_right_m");
                assert_eq!(hint.as_deref(), Some("width_rite_m"));
            }
            other => panic!("expected MissingColumn, got {other:?}"),
        }
    }

    #[test]
    fn rejects_non_monotone_s() {
        let bad = "\
s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale
0,0,0,0,0,5,5,1
5,5,0,0,0,5,5,1
5,6,0,0,0,5,5,1
";
        assert!(matches!(
            parse_centerline(bad, 1),
            Err(CenterlineError::NonMonotoneS { line: 4, .. })
        ));
    }

    #[test]
    fn rejects_negative_width() {
        let bad = "\
s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale
0,0,0,0,0,5,5,1
5,5,0,0,0,-1,5,1
";
        assert!(matches!(
            parse_centerline(bad, 1),
            Err(CenterlineError::BadValue { line: 3, .. })
        ));
    }

    #[test]
    fn skips_comments_and_blanks() {
        let with_junk = "\
# Catalunya, synthetic
s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale

0,0,0,0,0,5,5,1
# midpoint
10,10,0,0,0,5,5,1
20,20,0,0,0,5,5,1
";
        assert_eq!(parse_centerline(with_junk, 3).unwrap().rows.len(), 3);
    }
}
