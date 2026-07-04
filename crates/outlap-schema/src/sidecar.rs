// SPDX-License-Identifier: AGPL-3.0-only
//! Binary sidecar decoding: parquet map tables → [`GriddedTable`]/[`GriddedMapN`] (assembly-time).
//!
//! The neutral map files (`.ptm` efficiency/loss tables, `aero.map`, battery ECM tables) reference a
//! long/tidy parquet sidecar of `f64` columns (see the Python importers in `outlap.importers`). This
//! module decodes those bytes and pivots them onto the rectilinear grid consumed by the shared
//! monotone-cubic interpolant.
//!
//! **wasm strategy (Decision, PR1):** the decoder pulls the `parquet` crate, which is not
//! wasm-clean, so it lives behind the non-default `parquet` cargo feature. The *decoded* types
//! ([`GriddedTable`]/[`GriddedMapN`]) live in the wasm-clean `outlap-core`, so solvers never touch
//! parquet. wasm builds (`outlap-raceline`, `outlap-tire`) depend on `outlap-schema` with the
//! feature off; the parquet decode only runs at assembly time on the native/host edge.
//!
//! The importers write `f64` (parquet `DOUBLE`) columns with default pyarrow settings (SNAPPY
//! compression, PLAIN / RLE_DICTIONARY encoding); the record-based reader here handles all of them.

use bytes::Bytes;
use outlap_core::{GridMapError, GriddedMapN, GriddedTable, OutOfDomain};
use parquet::file::reader::{FileReader, SerializedFileReader};
use parquet::record::Field;

use crate::io::{SourceError, SourceLoader};

/// Error decoding a binary map sidecar.
#[derive(Debug, thiserror::Error)]
pub enum SidecarError {
    /// The underlying loader could not provide the sidecar bytes.
    #[error(transparent)]
    Source(#[from] SourceError),
    /// The parquet container could not be parsed.
    #[error("failed to read parquet sidecar: {0}")]
    Parquet(String),
    /// A column held a value that was not a floating-point number.
    #[error("column `{name}` has non-numeric value(s) (found {found})")]
    NonNumericColumn {
        /// The offending column name.
        name: String,
        /// The parquet field kind encountered.
        found: String,
    },
    /// The sidecar had no rows.
    #[error("parquet sidecar is empty")]
    Empty,
    /// Pivoting the columns onto a grid failed.
    #[error(transparent)]
    Grid(#[from] GridMapError),
}

impl From<parquet::errors::ParquetError> for SidecarError {
    fn from(e: parquet::errors::ParquetError) -> Self {
        SidecarError::Parquet(e.to_string())
    }
}

/// Decode every column of a parquet sidecar as an `f64` column, preserving file order.
///
/// Each `DOUBLE`/`FLOAT` value is taken as-is; a `NULL` becomes `NaN` (the same masking the maps use
/// for unreachable cells). Any other field kind is an error.
///
/// # Errors
/// [`SidecarError`] if the container is unreadable, empty, or a column is non-numeric.
pub fn read_columns(bytes: &[u8]) -> Result<Vec<(String, Vec<f64>)>, SidecarError> {
    let reader = SerializedFileReader::new(Bytes::copy_from_slice(bytes))?;
    let schema = reader.metadata().file_metadata().schema();
    let names: Vec<String> = schema
        .get_fields()
        .iter()
        .map(|f| f.name().to_owned())
        .collect();
    if names.is_empty() {
        return Err(SidecarError::Empty);
    }
    let mut columns: Vec<(String, Vec<f64>)> =
        names.iter().map(|n| (n.clone(), Vec::new())).collect();
    let mut n_rows = 0usize;
    for row in reader.get_row_iter(None)? {
        let row = row?;
        n_rows += 1;
        for (i, (_col, field)) in row.get_column_iter().enumerate() {
            let v = match field {
                Field::Double(d) => *d,
                Field::Float(f) => f64::from(*f),
                Field::Null => f64::NAN,
                other => {
                    return Err(SidecarError::NonNumericColumn {
                        name: names[i].clone(),
                        found: format!("{other:?}"),
                    })
                }
            };
            columns[i].1.push(v);
        }
    }
    if n_rows == 0 {
        return Err(SidecarError::Empty);
    }
    Ok(columns)
}

/// Decode a parquet sidecar and pivot it onto a rectilinear [`GriddedTable`].
///
/// `axis_names` selects, in tensor order (outermost first), which columns are the grid axes; the
/// remaining columns become value columns.
///
/// # Errors
/// [`SidecarError`] if the container is unreadable or the columns are not rectilinear.
pub fn read_gridded_table(
    bytes: &[u8],
    axis_names: &[&str],
) -> Result<GriddedTable<f64>, SidecarError> {
    let columns = read_columns(bytes)?;
    Ok(GriddedTable::from_long(&columns, axis_names)?)
}

/// Decode a parquet sidecar and build a single-value interpolant in one step.
///
/// # Errors
/// [`SidecarError`] if the container is unreadable, the columns are not rectilinear, or the value
/// column is missing.
pub fn read_gridded_map(
    bytes: &[u8],
    axis_names: &[&str],
    value_name: &str,
    modes: Vec<OutOfDomain>,
) -> Result<GriddedMapN<f64>, SidecarError> {
    let table = read_gridded_table(bytes, axis_names)?;
    Ok(table.map(value_name, modes)?)
}

/// Load a parquet sidecar through a [`SourceLoader`] and build an interpolant.
///
/// # Errors
/// [`SidecarError`] if the loader cannot supply the bytes or the sidecar cannot be decoded.
pub fn load_gridded_map(
    loader: &dyn SourceLoader,
    path: &str,
    axis_names: &[&str],
    value_name: &str,
    modes: Vec<OutOfDomain>,
) -> Result<GriddedMapN<f64>, SidecarError> {
    let bytes = loader.load_bytes(path)?;
    read_gridded_map(&bytes, axis_names, value_name, modes)
}
