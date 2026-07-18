// SPDX-License-Identifier: AGPL-3.0-only
//! Type/units conversions, error mapping, and small array helpers (no logic).

use crate::prelude::*;

/// Map any core error to a Python `ValueError` carrying its display text.
pub(crate) fn err<E: std::fmt::Display>(e: E) -> PyErr {
    PyValueError::new_err(e.to_string())
}

/// Convert a Python value to JSON for the override/conditions machinery.
///
/// `bool` is checked before `int` (Python bools are ints) so `True` stays a boolean.
pub(crate) fn py_to_json(v: &Bound<'_, PyAny>) -> PyResult<serde_json::Value> {
    if v.is_none() {
        Ok(serde_json::Value::Null)
    } else if let Ok(b) = v.extract::<bool>() {
        Ok(b.into())
    } else if let Ok(i) = v.extract::<i64>() {
        Ok(i.into())
    } else if let Ok(f) = v.extract::<f64>() {
        Ok(f.into())
    } else if let Ok(s) = v.extract::<String>() {
        Ok(s.into())
    } else if let Ok(d) = v.cast::<pyo3::types::PyDict>() {
        let mut m = serde_json::Map::new();
        for (k, val) in d.iter() {
            m.insert(k.extract::<String>()?, py_to_json(&val)?);
        }
        Ok(serde_json::Value::Object(m))
    } else if let Ok(l) = v.cast::<pyo3::types::PyList>() {
        let mut arr = Vec::with_capacity(l.len());
        for item in l.iter() {
            arr.push(py_to_json(&item)?);
        }
        Ok(serde_json::Value::Array(arr))
    } else {
        Err(PyValueError::new_err(format!(
            "unsupported value type in overrides: {}",
            v.get_type().name()?
        )))
    }
}

/// Build the vehicle-pipeline [`Overrides`] from a `{dotted.path: value}` dict.
pub(crate) fn overrides_from(d: Option<&Bound<'_, pyo3::types::PyDict>>) -> PyResult<Overrides> {
    let mut o = Overrides::new();
    if let Some(d) = d {
        for (k, v) in d.iter() {
            o = o.with(k.extract::<String>()?, py_to_json(&v)?);
        }
    }
    Ok(o)
}

/// Load + resolve a vehicle with optional dotted-path overrides (through the real pipeline:
/// schema-validated after the merge, recorded in provenance — Decision #35), threading
/// `allow_degraded` into the load pipeline (the ers↔battery integrity checks downgrade from hard
/// errors to recorded degradations — #40). The solve entry points build their `Sim` FIRST so the
/// real flag reaches the loader; the diagnostic report passes `true` so a degraded car surfaces.
pub(crate) fn resolve_with_overrides_opts(
    vehicle_dir: &str,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    allow_degraded: bool,
) -> PyResult<(FsLoader, ResolvedVehicle)> {
    let vl = FsLoader::new(vehicle_dir);
    let ov = overrides_from(overrides)?;
    let options = LoadOptions { allow_degraded };
    let resolved = load_vehicle_with("vehicle.yaml", &vl, &options, &ov).map_err(schema_err)?;
    Ok((vl, resolved))
}

/// Deep-merge a JSON patch onto typed [`Conditions`]: objects merge recursively, scalars replace.
///
/// The base serialization carries **every** field (a concrete struct), so a patch key that does
/// not already exist is a typo — rejected loudly with its dotted path, never silently ignored
/// (serde without `deny_unknown_fields` would otherwise drop it). Re-deserializing catches type
/// errors.
pub(crate) fn merge_conditions(
    base: Conditions,
    patch: &serde_json::Value,
) -> PyResult<Conditions> {
    fn merge(value: &mut serde_json::Value, patch: &serde_json::Value, path: &str) -> PyResult<()> {
        match (value, patch) {
            (serde_json::Value::Object(v), serde_json::Value::Object(p)) => {
                for (k, pv) in p {
                    let sub = if path.is_empty() {
                        k.clone()
                    } else {
                        format!("{path}.{k}")
                    };
                    if let Some(slot) = v.get_mut(k) {
                        merge(slot, pv, &sub)?;
                    } else {
                        let known: Vec<&String> = v.keys().collect();
                        return Err(PyValueError::new_err(format!(
                            "unknown conditions field `{sub}` (known fields here: {known:?})"
                        )));
                    }
                }
                Ok(())
            }
            (slot, p) => {
                *slot = p.clone();
                Ok(())
            }
        }
    }
    let mut value = serde_json::to_value(&base).map_err(err)?;
    merge(&mut value, patch, "")?;
    serde_json::from_value(value)
        .map_err(|e| PyValueError::new_err(format!("invalid conditions override: {e}")))
}

/// Whether a schema error is "the file does not exist" (as opposed to a malformed file).
pub(crate) fn is_not_found(e: &outlap_schema::SchemaError) -> bool {
    matches!(
        e,
        outlap_schema::SchemaError::Io(outlap_schema::io::SourceError::NotFound { .. })
    )
}

/// Map a schema error to Python: missing file → `FileNotFoundError`, anything else →
/// `ValueError` carrying the message **plus the diagnostic help line** (did-you-mean
/// suggestions etc. — config errors are a product surface, and Display alone drops them).
pub(crate) fn schema_err(e: outlap_schema::SchemaError) -> PyErr {
    use miette::Diagnostic;
    if is_not_found(&e) {
        return PyFileNotFoundError::new_err(e.to_string());
    }
    let msg = match e.help() {
        Some(help) => format!("{e}\nhelp: {help}"),
        None => e.to_string(),
    };
    PyValueError::new_err(msg)
}

/// Map a track error to Python, unwrapping the not-found case like [`schema_err`].
pub(crate) fn track_err(e: outlap_track::TrackError) -> PyErr {
    match e {
        outlap_track::TrackError::Schema(s) => schema_err(s),
        other => err(other),
    }
}

/// Reject a non-positive/NaN sampling step before it reaches the saturating-cast station count
/// (`length/0 → usize::MAX` would abort with a capacity-overflow panic, not a Python exception).
pub(crate) fn check_ds(ds_m: f64) -> PyResult<()> {
    if ds_m > 0.0 && ds_m.is_finite() {
        Ok(())
    } else {
        Err(PyValueError::new_err(format!(
            "ds_m must be a positive, finite number of metres, got {ds_m}"
        )))
    }
}

/// Split a file path into a directory-rooted [`FsLoader`] plus the bare file name.
pub(crate) fn loader_for(path: &str) -> PyResult<(FsLoader, String)> {
    let p = Path::new(path);
    let dir = p.parent().unwrap_or_else(|| Path::new("."));
    let name = p
        .file_name()
        .ok_or_else(|| PyValueError::new_err(format!("not a file path: {path}")))?
        .to_string_lossy()
        .into_owned();
    Ok((FsLoader::new(dir), name))
}

pub(crate) fn entries_to_pairs(entries: &[ReportEntry]) -> Vec<(String, String)> {
    entries
        .iter()
        .map(|e| (e.pointer.clone(), e.detail.clone()))
        .collect()
}

/// Build a `n × 4` numpy array (row-major) from a flat per-wheel channel, or `None`.
pub(crate) fn wheel_array<'py>(
    py: Python<'py>,
    v: Option<&Vec<f64>>,
) -> Option<Bound<'py, PyArray2<f64>>> {
    v.map(|flat| {
        let n = flat.len() / 4;
        numpy::ndarray::Array2::from_shape_vec((n, 4), flat.clone())
            .expect("n×4 per-wheel channel")
            .into_pyarray(py)
    })
}

/// Build the recorded line descriptor from the raceline provenance passed across the boundary.
///
/// `raceline_ds_m = None` ⇒ a centerline lap. Otherwise the generator kind (`"time_weighted"` vs
/// anything else ⇒ min-curvature) and its converged iteration count are recorded honestly.
pub(crate) fn line_descriptor(
    raceline_ds_m: Option<f64>,
    generator: Option<&str>,
    iterations: Option<usize>,
) -> LineDescriptor {
    match raceline_ds_m {
        Some(g) => {
            let iters = iterations.unwrap_or(1);
            if generator == Some("time_weighted") {
                LineDescriptor::TimeWeighted {
                    ds_m: g,
                    iterations: iters,
                }
            } else {
                LineDescriptor::MinCurvature {
                    ds_m: g,
                    iterations: iters,
                }
            }
        }
        None => LineDescriptor::Centerline,
    }
}

/// Row-major flatten of a per-wheel SoA channel (`Vec<[f64; 4]>` → `Vec<f64>`).
pub(crate) fn flat4(v: &[[f64; 4]]) -> Vec<f64> {
    let mut out = Vec::with_capacity(v.len() * 4);
    for row in v {
        out.extend_from_slice(row);
    }
    out
}

/// The recorded `fz_coupling` mode as its snake_case schema string.
pub(crate) fn fz_coupling_str(c: outlap_schema::sim::FzCoupling) -> String {
    match c {
        outlap_schema::sim::FzCoupling::OneStepLag => "one_step_lag".to_owned(),
        outlap_schema::sim::FzCoupling::FixedPoint => "fixed_point".to_owned(),
    }
}

/// Fraction of the QSS speed profile the transient driver tracks by default. The point-mass profile
/// spends the whole grip envelope longitudinally; a transient car with a real tyre needs a margin to
/// stay inside its combined-slip limit on a real circuit. Surfaced in the lap notes.
pub(crate) const DEFAULT_SPEED_MARGIN: f64 = 0.85;
/// Hard cap on transient steps, so a car that cannot complete the lap terminates instead of hanging.
pub(crate) const MAX_TRANSIENT_STEPS: usize = 2_000_000;

/// Build a `n × 4` numpy array (row-major) from a flat per-wheel channel.
pub(crate) fn wheel_array_2d<'py>(py: Python<'py>, flat: &[f64]) -> Bound<'py, PyArray2<f64>> {
    let n = flat.len() / 4;
    numpy::ndarray::Array2::from_shape_vec((n, 4), flat.to_vec())
        .expect("n×4 per-wheel channel")
        .into_pyarray(py)
}

/// Hard cap on stint length, so a typo cannot launch an unbounded run. Far beyond any real dry stint
/// (a full F1 race is ~60 laps); each QSS lap is a sub-second re-solve, each T2 lap seconds.
pub(crate) const MAX_STINT_LAPS: usize = 200;

/// Build a `rows × cols` numpy array (row-major) from a flat channel.
pub(crate) fn array2d<'py>(
    py: Python<'py>,
    flat: &[f64],
    rows: usize,
    cols: usize,
) -> Bound<'py, PyArray2<f64>> {
    numpy::ndarray::Array2::from_shape_vec((rows, cols), flat.to_vec())
        .expect("rows×cols stint channel")
        .into_pyarray(py)
}
