// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-py` — the `outlap_core` Python extension module (HANDOFF §11.1b).
//!
//! Thin, numpy-friendly bindings over the Rust core: the MF6.1 tire model (`Tyre`), the 3D track
//! (`Track`), the min-curvature racing line, and the T0 point-mass lap solver (`Lap`). The typed,
//! documented user API lives on the Python side (`outlap.core`); this layer only converts types
//! and maps errors, never adds logic.
//!
//! This is the sanctioned FFI crate (CLAUDE.md): PyO3's macros generate `unsafe` glue, so —
//! uniquely in the workspace — `forbid(unsafe_code)` is not applied here.

#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::needless_pass_by_value,
    clippy::doc_markdown,
    // Channel names mirror the physics API (s, v, x, y, z, n — paper symbols).
    clippy::many_single_char_names,
    clippy::similar_names,
    // The 5-tuple of force arrays IS the FFI contract; a type alias would just rename it.
    clippy::type_complexity,
    // `lap_time_s` matches the Rust LapResult field name (public contract).
    clippy::struct_field_names
)]

use std::collections::HashMap;
use std::path::Path;
use std::sync::{LazyLock, Mutex};

use numpy::{IntoPyArray, PyArray1, PyArray2, PyReadonlyArray1};
use pyo3::exceptions::{PyFileNotFoundError, PyValueError};
use pyo3::prelude::*;

use outlap_qss::{
    solve_t0, solve_t1, tier_not_implemented, GgvEnvelope, LineDescriptor, QssLap, SetupLog,
    SlowLog, T0Options, T0Path, T0Vehicle, T1Vehicle, WheelLog, DEFAULT_DS_M, WHEEL_ORDER,
};
use outlap_raceline::{min_curvature_line, RacelineOptions};
use outlap_schema::io::FsLoader;
use outlap_schema::load::load_tyr;
use outlap_schema::load::report::ReportEntry;
use outlap_schema::sim::{Sim, Tier};
use outlap_schema::{
    load_conditions, load_sim, load_vehicle_with, Conditions, LoadOptions, Overrides,
    ResolvedVehicle,
};
use outlap_tire::{peak_mu_x, peak_mu_y, Mf61, SlipState};

/// Map any core error to a Python `ValueError` carrying its display text.
fn err<E: std::fmt::Display>(e: E) -> PyErr {
    PyValueError::new_err(e.to_string())
}

/// Convert a Python value to JSON for the override/conditions machinery.
///
/// `bool` is checked before `int` (Python bools are ints) so `True` stays a boolean.
fn py_to_json(v: &Bound<'_, PyAny>) -> PyResult<serde_json::Value> {
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
fn overrides_from(d: Option<&Bound<'_, pyo3::types::PyDict>>) -> PyResult<Overrides> {
    let mut o = Overrides::new();
    if let Some(d) = d {
        for (k, v) in d.iter() {
            o = o.with(k.extract::<String>()?, py_to_json(&v)?);
        }
    }
    Ok(o)
}

/// Load + resolve a vehicle with optional dotted-path overrides (through the real pipeline:
/// schema-validated after the merge, recorded in provenance — Decision #35).
fn resolve_with_overrides(
    vehicle_dir: &str,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
) -> PyResult<(FsLoader, ResolvedVehicle)> {
    let vl = FsLoader::new(vehicle_dir);
    let ov = overrides_from(overrides)?;
    let resolved =
        load_vehicle_with("vehicle.yaml", &vl, &LoadOptions::default(), &ov).map_err(schema_err)?;
    Ok((vl, resolved))
}

/// Deep-merge a JSON patch onto typed [`Conditions`]: objects merge recursively, scalars replace.
///
/// The base serialization carries **every** field (a concrete struct), so a patch key that does
/// not already exist is a typo — rejected loudly with its dotted path, never silently ignored
/// (serde without `deny_unknown_fields` would otherwise drop it). Re-deserializing catches type
/// errors.
fn merge_conditions(base: Conditions, patch: &serde_json::Value) -> PyResult<Conditions> {
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
fn is_not_found(e: &outlap_schema::SchemaError) -> bool {
    matches!(
        e,
        outlap_schema::SchemaError::Io(outlap_schema::io::SourceError::NotFound { .. })
    )
}

/// Map a schema error to Python: missing file → `FileNotFoundError`, anything else →
/// `ValueError` carrying the message **plus the diagnostic help line** (did-you-mean
/// suggestions etc. — config errors are a product surface, and Display alone drops them).
fn schema_err(e: outlap_schema::SchemaError) -> PyErr {
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
fn track_err(e: outlap_track::TrackError) -> PyErr {
    match e {
        outlap_track::TrackError::Schema(s) => schema_err(s),
        other => err(other),
    }
}

/// Reject a non-positive/NaN sampling step before it reaches the saturating-cast station count
/// (`length/0 → usize::MAX` would abort with a capacity-overflow panic, not a Python exception).
fn check_ds(ds_m: f64) -> PyResult<()> {
    if ds_m > 0.0 && ds_m.is_finite() {
        Ok(())
    } else {
        Err(PyValueError::new_err(format!(
            "ds_m must be a positive, finite number of metres, got {ds_m}"
        )))
    }
}

/// Split a file path into a directory-rooted [`FsLoader`] plus the bare file name.
fn loader_for(path: &str) -> PyResult<(FsLoader, String)> {
    let p = Path::new(path);
    let dir = p.parent().unwrap_or_else(|| Path::new("."));
    let name = p
        .file_name()
        .ok_or_else(|| PyValueError::new_err(format!("not a file path: {path}")))?
        .to_string_lossy()
        .into_owned();
    Ok((FsLoader::new(dir), name))
}

fn entries_to_pairs(entries: &[ReportEntry]) -> Vec<(String, String)> {
    entries
        .iter()
        .map(|e| (e.pointer.clone(), e.detail.clone()))
        .collect()
}

/// A steady-state MF6.1 tire model loaded from a `.tyr` file.
#[pyclass(frozen)]
pub struct Tyre {
    model: Mf61<f64>,
    /// Load warnings + parameter-extraction notes, as `(json_pointer, detail)` pairs.
    #[pyo3(get)]
    notes: Vec<(String, String)>,
    /// Literature citation from the file's provenance block.
    #[pyo3(get)]
    citation: String,
    /// Nominal load `FNOMIN`, N.
    #[pyo3(get)]
    fnomin: f64,
    /// Unloaded radius `R0`, m.
    #[pyo3(get)]
    unloaded_radius: f64,
    /// Cold inflation pressure from the thermal block, Pa.
    #[pyo3(get)]
    p_cold: f64,
}

#[pymethods]
impl Tyre {
    /// Load a `.tyr` file and build the evaluatable MF6.1 model.
    #[staticmethod]
    fn load(path: &str) -> PyResult<Self> {
        let (loader, name) = loader_for(path)?;
        let (tyr, warnings) = load_tyr(&name, &loader).map_err(schema_err)?;
        let (model, param_notes) = Mf61::<f64>::from_tyr(&tyr).map_err(err)?;
        let mut notes = entries_to_pairs(&warnings);
        notes.extend(entries_to_pairs(&param_notes));
        let p = model.params();
        Ok(Self {
            fnomin: p.fnomin,
            unloaded_radius: p.r0,
            p_cold: tyr.thermal.p_cold * 1000.0, // kPa (file boundary) → Pa (SI internal)
            citation: tyr.provenance.citation.clone(),
            notes,
            model,
        })
    }

    /// Evaluate steady-state forces/moments over equal-length arrays of contact-patch states.
    ///
    /// Inputs: `kappa` (slip ratio), `alpha` (rad), `gamma` (rad), `fz` (N), `p` (Pa),
    /// `vx` (m/s). Returns `(fx, fy, mz, mx, my)` arrays (N, N·m), ISO 8855 sign convention.
    #[allow(clippy::too_many_arguments, clippy::similar_names)]
    fn forces<'py>(
        &self,
        py: Python<'py>,
        kappa: PyReadonlyArray1<'py, f64>,
        alpha: PyReadonlyArray1<'py, f64>,
        gamma: PyReadonlyArray1<'py, f64>,
        fz: PyReadonlyArray1<'py, f64>,
        p: PyReadonlyArray1<'py, f64>,
        vx: PyReadonlyArray1<'py, f64>,
    ) -> PyResult<(
        Bound<'py, PyArray1<f64>>,
        Bound<'py, PyArray1<f64>>,
        Bound<'py, PyArray1<f64>>,
        Bound<'py, PyArray1<f64>>,
        Bound<'py, PyArray1<f64>>,
    )> {
        let (kappa, alpha) = (kappa.as_slice()?, alpha.as_slice()?);
        let (gamma, fz) = (gamma.as_slice()?, fz.as_slice()?);
        let (p, vx) = (p.as_slice()?, vx.as_slice()?);
        let n = kappa.len();
        for (name, arr) in [
            ("alpha", alpha),
            ("gamma", gamma),
            ("fz", fz),
            ("p", p),
            ("vx", vx),
        ] {
            if arr.len() != n {
                return Err(PyValueError::new_err(format!(
                    "length mismatch: kappa has {n} elements, {name} has {}",
                    arr.len()
                )));
            }
        }

        let (mut fx, mut fy) = (Vec::with_capacity(n), Vec::with_capacity(n));
        let (mut mz, mut mx) = (Vec::with_capacity(n), Vec::with_capacity(n));
        let mut my = Vec::with_capacity(n);
        for i in 0..n {
            let f = self.model.forces(&SlipState::new(
                kappa[i], alpha[i], gamma[i], fz[i], p[i], vx[i],
            ));
            fx.push(f.fx);
            fy.push(f.fy);
            mz.push(f.mz);
            mx.push(f.mx);
            my.push(f.my);
        }
        Ok((
            fx.into_pyarray(py),
            fy.into_pyarray(py),
            mz.into_pyarray(py),
            mx.into_pyarray(py),
            my.into_pyarray(py),
        ))
    }

    /// Peak friction `(μx, μy)` from the pure-slip curves at load `fz` (N) and pressure `p` (Pa).
    fn peak_mu(&self, fz: f64, p: f64) -> (f64, f64) {
        (peak_mu_x(&self.model, fz, p), peak_mu_y(&self.model, fz, p))
    }
}

/// A loaded 3D track (queryable ribbon: position, curvature, grade, banking, width).
#[pyclass(frozen)]
pub struct Track {
    inner: outlap_track::Track,
}

#[pymethods]
impl Track {
    /// Load `track.yaml` (+ its centerline CSV) from a track directory.
    #[staticmethod]
    fn load(dir: &str) -> PyResult<Self> {
        let inner =
            outlap_track::Track::load("track.yaml", &FsLoader::new(dir)).map_err(track_err)?;
        Ok(Self { inner })
    }

    /// Track display name.
    fn name(&self) -> String {
        self.inner.name().to_owned()
    }

    /// Total arc length, m.
    fn length(&self) -> f64 {
        self.inner.length()
    }

    /// Whether the track is a closed loop.
    fn is_closed(&self) -> bool {
        self.inner.is_closed()
    }

    /// Sample the ribbon at a uniform `ds_m` (m) → dict of equal-length arrays:
    /// `s, x, y, z, kappa_h, kappa_v, grade, banking, width_left, width_right`.
    fn sample<'py>(&self, py: Python<'py>, ds_m: f64) -> PyResult<Bound<'py, pyo3::types::PyDict>> {
        check_ds(ds_m)?;
        let s = self.inner.sample_uniform(ds_m);
        let d = pyo3::types::PyDict::new(py);
        d.set_item("s", s.s.into_pyarray(py))?;
        d.set_item("x", s.x.into_pyarray(py))?;
        d.set_item("y", s.y.into_pyarray(py))?;
        d.set_item("z", s.z.into_pyarray(py))?;
        d.set_item("kappa_h", s.kappa_h.into_pyarray(py))?;
        d.set_item("kappa_v", s.kappa_v.into_pyarray(py))?;
        d.set_item("grade", s.grade.into_pyarray(py))?;
        d.set_item("banking", s.banking.into_pyarray(py))?;
        d.set_item("width_left", s.width_left.into_pyarray(py))?;
        d.set_item("width_right", s.width_right.into_pyarray(py))?;
        Ok(d)
    }
}

/// A generated minimum-curvature racing line.
#[pyclass(frozen)]
pub struct Raceline {
    s: Vec<f64>,
    n: Vec<f64>,
    line: Py<Track>,
    /// The sampling step the line was GENERATED with, m (recorded into lap provenance).
    #[pyo3(get)]
    ds_m: f64,
}

#[pymethods]
impl Raceline {
    /// Parent-centerline stations, m.
    fn s<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.s.clone().into_pyarray(py)
    }

    /// Signed lateral offsets at each station (`+` road-left), m.
    fn n<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.n.clone().into_pyarray(py)
    }

    /// The racing line as a first-class [`Track`] (own curvature/frames), drivable by the solver.
    fn line(&self, py: Python<'_>) -> Py<Track> {
        self.line.clone_ref(py)
    }
}

/// Generate the minimum-curvature racing line inside the track corridor (Decision #14).
///
/// `half_width_m` is the car half-width; `margin_m` extra safety margin; `ds_m` the QP sampling
/// step; `epsilon` the Tikhonov regularisation.
#[pyfunction]
#[pyo3(signature = (track, half_width_m, ds_m = 2.0, margin_m = 0.3, epsilon = 1e-8))]
fn min_curvature(
    py: Python<'_>,
    track: &Track,
    half_width_m: f64,
    ds_m: f64,
    margin_m: f64,
    epsilon: f64,
) -> PyResult<Raceline> {
    check_ds(ds_m)?;
    if !(half_width_m > 0.0 && half_width_m.is_finite()) {
        return Err(PyValueError::new_err(format!(
            "half_width_m must be a positive, finite number of metres, got {half_width_m}"
        )));
    }
    let opts = RacelineOptions {
        ds_m,
        margin_m,
        epsilon,
    };
    let r = min_curvature_line(&track.inner, half_width_m, &opts).map_err(err)?;
    Ok(Raceline {
        s: r.s,
        n: r.n,
        line: Py::new(py, Track { inner: r.line })?,
        ds_m,
    })
}

/// A queryable g-g-g-v envelope (the returnable `lap.envelope`): the T1-derived tyre-grip boundary
/// the QSS lap ran on. Zero-copy scalar queries; `to_dataset` is built on the Python side.
#[pyclass(frozen)]
pub struct Envelope {
    inner: GgvEnvelope,
}

#[pymethods]
impl Envelope {
    /// Lateral-acceleration boundary at `(v, a_x, g_normal)` (velocity frame), m/s².
    fn ay_boundary(&self, v: f64, ax: f64, g_normal: f64) -> f64 {
        self.inner.ay_boundary(v, ax, g_normal)
    }
    /// Maximum positive longitudinal acceleration (net of drag) at `(v, g_normal)`, m/s².
    fn accel_limit(&self, v: f64, g_normal: f64) -> f64 {
        self.inner.accel_limit(v, g_normal)
    }
    /// Maximum braking deceleration magnitude at `(v, g_normal)`, m/s².
    fn brake_limit(&self, v: f64, g_normal: f64) -> f64 {
        self.inner.brake_limit(v, g_normal)
    }
    /// Reference straight-line drag as an acceleration at speed `v`, m/s².
    fn drag_accel(&self, v: f64) -> f64 {
        self.inner.drag_accel(v)
    }
    /// The `[first, last]` bounds of the `(v, â_x, g_normal)` axes (`â_x` normalised to ±1).
    fn domain(&self) -> [[f64; 2]; 3] {
        self.inner.domain().map(|(lo, hi)| [lo, hi])
    }
    /// The grid shape `[n_v, n_âx, n_g_normal]`.
    fn shape(&self) -> [usize; 3] {
        self.inner.shape()
    }
    /// The reference mass the envelope was generated at, kg.
    fn mass_ref(&self) -> f64 {
        self.inner.mass_ref()
    }
    /// Generation notes / simplifications (nothing silent).
    #[getter]
    fn notes(&self) -> Vec<String> {
        self.inner.notes().to_vec()
    }
}

/// A solved QSS lap: point-mass channels over arc-length stations always; for `tier="t1"` also the
/// per-wheel (`s × wheel`) loads/slips/forces, the setup metrics, and any slow-state channels.
#[pyclass(frozen)]
pub struct Lap {
    s: Vec<f64>,
    v: Vec<f64>,
    ax: Vec<f64>,
    ay: Vec<f64>,
    t: Vec<f64>,
    x: Vec<f64>,
    y: Vec<f64>,
    z: Vec<f64>,
    // Per-wheel channels (row-major `n × 4`, wheel order FL/FR/RL/RR); `None` for t0.
    vertical_load_n: Option<Vec<f64>>,
    slip_ratio: Option<Vec<f64>>,
    slip_angle_rad: Option<Vec<f64>>,
    force_long_n: Option<Vec<f64>>,
    force_lat_n: Option<Vec<f64>>,
    // Setup metrics (per station); `None` for t0.
    understeer_gradient: Option<Vec<f64>>,
    aero_front_share: Option<Vec<f64>>,
    // Slow-state channels (per station); `None` unless a coupled electrified stack was active.
    state_of_charge: Option<Vec<f64>>,
    machine_temp_c: Option<Vec<f64>>,
    envelope: Option<GgvEnvelope>,
    /// Total lap time, s.
    #[pyo3(get)]
    lap_time_s: f64,
    /// The resolved solver tier (`"t0"` / `"t1"`).
    #[pyo3(get)]
    tier: String,
    /// The recorded normal-load coupling mode (`"one_step_lag"` / `"fixed_point"`).
    #[pyo3(get)]
    fz_coupling: String,
    /// Whether the lap ran in flat-track analysis mode.
    #[pyo3(get)]
    flat_track: bool,
    /// The wheel-channel order for the per-wheel arrays (`["FL","FR","RL","RR"]`).
    #[pyo3(get)]
    wheels: Vec<String>,
    /// Simplification/degradation notes (nothing silent).
    #[pyo3(get)]
    notes: Vec<String>,
    /// blake3 hash of the resolved vehicle spec that produced this lap.
    #[pyo3(get)]
    resolved_hash: String,
}

/// Build a `n × 4` numpy array (row-major) from a flat per-wheel channel, or `None`.
fn wheel_array<'py>(py: Python<'py>, v: Option<&Vec<f64>>) -> Option<Bound<'py, PyArray2<f64>>> {
    v.map(|flat| {
        let n = flat.len() / 4;
        numpy::ndarray::Array2::from_shape_vec((n, 4), flat.clone())
            .expect("n×4 per-wheel channel")
            .into_pyarray(py)
    })
}

#[pymethods]
impl Lap {
    /// Arc-length stations, m.
    fn s<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.s.clone().into_pyarray(py)
    }
    /// Speed, m/s.
    fn v<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.v.clone().into_pyarray(py)
    }
    /// Longitudinal acceleration, m/s².
    fn ax<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.ax.clone().into_pyarray(py)
    }
    /// Lateral acceleration (ISO 8855, `+` left), m/s².
    fn ay<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.ay.clone().into_pyarray(py)
    }
    /// Cumulative time, s.
    fn t<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.t.clone().into_pyarray(py)
    }
    /// World x at each station, m.
    fn x<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.x.clone().into_pyarray(py)
    }
    /// World y at each station, m.
    fn y<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.y.clone().into_pyarray(py)
    }
    /// World z (elevation) at each station, m.
    fn z<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.z.clone().into_pyarray(py)
    }
    /// Per-wheel vertical (normal) load `F_z`, N — shape `n × 4` (FL/FR/RL/RR), or `None` for t0.
    fn vertical_load_n<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.vertical_load_n.as_ref())
    }
    /// Per-wheel longitudinal slip ratio `κ`, shape `n × 4`, or `None` for t0.
    fn slip_ratio<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.slip_ratio.as_ref())
    }
    /// Per-wheel slip angle `α`, rad, shape `n × 4`, or `None` for t0.
    fn slip_angle_rad<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.slip_angle_rad.as_ref())
    }
    /// Per-wheel longitudinal force `F_x`, N, shape `n × 4`, or `None` for t0.
    fn force_long_n<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.force_long_n.as_ref())
    }
    /// Per-wheel lateral force `F_y`, N, shape `n × 4`, or `None` for t0.
    fn force_lat_n<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.force_lat_n.as_ref())
    }
    /// Understeer gradient per station (rad·s²/m), or `None` for t0.
    fn understeer_gradient<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.understeer_gradient
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Front axle downforce share per station (0..1), or `None` for t0.
    fn aero_front_share<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.aero_front_share
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Pack state of charge per station (0..1), or `None` when no coupled stack was active.
    fn state_of_charge<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.state_of_charge
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Machine winding temperature per station (°C), or `None` when no coupled stack was active.
    fn machine_temp_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.machine_temp_c
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// The g-g-g-v envelope this lap ran on (queryable), or `None` for the degenerate path.
    #[getter]
    fn envelope(&self) -> Option<Envelope> {
        self.envelope.clone().map(|inner| Envelope { inner })
    }
}

/// Build the `Sim` for a run: the vehicle-dir `sim.yaml` (or defaults), deep-merged with the `sim`
/// override dict, then the `tier=` convenience override. A missing `sim.yaml` is fine (defaults); a
/// present-but-broken one is a real error.
fn build_sim(
    vl: &FsLoader,
    sim_patch: Option<&Bound<'_, pyo3::types::PyDict>>,
    tier: Option<&str>,
) -> PyResult<Sim> {
    let base = match load_sim("sim.yaml", vl) {
        Ok(s) => s,
        Err(e) if is_not_found(&e) => Sim::default(),
        Err(e) => return Err(schema_err(e)),
    };
    let mut value = serde_json::to_value(&base).map_err(err)?;
    if let Some(patch) = sim_patch {
        merge_json(&mut value, &py_to_json(patch.as_any())?, "sim")?;
    }
    if let Some(t) = tier {
        value["tier"] = serde_json::Value::String(t.to_owned());
    }
    serde_json::from_value(value).map_err(|e| PyValueError::new_err(format!("invalid sim: {e}")))
}

/// Deep-merge a JSON `patch` onto `value`, erroring on an unknown object key (a product surface).
fn merge_json(
    value: &mut serde_json::Value,
    patch: &serde_json::Value,
    path: &str,
) -> PyResult<()> {
    match (value, patch) {
        (serde_json::Value::Object(v), serde_json::Value::Object(p)) => {
            for (k, pv) in p {
                let sub = format!("{path}.{k}");
                if let Some(slot) = v.get_mut(k) {
                    merge_json(slot, pv, &sub)?;
                } else {
                    let known: Vec<&String> = v.keys().collect();
                    return Err(PyValueError::new_err(format!(
                        "unknown sim field `{sub}` (known fields here: {known:?})"
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

/// Process-level cache of generated g-g-g-v envelopes. Generation is a seconds-scale cold step, so
/// a notebook or sweep running many laps of the same car+grid pays it once. Keyed by the resolved
/// vehicle hash, the session conditions, the envelope grid, and the coupling mode — everything that
/// changes the boundary. Bounded implicitly by the small number of distinct (car, grid) combos a
/// session touches; not evicted (a session is short-lived).
static ENV_CACHE: LazyLock<Mutex<HashMap<String, GgvEnvelope>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// The generated (or cached) envelope for a resolved car + numerics. Envelope generation ignores the
/// flat-track flag (it only reshapes the path), so that is not part of the key.
fn cached_envelope(
    t1v: &T1Vehicle,
    sim_cfg: &Sim,
    resolved_hash: &str,
    conditions: &Conditions,
) -> PyResult<GgvEnvelope> {
    let e = &sim_cfg.envelope;
    let cond = serde_json::to_string(conditions).map_err(err)?;
    let key = format!(
        "{resolved_hash}|{cond}|{}x{}x{}|{:?}",
        e.v_points, e.ax_points, e.g_normal_points, sim_cfg.fz_coupling
    );
    if let Some(env) = ENV_CACHE.lock().expect("env cache mutex").get(&key) {
        return Ok(env.clone());
    }
    let env = GgvEnvelope::generate(t1v, e, sim_cfg.fz_coupling).map_err(err)?;
    ENV_CACHE
        .lock()
        .expect("env cache mutex")
        .insert(key, env.clone());
    Ok(env)
}

/// Row-major flatten of a per-wheel SoA channel (`Vec<[f64; 4]>` → `Vec<f64>`).
fn flat4(v: &[[f64; 4]]) -> Vec<f64> {
    let mut out = Vec::with_capacity(v.len() * 4);
    for row in v {
        out.extend_from_slice(row);
    }
    out
}

/// Solve a QSS lap of `track` for the vehicle in `vehicle_dir` at the tier `sim.tier` selects.
///
/// `vehicle_dir` must hold a `vehicle.yaml` (plus its referenced `.ptm`/`.tyr` files); optional
/// `conditions.yaml` / `sim.yaml` next to it override the defaults (a *malformed* one is an error —
/// never silently ignored). `tier=` (`"t0"`/`"t1"`) and `sim=` (a nested override dict, e.g.
/// `{"flat_track": True, "envelope": {"v_points": 24}}`) override the file/defaults; `tier=` wins.
///
/// `t0` runs the point-mass velocity profile on the corrected g-g-g-v envelope; `t1` adds a
/// per-station re-trim for per-wheel loads/slips/forces + setup metrics; `t2`/`t3` raise (they land
/// in M4/M6). When `track` is a generated racing line, pass `raceline_ds_m` for honest provenance.
///
/// What-if experiments (Decision #35): `overrides` is a `{dotted.path: value}` vehicle patch;
/// `conditions` is a nested dict deep-merged onto the session conditions.
///
/// The call holds the GIL for its duration (envelope generation is a seconds-scale cold step in a
/// debug build); releasing it is deferred to the batch/sweep API milestone.
#[pyfunction]
#[pyo3(signature = (vehicle_dir, track, ds_m = DEFAULT_DS_M, raceline_ds_m = None, overrides = None, conditions = None, tier = None, sim = None))]
#[allow(clippy::too_many_arguments)]
fn solve_lap(
    vehicle_dir: &str,
    track: &Track,
    ds_m: f64,
    raceline_ds_m: Option<f64>,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    tier: Option<&str>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
) -> PyResult<Lap> {
    check_ds(ds_m)?;
    let (vl, resolved) = resolve_with_overrides(vehicle_dir, overrides)?;
    let sim_cfg = build_sim(&vl, sim, tier)?;
    // Missing conditions.yaml → ISA defaults; a PRESENT-but-broken one is a real error.
    let base_conditions = match load_conditions("conditions.yaml", &vl) {
        Ok(c) => c,
        Err(e) if is_not_found(&e) => Conditions::default(),
        Err(e) => return Err(schema_err(e)),
    };
    let conditions = match conditions {
        Some(patch) => merge_conditions(base_conditions, &py_to_json(patch.as_any())?)?,
        None => base_conditions,
    };
    let opts = T0Options {
        ds_m,
        allow_degraded: sim_cfg.allow_degraded,
        ..T0Options::default()
    };
    let path = if sim_cfg.flat_track {
        T0Path::from_track_flat(&track.inner, ds_m)
    } else {
        T0Path::from_track(&track.inner, ds_m)
    };
    let line = match raceline_ds_m {
        Some(g) => LineDescriptor::MinCurvature {
            ds_m: g,
            iterations: 1,
        },
        None => LineDescriptor::Centerline,
    };
    let hash = resolved.report.resolved_hash.clone();

    // Enum dispatch on the resolved tier (assembly-time, never in the loop).
    let qss: QssLap = match sim_cfg.tier {
        tier @ (Tier::T2 | Tier::T3) => return Err(err(tier_not_implemented(tier))),
        wanted => {
            let t1v = T1Vehicle::assemble(&resolved, &conditions, &vl, sim_cfg.allow_degraded)
                .map_err(err)?;
            let env = cached_envelope(&t1v, &sim_cfg, &hash, &conditions)?;
            let t0v = T0Vehicle::assemble(&resolved, &conditions, &vl, &opts).map_err(err)?;
            let mut notes = t0v.notes().to_vec();
            notes.extend(t1v.notes().iter().cloned());
            notes.extend(env.notes().iter().cloned());
            // The committed vehicles carry no battery+emotor stack, so the slow-state coupling is
            // inert here (`None`); the coupled path is exercised by the Rust full-stack fixture.
            if wanted == Tier::T0 {
                solve_t0(
                    &t0v,
                    env,
                    None,
                    &path,
                    line,
                    hash,
                    notes,
                    sim_cfg.fz_coupling,
                    sim_cfg.flat_track,
                )
                .map_err(err)?
            } else {
                solve_t1(
                    &t1v,
                    &t0v,
                    env,
                    None,
                    &path,
                    line,
                    hash,
                    notes,
                    sim_cfg.fz_coupling,
                    sim_cfg.flat_track,
                )
                .map_err(err)?
            }
        }
    };

    Ok(qss_lap_to_py(qss, track))
}

/// Convert a solved [`QssLap`] into the Python `Lap`, reconstructing world positions from the track.
fn qss_lap_to_py(qss: QssLap, track: &Track) -> Lap {
    let lap = qss.lap;
    let n = lap.s.len();
    let (mut x, mut y, mut z) = (
        Vec::with_capacity(n),
        Vec::with_capacity(n),
        Vec::with_capacity(n),
    );
    for &si in &lap.s {
        let p = track.inner.position(si);
        x.push(p[0]);
        y.push(p[1]);
        z.push(p[2]);
    }
    let wheels: Option<&WheelLog> = qss.wheels.as_ref();
    let setup: Option<&SetupLog> = qss.setup.as_ref();
    let slow: Option<&SlowLog> = qss.slow.as_ref();
    Lap {
        s: lap.s,
        v: lap.v,
        ax: lap.ax,
        ay: lap.ay,
        t: lap.t,
        x,
        y,
        z,
        vertical_load_n: wheels.map(|w| flat4(&w.vertical_load_n)),
        slip_ratio: wheels.map(|w| flat4(&w.slip_ratio)),
        slip_angle_rad: wheels.map(|w| flat4(&w.slip_angle_rad)),
        force_long_n: wheels.map(|w| flat4(&w.force_long_n)),
        force_lat_n: wheels.map(|w| flat4(&w.force_lat_n)),
        understeer_gradient: setup.map(|s| s.understeer_gradient.clone()),
        aero_front_share: setup.map(|s| s.aero_front_share.clone()),
        state_of_charge: slow.map(|s| s.state_of_charge.clone()),
        machine_temp_c: slow.map(|s| s.machine_temp_c.clone()),
        envelope: qss.envelope,
        lap_time_s: lap.lap_time_s,
        tier: format!("{:?}", qss.tier).to_lowercase(),
        fz_coupling: fz_coupling_str(qss.fz_coupling),
        flat_track: qss.flat_track,
        wheels: WHEEL_ORDER.iter().map(|s| (*s).to_owned()).collect(),
        notes: lap.notes,
        resolved_hash: lap.resolved_hash,
    }
}

/// The recorded `fz_coupling` mode as its snake_case schema string.
fn fz_coupling_str(c: outlap_schema::sim::FzCoupling) -> String {
    match c {
        outlap_schema::sim::FzCoupling::OneStepLag => "one_step_lag".to_owned(),
        outlap_schema::sim::FzCoupling::FixedPoint => "fixed_point".to_owned(),
    }
}

/// Load and resolve a vehicle, returning its loaded-model report as a dict:
/// `{name, resolved_hash, inherited, estimated, degraded, warnings, overrides}` (entry lists are
/// `(json_pointer, detail)` pairs). Nothing silent (Decision #41).
///
/// `overrides` is the same `{dotted.path: value}` what-if dict as [`solve_lap`]'s; the applied
/// paths are echoed back under the `overrides` key, and the `resolved_hash` reflects them.
#[pyfunction]
#[pyo3(signature = (vehicle_dir, overrides = None))]
fn vehicle_report<'py>(
    py: Python<'py>,
    vehicle_dir: &str,
    overrides: Option<&Bound<'py, pyo3::types::PyDict>>,
) -> PyResult<Bound<'py, pyo3::types::PyDict>> {
    let applied: Vec<(String, String)> = match overrides {
        Some(d) => d
            .iter()
            .map(|(k, v)| Ok((k.extract::<String>()?, v.str()?.to_string())))
            .collect::<PyResult<_>>()?,
        None => Vec::new(),
    };
    let (_vl, resolved) = resolve_with_overrides(vehicle_dir, overrides)?;
    let d = pyo3::types::PyDict::new(py);
    d.set_item("name", &resolved.spec.name)?;
    d.set_item("resolved_hash", &resolved.report.resolved_hash)?;
    d.set_item("inherited", entries_to_pairs(&resolved.report.inherited))?;
    d.set_item("estimated", entries_to_pairs(&resolved.report.estimated))?;
    d.set_item("degraded", entries_to_pairs(&resolved.report.degraded))?;
    d.set_item("warnings", entries_to_pairs(&resolved.report.warnings))?;
    d.set_item("overrides", applied)?;
    Ok(d)
}

/// The `outlap_core` extension module.
#[pymodule]
fn outlap_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Tyre>()?;
    m.add_class::<Track>()?;
    m.add_class::<Raceline>()?;
    m.add_class::<Lap>()?;
    m.add_class::<Envelope>()?;
    m.add_function(wrap_pyfunction!(min_curvature, m)?)?;
    m.add_function(wrap_pyfunction!(solve_lap, m)?)?;
    m.add_function(wrap_pyfunction!(vehicle_report, m)?)?;
    m.add("DEFAULT_DS_M", DEFAULT_DS_M)?;
    Ok(())
}
