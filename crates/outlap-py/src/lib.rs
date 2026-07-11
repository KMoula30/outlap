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
    solve_t0, solve_t1, tier_not_implemented, GgvEnvelope, LineDescriptor, MachineThermal, Pack,
    PackState, QssLap, SetupLog, SlowCoupling, SlowLog, T0Options, T0Path, T0Vehicle, T1Vehicle,
    WheelLog, DEFAULT_DS_M, WHEEL_ORDER,
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
    // Optional fields with `skip_serializing_if` are absent from the serialized base and would be
    // rejected as typos by the strict merge — inject them as nulls so the documented overrides
    // (e.g. `sim={"raceline": {"file": "line.csv"}}`) work.
    if let Some(r) = value.get_mut("raceline").and_then(|r| r.as_object_mut()) {
        r.entry("generator").or_insert(serde_json::Value::Null);
        r.entry("file").or_insert(serde_json::Value::Null);
    }
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

/// Load a sidecar table referenced from `referencing` (a YAML path inside the vehicle dir),
/// resolving `file` relative to the referencing document's directory FIRST (the PDT importers
/// emit sidecars next to their YAML) and falling back to the vehicle root. Returns the bytes, or
/// `None` when the file is absent at both locations (the caller notes the skip); a present-but-
/// unreadable file is a real error.
fn load_sidecar_bytes(
    vl: &FsLoader,
    referencing: &str,
    file: &str,
    notes: &mut Vec<String>,
) -> PyResult<Option<Vec<u8>>> {
    use outlap_schema::io::{SourceError, SourceLoader};
    let mut candidates: Vec<String> = Vec::with_capacity(2);
    if let Some((dir, _)) = referencing.rsplit_once('/') {
        candidates.push(format!("{dir}/{file}"));
    }
    if !candidates.iter().any(|c| c == file) {
        candidates.push(file.to_owned());
    }
    let mut resolved: Option<(usize, Vec<u8>)> = None;
    for (i, cand) in candidates.iter().enumerate() {
        match vl.load_bytes(cand) {
            Ok(bytes) => {
                if let Some((first, _)) = &resolved {
                    // Both candidates exist: the yaml-relative one wins — say so (nothing silent).
                    notes.push(format!(
                        "sidecar `{file}` exists at both `{}` and `{}` — using `{}`",
                        candidates[*first], cand, candidates[*first]
                    ));
                    break;
                }
                resolved = Some((i, bytes));
            }
            Err(SourceError::NotFound { .. }) => {}
            Err(e) => return Err(err(e)),
        }
    }
    Ok(resolved.map(|(_, bytes)| bytes))
}

/// FNV-1a over a byte slice — the sidecar-content fingerprint folded into the envelope cache key
/// (the resolved-vehicle hash covers the YAML spec only, not the binary sidecar bytes).
fn fnv1a(seed: u64, bytes: &[u8]) -> u64 {
    let mut h = seed ^ 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Decode and install the vehicle's declared binary sidecars onto an assembled [`T1Vehicle`] (the
/// native-edge step the wasm-clean core cannot do): the ride-height/yaw aero map (`aero.map`) and
/// each drive unit's `.ptm` efficiency/loss tables. A *missing* sidecar file is skipped with a note
/// (the constant-aero / peak-envelope fallbacks carry the lap — the status quo for vehicles whose
/// tables are not committed); a present-but-undecodable one is a real error (nothing silent).
///
/// Returns a fingerprint of every loaded sidecar's bytes (and each skip), folded into the envelope
/// cache key: two spec-identical cars with different (or differently-present) sidecar tables must
/// never share a cached envelope.
fn install_sidecars(
    t1v: &mut T1Vehicle,
    resolved: &ResolvedVehicle,
    vl: &FsLoader,
    notes: &mut Vec<String>,
) -> PyResult<u64> {
    use outlap_schema::io::SourceLoader;
    use outlap_schema::sidecar::read_gridded_table;

    let mut fp: u64 = 0;

    // Ride-height/yaw aero map.
    let map_path = resolved.spec.aero.map.as_str();
    if !map_path.is_empty() {
        match vl.load_bytes(map_path) {
            Ok(bytes) => {
                fp = fnv1a(fp, &bytes);
                let axes: Vec<&str> = resolved.spec.aero.axes.iter().map(String::as_str).collect();
                let table = read_gridded_table(&bytes, &axes).map_err(err)?;
                t1v.install_aero_map(&table, &resolved.spec.aero.axes)
                    .map_err(err)?;
            }
            Err(outlap_schema::io::SourceError::NotFound { .. }) => {
                fp = fnv1a(fp, b"aero:absent");
                notes.push(format!(
                    "aero map `{map_path}` not present — constant-aero fallback carries the lap"
                ));
            }
            Err(e) => return Err(err(e)),
        }
    }

    // Per-unit `.ptm` efficiency/loss tables (energy accounting + the Vdc–SoC coupling). The
    // sidecar resolves next to its `.ptm` first, then from the vehicle root (importer idiom).
    for (idx, unit) in resolved.spec.drivetrain.units.iter().enumerate() {
        let Ok(ptm) = outlap_schema::load::load_ptm(unit.source.as_str(), vl) else {
            continue; // assembly already validated/reported the source itself
        };
        let table_path = ptm.tables.file.as_str();
        if let Some(bytes) = load_sidecar_bytes(vl, unit.source.as_str(), table_path, notes)? {
            fp = fnv1a(fp, &bytes);
            let table = if ptm.axes.vdc_v.is_some() {
                read_gridded_table(&bytes, &outlap_qss::T1Powertrain::map_axis_names_vdc())
            } else {
                read_gridded_table(&bytes, &outlap_qss::T1Powertrain::map_axis_names())
            }
            .map_err(err)?;
            t1v.install_powertrain_maps(idx, &table).map_err(err)?;
        } else {
            fp = fnv1a(fp, b"ptm:absent");
            notes.push(format!(
                "powertrain tables `{table_path}` (unit {idx}) not present — peak-envelope \
                 traction only, no energy accounting"
            ));
        }
    }
    Ok(fp)
}

/// Assemble the slow-state stack (battery pack + machine thermal network) from the vehicle's own
/// references: `battery.params` plus the first drive unit carrying a `thermal:` `.emotor` ref. The
/// same missing-sidecar policy as [`install_sidecars`] applies — a vehicle whose stack files are
/// not present (e.g. `f1_2026`'s uncommitted `battery/f1_es.yaml`) keeps the coupling inert with a
/// note, while a present-but-broken file is a real error (nothing silent). Mass-heuristic fills in
/// the thermal assembly are surfaced as notes.
///
/// Returns the owned parts; the [`SlowCoupling`] itself borrows the `T1Vehicle` at the call site.
/// Load the vehicle's battery pack (document + ECM sidecar) into a runnable [`Pack`]. `None` when the
/// car declares no battery, or when its stack files are not present (a note says which — nothing
/// silent); a present-but-broken file is a real error. Shared by the QSS slow coupling and the T2
/// slow-state stack, so both see the same charge-acceptance model.
fn load_pack(
    resolved: &ResolvedVehicle,
    vl: &FsLoader,
    notes: &mut Vec<String>,
) -> PyResult<Option<(Pack, PackState)>> {
    use outlap_schema::sidecar::read_gridded_table;

    let Some(batt) = &resolved.spec.battery else {
        return Ok(None); // no battery block ⇒ single-voltage evaluation (PR6 coupling rule)
    };
    let params_path = batt.params.as_str();
    let doc = match outlap_schema::load::load_battery(params_path, vl) {
        Ok(doc) => doc,
        Err(e) if is_not_found(&e) => {
            notes.push(format!(
                "battery params `{params_path}` not present — slow-state coupling inert"
            ));
            return Ok(None);
        }
        Err(e) => return Err(schema_err(e)),
    };
    // The ECM sidecar resolves next to the battery YAML first, then from the vehicle root
    // (importer idiom — `pdt_h5 batterypack` writes the parquet beside its YAML).
    let ecm_path = doc.ecm.tables.file.as_str();
    let Some(ecm_bytes) = load_sidecar_bytes(vl, params_path, ecm_path, notes)? else {
        notes.push(format!(
            "battery ECM tables `{ecm_path}` not present — slow-state coupling inert"
        ));
        return Ok(None);
    };
    let ecm = read_gridded_table(&ecm_bytes, &Pack::ecm_axis_names()).map_err(err)?;
    let (pack, state) = Pack::assemble(&doc, &ecm, None).map_err(err)?;
    notes.extend(pack.notes().iter().cloned());
    Ok(Some((pack, state)))
}

fn build_slow_stack(
    resolved: &ResolvedVehicle,
    vl: &FsLoader,
    conditions: &Conditions,
    notes: &mut Vec<String>,
) -> PyResult<Option<(MachineThermal, Pack, PackState)>> {
    let Some((pack, pack_state)) = load_pack(resolved, vl, notes)? else {
        return Ok(None);
    };
    // The first drive unit with a `.emotor` thermal ref carries the machine slow state (the QSS
    // coupling marches ONE thermal network this milestone; multi-machine stacks arrive with the
    // ERS energy manager). Extra thermal declarations are dropped WITH a note (nothing silent).
    let thermal_units: Vec<usize> = resolved
        .spec
        .drivetrain
        .units
        .iter()
        .enumerate()
        .filter_map(|(i, u)| u.thermal.as_ref().map(|_| i))
        .collect();
    let Some(&unit_idx) = thermal_units.first() else {
        notes.push(
            "battery present but no drive unit declares a `.emotor` thermal model — slow-state \
             coupling inert"
                .to_owned(),
        );
        return Ok(None);
    };
    if thermal_units.len() > 1 {
        notes.push(format!(
            "{} drive units declare `.emotor` thermal models — the QSS coupling marches ONE \
             network (unit {unit_idx}); the aggregate powertrain loss heats it and the others \
             are not integrated this milestone",
            thermal_units.len()
        ));
    }
    let unit = &resolved.spec.drivetrain.units[unit_idx];
    let emotor_path = unit.thermal.as_ref().expect("filtered on thermal").as_str();
    let em = match outlap_schema::load::load_emotor(emotor_path, vl) {
        Ok(em) => em,
        Err(e) if is_not_found(&e) => {
            notes.push(format!(
                "machine thermal `{emotor_path}` not present — slow-state coupling inert"
            ));
            return Ok(None);
        }
        Err(e) => return Err(schema_err(e)),
    };
    let ptm = match outlap_schema::load::load_ptm(unit.source.as_str(), vl) {
        Ok(ptm) => ptm,
        // Unreachable in practice (T1 assembly hard-errors on a broken/missing unit source
        // first), but keep the policy symmetric with the battery/emotor refs above.
        Err(e) if is_not_found(&e) => {
            notes.push(format!(
                "drive-unit source `{}` not present — slow-state coupling inert",
                unit.source.as_str()
            ));
            return Ok(None);
        }
        Err(e) => return Err(schema_err(e)),
    };
    let thermal = MachineThermal::assemble(&em, conditions, ptm.mass_kg).map_err(err)?;
    notes.extend(
        thermal
            .estimates()
            .iter()
            .map(|e| format!("machine thermal: {e}")),
    );
    Ok(Some((thermal, pack, pack_state)))
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
    sidecar_fp: u64,
    conditions: &Conditions,
) -> PyResult<GgvEnvelope> {
    let e = &sim_cfg.envelope;
    let coupling = sim_cfg.resolved_fz_coupling();
    let cond = serde_json::to_string(conditions).map_err(err)?;
    let key = format!(
        "{resolved_hash}|{sidecar_fp:016x}|{cond}|{}x{}x{}|{:?}",
        e.v_points, e.ax_points, e.g_normal_points, coupling
    );
    if let Some(env) = ENV_CACHE.lock().expect("env cache mutex").get(&key) {
        return Ok(env.clone());
    }
    let env = GgvEnvelope::generate(t1v, e, coupling).map_err(err)?;
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
/// per-station re-trim for per-wheel loads/slips/forces + setup metrics. `t2` is the transient tier
/// and is time-indexed, so it has its own entry point ([`solve_transient_lap`]); `t3` raises (M6).
/// When `track` is a generated racing line, pass `raceline_ds_m` for honest provenance.
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
        // The transient tier is time-indexed, not station-indexed, so it returns a different
        // artifact. Point the caller at it rather than at a bare "not implemented".
        Tier::T2 => {
            return Err(PyValueError::new_err(
                "the transient tier (t2) produces a time-indexed lap: call \
                 `outlap.solve_transient_lap(...)`, or `outlap.solve_lap_dataset(..., tier=\"t2\")` \
                 for an xarray view",
            ))
        }
        tier @ Tier::T3 => return Err(err(tier_not_implemented(tier))),
        wanted => {
            let mut t1v = T1Vehicle::assemble(&resolved, &conditions, &vl, sim_cfg.allow_degraded)
                .map_err(err)?;
            let mut notes = Vec::new();
            // Native-edge sidecar decode: the aero map + `.ptm` tables (skipped with a note when
            // the files are not present).
            let sidecar_fp = install_sidecars(&mut t1v, &resolved, &vl, &mut notes)?;
            let env = cached_envelope(&t1v, &sim_cfg, &hash, sidecar_fp, &conditions)?;
            let t0v = T0Vehicle::assemble(&resolved, &conditions, &vl, &opts).map_err(err)?;
            notes.extend(t0v.notes().iter().cloned());
            notes.extend(t1v.notes().iter().cloned());
            notes.extend(env.notes().iter().cloned());
            // Slow-state coupling from the vehicle's own battery + `.emotor` refs (inert with a
            // note when the stack files are not present — the status quo for `f1_2026`).
            let stack = build_slow_stack(&resolved, &vl, &conditions, &mut notes)?;
            let coupling = stack.as_ref().map(|(thermal, pack, state)| SlowCoupling {
                vehicle: &t1v,
                thermal: thermal.clone(),
                pack: pack.clone(),
                pack_state: *state,
            });
            if wanted == Tier::T0 {
                solve_t0(
                    &t0v,
                    env,
                    coupling.as_ref(),
                    &path,
                    line,
                    hash,
                    notes,
                    sim_cfg.resolved_fz_coupling(),
                    sim_cfg.flat_track,
                )
                .map_err(err)?
            } else {
                solve_t1(
                    &t1v,
                    &t0v,
                    env,
                    coupling.as_ref(),
                    &path,
                    line,
                    hash,
                    notes,
                    sim_cfg.resolved_fz_coupling(),
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

// ---------------------------------------------------------------------------------------------
// T2 transient tier (PR7): Track → LineTable, the pack-backed slow stack, and the time-indexed lap.
// ---------------------------------------------------------------------------------------------

/// Fraction of the QSS speed profile the transient driver tracks by default. The point-mass profile
/// spends the whole grip envelope longitudinally; a transient car with a real tyre needs a margin to
/// stay inside its combined-slip limit on a real circuit. Surfaced in the lap notes.
const DEFAULT_SPEED_MARGIN: f64 = 0.85;
/// Hard cap on transient steps, so a car that cannot complete the lap terminates instead of hanging.
const MAX_TRANSIENT_STEPS: usize = 2_000_000;

/// The battery pack as the transient solver's slow-state stack (Decision #6): it Coulomb-counts the
/// recovered regen energy on the decimated slow clock and publishes back the pack's
/// **charge-acceptance ceiling** at the current SoC *and temperature*, which caps the series regen
/// blend. Held here at the native edge, so the wasm-clean transient crate keeps no QSS dependency.
///
/// The pack sees the **net** electrical power (regen recovered − traction drawn): it charges under
/// braking and discharges under power, so the SoC moves both ways over a lap.
struct PackSlowStack {
    pack: Pack,
    state: PackState,
}

impl outlap_transient::SlowStack for PackSlowStack {
    fn on_slow_step(&mut self, dt_s: f64, net_charge_power_w: f64) {
        // `Pack::step_power` takes discharge-positive terminal power, so charging is negative and
        // discharging is positive: pass `-net` and both directions fall out. The step advances SoC
        // (Coulomb count), the RC overpotential, and the lumped temperature.
        let _ = self
            .pack
            .step_power(&mut self.state, -net_charge_power_w, dt_s);
    }
    fn regen_power_limit_w(&self) -> f64 {
        self.pack.regen_power_limit_w(&self.state)
    }
    fn soc(&self) -> f64 {
        self.state.soc
    }
    fn temp_c(&self) -> f64 {
        self.state.temp_k - 273.15
    }
}

/// Build the transient [`LineTable`] from the (possibly raceline-offset) track, the T0 path, and the
/// QSS speed profile the driver tracks.
///
/// The chassis and driver curvature both come from the T0 path's **own smoothed** `kappa_l`, so
/// `κ_ref` aligns with the `v_ref` the point-mass solver braked for; feeding the driver the raw line
/// curvature instead makes it try to corner harder than the profile ever planned. Grade, banking and
/// vertical curvature are zeroed under `flat_track`; the world trajectory always comes from the line.
fn line_from_track(
    line: &outlap_track::Track,
    path: &T0Path,
    v_ref: &[f64],
    flat: bool,
) -> PyResult<outlap_transient::LineTable<f64>> {
    let s = &path.s;
    let n = s.len();
    if v_ref.len() != n {
        return Err(PyValueError::new_err(format!(
            "speed profile has {} stations but the path has {n}",
            v_ref.len()
        )));
    }
    let mut samples = outlap_transient::LineSamples {
        s: s.clone(),
        kappa_h: path.kappa_l.clone(),
        grade: vec![0.0; n],
        banking: vec![0.0; n],
        kappa_v: vec![0.0; n],
        n_ref: vec![0.0; n],
        kappa_ref: path.kappa_l.clone(),
        v_ref: v_ref.to_vec(),
        x_ref: Vec::with_capacity(n),
        y_ref: Vec::with_capacity(n),
        z_ref: vec![0.0; n],
        lat_x: Vec::with_capacity(n),
        lat_y: Vec::with_capacity(n),
        lat_z: vec![0.0; n],
        closed: path.closed,
    };
    for (i, &si) in s.iter().enumerate() {
        let f = line.road_frame(si);
        samples.x_ref.push(f.origin[0]);
        samples.y_ref.push(f.origin[1]);
        samples.lat_x.push(f.lateral[0]);
        samples.lat_y.push(f.lateral[1]);
        if !flat {
            samples.z_ref[i] = f.origin[2];
            samples.lat_z[i] = f.lateral[2];
            samples.grade[i] = f.grade;
            samples.banking[i] = f.banking;
            samples.kappa_v[i] = f.kappa_v;
        }
    }
    outlap_transient::LineTable::new(&samples).map_err(err)
}

/// Index of the straightest station (min `|κ|`). A cold transient — zero relaxation, zero yaw,
/// running straight — seeded *at* a corner is unphysical, so the lap starts on a straight and the
/// closed line wraps `s` back through the start/finish.
fn straightest_station(kappa: &[f64]) -> usize {
    (0..kappa.len())
        .min_by(|&a, &b| {
            kappa[a]
                .abs()
                .partial_cmp(&kappa[b].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(0)
}

/// A solved **transient (T2)** lap: fixed-step, time-indexed channels, per-wheel `time × wheel`
/// arrays, and the rule-based control layer's telemetry (gear, shift torque scale, torque-vectoring
/// yaw moment, per-axle regen torque, pack SoC/temperature).
#[pyclass(frozen)]
pub struct TransientLap {
    t: Vec<f64>,
    s: Vec<f64>,
    n: Vec<f64>,
    psi_rel: Vec<f64>,
    vx: Vec<f64>,
    vy: Vec<f64>,
    yaw_rate: Vec<f64>,
    ax: Vec<f64>,
    ay: Vec<f64>,
    steer: Vec<f64>,
    throttle: Vec<f64>,
    brake: Vec<f64>,
    x: Vec<f64>,
    y: Vec<f64>,
    z: Vec<f64>,
    gear: Vec<f64>,
    torque_scale: Vec<f64>,
    yaw_moment_nm: Vec<f64>,
    regen_power_w: Vec<f64>,
    traction_power_w: Vec<f64>,
    regen_torque_front_nm: Vec<f64>,
    regen_torque_rear_nm: Vec<f64>,
    // Per-wheel channels, row-major `n × 4` (FL/FR/RL/RR).
    omega: Vec<f64>,
    vertical_load_n: Vec<f64>,
    slip_ratio: Vec<f64>,
    slip_angle_rad: Vec<f64>,
    force_long_n: Vec<f64>,
    force_lat_n: Vec<f64>,
    // Slow states; `None` when the car has no battery (or its files are absent).
    state_of_charge: Option<Vec<f64>>,
    pack_temp_c: Option<Vec<f64>>,
    /// Total lap time, s.
    #[pyo3(get)]
    lap_time_s: f64,
    /// The resolved solver tier (always `"t2"`).
    #[pyo3(get)]
    tier: String,
    /// The recorded normal-load coupling mode (`"one_step_lag"` / `"fixed_point"`).
    #[pyo3(get)]
    fz_coupling: String,
    /// Whether the lap ran in flat-track analysis mode.
    #[pyo3(get)]
    flat_track: bool,
    /// Resolved fixed step size, s.
    #[pyo3(get)]
    dt_s: f64,
    /// Resolved integrator order (Heun: 2, RK4: 4).
    #[pyo3(get)]
    integrator_order: u32,
    /// The fraction of the QSS speed profile the driver tracked.
    #[pyo3(get)]
    speed_margin: f64,
    /// Whether the car reached the end of the lap within the step budget.
    #[pyo3(get)]
    completed: bool,
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

/// Build a `n × 4` numpy array (row-major) from a flat per-wheel channel.
fn wheel_array_2d<'py>(py: Python<'py>, flat: &[f64]) -> Bound<'py, PyArray2<f64>> {
    let n = flat.len() / 4;
    numpy::ndarray::Array2::from_shape_vec((n, 4), flat.to_vec())
        .expect("n×4 per-wheel channel")
        .into_pyarray(py)
}

#[pymethods]
impl TransientLap {
    /// Time since the lap start, s (the primary index — a fixed `dt` grid).
    fn t<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.t.clone().into_pyarray(py)
    }

    /// Arc length along the reference line, m.
    fn s<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.s.clone().into_pyarray(py)
    }

    /// Lateral offset from the reference line (ISO 8855, `+` left), m.
    fn n<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.n.clone().into_pyarray(py)
    }

    /// Heading relative to the road tangent, rad.
    fn psi_rel<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.psi_rel.clone().into_pyarray(py)
    }

    /// Body-frame longitudinal velocity, m/s.
    fn vx<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.vx.clone().into_pyarray(py)
    }

    /// Body-frame lateral velocity (`+` left), m/s.
    fn vy<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.vy.clone().into_pyarray(py)
    }

    /// Yaw rate (`+` CCW), rad/s.
    fn yaw_rate<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.yaw_rate.clone().into_pyarray(py)
    }

    /// Body-frame longitudinal acceleration, m/s².
    fn ax<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.ax.clone().into_pyarray(py)
    }

    /// Body-frame lateral acceleration (`+` left), m/s².
    fn ay<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.ay.clone().into_pyarray(py)
    }

    /// Front road-wheel steer angle, rad.
    fn steer<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.steer.clone().into_pyarray(py)
    }

    /// Throttle demand, 0..1.
    fn throttle<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.throttle.clone().into_pyarray(py)
    }

    /// Brake demand, 0..1.
    fn brake<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.brake.clone().into_pyarray(py)
    }

    /// World trajectory x, m.
    fn x<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.x.clone().into_pyarray(py)
    }

    /// World trajectory y, m.
    fn y<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.y.clone().into_pyarray(py)
    }

    /// World trajectory z, m.
    fn z<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.z.clone().into_pyarray(py)
    }

    /// Engaged gear index (0-based).
    fn gear<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.gear.clone().into_pyarray(py)
    }

    /// Drive-torque scale, 0..1 (`< 1` through a gear shift's torque cut/ramp).
    fn torque_scale<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.torque_scale.clone().into_pyarray(py)
    }

    /// Torque-vectoring yaw moment actually realised, N·m (`+` CCW).
    fn yaw_moment_nm<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.yaw_moment_nm.clone().into_pyarray(py)
    }

    /// Recovered electrical regen power, summed over the driven axles, W (≥ 0).
    fn regen_power_w<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.regen_power_w.clone().into_pyarray(py)
    }

    /// Electrical traction power drawn from the pack, W (≥ 0). `regen_power_w − this` is the net pack
    /// charge power (negative under drive, positive under braking).
    fn traction_power_w<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.traction_power_w.clone().into_pyarray(py)
    }

    /// Front-axle machine braking torque, N·m (≥ 0); the calipers supplied the rest.
    fn regen_torque_front_nm<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.regen_torque_front_nm.clone().into_pyarray(py)
    }

    /// Rear-axle machine braking torque, N·m (≥ 0); the calipers supplied the rest.
    fn regen_torque_rear_nm<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.regen_torque_rear_nm.clone().into_pyarray(py)
    }

    /// Per-wheel angular speed, rad/s (`time × wheel`).
    fn omega<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        wheel_array_2d(py, &self.omega)
    }

    /// Per-wheel normal load `F_z`, N (`time × wheel`).
    fn vertical_load_n<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        wheel_array_2d(py, &self.vertical_load_n)
    }

    /// Per-wheel lagged longitudinal slip `κ` (`time × wheel`).
    fn slip_ratio<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        wheel_array_2d(py, &self.slip_ratio)
    }

    /// Per-wheel lagged slip angle `α`, rad (`time × wheel`).
    fn slip_angle_rad<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        wheel_array_2d(py, &self.slip_angle_rad)
    }

    /// Per-wheel wheel-frame longitudinal force `F_x`, N (`time × wheel`).
    fn force_long_n<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        wheel_array_2d(py, &self.force_long_n)
    }

    /// Per-wheel wheel-frame lateral force `F_y`, N (`time × wheel`).
    fn force_lat_n<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        wheel_array_2d(py, &self.force_lat_n)
    }

    /// Pack state of charge, 0..1 — `None` when the car carries no battery.
    fn state_of_charge<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.state_of_charge.clone().map(|v| v.into_pyarray(py))
    }
    /// Pack temperature, °C — `None` when the car carries no battery.
    fn pack_temp_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.pack_temp_c.clone().map(|v| v.into_pyarray(py))
    }
    /// The number of recorded steps.
    fn __len__(&self) -> usize {
        self.t.len()
    }
}

/// Solve a **transient (T2)** lap: the 7-DOF chassis + tyre relaxation closed loop, driven by the
/// ideal driver along the QSS speed profile.
///
/// The T2 tier is a *closed-loop* simulation, so it needs a reference to follow: the point-mass QSS
/// profile for this car on this line, scaled by `speed_margin`. The lap is seeded at the straightest
/// station (a cold transient dropped into a corner is unphysical) and runs one full lap of arc length.
///
/// Returns a time-indexed [`TransientLap`]. Use `outlap.transient_lap_dataset` for an xarray view.
#[pyfunction]
#[pyo3(signature = (vehicle_dir, track, ds_m = DEFAULT_DS_M, raceline_ds_m = None, overrides = None, conditions = None, sim = None, speed_margin = DEFAULT_SPEED_MARGIN, initial_soc = None))]
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn solve_transient_lap(
    vehicle_dir: &str,
    track: &Track,
    ds_m: f64,
    raceline_ds_m: Option<f64>,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
    speed_margin: f64,
    initial_soc: Option<f64>,
) -> PyResult<TransientLap> {
    check_ds(ds_m)?;
    if !(speed_margin > 0.0 && speed_margin <= 1.0) {
        return Err(PyValueError::new_err(format!(
            "speed_margin must lie in (0, 1]; got {speed_margin}"
        )));
    }
    if let Some(soc) = initial_soc {
        if !(0.0..=1.0).contains(&soc) {
            return Err(PyValueError::new_err(format!(
                "initial_soc must lie in [0, 1]; got {soc}"
            )));
        }
    }
    let (vl, resolved) = resolve_with_overrides(vehicle_dir, overrides)?;
    let sim_cfg = build_sim(&vl, sim, Some("t2"))?;
    let base_conditions = match load_conditions("conditions.yaml", &vl) {
        Ok(c) => c,
        Err(e) if is_not_found(&e) => Conditions::default(),
        Err(e) => return Err(schema_err(e)),
    };
    let conditions = match conditions {
        Some(patch) => merge_conditions(base_conditions, &py_to_json(patch.as_any())?)?,
        None => base_conditions,
    };
    let hash = resolved.report.resolved_hash.clone();
    let mut notes = Vec::new();

    // The transient tier honours `flat_track` like the QSS tiers (PR7.5): the 7-DOF chassis carries
    // grade/banking/vertical-curvature terms and the driver is stabilised against the grade
    // disturbance (grade-aware speed reference + grade-scaled yaw recovery, `outlap-vehicle`), so a
    // real elevated circuit no longer spins the car out. The default stays 3-D (`flat_track` unset ⇒
    // full geometry); pass `flat_track: true` for a flat-plane analysis lap.
    let flat = sim_cfg.flat_track;
    if flat {
        notes.push(
            "T2 flat-track analysis mode: the chassis grade/banking/vertical-curvature terms are \
             held at zero (the world trajectory is still reconstructed from the line)"
                .to_owned(),
        );
    } else {
        notes.push(
            "T2 3-D road frame: grade, banking and the full elevated trajectory are live. The \
             vertical-curvature normal-load term (κ_v·v²) is transmitted in full when it loads the \
             tyres (dips/compressions) but its *unloading* over a crest is floored at 0.15 g below \
             the static load (estimated) — a rigid T2 chassis has no suspension travel (T3) to \
             absorb a sharp crest, so the raw term would over-unload the tyres and spin the car \
             where a sprung car stays planted; full vertical-load fidelity awaits the T3 suspension"
                .to_owned(),
        );
    }

    // --- The QSS reference: the same envelope + point-mass profile the T0/T1 tiers solve. ---------
    let opts = T0Options {
        ds_m,
        allow_degraded: sim_cfg.allow_degraded,
        ..T0Options::default()
    };
    let path = if flat {
        T0Path::from_track_flat(&track.inner, ds_m)
    } else {
        T0Path::from_track(&track.inner, ds_m)
    };
    let line_descriptor = match raceline_ds_m {
        Some(g) => LineDescriptor::MinCurvature {
            ds_m: g,
            iterations: 1,
        },
        None => LineDescriptor::Centerline,
    };
    let mut t1v =
        T1Vehicle::assemble(&resolved, &conditions, &vl, sim_cfg.allow_degraded).map_err(err)?;
    let sidecar_fp = install_sidecars(&mut t1v, &resolved, &vl, &mut notes)?;
    let env = cached_envelope(&t1v, &sim_cfg, &hash, sidecar_fp, &conditions)?;
    let t0v = T0Vehicle::assemble(&resolved, &conditions, &vl, &opts).map_err(err)?;
    notes.extend(t0v.notes().iter().cloned());
    notes.extend(t1v.notes().iter().cloned());
    notes.extend(env.notes().iter().cloned());
    let reference = solve_t0(
        &t0v,
        env,
        None,
        &path,
        line_descriptor,
        hash.clone(),
        Vec::new(),
        sim_cfg.resolved_fz_coupling(),
        flat,
    )
    .map_err(err)?;

    // --- The T2 block set, through the shared assembly pipeline. ----------------------------------
    let pack = load_pack(&resolved, &vl, &mut notes)?;
    let mut interner = outlap_transient::ChannelInterner::new();
    let t2_opts = outlap_vehicle::T2Options {
        battery_present: pack.is_some(),
        ..outlap_vehicle::T2Options::default()
    };
    let blocks: outlap_transient::T2Blocks<f64> =
        outlap_vehicle::assemble_t2(&t1v, &resolved.spec, &mut interner, &t2_opts, &mut notes)
            .into();

    let v_target: Vec<f64> = reference.lap.v.iter().map(|v| v * speed_margin).collect();
    let line = line_from_track(&track.inner, &path, &v_target, flat)?;
    let start_i = straightest_station(&path.kappa_l);
    let start_s = path.s[start_i];
    let length = reference.lap.s.last().copied().unwrap_or(0.0);

    let cfg = outlap_transient::SimConfig {
        fz_coupling: sim_cfg.resolved_fz_coupling(),
        start_s,
        ..outlap_transient::SimConfig::default()
    };
    let mut solver = outlap_transient::TransientSolver::new(blocks, line, &interner, cfg);
    if let Some((pack, mut state)) = pack {
        // A pack seeded at the top of its SoC window accepts no charge, so it would recover nothing
        // however hard the car brakes. That is correct physics and a useless default for a lap, so a
        // stint starts mid-window unless the caller says otherwise — and the choice is surfaced.
        let [lo, hi] = pack.soc_window();
        state.soc = initial_soc.unwrap_or_else(|| {
            let mid = 0.5 * (lo + hi);
            notes.push(format!(
                "T2 pack seeded at {:.0}% state of charge, the middle of its usable window \
                 [{lo:.2}, {hi:.2}] (estimated — pass `initial_soc` to pick a stint state); a \
                 pack at the top of its window accepts no charge and recovers nothing",
                mid * 100.0
            ));
            mid
        });
        if state.soc >= hi {
            notes.push(format!(
                "T2 pack starts at or above the top of its SoC window ({:.2} ≥ {hi:.2}): it can \
                 accept no charge, so the friction brakes do all the braking and regen is 0",
                state.soc
            ));
        }
        notes.push(
            "T2 slow stack: the pack Coulomb-counts the net electrical energy (regen recovered minus \
             traction drawn) and publishes its charge-acceptance ceiling (SoC + temperature), so the \
             state of charge falls under power and rises under braking. The traction draw is not \
             capped by the pack's discharge ceiling at this tier (the envelope, not the pack, limits \
             drive power)"
                .to_owned(),
        );
        solver = solver.with_slow_stack(Box::new(PackSlowStack { pack, state }));
    }
    notes.push(format!(
        "T2 driver tracks {:.0}% of the QSS speed profile (the point-mass profile spends the whole \
         grip envelope longitudinally); the lap is seeded at the straightest station, s = {start_s:.1} m",
        speed_margin * 100.0
    ));
    // Attach the gear-shift FSM: the crossover speeds where the best gear changes, and the gearbox's
    // own shift time. A single-speed (direct-drive) car has no up-shift speeds, so the FSM is a no-op
    // and the lap runs the best-gear envelope uninterrupted. The traction ceiling stays the best-gear
    // envelope either way; the FSM only adds the torque-cut interruption at each shift.
    let upshift_speeds = t1v.upshift_speeds();
    let gear_count = t1v.gear_count();
    let shift_time_s = t1v.shift_time_s();
    if upshift_speeds.is_empty() || shift_time_s <= 0.0 {
        notes.push(
            "T2 gear-shift FSM inert: the car is single-speed (direct drive) or declares no shift \
             time, so the lap runs the best-gear envelope with no torque interruption"
                .to_owned(),
        );
    } else {
        notes.push(format!(
            "T2 gear-shift FSM: {gear_count} gears, {shift_time_s:.3} s shift, up-shift speeds \
             {}; each shift costs the §8.2 torque interruption (the best-gear traction ceiling is \
             unchanged — the gear indexes no force in v1)",
            upshift_speeds
                .iter()
                .map(|v| format!("{v:.1}"))
                .collect::<Vec<_>>()
                .join("/")
        ));
        solver = solver.with_shifter(outlap_transient::Shifter::new(
            gear_count,
            upshift_speeds,
            shift_time_s,
        ));
    }

    let lap = solver.run(start_s + length, MAX_TRANSIENT_STEPS);
    let diverged = solver.diverged();
    let provenance = solver.provenance();
    // `run` breaks the moment the recorded arc length passes the finish line, so the last sample
    // tells us whether the car got there inside the step budget.
    let completed = lap.s.last().copied().unwrap_or(0.0) >= start_s + length;
    if diverged {
        notes.push(
            "T2 lap diverged: the closed loop left the physical envelope (a spin the driver could \
             not catch) and the run stopped early. The trace is truncated and `lap_time_s` is not a \
             lap time. Try a lower `speed_margin`"
                .to_owned(),
        );
    } else if !completed {
        notes.push(format!(
            "T2 lap did not reach the finish line within {MAX_TRANSIENT_STEPS} steps — the trace is \
             truncated and `lap_time_s` is not a lap time"
        ));
    }

    let has_slow = !lap.state_of_charge.is_empty();
    Ok(TransientLap {
        t: lap.t,
        s: lap.s,
        n: lap.n,
        psi_rel: lap.psi_rel,
        vx: lap.vx,
        vy: lap.vy,
        yaw_rate: lap.yaw_rate,
        ax: lap.ax,
        ay: lap.ay,
        steer: lap.steer,
        throttle: lap.throttle,
        brake: lap.brake,
        x: lap.x,
        y: lap.y,
        z: lap.z,
        gear: lap.gear,
        torque_scale: lap.torque_scale,
        yaw_moment_nm: lap.yaw_moment_nm,
        regen_power_w: lap.regen_power_w,
        traction_power_w: lap.traction_power_w,
        regen_torque_front_nm: lap.regen_torque_front_nm,
        regen_torque_rear_nm: lap.regen_torque_rear_nm,
        omega: flat4(&lap.omega),
        vertical_load_n: flat4(&lap.fz),
        slip_ratio: flat4(&lap.slip_kappa),
        slip_angle_rad: flat4(&lap.slip_alpha),
        force_long_n: flat4(&lap.fx),
        force_lat_n: flat4(&lap.fy),
        state_of_charge: has_slow.then(|| lap.state_of_charge.clone()),
        pack_temp_c: has_slow.then(|| lap.pack_temp_c.clone()),
        lap_time_s: lap.lap_time_s,
        tier: "t2".to_owned(),
        fz_coupling: fz_coupling_str(provenance.fz_coupling),
        flat_track: flat,
        dt_s: provenance.dt_s,
        integrator_order: provenance.integrator_order,
        speed_margin,
        completed,
        wheels: WHEEL_ORDER.iter().map(|s| (*s).to_owned()).collect(),
        notes,
        resolved_hash: hash,
    })
}

/// The `outlap_core` extension module.
#[pymodule]
fn outlap_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Tyre>()?;
    m.add_class::<Track>()?;
    m.add_class::<Raceline>()?;
    m.add_class::<Lap>()?;
    m.add_class::<TransientLap>()?;
    m.add_class::<Envelope>()?;
    m.add_function(wrap_pyfunction!(min_curvature, m)?)?;
    m.add_function(wrap_pyfunction!(solve_lap, m)?)?;
    m.add_function(wrap_pyfunction!(solve_transient_lap, m)?)?;
    m.add_function(wrap_pyfunction!(vehicle_report, m)?)?;
    m.add("DEFAULT_DS_M", DEFAULT_DS_M)?;
    Ok(())
}
