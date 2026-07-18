// SPDX-License-Identifier: AGPL-3.0-only
//! Input/geometry PyO3 classes: `Tyre`, `Track`, `Raceline`, `Envelope` + raceline generators.

use crate::prelude::*;

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
    pub(crate) inner: outlap_track::Track,
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

/// A generated racing line (min-curvature or time-weighted).
#[pyclass(frozen)]
pub struct Raceline {
    s: Vec<f64>,
    n: Vec<f64>,
    line: Py<Track>,
    /// The sampling step the line was GENERATED with, m (recorded into lap provenance).
    #[pyo3(get)]
    ds_m: f64,
    /// Which generator produced this line: `"min_curvature"` or `"time_weighted"`.
    #[pyo3(get)]
    generator: String,
    /// Outer iterations actually run (1 for min-curvature; the converged count for time-weighted).
    #[pyo3(get)]
    iterations: usize,
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
pub(crate) fn min_curvature(
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
        generator: "min_curvature".to_owned(),
        iterations: 1,
    })
}

/// Generate the **time-weighted** racing line (Decision #10): the min-curvature QP re-solved with
/// per-station weights `wᵢ = 1/vᵢ` (∝ Δt on the uniform grid) from a T0/GGV speed pre-pass on the
/// current line, in an outer reweight loop that keeps the fastest line and stops when the modelled
/// lap time stops improving (or after `iterations`).
///
/// Unlike [`min_curvature`], this needs the car — the speed pre-pass runs the vehicle's own g-g-g-v
/// envelope — so it takes `vehicle_dir` and honours `sim`/`overrides`/`conditions` exactly as the
/// solver does (`sim.flat_track` picks the flat vs 3-D speed model). The envelope is built once and
/// reused across iterations.
#[pyfunction]
#[pyo3(signature = (vehicle_dir, track, half_width_m, ds_m = 2.0, iterations = 3, margin_m = 0.3, epsilon = 1e-8, tol = 1e-3, overrides = None, conditions = None, sim = None))]
#[allow(
    clippy::too_many_arguments,
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
pub(crate) fn time_weighted(
    py: Python<'_>,
    vehicle_dir: &str,
    track: &Track,
    half_width_m: f64,
    ds_m: f64,
    iterations: usize,
    margin_m: f64,
    epsilon: f64,
    tol: f64,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
) -> PyResult<Raceline> {
    check_ds(ds_m)?;
    if !(half_width_m > 0.0 && half_width_m.is_finite()) {
        return Err(PyValueError::new_err(format!(
            "half_width_m must be a positive, finite number of metres, got {half_width_m}"
        )));
    }
    if !(1..=16).contains(&iterations) {
        return Err(PyValueError::new_err(format!(
            "iterations must be in 1..=16 (2-4 is typical), got {iterations}"
        )));
    }
    let opts = RacelineOptions {
        ds_m,
        margin_m,
        epsilon,
    };

    // --- assemble the speed pre-pass once (the envelope is line-independent) --------------------
    // Sim FIRST: its `allow_degraded` feeds the load pipeline (the ers↔battery integrity checks).
    let sim_cfg = build_sim(&FsLoader::new(vehicle_dir), sim, None)?;
    let (vl, resolved) =
        resolve_with_overrides_opts(vehicle_dir, overrides, sim_cfg.allow_degraded)?;
    let base_conditions = match load_conditions("conditions.yaml", &vl) {
        Ok(c) => c,
        Err(e) if is_not_found(&e) => Conditions::default(),
        Err(e) => return Err(schema_err(e)),
    };
    let conditions = match conditions {
        Some(patch) => merge_conditions(base_conditions, &py_to_json(patch.as_any())?)?,
        None => base_conditions,
    };
    let t0_opts = T0Options {
        ds_m,
        allow_degraded: sim_cfg.allow_degraded,
        ..T0Options::default()
    };
    let mut t1v =
        T1Vehicle::assemble(&resolved, &conditions, &vl, sim_cfg.allow_degraded).map_err(err)?;
    let mut asm_notes = Vec::new();
    let sidecar_fp = install_sidecars(&mut t1v, &resolved, &vl, &mut asm_notes)?;
    let hash = resolved.report.resolved_hash.clone();
    let env = cached_envelope(&t1v, &sim_cfg, &hash, sidecar_fp, &conditions)?;
    let t0v = T0Vehicle::assemble(&resolved, &conditions, &vl, &t0_opts).map_err(err)?;
    let flat = sim_cfg.flat_track;
    let fzc = sim_cfg.resolved_fz_coupling();

    // A T0/GGV speed pre-pass on `line`, returning its per-station (s, v) and modelled lap time.
    let prepass = |line: &outlap_track::Track| -> PyResult<(Vec<f64>, Vec<f64>, f64)> {
        let path = if flat {
            T0Path::from_track_flat(line, ds_m)
        } else {
            T0Path::from_track(line, ds_m)
        };
        let lap = solve_t0(
            &t0v,
            env.clone(),
            &Couplings::default(),
            &path,
            LapRequest {
                line: LineDescriptor::Centerline,
                resolved_hash: hash.clone(),
                notes: Vec::new(),
                fz_coupling: fzc,
                flat_track: flat,
            },
        )
        .map_err(err)?
        .lap;
        Ok((lap.s, lap.v, lap.lap_time_s))
    };

    // Resample a pre-pass speed profile onto the QP's `n` stations by fractional lap position, and
    // return the weights wᵢ = 1/vᵢ (∝ Δt on the uniform grid).
    let stations = raceline_stations(&track.inner, ds_m);
    let weights_from = |v: &[f64]| -> Vec<f64> {
        let p = v.len().max(1);
        let n = stations.len();
        (0..n)
            .map(|i| {
                let f = i as f64 / n as f64;
                let idx = f * p as f64;
                let lo = (idx.floor() as usize).min(p - 1);
                let hi = (lo + 1).min(p - 1);
                let frac = idx - lo as f64;
                let vi = v[lo] * (1.0 - frac) + v[hi] * frac;
                1.0 / vi.max(1.0) // v floor: 1 m/s guards a degenerate station
            })
            .collect()
    };

    // --- outer reweight loop: keep the fastest line, stop on lap-time convergence ----------------
    let mut best = min_curvature_line(&track.inner, half_width_m, &opts).map_err(err)?;
    let (_, mut best_v, mut best_time) = prepass(&best.line)?;
    let mut ran = 1usize;
    for _ in 1..iterations {
        let w = weights_from(&best_v);
        let cand =
            min_curvature_line_weighted(&track.inner, half_width_m, &w, &opts).map_err(err)?;
        let (_, cand_v, cand_time) = prepass(&cand.line)?;
        ran += 1;
        if cand_time < best_time - tol * best_time {
            best = cand;
            best_v = cand_v;
            best_time = cand_time;
        } else {
            break; // converged (no meaningful improvement)
        }
    }

    Ok(Raceline {
        s: best.s,
        n: best.n,
        line: Py::new(py, Track { inner: best.line })?,
        ds_m,
        generator: "time_weighted".to_owned(),
        iterations: ran,
    })
}

/// A queryable g-g-g-v envelope (the returnable `lap.envelope`): the T1-derived tyre-grip boundary
/// the QSS lap ran on. Zero-copy scalar queries; `to_dataset` is built on the Python side.
#[pyclass(frozen)]
pub struct Envelope {
    pub(crate) inner: GgvEnvelope,
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
