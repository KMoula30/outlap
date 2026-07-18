// SPDX-License-Identifier: AGPL-3.0-only
//! QSS (T0/T1) Python entry points and their result classes (`Lap`, `QssStint`).

use crate::prelude::*;

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
    // ERS energy-manager channels (per station, ELECTRICAL W, realized); `None` unless the
    // 2026 energy manager governed the march (M6 PR2).
    deploy_power_w: Option<Vec<f64>>,
    harvest_power_w: Option<Vec<f64>>,
    // Tyre-thermal slow-state channels (per station, the representative front tyre); `None` unless
    // the tyre-thermal march was opted in (`tire_thermal=True`).
    tire_surface_c: Option<Vec<f64>>,
    tire_carcass_c: Option<Vec<f64>>,
    tire_gas_c: Option<Vec<f64>>,
    tire_wear_mm: Option<Vec<f64>>,
    tire_damage: Option<Vec<f64>>,
    tire_grip: Option<Vec<f64>>,
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
    /// Machine winding temperature per station (°C), or `None` when no coupled stack was active
    /// (or the pack marches without a machine-thermal network).
    fn machine_temp_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.machine_temp_c
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Realized electrical ERS deployment power per station (W, CU-K DC bus), or `None` when the
    /// 2026 energy manager did not govern the lap.
    fn deploy_power_w<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.deploy_power_w
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Realized electrical ERS harvest power per station (W, all Recharge paths), or `None` when
    /// the 2026 energy manager did not govern the lap.
    fn harvest_power_w<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.harvest_power_w
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Representative front-tyre tread-surface temperature per station (°C), or `None` unless the
    /// tyre-thermal march was opted in (`tire_thermal=True`).
    fn tire_surface_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.tire_surface_c
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Representative front-tyre carcass (bulk) temperature per station (°C), or `None`.
    fn tire_carcass_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.tire_carcass_c
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Representative front-tyre inflation-gas temperature per station (°C), or `None`.
    fn tire_gas_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.tire_gas_c.as_ref().map(|v| v.clone().into_pyarray(py))
    }
    /// Representative front-tyre tread wear per station (mm, monotone), or `None`.
    fn tire_wear_mm<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.tire_wear_mm
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Representative front-tyre irreversible thermal damage per station (0..1), or `None`.
    fn tire_damage<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.tire_damage
            .as_ref()
            .map(|v| v.clone().into_pyarray(py))
    }
    /// Representative front-tyre total grip multiplier `λ_μ,total` per station, or `None`.
    fn tire_grip<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.tire_grip.as_ref().map(|v| v.clone().into_pyarray(py))
    }
    /// The g-g-g-v envelope this lap ran on (queryable), or `None` for the degenerate path.
    #[getter]
    fn envelope(&self) -> Option<Envelope> {
        self.envelope.clone().map(|inner| Envelope { inner })
    }
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
#[pyo3(signature = (vehicle_dir, track, ds_m = DEFAULT_DS_M, raceline_ds_m = None, raceline_generator = None, raceline_iterations = None, overrides = None, conditions = None, tier = None, sim = None, tire_thermal = false))]
#[allow(clippy::too_many_arguments)]
pub(crate) fn solve_lap(
    vehicle_dir: &str,
    track: &Track,
    ds_m: f64,
    raceline_ds_m: Option<f64>,
    raceline_generator: Option<&str>,
    raceline_iterations: Option<usize>,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    tier: Option<&str>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
    tire_thermal: bool,
) -> PyResult<Lap> {
    check_ds(ds_m)?;
    // Sim FIRST: its `allow_degraded` feeds the load pipeline (the ers↔battery integrity checks).
    let sim_cfg = build_sim(&FsLoader::new(vehicle_dir), sim, tier)?;
    let (vl, resolved) =
        resolve_with_overrides_opts(vehicle_dir, overrides, sim_cfg.allow_degraded)?;
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
    let line = line_descriptor(raceline_ds_m, raceline_generator, raceline_iterations);
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
            // With the tyre-thermal march opted in the envelope needs the (T_tire, wear) axes (a
            // full re-solve across the axis product — not cached; see PR4's recorded build cost).
            // Otherwise the cheap frozen envelope (bit-identical to pre-M5).
            let env = if tire_thermal {
                let coupling = sim_cfg.resolved_fz_coupling();
                GgvEnvelope::generate_with_tire_state(
                    &t1v,
                    &sim_cfg.envelope,
                    coupling,
                    TireStateRes::default(),
                )
                .map_err(err)?
            } else {
                cached_envelope(&t1v, &sim_cfg, &hash, sidecar_fp, &conditions)?
            };
            let t0v = T0Vehicle::assemble(&resolved, &conditions, &vl, &opts).map_err(err)?;
            notes.extend(t0v.notes().iter().cloned());
            notes.extend(t1v.notes().iter().cloned());
            notes.extend(env.notes().iter().cloned());
            // Slow-state coupling from the vehicle's own battery (+ optional `.emotor`) refs
            // (inert with a note when the stack files are not present).
            let stack = build_slow_stack(&resolved, &vl, &conditions, &mut notes)?;
            let coupling = stack.as_ref().map(|(thermal, pack, state)| SlowCoupling {
                vehicle: &t1v,
                thermal: thermal.clone(),
                pack: pack.clone(),
                pack_state: *state,
                active: t1v.has_energy_maps(),
            });
            // The 2026 ERS energy manager (M6 PR2): governs the march whenever the car has an
            // `ers:` block AND a pack to schedule; without a pack the greedy budget-free curve
            // still shapes the pedal availability, recorded as such.
            let ers_coupling = build_ers_coupling(
                &resolved,
                &t0v,
                coupling.is_some(),
                sim_cfg.allow_degraded,
                &mut notes,
            )?;
            // Tyre-thermal march (M5 PR5): opt-in, so the default lap stays bit-identical to pre-M5
            // (the synthetic .tyr params are pre-calibration — the default flips on at PR8).
            let tire_march = if tire_thermal {
                Some(build_tire_march(&t1v, &resolved, &conditions, &vl, &mut notes)?)
            } else {
                None
            };
            let couplings = Couplings {
                electro: coupling.as_ref(),
                tire: tire_march.as_ref(),
                ers: ers_coupling.as_ref(),
            };
            let req = LapRequest {
                line,
                resolved_hash: hash,
                notes,
                fz_coupling: sim_cfg.resolved_fz_coupling(),
                flat_track: sim_cfg.flat_track,
            };
            if wanted == Tier::T0 {
                solve_t0(&t0v, env, &couplings, &path, req).map_err(err)?
            } else {
                solve_t1(&t1v, &t0v, env, &couplings, &path, req).map_err(err)?
            }
        }
    };

    Ok(qss_lap_to_py(qss, track))
}

/// Build the QSS energy-manager coupling for an `ers:`-bearing vehicle (M6 PR2). `None` when the
/// car has no `ers:` block.
///
/// The load pipeline already hard-errors (unless `allow_degraded`) when an `ers:` car's battery
/// YAML is absent, so reaching here with `!pack_present` means the YAML loaded but the pack could
/// not be BUILT — a missing/broken ECM sidecar. That is the same missing-energy-store contract
/// violation, so it is a hard error too unless `allow_degraded` (the ONLY fallback path, which
/// then marks the run and runs the budget-free curve).
pub(crate) fn build_ers_coupling(
    resolved: &ResolvedVehicle,
    t0v: &T0Vehicle,
    pack_present: bool,
    allow_degraded: bool,
    notes: &mut Vec<String>,
) -> PyResult<Option<ErsCoupling>> {
    if resolved.spec.ers.is_none() {
        return Ok(None);
    }
    if !pack_present {
        if !allow_degraded {
            return Err(PyValueError::new_err(
                "an `ers:` vehicle's battery pack could not be built (its ECM sidecar is missing \
                 or unreadable) — the energy manager schedules the pack. Provide the battery ECM \
                 parquet sidecar, or set `allow_degraded: true` in sim.yaml to run with an inert \
                 (budget-free, harvest-less) ERS",
            ));
        }
        notes.push(
            "ERS present but no runnable battery pack — the energy manager is inert: deployment \
             follows the budget-free regulation curve and nothing is harvested (degraded path)"
                .to_owned(),
        );
        return Ok(None);
    }
    let coupling = ErsCoupling::assemble(
        &resolved.spec,
        t0v,
        outlap_qss::ers::ErsPolicy::RuleBased,
        false,
    )
    .map_err(err)?;
    Ok(coupling)
}

/// Convert a solved [`QssLap`] into the Python `Lap`, reconstructing world positions from the track.
pub(crate) fn qss_lap_to_py(qss: QssLap, track: &Track) -> Lap {
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
    let tire: Option<&TireSlowLog> = qss.tire.as_ref();
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
        machine_temp_c: slow.and_then(|s| s.machine_temp_c.clone()),
        deploy_power_w: slow.and_then(|s| s.ers.as_ref().map(|e| e.deploy_power_w.clone())),
        harvest_power_w: slow.and_then(|s| s.ers.as_ref().map(|e| e.harvest_power_w.clone())),
        tire_surface_c: tire.map(|t| t.surface_temp_c.clone()),
        tire_carcass_c: tire.map(|t| t.carcass_temp_c.clone()),
        tire_gas_c: tire.map(|t| t.gas_temp_c.clone()),
        tire_wear_mm: tire.map(|t| t.wear_mm.clone()),
        tire_damage: tire.map(|t| t.damage.clone()),
        tire_grip: tire.map(|t| t.grip_scale.clone()),
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

/// Load and resolve a vehicle, returning its loaded-model report as a dict:
/// `{name, resolved_hash, inherited, estimated, degraded, warnings, overrides}` (entry lists are
/// `(json_pointer, detail)` pairs). Nothing silent (Decision #41).
///
/// `overrides` is the same `{dotted.path: value}` what-if dict as [`solve_lap`]'s; the applied
/// paths are echoed back under the `overrides` key, and the `resolved_hash` reflects them.
#[pyfunction]
#[pyo3(signature = (vehicle_dir, overrides = None))]
pub(crate) fn vehicle_report<'py>(
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
    // The loaded-model report is the diagnostic surface: it must DESCRIBE a degraded car, not
    // hard-fail on it (the M6 PR2 ers↔battery integrity check would otherwise make an
    // ers-without-battery car un-inspectable). Load with `allow_degraded` so the missing-store
    // condition surfaces as a `degraded` report entry — exactly where a user should see it —
    // rather than as an exception (`estimated/inherited/degraded always surface in the report`).
    let (_vl, resolved) = resolve_with_overrides_opts(vehicle_dir, overrides, true)?;
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

/// A solved **QSS stint**: `n_laps` T0/T1 laps run back-to-back with the representative-tyre thermal
/// ring + wear **state carried across every lap boundary** (§6.1 segment-to-segment march, extended
/// across laps — lap N+1's march seeds from lap N's terminal `(T_s/T_c/T_g, wear, damage)`, no reset).
/// All laps share the arc-length station grid (the same line), so the stint is one clean
/// `(lap, station)` block; `lap_time_s` is the per-lap headline.
#[pyclass(frozen)]
pub struct QssStint {
    /// Number of laps run.
    #[pyo3(get)]
    n_laps: usize,
    /// Shared arc-length stations, m (n_stations).
    s: Vec<f64>,
    /// Per-lap lap time, s (n_laps).
    lap_time_s: Vec<f64>,
    /// Per-`(lap × station)` speed, m/s (row-major).
    v: Vec<f64>,
    // Per-`(lap × station)` representative-tyre channels; `None` when the tyre march was off.
    tire_surface_c: Option<Vec<f64>>,
    tire_carcass_c: Option<Vec<f64>>,
    tire_gas_c: Option<Vec<f64>>,
    tire_wear_mm: Option<Vec<f64>>,
    tire_damage: Option<Vec<f64>>,
    tire_grip: Option<Vec<f64>>,
    /// The resolved solver tier (`"t0"`/`"t1"`).
    #[pyo3(get)]
    tier: String,
    /// The recorded normal-load coupling mode.
    #[pyo3(get)]
    fz_coupling: String,
    /// Whether the stint ran in flat-track analysis mode.
    #[pyo3(get)]
    flat_track: bool,
    /// blake3 hash of the resolved vehicle spec.
    #[pyo3(get)]
    resolved_hash: String,
    /// Simplification/degradation notes (nothing silent).
    #[pyo3(get)]
    notes: Vec<String>,
}

impl QssStint {
    fn n_stations(&self) -> usize {
        self.s.len()
    }
}

#[pymethods]
impl QssStint {
    /// Shared arc-length stations, m.
    fn s<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.s.clone().into_pyarray(py)
    }
    /// Per-lap lap time, s (shape `n_laps`).
    fn lap_time_s<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.lap_time_s.clone().into_pyarray(py)
    }
    /// Speed, m/s (shape `n_laps × station`).
    fn v<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        array2d(py, &self.v, self.n_laps, self.n_stations())
    }
    /// Representative-tyre tread-surface temperature `T_s`, °C (`n_laps × station`), or `None`.
    fn tire_surface_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        let n = self.n_stations();
        self.tire_surface_c
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, n))
    }
    /// Representative-tyre carcass temperature `T_c`, °C (`n_laps × station`), or `None`.
    fn tire_carcass_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        let n = self.n_stations();
        self.tire_carcass_c
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, n))
    }
    /// Representative-tyre inflation-gas temperature `T_g`, °C (`n_laps × station`), or `None`.
    fn tire_gas_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        let n = self.n_stations();
        self.tire_gas_c
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, n))
    }
    /// Representative-tyre tread wear `w`, mm (`n_laps × station`, monotone), or `None`.
    fn tire_wear_mm<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        let n = self.n_stations();
        self.tire_wear_mm
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, n))
    }
    /// Representative-tyre irreversible thermal damage `D` (`n_laps × station`), or `None`.
    fn tire_damage<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        let n = self.n_stations();
        self.tire_damage
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, n))
    }
    /// Representative-tyre total grip multiplier `λ_μ,total` (`n_laps × station`), or `None`.
    fn tire_grip<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        let n = self.n_stations();
        self.tire_grip
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, n))
    }
    fn __len__(&self) -> usize {
        self.n_laps
    }
}

/// Solve a **QSS stint** — `n_laps` T0/T1 laps of `track`, carrying the tyre-thermal slow state across
/// each lap boundary so the tyres warm, wear, and degrade over the run (the T0/T1 tier's
/// stint-capability, §6.1). The tyre-state g-g-g-v envelope is built once and reused across laps; each
/// lap re-seeds the representative-tyre march from the previous lap's terminal state.
///
/// `tire_thermal` (default **on** — degradation is the point of a stint) drives the tyre-state axes;
/// with it off every lap is identical (a frozen-tyre stint). `initial_tire_temp_c` seeds the tyres
/// cold-uniform (the out-lap warm-up); the default seeds warm at the grip optimum. The electrified
/// slow stack (pack SoC / machine temperature), where present, is **not** carried lap-to-lap in the
/// QSS stint (it re-seeds each lap — recovery arrives with the ERS energy manager in M6); the tyre
/// state is what carries.
#[pyfunction]
#[pyo3(signature = (vehicle_dir, track, n_laps, ds_m = DEFAULT_DS_M, raceline_ds_m = None, raceline_generator = None, raceline_iterations = None, overrides = None, conditions = None, tier = None, sim = None, tire_thermal = true, initial_tire_temp_c = None))]
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub(crate) fn solve_stint(
    vehicle_dir: &str,
    track: &Track,
    n_laps: usize,
    ds_m: f64,
    raceline_ds_m: Option<f64>,
    raceline_generator: Option<&str>,
    raceline_iterations: Option<usize>,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    tier: Option<&str>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
    tire_thermal: bool,
    initial_tire_temp_c: Option<f64>,
) -> PyResult<QssStint> {
    check_ds(ds_m)?;
    if !(1..=MAX_STINT_LAPS).contains(&n_laps) {
        return Err(PyValueError::new_err(format!(
            "n_laps must lie in 1..={MAX_STINT_LAPS}, got {n_laps}"
        )));
    }
    // Sim FIRST: its `allow_degraded` feeds the load pipeline (the ers↔battery integrity checks).
    let sim_cfg = build_sim(&FsLoader::new(vehicle_dir), sim, tier)?;
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
    let hash = resolved.report.resolved_hash.clone();
    let fzc = sim_cfg.resolved_fz_coupling();
    let flat = sim_cfg.flat_track;

    let wanted = match sim_cfg.tier {
        Tier::T2 => {
            return Err(PyValueError::new_err(
                "the transient tier (t2) integrates in time: call \
                 `outlap.solve_transient_stint(...)` (or `outlap.solve_stint_dataset(..., \
                 tier=\"t2\")`) for a T2 stint",
            ))
        }
        tier @ Tier::T3 => return Err(err(tier_not_implemented(tier))),
        wanted => wanted,
    };

    let mut t1v =
        T1Vehicle::assemble(&resolved, &conditions, &vl, sim_cfg.allow_degraded).map_err(err)?;
    let mut notes = Vec::new();
    let sidecar_fp = install_sidecars(&mut t1v, &resolved, &vl, &mut notes)?;
    // With the tyre march on the envelope needs the (T_tire, wear) axes (built once, reused across
    // laps); otherwise the cheap frozen envelope (a frozen-tyre stint — every lap identical).
    let env = if tire_thermal {
        GgvEnvelope::generate_with_tire_state(&t1v, &sim_cfg.envelope, fzc, TireStateRes::default())
            .map_err(err)?
    } else {
        cached_envelope(&t1v, &sim_cfg, &hash, sidecar_fp, &conditions)?
    };
    let t0v = T0Vehicle::assemble(&resolved, &conditions, &vl, &opts).map_err(err)?;
    notes.extend(t0v.notes().iter().cloned());
    notes.extend(t1v.notes().iter().cloned());
    notes.extend(env.notes().iter().cloned());
    // The electrified coupling (inert unless the car's stack files are present). It re-seeds per lap in
    // the QSS stint (SoC is not carried across laps here — M6 PR3); the tyre state is what carries.
    let stack = build_slow_stack(&resolved, &vl, &conditions, &mut notes)?;
    // The 2026 ERS energy manager (per-lap budgets; the ledger resets each lap by construction).
    let ers_coupling = build_ers_coupling(
        &resolved,
        &t0v,
        stack.is_some(),
        sim_cfg.allow_degraded,
        &mut notes,
    )?;
    let base_march = if tire_thermal {
        Some(build_tire_march(
            &t1v,
            &resolved,
            &conditions,
            &vl,
            &mut notes,
        )?)
    } else {
        None
    };
    notes.push(format!(
        "QSS stint: {n_laps} laps run back-to-back with the representative-tyre thermal ring + wear \
         state carried across each lap boundary (§6.1 explicit-Euler march, extended across laps — no \
         reset); the tyre-state g-g-g-v envelope is built once and reused. Per-lap lap time responds \
         to the evolving tyre state."
    ));

    // The seed lap 1 starts from: an explicit cold-uniform temperature (the out-lap warm-up), else the
    // march's parity-safe warm-at-optimum default.
    let mut seed_state: Option<TireThermalState<f64>> = if tire_thermal {
        Some(match initial_tire_temp_c {
            Some(t) => TireThermalState::uniform(t + 273.15),
            None => base_march
                .as_ref()
                .expect("march built when tire_thermal")
                .seed(),
        })
    } else {
        None
    };

    let n_stations = path.len();
    let mut s_grid: Vec<f64> = Vec::new();
    let mut lap_time_s = Vec::with_capacity(n_laps);
    let mut v_flat = Vec::with_capacity(n_laps * n_stations);
    let (mut surf, mut carc, mut gas) = (Vec::new(), Vec::new(), Vec::new());
    let (mut wear, mut dmg, mut grip) = (Vec::new(), Vec::new(), Vec::new());
    let mut have_tire = false;
    let (mut tier_out, mut fz_out, mut hash_out) = (String::new(), String::new(), String::new());

    for lap_idx in 0..n_laps {
        let march_lap = base_march.as_ref().map(|bm| {
            bm.clone()
                .with_state(seed_state.expect("seed set when tire_thermal"))
        });
        let coupling = stack.as_ref().map(|(thermal, pack, state)| SlowCoupling {
            vehicle: &t1v,
            thermal: thermal.clone(),
            pack: pack.clone(),
            pack_state: *state,
            active: t1v.has_energy_maps(),
        });
        let line = line_descriptor(raceline_ds_m, raceline_generator, raceline_iterations);
        let couplings = Couplings {
            electro: coupling.as_ref(),
            tire: march_lap.as_ref(),
            ers: ers_coupling.as_ref(),
        };
        let req = LapRequest {
            line,
            resolved_hash: hash.clone(),
            notes: Vec::new(),
            fz_coupling: fzc,
            flat_track: flat,
        };
        let qss: QssLap = if wanted == Tier::T0 {
            solve_t0(&t0v, env.clone(), &couplings, &path, req).map_err(err)?
        } else {
            solve_t1(&t1v, &t0v, env.clone(), &couplings, &path, req).map_err(err)?
        };
        if lap_idx == 0 {
            s_grid.clone_from(&qss.lap.s);
            tier_out = format!("{:?}", qss.tier).to_lowercase();
            fz_out = fz_coupling_str(qss.fz_coupling);
            hash_out.clone_from(&qss.lap.resolved_hash);
        }
        // Surface the per-lap ERS record (manager mode, ledger MJ, C5.2.9 swing, convergence)
        // from the LAST lap — the per-lap request carries no assembly notes, so `qss.lap.notes`
        // here is exactly the manager's `finish_notes` output. Nothing silent (D-M6-3).
        if lap_idx + 1 == n_laps {
            for note in &qss.lap.notes {
                if !notes.contains(note) {
                    notes.push(format!("lap {}: {note}", lap_idx + 1));
                }
            }
        }
        lap_time_s.push(qss.lap.lap_time_s);
        v_flat.extend_from_slice(&qss.lap.v);
        if let Some(t) = &qss.tire {
            surf.extend_from_slice(&t.surface_temp_c);
            carc.extend_from_slice(&t.carcass_temp_c);
            gas.extend_from_slice(&t.gas_temp_c);
            wear.extend_from_slice(&t.wear_mm);
            dmg.extend_from_slice(&t.damage);
            grip.extend_from_slice(&t.grip_scale);
            have_tire = true;
        }
        // Carry the terminal tyre state into the next lap's seed (the stint's whole point).
        seed_state = qss.tire_terminal.or(seed_state);
    }

    Ok(QssStint {
        n_laps,
        s: s_grid,
        lap_time_s,
        v: v_flat,
        tire_surface_c: have_tire.then_some(surf),
        tire_carcass_c: have_tire.then_some(carc),
        tire_gas_c: have_tire.then_some(gas),
        tire_wear_mm: have_tire.then_some(wear),
        tire_damage: have_tire.then_some(dmg),
        tire_grip: have_tire.then_some(grip),
        tier: tier_out,
        fz_coupling: fz_out,
        flat_track: flat,
        resolved_hash: hash_out,
        notes,
    })
}
