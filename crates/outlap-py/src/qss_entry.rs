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
    // Fuel-mass slow-state channel (per station, kg); `None` unless the car carries a `fuel:` block
    // (§8.1, M6/PR5). Drains monotonically over the lap as the ICE burns.
    fuel_mass_kg: Option<Vec<f64>>,
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
    /// Fuel mass per station (kg), or `None` when the car carries no `fuel:` block (§8.1, M6/PR5).
    fn fuel_mass_kg<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.fuel_mass_kg
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

/// Which QSS entry point is assembling — selects the transient-redirect message when the resolved
/// tier is `t2` (the single lap and the stint point at different transient entry points).
#[derive(Clone, Copy)]
pub(crate) enum QssEntryKind {
    Lap,
    Stint,
}

/// The assembled QSS artifacts shared by [`solve_lap`] and [`solve_stint`] — the resolved tier +
/// vehicles + envelope + path, the optional electro stack / energy manager / tyre march, and the
/// recorded numerics. Built once by [`prepare_qss`] (the T2 `prepare_transient` mirror), so the
/// ~80-line lap/stint prologue lives in one place (PR3a).
pub(crate) struct PreparedQss {
    pub t0v: T0Vehicle,
    pub t1v: T1Vehicle,
    pub env: GgvEnvelope,
    pub path: T0Path,
    pub stack: Option<(Option<MachineThermal>, Pack, PackState)>,
    pub ers_coupling: Option<ErsCoupling>,
    pub base_march: Option<TireThermalMarch>,
    /// The fuel-mass model (§8.1, D-M6-4) parsed from the resolved vehicle, or `None` when the car
    /// carries no `fuel:` block. Paired with `t1v` at the solve into a `FuelCoupling`.
    pub fuel_model: Option<outlap_qss::fuel::FuelModel>,
    pub wanted: Tier,
    pub hash: String,
    pub fzc: outlap_schema::sim::FzCoupling,
    pub flat: bool,
    pub notes: Vec<String>,
}

/// Assemble the QSS artifacts for a `t0`/`t1` run: resolve the vehicle (with overrides), merge
/// conditions, build the path, generate (or reuse) the g-g-g-v envelope, and build the electro
/// slow stack (seeded from `initial_soc`), the energy manager, and the tyre march. A resolved `t2`
/// tier redirects to the transient entry point; `t3` raises the typed not-implemented error.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub(crate) fn prepare_qss(
    kind: QssEntryKind,
    vehicle_dir: &str,
    track: &Track,
    ds_m: f64,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    tier: Option<&str>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
    tire_thermal: bool,
    initial_soc: Option<f64>,
    override_active: bool,
    us_schedule: Option<outlap_qss::ers::UsSchedule<f64>>,
) -> PyResult<PreparedQss> {
    if let Some(soc) = initial_soc {
        if !(0.0..=1.0).contains(&soc) {
            return Err(PyValueError::new_err(format!(
                "initial_soc must lie in [0, 1]; got {soc}"
            )));
        }
    }
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

    // Enum dispatch on the resolved tier (assembly-time, never in the loop). T2 is time-indexed and
    // T3 is unimplemented — both before any T0/T1 assembly work.
    let wanted = match sim_cfg.tier {
        // The transient tiers (t2 = double-track, t3 = 14-DOF suspension) integrate in TIME, so they
        // are served by the time-indexed transient entry points, not this arc-length QSS one.
        transient @ (Tier::T2 | Tier::T3) => {
            let t = if transient == Tier::T2 { "t2" } else { "t3" };
            let msg = match kind {
                QssEntryKind::Lap => format!(
                    "the transient tier ({t}) produces a time-indexed lap: call \
                     `outlap.solve_transient_lap(..., tier=\"{t}\")`, or \
                     `outlap.solve_lap_dataset(..., tier=\"{t}\")` for an xarray view"
                ),
                QssEntryKind::Stint => format!(
                    "the transient tier ({t}) integrates in time and is time-indexed: call \
                     `outlap.solve_transient_stint(..., tier=\"{t}\")` (or \
                     `outlap.solve_stint_dataset(..., tier=\"{t}\")`) for a transient stint"
                ),
            };
            return Err(PyValueError::new_err(msg));
        }
        wanted => wanted,
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

    let mut t1v =
        T1Vehicle::assemble(&resolved, &conditions, &vl, sim_cfg.allow_degraded).map_err(err)?;
    let mut notes = Vec::new();
    // Native-edge sidecar decode: the aero map + `.ptm` tables (skipped with a note when absent).
    let sidecar_fp = install_sidecars(&mut t1v, &resolved, &vl, &mut notes)?;
    // With the tyre-thermal march opted in the envelope needs the (T_tire, wear) axes (a full
    // re-solve across the axis product — not cached). Otherwise the cheap frozen envelope
    // (bit-identical to pre-M5).
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
    // The electrified slow stack (inert with a note when the stack files are absent), seeded from
    // `initial_soc` when given, else mid-window (ers) / top-of-window (mapped EV — bit-identity).
    let stack = build_slow_stack(&resolved, &vl, &conditions, initial_soc, &mut notes)?;
    // The 2026 ERS energy manager (M6 PR2): governs the march whenever the car has an `ers:` block
    // AND a pack to schedule.
    // A finite `lift_point` on a QSS run is NOT silently honoured: the quasi-steady speed profile is
    // envelope-derived (the point-mass optimum), so lift-and-coast — an early throttle-lift that
    // trades lap time for harvest — is a closed-loop *driver* action, realised at the T2 tier (the
    // Driver caps its tracked reference, §8.3, D-M6-9). A physically-consistent QSS lift is a profile
    // re-solve with a per-station speed cap, deferred so it cannot perturb the QSS↔T2 parity gates.
    if let Some(u) = us_schedule.as_ref() {
        if u.lift_points().iter().any(|v| v.is_finite()) {
            notes.push(
                "QSS run: the u(s) lift_point is recorded but not applied at this tier — the \
                 quasi-steady speed profile is the envelope-derived point-mass optimum, so \
                 lift-and-coast (an early throttle-lift, banking harvest for lap time) is a T2 \
                 closed-loop driver action. Run the T2 tier to exercise the lift (§8.3)"
                    .to_owned(),
            );
        }
    }
    let ers_policy = us_schedule.map_or(outlap_qss::ers::ErsPolicy::RuleBased, |u| {
        outlap_qss::ers::ErsPolicy::Schedule(u)
    });
    // The governed pack's usable SoC window feeds the manager rulebook's recharge-target default
    // (the pack is the single source of truth, D-M6-13); [0,1] when no pack (the coupling is inert).
    let pack_window = stack
        .as_ref()
        .map_or([0.0, 1.0], |(_, pack, _)| pack.soc_window());
    let ers_coupling = build_ers_coupling(
        &resolved,
        &t0v,
        pack_window,
        stack.is_some(),
        sim_cfg.allow_degraded,
        ers_policy,
        override_active,
        &mut notes,
    )?;
    // Tyre-thermal march (M5 PR5): opt-in, so the default run stays bit-identical to pre-M5.
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

    let fuel_model = outlap_qss::fuel::FuelModel::from_spec(&resolved.spec);
    Ok(PreparedQss {
        t0v,
        t1v,
        env,
        path,
        stack,
        ers_coupling,
        base_march,
        fuel_model,
        wanted,
        hash,
        fzc,
        flat,
        notes,
    })
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
/// `conditions` is a nested dict deep-merged onto the session conditions. `initial_soc` seeds the
/// battery pack (default: the middle of its usable window) for an electrified car.
///
/// The call holds the GIL for its duration (envelope generation is a seconds-scale cold step in a
/// debug build); releasing it is deferred to the batch/sweep API milestone.
#[pyfunction]
#[pyo3(signature = (vehicle_dir, track, ds_m = DEFAULT_DS_M, raceline_ds_m = None, raceline_generator = None, raceline_iterations = None, overrides = None, conditions = None, tier = None, sim = None, tire_thermal = false, initial_soc = None, r#override = false, us_schedule = None))]
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
    initial_soc: Option<f64>,
    r#override: bool,
    us_schedule: Option<&Bound<'_, pyo3::types::PyDict>>,
) -> PyResult<Lap> {
    check_ds(ds_m)?;
    let us_schedule = crate::transient_entry::us_schedule_from_py(us_schedule)?;
    let PreparedQss {
        t0v,
        t1v,
        env,
        path,
        stack,
        ers_coupling,
        base_march: tire_march,
        fuel_model,
        wanted,
        hash,
        fzc,
        flat,
        notes,
    } = prepare_qss(
        QssEntryKind::Lap,
        vehicle_dir,
        track,
        ds_m,
        overrides,
        conditions,
        tier,
        sim,
        tire_thermal,
        initial_soc,
        r#override,
        us_schedule,
    )?;

    let coupling = stack.as_ref().map(|(thermal, pack, state)| SlowCoupling {
        vehicle: &t1v,
        thermal: thermal.clone(),
        pack: pack.clone(),
        pack_state: *state,
        active: t1v.has_energy_maps(),
    });
    let fuel_coupling = fuel_model.map(|model| outlap_qss::fuel::FuelCoupling {
        model,
        vehicle: &t1v,
    });
    let couplings = Couplings {
        electro: coupling.as_ref(),
        tire: tire_march.as_ref(),
        ers: ers_coupling.as_ref(),
        fuel: fuel_coupling.as_ref(),
    };
    let req = LapRequest {
        line: line_descriptor(raceline_ds_m, raceline_generator, raceline_iterations),
        resolved_hash: hash,
        notes,
        fz_coupling: fzc,
        flat_track: flat,
    };
    let qss: QssLap = if wanted == Tier::T0 {
        solve_t0(&t0v, env, &couplings, &path, req).map_err(err)?
    } else {
        solve_t1(&t1v, &t0v, env, &couplings, &path, req).map_err(err)?
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
#[allow(clippy::too_many_arguments)] // cold binding-edge assembly; the pack window joins the set (D-M6-13)
pub(crate) fn build_ers_coupling(
    resolved: &ResolvedVehicle,
    t0v: &T0Vehicle,
    pack_soc_window: [f64; 2],
    pack_present: bool,
    allow_degraded: bool,
    policy: outlap_qss::ers::ErsPolicy<f64>,
    override_active: bool,
    notes: &mut Vec<String>,
) -> PyResult<Option<ErsCoupling>> {
    if resolved.spec.policy.is_none() {
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
        pack_soc_window,
        policy,
        override_active,
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
        fuel_mass_kg: qss.fuel.as_ref().map(|f| f.fuel_mass_kg.clone()),
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
    // Per-`(lap × station)` pack state of charge (0..1), carried continuously across lap boundaries;
    // `None` when the car has no active electro stack.
    state_of_charge: Option<Vec<f64>>,
    // Per-`(lap × station)` fuel mass, kg (drains lap-over-lap as the ICE burns); `None` when the
    // car carries no `fuel:` block (§8.1, M6/PR5, D-M6-4).
    fuel_mass_kg: Option<Vec<f64>>,
    // Per-lap on-track SoC extremes + electrical ERS deploy/harvest energy, MJ (n_laps); `None`
    // unless the 2026 energy manager governed the stint. `deploy − harvest` is the per-lap net
    // electrical charge — the honest closure quantity for the M6 PR8 gate #2 check.
    soc_min: Option<Vec<f64>>,
    soc_max: Option<Vec<f64>>,
    deploy_mj: Option<Vec<f64>>,
    harvest_mj: Option<Vec<f64>>,
    // Per-lap END-of-lap pack + machine temperatures, °C (n_laps); `None` when absent (machine temp
    // is never surfaced under an energy manager — D-M6-10).
    pack_temp_c: Option<Vec<f64>>,
    machine_temp_c: Option<Vec<f64>>,
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
    /// Pack state of charge `SoC ∈ [0, 1]` (`n_laps × station`), continuous across lap boundaries,
    /// or `None` when the car carries no active battery.
    fn state_of_charge<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        let n = self.n_stations();
        self.state_of_charge
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, n))
    }
    /// Fuel mass, kg (`n_laps × station`), draining continuously across lap boundaries, or `None`
    /// when the car carries no `fuel:` block (§8.1, M6/PR5).
    fn fuel_mass_kg<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        let n = self.n_stations();
        self.fuel_mass_kg
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, n))
    }
    /// Per-lap minimum on-track pack SoC (`n_laps`), or `None` without an energy manager.
    fn soc_min<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.soc_min.as_ref().map(|f| f.clone().into_pyarray(py))
    }
    /// Per-lap maximum on-track pack SoC (`n_laps`), or `None` without an energy manager.
    fn soc_max<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.soc_max.as_ref().map(|f| f.clone().into_pyarray(py))
    }
    /// Per-lap electrical ERS deploy energy, MJ (`n_laps`), or `None` without an energy manager.
    fn deploy_mj<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.deploy_mj.as_ref().map(|f| f.clone().into_pyarray(py))
    }
    /// Per-lap electrical ERS harvest energy, MJ (`n_laps`), or `None` without an energy manager.
    fn harvest_mj<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.harvest_mj.as_ref().map(|f| f.clone().into_pyarray(py))
    }
    /// End-of-lap pack temperature, °C (`n_laps`), or `None` when the car carries no battery.
    fn pack_temp_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.pack_temp_c
            .as_ref()
            .map(|f| f.clone().into_pyarray(py))
    }
    /// End-of-lap machine winding temperature, °C (`n_laps`), or `None` (never surfaced under an
    /// energy manager — the caps apply to the electrical share, D-M6-10).
    fn machine_temp_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.machine_temp_c
            .as_ref()
            .map(|f| f.clone().into_pyarray(py))
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
/// slow stack — pack SoC / RC voltage / temperature, and the machine-thermal network — **carries**
/// lap-to-lap too (M6 PR3): a multi-lap run shows SoC falling with net consumption and rising with
/// regeneration, with only the per-lap ERS budget ledger resetting at the start/finish. `initial_soc`
/// seeds the pack (default: the middle of its usable window, matching T2).
#[pyfunction]
#[pyo3(signature = (vehicle_dir, track, n_laps, ds_m = DEFAULT_DS_M, raceline_ds_m = None, raceline_generator = None, raceline_iterations = None, overrides = None, conditions = None, tier = None, sim = None, tire_thermal = true, initial_tire_temp_c = None, initial_soc = None, r#override = false, us_schedule = None))]
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
    initial_soc: Option<f64>,
    r#override: bool,
    us_schedule: Option<&Bound<'_, pyo3::types::PyDict>>,
) -> PyResult<QssStint> {
    check_ds(ds_m)?;
    if !(1..=MAX_STINT_LAPS).contains(&n_laps) {
        return Err(PyValueError::new_err(format!(
            "n_laps must lie in 1..={MAX_STINT_LAPS}, got {n_laps}"
        )));
    }
    let us_schedule = crate::transient_entry::us_schedule_from_py(us_schedule)?;
    let PreparedQss {
        t0v,
        t1v,
        env,
        path,
        stack,
        ers_coupling,
        base_march,
        fuel_model,
        wanted,
        hash,
        fzc,
        flat,
        mut notes,
    } = prepare_qss(
        QssEntryKind::Stint,
        vehicle_dir,
        track,
        ds_m,
        overrides,
        conditions,
        tier,
        sim,
        tire_thermal,
        initial_soc,
        r#override,
        us_schedule,
    )?;
    notes.push(format!(
        "QSS stint: {n_laps} laps run back-to-back with the FULL slow stack — the representative-tyre \
         thermal ring + wear AND the battery pack (SoC / RC voltage / temperature) AND the \
         machine-thermal network — carried across each lap boundary (§6.1 march, extended across laps \
         — only the per-lap ERS ledger resets at s = 0); the g-g-g-v envelope is built once and \
         reused. Per-lap lap time and SoC respond to the evolving slow state."
    ));

    // The lap-1 tyre seed: an explicit cold-uniform temperature (the out-lap warm-up), else the
    // march's parity-safe warm-at-optimum default. (The pack seed rides in the stack, seeded by
    // `initial_soc` / mid-window in `prepare_qss`.)
    let tire_seed: Option<TireThermalState<f64>> = if tire_thermal {
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

    // Run the multi-lap loop in `outlap-qss` (PR3a): the lap-boundary carry lives there, cargo-tested.
    let electro = stack
        .as_ref()
        .map(|(thermal, pack, state)| outlap_qss::StintElectro {
            vehicle: &t1v,
            pack,
            thermal: thermal.as_ref(),
            pack_state: *state,
            active: t1v.has_energy_maps(),
        });
    let fuel_coupling = fuel_model.map(|model| outlap_qss::fuel::FuelCoupling {
        model,
        vehicle: &t1v,
    });
    let plan = outlap_qss::StintPlan {
        tier: wanted,
        t0: &t0v,
        t1: &t1v,
        env: &env,
        path: &path,
        electro,
        ers: ers_coupling.as_ref(),
        base_march: base_march.as_ref(),
        fuel: fuel_coupling.as_ref(),
        request: LapRequest {
            line: line_descriptor(raceline_ds_m, raceline_generator, raceline_iterations),
            resolved_hash: hash.clone(),
            notes: Vec::new(),
            fz_coupling: fzc,
            flat_track: flat,
        },
    };
    let result = outlap_qss::solve_stint(&plan, n_laps, outlap_qss::StintSeeds { tire: tire_seed })
        .map_err(err)?;

    // Surface the LAST lap's energy-manager + convergence notes (manager mode, ledger MJ, C5.2.9
    // swing, convergence) — nothing silent (D-M6-3).
    if let Some(last) = result.laps.last() {
        for note in &last.notes {
            let tagged = format!("lap {n_laps}: {note}");
            if !notes.contains(&tagged) {
                notes.push(tagged);
            }
        }
    }

    // Aggregate the per-lap lean results into the `(lap, station)` block channels.
    let n_stations = path.len();
    let mut lap_time_s = Vec::with_capacity(n_laps);
    let mut v_flat = Vec::with_capacity(n_laps * n_stations);
    let (mut surf, mut carc, mut gas) = (Vec::new(), Vec::new(), Vec::new());
    let (mut wear, mut dmg, mut grip) = (Vec::new(), Vec::new(), Vec::new());
    let mut soc_flat: Vec<f64> = Vec::new();
    let mut fuel_flat: Vec<f64> = Vec::new();
    let (mut pack_temp, mut machine_temp) = (Vec::new(), Vec::new());
    let (mut soc_lo, mut soc_hi) = (Vec::new(), Vec::new());
    let (mut deploy_mj, mut harvest_mj) = (Vec::new(), Vec::new());
    let mut have_tire = false;
    let (mut have_soc, mut have_pack_temp, mut have_machine_temp) = (false, false, false);
    let (mut have_fuel, mut have_ers) = (false, false);

    for lap in &result.laps {
        lap_time_s.push(lap.lap_time_s);
        v_flat.extend_from_slice(&lap.v);
        if let Some(t) = &lap.tire {
            surf.extend_from_slice(&t.surface_temp_c);
            carc.extend_from_slice(&t.carcass_temp_c);
            gas.extend_from_slice(&t.gas_temp_c);
            wear.extend_from_slice(&t.wear_mm);
            dmg.extend_from_slice(&t.damage);
            grip.extend_from_slice(&t.grip_scale);
            have_tire = true;
        }
        if let Some(s) = &lap.slow {
            soc_flat.extend_from_slice(&s.state_of_charge);
            have_soc = true;
            // The per-lap energy-manager ledger (was dropped before M6 PR8): surface the deploy /
            // harvest MJ and the on-track SoC extremes the closure gate reads.
            if let Some(e) = &s.ers {
                deploy_mj.push(e.ledger_deploy_j * 1e-6);
                harvest_mj.push(e.ledger_harvest_j * 1e-6);
                soc_lo.push(e.soc_min);
                soc_hi.push(e.soc_max);
                have_ers = true;
            }
        }
        if let Some(f) = &lap.fuel {
            fuel_flat.extend_from_slice(&f.fuel_mass_kg);
            have_fuel = true;
        }
        // End-of-lap pack + machine temperature from the terminal snapshot the next lap seeded from.
        if let Some(p) = &lap.terminal.pack {
            pack_temp.push(p.temp_k - 273.15);
            have_pack_temp = true;
        }
        if let Some(m) = &lap.terminal.machine {
            machine_temp.push(m.winding_temp_c());
            have_machine_temp = true;
        }
    }

    Ok(QssStint {
        n_laps,
        s: result.s,
        lap_time_s,
        v: v_flat,
        tire_surface_c: have_tire.then_some(surf),
        tire_carcass_c: have_tire.then_some(carc),
        tire_gas_c: have_tire.then_some(gas),
        tire_wear_mm: have_tire.then_some(wear),
        tire_damage: have_tire.then_some(dmg),
        tire_grip: have_tire.then_some(grip),
        state_of_charge: have_soc.then_some(soc_flat),
        soc_min: have_ers.then_some(soc_lo),
        soc_max: have_ers.then_some(soc_hi),
        deploy_mj: have_ers.then_some(deploy_mj),
        harvest_mj: have_ers.then_some(harvest_mj),
        fuel_mass_kg: have_fuel.then_some(fuel_flat),
        pack_temp_c: have_pack_temp.then_some(pack_temp),
        machine_temp_c: have_machine_temp.then_some(machine_temp),
        tier: format!("{:?}", result.tier).to_lowercase(),
        fz_coupling: fz_coupling_str(result.fz_coupling),
        flat_track: flat,
        resolved_hash: hash,
        notes,
    })
}
