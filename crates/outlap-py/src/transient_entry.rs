// SPDX-License-Identifier: AGPL-3.0-only
//! Transient (T2) Python entry points and result classes (`TransientLap`, `TransientStint`).

use crate::prelude::*;

/// The battery pack as the transient solver's slow-state stack (Decision #6): it Coulomb-counts the
/// recovered regen energy on the decimated slow clock and publishes back the pack's
/// **charge-acceptance ceiling** at the current SoC *and temperature*, which caps the series regen
/// blend. Held here at the native edge, so the wasm-clean transient crate keeps no QSS dependency.
///
/// The pack sees the **net** electrical power (regen recovered − traction drawn): it charges under
/// braking and discharges under power, so the SoC moves both ways over a lap.
pub(crate) struct PackSlowStack {
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
    fn discharge_power_limit_w(&self) -> f64 {
        self.pack.discharge_power_limit_w(&self.state)
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
pub(crate) fn line_from_track(
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
pub(crate) fn straightest_station(kappa: &[f64]) -> usize {
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
    // Per-wheel tyre-thermal channels (row-major `n × 4`, FL/FR/RL/RR); `None` unless the M5
    // tyre-thermal stack was attached (`tire_thermal=True`).
    tire_surface_c: Option<Vec<f64>>,
    tire_carcass_c: Option<Vec<f64>>,
    tire_gas_c: Option<Vec<f64>>,
    tire_wear_mm: Option<Vec<f64>>,
    tire_damage: Option<Vec<f64>>,
    tire_grip: Option<Vec<f64>>,
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
    /// Per-wheel tyre tread-surface temperature `T_s`, °C (`time × wheel`) — `None` unless the M5
    /// tyre-thermal stack was attached (`tire_thermal=True`).
    fn tire_surface_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.tire_surface_c.as_ref())
    }
    /// Per-wheel carcass (bulk) temperature `T_c`, °C (`time × wheel`), or `None`.
    fn tire_carcass_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.tire_carcass_c.as_ref())
    }
    /// Per-wheel inflation-gas temperature `T_g`, °C (`time × wheel`), or `None`.
    fn tire_gas_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.tire_gas_c.as_ref())
    }
    /// Per-wheel tread wear depth `w`, mm (`time × wheel`, monotone), or `None`.
    fn tire_wear_mm<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.tire_wear_mm.as_ref())
    }
    /// Per-wheel irreversible thermal damage `D` (`time × wheel`, 0..1), or `None`.
    fn tire_damage<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.tire_damage.as_ref())
    }
    /// Per-wheel total grip multiplier `λ_μ,total` (`time × wheel`) the force call used, or `None`.
    fn tire_grip<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        wheel_array(py, self.tire_grip.as_ref())
    }
    /// The number of recorded steps.
    fn __len__(&self) -> usize {
        self.t.len()
    }
}

/// Everything a transient run needs, assembled once through the shared pipeline: the block set +
/// interned bus, the sampled target line, the numerics, and the optional slow subsystems (battery
/// pack, per-wheel tyre-thermal ring, gear-shift FSM). Owned values only — the caller constructs the
/// [`TransientSolver`] over them (which borrows the interner) and runs one lap or a multi-lap stint,
/// so [`solve_transient_lap`] and [`solve_transient_stint`] share one assembly path (byte-identical
/// single-lap numerics).
pub(crate) struct PreparedTransient {
    blocks: outlap_transient::T2Blocks<f64>,
    line: outlap_transient::LineTable<f64>,
    interner: outlap_transient::ChannelInterner,
    cfg: outlap_transient::SimConfig<f64>,
    /// One-lap arc length (the finish line), m.
    length: f64,
    /// Whether the run is flat-track (grade/banking/κ_v zeroed).
    flat: bool,
    /// The corner-scaled speed-profile fraction the driver tracked.
    speed_margin: f64,
    resolved_hash: String,
    notes: Vec<String>,
    /// The vehicle's battery pack + its seeded state (`None` when the car carries no battery).
    pack: Option<(Pack, PackState)>,
    /// The per-wheel tyre-thermal ring + wear stack (`None` unless `tire_thermal` opted in).
    tire_stack: Option<outlap_transient::TireThermalStack<f64>>,
    /// The gear-shift FSM (`None` for a single-speed car or one declaring no shift time).
    shifter: Option<outlap_transient::Shifter<f64>>,
}

/// Assemble the transient block set + target line + slow subsystems for a T2 run (one lap or a
/// stint). This is the entire `solve_transient_lap` prologue factored out so the stint driver reuses
/// the identical assembly; only the run (one lap vs. `n_laps`) and the result surface differ.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub(crate) fn prepare_transient(
    vehicle_dir: &str,
    track: &Track,
    ds_m: f64,
    raceline_ds_m: Option<f64>,
    raceline_generator: Option<&str>,
    raceline_iterations: Option<usize>,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
    speed_margin: f64,
    initial_soc: Option<f64>,
    tire_thermal: bool,
    initial_tire_temp_c: Option<f64>,
) -> PyResult<PreparedTransient> {
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
    // Sim FIRST: its `allow_degraded` feeds the load pipeline (the ers↔battery integrity checks).
    let sim_cfg = build_sim(&FsLoader::new(vehicle_dir), sim, Some("t2"))?;
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
    let hash = resolved.report.resolved_hash.clone();
    let mut notes = Vec::new();

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
    let line_descriptor = line_descriptor(raceline_ds_m, raceline_generator, raceline_iterations);
    let mut t1v =
        T1Vehicle::assemble(&resolved, &conditions, &vl, sim_cfg.allow_degraded).map_err(err)?;
    let sidecar_fp = install_sidecars(&mut t1v, &resolved, &vl, &mut notes)?;
    let env = cached_envelope(&t1v, &sim_cfg, &hash, sidecar_fp, &conditions)?;
    let t0v = T0Vehicle::assemble(&resolved, &conditions, &vl, &opts).map_err(err)?;
    notes.extend(t0v.notes().iter().cloned());
    notes.extend(t1v.notes().iter().cloned());
    notes.extend(env.notes().iter().cloned());
    let env_shape = env.clone(); // kept for the corner-scaled target shaping (solve_t0 consumes env)
    let reference = solve_t0(
        &t0v,
        env,
        &Couplings::default(),
        &path,
        LapRequest {
            line: line_descriptor,
            resolved_hash: hash.clone(),
            notes: Vec::new(),
            fz_coupling: sim_cfg.resolved_fz_coupling(),
            flat_track: flat,
        },
    )
    .map_err(err)?;

    // --- The T2 block set, through the shared assembly pipeline. ----------------------------------
    let mut pack = load_pack(&resolved, &vl, &mut notes)?;
    let mut interner = outlap_transient::ChannelInterner::new();
    let t2_opts = outlap_vehicle::T2Options {
        battery_present: pack.is_some(),
        ..outlap_vehicle::T2Options::default()
    };
    let blocks: outlap_transient::T2Blocks<f64> =
        outlap_vehicle::assemble_t2(&t1v, &resolved.spec, &mut interner, &t2_opts, &mut notes)
            .into();

    let v_target = outlap_qss::corner_scaled_targets(
        &env_shape,
        &path,
        &reference.lap.v,
        &reference.lap.ax,
        speed_margin,
    );
    let line = line_from_track(&track.inner, &path, &v_target, flat)?;
    let start_i = straightest_station(&path.kappa_l);
    let start_s = path.s[start_i];
    let length = reference.lap.s.last().copied().unwrap_or(0.0);

    let cfg = outlap_transient::SimConfig {
        fz_coupling: sim_cfg.resolved_fz_coupling(),
        start_s,
        ..outlap_transient::SimConfig::default()
    };

    // Seed the pack mid-window (a pack at the top of its SoC accepts no charge — useless for a lap).
    if let Some((pack_ref, state)) = pack.as_mut() {
        let [lo, hi] = pack_ref.soc_window();
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
    }
    notes.push(format!(
        "T2 driver tracks a corner-scaled speed reference: the full QSS profile where lateral \
         demand is low, {:.0}% of it at the lateral grip limit (the profile rides an envelope not \
         filtered for open-loop stability), with braking feasibility enforced at {:.0}% of the \
         braking capability; the lap is seeded at the straightest station, s = {start_s:.1} m",
        speed_margin * 100.0,
        speed_margin * speed_margin * 100.0
    ));

    // The gear-shift FSM: the crossover speeds where the best gear changes + the gearbox shift time.
    let upshift_speeds = t1v.upshift_speeds();
    let gear_count = t1v.gear_count();
    let shift_time_s = t1v.shift_time_s();
    let shifter = if upshift_speeds.is_empty() || shift_time_s <= 0.0 {
        notes.push(
            "T2 gear-shift FSM inert: the car is single-speed (direct drive) or declares no shift \
             time, so the lap runs the best-gear envelope with no torque interruption"
                .to_owned(),
        );
        None
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
        Some(outlap_transient::Shifter::new(
            gear_count,
            upshift_speeds,
            shift_time_s,
        ))
    };

    // The M5 per-wheel tyre-thermal ring + wear stack (opt-in). Seeded warm (parity-safe) by default;
    // an explicit `initial_tire_temp_c` gives a uniform cold start (the warm-up transient).
    let tire_stack = if tire_thermal {
        let mut stack = build_tire_thermal(&resolved, &conditions, &vl, &mut notes)?;
        if let Some(t) = initial_tire_temp_c {
            stack.seed_uniform(t);
            notes.push(format!(
                "T2 tyres seeded cold-uniform at {t:.0} °C (the warm-up transient): the grip window \
                 starts off the optimum, so lap 1 warms up into the window before it settles"
            ));
        }
        Some(stack)
    } else {
        None
    };

    Ok(PreparedTransient {
        blocks,
        line,
        interner,
        cfg,
        length,
        flat,
        speed_margin,
        resolved_hash: hash,
        notes,
        pack,
        tire_stack,
        shifter,
    })
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
#[pyo3(signature = (vehicle_dir, track, ds_m = DEFAULT_DS_M, raceline_ds_m = None, raceline_generator = None, raceline_iterations = None, overrides = None, conditions = None, sim = None, speed_margin = DEFAULT_SPEED_MARGIN, initial_soc = None, tire_thermal = false))]
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub(crate) fn solve_transient_lap(
    vehicle_dir: &str,
    track: &Track,
    ds_m: f64,
    raceline_ds_m: Option<f64>,
    raceline_generator: Option<&str>,
    raceline_iterations: Option<usize>,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
    speed_margin: f64,
    initial_soc: Option<f64>,
    // Opt-in for the M5 tire-thermal ring + wear stack (default off). The physics is fully wired, but
    // the reference `.tyr` thermal/wear params are still synthetic placeholders — their loaded
    // steady-state sits below the grip window, so a default-on lap would under-report pace. The flag
    // flips on by default once FastF1 inverse calibration (M5 PR7/PR8) sets the params so the
    // steady-state lands in the window. Opt in to exercise the wired physics today.
    tire_thermal: bool,
) -> PyResult<TransientLap> {
    let PreparedTransient {
        blocks,
        line,
        interner,
        cfg,
        length,
        flat,
        speed_margin,
        resolved_hash,
        mut notes,
        pack,
        tire_stack,
        shifter,
    } = prepare_transient(
        vehicle_dir,
        track,
        ds_m,
        raceline_ds_m,
        raceline_generator,
        raceline_iterations,
        overrides,
        conditions,
        sim,
        speed_margin,
        initial_soc,
        tire_thermal,
        None,
    )?;
    let start_s = cfg.start_s;
    let mut solver = outlap_transient::TransientSolver::new(blocks, line, &interner, cfg);
    if let Some((pack, state)) = pack {
        solver = solver.with_slow_stack(Box::new(PackSlowStack { pack, state }));
    }
    if let Some(shifter) = shifter {
        solver = solver.with_shifter(shifter);
    }
    if let Some(stack) = tire_stack {
        solver = solver.with_tire_thermal(stack);
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
    let has_tire = !lap.tire_surface_c.is_empty();
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
        tire_surface_c: has_tire.then(|| flat4(&lap.tire_surface_c)),
        tire_carcass_c: has_tire.then(|| flat4(&lap.tire_carcass_c)),
        tire_gas_c: has_tire.then(|| flat4(&lap.tire_gas_c)),
        tire_wear_mm: has_tire.then(|| flat4(&lap.tire_wear_mm)),
        tire_damage: has_tire.then(|| flat4(&lap.tire_damage)),
        tire_grip: has_tire.then(|| flat4(&lap.tire_grip)),
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
        resolved_hash,
    })
}

// ---------------------------------------------------------------------------------------------
// Multi-lap stints (M5 PR6): carry the tyre-thermal (and, in T2, the battery) slow state lap-to-lap.
// ---------------------------------------------------------------------------------------------

/// A solved **transient (T2) stint**: `n_laps` laps integrated **continuously** (one run, no re-seed),
/// so the per-wheel tyre-thermal ring + wear and the battery SoC carry across the start/finish line
/// with no reset (§6.1 slow-state continuity). Surfaced as per-lap summaries over a `lap` axis: the
/// per-lap lap time, and the per-wheel end-of-lap + peak tyre state (and end-of-lap pack state).
#[pyclass(frozen)]
pub struct TransientStint {
    /// Number of laps that completed within the step / divergence budget.
    #[pyo3(get)]
    n_laps: usize,
    /// Number of laps requested (may exceed `n_laps` if the closed loop diverged early).
    #[pyo3(get)]
    requested_laps: usize,
    /// Per-lap lap time, s (n_laps).
    lap_time_s: Vec<f64>,
    // Per-`(lap × wheel)` end-of-lap tyre state; `None` unless the tyre-thermal stack was attached.
    tire_surface_c: Option<Vec<f64>>,
    tire_carcass_c: Option<Vec<f64>>,
    tire_gas_c: Option<Vec<f64>>,
    tire_wear_mm: Option<Vec<f64>>,
    tire_damage: Option<Vec<f64>>,
    tire_grip: Option<Vec<f64>>,
    /// Per-`(lap × wheel)` peak tread-surface temperature over the lap (the warm-up marker).
    tire_peak_surface_c: Option<Vec<f64>>,
    /// Per-lap end-of-lap pack state of charge (n_laps); `None` when the car carries no battery.
    state_of_charge: Option<Vec<f64>>,
    /// Per-lap end-of-lap pack temperature, °C (n_laps); `None` when no battery.
    pack_temp_c: Option<Vec<f64>>,
    /// The resolved tier (always `"t2"`).
    #[pyo3(get)]
    tier: String,
    /// The recorded normal-load coupling mode.
    #[pyo3(get)]
    fz_coupling: String,
    /// Whether the stint ran flat-track.
    #[pyo3(get)]
    flat_track: bool,
    /// Resolved fixed step, s.
    #[pyo3(get)]
    dt_s: f64,
    /// Resolved integrator order.
    #[pyo3(get)]
    integrator_order: u32,
    /// The QSS-profile fraction the driver tracked.
    #[pyo3(get)]
    speed_margin: f64,
    /// Whether all `requested_laps` completed.
    #[pyo3(get)]
    completed: bool,
    /// The wheel-channel order (`["FL","FR","RL","RR"]`).
    #[pyo3(get)]
    wheels: Vec<String>,
    /// blake3 hash of the resolved vehicle spec.
    #[pyo3(get)]
    resolved_hash: String,
    /// Simplification/degradation notes (nothing silent).
    #[pyo3(get)]
    notes: Vec<String>,
}

#[pymethods]
impl TransientStint {
    /// Per-lap lap time, s (shape `n_laps`).
    fn lap_time_s<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.lap_time_s.clone().into_pyarray(py)
    }
    /// Per-wheel end-of-lap tread-surface temperature `T_s`, °C (`n_laps × wheel`), or `None`.
    fn tire_surface_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.tire_surface_c
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, 4))
    }
    /// Per-wheel end-of-lap carcass temperature `T_c`, °C (`n_laps × wheel`), or `None`.
    fn tire_carcass_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.tire_carcass_c
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, 4))
    }
    /// Per-wheel end-of-lap inflation-gas temperature `T_g`, °C (`n_laps × wheel`), or `None`.
    fn tire_gas_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.tire_gas_c
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, 4))
    }
    /// Per-wheel end-of-lap tread wear `w`, mm (`n_laps × wheel`, monotone), or `None`.
    fn tire_wear_mm<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.tire_wear_mm
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, 4))
    }
    /// Per-wheel end-of-lap irreversible thermal damage `D` (`n_laps × wheel`), or `None`.
    fn tire_damage<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.tire_damage
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, 4))
    }
    /// Per-wheel end-of-lap total grip multiplier `λ_μ,total` (`n_laps × wheel`), or `None`.
    fn tire_grip<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.tire_grip
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, 4))
    }
    /// Per-wheel peak tread-surface temperature over each lap, °C (`n_laps × wheel`), or `None`.
    fn tire_peak_surface_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray2<f64>>> {
        self.tire_peak_surface_c
            .as_ref()
            .map(|f| array2d(py, f, self.n_laps, 4))
    }
    /// Per-lap end-of-lap pack state of charge (shape `n_laps`), or `None` without a battery.
    fn state_of_charge<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.state_of_charge.clone().map(|v| v.into_pyarray(py))
    }
    /// Per-lap end-of-lap pack temperature, °C (shape `n_laps`), or `None` without a battery.
    fn pack_temp_c<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<f64>>> {
        self.pack_temp_c.clone().map(|v| v.into_pyarray(py))
    }
    fn __len__(&self) -> usize {
        self.n_laps
    }
}

/// Solve a **transient (T2) stint** — `n_laps` laps integrated in one continuous run, so the per-wheel
/// tyre-thermal ring + wear and the battery SoC carry across the start/finish line with no reset (the
/// line table wraps `s`, so the geometry + reference profile repeat every lap). Returns per-lap
/// summaries: lap time, per-wheel end-of-lap + peak tyre state, and end-of-lap pack state.
#[pyfunction]
#[pyo3(signature = (vehicle_dir, track, n_laps, ds_m = DEFAULT_DS_M, raceline_ds_m = None, raceline_generator = None, raceline_iterations = None, overrides = None, conditions = None, sim = None, speed_margin = DEFAULT_SPEED_MARGIN, initial_soc = None, tire_thermal = true, initial_tire_temp_c = None))]
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub(crate) fn solve_transient_stint(
    vehicle_dir: &str,
    track: &Track,
    n_laps: usize,
    ds_m: f64,
    raceline_ds_m: Option<f64>,
    raceline_generator: Option<&str>,
    raceline_iterations: Option<usize>,
    overrides: Option<&Bound<'_, pyo3::types::PyDict>>,
    conditions: Option<&Bound<'_, pyo3::types::PyDict>>,
    sim: Option<&Bound<'_, pyo3::types::PyDict>>,
    speed_margin: f64,
    initial_soc: Option<f64>,
    tire_thermal: bool,
    initial_tire_temp_c: Option<f64>,
) -> PyResult<TransientStint> {
    if !(1..=MAX_STINT_LAPS).contains(&n_laps) {
        return Err(PyValueError::new_err(format!(
            "n_laps must lie in 1..={MAX_STINT_LAPS}, got {n_laps}"
        )));
    }
    let PreparedTransient {
        blocks,
        line,
        interner,
        cfg,
        length,
        flat,
        speed_margin,
        resolved_hash,
        mut notes,
        pack,
        tire_stack,
        shifter,
    } = prepare_transient(
        vehicle_dir,
        track,
        ds_m,
        raceline_ds_m,
        raceline_generator,
        raceline_iterations,
        overrides,
        conditions,
        sim,
        speed_margin,
        initial_soc,
        tire_thermal,
        initial_tire_temp_c,
    )?;
    notes.push(format!(
        "T2 stint: {n_laps} laps integrated continuously (one run, no re-seed) — the per-wheel \
         tyre-thermal ring + wear and the battery SoC carry across the start/finish line with no \
         reset (the line table wraps s, so the road geometry + speed reference repeat each lap)."
    ));
    let mut solver = outlap_transient::TransientSolver::new(blocks, line, &interner, cfg);
    if let Some((pack, state)) = pack {
        solver = solver.with_slow_stack(Box::new(PackSlowStack { pack, state }));
    }
    if let Some(shifter) = shifter {
        solver = solver.with_shifter(shifter);
    }
    if let Some(stack) = tire_stack {
        solver = solver.with_tire_thermal(stack);
    }

    let (lap, lap_end_idx) = solver.run_laps(length, n_laps, MAX_TRANSIENT_STEPS);
    let diverged = solver.diverged();
    let provenance = solver.provenance();
    let laps_done = lap_end_idx.len();
    let completed = laps_done == n_laps && !diverged;
    if diverged {
        notes.push(format!(
            "T2 stint diverged after {laps_done} of {n_laps} laps: the closed loop left the physical \
             envelope (a spin the driver could not catch). Only the completed laps are reported. Try \
             a lower `speed_margin`"
        ));
    } else if !completed {
        notes.push(format!(
            "T2 stint reached only {laps_done} of {n_laps} laps within {MAX_TRANSIENT_STEPS} steps"
        ));
    }

    let has_slow = !lap.state_of_charge.is_empty();
    let has_tire = !lap.tire_surface_c.is_empty();
    let mut lap_time_s = Vec::with_capacity(laps_done);
    let (mut surf, mut carc, mut gas) = (Vec::new(), Vec::new(), Vec::new());
    let (mut wear, mut dmg, mut grip, mut peak) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut soc, mut temp) = (Vec::new(), Vec::new());
    let mut prev_t = 0.0;
    for (k, &end_idx) in lap_end_idx.iter().enumerate() {
        let start_idx = if k == 0 { 0 } else { lap_end_idx[k - 1] };
        lap_time_s.push(lap.t[end_idx] - prev_t);
        prev_t = lap.t[end_idx];
        if has_tire {
            surf.extend_from_slice(&lap.tire_surface_c[end_idx]);
            carc.extend_from_slice(&lap.tire_carcass_c[end_idx]);
            gas.extend_from_slice(&lap.tire_gas_c[end_idx]);
            wear.extend_from_slice(&lap.tire_wear_mm[end_idx]);
            dmg.extend_from_slice(&lap.tire_damage[end_idx]);
            grip.extend_from_slice(&lap.tire_grip[end_idx]);
            let mut lap_peak = [f64::MIN; 4];
            for row in &lap.tire_surface_c[start_idx..=end_idx] {
                for (w, &val) in row.iter().enumerate() {
                    lap_peak[w] = lap_peak[w].max(val);
                }
            }
            peak.extend_from_slice(&lap_peak);
        }
        if has_slow {
            soc.push(lap.state_of_charge[end_idx]);
            temp.push(lap.pack_temp_c[end_idx]);
        }
    }

    Ok(TransientStint {
        n_laps: laps_done,
        requested_laps: n_laps,
        lap_time_s,
        tire_surface_c: has_tire.then_some(surf),
        tire_carcass_c: has_tire.then_some(carc),
        tire_gas_c: has_tire.then_some(gas),
        tire_wear_mm: has_tire.then_some(wear),
        tire_damage: has_tire.then_some(dmg),
        tire_grip: has_tire.then_some(grip),
        tire_peak_surface_c: has_tire.then_some(peak),
        state_of_charge: has_slow.then_some(soc),
        pack_temp_c: has_slow.then_some(temp),
        tier: "t2".to_owned(),
        fz_coupling: fz_coupling_str(provenance.fz_coupling),
        flat_track: flat,
        dt_s: provenance.dt_s,
        integrator_order: provenance.integrator_order,
        speed_margin,
        completed,
        wheels: WHEEL_ORDER.iter().map(|s| (*s).to_owned()).collect(),
        resolved_hash,
        notes,
    })
}
