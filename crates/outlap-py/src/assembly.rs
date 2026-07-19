// SPDX-License-Identifier: AGPL-3.0-only
//! Loading + building the solver couplings (packs, envelopes, tyre marches, sidecars).

use crate::prelude::*;

/// Build the `Sim` for a run: the vehicle-dir `sim.yaml` (or defaults), deep-merged with the `sim`
/// override dict, then the `tier=` convenience override. A missing `sim.yaml` is fine (defaults); a
/// present-but-broken one is a real error.
pub(crate) fn build_sim(
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
    // `fz_coupling` is Option<FzCoupling> (None = tier-resolved auto), so the serialized base
    // omits it too — inject a null so the documented `sim={"fz_coupling": "fixed_point"}` works.
    if let Some(o) = value.as_object_mut() {
        o.entry("fz_coupling").or_insert(serde_json::Value::Null);
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
pub(crate) fn merge_json(
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
pub(crate) fn load_sidecar_bytes(
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
pub(crate) fn fnv1a(seed: u64, bytes: &[u8]) -> u64 {
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
pub(crate) fn install_sidecars(
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
pub(crate) fn load_pack(
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

/// Assemble the electro slow-state stack per the typed [`plan_slow_stack`] rules (the pure
/// eligibility/pairing logic lives in `outlap-qss`; this function performs exactly the IO the
/// plan names). The machine-thermal network is OPTIONAL (M6 PR2): a pack marches without one.
pub(crate) fn build_slow_stack(
    resolved: &ResolvedVehicle,
    vl: &FsLoader,
    conditions: &Conditions,
    initial_soc: Option<f64>,
    notes: &mut Vec<String>,
) -> PyResult<Option<(Option<MachineThermal>, Pack, PackState)>> {
    let plan = outlap_qss::plan_slow_stack(&resolved.spec);
    let outlap_qss::SlowStackPlan::Pack {
        thermal: pairing,
        notes: plan_notes,
        ..
    } = plan
    else {
        return Ok(None);
    };
    let Some((pack, mut pack_state)) = load_pack(resolved, vl, notes)? else {
        return Ok(None);
    };
    // Pack SoC seed. An explicit `initial_soc` (validated in [0, 1] at the entry point) wins in
    // every case; otherwise: MID-window for an ERS car (D-M6-10) — matching the T2
    // `prepare_transient` seed, so the tiers agree by default and the pack can actually accept
    // harvest. A no-`ers:` mapped EV (discharge-only QSS march) with NO explicit seed keeps
    // `Pack::assemble`'s top-of-window default so its lap stays BYTE-IDENTICAL to v0.3.0 —
    // mid-window would buy that car nothing and would move an established golden/band (the critical
    // no-ers bit-identity invariant).
    let [lo, hi] = pack.soc_window();
    if let Some(soc) = initial_soc {
        pack_state.soc = soc;
        let clamped = soc.clamp(lo, hi);
        if (clamped - soc).abs() > 1e-12 {
            notes.push(format!(
                "QSS pack seeded at the requested {:.0}% state of charge — OUTSIDE its usable \
                 window [{lo:.2}, {hi:.2}]; the window derate will clamp deploy/harvest at the \
                 boundary",
                soc * 100.0
            ));
        } else {
            notes.push(format!(
                "QSS pack seeded at the requested {:.0}% state of charge (initial_soc)",
                soc * 100.0
            ));
        }
    } else if resolved.spec.ers.is_some() {
        pack_state.soc = 0.5 * (lo + hi);
        notes.push(format!(
            "QSS pack seeded at {:.0}% state of charge, the middle of its usable window \
             [{lo:.2}, {hi:.2}] (estimated — pass `initial_soc` to pick a stint state); a pack at \
             the top of its window accepts no charge and recovers nothing",
            pack_state.soc * 100.0
        ));
    }
    notes.extend(plan_notes);
    let Some(pairing) = pairing else {
        return Ok(Some((None, pack, pack_state)));
    };
    let em = match outlap_schema::load::load_emotor(&pairing.emotor_path, vl) {
        Ok(em) => em,
        Err(e) if is_not_found(&e) => {
            notes.push(format!(
                "machine thermal `{}` not present — the pack marches without a thermal network",
                pairing.emotor_path
            ));
            return Ok(Some((None, pack, pack_state)));
        }
        Err(e) => return Err(schema_err(e)),
    };
    let ptm = match outlap_schema::load::load_ptm(&pairing.ptm_path, vl) {
        Ok(ptm) => ptm,
        // Unreachable in practice (T1 assembly hard-errors on a broken/missing unit source
        // first), but keep the policy symmetric with the battery/emotor refs above.
        Err(e) if is_not_found(&e) => {
            notes.push(format!(
                "drive-unit source `{}` not present — the pack marches without a thermal network",
                pairing.ptm_path
            ));
            return Ok(Some((None, pack, pack_state)));
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
    Ok(Some((Some(thermal), pack, pack_state)))
}

/// Process-level cache of generated g-g-g-v envelopes. Generation is a seconds-scale cold step, so
/// a notebook or sweep running many laps of the same car+grid pays it once. Keyed by the resolved
/// vehicle hash, the session conditions, the envelope grid, and the coupling mode — everything that
/// changes the boundary. Bounded implicitly by the small number of distinct (car, grid) combos a
/// session touches; not evicted (a session is short-lived).
pub(crate) static ENV_CACHE: LazyLock<Mutex<HashMap<String, GgvEnvelope>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// The generated (or cached) envelope for a resolved car + numerics. Envelope generation ignores the
/// flat-track flag (it only reshapes the path), so that is not part of the key.
pub(crate) fn cached_envelope(
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

/// Build the per-wheel tire-thermal ring + wear stack for a T2 lap (M5 PR3) from the vehicle's
/// front/rear `.tyr` thermal + wear blocks and the session air / track-surface temperatures.
///
/// The stack seeds **warm at the grip optimum** (so the first step reproduces the frozen-tyre forces
/// bit-for-bit and the QSS↔T2 hull gate stays valid), then the tyres warm, wear, and degrade over the
/// lap — the grip window `λ_μ`, the wear cliff, and the gas-law inflation pressure feed the per-step
/// force call. Geometry the thermal block does not carry (external tread area, vertical stiffness)
/// comes from the MF6.1 coefficients with documented fallbacks; `Q_hyst` uses a modelling hysteresis
/// factor. The thermal/wear parameters are still synthetic pending the FastF1 calibration (PR7/PR8).
pub(crate) fn build_tire_thermal(
    resolved: &ResolvedVehicle,
    conditions: &Conditions,
    vl: &FsLoader,
    notes: &mut Vec<String>,
) -> PyResult<outlap_transient::TireThermalStack<f64>> {
    let spec = &resolved.spec;
    let (front, _) = load_tyr(spec.tires.front.as_str(), vl).map_err(schema_err)?;
    let (rear, _) = load_tyr(spec.tires.rear.as_str(), vl).map_err(schema_err)?;
    let axle_geom = |t: &outlap_schema::tyr::Tyr| {
        let c = &t.mf61.0;
        // Prefer the structured `.tyr` `vertical` block (tyr/1.2, T3); fall back to the raw
        // `VERTICAL_STIFFNESS` MF6.1 key for back-compat, then the 250 kN/m default downstream.
        let k_z = t
            .vertical
            .as_ref()
            .map(|v| v.stiffness_n_per_m)
            .or_else(|| c.get("VERTICAL_STIFFNESS").copied());
        outlap_transient::AxleGeometry::new(
            c.get("UNLOADED_RADIUS").copied().unwrap_or(0.33),
            c.get("WIDTH").copied(),
            k_z,
        )
    };
    notes.push(
        "T2 tire-thermal stack (M5): a per-wheel reduced Farroni-TRT ring + Archard wear advanced on \
         the decimated slow clock (the third slow subsystem). Tyres seed warm at the grip optimum \
         (frozen-tyre forces at step 0), then warm, wear, and degrade over the lap — the grip window \
         (λ_μ), the wear cliff, and the gas-law pressure feed the force call. Thermal/wear parameters \
         are synthetic placeholders pending FastF1 inverse calibration (M5 PR7/PR8)."
            .to_owned(),
    );
    Ok(outlap_transient::TireThermalStack::new(
        &front.thermal,
        &front.wear,
        &rear.thermal,
        &rear.wear,
        axle_geom(&front),
        axle_geom(&rear),
        conditions.air.temperature_c,
        conditions.track_surface_c,
    ))
}

/// Build the QSS tyre-thermal march (M5 PR5) — the representative front-tyre reduced Farroni-TRT ring
/// with Archard wear the T0/T1 slow-state coupling advances segment-to-segment along the velocity
/// profile, producing the per-station `(T_tire, wear)` the envelope's tyre-state axes index. Uses the
/// same representative front ring the tyre-state envelope is built from (`T1Vehicle::tire_thermal`) and
/// the front-tyre geometry (with the documented racing-slick fallbacks). Seeds warm at the grip optimum
/// so the reference slice reproduces the frozen-tyre lap bit-for-bit, then warms and wears over the lap.
pub(crate) fn build_tire_march(
    t1v: &T1Vehicle,
    resolved: &ResolvedVehicle,
    conditions: &Conditions,
    vl: &FsLoader,
    notes: &mut Vec<String>,
) -> PyResult<TireThermalMarch> {
    let (front, _) = load_tyr(resolved.spec.tires.front.as_str(), vl).map_err(schema_err)?;
    let c = &front.mf61.0;
    // Prefer the structured `.tyr` `vertical` block (tyr/1.2, T3); fall back to the raw
    // `VERTICAL_STIFFNESS` MF6.1 key, then the 250 kN/m default downstream.
    let k_z = front
        .vertical
        .as_ref()
        .map(|v| v.stiffness_n_per_m)
        .or_else(|| c.get("VERTICAL_STIFFNESS").copied());
    notes.push(
        "QSS tyre-thermal march (M5 PR5): a representative front-tyre reduced Farroni-TRT ring + \
         Archard wear advanced segment-to-segment along the velocity profile (the third QSS slow \
         subsystem). The evolving (T_tire, wear) index the g-g-g-v envelope's tyre-state axes, so a \
         QSS lap responds to tyre temperature + wear. Seeds warm at the grip optimum (reference slice \
         reproduces the frozen-tyre lap); thermal/wear parameters are synthetic pending FastF1 inverse \
         calibration (M5 PR7/PR8)."
            .to_owned(),
    );
    Ok(TireThermalMarch::new(
        t1v.tire_thermal().clone(),
        c.get("UNLOADED_RADIUS").copied(),
        c.get("WIDTH").copied(),
        k_z,
        front.thermal.t_opt,
        front.thermal.t_cold,
        conditions.air.temperature_c,
        conditions.track_surface_c,
    ))
}
