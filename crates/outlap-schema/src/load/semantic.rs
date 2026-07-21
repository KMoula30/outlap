// SPDX-License-Identifier: AGPL-3.0-only
//! Stage 6 — range/consistency checks on the typed models, plus per-document checks for the
//! referenced `.ptm`/`.tyr`/`.emotor` files. Spans are recovered from the [`SpanIndex`].

use crate::diagnostics::{Sources, SrcSpan};
use crate::error::{Result, SchemaError};
use crate::load::report::ReportEntry;
use crate::tree::SpanIndex;
use crate::{
    conditions::Conditions, emotor::Emotor, ptm::Ptm, refs::BatteryId, sim::RacelineGenerator,
    sim::Sim, track::TrackDoc, tyr, vehicle::Vehicle,
};

/// Aero map axis names this loader recognizes (unknown names are a semantic error with a hint).
pub const KNOWN_AERO_AXES: &[&str] = &[
    "ride_height_f_mm",
    "ride_height_r_mm",
    "ride_height_mm",
    "yaw_deg",
    "roll_deg",
    "steer_deg",
    "drs_flag",
    "speed_mps",
];

/// A span resolver over one document's index (falls back to a blank span in that file).
struct Spans<'a> {
    index: &'a SpanIndex,
    file: crate::diagnostics::SourceId,
}

impl Spans<'_> {
    fn at(&self, pointer: &str) -> SrcSpan {
        self.index
            .span_for(pointer)
            .unwrap_or_else(|| SrcSpan::blank(self.file))
    }
}

/// Run all semantic checks on a resolved [`Vehicle`].
pub fn check_vehicle(
    spec: &Vehicle,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    let s = Spans { index, file };

    positive(
        spec.chassis.mass_kg,
        "chassis.mass_kg",
        "/chassis/mass_kg",
        &s,
        sources,
    )?;
    positive(
        spec.chassis.wheelbase_m,
        "chassis.wheelbase_m",
        "/chassis/wheelbase_m",
        &s,
        sources,
    )?;
    for (i, t) in spec.chassis.track_m.iter().enumerate() {
        positive(
            *t,
            "chassis.track_m",
            &format!("/chassis/track_m/{i}"),
            &s,
            sources,
        )?;
    }

    unit_interval(
        spec.brakes.balance_bar,
        "brakes.balance_bar",
        "/brakes/balance_bar",
        &s,
        sources,
    )?;
    if let Some(rb) = &spec.brakes.regen_blend {
        unit_interval(
            rb.max_regen_frac,
            "brakes.regen_blend.max_regen_frac",
            "/brakes/regen_blend/max_regen_frac",
            &s,
            sources,
        )?;
    }

    for (label, axle, base) in [
        ("front", &spec.suspension.front, "/suspension/front"),
        ("rear", &spec.suspension.rear, "/suspension/rear"),
    ] {
        unit_interval(
            axle.roll_stiffness_share,
            &format!("suspension.{label}.roll_stiffness_share"),
            &format!("{base}/roll_stiffness_share"),
            &s,
            sources,
        )?;
        positive(
            axle.ride_rate_n_per_m,
            &format!("suspension.{label}.ride_rate_n_per_m"),
            &format!("{base}/ride_rate_n_per_m"),
            &s,
            sources,
        )?;
        // T3 suspension fields (optional; the consumer lands the checks — M6/PR6).
        if let Some(m_u) = axle.unsprung_mass_kg {
            positive(
                m_u,
                &format!("suspension.{label}.unsprung_mass_kg"),
                &format!("{base}/unsprung_mass_kg"),
                &s,
                sources,
            )?;
        }
        for (val, name) in [
            (axle.damper_bump_n_s_per_m, "damper_bump_n_s_per_m"),
            (axle.damper_rebound_n_s_per_m, "damper_rebound_n_s_per_m"),
        ] {
            if let Some(c) = val {
                positive(
                    c,
                    &format!("suspension.{label}.{name}"),
                    &format!("{base}/{name}"),
                    &s,
                    sources,
                )?;
            }
        }
        if let Some(k_arb) = axle.arb_stiffness_n_m_per_rad {
            non_negative(
                k_arb,
                &format!("suspension.{label}.arb_stiffness_n_m_per_rad"),
                &format!("{base}/arb_stiffness_n_m_per_rad"),
                &s,
                sources,
            )?;
        }
        if let Some(bs) = &axle.bumpstop {
            non_negative(
                bs.gap_m,
                &format!("suspension.{label}.bumpstop.gap_m"),
                &format!("{base}/bumpstop/gap_m"),
                &s,
                sources,
            )?;
            positive(
                bs.rate_n_per_m,
                &format!("suspension.{label}.bumpstop.rate_n_per_m"),
                &format!("{base}/bumpstop/rate_n_per_m"),
                &s,
                sources,
            )?;
        }
    }

    // Aero axes must be recognized.
    for (i, axis) in spec.aero.axes.iter().enumerate() {
        if !KNOWN_AERO_AXES.contains(&axis.as_str()) {
            let help = crate::diagnostics::suggest(axis, KNOWN_AERO_AXES.iter().copied())
                .map(|s| format!("did you mean `{s}`?"));
            return Err(SchemaError::semantic(
                sources,
                s.at(&format!("/aero/axes/{i}")),
                format!("unknown aero axis `{axis}`"),
                help,
            ));
        }
    }

    // Differential conditional-requirement + ramp sanity.
    check_drivetrain(spec, &s, sources)?;

    // Ideal-driver gains (optional; range-check whatever is given).
    if let Some(driver) = &spec.driver {
        check_driver(driver, &s, sources)?;
    }

    // Policy overlay (range-check the FULL block — the consumer lands the checks, D-M6-13). The
    // pack-anchored checks (recharge_target inside the governed pack's soc_window, regulatory
    // window ≤ physical pack energy) run at load time when the battery document is available.
    if let Some(policy) = &spec.policy {
        check_policy(policy, &s, sources)?;
    }

    // Fuel (optional; range-check the FULL block — the consumer lands the checks, M6/PR5, §8.1).
    if let Some(fuel) = &spec.fuel {
        check_fuel(fuel, &s, sources)?;
    }

    Ok(())
}

/// Range-check the policy overlay's scalar fields (D-M6-13). Pack-anchored checks (recharge target
/// inside the governed pack's `soc_window`; regulatory window fits the physical pack) run at load
/// time in [`check_policy_pack`] once the battery document is loaded. `governs`/id resolution is in
/// [`check_drivetrain`].
fn check_policy(policy: &crate::vehicle::Policy, s: &Spans, sources: &Sources) -> Result<()> {
    positive(
        policy.regulatory_window_mj,
        "regulatory_window_mj",
        "/policy/regulatory_window_mj",
        s,
        sources,
    )?;
    positive(
        policy.deployment.power_limit_kw,
        "power_limit_kw",
        "/policy/deployment/power_limit_kw",
        s,
        sources,
    )?;
    check_taper(
        &policy.deployment.taper_vs_speed,
        "/policy/deployment/taper_vs_speed",
        s,
        sources,
    )?;
    if let Some(b) = policy.deployment.per_lap_deploy_mj {
        positive(
            b,
            "per_lap_deploy_mj",
            "/policy/deployment/per_lap_deploy_mj",
            s,
            sources,
        )?;
    }
    if let Some(om) = &policy.override_mode {
        positive(
            om.power_limit_kw,
            "power_limit_kw",
            "/policy/override_mode/power_limit_kw",
            s,
            sources,
        )?;
        check_taper(
            &om.taper_vs_speed,
            "/policy/override_mode/taper_vs_speed",
            s,
            sources,
        )?;
        if let Some(e) = om.extra_energy_per_lap_mj {
            non_negative(
                e,
                "extra_energy_per_lap_mj",
                "/policy/override_mode/extra_energy_per_lap_mj",
                s,
                sources,
            )?;
        }
    }
    positive(
        policy.recovery.braking_power_limit_kw,
        "braking_power_limit_kw",
        "/policy/recovery/braking_power_limit_kw",
        s,
        sources,
    )?;
    non_negative(
        policy.recovery.per_lap_harvest_mj,
        "per_lap_harvest_mj",
        "/policy/recovery/per_lap_harvest_mj",
        s,
        sources,
    )?;
    if let Some(t) = policy.recovery.recharge_target_soc {
        // The pack-window containment is re-anchored at load time; here only the [0,1] SoC range.
        unit_interval(
            t,
            "recharge_target_soc",
            "/policy/recovery/recharge_target_soc",
            s,
            sources,
        )?;
    }
    for (v, label, ptr) in [
        (
            policy.recovery.ramp_initial_step_kw,
            "ramp_initial_step_kw",
            "/policy/recovery/ramp_initial_step_kw",
        ),
        (
            policy.recovery.ramp_rate_kw_per_s,
            "ramp_rate_kw_per_s",
            "/policy/recovery/ramp_rate_kw_per_s",
        ),
        (
            policy.recovery.ramp_total_kw,
            "ramp_total_kw",
            "/policy/recovery/ramp_total_kw",
        ),
    ] {
        if let Some(v) = v {
            positive(v, label, ptr, s, sources)?;
        }
    }
    if let Some(f) = policy.elec_mech_factor {
        if !(f > 0.0 && f <= 1.0 && f.is_finite()) {
            return Err(SchemaError::semantic(
                sources,
                s.at("/policy/elec_mech_factor"),
                "`elec_mech_factor` must lie in (0, 1]",
                None,
            ));
        }
    }
    Ok(())
}

/// Range-check the fuel block: masses non-negative, `initial_kg ≤ tank_kg`, LHV positive, and the
/// flow-limit caps positive (§8.1, D-M6-4/5).
fn check_fuel(fuel: &crate::vehicle::Fuel, s: &Spans, sources: &Sources) -> Result<()> {
    non_negative(
        fuel.initial_kg,
        "fuel.initial_kg",
        "/fuel/initial_kg",
        s,
        sources,
    )?;
    positive(fuel.tank_kg, "fuel.tank_kg", "/fuel/tank_kg", s, sources)?;
    if fuel.initial_kg > fuel.tank_kg {
        return Err(SchemaError::semantic(
            sources,
            s.at("/fuel/initial_kg"),
            format!(
                "`fuel.initial_kg` ({}) must not exceed `fuel.tank_kg` ({})",
                fuel.initial_kg, fuel.tank_kg
            ),
            None,
        ));
    }
    positive(
        fuel.lhv_j_per_kg,
        "fuel.lhv_j_per_kg",
        "/fuel/lhv_j_per_kg",
        s,
        sources,
    )?;
    if let Some(fl) = &fuel.flow_limit {
        positive(
            fl.mj_per_h,
            "fuel.flow_limit.mj_per_h",
            "/fuel/flow_limit/mj_per_h",
            s,
            sources,
        )?;
        if let Some(line) = &fl.rpm_line {
            positive(
                line.below_rpm,
                "fuel.flow_limit.rpm_line.below_rpm",
                "/fuel/flow_limit/rpm_line/below_rpm",
                s,
                sources,
            )?;
            positive(
                line.slope_mj_per_h_per_rpm,
                "fuel.flow_limit.rpm_line.slope_mj_per_h_per_rpm",
                "/fuel/flow_limit/rpm_line/slope_mj_per_h_per_rpm",
                s,
                sources,
            )?;
            non_negative(
                line.intercept_mj_per_h,
                "fuel.flow_limit.rpm_line.intercept_mj_per_h",
                "/fuel/flow_limit/rpm_line/intercept_mj_per_h",
                s,
                sources,
            )?;
        }
    }
    Ok(())
}

/// Range-check the ideal-driver gains: preview time and pedal-normalising accel must be positive,
/// the steer saturation must be a physical positive angle below a right angle, and every supplied
/// gain must be finite and non-negative (a negative feedback gain would drive the loop unstable).
fn check_driver(driver: &crate::vehicle::Driver, s: &Spans, sources: &Sources) -> Result<()> {
    if let Some(t) = driver.preview_time_s {
        positive(
            t,
            "driver.preview_time_s",
            "/driver/preview_time_s",
            s,
            sources,
        )?;
    }
    if let Some(a) = driver.ff_accel_scale_mps2 {
        positive(
            a,
            "driver.ff_accel_scale_mps2",
            "/driver/ff_accel_scale_mps2",
            s,
            sources,
        )?;
    }
    if let Some(d) = driver.max_steer_rad {
        if !(d > 0.0 && d < std::f64::consts::FRAC_PI_2) {
            return Err(SchemaError::semantic(
                sources,
                s.at("/driver/max_steer_rad"),
                "`driver.max_steer_rad` must lie in (0, π/2)".to_string(),
                Some("steer is a road-wheel angle in radians (e.g. 0.5 ≈ 28.6°)".into()),
            ));
        }
    }
    for (val, label, ptr) in [
        (
            driver.preview_gain,
            "driver.preview_gain",
            "/driver/preview_gain",
        ),
        (
            driver.heading_gain,
            "driver.heading_gain",
            "/driver/heading_gain",
        ),
        (
            driver.yaw_damping,
            "driver.yaw_damping",
            "/driver/yaw_damping",
        ),
        (driver.speed_kp, "driver.speed_kp", "/driver/speed_kp"),
        (driver.speed_ki, "driver.speed_ki", "/driver/speed_ki"),
        (
            driver.stability_slip_limit_rad,
            "driver.stability_slip_limit_rad",
            "/driver/stability_slip_limit_rad",
        ),
        (
            driver.stability_slip_gain,
            "driver.stability_slip_gain",
            "/driver/stability_slip_gain",
        ),
    ] {
        if let Some(g) = val {
            non_negative(g, label, ptr, s, sources)?;
        }
    }
    Ok(())
}

fn check_drivetrain(spec: &Vehicle, s: &Spans, sources: &Sources) -> Result<()> {
    use crate::vehicle::{Coupler, DiffKind};
    // Differential preload/ramp sanity on BOTH private unit paths and top-level graph couplers.
    let check_diff = |diff: &crate::vehicle::Diff, base: &str| -> Result<()> {
        let needs_preload = matches!(diff.kind, DiffKind::Lsd | DiffKind::Locked);
        if needs_preload && diff.preload_nm.is_none() {
            return Err(SchemaError::semantic(
                sources,
                s.at(base),
                format!(
                    "a `{}` differential requires `preload_nm`",
                    serde_plain_kind(diff.kind)
                ),
                Some("add `preload_nm: <N·m>` to this diff".into()),
            ));
        }
        if diff.ramp.is_some() && !matches!(diff.kind, DiffKind::Lsd) {
            return Err(SchemaError::semantic(
                sources,
                s.at(&format!("{base}/ramp")),
                "`ramp` only applies to an `lsd` differential",
                None,
            ));
        }
        Ok(())
    };
    for (ui, unit) in spec.drivetrain.units.iter().enumerate() {
        for (pi, coupler) in unit.path.iter().enumerate() {
            if let Coupler::Diff(diff) = coupler {
                check_diff(diff, &format!("/drivetrain/units/{ui}/path/{pi}/diff"))?;
            }
        }
    }
    for (ci, edge) in spec.drivetrain.couplers.iter().enumerate() {
        if let Coupler::Diff(diff) = &edge.coupler {
            check_diff(diff, &format!("/drivetrain/couplers/{ci}/diff"))?;
        }
    }

    check_drivetrain_ids(spec, s, sources)?;

    // Torque-vectoring gains (range-check whatever is given; the topology check covers reachability).
    let tv = &spec.drivetrain.control.torque_vectoring;
    if !tv.k_yaw.is_finite() || tv.k_yaw < 0.0 {
        return Err(SchemaError::semantic(
            sources,
            s.at("/drivetrain/control/torque_vectoring/k_yaw"),
            "`k_yaw` must be a finite, non-negative gain (N·m per rad/s)",
            Some("torque vectoring damps toward the reference yaw rate; a negative gain would drive the car unstable".into()),
        ));
    }
    if let Some(cap) = tv.max_yaw_moment_nm {
        positive(
            cap,
            "drivetrain.control.torque_vectoring.max_yaw_moment_nm",
            "/drivetrain/control/torque_vectoring/max_yaw_moment_nm",
            s,
            sources,
        )?;
    }
    check_shift_maps(spec, s, sources)?;
    Ok(())
}

/// Validate the in-document symbol tables (D-M6-13): unit ids unique; node ids (from `output`,
/// coupler `from`/`to`) disjoint from unit ids; every `policy.governs` id resolves to a unit; every
/// `unit.battery` id resolves to a `batteries` map key. All are intra-document checks (no IO);
/// unresolved ids get a did-you-mean over the candidate keys.
fn check_drivetrain_ids(spec: &Vehicle, s: &Spans, sources: &Sources) -> Result<()> {
    use crate::diagnostics::suggest;
    let dt = &spec.drivetrain;

    // Unit ids: unique.
    let mut unit_ids: std::collections::HashSet<&str> = std::collections::HashSet::new();
    for (ui, unit) in dt.units.iter().enumerate() {
        if !unit_ids.insert(unit.id.as_str()) {
            return Err(SchemaError::semantic(
                sources,
                s.at(&format!("/drivetrain/units/{ui}/id")),
                format!("duplicate drive-unit id `{}`", unit.id),
                Some("each `drivetrain.units[].id` must be unique".into()),
            ));
        }
    }

    // Node ids (implicitly declared by output / coupler from,to) must be disjoint from unit ids.
    let mut node_refs: Vec<(&str, String)> = Vec::new();
    for (ui, unit) in dt.units.iter().enumerate() {
        if let Some(n) = &unit.output {
            node_refs.push((n.as_str(), format!("/drivetrain/units/{ui}/output")));
        }
    }
    for (ci, edge) in dt.couplers.iter().enumerate() {
        node_refs.push((edge.from.as_str(), format!("/drivetrain/couplers/{ci}/from")));
        if let Some(n) = &edge.to {
            node_refs.push((n.as_str(), format!("/drivetrain/couplers/{ci}/to")));
        }
    }
    for (node, ptr) in &node_refs {
        if unit_ids.contains(node) {
            return Err(SchemaError::semantic(
                sources,
                s.at(ptr),
                format!("node id `{node}` collides with a drive-unit id"),
                Some("node ids and unit ids share one namespace and must be disjoint".into()),
            ));
        }
    }

    // policy.governs → declared unit ids.
    if let Some(policy) = &spec.policy {
        for (gi, id) in policy.governs.iter().enumerate() {
            if !unit_ids.contains(id.as_str()) {
                let hint = suggest(id.as_str(), unit_ids.iter().copied()).map_or_else(
                    || "`policy.governs` must name a declared drive unit".into(),
                    |c| format!("did you mean `{c}`?"),
                );
                return Err(SchemaError::semantic(
                    sources,
                    s.at(&format!("/policy/governs/{gi}")),
                    format!("`policy.governs` references unknown drive-unit id `{id}`"),
                    Some(hint),
                ));
            }
        }
    }

    // unit.battery → batteries map keys.
    let battery_keys: std::collections::HashSet<&str> =
        spec.batteries.keys().map(BatteryId::as_str).collect();
    for (ui, unit) in dt.units.iter().enumerate() {
        if let Some(id) = &unit.battery {
            if !battery_keys.contains(id.as_str()) {
                let hint = suggest(id.as_str(), battery_keys.iter().copied()).map_or_else(
                    || "add a matching entry to the `batteries` map".into(),
                    |c| format!("did you mean `{c}`?"),
                );
                return Err(SchemaError::semantic(
                    sources,
                    s.at(&format!("/drivetrain/units/{ui}/battery")),
                    format!("drive unit `{}` references unknown battery id `{id}`", unit.id),
                    Some(hint),
                ));
            }
        }
    }
    Ok(())
}

/// Range-check the named `shift_maps` (§8.3, D-M6-9): names unique (mirror [`check_emotor`]),
/// `factor` positive+finite, and explicit up-shift speeds strictly increasing, positive, finite,
/// and one fewer than the gear count of the up-shift unit (the runtime picks the unit with the
/// most gears — same rule as the derived default). Absent ⇒ nothing to check (derived default only).
fn check_shift_maps(spec: &Vehicle, s: &Spans, sources: &Sources) -> Result<()> {
    use crate::vehicle::{Coupler, ShiftMapKind};
    if spec.drivetrain.shift_maps.is_empty() {
        return Ok(());
    }
    // The gear count the up-shift schedule addresses = the max gearbox ratio count across all
    // gearboxes, whether on a unit's private path or a top-level graph coupler.
    let unit_gearboxes = spec
        .drivetrain
        .units
        .iter()
        .flat_map(|u| u.path.iter());
    let coupler_gearboxes = spec.drivetrain.couplers.iter().map(|e| &e.coupler);
    let max_gears = unit_gearboxes
        .chain(coupler_gearboxes)
        .filter_map(|c| match c {
            Coupler::Gearbox(g) => Some(g.ratios.len()),
            _ => None,
        })
        .max()
        .unwrap_or(0);
    let mut names: std::collections::HashSet<&str> = std::collections::HashSet::new();
    for (i, map) in spec.drivetrain.shift_maps.iter().enumerate() {
        let base = format!("/drivetrain/shift_maps/{i}");
        if !names.insert(map.name.as_str()) {
            return Err(SchemaError::semantic(
                sources,
                s.at(&format!("{base}/name")),
                format!("duplicate shift-map name `{}`", map.name),
                Some("each `shift_maps` entry needs a unique `name`".into()),
            ));
        }
        match &map.kind {
            ShiftMapKind::Factor(f) => {
                if !(f.is_finite() && *f > 0.0) {
                    return Err(SchemaError::semantic(
                        sources,
                        s.at(&format!("{base}/factor")),
                        format!(
                            "shift-map `{}` `factor` must be a finite, positive multiplier",
                            map.name
                        ),
                        None,
                    ));
                }
            }
            ShiftMapKind::UpshiftSpeedsMps(speeds) => {
                if max_gears >= 2 && speeds.len() != max_gears - 1 {
                    return Err(SchemaError::semantic(
                        sources,
                        s.at(&format!("{base}/upshift_speeds_mps")),
                        format!(
                            "shift-map `{}` has {} up-shift speeds but the gearbox needs {} (gears − 1)",
                            map.name,
                            speeds.len(),
                            max_gears - 1
                        ),
                        None,
                    ));
                }
                let mut prev = f64::NEG_INFINITY;
                for (j, v) in speeds.iter().enumerate() {
                    if !(v.is_finite() && *v > 0.0) {
                        return Err(SchemaError::semantic(
                            sources,
                            s.at(&format!("{base}/upshift_speeds_mps/{j}")),
                            format!(
                                "shift-map `{}` up-shift speed {j} must be finite and positive",
                                map.name
                            ),
                            None,
                        ));
                    }
                    if *v <= prev {
                        return Err(SchemaError::semantic(
                            sources,
                            s.at(&format!("{base}/upshift_speeds_mps/{j}")),
                            format!(
                                "shift-map `{}` up-shift speeds must strictly increase (speed {j} = {v} ≤ previous {prev})",
                                map.name
                            ),
                            None,
                        ));
                    }
                    prev = *v;
                }
            }
        }
    }
    Ok(())
}

fn serde_plain_kind(kind: crate::vehicle::DiffKind) -> &'static str {
    use crate::vehicle::DiffKind;
    match kind {
        DiffKind::Open => "open",
        DiffKind::Locked => "locked",
        DiffKind::Lsd => "lsd",
        DiffKind::Solid => "solid",
    }
}

fn check_taper(
    taper: &crate::vehicle::SpeedTaper,
    ptr: &str,
    s: &Spans,
    sources: &Sources,
) -> Result<()> {
    if taper.speed_kph.len() != taper.power_frac.len() {
        return Err(SchemaError::semantic(
            sources,
            s.at(ptr),
            format!(
                "taper arrays must be equal length (`speed_kph` has {}, `power_frac` has {})",
                taper.speed_kph.len(),
                taper.power_frac.len()
            ),
            None,
        ));
    }
    if taper.speed_kph.is_empty() {
        return Err(SchemaError::semantic(
            sources,
            s.at(ptr),
            "taper must have at least one point",
            None,
        ));
    }
    if !is_ascending(&taper.speed_kph) {
        return Err(SchemaError::semantic(
            sources,
            s.at(&format!("{ptr}/speed_kph")),
            "`speed_kph` must be strictly ascending",
            None,
        ));
    }
    for (i, p) in taper.power_frac.iter().enumerate() {
        if !(0.0..=1.0).contains(p) {
            return Err(SchemaError::semantic(
                sources,
                s.at(&format!("{ptr}/power_frac/{i}")),
                "`power_frac` values must lie in [0, 1]",
                None,
            ));
        }
    }
    // A speed taper de-rates power at speed; a rising fraction is always meaningless (it would
    // grant MORE power at higher speed than the declared limit). Matches the `SpeedTaper` promise.
    if let Some(i) = taper.power_frac.windows(2).position(|w| w[1] > w[0]) {
        return Err(SchemaError::semantic(
            sources,
            s.at(&format!("{ptr}/power_frac/{}", i + 1)),
            "`power_frac` must be monotone non-increasing with speed",
            None,
        ));
    }
    Ok(())
}

fn positive(v: f64, label: &str, ptr: &str, s: &Spans, sources: &Sources) -> Result<()> {
    if v > 0.0 && v.is_finite() {
        Ok(())
    } else {
        Err(SchemaError::semantic(
            sources,
            s.at(ptr),
            format!("`{label}` must be > 0"),
            None,
        ))
    }
}

fn non_negative(v: f64, label: &str, ptr: &str, s: &Spans, sources: &Sources) -> Result<()> {
    if v >= 0.0 && v.is_finite() {
        Ok(())
    } else {
        Err(SchemaError::semantic(
            sources,
            s.at(ptr),
            format!("`{label}` must be ≥ 0 and finite"),
            None,
        ))
    }
}

fn unit_interval(v: f64, label: &str, ptr: &str, s: &Spans, sources: &Sources) -> Result<()> {
    if (0.0..=1.0).contains(&v) {
        Ok(())
    } else {
        Err(SchemaError::semantic(
            sources,
            s.at(ptr),
            format!("`{label}` must lie in [0, 1]"),
            None,
        ))
    }
}

fn is_ascending(xs: &[f64]) -> bool {
    xs.windows(2).all(|w| w[0] < w[1])
}

// --- Referenced-file checks ------------------------------------------------------------------

/// Semantic checks for a `.ptm` document.
pub fn check_ptm(
    ptm: &Ptm,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    let s = Spans { index, file };
    if !is_ascending(&ptm.axes.speed_rpm) {
        return Err(SchemaError::semantic(
            sources,
            s.at("/axes/speed_rpm"),
            "`axes.speed_rpm` must be strictly ascending",
            None,
        ));
    }
    let curve = &ptm.limits.max_torque_nm_vs_speed;
    if curve.speed_rpm.len() != curve.torque_nm.len() || curve.speed_rpm.is_empty() {
        return Err(SchemaError::semantic(
            sources,
            s.at("/limits/max_torque_nm_vs_speed"),
            "`max_torque_nm_vs_speed` must have equal-length, non-empty `speed_rpm`/`torque_nm`",
            None,
        ));
    }
    positive(ptm.mass_kg, "mass_kg", "/mass_kg", &s, sources)?;
    // The optional regen envelope (ptm/1.2) is a *positive-magnitude* braking-torque curve; a signed
    // (negative) curve is the most likely authoring mistake, so name it explicitly.
    if let Some(regen) = &ptm.limits.max_regen_torque_nm_vs_speed {
        if regen.speed_rpm.len() != regen.torque_nm.len() || regen.speed_rpm.is_empty() {
            return Err(SchemaError::semantic(
                sources,
                s.at("/limits/max_regen_torque_nm_vs_speed"),
                "`max_regen_torque_nm_vs_speed` must have equal-length, non-empty `speed_rpm`/`torque_nm`",
                None,
            ));
        }
        if regen.torque_nm.iter().any(|&t| t < 0.0) {
            return Err(SchemaError::semantic(
                sources,
                s.at("/limits/max_regen_torque_nm_vs_speed"),
                "`max_regen_torque_nm_vs_speed` is a positive-magnitude braking envelope; drop the minus sign",
                None,
            ));
        }
    }
    // The optional Vdc axis (ptm/1.1) must be strictly ascending when present.
    if let Some(vdc) = &ptm.axes.vdc_v {
        if vdc.len() < 2 || !is_ascending(vdc) {
            return Err(SchemaError::semantic(
                sources,
                s.at("/axes/vdc_v"),
                "`axes.vdc_v` must have ≥ 2 strictly-ascending breakpoints",
                None,
            ));
        }
    }
    Ok(())
}

/// Semantic checks for a `battery/1.0` document: positive topology/capacity, an ascending SoC
/// window inside `[0, 1]`, and strictly-ascending ECM grid axes.
pub fn check_battery(
    b: &crate::battery::BatteryDoc,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    let s = Spans { index, file };
    if b.topology.ns == 0 || b.topology.np == 0 {
        return Err(SchemaError::semantic(
            sources,
            s.at("/topology"),
            "`topology.ns` and `topology.np` must be positive",
            None,
        ));
    }
    positive(
        b.capacity.q_pack_ah,
        "capacity.q_pack_ah",
        "/capacity/q_pack_ah",
        &s,
        sources,
    )?;
    let [lo, hi] = b.soc_window;
    if !(0.0..=1.0).contains(&lo) || !(0.0..=1.0).contains(&hi) || lo >= hi {
        return Err(SchemaError::semantic(
            sources,
            s.at("/soc_window"),
            "`soc_window` must be `[min, max]` ascending within [0, 1]",
            None,
        ));
    }
    if b.ecm.axes.soc.len() < 2 || !is_ascending(&b.ecm.axes.soc) {
        return Err(SchemaError::semantic(
            sources,
            s.at("/ecm/axes/soc"),
            "`ecm.axes.soc` must have ≥ 2 strictly-ascending breakpoints",
            None,
        ));
    }
    if b.ecm.axes.temp_c.len() < 2 || !is_ascending(&b.ecm.axes.temp_c) {
        return Err(SchemaError::semantic(
            sources,
            s.at("/ecm/axes/temp_c"),
            "`ecm.axes.temp_c` must have ≥ 2 strictly-ascending breakpoints",
            None,
        ));
    }
    positive(
        b.thermal.mass_kg,
        "thermal.mass_kg",
        "/thermal/mass_kg",
        &s,
        sources,
    )?;
    positive(
        b.thermal.cp_j_per_kgk,
        "thermal.cp_j_per_kgk",
        "/thermal/cp_j_per_kgk",
        &s,
        sources,
    )?;
    // The optional charge-acceptance derate (battery/1.1): ascending temperature breakpoints and a
    // factor in [0, 1]. A factor above 1 would *raise* the ceiling above the declared curve.
    if let Some(d) = &b.limits.regen_derate_vs_temp {
        if d.temp_c.len() != d.factor.len() || d.temp_c.len() < 2 || !is_ascending(&d.temp_c) {
            return Err(SchemaError::semantic(
                sources,
                s.at("/limits/regen_derate_vs_temp"),
                "`regen_derate_vs_temp` needs ≥ 2 paired points with strictly-ascending `temp_c`",
                None,
            ));
        }
        if d.factor.iter().any(|&f| !(0.0..=1.0).contains(&f)) {
            return Err(SchemaError::semantic(
                sources,
                s.at("/limits/regen_derate_vs_temp"),
                "`regen_derate_vs_temp.factor` must lie in [0, 1] (it scales the declared ceiling)",
                None,
            ));
        }
    }
    // Only 1 or 2 RC pairs are modelled (a 2nd pair carries the `r2_ohm`/`tau2_s` sidecar columns,
    // battery/1.2). Reported here as a config-surface error, not a bare runtime `Err` deep in the
    // solver assembly.
    if !matches!(b.ecm.rc_pairs, 1 | 2) {
        return Err(SchemaError::semantic(
            sources,
            s.at("/ecm/rc_pairs"),
            "`ecm.rc_pairs` must be 1 or 2",
            None,
        ));
    }
    // The peak-power curves (PR4h consumer-lands-the-checks): each is a paired, equal-length,
    // strictly-ascending-SoC ceiling the runtime fits into a monotone cubic — an unpaired or
    // out-of-order curve would fail deep in `Pack::assemble` with a bare string error.
    let check_power_curve = |p: &crate::battery::PowerVsSoc, ptr: &str| -> Result<()> {
        if p.soc.len() != p.power_w.len() || p.soc.len() < 2 || !is_ascending(&p.soc) {
            return Err(SchemaError::semantic(
                sources,
                s.at(ptr),
                "peak-power curve needs ≥ 2 paired points with strictly-ascending `soc`",
                None,
            ));
        }
        if p.power_w.iter().any(|&w| w < 0.0 || !w.is_finite()) {
            return Err(SchemaError::semantic(
                sources,
                s.at(ptr),
                "peak-power `power_w` must be finite and non-negative (a positive-magnitude ceiling)",
                None,
            ));
        }
        Ok(())
    };
    check_power_curve(
        &b.limits.peak_discharge_power_w_vs_soc,
        "/limits/peak_discharge_power_w_vs_soc",
    )?;
    check_power_curve(
        &b.limits.peak_regen_power_w_vs_soc,
        "/limits/peak_regen_power_w_vs_soc",
    )?;
    // Cell-voltage bounds and the C-rate: an ascending window and a positive C-rate.
    if !(b.limits.cell_v_min.is_finite()
        && b.limits.cell_v_max.is_finite()
        && b.limits.cell_v_min > 0.0
        && b.limits.cell_v_min < b.limits.cell_v_max)
    {
        return Err(SchemaError::semantic(
            sources,
            s.at("/limits"),
            "`cell_v_min` and `cell_v_max` must be positive with `cell_v_min < cell_v_max`",
            None,
        ));
    }
    positive(
        b.limits.max_c_rate,
        "limits.max_c_rate",
        "/limits/max_c_rate",
        &s,
        sources,
    )?;
    Ok(())
}

/// Relative tolerance for the policy regulatory-window vs physical-pack-energy fit (D-M6-13). The
/// pack ECM document is the single source of truth for the physical `soc_window`; the policy's
/// `regulatory_window_mj` (FIA C5.2.9 swing limit) must FIT WITHIN `(soc window span) × e_pack_wh`.
const POLICY_WINDOW_RTOL: f64 = 0.01;

/// Cross-document integrity between a `policy` overlay and the physical pack of the unit it governs
/// (D-M6-13). Anchored on the VEHICLE file's spans; the battery file's values ride in the message.
///
/// The pack `soc_window` is now the SINGLE source of truth (there is no longer a duplicate
/// `ers.es.soc_window` to reconcile). Two checks remain:
/// * the regulatory swing window fits the pack's physical usable-window energy, and
/// * the optional `recharge_target_soc` lies inside the pack's `soc_window`.
///
/// # Errors
/// [`SchemaError::Semantic`] when `regulatory_window_mj` exceeds the pack's usable-window energy
/// within [`POLICY_WINDOW_RTOL`], or when `recharge_target_soc` falls outside the pack window.
pub fn check_policy_pack(
    policy: &crate::vehicle::Policy,
    battery: &crate::battery::BatteryDoc,
    battery_path: &str,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    let s = Spans { index, file };
    let [b_lo, b_hi] = battery.soc_window;
    // The regulatory C5.2.9 on-track swing limit must FIT WITHIN the pack's physical usable-window
    // energy `(window span) × e_pack_wh`: you cannot be allowed to swing more energy than the store
    // physically holds. A physically larger window is permitted (the swing is then clipped below the
    // physical edge). `e_pack_wh` is load-bearing (the field-semantics policy freezes its meaning).
    let window_mj = (b_hi - b_lo) * battery.capacity.e_pack_wh * 3600.0e-6;
    let declared = policy.regulatory_window_mj;
    if declared > window_mj * (1.0 + POLICY_WINDOW_RTOL) {
        return Err(SchemaError::semantic(
            sources,
            s.at("/policy/regulatory_window_mj"),
            format!(
                "`policy.regulatory_window_mj` = {declared} MJ (the FIA C5.2.9 on-track swing \
                 limit) exceeds what the battery physically holds: (soc window {b_lo}..{b_hi}) × \
                 e_pack_wh {} Wh = {window_mj:.4} MJ (`{battery_path}`) — the swing cannot draw \
                 more energy than the usable window",
                battery.capacity.e_pack_wh
            ),
            Some(format!(
                "lower `regulatory_window_mj` to at most the usable-window energy ({window_mj:.4} \
                 MJ), or size the pack larger — a {declared} MJ swing over [{b_lo}, {b_hi}] needs a \
                 {:.4} MJ total pack",
                declared / (b_hi - b_lo).max(f64::MIN_POSITIVE)
            )),
        ));
    }
    // The recharge target (if set) must lie inside the governed pack's physical soc_window.
    if let Some(t) = policy.recovery.recharge_target_soc {
        if !(b_lo..=b_hi).contains(&t) {
            return Err(SchemaError::semantic(
                sources,
                s.at("/policy/recovery/recharge_target_soc"),
                format!(
                    "`recharge_target_soc` must lie inside the governed pack's soc_window \
                     [{b_lo}, {b_hi}] (`{battery_path}`), got {t}"
                ),
                None,
            ));
        }
    }
    Ok(())
}

/// Semantic checks for a `.tyr` document. Returns non-fatal warnings (unknown MF6.1 keys, a brush
/// block under an old MINOR, a partial MF6.1 force set alongside a brush block).
pub fn check_tyr(
    t: &crate::tyr::Tyr,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
    warnings: &mut Vec<ReportEntry>,
) -> Result<()> {
    let s = Spans { index, file };
    // Structural coefficients are always required.
    for key in tyr::REQUIRED_STRUCTURAL_KEYS {
        if !t.mf61.0.contains_key(*key) {
            return Err(SchemaError::semantic(
                sources,
                s.at("/mf61"),
                format!("MF6.1 block is missing required coefficient `{key}`"),
                None,
            ));
        }
    }
    // The MF6.1 force core is required unless a brush block supplies the force model instead.
    let force_present = tyr::REQUIRED_FORCE_KEYS
        .iter()
        .filter(|k| t.mf61.0.contains_key(**k))
        .count();
    let force_full = force_present == tyr::REQUIRED_FORCE_KEYS.len();
    match (&t.brush, force_full) {
        // No brush and an incomplete force core → hard error (an MF6.1 tire needs all of it).
        (None, false) => {
            let missing = tyr::REQUIRED_FORCE_KEYS
                .iter()
                .find(|k| !t.mf61.0.contains_key(**k))
                .expect("force set is incomplete");
            return Err(SchemaError::semantic(
                sources,
                s.at("/mf61"),
                format!("MF6.1 block is missing required coefficient `{missing}`"),
                Some("a force-model tire needs the full pure-slip core, or add a `brush:` block for a brush-model tire".into()),
            ));
        }
        // Brush present with a partial (non-empty, incomplete) force core → warning.
        (Some(_), false) if force_present > 0 => {
            warnings.push(ReportEntry::new(
                "/mf61",
                "partial MF6.1 force coefficients alongside a `brush` block — the brush model is \
                 used and the incomplete MF6.1 set is ignored",
            ));
        }
        _ => {}
    }
    // A brush block requires `tyr/1.1`; warn if the file declares an older MINOR.
    if t.brush.is_some() && t.schema.minor < tyr::TYR_MINOR_BRUSH {
        warnings.push(ReportEntry::new(
            "/brush",
            format!(
                "`brush` block requires schema `tyr/1.{}` but the file declares `{}`",
                tyr::TYR_MINOR_BRUSH,
                t.schema
            ),
        ));
    }
    // Brush field ranges (a present block must be physical).
    if let Some(b) = &t.brush {
        positive(
            b.c_kappa_n,
            "brush.c_kappa_n",
            "/brush/c_kappa_n",
            &s,
            sources,
        )?;
        positive(
            b.c_alpha_n_per_rad,
            "brush.c_alpha_n_per_rad",
            "/brush/c_alpha_n_per_rad",
            &s,
            sources,
        )?;
        positive(b.mu0, "brush.mu0", "/brush/mu0", &s, sources)?;
        positive(
            b.patch_half_length_m,
            "brush.patch_half_length_m",
            "/brush/patch_half_length_m",
            &s,
            sources,
        )?;
    }
    // Tyre vertical block ranges (T3; a present block must be physical — M6/PR6).
    if let Some(v) = &t.vertical {
        positive(
            v.stiffness_n_per_m,
            "vertical.stiffness_n_per_m",
            "/vertical/stiffness_n_per_m",
            &s,
            sources,
        )?;
        if let Some(c) = v.damping_n_s_per_m {
            non_negative(
                c,
                "vertical.damping_n_s_per_m",
                "/vertical/damping_n_s_per_m",
                &s,
                sources,
            )?;
        }
    }
    // Unknown coefficients → warning with did-you-mean.
    for name in t.mf61.0.keys() {
        if !tyr::KNOWN_MF61_KEYS.contains(&name.as_str()) {
            let hint = crate::diagnostics::suggest(name, tyr::KNOWN_MF61_KEYS.iter().copied())
                .map(|s| format!(" (did you mean `{s}`?)"))
                .unwrap_or_default();
            warnings.push(ReportEntry::new(
                format!("/mf61/{name}"),
                format!("unknown MF6.1 coefficient `{name}`{hint} — carried through unvalidated"),
            ));
        }
    }
    unit_interval(t.thermal.p_t, "thermal.p_t", "/thermal/p_t", &s, sources)?;
    Ok(())
}

/// Semantic checks for a `track.yaml` document (the centerline itself is validated in
/// [`crate::centerline`]). Banking keypoints must have strictly ascending `s_m`.
pub fn check_track(
    t: &TrackDoc,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    let s = Spans { index, file };
    if t.name.trim().is_empty() {
        return Err(SchemaError::semantic(
            sources,
            s.at("/name"),
            "`name` must not be empty",
            None,
        ));
    }
    let ks: Vec<f64> = t.banking_keypoints.iter().map(|k| k.s_m).collect();
    if !ks.is_empty() && !is_ascending(&ks) {
        return Err(SchemaError::semantic(
            sources,
            s.at("/banking_keypoints"),
            "`banking_keypoints` must have strictly ascending `s_m`",
            Some(
                "keypoints are interpolated in arc length; duplicates/reorderings are ambiguous"
                    .into(),
            ),
        ));
    }
    if let Some(first) = t.banking_keypoints.first() {
        if first.s_m < 0.0 {
            return Err(SchemaError::semantic(
                sources,
                s.at("/banking_keypoints/0/s_m"),
                "banking keypoint `s_m` must be >= 0",
                None,
            ));
        }
    }
    Ok(())
}

/// Semantic checks for a `conditions.yaml` document.
pub fn check_conditions(
    c: &Conditions,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    let s = Spans { index, file };
    positive(
        c.air.pressure_hpa,
        "air.pressure_hpa",
        "/air/pressure_hpa",
        &s,
        sources,
    )?;
    if c.wind.speed_mps < 0.0 || !c.wind.speed_mps.is_finite() {
        return Err(SchemaError::semantic(
            sources,
            s.at("/wind/speed_mps"),
            "`wind.speed_mps` must be >= 0",
            None,
        ));
    }
    Ok(())
}

/// Semantic checks for a `sim.yaml` document.
pub fn check_sim(
    sim: &Sim,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    let s = Spans { index, file };
    positive(sim.dt_s, "dt_s", "/dt_s", &s, sources)?;
    let e = &sim.envelope;
    if e.v_points < 2 || e.ax_points < 2 || e.g_normal_points < 1 {
        return Err(SchemaError::semantic(
            sources,
            s.at("/envelope"),
            "`envelope` needs v_points >= 2, ax_points >= 2, g_normal_points >= 1",
            None,
        ));
    }
    // Exactly one racing-line source.
    match (&sim.raceline.generator, &sim.raceline.file) {
        (Some(g), None) => {
            if let RacelineGenerator::TimeWeighted { iterations } = g {
                if *iterations < 1 || *iterations > 16 {
                    return Err(SchemaError::semantic(
                        sources,
                        s.at("/raceline/generator"),
                        "`time_weighted` needs `iterations` in 1..=16 (2–4 is typical)",
                        None,
                    ));
                }
            }
        }
        (None, Some(_)) => {}
        (Some(_), Some(_)) => {
            return Err(SchemaError::semantic(
                sources,
                s.at("/raceline"),
                "`raceline` sets both `generator` and `file`; choose one",
                None,
            ));
        }
        (None, None) => {
            return Err(SchemaError::semantic(
                sources,
                s.at("/raceline"),
                "`raceline` needs either a `generator` or a `file`",
                None,
            ));
        }
    }
    // Split-integrator numerics (transient tiers).
    if sim.slow_decimation < 1 {
        return Err(SchemaError::semantic(
            sources,
            s.at("/slow_decimation"),
            "`slow_decimation` must be >= 1 (the slow clock fires every N fast steps)",
            None,
        ));
    }
    let fp = &sim.fixed_point;
    if !(fp.damping > 0.0 && fp.damping <= 1.0) {
        return Err(SchemaError::semantic(
            sources,
            s.at("/fixed_point/damping"),
            "`fixed_point.damping` must lie in (0, 1] (1.0 = undamped)",
            None,
        ));
    }
    positive(fp.tol, "tol", "/fixed_point/tol", &s, sources)?;
    if fp.max_iter < 1 {
        return Err(SchemaError::semantic(
            sources,
            s.at("/fixed_point/max_iter"),
            "`fixed_point.max_iter` must be >= 1",
            None,
        ));
    }
    Ok(())
}

/// Semantic checks for an `.emotor` document (the data-declared N-node LPTN, §9.5).
pub fn check_emotor(
    e: &Emotor,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    use crate::emotor::{ConvModel, InitialTemp, NodeRole};

    let s = Spans { index, file };
    let err = |ptr: &str, msg: String| Err(SchemaError::semantic(sources, s.at(ptr), msg, None));

    // --- Nodes: ≥2, unique names, valid capacities and limit pairs, exactly one winding. ------
    if e.nodes.len() < 2 {
        return err(
            "/nodes",
            format!(
                "a thermal network needs at least 2 nodes, got {}",
                e.nodes.len()
            ),
        );
    }
    let mut names: std::collections::HashSet<&str> = std::collections::HashSet::new();
    let mut n_winding = 0;
    for (i, node) in e.nodes.iter().enumerate() {
        let ptr = format!("/nodes/{i}");
        if node.name.trim().is_empty() {
            return err(&ptr, "node `name` must not be empty".into());
        }
        if !names.insert(node.name.as_str()) {
            return err(&ptr, format!("duplicate node name `{}`", node.name));
        }
        if let Some(c) = node.c_j_per_k {
            positive(c, "c_j_per_k", &format!("{ptr}/c_j_per_k"), &s, sources)?;
        }
        match (node.t_warn_c, node.t_max_c) {
            (Some(w), Some(m)) if w > m => {
                return err(
                    &ptr,
                    format!("`{}.t_warn_c` must not exceed `t_max_c`", node.name),
                );
            }
            (Some(_), None) | (None, Some(_)) => {
                return err(
                    &ptr,
                    format!(
                        "node `{}` sets only one of t_warn_c/t_max_c — set both (to derate) or neither",
                        node.name
                    ),
                );
            }
            _ => {}
        }
        if node.role == Some(NodeRole::Winding) {
            n_winding += 1;
        }
    }
    if n_winding == 0 {
        return err(
            "/nodes",
            "at least one node must have `role: winding` — it is the required loss target and binds \
             the torque derate"
                .into(),
        );
    }
    let known = |name: &str| names.contains(name);

    // --- Ambient / coolant boundary. ----------------------------------------------------------
    if !known(&e.cooling.ambient_node) {
        return err(
            "/cooling/ambient_node",
            format!("unknown ambient node `{}`", e.cooling.ambient_node),
        );
    }
    if let Some(c) = &e.cooling.coolant {
        if !known(&c.node) {
            return err(
                "/cooling/coolant/node",
                format!("unknown coolant node `{}`", c.node),
            );
        }
        if c.node == e.cooling.ambient_node {
            return err(
                "/cooling/coolant/node",
                "coolant node must differ from the ambient node".into(),
            );
        }
        positive(
            c.rho_cp_mdot_w_per_k,
            "coolant.rho_cp_mdot_w_per_k",
            "/cooling/coolant/rho_cp_mdot_w_per_k",
            &s,
            sources,
        )?;
    }
    if e.cooling.coolant.is_some() && e.cooling.jacket.is_some() {
        return err(
            "/cooling/jacket",
            "declare either `cooling.coolant` or `cooling.jacket`, not both".into(),
        );
    }
    if let Some(j) = &e.cooling.jacket {
        for (label, node) in [
            ("housing_node", &j.housing_node),
            ("coolant_node", &j.coolant_node),
        ] {
            if !known(node) {
                return err(
                    &format!("/cooling/jacket/{label}"),
                    format!("jacket references unknown node `{node}`"),
                );
            }
        }
        if j.coolant_node == e.cooling.ambient_node {
            return err(
                "/cooling/jacket/coolant_node",
                "coolant node must differ from ambient".into(),
            );
        }
        for (v, label) in [
            (j.flow_rate_lps, "flow_rate_lps"),
            (j.channel_width_mm, "channel_width_mm"),
            (j.channel_height_mm, "channel_height_mm"),
            (j.wetted_area_m2, "wetted_area_m2"),
        ] {
            positive(
                v,
                &format!("jacket.{label}"),
                &format!("/cooling/jacket/{label}"),
                &s,
                sources,
            )?;
        }
        if j.channel_count == 0 {
            return err(
                "/cooling/jacket/channel_count",
                "channel_count must be at least 1".into(),
            );
        }
    }
    if let Some(a) = &e.cooling.air_gap {
        for node in [&a.between.0, &a.between.1] {
            if !known(node) {
                return err(
                    "/cooling/air_gap/between",
                    format!("air-gap references unknown node `{node}`"),
                );
            }
        }
        if a.between.0 == a.between.1 {
            return err(
                "/cooling/air_gap/between",
                "air-gap must couple two distinct nodes".into(),
            );
        }
        for (v, label) in [
            (a.rotor_outer_radius_mm, "rotor_outer_radius_mm"),
            (a.gap_mm, "gap_mm"),
            (a.stack_length_mm, "stack_length_mm"),
        ] {
            positive(
                v,
                &format!("air_gap.{label}"),
                &format!("/cooling/air_gap/{label}"),
                &s,
                sources,
            )?;
        }
    }

    // --- Constant conductance edges. ----------------------------------------------------------
    for (i, edge) in e.conductances.iter().enumerate() {
        let ptr = format!("/conductances/{i}");
        let (a, b) = (&edge.between.0, &edge.between.1);
        if !known(a) || !known(b) {
            return err(
                &ptr,
                format!("conductance references unknown node(s) `{a}`/`{b}`"),
            );
        }
        if a == b {
            return err(&ptr, format!("conductance connects node `{a}` to itself"));
        }
        if let Some(g) = edge.w_per_k {
            positive(g, "w_per_k", &format!("{ptr}/w_per_k"), &s, sources)?;
        }
    }

    // --- Convection edges. --------------------------------------------------------------------
    for (i, ce) in e.convection.iter().enumerate() {
        let ptr = format!("/convection/{i}");
        let (a, b) = (&ce.between.0, &ce.between.1);
        if !known(a) || !known(b) {
            return err(
                &ptr,
                format!("convection edge references unknown node(s) `{a}`/`{b}`"),
            );
        }
        if a == b {
            return err(
                &ptr,
                format!("convection edge connects node `{a}` to itself"),
            );
        }
        positive(
            ce.area_m2,
            "area_m2",
            &format!("{ptr}/area_m2"),
            &s,
            sources,
        )?;
        // Guard the correlation geometry parameters that would divide/degenerate.
        match &ce.model {
            ConvModel::AirGap {
                r_gap_m, gap0_m, ..
            } => {
                positive(
                    *r_gap_m,
                    "r_gap_m",
                    &format!("{ptr}/model/air_gap/r_gap_m"),
                    &s,
                    sources,
                )?;
                positive(
                    *gap0_m,
                    "gap0_m",
                    &format!("{ptr}/model/air_gap/gap0_m"),
                    &s,
                    sources,
                )?;
            }
            ConvModel::RotorAir { r_rotor_m, .. } => {
                positive(
                    *r_rotor_m,
                    "r_rotor_m",
                    &format!("{ptr}/model/rotor_air/r_rotor_m"),
                    &s,
                    sources,
                )?;
            }
            ConvModel::ShaftExternal { d_shaft_m } => {
                positive(
                    *d_shaft_m,
                    "d_shaft_m",
                    &format!("{ptr}/model/shaft_external/d_shaft_m"),
                    &s,
                    sources,
                )?;
            }
            ConvModel::LiquidChannel {
                hydraulic_diameter_m,
                fluid,
                ..
            } => {
                positive(
                    *hydraulic_diameter_m,
                    "hydraulic_diameter_m",
                    &format!("{ptr}/model/liquid_channel/hydraulic_diameter_m"),
                    &s,
                    sources,
                )?;
                positive(
                    fluid.nu,
                    "fluid.nu",
                    &format!("{ptr}/model/liquid_channel/fluid/nu"),
                    &s,
                    sources,
                )?;
            }
            ConvModel::FreeConvection { char_length_m, .. } => {
                positive(
                    *char_length_m,
                    "char_length_m",
                    &format!("{ptr}/model/free_convection/char_length_m"),
                    &s,
                    sources,
                )?;
            }
        }
    }

    // --- Loss routing: destination nodes exist, fractions in [0, 1]. --------------------------
    for (i, route) in e.loss_routing.iter().enumerate() {
        let ptr = format!("/loss_routing/{i}");
        if !known(&route.node) {
            return err(
                &ptr,
                format!("loss route targets unknown node `{}`", route.node),
            );
        }
        unit_interval(
            route.fraction,
            "fraction",
            &format!("{ptr}/fraction"),
            &s,
            sources,
        )?;
    }

    // --- Copper feedback + initial temperatures reference existing nodes. ----------------------
    if let Some(cu) = &e.cu_feedback {
        if cu.nodes.is_empty() {
            return err(
                "/cu_feedback/nodes",
                "list at least one winding node for Cu feedback".into(),
            );
        }
        for (i, nd) in cu.nodes.iter().enumerate() {
            if !known(nd) {
                return err(
                    &format!("/cu_feedback/nodes/{i}"),
                    format!("unknown Cu-feedback node `{nd}`"),
                );
            }
        }
        if !cu.alpha_per_k.is_finite() {
            return err(
                "/cu_feedback/alpha_per_k",
                "alpha_per_k must be finite".into(),
            );
        }
    }
    if let Some(InitialTemp::PerNodeC(temps)) = &e.initial_temp {
        for (i, nt) in temps.iter().enumerate() {
            if !known(&nt.node) {
                return err(
                    &format!("/initial_temp/per_node_c/{i}"),
                    format!("initial temperature targets unknown node `{}`", nt.node),
                );
            }
        }
    }
    Ok(())
}
