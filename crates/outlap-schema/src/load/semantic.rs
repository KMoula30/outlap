// SPDX-License-Identifier: AGPL-3.0-only
//! Stage 6 — range/consistency checks on the typed models, plus per-document checks for the
//! referenced `.ptm`/`.tyr`/`.emotor` files. Spans are recovered from the [`SpanIndex`].

use crate::diagnostics::{Sources, SrcSpan};
use crate::error::{Result, SchemaError};
use crate::load::report::ReportEntry;
use crate::tree::SpanIndex;
use crate::{
    conditions::Conditions, emotor::Emotor, ptm::Ptm, sim::Sim, track::TrackDoc, tyr,
    vehicle::Vehicle,
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

    // ERS.
    if let Some(ers) = &spec.ers {
        check_soc_window(ers.es.soc_window, "/ers/es/soc_window", &s, sources)?;
        check_taper(
            &ers.deployment.taper_vs_speed,
            "/ers/deployment/taper_vs_speed",
            &s,
            sources,
        )?;
        if let Some(om) = &ers.override_mode {
            check_taper(
                &om.taper_vs_speed,
                "/ers/override_mode/taper_vs_speed",
                &s,
                sources,
            )?;
        }
    }

    Ok(())
}

fn check_drivetrain(spec: &Vehicle, s: &Spans, sources: &Sources) -> Result<()> {
    use crate::vehicle::{Coupler, DiffKind};
    for (ui, unit) in spec.drivetrain.units.iter().enumerate() {
        for (pi, coupler) in unit.path.iter().enumerate() {
            let base = format!("/drivetrain/units/{ui}/path/{pi}/diff");
            if let Coupler::Diff(diff) = coupler {
                let needs_preload = matches!(diff.kind, DiffKind::Lsd | DiffKind::Locked);
                if needs_preload && diff.preload_nm.is_none() {
                    return Err(SchemaError::semantic(
                        sources,
                        s.at(&base),
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

fn check_soc_window(window: [f64; 2], ptr: &str, s: &Spans, sources: &Sources) -> Result<()> {
    let [lo, hi] = window;
    if !(0.0..=1.0).contains(&lo) || !(0.0..=1.0).contains(&hi) {
        return Err(SchemaError::semantic(
            sources,
            s.at(ptr),
            "`soc_window` bounds must lie in [0, 1]",
            None,
        ));
    }
    if lo >= hi {
        return Err(SchemaError::semantic(
            sources,
            s.at(ptr),
            "`soc_window` must be ascending (`[min, max]` with min < max)",
            None,
        ));
    }
    Ok(())
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
        (Some(_), None) | (None, Some(_)) => {}
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
    Ok(())
}

/// Semantic checks for an `.emotor` document.
pub fn check_emotor(
    e: &Emotor,
    index: &SpanIndex,
    sources: &Sources,
    file: crate::diagnostics::SourceId,
) -> Result<()> {
    let s = Spans { index, file };
    unit_interval(
        e.loss_routing.winding_split,
        "loss_routing.winding_split",
        "/loss_routing/winding_split",
        &s,
        sources,
    )?;
    for (label, node, base) in [
        ("winding", &e.nodes.winding, "/nodes/winding"),
        ("case", &e.nodes.case, "/nodes/case"),
    ] {
        if node.t_warn_c > node.t_max_c {
            return Err(SchemaError::semantic(
                sources,
                s.at(base),
                format!("`{label}.t_warn_c` must not exceed `t_max_c`"),
                None,
            ));
        }
    }
    Ok(())
}
