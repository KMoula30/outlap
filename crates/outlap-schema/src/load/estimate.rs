// SPDX-License-Identifier: AGPL-3.0-only
//! Stage 8 — fill estimable parameters from documented heuristics, recording each in the report
//! and in provenance ([`Origin::Estimated`]). Nothing is silent (#41).

use crate::load::provenance::{Origin, ProvenanceMap};
use crate::load::report::ReportEntry;
use crate::vehicle::{AxleKc, Vehicle};

/// Nominal front static ride height when unspecified, m (a generic downforce-car design height).
const FRONT_NOMINAL_RIDE_HEIGHT_M: f64 = 0.030;
/// Nominal rear static ride height when unspecified, m (rearward rake, larger than the front).
const REAR_NOMINAL_RIDE_HEIGHT_M: f64 = 0.050;

/// Fill estimable fields on a resolved vehicle in place.
pub fn estimate(spec: &mut Vehicle, prov: &mut ProvenanceMap, estimated: &mut Vec<ReportEntry>) {
    // Axle-nominal static ride heights when absent: a downforce car sits lower with rearward rake;
    // a road car sits far higher, but there the constant-aero path ignores the value entirely.
    estimate_axle(
        &mut spec.suspension.front,
        "/suspension/front",
        FRONT_NOMINAL_RIDE_HEIGHT_M,
        prov,
        estimated,
    );
    estimate_axle(
        &mut spec.suspension.rear,
        "/suspension/rear",
        REAR_NOMINAL_RIDE_HEIGHT_M,
        prov,
        estimated,
    );

    // `per_lap_deploy_mj` is deliberately NOT estimated: the 2026 F1 regulations impose no per-lap
    // deployment budget (C5.2 — absence verified), so an absent value means "unbounded", and any
    // back-filled number would become a phantom cap the moment budgets are enforced (M6/PR1).
    if let Some(policy) = &mut spec.policy {
        if let Some(om) = &mut policy.override_mode {
            if om.extra_energy_per_lap_mj.is_none() {
                om.extra_energy_per_lap_mj = Some(0.0);
                record(
                    prov,
                    estimated,
                    "/policy/override_mode/extra_energy_per_lap_mj",
                    "override_extra_energy_zero",
                    "assumed 0 MJ extra override energy".into(),
                );
            }
        }
    }

    estimate_driver(spec.driver.as_ref(), prov, estimated);
}

/// Surface the ideal-driver gains that fall back to the MacAdam/PI literature defaults (Decision #8).
/// This records-only (it does not fill the spec, so the resolved-set hash is unchanged for a vehicle
/// with no `driver:` block) — the block assembly applies the same defaults via the schema accessors.
fn estimate_driver(
    driver: Option<&crate::vehicle::Driver>,
    prov: &mut ProvenanceMap,
    estimated: &mut Vec<ReportEntry>,
) {
    use crate::vehicle::Driver;
    // (missing?, pointer, heuristic, default value, what the default is).
    let d = driver;
    let fields: [(bool, &str, &'static str, f64, &str); 10] = [
        (
            d.is_none_or(|v| v.preview_time_s.is_none()),
            "/driver/preview_time_s",
            "driver_preview_time",
            Driver::DEFAULT_PREVIEW_TIME_S,
            "MacAdam preview time, s",
        ),
        (
            d.is_none_or(|v| v.preview_gain.is_none()),
            "/driver/preview_gain",
            "driver_preview_gain",
            Driver::DEFAULT_PREVIEW_GAIN,
            "preview steer gain, rad/m",
        ),
        (
            d.is_none_or(|v| v.heading_gain.is_none()),
            "/driver/heading_gain",
            "driver_heading_gain",
            Driver::DEFAULT_HEADING_GAIN,
            "heading-error steer gain, rad/rad",
        ),
        (
            d.is_none_or(|v| v.yaw_damping.is_none()),
            "/driver/yaw_damping",
            "driver_yaw_damping",
            Driver::DEFAULT_YAW_DAMPING,
            "yaw-rate damping, rad/(rad/s)",
        ),
        (
            d.is_none_or(|v| v.max_steer_rad.is_none()),
            "/driver/max_steer_rad",
            "driver_max_steer",
            Driver::DEFAULT_MAX_STEER_RAD,
            "max road-wheel steer, rad",
        ),
        (
            d.is_none_or(|v| v.speed_kp.is_none()),
            "/driver/speed_kp",
            "driver_speed_kp",
            Driver::DEFAULT_SPEED_KP,
            "speed PI proportional gain, pedal/(m/s)",
        ),
        (
            d.is_none_or(|v| v.speed_ki.is_none()),
            "/driver/speed_ki",
            "driver_speed_ki",
            Driver::DEFAULT_SPEED_KI,
            "speed PI integral gain, pedal/(m/s·s)",
        ),
        (
            d.is_none_or(|v| v.ff_accel_scale_mps2.is_none()),
            "/driver/ff_accel_scale_mps2",
            "driver_ff_accel_scale",
            Driver::DEFAULT_FF_ACCEL_SCALE_MPS2,
            "feed-forward usable accel (gg headroom), m/s²",
        ),
        (
            d.is_none_or(|v| v.stability_slip_limit_rad.is_none()),
            "/driver/stability_slip_limit_rad",
            "driver_stability_slip_limit",
            Driver::DEFAULT_STABILITY_SLIP_LIMIT_RAD,
            "sideslip stability-cut threshold, rad",
        ),
        (
            d.is_none_or(|v| v.stability_slip_gain.is_none()),
            "/driver/stability_slip_gain",
            "driver_stability_slip_gain",
            Driver::DEFAULT_STABILITY_SLIP_GAIN,
            "sideslip stability-cut rate, 1/rad",
        ),
    ];
    for (missing, ptr, heuristic, value, what) in fields {
        if missing {
            record(
                prov,
                estimated,
                ptr,
                heuristic,
                format!("literature default {value} — {what} (tuned on limebeer_2014_f1)"),
            );
        }
    }
}

fn estimate_axle(
    axle: &mut AxleKc,
    base: &str,
    nominal_ride_height_m: f64,
    prov: &mut ProvenanceMap,
    estimated: &mut Vec<ReportEntry>,
) {
    if axle.static_ride_height_m.is_none() {
        axle.static_ride_height_m = Some(nominal_ride_height_m);
        record(
            prov,
            estimated,
            &format!("{base}/static_ride_height_m"),
            "static_ride_height_nominal",
            format!(
                "assumed {} mm nominal (only used by the ride-height aero map)",
                nominal_ride_height_m * 1000.0
            ),
        );
    }
    if axle.anti_dive.is_none() {
        axle.anti_dive = Some(0.0);
        record(
            prov,
            estimated,
            &format!("{base}/anti_dive"),
            "anti_dive_zero",
            "assumed 0 (no anti-dive geometry)".into(),
        );
    }
    if axle.anti_squat.is_none() {
        axle.anti_squat = Some(0.0);
        record(
            prov,
            estimated,
            &format!("{base}/anti_squat"),
            "anti_squat_zero",
            "assumed 0 (no anti-squat geometry)".into(),
        );
    }
    if axle.camber_map.is_none() {
        record(
            prov,
            estimated,
            &format!("{base}/camber_map"),
            "camber_identity",
            "no camber map — assumed zero camber change with travel".into(),
        );
    }
    if axle.toe_map.is_none() {
        record(
            prov,
            estimated,
            &format!("{base}/toe_map"),
            "toe_identity",
            "no toe map — assumed zero toe change with travel".into(),
        );
    }
}

fn record(
    prov: &mut ProvenanceMap,
    estimated: &mut Vec<ReportEntry>,
    pointer: &str,
    heuristic: &'static str,
    detail: String,
) {
    prov.set(pointer, Origin::Estimated { heuristic });
    estimated.push(ReportEntry::new(pointer, detail));
}
