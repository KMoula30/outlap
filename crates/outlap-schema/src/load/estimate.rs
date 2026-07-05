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

    if let Some(ers) = &mut spec.ers {
        if ers.deployment.per_lap_deploy_mj.is_none() {
            // Heuristic: assume the full usable store can be deployed each lap.
            let v = ers.es.capacity_mj;
            ers.deployment.per_lap_deploy_mj = Some(v);
            record(
                prov,
                estimated,
                "/ers/deployment/per_lap_deploy_mj",
                "per_lap_deploy_capacity",
                format!("assumed = usable capacity ({v} MJ)"),
            );
        }
        if let Some(om) = &mut ers.override_mode {
            if om.extra_energy_per_lap_mj.is_none() {
                om.extra_energy_per_lap_mj = Some(0.0);
                record(
                    prov,
                    estimated,
                    "/ers/override_mode/extra_energy_per_lap_mj",
                    "override_extra_energy_zero",
                    "assumed 0 MJ extra override energy".into(),
                );
            }
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
