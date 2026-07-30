// SPDX-License-Identifier: AGPL-3.0-only
//! The `track.yaml` schema (§9.3) — the first open **3D** racetrack format (Locked Decision #13).
//!
//! `track.yaml` is a thin descriptor that points at a `centerline.csv` sidecar (parsed by
//! [`centerline`](crate::centerline)) and carries loop topology (`closed`), optional sparse banking
//! keypoints, and provenance/accuracy metadata (Decision #13 forces DEM fusion, so per-track
//! provenance matters). The geometry itself — spline fit, κ(s), grade, banking, the road frame —
//! is built by the `outlap-track` crate from this document plus the centerline.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::refs::CenterlineRef;
use crate::version::SchemaVersion;

/// MINOR that introduced the banking-conflict rule (`banking_keypoints` may not accompany a
/// non-zero dense `banking_deg` column).
///
/// Under `track/1.0` the two forms were documented as coexisting, with keypoints overriding the
/// column, so rejecting that combination outright would reject input a past consumer gave meaning
/// to — a MAJOR-shaped change. Gating the check on the declared MINOR keeps `track/1.0`
/// documents loading exactly as before while `track/1.1` opts into the stricter rule.
pub const TRACK_MINOR_BANKING_CONFLICT: u16 = 1;

/// MINOR that introduced the **no-data** banking convention.
///
/// From `track/1.2` the `banking_deg` column separates two facts a single number used to
/// conflate: `0.0` is a *measurement* ("this station is flat") and `NaN` is an *absence* ("no
/// banking was resolved here" — the LiDAR cross-section did not clear its signal-to-noise gate).
/// That makes the two forms compose instead of compete: `banking_keypoints` fill the absences,
/// measured values win where they exist, and a hand-annotated corner can no longer overwrite a
/// real flat measurement. Absences left uncovered by keypoints are driven as flat and counted,
/// so the loaded-model report can say so rather than the geometry quietly implying it.
///
/// Older files are untouched: under `1.0` keypoints override the column outright, and `1.1`
/// rejects the combination (see [`TRACK_MINOR_BANKING_CONFLICT`]).
pub const TRACK_MINOR_BANKING_NODATA: u16 = 2;

/// A track descriptor: topology, the centerline reference, optional banking keypoints, and meta.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TrackDoc {
    /// Schema version, e.g. `track/1.0`.
    pub schema: SchemaVersion,
    /// Human-readable circuit name.
    pub name: String,
    /// Whether the track is a closed loop (periodic spline + closure check) or point-to-point.
    #[serde(default = "default_closed")]
    pub closed: bool,
    /// Reference to the `centerline.csv` sidecar (columns per §9.3).
    pub centerline: CenterlineRef,
    /// Optional sparse banking keypoints, interpolated in `s` (§9.3) — the hand-annotation path
    /// for corners the dense `banking_deg` column does not resolve.
    ///
    /// How the two combine depends on the declared MINOR, because the column's meaning changed.
    /// From [`TRACK_MINOR_BANKING_NODATA`] the keypoints **fill** its `NaN` (no-data) stations
    /// and measured values win everywhere else, so the two compose. `track/1.1` rejects a
    /// document carrying keypoints alongside any non-zero column value
    /// ([`TRACK_MINOR_BANKING_CONFLICT`]); `track/1.0` lets keypoints override the column
    /// outright.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub banking_keypoints: Vec<BankingKeypoint>,
    /// Provenance / accuracy metadata.
    #[serde(default)]
    pub meta: TrackMeta,
}

/// A sparse banking keypoint: banking angle at an arc-length station.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct BankingKeypoint {
    /// Arc-length station along the centerline, metres.
    pub s_m: f64,
    /// Banking angle at this station, degrees (positive raises the left/outside edge).
    pub banking_deg: f64,
}

/// Provenance and accuracy metadata for a track (§9.3, Decision #13).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct TrackMeta {
    /// How the centerline was sourced, e.g. `osm+dem`, `tumftm`, `survey`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    /// Digital elevation model used to fuse `z`, e.g. `copernicus-glo-30`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dem: Option<String>,
    /// Accuracy class of the geometry.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub accuracy_class: Option<AccuracyClass>,
    /// Required attribution string for redistributable sources (ODbL/Copernicus).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub attribution: Option<String>,
    /// Free-form notes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
    /// Where per-row widths came from (track/1.1, MT): `orthophoto` (edge-traced, U4),
    /// `declared` (an explicit `--half-width` degraded import — honest, recorded, never
    /// silent), `tumftm` (measured upstream), `survey`, …
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub width_source: Option<String>,
    /// SHA-256 (hex) of the width control-point CSV the edge trace ran with — pins the
    /// committed hand-QA input so a re-run is checkable against the manifest (KTD7).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub width_control_points_sha: Option<String>,
    /// LiDAR DTM dataset + version that fused `z`/banking, e.g.
    /// `icgc-lidar-territorial-dtm v3.1` (track/1.1, MT).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lidar_dataset: Option<String>,
    /// LiDAR tile IDs consumed by the fusion (mirrors the track dir's input manifest).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lidar_tiles: Option<Vec<String>>,
    /// Fitted georeference transform (track ENU frame → real-world), recorded as a compact
    /// string when telemetry georeferencing supplied one; the numeric transform of record
    /// lives in the reference-metrics CSV header (KTD7).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub georef_transform: Option<String>,
    /// Version of the importer build that produced this track dir (track/1.1, KTD7).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub importer_version: Option<String>,
    /// Which import stages ran (`base`, `widths`, `lidar`, `telemetry-audit`) — the honest
    /// record `accuracy_class` derives from (track/1.1, KTD10).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stages: Option<Vec<String>>,
}

/// Geometry accuracy class (§9.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub enum AccuracyClass {
    /// Surveyed / high-precision.
    A,
    /// DEM-fused (typical OSM+DEM import).
    B,
    /// Estimated / hand-annotated.
    C,
}

fn default_closed() -> bool {
    true
}
