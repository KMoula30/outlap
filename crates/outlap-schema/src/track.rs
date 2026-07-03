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
    /// Optional sparse banking keypoints, interpolated in `s` (§9.3). When present they OVERRIDE
    /// the centerline's `banking_deg` column (the importer writes keypoints when per-row DEM
    /// banking is not available).
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
