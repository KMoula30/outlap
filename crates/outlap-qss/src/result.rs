// SPDX-License-Identifier: AGPL-3.0-only
//! T0 solve outputs: the reusable [`T0Workspace`] and the owned [`LapResult`].

use crate::path::T0Path;

/// Which racing line a lap was run on (recorded in every result, §6.3).
#[derive(Clone, Debug, PartialEq)]
pub enum LineDescriptor {
    /// The track centerline.
    Centerline,
    /// A generated minimum-curvature line.
    MinCurvature {
        /// Sampling step used, metres.
        ds_m: f64,
        /// Re-linearization iterations run.
        iterations: usize,
    },
    /// A user-supplied line file.
    File {
        /// The line file path.
        path: String,
    },
}

/// Pre-allocated scratch for the zero-allocation solve. Reuse across laps/variants.
#[derive(Clone, Debug)]
pub struct T0Workspace {
    /// Curvature-limited speed per station, m/s.
    pub v_lim: Vec<f64>,
    /// Solved speed per station, m/s.
    pub v: Vec<f64>,
}

impl T0Workspace {
    /// Allocate a workspace sized for `path`.
    pub fn for_path(path: &T0Path) -> Self {
        Self {
            v_lim: vec![0.0; path.len()],
            v: vec![0.0; path.len()],
        }
    }

    /// The number of stations this workspace is sized for.
    pub fn len(&self) -> usize {
        self.v.len()
    }

    /// Whether the workspace has no stations.
    pub fn is_empty(&self) -> bool {
        self.v.is_empty()
    }
}

/// A solved T0 lap: SoA channels plus the lap time and provenance.
#[derive(Clone, Debug)]
pub struct LapResult {
    /// Arc-length stations, metres.
    pub s: Vec<f64>,
    /// Speed, m/s.
    pub v: Vec<f64>,
    /// Longitudinal acceleration, m/s².
    pub ax: Vec<f64>,
    /// Lateral acceleration (ISO 8855, `+` left), m/s².
    pub ay: Vec<f64>,
    /// Cumulative time at each station, s.
    pub t: Vec<f64>,
    /// Total lap time, s.
    pub lap_time_s: f64,
    /// Which line this lap ran on.
    pub line: LineDescriptor,
    /// Resolved-model hash (records which car spec produced this result).
    pub resolved_hash: String,
    /// T0 simplifications/degradations (nothing silent).
    pub notes: Vec<String>,
}
