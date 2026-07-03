// SPDX-License-Identifier: AGPL-3.0-only
//! [`T0Path`] — the per-station geometry the T0 passes run over.
//!
//! Sampled once from a [`Track`] at a uniform arc-length step, it precomputes the road-plane
//! curvatures and the gravity-projection trig so the hot passes touch only `f64` slices. The
//! curvatures follow the 3D-ribbon projection (Perantoni & Limebeer 2015; Lovato & Massaro 2022):
//!
//! * `κ_l = κ_h·cosθ_g·cosθ_b + κ_v·sinθ_b`  — road-plane lateral curvature
//! * `κ_n = κ_v·cosθ_b − κ_h·cosθ_g·sinθ_b`  — road-normal curvature (crest unloads, dip loads)

use outlap_track::Track;

/// The minimum number of stations, so the passes and lap-time sum are well defined.
const MIN_STATIONS: usize = 8;
/// Half-width (stations) of the curvature noise-rejection moving average. Imported real-world
/// centerlines (OSM geometry + DEM elevation) carry sub-car-length position noise that an
/// interpolating spline amplifies into spurious curvature spikes; a light centred average over
/// ~2·radius·ds metres removes them while preserving genuine corners (which span many stations).
/// This is a pragmatic mitigation — the principled fix for a fair lap is the min-curvature line.
const CURV_SMOOTH_RADIUS: usize = 6;

/// Per-station geometry for the T0 velocity passes (SoA; queried by index).
#[derive(Clone, Debug)]
pub struct T0Path {
    /// Arc-length station, metres.
    pub s: Vec<f64>,
    /// Road-plane lateral curvature `κ_l`, 1/m.
    pub kappa_l: Vec<f64>,
    /// Road-normal curvature `κ_n`, 1/m.
    pub kappa_n: Vec<f64>,
    /// `sinθ_b·cosθ_g` (lateral gravity projection factor).
    pub sin_b_cos_g: Vec<f64>,
    /// `cosθ_b·cosθ_g` (normal gravity projection factor).
    pub cos_b_cos_g: Vec<f64>,
    /// `sinθ_g` (longitudinal gravity projection factor; `+` uphill).
    pub sin_g: Vec<f64>,
    /// Grip scale `γ(s)`.
    pub grip: Vec<f64>,
    /// Uniform arc-length step, metres (divides the length exactly).
    pub ds: f64,
    /// Whether the path is a closed loop.
    pub closed: bool,
}

impl T0Path {
    /// Sample a track into a uniform-station path. `ds_target` is rounded so the step divides the
    /// length exactly (the wrap segment of a closed lap is then also `ds`).
    pub fn from_track(track: &Track, ds_target: f64) -> Self {
        let length = track.length();
        let closed = track.is_closed();
        let n_seg = ((length / ds_target).round() as usize).max(MIN_STATIONS);
        let ds = length / n_seg as f64;
        // Closed: N stations 0..N-1 (station N wraps to 0). Open: N+1 stations spanning [0, length].
        let n_stations = if closed { n_seg } else { n_seg + 1 };

        let mut p = T0Path {
            s: Vec::with_capacity(n_stations),
            kappa_l: Vec::with_capacity(n_stations),
            kappa_n: Vec::with_capacity(n_stations),
            sin_b_cos_g: Vec::with_capacity(n_stations),
            cos_b_cos_g: Vec::with_capacity(n_stations),
            sin_g: Vec::with_capacity(n_stations),
            grip: Vec::with_capacity(n_stations),
            ds,
            closed,
        };
        for i in 0..n_stations {
            let s = i as f64 * ds;
            let kappa_h = track.curvature_h(s);
            let kappa_v = track.curvature_v(s);
            let grade = track.grade(s);
            let banking = track.banking(s);
            let (sb, cb) = banking.sin_cos();
            let (sg, cg) = grade.sin_cos();
            p.s.push(s);
            p.kappa_l.push(kappa_h * cg * cb + kappa_v * sb);
            p.kappa_n.push(kappa_v * cb - kappa_h * cg * sb);
            p.sin_b_cos_g.push(sb * cg);
            p.cos_b_cos_g.push(cb * cg);
            p.sin_g.push(sg);
            p.grip.push(track.grip_scale(s));
        }
        // Reject import/DEM curvature noise (see CURV_SMOOTH_RADIUS).
        p.kappa_l = smooth(&p.kappa_l, CURV_SMOOTH_RADIUS, closed);
        p.kappa_n = smooth(&p.kappa_n, CURV_SMOOTH_RADIUS, closed);
        p
    }

    /// Number of stations.
    pub fn len(&self) -> usize {
        self.s.len()
    }

    /// Whether the path has no stations (never true after [`from_track`](Self::from_track)).
    pub fn is_empty(&self) -> bool {
        self.s.is_empty()
    }

    /// Number of arc-length segments (`N` for a closed loop, `N−1` for an open path).
    pub fn segments(&self) -> usize {
        if self.closed {
            self.s.len()
        } else {
            self.s.len() - 1
        }
    }

    /// Total path length, metres.
    pub fn length(&self) -> f64 {
        self.ds * self.segments() as f64
    }
}

/// Centred moving average of half-width `radius` (wrapping for a closed path, clamped for open).
fn smooth(x: &[f64], radius: usize, closed: bool) -> Vec<f64> {
    let n = x.len();
    if radius == 0 || n < 2 * radius + 1 {
        return x.to_vec();
    }
    let mut out = vec![0.0; n];
    let r = radius as isize;
    for (i, o) in out.iter_mut().enumerate() {
        let mut sum = 0.0;
        let mut count = 0.0;
        for d in -r..=r {
            let j = i as isize + d;
            let idx = if closed {
                j.rem_euclid(n as isize) as usize
            } else if j < 0 || j >= n as isize {
                continue;
            } else {
                j as usize
            };
            sum += x[idx];
            count += 1.0;
        }
        *o = sum / count;
    }
    out
}
