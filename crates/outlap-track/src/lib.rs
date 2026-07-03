// SPDX-License-Identifier: AGPL-3.0-only
//! `outlap-track` — load a track (`track.yaml` + `centerline.csv`) into a queryable 3D road model.
//!
//! The track is the second input of the sacred quartet. This crate takes the validated
//! [`TrackDoc`](outlap_schema::track::TrackDoc) and [`Centerline`](outlap_schema::centerline)
//! and builds a **full-3D road frame** (Locked Decision #13): the centerline geometry is fitted
//! with a **C² cubic spline** (periodic for closed circuits) so curvature is continuous, while the
//! per-`s` data channels (banking, widths, grip) use the shared **monotone cubic Hermite**
//! (Decision #30). Everything downstream — the T0–T3 solvers, the min-curvature racing line
//! (§6.3), the plots — queries this model by arc length `s`.
//!
//! # Sign & frame conventions (ISO 8855: x forward, y left, z up)
//! * `kappa_h > 0` is a left turn; `kappa_v < 0` is a crest, `> 0` a dip.
//! * `banking > 0` raises the left/outside edge; `grade > 0` is uphill.
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::doc_markdown,
    // Geometry kernels use single-letter symbols (x, y, z, s) by convention (Decision #33).
    clippy::many_single_char_names,
    clippy::similar_names,
    // TrackError embeds the miette-annotated SchemaError (source text) on the cold error path.
    clippy::result_large_err,
    // Cold-path track assembly: sample-count/index casts are safe at track sizes.
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]

use outlap_core::{CubicSpline, MonotoneCubic};

/// The three geometry splines (`x(s)`, `y(s)`, `z(s)`) plus the resulting track length.
type Geometry = (CubicSpline<f64>, CubicSpline<f64>, CubicSpline<f64>, f64);
use outlap_schema::centerline::{parse_centerline, Centerline, CenterlineError, CenterlineRow};
use outlap_schema::io::SourceLoader;
use outlap_schema::refs::CenterlineRef;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::SchemaVersion;

/// Minimum centerline points needed to fit the geometry splines.
const MIN_POINTS: usize = 4;
/// Two centerline endpoints closer than this (metres) are treated as an explicit duplicate closure.
const CLOSURE_COINCIDENT_TOL_M: f64 = 1e-6;
/// Floor for an offset line's residual half-widths (metres), keeping the line's own width envelope
/// positive even when the reference line is pushed to the track edge.
const MIN_RESIDUAL_WIDTH_M: f64 = 0.1;

/// An error building or loading a [`Track`].
#[derive(Debug, thiserror::Error)]
pub enum TrackError {
    /// A referenced source (the `track.yaml` or its `centerline.csv`) failed to load or validate.
    #[error(transparent)]
    Schema(#[from] outlap_schema::SchemaError),
    /// The `centerline.csv` failed to parse/validate.
    #[error(transparent)]
    Centerline(#[from] CenterlineError),
    /// Too few centerline points to fit a spline.
    #[error("centerline has {got} points; need at least {min}")]
    TooFewPoints {
        /// Points supplied.
        got: usize,
        /// Minimum required.
        min: usize,
    },
    /// The track is marked `closed` but the endpoints are far apart (likely not actually a loop).
    #[error("track marked closed but the start/finish gap is {gap_m:.1} m ({ratio:.0}× the median sample spacing) — set `closed: false` or fix the centerline")]
    NotClosed {
        /// The closing-chord gap, metres.
        gap_m: f64,
        /// Gap as a multiple of the median sample spacing.
        ratio: f64,
    },
    /// A spline fit failed (should not happen after validation; surfaced defensively).
    #[error("spline fit failed: {0}")]
    Spline(#[from] outlap_core::SplineError),
    /// A data-channel interpolant failed to build.
    #[error("channel fit failed: {0}")]
    Channel(#[from] outlap_core::InterpError),
    /// `offset_track` was given `s` and `n` slices of different lengths.
    #[error("offset arrays differ in length: s has {s}, n has {n}")]
    MismatchedOffsets {
        /// Length of the station array.
        s: usize,
        /// Length of the offset array.
        n: usize,
    },
    /// Two consecutive offset points coincide, so the offset line has no arc length there.
    #[error("offset line is degenerate at index {index} (consecutive points coincide)")]
    DegenerateOffsetLine {
        /// The offending index.
        index: usize,
    },
}

/// The ISO-8855 road frame at an arc-length station.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoadFrame {
    /// Arc-length station, metres.
    pub s: f64,
    /// Centerline point in world coordinates, metres.
    pub origin: [f64; 3],
    /// Unit tangent along increasing `s`.
    pub tangent: [f64; 3],
    /// Unit road-left lateral (horizontal-left, then tilted by banking).
    pub lateral: [f64; 3],
    /// Unit road-surface normal (banking + grade rotate it off world +z).
    pub normal: [f64; 3],
    /// Plan-view (yaw) curvature, signed: `+` = left turn, 1/m.
    pub kappa_h: f64,
    /// Vertical curvature of the elevation profile, signed: crest `< 0`, dip `> 0`, 1/m.
    pub kappa_v: f64,
    /// Road grade (tangent pitch above horizontal), radians.
    pub grade: f64,
    /// Banking angle, radians (`+` raises the left/outside edge).
    pub banking: f64,
}

/// A dense arc-length resample of a track — the SoA substrate for the racing-line QP and plots.
#[derive(Clone, Debug, Default)]
pub struct TrackSamples {
    /// Arc-length stations, metres.
    pub s: Vec<f64>,
    /// World x, metres.
    pub x: Vec<f64>,
    /// World y, metres.
    pub y: Vec<f64>,
    /// World z, metres.
    pub z: Vec<f64>,
    /// Plan-view curvature, 1/m.
    pub kappa_h: Vec<f64>,
    /// Vertical curvature, 1/m.
    pub kappa_v: Vec<f64>,
    /// Grade, radians.
    pub grade: Vec<f64>,
    /// Banking, radians.
    pub banking: Vec<f64>,
    /// Left half-width, metres.
    pub width_left: Vec<f64>,
    /// Right half-width, metres.
    pub width_right: Vec<f64>,
    /// Grip scale.
    pub grip_scale: Vec<f64>,
}

/// A loaded, validated, spline-fitted track, queryable by arc length `s ∈ [0, length]`.
#[derive(Clone, Debug)]
pub struct Track {
    name: String,
    closed: bool,
    length: f64,
    // Geometry: C² cubic splines parameterised by arc length s (Decision #13).
    gx: CubicSpline<f64>,
    gy: CubicSpline<f64>,
    gz: CubicSpline<f64>,
    // Data channels: monotone cubic Hermite in s (Decision #30).
    banking: MonotoneCubic<f64>,
    width_left: MonotoneCubic<f64>,
    width_right: MonotoneCubic<f64>,
    grip: MonotoneCubic<f64>,
}

impl Track {
    /// Load a track from a `track.yaml` reference via a [`SourceLoader`] (also loads the referenced
    /// `centerline.csv`).
    pub fn load(track_ref: &str, loader: &dyn SourceLoader) -> Result<Track, TrackError> {
        let doc = outlap_schema::load_track_doc(track_ref, loader)?;
        let csv = loader
            .load(doc.centerline.as_str())
            .map_err(outlap_schema::SchemaError::from)?;
        let centerline = parse_centerline(&csv, MIN_POINTS)?;
        Self::from_doc(&doc, &centerline)
    }

    /// Build a track from an already-validated document and centerline (identical to [`Track::load`],
    /// but for in-memory objects — provenance parity with the file path, Decision #44).
    pub fn from_doc(doc: &TrackDoc, centerline: &Centerline) -> Result<Track, TrackError> {
        let rows = &centerline.rows;
        let n = rows.len();
        if n < MIN_POINTS {
            return Err(TrackError::TooFewPoints {
                got: n,
                min: MIN_POINTS,
            });
        }

        // Normalise arc length to start at zero (queries are 0-based, track-relative).
        let s0 = rows[0].s_m;
        let s: Vec<f64> = rows.iter().map(|r| r.s_m - s0).collect();
        let x: Vec<f64> = rows.iter().map(|r| r.x_m).collect();
        let y: Vec<f64> = rows.iter().map(|r| r.y_m).collect();
        let z: Vec<f64> = rows.iter().map(|r| r.z_m).collect();

        let (gx, gy, gz, length) = if doc.closed {
            fit_closed_geometry(&s, &x, &y, &z)?
        } else {
            let len = s[n - 1];
            (
                CubicSpline::not_a_knot(s.clone(), x.clone())?,
                CubicSpline::not_a_knot(s.clone(), y.clone())?,
                CubicSpline::not_a_knot(s.clone(), z.clone())?,
                len,
            )
        };

        // Banking channel: sparse keypoints override the centerline column (§9.3).
        let (bank_s, bank_v): (Vec<f64>, Vec<f64>) = if doc.banking_keypoints.is_empty() {
            (
                s.clone(),
                rows.iter().map(|r| r.banking_deg.to_radians()).collect(),
            )
        } else {
            (
                doc.banking_keypoints.iter().map(|k| k.s_m - s0).collect(),
                doc.banking_keypoints
                    .iter()
                    .map(|k| k.banking_deg.to_radians())
                    .collect(),
            )
        };

        let banking = build_channel(&bank_s, &bank_v, doc.closed, length)?;
        let width_left = build_channel(
            &s,
            &rows.iter().map(|r| r.width_left_m).collect::<Vec<_>>(),
            doc.closed,
            length,
        )?;
        let width_right = build_channel(
            &s,
            &rows.iter().map(|r| r.width_right_m).collect::<Vec<_>>(),
            doc.closed,
            length,
        )?;
        let grip = build_channel(
            &s,
            &rows.iter().map(|r| r.grip_scale).collect::<Vec<_>>(),
            doc.closed,
            length,
        )?;

        Ok(Track {
            name: doc.name.clone(),
            closed: doc.closed,
            length,
            gx,
            gy,
            gz,
            banking,
            width_left,
            width_right,
            grip,
        })
    }

    /// The circuit name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Total arc length (the loop perimeter for a closed circuit), metres.
    pub fn length(&self) -> f64 {
        self.length
    }

    /// Whether this track is a closed loop.
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// Reduce a query station into the valid range: wrap for closed, clamp for open.
    fn norm_s(&self, s: f64) -> f64 {
        if self.closed {
            let mut u = s % self.length;
            if u < 0.0 {
                u += self.length;
            }
            u
        } else {
            s.clamp(0.0, self.length)
        }
    }

    /// Centerline world position at `s`.
    pub fn position(&self, s: f64) -> [f64; 3] {
        let s = self.norm_s(s);
        [self.gx.eval(s), self.gy.eval(s), self.gz.eval(s)]
    }

    /// Plan-view (yaw) curvature at `s`, signed (`+` = left), 1/m.
    pub fn curvature_h(&self, s: f64) -> f64 {
        let s = self.norm_s(s);
        let (xt, yt) = (self.gx.deriv(s), self.gy.deriv(s));
        let (xtt, ytt) = (self.gx.deriv2(s), self.gy.deriv2(s));
        let denom = (xt * xt + yt * yt).powf(1.5);
        if denom == 0.0 {
            0.0
        } else {
            (xt * ytt - yt * xtt) / denom
        }
    }

    /// Vertical curvature of the elevation profile at `s`, signed (crest `< 0`), 1/m.
    pub fn curvature_v(&self, s: f64) -> f64 {
        let s = self.norm_s(s);
        let (xt, yt, zt) = (self.gx.deriv(s), self.gy.deriv(s), self.gz.deriv(s));
        let (xtt, ytt, ztt) = (self.gx.deriv2(s), self.gy.deriv2(s), self.gz.deriv2(s));
        let horiz = (xt * xt + yt * yt).sqrt();
        if horiz == 0.0 {
            return 0.0;
        }
        // Curvature of the vertical-plane curve (H(t), z(t)), H = horizontal arc length.
        let h_tt = (xt * xtt + yt * ytt) / horiz;
        let denom = (horiz * horiz + zt * zt).powf(1.5);
        (horiz * ztt - zt * h_tt) / denom
    }

    /// Road grade (tangent pitch above horizontal) at `s`, radians.
    pub fn grade(&self, s: f64) -> f64 {
        let s = self.norm_s(s);
        let (xt, yt, zt) = (self.gx.deriv(s), self.gy.deriv(s), self.gz.deriv(s));
        zt.atan2((xt * xt + yt * yt).sqrt())
    }

    /// Banking angle at `s`, radians.
    pub fn banking(&self, s: f64) -> f64 {
        self.banking.eval(self.norm_s(s))
    }

    /// Left/right half-widths at `s`, metres.
    pub fn width(&self, s: f64) -> (f64, f64) {
        let s = self.norm_s(s);
        (self.width_left.eval(s), self.width_right.eval(s))
    }

    /// Grip-scale multiplier at `s`.
    pub fn grip_scale(&self, s: f64) -> f64 {
        self.grip.eval(self.norm_s(s))
    }

    /// The full road frame at `s` (the geometry the T0–T3 solvers consume).
    pub fn road_frame(&self, s: f64) -> RoadFrame {
        let sn = self.norm_s(s);
        let origin = self.position(sn);
        let rt = [self.gx.deriv(sn), self.gy.deriv(sn), self.gz.deriv(sn)];
        let speed = norm(rt).max(f64::MIN_POSITIVE);
        let tangent = scale(rt, 1.0 / speed);

        // Horizontal-left basis, then rotate about the tangent by the banking angle.
        let banking = self.banking.eval(sn);
        let l0 = normalize(cross([0.0, 0.0, 1.0], tangent));
        let n0 = normalize(cross(tangent, l0));
        let (sb, cb) = banking.sin_cos();
        let lateral = add(scale(l0, cb), scale(n0, sb));
        let normal = add(scale(l0, -sb), scale(n0, cb));

        RoadFrame {
            s: sn,
            origin,
            tangent,
            lateral,
            normal,
            kappa_h: self.curvature_h(sn),
            kappa_v: self.curvature_v(sn),
            grade: self.grade(sn),
            banking,
        }
    }

    /// Resample the whole track at a uniform arc-length step `ds` (metres).
    pub fn sample_uniform(&self, ds: f64) -> TrackSamples {
        let count = (self.length / ds).ceil() as usize + 1;
        let mut out = TrackSamples::default();
        for i in 0..count {
            let s = (i as f64 * ds).min(self.length);
            let f = self.road_frame(s);
            let (wl, wr) = self.width(s);
            out.s.push(s);
            out.x.push(f.origin[0]);
            out.y.push(f.origin[1]);
            out.z.push(f.origin[2]);
            out.kappa_h.push(f.kappa_h);
            out.kappa_v.push(f.kappa_v);
            out.grade.push(f.grade);
            out.banking.push(f.banking);
            out.width_left.push(wl);
            out.width_right.push(wr);
            out.grip_scale.push(self.grip_scale(s));
        }
        out
    }
}

/// Build a new [`Track`] by offsetting a parent track laterally by `n(s)` in its road plane.
///
/// `s` are stations along the *parent centerline* arc length and `n` the signed lateral offsets
/// (metres, `+` = road-left, ISO 8855). Each offset point is `origin(s_i) + n_i·lateral(s_i)` in the
/// banked road plane; banking and grip are inherited from the parent, the residual half-widths are
/// the parent widths minus the offset, and the arc length is recomputed along the offset points.
/// The result is a first-class [`Track`] — so a racing line has its own `κ(s)`, grade, and road
/// frame, queried through the identical API. This is the substrate the T0 solver runs on for a
/// generated or user-supplied line (§6.3).
///
/// # Errors
/// [`TrackError`] if `s`/`n` differ in length, there are too few points, consecutive offset points
/// coincide, or the resulting geometry fails to fit.
pub fn offset_track(track: &Track, s: &[f64], n: &[f64], name: &str) -> Result<Track, TrackError> {
    if s.len() != n.len() {
        return Err(TrackError::MismatchedOffsets {
            s: s.len(),
            n: n.len(),
        });
    }
    if s.len() < MIN_POINTS {
        return Err(TrackError::TooFewPoints {
            got: s.len(),
            min: MIN_POINTS,
        });
    }

    let m = s.len();
    let mut pts: Vec<[f64; 3]> = Vec::with_capacity(m);
    let mut rows: Vec<CenterlineRow> = Vec::with_capacity(m);
    for (&si, &ni) in s.iter().zip(n) {
        let frame = track.road_frame(si);
        let p = add(frame.origin, scale(frame.lateral, ni));
        let (wl, wr) = track.width(si);
        rows.push(CenterlineRow {
            s_m: 0.0, // filled below once the offset arc length is known
            x_m: p[0],
            y_m: p[1],
            z_m: p[2],
            banking_deg: frame.banking.to_degrees(),
            width_left_m: (wl - ni).max(MIN_RESIDUAL_WIDTH_M),
            width_right_m: (wr + ni).max(MIN_RESIDUAL_WIDTH_M),
            grip_scale: track.grip_scale(si),
        });
        pts.push(p);
    }

    // Recompute arc length along the offset line (its own geometry, not the parent's s).
    let mut acc = 0.0;
    for i in 1..m {
        let step = norm(sub(pts[i], pts[i - 1]));
        acc += step;
        // Positive test (not `<=`) also rejects a NaN arc length as degenerate.
        let strictly_increasing = acc > rows[i - 1].s_m;
        if !strictly_increasing {
            return Err(TrackError::DegenerateOffsetLine { index: i });
        }
        rows[i].s_m = acc;
    }

    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: name.to_owned(),
        closed: track.is_closed(),
        centerline: CenterlineRef("<offset>".to_owned()),
        banking_keypoints: Vec::new(),
        meta: TrackMeta::default(),
    };
    Track::from_doc(&doc, &Centerline { rows })
}

/// Fit the three geometry splines for a closed loop, returning them plus the loop length.
fn fit_closed_geometry(s: &[f64], x: &[f64], y: &[f64], z: &[f64]) -> Result<Geometry, TrackError> {
    let n = s.len();
    let first = [x[0], y[0], z[0]];
    let last = [x[n - 1], y[n - 1], z[n - 1]];
    let chord = norm(sub(last, first));
    let spacing = median_spacing(s);

    // A closed track whose endpoints are wildly apart is almost certainly mislabelled.
    if chord > 3.0 * spacing && chord > CLOSURE_COINCIDENT_TOL_M {
        return Err(TrackError::NotClosed {
            gap_m: chord,
            ratio: chord / spacing,
        });
    }

    if chord <= CLOSURE_COINCIDENT_TOL_M {
        // Explicit duplicate-closure: drop the repeated last row; the period is its arc length.
        let period = s[n - 1];
        let ks = s[..n - 1].to_vec();
        Ok((
            CubicSpline::periodic(ks.clone(), x[..n - 1].to_vec(), period)?,
            CubicSpline::periodic(ks.clone(), y[..n - 1].to_vec(), period)?,
            CubicSpline::periodic(ks, z[..n - 1].to_vec(), period)?,
            period,
        ))
    } else {
        // Distinct endpoints: the loop closes over the connecting chord.
        let period = s[n - 1] + chord;
        Ok((
            CubicSpline::periodic(s.to_vec(), x.to_vec(), period)?,
            CubicSpline::periodic(s.to_vec(), y.to_vec(), period)?,
            CubicSpline::periodic(s.to_vec(), z.to_vec(), period)?,
            period,
        ))
    }
}

/// Build a monotone-Hermite data channel in `s`. For closed tracks a wrap knot is appended at
/// `length` (value = first) so the channel is continuous across the seam. Constant single-keypoint
/// channels are widened to two knots.
fn build_channel(
    s: &[f64],
    v: &[f64],
    closed: bool,
    length: f64,
) -> Result<MonotoneCubic<f64>, outlap_core::InterpError> {
    let mut xs = s.to_vec();
    let mut ys = v.to_vec();

    if xs.len() == 1 {
        // Constant channel over the whole track.
        let val = ys[0];
        return MonotoneCubic::new(vec![0.0, length.max(f64::MIN_POSITIVE)], vec![val, val]);
    }

    if closed {
        let last_s = *xs.last().unwrap();
        if last_s < length - 1e-9 {
            xs.push(length);
            ys.push(ys[0]);
        }
    }
    MonotoneCubic::new(xs, ys)
}

/// Median spacing between consecutive arc-length stations.
fn median_spacing(s: &[f64]) -> f64 {
    let mut gaps: Vec<f64> = s.windows(2).map(|w| w[1] - w[0]).collect();
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
    gaps.get(gaps.len() / 2).copied().unwrap_or(1.0)
}

// --- Minimal 3-vector helpers (geometry is cold-path f64) ------------------------------------

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}
fn normalize(a: [f64; 3]) -> [f64; 3] {
    scale(a, 1.0 / norm(a).max(f64::MIN_POSITIVE))
}
