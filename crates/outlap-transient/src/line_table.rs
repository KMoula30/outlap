// SPDX-License-Identifier: AGPL-3.0-only
//! The **target-line table** (PR4 target-line plumbing; Decision #30).
//!
//! The transient tier never touches a [`Track`](outlap_track) or an envelope in the loop — it
//! **receives** the QSS artifacts already sampled into a flat table and queries them per step. The
//! table stores, on the ONE shared monotone cubic Hermite interpolant:
//!
//! * road geometry the chassis needs — plan-view curvature `κ_h(s)`, grade `θ(s)`, banking `φ(s)`,
//!   vertical curvature `κ_v(s)`;
//! * the target line the driver tracks — lateral offset `n_ref(s)`, curvature `κ_ref(s)`, speed
//!   `v_ref(s)` (from the T0/QSS profile);
//! * the reference-line world geometry `x(s), y(s), z(s)` and the road-left unit vector, so the
//!   time-integrated trajectory reconstructs as `world = ref(s) + n·lateral(s)` (Decision #13 —
//!   x/y/z from the integrated `(s, n)`, never from `track.position(s)` on a re-derived `s`).
//!
//! Queries are allocation-free and wrap `s` into `[0, L]` for closed loops. Building the splines
//! (once, at assembly) may allocate.

use num_traits::Float;

use outlap_core::interp::{InterpError, MonotoneCubic};

/// The seven road/reference channels at one station (from [`LineTable::road_sample`]).
#[derive(Clone, Copy, Debug)]
pub struct RoadSample<T> {
    /// Plan-view curvature `κ_h`, 1/m.
    pub kappa_h: T,
    /// Grade `θ`, rad.
    pub grade: T,
    /// Banking `φ`, rad.
    pub banking: T,
    /// Vertical curvature `κ_v`, 1/m.
    pub kappa_v: T,
    /// Target lateral offset `n_ref`, m.
    pub n_ref: T,
    /// Target-line curvature `κ_ref`, 1/m.
    pub kappa_ref: T,
    /// Target speed `v_ref`, m/s.
    pub v_ref: T,
}

/// The three target-line channels at the preview station (from [`LineTable::preview_sample`]).
#[derive(Clone, Copy, Debug)]
pub struct PreviewSample<T> {
    /// Target lateral offset `n_ref`, m.
    pub n_ref: T,
    /// Target-line curvature `κ_ref`, 1/m.
    pub kappa_ref: T,
    /// Target speed `v_ref`, m/s.
    pub v_ref: T,
}

/// A sampled target line + road geometry, queryable per step (see the module docs).
#[derive(Clone, Debug)]
pub struct LineTable<T> {
    length: T,
    closed: bool,
    kappa_h: MonotoneCubic<T>,
    grade: MonotoneCubic<T>,
    banking: MonotoneCubic<T>,
    kappa_v: MonotoneCubic<T>,
    n_ref: MonotoneCubic<T>,
    kappa_ref: MonotoneCubic<T>,
    v_ref: MonotoneCubic<T>,
    x_ref: MonotoneCubic<T>,
    y_ref: MonotoneCubic<T>,
    z_ref: MonotoneCubic<T>,
    lat_x: MonotoneCubic<T>,
    lat_y: MonotoneCubic<T>,
    lat_z: MonotoneCubic<T>,
}

/// The per-station samples used to build a [`LineTable`]. All slices share the station grid `s` and
/// must have equal length (≥ 2). `s` is strictly increasing and spans `[0, length]`.
#[derive(Clone, Debug)]
pub struct LineSamples<T> {
    /// Strictly increasing arc-length stations, m (last entry = `length`).
    pub s: Vec<T>,
    /// Plan-view curvature `κ_h(s)`, 1/m (+left).
    pub kappa_h: Vec<T>,
    /// Grade `θ(s)`, rad (+uphill).
    pub grade: Vec<T>,
    /// Banking `φ(s)`, rad (+ raises road-left edge).
    pub banking: Vec<T>,
    /// Vertical curvature `κ_v(s)`, 1/m (crest < 0, dip > 0).
    pub kappa_v: Vec<T>,
    /// Target lateral offset `n_ref(s)`, m (+left).
    pub n_ref: Vec<T>,
    /// Target-line curvature `κ_ref(s)`, 1/m.
    pub kappa_ref: Vec<T>,
    /// Target speed `v_ref(s)`, m/s.
    pub v_ref: Vec<T>,
    /// Reference-line world coordinates `x/y/z(s)`, m.
    pub x_ref: Vec<T>,
    /// Reference-line world coordinates `x/y/z(s)`, m.
    pub y_ref: Vec<T>,
    /// Reference-line world coordinates `x/y/z(s)`, m.
    pub z_ref: Vec<T>,
    /// Road-left unit vector components at `s`.
    pub lat_x: Vec<T>,
    /// Road-left unit vector components at `s`.
    pub lat_y: Vec<T>,
    /// Road-left unit vector components at `s`.
    pub lat_z: Vec<T>,
    /// Whether the reference line is a closed loop (wrap `s`) or open (clamp `s`).
    pub closed: bool,
}

impl<T: Float> LineTable<T> {
    /// Build a line table from per-station samples.
    ///
    /// # Errors
    /// [`InterpError`] if any channel is not a valid monotone-cubic input (mismatched lengths, fewer
    /// than two knots, or a non-increasing `s` grid).
    pub fn new(samples: &LineSamples<T>) -> Result<Self, InterpError> {
        let s = &samples.s;
        let length = *s.last().expect("at least two stations");
        let build = |ys: &[T]| MonotoneCubic::new(s.clone(), ys.to_vec());
        Ok(Self {
            length,
            closed: samples.closed,
            kappa_h: build(&samples.kappa_h)?,
            grade: build(&samples.grade)?,
            banking: build(&samples.banking)?,
            kappa_v: build(&samples.kappa_v)?,
            n_ref: build(&samples.n_ref)?,
            kappa_ref: build(&samples.kappa_ref)?,
            v_ref: build(&samples.v_ref)?,
            x_ref: build(&samples.x_ref)?,
            y_ref: build(&samples.y_ref)?,
            z_ref: build(&samples.z_ref)?,
            lat_x: build(&samples.lat_x)?,
            lat_y: build(&samples.lat_y)?,
            lat_z: build(&samples.lat_z)?,
        })
    }

    /// Track length `L`, m.
    #[must_use]
    pub fn length(&self) -> T {
        self.length
    }

    /// Whether the reference line is a closed loop.
    #[must_use]
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// Normalise `s` into the query domain: wrap into `[0, L]` for a closed loop, clamp otherwise.
    #[inline]
    #[must_use]
    pub fn norm_s(&self, s: T) -> T {
        if self.closed {
            let l = self.length;
            let mut r = s - (s / l).floor() * l; // rem_euclid for Float
            if r < T::zero() {
                r = r + l;
            }
            r
        } else {
            s.max(T::zero()).min(self.length)
        }
    }

    /// Plan-view curvature `κ_h` at `s`, 1/m.
    #[must_use]
    pub fn kappa_h(&self, s: T) -> T {
        self.kappa_h.eval(self.norm_s(s))
    }
    /// Grade `θ` at `s`, rad.
    #[must_use]
    pub fn grade(&self, s: T) -> T {
        self.grade.eval(self.norm_s(s))
    }
    /// Banking `φ` at `s`, rad.
    #[must_use]
    pub fn banking(&self, s: T) -> T {
        self.banking.eval(self.norm_s(s))
    }
    /// Vertical curvature `κ_v` at `s`, 1/m.
    #[must_use]
    pub fn kappa_v(&self, s: T) -> T {
        self.kappa_v.eval(self.norm_s(s))
    }
    /// Target lateral offset `n_ref` at `s`, m.
    #[must_use]
    pub fn n_ref(&self, s: T) -> T {
        self.n_ref.eval(self.norm_s(s))
    }
    /// Target-line curvature `κ_ref` at `s`, 1/m.
    #[must_use]
    pub fn kappa_ref(&self, s: T) -> T {
        self.kappa_ref.eval(self.norm_s(s))
    }
    /// Target speed `v_ref` at `s`, m/s.
    #[must_use]
    pub fn v_ref(&self, s: T) -> T {
        self.v_ref.eval(self.norm_s(s))
    }

    /// The seven road/reference channels at `s`, sharing **one** interval lookup (all channels are
    /// built on the same `s` grid). Bit-identical to seven separate `kappa_h(s)`/… calls, but with a
    /// single binary search instead of seven — the hot `publish_road` path (Decision #30's one
    /// interpolant, one lookup). Fields mirror the individual accessors.
    #[must_use]
    pub fn road_sample(&self, s: T) -> RoadSample<T> {
        let (k, t, h) = self.kappa_h.locate_at(self.norm_s(s));
        RoadSample {
            kappa_h: self.kappa_h.eval_segment(k, t, h),
            grade: self.grade.eval_segment(k, t, h),
            banking: self.banking.eval_segment(k, t, h),
            kappa_v: self.kappa_v.eval_segment(k, t, h),
            n_ref: self.n_ref.eval_segment(k, t, h),
            kappa_ref: self.kappa_ref.eval_segment(k, t, h),
            v_ref: self.v_ref.eval_segment(k, t, h),
        }
    }

    /// The three target-line channels at the preview station `sp`, sharing one interval lookup.
    #[must_use]
    pub fn preview_sample(&self, sp: T) -> PreviewSample<T> {
        let (k, t, h) = self.n_ref.locate_at(self.norm_s(sp));
        PreviewSample {
            n_ref: self.n_ref.eval_segment(k, t, h),
            kappa_ref: self.kappa_ref.eval_segment(k, t, h),
            v_ref: self.v_ref.eval_segment(k, t, h),
        }
    }

    /// The three road-geometry channels the load block needs (`grade`, `banking`, `κ_v`) at `s`,
    /// sharing one interval lookup. Bit-identical to the three separate accessors.
    #[must_use]
    pub fn load_geometry(&self, s: T) -> (T, T, T) {
        let (k, t, h) = self.grade.locate_at(self.norm_s(s));
        (
            self.grade.eval_segment(k, t, h),
            self.banking.eval_segment(k, t, h),
            self.kappa_v.eval_segment(k, t, h),
        )
    }

    /// Reconstruct the world position `ref(s) + n·lateral(s)` for the integrated `(s, n)` (m).
    #[must_use]
    pub fn world_position(&self, s: T, n: T) -> [T; 3] {
        let sn = self.norm_s(s);
        [
            self.x_ref.eval(sn) + n * self.lat_x.eval(sn),
            self.y_ref.eval(sn) + n * self.lat_y.eval(sn),
            self.z_ref.eval(sn) + n * self.lat_z.eval(sn),
        ]
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    fn ramp_samples(closed: bool) -> LineSamples<f64> {
        // s in [0, 100], κ ramps 0 → 0.02, v_ref 20 → 40, straight world geometry along +x.
        let s: Vec<f64> = (0..=10).map(|i| f64::from(i) * 10.0).collect();
        let mk = |f: &dyn Fn(f64) -> f64| s.iter().map(|&si| f(si)).collect::<Vec<_>>();
        LineSamples {
            kappa_h: mk(&|si| 0.02 * si / 100.0),
            grade: mk(&|_| 0.0),
            banking: mk(&|_| 0.0),
            kappa_v: mk(&|_| 0.0),
            n_ref: mk(&|_| 0.0),
            kappa_ref: mk(&|si| 0.02 * si / 100.0),
            v_ref: mk(&|si| 20.0 + 0.2 * si),
            x_ref: s.clone(),
            y_ref: mk(&|_| 0.0),
            z_ref: mk(&|_| 0.0),
            lat_x: mk(&|_| 0.0),
            lat_y: mk(&|_| 1.0),
            lat_z: mk(&|_| 0.0),
            s,
            closed,
        }
    }

    #[test]
    fn interpolates_channels_at_and_between_knots() {
        let table = LineTable::new(&ramp_samples(false)).unwrap();
        // Exact at a knot.
        assert!((table.v_ref(50.0) - 30.0).abs() < 1e-9);
        assert!((table.kappa_h(100.0) - 0.02).abs() < 1e-12);
        // Monotone between knots (linear ramp reproduces exactly on a straight line).
        assert!((table.v_ref(25.0) - 25.0).abs() < 1e-9);
    }

    #[test]
    fn closed_line_wraps_s_into_the_domain() {
        let table = LineTable::new(&ramp_samples(true)).unwrap();
        let length = table.length();
        assert!((table.norm_s(length + 10.0) - 10.0).abs() < 1e-9);
        assert!((table.norm_s(-10.0) - (length - 10.0)).abs() < 1e-9);
        // Wrapped query equals the in-domain query.
        assert!((table.v_ref(length + 30.0) - table.v_ref(30.0)).abs() < 1e-12);
    }

    #[test]
    fn open_line_clamps_s_at_the_edges() {
        let table = LineTable::new(&ramp_samples(false)).unwrap();
        assert_eq!(table.norm_s(-5.0), 0.0);
        assert_eq!(table.norm_s(500.0), table.length());
    }

    #[test]
    fn world_position_offsets_along_the_lateral_vector() {
        let table = LineTable::new(&ramp_samples(false)).unwrap();
        // At s = 30 (world x = 30, road-left = +y), offset n = 2 ⇒ (30, 2, 0).
        let p = table.world_position(30.0, 2.0);
        assert!((p[0] - 30.0).abs() < 1e-9);
        assert!((p[1] - 2.0).abs() < 1e-9);
        assert!(p[2].abs() < 1e-12);
    }
}
