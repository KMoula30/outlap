// SPDX-License-Identifier: AGPL-3.0-only
//! Corner-scaled speed targets for the transient (T2) driver.
//!
//! The T2 closed loop cannot track the raw QSS speed profile: the profile rides the grip boundary,
//! the boundary is not filtered for open-loop stability, and a driver commanded to the limit spins
//! (`docs/theory/ggv-envelope.md`, `docs/validation/limebeer.md`). The original guard was a *global*
//! speed margin — track `m₀ · v_ref` everywhere — which is stable but throws away straight-line
//! speed where there is no lateral stability risk at all (a ~15 % cap on an F1 top speed is
//! ~50 km/h given away for nothing).
//!
//! This module shapes the target per station instead. Every quantity is evaluated on the path's own
//! 3-D geometry (`demand_and_gn`: signed lateral demand including banking, and the road-normal
//! specific gravity `g_normal(v)` including the vertical-curvature `κ_n·v²` term), so a crest that
//! cuts real grip cuts the shaped budget too:
//!
//! 1. **Blend by lateral utilisation on the *combined* boundary.** `u = |a_y| / a_y_max(v, a_x,
//!    g_n)` measures how close the profile rides the friction ellipse; evaluating at the profile's
//!    own `a_x` keeps the margin engaged through trail-braking entry and throttle-on exit. The
//!    margin blends `m(u) = 1 − (1 − m₀)·min(1, u)`: full profile speed on the straights, the
//!    classic `m₀` at the limit. Stations below `A_LAT_FLOOR` of lateral demand take `u = 0`.
//! 2. **Corner margins reach their approaches.** `u` propagates backward through the profile's
//!    braking zones (arrive pre-scrubbed, as the proven global scheme did) and decays backward no
//!    faster than a bounded rate (`SETTLE_S` of travel), so a fast corner entered on a lift still
//!    announces itself early — the measured spins concentrate at turn-ins asked to lift-and-turn
//!    simultaneously.
//! 3. **A margined braking feasibility pass** (backward): demanded decel never exceeds `m₀²` of the
//!    braking capability *available at the local lateral demand* (friction-ellipse complement, with
//!    the margin's lateral headroom reserved) — braking finishes before serious lateral builds.
//! 4. **A margined traction feasibility pass** (forward): the corner-exit ramp never demands more
//!    than `m₀` of the drive capability available at the local lateral demand. Without it the
//!    m₀ → 1 catch-up stacked on the raw profile's own acceleration pins the throttle wide open
//!    while the car still carries lateral load — the measured trigger of the corner-exit drift.
//!
//! The shaping runs once at assembly (the transient solver receives the finished `v_ref` table);
//! nothing here touches the hot loop.

use crate::path::T0Path;
use crate::solver::demand_and_gn;
use crate::t1::GgvEnvelope;

/// Lateral demand (m/s²) below which a station counts as straight (`u = 0`): ~0.1 g poses no
/// lateral-stability risk, and it guards the `u` ratio where the combined boundary → 0 under full
/// longitudinal use.
const A_LAT_FLOOR: f64 = 1.0;

/// The settle-ramp horizon, s of travel: `u` may decay backward no faster than `1/SETTLE_S` per
/// second of travel, so every corner casts an upstream lift window.
const SETTLE_S: f64 = 1.5;

/// Shape the T2 driver's speed targets from the raw QSS profile (see the module docs).
///
/// * `path` — the (possibly 3-D) path the profile was solved on; supplies stations, curvature,
///   banking/grade projections, and the closed flag.
/// * `v`, `ax` — the solved profile's speeds (m/s) and longitudinal accelerations (m/s²), one per
///   path station.
/// * `margin` — the limit margin `m₀ ∈ (0, 1]` (1 disables the shaping entirely).
#[must_use]
pub fn corner_scaled_targets(
    env: &GgvEnvelope,
    path: &T0Path,
    v: &[f64],
    ax: &[f64],
    margin: f64,
) -> Vec<f64> {
    let n = v.len();
    debug_assert_eq!(path.s.len(), n);
    debug_assert_eq!(ax.len(), n);
    if n == 0 || margin >= 1.0 {
        return v.to_vec();
    }
    let closed = path.closed;
    let ds = path.ds;

    // (1) blend by lateral utilisation on the combined (friction-ellipse) boundary.
    let mut u: Vec<f64> = (0..n)
        .map(|i| {
            let (ay_dem, gn) = demand_and_gn(path, i, v[i]);
            let a_lat = ay_dem.abs();
            if a_lat < A_LAT_FLOOR {
                0.0
            } else {
                let a_max = env.ay_boundary(v[i], ax[i], gn).max(A_LAT_FLOOR);
                (a_lat / a_max).min(1.0)
            }
        })
        .collect();

    // (2) corner margins reach their approaches: full propagation through braking zones + the
    // bounded settle-ramp decay elsewhere.
    let sweeps = if closed { 2 } else { 1 };
    for _ in 0..sweeps {
        for i in (0..n).rev() {
            let j = (i + 1) % n;
            if !closed && i + 1 == n {
                continue;
            }
            if ax[i] < -0.5 && u[j] > u[i] {
                u[i] = u[j];
            }
            let rate = ds / (v[i].max(1.0) * SETTLE_S);
            if u[j] - rate > u[i] {
                u[i] = u[j] - rate;
            }
        }
    }

    let mut target: Vec<f64> = (0..n)
        .map(|i| (1.0 - (1.0 - margin) * u[i]) * v[i])
        .collect();

    // (3) margined braking feasibility (backward), lateral headroom reserved.
    let decel_frac = margin * margin;
    for _ in 0..sweeps {
        for i in (0..n).rev() {
            let j = (i + 1) % n;
            if !closed && i + 1 == n {
                continue;
            }
            // Fixed-point on the reachable speed: the decel budget depends on the lateral demand at
            // station i, which depends on the speed we arrive with (and so does g_normal).
            let blend_cap = target[i];
            let mut v_c = blend_cap;
            for _ in 0..4 {
                let (ay_dem, gn) = demand_and_gn(path, i, v_c);
                let a_lat = ay_dem.abs() / decel_frac;
                let b = decel_frac * brake_capability(env, target[j].max(v_c * 0.5), a_lat, gn);
                let reachable = (target[j] * target[j] + 2.0 * b * ds).sqrt();
                let next = blend_cap.min(reachable);
                if (next - v_c).abs() < 1e-3 {
                    v_c = next;
                    break;
                }
                v_c = next;
            }
            target[i] = v_c;
        }
    }

    // (4) margined traction feasibility (forward). The ellipse taper in `accel_capability` already
    // yields to lateral load; the fraction here is m₀ (not m₀²) so corner exits match the proven
    // global scheme's effective drive instead of undercutting it.
    for _ in 0..sweeps {
        for i in 0..n {
            let j = (i + 1) % n;
            if !closed && i + 1 == n {
                continue;
            }
            let (ay_dem, gn) = demand_and_gn(path, j, target[j]);
            let a = margin * accel_capability(env, target[i], ay_dem.abs(), gn);
            let reachable = (target[i] * target[i] + 2.0 * a * ds).sqrt();
            if target[j] > reachable {
                target[j] = reachable;
            }
        }
    }
    target
}

/// The braking deceleration (m/s², ≥ 0) available at speed `v` while sustaining lateral demand
/// `a_lat` at road-normal gravity `gn` — the friction-ellipse complement, found by bisecting the
/// envelope's combined boundary `ay_boundary(v, aₓ, g_n)` for the most negative `aₓ` that still
/// supports `a_lat`.
fn brake_capability(env: &GgvEnvelope, v: f64, a_lat: f64, gn: f64) -> f64 {
    let b_max = env.brake_limit(v, gn).max(0.0);
    if b_max <= 0.0 {
        return 0.0;
    }
    if env.ay_boundary(v, -b_max, gn) >= a_lat {
        return b_max;
    }
    if env.ay_boundary(v, 0.0, gn) <= a_lat {
        return 0.0;
    }
    let (mut lo, mut hi) = (0.0, b_max); // lo: fits, hi: does not
    for _ in 0..12 {
        let mid = 0.5 * (lo + hi);
        if env.ay_boundary(v, -mid, gn) >= a_lat {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// The forward acceleration (m/s², ≥ 0) available at speed `v` while sustaining lateral demand
/// `a_lat` at road-normal gravity `gn` — the drive-side friction-ellipse complement (also capped by
/// the powertrain through the envelope's `accel_limit`).
fn accel_capability(env: &GgvEnvelope, v: f64, a_lat: f64, gn: f64) -> f64 {
    let a_max = env.accel_limit(v, gn).max(0.0);
    if a_max <= 0.0 {
        return 0.0;
    }
    if env.ay_boundary(v, a_max, gn) >= a_lat {
        return a_max;
    }
    if env.ay_boundary(v, 0.0, gn) <= a_lat {
        return 0.0;
    }
    let (mut lo, mut hi) = (0.0, a_max); // lo: fits, hi: does not
    for _ in 0..12 {
        let mid = 0.5 * (lo + hi);
        if env.ay_boundary(v, mid, gn) >= a_lat {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

#[cfg(test)]
mod tests {
    #![allow(clippy::cast_precision_loss)]
    use super::*;
    use crate::t1::envelope::tests::{sample_car, small_res};
    use crate::t1::GgvEnvelope;
    use outlap_schema::sim::FzCoupling;

    const G: f64 = 9.806_65;

    fn test_envelope() -> GgvEnvelope {
        GgvEnvelope::generate(&sample_car(), &small_res(), FzCoupling::OneStepLag).unwrap()
    }

    /// A flat open path with the given per-station lateral curvature.
    fn flat_path(kappa: Vec<f64>, ds: f64) -> T0Path {
        let n = kappa.len();
        T0Path {
            s: (0..n).map(|i| i as f64 * ds).collect(),
            kappa_l: kappa,
            kappa_n: vec![0.0; n],
            sin_b_cos_g: vec![0.0; n],
            cos_b_cos_g: vec![1.0; n],
            sin_g: vec![0.0; n],
            grip: vec![1.0; n],
            ds,
            closed: false,
        }
    }

    #[test]
    fn straight_keeps_full_speed_and_corner_keeps_the_margin() {
        let env = test_envelope();
        let n = 200;
        // Half straight at 40 m/s, half a limit corner: v chosen so κv² == the boundary.
        let mut v = vec![40.0; n];
        let mut kappa = vec![0.0; n];
        for i in n / 2..n {
            let vc: f64 = 20.0;
            let ay = env.ay_boundary(vc, 0.0, G);
            v[i] = vc;
            kappa[i] = ay / (vc * vc); // exactly at the lateral limit → u = 1
        }
        let path = flat_path(kappa, 5.0);
        let ax = vec![0.0; n];
        let t = corner_scaled_targets(&env, &path, &v, &ax, 0.85);
        // Deep in the corner: exactly m₀·v.
        assert!((t[n - 1] - 0.85 * 20.0).abs() < 1e-9);
        // Start of the straight (far from the braking zone): full speed.
        assert!((t[0] - 40.0).abs() < 1e-9);
        // Never above the raw profile (the feasibility passes only ever lower it).
        for i in 0..n {
            assert!(t[i] <= v[i] + 1e-9, "above the raw profile at {i}");
        }
    }

    #[test]
    fn braking_into_the_corner_is_feasible_at_the_margined_decel() {
        let env = test_envelope();
        let n = 200;
        let ds = 5.0;
        let mut v = vec![40.0; n];
        let mut kappa = vec![0.0; n];
        for i in n / 2..n {
            let vc: f64 = 18.0;
            v[i] = vc;
            kappa[i] = env.ay_boundary(vc, 0.0, G) / (vc * vc);
        }
        let path = flat_path(kappa, ds);
        let ax = vec![0.0; n];
        let t = corner_scaled_targets(&env, &path, &v, &ax, 0.85);
        // The demanded decel between stations never exceeds m₀²·brake_limit (small tolerance for
        // the capability evaluation moving between the two endpoints).
        for i in 0..n - 1 {
            let dec = (t[i] * t[i] - t[i + 1] * t[i + 1]) / (2.0 * ds);
            let cap = 0.85 * 0.85 * env.brake_limit(t[i + 1], G);
            assert!(
                dec <= cap * 1.05 + 1e-6,
                "decel {dec:.2} > cap {cap:.2} at {i}"
            );
        }
    }

    #[test]
    fn exit_ramp_never_demands_more_than_the_margined_drive() {
        let env = test_envelope();
        let n = 200;
        let ds = 5.0;
        // A corner first, then a straight the target accelerates onto.
        let mut v = vec![40.0; n];
        let mut kappa = vec![0.0; n];
        for i in 0..n / 2 {
            let vc: f64 = 18.0;
            v[i] = vc;
            kappa[i] = env.ay_boundary(vc, 0.0, G) / (vc * vc);
        }
        let path = flat_path(kappa, ds);
        let ax = vec![0.0; n];
        let t = corner_scaled_targets(&env, &path, &v, &ax, 0.85);
        for i in 0..n - 1 {
            let acc = (t[i + 1] * t[i + 1] - t[i] * t[i]) / (2.0 * ds);
            let cap = 0.85 * env.accel_limit(t[i], G);
            assert!(
                acc <= cap * 1.05 + 1e-6,
                "accel {acc:.2} > cap {cap:.2} at {i}"
            );
        }
    }

    #[test]
    fn margin_one_is_the_identity() {
        let env = test_envelope();
        let path = flat_path(vec![0.01; 50], 5.0);
        let v = vec![30.0; 50];
        let ax = vec![0.0; 50];
        assert_eq!(corner_scaled_targets(&env, &path, &v, &ax, 1.0), v);
    }
}
