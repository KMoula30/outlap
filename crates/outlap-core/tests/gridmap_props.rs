// SPDX-License-Identifier: AGPL-3.0-only
//! Property tests for [`GriddedMapN`] (Decision #30): node exactness, axis-aligned monotonicity,
//! and analytic-gradient/finite-difference agreement over randomised grids.

use outlap_core::{GriddedMapN, MonotoneCubic, OutOfDomain};
use proptest::prelude::*;

/// A strictly-increasing axis of `n` breakpoints from sorted, gap-separated deltas.
fn axis_strategy(n: usize) -> impl Strategy<Value = Vec<f64>> {
    prop::collection::vec(0.2f64..3.0, n).prop_map(|deltas| {
        let mut xs = Vec::with_capacity(deltas.len());
        let mut acc = -1.0;
        for d in deltas {
            acc += d;
            xs.push(acc);
        }
        xs
    })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(200))]

    /// The map reproduces every tabulated node value exactly.
    #[test]
    fn node_values_exact(
        xs in axis_strategy(5),
        ys in axis_strategy(4),
        raw in prop::collection::vec(-50.0f64..50.0, 20),
    ) {
        let vals: Vec<f64> = (0..xs.len() * ys.len()).map(|i| raw[i % raw.len()]).collect();
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            vals.clone(),
            vec![OutOfDomain::Clamp; 2],
        ).unwrap();
        for (i, &x) in xs.iter().enumerate() {
            for (j, &y) in ys.iter().enumerate() {
                let got = map.eval(&[x, y]);
                let want = vals[i * ys.len() + j];
                prop_assert!((got - want).abs() < 1e-9, "node ({x},{y}): {got} vs {want}");
            }
        }
    }

    /// On a grid-aligned fibre the 2-D map equals `MonotoneCubic` on that fibre (value + slope),
    /// so it inherits shape preservation: a monotone column stays monotone with no overshoot.
    #[test]
    fn fibre_equals_monotone_cubic_and_preserves_monotonicity(
        xs in axis_strategy(6),
        ys in axis_strategy(3),
        seed in 0u64..1_000,
    ) {
        // Build values strictly increasing along x (index i) for every fixed y: a cumulative sum of
        // strictly-positive, seed-varied increments plus a per-column offset.
        let mut cum = vec![0.0f64; xs.len()];
        for i in 1..xs.len() {
            #[allow(clippy::cast_precision_loss)]
            let step = 1.0 + ((seed.wrapping_add(i as u64)) % 5) as f64;
            cum[i] = cum[i - 1] + step;
        }
        let mut vals = Vec::with_capacity(xs.len() * ys.len());
        #[allow(clippy::needless_range_loop)] // row-major fill; `i` indexes cum, `j` is the column.
        for i in 0..xs.len() {
            for j in 0..ys.len() {
                #[allow(clippy::cast_precision_loss)]
                let v = cum[i] + (j as f64) * 0.5;
                vals.push(v);
            }
        }
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            vals.clone(),
            vec![OutOfDomain::Clamp; 2],
        ).unwrap();
        for (j, &y0) in ys.iter().enumerate() {
            let fibre: Vec<f64> = (0..xs.len()).map(|i| vals[i * ys.len() + j]).collect();
            let mc = MonotoneCubic::new(xs.clone(), fibre).unwrap();
            let lo = xs[0];
            let hi = xs[xs.len() - 1];
            let mut prev = f64::NEG_INFINITY;
            for k in 0..=80 {
                let x = lo + (hi - lo) * f64::from(k) / 80.0;
                let v = map.eval(&[x, y0]);
                prop_assert!((v - mc.eval(x)).abs() < 1e-9, "fibre != MonotoneCubic at ({x},{y0})");
                prop_assert!(v >= prev - 1e-9, "non-monotone fibre at x={x}, y={y0}");
                prev = v;
            }
        }
    }

    /// The analytic gradient matches central finite differences at interior points.
    #[test]
    fn analytic_gradient_matches_fd(
        xs in axis_strategy(5),
        ys in axis_strategy(4),
        fx in 0.15f64..0.85,
        fy in 0.15f64..0.85,
    ) {
        // A smooth non-separable field sampled on the grid.
        let f = |x: f64, y: f64| (0.4 * x - 0.2 * y).sin() + 0.3 * x * y + x;
        let mut vals = Vec::with_capacity(xs.len() * ys.len());
        for &x in &xs {
            for &y in &ys {
                vals.push(f(x, y));
            }
        }
        let map = GriddedMapN::from_gridded(
            vec![xs.clone(), ys.clone()],
            vals,
            vec![OutOfDomain::Clamp; 2],
        ).unwrap();
        let x = xs[0] + (xs[xs.len() - 1] - xs[0]) * fx;
        let y = ys[0] + (ys[ys.len() - 1] - ys[0]) * fy;
        let h = 1e-6;
        let mut g = [0.0, 0.0];
        map.grad_into(&[x, y], &mut g);
        let fdx = (map.eval(&[x + h, y]) - map.eval(&[x - h, y])) / (2.0 * h);
        let fdy = (map.eval(&[x, y + h]) - map.eval(&[x, y - h])) / (2.0 * h);
        prop_assert!((g[0] - fdx).abs() < 1e-3, "d/dx: {} vs {fdx}", g[0]);
        prop_assert!((g[1] - fdy).abs() < 1e-3, "d/dy: {} vs {fdy}", g[1]);
    }
}
