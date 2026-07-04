// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for [`GriddedMapN`] evaluation (CLAUDE.md: allocs/step is CI-enforced).
//!
//! Construction (axis discovery, NaN fill, partial precompute) may allocate; `eval`, `eval_flagged`
//! and `grad_into` must not. dhat's testing profiler counts heap blocks; we assert the count is
//! unchanged across warmed evaluations — the same pattern as `outlap-tire/tests/alloc.rs`.

#![allow(clippy::many_single_char_names)] // x/y/z/t/v are the map's coordinates and value.

use outlap_core::{GriddedMapN, OutOfDomain};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

/// Build a 3-D map with a NaN cell and mixed out-of-domain modes (exercises the full weight path).
fn build_3d() -> GriddedMapN<f64> {
    let xs = vec![0.0, 1.0, 2.0, 3.0, 4.0];
    let ys = vec![0.0, 1.0, 2.5, 4.0];
    let zs = vec![700.0, 750.0, 800.0, 850.0];
    let mut vals = Vec::new();
    for &x in &xs {
        for &y in &ys {
            for &z in &zs {
                vals.push(0.3 * x * x - 0.5 * y + 0.001 * z + 0.2 * x * y);
            }
        }
    }
    // One NaN cell so the fill/hull path is populated.
    vals[10] = f64::NAN;
    GriddedMapN::from_gridded(
        vec![xs, ys, zs],
        vals,
        vec![OutOfDomain::Clamp, OutOfDomain::Clamp, OutOfDomain::Linear],
    )
    .unwrap()
}

#[test]
fn eval_paths_do_not_allocate() {
    let _profiler = dhat::Profiler::builder().testing().build();
    let map = build_3d();
    let mut grad = [0.0f64; 3];

    // Warm up (touches the same code, so any lazy one-time alloc happens before the window).
    let mut sink = map.eval(&[1.5, 2.0, 780.0]);
    map.grad_into(&[1.5, 2.0, 780.0], &mut grad);
    sink += map.eval_flagged(&[1.5, 2.0, 900.0]).0; // above-grid (Linear extrapolation)

    let before = dhat::HeapStats::get().total_blocks;
    for i in 0..64 {
        #[allow(clippy::cast_precision_loss)]
        let t = f64::from(i) / 64.0;
        let x = -0.5 + 5.0 * t; // spans below/in/above the x grid
        let y = 4.5 * t;
        let z = 680.0 + 200.0 * t; // spans below/in/above the Vdc grid
        let (v, flags) = map.eval_flagged(&[x, y, z]);
        sink +=
            v + f64::from(u8::from(flags.extrapolated)) + f64::from(u8::from(flags.out_of_hull));
        map.grad_into(&[x, y, z], &mut grad);
        sink += grad[0] + grad[1] + grad[2];
    }
    assert_eq!(
        before,
        dhat::HeapStats::get().total_blocks,
        "GriddedMapN eval/grad allocated on the heap"
    );
    assert!(sink.is_finite());
}
