// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the MF6.1 evaluation path (CLAUDE.md: allocs/step is CI-enforced).
//!
//! Construction (map → dense params) may allocate; `Mf61::forces` must not. dhat's testing
//! profiler counts heap blocks; we assert the count is unchanged across warmed evaluations —
//! the same pattern as `outlap-qss/tests/alloc.rs`.

use std::collections::BTreeMap;

use outlap_tire::{Mf61, Mf61Params, SlipState};

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

fn full_map() -> BTreeMap<String, f64> {
    let pairs: &[(&str, f64)] = &[
        ("FNOMIN", 4000.0),
        ("UNLOADED_RADIUS", 0.33),
        ("NOMPRES", 220_000.0),
        ("LONGVL", 16.7),
        ("PCX1", 1.65),
        ("PDX1", 1.30),
        ("PDX2", -0.05),
        ("PEX1", 0.10),
        ("PKX1", 22.0),
        ("PHX1", 0.002),
        ("PVX1", 0.01),
        ("PPX1", -0.3),
        ("PPX3", -0.1),
        ("RBX1", 13.0),
        ("RBX2", 10.0),
        ("RCX1", 1.0),
        ("PCY1", 1.40),
        ("PDY1", 1.25),
        ("PDY3", -1.0),
        ("PEY1", -1.0),
        ("PKY1", -20.0),
        ("PKY2", 1.8),
        ("PKY4", 2.0),
        ("PKY6", -1.0),
        ("PHY1", 0.003),
        ("PVY1", 0.02),
        ("PVY3", -0.2),
        ("PPY1", -0.5),
        ("RBY1", 11.0),
        ("RBY2", 8.0),
        ("RCY1", 1.0),
        ("RVY5", 1.9),
        ("RVY6", -10.0),
        ("QBZ1", 8.0),
        ("QCZ1", 1.1),
        ("QDZ1", 0.09),
        ("QDZ6", 0.002),
        ("QEZ1", -1.0),
        ("QHZ1", 0.002),
        ("QBZ9", 15.0),
        ("SSZ1", 0.02),
        ("QSX1", 0.005),
        ("QSX2", 1.0),
        ("QSX3", 0.05),
        ("QSY1", 0.01),
        ("QSY7", 0.85),
    ];
    pairs.iter().map(|(k, v)| ((*k).to_owned(), *v)).collect()
}

#[test]
fn forces_do_not_allocate() {
    let _profiler = dhat::Profiler::builder().testing().build();

    let (p, _notes) = Mf61Params::<f64>::from_coeffs(&full_map()).unwrap();
    let model = Mf61::new(p);

    // Warm-up evaluation.
    let mut sink = model
        .forces(&SlipState::new(0.05, -0.03, 0.01, 4200.0, 210_000.0, 40.0))
        .fx;

    let before = dhat::HeapStats::get().total_blocks;
    for i in 0..16 {
        #[allow(clippy::cast_precision_loss)]
        let t = f64::from(i) / 16.0;
        let s = SlipState::new(
            -0.2 + 0.4 * t,
            0.15 - 0.3 * t,
            0.02 * t,
            3000.0 + 2000.0 * t,
            200_000.0 + 40_000.0 * t,
            5.0 + 60.0 * t,
        );
        let f = model.forces(&s);
        sink += f.fx + f.fy + f.mz + f.mx + f.my;
    }
    let after = dhat::HeapStats::get().total_blocks;

    assert_eq!(before, after, "Mf61::forces allocated on the heap");
    assert!(sink.is_finite());
}
