// SPDX-License-Identifier: AGPL-3.0-only
//! Shared T2 block-assembly + synthetic-line helpers for the transient integration tests.
//!
//! The block set comes from the **shared production assembly** (`outlap_vehicle::assemble_t2`), the
//! same path the Python boundary takes, so the tests and the shipped pipeline cannot drift. Only the
//! synthetic line tables below are test-only. The QSS crates stay dev-only dependencies here (the
//! solver never touches them — it receives sampled tables).
#![allow(dead_code)] // shared across several test binaries; not all use every helper.
#![allow(clippy::cast_precision_loss)] // small loop counters → f64 grid coordinates.

use std::path::PathBuf;

use outlap_core::bus::ChannelInterner;
use outlap_core::interp::MonotoneCubic;
use outlap_qss::T1Vehicle;
use outlap_schema::io::FsLoader;
use outlap_schema::vehicle::Vehicle;
use outlap_schema::{load_conditions, load_vehicle, LoadOptions};
use outlap_transient::{LineSamples, LineTable, T2Blocks};
use outlap_vehicle::{AxleRegen, RegenParams, T2Options};

/// Absolute path into the repo `data/` tree.
#[must_use]
pub fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

/// Load + assemble a reference `T1Vehicle` from `data/vehicles/<name>`, alongside the resolved
/// vehicle document the shared assembly pipeline reads its `driver`/`brakes`/`control` sections from.
#[must_use]
pub fn assemble_car(name: &str) -> (T1Vehicle, Vehicle) {
    let vl = FsLoader::new(data(&format!("vehicles/{name}")));
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default()).unwrap();
    let conditions = load_conditions("conditions.yaml", &vl).unwrap();
    let t1 = T1Vehicle::assemble(&resolved, &conditions, &vl, false).unwrap();
    (t1, resolved.spec)
}

/// Load + assemble a reference `T1Vehicle` from `data/vehicles/<name>`.
#[must_use]
pub fn assemble(name: &str) -> T1Vehicle {
    assemble_car(name).0
}

/// The `limebeer_2014_f1` reference car (M4's tuning + parity baseline) + its vehicle document.
#[must_use]
pub fn limebeer() -> (T1Vehicle, Vehicle) {
    assemble_car("limebeer_2014_f1")
}

/// Assembly options for the test harness (the shared production defaults, with a battery assumed
/// present so the regen tests can turn the blend on).
#[must_use]
pub fn test_opts() -> T2Options {
    T2Options {
        battery_present: true,
        ..T2Options::default()
    }
}

/// Sample the best-gear wheel-force ceiling `F_drive_max(v)` into the shared monotone cubic.
#[must_use]
pub fn traction_curve(t1: &T1Vehicle) -> MonotoneCubic<f64> {
    outlap_vehicle::traction_curve(t1, &test_opts())
}

/// Build per-axle regen parameters directly from the vehicle's own machines, overriding the blend
/// authority and recovery efficiency (the vehicle document's `regen_blend` may be absent).
#[must_use]
pub fn regen_params(t1: &T1Vehicle, authority: f64, efficiency: f64) -> RegenParams<f64> {
    let opts = T2Options {
        regen_efficiency: efficiency,
        ..test_opts()
    };
    let [front, rear] = outlap_vehicle::assembly::regen_curves(t1, &opts);
    // A flat efficiency curve at the requested constant (the pre-PR4 proxy) — the tests that use
    // this helper assert energy = mech·η at a single constant η.
    let eff = MonotoneCubic::new(
        vec![0.0, opts.envelope_v_max_mps],
        vec![efficiency, efficiency],
    )
    .expect("flat efficiency curve");
    let axle = |c: Option<MonotoneCubic<f64>>| {
        c.map(|force_max| AxleRegen {
            force_max,
            efficiency: eff.clone(),
            authority,
        })
    };
    let (front, rear) = (axle(front), axle(rear));
    RegenParams {
        enabled: front.is_some() || rear.is_some(),
        front,
        rear,
    }
}

/// Build the full T2 block set through the **shared production assembly** (`assemble_t2`), so the
/// tests exercise the same path the Python boundary does. Callers tweak the returned blocks' fields.
#[must_use]
pub fn build_blocks(t1: &T1Vehicle, spec: &Vehicle, it: &mut ChannelInterner) -> T2Blocks<f64> {
    let mut notes = Vec::new();
    outlap_vehicle::assemble_t2(t1, spec, it, &test_opts(), &mut notes).into()
}

/// A synthetic line: a closed circle of `circle_radius` or a straight (`None`) along +x. `road_k` is
/// the road curvature the chassis frame follows; `steer_k` is the constant `κ_ref` the FF steers to;
/// `n_ref` is held at 0.
#[must_use]
pub fn line(
    len: f64,
    stations: usize,
    closed: bool,
    road_k: f64,
    steer_k: f64,
    v_ref: f64,
    circle: Option<f64>,
) -> LineTable<f64> {
    let s: Vec<f64> = (0..stations)
        .map(|i| i as f64 * len / (stations as f64 - 1.0))
        .collect();
    let mk = |v: f64| vec![v; stations];
    let (mut xr, mut yr, mut lx, mut ly) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for &si in &s {
        if let Some(r) = circle {
            let th = si / r;
            xr.push(r * th.sin());
            yr.push(r * (1.0 - th.cos()));
            lx.push(-th.sin());
            ly.push(th.cos());
        } else {
            xr.push(si);
            yr.push(0.0);
            lx.push(0.0);
            ly.push(1.0);
        }
    }
    LineTable::new(&LineSamples {
        s,
        kappa_h: mk(road_k),
        grade: mk(0.0),
        banking: mk(0.0),
        kappa_v: mk(0.0),
        n_ref: mk(0.0),
        kappa_ref: mk(steer_k),
        v_ref: mk(v_ref),
        x_ref: xr,
        y_ref: yr,
        z_ref: mk(0.0),
        lat_x: lx,
        lat_y: ly,
        lat_z: mk(0.0),
        closed,
    })
    .unwrap()
}
