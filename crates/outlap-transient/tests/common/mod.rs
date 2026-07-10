// SPDX-License-Identifier: AGPL-3.0-only
//! Shared T2 block-assembly + synthetic-line helpers for the transient integration tests.
//!
//! This is the deliberate **test-assembly path** (the production per-vehicle assembly + real-circuit
//! lap arrive with the Python boundary in PR7): it mirrors what that pipeline will do — build the T2
//! block set from an assembled `T1Vehicle`, tune the ideal driver from the vehicle's own understeer
//! gradient plus the MacAdam/PI literature defaults, and sample the best-gear traction envelope into
//! the shared monotone cubic. The QSS crates are dev-only dependencies (the solver never touches
//! them — it receives sampled tables), so this assembly lives in the tests, not the crate.
#![allow(dead_code)] // shared across several test binaries; not all use every helper.
#![allow(clippy::cast_precision_loss)] // small loop counters → f64 grid coordinates.

use std::path::PathBuf;

use outlap_core::bus::{ChannelInterner, WHEELS};
use outlap_core::interp::MonotoneCubic;
use outlap_qss::t1::LoadTransferGeometry;
use outlap_qss::T1Vehicle;
use outlap_schema::io::FsLoader;
use outlap_schema::vehicle::Driver as DriverCfg;
use outlap_schema::{load_conditions, load_vehicle, LoadOptions};
use outlap_transient::{LineSamples, LineTable, T2Blocks};
use outlap_vehicle::{
    drive_weights, ActuationChannels, Aero, AxleRegen, Chassis, ChassisParams, Driver,
    LoadTransfer, Powertrain, RegenParams, RelaxProvider, RoadChannels, Tire, TorqueVectoring, G,
};

const WHEEL_INERTIA_KGM2: f64 = 1.1;
const FALLBACK_RADIUS_M: f64 = 0.33;
/// Understeer-gradient probe speed for the curvature feed-forward, m/s (a representative mid speed).
const K_US_PROBE_MPS: f64 = 40.0;
/// Traction-envelope sampling grid: 0 → 120 m/s in 2 m/s steps.
const TRACTION_V_MAX_MPS: f64 = 120.0;
const TRACTION_SAMPLES: usize = 61;

/// Absolute path into the repo `data/` tree.
#[must_use]
pub fn data(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data")).join(rel)
}

/// Load + assemble a reference `T1Vehicle` from `data/vehicles/<name>`.
#[must_use]
pub fn assemble(name: &str) -> T1Vehicle {
    let vl = FsLoader::new(data(&format!("vehicles/{name}")));
    let resolved = load_vehicle("vehicle.yaml", &vl, &LoadOptions::default()).unwrap();
    let conditions = load_conditions("conditions.yaml", &vl).unwrap();
    T1Vehicle::assemble(&resolved, &conditions, &vl, false).unwrap()
}

/// The `limebeer_2014_f1` reference car (M4's tuning + parity baseline).
#[must_use]
pub fn limebeer() -> T1Vehicle {
    assemble("limebeer_2014_f1")
}

/// The tuned ideal driver: the MacAdam/PI literature defaults (the same `DEFAULT_*` the schema
/// surfaces as estimated) plus the vehicle's *own* understeer gradient `K_us` in the curvature FF.
#[must_use]
pub fn base_driver(t1: &T1Vehicle, road: RoadChannels) -> Driver<f64> {
    let k_us = t1.understeer_gradient(K_US_PROBE_MPS, G).unwrap_or(0.0);
    Driver {
        wheelbase: t1.wheelbase_m,
        understeer_gradient: k_us,
        preview_time: DriverCfg::DEFAULT_PREVIEW_TIME_S,
        preview_gain: DriverCfg::DEFAULT_PREVIEW_GAIN,
        heading_gain: DriverCfg::DEFAULT_HEADING_GAIN,
        yaw_damping: DriverCfg::DEFAULT_YAW_DAMPING,
        max_steer: DriverCfg::DEFAULT_MAX_STEER_RAD,
        speed_kp: DriverCfg::DEFAULT_SPEED_KP,
        speed_ki: DriverCfg::DEFAULT_SPEED_KI,
        ff_accel_scale: DriverCfg::DEFAULT_FF_ACCEL_SCALE_MPS2,
        slip_limit: DriverCfg::DEFAULT_STABILITY_SLIP_LIMIT_RAD,
        slip_gain: DriverCfg::DEFAULT_STABILITY_SLIP_GAIN,
        // Anti-windup backstop: k_i · limit = 0.05 · 20 = 1.0 pedal of integral authority.
        integral_limit: 20.0,
        road,
    }
}

/// Sample the best-gear wheel-force ceiling `F_drive_max(v)` into the shared monotone cubic
/// (instantaneous ideal shift — the QSS tier already picks the gear at each speed).
#[must_use]
pub fn traction_curve(t1: &T1Vehicle) -> MonotoneCubic<f64> {
    let vs: Vec<f64> = (0..TRACTION_SAMPLES)
        .map(|i| i as f64 * TRACTION_V_MAX_MPS / (TRACTION_SAMPLES as f64 - 1.0))
        .collect();
    let fs: Vec<f64> = vs.iter().map(|&v| t1.max_tractive_force(v)).collect();
    MonotoneCubic::new(vs, fs).expect("monotone traction speed grid")
}

/// The uniform speed grid the envelopes are sampled on, m/s.
fn speed_grid() -> Vec<f64> {
    (0..TRACTION_SAMPLES)
        .map(|i| i as f64 * TRACTION_V_MAX_MPS / (TRACTION_SAMPLES as f64 - 1.0))
        .collect()
}

/// Sample each axle's peak regen **braking** wheel force `F_regen_max(v)` into the shared monotone
/// cubic, one curve per axle: `[front, rear]`. An axle with no machine yields `None` and is braked by
/// friction alone. This is the assembly-time `.ptm` read — the hot loop only ever sees the spline.
#[must_use]
pub fn regen_curves(t1: &T1Vehicle) -> [Option<MonotoneCubic<f64>>; 2] {
    let vs = speed_grid();
    let mut out = [None, None];
    for (axle, slot) in out.iter_mut().enumerate() {
        let fs: Vec<f64> = vs
            .iter()
            .map(|&v| t1.max_regen_force_by_axle(v)[axle])
            .collect();
        // An axle with no machine has an all-zero curve; represent that as "no machine" so the block
        // skips it entirely rather than evaluating a spline that can only return zero.
        if fs.iter().all(|&f| f <= 0.0) {
            continue;
        }
        *slot = Some(MonotoneCubic::new(vs.clone(), fs).expect("monotone regen speed grid"));
    }
    out
}

/// Build per-axle regen parameters from the vehicle's own machines. `authority` is the blend
/// authority (`brakes.regen_blend.max_regen_frac`); `efficiency` the machine+inverter recovery.
#[must_use]
pub fn regen_params(t1: &T1Vehicle, authority: f64, efficiency: f64) -> RegenParams<f64> {
    let [front, rear] = regen_curves(t1);
    let axle = |c: Option<MonotoneCubic<f64>>| {
        c.map(|force_max| AxleRegen {
            force_max,
            efficiency,
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

/// Build the full T2 block set from an assembled vehicle (the tuned ideal driver + minimal
/// actuation). Callers tweak the returned `blocks.driver` / `blocks.powertrain` fields as needed.
#[must_use]
pub fn build_blocks(t1: &T1Vehicle, it: &mut ChannelInterner) -> T2Blocks<f64> {
    let road = RoadChannels::intern(it);
    let actuation = ActuationChannels::intern(it);
    let (a, b, tf, tr) = (t1.a_f, t1.b_r, t1.t_f, t1.t_r);
    let x = [a, a, -b, -b];
    let y = [tf * 0.5, -tf * 0.5, tr * 0.5, -tr * 0.5];
    let r_f = t1.tire_front.unloaded_radius(FALLBACK_RADIUS_M);
    let r_r = t1.tire_rear.unloaded_radius(FALLBACK_RADIUS_M);
    let radius = [r_f, r_f, r_r, r_r];
    let params = ChassisParams::from_f64(
        t1.mass_kg,
        t1.izz,
        x,
        y,
        [true, true, false, false],
        radius,
        [WHEEL_INERTIA_KGM2; WHEELS],
    );
    let geom = LoadTransferGeometry {
        mass_kg: t1.mass_kg,
        wheelbase_m: t1.wheelbase_m,
        a_f: t1.a_f,
        b_r: t1.b_r,
        t_f: t1.t_f,
        t_r: t1.t_r,
        h_cg: t1.h_cg,
        h_ra: t1.h_ra,
        rc_f: t1.rc_f,
        rc_r: t1.rc_r,
        roll_share_f: t1.roll_share_f,
        roll_share_r: t1.roll_share_r,
    };
    let wheels = params.wheels;
    T2Blocks {
        chassis: Chassis::new(params, road),
        tire: Tire {
            front: t1.tire_front.clone(),
            rear: t1.tire_rear.clone(),
            p_front: t1.p_front,
            p_rear: t1.p_rear,
            mu_scale: 1.0,
            relax_front: RelaxProvider::from_model(&t1.tire_front, 0.5 * r_f),
            relax_rear: RelaxProvider::from_model(&t1.tire_rear, 0.5 * r_r),
            wheels,
        },
        aero: Aero {
            qx: t1.qx,
            qz_f: t1.qz_f,
            qz_r: t1.qz_r,
        },
        load: LoadTransfer {
            geom,
            g_normal: G,
            speed: 0.0,
            ax: 0.0,
            ay: 0.0,
            qz_f: t1.qz_f,
            qz_r: t1.qz_r,
        },
        driver: base_driver(t1, road),
        powertrain: Powertrain {
            traction: traction_curve(t1),
            drive_weight: drive_weights(t1.driven, None, None),
            radius,
            max_brake_torque: 6000.0,
            brake_front_bias: t1.brake_front_bias,
            // Regen off by default (the base parity assembly); tests that exercise regen turn it on
            // with `regen_params(t1, authority, efficiency)`.
            regen: RegenParams::disabled(),
            actuation,
        },
        // Torque vectoring off by default (a no-op block); tests enable it explicitly.
        tv: TorqueVectoring {
            enabled: false,
            k_yaw: 0.0,
            max_moment: f64::INFINITY,
            mu: 1.5,
            y,
            radius,
            drive_capable: t1.driven,
            road,
            actuation,
        },
        road,
        actuation,
    }
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
