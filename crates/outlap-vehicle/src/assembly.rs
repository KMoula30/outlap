// SPDX-License-Identifier: AGPL-3.0-only
//! The **T2 assembly pipeline**: build the transient block set from the one canonical vehicle
//! description (HANDOFF §6.1, Decision #4 — all tiers evaluate the same parameter objects).
//!
//! Everything config-shaped happens here and nowhere else (CLAUDE.md hot-loop discipline): the
//! `.ptm` traction and regen envelopes are sampled into the shared monotone cubic, the driver gains
//! are resolved from the vehicle's own `driver:` section against the literature defaults, the static
//! drive split is folded, and the channels are interned. The blocks that come out are pure data.
//!
//! This crate can do all of it because it already depends on `outlap-qss` (the T1 trim/envelope
//! algebra) and stays wasm-clean. It cannot name [`T2Blocks`](outlap_transient::T2Blocks) — that type
//! lives one crate up — so it returns [`T2Parts`], which `outlap-transient` converts. The transient
//! solver therefore never gains a QSS dependency, and the test harness and the Python boundary share
//! *one* assembly path rather than each growing their own.
//!
//! Estimated values (a representative friction coefficient, a maximum brake torque the schema does
//! not carry) are pushed onto the caller's `notes` and surface in the loaded-model report — nothing
//! silent (Decision #41).

use outlap_core::bus::{ChannelInterner, WHEELS};
use outlap_core::interp::MonotoneCubic;
use outlap_qss::t1::LoadTransferGeometry;
use outlap_qss::T1Vehicle;
use outlap_schema::vehicle::{Driver as DriverCfg, Vehicle};

use crate::chassis::Chassis;
use crate::control::{drive_weights, AxleRegen, Driver, Powertrain, RegenParams, TorqueVectoring};
use crate::forces::{Aero, LoadTransfer, RelaxProvider, Tire};
use crate::params::{ActuationChannels, ChassisParams, RoadChannels, G};

/// Fallback rolling radius when a tyre model carries no unloaded radius, m.
const FALLBACK_RADIUS_M: f64 = 0.33;
/// Speed the understeer gradient is probed at for the curvature feed-forward, m/s.
const K_US_PROBE_MPS: f64 = 40.0;

/// Assembly knobs the vehicle document does not carry. Every non-obvious default is surfaced as an
/// estimate in the loaded-model report.
#[derive(Clone, Copy, Debug)]
pub struct T2Options {
    /// Per-wheel spin inertia (rim + tyre + hub-side driveline), kg·m².
    pub wheel_inertia_kgm2: f64,
    /// Top of the speed grid the traction/regen envelopes are sampled on, m/s.
    pub envelope_v_max_mps: f64,
    /// Number of samples on that grid.
    pub envelope_samples: usize,
    /// Maximum total friction-brake torque, N·m (the schema carries disc thermal data, not a torque
    /// ceiling, so this is an estimate).
    pub max_brake_torque_nm: f64,
    /// Representative peak grip `μ` — the torque-vectoring allocator's friction-ellipse radius
    /// coefficient (an estimate; the per-wheel combined-slip surface arrives with the QP allocator).
    pub mu: f64,
    /// Mechanical→electrical regen recovery efficiency (machine + inverter), `0..1`.
    pub regen_efficiency: f64,
    /// Anti-windup clamp on the driver's speed integral, m.
    pub integral_limit: f64,
    /// Whether a battery is present, so regen can recover into something.
    pub battery_present: bool,
}

impl Default for T2Options {
    fn default() -> Self {
        Self {
            wheel_inertia_kgm2: 1.1,
            envelope_v_max_mps: 120.0,
            envelope_samples: 61,
            max_brake_torque_nm: 6000.0,
            mu: 1.5,
            regen_efficiency: 0.9,
            integral_limit: 20.0,
            battery_present: false,
        }
    }
}

/// The assembled T2 blocks, before `outlap-transient` packs them into its `T2Blocks`.
pub struct T2Parts<T> {
    /// The chassis RHS block.
    pub chassis: Chassis<T>,
    /// The tyre block.
    pub tire: Tire<T>,
    /// The aero block.
    pub aero: Aero<T>,
    /// The load-transfer (algebraic `F_z`) block.
    pub load: LoadTransfer<T>,
    /// The ideal-driver block.
    pub driver: Driver<T>,
    /// The powertrain (drive/brake actuation + series regen blend) block.
    pub powertrain: Powertrain<T>,
    /// The torque-vectoring allocator block (a no-op when the vehicle does not enable it).
    pub tv: TorqueVectoring<T>,
    /// The interned road channels.
    pub road: RoadChannels,
    /// The interned actuation channels.
    pub actuation: ActuationChannels,
}

/// The uniform speed grid the envelopes are sampled on, m/s.
fn speed_grid(opts: &T2Options) -> Vec<f64> {
    let n = opts.envelope_samples.max(2);
    #[allow(clippy::cast_precision_loss)] // small sample counts
    (0..n)
        .map(|i| i as f64 * opts.envelope_v_max_mps / (n as f64 - 1.0))
        .collect()
}

/// Sample the best-gear wheel-force ceiling `F_drive_max(v)` into the shared monotone cubic
/// (instantaneous ideal shift — the QSS tier already picks the gear at each speed).
///
/// # Panics
/// Panics if the speed grid is not strictly ascending (it is, by construction).
#[must_use]
pub fn traction_curve(t1: &T1Vehicle, opts: &T2Options) -> MonotoneCubic<f64> {
    let vs = speed_grid(opts);
    let fs: Vec<f64> = vs.iter().map(|&v| t1.max_tractive_force(v)).collect();
    MonotoneCubic::new(vs, fs).expect("monotone traction speed grid")
}

/// Sample each axle's peak regen **braking** wheel force into the shared monotone cubic:
/// `[front, rear]`. An axle with no machine yields `None` and is braked by friction alone.
///
/// # Panics
/// Panics if the speed grid is not strictly ascending (it is, by construction).
#[must_use]
pub fn regen_curves(t1: &T1Vehicle, opts: &T2Options) -> [Option<MonotoneCubic<f64>>; 2] {
    let vs = speed_grid(opts);
    let mut out = [None, None];
    for (axle, slot) in out.iter_mut().enumerate() {
        let fs: Vec<f64> = vs
            .iter()
            .map(|&v| t1.max_regen_force_by_axle(v)[axle])
            .collect();
        // An axle with no machine has an all-zero curve; call that "no machine" so the block skips it
        // rather than evaluating a spline that can only return zero.
        if fs.iter().all(|&f| f <= 0.0) {
            continue;
        }
        *slot = Some(MonotoneCubic::new(vs.clone(), fs).expect("monotone regen speed grid"));
    }
    out
}

/// Sample an axle machine's recovery/motoring efficiency into a speed-indexed monotone cubic
/// (M6/PR4): the machine's `.ptm` efficiency map at the peak-torque operating point per speed, or
/// the documented constant `opts.regen_efficiency` where the machine carries no efficiency map (so a
/// car without a mapped drive unit stays byte-identical to the pre-PR4 flat-efficiency block). The
/// hot loop evaluates this pre-sampled curve and never touches a `.ptm` table.
#[must_use]
fn efficiency_curve(t1: &T1Vehicle, opts: &T2Options, axle: usize) -> MonotoneCubic<f64> {
    let vs = speed_grid(opts);
    let fs: Vec<f64> = vs
        .iter()
        .map(|&v| t1.machine_efficiency_by_axle(v)[axle].unwrap_or(opts.regen_efficiency))
        .collect();
    MonotoneCubic::new(vs, fs).expect("monotone efficiency speed grid")
}

/// Build the per-axle series regen blend from the vehicle's own machines and its
/// `brakes.regen_blend` policy. Disabled when the car has no battery to recover into, or no machine.
#[must_use]
pub fn regen_params(t1: &T1Vehicle, spec: &Vehicle, opts: &T2Options) -> RegenParams<f64> {
    let Some(blend) = spec.brakes.regen_blend.as_ref() else {
        return RegenParams::disabled();
    };
    if !opts.battery_present {
        return RegenParams::disabled();
    }
    let [front, rear] = regen_curves(t1, opts);
    let axle = |axle_idx: usize, c: Option<MonotoneCubic<f64>>| {
        c.map(|force_max| AxleRegen {
            force_max,
            efficiency: efficiency_curve(t1, opts, axle_idx),
            authority: blend.max_regen_frac,
        })
    };
    let (front, rear) = (axle(0, front), axle(1, rear));
    RegenParams {
        enabled: front.is_some() || rear.is_some(),
        front,
        rear,
    }
}

/// The ideal driver, tuned from the vehicle's own `driver:` section (unset gains take the MacAdam/PI
/// literature defaults) plus the vehicle's own understeer gradient in the curvature feed-forward.
#[must_use]
pub fn driver(t1: &T1Vehicle, spec: &Vehicle, road: RoadChannels, opts: &T2Options) -> Driver<f64> {
    let cfg = spec.driver.clone().unwrap_or_default();
    Driver {
        wheelbase: t1.wheelbase_m,
        understeer_gradient: t1.understeer_gradient(K_US_PROBE_MPS, G).unwrap_or(0.0),
        preview_time: cfg.preview_time_s(),
        preview_gain: cfg.preview_gain(),
        heading_gain: cfg.heading_gain(),
        yaw_damping: cfg.yaw_damping(),
        max_steer: cfg.max_steer_rad(),
        speed_kp: cfg.speed_kp(),
        speed_ki: cfg.speed_ki(),
        ff_accel_scale: cfg.ff_accel_scale_mps2(),
        slip_limit: cfg.stability_slip_limit_rad(),
        slip_gain: cfg.stability_slip_gain(),
        sideslip_damping: cfg.sideslip_damping(),
        traction_slip_limit: cfg.traction_slip_limit(),
        traction_slip_gain: cfg.traction_slip_gain(),
        integral_limit: opts.integral_limit,
        road,
    }
}

/// The roll-centre / roll-stiffness geometry the algebraic load-transfer block needs, lifted from the
/// same T1 quasi-static algebra both tiers share (HANDOFF §6.1 "one vehicle description").
fn load_transfer_geometry(t1: &T1Vehicle) -> LoadTransferGeometry {
    LoadTransferGeometry {
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
    }
}

/// The tyre block: the vehicle's own front/rear models plus their relaxation-length providers (the
/// contact patch lags a slip step by `σ/v`, so the transient tier cannot use the steady-state curve).
fn tire_block(
    t1: &T1Vehicle,
    wheels: crate::params::WheelGeometry<f64>,
    r_f: f64,
    r_r: f64,
) -> Tire<f64> {
    Tire {
        front: t1.tire_front.clone(),
        rear: t1.tire_rear.clone(),
        p_front: t1.p_front,
        p_rear: t1.p_rear,
        mu_scale: 1.0,
        relax_front: RelaxProvider::from_model(&t1.tire_front, 0.5 * r_f),
        relax_rear: RelaxProvider::from_model(&t1.tire_rear, 0.5 * r_r),
        wheels,
        thermal: None, // frozen-tire until the T2 tire-thermal stack wires it (M5 PR3)
    }
}

/// Assemble the full T2 block set from an assembled [`T1Vehicle`] and its resolved vehicle document.
///
/// Estimated values are pushed onto `notes` for the loaded-model report.
///
/// # Panics
/// Panics if no wheel is driven (an assembly-time topology error the loader rejects earlier).
#[must_use]
pub fn assemble_t2(
    t1: &T1Vehicle,
    spec: &Vehicle,
    interner: &mut ChannelInterner,
    opts: &T2Options,
    notes: &mut Vec<String>,
) -> T2Parts<f64> {
    let road = RoadChannels::intern(interner);
    let actuation = ActuationChannels::intern(interner);

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
        [opts.wheel_inertia_kgm2; WHEELS],
    );
    let geom = load_transfer_geometry(t1);
    let wheels = params.wheels;

    notes.push(format!(
        "T2 assembly: maximum friction-brake torque {:.0} N·m and per-wheel spin inertia {:.2} kg·m² \
         are estimated (the vehicle document carries neither)",
        opts.max_brake_torque_nm, opts.wheel_inertia_kgm2
    ));

    let split = &spec.drivetrain.control.split;
    let regen = regen_params(t1, spec, opts);
    if spec.brakes.regen_blend.is_some() && !regen.enabled {
        notes.push(
            "brakes.regen_blend is declared but no regen is possible (no battery, or no electric \
             machine on any driven axle); the friction brakes do all the braking"
                .to_owned(),
        );
    }

    let tv_cfg = &spec.drivetrain.control.torque_vectoring;
    if tv_cfg.enabled {
        notes.push(format!(
            "torque vectoring: friction-ellipse radius μ = {:.2} is a representative constant \
             (estimated); the per-wheel combined-slip surface arrives with the QP allocator",
            opts.mu
        ));
    }

    T2Parts {
        chassis: Chassis::new(params, road),
        tire: tire_block(t1, wheels, r_f, r_r),
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
        driver: driver(t1, spec, road, opts),
        powertrain: Powertrain {
            traction: traction_curve(t1, opts),
            drive_weight: drive_weights(t1.driven, split.front, split.left),
            radius,
            max_brake_torque: opts.max_brake_torque_nm,
            brake_front_bias: t1.brake_front_bias,
            regen,
            // A car with an `ers:` block is governed by the rule-based energy manager (a
            // schema-derived FACT, not an estimate — PR4i): the boundary controller schedules the
            // MGU-K deploy and owns the pack electrical accounting.
            ers_governed: spec.ers.is_some(),
            actuation,
        },
        tv: TorqueVectoring {
            enabled: tv_cfg.enabled,
            k_yaw: tv_cfg.k_yaw,
            max_moment: tv_cfg.max_yaw_moment_nm.unwrap_or(f64::INFINITY),
            mu: opts.mu,
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

/// The literature-default driver gains, for callers that want to report what was estimated.
#[must_use]
pub fn driver_defaults() -> DriverCfg {
    DriverCfg::default()
}
