// SPDX-License-Identifier: AGPL-3.0-only
//! Machine-thermal stability gate (Layer 2 Phase D, D-M6-13): the governed MGU-K's winding-thermal
//! derate feeds back into the ERS deploy, so the deploy loss that heats the winding is itself cut as
//! the winding warms. This test proves that feedback is STABLE — the winding settles at a bounded
//! equilibrium instead of running away (the failure mode the QSS distance march exhibits, and the
//! reason the honest stint-to-stint carry lives at this real-time tier). The production stack wraps
//! the QSS `MachineThermal` LPTN; this double isolates the orchestrator's derate↔loss loop.
#![allow(clippy::cast_precision_loss)]

mod common;

use common::{build_blocks, limebeer};
use outlap_core::bus::ChannelInterner;
use outlap_transient::{
    ErsGovernor, ErsStepInput, ErsStepOut, MachineThermalStack, SimConfig, SlowStack,
    TransientSolver,
};

// A single-node winding thermal double: heat in (the derate-cut deploy loss) minus convective cooling
// (∝ ΔT above ambient), with the standard 1→0 torque derate ramp between `T_WARN` and `T_MAX`. The
// same shape the real `MachineThermal` LPTN presents (winding capacity + jacket cooling + derate).
const AMBIENT_C: f64 = 25.0;
const C_WINDING_J_PER_K: f64 = 800.0; // ≈ the semi-virtual MGU-K's winding capacity
const K_COOL_W_PER_K: f64 = 40.0; // jacket + air convective conductance to ambient
const T_WARN_C: f64 = 150.0;
const T_MAX_C: f64 = 180.0;

struct WindingDouble {
    temp_c: f64,
}

impl MachineThermalStack for WindingDouble {
    fn on_slow_step(&mut self, dt_s: f64, loss_w: f64, _omega_rad_s: f64) {
        let cooling_w = K_COOL_W_PER_K * (self.temp_c - AMBIENT_C);
        self.temp_c += (loss_w - cooling_w) * dt_s / C_WINDING_J_PER_K;
    }
    fn derate(&self) -> f64 {
        ((T_MAX_C - self.temp_c) / (T_MAX_C - T_WARN_C)).clamp(0.0, 1.0)
    }
    fn winding_temp_c(&self) -> f64 {
        self.temp_c
    }
}

// A worst-case deploy governor: deploys EVERY step (no throttle gate), so the winding is driven as
// hard as the derate allows. The deploy loss — the heat — is scaled by the incoming winding derate,
// so this closes the exact feedback the production `ErsController` runs (hotter → less deploy → less
// loss → cools). Without that scaling the loop is open and the winding runs away.
struct DeployDouble {
    loss_at_full_derate_w: f64,
}

impl ErsGovernor for DeployDouble {
    fn decide(&mut self, inp: &ErsStepInput) -> ErsStepOut {
        ErsStepOut {
            deploy_force_n: 400.0 * inp.machine_derate,
            deploy_power_w: 0.0,
            harvest_power_w: 0.0,
            deploy_loss_w: self.loss_at_full_derate_w * inp.machine_derate,
            machine_omega_rad_s: 3000.0,
        }
    }
    fn reset_lap(&mut self) {}
    fn deploy_j(&self) -> f64 {
        0.0
    }
    fn harvest_j(&self) -> f64 {
        0.0
    }
}

// A never-binding pack so the governor keeps deploying (the loop under test is the WINDING, not SoC).
struct PackDouble;
impl SlowStack for PackDouble {
    fn on_slow_step(&mut self, _dt_s: f64, _net_charge_power_w: f64) {}
    fn regen_power_limit_w(&self) -> f64 {
        1.0e12
    }
    fn discharge_power_limit_w(&self) -> f64 {
        1.0e12
    }
    fn soc(&self) -> f64 {
        0.6
    }
    fn temp_c(&self) -> f64 {
        30.0
    }
}

#[test]
fn machine_thermal_deploy_feedback_settles_bounded_no_runaway() {
    let (t1, spec) = limebeer();
    let mut it = ChannelInterner::new();
    let blocks = build_blocks(&t1, &spec, &mut it);

    // A long constant-speed straight: the winding is driven continuously long enough (≈ many thermal
    // time constants, τ = C/k ≈ 20 s) to reach its equilibrium.
    let len = 12_000.0;
    let stations = 240;
    let s: Vec<f64> = (0..stations)
        .map(|i| i as f64 * len / (stations as f64 - 1.0))
        .collect();
    let mk = |v: f64| vec![v; stations];
    let table = outlap_transient::LineTable::new(&outlap_transient::LineSamples {
        s: s.clone(),
        kappa_h: mk(0.0),
        grade: mk(0.0),
        banking: mk(0.0),
        kappa_v: mk(0.0),
        n_ref: mk(0.0),
        kappa_ref: mk(0.0),
        v_ref: mk(60.0),
        x_ref: s.clone(),
        y_ref: mk(0.0),
        z_ref: mk(0.0),
        lat_x: mk(0.0),
        lat_y: mk(1.0),
        lat_z: mk(0.0),
        closed: false,
    })
    .unwrap();

    let cfg = SimConfig {
        fz_coupling: outlap_schema::sim::FzCoupling::OneStepLag,
        ..SimConfig::default()
    };
    let mut solver = TransientSolver::new(blocks, table, &it, cfg)
        .with_slow_stack(Box::new(PackDouble))
        .with_ers_governor(Box::new(DeployDouble {
            // 40 kW continuous machine loss at full deploy — against the 40 W/K jacket that alone
            // would settle near (25 + 40000/40) = 1025 °C, an order of magnitude past the 180 °C
            // limit. The derate is the ONLY thing that keeps it bounded.
            loss_at_full_derate_w: 40_000.0,
        }))
        .with_machine_thermal(Box::new(WindingDouble { temp_c: AMBIENT_C }));

    let _lap = solver.run(len - 50.0, 2_000_000);

    let winding_c = solver
        .machine_winding_temp_c()
        .expect("machine-thermal stack attached");
    // Stability: the derate↔loss↔cooling feedback settles the winding at a bounded equilibrium below
    // the hard limit (it self-limits AT T_MAX, where the derate hits 0 and the loss stops). A runaway
    // (open loop) would climb toward ~1000 °C; here it must stay finite and under T_MAX.
    assert!(
        winding_c.is_finite() && winding_c < T_MAX_C,
        "winding-thermal feedback did not settle (ran away): {winding_c:.1} °C ≥ {T_MAX_C} °C"
    );
    // ...and it genuinely reached the thermally-limited regime (the derate is actually binding),
    // otherwise the test would pass trivially without exercising the feedback.
    assert!(
        winding_c > T_WARN_C,
        "the winding never entered the derate ramp ({winding_c:.1} °C ≤ {T_WARN_C} °C) — the test \
         is not exercising the thermal feedback"
    );
}
