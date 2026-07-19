// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the T0 solve kernel (CLAUDE.md: allocs/step is CI-enforced).
//!
//! `solve_into` runs on a pre-allocated workspace and must not allocate. dhat's testing profiler
//! counts heap blocks; we assert the count is unchanged across a warmed solve. This is the template
//! for the hot-loop zero-alloc discipline.
#![allow(clippy::many_single_char_names)]

use std::f64::consts::PI;

use outlap_core::GriddedTable;
use outlap_qss::path::T0Path;
use outlap_qss::solver::{solve_into, solve_into_ggv, solve_into_ggv_coupled};
use outlap_qss::{
    GgvEnvelope, MachineThermal, Pack, PackState, T0Options, T0Vehicle, T0Workspace, T1Vehicle,
    TrimInput,
};
use outlap_schema::centerline::{Centerline, CenterlineRow};
use outlap_schema::io::MemLoader;
use outlap_schema::refs::CenterlineRef;
use outlap_schema::sim::Envelope as EnvelopeRes;
use outlap_schema::track::{TrackDoc, TrackMeta};
use outlap_schema::version::SchemaVersion;
use outlap_schema::{load_vehicle, Conditions, LoadOptions};
use outlap_track::Track;

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

const SLICK: &str = include_str!("../../outlap-schema/tests/fixtures/tyr/slick.tyr.yaml");

fn setup() -> (T0Vehicle, T0Path) {
    let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
        axes: {speed_rpm: [0.0, 30000.0], load_axis: {torque_nm: [0.0, 800.0]}, torque_nm: [0.0, 800.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 30000.0], torque_nm: [800.0, 800.0]}}\n\
        inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
    let veh = "schema: vehicle/1.0\nname: t\n\
        chassis: {mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}\n\
        aero: {map: a.parquet, axes: [], constant: {cx_a_m2: 1.0, cz_front_a_m2: 0.0, cz_rear_a_m2: 3.0}}\n\
        suspension: {model: lumped_kc, front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}, rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}\n\
        tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}\n\
        drivetrain: {units: [{source: ptm/u.ptm.yaml, path: [{fixed_ratio: 4.0}], wheels: [RL, RR]}]}\n\
        brakes: {balance_bar: 0.6, disc: {front: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}, rear: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}}\n";
    let loader = MemLoader::new()
        .with("vehicle.yaml", veh)
        .with("ptm/u.ptm.yaml", ptm)
        .with("tyr/slick.tyr.yaml", SLICK);
    let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    let veh =
        T0Vehicle::assemble(&rv, &Conditions::default(), &loader, &T0Options::default()).unwrap();

    let r = 120.0;
    let n = 400;
    let rows: Vec<CenterlineRow> = (0..n)
        .map(|i| {
            let th = 2.0 * PI * f64::from(i) / f64::from(n);
            CenterlineRow {
                s_m: r * th,
                x_m: r * th.cos(),
                y_m: r * th.sin(),
                z_m: 0.0,
                banking_deg: 0.0,
                width_left_m: 6.0,
                width_right_m: 6.0,
                grip_scale: 1.0,
            }
        })
        .collect();
    let doc = TrackDoc {
        schema: SchemaVersion::new("track", 1, 0),
        name: "c".into(),
        closed: true,
        centerline: CenterlineRef("m".into()),
        banking_keypoints: vec![],
        meta: TrackMeta::default(),
    };
    let track = Track::from_doc(&doc, &Centerline { rows }).unwrap();
    let path = T0Path::from_track(&track, 2.0);
    (veh, path)
}

/// Assemble a T1 vehicle from the same in-memory fixture as [`setup`].
fn setup_t1() -> T1Vehicle {
    let ptm = "schema: ptm/1.0\nkind: drive_unit\n\
        axes: {speed_rpm: [0.0, 30000.0], load_axis: {torque_nm: [0.0, 800.0]}, torque_nm: [0.0, 800.0]}\n\
        tables: {file: x.parquet}\n\
        limits: {max_torque_nm_vs_speed: {speed_rpm: [0.0, 30000.0], torque_nm: [800.0, 800.0]}}\n\
        inertia_kgm2: 0.05\nmass_kg: 60.0\nmeta: {upstream_ratio_applied: false}\n";
    let veh = "schema: vehicle/1.0\nname: t\n\
        chassis: {mass_kg: 1000.0, cg: [1.4, 0.0, 0.3], inertia: [100.0, 400.0, 450.0], wheelbase_m: 2.8, track_m: [1.6, 1.6]}\n\
        aero: {map: a.parquet, axes: [], constant: {cx_a_m2: 1.0, cz_front_a_m2: 0.0, cz_rear_a_m2: 3.0}}\n\
        suspension: {model: lumped_kc, front: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}, rear: {ride_rate_n_per_m: 30000.0, roll_stiffness_share: 0.5, roll_center_height_m: 0.05}}\n\
        tires: {front: tyr/slick.tyr.yaml, rear: tyr/slick.tyr.yaml}\n\
        drivetrain: {units: [{source: ptm/u.ptm.yaml, path: [{fixed_ratio: 4.0}], wheels: [RL, RR]}]}\n\
        brakes: {balance_bar: 0.6, disc: {front: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}, rear: {thermal_capacity_j_per_k: 40000.0, cooling_area_m2: 0.1}}}\n";
    let loader = MemLoader::new()
        .with("vehicle.yaml", veh)
        .with("ptm/u.ptm.yaml", ptm)
        .with("tyr/slick.tyr.yaml", SLICK);
    let rv = load_vehicle("vehicle.yaml", &loader, &LoadOptions::default()).unwrap();
    T1Vehicle::assemble(&rv, &Conditions::default(), &loader, false).unwrap()
}

/// Install a tiny 2-axis (ride-height) aero map on a T1 vehicle so the trim exercises the
/// aero-platform equilibrium fixed point (which must also stay zero-allocation).
fn install_map(car: &mut T1Vehicle) {
    let names = ["ride_height_f_mm", "ride_height_r_mm"];
    let (mut hf, mut hr, mut czf, mut czr, mut cx) = (vec![], vec![], vec![], vec![], vec![]);
    for &a in &[10.0_f64, 30.0, 60.0] {
        for &b in &[30.0_f64, 70.0, 140.0] {
            hf.push(a);
            hr.push(b);
            czf.push(0.0 + (60.0 - a) / 60.0); // rises as the front lowers
            czr.push(3.0 * (1.0 + (140.0 - b) / 140.0));
            cx.push(1.0);
        }
    }
    let cols = vec![
        ("ride_height_f_mm".to_owned(), hf),
        ("ride_height_r_mm".to_owned(), hr),
        ("cz_front_a_m2".to_owned(), czf),
        ("cz_rear_a_m2".to_owned(), czr),
        ("cx_a_m2".to_owned(), cx),
    ];
    let table = outlap_core::GriddedTable::from_long(&cols, &names).unwrap();
    let axes: Vec<String> = names.iter().map(|s| (*s).to_owned()).collect();
    car.install_aero_map(&table, &axes).unwrap();
}

/// The dhat testing profiler is process-global, so all hot paths share ONE profiler here: separate
/// `#[test]`s would race under the parallel runner (same pattern as `outlap-tire/tests/alloc.rs`).
/// Each kernel is measured in its own before/after window.
#[test]
#[allow(clippy::too_many_lines)] // one linear sequence of before/after windows; splitting races dhat.
fn hot_paths_are_zero_alloc() {
    let _profiler = dhat::Profiler::builder().testing().build();

    // --- T0 velocity-profile solve ---
    let (veh, path) = setup();
    let mut ws = T0Workspace::for_path(&path);
    solve_into(&veh, &path, &mut ws).unwrap(); // warm

    let before = dhat::HeapStats::get();
    for _ in 0..16 {
        solve_into(&veh, &path, &mut ws).unwrap();
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "solve_into allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );

    // --- g-g-g-v envelope query + T0-on-envelope velocity-profile solve ---
    // The envelope is generated cold (allocations allowed); the per-lap solve and the boundary
    // queries must not allocate.
    let t1 = setup_t1();
    let env_res = EnvelopeRes {
        v_points: 4,
        ax_points: 5,
        g_normal_points: 2,
    };
    let env = GgvEnvelope::generate(&t1, &env_res, outlap_schema::sim::FzCoupling::OneStepLag)
        .expect("envelope generates");
    solve_into_ggv(&veh, &env, &path, &mut ws).unwrap(); // warm

    let before = dhat::HeapStats::get();
    for _ in 0..16 {
        solve_into_ggv(&veh, &env, &path, &mut ws).unwrap();
        let _ = env.ay_boundary(40.0, -2.0, 9.81);
        let _ = env.ay_boundary_corrected(40.0, 1.0, 11.0, 1.1, 980.0, 1.2);
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "solve_into_ggv / envelope eval allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );

    // --- T1 double-track trim (Levenberg–Marquardt + FD Jacobian; homotopy continuation) ---
    let car = setup_t1();
    let _ = car.trim(&TrimInput::flat(40.0, 8.0, -3.0)); // warm the fast path
    let _ = car.trim(&TrimInput::flat(7.0, 11.0, 0.0)); // warm the continuation fallback

    let before = dhat::HeapStats::get();
    for i in 0..16 {
        let ay = -8.0 + f64::from(i);
        let _ = car.trim(&TrimInput::flat(40.0, ay, -2.0)); // fast direct solve
        let _ = car.trim(&TrimInput::flat(7.0, 8.0 + 0.3 * f64::from(i), 0.0)); // continuation path
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "T1Vehicle::trim allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );

    // --- T1 trim with a ride-height aero map (the platform equilibrium fixed point) ---
    let mut mapped = setup_t1();
    install_map(&mut mapped); // cold: allocations allowed here
    let _ = mapped.trim(&TrimInput::flat(40.0, 8.0, -3.0)); // warm
    let _ = mapped.trim(&TrimInput::flat(7.0, 11.0, 0.0));

    let before = dhat::HeapStats::get();
    for i in 0..16 {
        let ay = -8.0 + f64::from(i);
        let _ = mapped.trim(&TrimInput::flat(40.0, ay, -2.0));
        let _ = mapped.trim(&TrimInput::flat(7.0, 8.0 + 0.3 * f64::from(i), 0.0));
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "T1Vehicle::trim (mapped aero) allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );

    // --- Machine thermal slow-state advance (Crank–Nicolson step) ---
    let mut thermal = setup_thermal();
    let _ = thermal.step(3000.0, |_| None, 800.0, 0.1); // warm

    let before = dhat::HeapStats::get();
    for _ in 0..16 {
        let _ = thermal.step(3000.0, |_| None, 800.0, 0.1);
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "MachineThermal::step allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );

    // --- Battery slow-state advance (SoC / RC / temperature per segment) ---
    let (pack, mut st) = setup_pack();
    let _ = pack.step_power(&mut st, 100_000.0, 1.0); // warm

    let before = dhat::HeapStats::get();
    for _ in 0..16 {
        let _ = pack.step_power(&mut st, 100_000.0, 1.0);
        let _ = pack.step_current(&mut st, 200.0, 1.0);
        let _ = pack.terminal_voltage_v(&st);
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "Pack slow-state advance allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );

    // --- Coupled traction-scale solve + the per-segment traction-energy lookup (PR8) ---
    // `solve_into_ggv_scaled` is the coupled velocity-profile kernel; `traction_energy` is the
    // per-segment aggregate the slow-state march evaluates. Both must not allocate. (The march's
    // per-lap workspace vectors and thermal reset are cold, per-lap allocations by design.)
    let scale = vec![0.97; path.len()];
    outlap_qss::solver::solve_into_ggv_scaled(&veh, &env, &scale, &path, &mut ws).unwrap(); // warm
    let _ = t1.powertrain().traction_energy(30.0, 2000.0, Some(760.0));

    let before = dhat::HeapStats::get();
    for _ in 0..16 {
        outlap_qss::solver::solve_into_ggv_scaled(&veh, &env, &scale, &path, &mut ws).unwrap();
        let _ = t1.powertrain().traction_energy(30.0, 2000.0, Some(760.0));
        let _ = t1.powertrain().traction_energy(45.0, 3500.0, None);
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "coupled solve / traction_energy allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );

    // --- The energy-manager kernels (M6 PR2): the deploy-slice coupled solve, the per-station
    // T0 ERS queries, and the manager decide loop. The march composes exactly these primitives
    // (+ the alloc-gated `Pack::step_power` above), so a zero count here covers its hot loop.
    let deploy = vec![1200.0; path.len()];
    solve_into_ggv_coupled(
        &veh,
        &env,
        Some(&scale),
        Some(&deploy),
        None,
        None,
        None,
        &path,
        &mut ws,
    )
    .unwrap();
    let before = dhat::HeapStats::get();
    for _ in 0..16 {
        solve_into_ggv_coupled(
            &veh,
            &env,
            Some(&scale),
            Some(&deploy),
            None,
            None,
            None,
            &path,
            &mut ws,
        )
        .unwrap();
        let _ = veh.mech_tractive_force(41.0);
        let _ = veh.ers_deploy_force_n(41.0, 250e3);
        let _ = veh.ers_realized_deploy_w(250e3);
    }
    let after = dhat::HeapStats::get();
    assert_eq!(
        after.total_blocks,
        before.total_blocks,
        "deploy-slice coupled solve / ERS force queries allocated {} block(s)",
        after.total_blocks - before.total_blocks
    );
}

/// A constant-parameter Thevenin pack (cold assembly; allocations allowed here). The runtime
/// `step_power`/`step_current`/`terminal_voltage_v` must not allocate.
fn setup_pack() -> (Pack, PackState) {
    use outlap_schema::battery::{
        BatteryDoc, BatteryMeta, BatteryModelKind, BatteryTables, Ecm, EcmAxes, PackCapacity,
        PackLimits, PackThermal, PackTopology, PowerVsSoc, TableLevel,
    };
    let axis = |name: &str, v: f64| (name.to_owned(), vec![v, v, v, v]);
    let cols = vec![
        ("soc".to_owned(), vec![0.0, 0.0, 1.0, 1.0]),
        ("temp_c".to_owned(), vec![0.0, 50.0, 0.0, 50.0]),
        axis("ocv_v", 3.6),
        axis("r0_ohm", 0.02),
        axis("r1_ohm", 0.01),
        axis("tau1_s", 15.0),
        axis("dudt_v_per_k", -1.0e-4),
    ];
    let table = GriddedTable::from_long(&cols, &["soc", "temp_c"]).unwrap();
    let doc = BatteryDoc {
        schema: SchemaVersion::new("battery", 1, 0),
        model: BatteryModelKind::RcPairs,
        topology: PackTopology { ns: 200, np: 1 },
        capacity: PackCapacity {
            q_pack_ah: 90.0,
            e_pack_wh: 60000.0,
        },
        soc_window: [0.05, 0.98],
        ecm: Ecm {
            rc_pairs: 1,
            axes: EcmAxes {
                soc: vec![0.0, 1.0],
                temp_c: vec![0.0, 50.0],
            },
            tables: BatteryTables {
                file: "x.parquet".into(),
                level: TableLevel::Cell,
            },
        },
        limits: PackLimits {
            peak_discharge_power_w_vs_soc: PowerVsSoc {
                soc: vec![0.0, 1.0],
                power_w: vec![300_000.0, 300_000.0],
            },
            peak_regen_power_w_vs_soc: PowerVsSoc {
                soc: vec![0.0, 1.0],
                power_w: vec![300_000.0, 300_000.0],
            },
            regen_derate_vs_temp: None,
            cell_v_min: 2.5,
            cell_v_max: 4.2,
            max_c_rate: 3.0,
        },
        thermal: PackThermal {
            mass_kg: 400.0,
            cp_j_per_kgk: 900.0,
            thermal_resistance_k_per_w: 0.02,
            coolant_temp_c: 25.0,
        },
        meta: BatteryMeta::default(),
    };
    Pack::assemble(&doc, &table, None).unwrap()
}

/// A winding + housing + coolant + ambient lumped network (cold assembly; allocations allowed here).
fn setup_thermal() -> MachineThermal {
    use outlap_schema::emotor::{
        Conductance, CoolantSpec, Cooling, Emotor, EmotorMeta, NodeRole, ThermalNode,
    };
    use outlap_schema::version::SchemaVersion;

    let node = |name: &str, role: NodeRole, c: Option<f64>, lim: Option<(f64, f64)>| ThermalNode {
        name: name.into(),
        role: Some(role),
        c_j_per_k: c,
        t_warn_c: lim.map(|l| l.0),
        t_max_c: lim.map(|l| l.1),
    };
    let em = Emotor {
        schema: SchemaVersion::new("emotor", 1, 1),
        nodes: vec![
            node(
                "winding",
                NodeRole::Winding,
                Some(9000.0),
                Some((160.0, 180.0)),
            ),
            node("housing", NodeRole::Housing, Some(32000.0), None),
            node("coolant", NodeRole::Coolant, None, None),
            node("ambient", NodeRole::Ambient, None, None),
        ],
        conductances: vec![
            Conductance {
                between: ("winding".into(), "housing".into()),
                w_per_k: Some(12.0),
            },
            Conductance {
                between: ("housing".into(), "coolant".into()),
                w_per_k: Some(45.0),
            },
        ],
        convection: vec![],
        loss_routing: vec![],
        cooling: Cooling {
            ambient_node: "ambient".into(),
            ambient_fixed_c: None,
            jacket: None,
            air_gap: None,
            coolant: Some(CoolantSpec {
                node: "coolant".into(),
                inlet_c: 65.0,
                rho_cp_mdot_w_per_k: 900.0,
            }),
        },
        cu_feedback: None,
        initial_temp: None,
        meta: EmotorMeta::default(),
    };
    MachineThermal::assemble(&em, &Conditions::default(), 45.0).unwrap()
}
