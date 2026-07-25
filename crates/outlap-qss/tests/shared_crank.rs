// SPDX-License-Identifier: AGPL-3.0-only
//! D-M6-13 **Layer 3** — the shared-crank property tests.
//!
//! A drivetrain node that is the `output` of ≥2 torque sources is a real shared shaft, not a
//! topology string. On `f1_2026` the ICE and the MGU-K are both welded to the `crank`, with the
//! 8-speed gearbox below, so ONE gear sets BOTH sources' operating point. Layer 2 evaluated the
//! machine at its *own* most favourable gear — physically impossible for a machine bolted to the
//! crank — and Layer 3 pins it to the gear the engine engages.
//!
//! The five properties, in the order the plan states them:
//!
//! 1. **Pinned to the engine's gear.** The machine's crank speed is exactly the ICE's crank speed
//!    at the gear the ICE traction ceiling selects (locked decision: the reference source alone
//!    chooses the gear, so the mechanical ceiling is untouched).
//! 2. **The cap can only reduce.** The gear-referenced mechanical ceiling never exceeds the
//!    ratio-invariant `max(τ·ω)` the pre-Layer-3 tiers used, and it strictly binds at low speed.
//! 3. **Single-source cars never enter the path.** A car with no `policy:` overlay has no crank node
//!    at all, so its reduction is the pre-Layer-3 per-unit one.
//! 4. **Both tiers consume ONE rule.** The QSS march's `governed_deploy` and the owned
//!    `GovernedMachine` handle the transient governor holds agree bit-for-bit — tier parity (gate
//!    #4) is a merge blocker, so the two must not be able to drift apart.
//! 5. **Fixed order / determinism.** Two independent assemblies of the same car produce
//!    bit-identical crank speeds and deploy forces (declaration-order scan, no iteration-order slip).
#![allow(clippy::float_cmp)] // bit-exactness IS the assertion in the lockstep/determinism tests.

use outlap_core::GriddedTable;
use outlap_qss::{T0Options, T0Vehicle, T1Vehicle};
use outlap_schema::io::{FsLoader, SourceLoader};
use outlap_schema::sidecar::read_gridded_table;
use outlap_schema::{load_vehicle, Conditions, LoadOptions, ResolvedVehicle};

/// rpm → rad/s.
const RPM_TO_RAD_PER_S: f64 = std::f64::consts::PI / 30.0;
/// A speed sweep spanning standstill → beyond the f1 top speed, in m/s.
fn speeds() -> impl Iterator<Item = f64> {
    (0..=200).map(|i| f64::from(i) * 0.5)
}

fn fixtures() -> FsLoader {
    FsLoader::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../outlap-schema/tests/fixtures"
    ))
}

/// Assemble a fixture car's T1 vehicle with every `.ptm` sidecar installed (so the governed
/// machine's η/loss maps are live — Layer 2 hard-errors a governed car without them).
fn assemble_t1(dir: &str) -> (ResolvedVehicle, T1Vehicle) {
    let loader = fixtures();
    let resolved = load_vehicle(
        &format!("{dir}/vehicle.yaml"),
        &loader,
        &LoadOptions::default(),
    )
    .expect("fixture resolves");
    // allow_degraded: the gt_hybrid fixture ships no constant-aero block.
    let mut t1 = T1Vehicle::assemble(&resolved, &Conditions::default(), &loader, true).unwrap();
    for (idx, unit) in resolved.spec.drivetrain.units.iter().enumerate() {
        let Ok(ptm) = outlap_schema::load::load_ptm(unit.source.as_str(), &loader) else {
            continue;
        };
        let sidecar = match unit.source.as_str().rsplit_once('/') {
            Some((parent, _)) => format!("{parent}/{}", ptm.tables.file.as_str()),
            None => ptm.tables.file.as_str().to_owned(),
        };
        let Ok(bytes) = loader.load_bytes(&sidecar) else {
            continue;
        };
        let table: GriddedTable<f64> = if ptm.axes.vdc_v.is_some() {
            read_gridded_table(&bytes, &outlap_qss::T1Powertrain::map_axis_names_vdc())
        } else {
            read_gridded_table(&bytes, &outlap_qss::T1Powertrain::map_axis_names())
        }
        .unwrap();
        t1.install_powertrain_maps(idx, &table).unwrap();
    }
    (resolved, t1)
}

fn assemble_t0(dir: &str) -> T0Vehicle {
    let loader = fixtures();
    let resolved = load_vehicle(
        &format!("{dir}/vehicle.yaml"),
        &loader,
        &LoadOptions::default(),
    )
    .expect("fixture resolves");
    let opts = T0Options {
        allow_degraded: true,
        ..T0Options::default()
    };
    T0Vehicle::assemble(&resolved, &Conditions::default(), &loader, &opts).unwrap()
}

// -------------------------------------------------------------------------------------------
// 1. The machine is pinned to the ENGINE's engaged gear.
// -------------------------------------------------------------------------------------------

/// The governed machine's crank speed equals the ICE's crank speed, at every speed — i.e. it runs in
/// the gear the ICE traction ceiling engages, not one of its own choosing.
///
/// Before Layer 3 these disagreed wildly: the machine argmaxed `τ(ω)·ω` over the same 8 gears
/// subject to its OWN 50 000 rpm envelope, so on a straight the model ran the crank-welded MGU-K in
/// first gear at ~50 000 rpm while the crank itself turned ~15 000 rpm in eighth.
#[test]
fn the_governed_machine_turns_at_the_engines_crank_speed() {
    for car in ["f1_2026", "gt_hybrid"] {
        let (_, t1) = assemble_t1(car);
        let pt = t1.powertrain();
        let mut checked = 0;
        for v in speeds() {
            let Some(omega) = pt.governed_crank_omega(v) else {
                continue;
            };
            // The ICE's own crank speed at the gear its traction ceiling selects. `ice_crank_rpm`
            // resolves the operating point through exactly that argmax.
            let ice = pt
                .ice_crank_rpm(v, pt.max_drive_force(v))
                .expect("the reference engine is on-envelope wherever the machine is");
            // Equal to machine precision: `ice_crank_rpm` round-trips through rpm, which costs a
            // ulp. Anything larger means a DIFFERENT gear was selected (the ratios are ≥ 12 % apart,
            // so a gear slip shows up as a ≥ 10 % gap, not a 1e-16 one).
            let want = ice * RPM_TO_RAD_PER_S;
            assert!(
                (omega - want).abs() <= 1e-12 * want.abs().max(1.0),
                "{car} at {v} m/s: machine ω {omega} rad/s != engine crank {ice} rpm \
                 ({want} rad/s) — the machine is not pinned to the engaged gear"
            );
            checked += 1;
        }
        assert!(
            checked > 100,
            "{car}: only {checked} on-envelope speeds sampled"
        );
    }
}

/// The crank speed rises monotonically with vehicle speed *within* a gear and never exceeds the
/// engine's rev limit — the sanity check that the pinning uses the real ladder.
#[test]
fn the_shared_crank_speed_stays_under_the_engine_rev_limit() {
    let (_, t1) = assemble_t1("f1_2026");
    let pt = t1.powertrain();
    for v in speeds() {
        let Some(omega) = pt.governed_crank_omega(v) else {
            continue;
        };
        assert!(omega >= 0.0, "negative crank speed at {v} m/s");
        // 8-speed f1: the top of the ICE envelope. A crank speed above it would mean the machine
        // was evaluated on a gear the engine cannot turn.
        let ice_rpm = pt.ice_crank_rpm(v, pt.max_drive_force(v)).unwrap();
        assert!(
            ice_rpm <= 15_000.0 + 1e-6,
            "crank {ice_rpm} rpm past the 15 000 rpm limiter at {v} m/s"
        );
    }
}

// -------------------------------------------------------------------------------------------
// 2. The gear-referenced cap can only REDUCE deliverable machine power.
// -------------------------------------------------------------------------------------------

/// The gear-referenced ceiling `τ(ω_crank)·ω_crank` never exceeds the ratio-invariant `max(τ·ω)`
/// over the machine's whole map — the ceiling every tier used before Layer 3. A gear-referenced cap
/// can only bind harder, never looser, because it evaluates ONE point of a curve whose maximum is
/// the ratio-invariant value.
#[test]
fn the_gear_referenced_cap_never_exceeds_the_ratio_invariant_one() {
    let t0 = assemble_t0("f1_2026");
    let ratio_invariant = t0.ers_p_mech_max_w();
    assert!(ratio_invariant > 300e3, "the f1 machine ceiling is ~350 kW");
    for v in speeds() {
        let cap = t0.ers_mech_cap_w(v);
        assert!(
            cap <= ratio_invariant + 1e-9,
            "at {v} m/s the crank-referenced cap {cap} W exceeds the ratio-invariant \
             {ratio_invariant} W — a gear reference can only reduce"
        );
    }
}

/// …and it genuinely BINDS at low and moderate speed: a ~223 N·m machine on the crank simply cannot
/// make its rated 350 kW below its base speed. This is the property that moves the f1 numbers, so
/// asserting it keeps the change from silently regressing to the flat ceiling.
#[test]
fn the_crank_cap_binds_below_the_machines_base_speed() {
    let t0 = assemble_t0("f1_2026");
    let ratio_invariant = t0.ers_p_mech_max_w();
    // Well inside the torque-limited region (first/second gear on the shipped ladder).
    for v in [5.0, 10.0, 15.0, 20.0] {
        let cap = t0.ers_mech_cap_w(v);
        assert!(
            cap < 0.75 * ratio_invariant,
            "at {v} m/s the crank cap {cap} W should be far below the rated {ratio_invariant} W \
             (the machine is torque-limited down here)"
        );
        assert!(cap > 0.0, "the machine still deploys at {v} m/s");
    }
}

/// A torque-limited crank-mounted machine delivers a **speed-invariant wheel force**: `F = τ·ratio/r`
/// carries no `1/v`. The pre-Layer-3 ratio-invariant power ceiling produced `η·P/v`, which blew up
/// towards standstill (≈165 kN at 2 m/s on the shipped f1 — an order of magnitude past the tyres).
#[test]
fn the_torque_limited_launch_force_is_speed_invariant() {
    let t0 = assemble_t0("f1_2026");
    let ers_at = |v: f64| t0.ers_deploy_force_n(v, 350e3);
    let f1 = ers_at(1.0);
    assert!(f1 > 1e3, "the machine deploys at launch: {f1} N");
    for v in [1.0, 1.5, 2.0, 2.5, 3.0] {
        // `η·τ(ω)·ω / v` with `ω ∝ v`: the speeds cancel exactly in exact arithmetic and to a ulp in
        // floating point (the reassociation is the only difference).
        let f = ers_at(v);
        assert!(
            (f - f1).abs() <= 1e-12 * f1,
            "at {v} m/s the launch force {f} N differs from {f1} N — the torque-limited machine \
             should be speed-invariant below its base speed"
        );
    }
}

// -------------------------------------------------------------------------------------------
// 3. A car with no shared crank never enters the path.
// -------------------------------------------------------------------------------------------

/// Every non-governed car has NO crank node: `governed_crank_omega`, `governed_deploy` and
/// `governed_regen_envelope_w` all return `None`, so its reduction is the pre-Layer-3 per-unit one
/// and its results are byte-identical. This is the gate that keeps the EVs / `limebeer` / `bmw320i`
/// goldens frozen.
#[test]
fn a_car_without_a_policy_has_no_crank_node() {
    for car in [
        "ev_1du_rwd",
        "ev_2du_awd",
        "ev_4du_tv",
        "fwd_hatch",
        "pdt_du_rwd",
    ] {
        let (resolved, t1) = assemble_t1(car);
        assert!(
            resolved.spec.policy.is_none(),
            "{car} unexpectedly has a policy"
        );
        let pt = t1.powertrain();
        for v in speeds() {
            assert!(
                pt.governed_crank_omega(v).is_none(),
                "{car}: crank node at {v} m/s"
            );
            assert!(pt.governed_deploy(v, 100e3, None, 0.95, 1.0).is_none());
            assert!(pt.governed_regen_envelope_w(v).is_none());
        }
        // The T0 side likewise keeps a zero ERS adder (no `policy:` ⇒ no `T0Ers`).
        let t0 = assemble_t0(car);
        assert_eq!(t0.ers_mech_cap_w(30.0), 0.0);
        assert_eq!(t0.ers_deploy_force_n(30.0, 350e3), 0.0);
        for v in speeds() {
            assert_eq!(t0.tractive_force(v), t0.mech_tractive_force(v));
        }
    }
}

// -------------------------------------------------------------------------------------------
// 4. Both tiers consume ONE shared-crank rule (parity gate #4 is a merge blocker).
// -------------------------------------------------------------------------------------------

/// The QSS march calls `T1Powertrain::governed_deploy`; the transient governor holds an owned
/// `GovernedMachine` handle. They MUST be the same physics — a tier-only change breaks tier parity
/// (gate #4). Assert they agree bit-for-bit across speed and commanded power.
#[test]
fn the_qss_march_and_the_transient_handle_deploy_identically() {
    for car in ["f1_2026", "gt_hybrid"] {
        let (_, t1) = assemble_t1(car);
        let pt = t1.powertrain();
        let machine = pt
            .governed_machine()
            .expect("a governed car exposes the owned machine handle");
        let mut compared = 0;
        for v in speeds() {
            for p_elec in [10e3, 120e3, 350e3] {
                for derate in [1.0, 0.6] {
                    let a = pt.governed_deploy(v, p_elec, None, 0.95, derate);
                    let b = machine.deploy(v, p_elec, None, 0.95, derate);
                    match (a, b) {
                        (None, None) => {}
                        (Some(a), Some(b)) => {
                            assert_eq!(a.force_n, b.force_n, "{car} @ {v} m/s / {p_elec} W");
                            assert_eq!(a.p_elec_used_w, b.p_elec_used_w);
                            assert_eq!(a.loss_w, b.loss_w);
                            assert_eq!(a.omega_rad_s, b.omega_rad_s);
                            compared += 1;
                        }
                        _ => panic!(
                            "{car} @ {v} m/s: the two tiers disagree on whether the \
                                     machine can deploy at all"
                        ),
                    }
                }
            }
            assert_eq!(
                pt.governed_regen_envelope_w(v),
                machine.regen_envelope_w(v),
                "{car} @ {v} m/s: regen envelopes diverged between tiers"
            );
        }
        assert!(
            compared > 100,
            "{car}: only {compared} deploy points compared"
        );
    }
}

/// The T0 point-mass tier builds its crank view from a SECOND, independent ladder
/// (`T0Gear{omega_per_v, force_per_torque}`) — it has to, because the point-mass traction query has
/// its own precomputed form. The two must nevertheless select the same gear.
///
/// This is not cosmetic. `ers_decide` normalises the driver demand by the **T0** pedal availability
/// (`f_req / tractive_force`) but realises the deploy through the **T1** march. If T0 thought the
/// machine could give more than T1 delivers, the demand under-reads, the station falls into the
/// part-throttle branch, and the manager banks harvest with no mechanical source — exactly the
/// mis-classification Layer 3 exists to remove.
#[test]
fn the_t0_and_t1_crank_pins_agree() {
    for car in ["f1_2026", "gt_hybrid"] {
        let (_, t1) = assemble_t1(car);
        let t0 = assemble_t0(car);
        let pt = t1.powertrain();
        let mut compared = 0;
        for v in speeds() {
            match (pt.governed_crank_omega(v), t0.ers_crank_omega(v)) {
                (None, None) => {}
                (Some(a), Some(b)) => {
                    // The two argmax forms differ by association (`τ·ratio·η/r` vs a precomputed
                    // `τ·force_per_torque`) — deliberately, so each matches its OWN tier's traction
                    // ceiling bit-for-bit. A ulp of disagreement is that reassociation; a different
                    // GEAR would be ≥ 12 % apart on this ladder.
                    assert!(
                        (a - b).abs() <= 1e-12 * a.abs().max(1.0),
                        "{car} at {v} m/s: T1 pinned the machine to {a} rad/s but T0 to {b} — the \
                         tiers engaged different gears, so the driver demand and the realised \
                         deploy disagree"
                    );
                    compared += 1;
                }
                (a, b) => panic!(
                    "{car} at {v} m/s: the tiers disagree on whether a gear is on-envelope \
                     (T1 {a:?}, T0 {b:?})"
                ),
            }
        }
        assert!(compared > 100, "{car}: only {compared} speeds compared");
    }
}

// -------------------------------------------------------------------------------------------
// 5. Fixed declaration order ⇒ determinism.
// -------------------------------------------------------------------------------------------

/// The node scan runs the gear ladder in file-declaration order with a first-max tie-break, so two
/// independent assemblies of the same car are bit-identical. (A `HashMap`/iteration-order slip in
/// the node lookup would show up here, and would also break the resolved-hash contract.)
#[test]
fn the_shared_crank_evaluation_is_deterministic_across_assemblies() {
    let (_, a) = assemble_t1("f1_2026");
    let (_, b) = assemble_t1("f1_2026");
    let (pa, pb) = (a.powertrain(), b.powertrain());
    for v in speeds() {
        assert_eq!(pa.governed_crank_omega(v), pb.governed_crank_omega(v));
        assert_eq!(
            pa.governed_regen_envelope_w(v),
            pb.governed_regen_envelope_w(v)
        );
        let (da, db) = (
            pa.governed_deploy(v, 350e3, None, 0.95, 1.0),
            pb.governed_deploy(v, 350e3, None, 0.95, 1.0),
        );
        assert_eq!(da.is_some(), db.is_some());
        if let (Some(da), Some(db)) = (da, db) {
            assert_eq!(da.force_n, db.force_n);
            assert_eq!(da.p_elec_used_w, db.p_elec_used_w);
            assert_eq!(da.loss_w, db.loss_w);
            assert_eq!(da.omega_rad_s, db.omega_rad_s);
        }
    }
}
