// SPDX-License-Identifier: AGPL-3.0-only
//! Assembly stage: reduce a [`ResolvedVehicle`] + `conditions` into a compact [`T0Vehicle`].
//!
//! This is a cold path (allocations allowed): it re-loads the referenced `.ptm`/`.tyr` files,
//! derives the constant friction coefficients from the tyre MF6.1 pure-slip curve peaks (via the
//! validated [`peak_mu_x`]/[`peak_mu_y`] extractors at `Fz = FNOMIN`, cold inflation pressure,
//! `γ = 0`), lumps the aero coefficients with air density, and folds each drive unit's coupler
//! path into a set of gears so the hot [`T0Vehicle::tractive_force`] query is allocation-free.

use outlap_core::MonotoneCubic;
use outlap_powertrain::ErsRulebook;
use outlap_schema::conditions::Conditions;
use outlap_schema::io::SourceLoader;
use outlap_schema::load::{load_ptm, load_tyr};
use outlap_schema::ptm::TorqueCurve;
use outlap_schema::refs::NodeId;
use outlap_schema::tyr::Tyr;
use outlap_schema::vehicle::{Coupler, Efficiency, Gearbox, Wheel};
use outlap_schema::ResolvedVehicle;
use outlap_tire::TireModel;

use crate::error::T0Error;
use crate::DEFAULT_DS_M;

/// Revolutions per minute → radians per second.
const RPM_TO_RAD_PER_S: f64 = std::f64::consts::PI / 30.0;
/// Specific gas constant for dry air, J/(kg·K).
const DRY_AIR_R: f64 = 287.05;
/// Speed floor for the ERS power→force conversion (the friction ellipse caps launch force anyway).
pub(crate) const ERS_V_FLOOR_MPS: f64 = 1.0;

/// Options controlling T0 assembly and solving.
#[derive(Clone, Debug)]
pub struct T0Options {
    /// Arc-length step for the passes, metres (default [`DEFAULT_DS_M`]).
    pub ds_m: f64,
    /// Upper speed bound for the curvature-limited speed, m/s (safety cap; default 150).
    pub v_cap: f64,
    /// Max forward/backward pass iterations before declaring divergence on a closed lap (default 4).
    pub max_pass_iterations: usize,
    /// Allow documented-degraded combinations (e.g. missing constant aero → zero aero), recorded in
    /// the notes (Decision #40).
    pub allow_degraded: bool,
}

impl Default for T0Options {
    fn default() -> Self {
        Self {
            ds_m: DEFAULT_DS_M,
            v_cap: 150.0,
            max_pass_iterations: 4,
            allow_degraded: false,
        }
    }
}

/// A point-mass reduction of a vehicle for the T0 tier.
#[derive(Clone, Debug)]
pub struct T0Vehicle {
    /// Total mass, kg.
    pub mass_kg: f64,
    /// Longitudinal friction coefficient (MF6.1 pure-slip `Fx` peak @ FNOMIN/p_cold, mean of axles).
    pub mu_x: f64,
    /// Lateral friction coefficient (MF6.1 pure-slip `Fy` peak @ FNOMIN/p_cold, mean of axles).
    pub mu_y: f64,
    /// Lumped drag term `½·ρ·CxA` (N per (m/s)²).
    pub qx: f64,
    /// Lumped downforce term `½·ρ·CzA` (N per (m/s)²).
    pub qz: f64,
    /// Curvature-limited-speed cap, m/s.
    pub v_cap: f64,
    units: Vec<T0Unit>,
    ers: Option<T0Ers>,
    notes: Vec<String>,
}

/// One drive unit reduced to a peak-torque envelope and a set of gears.
#[derive(Clone, Debug)]
struct T0Unit {
    /// Peak torque vs shaft speed [rad/s → N·m].
    torque_env: MonotoneCubic<f64>,
    /// Highest shaft speed the envelope covers, rad/s (gears past this are rev-limited out).
    omega_max: f64,
    gears: Vec<T0Gear>,
}

/// A single selectable gear, precomputed for the point-mass force query.
#[derive(Clone, Copy, Debug)]
struct T0Gear {
    /// Shaft speed per unit vehicle speed, (rad/s)/(m/s) = total ratio / wheel radius.
    omega_per_v: f64,
    /// Wheel force per unit shaft torque, N/(N·m) = total ratio · efficiency / wheel radius.
    force_per_torque: f64,
}

/// An ERS reduced to a rulebook-governed tractive-force contribution.
///
/// The regulatory side (electrical cap × piecewise-LINEAR taper, the C5.2.14 0.97
/// electrical→mechanical seam) lives in the shared [`ErsRulebook`] — the same implementation the
/// energy manager enforces, so the greedy uncoupled path and the budget-enforced march can never
/// disagree on the curve. The machine ceiling `p_mech_max_w` and the driveline `eta` are the
/// T0-side mechanical composition (both factors distinct from the rulebook's 0.97 — see
/// [`T0Vehicle::ers_deploy_force_n`]).
#[derive(Clone, Debug)]
struct T0Ers {
    /// The shared FIA-style rulebook (electrical caps, piecewise-linear tapers, 0.97 seam).
    rulebook: ErsRulebook<f64>,
    /// Machine mechanical power ceiling `max(τ·ω)` over the map, W (ratio-invariant) — the
    /// speed-independent upper bound. It remains the reported machine ceiling and the regen proxy;
    /// the **binding** deploy ceiling is now the gear-referenced [`Self::crank`] envelope below.
    p_mech_max_w: f64,
    /// The shared-crank torque limit (D-M6-13 Layer 3): the governed machine's own `τ(ω)` envelope
    /// evaluated at the **engaged** gear's crank speed. Layer 2 and earlier capped the T0 deploy by
    /// the ratio-invariant `p_mech_max_w` alone, which let a torque-limited machine deliver its rated
    /// power at every speed; pinning it to the crank makes the pedal-availability curve agree with
    /// what the march actually realizes (so `driver_demand` stops mis-classifying deploy as
    /// part-throttle harvest). `None` for a governed machine with no reachable crank node.
    crank: Option<T0Crank>,
    /// Driveline (crank→wheel) efficiency applied to the ERS force.
    eta: f64,
}

/// The T0 view of a shared drivetrain node (D-M6-13 Layer 3) — the mirror of the T1 `CrankNode`,
/// built from the point-mass gear ladders so both tiers pin the machine to the same engaged gear.
///
/// The engaged gear is chosen by the **reference source alone** (the first-declared non-governed
/// unit on the node — the ICE), exactly as [`T0Vehicle::mech_tractive_force`] already picks it, so
/// the mechanical traction curve is untouched.
#[derive(Clone, Debug)]
struct T0Crank {
    /// The reference source's gear ladder, in file-declaration order.
    ref_gears: Vec<T0Gear>,
    /// The reference source's peak-torque envelope `τ(ω)`.
    ref_torque_env: MonotoneCubic<f64>,
    /// Highest shaft speed the reference envelope covers, rad/s.
    ref_omega_max: f64,
    /// The governed machine's own peak-torque envelope `τ(ω)`, N·m on its shaft.
    machine_env: MonotoneCubic<f64>,
    /// Highest shaft speed the machine's envelope covers, rad/s.
    machine_omega_max: f64,
    /// Governed-machine shaft speed per unit reference-source shaft speed (`1.0` when both sit
    /// directly on the node — the `f1_2026` case).
    machine_per_ref: f64,
}

impl T0Crank {
    /// The governed machine's shaft speed (rad/s) in the engaged gear at vehicle speed `v` — the
    /// gear maximising the reference source's wheel force. `None` when no gear is on-envelope.
    /// Zero-allocation, fixed-order.
    fn engaged_omega(&self, v: f64) -> Option<f64> {
        let mut best: Option<(f64, f64)> = None; // (reference wheel force, reference shaft speed)
        for g in &self.ref_gears {
            let omega = g.omega_per_v * v;
            if omega > self.ref_omega_max {
                continue;
            }
            let avail = self.ref_torque_env.eval(omega) * g.force_per_torque;
            if best.is_none_or(|(b, _)| avail > b) {
                best = Some((avail, omega));
            }
        }
        best.map(|(_, omega)| omega * self.machine_per_ref)
    }

    /// The machine's mechanical power ceiling at the engaged gear's crank speed, W — `τ(ω)·ω`. `0`
    /// when no gear is on-envelope, or when the crank turns faster than the machine's own envelope.
    fn engaged_mech_cap_w(&self, v: f64) -> f64 {
        let Some(omega) = self.engaged_omega(v) else {
            return 0.0;
        };
        if !omega.is_finite() || omega > self.machine_omega_max {
            return 0.0;
        }
        (self.machine_env.eval(omega) * omega).max(0.0)
    }
}

impl T0Vehicle {
    /// Assemble a [`T0Vehicle`] from a resolved vehicle, session conditions, and a source loader.
    // One linear assembly procedure (tyres → aero → mechanical units → ERS force-adder); the
    // D-M6-13 graph flatten + governed-unit de-dup pushed it a few lines past the pedantic cap.
    #[allow(clippy::too_many_lines)]
    pub fn assemble(
        vehicle: &ResolvedVehicle,
        conditions: &Conditions,
        loader: &dyn SourceLoader,
        opts: &T0Options,
    ) -> Result<Self, T0Error> {
        let spec = &vehicle.spec;
        let mut notes = Vec::new();

        // --- Tyres: friction coefficients (mean of axles) + driven-axle radii ---
        // μ comes from the validated tyre force model at the nominal load and cold inflation
        // pressure (γ = 0), not the raw PD*·LMU* factors: for MF6.1 the pure-slip curve peaks fold
        // in the load/pressure shape factors (the extractors fix γ = 0 and V_cx = LONGVL); for a
        // brush-only tyre the peak is the base friction μ0. Partial force sets never reach here —
        // schema validation rejects an incomplete MF6.1 core without a `brush` block.
        let (front, _) = load_tyr(spec.tires.front.as_str(), loader)?;
        let (rear, _) = load_tyr(spec.tires.rear.as_str(), loader)?;
        let (tm_front, front_notes) = TireModel::<f64>::from_tyr(&front)?;
        let (tm_rear, rear_notes) = TireModel::<f64>::from_tyr(&rear)?;
        let mu_x = 0.5 * (axle_mu_x(&tm_front, &front) + axle_mu_x(&tm_rear, &rear));
        let mu_y = 0.5 * (axle_mu_y(&tm_front, &front) + axle_mu_y(&tm_rear, &rear));
        // Surface any tyre-model notes (e.g. a brush tyre's Mx=My=0 / ignored camber) once.
        for n in front_notes.iter().chain(&rear_notes) {
            if !notes.contains(&n.detail) {
                notes.push(n.detail.clone());
            }
        }
        let r_front = coeff(&front, "UNLOADED_RADIUS", 0.33);
        let r_rear = coeff(&rear, "UNLOADED_RADIUS", 0.33);

        // --- Aero + air density (ideal gas) ---
        let rho = 100.0 * conditions.air.pressure_hpa
            / (DRY_AIR_R * (conditions.air.temperature_c + 273.15));
        let (qx, qz) = match &spec.aero.constant {
            Some(c) => (
                0.5 * rho * c.cx_a_m2,
                0.5 * rho * (c.cz_front_a_m2 + c.cz_rear_a_m2),
            ),
            None if opts.allow_degraded => {
                notes.push(
                    "aero.constant absent — T0 ran with zero aero (ride-height map arrives with T1)"
                        .to_owned(),
                );
                (0.0, 0.0)
            }
            None => return Err(T0Error::NoConstantAero),
        };

        // --- Drive units (mechanical sources only; a policy-governed machine is excluded here and
        // force-added by the ERS block below — the D-M6-13 T0 de-dup) ---
        let governed = crate::graph::governed_unit_ids(spec);
        // The regulatory engine rev limit (see the T1 twin in `t1/powertrain.rs::assemble`).
        let rev_limit = spec
            .policy
            .as_ref()
            .and_then(|p| p.max_engine_speed_rpm)
            .map(|rpm| rpm * RPM_TO_RAD_PER_S);
        let mut units = Vec::with_capacity(spec.drivetrain.units.len());
        // The drivetrain node each mechanical unit outputs onto, parallel to `units` — the Layer-3
        // shared-crank lookup reads it to find the governed machine's reference source.
        let mut unit_nodes: Vec<Option<&str>> = Vec::with_capacity(spec.drivetrain.units.len());
        for (i, unit) in spec.drivetrain.units.iter().enumerate() {
            if governed.contains(unit.id.as_str()) {
                continue;
            }
            let ptm = load_ptm(unit.source.as_str(), loader)?;
            // Flatten the shared graph: private path ++ couplers from the source's output node.
            let (chain, wheels) = crate::graph::flatten_chain(&spec.drivetrain, unit);
            let r_wheel = driven_radius(&wheels, r_front, r_rear, i, &mut notes);
            let (base_ratio, base_eff, gearbox) = fold_path(&chain, i, &mut notes)?;
            let torque_env = torque_env(&ptm.limits.max_torque_nm_vs_speed)?;
            let mut omega_max = torque_env.domain().1;
            if let Some(cap) = rev_limit {
                if ptm.kind == outlap_schema::ptm::PtmKind::Combustion && cap < omega_max {
                    omega_max = cap; // the T1 twin records the note; T0 clamps identically
                }
            }
            let gears = build_gears(base_ratio, base_eff, gearbox, r_wheel);
            units.push(T0Unit {
                torque_env,
                omega_max,
                gears,
            });
            unit_nodes.push(unit.output.as_ref().map(NodeId::as_str));
        }

        // --- ERS (rulebook-governed force from the policy-governed machine unit) ---
        // The pack SoC window is unused on the T0 pedal-availability path (only the deploy taper
        // and the 0.97 seam are read; `recharge_target` is a march-only concern), so the rulebook
        // is built with a placeholder window — the march (ErsCoupling) builds its own rulebook from
        // the real governed-pack window.
        let ers = match spec.policy.as_ref().and_then(|policy| {
            policy
                .governs
                .first()
                .and_then(|id| spec.drivetrain.units.iter().find(|u| u.id == *id))
                .map(|unit| (policy, unit))
        }) {
            Some((policy, gov_unit)) => {
                let ptm = load_ptm(gov_unit.source.as_str(), loader)?;
                let curve = &ptm.limits.max_torque_nm_vs_speed;
                let p_mech_max_w = curve
                    .speed_rpm
                    .iter()
                    .zip(&curve.torque_nm)
                    .map(|(rpm, t)| (rpm * RPM_TO_RAD_PER_S) * t.abs())
                    .fold(0.0_f64, f64::max);
                // The rulebook carries the electrical caps + piecewise-linear tapers + the 0.97
                // conversion seam; the gear-referenced crank torque limit is the `T0Crank` below.
                let rulebook = ErsRulebook::from_schema(policy, [0.0, 1.0], None)?;
                let eta = single_gearbox_eff(spec).unwrap_or(1.0);
                // Layer 3 (D-M6-13): the machine's own `τ(ω)` envelope on its gear ladder, pinned to
                // the gear the reference source on its output node engages.
                let crank = build_t0_crank(
                    spec,
                    gov_unit,
                    &ptm.limits.max_torque_nm_vs_speed,
                    &units,
                    &unit_nodes,
                    r_front,
                    r_rear,
                    &mut notes,
                )?;
                notes.push(
                    "ERS folded into the T0 pedal-availability force as the greedy, budget-free \
                     regulation-curve adder (piecewise-linear taper × the 0.97 crank factor); the \
                     2026 energy manager governs actual budget-limited deployment where it is \
                     wired (the QSS slow-state march; the T2 tier in M6 PR4). Where no manager \
                     governs, this greedy adder is the deployment"
                        .to_owned(),
                );
                if crank.is_some() {
                    notes.push(
                        "the ERS adder is capped by the governed machine's own torque envelope at \
                         the ENGAGED gear's crank speed (D-M6-13 Layer 3), not by its \
                         ratio-invariant peak power: a torque-limited machine can no longer deliver \
                         its rated deploy at every speed, and the T0 pedal availability now agrees \
                         with what the energy-manager march realizes"
                            .to_owned(),
                    );
                }
                Some(T0Ers {
                    rulebook,
                    p_mech_max_w,
                    crank,
                    eta,
                })
            }
            None => None,
        };

        if units.is_empty() && ers.is_none() {
            return Err(T0Error::NoDrive);
        }

        notes.push(
            "μ derived from MF6.1 pure-slip peak @ FNOMIN, p_cold (mean of front/rear); braking is \
             friction-limited only at T0"
                .to_owned(),
        );

        // Full-tank reference mass m₀ = dry + initial fuel when a `fuel:` block is present (D-M6-4b);
        // the point-mass F/m then starts at m₀ and the fuel slow state marches it down. No fuel ⇒
        // the raw chassis mass (byte-identical to pre-M6).
        let mass_kg = crate::fuel::FuelModel::from_spec(spec)
            .map_or(spec.chassis.mass_kg, |fm| fm.full_mass_kg());

        Ok(Self {
            mass_kg,
            mu_x,
            mu_y,
            qx,
            qz,
            v_cap: opts.v_cap,
            units,
            ers,
            notes,
        })
    }

    /// Total tractive force available at vehicle speed `v` (m/s), N — the sum over drive units of
    /// the best gear's wheel force plus the GREEDY (budget-free) ERS contribution on the
    /// piecewise-linear regulation taper. This is the *pedal availability* — what the car can put
    /// down when nothing but the deploy curve limits the ERS. The budget-enforced coupled solve
    /// splits the two shares instead ([`Self::mech_tractive_force`] + a per-station deploy-force
    /// slice from the energy-manager march). Allocation-free.
    pub fn tractive_force(&self, v: f64) -> f64 {
        let mut force = self.mech_tractive_force(v);
        if let Some(e) = &self.ers {
            let p_elec = e.rulebook.deploy_cap_electrical_w(v, false);
            force += self.ers_deploy_force_n(v, p_elec);
        }
        force
    }

    /// Mechanical tractive force from the drive units alone at speed `v` (m/s), N — best gear per
    /// unit, no ERS share. Allocation-free.
    pub fn mech_tractive_force(&self, v: f64) -> f64 {
        let mut force = 0.0;
        for unit in &self.units {
            let mut best = 0.0;
            for g in &unit.gears {
                let omega = g.omega_per_v * v;
                if omega <= unit.omega_max {
                    let f = unit.torque_env.eval(omega) * g.force_per_torque;
                    if f > best {
                        best = f;
                    }
                }
            }
            force += best;
        }
        force
    }

    /// The wheel-force contribution of an ELECTRICAL ERS deploy power `p_elec_w` (W at the CU-K DC
    /// bus) at speed `v`, N: `p_elec × 0.97 (C5.2.14, the rulebook seam) → min(machine mechanical
    /// ceiling at the engaged gear) → × η_driveline / v`. Both conversion factors stay distinct:
    /// 0.97 is the regulation's electrical→mechanical crank factor, `eta` the crank→wheel driveline
    /// loss. Returns 0 for a car without an `ers:` block. Allocation-free.
    pub fn ers_deploy_force_n(&self, v: f64, p_elec_w: f64) -> f64 {
        match &self.ers {
            Some(e) => {
                let p_mech = e
                    .rulebook
                    .mech_deploy_w(p_elec_w)
                    .min(self.ers_mech_cap_w(v))
                    .max(0.0);
                e.eta * p_mech / v.max(ERS_V_FLOOR_MPS)
            }
            None => 0.0,
        }
    }

    /// The mechanical crank power the ERS can realize at speed `v` for an electrical deploy
    /// `p_elec_w`, W — `min(0.97·p_elec, machine ceiling at the engaged gear)` — plus the electrical
    /// power actually drawn once the machine ceiling binds (`p_mech / 0.97`: the pack never pays for
    /// power the machine cannot convert). Returns `(p_mech_w, p_elec_realized_w)`; `(0, 0)` without
    /// an `ers:` block.
    pub fn ers_realized_deploy_w(&self, v: f64, p_elec_w: f64) -> (f64, f64) {
        match &self.ers {
            Some(e) => {
                let cap = self.ers_mech_cap_w(v);
                let p_mech = e.rulebook.mech_deploy_w(p_elec_w).min(cap).max(0.0);
                let p_elec = if e.rulebook.mech_deploy_w(p_elec_w) > cap {
                    e.rulebook.mech_harvest_w(p_mech) // p_mech / 0.97 — machine-bound draw
                } else {
                    p_elec_w.max(0.0)
                };
                (p_mech, p_elec)
            }
            None => (0.0, 0.0),
        }
    }

    /// The **binding** ERS machine mechanical ceiling at speed `v`, W: the machine's own `τ(ω)`
    /// envelope at the engaged gear's crank speed (D-M6-13 Layer 3). Falls back to the
    /// ratio-invariant `max(τ·ω)` over its map only when no crank view could be built at all. `0`
    /// without a `policy:` overlay. Allocation-free.
    pub fn ers_mech_cap_w(&self, v: f64) -> f64 {
        self.ers.as_ref().map_or(0.0, |e| match &e.crank {
            Some(c) => c.engaged_mech_cap_w(v),
            None => e.p_mech_max_w,
        })
    }

    /// The governed machine's shaft speed at speed `v`, rad/s — the engaged gear's crank speed
    /// (D-M6-13 Layer 3). `None` without a `policy:` overlay, or when no gear is on-envelope.
    ///
    /// The T0 point-mass tier builds its crank view from its OWN gear ladder
    /// (`T0Gear{omega_per_v, force_per_torque}`), independently of the T1 `CrankNode`. The two must
    /// select the same gear: `ers_decide` normalises the driver demand by the T0 pedal availability
    /// but realises the deploy through the T1 march, so a disagreement would mis-classify deploy as
    /// part-throttle harvest. Exposed so the property test can assert they agree.
    pub fn ers_crank_omega(&self, v: f64) -> Option<f64> {
        self.ers.as_ref()?.crank.as_ref()?.engaged_omega(v)
    }

    /// The ERS machine's ratio-invariant mechanical power ceiling `max(τ·ω)` over its `.ptm` map,
    /// W (0 without an `ers:` block). The regen envelope proxy for the harvest chain — the `.ptm`
    /// schema treats an absent regen curve as a symmetric machine.
    pub fn ers_p_mech_max_w(&self) -> f64 {
        self.ers.as_ref().map_or(0.0, |e| e.p_mech_max_w)
    }

    /// The ERS driveline (crank→wheel) efficiency (1 without an `ers:` block).
    pub fn ers_eta(&self) -> f64 {
        self.ers.as_ref().map_or(1.0, |e| e.eta)
    }

    /// Human-readable notes on T0 simplifications and any degradations (nothing silent).
    pub fn notes(&self) -> &[String] {
        &self.notes
    }
}

/// Peak longitudinal μ from the tyre force model at the nominal load and cold pressure (MF6.1:
/// pure-slip curve peak; brush: the base friction μ0, which ignores load/pressure).
fn axle_mu_x(model: &TireModel<f64>, tyr: &Tyr) -> f64 {
    model.peak_mu_x(coeff(tyr, "FNOMIN", 0.0), cold_pressure_pa(tyr))
}

/// Peak lateral μ from the tyre force model at the nominal load and cold pressure.
fn axle_mu_y(model: &TireModel<f64>, tyr: &Tyr) -> f64 {
    model.peak_mu_y(coeff(tyr, "FNOMIN", 0.0), cold_pressure_pa(tyr))
}

/// The cold inflation pressure in Pa (schema stores `thermal.p_cold` in kPa; kPa→Pa at the seam).
fn cold_pressure_pa(tyr: &Tyr) -> f64 {
    1000.0 * tyr.thermal.p_cold
}

/// An MF6.1 coefficient, or `default` if absent.
fn coeff(tyr: &Tyr, key: &str, default: f64) -> f64 {
    tyr.mf61.0.get(key).copied().unwrap_or(default)
}

/// The driven-axle wheel radius for a unit's terminal wheels (mean if it spans both axles).
fn driven_radius(
    wheels: &[Wheel],
    r_front: f64,
    r_rear: f64,
    index: usize,
    notes: &mut Vec<String>,
) -> f64 {
    let front = wheels.iter().any(|w| w.is_front());
    let rear = wheels.iter().any(|w| !w.is_front());
    match (front, rear) {
        (true, false) => r_front,
        (false, _) => r_rear, // rear-only, or no wheels declared → rear radius
        (true, true) => {
            if (r_front - r_rear).abs() > 1e-3 {
                notes.push(format!(
                    "drive unit {index} spans both axles; using the mean tyre radius"
                ));
            }
            0.5 * (r_front + r_rear)
        }
    }
}

/// Fold a coupler path into `(base_ratio, base_efficiency, optional gearbox)`. The gearbox (if any)
/// supplies the selectable gear ratios; fixed ratios multiply into `base_ratio`; diffs are 1:1.
fn fold_path<'a>(
    path: &'a [Coupler],
    unit: usize,
    notes: &mut Vec<String>,
) -> Result<(f64, f64, Option<&'a Gearbox>), T0Error> {
    let mut base_ratio = 1.0;
    let mut base_eff = 1.0;
    let mut gearbox: Option<&Gearbox> = None;
    for coupler in path {
        match coupler {
            Coupler::FixedRatio(r) => base_ratio *= r,
            Coupler::Diff(_) => {} // 1:1, unity efficiency at the point-mass level
            Coupler::Gearbox(g) => {
                base_eff *= efficiency_constant(&g.efficiency, unit)?;
                if gearbox.is_none() {
                    gearbox = Some(g);
                } else {
                    // A second gearbox in one path is unusual; fold it as a fixed reduction.
                    base_ratio *= g.final_drive * g.ratios.first().copied().unwrap_or(1.0);
                    notes.push(format!(
                        "drive unit {unit} has multiple gearboxes; only the first provides gears"
                    ));
                }
            }
        }
    }
    Ok((base_ratio, base_eff, gearbox))
}

/// A constant efficiency, or an error if it is a (not-yet-readable) gridded map.
fn efficiency_constant(eff: &Efficiency, unit: usize) -> Result<f64, T0Error> {
    match eff {
        Efficiency::Constant(e) => Ok(*e),
        Efficiency::Map { .. } => Err(T0Error::UnsupportedEfficiencyMap { unit }),
    }
}

/// Expand a folded path into gears (one per gearbox ratio, or a single direct-drive gear).
fn build_gears(
    base_ratio: f64,
    base_eff: f64,
    gearbox: Option<&Gearbox>,
    r_wheel: f64,
) -> Vec<T0Gear> {
    match gearbox {
        Some(g) => g
            .ratios
            .iter()
            .map(|&rk| {
                let ratio = base_ratio * rk * g.final_drive;
                T0Gear {
                    omega_per_v: ratio / r_wheel,
                    force_per_torque: ratio * base_eff / r_wheel,
                }
            })
            .collect(),
        None => vec![T0Gear {
            omega_per_v: base_ratio / r_wheel,
            force_per_torque: base_ratio * base_eff / r_wheel,
        }],
    }
}

/// Build the T0 shared-crank view for the policy-governed machine (D-M6-13 Layer 3).
///
/// The machine's gear ladder comes from its own flattened chain (private path ++ the shared couplers
/// below its output node); the **reference** source — the first-declared mechanical unit on the same
/// node — supplies the gear selection. `None` when the machine declares no output node, when no
/// mechanical source shares it, or when the two ladders are not index-aligned (the machine sits on a
/// gearbox of its own): the T0 adder then keeps the ratio-invariant power ceiling, and the note
/// records it.
#[allow(clippy::too_many_arguments)] // one cold assembly helper: spec + unit + envelope + ladders
fn build_t0_crank(
    spec: &outlap_schema::Vehicle,
    gov_unit: &outlap_schema::vehicle::DriveUnit,
    curve: &TorqueCurve,
    units: &[T0Unit],
    unit_nodes: &[Option<&str>],
    r_front: f64,
    r_rear: f64,
    notes: &mut Vec<String>,
) -> Result<Option<T0Crank>, T0Error> {
    // The machine's own ladder, folded exactly like a mechanical unit's.
    let (chain, wheels) = crate::graph::flatten_chain(&spec.drivetrain, gov_unit);
    let idx = spec.drivetrain.units.len(); // note index: the governed unit sits past `units`
    let r_wheel = driven_radius(&wheels, r_front, r_rear, idx, notes);
    let (base_ratio, base_eff, gearbox) = fold_path(&chain, idx, notes)?;
    let machine_gears = build_gears(base_ratio, base_eff, gearbox, r_wheel);
    let machine_env = torque_env(curve)?;
    let machine_omega_max = machine_env.domain().1;
    // The reference source: the FIRST-DECLARED mechanical unit that outputs onto the machine's own
    // node. Its ladder alone selects the engaged gear (locked decision: the engine chooses), and it
    // must be index-aligned with the machine's — which holds exactly when both flatten through the
    // same shared gearbox. Anything else (no node, no co-located source, a private gearbox) means
    // the machine is alone on its shaft: it then selects its OWN maximum-wheel-force gear, mirroring
    // the T1 `CrankNode` so the two tiers cannot classify the same car differently.
    let node = gov_unit.output.as_ref().map(NodeId::as_str);
    let reference = node
        .and_then(|node| unit_nodes.iter().position(|n| *n == Some(node)))
        .map(|i| &units[i])
        .filter(|r| r.gears.len() == machine_gears.len());
    if reference.is_none() {
        notes.push(
            "the policy-governed machine shares its output node with no mechanical source (or sits \
             on a gearbox of its own): its ERS adder is capped by its own torque envelope at its \
             own maximum-force gear rather than at an engine-selected crank speed"
                .to_owned(),
        );
    }
    // ω_machine / ω_reference — constant across a shared ladder, so gear 0 fixes it; 1 when the
    // machine references itself.
    let (ref_gears, ref_torque_env, ref_omega_max, machine_per_ref) = match reference {
        Some(r) => {
            let per_ref = match (machine_gears.first(), r.gears.first()) {
                (Some(m), Some(rg)) if rg.omega_per_v.abs() > 0.0 => m.omega_per_v / rg.omega_per_v,
                _ => 1.0,
            };
            (r.gears.clone(), r.torque_env.clone(), r.omega_max, per_ref)
        }
        None => (machine_gears, machine_env.clone(), machine_omega_max, 1.0),
    };
    Ok(Some(T0Crank {
        ref_gears,
        ref_torque_env,
        ref_omega_max,
        machine_env,
        machine_omega_max,
        machine_per_ref,
    }))
}

/// Fit a peak-torque envelope `τ(ω)` from a speed/torque curve (rpm → rad/s at the boundary).
fn torque_env(curve: &TorqueCurve) -> Result<MonotoneCubic<f64>, T0Error> {
    let omega: Vec<f64> = curve
        .speed_rpm
        .iter()
        .map(|r| r * RPM_TO_RAD_PER_S)
        .collect();
    MonotoneCubic::new(omega, curve.torque_nm.clone()).map_err(T0Error::from)
}

/// If the whole drivetrain has exactly one gearbox with a constant efficiency, return it. Scans
/// non-governed units' private paths AND the shared top-level couplers (an f1 gearbox now lives on
/// a coupler); the policy-governed machine is excluded (its ratio never enters the ICE eta).
fn single_gearbox_eff(spec: &outlap_schema::Vehicle) -> Option<f64> {
    let governed = crate::graph::governed_unit_ids(spec);
    let unit_couplers = spec
        .drivetrain
        .units
        .iter()
        .filter(|u| !governed.contains(u.id.as_str()))
        .flat_map(|u| u.path.iter());
    let graph_couplers = spec.drivetrain.couplers.iter().map(|e| &e.coupler);
    let mut found: Option<f64> = None;
    for coupler in unit_couplers.chain(graph_couplers) {
        if let Coupler::Gearbox(g) = coupler {
            if found.is_some() {
                return None; // ambiguous
            }
            match g.efficiency {
                Efficiency::Constant(e) => found = Some(e),
                Efficiency::Map { .. } => return None,
            }
        }
    }
    found
}
