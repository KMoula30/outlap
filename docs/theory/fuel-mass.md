<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Fuel mass as a slow state (§8.1)

Fuel is a **live slow state**, not a constant. The tank drains as the internal-combustion engine
burns fuel, and the shrinking mass and migrating centre of gravity feed back into the lap dynamics
in both tiers. A car with no `fuel:` block carries a constant mass and reproduces the pre-fuel
results byte-identically. This is the M6/PR5 realisation of the §8.1 fuel path (Decision D-M6-4).

The framing is a mean-value engine model (Eriksson & Nielsen, *Modeling and Control of Engines and
Drivelines*): the ICE is summarised by a brake-thermal efficiency map rather than a cylinder-by-
cylinder cycle, so the fuel rate is an algebraic function of the operating point.

## Mass semantics (D-M6-4a)

`chassis.mass_kg` is the ONE all-inclusive **dry** number — the car **plus driver**, no fuel (for
F1 2026 this is the ≤ 768 kg minimum-mass convention; the 768 − driver split is a documentation
note, not a separate field). The `fuel.initial_kg` race load (a typical 70–80 kg) **adds on top**:

$$ m_0 = \text{chassis.mass\_kg} + \text{fuel.initial\_kg} $$

and the running mass at any point is `m(t) = m₀ − ∫ ṁ_fuel dt`, clamped at the dry mass. The full-
tank mass `m₀` is the **envelope reference** (see below).

## Fuel-burn rate

The ICE burns fuel for the chemical power it draws to deliver its **mechanical** output:

$$ \dot m_\text{fuel} = \frac{P_\text{chem}}{\text{LHV}}, \qquad P_\text{chem} = \frac{P_\text{mech}}{\eta_\text{thermal}} $$

where `LHV` is the fuel's lower heating value (`fuel.lhv_j_per_kg`, default 43 MJ/kg — pump gasoline
/ F1 E-fuel) and `η_thermal` the brake-thermal efficiency from the ICE `.ptm` map. On a hybrid the
ICE covers only its **share** of traction — the drive demand net of the electric MGU-K deploy (which
draws from the battery, not fuel) — so `P_mech` is the ICE-attributed mechanical power, never the
full traction (that would double-count the electric energy and break the §14 closure).

- **QSS** consumes the ICE map's per-operating-point efficiency directly
  (`T1Powertrain::ice_fuel_rate_kg_per_s`), integrating the burn per path segment.
- **T2** uses a **representative** scalar efficiency sampled from the ICE map at assembly
  (`representative_ice_efficiency`), banking the burn each fast step and draining the tank on the
  decimated slow clock. The QSS↔T2 fuel-channel agreement is therefore a *recorded* parity gate
  (Decision #48), not an asserted one — the scalar-η simplification is the acknowledged difference.

## Fuel-flow limit — energy-only form (D-M6-5)

The F1 fuel-flow regulation (FIA Technical Regulations C5.2.3–C5.2.5) is expressed as an **energy**
constraint, `flow_limit = { mj_per_h, rpm_line? }`:

- a flat cap `mj_per_h` (the C5.2.4 ceiling), and
- below `below_rpm` the C5.2.5 line `EF(MJ/h) = slope·N + intercept` (F1 2026: `N < 10 500` rpm,
  `EF = 0.27·N + 165`).

The kg/h ↔ MJ/h equivalence goes through the configurable `LHV`, so §8.1's "ṁ_max" is **satisfied by
energy equivalence** — the same physical limit, expressed once in energy units. The limit is a
**constraint on available ICE power** (`P_crank ≤ η · EF_limit`) that shrinks the traction envelope;
it never clamps the ṁ accounting independently (clamping the flow while leaving the work untouched
would make traction work exceed the fuel burned, breaking §14 energy closure).

## Mass/CG feedback — separable corrections, NOT a grid axis (D-M6-4)

Fuel couples to the QSS g-g-g-v envelope through **separable multiplicative corrections** (the
Decision #31 mechanism), NOT a re-solved grid axis. This is the **opposite** conclusion to the
tyre-state amendment (Decision #49): tyre thermal/wear reshape grip **non-linearly and non-
monotonically** (the grip window peaks at `T_opt`, the wear cliff is a sigmoid), so a re-solved axis
buys real fidelity; mass and CG, by contrast, are **smooth and monotone** perturbations of the load-
transfer algebra, for which a first-order secant (`∂gg/∂mass`, `∂gg/∂cg`, validated against full T1
re-solves in CI) is accurate and avoids multiplying the 5–22 s envelope build. See the §1 note.

- **Envelope reference = full-tank m₀** (D-M6-4b): assembly builds the T1 vehicle — hence the
  envelope — at `m₀` and the full-tank CG, so the mass/CG correction is **exactly 1.0 at lap start**
  (the identity slice, mirroring the #49 `T_opt`/zero-wear invariant) and drifts as the tank drains.
- **CG migration ships in both tiers** (D-M6-4c). The fuel-tank centroid is an `[x, z]` offset from
  the **dry** CG (ISO 8855: +x forward, +z up, `cg_offset_m`); the running CG is the mass-weighted
  blend of the dry CG and the tank centroid, so burning fuel moves the CG **linearly** toward the dry
  position and shifts both the front/rear split (`a_f`/`b_r`) and the CG height (`h_cg`). QSS applies
  a `with_cg` envelope secant + per-station `a_f`/`h_cg` in the load-transfer algebra; T2 updates the
  block-resident mass/CG on the slow clock through the single `apply_mass_state` fan-out (which
  refreshes the load geometry, the chassis inertia block, and the tyre block's wheel geometry, with
  a conservation property test: `ΣF_z = m·g`, pitch balance about the new CG).

The point-mass longitudinal equations additionally use the per-station mass directly (`F/m`, and the
drag deceleration scales `∝ 1/m`), so a lighter car both corners and accelerates harder — an F1
stint **starts heavy and gets faster** as the tank drains.

## What this is not

No fuel sloshing / tank-level dynamics, no fuel temperature or density variation, no aero or thermal
coupling — only the inertial (mass) and grip (CG) consequences of burning fuel. The race-level
"finish with the FIA 1 L sample reserve" fuel-target optimisation and fuel-saving (lift-and-coast via
the `lift_point` `u(s)` hook) are **strategy-layer** features (HANDOFF §16), not fuel physics.

## References

- Eriksson & Nielsen, *Modeling and Control of Engines and Drivelines* — ICE mean-value framing.
- FIA 2026 Formula 1 Technical Regulations, C5.2.3–C5.2.5 — the fuel-mass-flow / energy-flow limit.
- HANDOFF §1 Locked Decisions #31 (envelope corrections vs axes) + the D-M6-4 note (this decision).
