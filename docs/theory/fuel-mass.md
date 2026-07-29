<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Fuel mass as a slow state (§8.1)

Fuel is a **live slow state**. It is not a constant. The tank drains as the internal-combustion
engine burns fuel. The mass shrinks, the center of gravity migrates, and both feed back into the lap
dynamics, in both tiers.

A car with no `fuel:` block carries a constant mass. It reproduces the results from before fuel
existed, byte for byte.

This is how M6 PR5 realizes the fuel path of §8.1 (Decision D-M6-4).

The framing is a mean-value engine model (Eriksson & Nielsen, *Modeling and Control of Engines and
Drivelines*). A map of brake-thermal efficiency summarizes the ICE. There is no cylinder-by-cylinder
cycle. The fuel rate is therefore an algebraic function of the operating point.

## What mass means here (D-M6-4a)

`chassis.mass_kg` is the ONE all-inclusive **dry** number. It covers the car **and the driver**. It
excludes fuel. For F1 2026 this is the minimum-mass convention of ≤ 768 kg. The split between 768 kg
and the driver is a note in the documentation. It is not a separate field.

The race load in `fuel.initial_kg`, typically 70 kg to 80 kg, **adds on top**:

$$ m_0 = \text{chassis.mass\_kg} + \text{fuel.initial\_kg} $$

The running mass at any point is `m(t) = m₀ − ∫ ṁ_fuel dt`, clamped at the dry mass. The full-tank
mass `m₀` is the **reference for the envelope**, as described below.

## The rate at which fuel burns

The ICE draws chemical power to deliver its **mechanical** output:

$$ \dot m_\text{fuel} = \frac{P_\text{chem}}{\text{LHV}}, \qquad P_\text{chem} = \frac{P_\text{mech}}{\eta_\text{thermal}} $$

`LHV` is the lower heating value of the fuel, held in `fuel.lhv_j_per_kg`. It defaults to 43 MJ/kg,
which suits pump gasoline and F1 E-fuel. `η_thermal` is the brake-thermal efficiency, read from the
`.ptm` map of the ICE.

On a hybrid the ICE covers only its **share** of traction. That share is the drive demand minus what
the electric MGU-K deploys, because the MGU-K draws from the battery and not from fuel. `P_mech` is
therefore the mechanical power attributed to the ICE. It is never the full traction. Using the full
traction would count the electric energy twice and break the closure of §14.

- **QSS** reads the efficiency of the ICE map at each operating point directly, through
  `T1Powertrain::ice_fuel_rate_kg_per_s`. It integrates the burn over each path segment.
- **T2** uses a **representative** scalar efficiency, sampled from the ICE map at assembly by
  `representative_ice_efficiency`. It banks the burn on each fast step and drains the tank on the
  decimated slow clock.

The two tiers therefore agree on the fuel channel only to within that simplification. The parity
gate for fuel is *recorded*, not asserted (Decision #48). The scalar η is the acknowledged
difference.

## The fuel-flow limit, expressed only in energy (D-M6-5)

The F1 regulation on fuel flow (FIA Technical Regulations C5.2.3–C5.2.5) is an **energy**
constraint. outlap writes it as `flow_limit = { mj_per_h, rpm_line? }`, which holds two parts:

- a flat cap, `mj_per_h`, which is the ceiling of C5.2.4; and
- below `below_rpm`, the line of C5.2.5, `EF(MJ/h) = slope·N + intercept`. For F1 2026 that applies
  at `N < 10 500` rpm, and it reads `EF = 0.27·N + 165`.

The configurable `LHV` converts between kg/h and MJ/h. The "ṁ_max" of §8.1 is therefore **satisfied
by energy equivalence**. It is the same physical limit, expressed once, in energy units.

The limit is a **constraint on the ICE power available**, `P_crank ≤ η · EF_limit`. It shrinks the
traction envelope. It never clamps the ṁ accounting on its own. Clamping the flow while leaving the
work untouched would make the traction work exceed the fuel burned, and that would break the energy
closure of §14.

## Feedback from mass and CG: separable corrections, NOT a grid axis (D-M6-4)

Fuel couples into the QSS g-g-g-v envelope through **separable multiplicative corrections**, which
is the mechanism of Decision #31. It does not add a re-solved axis to the grid.

This is the **opposite** conclusion to the tire-state amendment of Decision #49, and for a reason.
Tire thermal state and wear reshape grip **non-linearly and non-monotonically**: the grip window
peaks at `T_opt`, and the wear cliff is a sigmoid. A re-solved axis therefore buys real fidelity.

Mass and CG are different. They perturb the load-transfer algebra **smoothly and monotonically**. A
first-order secant is accurate for them: `∂gg/∂mass` and `∂gg/∂cg`, validated in CI against full T1
re-solves. It also avoids multiplying an envelope build that takes 5 s to 22 s. See the note in §1.

- **The envelope reference is the full-tank m₀** (D-M6-4b). Assembly builds the T1 vehicle, and
  therefore the envelope, at `m₀` and at the full-tank CG. The correction for mass and CG is
  therefore **exactly 1.0 at the start of a lap**, which is the identity slice. This mirrors the
  invariant of #49 at `T_opt` and zero wear. It then drifts as the tank drains.
- **CG migration ships in both tiers** (D-M6-4c). The centroid of the fuel tank is an `[x, z]`
  offset from the **dry** CG, held in `cg_offset_m` under ISO 8855, where +x is forward and +z is up.
  The running CG is the mass-weighted blend of the dry CG and the tank centroid. Burning fuel
  therefore moves the CG **linearly** toward the dry position. That shifts both the front and rear
  split, `a_f` and `b_r`, and the CG height, `h_cg`.

  QSS applies a `with_cg` secant to the envelope, plus `a_f` and `h_cg` at each station in the
  load-transfer algebra. T2 updates the mass and CG that live in the blocks, on the slow clock,
  through a single `apply_mass_state` fan-out. That fan-out refreshes the load geometry, the chassis
  inertia block, and the wheel geometry in the tire block. A conservation property test guards it:
  `ΣF_z = m·g`, and the pitch balances about the new CG.

The point-mass longitudinal equations use the mass at each station directly, through `F/m`. The drag
deceleration also scales as `1/m`. A lighter car therefore both corners and accelerates harder. An
F1 stint **starts heavy and gets faster** as the tank drains.

## What this model does not do

It has no fuel sloshing and no dynamics of tank level. It varies neither fuel temperature nor fuel
density. It couples to neither aero nor thermal. It models only two consequences of burning fuel:
the inertial one, through mass, and the grip one, through CG.

Two race-level features are **strategy-layer** work (HANDOFF §16), not fuel physics: optimizing the
fuel target so the car finishes with the 1 L sample reserve that the FIA requires, and saving fuel
by lift-and-coast through the `lift_point` hook on `u(s)`.

## References

- Eriksson & Nielsen, *Modeling and Control of Engines and Drivelines* — ICE mean-value framing.
- FIA 2026 Formula 1 Technical Regulations, C5.2.3–C5.2.5 — the fuel-mass-flow / energy-flow limit.
- HANDOFF §1 Locked Decisions #31 (envelope corrections vs axes) + the D-M6-4 note (this decision).
