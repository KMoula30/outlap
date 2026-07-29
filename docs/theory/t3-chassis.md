# T3 chassis: the 14-DOF ride and handling model

The T3 tier is the tier where *the downforce car becomes real* (HANDOFF §6.1). It adds four things
to the planar T2 chassis: sprung heave, pitch, and roll, plus four unsprung vertical degrees of
freedom.

Two consequences follow. The platform pitches under braking and rolls in a corner. And the vertical
load at each wheel, `F_z`, comes from a real tire spring, instead of from an algebraic formula for
load transfer.

This page documents the equations of motion. The lap integration for the tier, where dynamic ride
heights shift the aero balance, lands with the tier wiring.

The hand-written Rust right-hand side is
[`ChassisT3`](../../crates/outlap-vehicle/src/chassis.rs). It is checked against an **independent**
`SymPy` derivation, [`t3_chassis_kane.py`](../derivations/t3_chassis_kane.py), to **1e-12 relative**,
at 64 randomized states (Decision #32). That is the same discipline as the
[T2 chassis](transient_chassis.md). It is what guards the sign conventions below.

## Degrees of freedom

The fast state of the chassis has 24 slots. That is the frozen footprint of `ChassisState`. The T2
tier integrates the first ten and reads the rest as zero.

| Group | States | DOF |
|---|---|---|
| Curvilinear position | `s`, `n`, `ψ_rel` | (kinematic outputs) |
| Handling | `v_x`, `v_y`, `r` | 3 |
| Wheel spins | `ω_fl … ω_rr` | 4 |
| Sprung ride | `z` (heave), `θ` (pitch), `φ` (roll) + rates | 3 |
| Unsprung | `z_{u,i}` per corner + rates | 4 |

3 + 4 + 3 + 4 = **14 DOF**.

The whole car shares handling and yaw. Heave, pitch, and roll belong to the sprung mass alone. Each
unsprung mass has one vertical DOF. Each wheel spins.

### Sign conventions (ISO 8855: x forward, y left, z up)

- `z` and `z_u` are positive upward. Suspension **compression** `δ` is positive, and it loads the
  spring.
- `θ` is positive nose-**down**, which is dive. `φ` is positive rolling to the **right**, which
  raises the left side.
- The vertical displacement of a sprung corner is `z_corner,i = z − x_i·θ + y_i·φ`, at small angles.
  The suspension force acts upward on the sprung mass and downward on the unsprung mass, by Newton's
  third law.

## The convention for mass and inertia (D-M6, Option A)

The 14-DOF model needs the sprung mass, the four unsprung masses, and three rotational inertias.

- The total mass, `m = m_s + Σ m_{u,i}`, drives the in-plane handling, `v_x` and `v_y`.
- The sprung mass, `m_s = chassis.mass_kg − Σ(2·unsprung_mass_kg)`, heaves, pitches, and rolls.
- **`inertia[0]` (`I_xx`, roll) and `inertia[1]` (`I_yy`, pitch) are inertias of the sprung mass
  about the sprung CG.** Roll and pitch are motions of the sprung mass alone, so the inertia that
  resists them is a property of the sprung mass. A CAD sprung model, or a K&C rig, reports them the
  same way.
- **`inertia[2]` (`I_zz`, yaw) is the yaw inertia of the whole car.** This is the value that the T2
  tier already uses, unchanged. A spin rig measures it on the whole car.

The 14-DOF mass matrix is diagonal by construction, because it is referenced to the CG and the
inertia tensor is diagonal. Every coupling below is therefore a forcing term. Each acceleration is
explicit, so no step needs a linear solve, and the RHS allocates nothing.

## Equations of motion

### Handling for the whole car: the planar T2 equations, plus gyroscopic yaw

These are identical in structure to the
[T2 chassis](transient_chassis.md#2-equations-of-motion). Kane's method derives the transport terms
`r·v_y` and `r·v_x` there. T3 adds one term: the **gyroscopic yaw moment** from the four spinning
wheels.

```
v̇_x = ΣF_x/m + r·v_y
v̇_y = ΣF_y/m − r·v_x
ṙ   = (ΣM_z + M_gyro,z) / I_zz
```

`ΣF` and `ΣM_z` sum four contributions, exactly as T2 does: the wheel-frame tire forces rotated into
the body frame, the aero drag, the in-plane projection of gravity from grade and banking, and the
yaw-moment demand.

The body-frame accelerations of the CG, `a_x = ΣF_x/m` and `a_y = ΣF_y/m`, drive the ride load
transfer below.

### Two refinement terms, which the user locked to land here, and which T2 neglected

- **Gyroscopic coupling between spin and yaw.** Each wheel carries angular momentum
  `h_i = I_{w,i}·ω_i` about its steered spin axis. The angular velocity of the body,
  `Ω = φ̇ x̂ − θ̇ ŷ + r ẑ`, precesses that momentum. The reaction on the body is `−Ω × Σh_i`.

  Its yaw component enters the handling equations. Its roll and pitch components enter the ride
  equations. The precession is perpendicular to the spin axis, so the spin rate `ω̇` keeps the T2
  rotor law.
- **Frame transport in 3-D.** Following a crest, where the vertical curvature `κ_v < 0`, lightens
  the normal load by the centripetal term `κ_v·v_x²`. A dip loads it. The effective gravity along
  the normal direction is `g_n = g·cosθ_road·cosφ_road + κ_v·v_x²`.

  At T2 an ad-hoc floor capped this term, `CREST_UNLOADING_FLOOR_G`, because that tier has no
  suspension travel until T3. At T3 the term enters the vertical dynamics directly, and the floor
  retires with the tier.

### Ride: the sprung mass, about the sprung CG

```
m_s·z̈  = Σ F_susp,i − m_s·g_n
I_yy·θ̈ = Σ(−x_i)·F_susp,i + M_pitch,elastic − M_gyro,y
I_xx·φ̈ = Σ( y_i)·F_susp,i + M_roll,elastic  + M_gyro,x
```

The suspension force at each corner is the **elastic** load path. The roll and pitch degrees of
freedom deflect the springs, so elastic load transfer emerges here. Nothing injects it.

What *is* injected is the moment that drives the sprung mass:

- `M_roll,elastic = m_s·a_y·(h_s − h_ra)` is the lateral inertial reaction about the roll axis.
- `M_pitch,elastic = −m_s·a_x·h_s·(1 − anti)` is the longitudinal inertial reaction about the sprung
  CG. The mean anti-dive and anti-squat fraction routes part of it geometrically, as described
  below, which reduces this term.

### The unsprung masses at four corners, and the geometric load path

```
m_{u,i}·z̈_{u,i} = F_tyre,i − F_susp,i − m_{u,i}·g_n + F_geom,i
```

`F_tyre,i = k_{tz,i}·(δ_static,i + z_road,i − z_{u,i}) + c_{tz,i}·(ż_road,i − ż_{u,i})` is the
vertical spring and damper of the tire. **This is the per-wheel `F_z`** that the T3 tire call reads.
It replaces the algebraic load transfer.

`F_geom,i` is the **geometric** load transfer. It routes straight to the contact patch and bypasses
the springs. This is the lumped K&C model of §7.5:

- laterally, `−(h_ra/track)·(m·a_y)·side/2`, through the roll-center height;
- longitudinally, `anti·(m·a_x·h_cg/L)/2`, through anti-dive and anti-squat.

The elastic path, through the roll and pitch DOF, and the geometric path, through the roll center
and the anti geometry, sum at the tire. That is the Milliken decomposition. Nothing is counted
twice, because the two paths are disjoint.

### The elements of the suspension force

`F_susp,i = k_ride,i·δ_i + c_damp,i(δ̇_i) + F_bump,i(δ_i) + F_arb,i`, with compression
`δ_i = δ_static,i + z_{u,i} − z_corner,i`.

- **Spring.** A linear ride rate, `k_ride`. The schema field already exists.
- **Damper.** Coefficients for bump and rebound: `c_bump` when `δ̇ ≥ 0`, and `c_rebound` when
  `δ̇ < 0`. It always dissipates, because the force is `c·δ̇` and the power is therefore
  `c·δ̇² ≥ 0`.
- **Bumpstop.** A progressive rate `k_bs` engages past a gap. It is smoothed to be **C¹**, through
  `0 → p²/2s → p − s/2`. The RK path therefore never sees a discontinuous force, nor a discontinuous
  stiffness, at contact.
- **ARB.** An absolute roll stiffness `k_arb`, in N·m/rad. It resists differential travel across an
  axle, as a restoring couple of `∓` forces over the track.

### Static equilibrium

The states are displacements from the design ride position. The static compressions `δ_static` and
`δ_tyre,static` carry the corner loads.

A car at rest therefore has zero acceleration in every DOF. `Σ k_ride·δ_static = m_s·g`, and each
tire carries its sprung corner plus its unsprung weight. The Rust test
`static_equilibrium_settles` ties the compressions to gravity.

## Integrating the tier (PR7): the 14-DOF chassis around a lap

PR6 proved the RHS. PR7 wires it into a full transient lap. It has three pieces.

### One shared `TransientSolver`, with one Fz-coupling strategy for each tier (Decision #53)

The transient solver is a single type, generic over the block composition:
`TransientSolver<T, B: TierBlocks<T>>`. It is monomorphized for each tier. Dispatch is static, the
block structs are concrete, and there is no `dyn` and no per-step enum match.

The T2 path is instruction for instruction the solver as it stood before PR7. A T2 lap therefore
stays byte-identical.

The differences between tiers live behind the trait. There are three.

- **The block chain.** T2 runs `driver → powertrain → load(algebraic Fz) → aero → tyre → tv →
  chassis`. T3 runs `driver → powertrain → aero(ride-height) → t3-load(tyre-spring Fz) → tyre → tv →
  chassis(14-DOF)`.
- **The Fz-coupling strategy.** T2 resolves the *algebraic* load transfer, which depends on the
  accelerations. It therefore iterates a Picard fixed point at the start of the step, and it applies
  the crest floor. At T3 the per-wheel `F_z` is the *deflection of the tire spring*, which is a
  function of the suspension state alone. One evaluation therefore resolves the forces. There is no
  Picard loop, and the crest floor retires with the strategy.
- **The set of integrated slots.** `t2_integrated_slots` holds 10 DOF plus the controller.
  `t3_integrated_slots` holds all 24 plus the controller.

The machinery for the slow clock, the energy ledger, fuel, tire thermal state, and ERS is written
once against the trait, and both tiers share it. The seam does not fork it (Decision #53).

### Aero at the dynamic ride height, applied to the sprung mass (Decision #54)

The aero block evaluates drag and the downforce on each axle at the **instantaneous** ride heights,
`h_f = h_ref,f + (z − a_f·θ)` and `h_r = h_ref,r + (z + b_r·θ)`, in mm. It reads them through the
shared aero map over ride height (Decision #30).

Under braking the platform pitches nose-down, so the front ride height drops, and the map returns a
**forward** shift in aero balance. That is *the* defining behavior of a downforce car (§6.1).

A car without an aero map keeps its constant lumped coefficients. Those are inert to ride height, so
its T3 aero is byte-identical to its T2 aero.

The downforce is applied to the **sprung body**: a heave force of `−(F_z,f + F_z,r)`, and a pitch
moment of `F_z,f·a_f − F_z,r·b_r`. This is exactly what the PR6 RHS derives, in the `fzaf` and
`fzar` terms above.

It reaches the tires *through the springs*. The sprung mass sinks under load. That compresses the
suspension, which compresses the tire spring, which raises the per-wheel `F_z` that the contact
patch carries. This is the honest coupling of ground effect: more downforce lowers the platform, and
the map then re-reads a lower ride height.

That is why a per-wheel `F_z` that "comes from the deflection of the tire spring" carries the
downforce without a separate aero term at the contact patch.

PR7 extended the 14-DOF RHS and its 1e-12 fixture for this, through the `fzaf` and `fzar` inputs.
The T2 chassis and its fixture are untouched.

The solver seeds the suspension near its aero-loaded static equilibrium at the entry speed. The
platform therefore does not slam under the large downforce load on the first step.

### Numerics: bumpstop stiffness, and the sub-cycle

Free wheel hop, at about 15 Hz to 20 Hz on a stiff race car, sits comfortably inside the stability
region of Heun at `dt = 1 ms`.

The binding case is a very stiff bumpstop, or a platform that is over-stiffened. There the frequency
of the corner mode, `ω = √(k_eff/m_u)`, rises as `√k`, while the damping ratio falls as `1/√k`. Once
`ω·dt` approaches about 0.5, the explicit step goes unstable.

At the stiffness that `f1_2026` ships with, the T3 lap is comfortably stable at 1 ms. The parity
sweep over stiffness, described below, runs at a finer `dt`, so even a platform 30 times stiffer
stays inside Heun.

If a future setup needs more, the documented remedy is a deterministic fixed sub-cycle of the
unsprung block. It is never adaptive.

## Verification

- **The 1e-12 check on the equations of motion** compares all 24 RHS entries against the `SymPy`
  `KanesMethod` derivation, at 64 randomized states, in `kane_fixture_t3.rs`. The worst case is
  about 9e-14 relative.

  The gate is relative for a reason. The unsprung accelerations reach `O(10³ m/s²)`, because a stiff
  tire spring acts on a light unsprung mass. An absolute gate of 1e-12 would be about 1e-15
  relative, which is below the noise from summation order in f64. An error in a sign or a formula
  moves an entry by `O(1)` relative, which is orders of magnitude above the gate.
- Other tests cover: static equilibrium; the restoring action of the spring and the ARB; dissipation
  in the damper; live gyroscopic coupling; the sign of dive under braking; parity between `f32` and
  `f64`; and **zero allocation** in the RHS.
- The T2 chassis and its fixture are **untouched**, so a T2 lap is byte-identical.
- **Aero on the sprung mass** (PR7): downforce pushes the platform down, and front downforce pitches
  the nose down, which is the mechanism that shifts aero balance. Unit tests check both against the
  sign of the heave and pitch terms in the RHS.

### Gates on the tier integration (PR7)

- **Parity between T2 and T3.** On a flat skidpad, the two tiers share the same constant aero. The
  test stiffens the T3 suspension over `k ∈ {1, 3, 10, 30}×`, scales the dampers by `√k`, divides
  the static compressions by `k`, and holds the tire `k_z` at a physical value. Across the whole
  sweep, the T3 speed trajectory holds to **0.53 %** of the T2 one.

  The residual is small, and it does not depend on stiffness. It is the refinement physics that T3
  adds and T2 neglects: the gyroscopic coupling between spin and yaw, and the frame transport
  through `κ_v·v²`. It is recorded, and it is not an artifact of the suspension (Decision #48).
- **The Eau Rouge crest.** T3 rides a sustained crest whose unloading through `κ_v·v²` reaches about
  0.55 g. That is well past the T2 crest floor of 0.15 g. T3 stays finite on the honest 3-D physics,
  with **no floor**, because the suspension absorbs the unloading.
- **Throughput.** The T3 step runs at about **96 k steps/s on each core**. That is *faster* than the
  62 k of T2. The tire-spring `F_z` resolves in one evaluation for each RK stage, where T2 runs
  three extra Picard evaluations for the algebraic coupling. That saving more than pays for the
  heavier 24-DOF RHS. The tripwire for T3 is 40 k, which is about half the measured value. The
  30 k tripwire of T2 is untouched.
- Three more gates: the T3 step allocates nothing; the **T3 block schedule** is asserted to be a
  valid topological linearization, programmatically and not against a hardcoded order; and a vehicle
  with `tier: t3` that is missing suspension data fails at assembly, with a plain-language list of
  the fields. Nothing is estimated, and nothing panics.

## References

- Milliken & Milliken, *Race Car Vehicle Dynamics* — load transfer, roll-centre / anti geometry, K&C.
- Guiggiani, *The Science of Vehicle Dynamics* — the 14-DOF ride/handling split, roll axis.
- Pacejka, *Tyre and Vehicle Dynamics* (2012), ch. 1 — tyre vertical stiffness.
- Kane & Levinson, *Dynamics: Theory and Applications* — the method; `sympy.physics.mechanics`.

Open-source projects consulted for approach only, and re-authored from the literature and from the
FIA and K&C conventions, under the clean-room rule: none for this model, beyond the texts cited
above.
