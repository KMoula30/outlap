# T3 chassis — the 14-DOF ride/handling model

The T3 tier is the *"downforce car is real"* tier (HANDOFF §6.1): it adds sprung heave/pitch/roll and
four unsprung vertical degrees of freedom to the T2 planar chassis, so the platform pitches under
braking, rolls in a corner, and the per-wheel vertical load `F_z` comes from a real tyre spring
rather than an algebraic load-transfer formula. This page documents the equations of motion; the
per-tier lap integration (dynamic ride heights → aero-balance shift) lands with the tier wiring.

The hand-written Rust right-hand side ([`ChassisT3`](../../crates/outlap-vehicle/src/chassis.rs)) is
checked against an **independent** `SymPy` derivation
([`t3_chassis_kane.py`](../derivations/t3_chassis_kane.py)) to **1e-12 relative** at 64 randomised
states (Decision #32) — the same discipline as the [T2 chassis](transient_chassis.md). This is what
guards the sign conventions below.

## Degrees of freedom

The chassis fast state is 24 slots (the frozen `ChassisState` footprint; the T2 tier integrates the
first ten and reads the rest as zero):

| Group | States | DOF |
|---|---|---|
| Curvilinear position | `s`, `n`, `ψ_rel` | (kinematic outputs) |
| Handling | `v_x`, `v_y`, `r` | 3 |
| Wheel spins | `ω_fl … ω_rr` | 4 |
| Sprung ride | `z` (heave), `θ` (pitch), `φ` (roll) + rates | 3 |
| Unsprung | `z_{u,i}` per corner + rates | 4 |

3 + 4 + 3 + 4 = **14 DOF**. Handling and yaw are shared by the whole car; heave/pitch/roll belong to
the sprung mass alone; each unsprung mass has one vertical DOF; each wheel spins.

### Sign conventions (ISO 8855: x forward, y left, z up)

- `z`, `z_u` > 0 = up. Suspension **compression** `δ` > 0 loads the spring.
- `θ` > 0 = nose-**down** (dive). `φ` > 0 = roll to the **right** (left side up).
- The sprung corner vertical displacement is `z_corner,i = z − x_i·θ + y_i·φ` (small angle); the
  suspension force is `+`up on the sprung mass, `−`down on the unsprung (Newton's third law).

## Mass / inertia convention (D-M6, Option A)

The 14-DOF model needs the sprung mass, the four unsprung masses, and three rotational inertias:

- Total mass `m = m_s + Σ m_{u,i}` drives the in-plane handling (`v_x, v_y`).
- Sprung mass `m_s = chassis.mass_kg − Σ(2·unsprung_mass_kg)` heaves/pitches/rolls.
- **`inertia[0]` (`I_xx`, roll) and `inertia[1]` (`I_yy`, pitch) are the sprung-mass inertias about
  the sprung CG** — roll and pitch are motions of the sprung mass alone, so their resisting inertia
  is a sprung-mass property (this is also how a CAD sprung model / K&C rig reports them).
- **`inertia[2]` (`I_zz`, yaw) is the whole-car yaw inertia** — the value the T2 tier already uses,
  unchanged (a whole-car spin-rig quantity).

The 14-DOF mass matrix is diagonal by construction (CG-referenced, diagonal inertia tensor); every
coupling below is a forcing term, so each acceleration is explicit (no per-step linear solve, which
keeps the RHS allocation-free).

## Equations of motion

### Handling (whole car) — the T2 planar EOM + gyroscopic yaw

Identical in structure to the [T2 chassis](transient_chassis.md#eom) (the transport terms `r·v_y`,
`r·v_x` are Kane-derived there), with one addition — the **gyroscopic yaw moment** from the four
spinning wheels:

```
v̇_x = ΣF_x/m + r·v_y
v̇_y = ΣF_y/m − r·v_x
ṙ   = (ΣM_z + M_gyro,z) / I_zz
```

`ΣF`, `ΣM_z` sum the wheel-frame tyre forces rotated to the body frame, aero drag, the in-plane
gravity projection (grade/bank), and the yaw-moment demand — exactly as T2. The body-frame CG
accelerations `a_x = ΣF_x/m`, `a_y = ΣF_y/m` drive the ride load transfer below.

### Refinement terms (user-locked to land here; the T2 tier neglected both)

- **Gyroscopic spin×yaw coupling.** Each wheel carries angular momentum `h_i = I_{w,i}·ω_i` about
  its (steered) spin axis. The body angular velocity `Ω = φ̇ x̂ − θ̇ ŷ + r ẑ` precesses it; the
  reaction on the body is `−Ω × Σh_i`. Its yaw component enters the handling EOM; its roll and pitch
  components enter the ride EOM. (The precession is perpendicular to the spin axis, so the wheel spin
  rate `ω̇` keeps the T2 rotor law.)
- **3-D frame-transport.** Following a crest (vertical curvature `κ_v < 0`) lightens the normal load
  by the centripetal `κ_v·v_x²`; a dip loads it. The effective normal-direction gravity is
  `g_n = g·cosθ_road·cosφ_road + κ_v·v_x²`. At T2 this term was capped by an ad-hoc unloading floor
  (`CREST_UNLOADING_FLOOR_G`, "no suspension travel until T3"); at T3 it enters the vertical dynamics
  directly, and the floor retires with the tier.

### Ride (sprung mass, about the sprung CG)

```
m_s·z̈  = Σ F_susp,i − m_s·g_n
I_yy·θ̈ = Σ(−x_i)·F_susp,i + M_pitch,elastic − M_gyro,y
I_xx·φ̈ = Σ( y_i)·F_susp,i + M_roll,elastic  + M_gyro,x
```

The corner suspension force is the **elastic** load path; the roll/pitch DOF deflect the springs, so
elastic load transfer emerges here without being injected. What *is* injected is the moment that
drives the sprung mass:

- `M_roll,elastic = m_s·a_y·(h_s − h_ra)` — the lateral inertial reaction about the roll axis.
- `M_pitch,elastic = −m_s·a_x·h_s·(1 − anti)` — the longitudinal inertial reaction about the sprung
  CG, reduced by the mean anti-dive/anti-squat fraction routed geometrically (below).

### Unsprung (four corners) + the geometric load path

```
m_{u,i}·z̈_{u,i} = F_tyre,i − F_susp,i − m_{u,i}·g_n + F_geom,i
```

`F_tyre,i = k_{tz,i}·(δ_static,i + z_road,i − z_{u,i}) + c_{tz,i}·(ż_road,i − ż_{u,i})` is the tyre
vertical spring/damper — **this is the per-wheel `F_z`** the T3 tyre call reads (replacing the
algebraic load transfer). `F_geom,i` is the **geometric** load transfer routed straight to the
contact patch, bypassing the springs (§7.5 lumped K&C):

- lateral: `−(h_ra/track)·(m·a_y)·side/2` (through the roll-centre height);
- longitudinal: `anti·(m·a_x·h_cg/L)/2` (through anti-dive/anti-squat).

Elastic (through the roll/pitch DOF) + geometric (through the roll centre / anti geometry) sum at the
tyre — the Milliken decomposition, with no double-counting because the two paths are disjoint.

### Suspension force elements

`F_susp,i = k_ride,i·δ_i + c_damp,i(δ̇_i) + F_bump,i(δ_i) + F_arb,i`, with compression
`δ_i = δ_static,i + z_{u,i} − z_corner,i`:

- **Spring** — linear ride rate `k_ride` (the existing schema field).
- **Damper** — bump/rebound coefficients `c_bump` (δ̇ ≥ 0) / `c_rebound` (δ̇ < 0). Always dissipative:
  the force is `c·δ̇`, so the power `c·δ̇² ≥ 0`.
- **Bumpstop** — a progressive rate `k_bs` engaging past a gap, smoothed **C¹**
  (`0 → p²/2s → p − s/2`) so the RK path never sees a discontinuous force *or* stiffness at contact.
- **ARB** — an absolute roll stiffness `k_arb` (N·m/rad) resisting the differential travel across an
  axle: a restoring `∓` force couple over the track.

### Static equilibrium

With the states as displacements from the design ride position and the static compressions
`δ_static`, `δ_tyre,static` carrying the corner loads, a car at rest has zero acceleration in every
DOF (`Σ k_ride·δ_static = m_s·g`; each tyre carries its sprung corner + unsprung weight). The Rust
`static_equilibrium_settles` test ties the compressions to gravity.

## Verification

- **1e-12 EOM check** — all 24 RHS entries vs the `SymPy` `KanesMethod` derivation, 64 randomised
  states (`kane_fixture_t3.rs`; worst-case ~9e-14 relative). The unsprung accelerations reach
  `O(10³ m/s²)` — a stiff tyre spring over a light unsprung mass — so the gate is relative (an
  absolute 1e-12 would be ~1e-15 relative, below f64 summation-order noise; a sign/formula error
  moves an entry by `O(1)` relative, orders of magnitude above the gate).
- Static equilibrium; spring/ARB restoring; damper dissipation; gyroscopic coupling live;
  braking-dives sign; `f32`/`f64` parity; **alloc = 0** for the RHS.
- The T2 chassis and its fixture are **untouched** — a T2 lap is byte-identical.

## References

- Milliken & Milliken, *Race Car Vehicle Dynamics* — load transfer, roll-centre / anti geometry, K&C.
- Guiggiani, *The Science of Vehicle Dynamics* — the 14-DOF ride/handling split, roll axis.
- Pacejka, *Tyre and Vehicle Dynamics* (2012), ch. 1 — tyre vertical stiffness.
- Kane & Levinson, *Dynamics: Theory and Applications* — the method; `sympy.physics.mechanics`.

Consulted open-source projects (approach only, re-authored from the literature + FIA/K&C conventions
per the clean-room rule): none for this model beyond the cited texts.
