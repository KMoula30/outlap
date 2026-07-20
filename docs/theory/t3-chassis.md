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

## Tier integration (PR7): the 14-DOF chassis in a lap

PR6 proved the RHS; PR7 wires it into a full transient lap. The pieces:

### One shared `TransientSolver`, an Fz-coupling strategy per tier (Decision #53)

The transient solver is a single type generic over the block composition
(`TransientSolver<T, B: TierBlocks<T>>`), monomorphised per tier — static dispatch, concrete block
structs, no `dyn` and no per-step enum match. The T2 path is instruction-for-instruction the pre-PR7
solver, so a T2 lap stays byte-identical. The tier differences live behind the trait:

- **the block chain** — T2 runs `driver → powertrain → load(algebraic Fz) → aero → tyre → tv →
  chassis`; T3 runs `driver → powertrain → aero(ride-height) → t3-load(tyre-spring Fz) → tyre → tv →
  chassis(14-DOF)`;
- **the Fz-coupling strategy** — T2 resolves the *algebraic* load transfer, which depends on the
  accelerations, so it iterates a Picard fixed point at the step start and applies the crest floor;
  T3's per-wheel `F_z` is the *tyre-spring deflection* (a function of the suspension state alone), so
  one evaluation resolves the forces — no Picard loop, and the crest floor retires with the strategy;
- **the integrated slot set** — `t2_integrated_slots` (10 DOF + controller) vs `t3_integrated_slots`
  (the full 24 + controller).

The slow-clock / energy-ledger / fuel / tyre-thermal / ERS machinery is written once against the
trait and shared by both tiers — the seam does not fork it (Decision #53).

### Aero at dynamic ride height, applied to the sprung mass (Decision #54)

The aero block evaluates drag + per-axle downforce at the **instantaneous** ride heights
`h_f = h_ref,f + (z − a_f·θ)`, `h_r = h_ref,r + (z + b_r·θ)` (mm) through the shared ride-height aero
map (Decision #30). Under braking the platform pitches nose-down, the front ride height drops, and
the map returns a **forward** aero-balance shift — *the* defining downforce-car behaviour (§6.1). A
car without an aero map keeps the constant lumped coefficients (ride-height inert, so its T3 aero is
byte-identical to its T2 aero).

The downforce is applied to the **sprung body** (heave force `−(F_z,f + F_z,r)`, pitch moment
`F_z,f·a_f − F_z,r·b_r`), exactly as the PR6 RHS derives (the `fzaf`/`fzar` terms above). It reaches
the tyres *through the springs*: the sprung mass sinks under load, compressing the suspension, which
compresses the tyre spring, which raises the per-wheel `F_z` the contact patch carries — the honest
ground-effect coupling (more downforce → lower platform → the map re-reads a lower ride height). This
is why the per-wheel `F_z` "comes from the tyre-spring deflection" carries the downforce without a
separate contact-patch aero term. The 14-DOF RHS + its 1e-12 fixture were extended for this in PR7
(the `fzaf`/`fzar` inputs); the T2 chassis and its fixture are untouched.

The solver seeds the suspension near its aero-loaded static equilibrium at the entry speed so the
platform does not slam under the (large) downforce load at the first step.

### Numerics — bumpstop stiffness and the sub-cycle

Free wheel-hop (~15–20 Hz on a stiff race car) sits comfortably inside Heun's stability region at
`dt = 1 ms`. The binding case is a very stiff bumpstop or a heavily over-stiffened platform, where the
corner mode frequency `ω = √(k_eff/m_u)` rises as `√k` while the damping ratio falls as `1/√k`; once
`ω·dt` approaches ~0.5 the explicit step goes unstable. At the shipped f1_2026 stiffness the T3 lap is
comfortably stable at 1 ms; the parity stiffness sweep (below) runs at a finer `dt` so even a 30×
platform stays inside Heun. A deterministic fixed sub-cycle of the unsprung block (never adaptive) is
the documented remedy if a future setup needs it.

## Verification

- **1e-12 EOM check** — all 24 RHS entries vs the `SymPy` `KanesMethod` derivation, 64 randomised
  states (`kane_fixture_t3.rs`; worst-case ~9e-14 relative). The unsprung accelerations reach
  `O(10³ m/s²)` — a stiff tyre spring over a light unsprung mass — so the gate is relative (an
  absolute 1e-12 would be ~1e-15 relative, below f64 summation-order noise; a sign/formula error
  moves an entry by `O(1)` relative, orders of magnitude above the gate).
- Static equilibrium; spring/ARB restoring; damper dissipation; gyroscopic coupling live;
  braking-dives sign; `f32`/`f64` parity; **alloc = 0** for the RHS.
- The T2 chassis and its fixture are **untouched** — a T2 lap is byte-identical.
- **Aero on the sprung mass** (PR7): downforce pushes the platform down; front downforce pitches the
  nose down (the aero-balance-shift mechanism) — unit-tested against the sign of the heave/pitch RHS.

### Tier-integration gates (PR7)

- **T2↔T3 parity** — on a flat skidpad the two tiers share the same constant aero; stiffening the T3
  suspension over `k ∈ {1, 3, 10, 30}×` (dampers `×√k`, static compressions `÷k`, tyre `k_z` held
  physical) holds the T3 speed trajectory to **0.53 %** of the T2 one across the whole sweep. The
  small, stiffness-independent residual is the refinement physics T3 adds and T2 neglects (the
  gyroscopic spin×yaw coupling and the `κ_v·v²` frame transport) — recorded, not a suspension
  artifact (Decision #48).
- **Eau-Rouge crest** — T3 rides a sustained crest whose `κ_v·v²` unloading (~0.55 g) is well past the
  T2 crest floor (0.15 g), staying finite on the honest 3-D physics with **no floor** — the
  suspension absorbs the unloading.
- **Throughput** — the T3 step runs at ~**96 k steps/s/core**, *faster* than T2's ~62 k: the
  tyre-spring `F_z` resolves in one evaluation per RK stage where T2 runs three extra Picard
  evaluations for the algebraic coupling, more than paying for the heavier 24-DOF RHS. Its own
  regression tripwire (40 k) is set at ~half; T2's 30 k tripwire is untouched.
- **Zero-alloc** T3 step; the **T3 block schedule** is asserted a valid topological linearization
  (programmatically, not a hardcoded order); a `tier: t3` vehicle missing suspension data fails at
  assembly with a plain-language field list (never estimated, never a panic).

## References

- Milliken & Milliken, *Race Car Vehicle Dynamics* — load transfer, roll-centre / anti geometry, K&C.
- Guiggiani, *The Science of Vehicle Dynamics* — the 14-DOF ride/handling split, roll axis.
- Pacejka, *Tyre and Vehicle Dynamics* (2012), ch. 1 — tyre vertical stiffness.
- Kane & Levinson, *Dynamics: Theory and Applications* — the method; `sympy.physics.mechanics`.

Consulted open-source projects (approach only, re-authored from the literature + FIA/K&C conventions
per the clean-room rule): none for this model beyond the cited texts.
