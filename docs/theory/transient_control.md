<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# The transient rule-based control layer — shift FSM, regen blend, torque vectoring

This page documents the transient tier's **rule-based control layer** (HANDOFF §8.0/§8.2/§8.4): the
gear-shift finite state machine ([`outlap_transient::control::Shifter`]), the regen/friction brake
blend and slow-state battery stack, and the torque-vectoring yaw-moment allocator
([`outlap_vehicle::control::allocate_yaw_moment`], [`TorqueVectoring`]). It sits between the ideal
driver (which produces steer/throttle/brake demands — see [driver.md](driver.md)) and the tyre/chassis
blocks that turn wheel torques into forces.

Every model here is a clean-room implementation from the cited literature (§7); no other project's
source was consulted or copied.

The layer is deliberately **rule-based in v1** (Locked Decision #2): the allocator's interface — a
per-wheel feasibility set plus a fill rule — is shaped so that a quadratic-program allocator can
replace the body post-v1 without touching a single caller.

## 1. Where the discrete and slow state live

The continuous SoA fast buffer holds only the differentiable chassis/relaxation/controller states, and
its layout is frozen. The control layer's state is neither continuous nor fast, so it lives elsewhere:

| state | clock | home |
|-------|-------|------|
| engaged gear, shift timers | step boundary | the [`EventQueue`] (time-ordered, drained per step) |
| pack state of charge, temperature | decimated **slow** clock | the [`SlowStack`] trait object (Decision #6) |
| drive-torque scale, regen ceiling, realised `ΔM_z` | every step | interned bus channels |

The orchestrator owns one of each, and publishes their outputs onto the bus once per step, **frozen
across the Runge–Kutta sweep** — exactly the treatment the tyre relaxation and load-transfer coupling
already receive. Freezing matters: a discrete quantity that jumped between RK stages (a gear swapping
mid-sweep) would make the stages inconsistent and silently destroy the integrator's order.

The slow stack is touched once every `slow_decimation` fast steps, so its single dynamic dispatch is
off the hot path (the hot-loop discipline forbids dispatch inside a timestep, not outside it).

## 2. The gear-shift FSM: torque cut → ratio swap → clutch ramp

An up/down-shift is not instantaneous — it costs a **torque interruption**. Naunheimer et al. describe
the phase sequence of an automated/sequential transmission shift; the model reproduces its three
observable phases and charges the vehicle's own `Gearbox.shift_time_s` for the whole thing:

```
elapsed < f_cut·T_shift          →  torque_scale = 0                  (torque cut)
elapsed = f_cut·T_shift          →  Engage(to)   [EventQueue]         (ratio swap)
f_cut·T_shift ≤ elapsed < T_shift →  torque_scale = (elapsed − f_cut·T_shift)/(T_shift − f_cut·T_shift)
elapsed ≥ T_shift                →  Complete     [EventQueue]         (clutch fully re-engaged)
```

`torque_scale ∈ [0,1]` multiplies the powertrain's available wheel drive force each step, so the car
genuinely coasts through the cut and recovers drive over the clutch ramp. `f_cut = 0.35`
(`SHIFT_CUT_FRACTION`) is a **modelling constant**, not a measured value: it places the majority of the
shift in the re-engagement ramp, matching the qualitative shape of a seamless-shift race gearbox. It is
surfaced as estimated, and `shift_time_s = 0` recovers the pre-PR6 instantaneous ideal shift exactly.

**The engaged gear indexes no force.** The powertrain's wheel-force ceiling remains the *best-gear*
traction envelope (the QSS tier already picks the gear at each speed), so the FSM's entire physical
effect on the car is the torque interruption above. Gear-indexed traction curves — which would let a
mis-timed shift leave the car in the wrong ratio out of a corner — are a post-v1 change.

**Threshold crossing.** Up-shift thresholds are the traction crossover speeds supplied by the assembly
pipeline. When the speed crosses one during a step, the crossing *time* is recovered by a single linear
back-interpolation across that step (`back_interpolate`) — no root-finding (§11.2) — and the two
discrete transitions are scheduled on the event queue at `t_cross + f_cut·T_shift` and
`t_cross + T_shift`. Because the schedule is a pure function of the step boundary, the shift timeline
is bit-reproducible (asserted by `shift_timeline_is_deterministic`).

**Hysteresis.** A down-shift fires only once the speed falls below `0.93 ×` (`DOWNSHIFT_HYSTERESIS`)
the up-shift threshold of the gear below. Without it, a car cruising exactly at a shift point would
chatter between gears every step — a classic relay-with-noise limit cycle that the hysteresis band
removes.

## 3. Regen blending: series (blended) braking

Production EVs and full hybrids use **series regenerative braking**. The pedal demands a *total* brake
torque; the balance bar splits it front/rear; on each driven axle that axle's machine absorbs as much of
its share as it can; and the friction brakes supply only the **deficit**:

```
τ_brake,axle  =  τ_regen,axle  +  τ_friction,axle        (the commanded axle torque, unchanged)
```

Because the machine substitutes for the calipers *inside* the commanded torque rather than adding to
it, the axle total — which is what the tyre responds to — never moves. The car decelerates identically
whether the energy went into the pack or into the discs, and only the recovered energy differs. That is
the Decision #11 invariant, and it is asserted exactly (`assert_eq!` on the whole speed trace, not to a
tolerance) by `regen_is_energy_only_the_trajectory_is_identical_on_off`. It is not an approximation we
pay for: it is what a correct series blend does, and it is why regen cannot perturb the tier-parity
gates.

### 3.1 Each machine brakes its own axle

A machine can only ever apply torque to the wheels it drives. A rear-drive EV therefore regenerates on
the rear axle and not at all on the front; a dual-motor car runs two independent regen actuators that
happen to share one battery. The transient block models the two axles separately, and the QSS assembly
attributes each drive unit's braking capability to the axle(s) in its driven set
(`T1Vehicle::max_regen_force_by_axle`), splitting by driven-wheel count when a single motor spans both
axles through a centre differential.

An **internal-combustion engine recovers nothing**. It has no negative quadrant to command: its overrun
braking is parasitic pumping/friction drag, not recoverable energy. Its regen envelope is identically
zero and the loaded-model report says so.

### 3.2 The three ceilings on the machine

For each axle `a` with a machine, the regen braking force it takes is

```
F_regen,a = min( authority_a · F_brake,a ,  F_env,a(v_x) · fade(v_x) )
```

1. **Available regen torque** — `F_env,a(v_x)` is the machine's braking envelope
   (`ptm/1.2` `max_regen_torque_nm_vs_speed`, through the best gear, at the wheel), sampled into the
   shared monotone cubic at assembly so the hot loop never touches a `.ptm` map. When a map declares no
   regen envelope, the machine is assumed **symmetric** with its drive envelope — the usual first-order
   truth when inverter current sets the limit — and that assumption is surfaced as *estimated*, never
   applied silently (#41).
2. **Blend authority** — `authority_a = max_regen_frac`, a policy cap on the machine's share of *its own
   axle's* commanded brake torque. `1` means "take everything the envelope and the pack allow".
3. **Low-speed fade** — `fade(v_x) = clamp(v_x / 2 m/s, 0, 1)`. Real controllers hand braking back to
   the calipers at a walking pace: torque control degrades, the recoverable energy is negligible, and
   the machine must release the wheel before the car stops.

Driveline efficiency is deliberately *not* applied to `F_env`. Under drive a loss shrinks the force
reaching the wheel; under regen the power flows the other way, so a loss would *add* braking at the
wheel while shrinking what the machine recovers. Charging `η` once, against the recovered power, keeps
the ledger honest and understates rather than overstates the machine's braking authority.

### 3.3 One pack, shared: charge acceptance vs SoC *and* temperature

The two machines draw on one battery. Their combined electrical demand `Σ_a η_a · F_regen,a · v_x` is
capped by the pack's **charge-acceptance ceiling**; when the cap binds, both machines are scaled back by
the same factor — preserving the front/rear split — and the calipers absorb the remainder on each axle.
The axle totals are untouched, so the trajectory is unmoved either way.

The ceiling itself (`Pack::regen_power_limit_w`) is the minimum of three limits a real BMS enforces:

```
P_accept = min( peak_regen(SoC) · derate(T) ,  V_max·(V_max − emf)/R0(SoC,T) )     (0 above the SoC window)
```

- **Design/SoC ceiling** — the declared `peak_regen_power_w_vs_soc(SoC)` curve.
- **Kinetic (cold) derate** — `derate(T)` from `battery/1.1` `regen_derate_vs_temp`. **A cold lithium-ion
  cell cannot accept a fast charge.** Below roughly 10 °C the anode's intercalation kinetics slow until
  plating metallic lithium becomes the competing reaction, so a BMS cuts charge current hard — typically
  to zero below 0 °C — to avoid irreversible capacity loss and dendrite growth. This is a *kinetic*
  limit. It does **not** fall out of the ohmic grid and must be declared; absent, the pack is assumed to
  accept its full ceiling at any temperature and that is marked estimated.
- **Voltage (CV) ceiling** — charging drives the terminal voltage *above* the open-circuit EMF by
  `I·R0`, and it may not exceed `ns · cell_v_max`. With `emf = OCV(SoC,T) − V_RC`, the largest charge
  current is `(V_max − emf)/R0`, giving the bound above. This is the constant-voltage taper: it vanishes
  as the pack fills (`emf → V_max`) and tightens when cold (`R0` rises), for free.

The two terms are **not** redundant, and it is worth being precise about why. Take the committed
`synth_pack` fixture (220S1P, `cell_v_max` 4.2 V, so `V_max` = 924 V), at 25 °C and at two SoC grid
nodes:

| SoC | design curve | voltage ceiling | binds |
|-----|--------------|-----------------|-------|
| 0.40 | 180 kW | 750 kW | design |
| 0.80 | 90 kW  | 629 kW | design |

The voltage ceiling never binds on this pack, because its open-circuit voltage tops out near 3.64 V per
cell — a long way under the 4.2 V ceiling. Nor does it carry much temperature signal: this fixture's
`R0` is flat in temperature, so the ceiling moves only through `OCV(T)`, spanning 742 → 755 kW across
0 → 45 °C at SoC 0.4. That is a 1.8 % swing. **The ohmic term alone cannot reproduce cold-charge
refusal** — not here, and not on a real pack, where at mid-SoC it sits several times above the design
curve even below freezing. The kinetic derate is what makes a cold pack refuse charge.

The voltage ceiling earns its place at the *other* end: it bites as the open-circuit voltage climbs
toward `ns · cell_v_max`, where the headroom `V_max − emf` collapses and with it the admissible current.
That is the constant-voltage taper every charger exhibits, and it tightens further when `R0` rises with
cold. Together the two terms cover both regimes; neither covers both alone.

### 3.4 What is still not modelled

- **No ABS/grip cap on the regen share.** The commanded axle torque already respects the driver's
  demand, and regen only substitutes within it, so the tyre never sees more than it would have — but a
  wheel about to lock is not handed back to the friction brakes the way a real ABS event would.
- **`WheelBrakeTorque` combines friction and regen.** A future brake-thermal model must subtract the
  per-axle regen share (published on `ctrl.regen_torque_{front,rear}_nm`) before heating the discs, or
  it will cook them on a regen-heavy lap.
- **Constant recovery efficiency.** `η` is a documented constant proxy; the mapped `.ptm` efficiency
  drives QSS energy accounting, and the wasm-clean block may never touch a `.ptm` table.
- Rotor inertia reflected through the driveline is ignored (second order at these torques).

## 4. Torque vectoring: a yaw moment produced by the tyres, not injected

The controller tracks the corner's reference yaw rate `r_target = v_x · κ_ref` with proportional
feedback, capped by an optional machine-envelope proxy:

```
ΔM_z,demand = clamp( k_yaw · (r_target − r),  ±M_max )
```

This is the standard **direct yaw-moment control** law (Rajamani §8; van Zanten's ESP uses the same
yaw-rate error signal, realised through individual-wheel braking; Sawase & Sano realise it through
driving/braking force distribution, which is the case here).

What matters is the second half: the demanded moment is **not** added to the chassis as a lumped
couple. It is allocated across the four wheels as longitudinal force deltas `Δf_x,i`, each clamped
inside that wheel's **friction ellipse** — the combined-slip limit of Pacejka's tyre model, the
`F_x`–`F_y` ellipse of Milliken & Milliken's friction circle:

```
f_x,max,i = √( (μ·F_z,i)² − F_y,i² )                     (longitudinal headroom at the current F_y)
s_i       = −sign(ΔM_z,demand · y_i)                     (the delta sign that adds toward the demand)
h_i       = headroom in direction s_i, 0 if s_i>0 and wheel i has no machine   (drive-incapable ⇒ brake only)
M_feasible = Σ_i |y_i| · h_i
ΔM_z       = sign(demand) · min(|demand|, M_feasible)
Δf_x,i     = s_i · min(|demand|/M_feasible, 1) · h_i     (proportional fill of the feasible set)
```

Under ISO 8855 a longitudinal force at lateral arm `y_i` (+left) contributes `−y_i·Δf_x,i` to the yaw
moment, so the deltas realise **exactly** `ΔM_z` — an identity, not an approximation
(`reported_moment_equals_the_moment_the_deltas_produce`). The realised moment saturates at
`M_feasible`, which is what the tyres can actually deliver: with all four wheels at their lateral
limit, `f_x,max = 0`, no moment is feasible, and the block reports `0` rather than the demand.

The deltas are applied as extra wheel drive/brake **torque** (`Δf_x,i · R_i`). The wheel spin responds,
the slip evolves, and the tyre produces the extra longitudinal force — so the yaw moment emerges through
the contact patch over the tyre's relaxation lag, with all the phase lag that implies. Disabled, the
block is a no-op that only zeroes its telemetry channel, so a car that does not enable TV is
byte-identical to the pre-PR6 lap.

`μ` is the vehicle's representative peak grip (the ellipse radius coefficient), not a per-wheel
instantaneous friction estimate; a QP allocator with the real per-wheel combined-slip surface is the
post-v1 replacement (Decision #2).

## 5. The slow-state stack

`SlowStack` is the interface the orchestrator advances on the decimated slow clock: it Coulomb-counts
`regen_power_w` into the pack over the slow interval, self-discharges under any base draw, and
publishes back the charge-power ceiling `P_limit(SoC)` that caps §3 and the pack SoC/temperature for
telemetry. It is *received* as a boxed artifact — the concrete implementation wraps the QSS `Pack`
primitive at the Python boundary — so the wasm-clean transient crate never depends on the QSS
trim/envelope machinery, mirroring how the line table and traction envelope are handed in (§11.1).

## 6. Parameters and defaults

| symbol | field | default | meaning |
|--------|-------|---------|---------|
| `T_shift` | `drivetrain.gearbox.shift_time_s` | vehicle data | total shift duration (`0` ⇒ ideal instantaneous shift) |
| `f_cut` | `SHIFT_CUT_FRACTION` | 0.35 | fraction of the shift spent in the torque cut |
| — | `DOWNSHIFT_HYSTERESIS` | 0.93 | down-shift band below the up-shift threshold |
| `authority` | `brakes.regen_blend.max_regen_frac` | vehicle data | max machine share of *its own axle's* brake torque |
| `F_env` | `.ptm` `limits.max_regen_torque_nm_vs_speed` | drive envelope | machine braking envelope (symmetric when absent) |
| `derate(T)` | `battery` `limits.regen_derate_vs_temp` | `1` (no derate) | cold-charge acceptance factor, `0..1` |
| `V_max` | `battery` `limits.cell_v_max` × `ns` | vehicle data | pack charge-voltage ceiling (the CV taper) |
| `fade` | `REGEN_FADE_SPEED_MPS` | 2.0 m/s | speed below which regen fades linearly to zero |
| `η` | — | constant proxy | machine + inverter recovery efficiency |
| `k_yaw` | `drivetrain.control.torque_vectoring.k_yaw` | vehicle data | yaw-rate feedback gain, N·m per rad/s |
| `M_max` | `drivetrain.control.torque_vectoring.max_yaw_moment_nm` | `+∞` (unset) | hard cap on `|ΔM_z|` (machine-envelope proxy) |
| `μ` | derived | vehicle peak grip | friction-ellipse radius coefficient |

Schema growth, all additive: `max_yaw_moment_nm` (`vehicle/1.6`), `max_regen_torque_nm_vs_speed` (`ptm/1.2`), and `regen_derate_vs_temp` (`battery/1.1`).

## 7. Verification

The allocator's four contract invariants are property-tested over randomised physically-consistent
wheel states (`outlap-vehicle/tests/control_props.rs`): friction-ellipse containment, the sign
convention (the realised moment never opposes or overshoots the demand), moment exactness, and
drive-capability (a wheel with no machine may only brake). The suite is mutation-checked — letting the
fill overshoot the feasible set, or reporting the demand instead of the realised moment, each fails it.

The shift FSM's determinism, torque cut, and gear swap are unit-tested in place; the regen energy
bound, the Decision #11 bit-identical trajectory invariant, and the slow-stack SoC closure are
block-level integration tests (`outlap-transient/tests/control.rs`).

The regen blend is pinned at three levels, each mutation-checked:

- **Pack** (`outlap-qss/tests/battery.rs`) — a cold pack accepts less than a warm one and nothing below
  0 °C; the derate scales the design curve; an absent derate leaves acceptance temperature-independent
  (`battery/1.0` compatibility); a nearly-full pack tapers on the voltage ceiling; that ceiling itself
  tightens as `R0` rises when cold; acceptance is never negative anywhere on the `(SoC, T)` grid.
- **Machine** (`outlap-qss/tests/t1_powertrain.rs`) — a rear-drive EV regens only at the rear; each
  machine of a dual-motor car regens its own axle; an ICE recovers nothing and says so; an absent
  envelope is symmetric *and surfaced*; a declared envelope is used verbatim.
- **Blend** (`outlap-vehicle/src/control.rs`) — the machine takes its share and the calipers take the
  rest; a machine never brakes the other axle; a pack that cannot accept charge hands braking back to
  the calipers entirely; the machine envelope, the blend authority, and the low-speed fade each cap the
  share; a shared pack ceiling scales both axles proportionally; regen never exceeds the commanded
  braking. Letting a machine reach across axles, or forgetting to scale its braking torque when the pack
  ceiling binds, each fails this suite.

## References

- H. B. Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012 — combined slip and
  the longitudinal/lateral friction ellipse that bounds each wheel's force delta.
- W. F. Milliken & D. L. Milliken, *Race Car Vehicle Dynamics*, SAE, 1995 — the friction circle/ellipse
  construction and the tyre force budget.
- R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012 — direct yaw-moment control, the
  reference yaw rate `r = v·κ`, and yaw-rate error feedback.
- A. T. van Zanten, "Bosch ESP Systems: 5 Years of Experience," SAE Technical Paper 2000-01-1633, 2000
  — yaw-rate tracking realised through individual-wheel longitudinal force.
- Y. Sawase & Y. Sano, "Application of active yaw control to vehicle dynamics by utilizing
  driving/braking force," *JSAE Review* 20(3), 1999, pp. 289–295 — direct yaw moment generated by
  distributing driving/braking force across wheels (torque vectoring).
- H. Naunheimer, B. Bertsche, J. Ryborz & W. Novak, *Automotive Transmissions: Fundamentals,
  Selection, Design and Application*, 2nd ed., Springer, 2011 — the shift phase sequence (torque
  interruption, ratio change, clutch re-engagement).
- L. Guzzella & A. Sciarretta, *Vehicle Propulsion Systems: Introduction to Modeling and Optimization*,
  3rd ed., Springer, 2013 — series vs parallel regenerative braking and the recuperation power limit.
- G. L. Plett, *Battery Management Systems, Volume 2: Equivalent-Circuit Methods*, Artech House, 2015 —
  the voltage-limited power-capability bound `P = V_max·(V_max − emf)/R0` used for the CV taper, and the
  Thevenin equivalent-circuit parameterisation the pack is built on.
- J. Jaguemont, L. Boulon & Y. Dubé, "A comprehensive review of lithium-ion batteries used in hybrid and
  electric vehicles at cold temperatures," *Applied Energy* 164, 2016, pp. 99–114 — the collapse of
  charge acceptance at low temperature.
- M. Petzl & M. A. Danzer, "Nondestructive detection, characterization, and quantification of lithium
  plating in commercial lithium-ion batteries," *Journal of Power Sources* 254, 2014, pp. 80–87 — why a
  BMS must cut charge current when cold (the kinetic derate).
- T. D. Gillespie, *Fundamentals of Vehicle Dynamics*, SAE, 1992 — brake balance and the axle brake
  force split.

No external open-source project was consulted for this layer; it is authored from the literature above.
