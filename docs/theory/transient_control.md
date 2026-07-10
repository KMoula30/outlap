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
torque; the balance bar splits it front/rear; and on the driven axle the machine absorbs as much of
that axle's share as it can — bounded by its torque–speed envelope, the pack's charge-power ceiling,
and tyre grip. The friction brakes supply only the **deficit**:

```
τ_brake,w  =  τ_regen,w  +  τ_friction,w                (the axle's commanded brake torque, unchanged)
```

This model implements exactly that structure. The mechanical braking power at each driven wheel is
`P_w = (τ_brake,w / R_w) · v_x`; the machine takes a share `f_regen` of it and the calipers take the
rest, and the electrical yield is capped by the pack:

```
P_mech,regen = f_regen · Σ_driven P_w                   (mechanical power taken by the machine)
P_regen      = min( P_mech,regen · η ,  P_limit(SoC) )  (electrical power into the pack)
```

with `f_regen = max_regen_frac`, `η` the machine+inverter recovery efficiency (a documented constant
proxy — the mapped `.ptm` efficiency drives QSS energy accounting, and the wasm-clean block must never
touch a `.ptm` table), and `P_limit(SoC)` the pack's charge-power ceiling published on the slow clock.
When the pack ceiling binds, the machine cannot absorb its full share and the calipers implicitly take
more; the axle's total brake torque is unchanged either way, so the model stays self-consistent.

**The energy closes and the trajectory does not move (Locked Decision #11).** `WheelBrakeTorque` is the
axle's *total* brake torque — friction plus regen — which is precisely what the tyre responds to. The
machine substitutes for the calipers inside that total rather than adding to it, so the wheel
deceleration is untouched: a lap with regen on is **bit-identical** in trajectory to the same lap with
regen off, and only `regen_power_w` differs. That is asserted exactly (`assert_eq!` on the whole speed
trace, not to a tolerance) by `regen_is_energy_only_the_trajectory_is_identical_on_off`. This is not an
approximation artefact — it is what a correct series blend does, and it is why regen cannot perturb the
tier-parity gates. The energy ledger closes: `∫τ_brake·ω = ∫friction + ∫P_mech,regen`, and the pack
gains `η` of the second term (`regen_recovers_energy_and_never_creates_it` asserts the bound).

**What is *not* modelled**, in the order it will bite:

- `f_regen` is a **constant fraction**, not the machine's torque–speed envelope. A real system takes
  everything the machine can hold at that speed and lets the calipers fill the deficit, so `f_regen`
  is best read as a blend authority, not as a physical limit. `f_regen = 1` with the envelope as the
  cap is the faithful version, and needs the `.ptm` machine map at the actuation boundary.
- **No low-speed fade-out.** Real controllers ramp regen out below a few m/s, where torque control is
  poor and recovery is negligible; here it recovers to a standstill.
- **`WheelBrakeTorque` combines friction and regen.** A future brake-thermal model must subtract the
  regen share before heating the discs, or it will cook them on a regen-heavy lap.
- Rotor inertia reflected through the driveline is ignored (second order at these torques).

> Note: the schema field `brakes.regen_blend.max_regen_frac` is documented as a fraction of *total*
> brake torque, but the block applies it to the **driven axle's** brake torque. For a rear-drive car
> at high deceleration the rear axle carries well under half the total, so the two readings differ by
> a large factor. The driven-axle reading is the physical one (the machine can only ever act on its
> own axle); the schema wording is the part that is wrong.

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
| `f_regen` | `brakes.regen_blend.max_regen_frac` | vehicle data | max recovered fraction of driven-wheel braking power |
| `η` | — | constant proxy | machine + inverter recovery efficiency |
| `k_yaw` | `drivetrain.control.torque_vectoring.k_yaw` | vehicle data | yaw-rate feedback gain, N·m per rad/s |
| `M_max` | `drivetrain.control.torque_vectoring.max_yaw_moment_nm` | `+∞` (unset) | hard cap on `|ΔM_z|` (machine-envelope proxy) |
| `μ` | derived | vehicle peak grip | friction-ellipse radius coefficient |

The `max_yaw_moment_nm` cap is the only new schema field (additive ⇒ `vehicle/1.6`).

## 7. Verification

The allocator's four contract invariants are property-tested over randomised physically-consistent
wheel states (`outlap-vehicle/tests/control_props.rs`): friction-ellipse containment, the sign
convention (the realised moment never opposes or overshoots the demand), moment exactness, and
drive-capability (a wheel with no machine may only brake). The suite is mutation-checked — letting the
fill overshoot the feasible set, or reporting the demand instead of the realised moment, each fails it.

The shift FSM's determinism, torque cut, and gear swap are unit-tested in place; the regen energy
bound, the Decision #11 bit-identical trajectory invariant, and the slow-stack SoC closure are
block-level integration tests (`outlap-transient/tests/control.rs`).

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
  3rd ed., Springer, 2013 — regenerative braking energy recovery and the recuperation power limit.
- T. D. Gillespie, *Fundamentals of Vehicle Dynamics*, SAE, 1992 — brake balance and the axle brake
  force split.

No external open-source project was consulted for this layer; it is authored from the literature above.
