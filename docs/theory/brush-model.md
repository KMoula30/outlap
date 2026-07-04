<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Brush tire model (parabolic pressure)

`outlap-tire`'s `brush` module implements the classic physical brush tire model with a parabolic
pressure profile — a first-principles alternative to the empirical [MF6.1 force core](mf61-steady-state.md).
Implemented clean-room from Pacejka's book (3rd ed., 2012, Chapter 3) only. It is offered for tyres
supplied as a `brush:` block (`tyr/1.1`) instead of a full Magic-Formula coefficient set: a
handful of physical parameters — two tread stiffnesses, a base friction, and the contact
half-length — reproduce the pure- and combined-slip force with a closed form.

## The model

The contact patch is a row of elastic bristles. Under slip each bristle deflects; where the local
elastic shear would exceed the friction bound `μ0·p(x)` the bristle slides. With a **parabolic**
pressure distribution `p(x) ∝ 1 − (x/a)²` over the contact half-length `a`, integrating the
adhesion and sliding regions gives a closed-form force.

## Symbols

| symbol | meaning |
|---|---|
| `κ`, `α` | longitudinal slip ratio and slip angle (rad), ISO-W sign contract |
| `C_κ` | longitudinal tread stiffness, N (`∂F_x/∂κ` at the origin) |
| `C_α` | lateral (cornering) tread stiffness, N/rad (`−∂F_y/∂α` at the origin) |
| `μ0` | base sliding friction (scaled at runtime by `mu_scale_*`) |
| `a` | contact half-length, m |
| `F_z` | vertical load, N (`≤ 0` ⇒ all-zero output) |

## Equations (combined slip)

Theoretical slips, with an ε-guarded `1 + κ` so a locked wheel stays finite:

```
σx = κ / (1 + κ),   σy = tan α / (1 + κ)
```

Stiffness-weighted generalised-force magnitude and its reduced form:

```
‖·‖ = √((C_κ σx)² + (C_α σy)²),   ψ = ‖·‖ / (3 μ0 F_z)
```

The force magnitude is the cubic brush law, saturating at the friction bound:

```
|F| = 3 μ0 F_z · ψ(1 − ψ + ψ²/3)   for ψ < 1
|F| = μ0 F_z                       for ψ ≥ 1   (full sliding)
```

`ψ(1 − ψ + ψ²/3)` rises monotonically to `1/3` at `ψ = 1`, so `|F| ≤ μ0 F_z` always — the friction
circle is respected by construction. The force acts along the generalised-force direction
`(+C_κ σx, −C_α σy)/‖·‖`: the longitudinal sign flip is already carried by `κ` (driving `κ > 0` ⇒
`F_x > 0`), while the lateral force opposes the slip (`α > 0` ⇒ `F_y < 0`). The origin slopes are
therefore `∂F_x/∂κ|₀ = +C_κ` and `∂F_y/∂α|₀ = −C_α` — the sign pins the property tests assert.

The self-aligning moment uses the closed-form brush pneumatic trail

```
t = (a/3) · (1 − ψ)³ / (1 − ψ + ψ²/3),   M_z = −t · F_y
```

which runs from `t(0) = a/3` at vanishing slip down to `0` at full sliding (`ψ ≥ 1`). `M_z` is
restoring because `F_y < 0` for `α > 0` — the same sign contract as MF6.1 (see
[`mf61-steady-state.md`](mf61-steady-state.md)).

## Deliberate omissions (documented, not silent)

The brush tier models neither camber nor inflation pressure: `γ` and `p` are **accepted and
ignored**, and the overturning/rolling-resistance moments are `M_x = M_y ≡ 0`. When a brush tyre is
assembled these are surfaced as loaded-model notes (nothing silent). The runtime friction
multipliers `mu_scale_x`/`mu_scale_y` scale `μ0` per axis (both `1.0` until the M5 thermal grip
window); at `1.0` the model is isotropic in friction. At the T0 point-mass tier a brush tyre's
peak `μ` is simply `μ0` (load- and pressure-independent), while a tyre that carries the full MF6.1
force core uses that higher-fidelity model instead — a partial force set never constructs one.

## Numerical safety

Panic-free and finite for all finite inputs: `F_z ≤ 0` and zero slip short-circuit to zero;
`1 + κ` is ε-guarded sign-preservingly (the `κ = −1` locked-wheel pole); the trail denominator
`1 − ψ + ψ²/3 ≥ 1/3` on `ψ ∈ [0, 1]` needs no guard. Evaluation is pure, allocation-free
(dhat-gated in CI), and generic over `f32`/`f64`.

## Validation

Property tests pin: finiteness over a hostile input box, the airborne zero, the friction bound
`|F| ≤ μ0 F_z`, the origin slopes `+C_κ`/`−C_α`, the restoring `M_z` sign, exact saturation to
`μ0 F_z` at full sliding, and `mu_scale_*` scaling the peak per axis.

## References

- H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012 — Chapter 3
  "Theory of steady-state slip force and moment generation": the brush model with parabolic
  pressure, the cubic force law, and the pneumatic-trail expression.
