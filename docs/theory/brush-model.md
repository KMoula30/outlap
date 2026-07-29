<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# The brush tire model, with parabolic pressure

The `brush` module in `outlap-tire` implements the classic physical brush tire model with a
parabolic pressure profile. It is an alternative to the empirical
[MF6.1 force core](mf61-steady-state.md), derived from first principles. It is implemented
clean-room, from Chapter 3 of the Pacejka book (3rd ed., 2012), and from nothing else.

Use it for a tire that arrives as a `brush:` block (`tyr/1.1`) instead of a full Magic-Formula
coefficient set. A few physical parameters reproduce the force under pure slip and combined slip,
in closed form: two tread stiffnesses, a base friction, and the contact half-length.

## The model

The contact patch is a row of elastic bristles. Under slip, each bristle deflects. Where the local
elastic shear would exceed the friction bound `μ0·p(x)`, that bristle slides instead.

Give the pressure a **parabolic** distribution, `p(x) ∝ 1 − (x/a)²`, over the contact half-length
`a`. Integrating over the region that adheres and the region that slides then gives a closed-form
force.

## Symbols

| symbol | meaning |
|---|---|
| `κ`, `α` | Longitudinal slip ratio and slip angle (rad), under the ISO-W sign contract |
| `C_κ` | Longitudinal tread stiffness, N. It equals `∂F_x/∂κ` at the origin. |
| `C_α` | Lateral, or cornering, tread stiffness, N/rad. It equals `−∂F_y/∂α` at the origin. |
| `μ0` | Base sliding friction. At run time `mu_scale_*` scales it. |
| `a` | Contact half-length, m |
| `F_z` | Vertical load, N. At `F_z ≤ 0` every output is zero. |

## The equations, under combined slip

The theoretical slips use an ε-guarded `1 + κ`, so that a locked wheel stays finite:

```
σx = κ / (1 + κ),   σy = tan α / (1 + κ)
```

Next take the magnitude of the generalized force, weighted by the stiffnesses, and reduce it:

```
‖·‖ = √((C_κ σx)² + (C_α σy)²),   ψ = ‖·‖ / (3 μ0 F_z)
```

The magnitude of the force is the cubic brush law. It saturates at the friction bound:

```
|F| = 3 μ0 F_z · ψ(1 − ψ + ψ²/3)   for ψ < 1
|F| = μ0 F_z                       for ψ ≥ 1   (full sliding)
```

`ψ(1 − ψ + ψ²/3)` rises monotonically to `1/3` at `ψ = 1`. Therefore `|F| ≤ μ0 F_z` always, and the
model respects the friction circle by construction.

The force acts along the direction of the generalized force, `(+C_κ σx, −C_α σy)/‖·‖`. The
longitudinal sign flip is already carried by `κ`, because driving means `κ > 0` and therefore
`F_x > 0`. The lateral force opposes the slip, so `α > 0` gives `F_y < 0`. The slopes at the origin
are therefore `∂F_x/∂κ|₀ = +C_κ` and `∂F_y/∂α|₀ = −C_α`. The property tests assert these signs.

The self-aligning moment uses the closed-form pneumatic trail of the brush model:

```
t = (a/3) · (1 − ψ)³ / (1 − ψ + ψ²/3),   M_z = −t · F_y
```

The trail runs from `t(0) = a/3` at vanishing slip down to `0` at full sliding, where `ψ ≥ 1`. `M_z`
restores, because `F_y < 0` when `α > 0`. This is the same sign contract that MF6.1 uses. See
[`mf61-steady-state.md`](mf61-steady-state.md).

## What the model deliberately omits, documented and not silent

The brush tier models neither camber nor inflation pressure. It **accepts and ignores** `γ` and `p`.
The overturning moment and the rolling-resistance moment are `M_x = M_y ≡ 0`.

When outlap assembles a brush tire, it surfaces each of these as a note in the loaded-model report.
Nothing is silent.

The runtime friction multipliers `mu_scale_x` and `mu_scale_y` scale `μ0` on each axis. Both are
`1.0` until the M5 thermal grip window arrives. At `1.0` the friction of the model is isotropic.

At the T0 point-mass tier, the peak `μ` of a brush tire is simply `μ0`. It depends on neither load
nor pressure. A tire that carries the full MF6.1 force core uses that higher-fidelity model
instead. A partial force set never builds one.

## Numerical safety

The model is panic-free, and it returns finite values for every finite input.

`F_z ≤ 0` and zero slip short-circuit to zero. `1 + κ` is ε-guarded in a way that preserves sign,
which handles the locked-wheel pole at `κ = −1`. The denominator of the trail, `1 − ψ + ψ²/3`, is at
least `1/3` on `ψ ∈ [0, 1]`, so it needs no guard.

Evaluation is pure. It allocates nothing, which CI gates with dhat. It is generic over `f32` and
`f64`.

## Validation

The property tests pin seven things: that outputs stay finite over a hostile box of inputs; that an
airborne tire gives zero; that `|F| ≤ μ0 F_z`; that the slopes at the origin are `+C_κ` and `−C_α`;
that `M_z` restores; that the force saturates to exactly `μ0 F_z` at full sliding; and that
`mu_scale_*` scales the peak on each axis.

## References

- H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012 — Chapter 3
  "Theory of steady-state slip force and moment generation": the brush model with parabolic
  pressure, the cubic force law, and the pneumatic-trail expression.
