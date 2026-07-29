<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# T0: the point-mass lap solver

The T0 tier finds a lap time. It solves a velocity profile on the 3D road ribbon, with a forward
pass and a backward pass.

The model is quasi-steady-state. It integrates no ODE. It computes a speed at each station that
curvature limits, then sweeps twice: once limited by traction, and once limited by braking. Both
sweeps use a friction ellipse at constant μ.

The implementation is clean-room, from the published formulations listed at the end. It never comes
from the LGPL TUM source.

## Symbols

| symbol | meaning |
|---|---|
| `s` | arc length along the line (m) |
| `v` | speed (m/s), `u = v²` |
| `m` | vehicle mass (kg), `g` gravity |
| `κ_h`, `κ_v` | curvature of the line in plan view and in the vertical plane (1/m) |
| `θ_g`, `θ_b` | grade and banking (rad) |
| `μ_x`, `μ_y` | longitudinal and lateral friction. These are the peaks of the MF6.1 pure-slip curves, averaged over the axles. |
| `γ` | grip scale, `grip_scale(s)` |
| `q_x`, `q_z` | ½ρ·CxA and ½ρ·CzA, the aero terms. ρ comes from `conditions.air` through the ideal-gas law. |

## Geometry of the road plane (the 3D ribbon, Decision #13)

The solver resolves the curvature of the line into the banked road plane:

```
κ_l = κ_h·cosθ_g·cosθ_b + κ_v·sinθ_b      (lateral, in the road plane)
κ_n = κ_v·cosθ_b − κ_h·cosθ_g·sinθ_b      (road-normal)
```

## The point-mass equations

```
N(s,v)   = m·(g·cosθ_b·cosθ_g + κ_n·v²) + q_z·v²      normal load (crest unloads, dip loads)
F_y(s,v) = m·(κ_l·v² + g·sinθ_b·cosθ_g)               lateral tyre demand (banking of the
                                                       right sign reduces |F_y|)
m·v̇     = F_t − q_x·v² − m·g·sinθ_g                   longitudinal
```

They are subject to the friction ellipse `(F_t/(μ_x γ N))² + (F_y/(μ_y γ N))² ≤ 1`.

## Speed limited by curvature, in closed form

Set `F_t = 0`. Both sides of `|F_y| ≤ μ_y γ N` are then affine in `u = v²`. Write
`a = m·κ_l`, `b = m·g·sinθ_b·cosθ_g`, `c = μ_y γ (m·κ_n + q_z)`, and
`d = μ_y γ m·g·cosθ_b·cosθ_g`.

The constraint becomes `|a·u + b| ≤ c·u + d`. Its two sign branches give the largest feasible `u`.
Two bounds cap that value: the flight condition `N ≥ 0`, and a top-speed bound `v_cap`. No Newton
iteration is needed.

For a flat circle this reduces to `v = √(μ_y·g·R)`. On a banked turn it becomes
`v² = gR(μ_y cosφ + sinφ)/(cosφ − μ_y sinφ)`.

## The forward and backward passes

* **Forward, limited by traction.**
  `v²_{i+1} = min(v_lim², v_i² + 2Δs·(F_t − q_x v² − m g sinθ_g)/m)`, with
  `F_t = min(F_trac(v), grip remaining after F_y)`. `F_trac(v)` takes the best gear across the
  peak-torque envelopes in the `.ptm` maps of the drive units, plus the ERS after its power cap.
* **Backward, limited by braking.** At T0, friction is the only limit:
  `v²_i = min(v²_i, v²_{i+1} + 2Δs·a_dec)`, with
  `a_dec = (grip remaining + q_x v² + m g sinθ_g)/m`.
* **Closing the lap.** Seed at the global minimum of `v_lim`, which is a fixed point limited by
  lateral grip. Then sweep forward and backward around the loop until the profile converges.

Lap time is `Σ 2Δs/(v_i + v_{i+1})`, summed in a fixed order.

A centerline imported from the real world, through OSM and a DEM, carries noise in its positions.
The interpolating spline amplifies that noise into spurious spikes of curvature. The solver
therefore applies a light centered moving average to `κ_l` and `κ_n`, which rejects it. The
principled fix for a fair lap is the minimum-curvature racing line.

## Scope at T0 and M1

The model is a point mass, with 3D corrections to the normal load.

μ is constant. Assembly derives it once, from the peaks of the MF6.1 pure-slip curves. It uses
`peak_mu_x` and `peak_mu_y` at `Fz = FNOMIN`, at cold inflation pressure and `γ = 0`, averaged over
the two axles. It does not use the raw `PD*·LMU*` factors. This folds in the shape factors for load
and for pressure.

ERS enters as a power cap only. T0 does not enforce the energy budget for each lap, the override
mode, or thermal derating.

Braking is limited by friction. There is no brake-thermal model and no regeneration blend.

The loaded-model notes record all of these.

The magnitude of the lap time is a sanity check against published lap records. It is **not** the
Limebeer parity gate of ≤1%. That gate belongs to the full QSS tier, in M3.

## References

- A. Heilmeier, A. Wischnewski, L. Hermansdorfer, J. Betz, M. Lienkamp, B. Lohmann,
  *Minimum curvature trajectory planning and control for an autonomous race car*,
  Vehicle System Dynamics 58(10), 2020 — the `calc_vel_profile` forward/backward formulation.
- G. Perantoni & D. J. N. Limebeer, *Optimal Control of a Formula One Car on a Three-Dimensional
  Track*, Parts 1–2, ASME J. Dyn. Sys. Meas. Control 137, 2015 — 3D track modelling.
- S. Lovato & M. Massaro, *A three-dimensional free-trajectory quasi-steady-state optimal-control
  method for minimum-lap-time*, Vehicle System Dynamics 60(5), 2022 — g-g-g polar envelope.
