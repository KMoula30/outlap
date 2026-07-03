<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# T0 — point-mass lap solver

The T0 tier finds a lap time from a forward/backward velocity-profile solve on the 3D road ribbon.
It is a quasi-steady-state model: no ODE integration, just a curvature-limited speed per station
followed by traction- and braking-limited sweeps over a constant-μ friction ellipse. Implemented
clean-room from the published formulations below (never from the LGPL TUM source).

## Symbols

| symbol | meaning |
|---|---|
| `s` | arc length along the line (m) |
| `v` | speed (m/s), `u = v²` |
| `m` | vehicle mass (kg), `g` gravity |
| `κ_h`, `κ_v` | plan-view and vertical curvature of the line (1/m) |
| `θ_g`, `θ_b` | grade and banking (rad) |
| `μ_x`, `μ_y` | longitudinal / lateral friction (MF6.1 `PDX1·LMUX`, `PDY1·LMUY`) |
| `γ` | grip scale `grip_scale(s)` |
| `q_x`, `q_z` | ½ρ·CxA, ½ρ·CzA (aero, ρ from `conditions.air` by the ideal-gas law) |

## Road-plane geometry (3D ribbon, Decision #13)

The line's curvature is resolved into the banked road plane:

```
κ_l = κ_h·cosθ_g·cosθ_b + κ_v·sinθ_b      (lateral, in the road plane)
κ_n = κ_v·cosθ_b − κ_h·cosθ_g·sinθ_b      (road-normal)
```

## Point-mass equations

```
N(s,v)   = m·(g·cosθ_b·cosθ_g + κ_n·v²) + q_z·v²      normal load (crest unloads, dip loads)
F_y(s,v) = m·(κ_l·v² + g·sinθ_b·cosθ_g)               lateral tyre demand (banking of the
                                                       right sign reduces |F_y|)
m·v̇     = F_t − q_x·v² − m·g·sinθ_g                   longitudinal
```

subject to the friction ellipse `(F_t/(μ_x γ N))² + (F_y/(μ_y γ N))² ≤ 1`.

## Curvature-limited speed (closed form)

With `F_t = 0`, both sides of `|F_y| ≤ μ_y γ N` are affine in `u = v²`. Writing
`a = m·κ_l`, `b = m·g·sinθ_b·cosθ_g`, `c = μ_y γ (m·κ_n + q_z)`, `d = μ_y γ m·g·cosθ_b·cosθ_g`,
the constraint `|a·u + b| ≤ c·u + d` gives the largest feasible `u` from the two sign branches,
capped by the flight condition `N ≥ 0` and a top-speed bound `v_cap`. No Newton iteration.

For a flat circle this reduces to `v = √(μ_y·g·R)`; on a banked turn to
`v² = gR(μ_y cosφ + sinφ)/(cosφ − μ_y sinφ)`.

## Forward / backward passes

* **Forward** (traction): `v²_{i+1} = min(v_lim², v_i² + 2Δs·(F_t − q_x v² − m g sinθ_g)/m)`,
  `F_t = min(F_trac(v), grip remaining after F_y)`. `F_trac(v)` is the best gear across the drive
  units' `.ptm` peak-torque envelopes plus the power-capped ERS.
* **Backward** (braking, friction-limited only at T0): `v²_i = min(v²_i, v²_{i+1} + 2Δs·a_dec)`
  with `a_dec = (grip remaining + q_x v² + m g sinθ_g)/m`.
* **Closed lap**: seed at the global minimum of `v_lim` (a lateral-limited fixed point) and sweep
  forward then backward around the loop to convergence.

Lap time `= Σ 2Δs/(v_i + v_{i+1})` (fixed-order sum).

Imported real-world centerlines (OSM + DEM) carry position noise that the interpolating spline
amplifies into spurious curvature spikes; the solver applies a light centred moving average to
`κ_l`/`κ_n` to reject it. The principled fix for a fair lap is the min-curvature racing line.

## Scope at T0 / M1

Point-mass with 3D normal-load corrections; μ constant from the tyre peak factors; ERS as a power
cap (per-lap energy budgets, override mode, and thermal derating are not enforced); braking is
friction-limited (no brake-thermal or regen blend). These are recorded in the loaded-model notes.
The magnitude is a sanity check against published lap records, **not** the ≤1% Limebeer parity gate
(that is the full-QSS tier, M3).

## References

- A. Heilmeier, A. Wischnewski, L. Hermansdorfer, J. Betz, M. Lienkamp, B. Lohmann,
  *Minimum curvature trajectory planning and control for an autonomous race car*,
  Vehicle System Dynamics 58(10), 2020 — the `calc_vel_profile` forward/backward formulation.
- G. Perantoni & D. J. N. Limebeer, *Optimal Control of a Formula One Car on a Three-Dimensional
  Track*, Parts 1–2, ASME J. Dyn. Sys. Meas. Control 137, 2015 — 3D track modelling.
- S. Lovato & M. Massaro, *A three-dimensional free-trajectory quasi-steady-state optimal-control
  method for minimum-lap-time*, Vehicle System Dynamics 60(5), 2022 — g-g-g polar envelope.
