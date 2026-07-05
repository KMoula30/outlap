<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# QSS powertrain — topology graph in the traction limit

`outlap-qss`'s `t1::powertrain` module folds the **drivetrain topology graph** (§8.0) into the
quasi-steady-state trim: the powertrain torque envelope becomes the traction ceiling, the efficiency
and loss maps drive energy accounting, and the differential torque split enters the
[double-track trim](t1-trim.md) directly so open vs locked behaviour shapes the mid-corner per-wheel
forces. Powertrains are consumed only as neutral `.ptm` map files — the firewall (§1): outlap never
models a machine, inverter, or gearbox internally.

Implemented clean-room from published literature: Perantoni & Limebeer, *"Optimal control for a
Formula One car with variable parameters"*, Vehicle System Dynamics 52(5), 2014 (the reference F1
driveline); Guiggiani, *The Science of Vehicle Dynamics*, 2nd ed., 2018, ch. 3 (driveline torque
balance); Milliken & Milliken, *Race Car Vehicle Dynamics*, 1995, ch. 20 (differential torque-bias
models). No lap-time-optimiser or game-engine source is read for the implementation.

## The topology graph as data (§8.0)

A drivetrain is a directed graph: torque **sources** (`.ptm` maps — ICE, electric machines, or
lumped drive units) reach wheel **sinks** through an ordered **coupler** path (gearbox, fixed ratio,
differential). Any four-wheeled concept is a topology plus data — `drivetrain.units[]`, each
`{source, path: [couplers…], wheels: […]}` — and the assembler validates the graph (reachability,
no ratio conflicts, §8.0) at load time. The T1 reduction folds each unit's coupler path into a set
of **gears** using the T0 convention

```
ω_shaft = (ratio / r_wheel) · v            (shaft speed from vehicle speed)
F_wheel = (ratio · η_mech / r_wheel) · τ    (wheel force from source torque)
ratio   = Π(fixed ratios) · gear_ratio · final_drive
```

where `η_mech` is the constant (or mapped) **mechanical** gearbox efficiency and `r_wheel` the driven
tyre's unloaded radius. A `kind: drive_unit` map is already lumped at the wheel-side shaft, so the
topology applies no further ratio unless `meta.upstream_ratio_applied: false`.

### Traction ceiling

The largest wheel force a unit can put down at speed `v` is its best on-envelope gear,

```
F_max(v) = max over gears g on-envelope of  τ_peak(ω_g) · ratio_g · η_mech,g / r_wheel
```

with `τ_peak(ω)` the `.ptm` peak-torque envelope (the shared monotone cubic Hermite, Decision #30)
and gears whose shaft speed exceeds the envelope's top rev-limited out. Summed over the drive units
this is `max_tractive_force(v)`. It is the **powertrain traction ceiling** — the g-g-g-v envelope
(PR7) caps the acceleration boundary with it, while the tyre-grip limit is enforced by the trim
itself. The machine/thermal efficiency map does **not** reduce this force: the `.ptm` torque envelope
is already the mechanical output; the efficiency map governs the *energy drawn*, below.

The ceiling folds only the `drivetrain.units` — an F1's ERS/MGU-K lives in the separate `ers:` block
(§8.3) and its rule-based deployment (speed taper, per-lap energy budget) is **not** added to the
T1 traction ceiling in M3; that boost is surfaced in the loaded-model report as an assembly note and
folded in with the energy manager later.

## Wheel-torque conservation and static splits

A coupler is a linear torque gain: `Σ τ_wheel = τ_source · ratio · η`. Static allocation is data —
`control.split.front` (front/rear) and `control.split.left` (left/right) partition the source torque
across axles/sides — and every split's fractions sum to one. Rule-based control only in M3: the
yaw-moment torque-vectoring controller and QP allocation are M4/post-v1 (Locked Decisions #2, #11).

## The differential torque split (§8.2) inside the trim

The differential on the driven axle sets how an axle torque `τ` divides between its two wheels — and
that split is a genuine unknown of the trim, not a post-processing step. The trim's 9th unknown `w`
is the **driven-axle slip split** (`κ_left = s + w`, `κ_right = s − w`), closed by a 9th residual
that encodes the differential law:

| differential | trim residual (drive) | behaviour |
|---|---|---|
| open | `F_{x,left} − F_{x,right} = 0` | **equal torque**; `w` free, the two wheels take unequal slip |
| locked / solid | `w = 0` | **equal speed**; the wheels take equal slip, torque follows grip |
| LSD | `w = 0` (locks under load) | equal speed; preload/ramp bound the reported split |

Under braking the differential is inactive — the balance bar splits brake torque — so `w = 0`. An
**open** diff can carry no torque difference: the two wheels must produce equal longitudinal force,
so the inner (less-loaded) wheel slips more to match the outer wheel's torque, and its grip *caps*
the deliverable axle torque. When the demand exceeds that cap — maximum lateral and longitudinal at
once — the equal-torque root ceases to exist and the point is a clean traction boundary (the FWD
reference car shows exactly this at `|a_y| = 6, a_x = 3`). A **locked** or **solid** diff holds the
wheels at equal speed and lets torque follow grip, so the axle delivers the *sum* of the two wheels'
capability and the left/right force difference produces a yaw moment straight out of `R3`.

**LSD (a documented QSS simplification).** A preloaded limited-slip differential locks up at the
traction limit, so in the trim the LSD uses the **locked** (equal-speed) constraint; its preload and
ramp bound the reported torque split rather than unlocking a partial differential slip (a T2/M4
refinement). The standalone reference used for reporting and the property tests carries the full
range — `T_bias = preload + ramp·|τ_axle|`, the side-to-side torque difference clamped between the
open (`0`) and locked (grip-proportional) limits:

```
(τ_left, τ_right) = grip_proportional(τ, cap_left, cap_right), then clamp |τ_left − τ_right| ≤ T_bias
```

The schema's `ramp: [accel, decel]` is read as a **percent lock-up** (0–100 → fraction, values ≤ 1
taken as fractions directly) applied to the axle torque; the drive ramp is used under acceleration,
the brake ramp under braking.

![QSS powertrain: efficiency map, differential split, energy closure, ICE fuel](img/qss_powertrain.png)

*The committed synthetic maps (`python/tools/plot_qss_powertrain.py`): (a) the drive-unit efficiency
map η(speed, torque) from the importer-emitted parquet; (b) the differential torque split vs
left/right grip ratio — open holds 50/50, locked follows grip, the LSD sits between within its bias
band; (c) energy closure — source power and mechanical + loss coincide at the drive nodes; (d) the
ICE brake-thermal-efficiency map and the fuel-mass rate it implies under load.*

## Energy accounting and the efficiency/loss maps

The dense `efficiency`/`loss_w` tables in a `.ptm` sidecar (parquet, decoded at assembly time on the
native edge; the solver consumes the wasm-clean `GriddedTable`) drive energy accounting. At a source
shaft point `(n, τ)` with mechanical power `P_mech = τ·ω`:

```
drive (τ > 0):  P_source = P_mech / η        loss = P_mech · (1/η − 1)
regen (τ < 0):  P_source = P_mech · η         loss = |P_mech| · (1 − η)
ICE fuel rate:  ṁ_fuel = P_source / LHV       (η is brake thermal efficiency; LHV ≈ 43 MJ/kg)
```

so **energy closes**: `P_source = P_mech + loss`, exactly at the map's grid nodes when the importer
emits a consistent efficiency/loss pair, and to interpolation accuracy between them. Fuel mass is
accounted but held constant in M3 (no fuel slow state — that is M4/M5); the machine 2-node thermal
derating is PR5 and the battery Vdc–SoC coupling PR6.

### PDT round-trip gate (§10.5 / §13)

The importer (`outlap.importers.pdt_h5`) writes a long/tidy parquet — `speed_rpm, torque_nm,
efficiency, loss_w` — beside the `.ptm`. The round-trip gate loads that emitted `.ptm` plus its
parquet through the real `GriddedMapN` path and reproduces spot efficiencies from the source arrays
to **1e-6** (exact at the grid nodes the importer sampled). Unreachable cells beyond the torque
envelope carry `NaN` and are nearest-valid filled + flagged out-of-hull; the zero-torque spin column
is pinned to `η = 0`. CI runs on synthetic PDT-shaped fixtures only — real PDT data never enters the
repository (firewall, Decision #7).

## Property tests

Differential split limits (open ⇒ equal torque; locked/solid ⇒ grip-proportional/equal speed; LSD
between the two within its bias band); coupler torque conservation `Σ τ_out = τ_in·ratio·η`;
axle/side split fractions and diff outputs sum to one; energy closure `source = mechanical + loss` at
the drive nodes; ICE fuel-mass rate positive under load; the PDT round-trip reproducing spot
efficiencies to 1e-6 through `GriddedMapN`; the open diff splitting driven-wheel slip while
locked/LSD keeps it equal (in the live trim); a positive traction ceiling that falls with speed for a
geared engine; and a gearbox map efficiency assembling for T1 (retiring T0's
`UnsupportedEfficiencyMap` for the double-track tier).
