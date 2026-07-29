# limebeer_2014_f1: reference car #1 (Perantoni & Limebeer 2014)

This directory holds the complete published F1 parameter set of:

> G. Perantoni and D. J. N. Limebeer, *Optimal control for a Formula One car with variable
> parameters*, Vehicle System Dynamics **52**(5), 653–678, 2014 (Table 4 + §2). Open-access
> manuscript: Oxford University Research Archive, `uuid:ce1a7106-0a2c-41af-8449-41541220809f`.

The parameters are transcribed clean-room from the manuscript. HANDOFF §13 uses this car for the
flat-track cross-check at Catalunya. The paper publishes an optimal lap of **82.43 s** on a 2 m
grid, and 82.57 s in the mesh-asymptotic limit. Fig. 8 gives the speed trace, with a top speed near
88 m/s.

## Provenance of each parameter

| Field | Value | Source |
|---|---|---|
| `chassis.mass_kg` | 660 | Table 4: M |
| `chassis.cg` | [1.8, 0, 0.3] | Table 4: a, the distance from the CG to the front axle; symmetric; h |
| `chassis.inertia[2]` (Iz) | 450 kg·m² | Table 4 |
| `chassis.inertia[0..1]` (Ixx, Iyy) | 112.5, 425 | **NOT in PL2014.** These are plausible placeholders. The QSS tiers do not read them, because the steady-state trim uses no inertia. |
| `chassis.wheelbase_m` | 3.4 | Table 4: w |
| `chassis.track_m` | [1.46, 1.46] | Table 4: 2·wf and 2·wr, with wf = wr = 0.73 m |
| `aero.constant.cx_a_m2` | 1.35 | Cd·A = 0.9 × 1.5 (Table 4; eq. 33) |
| `aero.constant.cz_*_a_m2` | 1.98529 / 2.51471 | Cl·A = 3.0 × 1.5 = 4.5 (eq. 32), split by the center of pressure. That center sits at aA = 1.9 m from the front axle, so the front share is (w−aA)/w and the rear share is aA/w. |
| `suspension.*.roll_stiffness_share` | 0.5 | Table 4: D_roll (eq. 26) |
| `suspension.*.roll_center_height_m` | 0 | PL2014 gives no roll-center geometry. Zero heights make the lateral transfer in outlap purely elastic through D_roll, which is algebraically identical to eq. (26). |
| `suspension.*.ride_rate_n_per_m` | 200 000 | An **estimated placeholder.** No ride-height aero map is installed, so the aero-platform equilibrium never runs. That equilibrium is the only consumer of this value. |
| `tires` | tyr/f1.tyr.yaml | Table 3, transcribed to MF6.1. See `data/tires/limebeer_2014_f1/README.md`. Appendix A gives the same tire front and rear. |
| `drivetrain` | a 560 kW envelope at the wheel shaft, RWD, open differential | The manuscript **does not state the power.** 560 kW comes from the companion work, the doctoral thesis of Perantoni: "peak engine power of 560 kW … top speed 85.4 m/s". With the drag of Table 4 it reproduces the top speed near 88 m/s in Fig. 8, because P = ½ρ·CdA·u³ gives 88.4 m/s. Table 4 gives kd = 10.47 N·m·s/rad, which is near zero on the scale from open to locked. §2.5.2 states that kd→0 means open, so the differential is open. |
| `brakes.balance_bar` | 0.6 | **Estimated.** PL2014 leaves the brake ratio for each axle implicit, with equal caliper pressures on each axle (eq. 34). Braking is tire-limited either way. |
| `conditions.yaml` | 21.0 °C, 1013.25 hPa | Through the ideal-gas conversion in outlap, these reproduce ρ = 1.2 kg/m³ from Table 4. |

`tyr/f1.tyr.yaml` is a copy of `data/tires/limebeer_2014_f1/f1.tyr.yaml`, because a vehicle
resolves its references inside its own directory. The README for the tire carries the provenance of
each coefficient.

## What was consulted, under the clean-room policy

- `fastest-lap` (MIT, github.com/juanmanzanero/fastest-lap) was read as a **numerical cross-check
  and nothing else**. Its `limebeer-2014-f1.xml` transcribes Tables 3 and 4 the same way this
  directory does. Its powertrain is that project's own choice, not the choice of PL2014: it uses
  735.5 kW plus a 120 kW boost, and 5000 N·m of brake torque. Its published lap times are therefore
  not comparable gates. No code was taken.
