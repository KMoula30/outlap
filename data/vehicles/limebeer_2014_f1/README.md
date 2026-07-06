# limebeer_2014_f1 — reference car #1 (Perantoni & Limebeer 2014)

The complete published F1 parameter set of:

> G. Perantoni and D. J. N. Limebeer, *Optimal control for a Formula One car with variable
> parameters*, Vehicle System Dynamics **52**(5), 653–678, 2014 (Table 4 + §2). Open-access
> manuscript: Oxford University Research Archive, `uuid:ce1a7106-0a2c-41af-8449-41541220809f`.

Transcribed clean-room from the manuscript; used for the HANDOFF §13 flat-track Catalunya
cross-check (the paper's published optimal lap: **82.43 s** on a 2 m grid, 82.57 s
mesh-asymptotic; Fig. 8 speed trace, top speed ≈ 88 m/s).

## Per-parameter provenance

| Field | Value | Source |
|---|---|---|
| `chassis.mass_kg` | 660 | Table 4: M |
| `chassis.cg` | [1.8, 0, 0.3] | Table 4: a (CG→front axle), symmetric, h |
| `chassis.inertia[2]` (Iz) | 450 kg·m² | Table 4 |
| `chassis.inertia[0..1]` (Ixx, Iyy) | 112.5, 425 | **NOT in PL2014** — plausible placeholders, unused by the QSS tiers (steady-state trim uses no inertia) |
| `chassis.wheelbase_m` | 3.4 | Table 4: w |
| `chassis.track_m` | [1.46, 1.46] | Table 4: 2·wf, 2·wr (wf = wr = 0.73 m) |
| `aero.constant.cx_a_m2` | 1.35 | Cd·A = 0.9 × 1.5 (Table 4; eq. 33) |
| `aero.constant.cz_*_a_m2` | 1.98529 / 2.51471 | Cl·A = 3.0 × 1.5 = 4.5 (eq. 32) split by the centre of pressure aA = 1.9 m from the front axle: front (w−aA)/w, rear aA/w |
| `suspension.*.roll_stiffness_share` | 0.5 | Table 4: D_roll (eq. 26) |
| `suspension.*.roll_center_height_m` | 0 | PL2014 has no roll-centre geometry: zero heights make outlap's lateral transfer purely elastic through D_roll — algebraically identical to eq. (26) |
| `suspension.*.ride_rate_n_per_m` | 200 000 | **estimated placeholder** — no ride-height aero map is installed, so the aero-platform equilibrium (the only consumer) never runs |
| `tires` | tyr/f1.tyr.yaml | Table 3 transcribed to MF6.1 — see `data/tires/limebeer_2014_f1/README.md` (same tyre front/rear, per Appendix A) |
| `drivetrain` | 560 kW wheel-shaft envelope, RWD, open diff | power is **not stated in the manuscript**: 560 kW is the companion-work value (Perantoni's doctoral thesis, "peak engine power of 560 kW … top speed 85.4 m/s") and reproduces Fig. 8's ≈88 m/s top speed with Table 4 drag (P = ½ρ·CdA·u³ ⇒ 88.4 m/s); kd = 10.47 N·m·s/rad (Table 4) is near-zero on the open↔locked scale (kd→0 = open, §2.5.2) ⇒ open |
| `brakes.balance_bar` | 0.6 | **estimated** — PL2014 leaves the per-axle brake ratio implicit (equal caliper pressures per axle, eq. 34); braking is tyre-limited either way |
| `conditions.yaml` | 21.0 °C / 1013.25 hPa | reproduces the paper's ρ = 1.2 kg/m³ (Table 4) through outlap's ideal-gas conversion |

`tyr/f1.tyr.yaml` is a copy of `data/tires/limebeer_2014_f1/f1.tyr.yaml` (vehicles resolve
references inside their own directory); the tyre README carries the coefficient provenance.

## Consulted (per the clean-room policy)

- `fastest-lap` (MIT, github.com/juanmanzanero/fastest-lap): read as a **numerical cross-check
  only** — its `limebeer-2014-f1.xml` transcribes Tables 3/4 identically to this directory. Its
  powertrain (735.5 kW + 120 kW boost, 5000 N·m brake torque) is that project's own choice, *not*
  PL2014's, so its published lap times are not comparable gates. No code was taken.
