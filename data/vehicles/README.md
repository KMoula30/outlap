<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Reference vehicles

Self-contained reference `vehicle.yaml` quartet members used by the shipped examples and (from M7)
the hero demo (Locked Decision #1: all four reference vehicles). These are **synthetic reference
data**, not measured — plausible magnitudes clearly labelled at their source (Decision #15).

Each vehicle directory is loadable with an `FsLoader` rooted at it (referenced `.ptm`/`.tyr` files
live in `ptm/` and `tyr/` siblings).

## `f1_2026/` — F1 2026 hybrid (ICE + MGU-K → gearbox → LSD → rear axle)

- `vehicle.yaml` — the FIA 2026 regulation figures (Section C Issue 19) for the `policy:` energy
  rulebook, plus a SYNTHETIC `aero.constant` block (CzA 5.5 m², CxA 1.25 m²) calibrated against 2026
  Barcelona GP telemetry and consumed by the T0 point-mass tier.
- `aero/f1_2026.parquet` — the SYNTHETIC ride-height/yaw/DRS aero map consumed from T1 up
  (`python/tools/gen_f1_aero.py`); it reproduces the constant coefficients at its reference node.
- `ptm/ice_v6.ptm.yaml`, `ptm/mgu_k.ptm.yaml` (+ their `ptm/tables/*.parquet` efficiency/loss
  sidecars) — neutral powertrain maps: peak-torque envelopes for traction, η/loss surfaces for
  energy accounting. The MGU-K is *semi-virtual* (authored envelope, imported loss shape).
- `emotor/mgu_k.emotor.yaml` — the MGU-K machine-thermal LPTN (the deploy derate over a stint).
- `battery/f1_es.yaml` + `f1_es.tables.parquet` — the SYNTHETIC, regulation-sized 2026 energy store
  (`python/tools/gen_f1_es_pack.py`); its `soc_window` is cross-checked against `policy:` at load.
- `tyr/slick.tyr.yaml` — MF6.1 slick with the thermal ring + wear/cliff blocks.

Originally copied from the schema test fixtures; the two may diverge intentionally (fixtures serve
schema tests, these serve demos).

## `gt_hybrid/` — Reference GT hybrid (ICE + MGU-K on a shared crank → 6-speed → LSD → rear axle)

The **Option-coverage reference car** (D-M6-12): the same D-M6-13 drivetrain graph as `f1_2026` on a
1250 kg GT-class car under a *privately-regulated* 2.0 MJ / 120 kW energy policy instead of the FIA
rulebook. Everything here is **SYNTHETIC** — plausible magnitudes, no measured data, and no real GT
class is being modelled.

What it deliberately does **not** declare — the absence is the point, so keep it:

- no `fuel:` block (an ICE car whose mass never changes),
- no `policy.override_mode` (no overtake button),
- no `policy.recovery.recharge_phases` (no power-demand ramp-down),
- no `policy.max_engine_speed_rpm` (no regulatory rev limit),
- no `thermal:` `.emotor` on the MGU-K (no machine-thermal derate),
- no `limits.regen_derate_vs_temp` on the pack (temperature-independent charge acceptance).

Each of those keeps an absent-`Option` path exercised on a *runnable* car rather than only in a
schema fixture. The car loads warning-clean without `allow_degraded`; the substitutions above (plus
the omitted suspension geometry) surface as `estimated` entries in the loaded-model report and as
run notes on the lap.

| Field | Value | Source |
|---|---|---|
| `chassis.mass_kg` | 1250 | **synthetic** — GT-class hybrid magnitude (between a ≈1300 kg GT3 and a ≈1030 kg prototype) |
| `chassis.cg` | [1.55, 0, 0.34] | **synthetic** — a = 0.56·w ⇒ 44/56 front/rear static split (mid-engined), low GT CG height |
| `chassis.inertia` | 320, 1600, 1750 kg·m² | **synthetic** GT magnitudes (unused by the QSS trim) |
| `chassis.wheelbase_m` / `track_m` | 2.75 / [1.68, 1.64] | **synthetic** GT magnitudes |
| `aero.map` | `aero/none.parquet` | deliberately absent — the constant block below carries every tier (the repo idiom for a non-mapped car) |
| `aero.constant.cx_a_m2` | 1.10 | **synthetic** — Cd 0.58 × A 1.90 m² (closed GT body) |
| `aero.constant.cz_*_a_m2` | 0.99 / 1.36 | **synthetic** — CzA 2.35 m² ⇒ 6.9 kN at 250 km/h ≈ 0.57× car weight (vs ≈2.1× for `f1_2026`), L/D ≈ 2.1, balance 42 % front |
| `suspension.*.ride_rate_n_per_m` | 130 000 / 140 000 | **synthetic** — stiff GT platform |
| `suspension.*.roll_stiffness_share` | 0.56 / 0.44 | **synthetic** — front-biased GT balance |
| `suspension.*.static_ride_height_m`, `anti_dive`/`anti_squat` | omitted | filled by the load pipeline's documented estimator, **surfaced in the loaded-model report** |
| `tires` | `tyr/slick.tyr.yaml` | copy of the `f1_2026` synthetic slick (identical numbers, gt provenance header) — its peak μ ≈ 1.68 was calibrated against F1 data and is **optimistic for a GT slick** |
| `drivetrain` | ICE + MGU-K both `output: crank`; 6-speed + LSD couplers | the D-M6-13 shared-crank graph — one engaged gear sets BOTH sources' operating point |
| `ptm/ice_v6.ptm.yaml` | copy of the `f1_2026` map | **synthetic** ≈500 kW-class V6 (identical numbers, gt provenance header): the 80/20 GT-hybrid split's combustion side |
| `ptm/mgu_k.ptm.yaml` | copy of the `f1_2026` map | **synthetic** 223 N·m machine (identical numbers, gt provenance header); the hardware ceiling — the 120 kW policy cap is what actually binds |
| `policy` | 2.0 MJ window, 120 kW deploy tapering to zero at 320 kph, 3.0 MJ/lap harvest | **synthetic** private-series rulebook — explicitly **not** an FIA one |
| `batteries.gt_es` | `battery/gt_es.yaml` + `gt_es.tables.parquet` | **synthetic** 96S1P pack, ≈3.64 MJ total, `soc_window` [0.30, 0.85] ⇒ exactly the 2.0 MJ policy window; written by `python/tools/gen_f1_es_pack.py` |
| `brakes` | 0.6 balance bar, 52/40 kJ/K discs, `max_regen_frac` 0.45 | **synthetic** GT disc/blend scale |

Because the powertrain maps and the tyre are reused verbatim from `f1_2026`, this car is a
**topology and rulebook reference, not a performance model** of any real GT category — treat its lap
times as magnitudes only. Its schema-fixture twin
(`crates/outlap-schema/tests/fixtures/gt_hybrid/vehicle.yaml`) keeps a deliberately different shape
and may diverge; regenerate the pack for both with `python python/tools/gen_f1_es_pack.py`
(`--no-fixtures` writes the shipped cars only).

## `tesla_model3_rwd/` — Tesla Model 3 RWD, HV (800 V-class) variant study (1 DU → open diff → rear axle)

- `vehicle.yaml` — Model-3-plausible chassis/mass/aero (spec-sheet values vs documented estimates:
  see the per-parameter provenance in its `README.md`); constant road-car aero (the degenerate
  non-mapped case).
- `ptm/du_{small,medium,large}.ptm.yaml` — three SYNTHETIC Vdc-stacked (`ptm/2.0`, `kind: electric`)
  drive-unit sizings (the notebook 07 sensitivity axis), written by
  `python/tools/gen_model3_powertrain.py`.
- `battery/pack_800v.battery.yaml` — SYNTHETIC 800 V-class Thevenin pack (the Vdc–SoC coupling is
  live on this car).
- `emotor/rear_du.emotor.yaml` — hand-authored lumped machine-thermal network (estimates flagged).
- `tyr/road.tyr.yaml` — the published Pacejka (2006) 205/60R15 road tyre (documented proxy).
- `local/` (git-ignored) — where the real PDT imports land; never committed (firewall).

## The validation cars

Two more directories exist to be *validated against published numbers* rather than to demo, so their
parameters are transcribed rather than invented:

- `limebeer_2014_f1/` — the complete published F1 parameter set of Perantoni & Limebeer (2014),
  with the paper citations per parameter in its own `README.md`.
- `bmw320i/` — the CommonRoad "vehicle 2" (BMW 320i) chassis/inertia/tyre parameters (Althoff et
  al., TUM, BSD-3), the T3 14-DOF handling-validation car. Its lumped-KC suspension values are
  SYNTHETIC placeholders — flagged in the file header, as always.
