<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Reference vehicles

Each directory here holds one complete `vehicle.yaml`, the vehicle member of the input quartet. The
shipped examples use them. From M7 the hero demonstration will use them too. Locked Decision #1
names all four reference vehicles.

These are **synthetic reference data**. Nobody measured them. Their magnitudes are plausible, and
each source file labels them clearly (Decision #15).

Point an `FsLoader` at a vehicle directory and it loads. Each referenced `.ptm` file and `.tyr`
file sits in a sibling directory, `ptm/` or `tyr/`.

## `f1_2026/`: F1 2026 hybrid

The drivetrain runs ICE and MGU-K into a gearbox, then an LSD, then the rear axle.

- `vehicle.yaml` holds the FIA 2026 regulation figures (Section C, Issue 19) for the `policy:`
  energy rulebook. It also holds a SYNTHETIC `aero.constant` block (CzA 5.5 m², CxA 1.25 m²),
  calibrated against telemetry from the 2026 Barcelona GP. The T0 point-mass tier reads that block.
- `aero/f1_2026.parquet` is the SYNTHETIC aero map over ride height, yaw, and DRS. Tiers from T1 up
  read it. `python/tools/gen_f1_aero.py` writes it. At its reference node it reproduces the
  constant coefficients.
- `ptm/ice_v6.ptm.yaml` and `ptm/mgu_k.ptm.yaml`, with their efficiency and loss sidecars in
  `ptm/tables/*.parquet`, are the neutral powertrain maps. They give peak-torque envelopes for
  traction and η and loss surfaces for energy accounting. The MGU-K is *semi-virtual*: outlap
  authored its envelope and imported its loss shape.
- `emotor/mgu_k.emotor.yaml` is the machine-thermal LPTN for the MGU-K. It produces the deploy
  derate over a stint.
- `battery/f1_es.yaml` and `f1_es.tables.parquet` hold the SYNTHETIC 2026 energy store, sized to
  the regulation. `python/tools/gen_f1_es_pack.py` writes them. At load time outlap cross-checks
  its `soc_window` against `policy:`.
- `tyr/slick.tyr.yaml` is an MF6.1 slick with the thermal ring block and the wear and cliff block.

This car began as a copy of the schema test fixtures. The two may diverge, and that is acceptable.
The fixtures serve the schema tests. These files serve the demonstrations.

## `gt_hybrid/`: reference GT hybrid

The drivetrain runs ICE and MGU-K on a shared crank into a 6-speed gearbox, then an LSD, then the
rear axle.

This is the **reference car for Option coverage** (D-M6-12). It uses the same D-M6-13 drivetrain
graph as `f1_2026`, on a 1250 kg GT-class car. Its energy policy is *privately regulated* at 2.0 MJ
and 120 kW. It does not use the FIA rulebook. Everything here is **SYNTHETIC**. The magnitudes are
plausible, no data was measured, and the car models no real GT class.

Six things this car deliberately does **not** declare. Each absence is the point, so keep it:

- no `fuel:` block, so this ICE car never changes mass;
- no `policy.override_mode`, so there is no overtake button;
- no `policy.recovery.recharge_phases`, so there is no ramp-down of power demand;
- no `policy.max_engine_speed_rpm`, so there is no regulatory rev limit;
- no `thermal:` `.emotor` on the MGU-K, so there is no machine-thermal derate;
- no `limits.regen_derate_vs_temp` on the pack, so charge acceptance ignores temperature.

Each absence exercises an absent-`Option` path on a car that *runs*, rather than only in a schema
fixture. The car loads clean, with no warnings and without `allow_degraded`. The substitutions
above, and the omitted suspension geometry, appear as `estimated` entries in the loaded-model
report and as run notes on the lap.

| Field | Value | Source |
|---|---|---|
| `chassis.mass_kg` | 1250 | **synthetic.** A GT-class hybrid magnitude, between a GT3 at about 1300 kg and a prototype at about 1030 kg. |
| `chassis.cg` | [1.55, 0, 0.34] | **synthetic.** a = 0.56·w, which gives a static split of 44 front and 56 rear, as a mid-engined car has. The CG height is low, as a GT car has. |
| `chassis.inertia` | 320, 1600, 1750 kg·m² | **synthetic** GT magnitudes. The QSS trim does not read them. |
| `chassis.wheelbase_m` and `track_m` | 2.75 and [1.68, 1.64] | **synthetic** GT magnitudes. |
| `aero.map` | `aero/none.parquet` | Deliberately absent. The constant block below carries every tier. This is the idiom in this repository for a car with no aero map. |
| `aero.constant.cx_a_m2` | 1.10 | **synthetic.** Cd 0.58 × A 1.90 m², for a closed GT body. |
| `aero.constant.cz_*_a_m2` | 0.99 / 1.36 | **synthetic.** CzA 2.35 m² gives 6.9 kN at 250 km/h, which is about 0.57 times the weight of the car. `f1_2026` reaches about 2.1 times its weight. L/D is about 2.1, and the balance is 42 % front. |
| `suspension.*.ride_rate_n_per_m` | 130 000 / 140 000 | **synthetic.** A stiff GT platform. |
| `suspension.*.roll_stiffness_share` | 0.56 / 0.44 | **synthetic.** A front-biased GT balance. |
| `suspension.*.static_ride_height_m`, `anti_dive`, `anti_squat` | omitted | The documented estimator in the load pipeline fills these. They are **surfaced in the loaded-model report**. |
| `tires` | `tyr/slick.tyr.yaml` | A copy of the synthetic slick from `f1_2026`, with identical numbers and a gt provenance header. Its peak μ of about 1.68 was calibrated against F1 data, so it is **optimistic for a GT slick**. |
| `drivetrain` | ICE and MGU-K, both with `output: crank`, then 6-speed and LSD couplers | The shared-crank graph of D-M6-13. One engaged gear sets the operating point of BOTH sources. |
| `final_drive` | 6.4, sized to the F1 V6 map that this car references, not to its class | A GT-typical 3.4 assumes a redline near 7 400 rpm. Against a 15 000 rpm engine it put the top three gears beyond any reachable speed. At 6.4 the car uses all six ratios, at about 112, 146, 182, 224, 265, and 324 km/h. A GT-appropriate engine map would bring the ratio back down. |
| `ptm/ice_v6.ptm.yaml` | a copy of the `f1_2026` map | A **synthetic** V6 of about 500 kW, with identical numbers and a gt provenance header. It is the combustion side of the 80/20 GT-hybrid split. |
| `ptm/mgu_k.ptm.yaml` | a copy of the `f1_2026` map | A **synthetic** 223 N·m machine, with identical numbers and a gt provenance header. This is the hardware ceiling. The 120 kW cap in the policy is what actually binds. |
| `policy` | a 2.0 MJ window, 120 kW of deploy tapering to zero at 320 kph, and 3.0 MJ of harvest for each lap | A **synthetic** rulebook for a private series. It is explicitly **not** an FIA rulebook. |
| `batteries.gt_es` | `battery/gt_es.yaml` and `gt_es.tables.parquet` | A **synthetic** 96S1P pack of about 3.64 MJ. Its `soc_window` of [0.30, 0.85] gives exactly the 2.0 MJ policy window. `python/tools/gen_f1_es_pack.py` writes it. |
| `brakes` | a 0.6 balance bar, 52 and 40 kJ/K discs, and `max_regen_frac` 0.45 | A **synthetic** GT disc and blend scale. |

This car reuses the powertrain maps and the tire from `f1_2026` verbatim. It is therefore a
**reference for topology and for a rulebook. It is not a performance model** of any real GT
category. Treat its lap times as magnitudes only.

Its twin in the schema fixtures is
`crates/outlap-schema/tests/fixtures/gt_hybrid/vehicle.yaml`. That twin has a deliberately
different shape, and it may diverge further. To regenerate the pack for both cars, run
`python python/tools/gen_f1_es_pack.py`. Pass `--no-fixtures` to write the shipped cars only.

## `tesla_model3_rwd/`: Tesla Model 3 RWD, a study of the HV (800 V-class) variant

The drivetrain runs one drive unit into an open differential, then the rear axle.

- `vehicle.yaml` holds chassis, mass, and aero values that are plausible for a Model 3. Its
  `README.md` gives the provenance of each parameter and marks which are spec-sheet values and
  which are documented estimates. Its road-car aero is constant, which is the degenerate case with
  no map.
- `ptm/du_{small,medium,large}.ptm.yaml` hold three SYNTHETIC drive-unit sizings, stacked on Vdc
  (`ptm/2.0`, `kind: electric`). They are the sensitivity axis of notebook 07.
  `python/tools/gen_model3_powertrain.py` writes them.
- `battery/pack_800v.battery.yaml` is a SYNTHETIC Thevenin pack in the 800 V class. The Vdc–SoC
  coupling is live on this car.
- `emotor/rear_du.emotor.yaml` is a hand-authored lumped machine-thermal network. Its estimates are
  flagged.
- `tyr/road.tyr.yaml` is the published Pacejka (2006) 205/60R15 road tire, used as a documented
  proxy.
- `local/` is git-ignored. Real PDT imports land there. Nothing in it is ever committed, because of
  the firewall.

## The validation cars

Two more directories exist to be *validated against published numbers*, not to demonstrate
anything. Their parameters are therefore transcribed, not invented.

- `limebeer_2014_f1/` holds the complete published F1 parameter set of Perantoni and Limebeer
  (2014). Its own `README.md` cites the paper for each parameter.
- `bmw320i/` holds the chassis, inertia, and tire parameters of CommonRoad "vehicle 2", a BMW 320i
  (Althoff et al., TUM, BSD-3). It is the handling-validation car for the T3 14-DOF tier. Its
  lumped-KC suspension values are SYNTHETIC placeholders, and the file header flags them, as always.
