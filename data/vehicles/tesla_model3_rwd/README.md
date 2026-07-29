<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# tesla_model3_rwd: Tesla Model 3 RWD, a study of the HV variant

This car takes the identity of a production **Tesla Model 3 RWD** and re-imagines it as an **HV
variant in the 800 V class** (M3 user decision #6). Its chassis, mass, and aero are plausible for a
Model 3. Its powertrain is a drive unit and pack stack in the 800 V class, which makes the Vdc–SoC
coupling of §8.4 live on a road car. This car is the EV half of the capstone in notebook 07, and it
is the demonstration of sizing sensitivity.

Everything committed here is **synthetic or estimated**. The drive-unit maps and the pack tables
are invented smooth surfaces. A committed script writes them
(`python/tools/gen_model3_powertrain.py`). Nobody measured them, and nothing here derives from any
PDT export. This is the firewall, and M3 user decision #7. The real PDT imports stay **local and
untracked**. See "How to reproduce this locally" below.

## Provenance of each parameter

| Field | Value | Source |
|---|---|---|
| `chassis.mass_kg` | 1765 | The published curb mass of the current Model 3 RWD. The manufacturer EU spec gives about 1765 kg. |
| `chassis.wheelbase_m` | 2.875 | Published spec. |
| `chassis.track_m` | [1.58, 1.58] | Published spec: front and rear track are both 1580 mm. |
| `chassis.cg[0]`, the distance from the CG to the front axle | 1.524 m | **Estimated** from a static distribution near 47 front and 53 rear, because the motor is at the rear and the pack is rear-biased. This gives a = 0.53·w. |
| `chassis.cg[2]`, the CG height | 0.45 m | **Estimated.** The pack mounts in the floor, which keeps the CG low for a sedan. |
| `chassis.inertia` (Ixx, Iyy, Izz) | 560, 2800, 3200 kg·m² | **Estimated** magnitudes for a mid-size sedan. The QSS trim does not read them, except through published sensitivities, because the steady-state trim uses no inertia. |
| `aero.constant.cx_a_m2` | 0.51 | Cd·A = 0.23 × 2.22 m². Both the drag coefficient and the frontal area are published. |
| `aero.constant.cz_*_a_m2` | 0.0 / 0.0 | **Estimated.** This is a road body with no lift and no map over ride height or yaw. It is the degenerate passenger-car case of §7.4. |
| `suspension.*.ride_rate_n_per_m` | 38 000 / 45 000 | **Estimated** from k = m_corner·(2πf)², at ride frequencies near 1.52 Hz at the front and 1.56 Hz at the rear, with corner masses split 47/53, then rounded. |
| `suspension.*.roll_stiffness_share` | 0.58 / 0.42 | **Estimated.** A road-car balance biased toward the front bar, which is safe in understeer. |
| `suspension.*.roll_center_height_m` | 0.06 / 0.12 | **Estimated.** Typical geometry for a strut front and a multi-link rear. |
| `suspension.*.anti_dive` and `anti_squat` | omitted | The documented estimator in the load pipeline fills these. They are **surfaced in the loaded-model report**. |
| `tires` | `tyr/road.tyr.yaml` | The published Pacejka (2006) 205/60R15 book tire. It is a **documented proxy** for the real 235/45R18 (user decision #6), because no public MF set exists for the OE tire. This file is a copy of `data/tires/pacejka_2006_205_60r15/car.tyr.yaml`, and that README holds its provenance. |
| `drivetrain` | one DU, then an open differential, then RL and RR | The `ev_1du_rwd` reference topology. It is the production Model 3 RWD layout. |
| `drivetrain.units[0].source` | `ptm/du_medium.ptm.yaml` | **Synthetic**, as described below. The medium sizing gives about 203 kW, which is close to the 200 kW of a production Model 3 RWD. |
| `drivetrain.units[0].thermal` | `emotor/rear_du.emotor.yaml` | An **estimated**, hand-authored lumped LPTN from the menu in §9.5. Its capacities come from the mass fractions of the components in the 82 kg drive unit. Its conductances sit at magnitudes typical of a liquid-cooled traction machine. |
| `battery` | `battery/pack_800v.battery.yaml` | A **synthetic** pack in the 800 V class, as described below. |
| `brakes.balance_bar` | 0.62 | **Estimated.** Braking is tire-limited either way. |
| `brakes.disc.*` | 26/20 kJ/K, 0.07/0.05 m² | **Estimated** at road-car disc scale. |
| `brakes.regen_blend.max_regen_frac` | 0.6 | An **estimated** ceiling for the one-pedal blend. |

Estimated values stay visible on purpose. The vehicle loads **with no warnings**. The loaded-model
report notes every estimate; call `outlap.vehicle_report(...)` to see it. The `notes` attribute on
the lap records the simplifications made at run time. Nothing is silent (Decision #41).

## The committed synthetic powertrain: three sizings

`python/tools/gen_model3_powertrain.py` writes three drive-unit maps stacked on Vdc (`ptm/2.0`,
`kind: electric`), and it writes the pack. Notebook 07 sweeps the three as its axis of sizing
sensitivity:

| Variant | Peak torque (output shaft) | ≈Peak power | File |
|---|---|---|---|
| small | 1365 N·m | 100 kW | `ptm/du_small.ptm.yaml` |
| **medium (default)** | 2765 N·m | 203 kW | `ptm/du_medium.ptm.yaml` |
| large | 3381 N·m | 248 kW | `ptm/du_large.ptm.yaml` |

The three peak-torque scales mirror the local drive-unit sweep of the author. This makes the
committed story and the untracked real-data twin directly comparable. The surfaces themselves are
invented, and the generator documents them.

The pack is synthetic: 220S/1P, 64.064 kWh, in the 800 V class. Its open-circuit voltage runs from
about 634 V to 810 V across the SoC grid. Under load at low SoC, its terminal voltage sags below
the 730–850 V Vdc grid of the drive units. That exercises the documented linear extrapolation below
the grid in the Vdc–SoC coupling.

To swap a sizing without editing any file, use a what-if override (Decision #35):

```python
solve_lap_dataset(vehicle_dir, line, tier="t1",
                  overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"})
```

## How to reproduce this locally with real PDT data, which is NEVER committed

The real drive-unit maps and the real 704 V pack import into
`data/vehicles/tesla_model3_rwd/local/`. Git ignores that directory. Put the source files in
`~/pdt_reference/` and run these commands from `python/`:

```sh
mkdir -p ../data/vehicles/tesla_model3_rwd/local
uv run python -m outlap.importers.pdt_h5 driveunit \
  ~/pdt_reference/DriveUnit_9.3GR_1365NM_1938RPM_a2d6c_outlap.h5 \
  -o ../data/vehicles/tesla_model3_rwd/local/du_1365.ptm.yaml
uv run python -m outlap.importers.pdt_h5 driveunit \
  ~/pdt_reference/DriveUnit_9.3GR_2765NM_1938RPM_761a6_outlap.h5 \
  -o ../data/vehicles/tesla_model3_rwd/local/du_2765.ptm.yaml
uv run python -m outlap.importers.pdt_h5 driveunit \
  ~/pdt_reference/DriveUnit_9.3GR_3381NM_1938RPM_ce8cb_outlap.h5 \
  -o ../data/vehicles/tesla_model3_rwd/local/du_3381.ptm.yaml
uv run python -m outlap.importers.pdt_h5 batterypack \
  ~/pdt_reference/BatteryPack_220S_1P_64064Wh_704V_e884f_outlap.h5 \
  -o ../data/vehicles/tesla_model3_rwd/local/pack_704v.battery.yaml
```

By default the driveunit importer emits the **full Vdc stack**, from 730 V to 850 V. That is
exactly what the coupling needs. To point the car at a real import, use the same what-if overrides,
for example
`overrides={"drivetrain.units.0.source": "local/du_3381.ptm.yaml", "battery.params":
"local/pack_704v.battery.yaml"}`. Paths resolve inside this vehicle directory. The untracked
notebook `notebooks/07_qss_t1_local.ipynb` runs the whole sequence.

Never commit the `.h5` sources. Never commit anything the importer writes, which is the `.ptm`
files, the `.parquet` files, and the battery YAML. `.gitignore` covers `local/`, but stay
deliberate.
