<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# tesla_model3_rwd — Tesla Model 3 RWD (HV variant study)

A production **Tesla Model 3 RWD** identity re-imagined as an **HV (800 V-class) variant**
(M3 user decision #6): the chassis, mass, and aero are Model-3-plausible, while the powertrain
is an 800 V-class drive-unit + pack stack so the Vdc–SoC coupling (§8.4) is live on a road car.
The car is the EV half of the notebook 07 capstone and the sizing-sensitivity demo.

Everything committed here is **synthetic or estimated** — the drive-unit maps and the pack
tables are invented smooth surfaces written by a committed script
(`python/tools/gen_model3_powertrain.py`), never measured data and never derived from any PDT
export (firewall; M3 user decision #7). The real PDT imports stay **local and untracked** —
see "Local reproduction" below.

## Per-parameter provenance

| Field | Value | Source |
|---|---|---|
| `chassis.mass_kg` | 1765 | published curb mass of the current Model 3 RWD (manufacturer EU spec ≈1765 kg) |
| `chassis.wheelbase_m` | 2.875 | published spec |
| `chassis.track_m` | [1.58, 1.58] | published spec (front/rear track 1580 mm) |
| `chassis.cg[0]` (CG→front axle) | 1.524 m | **estimated** from a ≈47/53 front/rear static distribution (rear motor + rear-biased pack): a = 0.53·w |
| `chassis.cg[2]` (CG height) | 0.45 m | **estimated** — floor-mounted pack keeps it low for a sedan |
| `chassis.inertia` (Ixx, Iyy, Izz) | 560, 2800, 3200 kg·m² | **estimated** mid-size-sedan magnitudes; unused by the QSS trim except through published sensitivities (steady-state trim uses no inertia) |
| `aero.constant.cx_a_m2` | 0.51 | Cd·A = 0.23 (published drag coefficient) × 2.22 m² (published frontal area) |
| `aero.constant.cz_*_a_m2` | 0.0 / 0.0 | **estimated** — zero-lift road body (no ride-height/yaw map; the passenger-car degenerate case, §7.4) |
| `suspension.*.ride_rate_n_per_m` | 38 000 / 45 000 | **estimated** — k = m_corner·(2πf)² at ≈1.52 Hz front / ≈1.56 Hz rear ride frequencies (47/53 corner masses), rounded |
| `suspension.*.roll_stiffness_share` | 0.58 / 0.42 | **estimated** — front-bar-biased road-car balance (understeer-safe) |
| `suspension.*.roll_center_height_m` | 0.06 / 0.12 | **estimated** — typical strut front / multi-link rear geometry |
| `suspension.*.anti_dive`/`anti_squat` | omitted | filled by the load pipeline's documented estimator and **surfaced in the loaded-model report** |
| `tires` | `tyr/road.tyr.yaml` | the published Pacejka (2006) 205/60R15 book tyre — a **documented proxy** for the real 235/45R18 (user decision #6; no public MF set exists for the OE tyre); copy of `data/tires/pacejka_2006_205_60r15/car.tyr.yaml`, provenance in that README |
| `drivetrain` | 1 DU → open diff → RL/RR | the `ev_1du_rwd` reference topology; production Model 3 RWD layout |
| `drivetrain.units[0].source` | `ptm/du_medium.ptm.yaml` | **synthetic** (see below); the medium sizing ≈203 kW ≈ a production Model 3 RWD's ≈200 kW |
| `drivetrain.units[0].thermal` | `emotor/rear_du.emotor.yaml` | **estimated** hand-authored lumped LPTN menu (§9.5): capacities from component-mass fractions of the ≈82 kg DU, conductances at liquid-cooled traction-machine magnitudes |
| `battery` | `battery/pack_800v.battery.yaml` | **synthetic** 800 V-class pack (see below) |
| `brakes.balance_bar` | 0.62 | **estimated** — braking is tyre-limited either way |
| `brakes.disc.*` | 26/20 kJ/K, 0.07/0.05 m² | **estimated** road-car disc scale |
| `brakes.regen_blend.max_regen_frac` | 0.6 | **estimated** one-pedal blend ceiling |

Estimated values are deliberately visible: the vehicle loads **warning-clean**, with every
estimate noted in the loaded-model report (`outlap.vehicle_report(...)`) and the run-time
simplifications recorded in the lap's `notes` attr — nothing silent (Decision #41).

## Synthetic powertrain (committed) — the three sizings

`python/tools/gen_model3_powertrain.py` writes the three Vdc-stacked (`ptm/1.1`) drive-unit
maps and the pack; notebook 07 sweeps them as the sizing-sensitivity axis:

| Variant | Peak torque (output shaft) | ≈Peak power | File |
|---|---|---|---|
| small | 1365 N·m | 100 kW | `ptm/du_small.ptm.yaml` |
| **medium (default)** | 2765 N·m | 203 kW | `ptm/du_medium.ptm.yaml` |
| large | 3381 N·m | 248 kW | `ptm/du_large.ptm.yaml` |

The three peak-torque scales mirror the author's local drive-unit sizing sweep so this
committed story and the untracked real-data twin are directly comparable; the surfaces
themselves are invented (documented in the generator). The pack is a synthetic 220S/1P,
64.064 kWh, 800 V-class configuration (≈634–810 V open-circuit over the SoC grid) whose
terminal voltage sags below the drive units' 730–850 V Vdc grid under low-SoC load,
exercising the documented below-grid linear extrapolation of the Vdc–SoC coupling.

Swap a sizing without editing files (what-if override, Decision #35):

```python
solve_lap_dataset(vehicle_dir, line, tier="t1",
                  overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"})
```

## Local reproduction (real PDT data — NEVER committed)

The real drive-unit maps and the real 704 V pack import into the git-ignored
`data/vehicles/tesla_model3_rwd/local/` directory. With the source files in
`~/pdt_reference/`, run from `python/`:

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

The driveunit importer emits the **full Vdc stack** (730–850 V) by default — exactly what the
coupling wants. Point the car at a real import with the same what-if overrides, e.g.
`overrides={"drivetrain.units.0.source": "local/du_3381.ptm.yaml", "battery.params":
"local/pack_704v.battery.yaml"}` (paths resolve inside this vehicle directory). The untracked
notebook `notebooks/07_qss_t1_local.ipynb` runs this end-to-end. Never commit the `.h5`
sources or anything the importer writes (`.ptm`/`.parquet`/battery YAML) — `.gitignore`
covers `local/`, but stay deliberate.
