<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Notebooks

These notebooks are illustrated walkthroughs of outlap, and you can run every one of them. The Rust
core computes each number and each plot live, through the `outlap.core` Python bindings. Nothing is
hard-coded. The notebooks therefore also serve as end-to-end tests of the whole stack.

[`docs/GUIDE.md`](../docs/GUIDE.md) is the written guide, which takes a reader from zero to
competence. These notebooks are its runnable companion. Its chapters map onto the sequence below.

| Notebook | What it covers |
|---|---|
| [`00_tour_of_outlap.ipynb`](00_tour_of_outlap.ipynb) | A guided tour of everything. **Start here.** |
| [`01_car_as_data.ipynb`](01_car_as_data.ipynb) | The input quartet, the validation diagnostics, the loaded-model report, and the **what-if override API**. It adds a tornado chart of lap-time sensitivity and live sliders. |
| [`02_track.ipynb`](02_track.ipynb) | The 3D ribbon: corridor, curvature, vertical curvature, widths, and provenance. It includes a corner explorer and the elevation showcase at Spa-Francorchamps, which climbs about 100 m. |
| [`03_raceline.ipynb`](03_raceline.ipynb) | The minimum-curvature QP: offsets against the corridor bounds, the reduction in curvature, and a sweep over car width. It then shows the time-weighted line, which begins to close the gap between the minimum-curvature line and the minimum-time line. |
| [`04_t0_lap.ipynb`](04_t0_lap.ipynb) | The anatomy of the solver: acceleration populations, convergence in `ds`, determinism, and sweeps over session conditions. |
| [`05_tyre_mf61.ipynb`](05_tyre_mf61.ipynb) | MF6.1 in depth: families of curves over load and camber, the force map on the slip plane, and validation of each channel against the oracle. |
| [`06_powertrain_pdt.ipynb`](06_powertrain_pdt.ipynb) | The `.ptm` firewall, the PDT importer running on synthetic HDF5, the distilled 2-node thermal model, and the battery pack. |
| [`07_qss_t1.ipynb`](07_qss_t1.ipynb) | **The T1 capstone**: the double-track trim, per-wheel loads, setup metrics, and the g-g-g-v envelope. It then runs the Model 3 RWD in its HV variant, with the live Vdc–SoC coupling and the machine-thermal derate, swept across three drive-unit sizings. |
| [`08_transient_t2.ipynb`](08_transient_t2.ipynb) | **The T2 capstone**: the transient tier driven around a lap. It overlays QSS and T2 and checks hull containment. It shows the time-domain traces that a station solver cannot produce: steer, yaw rate, sideslip, per-wheel load and slip, and the shift FSM. It measures the lap time that the time-weighted line recovers, and runs a lap on the full 3-D road frame. |
| [`09_race_engineering.ipynb`](09_race_engineering.ipynb) | **Race engineering**: how to read the T2 traces like a data logger. It covers the anatomy of one corner, which is the braking point, trail braking, the apex, and throttle pickup, with the friction circle in action. It then covers car balance: understeer and oversteer through what-if aero overrides, and why the neutral car is the fast car. |
| [`10_stint_strategy.ipynb`](10_stint_strategy.ipynb) | **Stint strategy**: running many laps while the tire thermal state and wear state carry across each lap boundary. It shows warm-up, the degradation curve, and where the pace cliff falls. |
| [`11_race_energy.ipynb`](11_race_energy.ipynb) | **The energy capstone**: coupled energy accounting over a full race distance of 66 laps. State of charge moves in both directions. It shows the deploy and harvest ledger for each lap and which limit binds, fuel burn against pace, the deploy, harvest, and recharge phases within a lap, and a measurement of the override mode. It closes with a cross-check on the transient tier and with pitch under braking, and its effect on aero balance, on the 14-DOF tier. |

Some panels are interactive: ipywidgets sliders drive the override API. They are live in a running
Jupyter. Each one has a static twin, so the page that GitHub renders tells the same story.

`07_qss_t1_local.ipynb` is the **untracked real-data twin** of the capstone. Git ignores it by
name. It tells the same Model 3 story on real PDT drive-unit imports and the real 704 V pack. It
needs the local imports that `data/vehicles/tesla_model3_rwd/README.md` describes. It is never
committed, because of the firewall.

## How to run them

```bash
cd python
uv sync --group notebooks          # builds the Rust extension automatically (needs a Rust toolchain)
uv run --with jupyterlab jupyter lab ../notebooks/00_tour_of_outlap.ipynb
```

## Conventions

- Each notebook is committed **with its outputs**, so that it reads well on GitHub without running.
- CI re-executes every notebook headless on each PR, with `jupyter execute`. The build then fails
  until someone updates the notebook, if the API breaks or any cell raises an error. The
  in-notebook assertions can raise those errors too. Two of them are the 0.5 % tire gate and the
  check that the racing line beats the center line.
- Charts follow the data-viz style of this repository: a validated palette that is safe for
  colorblind readers, one axis for each chart, and SI units on the axes.
