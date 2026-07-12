<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Notebooks

Illustrated, executable walkthroughs of outlap. Every number and plot is computed live by the
Rust core through the `outlap.core` Python bindings — nothing is hard-coded, so the notebooks
double as end-to-end tests of the whole stack.

These notebooks are the runnable companion to [`docs/GUIDE.md`](../docs/GUIDE.md), the written
zero-to-hero guide; the guide's chapters map onto the notebook sequence below.

| Notebook | What it covers |
|---|---|
| [`00_tour_of_outlap.ipynb`](00_tour_of_outlap.ipynb) | The guided tour of everything. **Start here.** |
| [`01_car_as_data.ipynb`](01_car_as_data.ipynb) | The input quartet, validation diagnostics, the loaded-model report, and the **what-if override API** (+ a lap-time sensitivity tornado and live sliders). |
| [`02_track.ipynb`](02_track.ipynb) | The 3D ribbon: corridor, curvature, vertical curvature, widths, provenance — a corner explorer, and the Spa-Francorchamps elevation showcase (~100 m of climb). |
| [`03_raceline.ipynb`](03_raceline.ipynb) | The min-curvature QP: offsets vs corridor bounds, curvature reduction, a car-width sweep — and the time-weighted line that starts closing the min-curvature-vs-min-time gap. |
| [`04_t0_lap.ipynb`](04_t0_lap.ipynb) | Solver anatomy: acceleration populations, `ds` convergence, determinism, and session-conditions sweeps. |
| [`05_tyre_mf61.ipynb`](05_tyre_mf61.ipynb) | MF6.1 in depth: load/camber families, the slip-plane force map, and per-channel oracle validation. |
| [`06_powertrain_pdt.ipynb`](06_powertrain_pdt.ipynb) | The `.ptm` firewall, the PDT importer on synthetic HDF5, the distilled 2-node thermal model, and the battery pack. |
| [`07_qss_t1.ipynb`](07_qss_t1.ipynb) | **The T1 capstone**: double-track trim, per-wheel loads, setup metrics, the g-g-g-v envelope — then the Model 3 RWD (HV variant) with the live Vdc–SoC coupling + machine-thermal derate, swept across three drive-unit sizings. |
| [`08_transient_t2.ipynb`](08_transient_t2.ipynb) | **The T2 capstone**: the transient tier driven around the lap — the QSS↔T2 overlay and hull containment, the time-domain traces a station solver can't show (steer, yaw rate, sideslip, per-wheel load/slip, the shift FSM), the time-weighted line's lap-time recovery, and a lap on the full 3-D road frame. |
| [`09_race_engineering.ipynb`](09_race_engineering.ipynb) | **Race engineering**: reading the T2 traces like a data logger — the anatomy of one corner (braking point, trail braking, apex, throttle pickup, the friction circle in action) and car balance (understeer/oversteer via what-if aero overrides; why the neutral car is the fast car). |

Interactive panels (ipywidgets sliders driving the override API) are live in a running Jupyter;
each has a static twin so the GitHub-rendered page tells the same story.

`07_qss_t1_local.ipynb` is the capstone's **untracked real-data twin** (git-ignored by name): the
same Model 3 story on real PDT drive-unit imports and the real 704 V pack. It requires the local
imports described in `data/vehicles/tesla_model3_rwd/README.md` and is never committed (firewall).

## Running them

```bash
cd python
uv sync --group notebooks          # builds the Rust extension automatically (needs a Rust toolchain)
uv run --with jupyterlab jupyter lab ../notebooks/00_tour_of_outlap.ipynb
```

## Conventions

- Committed **with outputs** so they read well on GitHub without running anything.
- CI re-executes every notebook headless on each PR (`jupyter execute`): if the API breaks or
  any cell errors — including the in-notebook assertions (the 0.5 % tyre gate, the racing line
  beating the centerline) — the build fails until the notebook is updated.
- Charts follow the repo's data-viz style (validated colorblind-safe palette, one axis per
  chart, SI units on axes).
