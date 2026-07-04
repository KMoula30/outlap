<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Notebooks

Illustrated, executable walkthroughs of outlap. Every number and plot is computed live by the
Rust core through the `outlap.core` Python bindings — nothing is hard-coded, so the notebooks
double as end-to-end tests of the whole stack.

| Notebook | What it covers |
|---|---|
| [`00_tour_of_outlap.ipynb`](00_tour_of_outlap.ipynb) | The guided tour: the car-as-data contract, the 3D track, the min-curvature racing line, a T0 lap (speed profile, g-g diagram), the MF6.1 tyre model, and the ≤0.5 % oracle validation. Start here. |

Numbered deep-dives per subsystem land in a follow-up increment.

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
