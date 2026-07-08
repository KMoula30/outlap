<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# outlap

A parametric vehicle simulator — F1 → GT → passenger car — with a race-strategy Monte Carlo layer
planned on top. A Rust core (`crates/`), a Python API (`python/outlap/`, PyO3 + maturin), and
published JSON Schemas (`schemas/`). Code is AGPL-3.0; the schemas are Apache-2.0.

**New here? Read [`docs/GUIDE.md`](docs/GUIDE.md) — the zero-to-hero user guide.** It assumes no
vehicle-dynamics background and takes you from "what is a lap simulator" to running, understanding,
and extending outlap, with the physics, the API, and worked recipes.

The full architecture and specification live in [`docs/HANDOFF.md`](docs/HANDOFF.md); the working
agreement is in [`CLAUDE.md`](CLAUDE.md).

## What works at v0.2 (milestone M3)

- **One vehicle description** consumed by every solver tier — chassis, aero, suspension, tyres, a
  drivetrain topology graph, ERS/battery, brakes — with a strict, friendly load pipeline (miette
  spans, did-you-mean, plain-language topology errors). Powertrains enter *only* as neutral `.ptm`
  map files (the firewall).
- **Two quasi-steady-state solver tiers.** **T0**, a point-mass forward/backward velocity-profile
  solve on the 3D ribbon; and **T1**, a double-track per-station trim that emits per-wheel loads,
  slips, and forces plus setup metrics, and generates a **g-g-g-v envelope** the fast T0 path then
  consumes. (`sim.tier` selects the solver; `t2`/`t3` transient tiers arrive in M4/M6.)
- **Tyres**: a steady-state Magic Formula 6.1 model and a physical brush model, with a `.tir` codec
  and a Python MF6.1 fitting pipeline; citation-backed reference `.tyr` sets.
- **Powertrain, thermal, and battery**: `.ptm` maps flowing through the drivetrain topology graph
  (gearboxes, splits, open/locked/LSD/solid diffs); an N-node machine-thermal network with torque
  derating; and a Thévenin battery whose SoC-dependent terminal voltage feeds back into the
  drive-unit maps (the Vdc–SoC coupling). Machine temperature and pack SoC advance as slow states.
- **A 3D track model** (`track.yaml` + `centerline.csv`) with curvature, grade, banking, and the
  road frame by arc length, plus a **minimum-curvature racing line**. Ships 26 circuits: the 3D
  `catalunya_osm` (OSM + DEM) and 25 flat TUMFTM circuits (LGPL-3.0).
- **Importers**: OSM+DEM and TUMFTM tracks, PDT HDF5 powertrains (→ `.ptm` maps, battery params, an
  `.emotor` thermal network), and `.tir` tyre files.
- **Validation**: the Perantoni & Limebeer 2014 F1 cross-check at Catalunya (top speed within 1 %,
  corner apexes within 5 %; the lap-time delta is recorded, not gated — see `docs/validation/`).

These are sanity-to-corner-level tiers, not a full transient parity model — see
[`docs/GUIDE.md`](docs/GUIDE.md) for the full capability tour and `docs/HANDOFF.md` §13 for the
validation plan.

## Quick start

```sh
# Rust core: build, lint, test
cargo test --workspace

# First lap on Catalunya (T0 point-mass)
cargo run -p outlap-qss --example catalunya_lap
# → Lap time ~104.7 s, top speed ~335 km/h; writes a CSV for plotting

# Centerline vs min-curvature racing line
cargo run -p outlap-raceline --example catalunya_line

# Plots (Python)
cd python && uv sync --extra track-import
uv run python examples/plot_lap.py examples/output/catalunya_t0.csv
uv run python examples/plot_line_compare.py
```

The Catalunya T0 lap, coloured by speed (yellow on the straights, dark at the hairpins):

![Catalunya T0 lap](python/examples/output/catalunya_t0_map.png)

## Layout

| Path | What |
|------|------|
| `crates/outlap-schema` | file-format contract: serde/schemars types + load pipeline |
| `crates/outlap-core`   | shared numerics (monotone Hermite, C² splines, N-D gridded maps) |
| `crates/outlap-tire`   | MF6.1 + brush tyre models, `.tir` codec |
| `crates/outlap-track`  | 3D track model |
| `crates/outlap-thermal`| N-node machine thermal network (LPTN) |
| `crates/outlap-qss`    | T0/T1 quasi-steady-state lap solvers + g-g-g-v envelope |
| `crates/outlap-raceline` | minimum-curvature racing line |
| `crates/outlap-py`     | PyO3 bindings (the `outlap_core` extension) |
| `python/outlap`        | Python API, OSM/DEM + TUMFTM + PDT importers, tyre fitting, plotting |
| `schemas/`             | published JSON Schemas (generated from the Rust types) |
| `docs/GUIDE.md`        | the zero-to-hero user guide |
| `data/`                | reference vehicles and imported tracks |

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). Contributions are under AGPL-3.0 with a DCO sign-off.
