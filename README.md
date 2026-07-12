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

## What works at v0.2.5

- **One vehicle description** consumed by every solver tier — chassis, aero, suspension, tyres, a
  drivetrain topology graph, ERS/battery, brakes — with a strict, friendly load pipeline (miette
  spans, did-you-mean, plain-language topology errors). Powertrains enter *only* as neutral `.ptm`
  map files (the firewall).
- **Three solver tiers.** **T0**, a point-mass forward/backward velocity-profile solve on the 3D
  ribbon; **T1**, a double-track per-station trim that emits per-wheel loads, slips, and forces
  plus setup metrics, and generates a **g-g-g-v envelope** the fast T0 path then consumes; and
  **T2, the transient tier** — a 7-DOF chassis integrated through time at 1 ms in the curvilinear
  3D road frame (symbolically verified to 1e-12 in CI), with tyre relaxation, an ideal preview
  driver behind a corner-scaled stability margin, a gear-shift state machine, torque vectoring,
  and regen blending. T2 returns a time-indexed data-logger trace: steering, yaw, sideslip,
  per-wheel loads/slips, gear, regen power, SoC. (`t3`, the 14-DOF suspension model, is future.)
- **Tyres**: a steady-state Magic Formula 6.1 model and a physical brush model, with a `.tir` codec
  and a Python MF6.1 fitting pipeline; citation-backed reference `.tyr` sets; first-order slip
  relaxation, live in T2.
- **Powertrain, thermal, and battery**: `.ptm` maps flowing through the drivetrain topology graph
  (gearboxes, splits, open/locked/LSD/solid diffs); an N-node machine-thermal network with torque
  derating; and a Thévenin battery whose SoC-dependent terminal voltage feeds back into the
  drive-unit maps (the Vdc–SoC coupling). Slow states march along a QSS lap — and at T2 the pack
  charges under braking and discharges under power, live in the time loop.
- **A 3D track model** (`track.yaml` + `centerline.csv`) with curvature, grade, banking, and the
  road frame by arc length, plus **two racing-line generators**: the minimum-curvature QP and its
  **time-weighted** refinement (weights ∝ time spent, the first step toward the minimum-time
  line). Ships 27 circuits: the 3D `catalunya_osm` and `spa_osm` (OSM + DEM; Spa carries its real
  ~100 m of elevation) and 25 flat TUMFTM circuits (LGPL-3.0).
- **Importers**: OSM+DEM tracks (with closed-lap graph assembly for fragmented circuits), TUMFTM
  tracks, PDT HDF5 powertrains (→ `.ptm` maps, battery params, an `.emotor` thermal network), and
  `.tir` tyre files.
- **A notebook course** (`notebooks/00`–`09`, CI-executed with committed outputs): from the car as
  data to reading T2 traces like a race engineer — corner anatomy, the friction circle in action,
  car balance via what-if overrides.
- **Validation, honestly reported**: the Perantoni & Limebeer 2014 F1 cross-check (top speed
  within 1 %, corner apexes within 5 %); the QSS↔T2 **hull-containment** parity gate (every T2
  operating point inside the T1 grip envelope — measured 0.0 % exceedance on all three reference
  cars); and the numbers that do *not* meet their ambitions recorded with full decompositions
  instead of hidden — the T2 lap-time gap (driver stability margin) and the transient throughput
  ceiling (`docs/validation/`).

See [`docs/GUIDE.md`](docs/GUIDE.md) for the full capability tour, including an honest account of
the limits (Chapter 15).

## Quick start

```sh
# Rust core: build, lint, test
cargo test --workspace

# First lap on Catalunya (T0 point-mass)
cargo run -p outlap-qss --example catalunya_lap
# → Lap time ~104.7 s, top speed ~337 km/h; writes a CSV for plotting

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
| `crates/outlap-core`   | shared numerics (monotone Hermite, C² splines, N-D gridded maps) + the block/bus/SoA scaffolding and the fixed-step split integrator |
| `crates/outlap-tire`   | MF6.1 + brush tyre models, slip relaxation, `.tir` codec |
| `crates/outlap-track`  | 3D track model |
| `crates/outlap-thermal`| N-node machine thermal network (LPTN) |
| `crates/outlap-qss`    | T0/T1 quasi-steady-state lap solvers + g-g-g-v envelope + the corner-scaled T2 speed targets |
| `crates/outlap-raceline` | min-curvature + time-weighted racing lines |
| `crates/outlap-vehicle` | T2 physics blocks: 7-DOF chassis RHS, load transfer, tyres with relaxation, the preview driver |
| `crates/outlap-transient` | T2 lap orchestration: split-integrator step loop, line table, shift/TV/regen control layer |
| `crates/outlap-py`     | PyO3 bindings (the `outlap_core` extension) |
| `python/outlap`        | Python API, OSM/DEM + TUMFTM + PDT importers, tyre fitting, plotting |
| `schemas/`             | published JSON Schemas (generated from the Rust types) |
| `docs/GUIDE.md`        | the zero-to-hero user guide |
| `data/`                | reference vehicles and imported tracks |

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). Contributions are under AGPL-3.0 with a DCO sign-off.
