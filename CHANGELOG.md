# Changelog

All notable changes to outlap are documented here. This project follows
[Conventional Commits](https://www.conventionalcommits.org) and
[Semantic Versioning](https://semver.org).

## [0.1.0] - 2026-07-03

First tagged milestone (**M1**): the full input-quartet file-format contract, the 3D track model
with a real-circuit importer, the minimum-curvature racing line, and the T0 point-mass lap solver —
producing the first lap time on Circuit de Barcelona-Catalunya. Plus the PDT HDF5 importer.

### Features

- **schema**: `outlap-schema` — the file-format contract. `vehicle.yaml` (chassis, aero, suspension,
  tyres, the §8.0 drivetrain topology graph, ERS/MGU-K, battery, brakes) plus the referenced
  `.ptm`/`.tyr`/`.emotor` schemas, and the full staged load/validation pipeline (version gate,
  `extends` deep-merge with provenance, unknown-key walk with did-you-mean, semantic + topology
  checks, estimation, loaded-model report). JSON Schemas generated from the Rust types.
- **track**: `outlap-track` — the first open **3D** racetrack format (`track.yaml` + `centerline.csv`,
  §9.3). Fits the centerline with a C² cubic spline (periodic for closed circuits) and the per-`s`
  data channels with the shared monotone cubic Hermite; exposes curvature, grade, banking, the road
  frame, and widths by arc length. Completes the input quartet (`conditions.yaml`, `sim.yaml`).
- **track**: `offset_track` — a laterally-offset line is itself a first-class `Track`.
- **importers (Python)**: OSM + DEM track importer → `track.yaml`; imported **Catalunya**, Spa, and
  Silverstone presets. PDT HDF5 importer → `.ptm` (EDrive/DriveUnit) + provisional battery params,
  with a 2-node `.emotor` thermal distillation of the PDT 19-node LPTN.
- **qss**: `outlap-qss` — the **T0 point-mass lap solver**. Forward/backward velocity-profile solve
  on the 3D ribbon over a constant-μ friction ellipse with a velocity-resolved tractive-force
  envelope; friction from the tyre MF6.1 factors, aero from constant CdA/CzA. First Catalunya lap
  time (~105 s on the centerline), solved in a few ms, zero-allocation kernel.
- **raceline**: `outlap-raceline` — the minimum-curvature racing line (§6.3), a sparse box-constrained
  QP solved with clarabel; lowers the Catalunya T0 lap to ~101 s.
- **fixtures**: synthetic reference vehicles (F1 2026, GT hybrid, EV platform, passenger hatch) and a
  promoted `f1_2026` reference vehicle for the examples.

### Documentation

- Working agreement (`CLAUDE.md`), CI, and `CONTRIBUTING.md` (DCO).
- T0 theory page with the point-mass equations and citations.

### Miscellaneous

- Cargo workspace bootstrap; AGPL-3.0 code, Apache-2.0 schemas; `wasm32-unknown-unknown` built in CI.

[0.1.0]: https://github.com/KMoula30/outlap/releases/tag/v0.1.0
