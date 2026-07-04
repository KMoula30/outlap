# Changelog

All notable changes to outlap are documented here. This project follows
[Conventional Commits](https://www.conventionalcommits.org) and
[Semantic Versioning](https://semver.org).

## [Unreleased]

Milestone **M2** (tyre model) — in progress, not yet tagged.

### Added

- **tire**: physical **brush model** (Pacejka ch. 3, parabolic pressure) — a first-principles
  force core (`F_x`, `F_y`, `M_z` with the closed-form pneumatic trail; `M_x = M_y = 0`) from the
  two tread stiffnesses, base friction, and contact half-length. First-order **relaxation** helper
  with the exact-exponential update and `PT*`/carcass-stiffness/last-resort relaxation lengths. A
  static `TireModel` (`Mf61 | Brush`, no `dyn`) selects the force model at assembly. Theory pages
  `docs/theory/brush-model.md` and the relaxation section of `mf61-steady-state.md`.
- **schema** (`tyr/1.0 → 1.1`, MINOR): optional `brush:` block on `.tyr` documents (additive JSON
  Schema change). The required-key rule now splits into always-required structural keys
  (`FNOMIN`, `UNLOADED_RADIUS`) and the MF6.1 force core, which is required only when no `brush`
  block is present; a brush block alongside a partial force set is a warning. New diagnostics: a
  `brush` block under `tyr/1.0` warns, and an unknown key in a file declaring a newer schema MINOR
  hints at the version.
- **schema**: `.tir` (TNO MF-Tyre text format) **parser/writer/codec** in `outlap-schema::tir`,
  round-trippable with `.tyr`. String-in/string-out (wasm-clean); `load_tir` is the only IO entry,
  behind the `SourceLoader` trait. Grammar: `[SECTION]` headers, `KEY = value`, `$`/`!` comments,
  quoted strings, BOM/CRLF tolerant, duplicate-key last-wins with a warning. Non-SI `[UNITS]` is a
  hard error; the writer always emits SI. One canonical `f64` text format with documented
  exponent-switch thresholds (plain for decimal exponent `−4..=15`, else Python-`repr`-style
  scientific) so the PR7 Python codec can reproduce it byte-for-byte. `tir_to_tyr` synthesises the
  `thermal`/`wear` blocks `.tir` does not carry (`synthetic | from-donor | none` policy) and records
  the provenance. Round-trip is byte-stable for `tir→doc→tir` and numeric-exact over the mapping
  table (coefficient + housekeeping) keys for `tir→tyr→tir`. No JSON Schema change (`.tir` is text,
  not a schemars type).
- **fixtures**: **TUMFTM Roborace MF5.2 reference tyre** (`data/tires/roborace_devbot_mf52/`) —
  transcribed verbatim from Open-Car-Dynamics (Apache-2.0, pinned commit) and mapped MF5.2→6.1
  with a per-coefficient source table (no pressure model ⇒ `dpi ≡ 0` exact; camber `PHY3` route
  folded into `PKY6` by matching small-camber Fy sensitivity at `FNOMIN`; `QSY1` from the source's
  rolling-resistance coefficient; `Mz ≡ Mx ≡ 0`). Reference integration tests now glob every
  `data/tires/**` dataset for warning-free load + `.tir` codec round-trip, with per-tyre physics
  checks (class-plausible grip, signs, camber remap) alongside.
  The planned Perantoni–Limebeer F1 tyre is deferred: its published model is a reduced similarity
  form (no MF coefficient set) and the parameter appendix is not openly available — an MF6.1
  derivation would break the transcription-only provenance rule.
- **python**: **`.tir` codec** (`outlap.tir`, stdlib+pyyaml; `python -m outlap.tir
  {to-tyr, from-tyr}`) — grammar-identical to the Rust codec, **byte-parity** with the Rust
  writer pinned by a committed canonical fixture asserted from both languages (CPython `repr`
  digits match the Rust writer's `ryu` round-half-to-even digits; fuzz-validated). **MF6.1
  fitting pipeline** (`outlap.tirefit`; `python -m outlap.tirefit {fit, synth}`): TTC
  `.mat`/`.dat`/`.csv` readers to SI/ISO-8855 with a documented SAE→ISO sign map (parsers yes —
  redistribution of TTC data or TTC-derived sets, no; `ttc-data/` gitignored); a vectorized
  numpy MF6.1 forward model validated against the same golden CSVs and ≤0.5% rule as the Rust
  kernels; a deterministic staged fit (nominals → pure Fx0 → pure Fy0 → combined → Mz → Mx/My)
  with documented init/bounds tables and signal gates; JSON+MD fit reports (no plots in M2);
  seeded synthetic-data generation. scipy is confined to the `tire-fit` extra (lazy import,
  actionable error). Synthetic recovery gate: book tyre + 1% noise → pure curves ≤1%.
- **python**: ruff (+import-sort/pyupgrade/bugbear) and **pyright strict** configured in
  `pyproject.toml` with curated allows at the untyped h5py/scipy boundary; CI python job now
  gates `ruff check`, `ruff format --check`, and `pyright`, and syncs `--extra tire-fit`.

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
