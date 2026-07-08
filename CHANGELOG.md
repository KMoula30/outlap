# Changelog

All notable changes to outlap are documented here. This project follows
[Conventional Commits](https://www.conventionalcommits.org) and
[Semantic Versioning](https://semver.org).

## [Unreleased]

Milestone **M4** groundwork — the block/bus scaffolding and the fixed-step split integrator the
transient **T2** tier is built on. No solver behaviour changes yet: T2 still raises
`tier_not_implemented`; the QSS (T0/T1) paths are untouched.

### Added

- **core**: **Block/Bus/SoA scaffolding** (HANDOFF §6.2, Decision #39). A `Block` trait
  (`equilibrium` / `derivatives` / `slow_derivatives`, pure and generic over `f32`/`f64`) with
  statically declared ports; a flat struct-of-arrays signal `Bus` with a fixed compile-time core
  channel set (scalars + per-wheel groups) plus an interned dynamic named-channel region (interning
  at assembly only, never in the loop); a frozen fast-state registry with **14-DOF-ready** chassis
  slots (T2 integrates the first ten; heave/pitch/roll + four-unsprung reserved for T3) and a
  per-wheel relaxation region; `StateView`/`DerivView`/`SlowStateView`/`SlowDerivView` over the SoA
  buffers with an explicit batch dimension; a topological-sort assembler that fixes the
  `sense → control → actuate → integrate` phase order and linearises intra-phase data dependencies
  (cyclic loops are a hard error), deterministic by registration-index tie-break; enum dispatch
  (`CoreBlock`, no `dyn` in the loop) with a stubbed suspension block for the T3 groundwork.
- **core**: **fixed-step split integrator** (HANDOFF §11.2, Decision #30). A Butcher-tableau-generic
  explicit RK stepper (`SimArena`; Heun/RK2 default, RK4 selectable) with zero-allocation stepping;
  the shared **exact-exponential** relaxation primitive (`relax::exact_exponential` — the one
  implementation `outlap_tire::relax_step` now delegates to); a **semi-implicit Euler** slow-state
  substep (`relax::semi_implicit_decay`) on a decimated `SlowClock`; and a step-boundary
  `EventQueue` with a single linear `back_interpolate` crossing (no root-finding). An `O(dt²)`
  convergence test pins the stepper against a `diffsol` BDF reference; determinism is bit-exact.
  Theory pages `docs/theory/block-bus.md` and `docs/theory/integrator.md`.
- **qss**: exported the T1 quasi-static **load-transfer algebra** (`load_transfer`, `split_axle`,
  `LoadTransferGeometry`) so the forthcoming T2 chassis block derives per-wheel normal loads from
  the identical expressions (HANDOFF §6.1). Behaviour-preserving refactor: the trim solver now calls
  the free function.

### Changed

- **schema** (`sim/1.1 → 1.2`, MINOR): `fz_coupling` becomes optional (`null` = tier-dependent
  auto — `one_step_lag` for T0/T1, `fixed_point` for T2/T3, resolved and recorded at assembly);
  new `slow_decimation` (slow-clock decimation factor, default 20) and `fixed_point`
  (`damping`/`tol`/`max_iter`) knobs for the split integrator. Additive JSON-Schema change with
  miette-spanned semantic validation; the pydantic-mirror validation is unchanged (still deferred).

## [0.2.0] - 2026-07-08

Milestone **M3** — the full quasi-steady-state **T1** tier. v0.2 turns the T0 point-mass solver of
v0.1 into a double-track car: a per-station trim solve produces per-wheel loads, slips, and forces
and a **g-g-g-v envelope** that the fast T0 path consumes; powertrains flow through the drivetrain
topology graph; an N-node machine-thermal network and a battery model (with Vdc–SoC coupling)
advance as slow states and cap the traction limit. Ships the Perantoni & Limebeer cross-check, the
TUMFTM track library, a Tesla Model 3 HV-variant reference car, and a full user guide. (Also
folds in the M2 tyre model: MF6.1, the brush model, the `.tir` codec, and the fitting pipeline.)

### Added

- **core**: **N-D gridded maps + Parquet sidecar reader** — `GriddedMapN`, a rectilinear N-D
  tensor-product map built on the one shared monotone cubic Hermite (Decision #30): C¹, analytic
  partials for Newton, NaN-cell masking (clamp-to-valid-hull), and a per-axis out-of-domain mode
  (`clamp` default, or `linear` extrapolation from the boundary derivative). Binary sidecars load
  through `SourceLoader::load_bytes` (wasm-clean: bytes in, no filesystem) and are decoded from
  Parquet at assembly time only, never in the loop.
- **qss**: **T1 double-track trim solver** — a zero-allocation, panic-free damped-Newton solve of
  the quasi-static force/moment balance at each `(v, aₓ, a_y)`: unknowns are steer, body slip,
  yaw rate, throttle/brake split, and the four vertical loads; residuals close the X/Y/N balance,
  quasi-static load transfer (geometric via roll-centre heights + anti-dive/anti-squat, elastic via
  roll-stiffness distribution), and per-wheel `TireModel::forces`. `fz_coupling: one_step_lag`
  (default) | `fixed_point` (Decision #29) is recorded in every result. Emits setup metrics
  (understeer gradient, aero balance vs speed); infeasible points become envelope boundaries, never
  a panic. Theory page `docs/theory/t1-trim.md`.
- **qss**: **ride-height/yaw aero map + platform equilibrium** — the trim consumes an `aero.map`
  (`{C_z,f, C_z,r, C_x} = f(hꜰ, hᵣ, yaw[, DRS])`) with a damped fixed-point ride-height solve; yaw
  sensitivity makes the g-g asymmetric mid-corner. A passenger car degenerates to `aero.constant`.
  Ships a synthetic `data/vehicles/f1_2026/aero/f1_2026.parquet` anchored to the PL2014 aero at
  equilibrium ride heights (generator `python/tools/gen_f1_aero.py`, every assumption documented).
- **qss**: **topology powertrain in the traction limit** — traction/braking limits flow through the
  drivetrain graph: per-unit `.ptm` torque/efficiency maps, gearbox ratios + efficiency, static
  front/rear and left/right splits, and differentials (open/locked/LSD/solid). ICE `.ptm` maps
  (torque + optional fuel-flow) are consumed for energy accounting (fuel mass constant in M3). A
  PDT round-trip gate reproduces spot efficiencies to 1e-6 through the real `GriddedMapN` path.
  Theory page `docs/theory/qss-powertrain.md`.
- **thermal**: **N-node machine thermal network + derating** (`outlap-thermal`) — a data-declared
  lumped-parameter thermal network (LPTN) integrated per QSS segment with a zero-allocation,
  A-stable Crank–Nicolson step; pinned ambient, optional coolant node, copper-resistance feedback,
  and a linear 1→0 torque derate across `T_warn→T_max` taken as the min over rated nodes. The PDT
  heat-transfer correlations (air-gap film, end-cavity/shaft convection, Churchill–Chu, Gnielinski)
  are re-authored clean-room from the published forms — a deliberate, narrow amendment of the
  powertrain firewall for the author-owned thermal model only (Decision #25). Two authoring tiers
  (hand-authored *lumped* with mass-heuristic fills, or *detailed* aggregated from a PDT import),
  one integrator. Theory page `docs/theory/machine-thermal.md`.
- **schema/qss**: **battery model + Vdc–SoC coupling** — a Thévenin battery (`battery/1.0`: OCV/R0/
  R1/tau1 vs SoC & temperature) evaluated quasi-statically per segment, with SoC advancing as a
  second slow state alongside the machine temperatures. When a pack is present and the drive-unit
  `.ptm` carries a Vdc axis, the maps (torque, efficiency, and the thermal-loss lookup) are
  evaluated at the pack's SoC-dependent terminal voltage; voltages outside the Vdc grid are
  linearly extrapolated along the Vdc axis with physical floors (τ ≥ 0, 0 < η ≤ 1) and the
  extrapolated band is recorded. The thermal derate and the battery peak-power limit compose as
  `min` caps on the traction ceiling.
- **qss**: **g-g-g-v envelope + Decision #31 corrections** — a T1 trim over the `sim.envelope` grid
  (40×25×7 default, Lovato & Massaro polar form) builds a base table `gg(v, aₓ, g_normal)` stored
  as a `GriddedMapN`; T0 evaluates it, corrected by separable multiplicative sensitivities
  (∂/∂μ_tire, ∂/∂mass, ∂/∂ClA) that are identity at the reference state and CI-gated against full
  T1 re-solves. The envelope is a first-class returnable object. Theory page
  `docs/theory/ggv-envelope.md`.
- **qss/py**: **`sim.tier` dispatch + result surface** — `sim.tier` now selects the lap solver
  (`t0` = point-mass velocity profile on the corrected g-g-g-v envelope; `t1` = the same profile
  plus a per-station re-trim emitting per-wheel loads/slips/forces + setup metrics; `t2`/`t3` return
  a typed "not implemented until M4/M6" error). Machine-thermal derate and battery peak-power now
  compose as `min` caps on the traction ceiling with the machine temperatures and pack SoC advancing
  per segment (the QSS slow-state coupling). The Python xarray Dataset gains a `wheel` dimension
  (FL/FR/RL/RR), per-wheel + slow-state + setup channels, a returnable `lap.envelope`, and `tier`/
  `fz_coupling`/`flat_track` attrs; `s`-only T0 Datasets stay backward-compatible. `solve_lap` /
  `solve_lap_dataset` gain `tier=` and `sim=` arguments.
- **validation**: **flat-track mode + Limebeer cross-check** — a recorded `sim.flat_track` analysis
  mode zeroes track grade/banking/vertical curvature so the envelope collapses to a flat g-g, and
  the transcribed Perantoni & Limebeer 2014 F1 (reference car #1) is cross-checked at Catalunya.
  Per Decision #48 the CI gates what the committed track geometry honestly supports — **top speed
  within 1 %** (87.8 vs ≈88 m/s) and the **slowest-corner apex within 5 %** — while the lap-time
  delta (92.36 s vs the paper's 82.43 s) is *recorded with a term-by-term decomposition, not
  gated*, because a QSS solver on a fixed minimum-curvature line structurally cannot match a
  transient optimal-control lap that co-optimises its own line; the ≤ 1 % lap-time gate moves to
  M4. Comparison figure + golden parquet laps + a QSS-lap perf gate (≤ 50 ms).
- **schema** (`sim/1.0 → 1.1`, MINOR): optional `flat_track` flag on `sim` documents (additive JSON
  Schema change; default `false`).
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
- **schema**: **`emotor/1.1`** (any-N thermal nodes + roles, constant/convection edges, a
  raw-scalar cooling block the assembly expands into convection edges, parametric loss routing),
  **`battery/1.0`** (new — pack config + OCV/R0/R1/tau1 tables), and the **`.ptm` optional `vdc_v`
  axis** (`ptm/1.x` MINOR) plus per-component `loss_breakdown/*` columns. All additive; JSON
  Schemas regenerated from the Rust `schemars` types and the Python mirror updated.
- **tracks**: **TUMFTM importer + 26-circuit library** — `python -m outlap.importers.tumftm_track`
  converts the TU München racetrack-database centre lines to outlap's 8-column format (RIGHT/LEFT
  width columns mapped by name; resampled to a fixed ≈5 m step; flat `z=0`, `banking=0`,
  `grip_scale=1`, `accuracy_class: C`). Vendors 25 circuits (Catalunya, Monza, Spa, Silverstone,
  Suzuka, the Nürburgring **GP** layout, …) alongside the 3D `catalunya_osm` reference. **LGPL-3.0
  data addition**: upstream licence shipped verbatim with the required per-track attribution.
- **vehicles**: **Tesla Model 3 RWD (HV 800 V-class variant study)** — a Model-3-plausible chassis/
  mass/aero road car (≈1765 kg, constant CdA/ClA) on the `ev_1du_rwd` topology, with a **synthetic**
  800 V-class drive-unit stack (three sizings: `du_small`/`du_medium`/`du_large`), an 800 V pack,
  and a hand-authored `.emotor` LPTN — chosen so the Vdc–SoC coupling is live on a road car. Every
  estimated parameter surfaces in the loaded-model report (warning-clean); the real PDT-derived
  imports stay local/untracked (firewall). README documents per-parameter provenance.
- **notebooks**: **`07_qss_t1.ipynb`** — the T1 capstone (trim solve, per-wheel loads on the 3D
  ribbon, setup metrics, the returnable g-g-g-v envelope, machine temperatures) on the F1 car, then
  the Model 3 across the three synthetic drive-unit sizings. CI-executed on synthetic data; the
  real-data twin stays untracked.
- **docs**: **`docs/GUIDE.md`** — a 17-chapter zero-to-hero user guide (architecture, the input
  quartet, file formats, the physics of every tier, the full Python API, importers, the data
  library, validation, worked recipes, limitations, glossary, FAQ). Linked from `README.md` and
  `notebooks/README.md`.

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

[0.2.0]: https://github.com/KMoula30/outlap/releases/tag/v0.2.0
[0.1.0]: https://github.com/KMoula30/outlap/releases/tag/v0.1.0
