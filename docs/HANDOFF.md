# RACESIM PROJECT HANDOFF — Complete Bootstrap Document

> **Purpose of this file.** This is a self-contained engineering handoff for starting a new
> open-source project on a fresh Linux machine. The reader (human or AI assistant) is assumed to
> have **zero knowledge** of the author's prior work, employers, or tools. Everything needed —
> vision, constraints, verified open-source landscape, full system architecture, physics models,
> file-format contracts, the PDT HDF5 importer specification (with the actual file schemas
> documented from inspection), language/tooling decisions, milestones, and validation plan — is in
> this one document. It was produced on 2026-07-02 after a 29-agent research workflow (93 OSS
> projects surveyed, 18 license/activity-verified) plus direct inspection of three real PDT `.h5`
> files, and **updated 2026-07-03 after a 12-question decision round with the author** — see the
> Locked Decisions log at the end of §1; those answers override anything that contradicts them.

---

## Table of Contents

1. [Vision & Hard Constraints](#1-vision--hard-constraints)
2. [Project Name](#2-project-name)
3. [Development Environment (Linux)](#3-development-environment-linux)
4. [Verified Open-Source Landscape & Reuse Policy](#4-verified-open-source-landscape--reuse-policy)
5. [The Whitespace — Why This Project Wins](#5-the-whitespace--why-this-project-wins)
6. [System Architecture](#6-system-architecture)
7. [Physics Models](#7-physics-models)
8. [Powertrain, ERS (2026 Rules), Battery](#8-powertrain-ers-2026-rules-battery)
9. [File Formats — The Product Contract](#9-file-formats--the-product-contract)
10. [PDT HDF5 Importer Specification](#10-pdt-hdf5-importer-specification)
11. [Execution Architecture (Rust Core, Numerics, GPU, WASM)](#11-execution-architecture)
12. [V1 Milestones](#12-v1-milestones)
13. [Validation Plan & Parity Gates](#13-validation-plan--parity-gates)
14. [Testing & CI](#14-testing--ci)
15. [License & Clean-Room Policy](#15-license--clean-room-policy)
16. [Stage 2 Preview — Race Strategy Monte Carlo](#16-stage-2-preview--race-strategy-monte-carlo)
17. [Reading List](#17-reading-list)
18. [First-Week Task List](#18-first-week-task-list)
19. [Appendix A — repo CLAUDE.md](#appendix-a--repo-claudemd) · [Appendix B — CI workflow](#appendix-b--ci-workflow) · [Appendix C — CONTRIBUTING.md](#appendix-c--contributingmd) · [Appendix D — Ubuntu bootstrap commands](#appendix-d--ubuntu-bootstrap-exact-commands-fresh-machine-ssh-nothing-installed)

---

## 1. Vision & Hard Constraints

**What we are building.** An open-source, parametric vehicle simulation foundation for motorsport
and road cars — "anything with 4 wheels": a Formula 1 car, an LMP/GT car, a track-day hatchback —
built from shared foundational building blocks, where a specific car is *pure data*, never code.
On top of it (stage 2, designed-for now, built later): a Monte Carlo race-strategy simulator and
lap-time analysis tools.

**Versatility is the product.** Community adoption hinges on how easily *any* concept can be
expressed and compared. The canonical example (and v1's hero demo): take one EV chassis and
compare optimal laps at a given circuit for a 1-drive-unit RWD car vs 2-DU AWD vs 4-DU with
torque vectoring vs FWD — then swap to a GT car with a big ICE + small electric machine at an
80/20 or 90/10 hybrid split — all through data changes only. This forces the powertrain to be a
**topology graph** (§8.0), not a fixed layout, and every subsystem (tires, aero, dynamics) to be
independently swappable.

**Product philosophy.** V1 ships **feature-rich** — the community's role is next-generation
improvements, new data (tracks, vehicles, tire fits), and bug fixes — not building the foundation.
One experienced simulation engineer builds v1 solo.

**Hard constraints (non-negotiable):**

1. **Conflict-of-interest firewall.** The author works professionally in (a) humanoid robotics
   (actuators, motion control) and (b) electric-powertrain design tooling (electromagnetic motor
   design, drive units, battery packs — the "PDT" toolchain). Therefore this project:
   - **Never designs or models machines electromagnetically.** Powertrains (electric or ICE) enter
     ONLY as torque/speed/efficiency **maps** in a neutral, open file format (`.ptm`, §9).
   - **Never touches actuator/motion-control/robot-dynamics territory.**
   - The author's private tools export to the neutral format; this project only consumes it. A
     PDT→neutral converter ships as an importer (§10) so PDT users can bring their own maps —
     but the importer reads plain HDF5, never imports PDT code.
2. **Strong copyleft — AGPL-3.0** (author's explicit decision, 2026-07-03). The author wants the
   strongest available guarantee that anyone building on or serving this code must publish their
   source; commercial use is welcome but never closed-source. AGPL (not plain GPL) because the
   network-use clause also covers SaaS/web deployments — which matters because the Web UI is the
   declared endgame (constraint 5). Full dependency-compatibility policy in §15.
3. **Languages chosen deliberately to flex engineering range**: Rust systems core, Python user API,
   GPU batch path designed-in, WebAssembly as a first-class target. See §11.
4. **The single canonical vehicle description** feeds every fidelity tier (§6). No per-tier
   re-parameterization, ever.
5. **Web UI is the endgame.** V1 is API/CLI-first, but the WASM build is not a throwaway demo — it
   is the seed of the eventual primary interface (Stage 3, §16). Consequence enforced from day 1:
   core crates stay wasm-clean (no filesystem/threading assumptions in `outlap-core`; IO behind
   traits; schemas losslessly JSON-convertible).

### Locked Decisions log (Q&A with the author, 2026-07-03)

| # | Question | Decision |
|---|---|---|
| 1 | Reference vehicles in v1 | **All four**: configurable EV platform (1/2/4 DU × RWD/FWD/AWD/TV — the hero), F1 2026 hybrid, GT hybrid (big ICE + small EM, split as data), passenger hatchback |
| 2 | Torque-vectoring depth in v1 | **Rule-based TV**: static splits + diff models (open/locked/LSD) + yaw-moment-proportional TV controller with configurable gains; optimal QP allocation deferred (interface designed now) |
| 3 | Quick-start "10-parameter car" mode | **No — full schema only** in v1; a simplified mode can come later if the community wants it |
| 4 | Rain/wet weather | **Deferred to stage 2**; track format already carries `grip_scale` so no format change needed later |
| 5 | Project name | **`outlap`** (crates.io + PyPI verified free 2026-07-02) |
| 6 | Repo visibility | **Public from day 1** |
| 7 | License | **Strong copyleft → AGPL-3.0** (author: "forces disclosure of source, strongest, commercial OK but always open source") |
| 8 | GUI ambition | **Web UI is the endgame** — WASM surface is the seed of the primary interface |
| 9 | Validation data access | **None today** — calibrate/validate from published literature, FastF1 public data, and cross-tool oracles only |
| 10 | Sim-racing telemetry importers | **v1.x, right after 1.0** (community-growth push; also becomes the author's own validation data source) |
| 11 | PDT importers timing | **M3**, with the QSS tier (first consumer of `.ptm` maps) |
| 12 | Time budget | **10–20 h/week** → v1 (M7) plausible in ~6–9 months (revised to ~7–11 months after Decision #13) |

**Second Q&A round (technical depth, 2026-07-03):**

| # | Question | Decision |
|---|---|---|
| 13 | 3D tracks in v1 | **Full 3D in v1** — 3D road frame through ALL tiers (T0 g-g-g-v envelopes with banking/grade/vertical-curvature normal-load effects; T2/T3 transient in the curvilinear 3D road frame). Biggest scope add of the project (~+4–6 wk across M1/M3/M4); also forces DEM fusion in the track importer since no open 3D circuit data exists |
| 14 | Racing line | **Min-curvature generator (QP over lateral offset) + user-supplied lines** in v1; free-trajectory lap-time-optimal OCP deferred |
| 15 | Reference-car data | **Synthetic where needed** — published data where it exists; physically-plausible synthetic aeromaps/K&C clearly labeled SYNTHETIC with generation method documented |
| 16 | Powertrain thermal | ~~Thermal-budget state fitted to 10/20/30 s envelopes~~ **SUPERSEDED by #25** — author's correction: community users won't have overload/continuous envelopes, only peak + losses |
| 17 | Results API | **xarray Datasets** (labeled dims: s/time, wheel, variant, sweep axes; units in attrs) |
| 18 | Design studies | **First-class sweep API** (grid over schema fields → xarray cube, rayon-parallel) + documented cost-function interface + pymoo/optuna example notebook; optimizer itself stays user-side |
| 19 | CLI | **Working CLI**: `outlap lap`, `compare`, `import pdt-*`, `validate`, `migrate` — usable without Python |
| 20 | Bootstrap artifacts | **Appendices A–C in this doc**: repo CLAUDE.md, CI workflow, CONTRIBUTING.md — day 1 is copy-paste |
| 21 | Driver model | **Ideal deterministic only** in v1 (tunable preview/gains as data); skill/noise params arrive with stage 2 |
| 22 | Hero demo (redefined by author) | **Cross-class showcase**: F1 2026-config vs GT hybrid vs EV sports 2-DU AWD (front+rear) vs EV sports 1-DU RWD — **each on its own min-curvature line** + own speed profile. (4-DU TV and FWD remain platform capabilities + example configs, just not the hero four) |
| 23 | Demo circuits | **Catalunya** (forced — fastest-lap validation oracle) + **Spa-Francorchamps** (elevation showcase) + **Silverstone** (flat high-speed control case), all via OSM+DEM import |
| 24 | Post-1.0 integrations | **Gymnasium strategy env** (stage 2) + **FMU/FMI export**. ROS 2 bridge was initially selected, then **explicitly withdrawn by the author** (2026-07-03) — do not add it; it is also the firewall-riskiest of the three |

**Third Q&A round (programming architecture, style, physics/math, 2026-07-03):**

| # | Question | Decision |
|---|---|---|
| 25 | Machine thermal (supersedes #16) | **`emotor.yaml` per machine, an N-node lumped-parameter thermal network (LPTN)**, driven by the `.ptm` loss maps; node temps → derating. **AMENDED 2026-07-05 (author-authorized, supersedes the 2-node/"NOT PDT-grade" wording):** the network is now *any-N* and outlap **builds** the conductance operator from machine internals for the detailed path — the PDT heat-transfer correlations (air-gap film, end-cavity/shaft convection, liquid-jacket channel) are **ported into `outlap-thermal`** and evaluated per segment at the shaft speed and node temperatures. This is a deliberate, narrow reversal of the powertrain firewall (hard rule #1) for the (open-sourced, author-owned) thermal model only. Two authoring tiers share one Crank–Nicolson integrator: **lumped** (a hand-authored reduced node menu — winding/stator-iron/rotor/housing/coolant/ambient — with mass-heuristic-filled capacities/conductances, flagged as estimates; constant `G`) and **detailed** (the full FEA node set with explicit capacities + convection edges, from a PDT import). Loss rule: the `.ptm` supplies the total machine-heating loss; ≥1 node route is required, and whatever total is not routed lands on the winding node. See §8.5, §9.5, §10.2 |
| 26 | Model composition | **Runtime, data-driven** — one binary loads any vehicle.yaml; blocks assembled + topo-sorted at load; enum dispatch in the loop (required by "car = pure data" + WASM story) |
| 27 | Errors/panics | **Typed (thiserror) + panic-free core**: all fallible APIs return typed errors; kernels never panic; `debug_assert!` for physics invariants; anyhow only in CLI edges |
| 28 | Lint strictness | **Strict**: clippy::pedantic baseline (curated allow-list), `deny(missing_docs)` on pub items, `forbid(unsafe_code)` everywhere except the C-ABI/FFI crate, rustfmt defaults |
| 29 | 7-DOF Fz algebraic loop | **User-selectable solver setting**: `one_step_lag` (default) or `fixed_point` (2–3 damped iterations) — per the author, both ship in v1 as a simulation setting |
| 30 | Map interpolation | **Monotone cubic Hermite (Fritsch-Carlson), C¹**, one shared implementation for all gridded maps; analytic derivatives for Newton solvers |
| 31 | T0 envelope vs slow states | **Base table gg(v, ax, g_normal) + separable multiplicative corrections** from T1 sensitivities (μ_tire, mass, ClA); validated against full T1 re-solves in CI |
| 32 | EOM verification | **SymPy derive + verify**: docs/derivations notebooks derive 7/14-DOF EOMs symbolically; CI evaluates symbolic vs Rust RHS at random states, agreement to 1e-12 |
| 33 | Symbol naming | **Hybrid**: descriptive names at public APIs; paper symbols inside math kernels with doc-comment headers citing equation numbers (e.g. "Pacejka 2012 eq. 4.E19–4.E30") |
| 34 | Python tooling | **Strict modern**: uv, ruff (lint+format), pyright strict, full type hints on public API, pydantic v2 models validating against the JSON Schemas **generated from the Rust schemars types** (single source of truth) |
| 35 | Overrides/variants | **Dotted paths + YAML overlays**: programmatic sweeps via dotted-path dicts; named variants via deep-merged overlay files, schema-validated after merge |
| 36 | Git/release workflow | **Trunk + short-lived PRs (CI-gated even solo) + Conventional Commits**; tag + GitHub release + changelog per milestone (git-cliff/release-please) |

**Fourth Q&A round (block-architecture backbone, 2026-07-03) — final:**

| # | Question | Decision |
|---|---|---|
| 37 | Extensibility model | **Hybrid**: built-in model variants as enums in core (zero-cost dispatch, curated); exactly THREE designed plugin points — Rust trait + compile-time registration (custom blocks), the C-ABI tire interface, swappable controllers. Good community models get upstreamed |
| 38 | Controllers | **First-class swappable blocks (sense → control → actuate → integrate step phases), Rust/C-ABI ONLY** — no Python controller callbacks, ever; the "no Python in a timestep" rule is absolute |
| 39 | Signal bus | **Hybrid**: fixed core signal set with compile-time indices (hot path) + dynamic named-channel region for plugins/logging, string keys interned to indices once at assembly |
| 40 | Unsupported combos | **Strict + explicit opt-out**: hard error with actionable message; documented-fallback combos run only with `allow_degraded: true` → warning + degradation recorded in result metadata |
| 41 | Presets/defaults | **Presets + `extends:` + labeled estimation**: shipped class presets as data (formula_base, gt_base, passenger_base); deep-merge with post-merge validation; missing derivable params filled by documented heuristics, every estimated value listed in the loaded-model report — nothing silent |
| 42 | Sim settings | **Optional `sim.yaml`** (fourth… third input) with full defaults; CLI/API override file values; RESOLVED settings embedded in every result artifact |
| 43 | Diagnostics | **Rich**: miette-style YAML source spans, did-you-mean field suggestions, unit sanity checks, plain-language topology errors, `outlap validate --explain` — treated as the #1 user-friendliness lever |
| 44 | Programmatic input | **Files + in-memory objects**: every path-accepting API equally accepts the validated object (pydantic/dict in Python, serde struct in Rust); identical provenance hashing |
| 45 | Slipstream/dirty air | **Stage-2 empirical** (drag/downforce deltas vs gap scaling the aero map); v1 strictly single-car; no co-simulated wake, no multi-car state layout tax |
| 46 | Session conditions | **Fourth input `conditions.yaml`**: air temp/pressure→density, constant wind vector (v1), track surface temp, thermal ambient; full ISA defaults. The input quartet: **vehicle + track + conditions + sim** |
| 47 | Solid axle / karts | **`type: solid` in the axle/diff block from day 1** (locked-diff limit case, nearly free); actual kart reference car (frame-flex) is post-1.0/community territory |
| 48 | Limebeer gate re-scope (author-decided 2026-07-06, M3/PR8+9) | The §13 "lap time ≤1%" row compared a **QSS solver on a fixed heuristic line** against a **transient OCP that co-optimises the driven line** — unattainable by construction (PL2014 itself cites a 2.19 s QSS-vs-OCP gap at Barcelona, its ref [14]; measured floor for this solver class ≈ +5–8% once car and geometry are validated). **Re-scoped:** the M3 QSS gate hard-gates what the tier can honestly certify — top speed ≤1% and slow/fast-corner apex-speed bands ≤5% vs the PL2014 published traces; the lap-time delta is **recorded with its decomposition** in `docs/validation/`, not gated. The ≤1% lap-time ambition moves to **M4** via the honest chain (QSS↔T2 parity ≤0.3%, then T2 vs the OCP oracle). A **time-weighted raceline QP** (the dominant recoverable share of the gap) is scheduled as M4 work alongside the transient tier — a validation-motivated amendment to Decision #14's "min-curvature only in v1" scope |

---

## 2. Project Name

Availability checked 2026-07-02 (GitHub search, crates.io API, PyPI API):

| Candidate | GitHub | crates.io | PyPI | Verdict |
|---|---|---|---|---|
| **outlap** | ~clean (one 1★ fantasy-sport dashboard) | AVAILABLE | AVAILABLE | **Recommended** |
| apexsim | a few 0-1★ toy repos | AVAILABLE | AVAILABLE | Good fallback |
| open-race-sim | clean | AVAILABLE | AVAILABLE | Generic but descriptive |
| ovro (open-vehicle-racing-optimizer) | **collides**: Owens Valley Radio Observatory ecosystem (121 repos) | available | available | Avoid |
| undercut | **collides**: undercut-f1 (896★ F1 timing TUI) | available | available | Avoid |
| racelab | scattered | available | **taken** | Avoid |

**DECIDED (2026-07-03): `outlap`.** The out-lap is where tire temperature, fuel, and strategy all
converge before a flying lap; it is short, motorsport-native, and unclaimed on both package
registries. Repo description carries the descriptive long name:
*"outlap — open vehicle racing simulator & strategy optimizer"*. Register the crates.io and PyPI
names with placeholder 0.0.1 releases early (name-squatting insurance); repo is **public from
day 1**.

---

## 3. Development Environment (Linux)

**DECIDED: the author's existing Ubuntu 24.04 desktop is the dev machine.**

```
OS:  Ubuntu 24.04.4 LTS x86_64 (kernel 6.17)     RAM: 16 GB
CPU: Intel i5-6500 (4 cores) @ 3.6 GHz           GPU: NVIDIA GTX 1060 3GB
```

- This is fine for v1 development: the core is CPU-light during development, the GTX 1060 runs
  Vulkan/wgpu for later GPU experiments, and CI parity holds (GitHub Actions `ubuntu-latest`
  runners are Ubuntu — where the code is *developed* doesn't matter to CI, but local == CI
  eliminates "works on my machine").
- **Known limitation**: 4 cores means batch benchmarks run ~4× slower than a modern desktop.
  Treat this box as the *dev* machine; nominate a faster machine later as the documented
  "benchmarks-of-record" hardware (BENCHMARKS.md per release).
- **HARD RULE: never develop this project on the work laptop** (no dual-boot, no repo checkout).
  Personal OSS on employer-adjacent hardware undermines the §1 conflict firewall and risks the
  machine the author earns with. This is part of the firewall, not a convenience choice.

The toolchains are distro-agnostic anyway (`rustup` for Rust, `uv` for Python, `maturin` for
wheels); Ubuntu LTS just optimizes for boring stability + driver support.

Setup script (run once on the fresh machine):

```bash
# --- system toolchain ---
sudo apt update && sudo apt install -y \
  build-essential git curl pkg-config cmake \
  libssl-dev \
  mesa-vulkan-drivers vulkan-tools \
  libhdf5-dev  # optional: only for h5 CLI inspection tools

# --- Rust (rustup, NOT the distro package) ---
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
rustup component add clippy rustfmt
rustup target add wasm32-unknown-unknown
cargo install wasm-pack cargo-criterion iai-callgrind-runner maturin

# --- Python (uv manages everything; do not use distro python for the project) ---
curl -LsSf https://astral.sh/uv/install.sh | sh
uv python install 3.12

# --- repo bootstrap ---
git init outlap && cd outlap
cargo new --lib crates/outlap-core
uv init --package python/outlap
```

Sanity checks: `cargo --version`, `vulkaninfo --summary` (wgpu backend present),
`uv run python -c "import sys; print(sys.version)"`.

---

## 4. Verified Open-Source Landscape & Reuse Policy

All licenses and activity verified against the actual repos on 2026-07-02.

> **License directionality under AGPL-3.0 (our license, §15):** permissive code (MIT/Apache/BSD/
> Zlib) flows INTO an AGPL project freely — every dependency in §4.1 remains fully usable. The
> LGPL-3.0 wall that existed under the original permissive plan **drops**: LGPL libraries are now
> legally usable as dependencies too. We still *prefer* re-implementing core algorithms in Rust
> from the papers — for quality, integration, and because the flagship contributions must be our
> own — but the constraint is now engineering judgment, not law.

### 4.1 Dependencies (permissive, actively maintained — link/depend directly)

| Project | License | Role |
|---|---|---|
| [diffsol](https://github.com/martinjrobins/diffsol) | MIT | Rust ODE/DAE solvers: variable-order BDF, ESDIRK (TR-BDF2), events, sensitivities, C API. **Verification integrator** (production loop uses our own fixed-step scheme, §11.2). Risk: mostly one author — pin versions |
| [nalgebra](https://github.com/dimforge/nalgebra) | Apache-2.0 | Linear algebra substrate (faer as the heavy-duty alternative; diffsol supports both) |
| [rayon](https://github.com/rayon-rs/rayon) | MIT/Apache | CPU batch parallelism |
| [PyO3](https://github.com/PyO3/pyo3) + [maturin](https://github.com/PyO3/maturin) | MIT/Apache | Python bindings + abi3 wheel publishing (the polars/FASTSim pattern) |
| serde, serde_yaml, schemars, arrow/parquet crates | MIT/Apache | Config formats + sidecar tables |
| [wgpu](https://github.com/gfx-rs/wgpu) / [CubeCL](https://github.com/tracel-ai/cubecl) | MIT/Apache dual | **Later** GPU tier; CubeCL keeps kernels in Rust, targets CUDA/ROCm/Vulkan/WebGPU. Not in v1 |
| [OpenCRG C library](https://github.com/asam-ev/OpenCRG) | Apache-2.0 | Later: road-surface grids under the tire thermal model |
| [libOpenDRIVE](https://github.com/pageldev/libOpenDRIVE) | Apache-2.0 | Later: `.xodr` track import path |

### 4.2 Formulation references (permissive — port the math, cite; don't embed the project)

| Project | License / Status | What to take |
|---|---|---|
| [Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics) (TUMFTM) | Apache-2.0, active (v2.0.0 2026) | The best racing-validated (Indy AV21) modern-C++ double-track state-space formulation + composable-submodel architecture. Port the model structure to Rust |
| [Project Chrono / Chrono::Vehicle](https://github.com/projectchrono/chrono) | BSD-3, very active | **Cross-validation oracle** for chassis/suspension/tire baselines; JSON-parametric vehicle template design as inspiration. Too heavy to embed |
| [fastest-lap](https://github.com/juanmanzanero/fastest-lap) | MIT, dormant since 2023 | **Lap-level validation oracle**: its Limebeer-2014 3-DOF F1 results at Catalunya are our golden numbers. Also the only permissive OCP lap-sim worth reading |
| [thevenin](https://github.com/NREL/thevenin) (NREL) | BSD-3, active | Battery equivalent-circuit model (N×RC pairs, SOC/T-dependent params, hysteresis) — port to Rust (~300 lines), validate against it |
| [uwsbel/low-fidelity-dynamic-models](https://github.com/uwsbel/low-fidelity-dynamic-models) | MIT | Proof pattern: 18/24-DOF vehicle + TMeasy on GPU, ~300k vehicles real-time on an A100. Architecture inspiration for the batch tier |
| CommonRoad vehicle models (TUM, gitlab.lrz.de) | BSD-3 | Fully documented benchmark equations: point-mass → single-track → multi-body, with published parameter sets (BMW 320i etc.) — free parity targets |
| [Drake](https://github.com/RobotLocomotion/drake) / [MuJoCo](https://github.com/google-deepmind/mujoco) | BSD-3 / Apache-2.0 | API/design patterns only (systems framework, event handling). No tire/road abstractions — not vehicle-usable directly |

### 4.3 LGPL/GPL — now license-compatible; re-implement anyway where it's core IP

| Project | License | Policy |
|---|---|---|
| [TUMFTM race-simulation](https://github.com/TUMFTM/race-simulation) (Heilmeier et al.) | LGPL-3.0, frozen 2023 | The only serious OSS Monte Carlo race-strategy sim. Now legally readable/usable, but **still re-implement from the Heilmeier 2020 papers** — it's lap-discrete Python calibrated to F1 2014-2019; our time-discrete Rust + physics coupling supersedes it. Fine to read its source to resolve paper ambiguities |
| [TUMFTM trajectory_planning_helpers](https://github.com/TUMFTM/trajectory_planning_helpers) | LGPL-3.0 | The ggv + forward/backward velocity-profile solver. Usable as a Python-side dependency for cross-checking; the Rust production implementation is written from the formulation (textbook algorithm) |
| [TUMFTM laptime-simulation](https://github.com/TUMFTM/laptime-simulation), [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization), [velocity_optimization](https://github.com/TUMFTM/velocity_optimization), [TUMRT online_3D_racing_line_planning](https://github.com/TUMRT/online_3D_racing_line_planning) | LGPL-3.0 | Formulation references for QSS, raceline optimization, 3D gg-g-v diagrams (Lovato/Massaro polar method); may now also serve as executable cross-validation oracles in CI (run as external tools) |
| [OpenLAP](https://github.com/mc12027/OpenLAP-Lap-Time-Simulator) | GPL-3.0, MATLAB | Concept reference (its OpenTRACK segment model with elevation/banking/grip factors is a good format idea) |
| Speed Dreams SimuV4/V5, VDrift | GPL | The *only* OSS code with any tire temperature/degradation at all. **Do not derive our tire models from game-engine source regardless of compatibility** — the thermal/wear stack is the flagship and must be independently authored from the literature (§15); game heuristics would contaminate provenance and quality |
| [magic-formula-tyre-library](https://github.com/teasit/magic-formula-tyre-library) (MATLAB) | GPL-3.0 | Our MF6.1 comes from the Pacejka book directly; this may serve as a numerical cross-check oracle |

### 4.4 Data sources (for validation & reference vehicles)

| Source | License/Terms | Use |
|---|---|---|
| [FastF1](https://github.com/theOehrly/Fast-F1) | MIT | F1 timing/telemetry (~4 Hz interpolated; boolean brake; no tire temps). **Calibrate the wear model from stint pace deltas**; validate strategy sim |
| [jolpica-f1](https://github.com/jolpica/jolpica-f1) | Apache-2.0 | Ergast-successor API: results, pit stops, lap times to 1950 |
| [TUMFTM racetrack-database](https://github.com/TUMFTM/racetrack-database) | LGPL-3.0 (data) | 25 circuit centerlines+widths — the standard academic dataset. Under our AGPL license it is now redistributable (files keep their LGPL notice, `data/third_party/`); great **bootstrap** data for week-one T0 laps. Limits: strictly 2D (no elevation/banking — insufficient for our full-3D v1), smoothed centerlines, frozen since 2021 → the OSM+DEM importer (§9.3) remains the primary track source |
| [f1-circuits](https://github.com/bacinger/f1-circuits) | MIT | GeoJSON circuit outlines |
| Perantoni & Limebeer 2014, *Optimal control for a Formula One car with variable parameters* (VSD 52(5), open-access manuscript) | facts/citation | **Complete published F1 parameter set**: mass/inertia, speed-dependent aero maps, MF tire coefficients, powertrain. Reference car #1 |
| TUMFTM [sim_vehicle_dynamics](https://github.com/TUMFTM/sim_vehicle_dynamics) (Roborace) | LGPL | Published MF5.2 racing-tire parameter values (reused by Open-Car-Dynamics under Apache — take from there) |
| FIA/SRO GT3 BoP tables (public PDFs) | factual data | Mass/power fragments for a GT3-like reference car |
| EPA ALPHA published engine fuel maps | US-gov | Road-car ICE fuel-consumption maps for the passenger-car reference |
| FSAE TTC (Tire Test Consortium) | **membership-locked, NON-redistributable** | We ship the `.tir` parser + fitting pipeline so members fit locally. We NEVER redistribute TTC data or TTC-derived parameter sets |

---

## 5. The Whitespace — Why This Project Wins

Verified conclusions from the landscape sweep (each independently confirmed by 2+ research agents):

1. **No open-source tire thermal + wear/degradation model exists, anywhere, in any language.** The
   physics is published (Farroni TRT/TRT-EVO ring models; TameTire papers; Archard/frictional-energy
   wear laws) but every implementation is proprietary (MegaRide thermoRIDE/WeaRIDE, Michelin
   TameTire, FTire add-ons) or a dead 0-star repo. OSS uniformly stops at Pac89/Pac02/TMeasy with
   time-invariant grip. **→ Flagship contribution (§7.2).**
2. **No complete open MF6.1/6.2 outside MATLAB**; no pip-installable maintained MF package; **no
   Rust implementation of any tire model at all**.
3. **No credible Rust vehicle-dynamics crate exists.** The language niche is vacant; the substrate
   (diffsol/nalgebra) is ready.
4. **No open "race car as data" schema** (suspension + aeromap + tires + powertrain); every project
   invents its own. Defining one = chance to become the standard (§9).
5. **No open ride-height/rake-dependent aero-map representation** for ground-effect cars.
6. **No open ERS/hybrid race powertrain** (deployment strategies, energy limits) coupled to chassis
   dynamics; battery models exist (PyBaMM/thevenin) but uncoupled from vehicle simulation.
7. **Physics ↔ strategy is completely unconnected in OSS**: race-strategy sims use empirical
   lap-time-delta degradation; physics engines have no wear states. Our stage-2 thesis occupies
   exactly this seam.
8. **No open 3D racetrack format or dataset** (elevation+banking+grip); the academic standard is 2D
   and frozen since 2021.
9. **No batch/GPU story in any motorsport OSS tool**; nothing browser-runnable (WASM/WebGPU demo =
   cheap differentiator).

---

## 6. System Architecture

### 6.1 Core invariant: one vehicle description, four derived views

There is **one canonical parameter set** per vehicle (§9). Lower-fidelity tiers are **derived at
runtime by evaluating the same objects** — parity between tiers is a *solver property* (testable in
CI), not a data-entry discipline.

```
                 vehicle.yaml (+ track.yaml, *.tyr.yaml tires, *.ptm.yaml powertrain maps)
                                          │
     ┌──────────────────┬─────────────────┴──────────────────┬───────────────────────┐
T0: point-mass       T1: QSS trim solver               T2: transient 7-DOF       T3: transient 14-DOF
lap solver on        (double-track equilibrium         double-track ODE +        (adds heave/pitch/roll
gg-g-v envelope      → generates the gg-g-v            tire relaxation +         + 4 unsprung DOF →
(<50 ms/lap;         envelope + setup metrics:         slow states               dynamic ride height →
strategy inner       understeer gradient, aero         (the batchable            ground-effect aero;
loop)                balance vs speed)                 workhorse)                the "downforce car is
                                                                                 real" tier)
```

- **T0** — forward/backward velocity-profile solver (TUM `calc_vel_profile` formulation,
  re-implemented from the papers) on a spline track. **Full 3D (Locked Decision #13)**: the track
  is a 3D ribbon (curvature κ(s), grade, banking, vertical curvature); envelopes are
  **g-g-g-v** (Lovato/Massaro polar form) — the apparent-gravity/normal-load axis captures
  banking load, crest unloading, and compression (Eau Rouge). Constraint envelope:
  `gg(v, ax, g_normal | ride_heights, T_tire, wear, fuel_mass)` — **the tire-state axes are the
  differentiator**: strategy-tier laps see physical degradation.
- **T1** — for each (v, ay, ax): damped-Newton solve of the algebraic trim: unknowns
  z = [steer δ, sideslip β, yaw rate r = κv, throttle/brake split, 4×Fz]; equations = X/Y/N force-moment
  balance + quasi-static lateral/longitudinal load transfer (geometric via roll-center heights +
  anti-effects; elastic via roll-stiffness distribution) + aero-platform equilibrium (ride heights
  from wheel rates + aero loads, iterated against the aero map).
- **T2** — states [s, n, ψ_rel, vx, vy, r, ω₁..₄] in the **curvilinear 3D road frame** (position
  along track s + lateral offset n; road banking/grade rotate the gravity and load vectors) + tire
  relaxation states + slow states (below). Load transfer algebraic (same expressions as T1).
  Smooth ODE, no contact solver → GPU-batchable.
- **T3** — adds sprung heave/pitch/roll (z, φ, θ + rates) + 4 unsprung vertical DOF; nonlinear
  spring/damper tables, bumpstops, ARBs; kinematic camber/toe vs travel from K&C tables. Needed
  because pitch-under-braking → aero-balance shift is *the* defining downforce-car behavior.
- **Later (design-for, don't build):** full-multibody adapter to Chrono::Vehicle consuming the same
  schema; MF-Swift-style rigid-ring tire; OpenCRG surfaces.

**Slow vs fast state split** (used by every tier):
- *Fast*: chassis velocities, wheel speeds, tire relaxation, actuator lags.
- *Slow*: tire surface/carcass/gas temperatures, tread wear, thermal damage, brake disc temps, fuel
  mass, battery SOC + temperature.
- In T0/T1 (QSS) the slow states integrate **segment-to-segment with explicit Euler over the
  quasi-static solution** — this is what makes the QSS tier *stint-capable* (unique in OSS).

### 6.2 Block abstraction

A **Block** = (immutable parameters, states, typed ports on a flat struct-of-arrays signal Bus).
Blocks declare port reads/writes; the assembler topologically sorts once at model build — no
runtime graph, no virtual dispatch in the inner loop (enum dispatch).

```rust
trait Block {
    /// Tier T1/T0: algebraic equilibrium at a trim point
    fn equilibrium(&self, bus: &mut Bus, slow: &SlowState);
    /// Tier T2/T3: RHS evaluation
    fn derivatives(&self, x: &StateView, bus: &mut Bus, dx: &mut DerivView);
    /// Both tiers: slow-state evolution (thermal, wear, SOC, fuel)
    fn slow_derivatives(&self, bus: &Bus, dslow: &mut SlowDerivView);
}
```

Block set: `Chassis` (7/14-DOF variants), `Tire` ×4, `Aero`, `Suspension` (lumped K&C), `Brakes`,
`Ice`, `ElectricMachine`, `Gearbox`, `EnergyStore`, `EnergyManager`, `Driver`. **F1 vs hatchback is
pure data** — same blocks, different parameter files; absent subsystems (ERS on a hatchback) simply
don't instantiate.

### 6.2b The configuration backbone (Locked Decisions #37–47) — how variety stays fast AND friendly

**The input quartet:** every run = `vehicle.yaml + track.yaml + conditions.yaml + sim.yaml`
(the last two optional, fully defaulted). Car identity, road, environment, and numerics never mix.

**The assembly pipeline** (runs once per model load, never in the loop):

```
parse (all referenced files)                         # serde/pydantic, schema-versioned
  → resolve `extends:` preset chains (deep merge)    # class presets shipped as data
    → validate post-merge (JSON Schema + semantics)  # rich diagnostics: miette spans,
      → estimate missing derivables (documented      #   did-you-mean, plain-language
        heuristics) → LOADED-MODEL REPORT            #   topology errors (#43)
        → build drivetrain topology graph, check     # strict; `allow_degraded: true`
          (reachability, ratio conflicts, tier       #   is the only escape hatch, and
          capability match) (#40)                    #   degradations land in result meta
          → assemble blocks, topo-sort, intern       # hybrid bus: fixed core indices +
            dynamic bus channels (#39)               #   interned plugin channels
            → immutable CompiledVehicle              # hot loop sees only this
```

After assembly the hot loop touches **zero** strings, hashes, or config logic — variety is paid
for entirely at load time. The loaded-model report (what was inherited, what was estimated, what
was degraded) prints with every run and embeds in artifacts: *nothing silent*.

**Step phases:** `sense → control → actuate → integrate`. Controllers (TV, ERS deployment, brake
bias, shift logic) are first-class swappable blocks running in the `control` phase on the same
bus — **Rust or C-ABI only (#38); no Python inside a timestep, ever**. Experimentation with
custom control strategies happens by writing a Rust controller block (plugin point) or
pre-computing control schedules `u(s)` as data.

**Exactly three plugin points (#37)** — everything else is core enums (fast, curated):
1. Custom blocks: Rust trait + compile-time registration (a plugin crate depends on `outlap-core`,
   registers its blocks; users build a custom binary or the project upstreams the block).
2. Tire models: the stable C-ABI "Standard Tire Interface" (CPU-only by contract).
3. Controllers: same trait mechanism, `control`-phase blocks.

**Programmatic use (#44):** everything that accepts a file path accepts the equivalent validated
in-memory object; sweeps use dotted-path overrides (#35); optimizers never touch the filesystem.

### 6.3 Racing line (Locked Decision #14)

V1 ships a **minimum-curvature line generator**: QP over the lateral offset n(s) within track
bounds minimizing ∫κ² (TUM-style formulation, re-implemented from the papers), solved on the 3D
ribbon. Users can also supply their own line (`raceline.csv`, same s-based format). Every lap
result records which line it ran. Free-trajectory lap-time-optimal line+speed co-optimization
(collocation OCP) is deferred post-v1 — the min-curvature line is the fair common denominator for
comparisons (each vehicle variant gets its own generated line; see the hero demo, §12).

---

## 7. Physics Models

### 7.1 Tire force backbone

- **MF6.1** (Pacejka 2012, incl. Besselink inflation-pressure terms), clean-room from the book:
  steady-state Fx, Fy, Mz, Mx, combined slip via cosine weighting; turn-slip omitted in v1.
- **Transient**: first-order relaxation per slip channel, σ_κ κ̇ + |vx| κ = |vx| κ_ss (same for α);
  σ from PTX/PTY coefficients or Fz-dependent carcass stiffness.
- **Brush model** (5 params: Cκ, Cα, μ0, patch length, pressure profile) ships as the low-data tier
  for passenger cars / users without `.tir` files, and as the physical scaffold the thermal model
  hooks into identically.
- `.tir` parser/writer + scipy-based fitting pipeline (TTC-format ingestion for members) in the
  Python layer.

### 7.2 Tire thermal ring model — FLAGSHIP (per tire, 3+1 nodes, reduced Farroni-TRT)

States: **T_s** (tread surface), **T_c** (tread bulk/carcass), **T_g** (inflation gas); rim as
parameter or optional 4th node.

```
C_s·dT_s/dt = Q_fric − G_sc(T_s−T_c) − h(v)·A_ext·(1−a_cp)·(T_s−T_air) − G_road·a_cp·(T_s−T_road)
C_c·dT_c/dt = Q_hyst + G_sc(T_s−T_c) − G_cg(T_c−T_g)
C_g·dT_g/dt = G_cg(T_c−T_g) − G_gr(T_g−T_rim)
```

Drivers:
- Friction power `Q_fric = p_t·(|Fx·v_sx| + |Fy·v_sy|)`, sliding velocities from slip; partition
  p_t ≈ 0.6–0.7 into the tread (rest to road).
- Hysteresis `Q_hyst = c_h·Fz·δ_tire(Fz,p)·Ω` (deflection-rate/strain-energy-loss form; c_h fit to
  rolling-resistance data).
- Convection `h(v) = h₀ + h₁·v^0.8`; contact-patch fraction `a_cp = A_cp(Fz,p)/A_ext`.

Couplings back to the force model:
1. Gas law: `p = p_cold·T_g/T_cold` → MF6.1's native pressure terms (stiffnesses, μ, patch size).
2. Grip window: `λ_μ(T_s) = exp(−c_T·((T_s−T_opt)/T_opt)²)` scaling LMUX/LMUY (asymmetric option:
   separate cold/hot widths).
3. Carcass softening: PKY1/PKX1 × (1 − k_c(T_c − T_c,ref)).

### 7.3 Wear / degradation law — FLAGSHIP (two states)

- **Tread depth w** [mm]: Archard frictional-power form `dw/dt = (k_w / H(T_s)) · Q_fric / A_cp`,
  hardness H decreasing with T_s (hot tires wear faster). Effects: μ multiplier
  `f_w = 1 − c_w1·(w/w_max)`; **reduced tread mass → C_s(w) shrinks → worn tires run hotter — the
  physical positive-feedback cliff mechanism**.
- **Thermal damage D ∈ [0,1]** (irreversible): `dD/dt = (1/τ_D)·⟨(T_c−T_deg)/ΔT_ref⟩₊^β`
  (Arrhenius-like devulcanization proxy).
- **Total grip factor with cliff**:
  `λ_μ,total = λ_μ(T_s) · f_w · (1 − Δ_c·σ((w−w_c)/s_w)) · (1 − Δ_D·D)` — sigmoid σ gives a sharp
  but C¹ pace collapse at critical wear w_c.
- **Calibration**: thermal params from Farroni's published values scaled by tire size; T_opt/c_T per
  compound class; k_w and w_c calibrated *inversely* from FastF1 stint pace data (reproduce
  ~0.05–0.10 s/lap compound decay and observed cliff laps).

Everything above is implementable from public literature — no proprietary math (§15).

### 7.4 Aero

Map object `{C_z,front, C_z,rear, C_x} = f(h_front, h_rear, yaw [, roll, DRS_flag])` — gridded
lookup + monotone-regularized fit, evaluated at dynamic ride heights (T3) or equilibrium ride
heights (T1/T2). Yaw sensitivity makes the gg-diagram asymmetric mid-corner. Passenger car
degenerates to constant CdA/ClA. This is the first open ride-height aero-map representation (§5.5).

### 7.5 Suspension (v1 = lumped K&C, not hardpoints)

Per axle: ride rate, roll-stiffness share, roll-center height (geometric transfer), anti-dive/
anti-squat. Per corner: camber-vs-(heave,roll) and toe-vs-(heave, Fy, Mz) tables (kinematic +
compliance steer). Motorsport-credible because it reproduces load-transfer distribution and
camber/toe trajectories that dominate handling. A hardpoint→K&C preprocessing tool is a later
community-sized project (OSS gap #8).

### 7.6 Brakes

Pedal → total torque via balance bar (+ dynamic bias / regen blending with the MGU-K). Per-corner
disc thermal node `C_d·dT_d/dt = T_br·ω − h_d(v)·A_d·(T_d−T_air)`; pad fade `μ_pad(T_d)` table.
Simple slip-limit ABS flag for road cars.

### 7.7 Driver model (T2/T3)

Two loops:
- **Steering**: MacAdam-style preview point(s) on the target line + curvature feedforward
  `δ_ff = κ(L + K_us·v²)`.
- **Speed**: PI tracking of the **T0/T1 QSS speed profile** with gg-headroom feedforward, plus
  lift-and-coast and ERS-deployment inputs from the energy manager.

Using the QSS profile as the transient driver's reference makes tier parity a built-in regression
test.

---

## 8. Powertrain, ERS (2026 Rules), Battery

All power-producing hardware enters as **maps** (§1 firewall). Blocks and states:

### 8.0 Drivetrain topology graph (the versatility backbone)

**The powertrain is a directed graph, not a fixed layout**: torque **sources** (ICE, electric
machines / drive units) connect to wheel **sinks** through **coupler** elements (gearbox, clutch,
fixed ratio, differential, direct per-wheel). Any 4-wheeled concept is a topology + data:

| Concept | Topology |
|---|---|
| 1-DU RWD EV | DU → open/LSD diff → RL+RR |
| 2-DU AWD EV | DU_f → front diff → FL+FR; DU_r → rear diff → RL+RR (front/rear split = data or controller) |
| 4-DU torque-vectoring EV | DU×4 → one wheel each (TV controller allocates) |
| FWD hatchback | ICE → gearbox → front diff → FL+FR |
| GT hybrid 80/20 | ICE → gearbox → rear diff → RL+RR; EM (P2 or axle) in parallel — split ratio is data |
| F1 2026 | ICE + MGU-K on the same shaft → gearbox → rear diff → RL+RR |

Schema: `drivetrain.units[]` each declare `{source: <.ptm ref>, path: [couplers...], wheels: [...]}`.
The assembler validates the graph (every wheel reachable, no ratio conflicts) at load time.

**Control layer (v1 = rule-based, per Locked Decision #2):**
- Static split ratios (front/rear, left/right) as data.
- Differential models: open, locked, LSD (preload + ramp), **solid** (kart/historic solid axle —
  locked-diff limit case, Decision #47) — enters the double-track torque split.
- **Torque vectoring**: yaw-moment-proportional controller — `ΔM_z = K_p·(r_target − r)` with
  `r_target = v·κ_ref` (or steady-state yaw gain), allocated across available per-wheel sources
  within friction-ellipse and machine-envelope limits; gains are vehicle data.
- Regen/friction brake blending hooks into the same allocator (§7.6).
- The allocator interface is designed so a QP-based optimal allocation (per-wheel torque over
  friction ellipses) can replace the rule-based one post-v1 without touching the topology graph.

**Hero demo (ships with v1, M7):** one EV chassis, four drivetrain files (1-DU RWD / 2-DU AWD /
4-DU TV / FWD), same track → compared optimal laps + energy consumption in one notebook.

### 8.1 ICE
- Torque map T(n, throttle); fuel-flow map ṁ_fuel(n, T) (or BSFC map).
- State: fuel mass (feeds vehicle mass & CG migration).
- Optional fuel-flow-limit constraint (F1-style ṁ_max) as config.

### 8.2 Gearbox / driveline
- Ratios + final drive, efficiency map or constant, shift time with torque interruption (discrete
  event + small state machine: torque-cut timer → ratio swap → clutch ramp).
- Differential: open/locked/LSD preload+ramp as v1 options (enters the double-track torque split).

### 8.3 ERS — 2026-Formula-1-style, MGU-K ONLY (no MGU-H, per current regulations)

**Design decision (2026-07-02): the MGU-H is removed from the architecture entirely.** The 2026 F1
power-unit regulations deleted it; for any non-F1 car it never existed. What remains is one
electric machine on the crank/axle (MGU-K) + an energy store, with *deployment/recovery rules as
data*:

```yaml
ers:
  mgu_k: mguk.ptm.yaml            # torque/speed/efficiency map, bidirectional
  es:                              # energy store limits (battery physics lives in §8.4)
    capacity_MJ: 4.0
    soc_window: [0.1, 0.9]
  deployment:
    power_limit_kW: 350            # 2026-class MGU-K electrical power (VERIFY vs FIA regs)
    taper_vs_speed:                # deployment de-rate at high speed (2026 mechanism)
      speed_kph:  [0, 290, 345]
      power_frac: [1.0, 1.0, 0.0]  # full power to ~290 km/h, linear ramp to 0 by ~345
    per_lap_deploy_MJ: null        # optional integral constraint
  override_mode:                   # "boost / overtake" (2026 Manual Override Mode)
    power_limit_kW: 350
    taper_vs_speed:                # override holds full power to a higher speed
      speed_kph:  [0, 337, 355]
      power_frac: [1.0, 1.0, 0.0]
    extra_energy_per_lap_MJ: 0.5   # additional allowance while activated (VERIFY)
    activation: strategy           # stage-2 control input (e.g. within-1s detection rule)
  recovery:
    braking_power_limit_kW: 350    # regen through the MGU-K under braking (brake blending §7.6)
    per_lap_harvest_MJ: 8.5        # 2026-class harvest limit (VERIFY vs FIA regs)
    recharge_phases: true          # allow ICE-driven recharge (torque-split: ICE > wheel demand,
                                   # K harvests the surplus) + lift-and-coast harvesting
```

> ⚠ The numeric values above are approximate 2026-regulation figures from memory — **verify every
> number against the published FIA 2026 Technical Regulations before shipping reference cars.**
> The *mechanisms* (speed taper, override mode, per-lap energy limits, recharge phases) are the
> architecture; the numbers are config data.

Energy-management control vector per track segment: `u(s) = [deploy/regen ∈ [−1,1], override_flag,
lift_point, shift_map_id]`. V1 ships rule-based deployment (feed-forward "deploy below taper speed,
harvest under braking, recharge on designated straights") + configurable integral constraints.
Stage 2's strategy optimizer writes u(s).

Non-F1 hybrids are the same block with different data: LMDh = single 50 kW MGU on the rear axle;
road PHEV = P2 machine + big ES; pure EV = MGU-K *is* the powertrain (no ICE block).

### 8.4 Battery
Thevenin equivalent-circuit model (ported from NREL `thevenin`, BSD-3): states [SOC, V_RC1
(,V_RC2), T_batt]; parameters OCV(SOC,T), R0(SOC,T), R1(SOC,T), τ1(SOC,T); entropic-heating term
dU/dT; lumped thermal node with I²R + entropic heating; power derating vs T_batt and SOC window.
Pack scaling Ns×Np. The PDT BatteryPack importer (§10.4) fills this block directly.

### 8.5 Machine thermal model — `emotor.yaml` N-node LPTN (Locked Decision #25, amended 2026-07-05)

**AMENDMENT (2026-07-05, author-authorized — M3/PR5): the model below is generalized from a fixed
2-node network to a data-declared *N*-node LPTN, and outlap now *builds* the operator from machine
internals for the detailed path.** The heat-transfer correlations (air-gap film, end-cavity/shaft
convection, liquid-jacket channel) are ported into `outlap-thermal` and evaluated **per segment** at
`(ω, T)`; the network state advances with a semi-implicit **Crank–Nicolson** step (A-stable), one
pinned ambient node, and an optional coolant node closed by a quasi-static jacket balance. This is a
deliberate, narrow reversal of the firewall for the (author-owned) thermal model only — see Decision
#25. Two authoring tiers share the integrator: a **lumped** hand-authored reduced-node model
(mass-heuristic-filled `C`/`G`, constant conductances, flagged as estimates) and a **detailed**
imported model (full node set, explicit `C`, convection edges). The derating and loss treatment
below are unchanged. The original 2-node design rationale is retained for context:

**Design rationale (author's correction):** a community user typically has only a **peak torque
envelope + loss data** for their machine — not continuous/overload envelopes. So outlap does not
*consume* thermal capability curves; it *computes* capability from losses with a deliberately
simple **2-node lumped thermal network**, parameterized in a per-machine `emotor.yaml` (§9.5)
referenced from `vehicle.yaml`. Explicitly NOT PDT-grade: PDT's thermal sub-stage is a 19-node
LPTN with FEA-region geometry and coolant-channel Nusselt correlations — that fidelity stays on
PDT's side of the firewall. Outlap integrates whatever small network the data declares; it never
*builds* one from machine internals.

States: **T_w** (winding, fast) and **T_c** (case/stator lump, coupled to coolant):

```
C_w·dT_w/dt = split_w·P_loss(τ, n, T_w) − G_wc·(T_w − T_c)
C_c·dT_c/dt = (1 − split_w)·P_loss(τ, n, T_w) + G_wc·(T_w − T_c) − G_cool·(T_c − T_coolant)
```

- `P_loss` from the `.ptm` loss map (§9.2); if the map carries a loss *breakdown*
  (winding/core/…, as PDT exports), `split_w` is computed per operating point instead of being a
  constant. Optional copper-resistance feedback: `P_loss_w ∝ 1 + α_cu(T_w − T_ref)` (α_cu in
  emotor.yaml; default off for map-only users).
- **Derating**: commanded torque limit scales linearly from 1 → 0 as each node crosses
  `T_warn → T_max` (winding limit normally binds). Slow states in both tiers (§6.1) — lap 1 ≠
  lap 20, stints are honest.
- ~8 user parameters total; sensible defaults derivable from machine mass alone (documented
  heuristics: C_w ≈ 0.15·m·c_cu, etc., clearly labeled as estimates).
- If the `.ptm` *does* carry continuous/overload envelopes (PDT imports), they are used as
  **validation data**: CI fits nothing, but warns when the 2-node model's derived continuous
  capability disagrees with the imported envelope by more than a stated band.

---

## 9. File Formats — The Product Contract

**Pattern: YAML documents validated by published JSON Schema; bulk numeric tables in sidecar
CSV/Parquet; a vehicle is a directory or zipped `.apx` bundle** (the glTF pattern: readable scene +
binary buffers). Why not alternatives: XML/URDF-like = hostile to numeric arrays + poor diffs;
TOML = unreadable at vehicle nesting depth; pure JSON = no comments; HDF5-only = opaque to git/PR
review (fatal for a community data registry).

Versioning: every file carries `schema: <name>/<MAJOR.MINOR>`; loaders accept same-major; unknown
non-`x-` fields are hard errors (catches typos); `outlap migrate` ships migrations.

### 9.1 `vehicle.yaml` (sketch)

```yaml
schema: vehicle/1.0
extends: presets/formula_base        # optional preset chain (deep-merge, validated post-merge, #41)
name: "Generic F1 2026"
chassis:    { mass_kg: 800, cg: [...], inertia: [...], wheelbase_m: 3.6, track_m: [1.6, 1.55] }
aero:       { map: aero.parquet, axes: [ride_height_f_mm, ride_height_r_mm, yaw_deg] }
suspension: { model: lumped_kc, front: {...}, rear: {...} }   # §7.5 parameters
tires:      { front: c3_front.tyr.yaml, rear: c3_rear.tyr.yaml }
drivetrain:                                   # topology graph (§8.0) — THE versatility surface
  units:
    - source: engine.ptm.yaml                 # ICE or electric machine or lumped drive_unit
      path:   [{gearbox: {ratios: [...], final_drive: 3.2, shift_time_s: 0.05, efficiency: 0.985}},
               {diff: {type: lsd, preload_Nm: 50, ramp: [0.4, 0.6]}}]
      wheels: [RL, RR]
    - source: mguk.ptm.yaml                   # e.g. parallel hybrid EM on the same axle
      thermal: mguk.emotor.yaml               # §9.5 — 2-node thermal model params (EMs; optional)
      path:   [{fixed_ratio: 2.4}]
      wheels: [RL, RR]
  control:
    split: {front: 0.0}                       # static splits where applicable
    torque_vectoring: {enabled: false, k_yaw: 0.0}   # §8.0 rule-based TV
ers:        { ... }                           # §8.3 energy-limits block, optional
battery:    { model: rc_pairs, params: battery.yaml }            # optional
brakes:     { ... }
extensions: { x-anything: ... }               # namespaced, ignored-with-warning
```

### 9.2 `.ptm.yaml` — the neutral powertrain-map contract (THE FIREWALL)

```yaml
schema: ptm/1.0
kind: electric_machine        # electric_machine | ice | drive_unit
axes:
  speed_rpm: [...]            # monotonically increasing
  # tables are defined on (speed × torque) or (speed × load_fraction) — declare which:
  load_axis: torque_Nm        # or load_fraction (-1..1, negative = regen)
  torque_Nm: [...]
tables:                       # sidecar parquet, one column per table
  file: maps.parquet
  efficiency: eff             # 0..1, drive AND regen quadrants
  loss_W: loss                # optional; if both given they must be consistent
limits:
  max_torque_Nm_vs_speed: {file: maps.parquet, column: peak_torque}     # peak envelope (REQUIRED)
  # The following are OPTIONAL (Decision #25): thermal capability is COMPUTED by the emotor.yaml
  # 2-node model from the loss tables; when present (e.g. PDT imports) they serve as validation
  # references, not as the derating mechanism:
  cont_torque_Nm_vs_speed: {file: maps.parquet, column: cont_torque}
  overload: {durations_s: [10, 20, 30], torque_Nm_vs_speed: {file: maps.parquet, columns: [t10, t20, t30]}}
  drag_torque_Nm_vs_speed: {file: maps.parquet, column: drag}           # spin losses
inertia_kgm2: 0.0071          # referred to this map's shaft
mass_kg: 18.7
meta: { source: "user-supplied", dc_voltage_V: 400 }   # provenance, free-form
```

`kind: drive_unit` = machine+inverter+gearbox lumped at the **output shaft** (wheel side); the
consumer must not apply another gear ratio unless `meta.upstream_ratio_applied: false`.

### 9.3 `track.yaml` + `centerline.csv`

Columns: `s_m, x_m, y_m, z_m, banking_deg, width_left_m, width_right_m, grip_scale` — deliberately
the first open 3D racetrack format.

**Track importer (Python, elevated by the full-3D decision #13):** since no open 3D circuit data
exists, the importer builds it: (1) centerline + widths from OpenStreetMap (ODbL — redistributable
with attribution) or TUMFTM CSVs (LGPL, bootstrap only, 2D); (2) **elevation fused from open DEMs**
(Copernicus GLO-30 / USGS 3DEP / national LiDAR where available) sampled along the centerline and
smoothed spline-consistently (z and its derivatives must be C² for vertical curvature); (3) banking
estimated from cross-track DEM sampling where resolution allows, else hand-annotated per corner
(the format allows sparse banking keypoints interpolated in s). Document per-track provenance +
accuracy class in `track.yaml` meta. Later importers: `.xodr`/OpenCRG.

### 9.4 `.tyr.yaml`

MF6.1 coefficient block (superset of `.tir`, round-trippable) + `thermal:` (§7.2 params) +
`wear:` (§7.3 params) + provenance/citation fields.

### 9.5 `emotor.yaml` — machine thermal parameters (Locked Decision #25)

The *only* variables the simple 2-node model (§8.5) needs — a community user can fill this from a
datasheet and a scale:

```yaml
schema: emotor/1.0
nodes:
  winding: {C_J_per_K: 850,  T_max_C: 180, T_warn_C: 150}
  case:    {C_J_per_K: 4200, T_max_C: 120, T_warn_C: 100}
coupling:
  G_wc_W_per_K: 8.5           # winding ↔ case conductance
  G_cool_W_per_K: 45.0        # case ↔ coolant/ambient conductance
cooling: {kind: liquid, coolant_temp_C: 65}     # or {kind: air, ambient_C: 25}
loss_routing:
  winding_split: 0.7          # fraction of P_loss into the winding node
                              # (ignored if the .ptm carries a loss breakdown — then computed per point)
  copper_alpha_per_K: 0.00393 # optional resistance-rise feedback; omit to disable
meta: {source: datasheet | estimated | pdt-distilled, notes: "..."}
```

Defaults/heuristics for every field are documented (mass-based estimates, labeled as such), so a
minimal file is just node masses + coolant temperature + winding T_max.

### 9.6 `conditions.yaml` — session conditions (Locked Decision #46)

Fourth input of the quartet; same track, different day. Full ISA defaults (20 °C, 1013.25 hPa,
no wind) so it's optional:

```yaml
schema: conditions/1.0
air: {temperature_C: 28, pressure_hPa: 1005}     # → density for aero
wind: {speed_mps: 3.5, direction_deg: 240}       # constant vector in v1
track_surface_C: 41                              # tire thermal boundary (T_road, §7.2)
ambient_C: 28                                    # thermal models' ambient / coolant pre-rad proxy
```

### 9.7 `sim.yaml` — simulation settings (Locked Decision #42)

Optional; every field defaulted; CLI/API override file values; the **resolved** settings embed in
every result artifact:

```yaml
schema: sim/1.0
tier: t2                          # t0 | t1 | t2 | t3
dt_s: 0.001
fz_coupling: one_step_lag         # or fixed_point (Decision #29)
integrator: heun                  # heun | rk4 (tableau-selectable)
envelope: {v_points: 40, ax_points: 25, g_normal_points: 7}
raceline: {generator: min_curvature}   # or {file: my_line.csv}
allow_degraded: false             # Decision #40 escape hatch
```

**Presets (`extends:`, Decision #41):** `data/presets/*.yaml` ship with the repo
(formula_base, gt_base, passenger_base) and are ordinary vehicle-schema fragments — the deep-merge
+ post-merge validation + loaded-model report pipeline is described in §6.2b.

---

## 10. PDT HDF5 Importer Specification

**Purpose**: users of the author's professional toolchain ("PDT" — a motor/drive-unit/battery
design pipeline that writes one HDF5 file per stage) can import their results as `.ptm`/battery
files. The importer is a **pure-Python adapter** (`python/outlap/importers/pdt_h5.py`) reading with
`h5py` only — it never imports PDT code (firewall, §1). Three stage files matter; their actual
schemas were inspected on 2026-07-02 from these reference files:

- EDrive: `EDrive_121.0L_16Et_650.0I_400.0V_12ea1_SynRM_ref.h5` (a 136 kW SynRM traction machine)
- DriveUnit: `DriveUnit_16.2GR_168NM_369RPM_666f3_R250_ref.h5` (a small 48 V geared actuator unit)
- BatteryPack: `BatteryPack_13S_3P_722Wh_48V_7158a_cleanTest2Bot.h5`

### 10.1 Common PDT HDF5 conventions

- **Type-tagged tree**: most groups carry attrs `__mdt_type__` (e.g. `OperatingGrid`, `MotorInfo`,
  `PeakCapability`) and `__mdt_module__` (e.g. `models.edrive_types`). Use them for validation/
  dispatch; do not depend on them existing.
- Strings are HDF5 object/bytes → decode UTF-8. Scalars are float32/int64; arrays float32; some
  arrays are **complex128** (phasors — not needed for import).
- `compute/<StageName>/` = provenance (git commit, host, timestamp). `hash/` = pipeline lineage
  keys. `metrics/` = a flat camelCase scalar dump for a DB — **redundant; always prefer the
  structured groups**.
- **⚠ Unit pitfall (verified in the real files)**: summary scalars are inconsistent between files —
  e.g. the EDrive file has `performance/peak_power = 135872.84` (**W**) while its sibling
  `performance/power_at_base_speed = 135.87` (**kW**), and the DriveUnit file's
  `performance_at_vdc/peak_power = 1.585` (**kW**). **Rule: never trust summary scalars; rebuild
  power from arrays as τ[Nm] × ω[rad/s].** Reliable units in the arrays: speed axes in **RPM**
  (`sweep/speed`), `omega`/`rotor_speed_rad` in rad/s, torque **Nm**, losses **W**, efficiency
  **0–1**, voltage **V**, current **A**, temperature **°C**, mass **kg**, inertia **kg·m²**,
  lengths **mm**.

### 10.2 EDrive stage file → `.ptm` (`kind: electric_machine`, machine+inverter at motor shaft)

Verified layout (dataset → shape → meaning):

```
sweep/                          # grid axes (models.vector_types.Sweep)
  vdc                (4,)      # DC-link voltages [V], e.g. [330, 390, 400, 440]
  speed              (40,)     # motor speed [RPM], 0.015 … 30000
  load_ratio         (53,)     # commanded load ∈ [−1, 1]; <0 = regen, 1 = max drive
operating_grid/                 # THE map (models.edrive_types.OperatingGrid), all (4,40,53)
  shaft_torque                  # [Nm] torque delivered at each (vdc, speed, load_ratio)
  motor_efficiency              # 0..1 (0 where op-point infeasible → mask)
  inverter_efficiency           # 0..1
  system_efficiency             # 0..1  ← THE efficiency table for a lumped machine+inverter map
  motor_loss_total, inverter_loss_total, system_loss_total     # [W]
  loss_breakdown/…              # winding/core/inverter split, all (4,40,53) [W] (optional import)
  modulation_index, power_factor, phase_current_peak, …        # diagnostics (skip)
peak_capability/                # envelopes (models.edrive_types.PeakCapability)
  torque_drive       (4,40)    # peak drive torque vs (vdc, speed) [Nm]
  torque_regen       (4,40)    # peak regen torque (negative) [Nm]
  torque_drag        (40,)     # spin drag [Nm]
  thermal/continuous/torque (1,40)      # continuous (thermal steady-state) envelope [Nm]
  thermal/peak/durations    (3,)        # [10, 20, 30] s
  thermal/peak/torque       (1,40,3)    # overload envelopes for those durations [Nm]
  thermal/…/vdc_used        (1,)        # which vdc the thermal envelopes were solved at
inertia/rotor_inertia  ()      # [kg·m²]
mass/…                          # mass breakdown [kg]
info/                           # machine metadata: alias, machine_type, pole_count, max_current_rms, …
performance/                    # summary scalars — DO NOT TRUST UNITS (§10.1); recompute instead
```

**Conversion algorithm:**
1. Pick the vdc slice nearest the user's declared system voltage (or interpolate across vdc).
2. Re-grid from `load_ratio` to torque: for each speed, `shaft_torque[vdc, n, :]` is monotone in
   load_ratio → invert to get efficiency(τ, n) on a regular torque axis (both quadrants).
3. `max_torque_Nm_vs_speed` ← `peak_capability/torque_drive[vdc_idx]`;
   `cont_torque` ← `thermal/continuous/torque`; overload curves ← `thermal/peak/torque`.
4. Mask infeasible cells (efficiency == 0 AND |torque| > envelope) as NaN in the parquet table.
5. Emit `machine.ptm.yaml` + `maps.parquet`; stamp `meta.source: "PDT EDrive <alias> <git hash from
   compute/EDrive>"`, `meta.dc_voltage_V`.
6. **Emit `machine.emotor.yaml` (thermal distillation, Decision #25).** The PDT file carries a full
   19-node LPTN under `thermal_obj/` (`C (19,)` node capacitances, `G_const (19,19)` conductance
   matrix, `R_active`, `R_endturn`, `cu_temp_coeff`, and a `cooling/` group with
   `coolant_inlet_K`, geometry and fluid properties) — far more than outlap wants. The importer
   **distills** it to the 2-node parameters by least-squares fit: choose (C_w, C_c, G_wc, G_cool)
   so the 2-node model, driven by the exported loss maps, reproduces the PDT-solved
   `thermal/continuous/torque` envelope and the 10/20/30 s overload torques at 3–5 speeds.
   Direct copies: `coolant_temp_C` ← `cooling/coolant_inlet_K` − 273.15;
   `copper_alpha_per_K` ← `cu_temp_coeff`; winding split per-point from
   `operating_grid/loss_breakdown` (winding_stator vs core_total vs inverter — inverter losses
   route to the case node). Mark `meta.source: pdt-distilled` + fit residuals in `meta.notes`.
   The un-distilled envelopes also land in `.ptm` `limits:` as validation references (§9.2).

### 10.3 DriveUnit stage file → `.ptm` (`kind: drive_unit`, motor+inverter+gearbox at OUTPUT shaft)

Verified layout:

```
sweep/
  vdc                (6,)      # [V], e.g. [30, 36, 38, 48, 50, 60]
  speed              (28,)     # OUTPUT-shaft speed [RPM] (0.0004 … 369.3 in the reference file)
opt_op/                         # DU-level operating map, all (6, 28, 53)
  torque                        # OUTPUT-shaft torque [Nm] (±168 in reference)
  du_eff                        # 0..1 combined motor+inverter+gearbox efficiency ← THE table
  du_power                      # output power [W]
  du_total_losses               # [W]
  mot_eff, inv_eff, gb_eff      # component efficiencies (provenance/diagnostics)
  loss_parts/…                  # component loss split (6,28,53) [W]
peak_op/
  torque_drive       (6,28)    # peak output-shaft drive torque vs (vdc, speed) [Nm]
  torque_regen       (6,28)    # [Nm], negative
  torque_drag        (28,)     # output-shaft drag [Nm]
  Thermal/continuous/torque (1,28)   # continuous envelope [Nm]
  Thermal/peak/durations (3,)  # [10,20,30] s ; Thermal/peak/torque (1,28,3)
no_load/
  output_speed       (40,)     # [RPM]
  torque_drag        (40,)     # no-load drag at output [Nm]
  no_load_loss_w     (40,)     # [W] (== gearbox no-load loss here)
info/gearbox/                   # GBSVInfo: gear_ratio (16.2488 in ref), num_of_stages, stage1/stage2/…
inertia/
  at_output_j_kgm2   ()        # [kg·m²] referred to output shaft  ← use this
  at_input_j_kgm2, components/…
```

**Conversion**: same re-grid recipe as §10.2 but on `opt_op/torque` + `du_eff`, output-shaft side.
Set `kind: drive_unit`, `inertia_kgm2` ← `at_output_j_kgm2`, record `info/gearbox/gear_ratio` in
`meta` (informational — ratio already applied), `drag_torque` ← `no_load/torque_drag` interpolated
onto the speed axis. In a race car this block maps to a hub/corner drive or a whole e-axle.

### 10.4 BatteryPack stage file → battery block params

Verified layout (small file, ~190 datasets):

```
vector/                         # grid axes (BatteryVectorSettings)
  soc                (20,)     # 0.05 … 1.0
  current            (20,)     # 0 … 60 A (cell-level discharge axis)
  temperature        (5,)      # −10 … 45 °C
cell/                           # Thevenin 1-RC cell parameters on (soc, temperature) grids
  ocv_t              (20,5)    # OCV [V]
  r0                 (20,5)    # series resistance [Ω]
  r1                 (20,5)    # RC resistance [Ω]
  tau1               (20,5)    # RC time constant [s]  → C1 = tau1 / r1
  dudt               (20,5)    # entropic coefficient [V/K]
  cp                 ()        # specific heat [J/(kg·K)]
  ocv_min/ocv_max/…_ref        # scalars at reference conditions
pack/
  efficiency_map     (20,20,5) # (soc, current, T) 0..1
  loss_map           (20,20,5) # [W]
  voltage_map        (20,20,5) # pack terminal voltage [V]
  peak_discharge_power (20,)   # vs SOC [W];  peak_regen_power (20,) [W]
  v_pack_ocv         (20,)     # pack OCV vs SOC [V]
  q_pack ()  [Ah]; e_pack () [Wh]; mass () [kg]; thermal_resistance () [K/W]
info/                           # ns (13), np (3), cell name/chemistry/format, soc window,
                                # max_c_rate, max currents, min/max cell voltage, coolant temp
```

**Conversion**: emit `battery.yaml` for the §8.4 block: 1-RC ECM with bilinear (SOC,T) tables for
OCV/R0/R1/τ1 + dU/dT, pack topology ns×np from `info`, SOC window ← `info/min_soc,max_soc`, power
limits ← `pack/peak_*_power(SOC)`, lumped thermal node from mass·cp and `thermal_resistance`.

### 10.5 Importer design & CLI

```
outlap import pdt-edrive     <file.h5> -o machine.ptm.yaml   [--vdc 400]
outlap import pdt-driveunit  <file.h5> -o du.ptm.yaml        [--vdc 48]
outlap import pdt-batterypack <file.h5> -o battery.yaml
```

Implementation rules: `h5py` + `numpy` + `pyarrow` only; tolerate missing optional groups (PDT
files evolve — key on dataset presence, not `__mdt_type__`); after export, validate the emitted
files against the published JSON Schemas; round-trip test = load emitted `.ptm` in the Rust core
and reproduce ≥3 spot efficiencies from the source arrays to 1e-6. Keep golden mini-fixtures:
generate tiny synthetic PDT-shaped h5 files in the test suite (do **not** commit real PDT files —
they are the author's private data).

---

## 11. Execution Architecture

### 11.1 Language split (committed decisions)

- **Core: Rust.** Rationale: the OSS Rust niche is vacant (differentiator + contributor magnet);
  the permissive substrate exists (diffsol/nalgebra/rayon); every relevant C++ project is a
  formulation reference, not a linkable dependency, so C++ ABI buys nothing; Cargo+maturin makes
  solo maintenance viable; WASM and portable GPU are native.
- **Python: configuration-time.** API façade, schema validation, tire fitting (scipy), importers
  (§10), FastF1 adapters, plotting, notebooks. **Nothing Python inside a timestep.**
- **C: one place.** A stable `extern "C"` tire-model plug-in vtable (init/eval/advance) — the open
  "Standard Tire Interface" (the closed reference is Adams STI). Third parties write tire models in
  C/C++/Fortran without touching the core. CPU-only by contract. Note: under AGPL, *distributed*
  proprietary plugins are effectively derivative works — which matches the author's intent
  (internal/private use remains unrestricted); a plugin-linking exception can be added later if
  ever desired, since the author holds the copyright.
- **License: AGPL-3.0** (Locked Decision #7; policy details §15). The published JSON Schemas
  (§9) are licensed separately and permissively (Apache-2.0) so *other* tools can adopt the file
  formats without copyleft concerns — the formats should spread even where the code cannot.

Workspace layout:

```
outlap/
├─ crates/
│  ├─ outlap-schema/      # serde types for all formats; JSON-Schema generation (schemars)
│  ├─ outlap-core/        # Block trait, Bus, SoA state registry, units, integrators
│  ├─ outlap-tire/        # FLAGSHIP: MF6.1 + brush + relaxation + thermal ring + wear
│  ├─ outlap-track/       # spline track, curvature/elevation/banking, arc-length param
│  ├─ outlap-vehicle/     # chassis 7/14-DOF, aero map, lumped-K&C suspension, brakes, driver
│  ├─ outlap-powertrain/  # map-based ICE/EM, gearbox events, ERS energy manager, battery ECM
│  ├─ outlap-qss/         # T1 envelope generator + T0 fwd/bwd lap solver (stint-capable)
│  ├─ outlap-transient/   # T2/T3 closed-loop lap
│  ├─ outlap-batch/       # rayon batch runner; the stage-2 GPU seam
│  ├─ outlap-py/          # PyO3 bindings (abi3)
│  └─ outlap-wasm/        # wasm-bindgen demo build
├─ python/outlap/         # pip package: API, importers/pdt_h5.py, fitting, plotting
├─ schemas/               # published JSON Schemas (versioned) — a product in itself
├─ data/                  # citation-backed reference vehicles/tires/tracks (small, curated)
├─ examples/              # f1_lap.py, gt3_stint.py, hatchback_trackday.py, tire_fit.ipynb
└─ docs/                  # mkdocs-material; every physics module gets a theory page + citations
```

### 11.1b User-facing API surface (Locked Decisions #17–19)

- **Results: xarray Datasets.** Channel logs dims `(s | time)`, per-wheel dims `(wheel)`,
  comparisons dims `(variant)`, sweeps add one dim per swept field; units in attrs; `.to_parquet`/
  netCDF export. Zero-copy from the Rust batch buffers via rust-numpy where possible.
- **Sweep API (first-class):** `outlap.sweep(vehicle, track, over={"aero.map.scale": [...],
  "drivetrain.control.split.front": [...]})` → rayon-parallel batch → xarray cube. A documented
  **cost-function interface** (callable: vehicle-overrides → scalar(s)) plus a pymoo/optuna example
  notebook; optimizers stay user-side (no optimizer framework in v1, Decision #18).
- **CLI (working, not decorative):** `outlap lap car.yaml track.yaml [--line min_curv]`,
  `outlap compare car_a.yaml car_b.yaml track.yaml`, `outlap import pdt-{edrive,driveunit,
  batterypack}`, `outlap validate <file>`, `outlap migrate <file>`. Outputs parquet + optional PNG
  plots. The CLI wraps the Python API 1:1 — no separate code path.

### 11.2 Numerics

- **T0/T1 (QSS)**: not an ODE — damped-Newton trim solves (no allocation) + forward/backward
  velocity passes; slow states advance per-segment (explicit Euler is exact enough at 10–100 s
  timescales). Target: full lap < 50 ms.
- **T2/T3 (transient): fixed-step split integrator, NOT adaptive, NOT plain RK4**:
  - Chassis/driveline: Heun/RK2 at dt = 1 ms (generic over Butcher tableau; RK4 selectable for
    convergence studies).
  - **Tire relaxation: exact exponential update** `κ ← κ_ss + (κ−κ_ss)·exp(−V·dt/σ)` —
    unconditionally stable at all speeds, kills the stiffness without implicit solves. This is the
    single most important integrator decision.
  - Thermal/wear/SOC: semi-implicit Euler on the diagonal decay terms.
  - **diffsol BDF/ESDIRK as the CI verification integrator**: production stepper must converge to
    the reference solution at O(dt²).
- **Load-transfer algebraic loop (Locked Decision #29):** exposed as a *simulation setting* —
  `fz_coupling: one_step_lag` (default; previous step's accelerations feed load transfer,
  optionally low-pass filtered) or `fz_coupling: fixed_point` (2–3 damped Fz→forces→accel
  iterations per step, for users who want tighter coupling near the grip limit). Both
  deterministic; the setting is recorded in every result artifact.
- **Interpolation standard (Locked Decision #30):** ONE shared implementation — monotone cubic
  Hermite (Fritsch–Carlson) on rectilinear grids, C¹ with analytic derivatives — used by every
  gridded map (aero, efficiency/loss, envelopes, tire thermal params). No per-block interp choices.
- **Envelope × slow states (Locked Decision #31):** T0 consumes a dense base table
  `gg(v, ax, g_normal)` at reference state plus separable multiplicative corrections from T1
  sensitivities (∂gg/∂μ_tire, ∂/∂mass, ∂/∂ClA at reference points); CI validates the corrected
  envelope against full T1 re-solves at sampled off-reference states.
- **Events** (gear shifts, ERS mode changes, pit entry, stage-2 safety car): scheduled or
  condition-triggered discrete transitions at step boundaries, with one linear back-interpolation
  of crossing time where needed. No root-finding in the hot loop.
- **Determinism (CI-enforced)**: fixed dt; counter-based RNG (Philox/ChaCha8) keyed by
  (seed, rollout_id, stream, step); no fast-math; fixed-order reductions; same-target bit-exactness
  guaranteed, cross-platform tolerance-exactness documented; every artifact embeds seed + git hash
  + dt + feature flags.

### 11.3 Batch & GPU

Honest sizing: a strategy rollout at dt 0.1–0.25 s over a 2 h race ≈ 30–70k steps of a 30–60-state
model → ~0.1 s/rollout/core → **10k rollouts ≈ 5–10 s on a 16-core desktop with rayon**. Ship that.
Design NOW so a GPU tier is a drop-in later:
- Struct-of-arrays state with an explicit batch dimension (batch=1 for single runs); public API
  takes/returns batch views (zero-copy NumPy via rust-numpy).
- Zero per-step allocation (preallocated `SimArena`; CI alloc-counter test asserts 0 allocs/step).
- Block eval functions pure and generic over `f32/f64` (`Real` trait) → same code monomorphizes
  into rayon loops today, CubeCL kernels tomorrow.
- Discrete modes as small ints, mask/select-friendly logic.
- GPU decision gate: only when a use case demands ≥10⁵ rollouts; then CubeCL (kernels stay Rust,
  CUDA/Vulkan/Metal/WebGPU backends). JAX-vmap rewrite: rejected (branchy events, Python-locks the
  core, kills WASM).

### 11.4 WASM — first-class target (the Web UI seed)

Per Locked Decision #8, the **Web UI is the endgame**: `outlap-wasm` is not a throwaway demo but
the seed of the eventual primary interface (Stage 3, §16). V1 scope stays modest — QSS lap solver +
one transient rollout run > real-time single-threaded in-browser; a lap-time widget with live
sliders (wing, compound, fuel, drivetrain variant) over a bundled track — but the discipline is
permanent:
- `wasm32-unknown-unknown` builds in CI from M1 onward; a PR that breaks the wasm build fails.
- No filesystem/threading/clock assumptions inside `outlap-core`/`outlap-tire`/solvers; IO behind
  traits; heavyweight deps feature-gated out of the wasm profile.
- Gate: < 2 MB gzipped bundle.
- Stage-3 path: local-first browser app (files stay on the user's machine), WebGPU batch execution
  via CubeCL/wgpu when stage-2 Monte Carlo needs it in-browser; optional hosted compute later —
  the AGPL network clause (§15) protects exactly that surface.

### 11.5 Benchmarks & perf gates

Metrics: transient steps/s/core (target ≥ 500k = 500× real-time at 1 kHz); QSS lap ≤ 50 ms
(Spa-length); 10k rollouts ≤ 10 s (16-core); allocs/step = 0; Python dispatch ≤ 1 ms/batch call.
CI: iai-callgrind instruction-count gates on core kernels (fail > 3% regression) as the merge gate;
criterion wall-time as a nightly trend job on self-hosted hardware; WASM build + demo smoke test in
the same workflow.

### 11.6 Code architecture, style & workflow (Locked Decisions #26–36)

- **Composition (#26):** runtime, data-driven — one binary loads any `vehicle.yaml`; blocks are
  assembled and topologically sorted at load time; enum dispatch inside the loop. No
  per-vehicle-architecture compile paths, ever (required by "car = pure data" + WASM).
- **Errors (#27):** thiserror-typed error enums on every fallible public API (`SchemaError`,
  `AssemblyError`, `SolverDiverged{...}`, …); solver kernels are panic-free and return `Result`;
  `debug_assert!` guards physics invariants in dev builds; `anyhow` only inside `bin`/CLI edges.
  Rationale: panics poison PyO3 and abort WASM.
- **Lints (#28):** workspace-level `clippy::pedantic` with a curated, commented allow-list;
  `#![deny(missing_docs)]` on all public items; `#![forbid(unsafe_code)]` in every crate except
  the C-ABI tire-plugin crate (which isolates all `unsafe`); rustfmt defaults untouched.
- **Naming (#33):** hybrid — descriptive names on public APIs (`slip_ratio`, `vertical_load_n`);
  paper symbols inside math kernels (`kappa`, `f_z`, `sigma_y`) with a doc-comment header mapping
  symbols to cited equation numbers ("Pacejka 2012 eq. 4.E19–4.E30"). Kernels must be diff-able
  against the literature they implement.
- **EOM verification (#32):** `docs/derivations/` SymPy notebooks derive the 7/14-DOF chassis EOMs
  symbolically (Kane/Lagrange via `sympy.physics.mechanics`); CI lambdifies the symbolic RHS and
  asserts agreement with the hand-written Rust RHS at randomized states/parameters to 1e-12.
  Catches the classic sign errors; doubles as community-trust documentation.
- **Python (#34):** uv-managed; ruff for lint + format; pyright strict; full type hints on the
  public API; pydantic v2 models for config objects, validating against the JSON Schemas that are
  **generated from the Rust schemars types** — Rust is the single source of truth for formats,
  Python mirrors never drift (CI check: generated schemas == committed schemas).
- **Overrides & variants (#35):** programmatic sweeps take dotted-path dicts
  (`over={"aero.cl_scale": [...], "drivetrain.control.split.front": [...]}`); named variants are
  YAML overlay files deep-merged onto the base vehicle and schema-validated *after* merge. Both
  compose; every result records the resolved parameter set hash.
- **Git/release (#36):** trunk-based with short-lived PR branches — CI gates enforced even solo
  (keeps history reviewable and contributor-ready); Conventional Commits (`feat:`/`fix:`/`docs:`/
  `perf:`…); tag + GitHub release + generated changelog (git-cliff) at every milestone.

---

## 12. V1 Milestones

Calendar estimates assume the author's stated 10–20 h/week (Locked Decision #12). The full-3D
decision (#13) adds ~4–6 weeks across M1/M3/M4 → **v1 in roughly 7–11 months**. Every milestone
ends in something runnable and demo-able (public repo).

| M | Deliverable | ~Effort | Ships |
|---|---|---|---|
| M1 | `outlap-schema` (incl. drivetrain topology graph §8.0) + `outlap-track` (**3D ribbon**: κ(s), grade, banking, vertical curvature) + **OSM+DEM track importer** (§9.3) + min-curvature line generator (§6.3) + point-mass T0 with 3D normal-load corrections → first lap time on Catalunya. WASM build in CI from here | 4–6 wk | 0.1 |
| M2 | MF6.1 + `.tir` parser/writer + Python fitting pipeline + 3 citation-backed reference `.tyr` files | 4–5 wk | |
| M3 | Full QSS tier (T1 double-track trim → **g-g-g-v envelopes** on the 3D ribbon, ride-height/yaw aero maps, topology-graph powertrain with map-based ICE/EM + gearbox + static splits/diffs + **machine thermal-budget derating §8.5**) — cross-checked vs fastest-lap Limebeer F1 numbers (flat-track mode for the oracle comparison). **PDT importers (§10) land here** | 6–8 wk | 0.2 |
| M4 | Transient tier (T2 in the **curvilinear 3D road frame**, split integrator, ideal driver model, shift events, rule-based TV controller) + QSS↔transient parity gate in CI + **time-weighted raceline QP + the deferred ≤1% Limebeer lap-time gate (Decision #48)** | 5–7 wk | 0.2.5 — shipped 2026-07-13. Parity re-scoped in-flight (Decision #48 pattern): hull containment asserted (0.0% on 3 cars); lap/apex parity + the ≤1% Limebeer gate + the 250k steps/s floor recorded-and-decomposed, not gated (driver corner margin ~+14–17%; RHS-bound ~62k steps/s at MF6.1 fidelity) — see `docs/validation/limebeer.md`. Driver gained a corner-scaled margin, sideslip damper + wheel-slip governor beyond plan; `spa_osm` shipped; Spa fast gate still deferred |
| M5 | **Tire thermal ring + wear in both tiers — the headline. Stint-simulation demo** | 4–6 wk | 0.3 |
| M6 | ERS 2026-style (deploy taper, override mode, recharge phases) + battery ECM + fuel mass + T3 14-DOF | 4–5 wk | |
| M7 | `outlap-batch` (rayon, SoA) + sweep API + working CLI (§11.1b) + **all four reference vehicles** (Locked Decision #1) + the **hero demo as redefined by the author (Decision #22)**: F1 2026-config vs GT hybrid vs EV sports 2-DU AWD vs EV sports 1-DU RWD — each on **its own min-curvature line**, compared lap times + energy on Catalunya/Spa/Silverstone (4-DU TV + FWD ship as extra example configs) + docs site + WASM demo widget | 6–8 wk | **1.0** |

**Post-1.0 roadmap (in order):**
1. **v1.x — sim-racing telemetry importers** (MoTeC `.ld`, ACC, iRacing; Locked Decision #10): the
   community-growth push AND the author's own validation data source (no proprietary data access
   today — Locked Decision #9).
2. **Stage 2 — race-strategy Monte Carlo** (§16): time-discrete race sim + stochastic layer +
   strategy optimizer on the T0-with-slow-states physics.
3. **Stage 3 — outlap-web**: the browser app grows from the WASM widget into the primary interface
   (local-first; WebGPU batch; optional hosted compute under AGPL).
4. **Integrations backlog (Locked Decision #24, in this order):** Gymnasium race-strategy
   environment (with stage 2 — likely becomes the community RL reference), then FMU/FMI export of
   vehicle blocks (opens the Simulink/Modelica professional world). A ROS 2 bridge was considered
   and **withdrawn by the author** — out of scope (and robotics-adjacent, which sits badly with
   the §1 firewall).
5. Community surface throughout: a separate **CC-BY-SA-4.0 data registry** repo (schema-validated
   tracks/vehicles/tire fits, CI smoke-lap on every PR); plugin traits (`TireModel`, `DriverModel`,
   `AeroModel`) + Python entry points; stage-2 `StrategyPolicy`/`SafetyCarModel` plugins.
   Quick-start "10-parameter car" mode deliberately **not** in v1 (Locked Decision #3) —
   revisit on community demand.

---

## 13. Validation Plan & Parity Gates

| Subsystem | Oracle / data | Pass criterion |
|---|---|---|
| MF6.1 | Pacejka-book worked figures; golden CSVs generated once from MFeval (MATLAB outputs used as *data*) | Fx/Fy/Mz ≤ 0.5% over slip/load/pressure sweeps |
| Chassis 7/14-DOF | Chrono::Vehicle same-parameter skidpad / step-steer / sine-dwell; CommonRoad benchmark models; AV21 params | yaw-rate gain, understeer gradient, response time ≤ 3% |
| Lap level (M3, QSS — re-scoped by Decision #48) | Perantoni & Limebeer 2014 (VSD 52(5)) published Catalunya results: 82.43 s lap + Fig. 8 speed trace; fastest-lap (MIT) as a parameterisation cross-check only (its powertrain differs) | **top speed ≤ 1%; slow-corner and fast-corner apex-speed bands ≤ 5%**; lap-time delta recorded with decomposition in `docs/validation/limebeer.md` (QSS-on-heuristic-line floor vs OCP ≈ +5–8%), NOT gated |
| Lap level (M4, transient) | same oracle, via QSS↔T2 parity then T2 vs OCP; time-weighted raceline QP (Decision #48) | lap time ≤ 1% |
| Tire thermal | Farroni TRT published temperature traces; F1 broadcast tire-temp ranges | warm-up time constants + steady temps in published bands |
| Wear/cliff | FastF1 stint pace deltas (2022+ regs) | monotone pace loss + cliff lap reproduced after inverse calibration |
| Battery | NREL `thevenin` pulse-response on identical inputs | voltage RMS ≤ 1% |
| PDT importers | source arrays themselves | ≥3 spot values reproduced to 1e-6 through the emitted `.ptm` |
| Chassis EOMs | SymPy symbolic derivation (docs/derivations) | Rust RHS == symbolic RHS at randomized states to 1e-12 (CI) |
| Machine thermal (2-node) | imported PDT continuous/overload envelopes (when present) | derived continuous capability within stated band of the imported envelope (warn-level gate) |

**QSS↔transient parity gates (CI, every reference car, frozen tire state, smooth track):**
1. lap time |T2 − T0| ≤ 0.3%; 2. per-corner apex speeds ≤ 1%; 3. transient (ax, ay, v) samples
inside the T1 gg-g-v hull with ≤ 2% exceedance area; 4. fuel + ERS energy per lap ≤ 1%. With live
tire states: T0 stint lap-time decay vs T2 long-run ≤ 0.1 s/lap. These gates are exactly what
stage-2 Monte Carlo needs to trust T0.

---

## 14. Testing & CI

- **Golden files**: committed Parquet outputs per reference vehicle × track × tier, per-channel
  tolerances, regenerated only via explicit `--bless`.
- **Property tests** (proptest): tire force symmetry/sign conventions; friction-circle containment;
  energy-accounting closure (ES in/out + fuel LHV vs work + losses); wear monotonic in sliding
  energy; schema round-trip load→save→load.
- **Fuzzing** on all file loaders (YAML/CSV/`.tir`/HDF5 importer).
- **Determinism tests**: same seed twice + across thread counts → bit-identical.
- **Convergence test**: production split-stepper vs diffsol reference at O(dt²).
- GitHub Actions: Linux + macOS + Windows wheel builds (maturin), WASM build, docs build,
  iai-callgrind perf gate.

---

## 15. License & Clean-Room Policy

**Project license: AGPL-3.0** (Locked Decision #7 — author's words: *"forces the disclosure of
the source if our code will be used … the strongest … commercialization OK but always with open
source code"*). AGPL over plain GPL-3.0 because §13's network clause covers SaaS/web deployment —
without it, anyone could serve a modified outlap as a closed web product, which is exactly the
Stage-3 surface (§16).

**What AGPL means here, stated plainly:**
- Anyone may use, modify, sell, or host outlap — but distributed or network-served versions must
  publish their complete corresponding source under AGPL.
- Private/internal use (a race team running it in-house without offering it to others) carries no
  disclosure obligation — copyleft triggers on distribution/network service, not on use.
- Trade-off accepted knowingly: some corporations and permissive-minded contributors will pass.
  That is the price of the guarantee the author wants.

**Licensing structure:**
- Code: AGPL-3.0-only. Each file: SPDX header.
- `schemas/` (the published JSON Schemas): **Apache-2.0** — the file formats should spread to
  other tools even where the code cannot.
- Data registry (reference vehicles/tracks/tires): **CC-BY-SA-4.0** (share-alike, matching spirit).
- **Contributions: DCO now; decide on a CLA before the first significant external contribution.**
  As sole author, the author can later dual-license (e.g. sell commercial exceptions) — but only
  while holding all copyright. Accepting external contributions under DCO-only permanently forfeits
  unilateral relicensing. If commercial dual-licensing is a live option, adopt a CLA from day 1.

**Dependency compatibility (one-way flows INTO AGPL):**
- MIT / Apache-2.0 / BSD / Zlib: freely usable (all §4.1 dependencies remain valid).
- LGPL-3.0 (TUM ecosystem): now usable as dependencies/oracles (§4.3).
- GPL-3.0(+): compatible with AGPL-3.0 (GPLv3 §13); usable if genuinely needed.
- Incompatible: GPL-2.0-only, proprietary SDKs (e.g. AiM's closed DLL) — wrap externally or avoid.

**Authorship & provenance rules (unchanged by the license flip):**
- The flagship models (tire thermal ring, wear/cliff, ERS energy manager) are **implemented from
  the published literature** (Farroni, Pacejka, Archard, FIA regs), never derived from other
  codebases — GPL game-engine tire code (Speed Dreams, VDrift) is off-limits as a *source of
  derivation* regardless of license compatibility. The docs theory pages (equations + citations)
  are the provenance record.
- FSAE TTC data: parsers/fitting yes; redistribution of data or fitted parameter sets NO.
- FastF1/F1 data: calibration/validation artifacts only; do not redistribute raw telemetry.
- PDT `.h5` files: private to the author. Importer reads the documented schema (§10); synthetic
  fixtures in tests; no real files committed.

---

## 16. Stage 2 Preview — Race Strategy Monte Carlo

Designed-for now (hooks in v1), built after 1.0:

- **Time-discrete** race simulator (not lap-discrete) — the acknowledged right architecture (the
  TUM author's own Rust proof-of-concept validates the choice; it was never built out — that repo
  is dual Apache/MIT and may serve as a design seed).
- Physics coupling: T0-with-slow-states provides lap time as a function of (tire age/temp/wear,
  fuel, ERS/battery state, traffic) — replacing every prior project's empirical lap-time-delta
  model.
- Stochastic layer: safety car / VSC / red flag hazard models, pit-stop time distributions,
  overtaking/traffic model (Heilmeier 2020 formulation as the baseline), reliability. Counter-based
  RNG already in the core (§11.2).
- Optimizer: strategy tree search / policy optimization over pit laps, compounds, ERS deployment
  plans (`u(s)` from §8.3), override-mode usage.
- **Rain/wet weather lives here** (Locked Decision #4): wet tire parameter sets, track grip
  scaling (`grip_scale` already in the track format), crossover-lap estimation, drying-line
  evolution — a first-class strategy axis, not a v1 physics feature.
- Open dataset contribution: a maintained post-2019 SC/VSC/accident-phase dataset built on
  FastF1+jolpica (the existing annotated DB stops at 2019 — standalone whitespace).
- A Gymnasium-compatible strategy environment would likely become the community RL reference.

**Stage 3 — outlap-web (the declared endgame, Locked Decision #8):** the WASM widget grows into
the primary interface — local-first browser app (vehicle/track files never leave the user's
machine), interactive lap/stint/strategy studies, WebGPU batch Monte Carlo via CubeCL/wgpu, and
optionally hosted compute — the AGPL network clause (§15) guarantees any hosted derivative stays
open.

---

## 17. Reading List

**Books / core references**
- Pacejka, *Tire and Vehicle Dynamics*, 3rd ed. (2012) — MF6.x, relaxation, combined slip.
- Milliken & Milliken, *Race Car Vehicle Dynamics* — trim, load transfer, K&C, driver.
- Guiggiani, *The Science of Vehicle Dynamics* — rigorous double-track formulation.
- Eriksson & Nielsen, *Modeling and Control of Engines and Drivelines* — ICE mean-value (later).

**Papers (all public)**
- Perantoni & Limebeer 2014, "Optimal control of a Formula One car…" VSD 52(5) — F1 parameter set + 3D track companion papers.
- Heilmeier et al. 2020, "Application of Monte Carlo Methods … Race Simulation" — strategy MC formulation; + Heilmeier race-sim companion papers (tire deg as lap-time delta, overtaking model).
- Farroni et al. — TRT / TRT-EVO thermal tire model papers (the thermal ring formulation).
- Grosch — rubber friction/wear temperature dependence; Archard wear law literature.
- Lovato & Massaro 2022 (VSD) — 3D gg-g-v envelopes in polar form (T0 on 3D tracks).
- Rowold et al. 2023 (IEEE IV) — online 3D racing-line planning (gg-g-v application).
- Limebeer & Rao — review of minimum-lap-time optimal control.
- FIA 2026 Formula 1 Technical Regulations — ERS numbers (§8.3) **must be verified against this**.

**Codebases to study (in this order)**
1. Open-Car-Dynamics (Apache-2.0) — composable submodel architecture, MF52 usage.
2. fastest-lap (MIT) — OCP lap-sim structure, F1 3-DOF model, g-g computation.
3. Chrono::Vehicle JSON vehicle templates — parametric data design.
4. NREL thevenin (BSD-3) — battery ECM.
5. FASTSim v3 / polars — Rust-core + PyO3 packaging patterns.

---

## 18. First-Week Task List

1. **Day 1**: environment (§3); create **public** GitHub repo `outlap` with `LICENSE` =
   AGPL-3.0-only, DCO in `CONTRIBUTING.md` (CLA decision noted as open, §15), `schemas/LICENSE` =
   Apache-2.0; `cargo new` workspace skeleton (§11.1); CI skeleton (fmt/clippy/test + wasm32 build
   on push); reserve the `outlap` names on crates.io and PyPI with 0.0.1 placeholders.
2. **Day 2–3**: `outlap-schema`: vehicle (incl. drivetrain topology graph §8.0) / track / ptm /
   tyr serde types + schemars JSON-Schema emission + round-trip tests. This is the contract —
   review it hard before writing physics.
3. **Day 3–4**: `outlap-track`: centerline CSV → arc-length spline → κ(s), elevation, banking;
   OSM importer (Python) for one real circuit; plot sanity.
4. **Day 5**: T0 point-mass with constant-μ + simple power cap: first lap time on the real track.
   Compare magnitude vs published lap records (~sanity, not parity).
5. **Day 6–7**: `python/outlap/importers/pdt_h5.py` against §10 (the three reference files are on
   the author's machines; synthetic fixtures for CI): EDrive → `.ptm` first, then DriveUnit,
   BatteryPack. Round-trip validation per §13.
6. Then follow the milestone order (§12). At every milestone, update the docs theory page with the
   equations + citations *as they are implemented* (clean-room provenance, §15).

---

## Appendix A — repo CLAUDE.md

Copy verbatim to `outlap/CLAUDE.md` (the AI-assistant working agreement for the new repo):

````markdown
# CLAUDE.md — outlap

outlap is an AGPL-3.0 open-source parametric vehicle simulator (F1 → GT → passenger car) with a
race-strategy Monte Carlo layer planned on top. Rust core (Cargo workspace in `crates/`), Python
API (`python/outlap/`, PyO3+maturin), published JSON Schemas in `schemas/` (Apache-2.0).
The full architecture/spec lives in `docs/HANDOFF.md` — read the relevant section before
implementing anything new; the Locked Decisions log in §1 overrides everything else.

## Hard rules (never break)

1. **Firewall**: powertrains are consumed as `.ptm` map files ONLY. Never model electric machines,
   inverters, or gearboxes internally (no electromagnetic/thermal-network machine models). Never
   add actuator/robot-dynamics features. PDT importers read raw HDF5 with h5py — never import PDT
   code or commit real PDT files (synthetic fixtures only).
2. **Clean-room**: flagship models (tire thermal ring, wear/cliff, ERS energy manager) are
   implemented from published literature (Farroni, Pacejka 2012, Archard, FIA regs) with citations
   in the docs theory page, in the same PR. **Never copy or closely paraphrase another project's
   source.** You MAY *consult* other open-source projects whose licence permits reading them — to
   see how a problem was approached ("how did they solve this") — provided the code is re-authored
   independently from that understanding plus the cited literature, and the consulted repo is
   recorded (name + licence) alongside the citations. Take ideas, not expression; read
   strong-copyleft (GPL/AGPL) sources for approach only. Never lift code from GPL game engines
   (Speed Dreams, VDrift).
3. **License hygiene**: code AGPL-3.0-only with SPDX headers; `schemas/` Apache-2.0; deps must be
   MIT/Apache/BSD/Zlib/LGPL (GPL-3.0-compatible OK if genuinely needed — flag it in the PR).
4. **One vehicle description**: all solver tiers (T0/T1/T2/T3) evaluate the same parameter
   objects. Never add a tier-specific parameter path.

## Engineering conventions

- Hot loop discipline: zero allocations per step (CI-enforced), no Python inside a timestep
  (controllers included — Rust/C-ABI only), blocks pure + generic over f32/f64, SoA state with
  explicit batch dimension.
- Composition is runtime + data-driven: one binary loads any vehicle.yaml; enum dispatch in the
  loop; never add per-vehicle-architecture compile paths. ALL config logic (extends-merge,
  validation, estimation, topology checks, channel interning) happens in the assembly pipeline —
  never inside the loop. Step phases: sense → control → actuate → integrate.
- The input quartet is sacred: vehicle + track + conditions + sim — never mix car identity with
  environment or numerics. Estimated/inherited/degraded values always surface in the loaded-model
  report; `allow_degraded: true` is the only fallback path and it marks the results.
- Exactly three plugin points (custom blocks via Rust trait registration, C-ABI tires,
  controllers). Everything else is core enums — do not add dynamic dispatch to the hot path.
- Config errors are a product surface: miette spans, did-you-mean, plain-language topology
  messages. A bare serde error reaching the user is a bug.
- Errors: thiserror-typed enums on public APIs; solver kernels panic-free (`Result`), physics
  invariants via `debug_assert!`; `anyhow` only in CLI edges.
- Lints: `clippy::pedantic` (curated allows), `deny(missing_docs)` on pub items,
  `forbid(unsafe_code)` outside the FFI crate.
- Naming: descriptive at public APIs; paper symbols inside math kernels with a doc header citing
  the equation numbers being implemented.
- Determinism: fixed-step integrators only in production paths; counter-based RNG keyed by
  (seed, rollout, stream, step); no fast-math; fixed-order reductions. The Fz-coupling mode
  (one_step_lag | fixed_point) is a recorded simulation setting.
- Interpolation: ONE shared monotone cubic Hermite (C¹) implementation for all gridded maps.
- wasm-clean core: no filesystem/threads/clock in `outlap-core`/`outlap-tire`/solvers; IO behind
  traits; `wasm32-unknown-unknown` must keep building (CI gate).
- Units: SI internally (rad/s, Nm, W, K); RPM/°C only at file-format and display boundaries.
  Axis convention ISO 8855 (x forward, y left, z up).
- Schemas are semver contracts generated FROM the Rust schemars types (Python pydantic mirrors
  validate against them; CI checks generated == committed). Additive changes bump MINOR; anything
  else bumps MAJOR + needs a migration in `outlap migrate`. Unknown non-`x-` fields are hard errors.
- Results cross the Python boundary as xarray Datasets (dims: s/time, wheel, variant, sweep axes).
- Python: uv-managed, ruff lint+format, pyright strict, typed public API.
- Git: trunk + short-lived PRs, Conventional Commits, milestone tags with git-cliff changelogs.
- New chassis EOM terms require updating the SymPy derivation notebook — the CI symbolic-vs-Rust
  RHS check (1e-12) must stay green.

## Verification gates (run before claiming done)

- `cargo fmt --check && cargo clippy -- -D warnings && cargo test`
- `cargo build --target wasm32-unknown-unknown -p outlap-wasm`
- Golden-file tests: never regenerate without `--bless` + a PR note explaining the physics change.
- Tier parity: QSS↔transient gates (lap time ≤0.3%, apex speeds ≤1%) must stay green on all
  reference vehicles.
- New physics ⇒ new property test (sign conventions, friction-circle containment, energy closure).
````

## Appendix B — CI workflow

Copy to `.github/workflows/ci.yml` (trim as needed while bootstrapping):

```yaml
name: CI
on:
  push: {branches: [main]}
  pull_request:

env: {CARGO_TERM_COLOR: always}

jobs:
  rust:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
        with: {components: "clippy, rustfmt", targets: "wasm32-unknown-unknown"}
      - uses: Swatinem/rust-cache@v2
      - run: cargo fmt --all --check
      - run: cargo clippy --workspace --all-targets -- -D warnings
      - run: cargo test --workspace
      - run: cargo build --target wasm32-unknown-unknown -p outlap-wasm --release

  python:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: astral-sh/setup-uv@v4
      - run: uv sync --directory python
      - run: uv run --directory python pytest
      - run: uv run --directory python python -m outlap.schemas --check   # schemas in sync

  wheels:            # release-tag only; add maturin-action matrix (linux/mac/win) when publishing
    if: startsWith(github.ref, 'refs/tags/v')
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: PyO3/maturin-action@v1
        with: {command: build, args: --release -m crates/outlap-py/Cargo.toml}
```

(Perf gates — iai-callgrind instruction counts on tire-eval/step/gg-solve kernels — join the
matrix once those kernels exist, per §11.5.)

## Appendix C — CONTRIBUTING.md

```markdown
# Contributing to outlap

Thanks for helping! Ground rules — they are strict because they protect the project's legal
standing and its physics credibility.

## Sign-off (DCO)
Every commit must carry `Signed-off-by:` (`git commit -s`) certifying the Developer Certificate
of Origin (developercertificate.org): you wrote the change or have the right to submit it under
AGPL-3.0.

## Licensing
- Code: AGPL-3.0-only (SPDX header in every file). `schemas/`: Apache-2.0. Data: CC-BY-SA-4.0.
- Dependencies must be MIT/Apache/BSD/Zlib/LGPL. Anything else: open an issue first.

## Clean-room policy (non-negotiable)
- Physics models are implemented from published literature; PRs adding/changing models MUST update
  the matching docs theory page with equations + citations.
- **Never copy or closely paraphrase code** from other simulators/game engines or proprietary tools.
  You MAY *consult* other open-source projects whose licence permits it to understand the approach or
  avoid pitfalls ("how did they solve this") — record the repo (name + licence) alongside the
  citations, re-author independently (ideas, not expression), and read strong-copyleft (GPL/AGPL)
  sources for approach only. Never lift code from GPL game engines.
- No proprietary data: FSAE TTC data and fitted TTC parameter sets cannot be committed; raw F1
  telemetry cannot be committed. Synthetic/citable data only.

## PR checklist
- [ ] `cargo fmt` / `clippy -D warnings` / `cargo test` green, wasm target builds
- [ ] No new allocations in step paths (alloc-counter test green)
- [ ] Golden files unchanged, or regenerated with `--bless` + a physics justification in the PR
- [ ] New physics → property test + theory-page citation
- [ ] Schema changes → version bump + migration + round-trip test
```

---

## Appendix D — Ubuntu bootstrap, exact commands (fresh machine, SSH, nothing installed)

Assumes: Ubuntu 24.04, SSH session from the Windows machine, no files transferred yet, no Claude
Code. Run top to bottom.

### D.0 Transfer this document (run on the WINDOWS machine, PowerShell)

```powershell
scp "C:\Users\neomo\Documents\RACESIM_HANDOFF.md" <user>@<ubuntu-ip>:~/
```

### D.1 System packages (Ubuntu)

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential git curl wget pkg-config libssl-dev cmake \
                    tmux ripgrep gh mesa-vulkan-drivers vulkan-tools
```

`tmux` matters: run Claude Code inside tmux so dropped SSH connections never kill a session
(`tmux new -s outlap` / reattach with `tmux attach -t outlap`).

### D.2 Rust toolchain

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
rustup component add clippy rustfmt
rustup target add wasm32-unknown-unknown
```

(`wasm-pack`, `git-cliff`, `cargo-criterion`, `iai-callgrind-runner` install later when first
needed — they compile for a while on the i5-6500.)

### D.3 Python toolchain

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
uv python install 3.12
```

### D.4 Identity + GitHub auth

```bash
git config --global user.name  "Konstantinos Moulakis"
git config --global user.email "neomoula@gmail.com"
git config --global init.defaultBranch main
git config --global commit.gpgsign false
gh auth login
# choose: GitHub.com → HTTPS → Yes (authenticate Git) → Login with a web browser
# it prints a one-time code + URL — open the URL in the WINDOWS browser, enter the code
```

### D.4a Status check — skip whatever is already satisfied

Some tools (e.g. git) are already on the machine. Run this block first and skip any command in
steps D.1–D.5 whose check already passes:

```bash
# --- versions: a version string = installed, "command not found" = run that step ---
git --version
gh --version
rustc --version 2>/dev/null || echo "MISSING: run D.2"
cargo --version 2>/dev/null || echo "MISSING: run D.2"
uv --version 2>/dev/null || echo "MISSING: run D.3"
claude --version 2>/dev/null || echo "MISSING: run D.5"
tmux -V 2>/dev/null || echo "MISSING: apt install tmux (D.1)"
rg --version 2>/dev/null | head -1 || echo "MISSING: apt install ripgrep (D.1)"
vulkaninfo --summary 2>/dev/null | head -5 || echo "MISSING: apt install mesa-vulkan-drivers vulkan-tools (D.1)"

# --- git identity: prints value = already set (skip that git config line in D.4) ---
git config --global user.name       || echo "user.name NOT set"
git config --global user.email      || echo "user.email NOT set"
git config --global init.defaultBranch || echo "defaultBranch NOT set"
git config --global --list          # full picture

# --- GitHub auth: "Logged in to github.com as <user>" = skip gh auth login ---
gh auth status

# --- Rust extras (only if cargo exists) ---
rustup component list --installed 2>/dev/null | grep -E "clippy|rustfmt"
rustup target list --installed 2>/dev/null | grep wasm32 || echo "wasm32 target MISSING: rustup target add wasm32-unknown-unknown"

# --- workspace collision check before D.6 ---
ls -la ~/dev/outlap 2>/dev/null && echo "WARNING: ~/dev/outlap already exists — inspect before D.6" || echo "~/dev/outlap free"
ls ~/RACESIM_HANDOFF.md 2>/dev/null || echo "handoff not transferred yet: run D.0 on Windows"
```

Interpretation rules: a passing check means **skip that command, not the whole step** (e.g. git
installed but `user.email` unset → still run the two `git config` identity lines). `gh auth
status` failing with "not logged in" is the only trigger for `gh auth login`. If
`~/dev/outlap` already exists, look inside before D.6 — never overwrite it blindly.

> **Machine snapshot (verified 2026-07-03, host `kmoulakis-linux`, user `kmoulakis`):** already
> present — git 2.43.0 (user.name `KMoula30`, user.email set, `credential.helper=store`),
> gh 2.45.0 (**not authenticated**), rustc/cargo 1.96.1, uv 0.11.26, Claude Code 2.1.19, tmux 3.4,
> ripgrep 14.1.0, Vulkan 1.3.275. Still needed: `init.defaultBranch main`, `gh auth login`,
> handoff transfer. Unverified: build-essential/cmake/pkg-config/libssl-dev, rustup-managed vs
> apt rust, clippy/rustfmt components, wasm32 target, Claude login. Skip D.1–D.5 except those.

### D.5 Claude Code (terminal)

```bash
curl -fsSL https://claude.ai/install.sh | bash
export PATH="$HOME/.local/bin:$PATH"     # installer adds this to your shell profile too
claude --version
```

First login: run `claude` once anywhere, use `/login` — it prints a URL + code for the browser
(same device-flow dance as gh).

### D.6 Repository skeleton

```bash
mkdir -p ~/dev/outlap && cd ~/dev/outlap
git init
mkdir -p crates docs schemas data/presets data/vehicles data/tracks data/tires \
         examples python .github/workflows .claude
mv ~/RACESIM_HANDOFF.md docs/HANDOFF.md

# licenses (Decision #7 + schema carve-out)
curl -o LICENSE https://www.gnu.org/licenses/agpl-3.0.txt
curl -o schemas/LICENSE https://www.apache.org/licenses/LICENSE-2.0.txt

# Cargo workspace
cat > Cargo.toml <<'EOF'
[workspace]
resolver = "2"
members = ["crates/*"]

[workspace.package]
edition = "2021"
license = "AGPL-3.0-only"

[workspace.lints.clippy]
pedantic = { level = "warn", priority = -1 }
EOF

for c in outlap-schema outlap-core outlap-tire outlap-track outlap-vehicle \
         outlap-powertrain outlap-qss outlap-transient outlap-batch outlap-py outlap-wasm; do
  cargo new --lib "crates/$c" --vcs none
done
cargo build   # sanity: workspace compiles

# Python package (pure-python now; maturin wiring joins at M1 when outlap-py has content)
cd python
uv init --package --name outlap .
uv add numpy xarray h5py pydantic pyarrow
uv add --dev pytest ruff pyright
cd ..

# Claude Code permission allowlist (fewer prompts, still safe)
cat > .claude/settings.json <<'EOF'
{
  "permissions": {
    "allow": [
      "Bash(cargo build:*)", "Bash(cargo test:*)", "Bash(cargo clippy:*)",
      "Bash(cargo fmt:*)", "Bash(cargo doc:*)", "Bash(cargo run:*)",
      "Bash(uv run:*)", "Bash(uv sync:*)", "Bash(uv add:*)",
      "Bash(git status)", "Bash(git diff:*)", "Bash(git log:*)", "Bash(git add:*)",
      "Bash(rustup:*)", "Bash(gh pr:*)", "Bash(gh issue:*)", "Bash(gh run:*)"
    ]
  }
}
EOF

echo -e "target/\n__pycache__/\n.venv/\n*.egg-info/\ndist/\n.pytest_cache/" > .gitignore
```

### D.7 Create the public GitHub repo + first commit

```bash
cd ~/dev/outlap
git add -A
git commit -s -m "chore: bootstrap workspace, licenses, handoff doc"
gh repo create outlap --public \
  --description "outlap — open vehicle racing simulator & strategy optimizer (AGPL-3.0)" \
  --source=. --remote=origin --push
```

Then (once, from any browser): create the crates.io and PyPI accounts and reserve the `outlap`
name with 0.0.1 placeholder releases when convenient (§2).

### D.8 First Claude Code session

```bash
cd ~/dev/outlap
tmux new -s outlap
claude
```

Opening prompt (paste as-is):

> Read docs/HANDOFF.md in full before doing anything — it is the single source of truth for this
> project, and its Locked Decisions log (§1) overrides any other instinct. Then: (1) extract
> Appendix A into ./CLAUDE.md, Appendix B into .github/workflows/ci.yml, Appendix C into
> ./CONTRIBUTING.md, verbatim with any obvious path fixes; (2) commit that as `docs: add working
> agreement, CI, contributing`; (3) start §18 Day 2–3 — design `outlap-schema` (the serde types +
> schemars JSON-Schema emission for the vehicle/track/conditions/sim quartet, §6.2b + §9) and show
> me the vehicle schema types for review before implementing the rest.

Working habits that extract the most from Claude Code here:
- One milestone task per session; `/clear` between unrelated tasks; `claude --continue` to resume.
- Plan mode (Shift+Tab) for anything architectural; let it read HANDOFF.md sections first.
- Keep CLAUDE.md the lean working agreement (Appendix A); deep spec stays in docs/HANDOFF.md —
  Claude reads the relevant section on demand.
- Review diffs before accepting writes on schema/format code — the contracts are the product.
- Once CI exists, have Claude open PRs (`gh pr create`) instead of pushing to main, per #36.

---

*End of handoff. This document supersedes any prior conversation context — everything the project
needs to start is above.*
