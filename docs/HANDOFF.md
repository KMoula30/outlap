# RACESIM PROJECT HANDOFF — Complete Bootstrap Document

> **What this file is for.** This is a self-contained engineering handoff. It tells you how to start
> a new open-source project on a fresh Linux machine. The reader, whether human or AI assistant, is
> assumed to know **nothing** about the author's prior work, employers, or tools.
>
> Everything you need is in this one document: the vision, the constraints, the verified
> open-source landscape, the full system architecture, the physics models, the file-format
> contracts, the specification for the PDT HDF5 importer (with the actual file schemas, documented
> from inspection), the decisions on language and tooling, the milestones, and the validation plan.
>
> It was produced on 2026-07-02, after a research workflow of 29 agents that surveyed 93 OSS
> projects and verified 18 of them for license and activity, plus direct inspection of three real
> PDT `.h5` files. It was **updated on 2026-07-03, after a round of 12 questions with the author**.
> See the Locked Decisions log at the end of §1. Those answers override anything that contradicts
> them.

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

**What we are building.** A parametric foundation for vehicle simulation, open source, covering
motorsport and road cars. It must handle "anything with 4 wheels": a Formula 1 car, an LMP or GT
car, a track-day hatchback.

It is built from shared foundational blocks. A specific car is *pure data*. It is never code.

On top of that foundation, and designed for now but built later, comes stage 2: a Monte Carlo
simulator for race strategy, and tools for lap-time analysis.

**Versatility is the product.** The community will adopt this only if *any* concept is easy to
express and to compare.

Here is the canonical example, which is also the hero demonstration of v1. Take one EV chassis.
Compare optimal laps at a circuit for four layouts: a 1-drive-unit RWD car, a 2-DU AWD car, a 4-DU
car with torque vectoring, and an FWD car. Then swap to a GT car with a large ICE and a small
electric machine, at a hybrid split of 80/20 or 90/10. Change nothing but data.

That requirement forces two things. The powertrain must be a **topology graph** (§8.0), not a fixed
layout. And every subsystem — tires, aero, dynamics — must be swappable on its own.

**Product philosophy.** V1 ships **feature-rich**. The community's role is the next generation of
improvements, new data such as tracks, vehicles, and tire fits, and bug fixes. It is not to build
the foundation. One experienced simulation engineer builds v1 alone.

**Hard constraints. None of these is negotiable.**

1. **A firewall against conflicts of interest.** The author works professionally in two fields:
   humanoid robotics, covering actuators and motion control; and tooling for electric-powertrain
   design, covering electromagnetic motor design, drive units, and battery packs. That second
   toolchain is called "PDT" here. Therefore this project obeys three rules.
   - **It never designs or models a machine electromagnetically.** A powertrain, electric or ICE,
     enters ONLY as a **map** of torque, speed, and efficiency, in a neutral open file format:
     `.ptm`, §9.
   - **It never touches actuators, motion control, or robot dynamics.**
   - The author's private tools export to the neutral format, and this project only consumes it. A
     converter from PDT to that format ships as an importer (§10), so that PDT users can bring
     their own maps. The importer reads plain HDF5. It never imports PDT code.
2. **Strong copyleft: AGPL-3.0.** This is the author's explicit decision, of 2026-07-03. The author
   wants the strongest available guarantee: anyone who builds on this code, or serves it, must
   publish their source. Commercial use is welcome, but never closed-source.

   AGPL rather than plain GPL, because the network-use clause also covers SaaS and web deployments.
   That matters, because the Web UI is the declared endgame; see constraint 5. §15 gives the full
   policy on dependency compatibility.
3. **The languages were chosen deliberately, to flex engineering range**: a Rust systems core, a
   Python user API, a GPU batch path designed in from the start, and WebAssembly as a first-class
   target. See §11.
4. **One canonical vehicle description feeds every fidelity tier** (§6). There is never a
   re-parameterization for a tier.
5. **The Web UI is the endgame.** V1 is API-first and CLI-first. But the WASM build is not a
   throwaway demonstration. It is the seed of the eventual primary interface, which is Stage 3,
   §16.

   One consequence is enforced from day 1. The core crates stay wasm-clean: `outlap-core` assumes
   no filesystem and no threading, IO sits behind traits, and every schema converts to JSON without
   loss.

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
| 30 | Map interpolation | **Monotone cubic Hermite (Fritsch-Carlson), C¹**, one shared implementation for all gridded maps; analytic derivatives for Newton solvers. **AMENDED M6/PR1 (recorded exception):** regulatory *closed-form piecewise-linear formulas* (FIA C5.2.8 ERS tapers, C5.12 ramp bounds) are evaluated by the shared exact piecewise-linear interpolant — the Hermite bows a flat-plateau breakpoint set up to +78 kW above the regulation line at 315 kph. Closed-form regs only; gridded maps stay on the Hermite |
| 31 | T0 envelope vs slow states | **Base table gg(v, ax, g_normal) + separable multiplicative corrections** from T1 sensitivities (μ_tire, mass, ClA); validated against full T1 re-solves in CI. **AMENDED 2026-07-14 (author-authorized, M5/PR4 — see #49):** tyre thermal + wear are promoted from corrections to **genuine grid axes** the boundary is re-solved across (`gg(v, ax, g_normal, T_tire, wear)`); μ_tire/mass/ClA stay separable corrections |
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
| 49 | Envelope tyre-state axes (author-decided 2026-07-13, M5/PR4) | The g-g-g-v envelope gains **real `T_tire` and `wear` grid axes** — the boundary is re-solved at a uniform grip factor `g(T,w) = λ_μ(T)/λ_μ(T_opt) · wear_grip(w)/wear_grip(0)` per node (the same Farroni window + Archard/Grosch cliff the force model uses, applied through `with_mu_scale` as the tier feeds `mu_scale_total`) — **not** a μ-style multiplicative correction. This is the higher-fidelity path §6.1 calls "the tyre-state axes are the differentiator" and what makes the QSS tier stint-capable. **Hard invariant:** `T_opt` sits at the exact centre T_tire node and `wear = 0` at node 0, so the grip factor is `1.0` bit-for-bit there and the reference slice reproduces the frozen envelope exactly — the QSS↔T2 parity gates + goldens stay green with **no re-bless**. The axes are **opt-in** (`generate_with_tire_state`; the default `generate` is unchanged and cheap): the boundary sweep runs `t·w` times, so it is a cold-assembly cost paid only when the tiers request live tyre state (QSS march: PR5). Gas-law pressure + carcass softening (the other two §7.2 node-temperature couplings) stay at reference in the envelope — the dominant grip-magnitude effect is carried; the tier composes the rest per step. A validation-motivated amendment to Decision #31's separable-corrections scope. See `docs/theory/ggv-envelope.md` |
| 50 | Fuel mass/CG stay corrections, NOT axes (M6/PR5, D-M6-4) | The fuel slow state couples to the g-g-g-v envelope through Decision #31 **separable multiplicative corrections** — mass (the existing `∂gg/∂mass` secant) plus CG (`with_cg` → new `∂gg/∂a_f`, `∂gg/∂h_cg` secants), validated against full T1 re-solves in CI — **NOT** a re-solved grid axis. This is the **opposite** conclusion to #49, deliberately: tyre thermal/wear reshape grip non-linearly/non-monotonically (window peak, wear-cliff sigmoid) so an axis earns its keep, but mass/CG are **smooth and monotone** perturbations of the load-transfer algebra for which a first-order secant is accurate and a re-solved axis would only multiply the 5–22 s envelope build for no fidelity gain. **Envelope reference = full-tank m₀** at the full-tank CG (D-M6-4b), so the correction is `1.0` bit-for-bit at lap start (the #49 `T_opt`/zero-wear invariant, mirrored) and drifts as the tank drains; no `fuel:` block ⇒ constant mass ⇒ byte-identical. CG migration ships in BOTH tiers (D-M6-4c). See `docs/theory/fuel-mass.md` + `docs/theory/ggv-envelope.md` |
| 51 | T3 chassis inertia convention = Option A (M6/PR6, D-M6-6) | For the 14-DOF tier, `chassis.inertia[0]`/`[1]` (`Ixx`/`Iyy`, roll/pitch) are the **sprung-mass** inertias about the **sprung CG**; `inertia[2]` (`Izz`, yaw) is the **whole-car** yaw inertia (the value T2 uses, unchanged). Rationale: roll and pitch are motions of the sprung mass alone, so their resisting inertia is a sprung-mass property (and the exact quantity the CG-referenced Kane EOM consume, and how a CAD sprung model / K&C rig reports them); yaw is a whole-car motion. The alternative (all-whole-car, subtract the unsprung contribution by parallel-axis) reconstructs the same sprung inertia through more error-prone geometry — the sprung roll/pitch inertia is the fundamental quantity, so it is the direct input. `mass_kg` stays one lumped total; per-axle `unsprung_mass_kg` splits out the sprung mass. See `docs/theory/t3-chassis.md` |
| 52 | Tyre vertical `k_z`/`c_z` home = structured `.tyr` `vertical` block (M6/PR6, D-M6-6) | The T3 per-wheel `F_z` comes from a tyre vertical spring; its stiffness/damping live in a new structured `.tyr` `vertical: { stiffness_n_per_m, damping_n_s_per_m }` block (`tyr/1.2`), coherent with the `brush`/`thermal`/`wear` structured blocks — NOT a vehicle-side field (which would recreate the ers↔battery duplication PR2 fixed). The legacy `VERTICAL_STIFFNESS` MF6.1 map key stays supported as a fallback so existing `.tyr` files keep working; both consumers read structured → map key → 250 kN/m default. The T3 suspension fields (`unsprung_mass_kg`, dampers, absolute `arb_stiffness_n_m_per_rad`, `bumpstop`) join `suspension.*` as optional additive fields (`vehicle/1.9`), applied L/R-symmetrically, and are **not** estimated (a `t3` vehicle that omits them fails at assembly — the `per_lap_deploy_mj` trap pattern). See `docs/theory/t3-chassis.md` |
| 53 | ONE shared `TransientSolver` for T2+T3 (M6/PR6a, decided jointly for PR6+PR7) | The transient solver is a single type parameterised by an **Fz-coupling strategy** (`Algebraic{LoadTransfer, crest_floor}` for T2 | `TyreSpring` for T3, where `F_z` is the tyre-spring deflection and the `CREST_UNLOADING_FLOOR_G` retires with the strategy) plus a constructor-selected integrated-slot set (`t2_integrated_slots` / `t3_integrated_slots`) and a documented deterministic sub-cycle for stiff bumpstop engagement — NOT a sibling T3 solver, which would fork the slow-clock / ledger / fuel / tyre machinery PR4/PR5 built. PR6 records the seam; PR7 implements it. The `CoreBlock`/`SuspensionStub` scaffolding was deleted (dead fiction, no consumers): both tiers hold concrete block structs, dispatched statically (Decision #26, no `dyn`), so PR7 wires a concrete `ChassisT3` field, not a stub. **PR7 implemented it**: `TransientSolver<T, B: TierBlocks<T>>` generic over the block composition, T2 impl instruction-identical (byte-identical T2 lap), `T3Blocks` the sibling; the Fz strategy is expressed as the `uses_algebraic_coupling()` gate (T2 Picard + crest floor / T3 one-eval from state), not a stored enum. See `M6_PR7_PLAN.md` |
| 54 | T3 aero on the sprung mass (M6/PR7, D-M6-6) | The T3 tier evaluates drag + per-axle downforce at the **instantaneous** ride heights (`h_f/h_r` from heave/pitch, through the shared ride-height aero map) and applies the downforce to the **sprung body** (heave force + pitch moment), NOT at the contact patch. It reaches the tyres *through the springs* — the platform sinks under load, compressing the suspension and then the tyre spring — so the per-wheel `F_z` (the tyre-spring deflection) carries the downforce with no separate contact-patch term, and the ground-effect ride-height coupling (more downforce → lower platform → the map re-reads a lower height) is honest. The proven 14-DOF RHS + its 1e-12 fixture were **extended** for the two aero inputs (`fzaf`/`fzar`) in the same PR (the T2 chassis + fixture untouched). Rationale: "per-wheel `F_z` comes from the tyre-spring deflection" (§6.1) only holds physically if the downforce loads the springs; the contact-patch alternative would leave the modelled ride heights at their no-aero equilibrium (far too high for a downforce car). See `docs/theory/t3-chassis.md` |

---

## 2. Project Name

Availability was checked on 2026-07-02, by GitHub search and through the crates.io and PyPI APIs:

| Candidate | GitHub | crates.io | PyPI | Verdict |
|---|---|---|---|---|
| **outlap** | ~clean (one 1★ fantasy-sport dashboard) | AVAILABLE | AVAILABLE | **Recommended** |
| apexsim | a few 0-1★ toy repos | AVAILABLE | AVAILABLE | Good fallback |
| open-race-sim | clean | AVAILABLE | AVAILABLE | Generic but descriptive |
| ovro (open-vehicle-racing-optimizer) | **collides**: Owens Valley Radio Observatory ecosystem (121 repos) | available | available | Avoid |
| undercut | **collides**: undercut-f1 (896★ F1 timing TUI) | available | available | Avoid |
| racelab | scattered | available | **taken** | Avoid |

**DECIDED on 2026-07-03: `outlap`.** The out-lap is where tire temperature, fuel, and strategy all
converge, before a flying lap. The word is short, native to motorsport, and unclaimed on both
package registries.

The repository description carries the descriptive long name: *"outlap — open vehicle racing
simulator & strategy optimizer"*.

Register the names on crates.io and PyPI early, with placeholder 0.0.1 releases. That is insurance
against name-squatting. The repository is **public from day 1**.

---

## 3. Development Environment (Linux)

**DECIDED: the author's existing Ubuntu 24.04 desktop is the development machine.**

```
OS:  Ubuntu 24.04.4 LTS x86_64 (kernel 6.17)     RAM: 16 GB
CPU: Intel i5-6500 (4 cores) @ 3.6 GHz           GPU: NVIDIA GTX 1060 3GB
```

- This machine is fine for developing v1. The core is light on CPU during development. The GTX 1060
  runs Vulkan and wgpu for later GPU experiments. And CI parity holds: the `ubuntu-latest` runners
  of GitHub Actions run Ubuntu. Where the code is *developed* does not matter to CI, but a local
  environment that matches CI eliminates "works on my machine".
- **A known limitation.** Four cores means that batch benchmarks run about 4 times slower than on a
  modern desktop. Treat this box as the *development* machine. Nominate a faster machine later, and
  document it as the hardware of record for benchmarks, in a BENCHMARKS.md for each release.
- **HARD RULE: never develop this project on the work laptop.** No dual boot, and no checkout of
  the repository. Personal OSS on hardware adjacent to an employer undermines the firewall of §1,
  and it risks the machine the author earns with. This is part of the firewall. It is not a matter
  of convenience.

The toolchains are agnostic to the distribution in any case: `rustup` for Rust, `uv` for Python,
and `maturin` for wheels. Ubuntu LTS simply optimizes for boring stability and driver support.

Run this setup script once, on the fresh machine:

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

Then run three sanity checks: `cargo --version`; `vulkaninfo --summary`, to confirm the wgpu
backend is present; and `uv run python -c "import sys; print(sys.version)"`.

---

## 4. Verified Open-Source Landscape & Reuse Policy

Every license and every activity claim was verified against the actual repository on 2026-07-02.

> **How licenses flow under AGPL-3.0, which is our license, §15.** Permissive code, under MIT,
> Apache, BSD, or Zlib, flows INTO an AGPL project freely. Every dependency in §4.1 therefore stays
> fully usable.
>
> The LGPL-3.0 wall that existed under the original permissive plan **drops**. An LGPL library is
> now legally usable as a dependency too.
>
> We still *prefer* to re-implement a core algorithm in Rust, from the papers. Three reasons:
> quality, integration, and the fact that the flagship contributions must be our own. But the
> constraint is now engineering judgment. It is no longer law.

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

These conclusions come from the landscape sweep. Two or more research agents confirmed each one
independently.

1. **No open-source tire thermal model with wear and degradation exists. Anywhere, in any
   language.** The physics is published: the TRT and TRT-EVO ring models of Farroni, the TameTire
   papers, and the wear laws of Archard and frictional energy. But every implementation is
   proprietary — MegaRide thermoRIDE and WeaRIDE, Michelin TameTire, add-ons for FTire — or it is a
   dead repository with no stars. OSS uniformly stops at Pac89, Pac02, or TMeasy, with grip that
   does not vary in time. **→ This is the flagship contribution (§7.2).**
2. **No complete open MF6.1 or 6.2 exists outside MATLAB.** There is no maintained MF package that
   pip can install. And there is **no Rust implementation of any tire model at all**.
3. **No credible Rust crate for vehicle dynamics exists.** The niche in that language is vacant,
   and the substrate — diffsol and nalgebra — is ready.
4. **No open schema for a "race car as data" exists**, covering suspension, aero map, tires, and
   powertrain. Every project invents its own. Defining one is a chance to become the standard (§9).
5. **No open representation of an aero map that depends on ride height and rake exists**, for a
   ground-effect car.
6. **No open ERS or hybrid race powertrain exists**, with deployment strategies and energy limits,
   coupled to chassis dynamics. Battery models exist, such as PyBaMM and thevenin, but they are
   uncoupled from vehicle simulation.
7. **Physics and strategy are completely disconnected in OSS.** A race-strategy simulator uses an
   empirical delta in lap time for degradation. A physics engine has no wear states. Our stage-2
   thesis occupies exactly that seam.
8. **No open 3D racetrack format or dataset exists**, with elevation, banking, and grip. The
   academic standard is 2D, and frozen since 2021.
9. **No motorsport OSS tool has a batch or GPU story**, and nothing runs in a browser. A demo in
   WASM and WebGPU is therefore a cheap differentiator.

---

## 6. System Architecture

### 6.1 The core invariant: one vehicle description, four derived views

There is **one canonical parameter set** for each vehicle (§9). A lower-fidelity tier is **derived
at run time, by evaluating the same objects**. Parity between tiers is therefore a property of the
solver, and CI can test it. It is not a discipline of data entry.

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

- **T0** solves a velocity profile forward and backward, on a spline track. The formulation is that
  of TUM's `calc_vel_profile`, re-implemented from the papers.

  It runs in **full 3D** (Locked Decision #13). The track is a 3D ribbon, with curvature κ(s),
  grade, banking, and vertical curvature. The envelopes are therefore **g-g-g-v**, in the polar
  form of Lovato and Massaro. The axis of apparent gravity and normal load captures three effects:
  load from banking, unloading over a crest, and compression, as at Eau Rouge.

  The constraint envelope is `gg(v, ax, g_normal | ride_heights, T_tire, wear, fuel_mass)`. **The
  tire-state axes are the differentiator**: a lap run for the strategy tier sees physical
  degradation.
- **T1** solves, for each (v, ay, ax), the algebraic trim, by damped Newton. The unknowns are
  z = [steer δ, sideslip β, yaw rate r = κv, throttle and brake split, 4×Fz]. The equations are the
  force and moment balance in X, Y, and N, plus quasi-static load transfer in the lateral and
  longitudinal directions, plus the aero-platform equilibrium.

  Load transfer has a geometric route, through roll-center heights and the anti effects, and an
  elastic route, through the distribution of roll stiffness. The aero-platform equilibrium takes
  ride heights from the wheel rates and the aero loads, and iterates against the aero map.
- **T2** carries the states [s, n, ψ_rel, vx, vy, r, ω₁..₄], in the **curvilinear 3D road frame**.
  Position along the track is s, and lateral offset is n. Banking and grade rotate the gravity
  vector and the load vectors. It adds tire relaxation states, and the slow states listed below.

  Load transfer is algebraic, using the same expressions as T1. The ODE is smooth, and there is no
  contact solver. It is therefore batchable on a GPU.
- **T3** adds sprung heave, pitch, and roll — z, φ, θ, and their rates — plus 4 unsprung vertical
  DOF. It has nonlinear tables for springs and dampers, plus bumpstops and ARBs. It takes camber
  and toe against travel from K&C tables.

  It is needed because pitch under braking shifts the aero balance, and that is *the* defining
  behavior of a downforce car.
- **Later. Design for these, but do not build them.** An adapter to full multibody through
  Chrono::Vehicle, consuming the same schema. A rigid-ring tire in the MF-Swift style. OpenCRG
  surfaces.

**The split between slow and fast states**, which every tier uses:

- *Fast*: chassis velocities, wheel speeds, tire relaxation, actuator lags.
- *Slow*: tire temperatures at the surface, carcass, and gas; tread wear; thermal damage; brake
  disc temperatures; fuel mass; battery SOC and temperature.
- In the QSS tiers, T0 and T1, the slow states integrate **from segment to segment, by explicit
  Euler over the quasi-static solution**. That is what makes the QSS tier *capable of running a
  stint*, which is unique in OSS.

### 6.2 The Block abstraction

A **Block** is three things: immutable parameters, states, and typed ports on a flat
struct-of-arrays signal Bus.

A block declares which ports it reads and writes. The assembler sorts the blocks topologically,
once, when the model is built. There is therefore no graph at run time, and no virtual dispatch in
the inner loop. Dispatch is through an enum.

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

The block set is: `Chassis`, in 7-DOF and 14-DOF variants; `Tire` ×4; `Aero`; `Suspension`, as
lumped K&C; `Brakes`; `Ice`; `ElectricMachine`; `Gearbox`; `EnergyStore`; `EnergyManager`; and
`Driver`.

**An F1 car and a hatchback differ in data alone.** They use the same blocks with different
parameter files. A subsystem that a car does not have, such as ERS on a hatchback, simply does not
instantiate.

### 6.2b The configuration backbone (Locked Decisions #37–47): how variety stays fast AND friendly

**The input quartet.** Every run is `vehicle.yaml + track.yaml + conditions.yaml + sim.yaml`. The
last two are optional, and fully defaulted. Car identity, road, environment, and numerics never
mix.

**The assembly pipeline** runs once for each model load. It never runs in the loop.

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

After assembly, the hot loop touches **zero** strings, hashes, and configuration logic. Load time
pays for all the variety.

The loaded-model report states what was inherited, what was estimated, and what was degraded. It
prints with every run, and it embeds in every artifact. *Nothing is silent.*

**The step phases** are `sense → control → actuate → integrate`.

A controller — for torque vectoring, ERS deployment, brake bias, or shift logic — is a first-class
swappable block. It runs in the `control` phase, on the same bus. It is written in **Rust or C-ABI
only (#38). No Python inside a timestep, ever.**

To experiment with a custom control strategy, write a Rust controller block, which is a plugin
point, or pre-compute a control schedule `u(s)` as data.

The ERS energy manager, in `outlap-powertrain` from M6 PR1, is a pure struct with a policy enum.
The control phase of each tier consumes it. It is deliberately NOT plugin-registration machinery.
That machinery is the plugin surface after 1.0.

**The two-layer control contract (M6 PR4).** The **controllers** at a step boundary — the shift
FSM, the battery slow stack, and the ERS energy manager — decide ONCE for each step, at the
boundary, and publish frozen bus channels.

The **blocks** for each stage stay pure consumers. They read those channels on every evaluation of
the RHS. The bus is cleared and rebuilt on each evaluation, so a boundary value must be re-published
on every evaluation, and never once for each step.

The manager reaches the blocks in exactly this way. It publishes an additive deploy force for the
MGU-K, plus the realized electrical deploy and harvest, which the powertrain block republishes as
the draw on and charge into the pack.

The energy ledger for each lap accumulates on the fast path, from the bus values after each step. It
resets at the start and finish line.

Both tiers consume only the manager's `ErsCommand`. Tier parity, gate #4, therefore compares one
implementation of the regulations. It never compares two copies.

**There are exactly three plugin points (#37).** Everything else is a core enum, which is fast and
curated.

1. Custom blocks, through a Rust trait and registration at compile time. A plugin crate depends on
   `outlap-core` and registers its blocks. Users then build a custom binary, or the project
   upstreams the block.
2. Tire models, through the stable C-ABI "Standard Tire Interface". It is CPU-only by contract.
3. Controllers, through the same trait mechanism, as blocks in the `control` phase.

**Programmatic use (#44).** Anything that accepts a file path equally accepts the validated
in-memory object. Sweeps use dotted-path overrides (#35). An optimizer never touches the
filesystem.

### 6.3 The racing line (Locked Decision #14)

V1 ships a **generator for the minimum-curvature line**. It solves a QP over the lateral offset
n(s), within the track bounds, minimizing ∫κ². The formulation is TUM-style, re-implemented from
the papers, and solved on the 3D ribbon.

A user may also supply their own line, as `raceline.csv`, in the same format indexed by s. Every
lap result records which line it ran.

Co-optimizing the line and the speed for minimum lap time, over a free trajectory and by
collocation OCP, is deferred until after v1. The minimum-curvature line is the fair common
denominator for a comparison. Each vehicle variant gets its own generated line; see the hero
demonstration in §12.

---

## 7. Physics Models

### 7.1 The tire force backbone

- **MF6.1** (Pacejka 2012, including the Besselink terms for inflation pressure), clean-room from
  the book. It covers steady-state Fx, Fy, Mz, and Mx, with combined slip by cosine weighting.
  Turn-slip is omitted in v1.
- **Transient behavior**: first-order relaxation on each slip channel,
  σ_κ κ̇ + |vx| κ = |vx| κ_ss, and the same for α. σ comes from the PTX and PTY coefficients, or
  from a carcass stiffness that depends on Fz.
- **The brush model**, with 5 parameters: Cκ, Cα, μ0, patch length, and pressure profile. It ships
  as the tier for low data, which suits passenger cars and users with no `.tir` file. It is also
  the physical scaffold that the thermal model hooks into, identically.
- A parser and writer for `.tir`, plus a fitting pipeline built on scipy, which ingests the TTC
  format for members. Both live in the Python layer.

### 7.2 The tire thermal ring model — FLAGSHIP (3+1 nodes for each tire, a reduced Farroni-TRT)

The states are **T_s**, the tread surface; **T_c**, the tread bulk or carcass; and **T_g**, the
inflation gas. The rim is a parameter, or an optional fourth node.

```
C_s·dT_s/dt = Q_fric − G_sc(T_s−T_c) − h(v)·A_ext·(1−a_cp)·(T_s−T_air) − G_road·a_cp·(T_s−T_road)
C_c·dT_c/dt = Q_hyst + G_sc(T_s−T_c) − G_cg(T_c−T_g)
C_g·dT_g/dt = G_cg(T_c−T_g) − G_gr(T_g−T_rim)
```

The drivers are:

- Friction power, `Q_fric = p_t·(|Fx·v_sx| + |Fy·v_sy|)`. The sliding velocities come from the
  slip. The partition p_t is about 0.6 to 0.7 into the tread, and the rest goes to the road.
- Hysteresis, `Q_hyst = c_h·Fz·δ_tire(Fz,p)·Ω`. This is the form for loss of strain energy against
  deflection rate. Fit c_h to rolling-resistance data.
- Convection, `h(v) = h₀ + h₁·v^0.8`, with the contact-patch fraction
  `a_cp = A_cp(Fz,p)/A_ext`.

Three couplings run back to the force model:

1. The gas law, `p = p_cold·T_g/T_cold`, which drives the native pressure terms of MF6.1, and
   therefore the stiffnesses, μ, and patch size.
2. The grip window, `λ_μ(T_s) = exp(−c_T·((T_s−T_opt)/T_opt)²)`, which scales LMUX and LMUY. An
   asymmetric option would give separate widths on the cold and hot sides.
3. Carcass softening: PKY1 and PKX1, each times `(1 − k_c(T_c − T_c,ref))`.

### 7.3 The law for wear and degradation — FLAGSHIP (two states)

- **Tread depth w**, in mm. It follows the Archard form on frictional power,
  `dw/dt = (k_w / H(T_s)) · Q_fric / A_cp`. Hardness H decreases as T_s rises, so a hot tire wears
  faster.

  It has two effects. It multiplies μ by `f_w = 1 − c_w1·(w/w_max)`. And **the tread mass falls, so
  C_s(w) shrinks, so a worn tire runs hotter. That is the physical positive feedback that produces
  the cliff.**
- **Thermal damage D ∈ [0,1]**, which is irreversible:
  `dD/dt = (1/τ_D)·⟨(T_c−T_deg)/ΔT_ref⟩₊^β`. It is an Arrhenius-like proxy for devulcanization.
- **The total grip factor, including the cliff**:
  `λ_μ,total = λ_μ(T_s) · f_w · (1 − Δ_c·σ((w−w_c)/s_w)) · (1 − Δ_D·D)`. The sigmoid σ gives a
  collapse in pace at the critical wear w_c that is sharp, but still C¹.
- **Calibration.** The thermal parameters come from the published values of Farroni, scaled by tire
  size. `T_opt` and `c_T` come from the class of compound. `k_w` and `w_c` are calibrated
  *inversely*, from stint pace data in FastF1, so that the model reproduces a decay of about
  0.05 s to 0.10 s per lap for a compound, and the cliff lap that was observed.

Everything above is implementable from public literature. No math here is proprietary (§15).

### 7.4 Aero

The map object is `{C_z,front, C_z,rear, C_x} = f(h_front, h_rear, yaw [, roll, DRS_flag])`. It is
a gridded lookup with a fit that is regularized to stay monotone. It is evaluated at dynamic ride
heights in T3, or at equilibrium ride heights in T1 and T2.

Sensitivity to yaw makes the gg diagram asymmetric in mid-corner. A passenger car degenerates to a
constant CdA and ClA. This is the first open representation of an aero map over ride height (§5.5).

### 7.5 Suspension (v1 uses lumped K&C, not hardpoints)

For each axle: ride rate, share of roll stiffness, roll-center height for the geometric transfer,
and anti-dive and anti-squat.

For each corner: tables of camber against heave and roll, and of toe against heave, Fy, and Mz.
Those cover kinematic steer and compliance steer.

This is credible for motorsport, because it reproduces the two things that dominate handling: the
distribution of load transfer, and the trajectories of camber and toe.

A preprocessing tool from hardpoints to K&C is a later project, sized for the community. It is OSS
gap #8.

### 7.6 Brakes

The pedal maps to a total torque through the balance bar, plus dynamic bias, plus blending of
regeneration with the MGU-K.

Each corner has a disc thermal node: `C_d·dT_d/dt = T_br·ω − h_d(v)·A_d·(T_d−T_air)`. Pad fade
comes from a `μ_pad(T_d)` table.

A road car also gets a simple ABS flag at the slip limit.

### 7.7 The driver model (T2 and T3)

There are two loops.

- **Steering**: preview points on the target line, in the style of MacAdam, plus a curvature
  feed-forward, `δ_ff = κ(L + K_us·v²)`.
- **Speed**: a PI that tracks the **QSS speed profile from T0 or T1**, with a feed-forward on the
  gg headroom. It also takes lift-and-coast and ERS deployment inputs from the energy manager.

Using the QSS profile as the reference for the transient driver makes tier parity a built-in
regression test.

---

## 8. Powertrain, ERS (2026 Rules), Battery

All hardware that produces power enters as a **map**. That is the firewall of §1. The blocks and
states follow.

### 8.0 The drivetrain topology graph, which is the backbone of versatility

**The powertrain is a directed graph, not a fixed layout.** Torque **sources**, which are ICEs and
electric machines or drive units, connect to wheel **sinks**, through **coupler** elements: a
gearbox, a clutch, a fixed ratio, a differential, or a direct connection to one wheel.

Any concept with four wheels is therefore a topology plus data:

| Concept | Topology |
|---|---|
| 1-DU RWD EV | DU → open/LSD diff → RL+RR |
| 2-DU AWD EV | DU_f → front diff → FL+FR; DU_r → rear diff → RL+RR (front/rear split = data or controller) |
| 4-DU torque-vectoring EV | DU×4 → one wheel each (TV controller allocates) |
| FWD hatchback | ICE → gearbox → front diff → FL+FR |
| GT hybrid 80/20 | ICE → gearbox → rear diff → RL+RR; EM (P2 or axle) in parallel — split ratio is data |
| F1 2026 | ICE + MGU-K on the same shaft → gearbox → rear diff → RL+RR |

The schema is `drivetrain.units[]`. Each unit declares
`{source: <.ptm ref>, path: [couplers...], wheels: [...]}`. The assembler validates the graph at
load time: every wheel must be reachable, and no ratio may conflict.

**The control layer is rule-based in v1**, per Locked Decision #2.

- Static split ratios, front to rear and left to right, as data.
- Models for the differential: open, locked, LSD with preload and ramp, and **solid**, which suits
  a kart or a historic solid axle and is the limit case of a locked differential (Decision #47).
  This enters the torque split of the double-track model.
- **Torque vectoring**, through a controller proportional to yaw moment:
  `ΔM_z = K_p·(r_target − r)`, with `r_target = v·κ_ref`, or with the steady-state yaw gain. It is
  allocated across the available per-wheel sources, within the friction ellipse and the machine
  envelope. The gains are vehicle data.
- Blending of regeneration and friction braking hooks into the same allocator (§7.6).
- The interface of the allocator is designed so that a QP-based optimal allocation — per-wheel
  torque over the friction ellipses — can replace the rule-based one after v1, without touching the
  topology graph.

**The hero demonstration ships with v1, in M7.** One EV chassis, four drivetrain files — 1-DU RWD,
2-DU AWD, 4-DU TV, and FWD — on the same track. One notebook then compares their optimal laps and
their energy consumption.

### 8.1 ICE

- A torque map T(n, throttle), and a fuel-flow map ṁ_fuel(n, T), or a BSFC map.
- One state: fuel mass. It feeds vehicle mass and the migration of the CG.
- An optional constraint on fuel flow, the F1-style ṁ_max, as configuration.

### 8.2 Gearbox and driveline

- Ratios and a final drive. An efficiency map, or a constant. A shift time with an interruption of
  torque, modeled as a discrete event and a small state machine: a timer for the torque cut, then
  the ratio swap, then the clutch ramp.
- A differential, with open, locked, and LSD-with-preload-and-ramp as the v1 options. This enters
  the torque split of the double-track model.

### 8.3 ERS in the style of 2026 Formula 1, with the MGU-K ONLY (there is no MGU-H in the current regulations)

**A design decision of 2026-07-02: the MGU-H is removed from the architecture entirely.** The 2026
F1 regulations for the power unit deleted it. For a car outside F1 it never existed.

What remains is one electric machine on the crank or an axle, which is the MGU-K, plus an energy
store. The rules for deployment and recovery are *data*:

```yaml
ers:
  mgu_k: mguk.ptm.yaml            # torque/speed/efficiency map, bidirectional
  es:                              # energy store limits (battery physics lives in §8.4)
    capacity_MJ: 4.0               # the USABLE WINDOW energy (C5.2.9: max−min SoC ≤ 4 MJ on
    soc_window: [0.2, 0.9]         # track; the regs set no total capacity — pack sizing is design)
  deployment:
    power_limit_kW: 350            # ELECTRICAL DC power at the CU-K bus, both directions (C5.2.7)
    taper_vs_speed:                # C5.2.8i: P(kW)=1800−5v to 340 kph, 6900−20v to 345, 0 beyond;
      speed_kph:  [0, 290, 340, 345]           # as breakpoints: knee EXACTLY 2/7 at 340; the
      power_frac: [1.0, 1.0, 0.2857142857142857, 0.0]  # 0–290 plateau IS min(cap, curve)
    # NO per_lap_deploy_MJ in 2026 (C5.2, absence verified): deployment is bounded only by the
    # power curves + SoC window. The field stays as generic config and is NEVER estimated.
  override_mode:                   # "Overtake" (2026 Manual Override Mode)
    power_limit_kW: 350
    taper_vs_speed:                # C5.2.8ii: P = 7100 − 20v, zero at ≥355 → full power to 337.5
      speed_kph:  [0, 337.5, 355]
      power_frac: [1.0, 1.0, 0.0]
    extra_energy_per_lap_MJ: 0.5   # extra HARVEST allowance while activated (C5.2.10iii) — a
                                   # Recharge bonus, NOT a deployment budget
    activation: strategy           # stage-2 hint (Detection Gap, B7.2); the per-run flag wins
  recovery:
    braking_power_limit_kW: 350    # electrical, at the same C5.2.7 bus (mech ≈ 360.8 kW at cap)
    per_lap_harvest_MJ: 8.5        # C5.2.10 Recharge budget; ALL harvest paths count against it
    recharge_phases: true          # part-throttle harvest + full-throttle ICE back-drive
    recharge_target_soc: 0.55      # optional; the ECU's "Recharge target" (default mid-window)
    ramp_initial_step_kW: 150      # optional C5.12 ramp bounds (defaults 150 / 50 / 700):
    ramp_rate_kW_per_s: 50         #   initial demand step, then rate-limited reduction,
    ramp_total_kW: 700             #   episode total (= the full +350 → −350 swing)
  elec_mech_factor: 0.97           # optional; the fixed C5.2.14 electrical→mechanical correction
```

The figures were verified against **FIA 2026 Section C [Technical] Issue 19 (2026-06-25)** and
**Section B [Sporting] Issue 07 (2026-06-25)**. `docs/theory/ers-energy-manager.md` cites the
article numbers.

The *mechanisms* are the architecture: the speed taper, the override mode, the Recharge budget for
each lap, and the recharge phases. The numbers are configuration data, and most are per-event
parameters (B7.2.1b).

The tapers are regulatory closed-form piecewise-linear lines, and they are evaluated as such. That
is the recorded exception to Decision #30. A Hermite through the breakpoints bows up to 78 kW above
C5.2.8i at 315 kph.

Every cap and budget lives on the electrical side, at the DC bus of the CU-K, with ONE conversion
seam: 0.97, from C5.2.14 and C5.2.21. The ledger for each lap integrates electrical energy.

The control vector for energy management, for each track segment, is
`u(s) = [deploy/regen ∈ [−1,1], override_flag, lift_point, shift_map_id]`. It is accepted as a
data-driven schedule, which is an API input and not part of the vehicle schema.

V1 ships rule-based deployment: a feed-forward policy that deploys below the taper speed, harvests
under braking, and recharges on designated straights. Deployment is greedy and gated by demand,
with no SoC input. `recharge_target_soc` steers the automated Recharge paths. V1 also ships
configurable integral constraints. The strategy optimizer of stage 2 writes u(s).

A hybrid outside F1 is the same block with different data. An LMDh is a single 50 kW MGU on the
rear axle. A road PHEV is a P2 machine with a large ES. On a pure EV, the MGU-K *is* the
powertrain, and there is no ICE block.

### 8.4 Battery

The model is a Thevenin equivalent circuit, ported from NREL `thevenin` (BSD-3).

Its states are [SOC, V_RC1 (,V_RC2), T_batt]. Its parameters are OCV(SOC,T), R0(SOC,T), R1(SOC,T),
and τ1(SOC,T), plus the entropic-heating term dU/dT.

It has a lumped thermal node, heated by I²R and by entropic heating. Power derates against T_batt
and against the SOC window. The pack scales as Ns×Np.

The PDT importer for a BatteryPack (§10.4) fills this block directly.

### 8.5 The machine thermal model: an N-node LPTN in `emotor.yaml` (Locked Decision #25, amended 2026-07-05)

**AMENDMENT of 2026-07-05, authorized by the author, in M3 PR5. The model below is generalized,
from a fixed 2-node network to an *N*-node LPTN declared in data. outlap now also *builds* the
operator from machine internals, on the detailed path.**

The heat-transfer correlations — the air-gap film, convection in the end cavity and at the shaft,
and the liquid-jacket channel — are ported into `outlap-thermal`, and evaluated **on each segment**
at `(ω, T)`.

The network state advances with a semi-implicit **Crank–Nicolson** step, which is A-stable. There
is one pinned ambient node, and an optional coolant node, closed by a quasi-static jacket balance.

This is a deliberate, narrow reversal of the firewall, for the thermal model only, which the author
owns. See Decision #25.

Two tiers of authoring share the integrator. The **lumped** tier is a hand-authored model with a
reduced set of nodes, whose `C` and `G` values are filled by mass heuristics, flagged as estimates,
with constant conductances. The **detailed** tier is imported, with the full node set, explicit `C`
values, and convection edges.

The treatment of derating and of loss, below, is unchanged. The rationale for the original 2-node
design is retained here for context:

**The design rationale, from the author's correction.** A community user typically has only a **peak
torque envelope and loss data** for their machine. They do not have continuous or overload
envelopes.

outlap therefore does not *consume* curves of thermal capability. It *computes* capability from
losses, with a deliberately simple **2-node lumped thermal network**, parameterized in an
`emotor.yaml` for each machine (§9.5), which `vehicle.yaml` references.

This is explicitly NOT of PDT grade. The thermal sub-stage of PDT is a 19-node LPTN, with FEA-region
geometry and Nusselt correlations for the coolant channel. That fidelity stays on PDT's side of the
firewall. outlap integrates whatever small network the data declares. It never *builds* one from
machine internals.

The states are **T_w**, the winding, which is fast; and **T_c**, the case or stator lump, which
couples to the coolant:

```
C_w·dT_w/dt = split_w·P_loss(τ, n, T_w) − G_wc·(T_w − T_c)
C_c·dT_c/dt = (1 − split_w)·P_loss(τ, n, T_w) + G_wc·(T_w − T_c) − G_cool·(T_c − T_coolant)
```

- `P_loss` comes from the loss map in the `.ptm` file (§9.2). If that map carries a *breakdown* of
  loss, by winding, core, and so on, as PDT exports, then `split_w` is computed at each operating
  point instead of being a constant.

  There is an optional feedback from copper resistance:
  `P_loss_w ∝ 1 + α_cu(T_w − T_ref)`. `α_cu` lives in `emotor.yaml`, and it defaults to off for
  users who have only a map.
- **Derating.** The limit on commanded torque scales linearly from 1 to 0, as each node crosses
  from `T_warn` to `T_max`. The winding limit normally binds first. These are slow states in both
  tiers (§6.1). Lap 1 therefore differs from lap 20, and a stint is honest.
- There are about 8 user parameters in total. A sensible default for every one is derivable from
  machine mass alone, through documented heuristics such as `C_w ≈ 0.15·m·c_cu`. Each is clearly
  labeled as an estimate.
- If the `.ptm` file *does* carry continuous or overload envelopes, as a PDT import will, they are
  used as **validation data**. CI fits nothing. It warns when the continuous capability that the
  2-node model derives disagrees with the imported envelope by more than a stated band.

---

## 9. File Formats — The Product Contract

**The pattern.** A YAML document, validated by a published JSON Schema. Bulk numeric tables live in
a sidecar, as CSV or Parquet. A vehicle is a directory, or a zipped `.apx` bundle. This is the glTF
pattern: a readable scene plus binary buffers.

Why not the alternatives. XML, or anything URDF-like, is hostile to numeric arrays and diffs
poorly. TOML is unreadable at the nesting depth a vehicle needs. Pure JSON has no comments. And a
format that is only HDF5 is opaque to git and to PR review, which is fatal for a community data
registry.

**Versioning.** Every file carries `schema: <name>/<MAJOR.MINOR>`. A loader accepts the same major
version. An unknown field that does not start with `x-` is a hard error, which catches typos.
`outlap migrate` ships the migrations.

### 9.1 `vehicle.yaml` (a sketch)

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
    - source: engine.ptm.yaml                 # a combustion engine or an electric machine
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

### 9.2 `.ptm.yaml` — the neutral contract for a powertrain map (THE FIREWALL)

```yaml
schema: ptm/2.0
kind: electric                # combustion | electric (the energy source; ptm/2.0)
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

`kind` states the ENERGY SOURCE and nothing else: `combustion` or `electric`, in ptm/2.0.

Every map is referenced at the shaft that its drive unit outputs onto. A map authored at the
machine's own shaft declares the reduction in the unit's `fixed_ratio:` field (vehicle/2.1).

### 9.3 `track.yaml` + `centerline.csv`

The columns are `s_m, x_m, y_m, z_m, banking_deg, width_left_m, width_right_m, grip_scale`. This is
deliberately the first open 3D format for a racetrack.

**The track importer**, written in Python, was elevated in importance by the full-3D decision, #13.
No open 3D circuit data exists, so the importer builds it, in three steps.

1. It takes the centerline and the widths from OpenStreetMap, which is ODbL and therefore
   redistributable with attribution, or from TUMFTM CSVs, which are LGPL, 2D, and for bootstrap
   only.
2. It **fuses elevation from open DEMs** — Copernicus GLO-30, USGS 3DEP, or national LiDAR where
   that exists. It samples them along the centerline, and smooths them consistently with the
   spline. Both z and its derivatives must be C², for vertical curvature.
3. It estimates banking from cross-track sampling of the DEM, where resolution allows. Otherwise
   banking is annotated by hand, for each corner. The format allows sparse banking keypoints,
   interpolated in s.

Document the provenance and the accuracy class of each track, in the `meta` of its `track.yaml`.
Later importers will read `.xodr` and OpenCRG.

### 9.4 `.tyr.yaml`

This holds an MF6.1 coefficient block, which is a superset of `.tir` and round-trips to it. It adds
a `thermal:` block with the parameters of §7.2, a `wear:` block with the parameters of §7.3, and
fields for provenance and citation.

### 9.5 `emotor.yaml` — parameters for the machine thermal model (Locked Decision #25)

These are the *only* variables that the simple 2-node model of §8.5 needs. A community user can
fill this from a datasheet and a scale:

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

Every field has a documented default or heuristic, based on mass and labeled as such. A minimal
file is therefore just the node masses, the coolant temperature, and the winding `T_max`.

### 9.6 `conditions.yaml` — session conditions (Locked Decision #46)

This is the fourth input of the quartet: the same track, on a different day.

It has full ISA defaults — 20 °C, 1013.25 hPa, and no wind — so it is optional:

```yaml
schema: conditions/1.0
air: {temperature_C: 28, pressure_hPa: 1005}     # → density for aero
wind: {speed_mps: 3.5, direction_deg: 240}       # constant vector in v1
track_surface_C: 41                              # tire thermal boundary (T_road, §7.2)
ambient_C: 28                                    # thermal models' ambient / coolant pre-rad proxy
```

### 9.7 `sim.yaml` — simulation settings (Locked Decision #42)

This file is optional. Every field has a default. The CLI and the API override the values in the
file. The **resolved** settings embed in every result artifact:

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

**Presets, through `extends:`, per Decision #41.** `data/presets/*.yaml` ship with the repository:
formula_base, gt_base, and passenger_base. They are ordinary fragments of the vehicle schema. §6.2b
describes the pipeline that deep-merges them, validates after the merge, and prints the
loaded-model report.

---

## 10. PDT HDF5 Importer Specification

**Purpose.** A user of the author's professional toolchain, called "PDT", can import their results
as `.ptm` files and battery files. PDT is a design pipeline for motors, drive units, and battery
packs, and it writes one HDF5 file for each stage.

The importer is a **pure-Python adapter**, at `python/outlap/importers/pdt_h5.py`. It reads with
`h5py` and nothing else. It never imports PDT code. That is the firewall of §1.

Three stage files matter. Their actual schemas were inspected on 2026-07-02, from these reference
files:

- EDrive: `EDrive_121.0L_16Et_650.0I_400.0V_12ea1_SynRM_ref.h5`, a 136 kW SynRM traction machine
- DriveUnit: `DriveUnit_16.2GR_168NM_369RPM_666f3_R250_ref.h5`, a small 48 V geared actuator unit
- BatteryPack: `BatteryPack_13S_3P_722Wh_48V_7158a_cleanTest2Bot.h5`

### 10.1 Conventions common to PDT HDF5 files

- **The tree is tagged by type.** Most groups carry the attributes `__mdt_type__`, with values such
  as `OperatingGrid`, `MotorInfo`, and `PeakCapability`, and `__mdt_module__`, with values such as
  `models.edrive_types`. Use them to validate and to dispatch. Do not depend on their existing.
- Strings arrive as HDF5 objects or bytes, so decode them as UTF-8. Scalars are float32 or int64.
  Arrays are float32. Some arrays are **complex128**, which are phasors, and the import does not
  need them.
- `compute/<StageName>/` holds provenance: the git commit, the host, and a timestamp. `hash/` holds
  keys for pipeline lineage. `metrics/` is a flat dump of camelCase scalars for a database. It is
  **redundant. Always prefer the structured groups.**
- **⚠ A pitfall with units, verified in the real files.** The summary scalars are inconsistent
  between files.

  For example, the EDrive file has `performance/peak_power = 135872.84`, which is in **W**, while
  its sibling `performance/power_at_base_speed = 135.87` is in **kW**. And the DriveUnit file has
  `performance_at_vdc/peak_power = 1.585`, which is in **kW**.

  **The rule: never trust a summary scalar. Rebuild power from the arrays, as τ[Nm] × ω[rad/s].**

  The units in the arrays are reliable. Speed axes are in **RPM** (`sweep/speed`). `omega` and
  `rotor_speed_rad` are in rad/s. Torque is in **Nm**, losses in **W**, efficiency from **0 to 1**,
  voltage in **V**, current in **A**, temperature in **°C**, mass in **kg**, inertia in **kg·m²**,
  and lengths in **mm**.

### 10.2 The EDrive stage file → `.ptm` (`kind: electric`, machine and inverter at the motor shaft)

The layout below was verified. Each line gives a dataset, its shape, and its meaning.

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

**The conversion algorithm:**

1. Pick the vdc slice nearest the system voltage that the user declares. Or interpolate across vdc.
2. Re-grid from `load_ratio` to torque. At each speed, `shaft_torque[vdc, n, :]` is monotone in
   load_ratio. Invert it, to get efficiency(τ, n) on a regular torque axis, in both quadrants.
3. Set `max_torque_Nm_vs_speed` from `peak_capability/torque_drive[vdc_idx]`. Set
   `max_regen_torque_nm_vs_speed` from `|peak_capability/torque_regen[vdc_idx]|`, which is the
   MEASURED envelope in the 4th quadrant (ptm/2.0). An imported machine never falls back to the
   assumption of a symmetric machine. Set `cont_torque` from `thermal/continuous/torque`, and the
   overload curves from `thermal/peak/torque`.
4. Mask the infeasible cells as NaN in the parquet table. A cell is infeasible when efficiency is 0
   AND |torque| exceeds the envelope.
5. Emit `machine.ptm.yaml` and `maps.parquet`. Stamp
   `meta.source: "PDT EDrive <alias> <git hash from compute/EDrive>"`, and `meta.dc_voltage_V`.
6. **Emit `machine.emotor.yaml`, by distilling the thermal model (Decision #25).**

   The PDT file carries a full 19-node LPTN under `thermal_obj/`. It holds `C (19,)` for the node
   capacitances, `G_const (19,19)` for the conductance matrix, `R_active`, `R_endturn`,
   `cu_temp_coeff`, and a `cooling/` group with `coolant_inlet_K` plus geometry and fluid
   properties. That is far more than outlap wants.

   The importer therefore **distills** it to the 2-node parameters, by least squares. It chooses
   (C_w, C_c, G_wc, G_cool) so that the 2-node model, driven by the exported loss maps, reproduces
   two things: the `thermal/continuous/torque` envelope that PDT solved, and the overload torques
   at 10, 20, and 30 s, at 3 to 5 speeds.

   Three values are copied directly. `coolant_temp_C` is `cooling/coolant_inlet_K` − 273.15.
   `copper_alpha_per_K` is `cu_temp_coeff`. The winding split comes from
   `operating_grid/loss_breakdown` at each point, comparing winding_stator against core_total
   against inverter. Inverter losses route to the case node.

   Mark `meta.source: pdt-distilled`, and put the fit residuals in `meta.notes`.

   The undistilled envelopes also land in the `limits:` block of the `.ptm` file, as validation
   references (§9.2).

### 10.3 The DriveUnit stage file → `.ptm` (`kind: electric`, motor, inverter, and gearbox at the OUTPUT shaft)

The layout below was verified:

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

**The conversion** follows the same recipe for re-gridding as §10.2, but on `opt_op/torque` and
`du_eff`, at the output shaft.

Set `kind: electric`. Set `max_regen_torque_nm_vs_speed` from `|peak_op/torque_regen[vdc_idx]|`,
which is the MEASURED envelope in the 4th quadrant (ptm/2.0). Set `inertia_kgm2` from
`at_output_j_kgm2`. Record `info/gearbox/gear_ratio` in `meta`, for information only, because the
ratio is already applied. Set `drag_torque` from `no_load/torque_drag`, interpolated onto the speed
axis.

In a race car, this block maps to a hub drive, a corner drive, or a whole e-axle.

### 10.4 The BatteryPack stage file → parameters for the battery block

The layout below was verified. It is a small file, with about 190 datasets.

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

**The conversion** emits a `battery.yaml` for the block of §8.4. That is a 1-RC ECM with bilinear
tables over (SOC, T) for OCV, R0, R1, τ1, and dU/dT. The pack topology ns×np comes from `info`. The
SOC window comes from `info/min_soc` and `info/max_soc`. The power limits come from
`pack/peak_*_power(SOC)`. The lumped thermal node comes from mass·cp and `thermal_resistance`.

### 10.5 Design of the importer, and its CLI

```
outlap import pdt-edrive     <file.h5> -o machine.ptm.yaml   [--vdc 400]
outlap import pdt-driveunit  <file.h5> -o du.ptm.yaml        [--vdc 48]
outlap import pdt-batterypack <file.h5> -o battery.yaml
```

Four rules govern the implementation.

Use `h5py`, `numpy`, and `pyarrow`, and nothing else.

Tolerate a missing optional group. PDT files evolve, so key on the presence of a dataset, not on
`__mdt_type__`.

After export, validate the emitted files against the published JSON Schemas.

The round-trip test loads the emitted `.ptm` in the Rust core, and reproduces at least 3 spot
efficiencies from the source arrays, to 1e-6.

Keep small golden fixtures. Generate tiny synthetic h5 files, shaped like PDT files, in the test
suite. Do **not** commit a real PDT file. They are the author's private data.

---

## 11. Execution Architecture

### 11.1 The split between languages (committed decisions)

- **The core is Rust.** There are five reasons. The OSS niche in Rust is vacant, which is both a
  differentiator and a magnet for contributors. The permissive substrate exists, in diffsol,
  nalgebra, and rayon. Every relevant C++ project is a formulation reference, not a linkable
  dependency, so a C++ ABI buys nothing. Cargo and maturin make solo maintenance viable. And WASM
  and portable GPU are native.
- **Python is for configuration time.** It holds the API façade, schema validation, tire fitting
  with scipy, the importers of §10, the adapters for FastF1, plotting, and notebooks. **Nothing
  Python runs inside a timestep.**
- **C is used in one place.** A stable `extern "C"` vtable for a tire-model plugin, with init,
  eval, and advance. This is the open "Standard Tire Interface"; the closed reference is Adams STI.
  A third party can then write a tire model in C, C++, or Fortran, without touching the core. It is
  CPU-only by contract.

  Note one consequence of AGPL: a *distributed* proprietary plugin is effectively a derivative
  work. That matches the author's intent, and internal or private use stays unrestricted. An
  exception for plugin linking can be added later, if it is ever wanted, because the author holds
  the copyright.
- **The license is AGPL-3.0** (Locked Decision #7; §15 gives the policy details). The published
  JSON Schemas of §9 are licensed separately and permissively, under Apache-2.0. *Other* tools can
  therefore adopt the file formats with no concern about copyleft. The formats should spread even
  where the code cannot.

The workspace layout:

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

### 11.1b The user-facing API surface (Locked Decisions #17–19)

- **Results are xarray Datasets.** A channel log has dims `(s | time)`. A per-wheel channel adds
  `(wheel)`. A comparison adds `(variant)`. A sweep adds one dim for each swept field. Units live
  in attrs. Export goes through `.to_parquet` or netCDF. Where possible the data is zero-copy from
  the Rust batch buffers, through rust-numpy.
- **The sweep API is first-class.** Call
  `outlap.sweep(vehicle, track, over={"aero.map.scale": [...], "drivetrain.control.split.front":
  [...]})`. It runs a batch in parallel with rayon, and returns an xarray cube.

  There is also a documented **cost-function interface**: a callable that takes vehicle overrides
  and returns one or more scalars. An example notebook uses pymoo and optuna. The optimizers stay
  on the user's side. There is no optimizer framework in v1 (Decision #18).
- **The CLI works. It is not decorative.** The commands are
  `outlap lap car.yaml track.yaml [--line min_curv]`,
  `outlap compare car_a.yaml car_b.yaml track.yaml`,
  `outlap import pdt-{edrive,driveunit,batterypack}`, `outlap validate <file>`, and
  `outlap migrate <file>`.

  Output is parquet, plus optional PNG plots. The CLI wraps the Python API one to one. There is no
  separate code path.

### 11.2 Numerics

- **T0 and T1, the QSS tiers**, are not an ODE. They run damped-Newton trim solves, which allocate
  nothing, plus forward and backward velocity passes. The slow states advance on each segment;
  explicit Euler is exact enough at timescales of 10 s to 100 s. The target is a full lap in under
  50 ms.
- **T2 and T3, the transient tiers, use a fixed-step split integrator.** It is NOT adaptive, and it
  is NOT plain RK4.
  - The chassis and driveline use Heun, which is RK2, at dt = 1 ms. The stepper is generic over the
    Butcher tableau, and RK4 is selectable for convergence studies.
  - **Tire relaxation uses an exact exponential update**,
    `κ ← κ_ss + (κ−κ_ss)·exp(−V·dt/σ)`. It is stable at all speeds, without condition, and it
    removes the stiffness with no implicit solve. This is the single most important decision about
    the integrator.
  - Thermal state, wear, and SOC use semi-implicit Euler on the diagonal decay terms.
  - **diffsol, with BDF and ESDIRK, is the verification integrator in CI.** The production stepper
    must converge to the reference solution at O(dt²).
- **The algebraic loop in load transfer (Locked Decision #29)** is exposed as a *simulation
  setting*.

  `fz_coupling: one_step_lag` is the default. The accelerations from the previous step feed the
  load transfer, and they may optionally be low-pass filtered.

  `fz_coupling: fixed_point` runs 2 to 3 damped iterations of Fz, then forces, then acceleration,
  on each step. It suits a user who wants tighter coupling near the grip limit.

  Both are deterministic, and every result artifact records which one ran.
- **The interpolation standard (Locked Decision #30)** is ONE shared implementation: monotone cubic
  Hermite, in the Fritsch–Carlson form, on rectilinear grids, C¹ with analytic derivatives. Every
  gridded map uses it: aero, efficiency and loss, envelopes, and the tire thermal parameters. No
  block chooses its own interpolation.

  **Amended in M6 PR1.** A regulatory *closed-form piecewise-linear formula* is evaluated by the
  shared exact piecewise-linear interpolant, `outlap_core::PiecewiseLinear`. That covers the FIA
  C5.2.8 ERS tapers and the C5.12 ramp bounds. A Hermite through a breakpoint set with a flat
  plateau bows up to 78 kW above the regulation line. This applies to closed-form regulations only.
  A gridded map stays on the Hermite.
- **The envelope against the slow states (Locked Decision #31).** T0 consumes a dense base table,
  `gg(v, ax, g_normal)`, at the reference state. It then applies separable multiplicative
  corrections, from T1 sensitivities: ∂gg/∂μ_tire, ∂/∂mass, and ∂/∂ClA, taken at reference points.
  CI validates the corrected envelope against full T1 re-solves, at sampled off-reference states.

  **Amended in M5 PR4 (Decision #49).** The thermal state and wear state of the tire are instead
  carried as *genuine* re-solved grid axes, `gg(v, ax, g_normal, T_tire, wear)`, through the opt-in
  `generate_with_tire_state`. The `(T_opt, 0)` slice is bit-identical to this base table. μ_tire,
  mass, and ClA stay separable corrections.
- **Events** — gear shifts, changes of ERS mode, pit entry, and the safety car in stage 2 — are
  discrete transitions at step boundaries. They are either scheduled or triggered by a condition.
  Where the crossing time is needed, one linear back-interpolation recovers it. No root-finding
  runs in the hot loop.
- **Determinism is enforced in CI.** The step is fixed. The RNG is counter-based, either Philox or
  ChaCha8, keyed by (seed, rollout_id, stream, step). There is no fast-math, and every reduction
  runs in a fixed order.

  Bit-exactness is guaranteed on the same target. Exactness within a tolerance across platforms is
  documented. Every artifact embeds the seed, the git hash, the dt, and the feature flags.

### 11.3 Batch and GPU

Honest sizing first. A strategy rollout at dt of 0.1 s to 0.25 s, over a 2 h race, is about 30k to
70k steps of a model with 30 to 60 states. That is about 0.1 s for each rollout on each core.
**10k rollouts therefore take about 5 s to 10 s, on a 16-core desktop, with rayon.** Ship that.

Design NOW, so that a GPU tier drops in later. Five rules:

- Use struct-of-arrays state, with an explicit batch dimension. A single run has batch = 1. The
  public API takes and returns batch views, zero-copy to NumPy through rust-numpy.
- Allocate nothing on a step. Use a preallocated `SimArena`. A CI alloc-counter test asserts 0
  allocations for each step.
- Keep block evaluation functions pure, and generic over `f32` and `f64`, through a `Real` trait.
  The same code then monomorphizes into rayon loops today, and into CubeCL kernels tomorrow.
- Represent a discrete mode as a small integer, and keep the logic friendly to masking and
  selection.
- The gate for a GPU decision: only when a use case demands 10⁵ rollouts or more. Then use CubeCL,
  which keeps kernels in Rust and targets CUDA, Vulkan, Metal, and WebGPU. A rewrite in JAX with
  vmap is rejected: the events are branchy, it locks the core to Python, and it kills WASM.

### 11.4 WASM, a first-class target and the seed of the Web UI

Per Locked Decision #8, the **Web UI is the endgame**. `outlap-wasm` is not a throwaway
demonstration. It is the seed of the eventual primary interface, which is Stage 3, §16.

The scope in v1 stays modest: the QSS lap solver, plus one transient rollout that runs faster than
real time, single-threaded, in a browser; and a lap-time widget with live sliders for wing,
compound, fuel, and drivetrain variant, over a bundled track.

The discipline, however, is permanent:

- `wasm32-unknown-unknown` builds in CI from M1 onward. A PR that breaks the wasm build fails.
- `outlap-core`, `outlap-tire`, and the solvers assume no filesystem, no threading, and no clock.
  IO sits behind traits. A heavyweight dependency is feature-gated out of the wasm profile.
- The gate is a bundle under 2 MB, gzipped.
- The path to stage 3: a local-first browser app, where files stay on the user's machine; batch
  execution on WebGPU, through CubeCL and wgpu, when the Monte Carlo of stage 2 needs it in the
  browser; and optional hosted compute later. The network clause of AGPL (§15) protects exactly
  that surface.

### 11.5 Benchmarks and performance gates

The metrics are: transient steps per second on each core, targeting at least 500k, which is 500
times real time at 1 kHz; a QSS lap in 50 ms or less, at Spa length; 10k rollouts in 10 s or less,
on 16 cores; 0 allocations for each step; and Python dispatch of 1 ms or less for each batch call.

In CI, iai-callgrind gates the instruction count on the core kernels. A regression above 3 % fails,
and that is the merge gate. criterion measures wall time as a nightly trend job, on self-hosted
hardware. The WASM build and a smoke test of the demonstration run in the same workflow.

### 11.6 Code architecture, style, and workflow (Locked Decisions #26–36)

- **Composition (#26)** is runtime and data-driven. One binary loads any `vehicle.yaml`. Blocks are
  assembled and topologically sorted at load time. Dispatch inside the loop is through an enum.
  Never add a compile path for a specific vehicle architecture. "Car = pure data" and the WASM
  story both require this.
- **Errors (#27)** are typed with thiserror, on every fallible public API: `SchemaError`,
  `AssemblyError`, `SolverDiverged{...}`, and others. A solver kernel is panic-free and returns
  `Result`. `debug_assert!` guards physics invariants in development builds. `anyhow` appears only
  inside `bin` and CLI edges. The reason: a panic poisons PyO3, and it aborts WASM.
- **Lints (#28).** `clippy::pedantic` at workspace level, with a curated allow-list that carries
  comments. `#![deny(missing_docs)]` on every public item. `#![forbid(unsafe_code)]` in every
  crate, except the crate that holds the C-ABI tire plugin, which isolates all `unsafe`. rustfmt
  defaults are untouched.
- **Naming (#33)** is hybrid. Public APIs take descriptive names, such as `slip_ratio` and
  `vertical_load_n`. A math kernel uses the symbols of the paper, such as `kappa`, `f_z`, and
  `sigma_y`, with a doc-comment header that maps each symbol to the cited equation numbers, for
  example "Pacejka 2012 eq. 4.E19–4.E30". A kernel must be diff-able against the literature that it
  implements.
- **EOM verification (#32).** SymPy notebooks under `docs/derivations/` derive the 7-DOF and 14-DOF
  chassis equations symbolically, by Kane or Lagrange, through `sympy.physics.mechanics`. CI
  lambdifies the symbolic RHS, and asserts that it agrees with the hand-written Rust RHS, at
  randomized states and parameters, to 1e-12. This catches the classic sign errors, and it doubles
  as documentation that earns community trust.
- **Python (#34)** is managed by uv. ruff does lint and format. pyright runs strict. The public API
  carries full type hints. Configuration objects are pydantic v2 models, validated against the JSON
  Schemas that are **generated from the Rust schemars types**. Rust is therefore the single source
  of truth for the formats, and the Python mirrors cannot drift. CI checks that the generated
  schemas equal the committed schemas.
- **Overrides and variants (#35).** A programmatic sweep takes a dict of dotted paths, such as
  `over={"aero.cl_scale": [...], "drivetrain.control.split.front": [...]}`. A named variant is a
  YAML overlay file, deep-merged onto the base vehicle, and validated against the schema *after*
  the merge. The two compose. Every result records a hash of the resolved parameter set.
- **Git and release (#36).** Trunk-based, with short-lived PR branches. CI gates are enforced even
  when working alone, which keeps the history reviewable and ready for contributors. Commits follow
  the Conventional Commits format: `feat:`, `fix:`, `docs:`, `perf:`, and so on. Each milestone gets
  a tag, a GitHub release, and a changelog generated by git-cliff.

---

## 12. V1 Milestones

The calendar estimates assume the 10 to 20 hours each week that the author stated, in Locked
Decision #12. The full-3D decision, #13, adds about 4 to 6 weeks across M1, M3, and M4. That puts
**v1 at roughly 7 to 11 months**.

Every milestone ends in something that runs and that can be demonstrated, in the public repository.

| M | Deliverable | ~Effort | Ships |
|---|---|---|---|
| M1 | `outlap-schema` (incl. drivetrain topology graph §8.0) + `outlap-track` (**3D ribbon**: κ(s), grade, banking, vertical curvature) + **OSM+DEM track importer** (§9.3) + min-curvature line generator (§6.3) + point-mass T0 with 3D normal-load corrections → first lap time on Catalunya. WASM build in CI from here | 4–6 wk | 0.1 |
| M2 | MF6.1 + `.tir` parser/writer + Python fitting pipeline + 3 citation-backed reference `.tyr` files | 4–5 wk | |
| M3 | Full QSS tier (T1 double-track trim → **g-g-g-v envelopes** on the 3D ribbon, ride-height/yaw aero maps, topology-graph powertrain with map-based ICE/EM + gearbox + static splits/diffs + **machine thermal-budget derating §8.5**) — cross-checked vs fastest-lap Limebeer F1 numbers (flat-track mode for the oracle comparison). **PDT importers (§10) land here** | 6–8 wk | 0.2 |
| M4 | Transient tier (T2 in the **curvilinear 3D road frame**, split integrator, ideal driver model, shift events, rule-based TV controller) + QSS↔transient parity gate in CI + **time-weighted raceline QP + the deferred ≤1% Limebeer lap-time gate (Decision #48)** | 5–7 wk | 0.2.5 — shipped 2026-07-13. Parity re-scoped in-flight (Decision #48 pattern): hull containment asserted (0.0% on 3 cars); lap/apex parity + the ≤1% Limebeer gate + the 250k steps/s floor recorded-and-decomposed, not gated (driver corner margin ~+14–17%; RHS-bound ~62k steps/s at MF6.1 fidelity) — see `docs/validation/limebeer.md`. Driver gained a corner-scaled margin, sideslip damper + wheel-slip governor beyond plan; `spa_osm` shipped; Spa fast gate still deferred |
| M5 | **Tire thermal ring + wear in both tiers — the headline. Stint-simulation demo** | 4–6 wk | 0.3.0 — shipped 2026-07-16. Reduced Farroni-TRT 3-node ring + Archard wear/cliff + thermal damage, marched as slow states in T0/T1 and T2; envelope gained real `T_tire`/`wear` axes (Decision #49). `outlap.wearcal` inverse-calibrates from stint pace (FastF1 opt-in, §15); reference `.tyr` recalibrated + soft/med/hard compound presets; `10_stint_strategy` capstone. Validation (Decision #48): thermal warm-up/steady band asserted; wear/cliff reproduced after calibration + QSS↔T2 decay 0.041 ≤ 0.1 s/lap asserted, decomposed in `docs/validation/{tire-thermal,wear-cliff}.md` |
| M6 | ERS 2026-style (deploy taper, override mode, recharge phases) + battery ECM + fuel mass + T3 14-DOF | 4–5 wk | 0.4.0 — shipped 2026-07-27. The energy manager as a generic regulatory `policy:` overlay (deploy/override tapers on the exact piecewise-linear evaluator — the one recorded Decision #30 exception, per-lap harvest allowance, C5.12 recharge phases), the Thévenin battery ECM with a 2nd RC pair, fuel as mass (drains, migrates the CG, flow ceiling shrinking the ICE envelope), and **T3 14-DOF** (unsprung masses, dampers, ARBs, bumpstops; ride height/pitch/heave as states, verified against the SymPy derivation to 1e-12). Mid-milestone the ERS/drivetrain schema was restructured (D-M6-13: `vehicle/2.0`→`2.1`, MGU-K promoted to a first-class drivetrain unit on a shared crank, id-keyed `batteries:`) and `.ptm` `kind` became the pure energy source (`ptm/2.0`), which fixed two petrol engines that had silently inherited a symmetric regen envelope. `gt_hybrid` promoted to a runnable reference car. Validation (Decision #48): a real ~5% 14-DOF understeer vs the neutral CommonRoad BMW 320i recorded (gate #4); the f1 numbers re-verified against FIA Section C Issue 19 |
| MT | **Track fidelity overhaul (standalone) — the tracks are now the dominant sim-vs-real error source once the car is calibrated.** Real track widths + racing surface (OSM boundary tags / trackmap, not defaulted widths); a curvature-clean centerline pipeline that preserves true apex radii (audited against telemetry-derived corner radii); elevation + banking properly fused (C² z; banking from DEM cross-sections or per-corner keypoints); a **telemetry-derived importer** (FastF1 X/Y position → outlap track, robust circle-fit/spline — prototyped in the M6 calibration); and a **track-quality validation gate**: real lap telemetry vs the calibrated car on the SAME geometry (grip-matched) → corner-radius + apex-speed agreement within tolerance. Re-import/validate the reference set (Catalunya/Spa/Silverstone) so M7 compares on trustworthy geometry. Prereq for the M7 hero demo. | 3–5 wk | |
| M7 | `outlap-batch` (rayon, SoA) + sweep API + working CLI (§11.1b) + **all four reference vehicles** (Locked Decision #1) + the **hero demo as redefined by the author (Decision #22)**: F1 2026-config vs GT hybrid vs EV sports 2-DU AWD vs EV sports 1-DU RWD — each on **its own min-curvature line**, compared lap times + energy on Catalunya/Spa/Silverstone (4-DU TV + FWD ship as extra example configs) + docs site + WASM demo widget | 6–8 wk | **1.0** |

**MT: why track fidelity is now a standalone milestone.**

Through M1 to M5, the *car* models were validated against oracles: the chassis, the tire thermal
state and wear, and the powertrain. M6 then calibrated the grip of `f1_2026` to real 2026 telemetry
from Barcelona, through FastF1.

With the car correct, the **track** is the largest remaining error. The calibrated f1 car does
**83 s on the real, reverse-engineered Barcelona geometry**, against a real fastest lap of 80.1 s.
It does **94 s on the shipped `catalunya_osm`**. That gap of about 11 s is *pure geometry*. It is
not the vehicle.

There are three root causes. An OSM import defaults its widths, and leaves banking unresolved; the
`meta` of the `track.yaml` says so. No track was ever ground-truthed against real lap data. And
reverse-engineering geometry from telemetry position at about 10 Hz carries curvature noise: the
tightest apex comes out at about 31 m, against a circle-fit of about 34 m, which caps v_min about
10 % low.

The seed exists. `data/tracks/barcelona_real_2026`, derived from FastF1, plus the calibration
scripts from M6. What it needs is a real pipeline and a validation gate.

A diagnostic rule of thumb, from the M6 overlay. When the simulation is faster than reality by a
**uniform** offset on every straight, the cause is *vehicle state*: race fuel mass, since the
simulation runs 768 kg dry, plus race modes for the engine and ERS, plus lift-and-coast. When a
corner speed mismatches **locally**, the cause is *track geometry*.

Only the second is MT's problem. The first belongs to a future model of race trim and fuel load.

**The roadmap after 1.0, in order:**

1. **v1.x: importers for sim-racing telemetry** — MoTeC `.ld`, ACC, and iRacing (Locked Decision
   #10). This is the push for community growth, AND it is the author's own source of validation
   data, because there is no access to proprietary data today (Locked Decision #9).
2. **Stage 2: Monte Carlo for race strategy** (§16). A time-discrete race simulator, a stochastic
   layer, and a strategy optimizer, all on the physics of T0 with slow states.
3. **Stage 3: outlap-web.** The browser app grows from the WASM widget into the primary interface.
   It is local-first, with WebGPU batch execution, and optional hosted compute under AGPL.
4. **The backlog of integrations (Locked Decision #24), in this order:** a Gymnasium environment
   for race strategy, arriving with stage 2, which will likely become the RL reference for the
   community; then FMU and FMI export of the vehicle blocks, which opens the professional world of
   Simulink and Modelica.

   A ROS 2 bridge was considered, and **the author withdrew it**. It is out of scope. It is also
   adjacent to robotics, which sits badly with the firewall of §1.
5. **The community surface, throughout.** A separate **CC-BY-SA-4.0 data registry** repository,
   holding tracks, vehicles, and tire fits that are validated against the schema, with a CI smoke
   lap on every PR. Plugin traits — `TireModel`, `DriverModel`, `AeroModel` — plus Python entry
   points. And, in stage 2, `StrategyPolicy` and `SafetyCarModel` plugins.

   A quick-start "10-parameter car" mode is deliberately **not** in v1 (Locked Decision #3).
   Revisit it if the community asks.

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

**The parity gates between QSS and transient.** They run in CI, on every reference car, with frozen
tire state, on a smooth track.

1. Lap time: |T2 − T0| ≤ 0.3%.
2. Apex speed at each corner: ≤ 1%.
3. Transient samples of (ax, ay, v) sit inside the T1 gg-g-v hull, with an exceedance area of ≤ 2%.
4. Fuel and ERS energy for each lap: ≤ 1%.

With live tire states, one more gate applies: the decay in T0 stint lap time, against a long T2
run, must agree to ≤ 0.1 s per lap.

These gates are exactly what the Monte Carlo of stage 2 needs, in order to trust T0.

---

## 14. Testing & CI

- **Golden files.** Committed Parquet outputs, for each reference vehicle × track × tier, with a
  tolerance for each channel. Regenerate them only through an explicit `--bless`.
- **Property tests**, with proptest. They cover: symmetry and sign conventions of tire force;
  containment in the friction circle; closure of energy accounting, comparing ES in and out plus
  fuel LHV against work plus losses; wear monotone in sliding energy; and a schema round trip of
  load, save, load.
- **Fuzzing** on every file loader: YAML, CSV, `.tir`, and the HDF5 importer.
- **Determinism tests.** The same seed twice, and across thread counts, must give bit-identical
  results.
- **A convergence test.** The production split-stepper against diffsol, as the reference, at
  O(dt²).
- **GitHub Actions.** Wheel builds on Linux, macOS, and Windows, through maturin. A WASM build. A
  docs build. And the iai-callgrind performance gate.

---

## 15. License & Clean-Room Policy

**The project license is AGPL-3.0** (Locked Decision #7). In the author's words: *"forces the
disclosure of the source if our code will be used … the strongest … commercialization OK but always
with open source code"*.

AGPL rather than plain GPL-3.0, because the network clause of §13 covers SaaS and web deployment.
Without it, anyone could serve a modified outlap as a closed web product. That is exactly the
Stage-3 surface (§16).

**What AGPL means here, stated plainly:**

- Anyone may use outlap, modify it, sell it, or host it. But a version that is distributed, or
  served over a network, must publish its complete corresponding source, under AGPL.
- Private and internal use carries no obligation to disclose. A race team may run it in-house
  without offering it to others. Copyleft triggers on distribution and on network service. It does
  not trigger on use.
- The trade-off is accepted knowingly. Some corporations, and some contributors who prefer
  permissive licenses, will pass. That is the price of the guarantee the author wants.

**The licensing structure:**

- Code is AGPL-3.0-only. Each file carries an SPDX header.
- `schemas/`, which holds the published JSON Schemas, is **Apache-2.0**. The file formats should
  spread to other tools, even where the code cannot.
- The data registry — reference vehicles, tracks, and tires — is **CC-BY-SA-4.0**, which is
  share-alike and matches the spirit.
- **Contributions use DCO now. Decide on a CLA before the first significant external
  contribution.**

  As the sole author, the author can later dual-license, and for example sell commercial
  exceptions. But only while holding all copyright. Accepting external contributions under DCO
  alone permanently forfeits the ability to relicense unilaterally. If commercial dual-licensing is
  a live option, adopt a CLA from day 1.

**Dependency compatibility. Licenses flow one way, INTO AGPL:**

- MIT, Apache-2.0, BSD, and Zlib are freely usable. Every dependency in §4.1 stays valid.
- LGPL-3.0, which covers the TUM ecosystem, is now usable as a dependency and as an oracle (§4.3).
- GPL-3.0 and later are compatible with AGPL-3.0, through GPLv3 §13. Use them if genuinely needed.
- Incompatible: GPL-2.0-only, and proprietary SDKs such as the closed DLL from AiM. Wrap them
  externally, or avoid them.

**Rules on authorship and provenance, which the change of license did not alter:**

- The flagship models — the tire thermal ring, the wear and cliff model, and the ERS energy manager
  — are **implemented from the published literature**: Farroni, Pacejka, Archard, and the FIA
  regulations. They are never derived from another codebase.

  Tire code in a GPL game engine, such as Speed Dreams or VDrift, is off-limits as a *source of
  derivation*, whatever the license compatibility. The theory pages in the documentation, with
  their equations and citations, are the record of provenance.
- FSAE TTC data: parsers and fitting, yes. Redistribution of the data, or of a fitted parameter
  set, NO.
- FastF1 and F1 data: use them as artifacts for calibration and validation only. Do not
  redistribute raw telemetry.
- PDT `.h5` files are private to the author. The importer reads the documented schema (§10). The
  tests use synthetic fixtures. No real file is committed.

---

## 16. Stage 2 Preview — Race Strategy Monte Carlo

Design for this now, through hooks in v1. Build it after 1.0.

- A **time-discrete** race simulator, not a lap-discrete one. This is the acknowledged right
  architecture. The TUM author's own Rust proof-of-concept validates the choice. It was never built
  out, and that repository is dual Apache and MIT, so it may serve as a design seed.
- **The coupling to physics.** T0 with slow states gives lap time as a function of tire age,
  temperature, and wear, plus fuel, plus the state of ERS and the battery, plus traffic. That
  replaces the empirical delta in lap time that every prior project used.
- **The stochastic layer.** Hazard models for the safety car, the VSC, and a red flag.
  Distributions for pit-stop time. A model for overtaking and traffic, taking the Heilmeier 2020
  formulation as the baseline. And reliability. The counter-based RNG is already in the core
  (§11.2).
- **The optimizer.** Tree search over strategies, or policy optimization, over pit laps, compounds,
  plans for ERS deployment — which is `u(s)` from §8.3 — and use of the override mode.
- **Fuel to finish, and fuel saving. These are strategy, not physics.**

  The fuel model from M6 PR5 already burns fuel from real ICE work, and feeds mass and CG back into
  the lap (§8.1). It is a model of *consumption*. It is not a prescribed mass(t).

  The rule at race level sits on top. F1 requires at least 1.0 L of fuel to remain at the flag, for
  the FIA sample. Teams therefore start with just enough to finish as close to that reserve as
  possible.

  Two pieces belong here, and NOT in the fuel slow state or the vehicle schema.

  (1) **Optimizing the load to finish.** Pick the initial fuel load so that the terminal fuel is
  about the reserve at the flag. This is an outer fixed point: load gives lap times, which give
  total consumption, which gives the load you needed. Solve it by bisecting `initial_kg` against a
  simulated race. The reserve is `1.0 L × fuel_density`; F1 fuel is about 0.72 to 0.78 kg/L, so
  about 0.75 kg.

  The initial load for a *race* is a strategy input. It is not car identity. Keep it off the vehicle
  document, as an override or a value that the strategy computes. That preserves the firewall
  around the input quartet.

  (2) **Fuel saving, which means lift-and-coast.** When the car is projected to run short, the
  driver lifts early. That is exactly the `lift_point` component of the `u(s)` control vector
  (§8.3), wired at M6 PR5A. The strategy layer therefore *acts* on a fuel target by shaping the lift
  schedule, coupled to the ERS harvest that it also drives.

  There is an honest, cheap precursor that could land earlier, as reporting on the physics side: an
  optional fuel `reserve_kg`, or `1.0 L × density`, as a floor. A full-race result would then report
  the margin at the end of the race, and *flag that the car ran dry*, rather than silently clamping
  fuel at 0. That follows the discipline of surfacing estimates and degradations, §6.1. But the load
  optimizer itself stays here, in stage 2.
- **Rain and wet weather live here** (Locked Decision #4). That covers wet tire parameter sets,
  scaling of track grip, which `grip_scale` already carries in the track format, estimating the
  crossover lap, and the evolution of a drying line. It is a first-class axis of strategy. It is not
  a physics feature of v1.
- **A contribution of open data.** A maintained dataset of safety car, VSC, and accident phases,
  after 2019, built on FastF1 and jolpica. The existing annotated database stops at 2019, which is
  standalone whitespace.
- A Gymnasium-compatible environment for strategy would likely become the RL reference for the
  community.

**Stage 3: outlap-web, the declared endgame (Locked Decision #8).** The WASM widget grows into the
primary interface: a local-first browser app, where vehicle and track files never leave the user's
machine; interactive studies of laps, stints, and strategy; batch Monte Carlo on WebGPU, through
CubeCL and wgpu; and, optionally, hosted compute. The network clause of AGPL (§15) guarantees that
any hosted derivative stays open.

---

## 17. Reading List

**Books and core references**

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

**Codebases to study, in this order**

1. Open-Car-Dynamics (Apache-2.0) — composable submodel architecture, MF52 usage.
2. fastest-lap (MIT) — OCP lap-sim structure, F1 3-DOF model, g-g computation.
3. Chrono::Vehicle JSON vehicle templates — parametric data design.
4. NREL thevenin (BSD-3) — battery ECM.
5. FASTSim v3 / polars — Rust-core + PyO3 packaging patterns.

---

## 18. First-Week Task List

1. **Day 1.** Set up the environment (§3). Create the **public** GitHub repository `outlap`, with
   `LICENSE` set to AGPL-3.0-only, the DCO in `CONTRIBUTING.md`, the CLA decision noted as open
   (§15), and `schemas/LICENSE` set to Apache-2.0. Run `cargo new` for the workspace skeleton
   (§11.1). Add a CI skeleton, running fmt, clippy, test, and the wasm32 build on push. Reserve the
   `outlap` names on crates.io and PyPI, with 0.0.1 placeholders.
2. **Day 2–3.** Write `outlap-schema`: the serde types for vehicle, including the drivetrain
   topology graph of §8.0, plus track, ptm, and tyr. Add schemars JSON-Schema emission and
   round-trip tests. This is the contract. Review it hard before writing any physics.
3. **Day 3–4.** Write `outlap-track`: centerline CSV, then an arc-length spline, then κ(s),
   elevation, and banking. Add the OSM importer, in Python, for one real circuit. Plot the result
   as a sanity check.
4. **Day 5.** Write T0 as a point mass, with constant μ and a simple power cap. Get the first lap
   time on the real track. Compare the magnitude against published lap records. That is a sanity
   check, not parity.
5. **Day 6–7.** Write `python/outlap/importers/pdt_h5.py`, against §10. The three reference files
   are on the author's machines; CI uses synthetic fixtures. Do EDrive to `.ptm` first, then
   DriveUnit, then BatteryPack. Validate the round trip, per §13.
6. Then follow the milestone order of §12. At every milestone, update the theory page in the
   documentation with the equations and the citations, *as they are implemented*. That is the
   record of clean-room provenance, §15.

---

## Appendix A — repo CLAUDE.md

Copy this verbatim to `outlap/CLAUDE.md`. It is the working agreement for the AI assistant in the
new repository. It is quoted here as-is, so do not edit it in this document; edit `CLAUDE.md`
itself.

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

Copy this to `.github/workflows/ci.yml`. Trim it as needed while bootstrapping.

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

The performance gates join this matrix once the kernels exist, per §11.5. They are instruction
counts from iai-callgrind, on the kernels for tire evaluation, the step, and the gg solve.

## Appendix C — CONTRIBUTING.md

```markdown
# Contributing to outlap

Thank you for helping. The rules below are strict, because they protect two things: the legal
standing of the project, and the credibility of its physics.

## Sign-off (DCO)

Every commit must carry a `Signed-off-by:` line. Use `git commit -s` to add it. That line certifies
the Developer Certificate of Origin (developercertificate.org). It states that you wrote the change,
or that you have the right to submit it under AGPL-3.0.

## Licensing

- Code is AGPL-3.0-only. Put an SPDX header in every file. `schemas/` is Apache-2.0. Data is
  CC-BY-SA-4.0.
- A dependency must be MIT, Apache, BSD, Zlib, or LGPL. For anything else, open an issue first.

## Clean-room policy, which you must not break

- Implement each physics model from published literature. A PR that adds or changes a model MUST
  also update the matching theory page in the documentation, with the equations and the citations.
- **Never copy code from another simulator, game engine, or proprietary tool. Never closely
  paraphrase it either.**

  You MAY *consult* an open-source project whose license permits it, to understand the approach or
  to avoid a pitfall — that is, to see how they solved a problem. Three conditions apply. Record
  the repository, with its name and license, next to the citations. Re-author the code
  independently, taking ideas and not expression. And read a strong-copyleft source, under GPL or
  AGPL, for approach only.

  Never lift code from a GPL game engine.
- Commit no proprietary data. You may not commit FSAE TTC data, and you may not commit a parameter
  set fitted from it. You may not commit raw F1 telemetry. Commit only data that is synthetic, or
  data that you can cite.

## PR checklist

- [ ] `cargo fmt`, `clippy -D warnings`, and `cargo test` are green, and the wasm target builds
- [ ] No step path allocates. The alloc-counter test is green
- [ ] The golden files are unchanged. If you regenerated them with `--bless`, the PR justifies the
      change on physics grounds
- [ ] New physics comes with a property test and a citation on the theory page
- [ ] A schema change comes with a version bump, a migration, and a round-trip test
```

---

## Appendix D — Ubuntu bootstrap, exact commands (fresh machine, SSH, nothing installed)

This assumes Ubuntu 24.04, an SSH session from the Windows machine, no files transferred yet, and no
Claude Code. Run the steps from top to bottom.

### D.0 Transfer this document. Run this on the WINDOWS machine, in PowerShell

```powershell
scp "C:\Users\neomo\Documents\RACESIM_HANDOFF.md" <user>@<ubuntu-ip>:~/
```

### D.1 System packages (Ubuntu)

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential git curl wget pkg-config libssl-dev cmake \
                    tmux ripgrep gh mesa-vulkan-drivers vulkan-tools
```

`tmux` matters. Run Claude Code inside tmux, so that a dropped SSH connection never kills a session.
Start one with `tmux new -s outlap`, and reattach with `tmux attach -t outlap`.

### D.2 Rust toolchain

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
rustup component add clippy rustfmt
rustup target add wasm32-unknown-unknown
```

Install `wasm-pack`, `git-cliff`, `cargo-criterion`, and `iai-callgrind-runner` later, when you
first need them. They compile for a while on the i5-6500.

### D.3 Python toolchain

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
uv python install 3.12
```

### D.4 Identity and GitHub authentication

```bash
git config --global user.name  "Konstantinos Moulakis"
git config --global user.email "neomoula@gmail.com"
git config --global init.defaultBranch main
git config --global commit.gpgsign false
gh auth login
# choose: GitHub.com → HTTPS → Yes (authenticate Git) → Login with a web browser
# it prints a one-time code + URL — open the URL in the WINDOWS browser, enter the code
```

### D.4a A status check. Skip whatever is already satisfied

Some tools, git for example, are already on the machine. Run this block first. Then skip any command
in steps D.1 to D.5 whose check already passes:

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

How to read the results. A check that passes means **skip that command, not the whole step**. For
example, git may be installed while `user.email` is unset; then still run the two `git config`
identity lines.

`gh auth status` failing with "not logged in" is the only trigger for `gh auth login`.

If `~/dev/outlap` already exists, look inside it before D.6. Never overwrite it blindly.

> **Machine snapshot, verified 2026-07-03, on host `kmoulakis-linux`, user `kmoulakis`.**
>
> Already present: git 2.43.0, with user.name `KMoula30`, user.email set, and
> `credential.helper=store`; gh 2.45.0, which is **not authenticated**; rustc and cargo 1.96.1; uv
> 0.11.26; Claude Code 2.1.19; tmux 3.4; ripgrep 14.1.0; and Vulkan 1.3.275.
>
> Still needed: `init.defaultBranch main`, `gh auth login`, and the transfer of this handoff.
>
> Unverified: build-essential, cmake, pkg-config, libssl-dev; whether Rust is managed by rustup or
> by apt; the clippy and rustfmt components; the wasm32 target; and the Claude login. Skip D.1 to
> D.5, except for those.

### D.5 Claude Code (terminal)

```bash
curl -fsSL https://claude.ai/install.sh | bash
export PATH="$HOME/.local/bin:$PATH"     # installer adds this to your shell profile too
claude --version
```

To log in the first time, run `claude` once, anywhere, and use `/login`. It prints a URL and a code
for the browser. That is the same device flow as gh.

### D.6 The repository skeleton

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

### D.7 Create the public GitHub repository, and make the first commit

```bash
cd ~/dev/outlap
git add -A
git commit -s -m "chore: bootstrap workspace, licenses, handoff doc"
gh repo create outlap --public \
  --description "outlap — open vehicle racing simulator & strategy optimizer (AGPL-3.0)" \
  --source=. --remote=origin --push
```

Then do this once, from any browser. Create the accounts on crates.io and PyPI, and reserve the
`outlap` name with 0.0.1 placeholder releases when convenient (§2).

### D.8 The first Claude Code session

```bash
cd ~/dev/outlap
tmux new -s outlap
claude
```

Paste this opening prompt as-is:

> Read docs/HANDOFF.md in full before doing anything — it is the single source of truth for this
> project, and its Locked Decisions log (§1) overrides any other instinct. Then: (1) extract
> Appendix A into ./CLAUDE.md, Appendix B into .github/workflows/ci.yml, Appendix C into
> ./CONTRIBUTING.md, verbatim with any obvious path fixes; (2) commit that as `docs: add working
> agreement, CI, contributing`; (3) start §18 Day 2–3 — design `outlap-schema` (the serde types +
> schemars JSON-Schema emission for the vehicle/track/conditions/sim quartet, §6.2b + §9) and show
> me the vehicle schema types for review before implementing the rest.

Five working habits get the most out of Claude Code here.

- Do one milestone task in each session. Run `/clear` between unrelated tasks. Use
  `claude --continue` to resume.
- Use plan mode, Shift+Tab, for anything architectural. Let it read the relevant sections of
  HANDOFF.md first.
- Keep CLAUDE.md lean: it is the working agreement, Appendix A. The deep specification stays in
  docs/HANDOFF.md, and Claude reads the section it needs, on demand.
- Review the diff before you accept a write to schema or format code. The contracts are the
  product.
- Once CI exists, have Claude open PRs with `gh pr create`, instead of pushing to main. See #36.

---

*End of handoff. This document supersedes any prior conversation context. Everything the project
needs in order to start is above.*
