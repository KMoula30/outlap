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
   in the docs theory page, in the same PR. Never derive model code from other simulators'
   source (especially GPL game engines: Speed Dreams, VDrift).
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
