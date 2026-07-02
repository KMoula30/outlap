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
- Never port code from GPL simulators/game engines or proprietary tools, even "just to compare".
- No proprietary data: FSAE TTC data and fitted TTC parameter sets cannot be committed; raw F1
  telemetry cannot be committed. Synthetic/citable data only.

## PR checklist
- [ ] `cargo fmt` / `clippy -D warnings` / `cargo test` green, wasm target builds
- [ ] No new allocations in step paths (alloc-counter test green)
- [ ] Golden files unchanged, or regenerated with `--bless` + a physics justification in the PR
- [ ] New physics → property test + theory-page citation
- [ ] Schema changes → version bump + migration + round-trip test
