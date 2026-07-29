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
- Never port code from a GPL simulator, from a game engine, or from a proprietary tool. Do not do
  it to compare results either.
- Commit no proprietary data. You may not commit FSAE TTC data, and you may not commit a parameter
  set fitted from it. You may not commit raw F1 telemetry. Commit only data that is synthetic, or
  data that you can cite.

## Use of AI

- You may use AI. Claude Code with Opus 4.8 or Fable 5 wrote most of this project.
- State which AI coding tools you used.
- Send test results with any change to the physics engine or to the assumptions of a model. Cite
  the scientific publications that support the change.

## PR checklist

- [ ] `cargo fmt`, `clippy -D warnings`, and `cargo test` are green, and the wasm target builds
- [ ] No step path allocates. The alloc-counter test is green
- [ ] The golden files are unchanged. If you regenerated them with `--bless`, the PR justifies the
      change on physics grounds
- [ ] New physics comes with a property test and a citation on the theory page
- [ ] A schema change comes with a version bump, a migration, and a round-trip test
