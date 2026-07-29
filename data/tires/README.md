<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Reference tires

This directory holds the `.tyr` datasets that ship with outlap as reference data and as validation
data. They drive the MF6.1 force model (§7.1). Each coefficient value is a fact. Each value is
transcribed from a published source, and each dataset cites that source exactly in its
`provenance` block and in its `README.md`. outlap does not redistribute the source documents. Books
and papers stay outside the repository. See the root `.gitignore`.

These datasets are data, and their license is CC-BY-SA-4.0. They are not crate test fixtures. The
working agreement keeps the test fixtures synthetic and minimal in
`crates/outlap-schema/tests/fixtures/tyr/`.

## Datasets

| Directory | Tire | Source | Notes |
|---|---|---|---|
| [`pacejka_2006_205_60r15/`](pacejka_2006_205_60r15/) | 205/60R15 91V passenger car | Pacejka, *Tyre and Vehicle Dynamics*, 2nd ed. (2006), Table A3.1 | The worked-example car tire from the book. It is also the MF6.1 validation tire. This is the 2nd-edition set: it has no inflation-pressure (`PP*`) terms, `Mx ≡ 0`, and `My` comes from `qsy1`. |
| [`roborace_devbot_mf52/`](roborace_devbot_mf52/) | Roborace DevBot "sport focused road tire" | TUMFTM Open-Car-Dynamics (Apache-2.0), pinned commit `0a92c686` | An MF5.2 set mapped to MF6.1. Its README gives the map for each coefficient. The camber term `PHY3` folds into `PKY6`. There is no pressure model, so `dpi ≡ 0`. `Mz ≡ Mx ≡ 0`. |
| [`limebeer_2014_f1/`](limebeer_2014_f1/) | Perantoni & Limebeer 2014 reference F1 | Perantoni & Limebeer, *Optimal control for a Formula One car with variable parameters*, VSD 52(5), 2014 (Appendix A + Table 3) | An MF6.1 re-expression of the load-linear peak-μ similarity model in the paper. Its README records the provenance of each coefficient. The function μ(Fz) maps exactly onto `PDX*` and `PDY*`. The `PK*` terms are fitted numerically to the peak-slip locations in the paper. `Mz ≡ Mx ≡ 0`. This is reference car #1 for the validation cross-check (`docs/validation/`). |

The tire model in the Limebeer paper is a reduced similarity form that interpolates on load. It is
not an MF coefficient set. Therefore `limebeer_2014_f1/` is a re-expression in MF6.1 form, not a
verbatim transcription. Its README tabulates how each coefficient was derived. The work is
clean-room: no third-party source code was read.

## Blocks that no source publishes

A `Tyr` file must have a `thermal` block and a `wear` block. The published sources give only
force coefficients and moment coefficients. Therefore outlap supplies these two blocks itself.

The thermal ring (M5 PR1) is a lumped-node set that is physically plausible for the tire class.
`outlap.wearcal` calibrates the wear and cliff block by inversion (M5 PR7 and PR8). It fits a
representative stint-decay curve, which gives gradual wear and then a cliff. The earlier block
saturated instead. Each file records this in `provenance.source` and in a `# CALIBRATED` comment
on the block.

`synthetic: false` on a dataset means one thing only: the force and moment coefficients that carry
the physics are the published measured set. The thermal and wear blocks are the documented
exception, and outlap models them.

The soft, medium, and hard **compound presets** are in `f1_2026_compounds/`. They build on the
racing-slick core. Read the README in that directory. Each preset changes the calibrated baseline
in three ways: peak grip, temperature window, and wear rate. They exist for the multi-compound
strategy demonstration.

## Validation

CI checks every `data/**/*.tyr.yaml` against the published JSON Schemas with
`python -m outlap.schemas --check`. The Rust integration tests in
`crates/outlap-tire/tests/reference.rs` read every dataset in this directory. Each dataset must
load with no warnings. Each dataset must round-trip through the `.tir` codec with exact numbers.
Each tire also gets physics checks: a grip band that is plausible for its class, correct sign
conventions, and finite values.

The tighter cross-check against golden CSVs holds each value to `≤ 0.5%`. The teasit Magic-Formula
library generated those CSVs, and outlap uses the outputs only, never the code.
[`tools/goldens/`](../../tools/goldens/) describes this check.
