<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# How to generate the MF6.1 goldens

The golden CSVs are the numerical oracle for the MF6.1 force model. The Rust kernels
(`crates/outlap-tire`) and the Python forward model must both reproduce them, within the tolerance
given below. This is the `Fx/Fy/Mz ≤ 0.5%` gate of HANDOFF §12 and §13.

## Clean-room policy: read this before you change anything here

An external Magic-Formula implementation generates the goldens. outlap uses **its outputs as data
and nothing else**. Do not read that implementation. Do not port it. Do not vendor it. The MF6.1
model in outlap comes from the Pacejka book alone (§15).

The generator, `gen_mf61_goldens.m`, is our own work and is AGPL. It calls `addpath` on an oracle
checkout that **the user supplies**. That checkout is never committed here.

## The oracle

The committed goldens were generated with **teasit/magic-formula-tyre-library** (GPL-3.0), package
`magicformula.v61`, running under **GNU Octave**. GPL is acceptable in this case, because outlap
runs the library as a tool and captures only its numeric outputs. It never captures the source. The
generator calls `magicformula.v61.eval` with a parameter struct directly, which bypasses the object
reader and the `.tir` reader. Each CSV records the exact oracle commit and Octave version in its
provenance header. A test asserts this.

If teasit is not available, use MFeval by Marco Furlan under Octave. It is an equivalent oracle.
Read its LICENSE first. Digitized figures from the Pacejka book are the last-resort fallback. They
have a looser gate, and that gate carries its own label.

## The CSV contract

Each tire gets one file for each channel group, under
`crates/outlap-tire/tests/golden/<tyre-slug>/`. The files are `fx0.csv`, `fy0_mz.csv`, and
`combined.csv`. Each row holds one evaluation point, with the inputs on the left and the outputs on
the right. All values use **SI units and the ISO 8855 sign convention**.

Every file starts with provenance header lines that begin with `#`. The oracle must be pinned to a
**commit** in the form `oracle: … @ <hash>`. A test enforces this:

```
# generator: tools/goldens/gen_mf61_goldens.m, oracle: <name> @ <commit> (<license>), GNU Octave <version>
# tyre: <slug> (Pacejka 2006 Table A3.1), ISO 8855 sign
kappa,alpha_rad,gamma_rad,fz_n,p_pa,vx_mps,fx_n,fy_n,mz_nm,mx_nm,my_nm
```

## The sweep grid, held near or below 1 MB for each tire

- `fx0.csv`: 41 points of κ in [−0.30, 0.30], crossed with Fz in {0.5, 1, 1.5, 2}·FNOMIN, at
  γ = 0 and α = 0.
- `fy0_mz.csv`: 41 points of α in [−0.21, 0.21] rad, which is about ±12°, crossed with the same set
  of Fz and with γ in {−4°, 0, 4°}, at κ = 0.
- `combined.csv`: 11 points of κ crossed with 11 points of α, at Fz = FNOMIN and γ = 0.
- `V = LONGVL` throughout. Pressure is held at NOMPRES. The 2nd-edition reference tire carries no
  `PP*` terms, so a pressure sweep exercises nothing. See its README.

## The tolerance, which lives in the test code and not in the CSV

For each point, `|model − ref| ≤ max(0.005·|ref|, floor)`. Each channel has an absolute floor, so
that the zero crossings do not divide by a value near zero. Mz crosses zero, and so do Fx and Fy at
zero slip. The floors are:

- Fx: `floor = 0.005 · PDX1 · Fz`
- Fy: `floor = 0.005 · |PDY1| · Fz`
- Mz: `floor = 0.005 · max_row|Mz_ref|` within the Fz bin

The Rust golden test (`crates/outlap-tire/tests/golden.rs`) applies this rule. The Python
cross-check will reuse it verbatim when the Python forward model arrives (M2 PR7).

## How to regenerate, and the rule that governs it

These files come from an external oracle. Therefore there is no in-tree `--bless`. To regenerate
them:

1. Run `MF_ORACLE_SRC=/path/to/oracle/src ./run.sh`. This stages the package, runs Octave, and
   rewrites the CSVs.
2. Commit the CSVs **only in a PR that also updates the provenance headers and states the reason
   for the change**, in physics or in tooling. A reviewer must stop at a CSV diff that carries no
   such note.

## Status

The goldens for `pacejka_2006_205_60r15` are committed. `crates/outlap-tire/tests/golden.rs` gates
them at ≤ 0.5%. The reference tire also has the load check and the physical-sanity check in
`crates/outlap-tire/tests/reference.rs`.
