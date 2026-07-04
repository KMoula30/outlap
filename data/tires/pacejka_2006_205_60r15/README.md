<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Pacejka book reference tyre — 205/60R15 91V (2.2 bar)

The worked-example passenger-car tyre from **H. B. Pacejka, *Tyre and Vehicle Dynamics*, 2nd ed.
(2006), Appendix 3, Table A3.1** — the standard reference Magic-Formula parameter set. It doubles
as outlap's MF6.1 validation tyre.

- Nominal load `FNOMIN` = 4000 N, unloaded radius `R0` = 0.313 m, reference speed `V0` = 16.67 m/s,
  ISO sign convention.
- Peak grip in this model: `μx ≈ 1.21`, `μy ≈ 0.99` (longitudinal > lateral, as expected).

## Provenance and clean-room note

The MF6.1 force/moment coefficients are transcribed **verbatim** from Table A3.1. Coefficient
values are facts (not copyrightable expression); the book PDF itself is not committed. This is a
data transcription, not a derivation from any third-party code.

## Edition differences (this is the 2nd edition; outlap's kernels cite the 3rd)

outlap's MF6.1 kernels are implemented from the **3rd edition (2012)** equation numbering, which
adds the Besselink inflation-pressure terms. The 2nd-edition table therefore differs in ways that
are handled explicitly:

- **No inflation-pressure terms.** The table has no `PP*` coefficients, so pressure has no effect
  in the model. `NOMPRES` (220 kPa) is recorded for reference; golden sweeps for this tyre should
  hold pressure at nominal (a pressure sweep exercises nothing here).
- **Overturning moment `Mx ≡ 0`.** The book lists `qSx1 = qSx2 = qSx3 = 0` and App 3.2 excludes
  eq. 4.E68 for this set, so `Mx` is modelled as zero (the `QSX*` family is omitted from the file).
- **Rolling resistance `My`.** Only `qsy1 = 0.01` is tabulated (App 3.2 routes rolling resistance
  through the SWIFT form, eq. 9.231). The other `QSY*` default to zero.
- **Camber terms** follow 2nd-edition conventions, but the golden cross-check is against a 3rd-ed
  (6.1.2) oracle fed the *same* coefficients, so it validates our implementation against that
  standard — including `γ = ±4°` rows gated at ≤ 0.5% on `Fy` and `Mz`. (Book-*figure* comparison,
  which would surface the 2nd-vs-3rd edition camber-shift difference, is a separate exercise not
  done here.)
- **Sign set.** The book's ISO set uses **negative** `PDY1 = −0.990` and `PKY1 = −14.95`; the model
  handles this directly (no absolute values), yielding the correct `Fy(α>0) < 0` and restoring
  `Mz`. Verified in `crates/outlap-tire/tests/reference.rs`.

## Non-published blocks

`thermal` and `wear` are **synthetic placeholders** in a passenger-car band (labelled in the file
and in `provenance.source`); those models land in M5. `provenance.synthetic: false` reflects that
the force/moment coefficients are the published measured set.
