<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Pacejka book reference tire: 205/60R15 91V (2.2 bar)

This is the worked-example passenger-car tire from **H. B. Pacejka, *Tyre and Vehicle Dynamics*,
2nd ed. (2006), Appendix 3, Table A3.1**. It is the standard reference Magic-Formula parameter set.
outlap also uses it as the MF6.1 validation tire.

- Nominal load `FNOMIN` = 4000 N. Unloaded radius `R0` = 0.313 m. Reference speed `V0` = 16.67 m/s.
  ISO sign convention.
- Peak grip in this model: `μx ≈ 1.21` and `μy ≈ 0.99`. Longitudinal grip is higher than lateral
  grip, as expected.

## Provenance and clean-room note

The force and moment coefficients for MF6.1 are transcribed **verbatim** from Table A3.1. A
coefficient value is a fact, not copyrightable expression. The book PDF is not committed. This work
transcribes data. It derives nothing from third-party code.

## Differences between editions

This table comes from the 2nd edition. The outlap kernels cite the 3rd.

outlap implements its MF6.1 kernels from the equation numbers of the **3rd edition (2012)**, which
adds the Besselink inflation-pressure terms. Therefore the 2nd-edition table differs in four ways.
outlap handles each one explicitly.

- **No inflation-pressure terms.** The table has no `PP*` coefficients. Therefore pressure has no
  effect in this model. `NOMPRES` (220 kPa) is recorded for reference only. Hold the pressure at
  nominal for the golden sweeps of this tire, because a pressure sweep exercises nothing.
- **Overturning moment `Mx ≡ 0`.** The book lists `qSx1 = qSx2 = qSx3 = 0`, and App 3.2 excludes
  eq. 4.E68 for this set. Therefore outlap models `Mx` as zero and omits the `QSX*` family from the
  file.
- **Rolling resistance `My`.** The book tabulates only `qsy1 = 0.01`, because App 3.2 sends rolling
  resistance through the SWIFT form, eq. 9.231. The other `QSY*` coefficients default to zero.
- **Camber terms.** These follow the conventions of the 2nd edition. The golden cross-check uses a
  3rd-edition (6.1.2) oracle, and it feeds that oracle the *same* coefficients. Therefore the check
  validates the outlap implementation against that standard. It includes the `γ = ±4°` rows, gated
  at `≤ 0.5%` on `Fy` and `Mz`. A comparison against the *figures* in the book would show the
  camber-shift difference between the 2nd and 3rd editions. That is a separate exercise, and this
  directory does not do it.
- **Sign set.** The ISO set in the book uses **negative** values: `PDY1 = −0.990` and
  `PKY1 = −14.95`. The model handles these directly and takes no absolute value. It therefore
  yields the correct `Fy(α>0) < 0` and restores `Mz`. `crates/outlap-tire/tests/reference.rs`
  verifies this.

## Blocks that the book does not publish

The `thermal` block and the `wear` block are **synthetic placeholders** in a passenger-car band.
The file labels them, and so does `provenance.source`. Those models arrive in M5.
`provenance.synthetic: false` records one fact: the force and moment coefficients are the published
measured set.
