# limebeer_2014_f1 — the Perantoni & Limebeer 2014 reference F1 tyre (MF6.1 transcription)

Clean-room transcription of the tyre friction model of:

> G. Perantoni and D. J. N. Limebeer, *Optimal control for a Formula One car with variable
> parameters*, Vehicle System Dynamics **52**(5), 653–678, 2014 (Appendix A + Table 3).
> Open-access manuscript: Oxford University Research Archive,
> `uuid:ce1a7106-0a2c-41af-8449-41541220809f`.

The paper's model gives load-linear peak friction coefficients and peak-slip locations
(Table 3, eqs. A.3–A.6) with a `sin(Q·arctan(S·ρ))` magic-formula-like shape (eqs. A.11–A.14).
This directory re-expresses it in outlap's MF6.1 form. **No third-party source code was consulted
for this transcription**; the `fastest-lap` project (MIT,
github.com/juanmanzanero/fastest-lap, `database/vehicles/f1/limebeer-2014-f1.xml`) was *read as a
numerical cross-check* that its transcription of Table 3 matches ours (it does, verbatim), per the
project clean-room policy.

## Per-coefficient provenance

| MF6.1 | Value | Source / derivation |
|---|---|---|
| `FNOMIN` | 4000 N | mid-point of Table 3's reference loads (Fz1 = 2000 N, Fz2 = 6000 N) |
| `UNLOADED_RADIUS` | 0.33 m | Table 4: wheel radius R |
| `PDX1` | 1.575 | μx at FNOMIN: mean of μx1 = 1.75 (2000 N) and μx2 = 1.40 (6000 N) — the paper's μ(Fz) is linear (eq. A.3), which maps **exactly** onto MF6.1's `μx = PDX1 + PDX2·dfz` |
| `PDX2` | −0.35 | slope: (μx2 − μx at FNOMIN)/0.5 ⇒ reproduces 1.75 @ 2000 N and 1.40 @ 6000 N exactly |
| `PDY1` | 1.625 | as PDX1 for μy1 = 1.80, μy2 = 1.45 (eq. A.4) |
| `PDY2` | −0.35 | as PDX2 ⇒ 1.80 @ 2000 N and 1.45 @ 6000 N exactly |
| `PCX1`, `PCY1` | 1.9 | the paper's shape factors Qx = Qy = 1.9 (Table 3); MF's C plays the same peak-shape role |
| `PEX1`, `PEY1` | 0 | the paper's shape (A.11–A.14) has no curvature-adjustment term |
| `PKX1`, `PKX2` | 40.80, −5.21 | fitted numerically against this repository's MF6.1 implementation so the longitudinal friction peak sits where the paper's FORMULA actually peaks (see note below): κ = 0.0831 @ 2000 N → 0.0756 @ 6000 N (achieved: 0.0832 / 0.0757) |
| `PKY1`, `PKY2`, `PKY4` | −69.13, 4.40, 2.0 | fitted likewise for the lateral peak at 6.80° @ 2000 N → 6.05° @ 6000 N (achieved: 6.83° / 6.07°) |
| `RBX1`, `RBY1` | 10.985, 15.775 | combined-slip weighting (Pacejka 4.E50–4.E67), fitted so the MF6.1 attainable (Fx, Fy) boundary reproduces PL2014's normalised-ρ coupling (A.7–A.16) at the reference loads (force ratios within ~5% through the mixed region) |
| `LMUX`, `LMUY` | 1.0 | no scaling |

**Peak-location note (a PL2014 self-inconsistency):** Table 3 states κmax/αmax as "slip for the
friction peak" (0.11/0.10, 9°/8°), but the paper's own formula (A.11–A.14, with
S = π/(2·arctan Q)) reaches its maximum at ρ = tan(π/2Q)/S ≈ **0.756**, i.e. at 0.756 × the stated
values (κ ≈ 0.083/0.076, α ≈ 6.8°/6.0°). Since the validation target is the paper's *simulation*,
this transcription anchors the peaks where their formula actually peaks. Peak μ magnitudes are
unaffected (both models reach exactly μmax).

Verified against the built model (`Tyre.peak_mu` + force sweeps): peak μ exact at 2000/4000/6000 N;
peak slip locations within 0.5% of the formula-true values; combined-slip force ratios within ~5%
of the paper's ρ-coupling through the mixed region.

## Known modelling differences (documented, not hidden)

- **Combined slip**: the paper couples the slips through the normalised-slip magnitude ρ
  (eq. A.10); MF6.1 uses the standard cosine weighting functions. Both reduce to the same pure-slip
  peaks; they differ modestly in the mixed-slip interior.
- **`thermal:` / `wear:` blocks** are required by `tyr/1.0` but are *not* part of PL2014 (the paper
  models neither). They are synthetic racing-slick placeholders and are unused by the QSS
  validation laps (tyre thermal/wear land in M5).
- No aligning moment (`Mz = 0`), no camber and no pressure sensitivity — matching the paper, which
  models none of these.
