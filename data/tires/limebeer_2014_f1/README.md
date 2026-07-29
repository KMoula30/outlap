# limebeer_2014_f1: the Perantoni & Limebeer 2014 reference F1 tire (MF6.1 transcription)

This is a clean-room transcription of the tire friction model in:

> G. Perantoni and D. J. N. Limebeer, *Optimal control for a Formula One car with variable
> parameters*, Vehicle System Dynamics **52**(5), 653–678, 2014 (Appendix A + Table 3).
> Open-access manuscript: Oxford University Research Archive,
> `uuid:ce1a7106-0a2c-41af-8449-41541220809f`.

The model in the paper gives peak friction coefficients that are linear in load. It also gives the
peak-slip locations (Table 3, eqs. A.3–A.6). Its shape function is `sin(Q·arctan(S·ρ))`, which
resembles the Magic Formula (eqs. A.11–A.14). This directory re-expresses that model in the MF6.1
form that outlap uses.

**No third-party source code was read for this transcription.** The clean-room policy of the
project permits one related step, which was done: the `fastest-lap` project (MIT,
github.com/juanmanzanero/fastest-lap, `database/vehicles/f1/limebeer-2014-f1.xml`) was read as a
numerical cross-check only, to confirm that its transcription of Table 3 agrees with this one. It
agrees verbatim.

## Provenance of each coefficient

| MF6.1 | Value | Source or derivation |
|---|---|---|
| `FNOMIN` | 4000 N | The mid-point of the reference loads in Table 3 (Fz1 = 2000 N, Fz2 = 6000 N). |
| `UNLOADED_RADIUS` | 0.33 m | Table 4: wheel radius R. |
| `PDX1` | 1.575 | μx at FNOMIN. This is the mean of μx1 = 1.75 (2000 N) and μx2 = 1.40 (6000 N). The function μ(Fz) in the paper is linear (eq. A.3), so it maps **exactly** onto the MF6.1 form `μx = PDX1 + PDX2·dfz`. |
| `PDX2` | −0.35 | The slope, (μx2 − μx at FNOMIN)/0.5. It reproduces 1.75 at 2000 N and 1.40 at 6000 N exactly. |
| `PDY1` | 1.625 | Derived as PDX1, for μy1 = 1.80 and μy2 = 1.45 (eq. A.4). |
| `PDY2` | −0.35 | Derived as PDX2. It gives 1.80 at 2000 N and 1.45 at 6000 N exactly. |
| `PCX1`, `PCY1` | 1.9 | The shape factors in the paper, Qx = Qy = 1.9 (Table 3). The MF factor C has the same effect on the peak shape. |
| `PEX1`, `PEY1` | 0 | The shape in the paper (A.11–A.14) has no term that adjusts curvature. |
| `PKX1`, `PKX2` | 40.80, −5.21 | Fitted numerically against the MF6.1 implementation in this repository. The fit puts the longitudinal friction peak where the FORMULA in the paper actually peaks. See the note below. Target: κ = 0.0831 at 2000 N and 0.0756 at 6000 N. Achieved: 0.0832 and 0.0757. |
| `PKY1`, `PKY2`, `PKY4` | −69.13, 4.40, 2.0 | Fitted the same way, for the lateral peak. Target: 6.80° at 2000 N and 6.05° at 6000 N. Achieved: 6.83° and 6.07°. |
| `RBX1`, `RBY1` | 10.985, 15.775 | The weighting for combined slip (Pacejka 4.E50–4.E67). Fitted so that the attainable (Fx, Fy) boundary in MF6.1 reproduces the normalized-ρ coupling of PL2014 (A.7–A.16) at the reference loads. Force ratios agree to about 5% through the mixed region. |
| `LMUX`, `LMUY` | 1.0 | No scaling. |

**Note on peak location.** PL2014 is inconsistent with itself here. Table 3 states that κmax and
αmax are the "slip for the friction peak" (0.11 and 0.10, 9° and 8°). The formula in the paper
disagrees. With S = π/(2·arctan Q), eqs. A.11–A.14 reach their maximum at ρ = tan(π/2Q)/S ≈
**0.756**. The true peaks therefore sit at 0.756 times the stated values: κ ≈ 0.083 and 0.076, and
α ≈ 6.8° and 6.0°. The validation target is the *simulation* in the paper, not its table.
Therefore this transcription anchors the peaks where the formula peaks. This does not change the
peak μ magnitudes, because both models reach μmax exactly.

The built model was verified with `Tyre.peak_mu` and with force sweeps. Peak μ is exact at 2000 N,
4000 N, and 6000 N. Each peak-slip location is within 0.5% of the value that the formula gives.
Through the mixed region, the combined-slip force ratios are within about 5% of the ρ-coupling in
the paper.

## Known differences in modeling, documented and not hidden

- **Combined slip.** The paper couples the two slips through ρ, the normalized slip magnitude
  (eq. A.10). MF6.1 uses the standard cosine weighting functions instead. Both forms reduce to the
  same pure-slip peaks. They differ by a small amount inside the mixed-slip region.
- **The `thermal:` and `wear:` blocks.** The `tyr/1.0` schema requires both blocks. PL2014 models
  neither one. Therefore both blocks hold synthetic racing-slick placeholders. The QSS validation
  laps do not read them. Tire thermal and wear models arrive in M5.
- **Omitted effects.** There is no aligning moment (`Mz = 0`), no camber sensitivity, and no
  pressure sensitivity. This matches the paper, which models none of them.
