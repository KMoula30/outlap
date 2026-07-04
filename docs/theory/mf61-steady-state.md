<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# MF6.1 — steady-state tire force/moment model

`outlap-tire` implements the steady-state Magic Formula 6.1: pure- and combined-slip `Fx`, `Fy`,
aligning moment `Mz`, overturning moment `Mx`, and rolling-resistance moment `My`, including the
Besselink inflation-pressure terms. Implemented clean-room from Pacejka's book (3rd ed., 2012)
only; the MATLAB tools named in the validation plan are numerical oracles whose *outputs* are
used as data — never sources of code.

Equation numbers below refer to the "Full set of equations" block of Chapter 4 (§4.3.2,
eqs. 4.E1–4.E78). Anchored numbers were transcribed from the book; numbers marked `(~)` must be
re-verified against the physical text. Where a golden comparison disagrees with a book-literal
transcription, check the published 3rd-edition errata first — the `SHy` shift (eq. 4.E27) is the
documented hotspot.

## Symbols and sign conventions (ISO-W)

| symbol | meaning |
|---|---|
| `κ` | longitudinal slip ratio `−V_sx/\|V_cx\|` (dimensionless; > 0 driving, −1 locked wheel) |
| `α` | side-slip angle (rad), `tan α = V_sy/\|V_cx\|`; `α* = tan α · sgn(V_cx)` (4.E3) |
| `γ` | inclination/camber (rad); `γ* = sin γ` (4.E4) |
| `F_z` | normal load (N, compressive-positive); `F_z ≤ 0` ⇒ all outputs exactly zero |
| `p` | inflation pressure (Pa); `dpi = (p − p₀)/p₀` (4.E2b), `p₀ = NOMPRES` |
| `F'_z0` | scaled nominal load `λ_Fz0·F_z0` (4.E1); `dfz = (F_z − F'_z0)/F'_z0` (4.E2a) |
| `V_cx` | contact-center forward velocity (m/s); `V₀ = LONGVL` |
| `λ_*` | the `L*` scaling factors of the `.tir` `[SCALING_COEFFICIENTS]` section |

Axes are ISO 8855 (x forward, y left, z up). The load-bearing sign consequences, pinned by
property tests: `K_xκ = ∂Fx/∂κ|₀ > 0`; `K_yα = ∂Fy/∂α|₀` carries the sign of `PKY1` (negative),
so `Fy(α > 0) < 0`; and `Mz = −t·F_y + M_zr` is restoring *because* `F_y` is negative — no
absolute values appear anywhere in the sign chain. `sgn(0)` maps to +1 (branch selector, not a
true signum: a zero would annihilate force terms at standstill).

## Model structure

```
Fx  = G_xα(α*) · Fx0(κ)                        4.E50   (G: 4.E51–4.E57 ~)
Fy  = G_yκ(κ) · Fy0(α*) + SV_yκ(κ)             4.E58   (G: 4.E59–4.E67 ~)
Mz  = −t(α_t,eq)·(Fy − SV_yκ) + M_zr(α_r,eq) + s·Fx      4.E71–4.E78
Mx  = R0·Fz·λ_Mx·{QSX1..QSX11, PPMX1 terms}    4.E69 ~
My  = −sgn(V_cx)·R0·Fz·λ_My·{QSY1..QSY8}·(Fz/Fz0)^QSY7·(p/p₀)^QSY8    4.E70 ~
```

- **Pure slip** `Fx0` (4.E9–4.E18) and `Fy0` (4.E19–4.E30) are the sine magic formula
  `D·sin(C·atan(B·x − E·(B·x − atan(B·x)))) + SV` with load- (`dfz`), pressure- (`dpi`,
  Besselink `PPX*`/`PPY*`) and camber-dependent factors. `E` is clamped ≤ 1 (book requirement —
  beyond it the curve folds back).
- **Combined slip** uses the cosine-weighting (not friction-ellipse) formulation: normalized
  cosine magic formulas in the other slip quantity, plus the κ-induced ply-steer shift `SV_yκ`.
- **Aligning moment** composes the pneumatic trail acting on the **slip-only (zero-camber)**
  lateral force `G_yκ·Fy0|_{γ=0}` (eq. 4.E74), the residual torque `M_zr`, and the `s·Fx` lever
  arm; equivalent slip angles (4.E77/4.E78) fold κ in via the stiffness ratio `K_xκ/K'_yα`. Two
  subtleties the golden cross-check pinned down: the **entire aligning-moment lateral machinery**
  (`By`, `Cy`, `Kyα`, `SHy`, `SVy`, `Fy0`, *and* the `s`-lever camber term of eq. 4.E76) is
  evaluated at **zero camber** — camber enters `Mz` only through its own coefficients (SHt, Bt, Dt,
  Dr, Et). The book writes `γ*` in `s` (eq. 4.E76), but the operational MF6.1 (MFeval/teasit, which
  `.tir` data is fit against and the ≤0.5% oracle) drops it, so `SSZ3`/`SSZ4` are accepted-but-
  unused — matching keeps the model interoperable. `Et`'s curvature factor is fixed from the *base*
  trail angle `α_t` (shared by pure and combined). The `s·Fx` term is combined-slip only: at
  `κ = 0` the pure aligning moment (4.E31) has no longitudinal term — a deliberate C⁰ step at
  `κ = 0` that matches the standard/oracle (a measure-zero point in transient use; do not "smooth"
  it, or the golden cross-check breaks). Trail and residual carry a `cos α` weighting (the book's
  guarded `cos'α`) that bounds `Mz` at large slip.
- **`My` sign**: rolling resistance opposes rotation; in ISO 8855 forward roll spins +y, so
  `My < 0` at `V_cx > 0`. Confirmed against the oracle goldens.

## Turn-slip and other omissions (v1 scope)

- **Turn-slip/parking is omitted**: every ζ factor of the book equations is unity, written as
  named constants at their use sites so the later upgrade is a diff, not a rewrite.
- The velocity-digressive friction factor (4.E7's `LMUV` branch) is omitted — no `LMUV` in the
  v1 coefficient set; `λ*_μ = λ_μ`. The digressive shift scaling `λ'_μ` (4.E8, `A_μ = 10`)
  **is** implemented and applies to the vertical shifts `SV_x`, `SV_y`, `SV_yγ` only —
  applying it to `D` instead is a classic 0.5%-gate failure.
- `QBZ6` is accepted but unused: the implemented trail camber form (4.E40 ~) is
  `(1 + QBZ4·γ* + QBZ5·|γ*|)`.
- Relaxation transients (σ_κ, σ_α + the exact exponential update) land in a follow-up PR of
  this milestone; the thermal ring (§7.2) and wear (§7.3) are the M5 flagship.

## Parameter defaults (sparse files degrade, never collapse)

Coefficients absent from a `.tyr` default to 0, **except**: all `L*` scalings, `RCX1`, `RCY1`,
`QCZ1` → 1; `PKY2` → 1 and `PKY4` → 2 (a zero `PKY4` would collapse the cornering stiffness
`K_yα ≡ 0` — the 10-key minimum fixture must evaluate sanely); `LONGVL` → 16.7 m/s,
`VXLOW` → 1 m/s. Absent `NOMPRES` disables all pressure terms (`dpi ≡ 0`, `p/p₀ ≡ 1`). An
entirely absent family degrades to zero output (no `QDZ*` ⇒ `Mz ≡ 0`; no `R*` ⇒ combined =
pure), and every degradation is emitted as a note into the loaded-model report — nothing silent.

## Numerical safety

Kernels are panic-free and finite for all finite inputs: `F_z ≤ 0` short-circuits to zero;
`B = K/(C·D + ε)` uses the book's ε device implemented sign-preservingly (`d + ε·sgn(d)`, never
cancelling); the combined-weighting normalizing cosines get a magnitude floor; `α` is clamped to
±(π/2 − 10⁻³) before `tan`; `E ≤ 1` is clamped on the force magic formulas (`Ex`, `Ey`, and the
combined `Exα`/`Eyκ`) — the trail `Et` is deliberately not clamped, matching the standard; the
`Kxκ` `exp` argument and the `My` pressure ratio are bounded before their exp/power. Evaluation is
pure, allocation-free (dhat-gated in CI) and generic over `f32`/`f64`.

## Validation

- Property tests: sign pins, odd symmetry on shift-free subsets, combined-slip containment
  (`G ∈ (0,1]` — only guaranteed at zero shifts and `C ≤ 1`; false in general with `RHX1 ≠ 0`),
  value continuity across `κ = 0`, `α = 0`, `V_cx = 0⁺`, peak scaling linearity, closed-form
  peak agreement (`μ = PD·LMU` when `C > 1`), finiteness over a hostile input box.
- Golden cross-check (HANDOFF §12/§13): all five channels of the Pacejka book reference tyre match
  an independent Magic-Formula implementation (the GPL `teasit` library, run under Octave — its
  numeric outputs used as data only, never its source) to **≤ 0.5%** over pure-longitudinal,
  pure-lateral (incl. ±4° camber), and combined sweeps. The generation is documented and
  reproducible in `tools/goldens/`. This cross-check is what caught the `Mz` camber/`s·Fx`
  subtleties noted above.

## References

- H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012 — Chapter 4
  §4.3.2 "Full set of equations" (4.E1–4.E78): the complete MF6.1 steady-state model, including
  the inflation-pressure extensions.
- I. J. M. Besselink, A. J. C. Schmeitz, H. B. Pacejka, *An improved Magic Formula/Swift tyre
  model that can handle inflation pressure changes*, Vehicle System Dynamics 48(S1), 2010 — the
  pressure terms (`PPX*`, `PPY*`, `PPZ*`, `PPMX1`) folded into MF6.1.
