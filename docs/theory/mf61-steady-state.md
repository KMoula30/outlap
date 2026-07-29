<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# MF6.1: the steady-state tire force and moment model

`outlap-tire` implements the steady-state Magic Formula 6.1. It covers `Fx` and `Fy` under pure slip
and combined slip, the aligning moment `Mz`, the overturning moment `Mx`, and the rolling-resistance
moment `My`. It includes the Besselink terms for inflation pressure.

It is implemented clean-room, from the Pacejka book (3rd ed., 2012), and from nothing else. The
MATLAB tools named in the validation plan are numerical oracles. outlap uses *their outputs* as
data. It never uses them as sources of code.

The equation numbers below refer to the "Full set of equations" block of Chapter 4, §4.3.2, which
holds eqs. 4.E1–4.E78. A number without a mark was transcribed from the book. A number marked `(~)`
must be re-verified against the physical text.

Where a golden comparison disagrees with a literal transcription from the book, check the published
errata for the 3rd edition first. The `SHy` shift, eq. 4.E27, is the documented hotspot.

These equations read two hooks on `SlipState`: inflation pressure `p`, and the friction scalings
`mu_scale_x` and `mu_scale_y`. Here they hold at 1.0 and at cold-set values. Over a stint, the
[tire thermal ring](tire-thermal.md) computes the multipliers for pressure and for the grip window
that drive them.

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

The axes are ISO 8855: x forward, y left, z up.

Property tests pin the sign consequences that carry the physics. `K_xκ = ∂Fx/∂κ|₀ > 0`. `K_yα =
∂Fy/∂α|₀` carries the sign of `PKY1`, which is negative, so `Fy(α > 0) < 0`. And
`Mz = −t·F_y + M_zr` restores *because* `F_y` is negative. No absolute value appears anywhere in
that sign chain.

`sgn(0)` maps to +1. It is a branch selector, not a true signum. A zero would annihilate the force
terms at standstill.

## The structure of the model

```
Fx  = G_xα(α*) · Fx0(κ)                        4.E50   (G: 4.E51–4.E57 ~)
Fy  = G_yκ(κ) · Fy0(α*) + SV_yκ(κ)             4.E58   (G: 4.E59–4.E67 ~)
Mz  = −t(α_t,eq)·(Fy − SV_yκ) + M_zr(α_r,eq) + s·Fx      4.E71–4.E78
Mx  = R0·Fz·λ_Mx·{QSX1..QSX11, PPMX1 terms}    4.E69 ~
My  = −sgn(V_cx)·R0·Fz·λ_My·{QSY1..QSY8}·(Fz/Fz0)^QSY7·(p/p₀)^QSY8    4.E70 ~
```

**Pure slip.** `Fx0` (4.E9–4.E18) and `Fy0` (4.E19–4.E30) are the sine magic formula,
`D·sin(C·atan(B·x − E·(B·x − atan(B·x)))) + SV`. Its factors depend on load through `dfz`, on
pressure through `dpi` and the Besselink `PPX*` and `PPY*` terms, and on camber. `E` is clamped at
or below 1, which the book requires. Beyond 1 the curve folds back.

**Combined slip.** This uses the cosine-weighting formulation, not a friction ellipse. It applies
normalized cosine magic formulas in the other slip quantity, and it adds `SV_yκ`, the ply-steer
shift that κ induces.

**Aligning moment.** `Mz` composes three parts: the pneumatic trail acting on the lateral force from
**slip only, at zero camber**, which is `G_yκ·Fy0|_{γ=0}` (eq. 4.E74); the residual torque `M_zr`;
and the lever arm `s·Fx`. The equivalent slip angles, 4.E77 and 4.E78, fold κ in through the
stiffness ratio `K_xκ/K'_yα`.

The golden cross-check pinned down two subtleties here.

First, the **entire lateral machinery of the aligning moment** is evaluated at **zero camber**. That
covers `By`, `Cy`, `Kyα`, `SHy`, `SVy`, `Fy0`, *and* the camber term of the `s` lever in eq. 4.E76.
Camber enters `Mz` only through its own coefficients: SHt, Bt, Dt, Dr, and Et. The book writes `γ*`
in `s` (eq. 4.E76). The operational MF6.1 drops it, and that is what MFeval and teasit do. `.tir`
data is fitted against those tools, and they are the ≤0.5% oracle. `SSZ3` and `SSZ4` are therefore
accepted but unused. Matching them keeps the model interoperable.

Second, the curvature factor `Et` is fixed from the *base* trail angle `α_t`, which pure slip and
combined slip share.

The `s·Fx` term appears under combined slip only. At `κ = 0`, the pure aligning moment (4.E31) has
no longitudinal term. That is a deliberate C⁰ step at `κ = 0`, and it matches the standard and the
oracle. In transient use it is a point of measure zero. Do not "smooth" it, or the golden
cross-check breaks.

The trail and the residual both carry a `cos α` weighting, which the book writes as a guarded
`cos'α`. It bounds `Mz` at large slip.

**The sign of `My`.** Rolling resistance opposes rotation. Under ISO 8855, rolling forward spins the
wheel about +y. Therefore `My < 0` when `V_cx > 0`. The oracle goldens confirm this.

## Turn-slip and other omissions, in the v1 scope

- **Turn-slip and parking are omitted.** Every ζ factor in the book equations is unity. Each one is
  written as a named constant at its use site, so that the later upgrade is a diff and not a
  rewrite.
- **The velocity-digressive friction factor is omitted.** That is the `LMUV` branch of 4.E7. The v1
  coefficient set has no `LMUV`, so `λ*_μ = λ_μ`.

  The digressive scaling of the shifts, `λ'_μ` from 4.E8 with `A_μ = 10`, **is** implemented. It
  applies to the vertical shifts `SV_x`, `SV_y`, and `SV_yγ`, and to nothing else. Applying it to
  `D` instead is a classic way to fail the 0.5% gate.
- **`QBZ6` is accepted but unused.** The implemented camber form for the trail (4.E40 ~) is
  `(1 + QBZ4·γ* + QBZ5·|γ*|)`.
- **Relaxation transients**, which are σ_κ, σ_α, and the exact exponential update, land in a
  follow-up PR of this milestone. The thermal ring (§7.2) and wear (§7.3) are the M5 flagship.

## Defaults for parameters: a sparse file degrades, and never collapses

A coefficient that a `.tyr` file omits defaults to 0, with these exceptions. Every `L*` scaling, and
`RCX1`, `RCY1`, and `QCZ1`, default to 1. `PKY2` defaults to 1, and `PKY4` defaults to 2, because a
zero `PKY4` would collapse the cornering stiffness to `K_yα ≡ 0`, and the minimum fixture of 10 keys
must still evaluate sanely. `LONGVL` defaults to 16.7 m/s, and `VXLOW` to 1 m/s.

An absent `NOMPRES` disables every pressure term, so `dpi ≡ 0` and `p/p₀ ≡ 1`.

A family that is absent entirely degrades to zero output. With no `QDZ*`, `Mz ≡ 0`. With no `R*`,
combined slip equals pure slip.

Every degradation is emitted as a note in the loaded-model report. Nothing is silent.

## Numerical safety

The kernels are panic-free, and they return finite values for every finite input.

`F_z ≤ 0` short-circuits to zero. `B = K/(C·D + ε)` uses the ε device from the book, implemented so
that it preserves sign, as `d + ε·sgn(d)`. It never cancels. The normalizing cosines of the combined
weighting get a floor on their magnitude. `α` is clamped to ±(π/2 − 10⁻³) before `tan`.

`E ≤ 1` is clamped on the force magic formulas: `Ex`, `Ey`, and the combined `Exα` and `Eyκ`. The
trail `Et` is deliberately not clamped, which matches the standard. The `exp` argument of `Kxκ` and
the pressure ratio of `My` are both bounded before their exponential or power.

Evaluation is pure. It allocates nothing, which dhat gates in CI. It is generic over `f32` and
`f64`.

## Validation

- **Property tests** cover: the sign pins; odd symmetry on subsets that have no shifts; containment
  under combined slip, `G ∈ (0,1]`, which holds only at zero shifts and `C ≤ 1`, and is false in
  general when `RHX1 ≠ 0`; continuity of value across `κ = 0`, `α = 0`, and `V_cx = 0⁺`; linearity
  of peak scaling; agreement with the closed-form peak, `μ = PD·LMU` when `C > 1`; and finiteness
  over a hostile box of inputs.
- **The golden cross-check** (HANDOFF §12 and §13) takes all five channels of the reference tire
  from the Pacejka book and matches them against an independent Magic-Formula implementation, to
  **≤ 0.5%**. That implementation is the GPL `teasit` library, run under Octave. outlap uses its
  numeric outputs as data only, never its source. The sweeps cover pure longitudinal, pure lateral
  including ±4° camber, and combined. `tools/goldens/` documents the generation and makes it
  reproducible.

  This cross-check is what caught the subtleties in `Mz`, around camber and the `s·Fx` term, noted
  above.

## First-order relaxation, which is the transient lag

A tire does not reach its steady-state slip force instantly. The contact patch must roll a
*relaxation length* `σ` before the deflection catches up. Each slip channel `x ∈ {κ, α}` therefore
obeys

```
σ·ẋ + |V_x|·x = |V_x|·x_ss
```

(Pacejka 2012, §7.2 and §8.5). The `relax` module of `outlap-tire` advances this with the
**exact-exponential** update (HANDOFF §11.2). That update is stable at every speed, without
condition, and it needs no implicit solve. It is the single most important decision about the
integrator:

```
x ← x_ss + (x − x_ss)·exp(−|V_x|·dt/σ)
```

The relaxation lengths come from the MF5.2 `PT*` transient coefficients, when a file has them. The
forms are marked `(~)`, and must be re-checked against the book:

```
σ_κ = F_z·(PTX1 + PTX2·dfz)·exp(−PTX3·dfz)·(R0/FNOMIN)·λσκ
σ_α = PTY1·sin(2·atan(F_z/(PTY2·F'_z0)))·(1 − PKY3·|γ*|)·R0·λFz0·λσα
```

If the `PT*` set is absent, the lengths fall back to the identity from carcass stiffness,
`σ = K_slip / C_carcass`, using `LONGITUDINAL_STIFFNESS` and `LATERAL_STIFFNESS`. If that also
fails, they fall back to `0.5·R0`. That last resort is loud: the loaded-model report records it.

Every length is floored at `σ_min = 10⁻³ m`. The caller passes `|V_x|.max(VXLOW)`, so a car at
standstill still relaxes.

Property tests pin three properties of the update. It is a contraction, so `|x − x_ss|` never grows
for `dt ≥ 0`. It is exact against the analytic ratio. And it composes, so two half-steps equal one
full step. `relax_step` and the length queries allocate nothing, which dhat gates.

The transient tiers, T2 and T3, consume relaxation. The QSS tiers use the steady-state forces
directly.

## References

- H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012 — Chapter 4
  §4.3.2 "Full set of equations" (4.E1–4.E78): the complete MF6.1 steady-state model, including
  the inflation-pressure extensions. Chapter 7 (§7.2) / Chapter 8 (§8.5): first-order relaxation
  and the relaxation-length coefficients. Chapter 3: the physical brush model (see
  [`brush-model.md`](brush-model.md)).
- I. J. M. Besselink, A. J. C. Schmeitz, H. B. Pacejka, *An improved Magic Formula/Swift tyre
  model that can handle inflation pressure changes*, Vehicle System Dynamics 48(S1), 2010 — the
  pressure terms (`PPX*`, `PPY*`, `PPZ*`, `PPMX1`) folded into MF6.1.
