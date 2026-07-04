<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# TUMFTM Roborace MF5.2 reference tyre (DevBot)

The MF5.2 Magic-Formula parameter set from the **TUM Roborace software stack**, described by
TUMFTM as "resembl[ing] a sport focused road tire" on the Roborace DevBot autonomous racing
platform. It serves as outlap's racing-class reference tyre alongside the Pacejka book
passenger-car set.

- Nominal load `FNOMIN` = 3000 N, unloaded radius `R0` = 0.3 m, ISO sign convention.
- Peak grip in this model: `μx ≈ 1.5·0.97 ≈ 1.46`, `μy ≈ 1.2·0.97 ≈ 1.16` (the `LMUX`/`LMUY`
  = 0.97 scalings are part of the published set).

## Provenance, license, attribution

Transcribed **verbatim** from
[TUMFTM/Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics) (**Apache-2.0**), file
[`python3/ocd_vehicle_models_py/config/OCD_Vehicle_RWD_LSD__PT1__DOUBLE_TRACK__MF52__DEFAULT.json`
at commit `0a92c6868bed61ddbdfd42587225178c3bec8930`](https://github.com/TUMFTM/Open-Car-Dynamics/blob/0a92c6868bed61ddbdfd42587225178c3bec8930/python3/ocd_vehicle_models_py/config/OCD_Vehicle_RWD_LSD__PT1__DOUBLE_TRACK__MF52__DEFAULT.json),
which credits the parameters to
[TUMFTM/sim_vehicle_dynamics](https://github.com/TUMFTM/sim_vehicle_dynamics) (the Roborace
simulation). All four wheels in the source carry the identical set; this file is that set.
Copyright the Open-Car-Dynamics authors (TUM Institute of Automotive Technology); this
attribution is retained per Apache-2.0 alongside the CC-BY-SA-4.0 header on the transcription.
Coefficient values are facts (not copyrightable expression); no source code was derived from.

## MF5.2 → MF6.1 mapping

outlap's kernels implement MF6.1 (Pacejka 2012, 3rd ed.). The source set is MF5.2 expressed in
an MF6.1-ish superset (it already tabulates `PKY4`–`PKY7` and `PPY1`–`PPY5`). The mapping:

- **Inflation pressure — exact no-op.** The source has no pressure model (no
  `NOMPRES`/`INFLPRES`; `PPY1–5 = 0.0`). `NOMPRES` and all `PP*` are **omitted** from this file;
  the outlap loader then sets `dpi ≡ 0` and pressure-ratio ≡ 1 exactly
  (`crates/outlap-tire/src/mf61/params.rs`), which is identical to `PP* = 0` with
  `NOMPRES = INFLPRES`. Pressure sweeps exercise nothing on this tyre.
- **`PKY4 = 2.0`** — verbatim from the source (the MF5.2-implicit value).
- **Camber stiffness — the one non-verbatim value.** True MF5.2 routes camber through
  `PHY3` (= 0.004 here, with `LGAX/LGAY/LGAZ/LKYG` = 1.0), and that is how this parameter set was
  fitted and raced: the original `TUMFTM/sim_vehicle_dynamics` implementation (`MF_52.m`) reads
  `PHY3` in `S_Hy` with plain `LMUY` on the vertical shift. Note the **pinned OCD port differs
  from its own parameter set here**: its `mf_52.cpp` declares `PHY3` but never reads it — it
  already uses the MF6.1-style `PKY6` route with the file's `PKY6 = 0.0`, so the pinned OCD
  simulator produces essentially zero first-order camber stiffness for this tyre. outlap follows
  the *original* stack (where the fit lives), not the port's dead parameter: MF6.1 (and outlap)
  has no `PHY3` — camber enters via `K_yγ0 = Fz·(PKY6 + PKY7·dfz)·LKYC`, and the `PVY3/PVY4`
  vertical-shift route cancels exactly inside `S_Hy` (see `crates/outlap-tire/src/mf61/fy.rs`).
  We therefore fold the 5.2 route into `PKY6` by **equating the small-camber Fy sensitivity at
  `FNOMIN`** (`dfz = 0`, `dpi = 0`, unity camber scalings); this intentionally diverges from the
  pinned OCD port's Fy(γ) by ≈ −56 N per degree of camber at `FNOMIN`:

  ```text
  MF5.2:  ∂Fy/∂γ = K_yα(FNOMIN)·PHY3 + FNOMIN·PVY3·LMUY
  MF6.1:  ∂Fy/∂γ = K_yγ0 = FNOMIN·PKY6

  K_yα(FNOMIN) = PKY1·FNOMIN·sin(PKY4·atan(1/PKY2))
               = −75.5·3000·sin(2·atan(1/4.65)) = −93113 N/rad

  PKY6 = K_yα·PHY3/FNOMIN + PVY3·LMUY
       = (−93113·0.004)/3000 + (−0.97·0.97) = −0.1242 − 0.9409 = −1.0651
  ```

  `PKY7 = 0` (source value): the match is exact at `FNOMIN` only; away from nominal load the
  5.2-vs-6.1 camber-stiffness difference grows with `|dfz|` (small — `PHY3` is a minor term).
  Net camber stiffness: `K_yγ0 ≈ −3195 N/rad` (≈ −56 N per degree of camber at `FNOMIN`).
- **Rolling resistance.** The source's chassis-level `tire.rolling_resistance_coefficient`
  = 0.025 maps to `QSY1 = 0.025` (in the MF `My` form, `QSY1` *is* the rolling-resistance
  coefficient at nominal conditions). The other `QSY*` default to zero.
- **`Mz ≡ 0`, `Mx ≡ 0`.** The source tabulates no aligning-moment (`QBZ*/QCZ*/QDZ*/SSZ*`) or
  overturning (`QSX*`) coefficients, so those families are omitted here and both moments
  evaluate to zero with this parameter set.
- **Structural.** `UNLOADED_RADIUS` = 0.3 m from `tire.rolling_radius_m` (the source's only
  radius; rolling ≈ unloaded is the documented approximation), `VERTICAL_STIFFNESS` = 250000 N/m
  from `tire.spring_stiffness_Npm`. `WIDTH`/`RIM_RADIUS`/`LONGVL` are not in the source and are
  omitted (`LONGVL` falls back to the documented 16.7 m/s default).
- **Relaxation.** `PTX1–3`/`PTY1–2` carry over verbatim and feed the §7.1 first-order
  relaxation transient.

## Per-coefficient source table

| Coefficients | Status |
|---|---|
| `PCX1 PDX1 PDX2 PDX3 PEX1 PEX2 PEX3 PEX4 PKX1 PKX2 PKX3 PHX1 PHX2 PVX1 PVX2` | verbatim |
| `RBX1 RBX2 RBX3 RCX1 REX1 REX2 RHX1` | verbatim |
| `PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 PVY4` | verbatim |
| `RBY1 RBY2 RBY3 RBY4 RCY1 REY1 REY2 RHY1 RHY2 RVY1 RVY2 RVY3 RVY4 RVY5 RVY6` | verbatim |
| `PTX1 PTX2 PTX3 PTY1 PTY2` | verbatim |
| `FNOMIN` | verbatim |
| `LFZO LCX LMUX LEX LKX LHX LVX LCY LMUY LEY LKY LHY LVY LTR LRES LXAL LYKA LVYKA LS LMX LMY LVMX LGYR LSGKP LSGAL` | verbatim (all 1.0 except `LMUX`/`LMUY` = 0.97) |
| `PKY6` | **mapped** −1.0651 (source file 0.0; folds the `PHY3` = 0.004 camber route of the original `sim_vehicle_dynamics` implementation — equation and OCD-port caveat above) |
| `QSY1` | **mapped** 0.025 (from `tire.rolling_resistance_coefficient`) |
| `UNLOADED_RADIUS` | **mapped** 0.3 (from `tire.rolling_radius_m`) |
| `VERTICAL_STIFFNESS` | **mapped** 250000 (from `tire.spring_stiffness_Npm`) |
| `PHY3` (0.004), `LGAX LGAY LGAZ LKYG` (1.0) | **retired** — MF5.2-only camber route, folded into `PKY6` |
| `PPY1–PPY5` (0.0) | **omitted** — no pressure model in the source; `dpi ≡ 0` exactly without `NOMPRES` |
| `QBZ* QCZ* QDZ* QEZ* QHZ* SSZ* QSX*` | **not in source** — `Mz ≡ 0`, `Mx ≡ 0` |

## Non-published blocks

`thermal` and `wear` are **synthetic placeholders** in a racing-slick band (the schema
slick-fixture recipe, labelled in the file and in `provenance.source`); those models land in M5.
`provenance.synthetic: false` reflects that the force coefficients are the published set.
