<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# TUMFTM Roborace MF5.2 reference tire (DevBot)

This is the MF5.2 Magic-Formula parameter set from the **TUM Roborace software stack**. TUMFTM
describes it as a set that "resembl[es] a sport focused road tire". It belongs to the Roborace
DevBot autonomous racing platform. In outlap it is the racing-class reference tire. The
passenger-car reference is the set from the Pacejka book.

- Nominal load `FNOMIN` = 3000 N. Unloaded radius `R0` = 0.3 m. ISO sign convention.
- Peak grip in this model: `μx ≈ 1.5·0.97 ≈ 1.46` and `μy ≈ 1.2·0.97 ≈ 1.16`. The scalings
  `LMUX` = `LMUY` = 0.97 are part of the published set.

## Provenance, license, and attribution

The set is transcribed **verbatim** from
[TUMFTM/Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics) (**Apache-2.0**). The
source file is
[`python3/ocd_vehicle_models_py/config/OCD_Vehicle_RWD_LSD__PT1__DOUBLE_TRACK__MF52__DEFAULT.json`
at commit `0a92c6868bed61ddbdfd42587225178c3bec8930`](https://github.com/TUMFTM/Open-Car-Dynamics/blob/0a92c6868bed61ddbdfd42587225178c3bec8930/python3/ocd_vehicle_models_py/config/OCD_Vehicle_RWD_LSD__PT1__DOUBLE_TRACK__MF52__DEFAULT.json).
That file credits the parameters to
[TUMFTM/sim_vehicle_dynamics](https://github.com/TUMFTM/sim_vehicle_dynamics), the Roborace
simulation. All four wheels in the source carry the same set, and this file is that set.

Copyright belongs to the Open-Car-Dynamics authors at the TUM Institute of Automotive Technology.
Apache-2.0 requires this attribution, and the transcription keeps it next to its CC-BY-SA-4.0
header. A coefficient value is a fact, not copyrightable expression. No source code was derived
from.

## Mapping from MF5.2 to MF6.1

The outlap kernels implement MF6.1 (Pacejka 2012, 3rd ed.). The source set is MF5.2, but it is
written in a superset that is close to MF6.1: it already tabulates `PKY4`–`PKY7` and
`PPY1`–`PPY5`. The mapping follows.

- **Inflation pressure. An exact no-op.** The source has no pressure model. It has no `NOMPRES`
  and no `INFLPRES`, and `PPY1–5 = 0.0`. This file therefore **omits** `NOMPRES` and every `PP*`
  coefficient. The outlap loader then sets `dpi ≡ 0` and the pressure ratio to exactly 1
  (`crates/outlap-tire/src/mf61/params.rs`). This is identical to `PP* = 0` with
  `NOMPRES = INFLPRES`. A pressure sweep exercises nothing on this tire.
- **`PKY4 = 2.0`.** Verbatim from the source. This is the value that MF5.2 implies.
- **Camber stiffness. The one value that is not verbatim.** True MF5.2 routes camber through
  `PHY3`, which is 0.004 here, with `LGAX`, `LGAY`, `LGAZ`, and `LKYG` all 1.0. The parameter set
  was fitted and raced that way: the original `TUMFTM/sim_vehicle_dynamics` implementation
  (`MF_52.m`) reads `PHY3` in `S_Hy`, with plain `LMUY` on the vertical shift.

  Note that **the pinned OCD port disagrees with its own parameter set**. Its `mf_52.cpp` declares
  `PHY3` but never reads it. It uses the MF6.1-style `PKY6` route instead, and the file sets
  `PKY6 = 0.0`. Therefore the pinned OCD simulator produces almost no first-order camber stiffness
  for this tire.

  outlap follows the *original* stack, where the fit lives. It does not follow the dead parameter
  in the port. MF6.1 has no `PHY3`, and neither does outlap. Camber enters through
  `K_yγ0 = Fz·(PKY6 + PKY7·dfz)·LKYC`, and the vertical-shift route through `PVY3` and `PVY4`
  cancels exactly inside `S_Hy` (see `crates/outlap-tire/src/mf61/fy.rs`). The 5.2 route therefore
  folds into `PKY6`. To fold it, equate the Fy sensitivity at small camber at `FNOMIN`, with
  `dfz = 0`, `dpi = 0`, and unity camber scalings. This diverges from the Fy(γ) of the pinned OCD
  port on purpose, by about −56 N for each degree of camber at `FNOMIN`:

  ```text
  MF5.2:  ∂Fy/∂γ = K_yα(FNOMIN)·PHY3 + FNOMIN·PVY3·LMUY
  MF6.1:  ∂Fy/∂γ = K_yγ0 = FNOMIN·PKY6

  K_yα(FNOMIN) = PKY1·FNOMIN·sin(PKY4·atan(1/PKY2))
               = −75.5·3000·sin(2·atan(1/4.65)) = −93113 N/rad

  PKY6 = K_yα·PHY3/FNOMIN + PVY3·LMUY
       = (−93113·0.004)/3000 + (−0.97·0.97) = −0.1242 − 0.9409 = −1.0651
  ```

  `PKY7 = 0`, which is the source value. The match is therefore exact at `FNOMIN` only. Away from
  the nominal load, the difference in camber stiffness between 5.2 and 6.1 grows with `|dfz|`. The
  difference stays small, because `PHY3` is a minor term. Net camber stiffness:
  `K_yγ0 ≈ −3195 N/rad`, which is about −56 N for each degree of camber at `FNOMIN`.
- **Rolling resistance.** The source holds `tire.rolling_resistance_coefficient` = 0.025 at
  chassis level. This maps to `QSY1 = 0.025`, because in the MF form for `My`, `QSY1` *is* the
  rolling-resistance coefficient at nominal conditions. The other `QSY*` coefficients default to
  zero.
- **`Mz ≡ 0` and `Mx ≡ 0`.** The source tabulates no aligning-moment coefficients
  (`QBZ*`, `QCZ*`, `QDZ*`, `SSZ*`) and no overturning coefficients (`QSX*`). This file therefore
  omits those families, and both moments evaluate to zero with this parameter set.
- **Structural values.** `UNLOADED_RADIUS` = 0.3 m comes from `tire.rolling_radius_m`, the only
  radius in the source. Treating the rolling radius as the unloaded radius is a documented
  approximation. `VERTICAL_STIFFNESS` = 250000 N/m comes from `tire.spring_stiffness_Npm`. The
  source has no `WIDTH`, `RIM_RADIUS`, or `LONGVL`, so this file omits all three. `LONGVL` then
  falls back to the documented default of 16.7 m/s.
- **Relaxation.** `PTX1–3` and `PTY1–2` carry over verbatim. They feed the first-order relaxation
  transient of §7.1.

## Source of each coefficient

| Coefficients | Status |
|---|---|
| `PCX1 PDX1 PDX2 PDX3 PEX1 PEX2 PEX3 PEX4 PKX1 PKX2 PKX3 PHX1 PHX2 PVX1 PVX2` | verbatim |
| `RBX1 RBX2 RBX3 RCX1 REX1 REX2 RHX1` | verbatim |
| `PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 PVY4` | verbatim |
| `RBY1 RBY2 RBY3 RBY4 RCY1 REY1 REY2 RHY1 RHY2 RVY1 RVY2 RVY3 RVY4 RVY5 RVY6` | verbatim |
| `PTX1 PTX2 PTX3 PTY1 PTY2` | verbatim |
| `FNOMIN` | verbatim |
| `LFZO LCX LMUX LEX LKX LHX LVX LCY LMUY LEY LKY LHY LVY LTR LRES LXAL LYKA LVYKA LS LMX LMY LVMX LGYR LSGKP LSGAL` | verbatim. All are 1.0 except `LMUX` and `LMUY`, which are 0.97. |
| `PKY6` | **mapped** to −1.0651. The source file holds 0.0. This value folds in the `PHY3` = 0.004 camber route of the original `sim_vehicle_dynamics` implementation. See the equation and the note on the OCD port above. |
| `QSY1` | **mapped** to 0.025, from `tire.rolling_resistance_coefficient`. |
| `UNLOADED_RADIUS` | **mapped** to 0.3, from `tire.rolling_radius_m`. |
| `VERTICAL_STIFFNESS` | **mapped** to 250000, from `tire.spring_stiffness_Npm`. |
| `PHY3` (0.004), `LGAX LGAY LGAZ LKYG` (1.0) | **retired.** This camber route exists only in MF5.2. It folds into `PKY6`. |
| `PPY1–PPY5` (0.0) | **omitted.** The source has no pressure model, and `dpi ≡ 0` exactly when `NOMPRES` is absent. |
| `QBZ* QCZ* QDZ* QEZ* QHZ* SSZ* QSX*` | **not in the source.** Therefore `Mz ≡ 0` and `Mx ≡ 0`. |

## Blocks that the source does not publish

The `thermal` and `wear` blocks hold **synthetic placeholders** in a racing-slick band. They come
from the slick-fixture recipe in the schema. The file labels them, and so does `provenance.source`.
Those models arrive in M5. `provenance.synthetic: false` records one fact: the force coefficients
are the published set.
