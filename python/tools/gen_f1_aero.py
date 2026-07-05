# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the synthetic ride-height / yaw aero map for the reference F1 2026 car (§7.4).

Writes ``data/vehicles/f1_2026/aero/f1_2026.parquet`` — a long/tidy table with the columns the
outlap parquet sidecar reader expects (all ``DOUBLE``, default pyarrow settings = SNAPPY + PLAIN /
RLE_DICTIONARY encodings, matching the PDT importers)::

    ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag,   # grid axes (in the vehicle.yaml order)
    cz_front_a_m2, cz_rear_a_m2, cx_a_m2                      # value columns (coefficient × area, m²)

The map is **synthetic** — NOT measured data and NOT derived from any simulator. It is *anchored*
(user decision #4, M3) so that at the reference (equilibrium) ride heights the coefficients equal
the ``f1_2026`` constant-aero fallback (``cz_front_a_m2 = 1.9``, ``cz_rear_a_m2 = 2.6``,
``cx_a_m2 = 1.25``; total ClA 4.5 m², L/D ≈ 3.6). Those constant values stand in for the
Perantoni & Limebeer 2014 reference aero; PR9 reconciles them against the published PL2014 figures
and records the citation, keeping the ≤1 % Limebeer gate honest. Every modelling assumption is
listed below and marked estimated.

Physical assumptions (all ESTIMATED, documented per hard-rule "estimated values surface"):
  * ground effect: front/rear downforce rise as the respective ride height drops (linear in the
    normalised offset from the reference height), with a mild cross-axle rake coupling;
  * drag rises slightly as the platform lowers (more ground-effect load ⇒ more induced drag);
  * yaw: an even (symmetric) sensitivity — downforce falls and drag rises with |yaw| (a symmetric
    car has no left/right bias, so the g-g stays L/R-symmetric but shrinks off-centre);
  * DRS (rear-wing open, ``drs_flag = 1``): rear downforce −30 %, drag −18 %, front unchanged.

The functional form is affine in each ride-height axis and even-quadratic in yaw, so every
grid-aligned fibre is monotone (or single-peaked at yaw = 0) — safe for the shared monotone-cubic
interpolant (Decision #30). Run from anywhere:  ``python python/tools/gen_f1_aero.py``.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

# --- Grid axes (vehicle.yaml order: ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag) ---
RIDE_F_MM = np.array([10.0, 20.0, 30.0, 40.0, 60.0])
RIDE_R_MM = np.array([30.0, 50.0, 70.0, 100.0, 140.0])
YAW_DEG = np.array([-8.0, -4.0, 0.0, 4.0, 8.0])
DRS_FLAG = np.array([0.0, 1.0])

# --- Reference (anchor) point: the coefficients equal the f1_2026 constant-aero fallback here ---
REF_HF_MM = 30.0
REF_HR_MM = 70.0
CZ_FRONT_REF = 1.9
CZ_REAR_REF = 2.6
CX_REF = 1.25

# --- Sensitivity coefficients (ESTIMATED) ---
A_FF = 0.35  # front DF per (front ride-height drop), normalised
A_FR = 0.10  # front DF per (rear ride-height rise) — rake
A_RR = 0.30  # rear DF per (rear ride-height drop)
A_RF = 0.05  # rear DF per (front ride-height drop) — coupling
A_XF = 0.05  # drag per (front ride-height drop)
A_XR = 0.05  # drag per (rear ride-height drop)
C_YAW_DF = 0.08  # downforce loss at |yaw| = 10 deg
C_YAW_DRAG = 0.15  # drag rise at |yaw| = 10 deg
DRS_REAR_DF = 0.70  # rear DF multiplier with DRS open
DRS_DRAG = 0.82  # drag multiplier with DRS open


def _coeffs(hf: float, hr: float, yaw: float, drs: float) -> tuple[float, float, float]:
    """Return ``(cz_front_a_m2, cz_rear_a_m2, cx_a_m2)`` for one grid node."""
    df = (REF_HF_MM - hf) / REF_HF_MM  # >0 when lower than reference (more front DF)
    dr = (REF_HR_MM - hr) / REF_HR_MM  # >0 when lower than reference (more rear DF)
    rake = (hr - REF_HR_MM) / REF_HR_MM  # >0 with more rearward rake
    yaw_df = 1.0 - C_YAW_DF * (yaw / 10.0) ** 2
    yaw_drag = 1.0 + C_YAW_DRAG * (yaw / 10.0) ** 2
    drs_rear = DRS_REAR_DF if drs > 0.5 else 1.0
    drs_drag = DRS_DRAG if drs > 0.5 else 1.0

    cz_front = CZ_FRONT_REF * (1.0 + A_FF * df + A_FR * rake) * yaw_df
    cz_rear = CZ_REAR_REF * (1.0 + A_RR * dr + A_RF * df) * yaw_df * drs_rear
    cx = CX_REF * (1.0 + A_XF * df + A_XR * dr) * yaw_drag * drs_drag
    return cz_front, cz_rear, cx


def main() -> None:
    rows_hf, rows_hr, rows_yaw, rows_drs = [], [], [], []
    czf, czr, cx = [], [], []
    for hf in RIDE_F_MM:
        for hr in RIDE_R_MM:
            for yaw in YAW_DEG:
                for drs in DRS_FLAG:
                    a, b, c = _coeffs(hf, hr, yaw, drs)
                    rows_hf.append(hf)
                    rows_hr.append(hr)
                    rows_yaw.append(yaw)
                    rows_drs.append(drs)
                    czf.append(a)
                    czr.append(b)
                    cx.append(c)

    table = pa.table(
        {
            "ride_height_f_mm": np.asarray(rows_hf, dtype=np.float64),
            "ride_height_r_mm": np.asarray(rows_hr, dtype=np.float64),
            "yaw_deg": np.asarray(rows_yaw, dtype=np.float64),
            "drs_flag": np.asarray(rows_drs, dtype=np.float64),
            "cz_front_a_m2": np.asarray(czf, dtype=np.float64),
            "cz_rear_a_m2": np.asarray(czr, dtype=np.float64),
            "cx_a_m2": np.asarray(cx, dtype=np.float64),
        }
    )
    # Anchor sanity: the reference node reproduces the constant-aero fallback exactly.
    ref = _coeffs(REF_HF_MM, REF_HR_MM, 0.0, 0.0)
    assert np.allclose(ref, (CZ_FRONT_REF, CZ_REAR_REF, CX_REF)), ref

    root = Path(__file__).resolve().parents[2]
    out = root / "data" / "vehicles" / "f1_2026" / "aero" / "f1_2026.parquet"
    out.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, out)
    print(f"wrote {out} ({out.stat().st_size} bytes, {table.num_rows} rows)")


if __name__ == "__main__":
    main()
