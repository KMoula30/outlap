# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the SYNTHETIC M6 energy-store packs (f1_2026 ES + the gt_hybrid fixture ES).

Writes:

* ``data/vehicles/f1_2026/battery/f1_es.yaml`` + ``f1_es.tables.parquet`` — the
  2026-style F1 energy store. **No public team data exists**; every curve is an
  invented smooth surface (firewall, CLAUDE.md hard rule #1). Only the SIZING is
  regulation-derived (D-M6-3): the FIA 2026 C5.2.9 usable window is max−min SoC
  ≤ 4 MJ *on track*, and the vehicle's ``ers.es`` declares ``capacity_mj: 4.0``
  over ``soc_window: [0.2, 0.9]`` — so the pack TOTAL is 4.0 / 0.7 ≈ 5.7143 MJ
  (``e_pack_wh`` ≈ 1587.30) and the battery document's own ``soc_window`` matches
  the ers block (the load pipeline cross-checks both).
* ``crates/outlap-schema/tests/fixtures/battery/gt_es.yaml`` + ``gt_es.tables.parquet``
  — the gt_hybrid fixture's pack (D-M6-12): 2.0 MJ usable over [0.3, 0.85] ⇒
  ≈3.6364 MJ total (``e_pack_wh`` ≈ 1010.10). Deliberately declares NO
  ``regen_derate_vs_temp`` so the absent-Option charge-acceptance path (and its
  estimation note) stays exercised by the fixture chain.

Design intents baked into the f1 curves:

* Discharge/regen design curves sit ABOVE the FIA electrical caps (350 kW both ways,
  C5.2.7) away from the window edges — the REGULATION binds, not the pack — while the
  window-edge hard cuts still produce honest SoC starvation / full-store refusal.
* The pack runs on the car's radiator loop: ``coolant_temp_c: 45`` (a hot-side sink is
  car data, not weather — resolved M6 default), and ``regen_derate_vs_temp`` reaches
  1.0 at that temperature, so charge acceptance is design-curve-limited in normal
  running and kinetically derated only for a cold pack.
* Thermal: 35 kg × 900 J/(kg·K) with R_th = 0.002 K/W → τ ≈ 63 s and ≈8 K steady rise
  per 4 kW mean dissipation — an aggressively cooled race ES.

Idiom notes (matches ``gen_model3_powertrain.py``): long/tidy DOUBLE parquet columns
(``soc, temp_c, ocv_v, r0_ohm, r1_ohm, tau1_s, dudt_v_per_k``), default pyarrow
settings, YAML emitted beside its sidecar (the PDT-importer convention).

Run from anywhere:  python python/tools/gen_f1_es_pack.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

_ROOT = Path(__file__).resolve().parents[2]
_F1_DIR = _ROOT / "data" / "vehicles" / "f1_2026" / "battery"
_GT_DIR = _ROOT / "crates" / "outlap-schema" / "tests" / "fixtures" / "battery"


def _fmt_list(values: np.ndarray, nd: int = 3) -> str:
    return "[" + ", ".join(f"{v:.{nd}f}" for v in values) + "]"


# --- f1_es: the 2026-style energy store ------------------------------------------------

# 4.0 MJ usable over [0.2, 0.9] (D-M6-3) ⇒ total = 4.0/0.7 MJ; Wh = J / 3600.
F1_E_TOTAL_J = 4.0e6 / 0.7
F1_E_PACK_WH = F1_E_TOTAL_J / 3600.0  # ≈ 1587.3016
F1_NS, F1_NP = 180, 1
# Mean cell OCV over the grid at 45 °C is 3.675 V (curve below) → pack mean ≈ 661.5 V;
# q = E / (V̄·3600) ≈ 2.40 A·h (the implied energy is within 0.02 % of e_pack_wh).
F1_Q_AH = 2.40

F1_SOC = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
F1_TEMP_C = np.array([20.0, 45.0, 60.0])

F1_LIM_SOC = np.array([0.2, 0.3, 0.5, 0.7, 0.9, 1.0])
# Discharge design curve: above the 350 kW FIA cap except right at the window floor.
F1_DISCHARGE_W = np.array([250e3, 450e3, 600e3, 650e3, 650e3, 650e3])
# Regen design curve: above the cap through the window, tapering toward the top.
F1_REGEN_W = np.array([520e3, 510e3, 490e3, 460e3, 300e3, 0.0])

F1_DERATE_T = np.array([0.0, 20.0, 45.0, 60.0])
F1_DERATE_F = np.array([0.30, 0.85, 1.00, 1.00])


def _emit_f1_tables(path: Path) -> None:
    ns, nt = F1_SOC.size, F1_TEMP_C.size
    soc = np.repeat(F1_SOC, nt)
    temp = np.tile(F1_TEMP_C, ns)
    # Invented smooth power-cell curves; OCV linear in SoC with a mild thermal slope.
    ocv = 3.30 + 0.75 * soc - 2.0e-4 * (temp - 45.0)
    r0 = 2.0e-4 + 1.0e-4 * (1.0 - soc) + 4.0e-6 * (45.0 - temp)
    r1 = 8.0e-5 + 4.0e-5 * (1.0 - soc)
    tau1 = np.full(soc.size, 8.0)
    dudt = np.full(soc.size, -5.0e-5)
    # 2nd RC pair (battery/1.2) — a second, SLOWER relaxation branch. A real multi-timescale
    # Thevenin fit (as in the model3_awd 800 V-class study this ERS is scaled from) separates a
    # fast charge-transfer arc from a slow diffusion arc; here the slow branch is ~40 % of R1 at
    # ~5.5× the time constant. Synthetic and approximate — the regulation caps still bind the pack.
    r2 = 0.4 * r1
    tau2 = np.full(soc.size, 45.0)
    _write_tables(path, soc, temp, ocv, r0, r1, tau1, dudt, r2=r2, tau2=tau2)


def _emit_f1_yaml(path: Path) -> None:
    text = f"""\
# SPDX-License-Identifier: CC-BY-SA-4.0
# SYNTHETIC 2026-style F1 energy store. No public team data exists — every curve is an
# invented smooth surface (firewall). SIZING is regulation-derived (D-M6-3): the vehicle's
# ers.es declares 4.0 MJ usable over soc_window [0.2, 0.9] (FIA 2026 C5.2.9: max−min SoC
# ≤ 4 MJ on track), so the pack total is 4.0/0.7 ≈ 5.7143 MJ; the load pipeline cross-checks
# this file against the ers block. Discharge/regen design curves sit above the FIA 350 kW
# electrical caps away from the window edges — the regulation binds, not the pack.
# coolant_temp_c is the car's radiator-loop hot side (car data, not weather): 45 °C, where
# the kinetic charge-acceptance derate saturates at 1.0.
# The ECM is a 2-RC-pair (battery/1.2) Thevenin fit — a fast charge-transfer branch (tau1) plus
# a slow diffusion branch (tau2), the multi-timescale structure of the model3_awd 800 V-class pack
# study this ES is scaled down from; both branches are synthetic (firewall) and approximate.
# Regenerate the tables with: python python/tools/gen_f1_es_pack.py
schema: battery/1.2
model: rc_pairs
topology:
  ns: {F1_NS}
  np: {F1_NP}
capacity:
  q_pack_ah: {F1_Q_AH}
  e_pack_wh: {F1_E_PACK_WH:.4f}
soc_window: [0.2, 0.9]
ecm:
  rc_pairs: 2
  axes:
    soc: {_fmt_list(F1_SOC, 2)}
    temp_c: {_fmt_list(F1_TEMP_C, 1)}
  tables:
    # Sidecar next to this YAML (the PDT-importer convention).
    file: f1_es.tables.parquet
    level: cell
limits:
  peak_discharge_power_w_vs_soc:
    soc: {_fmt_list(F1_LIM_SOC, 2)}
    power_w: {_fmt_list(F1_DISCHARGE_W, 1)}
  peak_regen_power_w_vs_soc:
    soc: {_fmt_list(F1_LIM_SOC, 2)}
    power_w: {_fmt_list(F1_REGEN_W, 1)}
  regen_derate_vs_temp:
    temp_c: {_fmt_list(F1_DERATE_T, 1)}
    factor: {_fmt_list(F1_DERATE_F, 2)}
  cell_v_min: 2.9
  cell_v_max: 4.25
  # Informational: 350 kW / (661.5 V × 2.40 A·h) — an ultra-high-power race cell.
  max_c_rate: 220.0
thermal:
  mass_kg: 35.0
  cp_j_per_kgk: 900.0
  thermal_resistance_k_per_w: 0.002
  coolant_temp_c: 45.0
meta:
  source: synthetic 2026-style F1 ES (gen_f1_es_pack.py) — SYNTHETIC, regulation-sized
  cell: Generic_F1_power_cell
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print(f"wrote {path}")


# --- gt_es: the gt_hybrid fixture pack --------------------------------------------------

# 2.0 MJ usable over [0.3, 0.85] (the fixture's ers.es) ⇒ total = 2.0/0.55 MJ.
GT_E_TOTAL_J = 2.0e6 / 0.55
GT_E_PACK_WH = GT_E_TOTAL_J / 3600.0  # ≈ 1010.101
GT_NS, GT_NP = 96, 1
# Mean cell OCV 3.70 V → pack mean ≈ 355.2 V; q = E / (V̄·3600) ≈ 2.84 A·h.
GT_Q_AH = 2.84

GT_SOC = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
GT_TEMP_C = np.array([10.0, 30.0, 50.0])

GT_LIM_SOC = np.array([0.3, 0.5, 0.75, 0.9, 1.0])
GT_DISCHARGE_W = np.array([120e3, 200e3, 220e3, 220e3, 220e3])
GT_REGEN_W = np.array([200e3, 190e3, 160e3, 80e3, 0.0])


def _emit_gt_tables(path: Path) -> None:
    ns, nt = GT_SOC.size, GT_TEMP_C.size
    soc = np.repeat(GT_SOC, nt)
    temp = np.tile(GT_TEMP_C, ns)
    ocv = 3.35 + 0.70 * soc - 3.0e-4 * (temp - 30.0)
    r0 = 6.0e-4 + 3.0e-4 * (1.0 - soc)
    r1 = 2.5e-4 + 1.5e-4 * (1.0 - soc)
    tau1 = np.full(soc.size, 15.0)
    dudt = np.full(soc.size, -8.0e-5)
    _write_tables(path, soc, temp, ocv, r0, r1, tau1, dudt)


def _emit_gt_yaml(path: Path) -> None:
    text = f"""\
# SPDX-License-Identifier: CC-BY-SA-4.0
# SYNTHETIC gt_hybrid fixture energy store (D-M6-12). Sized so the fixture's ers.es
# (2.0 MJ usable over soc_window [0.3, 0.85]) cross-checks: total = 2.0/0.55 ≈ 3.6364 MJ.
# Deliberately declares NO regen_derate_vs_temp — the absent-Option charge-acceptance
# path (and its estimation note) stays fixture-exercised.
# Regenerate the tables with: python python/tools/gen_f1_es_pack.py
schema: battery/1.0
model: rc_pairs
topology:
  ns: {GT_NS}
  np: {GT_NP}
capacity:
  q_pack_ah: {GT_Q_AH}
  e_pack_wh: {GT_E_PACK_WH:.4f}
soc_window: [0.3, 0.85]
ecm:
  rc_pairs: 1
  axes:
    soc: {_fmt_list(GT_SOC, 2)}
    temp_c: {_fmt_list(GT_TEMP_C, 1)}
  tables:
    file: gt_es.tables.parquet
    level: cell
limits:
  peak_discharge_power_w_vs_soc:
    soc: {_fmt_list(GT_LIM_SOC, 2)}
    power_w: {_fmt_list(GT_DISCHARGE_W, 1)}
  peak_regen_power_w_vs_soc:
    soc: {_fmt_list(GT_LIM_SOC, 2)}
    power_w: {_fmt_list(GT_REGEN_W, 1)}
  cell_v_min: 2.8
  cell_v_max: 4.2
  max_c_rate: 70.0
thermal:
  mass_kg: 60.0
  cp_j_per_kgk: 900.0
  thermal_resistance_k_per_w: 0.01
  coolant_temp_c: 30.0
meta:
  source: synthetic GT hybrid ES (gen_f1_es_pack.py) — SYNTHETIC
  cell: Generic_GT_power_cell
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print(f"wrote {path}")


def _write_tables(
    path: Path,
    soc: np.ndarray,
    temp: np.ndarray,
    ocv: np.ndarray,
    r0: np.ndarray,
    r1: np.ndarray,
    tau1: np.ndarray,
    dudt: np.ndarray,
    r2: np.ndarray | None = None,
    tau2: np.ndarray | None = None,
) -> None:
    columns = {
        "soc": soc.astype(np.float64),
        "temp_c": temp.astype(np.float64),
        "ocv_v": ocv.astype(np.float64),
        "r0_ohm": r0.astype(np.float64),
        "r1_ohm": r1.astype(np.float64),
        "tau1_s": tau1.astype(np.float64),
        "dudt_v_per_k": dudt.astype(np.float64),
    }
    # The 2nd RC pair columns (battery/1.2) are additive: present only for a 2-pair pack.
    if r2 is not None and tau2 is not None:
        columns["r2_ohm"] = r2.astype(np.float64)
        columns["tau2_s"] = tau2.astype(np.float64)
    table = pa.table(columns)
    path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, path)
    print(f"wrote {path} ({path.stat().st_size} bytes)")


def main() -> None:
    _emit_f1_tables(_F1_DIR / "f1_es.tables.parquet")
    _emit_f1_yaml(_F1_DIR / "f1_es.yaml")
    # The schema-fixture twin of the f1_2026 vehicle references the same battery path.
    _emit_f1_tables(_GT_DIR / "f1_es.tables.parquet")
    _emit_f1_yaml(_GT_DIR / "f1_es.yaml")
    _emit_gt_tables(_GT_DIR / "gt_es.tables.parquet")
    _emit_gt_yaml(_GT_DIR / "gt_es.yaml")


if __name__ == "__main__":
    main()
