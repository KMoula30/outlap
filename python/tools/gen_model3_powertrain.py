# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the SYNTHETIC Tesla Model 3 RWD (HV variant) powertrain artifacts.

Writes, under ``data/vehicles/tesla_model3_rwd/``:

* ``ptm/du_{small,medium,large}.ptm.yaml`` + ``ptm/du_{size}.maps.parquet`` — three
  Vdc-stacked (``ptm/1.1``) drive-unit maps at the OUTPUT shaft (gear ratio applied),
  the sizing-sensitivity axis of notebook 07.
* ``battery/pack_800v.battery.yaml`` + ``battery/pack_800v.tables.parquet`` — the
  synthetic 800 V-class Thevenin pack of the HV variant study.

Everything here is **synthetic** — invented smooth surfaces, NOT measured data and NOT
derived from any PDT export (firewall, CLAUDE.md hard rule #1 + M3 user decision #7).
Only headline scale figures are taken from public/format facts:

* The three peak-torque scales (1365 / 2765 / 3381 N·m at the output shaft) mirror the
  author's local drive-unit sizing sweep so the tracked notebook and its untracked
  real-data twin tell the same sensitivity story; with the shared 700 rpm base speed
  they give ≈100 / 203 / 248 kW peak — the medium sits at a production Model 3 RWD's
  ≈200 kW. All ESTIMATED.
* The pack is a 220S/1P, 64.064 kWh, 800 V-class configuration (the HV variant premise);
  its cell OCV grid spans ≈2.88–3.68 V → pack ≈634–810 V open-circuit, so under low-SoC load
  the terminal voltage sags BELOW the 730–850 V drive-unit Vdc grid and the lap exercises
  the documented below-grid linear extrapolation (M3 user decision on the Vdc policy).

Idiom notes (matches ``gen_ptm_maps.py`` / ``gen_f1_aero.py``):

* The efficiency/loss pair is emitted CONSISTENT so the energy-closure identity holds
  exactly at grid nodes: drive ``loss = P_mech·(1/η − 1)``; regen ``loss = |P_mech|·(1 − η)``;
  η is forced to 0 at the τ=0 spin point (like the PDT importers) with an idle draw.
* The efficiency is LINEAR in Vdc on purpose: the shared monotone-Hermite reproduces a
  linear axis exactly in-grid and extrapolates it exactly out-of-grid.
* Long/tidy DOUBLE columns, default pyarrow settings (SNAPPY), matching the importers.

Run from anywhere:  python python/tools/gen_model3_powertrain.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

_ROOT = Path(__file__).resolve().parents[2]
_VEH = _ROOT / "data" / "vehicles" / "tesla_model3_rwd"

RPM_TO_RAD = np.pi / 30.0

# Output-shaft speed axis, rpm. 1990 rpm ≈ 65 m/s (~235 km/h) on the 205/60R15 road tyre
# (unloaded radius 0.313 m) — the study's plausible top speed.
SPEED_RPM = np.array([10.0, 340.0, 670.0, 1000.0, 1330.0, 1660.0, 1990.0])

# Base speed (constant-torque → constant-power knee) shared by the three sizings, rpm.
BASE_SPEED_RPM = 700.0

# The three output-shaft peak-torque sizings, N·m (see module docstring).
SIZINGS = {"small": 1365.0, "medium": 2765.0, "large": 3381.0}

# Normalized torque grid (fractions of the sizing's peak; negative = regen quadrant).
TAU_FRACS = np.array([-0.6, -0.45, -0.3, -0.15, 0.0, 0.125, 0.25, 0.5, 0.75, 1.0])

# Vdc grid, V — deliberately NARROWER than the pack's terminal swing (≈634–810 V open-circuit,
# lower under load) so a low-SoC lap evaluates below the grid via linear extrapolation.
VDC_V = np.array([730.0, 790.0, 850.0])
VDC_REF = 790.0

# Per-unit scalars (ESTIMATED, Model-3-plausible): drive-unit mass incl. inverter+gearbox,
# and rotational inertia referred to the output shaft.
DU_MASS_KG = {"small": 65.0, "medium": 82.0, "large": 90.0}
DU_INERTIA_KGM2 = {"small": 1.1, "medium": 1.4, "large": 1.6}


def _eta(speed: float, tau: float, vdc: float, tau_pk: float) -> float:
    """Synthetic DU efficiency (machine+inverter+gearbox at the output shaft).

    Peaks ≈0.955 at light load / low speed, droops with load and speed, LINEAR in Vdc
    (higher bus voltage → slightly better η). 0 at the spin point, like real PDT maps.
    """
    if tau == 0.0:
        return 0.0
    base = 0.955 - 0.06 * abs(tau) / tau_pk - 2.5e-5 * speed
    return float(np.clip(base + 3.0e-4 * (vdc - VDC_REF), 0.50, 0.985))


def _consistent_loss(speed: float, tau: float, eta: float) -> float:
    """The loss (W) that closes ``source = mech + loss`` at this node for efficiency ``eta``."""
    p_mech = tau * (speed * RPM_TO_RAD)
    if tau == 0.0:
        return 120.0 + 0.4 * speed  # idle spin / inverter standby draw
    if tau > 0.0:  # drive: electrical in, mechanical out
        return p_mech * (1.0 / eta - 1.0)
    return abs(p_mech) * (1.0 - eta)  # regen: mechanical in, less recovered


def _peak_torque_curve(tau_pk: float) -> np.ndarray:
    """min(τ_pk, P_pk/ω) sampled at SPEED_RPM, with P_pk = τ_pk · ω_base."""
    p_pk = tau_pk * BASE_SPEED_RPM * RPM_TO_RAD
    omega = np.maximum(SPEED_RPM, 1.0) * RPM_TO_RAD
    return np.minimum(tau_pk, p_pk / omega)


def _fmt_list(values: np.ndarray, nd: int = 3) -> str:
    return "[" + ", ".join(f"{v:.{nd}f}" for v in values) + "]"


def _emit_du_maps(path: Path, tau_pk: float) -> None:
    torque = TAU_FRACS * tau_pk
    ns, nt, nv = SPEED_RPM.size, torque.size, VDC_V.size
    speed_c = np.repeat(SPEED_RPM, nt * nv)
    torque_c = np.tile(np.repeat(torque, nv), ns)
    vdc_c = np.tile(VDC_V, ns * nt)
    eff = np.empty(ns * nt * nv)
    loss = np.empty(ns * nt * nv)
    for k in range(eff.size):
        s, t, v = float(speed_c[k]), float(torque_c[k]), float(vdc_c[k])
        e = _eta(s, t, v, tau_pk)
        eff[k] = e
        loss[k] = _consistent_loss(s, t, e if e > 0.0 else 1.0)
    table = pa.table(
        {
            "speed_rpm": speed_c.astype(np.float64),
            "torque_nm": torque_c.astype(np.float64),
            "vdc_v": vdc_c.astype(np.float64),
            "efficiency": eff.astype(np.float64),
            "loss_w": loss.astype(np.float64),
        }
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, path)
    print(f"wrote {path} ({path.stat().st_size} bytes)")


def _emit_du_yaml(path: Path, size: str, tau_pk: float) -> None:
    torque = TAU_FRACS * tau_pk
    peak = _peak_torque_curve(tau_pk)
    p_pk_kw = tau_pk * BASE_SPEED_RPM * RPM_TO_RAD / 1e3
    axis = _fmt_list(torque)
    text = f"""\
# SPDX-License-Identifier: CC-BY-SA-4.0
# SYNTHETIC Model-3-scale drive unit ({size}: {tau_pk:.0f} N·m / ≈{p_pk_kw:.0f} kW peak at the
# output shaft) — one of the three sizing-sensitivity variants of the HV (800 V-class) study.
# NOT measured data and NOT derived from any PDT export (firewall; M3 user decision #7).
# Regenerate with: python python/tools/gen_model3_powertrain.py
schema: ptm/1.1
kind: drive_unit
axes:
  speed_rpm: {_fmt_list(SPEED_RPM)}
  load_axis:
    torque_nm: {axis}
  torque_nm: {axis}
  vdc_v: {_fmt_list(VDC_V)}
tables:
  # Sidecar next to this YAML (the PDT-importer convention).
  file: du_{size}.maps.parquet
  efficiency: true
  loss_w: true
limits:
  max_torque_nm_vs_speed:
    speed_rpm: {_fmt_list(SPEED_RPM)}
    torque_nm: {_fmt_list(peak)}
inertia_kgm2: {DU_INERTIA_KGM2[size]}
mass_kg: {DU_MASS_KG[size]}
meta:
  source: synthetic Model-3-scale drive unit (gen_model3_powertrain.py) — ESTIMATED
  dc_voltage_v: {VDC_REF}
  upstream_ratio_applied: true
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print(f"wrote {path}")


# Battery ECM cell grid (synthetic NMC-like 800 V-class cell; see module docstring).
BATT_SOC = np.array([0.05, 0.2, 0.4, 0.6, 0.8, 1.0])
BATT_TEMP_C = np.array([0.0, 25.0, 45.0])
PACK_NS, PACK_NP = 220, 1
PACK_WH = 64064.0
PACK_AH = 92.0
# Peak power limits vs SoC, W. The cap applies to ELECTRICAL source power: the large DU's
# ≈248 kW mechanical peak needs ≈283 kW electrical, so the pack trims the large sizing's peak
# at every SoC (and bites harder as it depletes) — part of the diminishing-returns story.
LIM_SOC = np.array([0.05, 0.2, 0.4, 0.6, 0.8, 1.0])
LIM_DISCHARGE_W = np.array([70e3, 160e3, 230e3, 255e3, 265e3, 265e3])
LIM_REGEN_W = np.array([190e3, 190e3, 170e3, 140e3, 85e3, 30e3])


def _emit_pack_tables(path: Path) -> None:
    ns, nt = BATT_SOC.size, BATT_TEMP_C.size
    soc = np.repeat(BATT_SOC, nt)
    temp = np.tile(BATT_TEMP_C, ns)
    ocv = 2.85 + 0.82 * soc - 4.0e-4 * (temp - 25.0)
    r0 = 8.5e-4 + 6.5e-4 * (1.0 - soc)
    r1 = 4.0e-4 + 3.0e-4 * (1.0 - soc)
    tau1 = np.full(soc.size, 22.0)
    dudt = np.full(soc.size, -1.0e-4)
    table = pa.table(
        {
            "soc": soc.astype(np.float64),
            "temp_c": temp.astype(np.float64),
            "ocv_v": ocv.astype(np.float64),
            "r0_ohm": r0.astype(np.float64),
            "r1_ohm": r1.astype(np.float64),
            "tau1_s": tau1.astype(np.float64),
            "dudt_v_per_k": dudt.astype(np.float64),
        }
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, path)
    print(f"wrote {path} ({path.stat().st_size} bytes)")


def _emit_pack_yaml(path: Path) -> None:
    text = f"""\
# SPDX-License-Identifier: CC-BY-SA-4.0
# SYNTHETIC 800 V-class Thevenin pack for the Model 3 HV variant study. NOT real PDT data;
# invented smooth cell curves (firewall; M3 user decision #7). Cell OCV grid ≈2.88–3.68 V →
# pack ≈634–810 V open-circuit: under low-SoC load the terminal voltage sags BELOW the drive units'
# 730–850 V Vdc grid, exercising the documented below-grid linear extrapolation.
# Regenerate the tables with: python python/tools/gen_model3_powertrain.py
schema: battery/1.0
model: rc_pairs
topology:
  ns: {PACK_NS}
  np: {PACK_NP}
capacity:
  q_pack_ah: {PACK_AH}
  e_pack_wh: {PACK_WH}
soc_window: [0.05, 0.98]
ecm:
  rc_pairs: 1
  axes:
    soc: {_fmt_list(BATT_SOC, 2)}
    temp_c: {_fmt_list(BATT_TEMP_C)}
  tables:
    # Sidecar next to this YAML (the PDT-importer convention).
    file: pack_800v.tables.parquet
    level: cell
limits:
  peak_discharge_power_w_vs_soc:
    soc: {_fmt_list(LIM_SOC, 2)}
    power_w: {_fmt_list(LIM_DISCHARGE_W)}
  peak_regen_power_w_vs_soc:
    soc: {_fmt_list(LIM_SOC, 2)}
    power_w: {_fmt_list(LIM_REGEN_W)}
  cell_v_min: 2.7
  cell_v_max: 4.2
  max_c_rate: 4.5
thermal:
  mass_kg: 460.0
  cp_j_per_kgk: 900.0
  thermal_resistance_k_per_w: 0.02
  coolant_temp_c: 25.0
meta:
  source: synthetic 800 V-class pack (gen_model3_powertrain.py) — ESTIMATED
  cell: Generic_NMC_800V_class
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print(f"wrote {path}")


def main() -> None:
    for size, tau_pk in SIZINGS.items():
        _emit_du_maps(_VEH / "ptm" / f"du_{size}.maps.parquet", tau_pk)
        _emit_du_yaml(_VEH / "ptm" / f"du_{size}.ptm.yaml", size, tau_pk)
    _emit_pack_tables(_VEH / "battery" / "pack_800v.tables.parquet")
    _emit_pack_yaml(_VEH / "battery" / "pack_800v.battery.yaml")


if __name__ == "__main__":
    main()
