# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the synthetic `.ptm` efficiency/loss parquet sidecars used by the T1 powertrain tests.

Writes two long/tidy tables (``speed_rpm, torque_nm, efficiency, loss_w`` as ``DOUBLE`` columns,
default pyarrow settings = SNAPPY + PLAIN/RLE_DICTIONARY) shaped **exactly** like the PDT
drive-unit / ICE importer output (`outlap.importers.pdt_h5.common.write_maps_parquet`):

* ``d.ptm.maps.parquet``  — companion of ``ptm/pdt_synth_du.ptm.yaml`` (a lumped drive unit).
  Used for the PDT round-trip gate (§10.5/§13: reproduce spot efficiencies to 1e-6 through the Rust
  `GriddedMapN`) and the energy-closure property test.
* ``tables/ice_v6.parquet`` — companion of ``ptm/ice_v6.ptm.yaml`` (a 1.6 L V6). Its ``efficiency``
  column is BRAKE THERMAL efficiency, so ICE fuel-mass accounting can be exercised.

The efficiency/loss pair is emitted CONSISTENT so the energy-closure identity holds exactly at the
grid nodes: for drive (τ>0) ``loss = P_mech·(1/η − 1)``; for regen (τ<0) ``loss = |P_mech|·(1 − η)``;
at the τ=0 spin point η is forced to 0 (matching the importer) and the loss is an idle draw.
``P_mech = τ · ω`` with ``ω = speed_rpm · π/30``. Synthetic only — never derived from PDT data.

Run from anywhere:  python gen_ptm_maps.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

RPM_TO_RAD = np.pi / 30.0

# Drive-unit map — axes match ptm/pdt_synth_du.ptm.yaml exactly.
DU_SPEED = np.array([10.0, 81.8, 153.6, 225.4, 297.2, 369.0])
DU_TORQUE = np.array([-150.0, -112.5, -75.0, -37.5, 0.0, 42.0, 84.0, 126.0, 168.0])
DU_TAU_PEAK = 168.0

# Vdc-stacked drive-unit map — axes match ptm/pdt_synth_du_vdc.ptm.yaml exactly (PR6). The Vdc grid
# is 730–850 V; the real pack swings ~620–808 V, so a low/mid-SoC terminal voltage sits BELOW the
# grid and the Vdc axis extrapolates linearly. The efficiency is LINEAR in vdc so the Rust
# monotone-Hermite reproduces it exactly in-grid and extrapolates it exactly linearly out-of-grid —
# the property test asserts both.
DU_VDC = np.array([730.0, 790.0, 850.0])
DU_VDC_REF = 790.0

# ICE map — axes match ptm/ice_v6.ptm.yaml exactly.
ICE_SPEED = np.array([1000.0, 4000.0, 8000.0, 12000.0, 15000.0])
ICE_TORQUE = np.array([0.0, 100.0, 200.0, 300.0, 400.0])
ICE_TAU_PEAK = 400.0


def _consistent_loss(speed: float, tau: float, eta: float) -> float:
    """The loss (W) that closes ``source = mech + loss`` at this node for efficiency ``eta``."""
    p_mech = tau * (speed * RPM_TO_RAD)
    if tau == 0.0:
        return 40.0 + 0.2 * speed  # idle spin / pumping draw
    if tau > 0.0:  # drive: electrical/fuel in, mechanical out
        return p_mech * (1.0 / eta - 1.0)
    return abs(p_mech) * (1.0 - eta)  # regen: mechanical in, less recovered


def _drive_unit_eta(speed: float, tau: float) -> float:
    """Machine+inverter+gearbox efficiency: peaks near mid load, mild speed droop; 0 at the spin point."""
    if tau == 0.0:
        return 0.0
    return float(np.clip(0.95 - 0.10 * abs(tau) / DU_TAU_PEAK - 5.0e-5 * speed, 0.30, 0.97))


def _ice_eta(speed: float, tau: float) -> float:
    """Brake thermal efficiency: rises with load, best near mid speed; 0 at the (idling) spin point."""
    if tau == 0.0:
        return 0.0
    load = tau / ICE_TAU_PEAK
    speed_pen = 6.0e-6 * abs(speed - 8000.0)
    return float(np.clip(0.14 + 0.24 * load - speed_pen, 0.10, 0.38))


def _emit(path: Path, speed_axis, torque_axis, eta_fn) -> None:
    ns, nt = speed_axis.size, torque_axis.size
    speed = np.repeat(speed_axis, nt)  # speed-major, one row per (speed, torque) cell
    torque = np.tile(torque_axis, ns)
    eff = np.empty(ns * nt)
    loss = np.empty(ns * nt)
    for k in range(ns * nt):
        s, t = float(speed[k]), float(torque[k])
        e = eta_fn(s, t)
        eff[k] = e
        loss[k] = _consistent_loss(s, t, e if e > 0.0 else 1.0)
    table = pa.table(
        {
            "speed_rpm": speed.astype(np.float64),
            "torque_nm": torque.astype(np.float64),
            "efficiency": eff.astype(np.float64),
            "loss_w": loss.astype(np.float64),
        }
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, path)
    print(f"wrote {path} ({path.stat().st_size} bytes)")


def _drive_unit_eta_vdc(speed: float, tau: float, vdc: float) -> float:
    """Drive-unit efficiency with a LINEAR Vdc dependence (higher bus voltage → slightly better η).

    Linear in vdc on purpose: the shared monotone-Hermite reproduces a linear axis exactly and its
    linear out-of-domain mode extrapolates it exactly, so the round-trip and the below-grid
    extrapolation are both checkable to ~1e-9.
    """
    if tau == 0.0:
        return 0.0
    base = 0.95 - 0.10 * abs(tau) / DU_TAU_PEAK - 5.0e-5 * speed
    return float(np.clip(base + 3.0e-4 * (vdc - DU_VDC_REF), 0.30, 0.985))


def _emit_vdc(path: Path, speed_axis, torque_axis, vdc_axis, eta_fn) -> None:
    """Emit a 3-D (speed, torque, vdc) long/tidy table with a `vdc_v` axis column (ptm/1.1)."""
    ns, nt, nvdc = speed_axis.size, torque_axis.size, vdc_axis.size
    speed = np.repeat(speed_axis, nt * nvdc)
    torque = np.tile(np.repeat(torque_axis, nvdc), ns)
    vdc = np.tile(vdc_axis, ns * nt)
    eff = np.empty(ns * nt * nvdc)
    loss = np.empty(ns * nt * nvdc)
    for k in range(eff.size):
        s, t, v = float(speed[k]), float(torque[k]), float(vdc[k])
        e = eta_fn(s, t, v)
        eff[k] = e
        loss[k] = _consistent_loss(s, t, e if e > 0.0 else 1.0)
    table = pa.table(
        {
            "speed_rpm": speed.astype(np.float64),
            "torque_nm": torque.astype(np.float64),
            "vdc_v": vdc.astype(np.float64),
            "efficiency": eff.astype(np.float64),
            "loss_w": loss.astype(np.float64),
        }
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, path)
    print(f"wrote {path} ({path.stat().st_size} bytes)")


# Battery ECM cell table — companion of battery/synth_pack.battery.yaml (PR6). A 220S1P pack: cell
# OCV 2.85–3.67 V → pack 627–807 V, so under load the terminal drops into/below the DU Vdc grid.
BATT_SOC = np.array([0.05, 0.2, 0.4, 0.6, 0.8, 1.0])
BATT_TEMP_C = np.array([0.0, 25.0, 45.0])


def _emit_battery(path: Path) -> None:
    ns, nt = BATT_SOC.size, BATT_TEMP_C.size
    soc = np.repeat(BATT_SOC, nt)
    temp = np.tile(BATT_TEMP_C, ns)
    ocv = 2.85 + 0.82 * soc - 4.0e-4 * (temp - 25.0)
    r0 = 9.0e-4 + 6.0e-4 * (1.0 - soc)
    r1 = 4.0e-4 + 3.0e-4 * (1.0 - soc)
    tau1 = np.full(soc.size, 20.0)
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


def main() -> None:
    here = Path(__file__).parent
    _emit(here / "d.ptm.maps.parquet", DU_SPEED, DU_TORQUE, _drive_unit_eta)
    _emit(here / "tables" / "ice_v6.parquet", ICE_SPEED, ICE_TORQUE, _ice_eta)
    _emit_vdc(
        here / "pdt_synth_du_vdc.maps.parquet",
        DU_SPEED,
        DU_TORQUE,
        DU_VDC,
        _drive_unit_eta_vdc,
    )
    _emit_battery(here / "battery" / "synth_pack.tables.parquet")


if __name__ == "__main__":
    main()
