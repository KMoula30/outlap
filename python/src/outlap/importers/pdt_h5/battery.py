# SPDX-License-Identifier: AGPL-3.0-only
"""BatteryPack stage file → ``battery.yaml`` (+ ``tables.parquet``) — §10.4.

PROVISIONAL format ``battery/1.0``: there is no Rust schema for it yet (it lands in M3), so the
emitted document is validated structurally here rather than against a committed JSON Schema. The
layout is designed so M3 can adopt it verbatim.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import h5py
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from . import common as c


def convert_batterypack(
    src: Path,
    out_yaml: Path,
    *,
    tables_path: Path | None = None,
) -> dict[str, Any]:
    """Convert a BatteryPack HDF5 file to a provisional `battery.yaml` + parquet. Returns a summary."""
    tables_path = tables_path or out_yaml.with_suffix(".tables.parquet")
    with h5py.File(src, "r") as f:
        soc = c.arr(f, "vector/soc")
        temp_c = c.arr(f, "vector/temperature")
        ocv = c.arr(f, "cell/ocv_t")  # (soc, temp)
        r0 = c.arr(f, "cell/r0")
        r1 = c.arr(f, "cell/r1")
        tau1 = c.arr(f, "cell/tau1")
        dudt = c.arr(f, "cell/dudt")
        cp = c.scalar(f, "cell/cp", default=1000.0)

        ns = int(c.scalar(f, "info/ns"))
        npar = int(c.scalar(f, "info/np"))
        min_soc = c.scalar(f, "info/min_soc", default=float(soc.min()))
        max_soc = c.scalar(f, "info/max_soc", default=float(soc.max()))

        q_pack = c.scalar(f, "pack/q_pack", default=0.0)
        e_pack = c.scalar(f, "pack/e_pack", default=0.0)
        mass = c.scalar(f, "pack/mass", default=0.0)
        rth = c.scalar(f, "pack/thermal_resistance", default=0.0)
        pk_dis = c.arr(f, "pack/peak_discharge_power")
        pk_reg = c.arr(f, "pack/peak_regen_power")

        cell_name = c.str_at(f, "info/cell_name", "cell")
        chem = c.str_at(f, "info/cell_chemistry", "?")
        coolant = c.scalar(f, "info/coolant_temperature", default=25.0)
        max_c = c.scalar(f, "info/max_c_rate", default=0.0)
        vmin = c.scalar(f, "info/min_voltage", default=0.0)
        vmax = c.scalar(f, "info/max_voltage", default=0.0)
        git = c.find_git_hash(f, "BatteryPack")
        alias = c.str_at(f, "info/alias", "battery")

    doc: dict[str, Any] = {
        "schema": "battery/1.0",
        "model": "rc_pairs",
        "topology": {"ns": ns, "np": npar},
        "capacity": {"q_pack_ah": round(q_pack, 4), "e_pack_wh": round(e_pack, 3)},
        "soc_window": [round(min_soc, 4), round(max_soc, 4)],
        "ecm": {
            "rc_pairs": 1,
            "axes": {
                "soc": [round(float(s), 5) for s in soc],
                "temp_c": [round(float(t), 4) for t in temp_c],
            },
            "tables": {"file": tables_path.name, "level": "cell"},
        },
        "limits": {
            "peak_discharge_power_w_vs_soc": {
                "soc": [round(float(s), 5) for s in soc],
                "power_w": [round(float(p), 3) for p in pk_dis],
            },
            "peak_regen_power_w_vs_soc": {
                "soc": [round(float(s), 5) for s in soc],
                "power_w": [round(float(p), 3) for p in pk_reg],
            },
            "cell_v_min": round(vmin, 4),
            "cell_v_max": round(vmax, 4),
            "max_c_rate": round(max_c, 3),
        },
        "thermal": {
            "mass_kg": round(mass, 4),
            "cp_j_per_kgk": round(cp, 3),
            "thermal_resistance_k_per_w": round(rth, 5),
            "coolant_temp_c": round(coolant, 3),
        },
        "meta": {
            "source": f"PDT BatteryPack {alias} {git}",
            "cell": f"{cell_name} {chem}",
        },
    }
    validate_battery_doc(doc)

    # Long/tidy cell table on (soc, temp).
    n_soc, n_t = ocv.shape
    ss = np.repeat(soc, n_t)
    tt = np.tile(temp_c, n_soc)
    table = pa.table(
        {
            "soc": ss.astype(np.float64),
            "temp_c": tt.astype(np.float64),
            "ocv_v": ocv.reshape(-1),
            "r0_ohm": r0.reshape(-1),
            "r1_ohm": r1.reshape(-1),
            "tau1_s": tau1.reshape(-1),
            "dudt_v_per_k": dudt.reshape(-1),
        }
    )
    pq.write_table(table, tables_path)
    c.write_yaml(
        out_yaml,
        doc,
        [
            f"Imported by outlap.importers.pdt_h5 from {src.name} (§10.4)",
            "PROVISIONAL battery/1.0 — Rust schema lands in M3",
        ],
    )
    return {"out": str(out_yaml), "tables": str(tables_path), "ns": ns, "np": npar}


def validate_battery_doc(doc: dict[str, Any]) -> None:
    """Structural validation of a provisional `battery/1.0` document (no JSON Schema yet)."""
    if doc.get("schema") != "battery/1.0":
        raise c.PdtImportError("battery doc missing schema battery/1.0")
    topo = doc["topology"]
    if not (
        isinstance(topo["ns"], int)
        and topo["ns"] > 0
        and isinstance(topo["np"], int)
        and topo["np"] > 0
    ):
        raise c.PdtImportError("battery topology ns/np must be positive ints")
    lo, hi = doc["soc_window"]
    if not (0.0 <= lo < hi <= 1.0):
        raise c.PdtImportError(
            f"battery soc_window {doc['soc_window']} must be ascending within [0,1]"
        )
    soc = doc["ecm"]["axes"]["soc"]
    temp = doc["ecm"]["axes"]["temp_c"]
    if any(b <= a for a, b in zip(soc, soc[1:])):
        raise c.PdtImportError("battery soc axis must be strictly ascending")
    if any(b <= a for a, b in zip(temp, temp[1:])):
        raise c.PdtImportError("battery temp_c axis must be strictly ascending")
