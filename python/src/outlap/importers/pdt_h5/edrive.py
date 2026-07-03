# SPDX-License-Identifier: AGPL-3.0-only
"""EDrive stage file → ``machine.ptm.yaml`` (+ ``maps.parquet``), kind ``electric_machine`` (§10.2).

The system (machine + inverter) efficiency is ``motor_efficiency · inverter_efficiency`` and the
system loss is ``motor_loss_total + inverter_loss_total`` — the real files carry the two stages
separately, not a lumped ``system_efficiency``. The torque coordinate is ``airgap_torque``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import h5py
import numpy as np

from . import common as c


def convert_edrive(
    src: Path,
    out_yaml: Path,
    *,
    vdc: float | None = None,
    torque_points: int = 101,
    maps_path: Path | None = None,
) -> dict[str, Any]:
    """Convert an EDrive HDF5 file to a `.ptm` document + parquet sidecar. Returns a summary."""
    maps_path = maps_path or out_yaml.with_suffix(".maps.parquet")
    with h5py.File(src, "r") as f:
        speed_rpm = c.arr(f, "sweep/speed")
        vdc_grid = c.arr(f, "sweep/vdc")
        vdc_used = c.opt_arr(f, "peak_capability/thermal/continuous/vdc_used")
        choice = c.select_vdc(
            vdc_grid, vdc, None if vdc_used is None else float(vdc_used.reshape(-1)[0])
        )
        iv = choice.index

        # Operating grid at the chosen vdc: torque + system efficiency/loss.
        tau = c.arr(f, "operating_grid/airgap_torque")[iv]  # (speed, load)
        mot_eff = c.arr(f, "operating_grid/motor_efficiency")[iv]
        inv_eff = c.arr(f, "operating_grid/inverter_efficiency")[iv]
        mot_loss = c.arr(f, "operating_grid/motor_loss_total")[iv]
        inv_loss = c.arr(f, "operating_grid/inverter_loss_total")[iv]
        sys_eff = mot_eff * inv_eff
        sys_loss = mot_loss + inv_loss

        torque_drive = c.arr(f, "peak_capability/torque_drive")[iv]  # (speed,)
        torque_regen = c.arr(f, "peak_capability/torque_regen")[iv]
        drag = c.opt_arr(f, "peak_capability/torque_drag")

        regrid = c.regrid_map(
            speed_rpm,
            tau,
            sys_eff,
            sys_loss,
            torque_drive,
            torque_regen,
            choice,
            torque_points,
        )

        # Thermal envelopes (continuous + overload) — optional.
        cont = c.opt_arr(f, "peak_capability/thermal/continuous/torque")
        peak = c.opt_arr(f, "peak_capability/thermal/peak/torque")
        durations = c.opt_arr(f, "peak_capability/thermal/peak/durations")

        inertia = c.scalar(f, "inertia/rotor_inertia")
        mass = c.scalar(
            f,
            "mass/drive_total_mass",
            default=c.scalar(f, "mass/motor_total_mass", 0.0),
        )
        if mass <= 0.0:
            raise c.PdtImportError(
                "could not resolve a positive mass_kg (need mass/drive_total_mass)"
            )
        alias = c.str_at(f, "info/alias", "edrive")
        git = c.find_git_hash(f, "EDrive")

    limits: dict[str, Any] = {
        "max_torque_nm_vs_speed": c.torque_curve(speed_rpm, torque_drive),
    }
    if cont is not None:
        limits["cont_torque_nm_vs_speed"] = c.torque_curve(speed_rpm, cont.reshape(-1))
    if peak is not None and durations is not None:
        peak2 = peak.reshape(peak.shape[-2], peak.shape[-1])  # (speed, n_durations)
        limits["overload"] = {
            "durations_s": [round(float(d), 3) for d in durations.reshape(-1)],
            "torque_nm_vs_speed": [
                c.torque_curve(speed_rpm, peak2[:, k]) for k in range(peak2.shape[1])
            ],
        }
    if drag is not None:
        limits["drag_torque_nm_vs_speed"] = c.torque_curve(speed_rpm, drag)

    doc: dict[str, Any] = {
        "schema": "ptm/1.0",
        "kind": "electric_machine",
        "axes": {
            "speed_rpm": [round(float(s), 4) for s in speed_rpm],
            "load_axis": {"torque_nm": [round(float(t), 4) for t in regrid.torque_nm]},
            "torque_nm": [round(float(t), 4) for t in regrid.torque_nm],
        },
        "tables": {"file": maps_path.name, "efficiency": True, "loss_w": True},
        "limits": limits,
        "inertia_kgm2": round(inertia, 6),
        "mass_kg": round(mass, 4),
        "meta": {"source": f"PDT EDrive {alias} {git}", "dc_voltage_v": choice.value},
    }

    c.validate_against_schema(doc, "ptm")
    c.write_maps_parquet(maps_path, regrid)
    c.write_yaml(
        out_yaml,
        doc,
        [
            f"Imported by outlap.importers.pdt_h5 from {src.name} (§10.2)",
            "electric_machine map",
        ],
    )
    nan_frac = float(np.isnan(regrid.efficiency).mean())
    return {
        "out": str(out_yaml),
        "maps": str(maps_path),
        "vdc": choice.value,
        "speeds": int(speed_rpm.size),
        "torque_points": int(regrid.torque_nm.size),
        "nan_fraction": round(nan_frac, 3),
        "warnings": choice.warnings,
    }
