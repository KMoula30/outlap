# SPDX-License-Identifier: AGPL-3.0-only
"""DriveUnit stage file → ``du.ptm.yaml`` (+ ``maps.parquet``), kind ``drive_unit`` (§10.3).

Output-shaft side: ``opt_op/torque`` + ``opt_op/du_eff`` (combined motor+inverter+gearbox), thermal
under the capital-T ``peak_op/Thermal`` group, drag from ``no_load``, inertia ``at_output_j_kgm2``.
The gear ratio is already applied at the output shaft (``upstream_ratio_applied: true``); it is only
recorded in ``meta.source``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import h5py
import numpy as np

from . import common as c


def _resolve_mass(f: h5py.File, cli_mass: float | None) -> float:
    for path in (
        "mass/drive_total_mass",
        "mass/total",
        "mass/du_total_mass",
        "mass/motor_total_mass",
    ):
        node = f.get(path)
        if isinstance(node, h5py.Dataset):
            m = float(np.asarray(node[()]).reshape(-1)[0])
            if m > 0.0:
                return m
    if cli_mass and cli_mass > 0.0:
        return cli_mass
    raise c.PdtImportError(
        "could not resolve mass_kg (the Rust loader requires mass_kg > 0) — pass --mass-kg"
    )


def convert_driveunit(
    src: Path,
    out_yaml: Path,
    *,
    vdc: float | None = None,
    torque_points: int = 101,
    maps_path: Path | None = None,
    mass_kg: float | None = None,
) -> dict[str, Any]:
    """Convert a DriveUnit HDF5 file to a `.ptm` document + parquet sidecar. Returns a summary."""
    maps_path = maps_path or out_yaml.with_suffix(".maps.parquet")
    with h5py.File(src, "r") as f:
        speed_rpm = c.arr(f, "sweep/speed")
        vdc_grid = c.arr(f, "sweep/vdc")
        peak_op = f.get("peak_op")
        if not isinstance(peak_op, h5py.Group):
            raise c.PdtImportError("missing required group: peak_op")
        thermal = c.child(peak_op, "Thermal", "thermal")
        vdc_used = None
        if isinstance(thermal, h5py.Group):
            vu = thermal.get("continuous/vdc_used")
            if isinstance(vu, h5py.Dataset):
                vdc_used = float(np.asarray(vu[()]).reshape(-1)[0])
        choice = c.select_vdc(vdc_grid, vdc, vdc_used)
        iv = choice.index

        tau = c.arr(f, "opt_op/torque")[iv]
        eff = c.arr(f, "opt_op/du_eff")[iv]
        loss = c.arr(f, "opt_op/du_total_losses")[iv]
        torque_drive = c.arr(f, "peak_op/torque_drive")[iv]
        torque_regen = c.arr(f, "peak_op/torque_regen")[iv]

        regrid = c.regrid_map(
            speed_rpm, tau, eff, loss, torque_drive, torque_regen, choice, torque_points
        )

        # Thermal envelopes via the capital-T group.
        cont = peak_torque = durations = None
        if isinstance(thermal, h5py.Group):
            ct = thermal.get("continuous/torque")
            cont = (
                np.asarray(ct[()], dtype=np.float64)
                if isinstance(ct, h5py.Dataset)
                else None
            )
            pt = thermal.get("peak/torque")
            peak_torque = (
                np.asarray(pt[()], dtype=np.float64)
                if isinstance(pt, h5py.Dataset)
                else None
            )
            dd = thermal.get("peak/durations")
            durations = (
                np.asarray(dd[()], dtype=np.float64)
                if isinstance(dd, h5py.Dataset)
                else None
            )

        # Drag on the output-speed axis, resampled onto the map speed axis.
        drag = None
        no_load = f.get("no_load")
        if (
            isinstance(no_load, h5py.Group)
            and "torque_drag" in no_load
            and "output_speed" in no_load
        ):
            ns_speed = c.arr(no_load, "output_speed")
            ns_drag = c.arr(no_load, "torque_drag")
            order = np.argsort(ns_speed)
            drag = np.interp(speed_rpm, ns_speed[order], ns_drag[order])
        else:
            drag_env = c.opt_arr(f, "peak_op/torque_drag")
            drag = drag_env

        inertia = c.scalar(f, "inertia/at_output_j_kgm2")
        mass = _resolve_mass(f, mass_kg)
        alias = c.str_at(f, "info/gearbox/alias") or c.str_at(
            f, "info/alias", "driveunit"
        )
        gear_ratio = c.scalar(f, "info/gearbox/gear_ratio", default=0.0)
        git = c.find_git_hash(f, "DriveUnit")

    limits: dict[str, Any] = {
        "max_torque_nm_vs_speed": c.torque_curve(speed_rpm, torque_drive)
    }
    if cont is not None:
        limits["cont_torque_nm_vs_speed"] = c.torque_curve(speed_rpm, cont.reshape(-1))
    if peak_torque is not None and durations is not None:
        p2 = peak_torque.reshape(peak_torque.shape[-2], peak_torque.shape[-1])
        limits["overload"] = {
            "durations_s": [round(float(d), 3) for d in durations.reshape(-1)],
            "torque_nm_vs_speed": [
                c.torque_curve(speed_rpm, p2[:, k]) for k in range(p2.shape[1])
            ],
        }
    if drag is not None:
        limits["drag_torque_nm_vs_speed"] = c.torque_curve(speed_rpm, drag)

    ratio_note = (
        f" (gear ratio {gear_ratio:g} applied at output shaft)" if gear_ratio else ""
    )
    doc: dict[str, Any] = {
        "schema": "ptm/1.0",
        "kind": "drive_unit",
        "axes": {
            "speed_rpm": [round(float(s), 4) for s in speed_rpm],
            "load_axis": {"torque_nm": [round(float(t), 4) for t in regrid.torque_nm]},
            "torque_nm": [round(float(t), 4) for t in regrid.torque_nm],
        },
        "tables": {"file": maps_path.name, "efficiency": True, "loss_w": True},
        "limits": limits,
        "inertia_kgm2": round(inertia, 6),
        "mass_kg": round(mass, 4),
        "meta": {
            "source": f"PDT DriveUnit {alias} {git}{ratio_note}",
            "dc_voltage_v": choice.value,
            "upstream_ratio_applied": True,
        },
    }

    c.validate_against_schema(doc, "ptm")
    c.write_maps_parquet(maps_path, regrid)
    c.write_yaml(
        out_yaml,
        doc,
        [
            f"Imported by outlap.importers.pdt_h5 from {src.name} (§10.3)",
            "drive_unit map",
        ],
    )
    return {
        "out": str(out_yaml),
        "maps": str(maps_path),
        "vdc": choice.value,
        "gear_ratio": gear_ratio,
        "nan_fraction": round(float(np.isnan(regrid.efficiency).mean()), 3),
        "warnings": choice.warnings,
    }
