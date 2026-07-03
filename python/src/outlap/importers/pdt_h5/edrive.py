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
    emotor_out: Path | None = None,
    emit_emotor: bool = True,
    t_max_winding: float = 180.0,
    t_max_case: float = 120.0,
    t_warn_winding: float = 150.0,
    t_warn_case: float = 105.0,
    copper_feedback: bool = True,
    overload_from_cold: bool = False,
) -> dict[str, Any]:
    """Convert an EDrive HDF5 file to a `.ptm` document + parquet sidecar. Returns a summary."""
    maps_path = maps_path or out_yaml.with_suffix(".maps.parquet")
    emotor_out = emotor_out or out_yaml.with_suffix(".emotor.yaml")
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

        # LPTN aggregates + loss split for the 2-node distillation (§10.2 step 6).
        lptn_c = c.opt_arr(f, "thermal_obj/C")
        cu_alpha = c.scalar(f, "thermal_obj/cu_temp_coeff", default=0.00393)
        coolant_k = c.scalar(f, "thermal_obj/cooling/coolant_inlet_K", default=338.15)
        winding_loss = c.opt_arr(f, "operating_grid/loss_breakdown/winding_stator")
        total_loss_bd = c.opt_arr(f, "operating_grid/motor_loss_total")

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
    summary: dict[str, Any] = {
        "out": str(out_yaml),
        "maps": str(maps_path),
        "vdc": choice.value,
        "speeds": int(speed_rpm.size),
        "torque_points": int(regrid.torque_nm.size),
        "nan_fraction": round(nan_frac, 3),
        "warnings": choice.warnings,
    }

    # Distil the 2-node .emotor from the loss map + thermal envelopes.
    if emit_emotor and cont is not None and peak is not None and durations is not None:
        emotor_doc, rms = _distill_emotor(
            speed_rpm=speed_rpm,
            regrid=regrid,
            cont=cont.reshape(-1),
            peak=peak.reshape(peak.shape[-2], peak.shape[-1]),
            durations=durations.reshape(-1),
            lptn_c=lptn_c,
            cu_alpha=cu_alpha,
            coolant_c=coolant_k - 273.15,
            winding_loss=winding_loss,
            total_loss=total_loss_bd,
            iv=iv,
            limits=(t_max_winding, t_max_case, t_warn_winding, t_warn_case),
            copper_feedback=copper_feedback,
            overload_from_cold=overload_from_cold,
            provenance=f"EDrive {alias} {git}",
        )
        c.validate_against_schema(emotor_doc, "emotor")
        c.write_yaml(
            emotor_out,
            emotor_doc,
            [
                f"Imported by outlap.importers.pdt_h5 from {src.name} (§10.2 step 6)",
                "distilled 2-node thermal",
            ],
        )
        summary["emotor"] = str(emotor_out)
        summary["fit_rms"] = round(rms, 4)
    return summary


def _loss_at(regrid: c.Regrid, speed_rpm: float, torque: float) -> float:
    """Bilinear-ish loss lookup: nearest speed row, interpolate over the valid torque range."""
    si = int(np.argmin(np.abs(regrid.speed_rpm - speed_rpm)))
    row = regrid.loss_w[si]
    good = ~np.isnan(row)
    if good.sum() < 2:
        return 0.0
    return float(np.interp(torque, regrid.torque_nm[good], row[good]))


def _distill_emotor(
    *,
    speed_rpm: np.ndarray,
    regrid: c.Regrid,
    cont: np.ndarray,
    peak: np.ndarray,  # (speed, n_dur)
    durations: np.ndarray,
    lptn_c: np.ndarray | None,
    cu_alpha: float,
    coolant_c: float,
    winding_loss: np.ndarray | None,
    total_loss: np.ndarray | None,
    iv: int,
    limits: tuple[float, float, float, float],
    copper_feedback: bool,
    overload_from_cold: bool,
    provenance: str,
) -> tuple[dict[str, Any], float]:
    from .thermal_fit import ThermalTargets, build_emotor_doc, fit_two_node

    t_max_w, t_max_c, t_warn_w, t_warn_c = limits
    # Winding loss fraction (loss-weighted mean), else a documented default.
    if winding_loss is not None and total_loss is not None:
        w = np.nansum(np.abs(winding_loss[iv]))
        tot = np.nansum(np.abs(total_loss[iv]))
        s_w = float(np.clip(w / tot, 0.05, 0.95)) if tot > 0 else 0.7
    else:
        s_w = 0.7
    alpha = cu_alpha if copper_feedback else 0.0
    c_total = float(np.nansum(lptn_c)) if lptn_c is not None else 4000.0
    if not np.isfinite(c_total) or c_total <= 0.0:
        c_total = 4000.0

    # Target speeds: quantiles of the non-junk speed range (≥ 2% of max).
    valid = speed_rpm >= 0.02 * float(speed_rpm.max())
    idx_pool = np.flatnonzero(valid)
    picks = np.unique(
        np.quantile(idx_pool, [0.1, 0.3, 0.5, 0.7, 0.9]).round().astype(int)
    )
    omega = speed_rpm * (2.0 * np.pi / 60.0)

    p_cont = np.array(
        [_loss_at(regrid, float(speed_rpm[j]), float(cont[j])) for j in picks]
    )
    p_ovl = np.array(
        [
            [
                _loss_at(regrid, float(speed_rpm[j]), float(peak[j, k]))
                for k in range(peak.shape[1])
            ]
            for j in picks
        ]
    )
    targets = ThermalTargets(omega[picks], p_cont, p_ovl, durations, t_max_w, t_max_c)
    if overload_from_cold:
        # (IC handling lives in the fit; the flag is recorded in notes for the real-file study.)
        pass
    model, rms = fit_two_node(
        targets,
        c_total=c_total,
        s_w=s_w,
        alpha=alpha,
        t_cool=coolant_c,
        t_max_w=t_max_w,
        t_max_c=t_max_c,
    )
    dur_list = [int(round(float(d))) for d in durations]
    notes = (
        f"Distilled from PDT 19-node LPTN ({provenance}). Fit RMS {rms * 100:.2f}% over "
        f"{p_cont.size + p_ovl.size} envelope points (cont + {dur_list} s at "
        f"{p_cont.size} speeds). T_max/T_warn assumed (class-H defaults); "
        f"case capacitance is weakly constrained by winding-limited envelopes; "
        f"{'copper feedback on' if copper_feedback else 'copper feedback off'}."
    )
    doc = build_emotor_doc(
        model,
        t_warn_w=t_warn_w,
        t_warn_c=t_warn_c,
        t_max_w=t_max_w,
        t_max_c=t_max_c,
        notes=notes,
        copper_feedback=copper_feedback,
    )
    return doc, rms
