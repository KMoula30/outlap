# SPDX-License-Identifier: AGPL-3.0-only
"""Synthetic PDT-shaped HDF5 builders for the importer tests (§10.5).

Tiny files that mirror the real group/dataset structure but with internally-consistent, mostly
piecewise-linear physics, so source→regular-grid→source reproduction is exact to ~1e-6. No real PDT
data is ever committed (firewall, §1); these builders run in tmp during the test session.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np

TAU_PK = 128.0  # binary-exact so float32 storage is lossless
TAU_REGEN = -0.9 * TAU_PK


@dataclass
class EdriveTruth:
    """What the synthetic EDrive encodes, for the tests to check against."""

    speed_rpm: np.ndarray
    vdc: np.ndarray
    load_ratio: np.ndarray
    airgap_torque: np.ndarray  # (nv, ns, nl)
    system_eff: np.ndarray  # (nv, ns, nl)


def _motor_eff(tau: np.ndarray, s_frac: float) -> np.ndarray:
    """Piecewise-linear in τ (kink only at 0), so re-gridding is exact away from 0."""
    return np.clip(0.94 - 0.1 * np.abs(tau) / TAU_PK - 0.05 * s_frac, 0.2, 1.0)


def make_edrive(
    path: Path,
    *,
    nv: int = 2,
    ns: int = 6,
    nl: int = 9,
    loss_breakdown: bool = True,
    mdt_attrs: bool = True,
) -> EdriveTruth:
    """Write a synthetic EDrive-shaped file; return its ground truth."""
    vdc = np.array([390.0, 400.0][:nv] + [400.0] * (nv - 2), dtype=np.float32)
    speed = np.linspace(100.0, 12000.0, ns, dtype=np.float32)
    omega = speed * (2.0 * np.pi / 60.0)
    load_ratio = np.linspace(-1.0, 1.0, nl, dtype=np.float32)  # odd nl → exact 0

    tau = np.zeros((nv, ns, nl), np.float32)
    meff = np.zeros((nv, ns, nl), np.float32)
    mloss = np.zeros((nv, ns, nl), np.float32)
    for v in range(nv):
        for s in range(ns):
            t = np.where(
                load_ratio >= 0, load_ratio * TAU_PK, load_ratio * abs(TAU_REGEN)
            )
            tau[v, s] = t
            e = _motor_eff(t, s / max(ns - 1, 1))
            e[np.abs(t) < 1e-6] = (
                0.0  # η = 0 at zero torque (spin point), like real PDT
            )
            meff[v, s] = e
            p_mech = np.abs(t) * omega[s]
            # Definitional loss from η (spin loss p0 at τ=0), drive vs regen.
            p0 = 20.0 + 5.0 * s
            loss = np.where(
                t > 0, p_mech * (1.0 / np.maximum(e, 1e-3) - 1.0), p_mech * (1.0 - e)
            )
            mloss[v, s] = np.where(np.abs(t) < 1e-6, p0, loss)

    with h5py.File(path, "w") as f:
        sw = f.create_group("sweep")
        sw["vdc"] = vdc
        sw["speed"] = speed
        sw["omega"] = omega.astype(np.float32)
        sw["load_ratio"] = load_ratio
        og = f.create_group("operating_grid")
        if mdt_attrs:
            og.attrs["__mdt_type__"] = "OperatingGrid"
        og["airgap_torque"] = tau
        og["motor_efficiency"] = meff
        og["inverter_efficiency"] = np.ones((nv, ns, nl), np.float32)
        og["motor_loss_total"] = mloss
        og["inverter_loss_total"] = np.zeros((nv, ns, nl), np.float32)
        if loss_breakdown:
            lb = og.create_group("loss_breakdown")
            lb["winding_stator"] = (0.6 * mloss).astype(np.float32)
            lb["core_total"] = (0.3 * mloss).astype(np.float32)
            lb["inverter_conduction"] = (0.1 * mloss).astype(np.float32)
        pc = f.create_group("peak_capability")
        pc["torque_drive"] = np.full((nv, ns), TAU_PK, np.float32)
        pc["torque_regen"] = np.full((nv, ns), TAU_REGEN, np.float32)
        pc["torque_drag"] = np.full((ns,), -2.0, np.float32)
        th = pc.create_group("thermal")
        th["continuous/torque"] = np.full((1, ns), 0.6 * TAU_PK, np.float32)
        th["continuous/vdc_used"] = np.array([400.0], np.float32)
        th["peak/durations"] = np.array([10.0, 20.0, 30.0], np.float32)
        peak = np.stack([np.full(ns, k * TAU_PK) for k in (0.9, 0.8, 0.7)], axis=-1)
        th["peak/torque"] = peak.reshape(1, ns, 3).astype(np.float32)
        th["peak/vdc_used"] = np.array([400.0], np.float32)
        f["inertia/rotor_inertia"] = np.float32(0.007)
        f["mass/drive_total_mass"] = np.float32(20.0)
        f["info/alias"] = np.bytes_(b"synth_edrive")
        # 19-node LPTN skeleton (for the PR-7 thermal fit).
        to = f.create_group("thermal_obj")
        to["C"] = np.full((19,), 200.0, np.float32)
        to["G_const"] = np.eye(19, dtype=np.float32) * 5.0
        to["cu_temp_coeff"] = np.float32(0.00393)
        to["cooling/coolant_inlet_K"] = np.float32(338.15)
        f["compute/EDrive/git_hash"] = np.bytes_(b"abcdef123456")

    return EdriveTruth(
        speed_rpm=speed.astype(np.float64),
        vdc=vdc.astype(np.float64),
        load_ratio=load_ratio.astype(np.float64),
        airgap_torque=tau.astype(np.float64),
        system_eff=meff.astype(np.float64),  # inv_eff = 1 → system == motor
    )


def make_driveunit(
    path: Path, *, nv: int = 2, ns: int = 6, nl: int = 9, thermal_name: str = "Thermal"
) -> None:
    """Write a synthetic DriveUnit-shaped file (capital-T `Thermal` by default)."""
    vdc = np.array([48.0, 60.0][:nv], dtype=np.float32)
    speed = np.linspace(10.0, 369.0, ns, dtype=np.float32)
    omega = speed * (2.0 * np.pi / 60.0)
    load_ratio = np.linspace(-1.0, 1.0, nl, dtype=np.float32)
    tau = np.zeros((nv, ns, nl), np.float32)
    eff = np.zeros((nv, ns, nl), np.float32)
    loss = np.zeros((nv, ns, nl), np.float32)
    for v in range(nv):
        for s in range(ns):
            t = np.where(load_ratio >= 0, load_ratio * 168.0, load_ratio * 150.0)
            tau[v, s] = t
            e = np.clip(0.92 - 0.08 * np.abs(t) / 168.0, 0.2, 1.0)
            eff[v, s] = e
            p = np.abs(t) * omega[s]
            loss[v, s] = np.where(
                np.abs(t) < 1e-6, 15.0, p * (1.0 / np.maximum(e, 1e-3) - 1.0)
            )
    with h5py.File(path, "w") as f:
        f["sweep/vdc"] = vdc
        f["sweep/speed"] = speed
        f["sweep/load_ratio"] = load_ratio
        oo = f.create_group("opt_op")
        oo["torque"] = tau
        oo["du_eff"] = eff
        oo["du_total_losses"] = loss
        po = f.create_group("peak_op")
        po["torque_drive"] = np.full((nv, ns), 168.0, np.float32)
        po["torque_regen"] = np.full((nv, ns), -150.0, np.float32)
        th = po.create_group(thermal_name)
        th["continuous/torque"] = np.full((1, ns), 0.6 * 168.0, np.float32)
        th["peak/durations"] = np.array([10.0, 20.0, 30.0], np.float32)
        th["peak/torque"] = (
            np.stack([np.full(ns, k * 168.0) for k in (0.9, 0.8, 0.7)], axis=-1)
            .reshape(1, ns, 3)
            .astype(np.float32)
        )
        nlg = f.create_group("no_load")
        nlg["output_speed"] = np.linspace(0.0, 400.0, 12, dtype=np.float32)
        nlg["torque_drag"] = np.linspace(-1.0, -6.0, 12, dtype=np.float32)
        f["inertia/at_output_j_kgm2"] = np.float32(0.021)
        f["inertia/at_input_j_kgm2"] = np.float32(8e-5)
        f["mass/drive_total_mass"] = np.float32(9.5)
        f["info/gearbox/gear_ratio"] = np.float32(16.2489)
        f["info/gearbox/num_of_stages"] = np.int64(2)
        f["info/gearbox/alias"] = np.bytes_(b"synth_du")


def make_batterypack(path: Path, *, n_soc: int = 6, n_temp: int = 3) -> None:
    """Write a synthetic BatteryPack-shaped file with affine cell tables."""
    soc = np.linspace(0.05, 1.0, n_soc, dtype=np.float32)
    temp = np.linspace(-10.0, 45.0, n_temp, dtype=np.float32)
    SS, TT = np.meshgrid(soc, temp, indexing="ij")
    with h5py.File(path, "w") as f:
        f["vector/soc"] = soc
        f["vector/temperature"] = temp
        f["vector/current"] = np.linspace(0.0, 60.0, n_soc, dtype=np.float32)
        f["cell/ocv_t"] = (3.0 + 1.2 * SS - 0.001 * (TT - 25.0)).astype(np.float32)
        f["cell/r0"] = (0.018 + 0.002 * (1.0 - SS)).astype(np.float32)
        f["cell/r1"] = (0.005 + 0.001 * (1.0 - SS)).astype(np.float32)
        f["cell/tau1"] = np.full((n_soc, n_temp), 15.0, np.float32)
        f["cell/dudt"] = np.full((n_soc, n_temp), 1e-4, np.float32)
        f["cell/cp"] = np.float32(653.0)
        f["pack/q_pack"] = np.float32(15.0)
        f["pack/e_pack"] = np.float32(721.5)
        f["pack/mass"] = np.float32(2.65)
        f["pack/thermal_resistance"] = np.float32(0.128)
        f["pack/peak_discharge_power"] = np.linspace(
            1000.0, 2600.0, n_soc, dtype=np.float32
        )
        f["pack/peak_regen_power"] = np.linspace(800.0, 2600.0, n_soc, dtype=np.float32)
        f["info/ns"] = np.int64(13)
        f["info/np"] = np.int64(3)
        f["info/min_soc"] = np.float32(0.05)
        f["info/max_soc"] = np.float32(1.0)
        f["info/cell_name"] = np.bytes_(b"Generic_21700_5Ah")
        f["info/cell_chemistry"] = np.bytes_(b"NMC")
        f["info/coolant_temperature"] = np.float32(25.0)
        f["info/max_c_rate"] = np.float32(2.0)
        f["info/min_voltage"] = np.float32(2.75)
        f["info/max_voltage"] = np.float32(4.2)
        f["info/alias"] = np.bytes_(b"synth_batt")
