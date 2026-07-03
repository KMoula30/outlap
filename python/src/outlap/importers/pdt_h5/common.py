# SPDX-License-Identifier: AGPL-3.0-only
"""Shared helpers for the PDT HDF5 importers (§10).

Firewall (§1): reads raw HDF5 with ``h5py`` only, never imports PDT code. Rebuilds power as
``τ[Nm]·ω[rad/s]`` and never trusts the ``performance``/``metrics`` summary scalars (§10.1).
Keys on dataset *presence*, not the ``__mdt_type__`` attrs.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import h5py
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import yaml


class PdtImportError(Exception):
    """A PDT file was missing a required dataset or was otherwise unusable."""


def h5_str(node: h5py.Dataset | bytes | str) -> str:
    """Decode an HDF5 string/bytes scalar to ``str``."""
    val = node[()] if isinstance(node, h5py.Dataset) else node
    if isinstance(val, bytes):
        return val.decode("utf-8", "replace")
    if isinstance(val, np.ndarray):
        return h5_str(val.reshape(-1)[0])
    return str(val)


def str_at(f: h5py.File | h5py.Group, path: str, default: str = "") -> str:
    """Decode a string dataset at ``path``, or ``default`` if absent."""
    node = f.get(path)
    return h5_str(node) if isinstance(node, h5py.Dataset) else default


def child(group: h5py.Group, *names: str) -> h5py.Group | h5py.Dataset | None:
    """First present child among ``names`` (handles e.g. ``Thermal`` vs ``thermal``)."""
    for name in names:
        node = group.get(name)
        if isinstance(node, (h5py.Group, h5py.Dataset)):
            return node
    return None


def require(f: h5py.File | h5py.Group, path: str) -> h5py.Dataset:
    """Fetch a dataset by path, or raise :class:`PdtImportError`."""
    node = f.get(path)
    if not isinstance(node, h5py.Dataset):
        raise PdtImportError(f"missing required dataset: {path}")
    return node


def arr(f: h5py.File | h5py.Group, path: str) -> np.ndarray:
    """A required dataset as a ``float64`` array."""
    return np.asarray(require(f, path)[()], dtype=np.float64)


def opt_arr(f: h5py.File | h5py.Group, path: str) -> np.ndarray | None:
    """An optional dataset as a ``float64`` array, or ``None`` if absent."""
    node = f.get(path)
    return (
        np.asarray(node[()], dtype=np.float64)
        if isinstance(node, h5py.Dataset)
        else None
    )


def scalar(f: h5py.File | h5py.Group, path: str, default: float | None = None) -> float:
    """A required scalar as ``float`` (``default`` if absent and provided)."""
    node = f.get(path)
    if not isinstance(node, h5py.Dataset):
        if default is not None:
            return default
        raise PdtImportError(f"missing required scalar: {path}")
    return float(np.asarray(node[()]).reshape(-1)[0])


def find_git_hash(f: h5py.File, stage: str) -> str:
    """Best-effort pipeline git hash from ``compute/<stage>*`` (else ``"unknown"``)."""
    compute = f.get("compute")
    if isinstance(compute, h5py.Group):
        for key in compute:
            if stage.lower() not in str(key).lower():
                continue
            grp = compute[key]
            if not isinstance(grp, h5py.Group):
                continue
            for cand in ("git_hash", "git_commit", "commit", "sha", "hash"):
                node = grp.get(cand)
                if isinstance(node, h5py.Dataset):
                    return h5_str(node)[:12]
    return "unknown"


@dataclass
class VdcChoice:
    """The DC-voltage slice actually used."""

    index: int
    value: float
    warnings: list[str] = field(default_factory=list)


def select_vdc(
    vdc_grid: np.ndarray, requested: float | None, vdc_used: float | None
) -> VdcChoice:
    """Pick the nearest grid vdc (no cross-vdc interpolation — thermal envelopes are single-vdc)."""
    warnings: list[str] = []
    target = requested if requested is not None else vdc_used
    if target is None:
        idx = int(np.argmax(vdc_grid))
        warnings.append(
            f"no --vdc and no thermal vdc_used; defaulting to max grid {vdc_grid[idx]:g} V"
        )
    else:
        idx = int(np.argmin(np.abs(vdc_grid - target)))
        if abs(vdc_grid[idx] - target) > 0.02 * max(abs(target), 1.0):
            warnings.append(
                f"requested vdc {target:g} V snapped to grid {vdc_grid[idx]:g} V"
            )
    return VdcChoice(index=idx, value=float(vdc_grid[idx]), warnings=warnings)


def build_torque_axis(tau_min: float, tau_max: float, n: int) -> np.ndarray:
    """A regular torque grid with an exact 0 node and asymmetric drive/regen bounds."""
    if tau_max <= 0.0:
        tau_max = 1.0
    if tau_min >= 0.0:
        tau_min = -1.0
    span = tau_max - tau_min
    n_pos = max(1, round((n - 1) * tau_max / span))
    n_neg = max(1, (n - 1) - n_pos)
    neg = np.linspace(tau_min, 0.0, n_neg + 1)[:-1]
    pos = np.linspace(0.0, tau_max, n_pos + 1)
    return np.concatenate([neg, pos])


def invert_load_to_torque(
    tau_raw: np.ndarray, val_raw: np.ndarray, tau_axis: np.ndarray, eff_raw: np.ndarray
) -> np.ndarray:
    """Re-grid one speed row's ``val(load_ratio)`` onto a regular torque axis.

    ``tau_raw`` is monotone-nondecreasing in load ratio but saturates (repeats) in the infeasible
    tails where efficiency is zero. The valid core is the strictly-increasing block containing the
    zero-torque point; values outside its torque range become ``NaN``.
    """
    valid = (eff_raw > 0.0) | (np.abs(tau_raw) <= 1e-9)
    idx = np.flatnonzero(valid)
    if idx.size < 2:
        return np.full_like(tau_axis, np.nan)
    t = tau_raw[idx]
    v = val_raw[idx]
    # Keep strictly-increasing samples (trim clipped duplicates).
    keep = np.concatenate([[True], np.diff(t) > 1e-9 * max(np.ptp(t), 1.0)])
    t, v = t[keep], v[keep]
    if t.size < 2:
        return np.full_like(tau_axis, np.nan)
    out = np.interp(tau_axis, t, v)
    out[(tau_axis < t[0]) | (tau_axis > t[-1])] = np.nan
    return out


@dataclass
class Regrid:
    """A re-gridded machine map: axes plus the efficiency/loss tables (NaN-masked)."""

    speed_rpm: np.ndarray
    torque_nm: np.ndarray
    efficiency: np.ndarray  # (n_speed, n_torque)
    loss_w: np.ndarray  # (n_speed, n_torque)
    vdc: VdcChoice
    warnings: list[str] = field(default_factory=list)


def regrid_map(
    speed_rpm: np.ndarray,
    tau_grid: np.ndarray,  # (n_speed, n_load) torque vs load ratio at the chosen vdc
    eff_grid: np.ndarray,  # (n_speed, n_load) system efficiency 0..1
    loss_grid: np.ndarray,  # (n_speed, n_load) system loss W
    torque_drive: np.ndarray,  # (n_speed,) positive envelope
    torque_regen: np.ndarray,  # (n_speed,) negative envelope
    vdc: VdcChoice,
    torque_points: int,
) -> Regrid:
    """Invert load-ratio→torque per speed and mask cells beyond the peak envelope."""
    tau_max = float(np.nanmax(torque_drive))
    tau_min = float(np.nanmin(torque_regen))
    axis = build_torque_axis(tau_min, tau_max, torque_points)
    ns, nt = speed_rpm.size, axis.size
    eff = np.full((ns, nt), np.nan)
    loss = np.full((ns, nt), np.nan)
    for i in range(ns):
        eff[i] = invert_load_to_torque(tau_grid[i], eff_grid[i], axis, eff_grid[i])
        loss[i] = invert_load_to_torque(tau_grid[i], loss_grid[i], axis, eff_grid[i])
        # Envelope mask: drop cells beyond the per-speed peak torque.
        hi = torque_drive[i] + 1e-6 * abs(torque_drive[i]) + 1e-9
        lo = torque_regen[i] - 1e-6 * abs(torque_regen[i]) - 1e-9
        beyond = (axis > hi) | (axis < lo)
        eff[i][beyond] = np.nan
        loss[i][beyond] = np.nan
        # Keep the zero-torque column at η = 0 (spin point) rather than NaN.
        zc = int(np.argmin(np.abs(axis)))
        if np.isnan(eff[i][zc]):
            eff[i][zc] = 0.0
        # Loss is NaN exactly where efficiency is NaN.
        loss[i][np.isnan(eff[i])] = np.nan
    return Regrid(
        speed_rpm=speed_rpm, torque_nm=axis, efficiency=eff, loss_w=loss, vdc=vdc
    )


def write_maps_parquet(path: Path, r: Regrid) -> None:
    """Write the long/tidy efficiency+loss table (``speed_rpm,torque_nm,efficiency,loss_w``)."""
    ns, nt = r.efficiency.shape
    speed = np.repeat(r.speed_rpm, nt)
    torque = np.tile(r.torque_nm, ns)
    table = pa.table(
        {
            "speed_rpm": speed.astype(np.float64),
            "torque_nm": torque.astype(np.float64),
            "efficiency": r.efficiency.reshape(-1),
            "loss_w": r.loss_w.reshape(-1),
        }
    )
    pq.write_table(table, path)


def torque_curve(
    speed_rpm: np.ndarray, torque_nm: np.ndarray
) -> dict[str, list[float]]:
    """A ``TorqueCurve`` mapping for the .ptm ``limits`` block."""
    return {
        "speed_rpm": [round(float(s), 4) for s in speed_rpm],
        "torque_nm": [round(float(t), 4) for t in torque_nm],
    }


def write_yaml(path: Path, doc: dict[str, Any], header: list[str]) -> None:
    """Write a YAML document with an ``# Imported by …`` comment header (keys unsorted)."""
    text = "\n".join(f"# {h}" for h in header) + "\n"
    text += yaml.safe_dump(doc, sort_keys=False, default_flow_style=False)
    path.write_text(text, encoding="utf-8")


def validate_against_schema(doc: dict[str, Any], schema_name: str) -> None:
    """Validate an emitted document against a committed JSON Schema (`schemas/<name>.json`)."""
    from jsonschema import Draft202012Validator

    root = Path(__file__).resolve().parents[5]
    schema = json.loads(
        (root / "schemas" / f"{schema_name}.json").read_text(encoding="utf-8")
    )
    errors = sorted(Draft202012Validator(schema).iter_errors(doc), key=str)
    if errors:
        locs = "; ".join(
            f"{'/'.join(map(str, e.absolute_path))}: {e.message}" for e in errors[:5]
        )
        raise PdtImportError(f"emitted {schema_name} failed schema validation: {locs}")
