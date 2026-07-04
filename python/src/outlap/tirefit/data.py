# SPDX-License-Identifier: AGPL-3.0-only
"""Tire test-data ingestion: TTC ``.mat`` (v7 and v7.3), ``.dat``, and ``.csv`` → SI/ISO-8855.

**Redistribution policy (restated from the package README): parsers yes — REDISTRIBUTION OF TTC
DATA OR TTC-DERIVED PARAMETER SETS, NO.** FSAE TTC data is membership-locked; these readers let
members fit locally. Never commit raw TTC files or fitted parameter sets derived from them to
this repository (a local ``ttc-data/`` directory is gitignored for exactly this workflow).

Channels are mapped to SI, ISO 8855 (x forward, y left, z up; ISO-W tire axes). TTC channels
arrive in the SAE tire axis system (z down); the conversion is the proper rotation by π about
the x-axis, ``(x, y, z) → (x, −y, −z)``, applied consistently to angles, forces, and moments
(:func:`sae_to_iso`):

* ``α → −α``, ``γ → −γ`` (the y axis flips, so slip angle and inclination flip);
* ``Fx → Fx``, ``Fy → −Fy``, ``Fz → −Fz`` (SAE logs the normal load negative; ISO wants > 0);
* ``Mx → Mx``, ``My → −My``, ``Mz → −Mz`` (moments transform as vectors under the proper
  rotation; TTC does not log My);
* ``κ → κ`` (a ratio, axis-free).

Unit conversions at this boundary only: deg → rad, kPa → Pa, kph → m/s.
"""

from __future__ import annotations

import csv as _csv
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray

F = NDArray[np.float64]

#: TTC channel names (SAE, TTC units) → our field + SI conversion factor. SA/IA in deg, P in
#: kPa, V in kph, forces in N, moments in N·m.
_TTC_CHANNELS: dict[str, tuple[str, float]] = {
    "SA": ("alpha_rad", np.pi / 180.0),
    "IA": ("gamma_rad", np.pi / 180.0),
    "SR": ("kappa", 1.0),
    "SL": ("kappa", 1.0),  # some rounds log slip as SL
    "FX": ("fx_n", 1.0),
    "FY": ("fy_n", 1.0),
    "FZ": ("fz_n", 1.0),
    "MX": ("mx_nm", 1.0),
    "MZ": ("mz_nm", 1.0),
    "P": ("p_pa", 1000.0),
    "V": ("vx_mps", 1.0 / 3.6),
}

_REQUIRED = ("alpha_rad", "fz_n", "fy_n")


@dataclass
class TireTestData:
    """A flat, SI/ISO-W set of test samples (equal-length 1-D arrays)."""

    kappa: F
    alpha_rad: F
    gamma_rad: F
    fz_n: F
    p_pa: F
    vx_mps: F
    fx_n: F
    fy_n: F
    mz_nm: F
    mx_nm: F

    def __len__(self) -> int:
        return int(self.alpha_rad.shape[0])


def sae_to_iso(fields: dict[str, F]) -> dict[str, F]:
    """Apply the SAE → ISO-W sign map (module docs) to a dict of SI channel arrays."""
    out = dict(fields)
    for key in ("alpha_rad", "gamma_rad", "fy_n", "fz_n", "mz_nm"):
        if key in out:
            out[key] = -out[key]
    # Mx keeps its sign (axis x is unchanged under the π rotation about x); My would flip but
    # TTC does not log My.
    return out


def _assemble(fields: dict[str, F], label: str) -> TireTestData:
    for key in _REQUIRED:
        if key not in fields:
            raise ValueError(f"{label}: missing required channel for `{key}`")
    n = fields["alpha_rad"].shape[0]
    zeros = np.zeros(n, dtype=np.float64)

    def get(key: str, default: F) -> F:
        arr = fields.get(key, default)
        if arr.shape[0] != n:
            raise ValueError(f"{label}: channel `{key}` length {arr.shape[0]} != {n}")
        return arr

    return TireTestData(
        kappa=get("kappa", zeros),
        alpha_rad=fields["alpha_rad"],
        gamma_rad=get("gamma_rad", zeros),
        fz_n=fields["fz_n"],
        p_pa=get("p_pa", zeros),
        vx_mps=get("vx_mps", zeros),
        fx_n=get("fx_n", zeros),
        fy_n=fields["fy_n"],
        mz_nm=get("mz_nm", zeros),
        mx_nm=get("mx_nm", zeros),
    )


def _map_channels(raw: dict[str, F]) -> dict[str, F]:
    """TTC channel names → SI field arrays (upper-cased name lookup, unit factors applied)."""
    fields: dict[str, F] = {}
    for name, arr in raw.items():
        spec = _TTC_CHANNELS.get(name.upper())
        if spec is None:
            continue
        field, factor = spec
        fields[field] = np.asarray(arr, dtype=np.float64).ravel() * factor
    return fields


def load_ttc_mat(path: str | Path) -> TireTestData:
    """Load a TTC MATLAB file (v7 via ``scipy.io``, v7.3/HDF5 via ``h5py``) → SI/ISO-W.

    Requires the ``tire-fit`` extra for v7 files (``scipy``); v7.3 needs only ``h5py``.
    """
    path = Path(path)
    raw: dict[str, F] = {}
    if _is_hdf5(path):
        import h5py

        with h5py.File(path, "r") as f:
            # h5py ships no type stubs; cross its boundary through explicit Any.
            for name in cast("list[str]", list(f.keys())):
                obj = f[name]
                if not isinstance(obj, h5py.Dataset):
                    continue
                ds: Any = obj
                if ds.dtype.kind in "fiu":
                    raw[name] = np.asarray(ds[()], dtype=np.float64)
    else:
        loadmat = _scipy_loadmat()
        mat: dict[str, Any] = loadmat(str(path))
        for name, value in mat.items():
            if name.startswith("__"):
                continue
            arr = np.asarray(value)
            if arr.dtype.kind in "fiu":
                raw[str(name)] = arr.astype(np.float64)
    return _assemble(sae_to_iso(_map_channels(raw)), str(path))


def load_dat(path: str | Path) -> TireTestData:
    """Load a TTC ASCII ``.dat`` (title line, channel-name line, units line, then columns)."""
    path = Path(path)
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    if len(lines) < 4:
        raise ValueError(f"{path}: too short for a TTC .dat (title/names/units/data)")
    names = lines[1].split()
    data = np.loadtxt(lines[3:], dtype=np.float64, ndmin=2)
    if data.shape[1] != len(names):
        raise ValueError(
            f"{path}: {len(names)} channel names but {data.shape[1]} data columns"
        )
    raw = {name: data[:, i] for i, name in enumerate(names)}
    return _assemble(sae_to_iso(_map_channels(raw)), str(path))


def load_csv(
    path: str | Path, *, sae_signs: bool = True, has_units_row: bool = False
) -> TireTestData:
    """Load a headered CSV of TTC-named channels (set ``sae_signs=False`` if already ISO)."""
    path = Path(path)
    with path.open(newline="", encoding="utf-8") as f:
        reader = _csv.reader(f)
        header = next(reader)
        rows = list(reader)
    if has_units_row and rows:
        rows = rows[1:]
    if not rows:
        raise ValueError(f"{path}: no data rows")
    data = np.array([[float(x) for x in row] for row in rows], dtype=np.float64)
    raw = {name.strip(): data[:, i] for i, name in enumerate(header)}
    fields = _map_channels(raw)
    if sae_signs:
        fields = sae_to_iso(fields)
    return _assemble(fields, str(path))


@dataclass
class SweepBin:
    """One nominal-condition bin: the mask plus the bin's nominal levels."""

    mask: NDArray[np.bool_]
    fz_n: float
    p_pa: float
    gamma_rad: float


def bin_sweeps(
    data: TireTestData,
    *,
    fz_step_n: float = 500.0,
    p_step_pa: float = 10_000.0,
    gamma_step_rad: float = np.pi / 180.0,
    min_samples: int = 20,
) -> list[SweepBin]:
    """Group samples into nominal (Fz, p, γ) bins for per-condition fitting/reporting.

    Levels are the medians of samples rounded to the given steps; bins with fewer than
    ``min_samples`` samples are dropped (transients between setpoints).
    """
    keys = (
        np.round(data.fz_n / fz_step_n).astype(np.int64),
        np.round(data.p_pa / p_step_pa).astype(np.int64),
        np.round(data.gamma_rad / gamma_step_rad).astype(np.int64),
    )
    combined = np.stack(keys, axis=1)
    bins: list[SweepBin] = []
    for level in np.unique(combined, axis=0):
        mask = np.all(combined == level, axis=1)
        if int(mask.sum()) < min_samples:
            continue
        bins.append(
            SweepBin(
                mask=mask,
                fz_n=float(np.median(data.fz_n[mask])),
                p_pa=float(np.median(data.p_pa[mask])),
                gamma_rad=float(np.median(data.gamma_rad[mask])),
            )
        )
    bins.sort(key=lambda b: (b.fz_n, b.p_pa, b.gamma_rad))
    return bins


def _is_hdf5(path: Path) -> bool:
    with path.open("rb") as f:
        return f.read(8) == b"\x89HDF\r\n\x1a\n"


def _scipy_loadmat() -> Callable[..., dict[str, Any]]:
    """Lazy scipy import with an actionable error (the ``osm_track`` precedent)."""
    try:
        from scipy.io import loadmat  # pyright: ignore[reportUnknownVariableType]
    except ImportError as err:  # pragma: no cover - exercised only without the extra
        raise ImportError(
            "reading MATLAB v7 .mat files needs scipy — install the extra: "
            "`uv sync --extra tire-fit` (or `pip install 'outlap[tire-fit]'`)"
        ) from err
    # scipy's stubs leave loadmat partially unknown; pin the shape we rely on.
    return cast("Callable[..., dict[str, Any]]", loadmat)
