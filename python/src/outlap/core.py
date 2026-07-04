# SPDX-License-Identifier: AGPL-3.0-only
"""Typed, pythonic surface over the ``outlap_core`` Rust bindings.

The Rust extension returns plain numpy arrays; this module adds the ergonomics the notebooks and
user code want: scalar/array broadcasting for tyre evaluation, and results as labelled
:class:`xarray.Dataset` objects (the designed results boundary — dims/coords/attrs, Decision #17).

Everything here is a thin veneer: no physics, no defaults beyond the Rust core's own.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np
import xarray as xr
from numpy.typing import ArrayLike, NDArray

from outlap_core import (
    DEFAULT_DS_M,
    Lap,
    Raceline,
    Track,
    Tyre,
    min_curvature,
    solve_lap,
    vehicle_report,
)

__all__ = [
    "DEFAULT_DS_M",
    "Lap",
    "Raceline",
    "Track",
    "Tyre",
    "TyreForces",
    "lap_dataset",
    "min_curvature",
    "solve_lap",
    "solve_lap_dataset",
    "track_dataset",
    "tyre_forces",
    "vehicle_report",
]


class TyreForces(NamedTuple):
    """Steady-state tyre outputs (ISO 8855): forces in N, moments in N·m."""

    fx: NDArray[np.float64]
    fy: NDArray[np.float64]
    mz: NDArray[np.float64]
    mx: NDArray[np.float64]
    my: NDArray[np.float64]


def tyre_forces(
    tyre: Tyre,
    *,
    kappa: ArrayLike = 0.0,
    alpha: ArrayLike = 0.0,
    gamma: ArrayLike = 0.0,
    fz: ArrayLike | None = None,
    p: ArrayLike | None = None,
    vx: ArrayLike = 16.7,
) -> TyreForces:
    """Evaluate the MF6.1 model, broadcasting scalars and arrays numpy-style.

    Defaults: ``fz`` → the tyre's nominal load, ``p`` → its cold inflation pressure. Angles are
    rad, loads N, pressure Pa, speed m/s.
    """
    fz = tyre.fnomin if fz is None else fz
    p = tyre.p_cold if p is None else p
    arrays = np.broadcast_arrays(
        *(np.asarray(a, dtype=np.float64) for a in (kappa, alpha, gamma, fz, p, vx))
    )
    flat = [np.ascontiguousarray(a).ravel() for a in arrays]
    out = tyre.forces(flat[0], flat[1], flat[2], flat[3], flat[4], flat[5])
    shape = arrays[0].shape
    return TyreForces(*(a.reshape(shape) for a in out))


def lap_dataset(lap: Lap) -> xr.Dataset:
    """Convert a solved lap into a labelled :class:`xarray.Dataset` over the ``s`` dimension."""
    s = lap.s()
    return xr.Dataset(
        {
            "v": ("s", lap.v(), {"units": "m/s", "long_name": "speed"}),
            "ax": (
                "s",
                lap.ax(),
                {"units": "m/s²", "long_name": "longitudinal acceleration"},
            ),
            "ay": (
                "s",
                lap.ay(),
                {"units": "m/s²", "long_name": "lateral acceleration (+left)"},
            ),
            "t": ("s", lap.t(), {"units": "s", "long_name": "cumulative time"}),
            "x": ("s", lap.x(), {"units": "m"}),
            "y": ("s", lap.y(), {"units": "m"}),
            "z": ("s", lap.z(), {"units": "m", "long_name": "elevation"}),
        },
        coords={"s": ("s", s, {"units": "m", "long_name": "arc length"})},
        attrs={
            "lap_time_s": lap.lap_time_s,
            "resolved_hash": lap.resolved_hash,
            # Tuple of str, not list/bool: keeps the attrs netCDF-serializable (no bool attr
            # type in netCDF; empty-list attrs coerce badly).
            "notes": tuple(lap.notes),
        },
    )


def solve_lap_dataset(
    vehicle_dir: str,
    line: Track | Raceline,
    *,
    ds_m: float = DEFAULT_DS_M,
    overrides: dict[str, bool | int | float | str] | None = None,
    conditions: dict[str, object] | None = None,
) -> xr.Dataset:
    """Solve a T0 lap and return it directly as a labelled dataset (see :func:`lap_dataset`).

    ``line`` may be a plain :class:`Track` (a lap of its centerline) or a :class:`Raceline`
    (a lap of the generated line, with its generation step recorded in the result provenance).

    What-if experiments: ``overrides`` patches dotted paths onto the vehicle through the real
    validation pipeline (e.g. ``{"chassis.mass_kg": 750.0}``), and ``conditions`` deep-merges
    onto the session conditions (e.g. ``{"air": {"temp_c": 35.0}}``) — invalid paths or types
    fail loudly, never silently.
    """
    if isinstance(line, Raceline):
        lap = solve_lap(
            vehicle_dir,
            line.line(),
            ds_m=ds_m,
            raceline_ds_m=line.ds_m,
            overrides=overrides,
            conditions=conditions,
        )
    else:
        lap = solve_lap(
            vehicle_dir, line, ds_m=ds_m, overrides=overrides, conditions=conditions
        )
    return lap_dataset(lap)


def track_dataset(track: Track, ds_m: float = 10.0) -> xr.Dataset:
    """Sample a track ribbon into a labelled dataset over ``s`` (positions, curvature, banking)."""
    m = track.sample(ds_m)
    data = {
        name: ("s", m[name], attrs)
        for name, attrs in (
            ("x", {"units": "m"}),
            ("y", {"units": "m"}),
            ("z", {"units": "m", "long_name": "elevation"}),
            ("kappa_h", {"units": "1/m", "long_name": "plan-view curvature"}),
            ("kappa_v", {"units": "1/m", "long_name": "vertical curvature"}),
            ("grade", {"units": "rad"}),
            ("banking", {"units": "rad"}),
            ("width_left", {"units": "m"}),
            ("width_right", {"units": "m"}),
        )
    }
    return xr.Dataset(
        data,
        coords={"s": ("s", m["s"], {"units": "m", "long_name": "arc length"})},
        attrs={
            "name": track.name(),
            "length_m": track.length(),
            # int, not bool: netCDF attrs have no bool type.
            "closed": int(track.is_closed()),
        },
    )
