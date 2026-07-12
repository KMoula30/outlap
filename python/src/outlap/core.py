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
    Envelope,
    Lap,
    Raceline,
    Track,
    TransientLap,
    Tyre,
    min_curvature,
    solve_lap,
    solve_transient_lap,
    time_weighted,
    vehicle_report,
)

__all__ = [
    "DEFAULT_DS_M",
    "Envelope",
    "Lap",
    "Raceline",
    "Track",
    "TransientLap",
    "TyreForces",
    "Tyre",
    "lap_dataset",
    "min_curvature",
    "solve_lap",
    "solve_lap_dataset",
    "solve_transient_lap",
    "time_weighted",
    "track_dataset",
    "transient_lap_dataset",
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
    """Convert a solved lap into a labelled :class:`xarray.Dataset`.

    Every lap carries the point-mass channels over the ``s`` (arc-length) dimension. A ``t1`` lap
    additionally carries the per-wheel channels over ``(s, wheel)`` (``wheel`` = FL/FR/RL/RR), the
    setup metrics, and — when a coupled electrified stack was active — the slow-state channels. A
    ``t0`` lap stays ``s``-only (backward-compatible). The resolved ``tier``, ``fz_coupling`` and
    ``flat_track`` are recorded in ``attrs``.
    """
    s = lap.s()
    data: dict[str, object] = {
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
    }
    coords: dict[str, object] = {
        "s": ("s", s, {"units": "m", "long_name": "arc length"})
    }

    # Per-wheel channels (t1 only): dims (s, wheel) with wheel = FL/FR/RL/RR.
    fz = lap.vertical_load_n()
    if fz is not None:
        coords["wheel"] = (
            "wheel",
            list(lap.wheels),
            {"long_name": "wheel (FL, FR, RL, RR)"},
        )
        data["vertical_load_n"] = (
            ("s", "wheel"),
            fz,
            {"units": "N", "long_name": "normal load"},
        )
        data["slip_ratio"] = (
            ("s", "wheel"),
            lap.slip_ratio(),
            {"units": "1", "long_name": "longitudinal slip ratio κ"},
        )
        data["slip_angle_rad"] = (
            ("s", "wheel"),
            lap.slip_angle_rad(),
            {"units": "rad", "long_name": "slip angle α"},
        )
        data["force_long_n"] = (
            ("s", "wheel"),
            lap.force_long_n(),
            {"units": "N", "long_name": "longitudinal tyre force Fx"},
        )
        data["force_lat_n"] = (
            ("s", "wheel"),
            lap.force_lat_n(),
            {"units": "N", "long_name": "lateral tyre force Fy"},
        )

    # Setup metrics (t1 only).
    ug = lap.understeer_gradient()
    if ug is not None:
        data["understeer_gradient"] = (
            "s",
            ug,
            {"units": "rad·s²/m", "long_name": "understeer gradient K"},
        )
        data["aero_front_share"] = (
            "s",
            lap.aero_front_share(),
            {"units": "1", "long_name": "front axle downforce share"},
        )

    # Slow-state channels (only when a coupled electrified stack was active).
    soc = lap.state_of_charge()
    if soc is not None:
        data["state_of_charge"] = (
            "s",
            soc,
            {"units": "1", "long_name": "pack state of charge"},
        )
        data["machine_temp_c"] = (
            "s",
            lap.machine_temp_c(),
            {"units": "°C", "long_name": "machine winding temperature"},
        )

    return xr.Dataset(
        data,
        coords=coords,
        attrs={
            "lap_time_s": lap.lap_time_s,
            "resolved_hash": lap.resolved_hash,
            "tier": lap.tier,
            "fz_coupling": lap.fz_coupling,
            # int, not bool: netCDF attrs have no bool type.
            "flat_track": int(lap.flat_track),
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
    tier: str | None = None,
    sim: dict[str, object] | None = None,
    overrides: dict[str, bool | int | float | str] | None = None,
    conditions: dict[str, object] | None = None,
) -> xr.Dataset:
    """Solve a QSS lap and return it directly as a labelled dataset (see :func:`lap_dataset`).

    ``line`` may be a plain :class:`Track` (a lap of its centerline) or a :class:`Raceline`
    (a lap of the generated line, with its generation step recorded in the result provenance).

    ``tier`` (``"t0"``/``"t1"``) and ``sim`` (a nested override dict, e.g.
    ``{"flat_track": True, "envelope": {"v_points": 24}}``) select and configure the solver over
    the vehicle-dir ``sim.yaml`` (or defaults); ``tier`` wins over ``sim["tier"]``.

    What-if experiments: ``overrides`` patches dotted paths onto the vehicle through the real
    validation pipeline (e.g. ``{"chassis.mass_kg": 750.0}``), and ``conditions`` deep-merges
    onto the session conditions (e.g. ``{"air": {"temp_c": 35.0}}``) — invalid paths or types
    fail loudly, never silently.

    ``tier="t2"`` runs the transient tier instead, returning the **time-indexed** dataset of
    :func:`transient_lap_dataset` (dims ``time``/``wheel``) rather than the arc-length one. It takes
    the same arguments; ``speed_margin`` is available on :func:`solve_transient_lap` directly.
    """
    if isinstance(line, Raceline):
        track, raceline_ds_m = line.line(), line.ds_m
        raceline_generator, raceline_iterations = line.generator, line.iterations
    else:
        track, raceline_ds_m = line, None
        raceline_generator, raceline_iterations = None, None
    if tier == "t2":
        return transient_lap_dataset(
            solve_transient_lap(
                vehicle_dir,
                track,
                ds_m=ds_m,
                raceline_ds_m=raceline_ds_m,
                raceline_generator=raceline_generator,
                raceline_iterations=raceline_iterations,
                overrides=overrides,
                conditions=conditions,
                sim=sim,
            )
        )
    lap = solve_lap(
        vehicle_dir,
        track,
        ds_m=ds_m,
        raceline_ds_m=raceline_ds_m,
        raceline_generator=raceline_generator,
        raceline_iterations=raceline_iterations,
        overrides=overrides,
        conditions=conditions,
        tier=tier,
        sim=sim,
    )
    return lap_dataset(lap)


def transient_lap_dataset(lap: TransientLap) -> xr.Dataset:
    """Convert a solved transient (T2) lap into a labelled :class:`xarray.Dataset`.

    The primary dimension is ``time`` (a fixed ``dt`` grid), not arc length: a transient lap is
    integrated in time, and ``s`` is a data variable that advances along it (and wraps past the
    start/finish on a closed line). Per-wheel channels carry dims ``("time", "wheel")`` with
    ``wheel = FL/FR/RL/RR``.

    Rule-based control-layer telemetry rides along: the engaged ``gear`` and the shift
    ``torque_scale``, the realised torque-vectoring ``yaw_moment_nm``, the recovered
    ``regen_power_w`` and the per-axle machine braking torques (the friction brakes supplied the
    rest of each axle's commanded torque), and — when the car carries a battery — the pack
    ``state_of_charge`` and ``pack_temp_c``.
    """
    t = lap.t()
    data: dict[str, object] = {
        "s": ("time", lap.s(), {"units": "m", "long_name": "arc length"}),
        "n": ("time", lap.n(), {"units": "m", "long_name": "lateral offset (+left)"}),
        "psi_rel": (
            "time",
            lap.psi_rel(),
            {"units": "rad", "long_name": "heading relative to the road tangent"},
        ),
        "vx": (
            "time",
            lap.vx(),
            {"units": "m/s", "long_name": "longitudinal velocity"},
        ),
        "vy": (
            "time",
            lap.vy(),
            {"units": "m/s", "long_name": "lateral velocity (+left)"},
        ),
        "yaw_rate": (
            "time",
            lap.yaw_rate(),
            {"units": "rad/s", "long_name": "yaw rate (+CCW)"},
        ),
        "ax": (
            "time",
            lap.ax(),
            {"units": "m/s²", "long_name": "longitudinal acceleration"},
        ),
        "ay": (
            "time",
            lap.ay(),
            {"units": "m/s²", "long_name": "lateral acceleration (+left)"},
        ),
        "steer": (
            "time",
            lap.steer(),
            {"units": "rad", "long_name": "road-wheel steer"},
        ),
        "throttle": ("time", lap.throttle(), {"units": "1"}),
        "brake": ("time", lap.brake(), {"units": "1"}),
        "x": ("time", lap.x(), {"units": "m"}),
        "y": ("time", lap.y(), {"units": "m"}),
        "z": ("time", lap.z(), {"units": "m", "long_name": "elevation"}),
        "gear": ("time", lap.gear(), {"units": "1", "long_name": "engaged gear index"}),
        "torque_scale": (
            "time",
            lap.torque_scale(),
            {
                "units": "1",
                "long_name": "drive-torque scale (shift torque interruption)",
            },
        ),
        "yaw_moment_nm": (
            "time",
            lap.yaw_moment_nm(),
            {
                "units": "N·m",
                "long_name": "realised torque-vectoring yaw moment (+CCW)",
            },
        ),
        "regen_power_w": (
            "time",
            lap.regen_power_w(),
            {"units": "W", "long_name": "recovered electrical regen power"},
        ),
        "traction_power_w": (
            "time",
            lap.traction_power_w(),
            {
                "units": "W",
                "long_name": "electrical traction power drawn from the pack",
            },
        ),
        "regen_torque_front_nm": (
            "time",
            lap.regen_torque_front_nm(),
            {"units": "N·m", "long_name": "front-axle machine braking torque"},
        ),
        "regen_torque_rear_nm": (
            "time",
            lap.regen_torque_rear_nm(),
            {"units": "N·m", "long_name": "rear-axle machine braking torque"},
        ),
        "omega": (
            ("time", "wheel"),
            lap.omega(),
            {"units": "rad/s", "long_name": "wheel angular speed"},
        ),
        "vertical_load_n": (
            ("time", "wheel"),
            lap.vertical_load_n(),
            {"units": "N", "long_name": "normal load"},
        ),
        "slip_ratio": (
            ("time", "wheel"),
            lap.slip_ratio(),
            {"units": "1", "long_name": "longitudinal slip ratio κ (lagged)"},
        ),
        "slip_angle_rad": (
            ("time", "wheel"),
            lap.slip_angle_rad(),
            {"units": "rad", "long_name": "slip angle α (lagged)"},
        ),
        "force_long_n": (
            ("time", "wheel"),
            lap.force_long_n(),
            {"units": "N", "long_name": "longitudinal tyre force Fx"},
        ),
        "force_lat_n": (
            ("time", "wheel"),
            lap.force_lat_n(),
            {"units": "N", "long_name": "lateral tyre force Fy"},
        ),
    }
    coords: dict[str, object] = {
        "time": ("time", t, {"units": "s", "long_name": "time since lap start"}),
        "wheel": ("wheel", list(lap.wheels), {"long_name": "wheel (FL, FR, RL, RR)"}),
    }

    soc = lap.state_of_charge()
    if soc is not None:
        data["state_of_charge"] = (
            "time",
            soc,
            {"units": "1", "long_name": "pack state of charge"},
        )
        data["pack_temp_c"] = (
            "time",
            lap.pack_temp_c(),
            {"units": "°C", "long_name": "pack temperature"},
        )

    return xr.Dataset(
        data,
        coords=coords,
        attrs={
            "lap_time_s": lap.lap_time_s,
            "resolved_hash": lap.resolved_hash,
            "tier": lap.tier,
            "fz_coupling": lap.fz_coupling,
            "dt_s": lap.dt_s,
            "integrator_order": lap.integrator_order,
            "speed_margin": lap.speed_margin,
            # int, not bool: netCDF attrs have no bool type.
            "flat_track": int(lap.flat_track),
            "completed": int(lap.completed),
            "notes": tuple(lap.notes),
        },
    )


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
