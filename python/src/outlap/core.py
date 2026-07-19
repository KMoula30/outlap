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
    QssStint,
    Raceline,
    Track,
    TransientLap,
    TransientStint,
    Tyre,
    min_curvature,
    solve_lap,
    solve_stint,
    solve_transient_lap,
    solve_transient_stint,
    time_weighted,
    vehicle_report,
)

__all__ = [
    "CHANNEL_ATTRS",
    "DEFAULT_DS_M",
    "Envelope",
    "Lap",
    "QssStint",
    "Raceline",
    "Track",
    "TransientLap",
    "TransientStint",
    "TyreForces",
    "Tyre",
    "lap_dataset",
    "min_curvature",
    "solve_lap",
    "solve_lap_dataset",
    "solve_stint",
    "solve_stint_dataset",
    "solve_transient_lap",
    "solve_transient_stint",
    "stint_dataset",
    "time_weighted",
    "track_dataset",
    "transient_lap_dataset",
    "transient_stint_dataset",
    "tyre_forces",
    "vehicle_report",
]

#: The ONE units/long_name table every dataset builder consumes (a channel means the same thing
#: whichever artifact carries it). Builders may prefix the ``long_name`` for per-lap summaries
#: (``end-of-lap …``) or suffix it for tier-specific caveats (``… (lagged)``) via :func:`_attrs`.
CHANNEL_ATTRS: dict[str, dict[str, str]] = {
    # Coordinates / geometry.
    "s": {"units": "m", "long_name": "arc length"},
    "time": {"units": "s", "long_name": "time since lap start"},
    "lap": {"long_name": "lap number"},
    "wheel": {"long_name": "wheel (FL, FR, RL, RR)"},
    "x": {"units": "m"},
    "y": {"units": "m"},
    "z": {"units": "m", "long_name": "elevation"},
    # Point-mass / chassis channels.
    "v": {"units": "m/s", "long_name": "speed"},
    "ax": {"units": "m/s²", "long_name": "longitudinal acceleration"},
    "ay": {"units": "m/s²", "long_name": "lateral acceleration (+left)"},
    "t": {"units": "s", "long_name": "cumulative time"},
    "lap_time_s": {"units": "s", "long_name": "lap time"},
    "n": {"units": "m", "long_name": "lateral offset (+left)"},
    "psi_rel": {"units": "rad", "long_name": "heading relative to the road tangent"},
    "vx": {"units": "m/s", "long_name": "longitudinal velocity"},
    "vy": {"units": "m/s", "long_name": "lateral velocity (+left)"},
    "yaw_rate": {"units": "rad/s", "long_name": "yaw rate (+CCW)"},
    "steer": {"units": "rad", "long_name": "road-wheel steer"},
    "throttle": {"units": "1"},
    "brake": {"units": "1"},
    # Per-wheel tyre channels.
    "omega": {"units": "rad/s", "long_name": "wheel angular speed"},
    "vertical_load_n": {"units": "N", "long_name": "normal load"},
    "slip_ratio": {"units": "1", "long_name": "longitudinal slip ratio κ"},
    "slip_angle_rad": {"units": "rad", "long_name": "slip angle α"},
    "force_long_n": {"units": "N", "long_name": "longitudinal tyre force Fx"},
    "force_lat_n": {"units": "N", "long_name": "lateral tyre force Fy"},
    # Setup metrics.
    "understeer_gradient": {"units": "rad·s²/m", "long_name": "understeer gradient K"},
    "aero_front_share": {"units": "1", "long_name": "front axle downforce share"},
    # Control-layer telemetry.
    "gear": {"units": "1", "long_name": "engaged gear index"},
    "torque_scale": {
        "units": "1",
        "long_name": "drive-torque scale (shift torque interruption)",
    },
    "yaw_moment_nm": {
        "units": "N·m",
        "long_name": "realised torque-vectoring yaw moment (+CCW)",
    },
    # Energy channels.
    "regen_power_w": {"units": "W", "long_name": "recovered electrical regen power"},
    "traction_power_w": {
        "units": "W",
        "long_name": "electrical traction power drawn from the pack",
    },
    "regen_torque_front_nm": {
        "units": "N·m",
        "long_name": "front-axle machine braking torque",
    },
    "regen_torque_rear_nm": {
        "units": "N·m",
        "long_name": "rear-axle machine braking torque",
    },
    "deploy_power_w": {
        "units": "W",
        "long_name": "ERS electrical deployment power (realized)",
    },
    "harvest_power_w": {
        "units": "W",
        "long_name": "ERS electrical harvest power (realized, all Recharge paths)",
    },
    "state_of_charge": {"units": "1", "long_name": "pack state of charge"},
    "machine_temp_c": {"units": "°C", "long_name": "machine winding temperature"},
    "pack_temp_c": {"units": "°C", "long_name": "pack temperature"},
    "fuel_mass_kg": {"units": "kg", "long_name": "on-board fuel mass"},
    # Tyre-thermal slow-state channels (the reduced Farroni-TRT ring + Archard wear).
    "tire_surface_c": {
        "units": "°C",
        "long_name": "tyre tread-surface temperature T_s",
    },
    "tire_peak_surface_c": {
        "units": "°C",
        "long_name": "peak tyre tread-surface temperature over the lap",
    },
    "tire_carcass_c": {
        "units": "°C",
        "long_name": "tyre carcass (bulk) temperature T_c",
    },
    "tire_gas_c": {"units": "°C", "long_name": "tyre inflation-gas temperature T_g"},
    "tire_wear_mm": {"units": "mm", "long_name": "tyre tread wear depth w"},
    "tire_damage": {"units": "1", "long_name": "tyre irreversible thermal damage D"},
    "tire_grip": {"units": "1", "long_name": "tyre total grip multiplier λ_μ,total"},
    # Track ribbon channels.
    "kappa_h": {"units": "1/m", "long_name": "plan-view curvature"},
    "kappa_v": {"units": "1/m", "long_name": "vertical curvature"},
    "grade": {"units": "rad"},
    "banking": {"units": "rad"},
    "width_left": {"units": "m"},
    "width_right": {"units": "m"},
}


def _attrs(name: str, *, prefix: str = "", suffix: str = "") -> dict[str, str]:
    """The registry attrs for ``name``, with an optional ``long_name`` prefix/suffix.

    ``prefix`` marks per-lap summaries (``"end-of-lap "``), ``suffix`` tier-specific caveats
    (``" (lagged)"``); a channel with no ``long_name`` ignores both.
    """
    attrs = dict(CHANNEL_ATTRS[name])
    if (prefix or suffix) and "long_name" in attrs:
        attrs["long_name"] = f"{prefix}{attrs['long_name']}{suffix}"
    return attrs


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
    setup metrics, and — when a coupled electrified stack was active — the slow-state channels
    (plus the realized ERS deploy/harvest powers when the 2026 energy manager governed the lap). A
    ``t0`` lap stays ``s``-only (backward-compatible). The resolved ``tier``, ``fz_coupling`` and
    ``flat_track`` are recorded in ``attrs``.
    """
    s = lap.s()
    data: dict[str, object] = {
        "v": ("s", lap.v(), _attrs("v")),
        "ax": ("s", lap.ax(), _attrs("ax")),
        "ay": ("s", lap.ay(), _attrs("ay")),
        "t": ("s", lap.t(), _attrs("t")),
        "x": ("s", lap.x(), _attrs("x")),
        "y": ("s", lap.y(), _attrs("y")),
        "z": ("s", lap.z(), _attrs("z")),
    }
    coords: dict[str, object] = {"s": ("s", s, _attrs("s"))}

    # Per-wheel channels (t1 only): dims (s, wheel) with wheel = FL/FR/RL/RR.
    fz = lap.vertical_load_n()
    if fz is not None:
        coords["wheel"] = ("wheel", list(lap.wheels), _attrs("wheel"))
        data["vertical_load_n"] = (("s", "wheel"), fz, _attrs("vertical_load_n"))
        data["slip_ratio"] = (("s", "wheel"), lap.slip_ratio(), _attrs("slip_ratio"))
        data["slip_angle_rad"] = (
            ("s", "wheel"),
            lap.slip_angle_rad(),
            _attrs("slip_angle_rad"),
        )
        data["force_long_n"] = (
            ("s", "wheel"),
            lap.force_long_n(),
            _attrs("force_long_n"),
        )
        data["force_lat_n"] = (("s", "wheel"), lap.force_lat_n(), _attrs("force_lat_n"))

    # Setup metrics (t1 only).
    ug = lap.understeer_gradient()
    if ug is not None:
        data["understeer_gradient"] = ("s", ug, _attrs("understeer_gradient"))
        data["aero_front_share"] = (
            "s",
            lap.aero_front_share(),
            _attrs("aero_front_share"),
        )

    # Slow-state channels (only when a coupled electrified stack was active). The machine
    # temperature is independently gated: a pack may march without a thermal network (M6 PR2).
    soc = lap.state_of_charge()
    if soc is not None:
        data["state_of_charge"] = ("s", soc, _attrs("state_of_charge"))
    fuel = lap.fuel_mass_kg()
    if fuel is not None:
        data["fuel_mass_kg"] = ("s", fuel, _attrs("fuel_mass_kg"))
    machine_temp = lap.machine_temp_c()
    if machine_temp is not None:
        data["machine_temp_c"] = ("s", machine_temp, _attrs("machine_temp_c"))

    # ERS energy-manager channels (only when the 2026 manager governed the march, M6 PR2).
    deploy = lap.deploy_power_w()
    if deploy is not None:
        data["deploy_power_w"] = ("s", deploy, _attrs("deploy_power_w"))
        data["harvest_power_w"] = (
            "s",
            lap.harvest_power_w(),
            _attrs("harvest_power_w"),
        )

    # Tyre-thermal slow-state channels (only when `tire_thermal=True` opted the march in). The
    # representative front tyre's reduced Farroni-TRT ring + Archard wear marched along the profile.
    tire_surface = lap.tire_surface_c()
    if tire_surface is not None:
        data["tire_surface_c"] = ("s", tire_surface, _attrs("tire_surface_c"))
        data["tire_carcass_c"] = ("s", lap.tire_carcass_c(), _attrs("tire_carcass_c"))
        data["tire_gas_c"] = ("s", lap.tire_gas_c(), _attrs("tire_gas_c"))
        data["tire_wear_mm"] = ("s", lap.tire_wear_mm(), _attrs("tire_wear_mm"))
        data["tire_damage"] = ("s", lap.tire_damage(), _attrs("tire_damage"))
        data["tire_grip"] = ("s", lap.tire_grip(), _attrs("tire_grip"))

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
    tire_thermal: bool = False,
    initial_soc: float | None = None,
    speed_margin: float | None = None,
    override: bool = False,
    us_schedule: dict[str, object] | None = None,
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

    This wrapper accepts the **union** of the QSS and transient lap kwargs and forwards each to
    whichever solver the resolved tier selects: ``tire_thermal``, ``initial_soc`` (the starting
    pack state of charge), and the 2026 ERS controls ``override`` (enable the Overtake envelope +
    the extra harvest allowance) and ``us_schedule`` (a ``u(s)`` control schedule) apply to both,
    while ``speed_margin`` is transient-only. A kwarg the resolved tier cannot honour raises
    :class:`ValueError` rather than being silently dropped.

    ``tier="t2"`` runs the transient tier instead, returning the **time-indexed** dataset of
    :func:`transient_lap_dataset` (dims ``time``/``wheel``) rather than the arc-length one.
    """
    if isinstance(line, Raceline):
        track, raceline_ds_m = line.line(), line.ds_m
        raceline_generator, raceline_iterations = line.generator, line.iterations
    else:
        track, raceline_ds_m = line, None
        raceline_generator, raceline_iterations = None, None
    if tier == "t2":
        margin_kw = {} if speed_margin is None else {"speed_margin": speed_margin}
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
                initial_soc=initial_soc,
                tire_thermal=tire_thermal,
                override=override,
                us_schedule=us_schedule,
                **margin_kw,
            )
        )
    if speed_margin is not None:
        raise ValueError(
            "speed_margin applies only to the transient tier (tier='t2'); the QSS tiers "
            "size their speed margin from the g-g-g-v envelope"
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
        tire_thermal=tire_thermal,
        initial_soc=initial_soc,
        override=override,
        us_schedule=us_schedule,
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
        "s": ("time", lap.s(), _attrs("s")),
        "n": ("time", lap.n(), _attrs("n")),
        "psi_rel": ("time", lap.psi_rel(), _attrs("psi_rel")),
        "vx": ("time", lap.vx(), _attrs("vx")),
        "vy": ("time", lap.vy(), _attrs("vy")),
        "yaw_rate": ("time", lap.yaw_rate(), _attrs("yaw_rate")),
        "ax": ("time", lap.ax(), _attrs("ax")),
        "ay": ("time", lap.ay(), _attrs("ay")),
        "steer": ("time", lap.steer(), _attrs("steer")),
        "throttle": ("time", lap.throttle(), _attrs("throttle")),
        "brake": ("time", lap.brake(), _attrs("brake")),
        "x": ("time", lap.x(), _attrs("x")),
        "y": ("time", lap.y(), _attrs("y")),
        "z": ("time", lap.z(), _attrs("z")),
        "gear": ("time", lap.gear(), _attrs("gear")),
        "torque_scale": ("time", lap.torque_scale(), _attrs("torque_scale")),
        "yaw_moment_nm": ("time", lap.yaw_moment_nm(), _attrs("yaw_moment_nm")),
        "regen_power_w": ("time", lap.regen_power_w(), _attrs("regen_power_w")),
        "traction_power_w": (
            "time",
            lap.traction_power_w(),
            _attrs("traction_power_w"),
        ),
        "regen_torque_front_nm": (
            "time",
            lap.regen_torque_front_nm(),
            _attrs("regen_torque_front_nm"),
        ),
        "regen_torque_rear_nm": (
            "time",
            lap.regen_torque_rear_nm(),
            _attrs("regen_torque_rear_nm"),
        ),
        "omega": (("time", "wheel"), lap.omega(), _attrs("omega")),
        "vertical_load_n": (
            ("time", "wheel"),
            lap.vertical_load_n(),
            _attrs("vertical_load_n"),
        ),
        "slip_ratio": (
            ("time", "wheel"),
            lap.slip_ratio(),
            _attrs("slip_ratio", suffix=" (lagged)"),
        ),
        "slip_angle_rad": (
            ("time", "wheel"),
            lap.slip_angle_rad(),
            _attrs("slip_angle_rad", suffix=" (lagged)"),
        ),
        "force_long_n": (
            ("time", "wheel"),
            lap.force_long_n(),
            _attrs("force_long_n"),
        ),
        "force_lat_n": (("time", "wheel"), lap.force_lat_n(), _attrs("force_lat_n")),
    }
    coords: dict[str, object] = {
        "time": ("time", t, _attrs("time")),
        "wheel": ("wheel", list(lap.wheels), _attrs("wheel")),
    }

    soc = lap.state_of_charge()
    if soc is not None:
        data["state_of_charge"] = ("time", soc, _attrs("state_of_charge"))
        data["pack_temp_c"] = ("time", lap.pack_temp_c(), _attrs("pack_temp_c"))

    # Per-wheel tyre-thermal channels (only when the M5 tyre-thermal stack was attached,
    # `tire_thermal=True`): the reduced Farroni-TRT ring + Archard wear stepped on the slow clock.
    tire_surface = lap.tire_surface_c()
    if tire_surface is not None:
        data["tire_surface_c"] = (
            ("time", "wheel"),
            tire_surface,
            _attrs("tire_surface_c"),
        )
        data["tire_carcass_c"] = (
            ("time", "wheel"),
            lap.tire_carcass_c(),
            _attrs("tire_carcass_c"),
        )
        data["tire_gas_c"] = (("time", "wheel"), lap.tire_gas_c(), _attrs("tire_gas_c"))
        data["tire_wear_mm"] = (
            ("time", "wheel"),
            lap.tire_wear_mm(),
            _attrs("tire_wear_mm"),
        )
        data["tire_damage"] = (
            ("time", "wheel"),
            lap.tire_damage(),
            _attrs("tire_damage"),
        )
        data["tire_grip"] = (("time", "wheel"), lap.tire_grip(), _attrs("tire_grip"))

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
    names = (
        "x",
        "y",
        "z",
        "kappa_h",
        "kappa_v",
        "grade",
        "banking",
        "width_left",
        "width_right",
    )
    data = {name: ("s", m[name], _attrs(name)) for name in names}
    return xr.Dataset(
        data,
        coords={"s": ("s", m["s"], _attrs("s"))},
        attrs={
            "name": track.name(),
            "length_m": track.length(),
            # int, not bool: netCDF attrs have no bool type.
            "closed": int(track.is_closed()),
        },
    )


def stint_dataset(stint: QssStint) -> xr.Dataset:
    """Convert a solved QSS stint into a labelled ``(lap, s)`` :class:`xarray.Dataset`.

    Every lap shares the arc-length station grid (the same line), so the whole stint is one clean
    ``(lap, s)`` block. The per-lap ``lap_time_s`` is the headline — monotone pace loss as the tyres
    wear — and, when the tyre march was on, the representative front tyre's ``T_s``/``T_c``/``T_g``,
    wear, damage, and total grip multiplier evolve over both axes: warm-up along ``s`` on lap 1, and
    degradation across ``lap`` as wear accumulates with no reset at the lap boundary.
    """
    laps = np.arange(1, stint.n_laps + 1)
    data: dict[str, object] = {
        "lap_time_s": ("lap", stint.lap_time_s(), _attrs("lap_time_s")),
        "v": (("lap", "s"), stint.v(), _attrs("v")),
    }
    coords: dict[str, object] = {
        "lap": ("lap", laps, _attrs("lap")),
        "s": ("s", stint.s(), _attrs("s")),
    }

    tire_surface = stint.tire_surface_c()
    if tire_surface is not None:
        data["tire_surface_c"] = (("lap", "s"), tire_surface, _attrs("tire_surface_c"))
        data["tire_carcass_c"] = (
            ("lap", "s"),
            stint.tire_carcass_c(),
            _attrs("tire_carcass_c"),
        )
        data["tire_gas_c"] = (("lap", "s"), stint.tire_gas_c(), _attrs("tire_gas_c"))
        data["tire_wear_mm"] = (
            ("lap", "s"),
            stint.tire_wear_mm(),
            _attrs("tire_wear_mm"),
        )
        data["tire_damage"] = (("lap", "s"), stint.tire_damage(), _attrs("tire_damage"))
        data["tire_grip"] = (("lap", "s"), stint.tire_grip(), _attrs("tire_grip"))

    # The electrified slow stack, when present: the pack SoC trace carries continuously across the
    # lap boundary (M6 PR3 — it falls with net consumption, rises with regeneration), and the
    # end-of-lap pack (and machine) temperature ride a `lap` axis.
    soc = stint.state_of_charge()
    if soc is not None:
        data["state_of_charge"] = (("lap", "s"), soc, _attrs("state_of_charge"))
        data["pack_temp_c"] = (
            "lap",
            stint.pack_temp_c(),
            _attrs("pack_temp_c", prefix="end-of-lap "),
        )
    fuel = stint.fuel_mass_kg()
    if fuel is not None:
        data["fuel_mass_kg"] = (("lap", "s"), fuel, _attrs("fuel_mass_kg"))
        machine = stint.machine_temp_c()
        if machine is not None:
            data["machine_temp_c"] = (
                "lap",
                machine,
                _attrs("machine_temp_c", prefix="end-of-lap "),
            )

    return xr.Dataset(
        data,
        coords=coords,
        attrs={
            "tier": stint.tier,
            "resolved_hash": stint.resolved_hash,
            "fz_coupling": stint.fz_coupling,
            # int, not bool: netCDF attrs have no bool type.
            "flat_track": int(stint.flat_track),
            "n_laps": stint.n_laps,
            "notes": tuple(stint.notes),
        },
    )


def transient_stint_dataset(stint: TransientStint) -> xr.Dataset:
    """Convert a solved transient (T2) stint into a ``lap``-indexed summary :class:`xarray.Dataset`.

    A T2 stint integrates continuously — a variable number of fixed steps per lap — so it surfaces
    per-lap **summaries** rather than a time series: the per-lap ``lap_time_s``, the per-wheel
    end-of-lap and peak tyre state, and the end-of-lap pack state. The slow states (per-wheel
    tyre-thermal ring + wear, battery SoC) carry across the start/finish line with no reset.
    """
    laps = np.arange(1, stint.n_laps + 1)
    data: dict[str, object] = {
        "lap_time_s": ("lap", stint.lap_time_s(), _attrs("lap_time_s")),
    }
    coords: dict[str, object] = {"lap": ("lap", laps, _attrs("lap"))}

    eol = "end-of-lap "
    tire_surface = stint.tire_surface_c()
    if tire_surface is not None:
        coords["wheel"] = ("wheel", list(stint.wheels), _attrs("wheel"))
        data["tire_surface_c"] = (
            ("lap", "wheel"),
            tire_surface,
            _attrs("tire_surface_c", prefix=eol),
        )
        data["tire_peak_surface_c"] = (
            ("lap", "wheel"),
            stint.tire_peak_surface_c(),
            _attrs("tire_peak_surface_c"),
        )
        data["tire_carcass_c"] = (
            ("lap", "wheel"),
            stint.tire_carcass_c(),
            _attrs("tire_carcass_c", prefix=eol),
        )
        data["tire_gas_c"] = (
            ("lap", "wheel"),
            stint.tire_gas_c(),
            _attrs("tire_gas_c", prefix=eol),
        )
        data["tire_wear_mm"] = (
            ("lap", "wheel"),
            stint.tire_wear_mm(),
            _attrs("tire_wear_mm", prefix=eol),
        )
        data["tire_damage"] = (
            ("lap", "wheel"),
            stint.tire_damage(),
            _attrs("tire_damage", prefix=eol),
        )
        data["tire_grip"] = (
            ("lap", "wheel"),
            stint.tire_grip(),
            _attrs("tire_grip", prefix=eol),
        )

    soc = stint.state_of_charge()
    if soc is not None:
        data["state_of_charge"] = (
            "lap",
            soc,
            _attrs("state_of_charge", prefix=eol),
        )
        data["pack_temp_c"] = (
            "lap",
            stint.pack_temp_c(),
            _attrs("pack_temp_c", prefix=eol),
        )

    return xr.Dataset(
        data,
        coords=coords,
        attrs={
            "tier": stint.tier,
            "resolved_hash": stint.resolved_hash,
            "fz_coupling": stint.fz_coupling,
            "dt_s": stint.dt_s,
            "integrator_order": stint.integrator_order,
            "speed_margin": stint.speed_margin,
            # int, not bool: netCDF attrs have no bool type.
            "flat_track": int(stint.flat_track),
            "completed": int(stint.completed),
            "requested_laps": stint.requested_laps,
            "n_laps": stint.n_laps,
            "notes": tuple(stint.notes),
        },
    )


def solve_stint_dataset(
    vehicle_dir: str,
    line: Track | Raceline,
    *,
    n_laps: int,
    ds_m: float = DEFAULT_DS_M,
    tier: str | None = None,
    sim: dict[str, object] | None = None,
    overrides: dict[str, bool | int | float | str] | None = None,
    conditions: dict[str, object] | None = None,
    tire_thermal: bool = True,
    initial_tire_temp_c: float | None = None,
    initial_soc: float | None = None,
    speed_margin: float | None = None,
    override: bool = False,
    us_schedule: dict[str, object] | None = None,
) -> xr.Dataset:
    """Solve a multi-lap **stint** and return it as a labelled dataset.

    The tyre-thermal slow state AND the battery pack (SoC / RC voltage / temperature) carry across
    every lap boundary in **both** tiers (M6 PR3), so the tyres warm/wear/degrade and the SoC falls
    with net consumption and rises with regeneration over the run. ``initial_tire_temp_c`` seeds the
    tyres cold-uniform (the out-lap warm-up); ``initial_soc`` seeds the pack (default: the middle of
    its usable window).

    This wrapper accepts the **union** of the QSS and transient stint kwargs and forwards each to the
    resolved tier: ``tire_thermal`` / ``initial_tire_temp_c`` / ``initial_soc`` and the 2026 ERS
    controls ``override`` / ``us_schedule`` apply to both, while ``speed_margin`` is transient-only;
    a kwarg the resolved tier cannot honour raises :class:`ValueError` rather than being silently
    dropped.

    ``tier="t2"`` runs the transient tier and returns the per-lap summary of
    :func:`transient_stint_dataset` (dims ``lap``/``wheel``); the QSS tiers (``"t0"``/``"t1"``) return
    the ``(lap, s)`` dataset of :func:`stint_dataset`. ``line`` may be a :class:`Track` (centerline)
    or a generated :class:`Raceline`.
    """
    if isinstance(line, Raceline):
        track, raceline_ds_m = line.line(), line.ds_m
        raceline_generator, raceline_iterations = line.generator, line.iterations
    else:
        track, raceline_ds_m = line, None
        raceline_generator, raceline_iterations = None, None
    if tier == "t2":
        margin_kw = {} if speed_margin is None else {"speed_margin": speed_margin}
        return transient_stint_dataset(
            solve_transient_stint(
                vehicle_dir,
                track,
                n_laps,
                ds_m=ds_m,
                raceline_ds_m=raceline_ds_m,
                raceline_generator=raceline_generator,
                raceline_iterations=raceline_iterations,
                overrides=overrides,
                conditions=conditions,
                sim=sim,
                tire_thermal=tire_thermal,
                initial_tire_temp_c=initial_tire_temp_c,
                initial_soc=initial_soc,
                override=override,
                us_schedule=us_schedule,
                **margin_kw,
            )
        )
    if speed_margin is not None:
        raise ValueError(
            "speed_margin applies only to the transient tier (tier='t2'); the QSS tiers "
            "size their speed margin from the g-g-g-v envelope"
        )
    return stint_dataset(
        solve_stint(
            vehicle_dir,
            track,
            n_laps,
            ds_m=ds_m,
            raceline_ds_m=raceline_ds_m,
            raceline_generator=raceline_generator,
            raceline_iterations=raceline_iterations,
            overrides=overrides,
            conditions=conditions,
            tier=tier,
            sim=sim,
            override=override,
            us_schedule=us_schedule,
            tire_thermal=tire_thermal,
            initial_tire_temp_c=initial_tire_temp_c,
            initial_soc=initial_soc,
        )
    )
