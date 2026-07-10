# SPDX-License-Identifier: AGPL-3.0-only
"""Tests for the transient (T2) tier at the Python boundary.

These pin the *plumbing* the Rust suite cannot see: that a closed-loop lap reaches the finish line
through the real assembly pipeline, that its dataset is time-indexed with `(time, wheel)` per-wheel
channels, that the rule-based control layer's telemetry survives the boundary, and that the
series regen blend behaves at the two ends its physics demands — each machine braking only its own
axle, and a full pack refusing charge entirely.

The physics itself is gated in Rust (`outlap-transient/tests`, `outlap-vehicle`, `outlap-qss`).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import xarray as xr

from outlap.core import (
    DEFAULT_DS_M,
    Track,
    solve_lap,
    solve_lap_dataset,
    solve_transient_lap,
    transient_lap_dataset,
)

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"

CATALUNYA = str(_DATA / "tracks/catalunya_osm")
LIMEBEER = str(_DATA / "vehicles/limebeer_2014_f1")
# The only committed car that carries a battery *and* its ECM sidecar: a rear-drive EV, so the front
# axle has no machine and must recover nothing.
MODEL3_RWD = str(_DATA / "vehicles/tesla_model3_rwd")

# CI-speed envelope: what these tests assert (dataset shape, attrs, regen sign conventions) is
# fidelity-independent, and every distinct override regenerates the g-g-g-v envelope (a cold step).
COARSE_SIM: dict[str, object] = {
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}
}

WHEELS = ["FL", "FR", "RL", "RR"]


@pytest.fixture(scope="module")
def catalunya() -> Track:
    return Track.load(CATALUNYA)


@pytest.fixture(scope="module")
def f1_lap(catalunya: Track) -> xr.Dataset:
    """A transient lap of the reference F1 car (no battery ⇒ no regen channels)."""
    return solve_lap_dataset(
        LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, tier="t2", sim=COARSE_SIM
    )


@pytest.fixture(scope="module")
def ev_lap(catalunya: Track) -> xr.Dataset:
    """A transient lap of the rear-drive EV (battery + regen blend ⇒ the full slow stack)."""
    return transient_lap_dataset(
        solve_transient_lap(MODEL3_RWD, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM)
    )


# --- The dataset contract ------------------------------------------------------------------------


def test_transient_dataset_is_time_indexed(f1_lap: xr.Dataset) -> None:
    """T2 integrates in time, so `time` is the dimension and `s` is a data variable on it."""
    assert "time" in f1_lap.sizes
    assert "s" not in f1_lap.sizes, (
        "arc length is a variable, not a dimension, in a transient lap"
    )
    assert f1_lap["s"].dims == ("time",)
    t = f1_lap["time"].values
    assert (np.diff(t) > 0).all(), "time advances monotonically on a fixed-step grid"
    # A fixed step: every interval is dt.
    assert np.allclose(np.diff(t), f1_lap.attrs["dt_s"])


def test_per_wheel_channels_carry_time_and_wheel_dims(f1_lap: xr.Dataset) -> None:
    assert list(f1_lap.coords["wheel"].values) == WHEELS
    for name in (
        "omega",
        "vertical_load_n",
        "slip_ratio",
        "slip_angle_rad",
        "force_long_n",
        "force_lat_n",
    ):
        assert f1_lap[name].dims == ("time", "wheel"), name
    assert (f1_lap["vertical_load_n"] >= 0.0).all(), "normal loads never go negative"


def test_control_layer_telemetry_crosses_the_boundary(f1_lap: xr.Dataset) -> None:
    for name in (
        "gear",
        "torque_scale",
        "yaw_moment_nm",
        "regen_power_w",
        "regen_torque_front_nm",
        "regen_torque_rear_nm",
    ):
        assert f1_lap[name].dims == ("time",), name
    # No shift FSM is attached yet, so the drive torque is never interrupted.
    assert (f1_lap["torque_scale"] == 1.0).all()
    assert (f1_lap["regen_power_w"] >= 0.0).all(), (
        "regen power is a non-negative magnitude"
    )


def test_provenance_attrs_are_recorded(f1_lap: xr.Dataset) -> None:
    a = f1_lap.attrs
    assert a["tier"] == "t2"
    assert a["fz_coupling"] == "fixed_point", "the T2 default coupling"
    assert a["dt_s"] > 0.0
    assert a["integrator_order"] in (2, 4)
    assert 0.0 < a["speed_margin"] <= 1.0
    assert a["completed"] == 1, "the lap reached the finish line"
    assert isinstance(a["notes"], tuple) and a["notes"], "nothing silent"
    assert a["resolved_hash"]
    # netCDF attrs carry no bool type.
    assert isinstance(a["flat_track"], int)


def test_the_car_completes_a_lap_and_tracks_the_qss_profile(
    catalunya: Track, f1_lap: xr.Dataset
) -> None:
    """The closed loop follows the point-mass reference: it finishes, and near its lap time."""
    t0 = solve_lap_dataset(
        LIMEBEER,
        catalunya,
        ds_m=DEFAULT_DS_M,
        tier="t0",
        sim={"flat_track": True, **COARSE_SIM},
    )
    ratio = f1_lap.attrs["lap_time_s"] / t0.attrs["lap_time_s"]
    # The driver tracks `speed_margin` of the profile and has no active yaw moment enabled on this
    # car, so it is slower — but it must be in the same league, not a different one.
    assert 1.0 < ratio < 1.4, f"T2/T0 lap-time ratio {ratio:.3f}"
    assert (f1_lap["vx"] > 0.0).all(), "the car never stops or reverses"
    assert float(f1_lap["vx"].max()) < 120.0, "and never runs away"


def test_transient_laps_are_deterministic(catalunya: Track) -> None:
    a = solve_transient_lap(LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM)
    b = solve_transient_lap(LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM)
    assert a.lap_time_s == b.lap_time_s
    np.testing.assert_array_equal(a.vx(), b.vx())


# --- Series regen braking, end to end -------------------------------------------------------------


def test_a_car_without_a_battery_has_no_slow_states(f1_lap: xr.Dataset) -> None:
    assert "state_of_charge" not in f1_lap
    assert "pack_temp_c" not in f1_lap
    assert float(f1_lap["regen_power_w"].max()) == 0.0, "nothing to recover into"


def test_each_machine_regens_only_its_own_axle(ev_lap: xr.Dataset) -> None:
    """A rear-drive EV: the front axle has no machine, so it recovers exactly nothing."""
    assert float(ev_lap["regen_torque_front_nm"].max()) == 0.0
    assert float(ev_lap["regen_torque_rear_nm"].max()) > 0.0
    assert float(ev_lap["regen_power_w"].max()) > 0.0


def test_recovered_energy_raises_the_state_of_charge(ev_lap: xr.Dataset) -> None:
    soc = ev_lap["state_of_charge"].values
    assert soc[-1] > soc[0], "braking energy reaches the pack"
    assert (np.diff(soc) >= -1e-12).all(), (
        "the T2 pack only charges (no traction draw yet)"
    )
    # The recovered energy is finite and the pack warms as it takes it.
    recovered_j = float(
        np.trapezoid(ev_lap["regen_power_w"].values, ev_lap["time"].values)
    )
    assert recovered_j > 0.0
    assert float(ev_lap["pack_temp_c"][-1]) >= float(ev_lap["pack_temp_c"][0])


def test_a_full_pack_refuses_charge_and_says_so(catalunya: Track) -> None:
    """Charge acceptance is zero at the top of the SoC window, so the calipers do all the braking —
    correct physics, and surfaced rather than silently producing a dead regen channel."""
    lap = solve_transient_lap(
        MODEL3_RWD, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM, initial_soc=0.98
    )
    ds = transient_lap_dataset(lap)
    assert float(ds["regen_power_w"].max()) == 0.0
    assert float(ds["regen_torque_rear_nm"].max()) == 0.0
    assert any("accept no charge" in n for n in ds.attrs["notes"])


def test_the_default_pack_seed_is_surfaced(ev_lap: xr.Dataset) -> None:
    assert any(
        "state of charge" in n and "estimated" in n for n in ev_lap.attrs["notes"]
    )


def test_initial_soc_is_validated(catalunya: Track) -> None:
    with pytest.raises(ValueError, match=r"initial_soc"):
        solve_transient_lap(MODEL3_RWD, catalunya, sim=COARSE_SIM, initial_soc=1.5)


# --- Tier dispatch and the product surface --------------------------------------------------------


def test_speed_margin_is_validated(catalunya: Track) -> None:
    with pytest.raises(ValueError, match=r"speed_margin"):
        solve_transient_lap(LIMEBEER, catalunya, sim=COARSE_SIM, speed_margin=0.0)


def test_a_three_dimensional_transient_is_refused_not_diverged(
    catalunya: Track,
) -> None:
    """The chassis carries grade/banking terms but the closed loop through them is not gated, so a
    3-D transient is refused outright rather than handed back as a diverged trace."""
    with pytest.raises(ValueError, match="flat-track only"):
        solve_transient_lap(
            LIMEBEER, catalunya, sim={"flat_track": False, **COARSE_SIM}
        )


def test_a_transient_lap_runs_flat_and_records_it(f1_lap: xr.Dataset) -> None:
    assert f1_lap.attrs["flat_track"] == 1
    assert any("flat-track" in n for n in f1_lap.attrs["notes"])


def test_solve_lap_points_at_the_transient_entry_point(catalunya: Track) -> None:
    """`solve_lap` returns an arc-length `Lap`; the transient tier is time-indexed, so it redirects
    rather than raising a bare 'not implemented'."""
    with pytest.raises(ValueError, match="time-indexed"):
        solve_lap(LIMEBEER, catalunya, tier="t2", sim=COARSE_SIM)


def test_solve_lap_dataset_dispatches_t2_to_the_transient_tier(
    f1_lap: xr.Dataset,
) -> None:
    assert f1_lap.attrs["tier"] == "t2"
    assert "time" in f1_lap.sizes
