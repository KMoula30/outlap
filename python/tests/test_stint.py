# SPDX-License-Identifier: AGPL-3.0-only
"""Tests for multi-lap **stints** (M5 PR6) at the Python boundary.

A stint runs `n_laps` laps back-to-back carrying the tyre-thermal slow state (and, in T2, the
battery SoC) across each lap boundary — the §6.1 slow-state continuity that makes the tiers
stint-capable. These pin the plumbing the Rust suite cannot see: that the stint dataset carries a
`lap` axis, that the tyre state is genuinely continuous across the lap boundary (no reset), that a
warm-seeded stint loses pace as the tyres degrade while a cold-seeded one warms up on lap 1, and that
the T2 stint runs one continuous integration.

The reference `.tyr` thermal/wear params are still synthetic (calibration is PR7/PR8), so the
absolute decay magnitude is not asserted — only the sign conventions and the continuity invariant,
which are calibration-independent.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import xarray as xr

from outlap.core import (
    Track,
    solve_stint,
    solve_stint_dataset,
    solve_transient_lap,
    transient_lap_dataset,
)

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"

CATALUNYA = str(_DATA / "tracks/catalunya_osm")
LIMEBEER = str(_DATA / "vehicles/limebeer_2014_f1")

# CI-speed envelope: the assertions (dataset shape, continuity, sign conventions) are
# fidelity-independent, and every distinct override regenerates the g-g-g-v envelope (a cold step).
COARSE: dict[str, object] = {
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}
}
FLAT_COARSE: dict[str, object] = {"flat_track": True, **COARSE}
WHEELS = ["FL", "FR", "RL", "RR"]


@pytest.fixture(scope="module")
def catalunya() -> Track:
    return Track.load(CATALUNYA)


@pytest.fixture(scope="module")
def qss_stint(catalunya: Track) -> xr.Dataset:
    """A warm-seeded 5-lap QSS stint: the tyres degrade over the run (wear + thermal)."""
    return solve_stint_dataset(
        LIMEBEER,
        catalunya,
        n_laps=5,
        tier="t1",
        ds_m=8.0,
        sim=FLAT_COARSE,
        tire_thermal=True,
    )


# --- The stint dataset contract ------------------------------------------------------------------


def test_qss_stint_has_a_lap_axis(qss_stint: xr.Dataset) -> None:
    assert qss_stint.sizes["lap"] == 5
    assert list(qss_stint["lap"].values) == [1, 2, 3, 4, 5]
    assert qss_stint["lap_time_s"].dims == ("lap",)
    assert qss_stint["v"].dims == ("lap", "s")
    for name in (
        "tire_surface_c",
        "tire_carcass_c",
        "tire_gas_c",
        "tire_wear_mm",
        "tire_damage",
        "tire_grip",
    ):
        assert qss_stint[name].dims == ("lap", "s"), name
    assert qss_stint.attrs["tier"] == "t1"
    assert qss_stint.attrs["n_laps"] == 5
    assert isinstance(qss_stint.attrs["notes"], tuple) and qss_stint.attrs["notes"]
    assert any("stint" in n.lower() for n in qss_stint.attrs["notes"])


def test_qss_stint_carries_the_tyre_state_across_laps(qss_stint: xr.Dataset) -> None:
    """The core PR6 invariant: the tyre state carries across the lap boundary — no reset.

    On a closed loop station 0 is the start/finish, so lap k+1's start is lap k's terminal state
    (one march segment past the last recorded station k-1). Continuity therefore holds to within one
    segment; a *reset* would fling every lap-start back to the seed (a large jump), which these
    assertions rule out.
    """
    wear = qss_stint["tire_wear_mm"].values  # (lap, s)
    surf = qss_stint["tire_surface_c"].values
    carcass = qss_stint["tire_carcass_c"].values
    # Wear does not reset to a fresh tyre each lap, and lap-start wear only grows (carries).
    assert (wear[1:, 0] > 0.0).all(), "wear does not reset to a fresh tyre each lap"
    assert (np.diff(wear[:, 0]) >= -1e-9).all(), "lap-start wear carries and grows"
    # Temperatures carry across the boundary (to within one marched segment).
    assert np.allclose(surf[1:, 0], surf[:-1, -1], atol=0.5), (
        "surface carries across the boundary"
    )
    assert np.allclose(carcass[1:, 0], carcass[:-1, -1], atol=0.5), (
        "carcass carries too"
    )


def test_qss_stint_wear_is_monotone_and_pace_degrades(qss_stint: xr.Dataset) -> None:
    # Wear never falls anywhere along the concatenated stint (Archard: sliding energy only adds).
    wear_flat = qss_stint["tire_wear_mm"].values.reshape(-1)
    assert (np.diff(wear_flat) >= -1e-9).all(), "wear is monotone non-decreasing"
    # Warm-seeded, the tyres degrade: pace is lost monotonically and grip does not rise.
    lap_time = qss_stint["lap_time_s"].values
    assert lap_time[-1] > lap_time[0], (
        "the warm-seeded stint loses pace as the tyres degrade"
    )
    assert (np.diff(lap_time) >= -1e-2).all(), "pace loss is monotone (within noise)"
    grip_end = qss_stint["tire_grip"].values[:, -1]
    assert grip_end[-1] <= grip_end[0] + 1e-9, "grip does not rise on a degrading stint"


def test_qss_stint_cold_seed_warms_up_on_lap_1(catalunya: Track) -> None:
    ds = solve_stint_dataset(
        LIMEBEER,
        catalunya,
        n_laps=3,
        tier="t1",
        ds_m=8.0,
        sim=FLAT_COARSE,
        tire_thermal=True,
        initial_tire_temp_c=20.0,
    )
    surf = ds["tire_surface_c"].values  # (lap, s)
    assert surf[0, 0] == pytest.approx(20.0, abs=1.0), "lap 1 starts at the cold seed"
    assert surf[0, -1] > surf[0, 0] + 2.0, "the tyres warm up over lap 1"
    # That warmed state carries into lap 2 (well above the cold seed — no reset).
    assert surf[1, 0] > surf[0, 0] + 2.0, (
        "lap 2 starts warm, not reset to the cold seed"
    )
    assert np.allclose(surf[1:, 0], surf[:-1, -1], atol=0.5), (
        "surface carries across the boundary"
    )


def test_qss_stint_is_deterministic(catalunya: Track) -> None:
    kwargs: dict[str, object] = {
        "n_laps": 3,
        "tier": "t1",
        "ds_m": 8.0,
        "sim": FLAT_COARSE,
        "tire_thermal": True,
    }
    a = solve_stint_dataset(LIMEBEER, catalunya, **kwargs)  # type: ignore[arg-type]
    b = solve_stint_dataset(LIMEBEER, catalunya, **kwargs)  # type: ignore[arg-type]
    np.testing.assert_array_equal(a["lap_time_s"].values, b["lap_time_s"].values)
    np.testing.assert_array_equal(a["tire_wear_mm"].values, b["tire_wear_mm"].values)


def test_frozen_tyre_stint_repeats_the_same_lap(catalunya: Track) -> None:
    """With the tyre march off, every lap is identical — there is no slow state to carry."""
    ds = solve_stint_dataset(
        LIMEBEER,
        catalunya,
        n_laps=3,
        tier="t1",
        ds_m=8.0,
        sim=FLAT_COARSE,
        tire_thermal=False,
    )
    lap_time = ds["lap_time_s"].values
    assert np.ptp(lap_time) == 0.0, "a frozen-tyre stint repeats the same lap time"
    assert "tire_wear_mm" not in ds, "no tyre channels without the march"


def test_solve_stint_validates_n_laps(catalunya: Track) -> None:
    with pytest.raises(ValueError, match="n_laps"):
        solve_stint(LIMEBEER, catalunya, 0, sim=COARSE)


def test_solve_stint_redirects_the_transient_tier(catalunya: Track) -> None:
    with pytest.raises(ValueError, match="solve_transient_stint"):
        solve_stint(LIMEBEER, catalunya, 3, tier="t2", sim=COARSE)


# --- The transient (T2) stint --------------------------------------------------------------------


def test_t2_stint_runs_continuously_and_carries_state(catalunya: Track) -> None:
    ds = solve_stint_dataset(
        LIMEBEER,
        catalunya,
        n_laps=2,
        tier="t2",
        ds_m=8.0,
        sim=COARSE,
        tire_thermal=True,
    )
    assert ds.sizes["lap"] == 2
    assert ds.attrs["tier"] == "t2"
    assert ds.attrs["completed"] == 1, "both laps reached the finish line"
    assert ds.attrs["requested_laps"] == 2
    lap_time = ds["lap_time_s"].values
    assert (lap_time > 0).all() and np.isfinite(lap_time).all()
    assert ds["tire_wear_mm"].dims == ("lap", "wheel")
    assert list(ds["wheel"].values) == WHEELS
    # The per-wheel tyre state carries across the start/finish line: end-of-lap wear never falls.
    wear = ds["tire_wear_mm"].values  # (lap, wheel)
    assert (wear[1] >= wear[0] - 1e-9).all(), (
        "end-of-lap wear never falls (state carries)"
    )
    assert any("stint" in n.lower() for n in ds.attrs["notes"])


def test_t2_single_lap_surfaces_the_tyre_channels(catalunya: Track) -> None:
    """PR6 also surfaces the T2 single-lap per-wheel tyre channels (a PR3 boundary gap)."""
    warm = transient_lap_dataset(
        solve_transient_lap(
            LIMEBEER, catalunya, ds_m=8.0, sim=COARSE, tire_thermal=True
        )
    )
    for name in ("tire_surface_c", "tire_wear_mm", "tire_grip"):
        assert warm[name].dims == ("time", "wheel"), name
    # Default off: the frozen path carries no tyre channels.
    frozen = transient_lap_dataset(
        solve_transient_lap(LIMEBEER, catalunya, ds_m=8.0, sim=COARSE)
    )
    assert "tire_surface_c" not in frozen
