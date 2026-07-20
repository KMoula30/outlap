# SPDX-License-Identifier: AGPL-3.0-only
"""T3 (14-DOF) tier integration tests (M6 PR7).

The transient solver runs a full lap with live suspension states: sprung heave/pitch/roll + four
unsprung verticals, per-wheel F_z from the tyre vertical spring, and aero evaluated at the
instantaneous ride heights. These tests exercise the whole `tier="t3"` pipeline end-to-end and check
the physics the tier exists for (§6.1): the platform sinks under downforce and pitches under braking.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import xarray as xr

from outlap.core import Track, solve_lap_dataset

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"

CATALUNYA = str(_DATA / "tracks/catalunya_osm")
F1_2026 = str(_DATA / "vehicles/f1_2026")
LIMEBEER = str(_DATA / "vehicles/limebeer_2014_f1")

# Flat-track analysis mode + a coarse envelope keep the (cold) envelope build cheap; the transient
# lap itself is full-resolution.
SIM: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2},
}
DS_M = 8.0


@pytest.fixture(scope="module")
def catalunya() -> Track:
    return Track.load(CATALUNYA)


@pytest.fixture(scope="module")
def t2_lap(catalunya: Track) -> xr.Dataset:
    return solve_lap_dataset(F1_2026, catalunya, ds_m=DS_M, tier="t2", sim=SIM)


@pytest.fixture(scope="module")
def t3_lap(catalunya: Track) -> xr.Dataset:
    return solve_lap_dataset(F1_2026, catalunya, ds_m=DS_M, tier="t3", sim=SIM)


def test_t3_lap_returns_live_suspension_channels(t3_lap: xr.Dataset) -> None:
    assert t3_lap.attrs["tier"] == "t3"
    assert bool(t3_lap.attrs["completed"])
    for ch in (
        "heave_m",
        "pitch_rad",
        "roll_rad",
        "ride_height_f_m",
        "ride_height_r_m",
        "suspension_travel_m",
    ):
        assert ch in t3_lap, f"the t3 lap is missing the {ch} channel"
    assert t3_lap["suspension_travel_m"].dims == ("time", "wheel")
    # The suspension is actually moving (not a frozen platform).
    assert float(t3_lap["heave_m"].std()) > 1e-4


def test_t3_platform_sinks_under_downforce(t3_lap: xr.Dataset) -> None:
    """Aero downforce loads the sprung mass, compresses the springs, and sinks the platform: the
    ride heights at speed sit BELOW the design static heights (40 mm front / 90 mm rear)."""
    hf = t3_lap["ride_height_f_m"].values
    hr = t3_lap["ride_height_r_m"].values
    # The platform rides lower than static almost everywhere (it only approaches static at rest).
    assert float(np.mean(hf)) < 0.040, "front platform sinks under downforce"
    assert float(np.mean(hr)) < 0.090, "rear platform sinks under downforce"
    # And the heave is predominantly downward (negative).
    assert float(np.mean(t3_lap["heave_m"].values)) < 0.0


def test_pitch_tracks_longitudinal_load(t3_lap: xr.Dataset) -> None:
    """Under braking (a_x < 0) the platform pitches nose-down (θ > 0); under power it squats the
    other way. So the pitch angle is anti-correlated with the longitudinal acceleration — the
    mechanism behind the pitch-under-braking aero-balance shift (§6.1)."""
    ax = t3_lap["ax"].values
    pitch = t3_lap["pitch_rad"].values
    # Restrict to the meaningful longitudinal events (ignore near-zero a_x noise).
    strong = np.abs(ax) > 3.0
    corr = np.corrcoef(ax[strong], pitch[strong])[0, 1]
    assert corr < -0.3, f"pitch should be nose-down under braking (corr {corr:.2f} < 0)"


def test_t3_lap_time_tracks_t2(t2_lap: xr.Dataset, t3_lap: xr.Dataset) -> None:
    """The T3 lap time is close to the T2 lap time on the same car (the tiers agree on pace; T3 adds
    ride fidelity, not a different car). Recorded, not a tight gate — the dynamic-ride-height aero +
    the refinement terms move it a little (Decision #48)."""
    assert bool(t2_lap.attrs["completed"]) and bool(t3_lap.attrs["completed"])
    t2 = float(t2_lap.attrs["lap_time_s"])
    t3 = float(t3_lap.attrs["lap_time_s"])
    rel = abs(t3 - t2) / t2
    assert rel < 0.03, f"t3 {t3:.2f}s vs t2 {t2:.2f}s differ by {rel * 100:.2f}% (> 3%)"


def test_t3_needs_suspension_data(catalunya: Track) -> None:
    """A car without the T3 suspension block fails at assembly with a plain-language field list
    (never estimated, never a panic) — the `per_lap_deploy_mj` trap pattern (PR7c)."""
    with pytest.raises(
        ValueError, match="t3.*suspension|unsprung_mass|damper|bumpstop"
    ):
        solve_lap_dataset(LIMEBEER, catalunya, ds_m=DS_M, tier="t3", sim=SIM)
