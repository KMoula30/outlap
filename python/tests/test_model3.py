# SPDX-License-Identifier: AGPL-3.0-only
"""The Tesla Model 3 RWD (HV variant) reference vehicle — M3 PR11.

Loads warning-clean (estimates NOTED in the loaded-model report, never warned), completes T1
laps on the 3D reference Catalunya (``catalunya_osm``) and on the flat TUMFTM Nürburgring GP
inside plausible road-car bands, and exercises the full slow-state coupling end-to-end through
the Python surface: the synthetic Vdc-stacked drive unit + 800 V-class pack + lumped `.emotor`
produce ``state_of_charge`` / ``machine_temp_c`` channels, a net discharge with braking regen (the
machine recovers energy under braking, M6 PR3), and the sizing-sensitivity ordering (small ≤ large).

Bands are wide sanity tripwires (the ``catalunya.rs`` idiom), anchored to the first-run numbers
recorded in the PR: medium sizing ≈161 s Nürburgring GP / ≈154 s Catalunya (a ≈200 kW road
sedan's 2:3x lap), top speed ≈60–65 m/s.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import xarray as xr

from outlap.core import Track, min_curvature, solve_lap_dataset, vehicle_report

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"

MODEL3 = str(_DATA / "vehicles/tesla_model3_rwd")
CATALUNYA = str(_DATA / "tracks/catalunya_osm")
NUERBURGRING = str(_DATA / "tracks/nuerburgring")

# CI-speed envelope grid (the notebooks' FAST idiom): every sizing override regenerates the
# g-g-g-v envelope, so the sweep stays cheap. Bands below are calibrated at THIS grid.
FAST_SIM: dict[str, object] = {
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}
}


def _lap(
    track_dir: str, overrides: dict[str, bool | int | float | str] | None = None
) -> xr.Dataset:
    track = Track.load(track_dir)
    rl = min_curvature(track, 1.5)
    return solve_lap_dataset(MODEL3, rl, tier="t1", sim=FAST_SIM, overrides=overrides)


@pytest.fixture(scope="module")
def nuerburgring_lap() -> xr.Dataset:
    return _lap(NUERBURGRING)


def test_report_warning_clean_estimates_noted() -> None:
    rep = vehicle_report(MODEL3)
    assert rep["name"] == "Tesla Model 3 RWD (HV variant)"
    assert rep["warnings"] == [], f"must load warning-clean: {rep['warnings']}"
    assert rep["degraded"] == [], f"nothing degraded: {rep['degraded']}"
    # The deliberate estimates surface in the report — noted, not warned (Decision #41).
    estimated = rep["estimated"]
    assert isinstance(estimated, list)
    pointers = [p for p, _ in estimated]
    assert any("anti_dive" in p for p in pointers), pointers
    assert any("anti_squat" in p for p in pointers), pointers


def test_t1_lap_nuerburgring_plausible_road_car(nuerburgring_lap: xr.Dataset) -> None:
    ds = nuerburgring_lap
    lap = ds.attrs["lap_time_s"]
    # ≈200 kW road sedan on the 5.14 km GP-Strecke: first-run 160.8 s; wide sanity band.
    assert 140.0 < lap < 195.0, f"lap {lap:.2f} s outside the road-car band"
    top = float(ds.v.max())
    assert 50.0 < top < 70.0, f"top speed {top:.1f} m/s"
    assert float(ds.v.min()) > 8.0, "no near-stop on a GP circuit"
    assert ds.attrs["tier"] == "t1"
    fz = ds.vertical_load_n.to_numpy()
    assert np.isfinite(fz).mean() > 0.9, "most stations must re-trim feasibly"
    assert float(np.nanmin(fz)) > 0.0, "no wheel lifts on a QSS lap"


def test_t1_lap_catalunya_3d_plausible_road_car() -> None:
    ds = _lap(CATALUNYA)
    lap = ds.attrs["lap_time_s"]
    # First-run 154.3 s on the 4.67 km OSM+DEM 3D reference Catalunya; wide sanity band.
    assert 135.0 < lap < 190.0, f"lap {lap:.2f} s outside the road-car band"
    assert "state_of_charge" in ds, (
        "the coupled stack must be active on the 3D track too"
    )


def test_slow_state_coupling_live(nuerburgring_lap: xr.Dataset) -> None:
    ds = nuerburgring_lap
    # The synthetic stack (Vdc-mapped DU + pack + .emotor) drives the slow-state channels.
    assert "state_of_charge" in ds and "machine_temp_c" in ds
    soc = ds.state_of_charge.to_numpy()
    dsoc = np.diff(soc)
    # SoC moves BOTH ways: it falls under traction and RISES under braking — the mapped EV recovers
    # energy through its electric machine (M6 PR3), the same regen the transient tier already models.
    # It is NOT the pre-PR3 discharge-only monotone trace.
    assert dsoc.min() < 0.0, "SoC falls under traction draw"
    assert dsoc.max() > 0.0, "SoC rises under braking — the machine regenerates"
    assert soc[-1] < soc[0], (
        "a full-throttle lap still draws net charge (consumption > regen)"
    )
    assert 0.0 < soc[-1] < 0.98, f"end SoC {soc[-1]:.3f}"
    temp = ds.machine_temp_c.to_numpy()
    assert temp.max() > temp[0] + 5.0, "the winding must heat under traction loss"
    # model3 has no `ers:` block, so its pack keeps the v0.3.0 top-of-window seed (the M6 PR2
    # mid-window seed is scoped to ERS cars). The drive-segment trajectory is unchanged from v0.3.0;
    # the SoC channel now carries the braking regen (a deliberate physics gain, not a regression).
    assert float(temp.max()) < 180.0, "winding stays below t_max"
    # The machine-thermal mass-heuristic fills (none expected — the emotor is fully authored)
    # and the missing-map fallbacks are recorded in the notes: nothing silent.
    assert isinstance(ds.attrs["notes"], tuple)


def test_sizing_sensitivity_ordering(nuerburgring_lap: xr.Dataset) -> None:
    small = _lap(
        NUERBURGRING, overrides={"drivetrain.units.0.source": "ptm/du_small.ptm.yaml"}
    )
    large = _lap(
        NUERBURGRING, overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"}
    )
    t_small = small.attrs["lap_time_s"]
    t_medium = nuerburgring_lap.attrs["lap_time_s"]
    t_large = large.attrs["lap_time_s"]
    assert t_small > t_medium > t_large, (
        f"sizing must order lap times: {t_small:.2f} / {t_medium:.2f} / {t_large:.2f}"
    )
    # Thermal + battery caps bite harder on the big DU: the medium→large gain is smaller
    # than the small→medium gain (the capstone's diminishing-returns story).
    assert (t_small - t_medium) > (t_medium - t_large), "diminishing returns expected"
