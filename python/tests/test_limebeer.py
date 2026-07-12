# SPDX-License-Identifier: AGPL-3.0-only
"""The Decision #48 Limebeer gates + the golden-lap regression suite (HANDOFF §13/§14).

The PL2014 cross-check runs at the PRODUCTION envelope resolution (this is a physics gate, not a
plumbing test) on the committed OSM+DEM Catalunya import (``catalunya_osm``), flat-track mode. It
gates what this geometry supports — top speed and the slowest-corner apex. The fast-corner band
stays deferred to M4: the TUMFTM racetrack-database Catalunya vendored in PR10 is a class-C
*smoothed* centre line that rounds the slow chicane open (QSS slow apex +15.6%) and tightens the
fast corners, so it does not reproduce the PL2014 apex bands under QSS-on-min-curvature; the
residual is the line-optimality gap the M4 time-weighted raceline QP addresses (Decision #48). See
docs/validation/limebeer.md for the oracle provenance and the lap-time decomposition.

Golden laps: committed parquet channel sets per vehicle × track × tier, compared within
per-channel tolerances. Regenerate ONLY via `OUTLAP_BLESS=1 uv run pytest tests/test_limebeer.py`
plus a PR note explaining the physics change (never silently).
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import xarray as xr

from outlap.core import (
    Track,
    min_curvature,
    solve_lap_dataset,
    solve_transient_lap,
    time_weighted,
    transient_lap_dataset,
)

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"
GOLDEN_DIR = Path(__file__).resolve().parent / "golden"

LIMEBEER = str(_DATA / "vehicles/limebeer_2014_f1")
F1_2026 = str(_DATA / "vehicles/f1_2026")
# The OSM+DEM 3D import (era-appropriate slow chicane); the flat TUMFTM `catalunya` is a class-C
# smoothed layout that does not reproduce the PL2014 apex bands — see the module docstring.
CATALUNYA = str(_DATA / "tracks/catalunya_osm")

# PL2014 published values (docs/validation/limebeer.md; digitised Fig. 8 where noted).
ORACLE_LAP_S = 82.43
ORACLE_TOP_MPS = 88.0  # Fig. 8 (digitised)
ORACLE_SLOWEST_APEX_MPS = 17.0  # Fig. 8 (digitised)


@pytest.fixture(scope="module")
def limebeer_flat_lap() -> xr.Dataset:
    """The validation lap: min-curvature line, flat track, production envelope grid."""
    track = Track.load(CATALUNYA)
    rl = min_curvature(track, 1.1)
    return solve_lap_dataset(LIMEBEER, rl, tier="t0", sim={"flat_track": True})


def test_limebeer_top_speed_within_1pct(limebeer_flat_lap: xr.Dataset) -> None:
    top = float(limebeer_flat_lap.v.max())
    assert abs(top - ORACLE_TOP_MPS) / ORACLE_TOP_MPS < 0.01, f"top speed {top:.1f} m/s"


def test_limebeer_slowest_apex_within_5pct(limebeer_flat_lap: xr.Dataset) -> None:
    slowest = float(limebeer_flat_lap.v.min())
    err = abs(slowest - ORACLE_SLOWEST_APEX_MPS) / ORACLE_SLOWEST_APEX_MPS
    assert err < 0.05, f"slowest apex {slowest:.1f} m/s vs {ORACLE_SLOWEST_APEX_MPS}"


def test_limebeer_lap_time_recorded_band(limebeer_flat_lap: xr.Dataset) -> None:
    # NOT the ≤1% gate (Decision #48 moved that to M4): a wide tripwire that the recorded
    # structural delta stays in its analysed band (+5–15% over the OCP oracle on this geometry).
    lap = limebeer_flat_lap.attrs["lap_time_s"]
    assert ORACLE_LAP_S * 1.00 < lap < ORACLE_LAP_S * 1.15, f"lap {lap:.2f} s"
    assert limebeer_flat_lap.attrs["flat_track"] == 1
    assert limebeer_flat_lap.attrs["tier"] == "t0"


# --- Golden laps (§14): vehicle × track × tier, bless-only regeneration --------------------------

_GOLDEN_CASES = [
    ("limebeer_t0_flat", LIMEBEER, "t0", True),
    ("limebeer_t1_flat", LIMEBEER, "t1", True),
    ("f1_2026_t0", F1_2026, "t0", False),
]
# Per-channel relative tolerances (fraction of the channel's max magnitude).
_TOLS = {
    "v": 0.005,
    "ax": 0.02,
    "ay": 0.02,
    "t": 0.005,
    "vertical_load_n": 0.01,
    "slip_ratio": 0.05,
    "slip_angle_rad": 0.05,
}


def _solve(vehicle: str, tier: str, flat: bool) -> xr.Dataset:
    track = Track.load(CATALUNYA)
    rl = min_curvature(track, 1.1)
    return solve_lap_dataset(vehicle, rl, tier=tier, sim={"flat_track": flat})


@pytest.mark.parametrize(("name", "vehicle", "tier", "flat"), _GOLDEN_CASES)
def test_golden_lap(name: str, vehicle: str, tier: str, flat: bool) -> None:
    ds = _solve(vehicle, tier, flat)
    path = GOLDEN_DIR / f"{name}.parquet"
    frame = ds.drop_vars([v for v in ("x", "y", "z") if v in ds]).to_dataframe()

    if os.environ.get("OUTLAP_BLESS") == "1":
        GOLDEN_DIR.mkdir(exist_ok=True)
        frame.to_parquet(path)
        pytest.skip(f"blessed {path.name}")

    assert path.exists(), (
        f"golden {path.name} missing — regenerate via OUTLAP_BLESS=1 (with a PR note)"
    )
    gold = xr.Dataset.from_dataframe(pd.read_parquet(path))
    got = xr.Dataset.from_dataframe(frame)
    assert float(got.t.max()) == pytest.approx(float(gold.t.max()), rel=_TOLS["t"]), (
        "lap time drifted"
    )
    for var, tol in _TOLS.items():
        if var not in gold:
            continue
        g = gold[var].to_numpy()
        o = got[var].to_numpy()
        assert g.shape == o.shape, f"{name}/{var} shape {o.shape} vs golden {g.shape}"
        # Feasibility drift first: per-wheel channels are NaN at infeasible re-trim stations, and
        # a NaN on either side would silently drop out of nanmax below.
        assert np.array_equal(np.isnan(o), np.isnan(g)), (
            f"{name}/{var} NaN pattern drifted"
        )
        scale = float(np.nanmax(np.abs(g)))
        if not np.isfinite(scale) or scale == 0.0:
            scale = 1.0
        worst = float(np.nanmax(np.abs(o - g))) / scale if np.isfinite(g).any() else 0.0
        assert worst < tol, f"{name}/{var} drifted: {worst:.4f} > {tol}"


# --- Golden transient (T2) lap: the time-indexed regression (dims time/wheel) --------------------

# A fixed coarse envelope so the golden is deterministic and CI-fast (the T2 lap itself is the cost).
_T2_SIM: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 12, "ax_points": 10, "g_normal_points": 3},
}
# Per-channel relative tolerances for the time-indexed channels.
_T2_TOLS = {
    "s": 0.005,
    "vx": 0.01,
    "vy": 0.02,
    "yaw_rate": 0.02,
    "steer": 0.02,
    "gear": 0.0,  # discrete → must match exactly (determinism)
    "vertical_load_n": 0.02,
    "slip_ratio": 0.05,
    "slip_angle_rad": 0.05,
}


# A ~90 s lap at 1 ms is ~90k rows; decimate the golden to keep the committed parquet small (the
# regression still pins the whole trajectory — a drift shows at the sampled stations too).
_T2_STRIDE = 100


def test_golden_transient_lap() -> None:
    ds = solve_lap_dataset(
        LIMEBEER, min_curvature(Track.load(CATALUNYA), 1.1), tier="t2", sim=_T2_SIM
    )
    assert str(ds.attrs.get("completed")) in ("1", "True"), (
        "T2 golden lap did not close"
    )
    ds = ds.isel(time=slice(None, None, _T2_STRIDE))
    path = GOLDEN_DIR / "limebeer_t2_flat.parquet"
    frame = ds.drop_vars([v for v in ("x", "y", "z") if v in ds]).to_dataframe()

    if os.environ.get("OUTLAP_BLESS") == "1":
        GOLDEN_DIR.mkdir(exist_ok=True)
        frame.to_parquet(path)
        pytest.skip(f"blessed {path.name}")

    assert path.exists(), (
        f"golden {path.name} missing — regenerate via OUTLAP_BLESS=1 (with a PR note)"
    )
    gold = xr.Dataset.from_dataframe(pd.read_parquet(path))
    got = xr.Dataset.from_dataframe(frame)
    # Same number of time steps (fixed dt ⇒ deterministic grid).
    assert got.sizes["time"] == gold.sizes["time"], (
        f"T2 step count {got.sizes['time']} vs golden {gold.sizes['time']}"
    )
    for var, tol in _T2_TOLS.items():
        if var not in gold:
            continue
        g = gold[var].to_numpy()
        o = got[var].to_numpy()
        assert g.shape == o.shape, f"t2/{var} shape {o.shape} vs golden {g.shape}"
        scale = float(np.nanmax(np.abs(g)))
        if not np.isfinite(scale) or scale == 0.0:
            scale = 1.0
        worst = float(np.nanmax(np.abs(o - g))) / scale
        assert worst <= tol, f"t2/{var} drifted: {worst:.4f} > {tol}"


# --- The Limebeer T2 lap-time delta: RECORDED, not the ≤1% gate ----------------------------------


def test_limebeer_t2_lap_time_recorded_not_gated() -> None:
    """The transient (T2) Limebeer lap on ``catalunya_osm`` vs the 82.43 s OCP oracle.

    This is **recorded with a decomposition, not the ≤1% gate** (docs/validation/limebeer.md). The
    ≤1% Limebeer lap-time gate is not achievable at the T2 tier on this geometry: on the committed
    ``catalunya_osm`` the T2 lap is ~+28% over the OCP oracle, dominated by the ideal driver's
    corner-scaled stability margin (~+14% of T0 — full profile speed on the straights, 0.85 at the
    lateral grip limit), on top of the ~5% track-geometry offset
    (``catalunya_osm`` vs the PL2014 Fig. 6 centre line), the ~2.2 s structural QSS-vs-OCP floor the
    paper itself cites, and ~1.5 s of envelope conservatism. The time-weighted line recovers only
    ~0.3 s of the line-optimality residual. No paper-geometry fixture is committed, so the ungated
    ``catalunya_osm`` figure is the recorded cross-check. A wide tripwire keeps the recorded band from
    drifting silently.
    """
    track = Track.load(CATALUNYA)
    tw = time_weighted(LIMEBEER, track, 1.1, iterations=4, sim=_T2_SIM)
    ds = transient_lap_dataset(
        solve_transient_lap(
            LIMEBEER,
            tw.line(),
            raceline_ds_m=tw.ds_m,
            raceline_generator=tw.generator,
            raceline_iterations=tw.iterations,
            sim=_T2_SIM,
        )
    )
    assert str(ds.attrs.get("completed")) in ("1", "True"), (
        "T2 Limebeer lap did not close"
    )
    lap = float(ds.attrs["lap_time_s"])
    delta_pct = 100.0 * (lap - ORACLE_LAP_S) / ORACLE_LAP_S
    print(
        f"[limebeer T2] lap = {lap:.2f} s vs OCP {ORACLE_LAP_S} s ({delta_pct:+.1f}%) "
        f"— RECORDED, not the ≤1% gate (driver-margin + geometry + QSS-OCP floor; see limebeer.md)"
    )
    # Wide tripwire around the analysed +20–45% structural band — NOT the ≤1% assertion.
    assert ORACLE_LAP_S * 1.20 < lap < ORACLE_LAP_S * 1.45, (
        f"T2 Limebeer lap {lap:.2f} s left its recorded structural band — re-analyse (not a ≤1% gate)"
    )
