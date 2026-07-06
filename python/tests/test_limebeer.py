# SPDX-License-Identifier: AGPL-3.0-only
"""The Decision #48 Limebeer gates + the golden-lap regression suite (HANDOFF §13/§14).

The PL2014 cross-check runs at the PRODUCTION envelope resolution (this is a physics gate, not a
plumbing test) on the committed Catalunya import, flat-track mode. It gates what this geometry
supports — top speed and the slowest-corner apex; the fast-corner band was validated against the
paper's own extracted centre-line curvature and becomes a CI gate with the TUMFTM track (PR10).
See docs/validation/limebeer.md for the oracle provenance and the lap-time decomposition.

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

from outlap.core import Track, min_curvature, solve_lap_dataset

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"
GOLDEN_DIR = Path(__file__).resolve().parent / "golden"

LIMEBEER = str(_DATA / "vehicles/limebeer_2014_f1")
F1_2026 = str(_DATA / "vehicles/f1_2026")
CATALUNYA = str(_DATA / "tracks/catalunya")

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
        assert np.array_equal(np.isnan(o), np.isnan(g)), f"{name}/{var} NaN pattern drifted"
        scale = float(np.nanmax(np.abs(g)))
        if not np.isfinite(scale) or scale == 0.0:
            scale = 1.0
        worst = float(np.nanmax(np.abs(o - g))) / scale if np.isfinite(g).any() else 0.0
        assert worst < tol, f"{name}/{var} drifted: {worst:.4f} > {tol}"
