# SPDX-License-Identifier: AGPL-3.0-only
"""Faithful (but slow) forward model: the real Rust stint driver behind the calibrator.

The surrogate in :mod:`outlap.wearcal.model` is what the optimiser inverts in CI. This module is
the *ground truth* it is anchored to: it patches a vehicle's ``.tyr`` thermal/wear blocks in a
scratch copy and runs ``outlap.core.solve_stint`` end-to-end. Use it to (a) sanity-check that
surrogate-calibrated parameters reproduce the intended decay in the real driver, (b) run a
faithful (opt-in, minutes-scale) fit against real pace data, and (c) drive the PR9 validation
gates. Every real stint rebuilds the g-g-g-v envelope across its tyre-state axes, so keep the
envelope grid coarse and the lap count modest.
"""

from __future__ import annotations

import shutil
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np
import yaml
from numpy.typing import NDArray

from .data import StintObservation
from .model import SurrogateParams

if TYPE_CHECKING:
    from collections.abc import Generator

F = NDArray[np.float64]

# A coarse envelope grid keeps a real stint eval to a few seconds (the envelope build dominates).
FAST_SIM: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2},
}


@dataclass(frozen=True)
class SimStint:
    """A real-driver stint result: per-lap times plus the per-lap terminal wear/grip trace."""

    lap: F
    lap_time_s: F
    wear_mm: F
    grip: F
    surface_c: F

    def observation(self, *, label: str = "sim") -> StintObservation:
        """As a calibration observation (lap times only)."""
        return StintObservation(lap=self.lap, lap_time_s=self.lap_time_s, label=label)


def patch_tyr_document(doc: dict[str, Any], params: SurrogateParams) -> dict[str, Any]:
    """Return ``doc`` with its ``thermal``/``wear`` blocks updated from ``params`` (in place)."""
    thermal_updates, wear_updates = params.tyr_updates()
    doc.setdefault("thermal", {}).update(thermal_updates)
    doc.setdefault("wear", {}).update(wear_updates)
    return doc


@contextmanager
def scratch_vehicle(
    vehicle_dir: Path, params: SurrogateParams
) -> Generator[Path, None, None]:
    """A temporary copy of ``vehicle_dir`` with every ``.tyr`` thermal/wear block set from ``params``.

    Yields the scratch vehicle directory; cleaned up on exit. Used because ``.tyr`` parameters live
    in a referenced file, not the vehicle document, so they cannot be reached by dotted overrides.
    """
    tmp = Path(tempfile.mkdtemp(prefix="wearcal_"))
    try:
        dst = tmp / vehicle_dir.name
        shutil.copytree(vehicle_dir, dst)
        for tyr_path in dst.rglob("*.tyr.yaml"):
            doc = cast(
                "dict[str, Any]", yaml.safe_load(tyr_path.read_text(encoding="utf-8"))
            )
            patch_tyr_document(doc, params)
            tyr_path.write_text(
                yaml.safe_dump(doc, sort_keys=False, allow_unicode=True),
                encoding="utf-8",
            )
        yield dst
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def sim_stint(
    vehicle_dir: Path,
    track: Any,
    params: SurrogateParams,
    n_laps: int,
    *,
    tier: str = "t0",
    ds_m: float = 12.0,
    sim: dict[str, object] | None = None,
    initial_tire_temp_c: float | None = None,
) -> SimStint:
    """Run a real stint on ``vehicle_dir`` with ``params`` patched in; return the per-lap trace.

    ``tier`` is ``"t0"``/``"t1"`` (QSS) or ``"t2"`` (transient). Lap times and per-lap terminal wear
    / grip / surface temperature are extracted from the stint dataset.
    """
    from outlap.core import solve_stint_dataset

    with scratch_vehicle(vehicle_dir, params) as veh:
        ds: Any = solve_stint_dataset(
            str(veh),
            track,
            n_laps=n_laps,
            tier=tier,
            ds_m=ds_m,
            sim=sim if sim is not None else FAST_SIM,
            tire_thermal=True,
            initial_tire_temp_c=initial_tire_temp_c,
        )
    lap = np.asarray(ds["lap"].values, dtype=np.float64)
    lap_time = np.asarray(ds["lap_time_s"].values, dtype=np.float64)
    wear = _terminal(ds, "tire_wear_mm")
    grip = _terminal(ds, "tire_grip")
    surface = _terminal(ds, "tire_surface_c")
    return SimStint(
        lap=lap, lap_time_s=lap_time, wear_mm=wear, grip=grip, surface_c=surface
    )


def _terminal(ds: Any, name: str) -> F:
    """End-of-lap value of a per-lap tyre channel (max over the lateral axis if present)."""
    if name not in ds:
        return np.full(ds["lap"].size, np.nan, dtype=np.float64)
    arr = np.asarray(ds[name].values, dtype=np.float64)
    if arr.ndim == 1:
        return arr
    # (lap, s) QSS or (lap, wheel) T2 → reduce the trailing axis to one representative value.
    if ds[name].dims[-1] == "s":
        return arr[:, -1]
    return arr.mean(axis=1)
