# SPDX-License-Identifier: AGPL-3.0-only
"""Compound-preset tests (M5 PR8): every reference/compound .tyr loads + validates, the
soft/medium/hard presets are ordered as a compound family, and a smoke stint reproduces the
expected relative ordering (soft grips most fresh and wears fastest; hard the opposite)."""

from __future__ import annotations

import shutil
import tempfile
from collections.abc import Generator
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import pytest
import yaml

from outlap.core import Track, Tyre, solve_stint_dataset

_ROOT = Path(__file__).resolve().parents[2]
_COMPOUNDS = _ROOT / "data" / "tires" / "f1_2026_compounds"
_F1 = _ROOT / "data" / "vehicles" / "f1_2026"
_CATALUNYA = str(_ROOT / "data" / "tracks" / "catalunya_osm")

_RECALIBRATED = [
    "data/tires/limebeer_2014_f1/f1.tyr.yaml",
    "data/tires/pacejka_2006_205_60r15/car.tyr.yaml",
    "data/tires/roborace_devbot_mf52/car.tyr.yaml",
    "data/vehicles/f1_2026/tyr/slick.tyr.yaml",
]
_COMPOUND_FILES = ["soft.tyr.yaml", "medium.tyr.yaml", "hard.tyr.yaml"]


def _blocks(path: Path) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    return doc["mf61"], doc["thermal"], doc["wear"]


# --- Loading + validation ------------------------------------------------------------------


@pytest.mark.parametrize(
    "rel",
    _RECALIBRATED + [f"data/tires/f1_2026_compounds/{c}" for c in _COMPOUND_FILES],
)
def test_tyr_loads_and_validates(rel: str) -> None:
    """Every recalibrated reference tyre and compound preset loads through the schema."""
    Tyre.load(str(_ROOT / rel))


def test_recalibrated_wear_is_no_longer_saturating() -> None:
    """The recalibrated slicks carry a physically-small k_w (not the ~1e-3 saturating default)."""
    for rel in _RECALIBRATED:
        _, _, wear = _blocks(_ROOT / rel)
        assert 1e-10 < wear["k_w"] < 1e-7, (
            f"{rel}: k_w {wear['k_w']} out of calibrated band"
        )
        assert wear["delta_c"] <= 0.2, f"{rel}: cliff too deep for a realistic stint"


# --- Compound family ordering (structural) -------------------------------------------------


def test_compounds_form_an_ordered_family() -> None:
    """soft → medium → hard: more grip, lower optimal temp, faster wear, earlier cliff for soft."""
    soft = _blocks(_COMPOUNDS / "soft.tyr.yaml")
    med = _blocks(_COMPOUNDS / "medium.tyr.yaml")
    hard = _blocks(_COMPOUNDS / "hard.tyr.yaml")
    # Peak grip: soft > medium > hard.
    assert soft[0]["LMUX"] > med[0]["LMUX"] > hard[0]["LMUX"]
    assert soft[0]["LMUY"] > med[0]["LMUY"] > hard[0]["LMUY"]
    # Optimal temperature: soft (switches on early) < medium < hard.
    assert soft[1]["t_opt"] < med[1]["t_opt"] < hard[1]["t_opt"]
    # Wear rate: soft > medium > hard.
    assert soft[2]["k_w"] > med[2]["k_w"] > hard[2]["k_w"]
    # Cliff onset: soft (earliest) < medium < hard.
    assert soft[2]["w_c"] < med[2]["w_c"] < hard[2]["w_c"]


# --- Smoke stint: the crossover ordering ---------------------------------------------------


@contextmanager
def _compound_vehicle(compound: str) -> Generator[Path, None, None]:
    tmp = Path(tempfile.mkdtemp(prefix="compound_"))
    try:
        veh = tmp / "f1_2026"
        shutil.copytree(_F1, veh)
        shutil.copy(_COMPOUNDS / f"{compound}.tyr.yaml", veh / "tyr" / "slick.tyr.yaml")
        yield veh
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_compound_smoke_stint_ordering() -> None:
    """A short warm stint per compound: soft is quickest fresh and wears fastest; hard the opposite."""
    track = Track.load(_CATALUNYA)
    sim: dict[str, object] = {
        "flat_track": True,
        "envelope": {"v_points": 7, "ax_points": 6, "g_normal_points": 2},
    }
    fresh: dict[str, float] = {}
    end_wear: dict[str, float] = {}
    for compound in ("soft", "medium", "hard"):
        with _compound_vehicle(compound) as veh:
            ds = solve_stint_dataset(
                str(veh),
                track,
                n_laps=6,
                tier="t0",
                ds_m=16.0,
                sim=sim,
                tire_thermal=True,
                initial_tire_temp_c=None,
            )
        fresh[compound] = float(ds["lap_time_s"].values[0])
        end_wear[compound] = float(np.max(ds["tire_wear_mm"].values[-1]))
    # Fresh pace: soft quickest (most grip), hard slowest.
    assert fresh["soft"] < fresh["medium"] < fresh["hard"], fresh
    # Degradation: soft wears fastest, hard slowest.
    assert end_wear["soft"] > end_wear["medium"] > end_wear["hard"], end_wear
