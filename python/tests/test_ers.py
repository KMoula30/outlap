# SPDX-License-Identifier: AGPL-3.0-only
"""M6 PR2 — the QSS 2026 ERS energy manager surface: deploy/harvest channels, budgets, notes.

The physics gates live in Rust (`crates/outlap-qss/tests/ers_march.rs`); these tests pin the
Python-visible surface: the dataset channels, the recorded notes, and the loud degraded path.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import pytest

from outlap.core import Track, solve_lap_dataset, vehicle_report

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"

CATALUNYA = str(_DATA / "tracks/catalunya_osm")
F1 = str(_DATA / "vehicles/f1_2026")

# CI-speed envelope: the surface assertions are fidelity-independent.
COARSE: dict[str, object] = {
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}
}


@pytest.fixture(scope="module")
def catalunya() -> Track:
    return Track.load(CATALUNYA)


def test_f1_lap_carries_the_energy_manager_channels(catalunya: Track) -> None:
    ds = solve_lap_dataset(F1, catalunya, tier="t0", sim=COARSE)
    # The managed lap surfaces the realized electrical command + the pack trace.
    for ch in ("deploy_power_w", "harvest_power_w", "state_of_charge"):
        assert ch in ds, f"missing {ch}"
    deploy = ds.deploy_power_w.to_numpy()
    harvest = ds.harvest_power_w.to_numpy()
    soc = ds.state_of_charge.to_numpy()
    assert deploy.max() > 0.0, "an f1 lap must deploy"
    assert harvest.max() > 0.0, "an f1 lap must harvest under braking"
    # The FIA cap (350 kW electrical) bounds the realized deploy everywhere.
    assert deploy.max() <= 350e3 + 1e-6
    assert np.all((soc >= 0.0) & (soc <= 1.0))
    # SoC moves BOTH ways over the lap (the author's M6 acceptance direction, single-lap half).
    dsoc = np.diff(soc)
    assert dsoc.min() < 0.0 and dsoc.max() > 0.0, (
        "SoC must fall under deploy and rise under harvest"
    )
    # No machine-thermal network on the f1 ES stack (relaxed pairing): no machine channel.
    assert "machine_temp_c" not in ds
    # The manager + convergence + C5.2.9 records are in the notes (nothing silent).
    joined = "\n".join(ds.attrs["notes"])
    assert "2026 ERS energy manager active" in joined
    assert "recorded per FIA C5.2.9" in joined
    assert "outer-iteration convergence" in joined
    assert "seeded at" in joined and "middle of its usable window" in joined


def test_no_ers_vehicle_carries_no_energy_channels(catalunya: Track) -> None:
    limebeer = str(_DATA / "vehicles/limebeer_2014_f1")
    ds = solve_lap_dataset(limebeer, catalunya, tier="t0", sim=COARSE)
    assert "deploy_power_w" not in ds
    assert "harvest_power_w" not in ds
    assert "state_of_charge" not in ds


def _f1_without(missing: str, tmp_path: Path) -> str:
    """A copy of the f1_2026 vehicle dir with one battery file removed."""
    dst = tmp_path / "f1_2026"
    shutil.copytree(F1, dst)
    (dst / "battery" / missing).unlink()
    return str(dst)


def test_missing_ecm_sidecar_is_gated_like_a_missing_yaml(
    catalunya: Track, tmp_path: Path
) -> None:
    # An `ers:` car needs a runnable pack. A missing ECM parquet sidecar is the same
    # missing-energy-store contract violation as a missing YAML — a hard error by default,
    # solvable only via allow_degraded (the ONLY fallback path, which marks the run).
    veh = _f1_without("f1_es.tables.parquet", tmp_path)
    with pytest.raises(ValueError, match="ECM sidecar|energy manager schedules"):
        solve_lap_dataset(veh, catalunya, tier="t0", sim=COARSE)
    # allow_degraded keeps it solvable, inert, with no harvest — and marks the run.
    ds = solve_lap_dataset(
        veh, catalunya, tier="t0", sim={**COARSE, "allow_degraded": True}
    )
    assert "state_of_charge" not in ds
    assert any("degraded path" in n for n in ds.attrs["notes"])


def test_vehicle_report_describes_a_degraded_ers_car(tmp_path: Path) -> None:
    # The loaded-model report is the diagnostic surface: it must DESCRIBE an ers car whose
    # battery is absent (a hard error for a solve), never hard-fail on it.
    veh = _f1_without("f1_es.yaml", tmp_path)
    rep = vehicle_report(veh)
    degraded: list[tuple[str, str]] = rep["degraded"]  # type: ignore[assignment]
    assert degraded, "the missing energy store must surface as a degradation"
    assert any("battery" in ptr for ptr, _ in degraded)
