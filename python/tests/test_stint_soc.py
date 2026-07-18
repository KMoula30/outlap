# SPDX-License-Identifier: AGPL-3.0-only
"""M6 PR3 — the QSS stint **SoC carry** at the Python boundary (the author's named acceptance check).

A multi-lap QSS run now carries the full electro slow stack (pack SoC / RC voltage / temperature)
across every lap boundary, exactly as the tyre state already does: SoC falls with net consumption
and rises with regeneration lap-over-lap, with only the per-lap ERS budget ledger resetting at the
start/finish. The march physics are gated in Rust (`crates/outlap-qss/tests/stint.rs`); these pin the
Python-visible surface — the `(lap, s)` SoC channel, the end-of-lap pack temperature, continuity
across the boundary, the EV vs hybrid signatures — and the wrapper kwarg-forwarding policy.
"""

from __future__ import annotations

import inspect
from pathlib import Path

import numpy as np
import outlap_core
import pytest
import xarray as xr

from outlap.core import (
    Track,
    solve_lap_dataset,
    solve_stint_dataset,
)

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"

CATALUNYA = str(_DATA / "tracks/catalunya_osm")
F1 = str(_DATA / "vehicles/f1_2026")
MODEL3 = str(_DATA / "vehicles/tesla_model3_rwd")

# CI-speed: the SoC assertions (continuity, sign, shape) are fidelity-independent, and flat-track +
# a coarse envelope keeps every re-solve cheap.
COARSE: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2},
}


@pytest.fixture(scope="module")
def catalunya() -> Track:
    return Track.load(CATALUNYA)


@pytest.fixture(scope="module")
def f1_stint(catalunya: Track) -> xr.Dataset:
    """A 5-lap f1_2026 QSS stint (managed ERS), frozen tyre — the SoC carry is the point."""
    return solve_stint_dataset(
        F1, catalunya, n_laps=5, tier="t0", sim=COARSE, tire_thermal=False
    )


# --- The SoC channel surface ---------------------------------------------------------------------


def test_qss_stint_surfaces_soc_channels(f1_stint: xr.Dataset) -> None:
    ds = f1_stint
    assert ds.sizes["lap"] == 5
    assert ds["state_of_charge"].dims == ("lap", "s")
    assert ds["pack_temp_c"].dims == ("lap",)
    soc = ds["state_of_charge"].values
    assert np.all((soc >= 0.0) & (soc <= 1.0)), "SoC stays in [0, 1]"
    # No machine-thermal network on the f1 ES stack (relaxed pairing) → no machine channel.
    assert "machine_temp_c" not in ds


def test_qss_stint_soc_is_continuous_across_lap_boundaries(
    f1_stint: xr.Dataset,
) -> None:
    """The headline invariant: lap k+1 STARTS at lap k's terminal SoC — no reset to the seed.

    On a closed loop station 0 is the start/finish, so lap k+1's entry SoC equals lap k's terminal
    (one march segment past the last recorded station). Continuity therefore holds to within one
    segment; a *reset* would fling every lap-start back to the mid-window seed (a ~0.3 SoC jump for
    the f1 pack, which recharges toward the top of its window), which this rules out.
    """
    soc = f1_stint["state_of_charge"].values  # (lap, s)
    assert np.allclose(soc[1:, 0], soc[:-1, -1], atol=0.05), (
        "SoC carries across the lap boundary (no reset)"
    )
    # The stint is genuinely multi-lap-stateful: the pack state evolves lap to lap.
    assert not np.allclose(soc[0, 0], soc[1:, 0], atol=1e-6), (
        "a carried stint diverges from the lap-1 seed"
    )


def test_qss_stint_hybrid_recovers_within_a_lap(f1_stint: xr.Dataset) -> None:
    """Within a lap, SoC both FALLS (deploy) and RISES (braking harvest) — recovery is real."""
    soc0 = f1_stint["state_of_charge"].values[0]  # lap 1
    dsoc = np.diff(soc0)
    assert dsoc.min() < 0.0, "SoC must fall under deployment"
    assert dsoc.max() > 0.0, "SoC must rise under braking harvest"


def test_qss_ev_stint_soc_declines_monotonically(catalunya: Track) -> None:
    """A pure-EV stint (no ERS manager, but a battery + machine → braking regen) loses SoC NET every
    lap: consumption exceeds recovery on a hot lap, so the lap-start SoC steps down monotonically and
    carries lap-to-lap — the consumption side of the carry, with the machine's regen folded in."""
    # The model3 3-D chassis is numerically happier off the flat-track path (flat + coarse diverges
    # on the closed-lap velocity passes); a plain 3-D coarse envelope converges.
    ev_sim: dict[str, object] = {
        "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}
    }
    ds = solve_stint_dataset(
        MODEL3, catalunya, n_laps=3, tier="t0", sim=ev_sim, tire_thermal=False
    )
    assert "deploy_power_w" not in ds, "a mapped EV has no energy manager"
    soc = ds["state_of_charge"].values  # (lap, s)
    # Lap-start SoC carries DOWN across the stint (no reset up), and the whole run declines.
    assert (np.diff(soc[:, 0]) <= 1e-9).all(), (
        "EV lap-start SoC never rises (no harvest, no reset)"
    )
    assert soc[-1, -1] < soc[0, 0] - 1e-6, "the EV stint drains the pack over the run"
    # The decline is roughly the same each lap (no machine-thermal derate runaway stalling the car —
    # the regression guard for the QSS distance-march feedback the carry originally tripped).
    lap_drop = -np.diff(soc[:, 0])  # per-lap start-to-start drop
    assert lap_drop.min() > 0.02, "each lap drains a real, comparable chunk of SoC"
    lap_time = ds["lap_time_s"].values
    assert lap_time.max() < 2.0 * lap_time.min(), (
        "no lap stalls (bounded lap time across the stint)"
    )


# --- The wrapper kwarg policy (PR3c) --------------------------------------------------------------


def test_lap_dataset_forwards_tire_thermal_to_t2(catalunya: Track) -> None:
    """The fixed live bug: `solve_lap_dataset(tier="t2")` used to silently DROP `tire_thermal`.

    With it forwarded, the transient tyre-thermal channels appear only when opted in.
    """
    off = solve_lap_dataset(
        MODEL3, catalunya, tier="t2", sim=COARSE, tire_thermal=False
    )
    on = solve_lap_dataset(MODEL3, catalunya, tier="t2", sim=COARSE, tire_thermal=True)
    assert "tire_surface_c" not in off
    assert "tire_surface_c" in on, "tire_thermal must reach the transient solver"


def test_lap_dataset_rejects_qss_incompatible_kwargs(catalunya: Track) -> None:
    """`speed_margin` is transient-only: forwarding it to a QSS tier raises, never silently drops."""
    with pytest.raises(ValueError, match="speed_margin"):
        solve_lap_dataset(MODEL3, catalunya, tier="t1", sim=COARSE, speed_margin=0.9)
    with pytest.raises(ValueError, match="speed_margin"):
        solve_stint_dataset(
            MODEL3, catalunya, n_laps=2, tier="t1", sim=COARSE, speed_margin=0.9
        )


def test_lap_dataset_forwards_initial_soc(catalunya: Track) -> None:
    """`initial_soc` seeds the pack on BOTH tiers through the wrappers."""
    hi = solve_lap_dataset(F1, catalunya, tier="t0", sim=COARSE, initial_soc=0.85)
    lo = solve_lap_dataset(F1, catalunya, tier="t0", sim=COARSE, initial_soc=0.45)
    assert hi["state_of_charge"].values[0] == pytest.approx(0.85)
    assert lo["state_of_charge"].values[0] == pytest.approx(0.45)


@pytest.mark.parametrize(
    ("wrapper", "entries"),
    [
        ("solve_lap_dataset", ["solve_lap", "solve_transient_lap"]),
        ("solve_stint_dataset", ["solve_stint", "solve_transient_stint"]),
    ],
)
def test_dataset_wrappers_cover_the_union_of_tier_kwargs(
    wrapper: str, entries: list[str]
) -> None:
    """Signature guard: every keyword the underlying tier entry points accept must be namable on the
    dataset wrapper (forwarded or explicitly rejected) — so a future kwarg cannot be added to the
    Rust surface and then silently dropped by the wrapper (the bug this PR fixes)."""
    import outlap.core as core

    wrapper_params = set(inspect.signature(getattr(core, wrapper)).parameters)
    # Positional plumbing the wrapper owns itself, not tier knobs.
    plumbing = {
        "vehicle_dir",
        "track",
        "n_laps",
        "raceline_ds_m",
        "raceline_generator",
        "raceline_iterations",
    }
    for entry in entries:
        for name in inspect.signature(getattr(outlap_core, entry)).parameters:
            if name in plumbing:
                continue
            assert name in wrapper_params, (
                f"{wrapper} does not surface `{name}` from outlap_core.{entry} — decide whether to "
                f"forward it or reject it (PR3c kwarg policy)"
            )
