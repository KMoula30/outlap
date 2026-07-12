# SPDX-License-Identifier: AGPL-3.0-only
"""QSS↔T2 parity across the three reference cars on ``catalunya_osm`` (PR10, Decision #11/#16).

The **asserted** gate is **hull containment**: the T2 closed-loop operating points must stay inside
the T1 g-g-g-v envelope the QSS tiers solve on (≤2% exceedance). That is the physics-fidelity parity
check — it holds regardless of how competitive the driver is.

The lap-time / apex deltas are **recorded, not gated**. The ideal driver tracks a stability margin
(~0.85 of the QSS profile) and spins if pushed to the limit, so a T2 lap is ~+17% off T0 — a
driver-competitiveness gap (Decision #13), not a chassis/tyre error. Flipping the ≤0.3% lap / ≤1%
apex gates needs a competitive driver, which is out of this PR's scope; here we assert the laps
complete and stay inside the hull, and print the deltas for the record (the Decision #48 pattern).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from outlap.core import (
    Track,
    min_curvature,
    solve_lap,
    solve_transient_lap,
    transient_lap_dataset,
)

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"
CATALUNYA = str(_DATA / "tracks/catalunya_osm")
CARS = ["limebeer_2014_f1", "f1_2026", "tesla_model3_rwd"]
# A moderate envelope keeps the three full T2 laps inside the CI budget while still being a real hull.
PARITY_SIM: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 16, "ax_points": 12, "g_normal_points": 3},
}
_G = 9.80665


@pytest.fixture(scope="module")
def track() -> Track:
    return Track.load(CATALUNYA)


@pytest.mark.parametrize("car", CARS)
def test_t2_stays_inside_the_t1_hull(car: str, track: Track) -> None:
    veh = str(_DATA / "vehicles" / car)
    rl = min_curvature(track, 1.1)

    # T1 lap to obtain this car's g-g-g-v envelope (the hull), and T0 for the recorded delta.
    t1_lap = solve_lap(veh, rl.line(), tier="t1", sim=PARITY_SIM, raceline_ds_m=rl.ds_m)
    env = t1_lap.envelope
    assert env is not None, f"{car}: T1 lap did not carry a g-g-g-v envelope"
    t0_time = solve_lap(
        veh, rl.line(), tier="t0", sim=PARITY_SIM, raceline_ds_m=rl.ds_m
    ).lap_time_s

    ds = transient_lap_dataset(
        solve_transient_lap(veh, rl.line(), raceline_ds_m=rl.ds_m, sim=PARITY_SIM)
    )
    assert str(ds.attrs.get("completed")) in ("1", "True"), (
        f"{car}: T2 lap did not close"
    )

    vx = ds["vx"].to_numpy()
    ax = ds["ax"].to_numpy()
    ay = ds["ay"].to_numpy()
    t2_time = float(ds.attrs["lap_time_s"])

    # Hull containment (ASSERTED): fraction of samples whose lateral accel exceeds the envelope
    # boundary (at the sample's speed + longitudinal accel) by more than 2%.
    samples = 0
    exceed = 0
    for i in range(len(vx)):
        v = max(float(vx[i]), 1.0)
        ay_max = env.ay_boundary(v, float(ax[i]), _G)  # flat lap ⇒ g_normal = g
        if ay_max <= 0.0:
            continue
        samples += 1
        if abs(float(ay[i])) > ay_max * 1.02:
            exceed += 1
    hull_pct = 100.0 * exceed / max(samples, 1)

    lap_delta_pct = 100.0 * (t2_time - t0_time) / t0_time
    # Recorded (not asserted): the driver-margin lap-time gap.
    print(
        f"[parity {car}] T0={t0_time:.2f}s T2={t2_time:.2f}s Δlap={lap_delta_pct:+.1f}% "
        f"(recorded) | hull exceed={hull_pct:.2f}% of {samples} (gate ≤2%)"
    )
    assert hull_pct <= 2.0, (
        f"{car}: T2 hull containment {hull_pct:.2f}% > 2% — operating points leave the T1 envelope"
    )
    # A sanity floor on the recorded delta: T2 is slower (driver margin), never implausibly faster.
    assert lap_delta_pct > -5.0, (
        f"{car}: T2 implausibly faster than T0 ({lap_delta_pct:+.1f}%)"
    )
