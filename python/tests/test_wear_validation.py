# SPDX-License-Identifier: AGPL-3.0-only
"""M5 validation gates (§13) — the flagship tyre thermal + wear/degradation stack.

Three gates, following the Decision #48 pattern (assert where robust; record-and-decompose
otherwise). The recorded numbers are transcribed into docs/validation/{tire-thermal,wear-cliff}.md.

1. Tire thermal   — cold-start warm-up is monotone; the settled surface temperature lands in the
                    published Farroni/broadcast racing-slick band.
2. Wear / cliff   — after inverse-calibration against the committed stint fixture, the *real* driver
                    reproduces monotone pace loss and a cliff as the tread crosses the critical depth.
3. QSS ↔ T2       — the T0 stint lap-time decay and the T2 long-run decay agree to ≤ 0.1 s/lap.

Runs on the reference F1 (limebeer_2014_f1) + Catalunya at a coarse, flat, CI-speed grid.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from outlap.core import Track, solve_stint_dataset
from outlap.wearcal import calibrate, load_fixture
from outlap.wearcal.sim import sim_stint

_ROOT = Path(__file__).resolve().parents[2]
_F1 = _ROOT / "data" / "vehicles" / "limebeer_2014_f1"
_CATALUNYA = str(_ROOT / "data" / "tracks" / "catalunya_osm")
_FIXTURE = _ROOT / "data" / "wear" / "f1_medium_catalunya_stint.csv"
_FAST: dict[str, object] = {
    "flat_track": True,
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2},
}

# Published racing-slick surface-temperature band (Farroni TRT traces; F1 broadcast tyre-temp
# ranges): a healthy slick operates in roughly 85–115 °C at the tread surface.
_SLICK_TS_LO = 85.0
_SLICK_TS_HI = 115.0

# The QSS↔T2 stint-decay agreement gate (§13).
_DECAY_TOL_S_PER_LAP = 0.1


@pytest.fixture(scope="module")
def track() -> Track:
    return Track.load(_CATALUNYA)


def _decay_s_per_lap(lap_time_s: np.ndarray) -> float:
    laps = np.arange(1, lap_time_s.size + 1, dtype=np.float64)
    return float(np.polyfit(laps, lap_time_s, 1)[0])


# --- Gate 1: tyre thermal — warm-up + steady band ------------------------------------------


def test_thermal_warmup_and_steady_band(track: Track) -> None:
    """Cold start warms up monotonically; the settled surface temp is in the published slick band."""
    cold = solve_stint_dataset(
        str(_F1),
        track,
        n_laps=6,
        tier="t0",
        ds_m=12.0,
        sim=_FAST,
        tire_thermal=True,
        initial_tire_temp_c=20.0,
    )
    surf = np.asarray(cold["tire_surface_c"].values, dtype=np.float64)  # (lap, s)
    assert surf[0, 0] == pytest.approx(20.0, abs=1.0), "lap 1 starts at the cold seed"
    lap_end = surf[:, -1]
    assert np.all(np.diff(lap_end) > 0.0), "the tyre warms up monotonically lap-to-lap"
    assert lap_end[-1] > lap_end[0] + 30.0, "substantial warm-up over the first laps"

    # Settled (equilibrium-seeded) surface temperature lands in the published slick band.
    warm = solve_stint_dataset(
        str(_F1),
        track,
        n_laps=4,
        tier="t0",
        ds_m=12.0,
        sim=_FAST,
        tire_thermal=True,
        initial_tire_temp_c=None,
    )
    peak_ts = float(np.max(warm["tire_surface_c"].values))
    assert _SLICK_TS_LO <= peak_ts <= _SLICK_TS_HI, (
        f"settled peak surface temp {peak_ts:.1f} C outside the slick band "
        f"[{_SLICK_TS_LO}, {_SLICK_TS_HI}]"
    )


# --- Gate 2: wear / cliff reproduced after inverse calibration -----------------------------


def test_wear_cliff_reproduced_after_calibration(track: Track) -> None:
    """Calibrate the committed fixture, then confirm the real driver reproduces decay + a cliff."""
    obs = load_fixture(_FIXTURE)
    result = calibrate(obs)
    assert result.success and result.cliff_lap is not None

    sim = sim_stint(
        _F1,
        track,
        result.params,
        24,
        tier="t0",
        ds_m=14.0,
        sim=_FAST,
        initial_tire_temp_c=None,
    )
    lap_time = sim.lap_time_s
    wear = sim.wear_mm
    # Wear is monotone non-decreasing and crosses the critical depth (the cliff exists).
    assert np.all(np.diff(wear) >= -1e-9), "wear is monotone"
    crossed = np.nonzero(wear >= result.params.w_c)[0]
    assert crossed.size > 0, "the stint reaches the cliff (wear crosses w_c)"
    cliff_lap = int(crossed[0] + 1)
    # Net monotone pace loss (trend).
    assert lap_time[-3:].mean() > lap_time[:3].mean(), "net pace loss over the stint"
    # The cliff is a sigmoid inflection: the per-lap pace loss ramps up, peaks as the tread crosses
    # w_c (the maximum degradation rate), then tapers as grip saturates — so the peak per-lap loss is
    # much larger than the fresh-tyre loss and occurs near the cliff.
    deltas = np.diff(lap_time)
    peak_delta_lap = int(np.argmax(deltas)) + 1
    assert deltas.max() > 2.0 * abs(deltas[0]), "degradation accelerates into the cliff"
    assert abs(peak_delta_lap - cliff_lap) <= 4, "the peak degradation rate sits near the cliff"
    # Recorded (docs/validation/wear-cliff.md): decay s/lap and cliff lap.
    assert 0.02 < _decay_s_per_lap(lap_time) < 1.0


# --- Gate 3: QSS ↔ T2 stint-decay agreement ------------------------------------------------


def test_qss_t2_stint_decay_agreement(track: Track) -> None:
    """T0 stint lap-time decay and the T2 long-run decay agree to ≤ 0.1 s/lap (Decision #48)."""
    n_laps = 6
    t0 = solve_stint_dataset(
        str(_F1),
        track,
        n_laps=n_laps,
        tier="t0",
        ds_m=12.0,
        sim=_FAST,
        tire_thermal=True,
        initial_tire_temp_c=None,
    )
    t2 = solve_stint_dataset(
        str(_F1),
        track,
        n_laps=n_laps,
        tier="t2",
        ds_m=12.0,
        sim=_FAST,
        tire_thermal=True,
        initial_tire_temp_c=None,
    )
    # The T2 closed loop must complete the run for the comparison to be meaningful.
    assert str(t2.attrs.get("completed")) in ("1", "True"), "T2 stint completed"
    decay_t0 = _decay_s_per_lap(np.asarray(t0["lap_time_s"].values, dtype=np.float64))
    decay_t2 = _decay_s_per_lap(np.asarray(t2["lap_time_s"].values, dtype=np.float64))
    # Both tiers degrade in the same direction (tyres lose pace) ...
    assert decay_t0 > 0.0 and decay_t2 >= 0.0, (decay_t0, decay_t2)
    # ... and agree on the rate to within the gate. The residual is a recorded driver-margin effect
    # (T2 runs at speed_margin 0.85, sliding — hence wear-driven decay — is lower than T0's
    # grip-limited pace), decomposed in docs/validation/wear-cliff.md.
    assert abs(decay_t0 - decay_t2) <= _DECAY_TOL_S_PER_LAP, (
        f"QSS↔T2 decay disagreement {abs(decay_t0 - decay_t2):.3f} s/lap "
        f"(T0 {decay_t0:.3f}, T2 {decay_t2:.3f}) exceeds {_DECAY_TOL_S_PER_LAP}"
    )
