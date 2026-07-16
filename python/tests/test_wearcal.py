# SPDX-License-Identifier: AGPL-3.0-only
"""wearcal tests: surrogate physics invariants, synthetic round-trip recovery, the offline
fixture, and the CLI. The scipy fit is gated with importorskip (present in CI via `tire-fit`);
the live FastF1 path is gated with importorskip("fastf1") so it auto-skips in CI."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from outlap.wearcal import (
    CalibConfig,
    StintAnchor,
    SurrogateParams,
    calibrate,
    load_fixture,
    stint_trace,
    synth_observation,
)
from outlap.wearcal.model import (
    WEAR_HARDNESS_SENS_PER_C,
    WEAR_INV_HARDNESS_MAX,
    cliff,
    inv_hardness,
)

_ROOT = Path(__file__).resolve().parents[2]
_FIXTURE = _ROOT / "data" / "wear" / "f1_medium_catalunya_stint.csv"
_SLICK = _ROOT / "data" / "tires" / "roborace_devbot_mf52" / "car.tyr.yaml"


# --- Surrogate physics invariants (§14 property tests) -------------------------------------


def test_wear_monotone_nondecreasing() -> None:
    """Wear only grows lap-to-lap (Archard is non-negative) and clamps at w_max."""
    p = SurrogateParams(k_w=6e-9, w_max=5.0)
    tr = stint_trace(p, StintAnchor(), 40)
    assert np.all(np.diff(tr.wear_mm) >= -1e-12)
    assert tr.wear_mm.max() <= p.w_max + 1e-9


def test_grip_and_pace_monotone_in_wear() -> None:
    """As wear accumulates grip falls and lap time rises (warm start, no warm-up)."""
    tr = stint_trace(SurrogateParams(k_w=5e-9, delta_c=0.15), StintAnchor(), 30)
    assert np.all(np.diff(tr.grip) <= 1e-12)
    assert np.all(np.diff(tr.lap_time_s) >= -1e-9)
    assert tr.grip[0] <= 1.0 + 1e-12


def test_wear_monotone_in_k_w() -> None:
    """More wear coefficient → more wear at every lap (sign convention)."""
    a = StintAnchor()
    lo = stint_trace(SurrogateParams(k_w=2e-9), a, 20).wear_mm
    hi = stint_trace(SurrogateParams(k_w=8e-9), a, 20).wear_mm
    assert np.all(hi >= lo - 1e-12)
    assert hi[-1] > lo[-1]


def test_hardness_matches_rust_constants() -> None:
    """1/H(T_s) mirrors thermal.rs: exp(c_H·ΔT) capped at 20, unity at T_opt."""
    assert inv_hardness(95.0, 95.0) == pytest.approx(1.0)
    assert inv_hardness(115.0, 95.0) == pytest.approx(
        math.exp(WEAR_HARDNESS_SENS_PER_C * 20.0)
    )
    assert inv_hardness(1000.0, 95.0) == pytest.approx(WEAR_INV_HARDNESS_MAX)


def test_cliff_is_c1_continuous() -> None:
    """The grip cliff has a continuous first derivative — no kink (fine-grid finite difference)."""
    w_c, s_w, delta_c = 2.0, 0.4, 0.3
    grid = np.linspace(w_c - 2.0 * s_w, w_c + 2.0 * s_w, 400)
    dw = float(grid[1] - grid[0])
    curve = np.array([cliff(float(w), w_c, s_w, delta_c) for w in grid])
    deriv = np.diff(curve) / dw  # first derivative
    # A C¹ function's finite-difference derivative varies smoothly: no jump between neighbours.
    assert float(np.max(np.abs(np.diff(deriv)))) < 1e-3
    assert np.all(curve <= 1.0) and np.all(curve >= 1.0 - delta_c - 1e-9)


def test_damage_irreversible_and_costs_grip() -> None:
    """Above T_deg damage accumulates monotonically and only reduces grip."""
    p = SurrogateParams(t_deg=90.0, delta_d=0.4, tau_d=200.0, beta=2.0)
    hot = StintAnchor(t_op_c=95.0, t_c_c=130.0, lap_time_s=90.0)
    tr = stint_trace(p, hot, 25)
    assert np.all(np.diff(tr.damage) >= -1e-12)
    assert tr.damage[-1] > 0.0
    assert tr.grip[-1] < tr.grip[0]


def test_determinism() -> None:
    """Same inputs → bit-identical trace."""
    p = SurrogateParams(k_w=4e-9)
    a = StintAnchor()
    assert np.array_equal(
        stint_trace(p, a, 20).lap_time_s, stint_trace(p, a, 20).lap_time_s
    )


# --- Synthetic round-trip recovery (the PR7 gate) ------------------------------------------


def test_round_trip_recovers_known_params() -> None:
    """Generate a stint from known params → calibrate → recover them to tolerance."""
    pytest.importorskip("scipy")
    truth = SurrogateParams(k_w=4.5e-9, w_c=2.0, s_w=0.5, delta_c=0.14)
    anchor = StintAnchor(t_ref_s=85.8, t_op_c=truth.t_opt, t_c_c=truth.t_opt)
    obs = synth_observation(truth, anchor, 24, noise_s=0.0, label="truth")
    result = calibrate(obs, CalibConfig(base=SurrogateParams(), anchor=anchor))
    assert result.success
    assert result.rms_s < 1e-3
    assert result.fitted["k_w"] == pytest.approx(truth.k_w, rel=0.05)
    assert result.fitted["w_c"] == pytest.approx(truth.w_c, abs=0.15)
    assert result.fitted["delta_c"] == pytest.approx(truth.delta_c, abs=0.02)


def test_round_trip_robust_to_noise() -> None:
    """Recovery survives realistic lap-time noise (the fit is not overfit to exact values)."""
    pytest.importorskip("scipy")
    truth = SurrogateParams(k_w=5.0e-9, w_c=2.2, s_w=0.6, delta_c=0.12)
    anchor = StintAnchor(t_ref_s=86.0, t_op_c=truth.t_opt, t_c_c=truth.t_opt)
    obs = synth_observation(truth, anchor, 26, noise_s=0.03, seed=3, label="noisy")
    result = calibrate(obs, CalibConfig(base=SurrogateParams(), anchor=anchor))
    assert result.rms_s < 0.06
    assert result.fitted["k_w"] == pytest.approx(truth.k_w, rel=0.25)


# --- The committed offline fixture (PR9 wear/cliff gate reuses this) ------------------------


def test_fixture_loads_and_shows_decay_and_cliff() -> None:
    """The committed derived fixture is a monotone-ish decay curve with a visible cliff."""
    obs = load_fixture(_FIXTURE)
    assert obs.n_laps >= 20
    # A clear cliff: the late-stint per-lap deltas exceed the early ones.
    deltas = np.diff(obs.lap_time_s)
    assert float(np.median(deltas[-5:])) > float(np.median(deltas[:5]))
    assert obs.lap_time_s[-1] > obs.lap_time_s[0]


def test_calibrate_offline_fixture() -> None:
    """The calibrator fits the committed fixture and reports a physical decay + a cliff lap."""
    pytest.importorskip("scipy")
    obs = load_fixture(_FIXTURE)
    result = calibrate(obs)
    assert result.success
    assert result.rms_s < 0.1
    assert result.decay_s_per_lap > 0.0
    assert result.cliff_lap is not None
    # Recovered wear coefficient sits in the racing-slick band (not the broken synthetic default).
    assert 1e-10 < result.fitted["k_w"] < 1e-7


# --- CLI ------------------------------------------------------------------------------------


def test_cli_synth_then_calibrate(tmp_path: Path) -> None:
    """`synth` writes a fixture from a .tyr; `calibrate` fits it back to a .tyr."""
    pytest.importorskip("scipy")
    from outlap.wearcal.__main__ import main

    fixture = tmp_path / "stint.csv"
    out_tyr = tmp_path / "fitted.tyr.yaml"
    assert main(["synth", str(_SLICK), "-o", str(fixture), "--n-laps", "24"]) == 0
    assert fixture.exists()
    assert (
        main(
            [
                "calibrate",
                str(fixture),
                "--base",
                str(_SLICK),
                "-o",
                str(out_tyr),
                "--free",
                "k_w,w_c,s_w,delta_c",
                "--report-dir",
                str(tmp_path / "report"),
            ]
        )
        == 0
    )
    doc = yaml.safe_load(out_tyr.read_text(encoding="utf-8"))
    assert "thermal" in doc and "wear" in doc
    assert "wearcal" in doc["provenance"]["source"]
    assert (tmp_path / "report" / "report.json").exists()


def test_live_fastf1_loader_is_opt_in() -> None:
    """The live FastF1 loader is exercised only when fastf1 is installed (never in CI)."""
    pytest.importorskip("fastf1")
    from outlap.wearcal import load_fastf1

    assert callable(load_fastf1)
