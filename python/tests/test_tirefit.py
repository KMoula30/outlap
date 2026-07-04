# SPDX-License-Identifier: AGPL-3.0-only
"""tirefit tests: golden-CSV parity (same files + tolerance rule as Rust), synthetic
recovery, data parsing/sign conventions, and the CLI."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from outlap.tirefit import (
    FitConfig,
    bin_sweeps,
    forces,
    load_csv,
    load_dat,
    params_from_coeffs,
    params_from_tyr,
    staged_fit,
    synthesize,
)
from outlap.tirefit.data import sae_to_iso

_ROOT = Path(__file__).resolve().parents[2]
_PACEJKA = _ROOT / "data" / "tires" / "pacejka_2006_205_60r15" / "car.tyr.yaml"
_GOLDEN = (
    _ROOT / "crates" / "outlap-tire" / "tests" / "golden" / "pacejka_2006_205_60r15"
)

# The Rust golden gate, verbatim (crates/outlap-tire/tests/golden.rs).
_REL = 0.005
_PDX1 = 1.210
_PDY1_ABS = 0.990
_R0 = 0.313
_QSY1 = 0.01


def _pacejka_coeffs() -> dict[str, float]:
    tyr = yaml.safe_load(_PACEJKA.read_text(encoding="utf-8"))
    return {str(k): float(v) for k, v in tyr["mf61"].items()}


def _pacejka_params() -> dict[str, float]:
    return params_from_tyr(yaml.safe_load(_PACEJKA.read_text(encoding="utf-8")))


# --- Golden parity: the Python forward model against THE SAME oracle CSVs as Rust ----------


@pytest.mark.parametrize(
    "filename", ["fx0.csv", "fy0_mz.csv", "combined.csv", "combined_camber.csv"]
)
def test_forward_model_matches_golden_csvs(filename: str) -> None:
    rows = np.loadtxt(_GOLDEN / filename, delimiter=",", comments="#", skiprows=3)
    kappa, alpha, gamma = rows[:, 0], rows[:, 1], rows[:, 2]
    fz, pres, vx = rows[:, 3], rows[:, 4], rows[:, 5]
    ref = {
        "fx": rows[:, 6],
        "fy": rows[:, 7],
        "mz": rows[:, 8],
        "mx": rows[:, 9],
        "my": rows[:, 10],
    }
    out = forces(_pacejka_params(), kappa, alpha, gamma, fz, pres, vx)

    # Per-Fz-bin max |Mz| floor, exactly as golden.rs computes it.
    fz_key = np.round(fz)
    mz_floor = {
        z: float(np.max(np.abs(ref["mz"][fz_key == z]))) for z in np.unique(fz_key)
    }
    floors = {
        "fx": _REL * _PDX1 * fz,
        "fy": _REL * _PDY1_ABS * fz,
        "mz": _REL * np.array([mz_floor[z] for z in fz_key]),
        "mx": np.full_like(fz, 1e-6),
        "my": _REL * _R0 * _QSY1 * fz,
    }
    for channel in ("fx", "fy", "mz", "mx", "my"):
        model = getattr(out, channel)
        tol = np.maximum(_REL * np.abs(ref[channel]), floors[channel])
        excess = np.abs(model - ref[channel]) - tol
        worst = int(np.argmax(excess))
        assert float(excess[worst]) <= 0.0, (
            f"{filename}/{channel}: model {model[worst]:.4f} vs ref "
            f"{ref[channel][worst]:.4f} (tol {tol[worst]:.4f}) at row {worst}"
        )


def test_airborne_wheel_is_exactly_zero() -> None:
    p = _pacejka_params()
    out = forces(
        p,
        np.array([0.1]),
        np.array([0.05]),
        np.array([0.0]),
        np.array([0.0]),  # Fz = 0: airborne
        np.array([220_000.0]),
        np.array([16.67]),
    )
    for channel in ("fx", "fy", "mz", "mx", "my"):
        assert float(getattr(out, channel)[0]) == 0.0


# --- Synthetic recovery: book tyre + seeded noise → fit → curves ≤ 1% -----------------------


def test_synthetic_recovery() -> None:
    pytest.importorskip("scipy")
    truth = _pacejka_coeffs()
    data = synthesize(truth, seed=42, noise=0.01)
    result = staged_fit(
        data,
        FitConfig(unloaded_radius_m=0.313, fnomin_n=4000.0, nompres_pa=220_000.0),
    )

    # Headline parameters (peak grip + slip stiffness + trail peak) within a few percent.
    for key, tol in [
        ("PDX1", 0.03),
        ("PDY1", 0.03),
        ("PKX1", 0.05),
        ("PKY1", 0.05),
        ("QDZ1", 0.05),
    ]:
        got = result.coeffs[key]
        want = truth[key]
        assert abs(got - want) <= tol * abs(want), f"{key}: fitted {got}, truth {want}"

    # Whole-curve agreement ≤ 1% of the channel peak on pure sweeps at FNOMIN.
    n = 101
    zeros = np.zeros(n)
    fz = np.full(n, 4000.0)
    pres = np.full(n, 220_000.0)
    vx = np.full(n, 16.67)
    p_truth = params_from_coeffs(truth)
    p_fit = params_from_coeffs(result.coeffs)

    alpha = np.linspace(-0.15, 0.15, n)
    ref = forces(p_truth, zeros, alpha, zeros, fz, pres, vx)
    fit = forces(p_fit, zeros, alpha, zeros, fz, pres, vx)
    assert float(np.max(np.abs(fit.fy - ref.fy)) / np.max(np.abs(ref.fy))) <= 0.01

    kappa = np.linspace(-0.2, 0.2, n)
    ref = forces(p_truth, kappa, zeros, zeros, fz, pres, vx)
    fit = forces(p_fit, kappa, zeros, zeros, fz, pres, vx)
    assert float(np.max(np.abs(fit.fx - ref.fx)) / np.max(np.abs(ref.fx))) <= 0.01


def test_synthesize_is_deterministic() -> None:
    truth = _pacejka_coeffs()
    a = synthesize(truth, seed=7, noise=0.02)
    b = synthesize(truth, seed=7, noise=0.02)
    assert np.array_equal(a.fy_n, b.fy_n)
    c = synthesize(truth, seed=8, noise=0.02)
    assert not np.array_equal(a.fy_n, c.fy_n)


def test_noise_free_moment_channels_skip_their_stages() -> None:
    pytest.importorskip("scipy")
    # The book tyre has Mx ≡ 0: the mxmy stage must skip rather than fit noise into QSX*.
    truth = _pacejka_coeffs()
    data = synthesize(truth, seed=1, noise=0.005)
    result = staged_fit(
        data, FitConfig(unloaded_radius_m=0.313, fnomin_n=4000.0, nompres_pa=220_000.0)
    )
    mxmy = next(s for s in result.stages if s.name == "mxmy")
    assert mxmy.skipped is not None
    assert "QSX1" not in result.coeffs


def test_mx_stage_never_fabricates_rolling_resistance() -> None:
    pytest.importorskip("scipy")
    # A truth WITH Mx (QSX*) but no QSY*: the mx stage runs, and QSY* must not appear in the
    # output (their residual Jacobian against the mx channel is identically zero — freeing
    # them would return their inits as fake fitted values).
    truth = _pacejka_coeffs() | {"QSX2": 0.6, "QSX3": 0.05}
    data = synthesize(truth, seed=3, noise=0.005)
    result = staged_fit(
        data, FitConfig(unloaded_radius_m=0.313, fnomin_n=4000.0, nompres_pa=220_000.0)
    )
    mx_stage = next(s for s in result.stages if s.name == "mxmy")
    assert mx_stage.skipped is None, "Mx signal present: the stage should run"
    assert "QSY1" not in result.coeffs and "QSY2" not in result.coeffs


def test_nonpositive_nompres_or_longvl_rejected_like_rust() -> None:
    base = _pacejka_coeffs()
    with pytest.raises(ValueError, match="NOMPRES"):
        params_from_coeffs(base | {"NOMPRES": 0.0})
    with pytest.raises(ValueError, match="LONGVL"):
        params_from_coeffs(base | {"LONGVL": 0.0})


# --- Data ingestion --------------------------------------------------------------------------


def test_sae_to_iso_sign_map() -> None:
    one = np.array([1.0])
    fields = {
        "alpha_rad": one,
        "gamma_rad": one,
        "fx_n": one,
        "fy_n": one,
        "fz_n": -one,  # SAE logs compression negative
        "mx_nm": one,
        "mz_nm": one,
        "kappa": one,
    }
    iso = sae_to_iso(fields)
    assert iso["alpha_rad"][0] == -1.0
    assert iso["gamma_rad"][0] == -1.0
    assert iso["fy_n"][0] == -1.0
    assert iso["fz_n"][0] == 1.0  # positive load in ISO
    assert iso["mz_nm"][0] == -1.0
    assert iso["mx_nm"][0] == 1.0  # x axis unchanged under the π rotation about x
    assert iso["fx_n"][0] == 1.0
    assert iso["kappa"][0] == 1.0


def test_load_dat_and_bin_sweeps(tmp_path: Path) -> None:
    # A tiny TTC-shaped .dat: title, names, units, then columns (SAE signs, TTC units).
    lines = ["synthetic mini rig", "ET SA IA FZ P V FY", "s deg deg N kPa kph N"]
    rng = np.random.default_rng(3)
    for fz in (-1100.0, -2200.0):
        for sa in np.linspace(-12.0, 12.0, 25):
            fy = 2.0 * fz * np.tanh(0.2 * sa) + float(rng.normal(0.0, 5.0))
            lines.append(f"0.0 {sa:.3f} 0.0 {fz:.1f} 82.7 40.0 {fy:.2f}")
    path = tmp_path / "mini.dat"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    data = load_dat(path)
    assert len(data) == 50
    # SAE -> ISO: loads positive, pressure in Pa, speed in m/s.
    assert np.all(data.fz_n > 0.0)
    assert data.p_pa[0] == pytest.approx(82_700.0)
    assert data.vx_mps[0] == pytest.approx(40.0 / 3.6)

    bins = bin_sweeps(data, min_samples=10)
    assert len(bins) == 2
    assert bins[0].fz_n < bins[1].fz_n


def test_load_csv_iso_passthrough(tmp_path: Path) -> None:
    path = tmp_path / "iso.csv"
    path.write_text(
        "SA,FZ,FY\n"
        + "\n".join(f"{a},3000.0,{-a * 100.0}" for a in range(-5, 6))
        + "\n",
        encoding="utf-8",
    )
    data = load_csv(path, sae_signs=False)
    assert np.all(data.fz_n == 3000.0)
    assert data.alpha_rad[0] == pytest.approx(np.radians(-5.0))


# --- CLI --------------------------------------------------------------------------------------


def test_cli_synth_then_fit(tmp_path: Path) -> None:
    pytest.importorskip("scipy")
    from outlap.tirefit.__main__ import main as tirefit_main

    csv_out = tmp_path / "synth.csv"
    assert (
        tirefit_main(["synth", str(_PACEJKA), "-o", str(csv_out), "--seed", "5"]) == 0
    )

    tyr_out = tmp_path / "fitted.tyr.yaml"
    report_dir = tmp_path / "report"
    rc = tirefit_main(
        [
            "fit",
            str(csv_out),
            "--unloaded-radius",
            "0.313",
            "--fnomin",
            "4000",
            "--nompres",
            "220000",
            "-o",
            str(tyr_out),
            "--report-dir",
            str(report_dir),
        ]
    )
    assert rc == 0
    fitted = yaml.safe_load(tyr_out.read_text(encoding="utf-8"))
    assert fitted["provenance"]["synthetic"] is True
    assert abs(fitted["mf61"]["PDX1"] - 1.210) < 0.06
    assert (report_dir / "report.json").exists()
    assert (report_dir / "report.md").exists()
