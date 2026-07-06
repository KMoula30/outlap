# SPDX-License-Identifier: AGPL-3.0-only
"""Tests for the outlap_core bindings + the outlap.core wrapper.

These pin the Python surface to values the Rust test suite already guarantees (peak μ of the
reference tyres, sign conventions, raceline improvement, lap sanity), so a binding-layer
regression — wrong argument order, unit slip, axis flip — surfaces here even though the physics
is tested in Rust.
"""

from pathlib import Path

import numpy as np
import pytest
import xarray as xr

from outlap.core import (
    Track,
    Tyre,
    min_curvature,
    solve_lap_dataset,
    track_dataset,
    tyre_forces,
    vehicle_report,
)

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"

PACEJKA = str(_DATA / "tires/pacejka_2006_205_60r15/car.tyr.yaml")
F1_DIR = str(_DATA / "vehicles/f1_2026")
F1_SLICK = str(_DATA / "vehicles/f1_2026/tyr/slick.tyr.yaml")
# The OSM+DEM 3D import: these core tests exercise elevation and flat-track flattening, which the
# flat TUMFTM `catalunya` (z=0) cannot. `catalunya_osm` is the same geometry these were written on.
CATALUNYA = str(_DATA / "tracks/catalunya_osm")

# CI-speed envelope for the plumbing tests below: what they assert (dataset shape, attrs,
# overrides plumbing, determinism) is fidelity-independent, and every distinct override /
# conditions combination generates its own g-g-g-v envelope (a cold step). The physics gates
# (Limebeer <=1%, golden laps) run at the full production 40x25x7 grid in their own tests.
COARSE_SIM: dict[str, object] = {
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}
}


def solve_fast(vehicle_dir: str, line: object, **kw: object) -> xr.Dataset:
    """A CI-speed lap: point-mass tier on a coarse envelope (plumbing tests only)."""
    kw.setdefault("tier", "t0")
    kw.setdefault("sim", COARSE_SIM)
    return solve_lap_dataset(vehicle_dir, line, **kw)  # type: ignore[arg-type]


@pytest.fixture(scope="module")
def pacejka() -> Tyre:
    return Tyre.load(PACEJKA)


@pytest.fixture(scope="module")
def catalunya() -> Track:
    return Track.load(CATALUNYA)


def test_tyre_metadata_and_peaks(pacejka: Tyre) -> None:
    assert pacejka.fnomin == 4000.0
    assert pacejka.unloaded_radius == 0.313
    assert "Pacejka" in pacejka.citation
    # Rust tests pin these bands (reference.rs); the binding must reproduce them.
    mux, muy = pacejka.peak_mu(4000.0, 220_000.0)
    assert 1.15 < mux < 1.30
    assert 0.90 < muy < 1.10
    assert mux > muy


def test_slick_peaks_match_rust_closed_form() -> None:
    # The synthetic slick: peak μ = PDX1/PDY1 exactly (Rust eval.rs pins 1.30/1.25).
    slick = Tyre.load(F1_SLICK)
    mux, muy = slick.peak_mu(4000.0, 200_000.0)
    assert mux == pytest.approx(1.30, abs=1e-6)
    assert muy == pytest.approx(1.25, abs=1e-6)


def test_tyre_forces_signs_and_broadcasting(pacejka: Tyre) -> None:
    # ISO-W sign contract through the FFI: drive → Fx > 0; α > 0 → Fy < 0, restoring Mz > 0.
    f = tyre_forces(pacejka, kappa=np.array([0.08, -0.08]))
    assert f.fx[0] > 0 > f.fx[1]
    f = tyre_forces(pacejka, alpha=0.06)  # scalar broadcast → 0-d arrays
    assert f.fy.item() < 0 < f.mz.item()
    # Shape is preserved through broadcast (grid eval).
    kk, aa = np.meshgrid(np.linspace(-0.1, 0.1, 7), np.linspace(-0.1, 0.1, 5))
    g = tyre_forces(pacejka, kappa=kk, alpha=aa)
    assert g.fx.shape == (5, 7)
    assert np.isfinite(g.fx).all() and np.isfinite(g.mz).all()


def test_tyre_forces_length_mismatch_raises(pacejka: Tyre) -> None:
    with pytest.raises(ValueError, match="length mismatch"):
        pacejka.forces(
            np.zeros(3),
            np.zeros(2),
            np.zeros(3),
            np.full(3, 4000.0),
            np.full(3, 2e5),
            np.full(3, 16.7),
        )


def test_missing_file_raises() -> None:
    with pytest.raises(FileNotFoundError):
        Tyre.load(str(_DATA / "tires/nope.tyr.yaml"))


def test_track_load_and_dataset(catalunya: Track) -> None:
    assert catalunya.is_closed()
    assert 4000 < catalunya.length() < 5500
    ds = track_dataset(catalunya, ds_m=25.0)
    assert ds.attrs["name"].startswith("Circuit")
    assert ds.sizes["s"] > 100
    # The imported Catalunya has real elevation: z varies by tens of metres.
    assert float(ds.z.max() - ds.z.min()) > 10.0


def test_raceline_improves_lap(catalunya: Track) -> None:
    rl = min_curvature(catalunya, 1.1)
    n = rl.n()
    assert np.abs(n).max() > 1.0  # actually moves off the centerline
    assert rl.ds_m == 2.0  # generation step recorded for provenance
    lap_c = solve_fast(F1_DIR, catalunya)
    lap_r = solve_fast(F1_DIR, rl)  # Raceline accepted directly
    assert lap_r.attrs["lap_time_s"] < lap_c.attrs["lap_time_s"]


def test_lap_dataset_shape_and_sanity(catalunya: Track) -> None:
    lap = solve_fast(F1_DIR, catalunya)
    assert 60.0 < lap.attrs["lap_time_s"] < 200.0
    for var in ("v", "ax", "ay", "t", "x", "y", "z"):
        assert var in lap
    t = lap.t.to_numpy()
    assert (np.diff(t) > 0).all()  # time strictly increases along s
    assert float(lap.v.min()) > 0.0
    assert lap.attrs["resolved_hash"]  # provenance recorded
    # Tuple (netCDF-serializable attrs), never a bare list.
    assert isinstance(lap.attrs["notes"], tuple)


def test_vehicle_report_surface() -> None:
    rep = vehicle_report(F1_DIR)
    assert rep["name"]
    assert isinstance(rep["estimated"], list)
    # The f1_2026 reference vehicle has estimated K&C values — nothing silent.
    assert len(rep["estimated"]) > 0


def test_gamma_is_third_argument_not_alpha(pacejka: Tyre) -> None:
    # Camber-only input must produce a much smaller lateral force than the same angle as
    # side-slip — a swapped alpha/gamma in the FFI would make these comparable.
    f_alpha = tyre_forces(pacejka, alpha=0.06)
    f_gamma = tyre_forces(pacejka, gamma=0.06)
    assert abs(f_alpha.fy.item()) > 5.0 * abs(f_gamma.fy.item())


def test_p_cold_is_pascals(pacejka: Tyre) -> None:
    # thermal.p_cold is kPa in the file; the binding converts to Pa (2.2 bar → 220000).
    assert pacejka.p_cold == pytest.approx(220_000.0)
    # And peak_mu at that pressure matches the reference band used everywhere else.
    mux, _ = pacejka.peak_mu(pacejka.fnomin, pacejka.p_cold)
    assert 1.15 < mux < 1.30


def test_lap_is_deterministic(catalunya: Track) -> None:
    a = solve_fast(F1_DIR, catalunya)
    b = solve_fast(F1_DIR, catalunya)
    assert a.attrs["lap_time_s"] == b.attrs["lap_time_s"]
    assert np.array_equal(a.v.to_numpy(), b.v.to_numpy())


def test_bad_ds_raises_not_panics(catalunya: Track) -> None:
    from outlap.core import solve_lap as raw_solve

    for bad in (0.0, -1.0, float("nan")):
        with pytest.raises(ValueError, match="ds_m"):
            raw_solve(F1_DIR, catalunya, ds_m=bad)
        with pytest.raises(ValueError, match="ds_m"):
            min_curvature(catalunya, 1.1, ds_m=bad)


def test_override_mass_slows_the_lap(catalunya: Track) -> None:
    base = solve_fast(F1_DIR, catalunya)
    heavy = solve_fast(F1_DIR, catalunya, overrides={"chassis.mass_kg": 968.0})
    assert heavy.attrs["lap_time_s"] > base.attrs["lap_time_s"]
    # The overridden car is a different resolved spec — provenance hash must change.
    assert heavy.attrs["resolved_hash"] != base.attrs["resolved_hash"]


def test_override_bad_path_fails_loudly(catalunya: Track) -> None:
    with pytest.raises(ValueError, match="mas_kg"):
        solve_fast(F1_DIR, catalunya, overrides={"chassis.mas_kg": 900.0})


def test_override_bad_type_fails_loudly(catalunya: Track) -> None:
    with pytest.raises(ValueError):
        solve_fast(F1_DIR, catalunya, overrides={"chassis.mass_kg": "heavy"})


def test_conditions_change_the_lap(catalunya: Track) -> None:
    cold = solve_fast(F1_DIR, catalunya, conditions={"air": {"temperature_c": 0.0}})
    hot = solve_fast(F1_DIR, catalunya, conditions={"air": {"temperature_c": 45.0}})
    # Air density changes drag AND downforce; the laps must differ measurably.
    assert cold.attrs["lap_time_s"] != hot.attrs["lap_time_s"]
    assert abs(cold.attrs["lap_time_s"] - hot.attrs["lap_time_s"]) > 0.05


def test_conditions_typo_fails_loudly(catalunya: Track) -> None:
    # A misspelled conditions field must never be silently dropped.
    with pytest.raises(ValueError, match="air.temp_c"):
        solve_fast(F1_DIR, catalunya, conditions={"air": {"temp_c": 30.0}})


def test_vehicle_report_echoes_overrides() -> None:
    rep = vehicle_report(F1_DIR, overrides={"chassis.mass_kg": 800.0})
    applied = rep["overrides"]
    assert isinstance(applied, list)
    assert ("chassis.mass_kg", "800.0") in applied
    base = vehicle_report(F1_DIR)
    assert rep["resolved_hash"] != base["resolved_hash"]


def test_malformed_conditions_is_an_error(tmp_path: Path) -> None:
    # Copy the f1_2026 vehicle dir and plant a broken conditions.yaml: solve_lap must raise,
    # never silently fall back to ISA defaults (nothing silent).
    import shutil

    vdir = tmp_path / "veh"
    shutil.copytree(F1_DIR, vdir)
    (vdir / "conditions.yaml").write_text(
        "schema: conditions/1.0\nair: {temp_c: [not, a, number]}\n"
    )
    tr = Track.load(CATALUNYA)
    from outlap.core import solve_lap as raw_solve

    with pytest.raises(ValueError):
        raw_solve(str(vdir), tr)


# --- PR8: tier dispatch + the extended result surface -------------------------------------------


def test_t0_dataset_stays_s_only(catalunya: Track) -> None:
    lap = solve_fast(F1_DIR, catalunya)
    assert lap.attrs["tier"] == "t0"
    assert lap.attrs["fz_coupling"] == "one_step_lag"
    assert lap.attrs["flat_track"] == 0
    assert "wheel" not in lap.sizes  # backward-compatible: point-mass laps stay s-only
    assert "vertical_load_n" not in lap


def test_t1_dataset_has_wheel_dim_and_channels(catalunya: Track) -> None:
    lap = solve_lap_dataset(F1_DIR, catalunya, ds_m=25.0, tier="t1", sim=COARSE_SIM)
    assert lap.attrs["tier"] == "t1"
    # The point-mass channels stay present (additive extension).
    for var in ("v", "ax", "ay", "t", "x", "y", "z"):
        assert var in lap
    # Per-wheel channels over (s, wheel), FL/FR/RL/RR.
    assert list(lap.coords["wheel"].values) == ["FL", "FR", "RL", "RR"]
    for var in (
        "vertical_load_n",
        "slip_ratio",
        "slip_angle_rad",
        "force_long_n",
        "force_lat_n",
    ):
        assert lap[var].dims == ("s", "wheel"), var
    # Setup metrics per station.
    assert lap.understeer_gradient.dims == ("s",)
    assert lap.aero_front_share.dims == ("s",)
    # Wheel loads: converged stations carry a plausible total (weight + downforce).
    fz = lap.vertical_load_n.to_numpy()
    finite = np.isfinite(fz).all(axis=1)
    assert finite.mean() > 0.5, "most stations must re-trim"
    totals = fz[finite].sum(axis=1)
    assert (totals > 0.5 * 798.0 * 9.81).all()


def test_flat_track_mode_records_and_flattens(catalunya: Track) -> None:
    flat = solve_fast(F1_DIR, catalunya, sim={**COARSE_SIM, "flat_track": True})
    full = solve_fast(F1_DIR, catalunya)
    assert flat.attrs["flat_track"] == 1
    assert full.attrs["flat_track"] == 0
    # Catalunya has real elevation; flattening it must change the lap.
    assert flat.attrs["lap_time_s"] != full.attrs["lap_time_s"]


def test_transient_tiers_raise_typed_errors(catalunya: Track) -> None:
    for tier, milestone in (("t2", "M4"), ("t3", "M6")):
        with pytest.raises(ValueError, match=milestone):
            solve_lap_dataset(F1_DIR, catalunya, tier=tier, sim=COARSE_SIM)


def test_unknown_sim_field_fails_loudly(catalunya: Track) -> None:
    with pytest.raises(ValueError, match="sim.flat_trak"):
        solve_lap_dataset(F1_DIR, catalunya, sim={"flat_trak": True})


def test_envelope_is_returnable(catalunya: Track) -> None:
    from outlap.core import solve_lap as raw_solve

    lap = raw_solve(F1_DIR, catalunya, tier="t0", sim=COARSE_SIM)
    env = lap.envelope
    assert env is not None
    # The boundary is a physical, positive lateral limit inside the domain.
    (v_lo, v_hi), _, _ = env.domain()
    v_mid = 0.5 * (v_lo + v_hi)
    assert env.ay_boundary(v_mid, 0.0, 9.81) > 5.0
    assert env.accel_limit(v_mid, 9.81) > 0.0
    assert env.notes
