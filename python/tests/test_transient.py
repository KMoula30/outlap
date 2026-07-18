# SPDX-License-Identifier: AGPL-3.0-only
"""Tests for the transient (T2) tier at the Python boundary.

These pin the *plumbing* the Rust suite cannot see: that a closed-loop lap reaches the finish line
through the real assembly pipeline, that its dataset is time-indexed with `(time, wheel)` per-wheel
channels, that the rule-based control layer's telemetry survives the boundary, and that the
series regen blend behaves at the two ends its physics demands — each machine braking only its own
axle, and a full pack refusing charge entirely.

The physics itself is gated in Rust (`outlap-transient/tests`, `outlap-vehicle`, `outlap-qss`).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import xarray as xr

from outlap.core import (
    DEFAULT_DS_M,
    Track,
    solve_lap,
    solve_lap_dataset,
    solve_transient_lap,
    transient_lap_dataset,
)

_ROOT = Path(__file__).resolve().parents[2]
_DATA = _ROOT / "data"

CATALUNYA = str(_DATA / "tracks/catalunya_osm")
LIMEBEER = str(_DATA / "vehicles/limebeer_2014_f1")
# The only committed car that carries a battery *and* its ECM sidecar: a rear-drive EV, so the front
# axle has no machine and must recover nothing.
MODEL3_RWD = str(_DATA / "vehicles/tesla_model3_rwd")
# The only committed car with a multi-ratio gearbox (8 speeds) — exercises the shift FSM.
F1_2026 = str(_DATA / "vehicles/f1_2026")

# CI-speed envelope: what these tests assert (dataset shape, attrs, regen sign conventions) is
# fidelity-independent, and every distinct override regenerates the g-g-g-v envelope (a cold step).
COARSE_SIM: dict[str, object] = {
    "envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}
}

WHEELS = ["FL", "FR", "RL", "RR"]


@pytest.fixture(scope="module")
def catalunya() -> Track:
    return Track.load(CATALUNYA)


@pytest.fixture(scope="module")
def f1_lap(catalunya: Track) -> xr.Dataset:
    """A transient lap of the reference F1 car (no battery ⇒ no regen channels)."""
    return solve_lap_dataset(
        LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, tier="t2", sim=COARSE_SIM
    )


@pytest.fixture(scope="module")
def ev_lap(catalunya: Track) -> xr.Dataset:
    """A transient lap of the rear-drive EV (battery + regen blend ⇒ the full slow stack)."""
    return transient_lap_dataset(
        solve_transient_lap(MODEL3_RWD, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM)
    )


# --- The dataset contract ------------------------------------------------------------------------


def test_transient_dataset_is_time_indexed(f1_lap: xr.Dataset) -> None:
    """T2 integrates in time, so `time` is the dimension and `s` is a data variable on it."""
    assert "time" in f1_lap.sizes
    assert "s" not in f1_lap.sizes, (
        "arc length is a variable, not a dimension, in a transient lap"
    )
    assert f1_lap["s"].dims == ("time",)
    t = f1_lap["time"].values
    assert (np.diff(t) > 0).all(), "time advances monotonically on a fixed-step grid"
    # A fixed step: every interval is dt.
    assert np.allclose(np.diff(t), f1_lap.attrs["dt_s"])


def test_per_wheel_channels_carry_time_and_wheel_dims(f1_lap: xr.Dataset) -> None:
    assert list(f1_lap.coords["wheel"].values) == WHEELS
    for name in (
        "omega",
        "vertical_load_n",
        "slip_ratio",
        "slip_angle_rad",
        "force_long_n",
        "force_lat_n",
    ):
        assert f1_lap[name].dims == ("time", "wheel"), name
    assert (f1_lap["vertical_load_n"] >= 0.0).all(), "normal loads never go negative"


def test_control_layer_telemetry_crosses_the_boundary(f1_lap: xr.Dataset) -> None:
    for name in (
        "gear",
        "torque_scale",
        "yaw_moment_nm",
        "regen_power_w",
        "traction_power_w",
        "regen_torque_front_nm",
        "regen_torque_rear_nm",
    ):
        assert f1_lap[name].dims == ("time",), name
    # limebeer is single-speed (its drive unit is defined at the wheel shaft), so no shift interrupts
    # the drive torque — gears change and torque cuts are covered by the f1_2026 test below.
    assert (f1_lap["torque_scale"] == 1.0).all()
    assert (f1_lap["regen_power_w"] >= 0.0).all() and (
        f1_lap["traction_power_w"] >= 0.0
    ).all(), "regen and traction power are non-negative magnitudes"


def test_provenance_attrs_are_recorded(f1_lap: xr.Dataset) -> None:
    a = f1_lap.attrs
    assert a["tier"] == "t2"
    assert a["fz_coupling"] == "fixed_point", "the T2 default coupling"
    assert a["dt_s"] > 0.0
    assert a["integrator_order"] in (2, 4)
    assert 0.0 < a["speed_margin"] <= 1.0
    assert a["completed"] == 1, "the lap reached the finish line"
    assert isinstance(a["notes"], tuple) and a["notes"], "nothing silent"
    assert a["resolved_hash"]
    # netCDF attrs carry no bool type.
    assert isinstance(a["flat_track"], int)


def test_the_car_completes_a_lap_and_tracks_the_qss_profile(
    catalunya: Track, f1_lap: xr.Dataset
) -> None:
    """The closed loop follows the point-mass reference: it finishes, and near its lap time."""
    t0 = solve_lap_dataset(
        LIMEBEER,
        catalunya,
        ds_m=DEFAULT_DS_M,
        tier="t0",
        sim={"flat_track": True, **COARSE_SIM},
    )
    ratio = f1_lap.attrs["lap_time_s"] / t0.attrs["lap_time_s"]
    # The driver tracks `speed_margin` of the profile and has no active yaw moment enabled on this
    # car, so it is slower — but it must be in the same league, not a different one.
    assert 1.0 < ratio < 1.4, f"T2/T0 lap-time ratio {ratio:.3f}"
    assert (f1_lap["vx"] > 0.0).all(), "the car never stops or reverses"
    assert float(f1_lap["vx"].max()) < 120.0, "and never runs away"


def test_transient_laps_are_deterministic(catalunya: Track) -> None:
    a = solve_transient_lap(LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM)
    b = solve_transient_lap(LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM)
    assert a.lap_time_s == b.lap_time_s
    np.testing.assert_array_equal(a.vx(), b.vx())


def test_the_tire_thermal_stack_is_opt_in_and_changes_the_lap(catalunya: Track) -> None:
    """M5 PR3: the ring+wear stack is opt-in (default off ⇒ frozen tyres, byte-identical), and turning
    it on makes the lap respond to tyre temperature/wear.

    With the still-synthetic reference `.tyr` params the loaded steady-state sits below the grip
    window, so the wired lap loses grip and is *slower* than frozen — that pace change is the wiring's
    first lap-level effect. FastF1 calibration (PR7/PR8) moves the steady-state into the window and the
    flag flips on by default there. The stack is deterministic and the lap still completes.
    """
    frozen = solve_transient_lap(LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM)
    warm = solve_transient_lap(
        LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM, tire_thermal=True
    )
    # Default off is byte-identical to the frozen path.
    default_off = solve_transient_lap(
        LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM
    )
    assert default_off.lap_time_s == frozen.lap_time_s, (
        "default is byte-identical frozen tyres"
    )

    # The wired lap completes, is deterministic, and its pace responds to the tyre state.
    warm_again = solve_transient_lap(
        LIMEBEER, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM, tire_thermal=True
    )
    assert warm.lap_time_s == warm_again.lap_time_s, "the wired lap is deterministic"
    assert warm.lap_time_s != frozen.lap_time_s, "the tyre thermal state moved the lap"
    assert float(np.asarray(warm.vx()).max()) < 120.0, (
        "the wired lap stays bounded (no runaway)"
    )


# --- Series regen braking, end to end -------------------------------------------------------------


def test_a_car_without_a_battery_has_no_slow_states(f1_lap: xr.Dataset) -> None:
    assert "state_of_charge" not in f1_lap
    assert "pack_temp_c" not in f1_lap
    assert float(f1_lap["regen_power_w"].max()) == 0.0, "nothing to recover into"


def test_each_machine_regens_only_its_own_axle(ev_lap: xr.Dataset) -> None:
    """A rear-drive EV: the front axle has no machine, so it recovers exactly nothing."""
    assert float(ev_lap["regen_torque_front_nm"].max()) == 0.0
    assert float(ev_lap["regen_torque_rear_nm"].max()) > 0.0
    assert float(ev_lap["regen_power_w"].max()) > 0.0


def test_state_of_charge_moves_both_ways_under_power_and_braking(
    ev_lap: xr.Dataset,
) -> None:
    """The pack discharges under power (the machines draw electrical energy) and charges under braking
    (regen), so the state of charge moves both ways over a lap — not a monotone rise."""
    soc = ev_lap["state_of_charge"].values
    assert soc.min() < soc.max(), "SoC is not flat"
    # Both directions are exercised across the lap.
    dsoc = np.diff(soc)
    assert (dsoc < 0).any(), "the pack discharges somewhere (under power)"
    assert (dsoc > 0).any(), "the pack charges somewhere (under braking)"
    # Both electrical channels are non-negative magnitudes and both are exercised.
    tp = ev_lap["traction_power_w"].values
    rp = ev_lap["regen_power_w"].values
    assert (tp >= 0).all() and (rp >= 0).all()
    assert float(tp.max()) > 0.0, "the machines draw traction power"
    assert float(rp.max()) > 0.0, "the machines recover regen power"


def test_net_energy_closes_the_state_of_charge(ev_lap: xr.Dataset) -> None:
    """ΔSoC tracks the net electrical energy (regen recovered − traction drawn) — the pack neither
    creates nor loses energy in the plumbing (the exact capacity is internal, so check the sign)."""
    t = ev_lap["time"].values
    recovered = float(np.trapezoid(ev_lap["regen_power_w"].values, t))
    drawn = float(np.trapezoid(ev_lap["traction_power_w"].values, t))
    soc = ev_lap["state_of_charge"].values
    net = recovered - drawn
    # Net and ΔSoC share a sign: draw-dominated ⇒ SoC falls, regen-dominated ⇒ SoC rises.
    assert np.sign(soc[-1] - soc[0]) == np.sign(net) or abs(net) < 1e3


def test_a_full_pack_discharges_to_make_headroom_then_regenerates(
    catalunya: Track,
) -> None:
    """The battery point: a stint seeded FULL (98%) is no longer stuck. Under power the machines draw
    it down, opening headroom, so regen recovers energy as the lap goes on — the SoC moves both ways
    rather than sitting pinned at a dead full pack."""
    lap = solve_transient_lap(
        MODEL3_RWD, catalunya, ds_m=DEFAULT_DS_M, sim=COARSE_SIM, initial_soc=0.98
    )
    ds = transient_lap_dataset(lap)
    soc = ds["state_of_charge"].values
    assert soc[0] == pytest.approx(0.98, abs=1e-6), "seeded full"
    assert soc.min() < 0.98, "the full pack discharges under power (makes headroom)"
    # Once there is headroom, regen recovers — a full pack is no longer a dead channel.
    assert float(ds["regen_power_w"].max()) > 0.0, "regen works once headroom opens"


def test_the_default_pack_seed_is_surfaced(ev_lap: xr.Dataset) -> None:
    assert any(
        "state of charge" in n and "estimated" in n for n in ev_lap.attrs["notes"]
    )


def test_initial_soc_is_validated(catalunya: Track) -> None:
    with pytest.raises(ValueError, match=r"initial_soc"):
        solve_transient_lap(MODEL3_RWD, catalunya, sim=COARSE_SIM, initial_soc=1.5)


# --- Tier dispatch and the product surface --------------------------------------------------------


def test_speed_margin_is_validated(catalunya: Track) -> None:
    with pytest.raises(ValueError, match=r"speed_margin"):
        solve_transient_lap(LIMEBEER, catalunya, sim=COARSE_SIM, speed_margin=0.0)


def test_a_three_dimensional_transient_completes_without_diverging(
    catalunya: Track,
) -> None:
    """The 3-D road frame is live (grade, banking, the elevated trajectory): a lap on the real
    elevated `catalunya_osm` completes, stays finite, and stays planted — no spin, no runaway."""
    lap = solve_transient_lap(
        LIMEBEER, catalunya, sim={"flat_track": False, **COARSE_SIM}
    )
    ds = transient_lap_dataset(lap)
    assert ds.attrs["flat_track"] == 0, "the lap ran the 3-D road frame"
    assert ds.attrs["completed"] == 1, "the car reached the finish line"
    vx = ds["vx"].values
    assert (vx > 0.0).all() and float(vx.max()) < 120.0, (
        "never stops, reverses, or runs away"
    )
    assert float(np.abs(ds["yaw_rate"].values).max()) < 5.0, "no spin"


def test_the_default_transient_lap_is_three_dimensional(f1_lap: xr.Dataset) -> None:
    """`flat_track` defaults to false, like the QSS tiers — the default T2 lap is the full 3-D road
    frame, and the crest-unloading closure is surfaced."""
    assert f1_lap.attrs["flat_track"] == 0
    assert any("3-D road frame" in n for n in f1_lap.attrs["notes"])


def test_a_flat_track_transient_lap_runs_flat_and_records_it(
    catalunya: Track,
) -> None:
    lap = transient_lap_dataset(
        solve_transient_lap(LIMEBEER, catalunya, sim={"flat_track": True, **COARSE_SIM})
    )
    assert lap.attrs["flat_track"] == 1
    assert any("flat-track" in n for n in lap.attrs["notes"])


def test_solve_lap_points_at_the_transient_entry_point(catalunya: Track) -> None:
    """`solve_lap` returns an arc-length `Lap`; the transient tier is time-indexed, so it redirects
    rather than raising a bare 'not implemented'."""
    with pytest.raises(ValueError, match="time-indexed"):
        solve_lap(LIMEBEER, catalunya, tier="t2", sim=COARSE_SIM)


def test_solve_lap_dataset_dispatches_t2_to_the_transient_tier(
    f1_lap: xr.Dataset,
) -> None:
    assert f1_lap.attrs["tier"] == "t2"
    assert "time" in f1_lap.sizes


# --- Gear-shift FSM -------------------------------------------------------------------------------


def test_a_geared_car_shifts_and_the_shifts_cut_torque(catalunya: Track) -> None:
    """The 8-speed f1_2026 changes gear as it speeds up, and each shift shows as a drive-torque cut
    (the §8.2 torque interruption). A single-speed car (limebeer) never shifts."""
    ds = solve_lap_dataset(
        F1_2026, catalunya, ds_m=DEFAULT_DS_M, tier="t2", sim=COARSE_SIM
    )
    gear = ds["gear"].values
    ts = ds["torque_scale"].values
    assert len(set(int(g) for g in np.unique(gear))) >= 2, (
        "the car uses more than one gear"
    )
    assert int((np.diff(gear) != 0).sum()) > 0, "gears change over the lap"
    # A shift interrupts the drive torque: torque_scale dips below 1 during the cut/ramp.
    assert float(ts.min()) < 1.0, "a shift cuts the drive torque"
    assert (ts >= 0.0).all() and (ts <= 1.0 + 1e-9).all(), (
        "torque_scale stays in [0, 1]"
    )
    assert any("gear-shift FSM:" in n for n in ds.attrs["notes"]), (
        "the shift plan is surfaced"
    )


def test_a_single_speed_car_never_shifts(f1_lap: xr.Dataset) -> None:
    # limebeer's drive unit is defined at the wheel shaft (no gearbox) → the FSM is inert.
    assert (f1_lap["gear"] == 0.0).all()
    assert (f1_lap["torque_scale"] == 1.0).all()
    assert any("gear-shift FSM inert" in n for n in f1_lap.attrs["notes"])


# --- M6 PR4: the 2026 ERS energy manager at T2 -------------------------------------------------


@pytest.fixture(scope="module")
def f1_ers_lap(catalunya: Track) -> xr.Dataset:
    """An f1_2026 T2 lap with the 2026 ERS energy manager active (the MGU-K deploys/harvests)."""
    return transient_lap_dataset(
        solve_transient_lap(F1_2026, catalunya, ds_m=12.0, sim=COARSE_SIM)
    )


def test_the_mgu_k_deploys_and_harvests_at_t2(f1_ers_lap: xr.Dataset) -> None:
    ds = f1_ers_lap
    # For an ERS car the powertrain publishes the manager's realized electrical deploy as
    # `traction_power_w` (pack draw = deploy only, D-M6-10) and the harvest as `regen_power_w`.
    deploy = ds["traction_power_w"].to_numpy()
    harvest = ds["regen_power_w"].to_numpy()
    assert deploy.max() > 100e3, (
        "the MGU-K actually deploys at T2 (it never did before PR4)"
    )
    assert harvest.max() > 10e3, "the MGU-K harvests under braking / recharge"
    # The realized electrical deploy respects the FIA C5.2.7 350 kW electrical cap at every sample.
    assert deploy.max() <= 350e3 * 1.001, (
        f"deploy {deploy.max() / 1e3:.1f} kW exceeds the 350 kW electrical cap"
    )
    assert (deploy >= -1e-6).all() and (harvest >= -1e-6).all()
    # The state of charge moves BOTH ways over the lap (the author's acceptance check at T2):
    # discharges under deploy, recovers under braking. The on-track swing stays inside the pack's
    # usable window (the f1 pack's window is sized to the FIA C5.2.9 4 MJ).
    soc = ds["state_of_charge"].to_numpy()
    assert soc.max() - soc.min() > 1e-3, "the SoC changes lap-over-lap (deploy + regen)"
    assert 0.2 - 1e-6 <= soc.min() and soc.max() <= 0.9 + 1e-6, (
        "the on-track SoC stays inside the usable window"
    )
    assert any("ERS energy manager active" in n for n in ds.attrs["notes"]), (
        "the ERS wiring is surfaced in the loaded-model report"
    )


def test_the_override_flag_changes_the_ers_lap(catalunya: Track) -> None:
    # Override ("Overtake") enables the higher-speed deployment envelope + the extra harvest
    # allowance, so the deploy/energy picture differs from the rule-based lap (D-M6-5).
    base = transient_lap_dataset(
        solve_transient_lap(F1_2026, catalunya, ds_m=12.0, sim=COARSE_SIM)
    )
    over = transient_lap_dataset(
        solve_transient_lap(
            F1_2026, catalunya, ds_m=12.0, sim=COARSE_SIM, override=True
        )
    )
    d_base = float(base["traction_power_w"].to_numpy().sum())
    d_over = float(over["traction_power_w"].to_numpy().sum())
    assert d_base != d_over, "the override flag changes the deployment picture"
    assert any("override/Overtake enabled" in n for n in over.attrs["notes"])
    assert not any("override/Overtake enabled" in n for n in base.attrs["notes"])


def test_a_us_schedule_drives_the_deploy_fraction(catalunya: Track) -> None:
    # A u(s) schedule that forbids deployment everywhere (deploy_regen = 0) banks strictly less
    # deploy energy than the greedy rule-based lap — the schedule policy actually reaches the tier.
    n = 60
    off = {"deploy_regen": [0.0] * n}
    greedy = transient_lap_dataset(
        solve_transient_lap(F1_2026, catalunya, ds_m=12.0, sim=COARSE_SIM)
    )
    scheduled = transient_lap_dataset(
        solve_transient_lap(
            F1_2026, catalunya, ds_m=12.0, sim=COARSE_SIM, us_schedule=off
        )
    )
    assert scheduled["traction_power_w"].to_numpy().max() <= (
        greedy["traction_power_w"].to_numpy().max() + 1.0
    ), "a deploy-off schedule never deploys more than the greedy policy"


def test_an_invalid_us_schedule_is_a_value_error(catalunya: Track) -> None:
    with pytest.raises(ValueError, match="us_schedule"):
        solve_transient_lap(
            F1_2026,
            catalunya,
            ds_m=20.0,
            sim=COARSE_SIM,
            us_schedule={"deploy_regen": [2.0, 0.0]},  # 2.0 is out of [-1, 1]
        )
