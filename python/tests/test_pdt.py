# SPDX-License-Identifier: AGPL-3.0-only
"""Tests for the PDT HDF5 importers against synthetic PDT-shaped fixtures."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
import pytest
import yaml

import pdt_fixtures as fx
from outlap.importers.pdt_h5 import (
    PdtImportError,
    convert_batterypack,
    convert_driveunit,
    convert_edrive,
    validate_battery_doc,
)
from outlap.importers.pdt_h5.__main__ import main


def _read_map(
    parquet: Path, ns: int, nt: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = pq.read_table(parquet)
    speed = np.asarray(t.column("speed_rpm"))
    torque = np.asarray(t.column("torque_nm"))
    eff = np.asarray(t.column("efficiency")).reshape(ns, nt)
    speeds = speed.reshape(ns, nt)[:, 0]
    torques = torque.reshape(ns, nt)[0]
    return speeds, torques, eff


# --- EDrive ------------------------------------------------------------------------------------


def test_edrive_converts_and_validates(tmp_path: Path):
    src = tmp_path / "edrive.h5"
    fx.make_edrive(src)
    out = tmp_path / "machine.ptm.yaml"
    summary = convert_edrive(src, out, vdc=400.0)
    assert out.exists() and Path(summary["maps"]).exists()
    doc = yaml.safe_load(out.read_text())
    assert doc["schema"] == "ptm/1.0"
    assert doc["kind"] == "electric_machine"
    # load_axis torque grid == the axes torque grid (schema requires both).
    assert doc["axes"]["load_axis"]["torque_nm"] == doc["axes"]["torque_nm"]
    # Speed axis strictly ascending; meta carries alias + hash + vdc.
    sp = doc["axes"]["speed_rpm"]
    assert all(b > a for a, b in zip(sp, sp[1:]))
    assert (
        "synth_edrive" in doc["meta"]["source"]
        and "abcdef123456" in doc["meta"]["source"]
    )
    assert doc["meta"]["dc_voltage_v"] == 400.0
    assert doc["mass_kg"] == 20.0
    # Overload has the 3 durations.
    assert doc["limits"]["overload"]["durations_s"] == [10.0, 20.0, 30.0]


def test_edrive_spot_efficiencies_reproduce_to_1e6(tmp_path: Path):
    src = tmp_path / "edrive.h5"
    truth = fx.make_edrive(src, ns=6, nl=9)
    out = tmp_path / "m.ptm.yaml"
    convert_edrive(src, out, vdc=400.0, torque_points=41)
    doc = yaml.safe_load(out.read_text())
    ns, nt = len(doc["axes"]["speed_rpm"]), len(doc["axes"]["torque_nm"])
    speeds, torques, eff = _read_map(out.with_suffix(".maps.parquet"), ns, nt)

    iv = list(truth.vdc).index(400.0)
    checked = 0
    for s in range(1, truth.speed_rpm.size - 1):
        for lo in (6, 7):  # upper drive quadrant, away from 0 and the envelope edge
            tau = truth.airgap_torque[iv, s, lo]
            src_eff = truth.system_eff[iv, s, lo]
            row = eff[s]
            good = ~np.isnan(row)
            emit = np.interp(tau, torques[good], row[good])
            assert abs(emit - src_eff) < 1e-6, (
                f"speed {s} tau {tau}: {emit} vs {src_eff}"
            )
            checked += 1
    assert checked >= 3


def test_edrive_masks_beyond_envelope(tmp_path: Path):
    src = tmp_path / "edrive.h5"
    fx.make_edrive(src)
    out = tmp_path / "m.ptm.yaml"
    convert_edrive(src, out, vdc=400.0, torque_points=41)
    doc = yaml.safe_load(out.read_text())
    ns, nt = len(doc["axes"]["speed_rpm"]), len(doc["axes"]["torque_nm"])
    _, torques, eff = _read_map(out.with_suffix(".maps.parquet"), ns, nt)
    # Cells beyond the ±128 / −115 envelope are NaN; the τ=0 column is η=0 (spin), not NaN.
    zc = int(np.argmin(np.abs(torques)))
    assert np.allclose(eff[:, zc], 0.0)
    beyond = torques > 128.0 + 1e-6
    assert np.isnan(eff[:, beyond]).all()


def test_edrive_vdc_selection_warns_when_far(tmp_path: Path):
    src = tmp_path / "edrive.h5"
    fx.make_edrive(src)
    out = tmp_path / "m.ptm.yaml"
    summary = convert_edrive(src, out, vdc=420.0)  # snaps to 400 (> 2% gap)
    assert summary["vdc"] == 400.0
    assert any("snapped" in w for w in summary["warnings"])


def test_edrive_mdt_attrs_not_required(tmp_path: Path):
    src = tmp_path / "edrive.h5"
    fx.make_edrive(src, mdt_attrs=False)
    out = tmp_path / "m.ptm.yaml"
    convert_edrive(src, out, vdc=400.0)  # presence-keyed, not attr-keyed
    assert out.exists()


# --- DriveUnit ---------------------------------------------------------------------------------


@pytest.mark.parametrize("thermal_name", ["Thermal", "thermal"])
def test_driveunit_handles_capital_t_thermal(tmp_path: Path, thermal_name: str):
    src = tmp_path / "du.h5"
    fx.make_driveunit(src, thermal_name=thermal_name)
    out = tmp_path / "du.ptm.yaml"
    convert_driveunit(src, out, vdc=48.0)
    doc = yaml.safe_load(out.read_text())
    assert doc["kind"] == "drive_unit"
    assert doc["meta"]["upstream_ratio_applied"] is True
    assert "16.24" in doc["meta"]["source"]  # gear ratio recorded in the source string
    assert doc["inertia_kgm2"] == 0.021  # at_output, not at_input
    assert "cont_torque_nm_vs_speed" in doc["limits"]  # thermal group was found


def test_driveunit_drag_resampled_from_no_load(tmp_path: Path):
    src = tmp_path / "du.h5"
    fx.make_driveunit(src)
    out = tmp_path / "du.ptm.yaml"
    convert_driveunit(src, out, vdc=48.0)
    doc = yaml.safe_load(out.read_text())
    drag = doc["limits"]["drag_torque_nm_vs_speed"]
    assert len(drag["speed_rpm"]) == len(doc["axes"]["speed_rpm"])
    assert all(t <= 0 for t in drag["torque_nm"])  # drag is negative


# --- BatteryPack -------------------------------------------------------------------------------


def test_batterypack_converts_and_validates(tmp_path: Path):
    src = tmp_path / "bp.h5"
    fx.make_batterypack(src)
    out = tmp_path / "battery.yaml"
    summary = convert_batterypack(src, out)
    assert summary["ns"] == 13 and summary["np"] == 3
    doc = yaml.safe_load(out.read_text())
    assert doc["schema"] == "battery/1.0"
    assert doc["soc_window"] == [0.05, 1.0]
    assert Path(summary["tables"]).exists()
    cols = pq.read_table(summary["tables"]).column_names
    assert {
        "soc",
        "temp_c",
        "ocv_v",
        "r0_ohm",
        "r1_ohm",
        "tau1_s",
        "dudt_v_per_k",
    } <= set(cols)


def test_battery_validator_rejects_bad_docs():
    good = {
        "schema": "battery/1.0",
        "topology": {"ns": 13, "np": 3},
        "soc_window": [0.05, 1.0],
        "ecm": {"axes": {"soc": [0.1, 0.5, 0.9], "temp_c": [0.0, 25.0]}},
    }
    validate_battery_doc(good)  # no raise
    bad = dict(good, soc_window=[0.9, 0.1])  # descending
    with pytest.raises(PdtImportError):
        validate_battery_doc(bad)


# --- CLI ---------------------------------------------------------------------------------------


def test_cli_all_subcommands(tmp_path: Path):
    e, d, b = tmp_path / "e.h5", tmp_path / "d.h5", tmp_path / "b.h5"
    fx.make_edrive(e)
    fx.make_driveunit(d)
    fx.make_batterypack(b)
    assert (
        main(["edrive", str(e), "-o", str(tmp_path / "e.ptm.yaml"), "--vdc", "400"])
        == 0
    )
    assert (
        main(["driveunit", str(d), "-o", str(tmp_path / "d.ptm.yaml"), "--vdc", "48"])
        == 0
    )
    assert main(["batterypack", str(b), "-o", str(tmp_path / "b.yaml")]) == 0
    assert (tmp_path / "e.ptm.yaml").exists()


def test_cli_missing_dataset_errors_cleanly(tmp_path: Path, capsys):
    import h5py

    bad = tmp_path / "bad.h5"
    with h5py.File(bad, "w") as f:
        f["sweep/vdc"] = np.array([400.0], np.float32)  # missing everything else
    assert main(["edrive", str(bad), "-o", str(tmp_path / "x.yaml")]) == 1
    assert "error" in capsys.readouterr().err.lower()
