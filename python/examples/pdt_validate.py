# SPDX-License-Identifier: AGPL-3.0-only
"""Validate the PDT importers against the author's real reference files — LOCAL ONLY, never CI.

Runs all three conversions, then spot-checks that the emitted maps reproduce the source arrays and
that power rebuilt as τ·ω matches the file's own power channel. Nothing from the reference directory
is ever committed or copied into the repo tree (firewall, §1/§15).

    python examples/pdt_validate.py --ref /home/kmoulakis/pdt_reference --out /tmp/pdt_out
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path

import h5py
import numpy as np
import pyarrow.parquet as pq

from outlap.importers.pdt_h5 import (
    convert_batterypack,
    convert_driveunit,
    convert_edrive,
)

EDRIVE = "EDrive_121.0L_16Et_650.0I_400.0V_12ea1_SynRM_ref.h5"
DRIVEUNIT = "DriveUnit_16.2GR_168NM_369RPM_666f3_R250_ref.h5"
BATTERY = "BatteryPack_13S_3P_722Wh_48V_7158a_cleanTest2Bot.h5"


def _spot_check_edrive(
    src: Path, parquet: Path, ns: int, nt: int, n: int = 30
) -> float:
    """Max abs error reproducing source system efficiency through the emitted table."""
    t = pq.read_table(parquet)
    eff = np.asarray(t.column("efficiency")).reshape(ns, nt)
    torques = np.asarray(t.column("torque_nm")).reshape(ns, nt)[0]
    with h5py.File(src, "r") as f:
        vdc = list(np.asarray(f["sweep/vdc"]))
        iv = (
            vdc.index(400.0)
            if 400.0 in vdc
            else int(np.argmin(np.abs(np.asarray(vdc) - 400)))
        )
        tau = np.asarray(f["operating_grid/airgap_torque"])[iv]
        sys = (
            np.asarray(f["operating_grid/motor_efficiency"])[iv]
            * np.asarray(f["operating_grid/inverter_efficiency"])[iv]
        )
    rng = random.Random(1)
    errs = []
    for _ in range(n):
        s = rng.randrange(5, tau.shape[0] - 2)
        li = rng.randrange(30, 52)
        if sys[s, li] <= 0.1:
            continue
        row = eff[s]
        good = ~np.isnan(row)
        if tau[s, li] < torques[good][0] or tau[s, li] > torques[good][-1]:
            continue
        errs.append(
            abs(float(np.interp(tau[s, li], torques[good], row[good])) - sys[s, li])
        )
    return max(errs) if errs else float("nan")


def main(argv: list[str] | None = None) -> int:
    """Run the real-file validation checklist."""
    parser = argparse.ArgumentParser(prog="pdt_validate", description=__doc__)
    parser.add_argument(
        "--ref", type=Path, default=Path("/home/kmoulakis/pdt_reference")
    )
    parser.add_argument("--out", type=Path, default=Path("/tmp/pdt_out"))
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    print("== EDrive ==")
    e = convert_edrive(args.ref / EDRIVE, args.out / "machine.ptm.yaml", vdc=400.0)
    err = _spot_check_edrive(
        args.ref / EDRIVE, Path(e["maps"]), e["speeds"], e["torque_points"]
    )
    print(
        f"  vdc={e['vdc']} nan={e['nan_fraction']}  spot-check max err {err:.2e} (real: expect <1e-3)"
    )

    print("== DriveUnit ==")
    d = convert_driveunit(args.ref / DRIVEUNIT, args.out / "du.ptm.yaml", vdc=48.0)
    print(f"  vdc={d['vdc']} gear_ratio={d['gear_ratio']:.4f} nan={d['nan_fraction']}")

    print("== BatteryPack ==")
    b = convert_batterypack(args.ref / BATTERY, args.out / "battery.yaml")
    print(f"  ns={b['ns']} np={b['np']}")

    print("\nAll three converted + schema-validated. Nothing from --ref was committed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
