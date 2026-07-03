# SPDX-License-Identifier: AGPL-3.0-only
"""CLI for the PDT importers (mirrors the future Rust ``outlap import pdt-*`` 1:1).

python -m outlap.importers.pdt_h5 edrive      <file.h5> -o machine.ptm.yaml [--vdc 400]
python -m outlap.importers.pdt_h5 driveunit   <file.h5> -o du.ptm.yaml      [--vdc 48] [--mass-kg X]
python -m outlap.importers.pdt_h5 batterypack <file.h5> -o battery.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .battery import convert_batterypack
from .common import PdtImportError
from .driveunit import convert_driveunit
from .edrive import convert_edrive


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        prog="outlap.importers.pdt_h5", description=__doc__
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    pe = sub.add_parser("edrive", help="EDrive .h5 → electric_machine .ptm")
    pd = sub.add_parser("driveunit", help="DriveUnit .h5 → drive_unit .ptm")
    pb = sub.add_parser(
        "batterypack", help="BatteryPack .h5 → provisional battery.yaml"
    )
    for p in (pe, pd, pb):
        p.add_argument("src", type=Path, help="source .h5 file")
        p.add_argument("-o", "--out", type=Path, required=True, help="output YAML path")
    for p in (pe, pd):
        p.add_argument(
            "--vdc", type=float, help="DC voltage to select (nearest grid slice)"
        )
        p.add_argument(
            "--torque-points", type=int, default=101, help="regular torque-axis size"
        )
        p.add_argument(
            "--maps",
            type=Path,
            help="parquet sidecar path (default: <out>.maps.parquet)",
        )
    pd.add_argument(
        "--mass-kg", type=float, help="mass override if the file lacks a mass group"
    )
    pb.add_argument(
        "--tables",
        type=Path,
        help="parquet sidecar path (default: <out>.tables.parquet)",
    )

    args = parser.parse_args(argv)
    try:
        if args.cmd == "edrive":
            summary = convert_edrive(
                args.src,
                args.out,
                vdc=args.vdc,
                torque_points=args.torque_points,
                maps_path=args.maps,
            )
        elif args.cmd == "driveunit":
            summary = convert_driveunit(
                args.src,
                args.out,
                vdc=args.vdc,
                torque_points=args.torque_points,
                maps_path=args.maps,
                mass_kg=args.mass_kg,
            )
        else:
            summary = convert_batterypack(args.src, args.out, tables_path=args.tables)
    except PdtImportError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except (OSError, KeyError) as exc:  # noqa: F841 - surfaced below
        print(f"error reading {args.src}: {exc}", file=sys.stderr)
        return 1

    for w in summary.get("warnings", []):
        print(f"  warning: {w}", file=sys.stderr)
    print(f"wrote {summary['out']}")
    for k, v in summary.items():
        if k not in ("out", "warnings"):
            print(f"  {k}: {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
