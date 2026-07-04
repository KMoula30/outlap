# SPDX-License-Identifier: AGPL-3.0-only
"""CLI for the ``.tir`` codec (mirrors the pdt_h5 importer CLI shape).

python -m outlap.tir to-tyr   <in.tir>      -o out.tyr.yaml [--thermal-wear synthetic|from-donor|none] [--donor donor.tyr.yaml]
python -m outlap.tir from-tyr <in.tyr.yaml> -o out.tir
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, cast

import yaml

from .convert import ThermalWearPolicy, tir_to_tyr, tyr_to_tir
from .doc import TirError
from .parse import parse_tir
from .write import write_tir


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(prog="outlap.tir", description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    pt = sub.add_parser("to-tyr", help=".tir → .tyr.yaml (thermal/wear per policy)")
    pt.add_argument("input", type=Path)
    pt.add_argument("-o", "--output", type=Path, required=True)
    pt.add_argument(
        "--thermal-wear",
        choices=["synthetic", "from-donor", "none"],
        default="synthetic",
        help="policy for the thermal/wear blocks a .tir cannot carry",
    )
    pt.add_argument(
        "--donor", type=Path, help="donor .tyr.yaml for --thermal-wear from-donor"
    )

    pf = sub.add_parser("from-tyr", help=".tyr.yaml → canonical .tir")
    pf.add_argument("input", type=Path)
    pf.add_argument("-o", "--output", type=Path, required=True)

    args = parser.parse_args(argv)
    try:
        if args.cmd == "to-tyr":
            return _to_tyr(args)
        return _from_tyr(args)
    except (TirError, OSError, yaml.YAMLError) as err:
        print(f"error: {err}", file=sys.stderr)
        return 1


def _to_tyr(args: argparse.Namespace) -> int:
    doc, warnings = parse_tir(str(args.input), args.input.read_text(encoding="utf-8"))
    donor: dict[str, Any] | None = None
    if args.donor is not None:
        donor = cast(
            "dict[str, Any]", yaml.safe_load(args.donor.read_text(encoding="utf-8"))
        )
    policy = cast("ThermalWearPolicy", args.thermal_wear)
    tyr, w2 = tir_to_tyr(doc, policy, donor)
    warnings.extend(w2)
    for w in warnings:
        print(f"warning: {w}", file=sys.stderr)
    text = yaml.safe_dump(tyr, sort_keys=False, allow_unicode=True)
    args.output.write_text(text, encoding="utf-8")
    print(f"wrote {args.output}")
    return 0


def _from_tyr(args: argparse.Namespace) -> int:
    tyr = cast("dict[str, Any]", yaml.safe_load(args.input.read_text(encoding="utf-8")))
    doc = tyr_to_tir(tyr)
    args.output.write_text(write_tir(doc), encoding="utf-8")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
