# SPDX-License-Identifier: AGPL-3.0-only
"""CLI for the MF6.1 fitting pipeline.

python -m outlap.tirefit fit   <data...> --unloaded-radius R0 -o out.tyr.yaml [--report-dir DIR]
                               [--fnomin N] [--nompres PA] [--longvl MPS]
python -m outlap.tirefit synth <in.tyr.yaml> -o out.csv [--seed 0] [--noise 0.01]

`fit` reads one or more test files (.mat TTC v7/v7.3, .dat, .csv), runs the staged fit, and
writes a `.tyr.yaml` (thermal/wear as labelled synthetic placeholders) plus a JSON+MD report.
`synth` generates a deterministic synthetic dataset from an existing `.tyr` — the recovery-test
harness, and a way to exercise the pipeline without membership-locked data.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, cast

import numpy as np
import yaml

from ..tir.convert import SYNTHETIC_THERMAL, SYNTHETIC_WEAR
from .data import TireTestData, load_csv, load_dat, load_ttc_mat
from .report import write_report
from .stages import FitConfig, staged_fit, synthesize


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(prog="outlap.tirefit", description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    pf = sub.add_parser(
        "fit", help="staged MF6.1 fit from test data → .tyr.yaml + report"
    )
    pf.add_argument("data", nargs="+", type=Path)
    pf.add_argument("-o", "--output", type=Path, required=True)
    pf.add_argument("--unloaded-radius", type=float, required=True, metavar="M")
    pf.add_argument("--fnomin", type=float, metavar="N")
    pf.add_argument("--nompres", type=float, metavar="PA")
    pf.add_argument("--longvl", type=float, default=16.7, metavar="MPS")
    pf.add_argument("--report-dir", type=Path)

    ps = sub.add_parser(
        "synth", help="synthetic dataset from a .tyr (seeded, deterministic)"
    )
    ps.add_argument("tyr", type=Path)
    ps.add_argument("-o", "--output", type=Path, required=True)
    ps.add_argument("--seed", type=int, default=0)
    ps.add_argument("--noise", type=float, default=0.01)

    args = parser.parse_args(argv)
    try:
        if args.cmd == "fit":
            return _fit(args)
        return _synth(args)
    except (ValueError, OSError, ImportError, yaml.YAMLError) as err:
        print(f"error: {err}", file=sys.stderr)
        return 1


def _load_any(path: Path) -> TireTestData:
    suffix = path.suffix.lower()
    if suffix == ".mat":
        return load_ttc_mat(path)
    if suffix == ".dat":
        return load_dat(path)
    if suffix == ".csv":
        # The synth CLI writes ISO signs; measured TTC CSV exports are SAE — the fit CLI
        # standardises on our own synth format here, so ISO. Use the library API directly
        # for SAE-signed CSVs.
        return load_csv(path, sae_signs=False)
    raise ValueError(f"unsupported test-data format: {path}")


def _concat(parts: list[TireTestData]) -> TireTestData:
    def cat(name: str) -> Any:
        return np.concatenate([getattr(p, name) for p in parts])

    return TireTestData(
        kappa=cat("kappa"),
        alpha_rad=cat("alpha_rad"),
        gamma_rad=cat("gamma_rad"),
        fz_n=cat("fz_n"),
        p_pa=cat("p_pa"),
        vx_mps=cat("vx_mps"),
        fx_n=cat("fx_n"),
        fy_n=cat("fy_n"),
        mz_nm=cat("mz_nm"),
        mx_nm=cat("mx_nm"),
    )


def _fit(args: argparse.Namespace) -> int:
    data = _concat([_load_any(p) for p in args.data])
    config = FitConfig(
        unloaded_radius_m=args.unloaded_radius,
        fnomin_n=args.fnomin,
        nompres_pa=args.nompres,
        longvl_mps=args.longvl,
    )
    result = staged_fit(data, config)

    sources = ", ".join(str(p) for p in args.data)
    tyr: dict[str, Any] = {
        "schema": "tyr/1.0",
        "mf61": result.coeffs,
        "thermal": dict(SYNTHETIC_THERMAL),
        "wear": dict(SYNTHETIC_WEAR),
        "provenance": {
            "citation": "MF6.1 coefficients fitted by outlap.tirefit (staged least-squares)",
            "source": f"fitted from {sources}; thermal/wear are synthetic placeholders",
            "synthetic": True,
        },
    }
    args.output.write_text(
        yaml.safe_dump(tyr, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    print(f"wrote {args.output}")

    for stage in result.stages:
        if stage.skipped is not None:
            print(f"stage {stage.name}: skipped ({stage.skipped})")
        else:
            print(
                f"stage {stage.name}: n={stage.n_samples} "
                f"rms={stage.rms_n:.4g} max={stage.max_abs_n:.4g}"
            )
    if args.report_dir is not None:
        json_path, md_path = write_report(result, args.report_dir, source=sources)
        print(f"wrote {json_path} and {md_path}")
    return 0


def _synth(args: argparse.Namespace) -> int:
    tyr = cast("dict[str, Any]", yaml.safe_load(args.tyr.read_text(encoding="utf-8")))
    mf61_map: dict[str, Any] = tyr["mf61"]
    coeffs = {str(k): float(v) for k, v in mf61_map.items()}
    data = synthesize(coeffs, seed=args.seed, noise=args.noise)

    header = "SR,SA,IA,FZ,P,V,FX,FY,MZ,MX"
    columns = np.column_stack(
        [
            data.kappa,
            data.alpha_rad,
            data.gamma_rad,
            data.fz_n,
            data.p_pa,
            data.vx_mps,
            data.fx_n,
            data.fy_n,
            data.mz_nm,
            data.mx_nm,
        ]
    )
    # ISO signs and SI units throughout (this is our own format, not a TTC export); loaded
    # back with `load_csv(..., sae_signs=False)` after undoing the unit factors — so write
    # the channels in the reader's expected units instead: deg, kPa, kph.
    columns[:, 1] = np.degrees(columns[:, 1])
    columns[:, 2] = np.degrees(columns[:, 2])
    columns[:, 4] = columns[:, 4] / 1000.0
    columns[:, 5] = columns[:, 5] * 3.6
    body = "\n".join(",".join(repr(float(v)) for v in row) for row in columns)
    args.output.write_text(header + "\n" + body + "\n", encoding="utf-8")
    print(f"wrote {args.output} ({columns.shape[0]} samples)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
