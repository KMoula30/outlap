# SPDX-License-Identifier: AGPL-3.0-only
"""MF6.1 tyre fitting: test-data ingestion, a numpy forward model, and a staged fit.

**Redistribution policy: parsers yes — REDISTRIBUTION OF TTC DATA OR TTC-DERIVED PARAMETER SETS,
NO.** FSAE TTC data is membership-locked; this package lets members fit locally (keep raw files
in the gitignored ``ttc-data/``). Never commit TTC data or parameter sets fitted from it.

The forward model (:mod:`outlap.tirefit.mf61`) is a clean-room numpy mirror of outlap's Rust
kernels, validated against the same committed golden CSVs and tolerance rule. The staged fit
(:mod:`outlap.tirefit.stages`) needs scipy — install the ``tire-fit`` extra.

CLI: ``python -m outlap.tirefit {fit, synth}``.
"""

from __future__ import annotations

from .data import SweepBin, TireTestData, bin_sweeps, load_csv, load_dat, load_ttc_mat
from .mf61 import DEFAULTS, Forces, forces, params_from_coeffs, params_from_tyr
from .report import render_markdown, report_dict, write_report
from .stages import FitConfig, FitResult, StageReport, staged_fit, synthesize

__all__ = [
    "DEFAULTS",
    "FitConfig",
    "FitResult",
    "Forces",
    "StageReport",
    "SweepBin",
    "TireTestData",
    "bin_sweeps",
    "forces",
    "load_csv",
    "load_dat",
    "load_ttc_mat",
    "params_from_coeffs",
    "params_from_tyr",
    "render_markdown",
    "report_dict",
    "staged_fit",
    "synthesize",
    "write_report",
]
