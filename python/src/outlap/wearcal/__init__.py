# SPDX-License-Identifier: AGPL-3.0-only
"""Tyre wear/grip inverse calibration from stint pace curves.

The flagship thermal-ring wear/degradation model (HANDOFF §7.2/§7.3) has physically-meaningful but
initially-uncalibrated parameters. This package fits them **inversely from observed per-lap pace**:
given a stint's lap-time curve, recover the wear coefficient ``k_w``, cliff onset ``w_c``, cliff
shape ``s_w``/``Δ_c`` (and optionally the grip-window/thermal-damage terms) so a simulated stint
reproduces the observed ~0.05–0.10 s/lap decay and the cliff lap.

**Redistribution policy (HANDOFF §15).** FastF1 telemetry and any parameters fitted from it are
calibration/validation artefacts only. This package never commits raw telemetry or fitted TTC
parameter sets. The live FastF1 loader (:func:`load_fastf1`) is opt-in (the ``wear-cal`` extra) and
retains only anonymised per-lap times; the committed offline fixture is a small *derived* pace
curve for the CI gate.

The forward model the optimiser inverts is a fast reduced-order surrogate
(:mod:`outlap.wearcal.model`) — a clean-room numpy mirror of the Rust ring's Archard/Grosch/cliff
laws, the same relationship ``tirefit`` bears to the Rust force kernels. A faithful (slow, opt-in)
variant wrapping the real stint driver lives in :mod:`outlap.wearcal.sim`.

CLI: ``python -m outlap.wearcal {calibrate, synth, sim-check}``.
"""

from __future__ import annotations

from .data import StintObservation, load_fastf1, load_fixture
from .model import (
    StintAnchor,
    StintTrace,
    SurrogateParams,
    cliff,
    grip_window,
    inv_hardness,
    stint_lap_times,
    stint_trace,
)
from .optimize import (
    CALIB_TABLE,
    DEFAULT_FREE,
    CalibConfig,
    CalibParam,
    CalibResult,
    calibrate,
    synth_observation,
)
from .report import render_markdown, report_dict, write_report

__all__ = [
    "CALIB_TABLE",
    "DEFAULT_FREE",
    "CalibConfig",
    "CalibParam",
    "CalibResult",
    "StintAnchor",
    "StintObservation",
    "StintTrace",
    "SurrogateParams",
    "calibrate",
    "cliff",
    "grip_window",
    "inv_hardness",
    "load_fastf1",
    "load_fixture",
    "render_markdown",
    "report_dict",
    "stint_lap_times",
    "stint_trace",
    "synth_observation",
    "write_report",
]
