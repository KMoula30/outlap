# SPDX-License-Identifier: AGPL-3.0-only
"""Track geometry calibration: curvature-first fitting, corner metrics, reference CSVs.

The shared analysis core of the MT track-fidelity milestone (HANDOFF §12). Three modules:

* :mod:`outlap.trackcal.geometry` — penalised periodic smoothing-spline centerline fits with
  per-corner-adaptive regularization; curvature is evaluated analytically from the fit, never
  finite-differenced from raw points (the naive interpolating-spline second-derivative route
  is a documented biased anti-pattern — see the module docstring for the method + citations).
* :mod:`outlap.trackcal.corners` — apex detection on fitted curvature, windowed short-arc
  extraction, and robust apex radii via the Hyper algebraic circle fit (Al-Sharadqah &
  Chernov 2009) refined geometrically; robust apex-speed extraction from pooled samples.
* :mod:`outlap.trackcal.data` — the committed reference-metrics CSV format the CI
  track-quality gate reads (georeference transform + session key recorded in the header,
  KTD7), and the opt-in, lazily-imported FastF1 position loader (never in CI).

Everything here runs on numpy alone — no scipy, no network — so the core and its tests work
wherever the base package installs. Clean-room note: TUMFTM ``trajectory_planning_helpers``
and ``racetrack-database`` (LGPL-3.0) were consulted for approach only; all code here is
authored independently from the cited literature.
"""

from __future__ import annotations

from .corners import (
    MIN_ARC_POINTS,
    CircleFit,
    Corner,
    detect_corners,
    fit_circle,
)
from .data import (
    METRICS_FORMAT,
    GeorefTransform,
    MetricsFormatError,
    PositionTrace,
    TrackMetrics,
    format_georef,
    load_fastf1_positions,
    load_metrics,
    metrics_from_corners,
    parse_georef,
    write_metrics,
)
from .geometry import (
    MIN_POINTS,
    CenterlineFit,
    CenterlineSamples,
    DegenerateInputError,
    TrackCalError,
    fit_centerline,
)

__all__ = [
    "METRICS_FORMAT",
    "MIN_ARC_POINTS",
    "MIN_POINTS",
    "CenterlineFit",
    "CenterlineSamples",
    "CircleFit",
    "Corner",
    "DegenerateInputError",
    "GeorefTransform",
    "MetricsFormatError",
    "PositionTrace",
    "TrackCalError",
    "TrackMetrics",
    "detect_corners",
    "fit_centerline",
    "fit_circle",
    "format_georef",
    "load_fastf1_positions",
    "load_metrics",
    "metrics_from_corners",
    "parse_georef",
    "write_metrics",
]
