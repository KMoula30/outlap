# SPDX-License-Identifier: AGPL-3.0-only
"""Reference-metrics CSV IO and the opt-in FastF1 position loader.

Two halves, split exactly like :mod:`outlap.wearcal.data`:

* :func:`load_metrics` / :func:`write_metrics` read and write the small committed per-track
  reference-metrics CSVs the CI track-quality gate runs from (network-free). Per HANDOFF §15
  these are *derived* artefacts — per-corner circle-fit radii and apex speeds, never raw
  telemetry.
* :func:`load_fastf1_positions` fetches session position telemetry through FastF1. It is
  **opt-in** (lazy import, never in CI), takes a ``cache_dir``, and returns only derived data:
  per-lap position traces in metres (FastF1 positions arrive in a local, unscaled frame at
  1/10 m units), with ``Source == 'interpolated'`` rows dropped so reconstructed samples never
  masquerade as measurements.

**Metrics CSV format** (``outlap-track-metrics/1``). ``#``-prefixed header lines carry the
provenance the gate needs to be reproducible (KTD7): the source session key and the fitted
georeference similarity transform (FastF1's local frame → real-world coordinates), then one row
per corner::

    # outlap-track-metrics/1
    # label: catalunya_osm
    # source_session: 2026 Spanish Grand Prix R
    # georef: scale=... rotation_rad=... tx_m=... ty_m=... residual_rms_m=...
    corner,s_m,radius_m,apex_speed_mps
    1,412.3,34.06,24.83

Floats are serialized with ``repr`` (shortest round-trip form), so write-then-read preserves
every value bit-exactly; a missing apex speed is an empty field (NaN in memory).
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from numpy.typing import NDArray

from .corners import Corner
from .geometry import TrackCalError

if TYPE_CHECKING:
    from collections.abc import Sequence

F = NDArray[np.float64]

#: Format tag written as the first header line of every metrics CSV.
METRICS_FORMAT = "outlap-track-metrics/1"

_COLUMNS = ("corner", "s_m", "radius_m", "apex_speed_mps")
_GEOREF_KEYS = ("scale", "rotation_rad", "tx_m", "ty_m", "residual_rms_m")


class MetricsFormatError(TrackCalError):
    """A metrics CSV is malformed (missing format tag, header fields, or columns)."""


@dataclass(frozen=True)
class GeorefTransform:
    """A fitted similarity transform (local telemetry frame → real-world metres).

    ``x' = scale·(x cosθ − y sinθ) + tx``, ``y' = scale·(x sinθ + y cosθ) + ty`` with
    ``θ = rotation_rad``. ``residual_rms_m`` is the anchor-point residual of the fit (the
    importer gates on it before trusting the transform — KTD7 requires it recorded here).
    """

    scale: float
    rotation_rad: float
    tx_m: float
    ty_m: float
    residual_rms_m: float

    def apply(self, x_m: F, y_m: F) -> tuple[F, F]:
        """Apply the transform to local-frame coordinates (metres in, metres out)."""
        c, s = math.cos(self.rotation_rad), math.sin(self.rotation_rad)
        x = np.asarray(x_m, dtype=np.float64)
        y = np.asarray(y_m, dtype=np.float64)
        return (
            self.scale * (x * c - y * s) + self.tx_m,
            self.scale * (x * s + y * c) + self.ty_m,
        )


@dataclass(frozen=True)
class TrackMetrics:
    """Per-corner reference metrics for one track (the committed gate fixture).

    ``apex_speed_mps`` is NaN where no speed source resolved (radius stays asserted; speed is
    recorded-only per KTD4).
    """

    label: str
    source_session: str
    transform: GeorefTransform
    corner: NDArray[np.int64]
    s_m: F
    radius_m: F
    apex_speed_mps: F

    def __post_init__(self) -> None:
        n = self.corner.shape
        if not (self.s_m.shape == n and self.radius_m.shape == n):
            raise TrackCalError("metrics columns must have equal length")
        if self.apex_speed_mps.shape != n:
            raise TrackCalError("apex_speed_mps must match the corner count")

    @property
    def n_corners(self) -> int:
        """Number of corner rows."""
        return int(self.corner.size)


def metrics_from_corners(
    corners: Sequence[Corner],
    *,
    label: str,
    source_session: str,
    transform: GeorefTransform,
) -> TrackMetrics:
    """Assemble a :class:`TrackMetrics` record from detected corners (None speed → NaN)."""
    return TrackMetrics(
        label=label,
        source_session=source_session,
        transform=transform,
        corner=np.array([c.index for c in corners], dtype=np.int64),
        s_m=np.array([c.s_apex_m for c in corners], dtype=np.float64),
        radius_m=np.array([c.apex_radius_m for c in corners], dtype=np.float64),
        apex_speed_mps=np.array(
            [
                math.nan if c.apex_speed_mps is None else c.apex_speed_mps
                for c in corners
            ],
            dtype=np.float64,
        ),
    )


def write_metrics(path: Path, metrics: TrackMetrics) -> None:
    """Write a metrics CSV (``repr`` floats: the round-trip is bit-exact)."""
    georef = " ".join(
        f"{key}={getattr(metrics.transform, key)!r}" for key in _GEOREF_KEYS
    )
    lines = [
        f"# {METRICS_FORMAT}",
        f"# label: {metrics.label}",
        f"# source_session: {metrics.source_session}",
        f"# georef: {georef}",
        ",".join(_COLUMNS),
    ]
    for i in range(metrics.n_corners):
        speed = float(metrics.apex_speed_mps[i])
        lines.append(
            ",".join(
                [
                    str(int(metrics.corner[i])),
                    repr(float(metrics.s_m[i])),
                    repr(float(metrics.radius_m[i])),
                    "" if math.isnan(speed) else repr(speed),
                ]
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_metrics(path: Path) -> TrackMetrics:
    """Read a committed metrics CSV; raises :class:`MetricsFormatError` on malformed input."""
    text = path.read_text(encoding="utf-8").splitlines()
    header: dict[str, str] = {}
    rows: list[str] = []
    for line in text:
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            body = stripped.lstrip("#").strip()
            if ":" in body:
                key, _, value = body.partition(":")
                header[key.strip()] = value.strip()
            else:
                header.setdefault("format", body)
            continue
        rows.append(stripped)
    if header.get("format") != METRICS_FORMAT:
        raise MetricsFormatError(
            f"{path}: missing `# {METRICS_FORMAT}` format header (got {header.get('format')!r})"
        )
    for key in ("label", "source_session", "georef"):
        if key not in header:
            raise MetricsFormatError(f"{path}: missing `# {key}:` header line")
    transform = _parse_georef(header["georef"], path)

    reader = csv.reader(rows)
    columns = next(reader, None)
    if columns is None or tuple(c.strip() for c in columns) != _COLUMNS:
        raise MetricsFormatError(
            f"{path}: expected columns {','.join(_COLUMNS)}, got {columns!r}"
        )
    corner: list[int] = []
    s_m: list[float] = []
    radius: list[float] = []
    speed: list[float] = []
    for row in reader:
        if len(row) != len(_COLUMNS):
            raise MetricsFormatError(f"{path}: malformed row {row!r}")
        corner.append(int(row[0]))
        s_m.append(float(row[1]))
        radius.append(float(row[2]))
        speed.append(math.nan if row[3] == "" else float(row[3]))
    return TrackMetrics(
        label=header["label"],
        source_session=header["source_session"],
        transform=transform,
        corner=np.array(corner, dtype=np.int64),
        s_m=np.array(s_m, dtype=np.float64),
        radius_m=np.array(radius, dtype=np.float64),
        apex_speed_mps=np.array(speed, dtype=np.float64),
    )


# --- the opt-in FastF1 position loader ------------------------------------------------------


@dataclass(frozen=True)
class PositionTrace:
    """One lap's genuine position samples (metres, local FastF1 frame), derived data only."""

    x_m: F
    y_m: F
    t_s: F
    label: str


def load_fastf1_positions(
    year: int,
    event: str | int,
    session: str,
    *,
    drivers: Sequence[str] | None = None,
    max_laps_per_driver: int | None = None,
    cache_dir: Path | None = None,
) -> list[PositionTrace]:
    """Fetch per-lap position traces from FastF1 (**opt-in**, lazy import, never in CI).

    Pools the quicklaps of ``drivers`` (default: all drivers in the session), one trace per
    lap. Rows with ``Source == 'interpolated'`` are dropped — only measured position samples
    survive (the U2 importer's pooling/georeferencing builds on these). FastF1 positions are
    1/10 m in a local unscaled frame; the returned traces are converted to metres but stay in
    that local frame. Requires network access on first fetch; FastF1 caches under
    ``cache_dir``.
    """
    ff1 = _import_fastf1()
    if cache_dir is not None:
        cache_dir.mkdir(parents=True, exist_ok=True)
        ff1.Cache.enable_cache(str(cache_dir))
    sess: Any = ff1.get_session(year, event, session)
    sess.load(laps=True, telemetry=True, weather=False, messages=False)
    laps: Any = sess.laps
    if drivers is not None:
        laps = laps.pick_drivers(list(drivers))
    laps = laps.pick_quicklaps()
    traces: list[PositionTrace] = []
    seen: dict[str, int] = {}
    for _, lap in laps.iterlaps():
        driver = str(lap["Driver"])
        if (
            max_laps_per_driver is not None
            and seen.get(driver, 0) >= max_laps_per_driver
        ):
            continue
        pos: Any = lap.get_pos_data()
        pos = pos[pos["Source"] != "interpolated"]
        if len(pos) == 0:
            continue
        seen[driver] = seen.get(driver, 0) + 1
        traces.append(
            PositionTrace(
                x_m=np.asarray(pos["X"], dtype=np.float64) / 10.0,
                y_m=np.asarray(pos["Y"], dtype=np.float64) / 10.0,
                t_s=np.asarray(
                    [t.total_seconds() for t in pos["Time"]], dtype=np.float64
                ),
                label=f"{driver} {year} {event} L{int(lap['LapNumber'])}",
            )
        )
    if not traces:
        raise TrackCalError(
            f"no measured position samples for {year} {event} {session}"
        )
    return traces


def _import_fastf1() -> Any:
    """Lazy FastF1 import with an actionable error naming the extra (wearcal precedent)."""
    try:
        import fastf1  # pyright: ignore[reportMissingImports]
    except ImportError as err:  # pragma: no cover - live path, never exercised in CI
        raise ImportError(
            "the live FastF1 position loader needs fastf1 — install the extra: "
            "`uv sync --extra wear-cal` (or `pip install fastf1`)"
        ) from err
    return cast("Any", fastf1)


# --- header parsing internals ---------------------------------------------------------------


def _parse_georef(body: str, path: Path) -> GeorefTransform:
    values: dict[str, float] = {}
    for token in body.split():
        key, sep, value = token.partition("=")
        if sep != "=" or key not in _GEOREF_KEYS:
            raise MetricsFormatError(f"{path}: malformed georef token {token!r}")
        try:
            values[key] = float(value)
        except ValueError as err:
            raise MetricsFormatError(
                f"{path}: non-numeric georef value {token!r}"
            ) from err
    missing = [k for k in _GEOREF_KEYS if k not in values]
    if missing:
        raise MetricsFormatError(f"{path}: georef header missing {missing}")
    return GeorefTransform(
        scale=values["scale"],
        rotation_rad=values["rotation_rad"],
        tx_m=values["tx_m"],
        ty_m=values["ty_m"],
        residual_rms_m=values["residual_rms_m"],
    )
