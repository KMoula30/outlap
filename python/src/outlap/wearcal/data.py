# SPDX-License-Identifier: AGPL-3.0-only
"""Stint-pace observations: the offline fixture reader and the opt-in FastF1 loader.

An observation is just a per-lap pace curve — the target the calibrator matches. Two sources:

* :func:`load_fixture` reads a small committed CSV (``lap,lap_time_s``) — a *derived, anonymised*
  stint used as the offline CI gate. Per HANDOFF §15 we never commit raw telemetry or fitted TTC
  parameter sets; a per-lap pace delta is a derived artefact.
* :func:`load_fastf1` fetches a real race stint through the FastF1 API. It is **opt-in** (the
  ``wear-cal`` extra), lazily imported, and lives outside CI — it must never redistribute the raw
  telemetry it downloads. Use it to produce your own private fixtures.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    from collections.abc import Sequence

F = NDArray[np.float64]


@dataclass(frozen=True)
class StintObservation:
    """A per-lap pace curve to calibrate against.

    ``lap`` is 1-based lap number within the stint; ``lap_time_s`` the corresponding lap time (s).
    ``label`` names the provenance (e.g. a compound + event) for reports — never raw telemetry.
    """

    lap: F
    lap_time_s: F
    label: str = "stint"

    def __post_init__(self) -> None:
        if self.lap.shape != self.lap_time_s.shape:
            raise ValueError("lap and lap_time_s must have the same length")
        if self.lap.size < 2:
            raise ValueError("a stint observation needs at least two laps")

    @property
    def n_laps(self) -> int:
        """Number of laps in the observation."""
        return int(self.lap.size)

    @property
    def pace_delta_s(self) -> F:
        """Lap times relative to the fastest lap in the stint (the degradation signal)."""
        return self.lap_time_s - float(np.min(self.lap_time_s))

    @classmethod
    def from_sequences(
        cls, lap: Sequence[float], lap_time_s: Sequence[float], *, label: str = "stint"
    ) -> StintObservation:
        """Build from plain sequences (list/tuple)."""
        return cls(
            lap=np.asarray(lap, dtype=np.float64),
            lap_time_s=np.asarray(lap_time_s, dtype=np.float64),
            label=label,
        )


def load_fixture(path: Path, *, label: str | None = None) -> StintObservation:
    """Read a committed ``lap,lap_time_s`` CSV stint-delta fixture.

    A ``#``-prefixed comment header is allowed (and used to document provenance). The lap column may
    be absent, in which case laps are numbered ``1..N`` in file order.
    """
    laps: list[float] = []
    times: list[float] = []
    rows = [
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    reader = csv.reader(rows)
    header = next(reader, None)
    if header is None:
        raise ValueError(f"empty fixture: {path}")
    cols = [c.strip().lower() for c in header]
    if "lap_time_s" not in cols:
        raise ValueError(f"fixture {path} has no `lap_time_s` column (got {cols})")
    ti = cols.index("lap_time_s")
    li = cols.index("lap") if "lap" in cols else None
    for i, row in enumerate(reader, start=1):
        times.append(float(row[ti]))
        laps.append(float(row[li]) if li is not None else float(i))
    return StintObservation.from_sequences(laps, times, label=label or path.stem)


def load_fastf1(
    year: int,
    event: str | int,
    session: str,
    driver: str,
    *,
    stint: int | None = None,
    cache_dir: Path | None = None,
) -> StintObservation:
    """Fetch a driver's stint lap times from FastF1 (**opt-in**, needs the ``wear-cal`` extra).

    Returns only anonymised per-lap times (no raw telemetry is retained). ``stint`` selects one of
    the driver's stints (default: the longest). Requires network access on first fetch; FastF1
    caches downloads under ``cache_dir``.
    """
    ff1 = _import_fastf1()
    if cache_dir is not None:
        cache_dir.mkdir(parents=True, exist_ok=True)
        ff1.Cache.enable_cache(str(cache_dir))
    sess: Any = ff1.get_session(year, event, session)
    sess.load(laps=True, telemetry=False, weather=False, messages=False)
    laps_df: Any = sess.laps.pick_drivers(driver)
    if laps_df is None or len(laps_df) == 0:
        raise ValueError(f"no laps for driver {driver!r} in {year} {event} {session}")
    stints: Any = laps_df.groupby("Stint")
    chosen = stint if stint is not None else _longest_stint(stints)
    stint_laps: Any = stints.get_group(chosen).sort_values("LapNumber")
    seconds = [t.total_seconds() for t in stint_laps["LapTime"]]
    valid = [(i + 1, s) for i, s in enumerate(seconds) if s == s and s > 0.0]
    if len(valid) < 2:
        raise ValueError(f"stint {chosen} for {driver!r} has < 2 timed laps")
    lap = [float(i) for i, _ in valid]
    times = [float(s) for _, s in valid]
    return StintObservation.from_sequences(
        lap, times, label=f"{driver} {year} {event} S{chosen}"
    )


def _longest_stint(stints: Any) -> Any:
    """The stint id with the most laps (the default when the caller does not pick one)."""
    best_id: Any = None
    best_len = -1
    for stint_id, group in stints:
        if len(group) > best_len:
            best_len, best_id = len(group), stint_id
    return best_id


def _import_fastf1() -> Any:
    """Lazy FastF1 import with an actionable error naming the extra."""
    try:
        import fastf1  # pyright: ignore[reportMissingImports]
    except ImportError as err:  # pragma: no cover - live path, never exercised in CI
        raise ImportError(
            "the live FastF1 stint loader needs fastf1 — install the extra: "
            "`uv sync --extra wear-cal` (or `pip install 'outlap[wear-cal]'`)"
        ) from err
    return cast("Any", fastf1)
