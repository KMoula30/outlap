# SPDX-License-Identifier: AGPL-3.0-only
"""TUMFTM ``racetrack-database`` CSV → outlap ``track.yaml`` + ``centerline.csv`` (§9.3).

The `TUMFTM racetrack-database <https://github.com/TUMFTM/racetrack-database>`_ (LGPL-3.0) ships
25 circuit centre lines with **measured** corridor widths — the standard academic bootstrap set
(HANDOFF §4.4). Each source file is ``[x_m, y_m, w_tr_right_m, w_tr_left_m]``: a smoothed centre
line in a local metric frame plus the track half-widths to the **right** and **left** of it,
already resampled on a uniform ≈5 m arc-length grid with the loop left open (the last point sits
one sample before the start).

This importer converts one such file (or a whole ``tracks/`` directory) into outlap's 8-column
centre line ``s_m, x_m, y_m, z_m, banking_deg, width_left_m, width_right_m, grip_scale`` plus a
``track.yaml`` descriptor. Three points of care, each verified against the upstream format:

1. **Widths are mapped by NAME, never by column position.** The source order is RIGHT then LEFT;
   outlap's order is LEFT then RIGHT. ``w_tr_left_m → width_left_m`` (road ``+y``, ISO 8855) and
   ``w_tr_right_m → width_right_m`` (road ``−y``). A position map would silently swap the corridor
   and flip the min-curvature line.
2. **The data is strictly 2-D.** ``z``, ``banking_deg`` are emitted as ``0`` and ``grip_scale`` as
   ``1`` — this is legitimate (the source has no elevation), not fabricated, and is recorded in the
   track ``meta.notes`` and accuracy class ``C``.
3. **Closure.** ``s`` is cumulative chord length over the source points; the loop is left open so
   the last point sits ~one sample before the first. outlap's loader closes it over the connecting
   chord (its ``NotClosed`` guard trips only when that chord exceeds 3× the median spacing).

Like :mod:`outlap.importers.osm_track` this is a **one-time local vendoring tool**; it reads only
the redistributable LGPL data and is never run in CI. Vendored outputs keep the upstream LGPL
notice (``data/tracks/LICENSE-tumftm-LGPL-3.0.txt``) and the attribution string in each
``track.yaml``.

Example::

    # clone the upstream data at the pinned commit, then convert every track:
    git clone https://github.com/TUMFTM/racetrack-database.git /tmp/tumftm
    git -C /tmp/tumftm checkout e59595d
    python -m outlap.importers.tumftm_track --input /tmp/tumftm/tracks --out data/tracks
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

# The exact attribution required for every vendored track (HANDOFF §4.4, USER DECISION #9).
ATTRIBUTION = (
    "Centerline © TU München, Institute of Automotive Technology "
    "(TUMFTM racetrack-database), LGPL-3.0"
)
# Upstream commit the vendored data was converted from (pin for reproducibility).
UPSTREAM_COMMIT = "e59595d1f3573b30d1ded6a08984935b957688e0"

# TUMFTM stem → (outlap directory name, human-readable circuit name). The directory name is the
# stable identifier used across data/tracks/; the display name feeds the loaded-model report.
TRACKS: dict[str, tuple[str, str]] = {
    "Austin": ("austin", "Circuit of the Americas"),
    "BrandsHatch": ("brands_hatch", "Brands Hatch Circuit"),
    "Budapest": ("budapest", "Hungaroring"),
    "Catalunya": ("catalunya", "Circuit de Barcelona-Catalunya"),
    "Hockenheim": ("hockenheim", "Hockenheimring"),
    "IMS": ("ims", "Indianapolis Motor Speedway (oval)"),
    "Melbourne": ("melbourne", "Albert Park Circuit"),
    "MexicoCity": ("mexico_city", "Autódromo Hermanos Rodríguez"),
    "Montreal": ("montreal", "Circuit Gilles Villeneuve"),
    "Monza": ("monza", "Autodromo Nazionale Monza"),
    "MoscowRaceway": ("moscow_raceway", "Moscow Raceway"),
    "Norisring": ("norisring", "Norisring"),
    "Nuerburgring": ("nuerburgring", "Nürburgring GP"),
    "Oschersleben": ("oschersleben", "Motorsport Arena Oschersleben"),
    "Sakhir": ("sakhir", "Bahrain International Circuit"),
    "SaoPaulo": ("sao_paulo", "Autódromo José Carlos Pace (Interlagos)"),
    "Sepang": ("sepang", "Sepang International Circuit"),
    "Shanghai": ("shanghai", "Shanghai International Circuit"),
    "Silverstone": ("silverstone", "Silverstone Circuit"),
    "Sochi": ("sochi", "Sochi Autodrom"),
    "Spa": ("spa", "Circuit de Spa-Francorchamps"),
    "Spielberg": ("spielberg", "Red Bull Ring"),
    "Suzuka": ("suzuka", "Suzuka Circuit"),
    "YasMarina": ("yas_marina", "Yas Marina Circuit"),
    "Zandvoort": ("zandvoort", "Circuit Zandvoort"),
}

# A source point closer than this to its predecessor is dropped (would break strictly-increasing s).
_COINCIDENT_TOL_M = 1e-6


@dataclass
class TumftmTrack:
    """A converted centre line in outlap's frame (SI metres; ISO 8855: x forward, y left, z up)."""

    dir_name: str
    name: str
    s: list[float]
    x: list[float]
    y: list[float]
    width_left: list[float]
    width_right: list[float]

    def __len__(self) -> int:
        return len(self.s)


def parse_tumftm_csv(
    text: str,
) -> tuple[list[float], list[float], list[float], list[float]]:
    """Parse a TUMFTM track CSV into ``(x, y, w_tr_right, w_tr_left)`` column lists.

    The header is a ``#`` comment line (``# x_m,y_m,w_tr_right_m,w_tr_left_m``); columns are
    positional in the source but named in outlap. Blank and comment lines are skipped.
    """
    x: list[float] = []
    y: list[float] = []
    w_right: list[float] = []
    w_left: list[float] = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = [f.strip() for f in line.split(",")]
        if len(fields) != 4:
            raise ValueError(
                f"line {lineno}: expected 4 columns (x_m,y_m,w_tr_right_m,w_tr_left_m), "
                f"found {len(fields)}"
            )
        try:
            xv, yv, wr, wl = (float(f) for f in fields)
        except ValueError as exc:
            raise ValueError(f"line {lineno}: non-numeric field ({exc})") from exc
        x.append(xv)
        y.append(yv)
        w_right.append(wr)
        w_left.append(wl)
    if len(x) < 4:
        raise ValueError("need at least 4 centre-line points")
    return x, y, w_right, w_left


def _cumulative_s(x: list[float], y: list[float]) -> list[float]:
    s = [0.0]
    for i in range(1, len(x)):
        s.append(s[-1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]))
    return s


def _dedup(
    x: list[float],
    y: list[float],
    w_right: list[float],
    w_left: list[float],
) -> tuple[list[float], list[float], list[float], list[float]]:
    """Drop points coincident with their predecessor (would break strictly-increasing ``s``)."""
    kx, ky, kr, kl = [x[0]], [y[0]], [w_right[0]], [w_left[0]]
    for i in range(1, len(x)):
        if math.hypot(x[i] - kx[-1], y[i] - ky[-1]) <= _COINCIDENT_TOL_M:
            continue
        kx.append(x[i])
        ky.append(y[i])
        kr.append(w_right[i])
        kl.append(w_left[i])
    return kx, ky, kr, kl


def _resample_closed(
    s: list[float],
    cols: dict[str, list[float]],
    perimeter: float,
    ds: float,
) -> tuple[list[float], dict[str, list[float]]]:
    """Resample a closed loop to a uniform ``ds`` grid (``endpoint=False``), interpolating the seam.

    ``perimeter`` is the full loop length including the closing chord; each column is extended with
    a wrap knot at ``s = perimeter`` (value = the first sample) so interpolation is continuous
    across the start/finish.
    """
    import numpy as np

    knots_s = [*s, perimeter]
    n = max(int(round(perimeter / ds)), 4)
    grid = [float(v) for v in np.linspace(0.0, perimeter, n, endpoint=False)]
    out: dict[str, list[float]] = {}
    for key, values in cols.items():
        knots_v = [*values, values[0]]
        out[key] = [float(v) for v in np.interp(grid, knots_s, knots_v)]
    return grid, out


def convert(
    text: str,
    dir_name: str,
    name: str,
    *,
    ds: float | None = None,
) -> TumftmTrack:
    """Convert one TUMFTM CSV into a :class:`TumftmTrack`.

    With ``ds is None`` the native ≈5 m source grid is passed through unchanged (exact — no
    interpolation of the measured widths); otherwise the closed loop is resampled to a uniform
    ``ds`` metres.
    """
    x, y, w_right, w_left = parse_tumftm_csv(text)
    x, y, w_right, w_left = _dedup(x, y, w_right, w_left)
    s = _cumulative_s(x, y)
    # Full perimeter includes the chord from the last source point back to the first.
    perimeter = s[-1] + math.hypot(x[0] - x[-1], y[0] - y[-1])

    if ds is not None:
        cols = {"x": x, "y": y, "wr": w_right, "wl": w_left}
        s, r = _resample_closed(s, cols, perimeter, ds)
        x, y, w_right, w_left = r["x"], r["y"], r["wr"], r["wl"]

    return TumftmTrack(
        dir_name=dir_name,
        name=name,
        s=s,
        x=x,
        y=y,
        # Map BY NAME: source LEFT → outlap width_left_m (+y); source RIGHT → width_right_m (−y).
        width_left=w_left,
        width_right=w_right,
    )


def write_track(track: TumftmTrack, out_root: Path) -> tuple[Path, Path]:
    """Write ``<out_root>/<dir_name>/{track.yaml,centerline.csv}`` for a converted track."""
    out_dir = out_root / track.dir_name
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "centerline.csv"
    yaml_path = out_dir / "track.yaml"

    with csv_path.open("w", encoding="utf-8") as f:
        f.write(
            f"# {track.name} — TUMFTM racetrack-database (LGPL-3.0), flat 2-D centre line "
            f"+ measured widths; upstream commit {UPSTREAM_COMMIT[:7]}.\n"
        )
        f.write("s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale\n")
        for i in range(len(track)):
            f.write(
                f"{track.s[i]:.4f},{track.x[i]:.4f},{track.y[i]:.4f},0.0000,0.000,"
                f"{track.width_left[i]:.3f},{track.width_right[i]:.3f},1.0000\n"
            )

    notes = (
        "flat 2-D: elevation and banking are zero (the TUMFTM data is strictly 2-D); corridor "
        f"widths are measured from satellite imagery; converted from upstream commit "
        f"{UPSTREAM_COMMIT[:7]}. See data/tracks/LICENSE-tumftm-LGPL-3.0.txt."
    )
    lines = [
        "# Imported by outlap.importers.tumftm_track — TUMFTM racetrack-database (LGPL-3.0 data).",
        "schema: track/1.0",
        f"name: {track.name}",
        "closed: true",
        "centerline: centerline.csv",
        "meta:",
        "  source: tumftm",
        "  accuracy_class: C",
        f'  attribution: "{ATTRIBUTION}"',
        f'  notes: "{notes}"',
        "",
    ]
    yaml_path.write_text("\n".join(lines), encoding="utf-8")
    return yaml_path, csv_path


def resolve(stem: str) -> tuple[str, str]:
    """Map a TUMFTM file stem to ``(dir_name, display_name)``; fall back to a snake_case slug."""
    if stem in TRACKS:
        return TRACKS[stem]
    slug = "".join(f"_{c.lower()}" if c.isupper() else c for c in stem).lstrip("_")
    return slug, stem


def _inputs(path: Path) -> list[Path]:
    if path.is_dir():
        return sorted(p for p in path.glob("*.csv"))
    return [path]


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        prog="outlap.importers.tumftm_track", description=__doc__
    )
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="a TUMFTM track CSV, or a directory of them (e.g. racetrack-database/tracks)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="output root; each track is written to <out>/<name>/",
    )
    parser.add_argument(
        "--ds",
        type=float,
        default=None,
        help="resample spacing, m (default: pass through the native ≈5 m grid unchanged)",
    )
    args = parser.parse_args(argv)

    files = _inputs(args.input)
    if not files:
        parser.error(f"no CSV files found at {args.input}")

    failed = False
    for path in files:
        dir_name, name = resolve(path.stem)
        try:
            track = convert(
                path.read_text(encoding="utf-8"), dir_name, name, ds=args.ds
            )
        except ValueError as exc:
            print(f"skipping {path.name}: {exc}", file=sys.stderr)
            failed = True
            continue
        yaml_path, _ = write_track(track, args.out)
        print(f"wrote {yaml_path.parent} ({len(track)} points, {track.s[-1]:.0f} m)")
    # Skip malformed files but still convert the rest; signal a nonzero exit if any were skipped.
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
