# SPDX-License-Identifier: AGPL-3.0-only
"""Semi-automated per-row track widths from orthophoto edge tracing (track fidelity, §9.3).

Traces the left/right track edges at every centerline station of an open orthophoto and emits
per-row ``width_left_m``/``width_right_m`` plus a provenance record for ``track.yaml`` meta.
The workflow is *semi*-automated by design:

1. an automatic pass finds, per station and side, the strongest cross-track intensity gradient
   within a plausible search band (asphalt→grass edges are high-contrast; the band's outer
   limit doubles as the pit-lane guard — a dark feature beyond it can never widen the track);
2. where detection fails (chicanes, kerbs, pit walls) a hand-placed :class:`ControlPoint` set —
   the small, reviewable, committed artifact (``s_m,side,offset_m`` CSV, see
   :func:`load_control_points`) — corrects it: a control point wins outright at its own station
   and blends smoothly into detection over a C¹ smoothstep hat of half-width
   :attr:`TraceParams.blend_window_m` on either side;
3. optional LiDAR section widths and a telemetry-corridor spread cross-check the traced total
   width: disagreement beyond :attr:`TraceParams.crosscheck_band_m` flags the station in the
   provenance (and on the QA overlay, ``tools/plot_track_width_qa.py``) but never mutates the
   traced widths.

Widths NEVER silently default (R1): a station with neither a detected edge nor control-point
coverage raises :class:`UnresolvedStationsError` naming every such station — no output exists
to write. Imagery reaches the tracer through the :class:`ImageSource` seam: tests draw
synthetic orthophotos in-memory (:class:`ArrayImageSource`); the real-data session plugs
orthophoto tile fetchers into the same seam. Units are SI metres in the track's local ENU
frame, axes ISO 8855 (x forward, y left): the LEFT edge lies at ``+offset`` along the local
left normal ``(-sin ψ, cos ψ)`` of the station heading ``ψ``.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Protocol

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]
"""A 1-D or 2-D array of SI floats (the module's working dtype)."""

Side = Literal["left", "right"]
"""Which track edge an offset refers to (ISO 8855: left = +y of the heading)."""

# Parabolic peak refinement guards: smaller curvature than this is numerically flat.
_REFINE_EPS = 1e-12
# Control-point weights below this are "no coverage" (R1 gap rescue needs real influence).
_WEIGHT_EPS = 1e-12

_CP_HEADER = "s_m,side,offset_m"


class WidthTraceError(Exception):
    """Base error for the width tracer (parse, geometry, or configuration problems)."""


class UnresolvedStationsError(WidthTraceError):
    """R1: stations with neither a detected edge nor a control point are a hard error.

    ``stations`` lists every unresolved ``(s_m, side)`` pair, sorted by ``s_m`` — widths are
    never silently defaulted, so the caller gets no output at all until each named station is
    covered (add a control point, or widen the search band / contrast threshold).
    """

    def __init__(self, stations: list[tuple[float, Side]]) -> None:
        self.stations: list[tuple[float, Side]] = stations
        parts: list[str] = []
        for side in ("left", "right"):
            svals = [s for s, sd in stations if sd == side]
            if svals:
                listing = ", ".join(f"s={s:.1f}" for s in svals)
                parts.append(f"{side}: {listing}")
        super().__init__(
            f"{len(stations)} unresolved station side(s) — no edge detected within the "
            "search band and no control point in range; add control points or adjust "
            "TraceParams. " + "; ".join(parts)
        )


@dataclass(frozen=True)
class AffineTransform:
    """Pixel (col, row) → world (x, y) affine map, rasterio coefficient order.

    ``x = a·col + b·row + c`` and ``y = d·col + e·row + f`` with (col, row) measured from the
    raster's top-left CORNER (so the centre of pixel ``[row, col]`` is at ``col + 0.5``,
    ``row + 0.5``). :meth:`from_origin` builds the common north-up case.
    """

    a: float
    b: float
    c: float
    d: float
    e: float
    f: float

    def __post_init__(self) -> None:
        if abs(self.a * self.e - self.b * self.d) < 1e-15:
            raise WidthTraceError("affine transform is singular (zero pixel area)")

    @classmethod
    def from_origin(
        cls, west: float, north: float, pixel_size_m: float
    ) -> AffineTransform:
        """North-up transform: top-left corner at ``(west, north)``, square pixels."""
        if pixel_size_m <= 0.0:
            raise WidthTraceError(f"pixel_size_m must be > 0, got {pixel_size_m}")
        return cls(pixel_size_m, 0.0, west, 0.0, -pixel_size_m, north)

    def world_to_pixel(
        self, x: FloatArray, y: FloatArray
    ) -> tuple[FloatArray, FloatArray]:
        """Invert the map: world (x, y) → fractional corner-based (col, row)."""
        det = self.a * self.e - self.b * self.d
        dx = np.asarray(x, dtype=np.float64) - self.c
        dy = np.asarray(y, dtype=np.float64) - self.f
        col = (self.e * dx - self.b * dy) / det
        row = (self.a * dy - self.d * dx) / det
        return col, row

    def pixel_centers(self, shape: tuple[int, int]) -> tuple[FloatArray, FloatArray]:
        """World coordinates of every pixel CENTRE of an ``(nrows, ncols)`` raster."""
        nrows, ncols = shape
        cols = np.arange(ncols, dtype=np.float64) + 0.5
        rows = np.arange(nrows, dtype=np.float64) + 0.5
        cgrid, rgrid = np.meshgrid(cols, rows)
        x = self.a * cgrid + self.b * rgrid + self.c
        y = self.d * cgrid + self.e * rgrid + self.f
        return x, y


class ImageSource(Protocol):
    """The imagery seam: grayscale intensity at world ENU points.

    Tests draw synthetic orthophotos in-memory (:class:`ArrayImageSource`); the data session
    plugs real tile fetchers into the same interface. Implementations return intensities in
    ``[0, 1]`` with the same shape as the query arrays.
    """

    def sample(self, x: FloatArray, y: FloatArray) -> FloatArray:
        """Sample intensity at world ``(x, y)`` [m]; arrays share one shape."""
        ...


class ArrayImageSource:
    """A georeferenced in-memory raster: bilinear sampling, border pixels clamp outward.

    ``image`` is ``H×W`` grayscale or ``H×W×3`` RGB (RGB is averaged); integer dtypes are
    scaled by 1/255. The clamp-at-border rule means the world outside the raster is a
    constant extension — it can never fabricate an intensity edge.
    """

    def __init__(self, image: npt.ArrayLike, transform: AffineTransform) -> None:
        raw = np.asarray(image)
        arr = raw.astype(np.float64)
        if arr.ndim == 3:
            arr = arr.mean(axis=-1)
        elif arr.ndim != 2:
            raise WidthTraceError(f"image must be HxW or HxWx3, got shape {raw.shape}")
        if np.issubdtype(raw.dtype, np.integer):
            arr = arr / 255.0
        if arr.shape[0] < 2 or arr.shape[1] < 2:
            raise WidthTraceError(f"image must be at least 2x2 pixels, got {arr.shape}")
        self._image = arr
        self.transform = transform

    @property
    def image(self) -> FloatArray:
        """The normalized grayscale raster (H×W, float64 in [0, 1])."""
        return self._image

    def sample(self, x: FloatArray, y: FloatArray) -> FloatArray:
        """Bilinear intensity at world ``(x, y)``; outside the raster clamps to the border."""
        col, row = self.transform.world_to_pixel(x, y)
        nrows, ncols = self._image.shape
        cf = np.clip(col - 0.5, 0.0, float(ncols - 1))
        rf = np.clip(row - 0.5, 0.0, float(nrows - 1))
        c0 = np.minimum(np.floor(cf).astype(np.intp), ncols - 2)
        r0 = np.minimum(np.floor(rf).astype(np.intp), nrows - 2)
        tc = cf - c0
        tr = rf - r0
        img = self._image
        top = (1.0 - tc) * img[r0, c0] + tc * img[r0, c0 + 1]
        bot = (1.0 - tc) * img[r0 + 1, c0] + tc * img[r0 + 1, c0 + 1]
        return (1.0 - tr) * top + tr * bot


@dataclass
class Stations:
    """Centerline stations in the track's local ENU frame (SI m / rad, ISO 8855).

    ``heading_rad`` is the travel direction ψ from +x; the LEFT edge search runs along
    ``(-sin ψ, cos ψ)``. ``length_m`` (closed loops: the full lap length, > ``s_m[-1]``)
    enables circular control-point distance across the start/finish seam.
    """

    s_m: FloatArray
    x_m: FloatArray
    y_m: FloatArray
    heading_rad: FloatArray
    length_m: float | None = None

    def __post_init__(self) -> None:
        self.s_m = np.asarray(self.s_m, dtype=np.float64)
        self.x_m = np.asarray(self.x_m, dtype=np.float64)
        self.y_m = np.asarray(self.y_m, dtype=np.float64)
        self.heading_rad = np.asarray(self.heading_rad, dtype=np.float64)
        n = len(self.s_m)
        if not (len(self.x_m) == len(self.y_m) == len(self.heading_rad) == n):
            raise WidthTraceError(
                "stations arrays must share one length: "
                f"s={n}, x={len(self.x_m)}, y={len(self.y_m)}, ψ={len(self.heading_rad)}"
            )
        if n < 2:
            raise WidthTraceError(f"need at least 2 stations, got {n}")
        if bool(np.any(np.diff(self.s_m) <= 0.0)):
            raise WidthTraceError("s_m must be strictly increasing")
        if self.length_m is not None and self.length_m <= float(self.s_m[-1]):
            raise WidthTraceError(
                f"length_m ({self.length_m}) must exceed the last station s ({self.s_m[-1]})"
            )

    def __len__(self) -> int:
        return len(self.s_m)


@dataclass(frozen=True)
class ControlPoint:
    """A hand-placed edge correction: at station ``s_m`` the ``side`` edge is ``offset_m`` out.

    The committed, reviewable artifact of the hand-QA session — re-running the trace with the
    same control points reproduces the same widths.
    """

    s_m: float
    side: Side
    offset_m: float


@dataclass(frozen=True)
class TraceParams:
    """Tuning for the automatic edge pass, the blend, and the cross-check band (SI m)."""

    search_min_m: float = 2.0
    """Inner edge of the cross-track search band (skips centerline paint / racing line)."""
    search_max_m: float = 12.0
    """Outer edge of the search band — also the pit-lane guard: features beyond it never win."""
    step_m: float = 0.1
    """Cross-track sampling step of the intensity profile."""
    min_gradient_per_m: float = 0.3
    """Minimum |dI/d(offset)| (intensity/m) for a detection; below it the station is a gap."""
    blend_window_m: float = 25.0
    """Half-width of the smoothstep hat over which a control point blends into detection."""
    crosscheck_band_m: float = 1.0
    """Total-width disagreement with LiDAR/telemetry beyond this flags the station."""


@dataclass(frozen=True)
class SideProvenance:
    """How one edge was traced: imagery source, method chain, and control-point usage."""

    source: str
    method: str
    control_point_count: int
    stations_influenced: int
    """Stations where control-point weight was nonzero (overridden or blended)."""


@dataclass(frozen=True)
class CrossCheckFlag:
    """One station where an independent width source disagrees beyond the band."""

    s_m: float
    source: str
    width_traced_m: float
    width_ref_m: float


@dataclass(frozen=True)
class WidthProvenance:
    """The provenance record the caller writes into ``track.yaml`` meta (see :meth:`as_meta`)."""

    left: SideProvenance
    right: SideProvenance
    flags: tuple[CrossCheckFlag, ...] = ()

    def flagged_stations_m(self) -> list[float]:
        """Stations flagged by any cross-check, in station order."""
        return [f.s_m for f in self.flags]

    def as_meta(self) -> dict[str, object]:
        """A plain-YAML-safe dict (scalars/lists/dicts only) for ``track.yaml`` meta."""

        def side(p: SideProvenance) -> dict[str, object]:
            return {
                "source": p.source,
                "method": p.method,
                "control_points": p.control_point_count,
                "stations_influenced": p.stations_influenced,
            }

        return {
            "width_method": "width_trace",
            "left": side(self.left),
            "right": side(self.right),
            "flagged_stations_m": [round(f.s_m, 2) for f in self.flags],
            "crosscheck_flags": [
                {
                    "s_m": round(f.s_m, 2),
                    "source": f.source,
                    "width_traced_m": round(f.width_traced_m, 2),
                    "width_ref_m": round(f.width_ref_m, 2),
                }
                for f in self.flags
            ],
        }


@dataclass
class WidthTraceResult:
    """Per-station widths plus everything the QA overlay needs to second-guess them.

    ``detected_*`` are the RAW automatic detections (NaN where the pass found no edge),
    kept alongside the final blended widths so the overlay can show what the control
    points changed.
    """

    s_m: FloatArray
    width_left_m: FloatArray
    width_right_m: FloatArray
    detected_left_m: FloatArray
    detected_right_m: FloatArray
    provenance: WidthProvenance = field(kw_only=True)

    @property
    def width_total_m(self) -> FloatArray:
        """Total corridor width (left + right) per station."""
        return self.width_left_m + self.width_right_m


def load_control_points(path: Path | str) -> list[ControlPoint]:
    """Read a control-point CSV (``s_m,side,offset_m``; ``#`` comments, header optional).

    This file is the committed hand-QA artifact; malformed rows raise
    :class:`WidthTraceError` with the offending line number.
    """
    path = Path(path)
    out: list[ControlPoint] = []
    for lineno, raw in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.replace(" ", "").lower() == _CP_HEADER:
            continue
        fields = [f.strip() for f in line.split(",")]
        if len(fields) != 3:
            raise WidthTraceError(
                f"{path}: line {lineno}: expected 3 columns ({_CP_HEADER}), "
                f"found {len(fields)}"
            )
        try:
            s_val = float(fields[0])
            offset = float(fields[2])
        except ValueError as exc:
            raise WidthTraceError(
                f"{path}: line {lineno}: non-numeric field ({exc})"
            ) from exc
        side_raw = fields[1].lower()
        if side_raw not in ("left", "right"):
            raise WidthTraceError(
                f"{path}: line {lineno}: side must be 'left' or 'right', got '{fields[1]}'"
            )
        side: Side = "left" if side_raw == "left" else "right"
        if offset <= 0.0:
            raise WidthTraceError(
                f"{path}: line {lineno}: offset_m must be > 0 (widths never default)"
            )
        out.append(ControlPoint(s_m=s_val, side=side, offset_m=offset))
    return out


def _validate_params(p: TraceParams) -> None:
    if not 0.0 <= p.search_min_m < p.search_max_m:
        raise WidthTraceError(
            f"search band must satisfy 0 <= min < max, got [{p.search_min_m}, {p.search_max_m}]"
        )
    if not 0.0 < p.step_m <= p.search_max_m - p.search_min_m:
        raise WidthTraceError(f"step_m must be in (0, band width], got {p.step_m}")
    if p.min_gradient_per_m <= 0.0:
        raise WidthTraceError(
            f"min_gradient_per_m must be > 0, got {p.min_gradient_per_m}"
        )
    if p.blend_window_m <= 0.0:
        raise WidthTraceError(f"blend_window_m must be > 0, got {p.blend_window_m}")
    if p.crosscheck_band_m <= 0.0:
        raise WidthTraceError(
            f"crosscheck_band_m must be > 0, got {p.crosscheck_band_m}"
        )


def _detect_side(
    stations: Stations, image: ImageSource, params: TraceParams, sign: float
) -> FloatArray:
    """Strongest cross-track |intensity gradient| within the search band; NaN where none.

    ``sign`` is +1 for the LEFT edge (along the left normal ``(-sin ψ, cos ψ)``), −1 for the
    RIGHT. The peak is refined to sub-step accuracy with a 3-point parabolic fit and clamped
    back into the band, so an off-band feature (a pit lane) can never win — the guard that
    keeps a plausible-band miss an explicit *gap* rather than a silently wrong width.
    """
    offsets = np.arange(0.0, params.search_max_m + 2.0 * params.step_m, params.step_m)
    nx = -np.sin(stations.heading_rad) * sign
    ny = np.cos(stations.heading_rad) * sign
    px = stations.x_m[:, None] + nx[:, None] * offsets[None, :]
    py = stations.y_m[:, None] + ny[:, None] * offsets[None, :]
    profile = image.sample(px, py)
    grad = np.abs(np.gradient(profile, params.step_m, axis=1))
    band = (offsets >= params.search_min_m) & (offsets <= params.search_max_m)
    banded = np.where(band[None, :], grad, -np.inf)
    idx = np.argmax(banded, axis=1)
    rows = np.arange(len(stations))
    resolved = banded[rows, idx] >= params.min_gradient_per_m
    # Parabolic sub-step refinement around the peak (raw neighbours, then re-clamped to band).
    im1 = np.clip(idx - 1, 0, grad.shape[1] - 1)
    ip1 = np.clip(idx + 1, 0, grad.shape[1] - 1)
    gm, g0, gp = grad[rows, im1], grad[rows, idx], grad[rows, ip1]
    denom = gm - 2.0 * g0 + gp
    with np.errstate(divide="ignore", invalid="ignore"):
        # The 0/0 branch is discarded by the np.where mask (flat-gradient stations).
        delta = np.where(np.abs(denom) > _REFINE_EPS, 0.5 * (gm - gp) / denom, 0.0)
    delta = np.clip(delta, -1.0, 1.0)
    edge = np.clip(
        offsets[idx] + delta * params.step_m, params.search_min_m, params.search_max_m
    )
    return np.where(resolved, edge, np.nan)


def _apply_control_points(
    s: FloatArray,
    detected: FloatArray,
    cps: Sequence[ControlPoint],
    window_m: float,
    wrap_length_m: float | None,
) -> tuple[FloatArray, FloatArray]:
    """Blend control points over the detected offsets; returns ``(offsets, cp_weight)``.

    Each control point carries a C¹ smoothstep hat of half-width ``window_m`` (circular
    distance when ``wrap_length_m`` is given). Where the total hat weight reaches 1 the
    control points own the value outright (weighted mean); below 1 the remainder comes from
    detection. A detection gap (NaN) covered by ANY control-point influence takes the pure
    control-point value — the R1 rescue path.
    """
    w_tot = np.zeros_like(s)
    acc = np.zeros_like(s)
    for cp in cps:
        d = np.abs(s - cp.s_m)
        if wrap_length_m is not None:
            d = np.minimum(d, wrap_length_m - d)
        u = np.clip(1.0 - d / window_m, 0.0, 1.0)
        w = u * u * (3.0 - 2.0 * u)
        acc += w * cp.offset_m
        w_tot += w
    if not cps:
        return detected.copy(), w_tot
    cp_mean = acc / np.maximum(w_tot, _WEIGHT_EPS)
    blended = np.where(w_tot >= 1.0, cp_mean, acc + (1.0 - w_tot) * detected)
    gap_rescued = np.isnan(detected) & (w_tot > _WEIGHT_EPS)
    return np.where(gap_rescued, cp_mean, blended), w_tot


def _side_provenance(
    cps: Sequence[ControlPoint], cp_weight: FloatArray
) -> SideProvenance:
    method = "cross_track_gradient"
    if cps:
        method += "+control_points"
    return SideProvenance(
        source="orthophoto",
        method=method,
        control_point_count=len(cps),
        stations_influenced=int(np.count_nonzero(cp_weight > _WEIGHT_EPS)),
    )


def trace_widths(
    stations: Stations,
    image: ImageSource,
    *,
    params: TraceParams | None = None,
    control_points: Sequence[ControlPoint] = (),
    lidar_width_m: tuple[npt.ArrayLike, npt.ArrayLike] | None = None,
    telemetry_width_m: tuple[npt.ArrayLike, npt.ArrayLike] | None = None,
) -> WidthTraceResult:
    """Trace per-station edge offsets from the orthophoto; error-not-default on gaps (R1).

    ``lidar_width_m`` / ``telemetry_width_m`` are optional independent TOTAL-width sources as
    ``(s_m, width_m)`` arrays (strictly increasing ``s_m``; interpolated onto the stations,
    clamped at the ends). They only *flag* — see :class:`WidthProvenance`.

    Raises :class:`UnresolvedStationsError` if any station side has neither a detected edge
    nor control-point coverage, and :class:`WidthTraceError` for invalid parameters or
    cross-check arrays.
    """
    p = params if params is not None else TraceParams()
    _validate_params(p)
    for cp in control_points:
        if cp.offset_m <= 0.0:
            raise WidthTraceError(
                f"control point at s={cp.s_m:.1f} ({cp.side}): offset_m must be > 0"
            )

    detected_left = _detect_side(stations, image, p, +1.0)
    detected_right = _detect_side(stations, image, p, -1.0)
    cps_left = [cp for cp in control_points if cp.side == "left"]
    cps_right = [cp for cp in control_points if cp.side == "right"]
    width_left, cpw_left = _apply_control_points(
        stations.s_m, detected_left, cps_left, p.blend_window_m, stations.length_m
    )
    width_right, cpw_right = _apply_control_points(
        stations.s_m, detected_right, cps_right, p.blend_window_m, stations.length_m
    )

    unresolved: list[tuple[float, Side]] = []
    side_widths: list[tuple[Side, FloatArray]] = [
        ("left", width_left),
        ("right", width_right),
    ]
    for side, values in side_widths:
        for s_val in stations.s_m[np.isnan(values)]:
            unresolved.append((float(s_val), side))
    if unresolved:
        unresolved.sort()
        raise UnresolvedStationsError(unresolved)

    total = width_left + width_right
    flags: list[CrossCheckFlag] = []
    for name, ref in (("lidar", lidar_width_m), ("telemetry", telemetry_width_m)):
        if ref is None:
            continue
        s_ref = np.asarray(ref[0], dtype=np.float64)
        w_ref = np.asarray(ref[1], dtype=np.float64)
        if s_ref.ndim != 1 or s_ref.shape != w_ref.shape or len(s_ref) < 2:
            raise WidthTraceError(
                f"{name}_width_m must be two equal-length 1-D arrays (s_m, width_m)"
            )
        if bool(np.any(np.diff(s_ref) <= 0.0)):
            raise WidthTraceError(f"{name}_width_m s_m must be strictly increasing")
        ref_on_stations = np.interp(stations.s_m, s_ref, w_ref)
        for i in np.flatnonzero(np.abs(total - ref_on_stations) > p.crosscheck_band_m):
            flags.append(
                CrossCheckFlag(
                    s_m=float(stations.s_m[i]),
                    source=name,
                    width_traced_m=float(total[i]),
                    width_ref_m=float(ref_on_stations[i]),
                )
            )

    provenance = WidthProvenance(
        left=_side_provenance(cps_left, cpw_left),
        right=_side_provenance(cps_right, cpw_right),
        flags=tuple(sorted(flags, key=lambda fl: fl.s_m)),
    )
    return WidthTraceResult(
        s_m=stations.s_m.copy(),
        width_left_m=width_left,
        width_right_m=width_right,
        detected_left_m=detected_left,
        detected_right_m=detected_right,
        provenance=provenance,
    )
