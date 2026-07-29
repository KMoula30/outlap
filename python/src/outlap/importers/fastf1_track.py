# SPDX-License-Identifier: AGPL-3.0-only
"""FastF1 position telemetry → a georeferenced **driven-line** track dir + input manifest.

The calibration counterpart to :mod:`outlap.importers.osm_track` (HANDOFF §12 MT, unit U2).
Where the OSM importer builds a *corridor* to drive a raceline through, this one reconstructs
the line the cars actually drove, in real-world coordinates, so imported geometry can be
audited against it (apex radii, corridor feasibility) and the car calibrated on it. The
artifact class is deliberately narrow — see **Driven line, not a corridor** below.

**Stages** (all mandatory; this importer has no optional enrichment — its artifact class is
fixed, unlike the staged OSM pipeline):

1. **Pool** position samples across the selected laps and drivers through
   :func:`outlap.trackcal.data.load_fastf1_positions`, which drops every
   ``Source == 'interpolated'`` row so reconstructed samples never masquerade as
   measurements. Counts before *and* after the filter are recorded. FastF1 emits X/Y in
   1/10 m in a **local, unscaled, non-georeferenced frame** (``circuit_info().rotation`` is
   only a display hint), so the pooled cloud is not yet metric truth. Laps are reduced to one
   ordered **mean line** — projected onto a reference lap for a common arc length, then
   averaged into fixed arc-length bins — because neither concatenation nor phase-merging
   survives contact with real lap boundaries, and stacking redundant samples makes the
   discrepancy principle buy flexibility it cannot support. :func:`pool_positions` records
   what each failure mode measured.
2. **Georeference** — the load-bearing stage. A similarity transform (rotation + uniform
   scale + translation) is fitted from hand-picked anchor correspondences to real-world ENU
   by the closed-form Umeyama/Kabsch solution, and its **anchor residual is gated**
   (:data:`MAX_ANCHOR_RESIDUAL_M`) before anything downstream runs: a misregistered anchor
   set raises :class:`GeoreferenceResidualError` and no file is written. Reflections are
   excluded (a mirrored correspondence set is a misregistration, not a valid fit), and fewer
   than :data:`MIN_ANCHORS` anchors is an error because a 2-point similarity fit is exactly
   determined — its zero residual would certify nothing. The transform and its residual are
   recorded in the manifest (KTD7), in ``track.yaml`` meta, and in the reference-metrics CSV
   header.
3. **Fit** the georeferenced cloud with the :mod:`outlap.trackcal.geometry` penalised
   periodic smoothing spline at the declared FastF1 noise, and **audit** it with
   :func:`outlap.trackcal.corners.detect_corners` (Hyper circle fits on short arcs) into a
   committed-format reference-metrics CSV (R4).

**Driven line, not a corridor.** Widths are emitted as an honest narrow constant
(:data:`DRIVEN_LINE_HALF_WIDTH_M` per side, the shipped ``barcelona_real_2026`` precedent)
and ``width_source`` says ``driven-line``; z and banking are zero because position telemetry
carries no usable elevation. ``accuracy_class`` is therefore ``C`` and the meta notes say so
in words. Do **not** solve a raceline in this corridor — it is for audit and calibration.

**Reproducibility (KTD7, R7).** Unlike the OSM snapshot, the input here *cannot* be committed:
HANDOFF §15 permits FastF1-derived calibration artifacts but forbids redistributing raw
telemetry. So the pinned inputs are the anchor CSV (committed, hashed) plus a manifest that
records the session key, the laps and drivers pooled, the sample counts either side of the
interpolated filter, a content hash of the pooled samples, every fit numeric, and the importer
version. Given the same session the import is a pure function of those inputs and reproduces
``centerline.csv`` byte-identically; the manifest's ``positions.sha256`` is what makes a re-run
*checkable* against the original.

**Anchor file** (``outlap-georef-anchors/1``) — the small, reviewable, committed registration
artifact. ``#`` comment lines, then one row per correspondence::

    # outlap-georef-anchors/1
    label,local_x_m,local_y_m,ref_lat_deg,ref_lon_deg
    start_finish,137.2761,-25.5590,41.5699010,2.2612340

``local_*`` are FastF1-frame metres (i.e. the raw 1/10 m units divided by ten, exactly what
:func:`outlap.trackcal.data.load_fastf1_positions` returns); ``ref_*`` are WGS84 degrees for
the same physical point, read off the georeferenced source (OSM start/finish node,
unambiguous corner apexes).

Builds are atomic and ``--force`` is required over existing outputs — the write path is
:func:`outlap.importers.osm_track.write_track_dir`. That module owns the whole emitted-format
surface (the CSV and YAML renderers, the manifest hashes, the station headings) and this one
imports it, so there is exactly one implementation of each between the two importers and the
byte layout cannot drift.

``fastf1`` is imported lazily (inside the loader) and lives in the ``wear-cal`` extra; CI never
installs it and never runs this module.

Citation: S. Umeyama (1991), "Least-squares estimation of transformation parameters between
two point patterns", IEEE TPAMI 13(4), 376–380 — the closed-form similarity fit (§3, the
scale/rotation/translation solution and its reflection correction) used in
:func:`fit_similarity`.

Example:
    python -m outlap.importers.fastf1_track --out data/tracks/barcelona_real_2026 \\
        --year 2026 --event "Spanish Grand Prix" --session R \\
        --anchors data/tracks/barcelona_real_2026/georef_anchors.csv \\
        --drivers VER,NOR --max-laps-per-driver 5 --enu-origin 41.5700,2.2611
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray

from outlap.importers.lidar_dem import EnuFrame
from outlap.importers.osm_track import (
    FittedCenterline,
    headings,
    render_centerline_csv,
    render_yaml,
    sha256_file,
    sha256_text,
    verify_track_dir,
    write_track_dir,
)
from outlap.trackcal.corners import detect_corners
from outlap.trackcal.data import (
    GeorefTransform,
    PositionTrace,
    TrackMetrics,
    format_georef,
    load_fastf1_positions,
    metrics_from_corners,
    write_metrics,
)
from outlap.trackcal.geometry import fit_centerline

if TYPE_CHECKING:
    from collections.abc import Sequence

F = NDArray[np.float64]

#: Importer version recorded in every manifest + ``track.yaml`` meta (KTD7). Bump on any
#: change that can move the emitted geometry.
IMPORTER_VERSION = "1.0.0"

#: Generated outputs in a driven-line track dir (the anchor CSV is a committed *input*).
MANIFEST_FILE = "manifest.yaml"
METRICS_FILE = "track_metrics.csv"
OUTPUT_FILES = ("centerline.csv", "track.yaml", METRICS_FILE, MANIFEST_FILE)

#: Format tag of the committed anchor-correspondence CSV.
ANCHORS_FORMAT = "outlap-georef-anchors/1"
_ANCHOR_COLUMNS = ("label", "local_x_m", "local_y_m", "ref_lat_deg", "ref_lon_deg")

#: A 2-D similarity has 4 degrees of freedom, so two correspondences determine it exactly and
#: leave a zero residual that certifies nothing. Three is the smallest set the gate can judge.
MIN_ANCHORS = 3

#: Anchor-residual ceiling (RMS, metres) the georeference must clear before the import runs.
#: Chosen from three independent bounds: it is ~15x the 0.3 m FastF1 position noise floor, so
#: it never fires on noise; it sits under the narrowest F1 corridor half-width, so a
#: georeference error that passes cannot by itself push the driven line outside the corridor
#: it will be audited against; and any true misregistration (a swapped or mislabelled
#: correspondence on a kilometres-long circuit) residuals two to three orders of magnitude
#: above it. Hand-picking a corner apex on both sources is worth a couple of metres, so this
#: is a real but reachable bar.
MAX_ANCHOR_RESIDUAL_M = 5.0

#: Smallest admissible ratio between the minor and major axes of the anchor cloud.
#:
#: The residual gate cannot see this failure. Anchors strung along one straight leave the
#: perpendicular direction undetermined: mirroring the track about that line moves the anchors
#: themselves barely at all, so the fit clears its residual ceiling while every corner comes out
#: reflected. Umeyama's reflection guard does not help — it returns the nearest *proper*
#: rotation, which for a collinear set is exactly the mirrored one. So the anchor geometry has to
#: be checked directly: the cloud must span at least this fraction of its own length across.
#: 0.05 admits any anchor set picked from more than one part of a circuit (a lap is roughly as
#: wide as it is long) and rejects the module's own worst suggestion — start/finish plus two
#: braking markers down the same straight.
MIN_ANCHOR_ASPECT = 0.05

#: Declared per-axis measurement noise of FastF1 positions (the discrepancy principle matches
#: it). Pooling several *drivers* widens the cloud with genuine racing-line spread rather than
#: noise — raise the declaration when doing that.
FASTF1_NOISE_STD_M = 0.3

#: Honest half-width of a driven line (the shipped ``barcelona_real_2026`` precedent).
DRIVEN_LINE_HALF_WIDTH_M = 0.8

_DS_M = 3.0
_KNOT_SPACING_M = 3.0
_CORNER_MIN_RADIUS_M = 200.0
#: Floor on the pooling bin count (a track too short to bin is a degenerate input anyway).
MIN_BINS = 16

_STAGES = ("positions", "georeference", "fit")


class FastF1TrackError(ValueError):
    """Base error for the FastF1 driven-line importer (typed, never a bare crash)."""


class AnchorFormatError(FastF1TrackError):
    """The anchor CSV is malformed (missing format tag, columns, or numeric fields)."""


class TooFewAnchorsError(FastF1TrackError):
    """Fewer than :data:`MIN_ANCHORS` correspondences — the residual gate cannot bind."""


class GeoreferenceResidualError(FastF1TrackError):
    """The fitted similarity transform misses its anchors by more than the ceiling."""


class CollinearAnchorsError(FastF1TrackError):
    """The anchor cloud is too close to a straight line to determine the transform.

    Distinct from :class:`GeoreferenceResidualError`: the fit *succeeds* here, with a small
    residual, and is still wrong — a mirrored solution fits collinear anchors equally well.
    """


class NoPositionSamplesError(FastF1TrackError):
    """No measured position samples survived the ``Source == 'interpolated'`` filter."""


# --- session identity -------------------------------------------------------------------------


@dataclass(frozen=True)
class SessionKey:
    """The pinned FastF1 session (KTD7): ``2026 Spanish Grand Prix R``."""

    year: int
    event: str | int
    session: str

    def __str__(self) -> str:
        return f"{self.year} {self.event} {self.session}"

    def as_manifest(self) -> dict[str, Any]:
        """Manifest fields for this session key."""
        return {
            "year": int(self.year),
            "event": self.event,
            "session": self.session,
            "key": str(self),
        }


# --- anchors + the georeference (the load-bearing stage) ---------------------------------------


@dataclass(frozen=True)
class Anchor:
    """One hand-picked correspondence: a FastF1-frame point and its WGS84 position."""

    label: str
    local_x_m: float
    local_y_m: float
    ref_lat_deg: float
    ref_lon_deg: float


@dataclass(frozen=True)
class Georeference:
    """A gated similarity fit: the transform, its residuals, and the frame it targets.

    ``worst_residual_m`` is *measured* — how far the fit misses its worst anchor;
    ``residual_ceiling_m`` is *declared* — the bar the caller set the RMS residual against
    (:data:`MAX_ANCHOR_RESIDUAL_M` by default). Both are recorded so a manifest states the
    gate and its outcome, never one without the other.
    """

    transform: GeorefTransform
    frame: EnuFrame
    worst_residual_m: float
    residual_ceiling_m: float
    labels: tuple[str, ...]

    @property
    def residual_rms_m(self) -> float:
        """RMS anchor residual of the fit (metres)."""
        return self.transform.residual_rms_m

    def as_manifest(self) -> dict[str, Any]:
        """Manifest fields for the fitted transform (KTD7)."""
        return {
            "method": "umeyama-similarity-2d",
            "scale": float(self.transform.scale),
            "rotation_rad": float(self.transform.rotation_rad),
            "tx_m": float(self.transform.tx_m),
            "ty_m": float(self.transform.ty_m),
            "residual_rms_m": float(self.transform.residual_rms_m),
            "worst_residual_m": float(self.worst_residual_m),
            "residual_ceiling_m": float(self.residual_ceiling_m),
            "enu_origin": {
                "lat_deg": self.frame.lat0_deg,
                "lon_deg": self.frame.lon0_deg,
            },
            "anchors": list(self.labels),
        }


def load_anchors(path: Path) -> list[Anchor]:
    """Read the committed anchor-correspondence CSV (``outlap-georef-anchors/1``)."""
    if not path.exists():
        raise AnchorFormatError(f"no anchor file at {path}")
    rows: list[str] = []
    tagged = False
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            tagged = tagged or stripped.lstrip("#").strip() == ANCHORS_FORMAT
            continue
        rows.append(stripped)
    if not tagged:
        raise AnchorFormatError(
            f"{path}: missing the `# {ANCHORS_FORMAT}` format header"
        )
    if not rows or tuple(c.strip() for c in rows[0].split(",")) != _ANCHOR_COLUMNS:
        got = rows[0] if rows else "<empty>"
        raise AnchorFormatError(
            f"{path}: expected columns {','.join(_ANCHOR_COLUMNS)}, got {got!r}"
        )
    anchors: list[Anchor] = []
    for row in rows[1:]:
        fields = [f.strip() for f in row.split(",")]
        if len(fields) != len(_ANCHOR_COLUMNS):
            raise AnchorFormatError(f"{path}: malformed anchor row {row!r}")
        try:
            anchors.append(
                Anchor(
                    label=fields[0],
                    local_x_m=float(fields[1]),
                    local_y_m=float(fields[2]),
                    ref_lat_deg=float(fields[3]),
                    ref_lon_deg=float(fields[4]),
                )
            )
        except ValueError as err:
            raise AnchorFormatError(
                f"{path}: non-numeric coordinate in anchor row {row!r}"
            ) from err
    return anchors


def fit_similarity(local_xy: F, ref_xy: F) -> tuple[GeorefTransform, F]:
    """Closed-form 2-D similarity fit (Umeyama 1991 §3): local frame → reference metres.

    Minimises ``Σ ‖q_i − (c R p_i + t)‖²`` over uniform scale ``c``, rotation ``R`` and
    translation ``t``. With centred point sets and ``Σ = (1/n) Q̃ᵀ P̃ = U D Vᵀ``, the solution
    is ``R = U S Vᵀ``, ``c = tr(D S) / σ_p²`` and ``t = q̄ − c R p̄``, where ``S = I`` unless
    ``det(U) det(V) < 0``, in which case its last diagonal entry flips. That correction keeps
    the result a **proper rotation**: a correspondence set best explained by a reflection is
    a misregistration, and forcing a rotation lets the residual gate say so instead of
    silently mirroring the track.

    Returns the transform (with its RMS residual filled in) and the per-anchor residual
    distances in metres.
    """
    p = np.asarray(local_xy, dtype=np.float64)
    q = np.asarray(ref_xy, dtype=np.float64)
    if p.shape != q.shape or p.ndim != 2 or p.shape[1] != 2:
        raise FastF1TrackError("similarity fit needs two matching (n, 2) point sets")
    n = p.shape[0]
    p_bar, q_bar = p.mean(axis=0), q.mean(axis=0)
    p_c, q_c = p - p_bar, q - q_bar
    var_p = float(np.sum(p_c**2)) / n
    if var_p <= 1e-12:
        raise FastF1TrackError(
            "anchor points are coincident — no transform is determined"
        )
    cov = (q_c.T @ p_c) / n
    u_mat, sing, vt_mat = np.linalg.svd(cov)
    s_diag = np.ones(2, dtype=np.float64)
    if float(np.linalg.det(u_mat) * np.linalg.det(vt_mat)) < 0.0:
        s_diag[-1] = -1.0  # reflection guard (Umeyama 1991, eq. 39)
    rot = u_mat @ np.diag(s_diag) @ vt_mat
    scale = float(np.sum(sing * s_diag)) / var_p
    trans = q_bar - scale * (rot @ p_bar)
    transform = GeorefTransform(
        scale=scale,
        rotation_rad=float(np.arctan2(rot[1, 0], rot[0, 0])),
        tx_m=float(trans[0]),
        ty_m=float(trans[1]),
        residual_rms_m=0.0,
    )
    fitted = np.stack(transform.apply(p[:, 0], p[:, 1]), axis=1)
    residuals = np.hypot(fitted[:, 0] - q[:, 0], fitted[:, 1] - q[:, 1])
    rms = float(np.sqrt(np.mean(residuals**2)))
    return (
        GeorefTransform(
            scale=transform.scale,
            rotation_rad=transform.rotation_rad,
            tx_m=transform.tx_m,
            ty_m=transform.ty_m,
            residual_rms_m=rms,
        ),
        residuals,
    )


def georeference(
    anchors: Sequence[Anchor],
    *,
    frame: EnuFrame | None = None,
    max_residual_m: float = MAX_ANCHOR_RESIDUAL_M,
) -> Georeference:
    """Fit the anchor correspondences and **assert the residual ceiling** (KTD7).

    ``frame`` pins the real-world ENU origin (pass the OSM import's frame to land in exactly
    its coordinates); the anchor centroid is used when it is omitted. Raises
    :class:`TooFewAnchorsError` below :data:`MIN_ANCHORS` and
    :class:`GeoreferenceResidualError` when the fit misses its anchors — in both cases before
    any downstream stage runs, so a rejected georeference writes nothing.
    """
    if len(anchors) < MIN_ANCHORS:
        raise TooFewAnchorsError(
            f"the georeference needs >= {MIN_ANCHORS} anchor correspondences, got "
            f"{len(anchors)}: a 2-point similarity fit is exactly determined, so its zero "
            "residual would certify nothing"
        )
    lat = np.array([a.ref_lat_deg for a in anchors], dtype=np.float64)
    lon = np.array([a.ref_lon_deg for a in anchors], dtype=np.float64)
    if frame is None:
        frame = EnuFrame(lat0_deg=float(lat.mean()), lon0_deg=float(lon.mean()))
    ref = np.stack(frame.to_enu(lat, lon), axis=1)
    local = np.array([[a.local_x_m, a.local_y_m] for a in anchors], dtype=np.float64)
    # Conditioning first: the residual gate below is blind to a near-collinear anchor set, and a
    # mirrored fit through such a set passes it (see MIN_ANCHOR_ASPECT).
    spread = np.linalg.svd(local - local.mean(axis=0), compute_uv=False)
    aspect = float(spread[1] / spread[0]) if spread[0] > 0.0 else 0.0
    if aspect < MIN_ANCHOR_ASPECT:
        raise CollinearAnchorsError(
            f"georeference rejected: the anchors are nearly collinear (minor/major axis "
            f"{aspect:.4f} < {MIN_ANCHOR_ASPECT}). Their perpendicular direction is "
            "undetermined, so a mirrored fit would clear the residual ceiling and every corner "
            "would come out reflected. Spread the anchors around the lap — a start/finish plus "
            "two corner apexes on opposite sides beats three points down one straight. "
            "Nothing was written."
        )
    transform, residuals = fit_similarity(local, ref)
    worst = int(np.argmax(residuals))
    if transform.residual_rms_m > max_residual_m:
        raise GeoreferenceResidualError(
            f"georeference rejected: anchor residual {transform.residual_rms_m:.2f} m RMS "
            f"exceeds the {max_residual_m:.2f} m ceiling (worst anchor "
            f"`{anchors[worst].label}` off by {float(residuals[worst]):.2f} m) — the "
            "correspondences do not describe one similarity transform; check for a swapped, "
            "mislabelled or mirrored pair. Nothing was written."
        )
    return Georeference(
        transform=transform,
        frame=frame,
        worst_residual_m=float(residuals[worst]),
        residual_ceiling_m=float(max_residual_m),
        labels=tuple(a.label for a in anchors),
    )


# --- pooling ------------------------------------------------------------------------------------


@dataclass(frozen=True)
class PooledPositions:
    """The pooled **mean driven line**: one ordered station per bin, local FastF1 frame.

    ``x_m``/``y_m`` are per-bin means of the measured samples that fell in them, so
    ``effective_noise_std_m`` is what the fit must be declared against, not the per-sample
    noise: averaging ``mᵢ`` samples leaves ``σ/√mᵢ`` per bin, and the discrepancy principle
    matches an RMS over bins, hence ``σ·√(mean(1/mᵢ))``.

    That formula assumes the only thing separating samples in a bin is measurement noise, which
    stops being true the moment more than one driver is pooled — different racing lines are real
    spread, not error. So the declaration is the larger of that figure and the **measured**
    within-bin scatter, and pooling drivers honestly widens it instead of quietly over-fitting
    the mean line.
    """

    x_m: F
    y_m: F
    s_m: F
    samples_per_bin: NDArray[np.int64]
    effective_noise_std_m: float
    lap_labels: tuple[str, ...]
    drivers: tuple[str, ...]
    n_used: int
    n_dropped_interpolated: int
    lap_length_m: float

    @property
    def n_raw(self) -> int:
        """Sample count *before* the ``Source == 'interpolated'`` filter."""
        return self.n_used + self.n_dropped_interpolated

    @property
    def n_bins(self) -> int:
        """Number of occupied bins — the station count the fit actually sees."""
        return int(self.x_m.size)

    def sha256(self) -> str:
        """Content hash of the pooled line — pins an input that must not be committed."""
        digest = hashlib.sha256()
        for arr in (self.x_m, self.y_m):
            digest.update(np.ascontiguousarray(arr, dtype=np.float64).tobytes())
        return digest.hexdigest()


#: Reference-curve resolution the pooled samples are projected onto (metres). The resulting
#: ±half-step quantisation of the ordering coordinate must stay well under the position noise.
_PROJECTION_STEP_M = 0.25
#: Samples per projection chunk — bounds the temporary distance matrix on real-circuit sizes.
_PROJECTION_CHUNK = 256
#: Arc-length width of a pooling bin (metres); see :func:`pool_positions`.
POOL_BIN_M = 4.0


def _project_arclength(x: F, y: F, ref_x: F, ref_y: F, ref_s: F) -> F:
    """Arc length of the nearest point on a densely-sampled reference curve, per sample."""
    s = np.empty(x.size, dtype=np.float64)
    for i in range(0, x.size, _PROJECTION_CHUNK):
        sl = slice(i, i + _PROJECTION_CHUNK)
        dx = x[sl, None] - ref_x[None, :]
        dy = y[sl, None] - ref_y[None, :]
        s[sl] = ref_s[np.argmin(dx * dx + dy * dy, axis=1)]
    return s


def pool_positions(
    traces: Sequence[PositionTrace],
    *,
    noise_std_m: float = FASTF1_NOISE_STD_M,
    bin_m: float = POOL_BIN_M,
) -> PooledPositions:
    """Reduce per-lap traces to one ordered mean line (measured samples only).

    Two things have to happen before a pooled cloud can be fitted, and both were *measured*
    on the synthetic session rather than assumed:

    1. **A common ordering.** The fit reads its curve parameter off the order of the points,
       and each lap restarts at the line. Merging on a per-lap phase ``s/L`` is not enough:
       FastF1 slices a lap at the timing line, so lap starts differ by up to one 10 Hz sample
       (~6 m at racing speed), and phase-merging scrambles the pooled order longitudinally by
       that much — the fit then chases the scramble (residual 1.0 m against a 0.3 m
       declaration; the 34 m reference apex collapsed to 2.6 m). So the densest lap is fitted
       alone — one lap is ordered by construction — and every pooled sample is projected onto
       that reference curve for a common arc length.
    2. **A stable sample count.** Simply stacking N laps of the *same* line multiplies the
       sample count without adding independent geometry, and the discrepancy principle
       responds by buying flexibility it cannot support: effective dof rose 25 → 41 → 92 from
       one to eight laps, and the resulting curvature ripple both invented corners (2 → 13
       detections on a two-corner circuit) and split the real ones (reference apex −27%). So
       samples are averaged into fixed ``bin_m`` arc-length bins: the station count is set by
       the track, not the pooling depth, and pooling shows up where it belongs — as the
       ``σ/√m`` reduction in :attr:`PooledPositions.effective_noise_std_m` (measured apex
       error −4.2% at one lap → −0.6% at four → −0.1% at eight, with no spurious corners).

    The chord-to-arc bias of averaging along a bin is negligible by construction: it shrinks a
    radius by ``(bin/2R)²/6``, i.e. 0.06% for a 4 m bin on a 34 m apex.

    Each trace is already ``Source``-filtered by
    :func:`outlap.trackcal.data.load_fastf1_positions`; the dropped counts it carries are
    summed here so the manifest can state the sample count either side of the filter.
    """
    if bin_m <= 0.0:
        raise FastF1TrackError(f"bin_m must be > 0, got {bin_m}")
    usable = [t for t in traces if t.x_m.size >= 2]
    if not usable:
        raise NoPositionSamplesError(
            "no measured position samples to pool — every lap was empty after the "
            "`Source == 'interpolated'` filter"
        )
    reference = max(usable, key=lambda t: int(t.x_m.size))
    ref_fit = fit_centerline(
        reference.x_m, reference.y_m, closed=True, noise_std_m=noise_std_m
    )
    dense = ref_fit.sample_uniform(_PROJECTION_STEP_M)
    x_all = np.concatenate([t.x_m for t in usable])
    y_all = np.concatenate([t.y_m for t in usable])
    s_all = _project_arclength(x_all, y_all, dense.x_m, dense.y_m, dense.s_m)

    length = ref_fit.length_m
    n_bins = max(int(round(length / bin_m)), MIN_BINS)
    idx = np.minimum((s_all / length * n_bins).astype(np.int64), n_bins - 1)
    counts = np.bincount(idx, minlength=n_bins)
    occupied = counts > 0
    per_bin = counts[occupied]
    x = np.bincount(idx, weights=x_all, minlength=n_bins)[occupied] / per_bin
    y = np.bincount(idx, weights=y_all, minlength=n_bins)[occupied] / per_bin
    s = np.asarray(
        (np.nonzero(occupied)[0] + 0.5) * (length / n_bins), dtype=np.float64
    )

    # How well each bin mean is actually determined. Declaring only sigma/sqrt(m) assumes every
    # sample in a bin is the same point plus noise, which holds for one driver and fails across
    # drivers: their racing lines genuinely differ, and that spread is signal about where the
    # cars drove, not measurement error. Under-declaring it makes the fit chase the scatter and
    # biases the apex radii — the very numbers the reference metrics commit to. So measure the
    # within-bin spread and take whichever is larger.
    # Cross-track only: samples also spread *along* the bin, and that extent is the binning
    # geometry, not disagreement about where the line runs. Measuring the full 2-D deviation
    # would read a 4 m bin as ~1 m of scatter and over-smooth every corner.
    tan_x = np.gradient(dense.x_m)
    tan_y = np.gradient(dense.y_m)
    tan_len = np.hypot(tan_x, tan_y)
    tan_len[tan_len == 0.0] = 1.0
    nx = np.interp(s_all, dense.s_m, -tan_y / tan_len)
    ny = np.interp(s_all, dense.s_m, tan_x / tan_len)
    n_len = np.hypot(nx, ny)
    n_len[n_len == 0.0] = 1.0
    mean_x = np.zeros(n_bins, dtype=np.float64)
    mean_y = np.zeros(n_bins, dtype=np.float64)
    mean_x[occupied] = x
    mean_y[occupied] = y
    lateral = ((x_all - mean_x[idx]) * nx + (y_all - mean_y[idx]) * ny) / n_len
    sum_sq = np.bincount(idx, weights=lateral**2, minlength=n_bins)[occupied]
    within_var = np.where(
        per_bin >= 2,
        sum_sq / np.maximum(per_bin - 1, 1),
        noise_std_m
        ** 2,  # a lone sample says nothing about spread; fall back to the declaration
    )
    measured = float(np.sqrt(np.mean(within_var / per_bin)))
    declared = noise_std_m * float(np.sqrt(np.mean(1.0 / per_bin)))

    drivers: list[str] = []
    for trace in usable:
        driver = trace.label.split(" ", 1)[0]
        if driver not in drivers:
            drivers.append(driver)
    return PooledPositions(
        x_m=x,
        y_m=y,
        s_m=s,
        samples_per_bin=per_bin.astype(np.int64),
        effective_noise_std_m=max(declared, measured),
        lap_labels=tuple(t.label for t in usable),
        drivers=tuple(drivers),
        n_used=int(x_all.size),
        n_dropped_interpolated=sum(int(t.n_interpolated) for t in usable),
        lap_length_m=length,
    )


# --- the import ----------------------------------------------------------------------------------


@dataclass(frozen=True)
class ImportResult:
    """What a driven-line import produced (paths are inside ``track_dir``)."""

    track_dir: Path
    session: SessionKey
    georeference: Georeference
    accuracy_class: str
    length_m: float
    n_stations: int
    n_laps: int
    n_samples: int
    n_samples_raw: int
    n_dropped_interpolated: int
    n_corners: int
    tightest_radius_m: float
    files: tuple[str, ...]


def _render_metrics(metrics: TrackMetrics) -> str:
    """Render the metrics CSV to text so it joins the atomic write batch.

    :func:`outlap.trackcal.data.write_metrics` owns the committed format and writes to a path,
    so it renders into a scratch file here rather than growing a second serializer — that
    keeps the gate's format single-sourced while every output of this importer stays under
    one ``--force`` guard and one atomic rename (KTD10).
    """
    with tempfile.TemporaryDirectory() as tmp:
        scratch = Path(tmp) / METRICS_FILE
        write_metrics(scratch, metrics)
        return scratch.read_text(encoding="utf-8")


def run_import(
    track_dir: Path,
    *,
    session: SessionKey,
    anchors_path: Path,
    name: str | None = None,
    drivers: Sequence[str] | None = None,
    max_laps_per_driver: int | None = None,
    cache_dir: Path | None = None,
    enu_origin: tuple[float, float] | None = None,
    max_residual_m: float = MAX_ANCHOR_RESIDUAL_M,
    ds_m: float = _DS_M,
    noise_std_m: float = FASTF1_NOISE_STD_M,
    bin_m: float = POOL_BIN_M,
    knot_spacing_m: float = _KNOT_SPACING_M,
    half_width_m: float = DRIVEN_LINE_HALF_WIDTH_M,
    corner_min_radius_m: float = _CORNER_MIN_RADIUS_M,
    force: bool = False,
    positions: Sequence[PositionTrace] | None = None,
) -> ImportResult:
    """Build a georeferenced driven-line track dir from a session's position telemetry.

    ``positions`` is the injectable loader seam (mirroring ``osm_track``'s ``lidar_sampler``):
    when it is ``None`` the traces are fetched live through FastF1 — the only network path in
    this module — otherwise the supplied traces are used as-is.
    """
    if half_width_m <= 0.0:
        raise FastF1TrackError(f"half_width_m must be > 0, got {half_width_m}")

    anchors = load_anchors(anchors_path)
    frame = None if enu_origin is None else EnuFrame(*enu_origin)
    # Gate the registration BEFORE touching anything else: a rejected transform writes nothing.
    georef = georeference(anchors, frame=frame, max_residual_m=max_residual_m)

    traces = (
        list(positions)
        if positions is not None
        else load_fastf1_positions(
            session.year,
            session.event,
            session.session,
            drivers=drivers,
            max_laps_per_driver=max_laps_per_driver,
            cache_dir=cache_dir,
        )
    )
    pooled = pool_positions(traces, noise_std_m=noise_std_m, bin_m=bin_m)
    print(
        f"pooled {pooled.n_used} measured samples from {len(pooled.lap_labels)} lap(s) "
        f"({pooled.n_dropped_interpolated} interpolated rows dropped of {pooled.n_raw}) "
        f"into {pooled.n_bins} bins — declared noise {noise_std_m:.2f} m per sample, "
        f"{pooled.effective_noise_std_m:.2f} m per bin",
        file=sys.stderr,
    )

    world_x, world_y = georef.transform.apply(pooled.x_m, pooled.y_m)
    fit = fit_centerline(
        world_x,
        world_y,
        closed=True,
        noise_std_m=pooled.effective_noise_std_m,
        knot_spacing_m=knot_spacing_m,
    )
    samples = fit.sample_uniform(ds_m)
    track_name = name or f"{session.event} {session.year} (driven line)"
    lat, lon = georef.frame.to_latlon(samples.x_m, samples.y_m)
    fc = FittedCenterline(
        name=track_name,
        closed=True,
        length_m=fit.length_m,
        s=samples.s_m,
        x=samples.x_m,
        y=samples.y_m,
        kappa=samples.kappa_per_m,
        heading=headings(samples.x_m, samples.y_m, closed=True),
        lat=lat,
        lon=lon,
        frame=georef.frame,
        residual_rms_m=fit.residual_rms_m,
        discrepancy_rms_m=fit.discrepancy_rms_m,
        smoothing_lambda=fit.smoothing_lambda,
        bias_corrected=fit.bias_corrected,
        effective_dof=fit.effective_dof,
    )
    print(
        f"  fitted {len(fc)} stations, {fc.length_m:.0f} m (noise declared "
        f"{noise_std_m:.2f} m, λ matched {fc.discrepancy_rms_m:.2f} m, final residual "
        f"{fc.residual_rms_m:.2f} m)",
        file=sys.stderr,
    )

    corners = detect_corners(fit, min_radius_m=corner_min_radius_m)
    tightest = min((c.apex_radius_m for c in corners), default=float("inf"))
    metrics_text = _render_metrics(
        metrics_from_corners(
            corners,
            label=track_dir.name,
            source_session=str(session),
            transform=georef.transform,
        )
    )

    zeros = np.zeros(len(fc), dtype=np.float64)
    widths = np.full(len(fc), float(half_width_m), dtype=np.float64)
    csv_text = render_centerline_csv(
        fc,
        zeros,
        zeros,
        widths,
        widths,
        elevation_note=None,
        source_note=f"FastF1 driven line, {session} (derived data only)",
    )

    georef_str = format_georef(georef.transform)
    notes = (
        f"driven line pooled from {len(pooled.lap_labels)} lap(s) of {session} — this is an "
        "audit/calibration artifact, NOT a corridor: widths are an honest constant "
        f"{half_width_m:g} m per side around the line the cars drove, so do not solve a "
        "raceline in it. z and banking are 0: position telemetry carries no usable elevation."
    )
    track_doc: dict[str, Any] = {
        "schema": "track/1.1",
        "name": track_name,
        "closed": True,
        "centerline": "centerline.csv",
        "meta": {
            "source": "fastf1-position",
            "accuracy_class": "C",
            "attribution": (
                f"FastF1 position telemetry, {session}; derived driven line only "
                "(no raw telemetry redistributed)"
            ),
            "width_source": "driven-line",
            "georef_transform": georef_str,
            "importer_version": IMPORTER_VERSION,
            "stages": list(_STAGES),
            "notes": notes,
        },
    }

    manifest: dict[str, Any] = {
        "importer": "outlap.importers.fastf1_track",
        "importer_version": IMPORTER_VERSION,
        "track": track_name,
        "stages": list(_STAGES),
        "session": {
            **session.as_manifest(),
            "drivers": list(pooled.drivers),
            "laps": list(pooled.lap_labels),
            "n_laps": len(pooled.lap_labels),
        },
        "parameters": {
            "ds_m": float(ds_m),
            "noise_std_m": float(noise_std_m),
            "pool_bin_m": float(bin_m),
            "effective_noise_std_m": float(pooled.effective_noise_std_m),
            "knot_spacing_m": float(knot_spacing_m),
            "half_width_m": float(half_width_m),
            "corner_min_radius_m": float(corner_min_radius_m),
        },
        "inputs": {
            "anchors": {
                "file": anchors_path.name,
                "sha256": sha256_file(anchors_path),
                "count": len(anchors),
            },
            "positions": {
                # HANDOFF §15: FastF1 telemetry may seed calibration artifacts but must not be
                # redistributed, so the pooled cloud is pinned by hash, never committed.
                "committed": False,
                "samples_raw": pooled.n_raw,
                "samples_used": pooled.n_used,
                "samples_dropped_interpolated": pooled.n_dropped_interpolated,
                "bins": pooled.n_bins,
                "mean_samples_per_bin": float(np.mean(pooled.samples_per_bin)),
                "sha256": pooled.sha256(),
            },
        },
        "georeference": georef.as_manifest(),
        "fit": {
            "length_m": float(fit.length_m),
            "n_stations": len(fc),
            "residual_rms_m": float(fit.residual_rms_m),
            "discrepancy_rms_m": float(fit.discrepancy_rms_m),
            "smoothing_lambda": float(fit.smoothing_lambda),
            "bias_corrected": bool(fit.bias_corrected),
            "effective_dof": float(fit.effective_dof),
            "lambda_capped": bool(fit.lambda_capped),
            "n_corners": len(corners),
            "tightest_radius_m": float(tightest),
        },
        "outputs": {"centerline_csv_sha256": sha256_text(csv_text)},
    }

    files: dict[str, str] = {
        "centerline.csv": csv_text,
        METRICS_FILE: metrics_text,
        "track.yaml": render_yaml(
            track_doc,
            "# Imported by outlap.importers.fastf1_track — derived driven line, "
            "not a corridor.\n",
        ),
        MANIFEST_FILE: render_yaml(
            manifest,
            "# Input manifest (KTD7): the session, anchors and numerics this import is a "
            "pure function of.\n",
        ),
    }
    write_track_dir(track_dir, files, force=force)
    verify_track_dir(track_dir)  # per-file atomicity: prove the dir is coherent
    return ImportResult(
        track_dir=track_dir,
        session=session,
        georeference=georef,
        accuracy_class="C",
        length_m=fit.length_m,
        n_stations=len(fc),
        n_laps=len(pooled.lap_labels),
        n_samples=pooled.n_used,
        n_samples_raw=pooled.n_raw,
        n_dropped_interpolated=pooled.n_dropped_interpolated,
        n_corners=len(corners),
        tightest_radius_m=tightest,
        files=tuple(files),
    )


# --- CLI -------------------------------------------------------------------------------------------


def _parse_origin(text: str) -> tuple[float, float]:
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("--enu-origin takes `lat,lon` in degrees")
    try:
        return float(parts[0]), float(parts[1])
    except ValueError as err:
        raise argparse.ArgumentTypeError(f"bad --enu-origin {text!r}") from err


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        prog="outlap.importers.fastf1_track", description=__doc__
    )
    parser.add_argument("--out", type=Path, required=True, help="the track directory")
    parser.add_argument("--year", type=int, required=True)
    parser.add_argument("--event", required=True, help="event name or round number")
    parser.add_argument("--session", default="R", help="session code (R, Q, FP2, …)")
    parser.add_argument(
        "--anchors",
        type=Path,
        required=True,
        help=f"the committed `{ANCHORS_FORMAT}` correspondence CSV",
    )
    parser.add_argument("--name", help="track name (default: derived from the session)")
    parser.add_argument("--drivers", default="", help="comma-separated driver codes")
    parser.add_argument("--max-laps-per-driver", type=int, default=None)
    parser.add_argument(
        "--cache", type=Path, help="FastF1 cache directory (never committed)"
    )
    parser.add_argument(
        "--enu-origin",
        type=_parse_origin,
        help="real-world ENU origin `lat,lon` (default: the anchor centroid)",
    )
    parser.add_argument(
        "--max-residual",
        type=float,
        default=MAX_ANCHOR_RESIDUAL_M,
        help="anchor-residual ceiling the georeference must clear, m",
    )
    parser.add_argument("--ds", type=float, default=_DS_M, help="station spacing, m")
    parser.add_argument(
        "--noise-std",
        type=float,
        default=FASTF1_NOISE_STD_M,
        help="declared per-axis position noise for the fit, m",
    )
    parser.add_argument(
        "--bin",
        type=float,
        default=POOL_BIN_M,
        dest="bin_m",
        help="arc-length width of a pooling bin, m",
    )
    parser.add_argument(
        "--half-width",
        type=float,
        default=DRIVEN_LINE_HALF_WIDTH_M,
        help="honest driven-line half-width, m (this artifact is not a corridor)",
    )
    parser.add_argument(
        "--force", action="store_true", help="overwrite existing generated files"
    )
    args = parser.parse_args(argv)

    result = run_import(
        args.out,
        session=SessionKey(year=args.year, event=args.event, session=args.session),
        anchors_path=args.anchors,
        name=args.name,
        drivers=[d.strip() for d in args.drivers.split(",") if d.strip()] or None,
        max_laps_per_driver=args.max_laps_per_driver,
        cache_dir=args.cache,
        enu_origin=args.enu_origin,
        max_residual_m=args.max_residual,
        ds_m=args.ds,
        noise_std_m=args.noise_std,
        bin_m=args.bin_m,
        half_width_m=args.half_width,
        force=args.force,
    )
    print(
        f"wrote {', '.join(result.files)} in {result.track_dir} "
        f"({result.n_stations} points, {result.length_m:.0f} m, class "
        f"{result.accuracy_class}, {result.n_corners} corners, tightest "
        f"{result.tightest_radius_m:.1f} m, georef residual "
        f"{result.georeference.residual_rms_m:.2f} m RMS)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
