<!-- SPDX-License-Identifier: AGPL-3.0-only -->

# Residual review findings — MT track-fidelity pipeline

Findings from the reviews of PR #72 (units U1–U6) and PR #73 (hardening) that were **not**
applied, recorded so they survive the scratch directory they were produced in. Reviewed at
`65421dc..main`; eight lenses ran (coherence, feasibility, scope, security, project-standards,
testing, maintainability, api-contract, reliability, adversarial, correctness — the correctness
lens ran late, after #72 had merged).

Everything below was judged real and left undone deliberately. Nothing here is a known-wrong
claim; they are ranked roughly by what would bite first.

## Worth doing before or during U7

**Start/finish seam is untested for corner detection.** The wrap-around merge, the "wrapped
window" contract where start > end, and the wrapped apex-speed selection have no coverage. The
stadium fixture carries a `roll=` parameter built for exactly this and no test passes it a
non-zero value. A corner straddling the seam is common on real circuits (Monaco, Spa's La
Source is close), and these numbers feed the U8 track-quality gate.
*Source: testing lens, P1.*

**Empty pooling bins are silently dropped.** In `fastf1_track.pool_positions`, a stretch with no
surviving samples — every row flagged interpolated, a safety-car window, drivers who pitted —
simply vanishes from the station list, and the periodic spline is fitted across the gap, cutting
the corner. The emitted driven line then contains geometry no car drove. Suggested: raise a typed
error naming the empty arc-length ranges, and record `min_samples_per_bin` in the manifest
alongside the mean.
*Source: adversarial lens, P2.*

**The Overpass mirror that served a snapshot is not recorded.** The manifest pins the fetched
bytes by hash but not who served them, while the DEM branch fourteen lines below already records
its dataset. `_overpass` walks a mirror list ending at a third-party host, swallowing TLS errors
on the way.
*Source: security lens, P2.*

**Six `OsmTrackError` branches in the stage functions are untested**, though the sibling
`run_import`-level branches are each tested individually a few lines above.
*Source: testing lens, P2.*

## Structural, no urgency

**`osm_track.py` is 1,187 lines and owns the shared emitted-format surface**, so
`fastf1_track.py` imports its hashing, YAML/CSV rendering, headings and atomic-write helpers from
a module otherwise full of OSM-specific network and CLI code. Extracting those into one small
shared module would leave exactly one shared surface rather than an importer depending on a
sibling importer.
*Source: maintainability lens, P1 (the line count alone is not a project-standards violation —
the layering is the real point).*

**`CenterlineFit` and `ElevationProfile` duplicate the same six-field fit-diagnostics shape**,
including the docstring. A shared frozen dataclass in the smoother kernel would carry it once.
*Source: maintainability lens, P2, advisory.*

**The width gap-rescue reuses the blend window.** A single control point suppresses the
unresolved-station error for every station within 25 m either side — about sixteen at 3 m
spacing — and each takes the control point's offset at full strength rather than scaled by the
smoothstep weight. A tighter dedicated `rescue_window_m` plus a `stations_rescued` provenance
count would keep traced and extrapolated stations distinguishable.
*Source: adversarial lens, P2, confidence 50.*

## Documentation and fixtures

**`qss.sim.yaml` is headed "a QSS lap with the default numerics" and pins
`path_curvature_smooth_m: 25.0`, which is not the default** — the default is the absent field,
resolving per-consumer. The two agree only at `ds = 2.0 m`; at `ds = 3.0 m` the omitted default
gives a 36 m window and the explicit 25.0 rounds to 24 m. Anyone copying the fixture as a
starting point inherits a step-dependent surprise.
*Source: adversarial lens, P3, confidence 100.*

**"Geometry is year-stable" is overstated.** True for the seasons in play, but Catalunya removed
its final chicane in 2023, so a fallback season chosen under the plan's stop-condition could
radius-gate a track against a superseded layout. A one-line qualifier — the manifest season must
postdate the venue's last layout change — makes the rule safe for future tracks.
*Source: adversarial lens, P3, confidence 50.*

## Accepted limitations (not defects)

- **Byte-identical re-import is proven only within one environment.** The emitted CSV comes from
  dense LAPACK solves at fixed precision; a ~3.1e-13 coefficient shift from LAPACK blocking is
  already documented in `geometry.py`. The determinism test runs both imports in one process on
  one BLAS, so it proves the code carries no non-deterministic state but cannot prove byte
  identity across a numpy/BLAS/thread-count change. Re-verify committed tracks whenever the numpy
  pin moves.
- **The bias-correction veto's yardstick is conservative.** It models the step as `A₁·ε` while the
  step actually taken is `A₁(I−S)z`; since `(I−S)` is a contraction the yardstick overstates the
  noise-driven step, so the guard vetoes slightly more often than its docstring claims. Safe
  direction, but not quite the quantity named.
- **The λ-floor branch is unflagged.** `lambda_capped` surfaces the over-smoothed saturation but
  not the under-smoothed one, so `discrepancy_rms_m != noise_std_m` can occur without a flag.
  Numerically benign; the reporting contract is asymmetric.
- **An oversized curvature-smoothing window produces no smoothing rather than maximum
  smoothing** — the radius is capped at the station count and `smooth()` no-ops below
  `2·radius+1` stations. Recorded honestly as 0.0, but non-monotonic across that cliff.
- **The 27 shipped track files keep pre-`track/1.2` banking semantics.** Their zeros are
  absences, not measurements, but they are not rewritten as a side effect of the convention
  change; the three that matter are re-imported in U7.
