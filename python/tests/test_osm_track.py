# SPDX-License-Identifier: AGPL-3.0-only
"""Offline unit tests for the staged OSM track importer (`outlap.importers.osm_track`).

The network paths (Overpass + opentopodata, only reachable under ``--refresh-snapshot``) are
never run in CI; everything here drives the importer from small synthetic snapshot fixtures
built in-test (KTD7: the import is a pure function of committed inputs). Covered:

* the pure graph assembly (`_assemble_circuit`) — the load-bearing **theta junction** case
  (the pit-bypass chord) plus the `disused:highway=raceway` filter;
* snapshot determinism (two runs → byte-identical ``centerline.csv``);
* the staged, honest degradation contract (R1: no silent width default; accuracy class from
  what ran; meta + manifest provenance);
* atomic writes (interrupted build leaves the target untouched; ``--force`` over outputs);
* curvature quality (a noisy-circle snapshot fits the true radius within 2% — proof the
  trackcal penalised fit is wired, not a linear ``np.interp`` resample).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

from outlap.importers import osm_track
from outlap.importers.osm_track import (
    DEM_SAMPLES_FILE,
    IMPORTER_VERSION,
    MANIFEST_FILE,
    SNAPSHOT_FILE,
    MissingElevationError,
    MissingSnapshotError,
    MissingWidthSourceError,
    OsmTrackError,
    OutputExistsError,
    TornBuildError,
    fit_snapshot_centerline,
    run_import,
    verify_track_dir,
)

# --- fixture builders ----------------------------------------------------------------------------


def _node(nid: int, lat: float, lon: float) -> dict[str, Any]:
    return {"type": "node", "id": nid, "lat": lat, "lon": lon}


def _way(
    wid: int, nodes: list[int], name: str = "", tags: dict[str, str] | None = None
) -> dict[str, Any]:
    w: dict[str, Any] = {"type": "way", "id": wid, "nodes": nodes}
    all_tags = dict(tags or {})
    if name:
        all_tags["name"] = name
    if all_tags:
        w["tags"] = all_tags
    return w


def _theta_osm() -> dict[str, Any]:
    """A theta graph: junctions A(0) and B(1) joined by a long loop (2..) and a short bypass chord.

    Node layout (lon, lat degrees, tiny so haversine ~ linear):
      A=0 at (0,0), B=1 at (0,0.010). A long path A→…→B going out to lon=0.02 (the racing loop side),
      a medium return path A→…→B near lon=0 (the other loop side), and a short direct A→B chord (the
      bypass). Plus a pit-lane spur hanging off A that must be pruned.
    """
    nodes = [
        _node(0, 0.000, 0.000),  # A
        _node(1, 0.010, 0.000),  # B
        # long path A -> 2 -> 3 -> B (bulges out to lon 0.02)
        _node(2, 0.003, 0.020),
        _node(3, 0.007, 0.020),
        # medium path A -> 4 -> 5 -> B (bulges out to lon -0.01)
        _node(4, 0.003, -0.010),
        _node(5, 0.007, -0.010),
        # short bypass chord A -> 6 -> B (near lon 0)
        _node(6, 0.005, 0.001),
        # pit spur off A (dead end) -> must be pruned
        _node(7, -0.002, 0.001),
        _node(8, -0.004, 0.001),
    ]
    ways = [
        _way(100, [0, 2, 3, 1], "Kemmel"),
        _way(101, [0, 4, 5, 1], "Blanchimont"),
        _way(102, [0, 6, 1], ""),  # bypass chord (unnamed, short)
        _way(103, [1, 7, 8], "Pit Lane"),  # excluded by name AND a spur
    ]
    return {"elements": nodes + ways}


def _circle_snapshot(
    radius_m: float = 34.0,
    spacing_m: float = 5.0,
    noise_m: float = 0.0,
    seed: int = 42,
    first_id: int = 1,
    way_id: int = 1000,
    name: str = "Ring",
    tags: dict[str, str] | None = None,
) -> dict[str, Any]:
    """A closed circular `highway=raceway` snapshot around (0°, 0°) with optional position noise."""
    n = int(round(2.0 * math.pi * radius_m / spacing_m))
    ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    x = radius_m * np.cos(ang)
    y = radius_m * np.sin(ang)
    if noise_m > 0.0:
        rng = np.random.default_rng(seed)
        x = x + rng.normal(0.0, noise_m, n)
        y = y + rng.normal(0.0, noise_m, n)
    # Tiny angles at lat0 ~ 0: the equirectangular ENU projection is metric-exact here.
    earth = 6_371_000.0
    lat = np.degrees(y / earth)
    lon = np.degrees(x / earth)
    ids = list(range(first_id, first_id + n))
    nodes = [_node(ids[i], float(lat[i]), float(lon[i])) for i in range(n)]
    ring = _way(way_id, [*ids, ids[0]], name, tags)
    return {"elements": [*nodes, ring]}


def _write_snapshot(track_dir: Path, snapshot: dict[str, Any]) -> None:
    track_dir.mkdir(parents=True, exist_ok=True)
    (track_dir / SNAPSHOT_FILE).write_text(json.dumps(snapshot), encoding="utf-8")


def _ring_orthophoto_npz(
    path: Path, radius_m: float, half_width_m: float, extent_m: float, px_m: float = 0.5
) -> None:
    """Draw a dark asphalt annulus of the given half-width on bright grass and save an .npz."""
    n = int(round(2.0 * extent_m / px_m))
    cols = np.arange(n) * px_m + px_m / 2.0 - extent_m  # pixel-centre world x
    rows = extent_m - (np.arange(n) * px_m + px_m / 2.0)  # world y (north-up)
    xg, yg = np.meshgrid(cols, rows)
    dist = np.hypot(xg, yg)
    image = np.where(np.abs(dist - radius_m) <= half_width_m, 0.2, 0.9)
    transform = np.array([px_m, 0.0, -extent_m, 0.0, -px_m, extent_m])
    np.savez(path, image=image, transform=transform)


# --- circuit assembly (the pure graph core) ------------------------------------------------------


def test_assemble_circuit_resolves_theta_to_the_long_loop() -> None:
    osm = _theta_osm()
    loop, method = osm_track._assemble_circuit(osm)  # pyright: ignore[reportPrivateUsage]
    assert method == osm_track.ASSEMBLY_THETA
    # The bypass chord node (6) and the pit spur nodes (7, 8) must NOT be in the main lap.
    assert 6 not in loop, "the short bypass chord was taken instead of the racing loop"
    assert 7 not in loop and 8 not in loop, "the pit-lane spur was not pruned"
    # Both long/medium loop sides ARE in it (2,3 and 4,5), and both junctions.
    for nid in (0, 1, 2, 3, 4, 5):
        assert nid in loop, f"node {nid} missing from the assembled lap"
    # It is a ring: first node repeated at the end so the closing edge enters the arc length.
    assert loop[0] == loop[-1]


def test_assemble_circuit_falls_back_to_longest_way_without_a_cycle() -> None:
    # A single open way (no cycle) → the 2-core is empty → longest-way fallback returns that way.
    osm = {
        "elements": [
            _node(0, 0.0, 0.0),
            _node(1, 0.0, 0.01),
            _node(2, 0.0, 0.02),
            _way(200, [0, 1, 2], "Main Straight"),
        ]
    }
    loop, method = osm_track._assemble_circuit(osm)  # pyright: ignore[reportPrivateUsage]
    assert loop == [0, 1, 2]
    # The fallback must announce itself — it caps the accuracy class downstream.
    assert method == osm_track.ASSEMBLY_LONGEST_WAY


def test_assemble_circuit_drops_disused_raceway() -> None:
    # A longer DISUSED ring next to the live one must not enter the lap (lifecycle tagging).
    live = _circle_snapshot(radius_m=34.0, first_id=1, way_id=1000, name="Live")
    disused = _circle_snapshot(
        radius_m=80.0,
        first_id=5000,
        way_id=2000,
        name="Old Layout",
        tags={"disused:highway": "raceway"},
    )
    osm = {"elements": live["elements"] + disused["elements"]}
    loop, _ = osm_track._assemble_circuit(osm)  # pyright: ignore[reportPrivateUsage]
    assert all(nid < 5000 for nid in loop), "a disused raceway way entered the lap"


# --- curvature quality (KTD2: the trackcal fit is wired, not np.interp) --------------------------


def test_noisy_circle_snapshot_radius_within_two_percent() -> None:
    snapshot = _circle_snapshot(radius_m=34.0, spacing_m=5.0, noise_m=0.3, seed=42)
    fc = fit_snapshot_centerline(snapshot, "Ring", noise_std_m=0.3)
    assert fc.closed
    assert fc.length_m == pytest.approx(2.0 * math.pi * 34.0, rel=2e-2)
    radius = 1.0 / float(np.median(np.abs(fc.kappa)))
    assert radius == pytest.approx(34.0, rel=2e-2)


# --- the staged import: pinning, honesty, atomicity ----------------------------------------------


def test_import_from_snapshot_is_deterministic(tmp_path: Path) -> None:
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot(noise_m=0.3))
    run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False)
    first = (track_dir / "centerline.csv").read_bytes()
    run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False, force=True)
    second = (track_dir / "centerline.csv").read_bytes()
    assert first == second, (
        "same committed inputs must reproduce centerline.csv byte-identically"
    )


def test_base_only_import_emits_honest_meta_and_manifest(tmp_path: Path) -> None:
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())
    result = run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False)
    assert result.accuracy_class == "C"
    assert result.stages == ("base",)

    doc = yaml.safe_load((track_dir / "track.yaml").read_text(encoding="utf-8"))
    assert doc["schema"] == "track/1.2"
    assert doc["closed"] is True
    meta = doc["meta"]
    assert meta["accuracy_class"] == "C"
    assert meta["width_source"] == "declared"
    assert meta["importer_version"] == IMPORTER_VERSION
    assert meta["stages"] == ["base"]
    assert "declared" in meta["notes"].lower() or "DECLARED" in meta["notes"]

    manifest = yaml.safe_load((track_dir / MANIFEST_FILE).read_text(encoding="utf-8"))
    assert manifest["importer_version"] == IMPORTER_VERSION
    assert manifest["stages"] == ["base"]
    assert manifest["inputs"]["half_width_m"] == 6.0
    snap = manifest["inputs"]["osm_snapshot"]
    assert snap["file"] == SNAPSHOT_FILE
    assert snap["sha256"] == osm_track.sha256_file(track_dir / SNAPSHOT_FILE)
    out_sha = manifest["outputs"]["centerline_csv_sha256"]
    assert out_sha == osm_track.sha256_file(track_dir / "centerline.csv")


def test_no_width_source_is_a_typed_error(tmp_path: Path) -> None:
    # R1: the old silent 6.0 m default is GONE — base import without a width source errors.
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())
    with pytest.raises(MissingWidthSourceError, match="never silently defaulted"):
        run_import(track_dir, name="Ring", elevation=False)
    assert not (track_dir / "track.yaml").exists(), "no output may exist on error"


def test_widths_stage_and_half_width_conflict(tmp_path: Path) -> None:
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())
    with pytest.raises(OsmTrackError, match="ONE width source"):
        run_import(
            track_dir,
            name="Ring",
            stages=["widths"],
            half_width_m=6.0,
            elevation=False,
        )


def test_missing_snapshot_is_a_typed_error(tmp_path: Path) -> None:
    with pytest.raises(MissingSnapshotError, match="refresh-snapshot"):
        run_import(
            tmp_path / "empty", name="Nowhere", half_width_m=6.0, elevation=False
        )


def test_missing_dem_samples_is_a_typed_error(tmp_path: Path) -> None:
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())
    with pytest.raises(MissingElevationError, match=DEM_SAMPLES_FILE):
        run_import(track_dir, name="Ring", half_width_m=6.0)  # elevation defaults on


def test_telemetry_audit_stage_is_not_implemented(tmp_path: Path) -> None:
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())
    with pytest.raises(NotImplementedError, match="U8"):
        run_import(
            track_dir,
            name="Ring",
            stages=["telemetry-audit"],
            half_width_m=6.0,
            elevation=False,
        )


def test_interrupted_build_leaves_target_untouched(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())

    def boom(src: str, dst: str) -> None:
        raise OSError("simulated crash between temp write and rename")

    monkeypatch.setattr(osm_track.os, "replace", boom)
    with pytest.raises(OSError, match="simulated crash"):
        run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False)
    # The target dir still holds ONLY the committed snapshot; the temp build dir is gone.
    assert sorted(p.name for p in track_dir.iterdir()) == [SNAPSHOT_FILE]
    assert not [p for p in tmp_path.iterdir() if p.name.startswith(".ring.build-")]


def test_force_required_over_existing_outputs(tmp_path: Path) -> None:
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())
    run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False)
    with pytest.raises(OutputExistsError, match="--force"):
        run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False)
    run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False, force=True)


# --- enrichment stages ---------------------------------------------------------------------------


def test_widths_stage_traces_and_records_provenance(tmp_path: Path) -> None:
    radius, half_width = 60.0, 5.0
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot(radius_m=radius, spacing_m=5.0))
    image_path = track_dir / "orthophoto.npz"
    _ring_orthophoto_npz(image_path, radius, half_width, extent_m=radius + 20.0)
    cp_path = track_dir / "width_control_points.csv"
    cp_path.write_text("s_m,side,offset_m\n0.0,left,5.0\n", encoding="utf-8")

    result = run_import(
        track_dir,
        name="Ring",
        stages=["widths"],
        elevation=False,
        width_image=image_path,
        width_control_points=cp_path,
    )
    assert result.stages == ("base", "widths")
    assert result.accuracy_class == "C"  # widths alone do not resolve elevation

    rows = np.genfromtxt(
        track_dir / "centerline.csv", delimiter=",", names=True, skip_header=1
    )
    for col in ("width_left_m", "width_right_m"):
        assert float(np.mean(np.abs(rows[col] - half_width))) < 0.3, col

    meta = yaml.safe_load((track_dir / "track.yaml").read_text(encoding="utf-8"))[
        "meta"
    ]
    assert meta["width_source"] == "orthophoto"
    assert meta["width_control_points_sha"] == osm_track.sha256_file(cp_path)
    manifest = yaml.safe_load((track_dir / MANIFEST_FILE).read_text(encoding="utf-8"))
    widths_in = manifest["inputs"]["widths"]
    assert widths_in["image"]["file"] == "orthophoto.npz"
    assert widths_in["control_points"]["sha256"] == meta["width_control_points_sha"]


def test_lidar_stage_with_injected_sampler_reaches_class_a(tmp_path: Path) -> None:
    """widths + lidar (analytic tilted-plane sampler) → real z + banking and accuracy A."""
    radius, half_width = 60.0, 5.0
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot(radius_m=radius, spacing_m=5.0))
    image_path = track_dir / "orthophoto.npz"
    _ring_orthophoto_npz(image_path, radius, half_width, extent_m=radius + 20.0)

    def plane(x: Any, y: Any) -> Any:
        return 10.0 + 0.02 * np.asarray(y, dtype=np.float64)

    result = run_import(
        track_dir,
        name="Ring",
        stages=["widths", "lidar"],
        width_image=image_path,
        lidar_source="icgc_catalunya",
        lidar_tiles=["tile_a", "tile_b"],
        lidar_sampler=plane,
    )
    assert result.accuracy_class == "A"
    assert result.stages == ("base", "widths", "lidar")

    rows = np.genfromtxt(
        track_dir / "centerline.csv", delimiter=",", names=True, skip_header=1
    )
    assert float(np.ptp(rows["z_m"])) > 1.0, (
        "LiDAR z must not be flat on a tilted plane"
    )
    # A plane tilted in y banks the ring where the left normal has a y component.
    assert float(np.max(np.abs(rows["banking_deg"]))) > 0.5

    meta = yaml.safe_load((track_dir / "track.yaml").read_text(encoding="utf-8"))[
        "meta"
    ]
    assert meta["lidar_dataset"] == "icgc-lidar-territorial-dtm v3.1"
    assert meta["lidar_tiles"] == ["tile_a", "tile_b"]
    manifest = yaml.safe_load((track_dir / MANIFEST_FILE).read_text(encoding="utf-8"))
    assert manifest["inputs"]["lidar"]["tiles"] == ["tile_a", "tile_b"]
    assert manifest["inputs"]["lidar"]["banking"]["method"] == "lidar-cross-section"


# --- hardening: degraded results must never masquerade as measurements --------------------------


def test_longest_way_fallback_caps_the_accuracy_class() -> None:
    """Enriching a fragment very precisely is still the wrong shape — it cannot earn an A."""
    assert (
        osm_track.derive_accuracy_class(
            ["widths", "lidar"],
            has_elevation=True,
            assembly=osm_track.ASSEMBLY_LONGEST_WAY,
        )
        == "C"
    )
    # The same stages on a properly assembled lap do earn it.
    assert (
        osm_track.derive_accuracy_class(
            ["widths", "lidar"], has_elevation=True, assembly=osm_track.ASSEMBLY_CYCLE
        )
        == "A"
    )


def test_import_records_the_assembly_method_and_fit(tmp_path: Path) -> None:
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())
    run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False)

    meta = yaml.safe_load((track_dir / "track.yaml").read_text(encoding="utf-8"))[
        "meta"
    ]
    assert meta["assembly"] == osm_track.ASSEMBLY_CYCLE
    fit = yaml.safe_load((track_dir / MANIFEST_FILE).read_text(encoding="utf-8"))["fit"]
    # Which smoother ran is not derivable from the declared settings — the bias-correction
    # step is a data-dependent branch — so the manifest has to carry it.
    for key in (
        "residual_rms_m",
        "discrepancy_rms_m",
        "smoothing_lambda",
        "bias_corrected",
        "effective_dof",
        "assembly",
    ):
        assert key in fit, f"manifest fit record is missing {key}"


def test_torn_build_is_detected_not_silent(tmp_path: Path) -> None:
    """New geometry beside a stale manifest must be loud, not a silently wrong track."""
    track_dir = tmp_path / "ring"
    _write_snapshot(track_dir, _circle_snapshot())
    run_import(track_dir, name="Ring", half_width_m=6.0, elevation=False)
    verify_track_dir(track_dir)  # a complete build is coherent

    # Simulate an interruption after centerline.csv landed but before the manifest did.
    centerline = track_dir / "centerline.csv"
    centerline.write_text(
        centerline.read_text(encoding="utf-8") + "# torn\n", encoding="utf-8"
    )
    with pytest.raises(TornBuildError, match="interrupted"):
        verify_track_dir(track_dir)


def test_null_dem_elevation_is_an_error_not_a_fabricated_zero(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A DEM with no coverage must not commit a fake sea-level sample."""

    class _Resp:
        status_code = 200

        def raise_for_status(self) -> None:
            return None

        def json(self) -> dict[str, Any]:
            return {
                "status": "OK",
                "results": [
                    {"elevation": 12.5, "location": {"lat": 0.0, "lng": 0.0}},
                    {"elevation": None, "location": {"lat": 0.1, "lng": 0.1}},
                ],
            }

    fake = type("_R", (), {"post": staticmethod(lambda *a, **k: _Resp())})
    monkeypatch.setitem(__import__("sys").modules, "requests", fake)
    with pytest.raises(MissingElevationError, match="no elevation"):
        osm_track._dem_batch(  # pyright: ignore[reportPrivateUsage]
            "eudem25m", [(0.0, 0.0), (0.1, 0.1)]
        )


def test_two_disjoint_cycles_take_the_longest_not_an_arbitrary_one() -> None:
    """A second surviving ring must not be able to become the lap by dict order.

    The accuracy class trusts the `cycle` provenance label, so walking whichever ring the
    iteration happened to reach would let a service loop wear the circuit's grade.
    """
    circuit = _circle_snapshot(radius_m=80.0, first_id=1, way_id=1000, name="Circuit")
    # A smaller closed ring the name filter does not catch (no pit/kart/service in the name).
    stray = _circle_snapshot(
        radius_m=15.0, first_id=9000, way_id=2000, name="Perimeter Road"
    )
    osm = {"elements": circuit["elements"] + stray["elements"]}
    loop, method = osm_track._assemble_circuit(osm)  # pyright: ignore[reportPrivateUsage]
    assert method == osm_track.ASSEMBLY_CYCLE
    assert all(nid < 9000 for nid in loop), "the shorter stray ring became the lap"
