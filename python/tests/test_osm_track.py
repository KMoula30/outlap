# SPDX-License-Identifier: AGPL-3.0-only
"""Offline unit tests for the OSM circuit assembler (`outlap.importers.osm_track`).

The network fetch (Overpass + DEM) is never run in CI; these exercise the pure graph assembly
(`_assemble_circuit`) that turns fragmented `highway=raceway` ways into the main closed lap. The
load-bearing case is the **theta junction** (the pit-bypass chord) that a naive longest-way or
greedy-chain heuristic gets wrong — Spa's real topology.
"""

from __future__ import annotations

from typing import Any

from outlap.importers import osm_track


def _node(nid: int, lat: float, lon: float) -> dict[str, Any]:
    return {"type": "node", "id": nid, "lat": lat, "lon": lon}


def _way(wid: int, nodes: list[int], name: str = "") -> dict[str, Any]:
    w: dict[str, Any] = {"type": "way", "id": wid, "nodes": nodes}
    if name:
        w["tags"] = {"name": name}
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


def test_assemble_circuit_resolves_theta_to_the_long_loop() -> None:
    osm = _theta_osm()
    loop = osm_track._assemble_circuit(osm)  # pyright: ignore[reportPrivateUsage]
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
    assert osm_track._assemble_circuit(osm) == [  # pyright: ignore[reportPrivateUsage]
        0,
        1,
        2,
    ]
