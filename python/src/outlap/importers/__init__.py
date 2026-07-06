# SPDX-License-Identifier: AGPL-3.0-only
"""Track and powertrain importers.

The OSM+DEM track importer ([`osm_track`][outlap.importers.osm_track]) builds the first open 3D
racetrack format (§9.3) from public data. The TUMFTM importer
([`tumftm_track`][outlap.importers.tumftm_track]) converts the LGPL-3.0 `racetrack-database` (flat
2-D centre lines + measured widths) into the same format. Both are one-time local vendoring tools,
not part of the core solver package, and are never exercised in CI (synthetic fixtures cover the
parsing/geometry contract).
"""
