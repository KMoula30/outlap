# SPDX-License-Identifier: AGPL-3.0-only
"""Track and powertrain importers.

The OSM+DEM track importer ([`osm_track`][outlap.importers.osm_track]) builds the first open 3D
racetrack format (§9.3) from public data. It is network-facing tooling, not part of the core solver
package, and is never exercised in CI (synthetic fixtures cover the parsing/geometry contract).
"""
