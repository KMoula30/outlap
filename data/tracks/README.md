<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Reference tracks

Imported 3D circuits (`track.yaml` + `centerline.csv`, §9.3), built from **public data only** by
`outlap.importers.osm_track`. These are real-world outputs — they live here, **never** in CI golden
fixtures (which stay synthetic, per the working agreement).

## Attribution

- **Centerline geometry**: © OpenStreetMap contributors, licensed under the
  [Open Database License (ODbL)](https://www.openstreetmap.org/copyright). Redistribution here is
  under ODbL; the derived database keeps the same terms.
- **Elevation**: sampled from open DEMs (EU-DEM 25 m / SRTM) via
  [opentopodata.org](https://www.opentopodata.org). See each `track.yaml` `meta.dem` for the exact
  dataset.

Per-track provenance and accuracy class are recorded in each `track.yaml` `meta` block.

## Regenerating

```sh
cd python
uv sync --extra track-import
uv run python -m outlap.importers.osm_track --preset catalunya --out ../data/tracks/catalunya
uv run python examples/plot_track.py ../data/tracks/catalunya --out examples/output
```

Presets ship for Catalunya, Spa, and Silverstone (Decision #23). Banking is not resolved from
coarse public DEMs — add sparse `banking_keypoints` to `track.yaml` to refine it (accuracy class
moves from B toward A).
