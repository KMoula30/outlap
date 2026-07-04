<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# outlap.tirefit — MF6.1 fitting pipeline

Test-data ingestion (TTC `.mat` v7/v7.3, `.dat`, `.csv` → SI/ISO-8855), a vectorized numpy
MF6.1 forward model (clean-room mirror of the Rust kernels, validated against the same golden
CSVs and tolerance rule), and a staged least-squares fit
(nominals → pure Fx0 → pure Fy0 → combined → Mz → Mx/My) with documented init/bounds tables.

```
python -m outlap.tirefit fit   run1.mat run2.mat --unloaded-radius 0.26 -o car.tyr.yaml --report-dir report/
python -m outlap.tirefit synth car.tyr.yaml -o synth.csv --seed 0
```

The fit stages need scipy — install the extra: `uv sync --extra tire-fit`.

## Redistribution policy (read this)

**Parsers yes — redistribution of TTC data or TTC-derived parameter sets, NO.**

FSAE Tire Test Consortium data is membership-locked and non-redistributable. This package
exists so members can fit locally:

- keep raw TTC files in a local `ttc-data/` directory — it is gitignored at the repo root;
- never commit TTC files, excerpts, or **parameter sets fitted from TTC data** to this
  repository or any public artifact;
- fit reports (`report.json`/`report.md`) do not embed input data, but a report fitted from
  TTC data still describes a TTC-derived parameter set — treat it the same way.

Synthetic data (`synth`) and literature-cited parameter sets (see `data/tires/`) are the only
things that ship with outlap.
