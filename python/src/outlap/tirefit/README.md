<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# outlap.tirefit: the MF6.1 fitting pipeline

This package does three things.

It reads test data and converts it to SI units and the ISO 8855 convention. It accepts TTC `.mat`
files in v7 and v7.3, `.dat` files, and `.csv` files.

It holds a vectorized MF6.1 forward model in numpy. That model is a clean-room mirror of the Rust
kernels. It is validated against the same golden CSVs, under the same tolerance rule.

It runs a least-squares fit in stages: nominals, then pure Fx0, then pure Fy0, then combined, then
Mz, then Mx and My. Documented tables give the initial values and the bounds.

```
python -m outlap.tirefit fit   run1.mat run2.mat --unloaded-radius 0.26 -o car.tyr.yaml --report-dir report/
python -m outlap.tirefit synth car.tyr.yaml -o synth.csv --seed 0
```

The fit stages need scipy. Install the extra with `uv sync --extra tire-fit`.

## Redistribution policy: read this

**Parsers are permitted. Redistribution of TTC data, or of a parameter set derived from TTC data,
is NOT.**

Data from the FSAE Tire Test Consortium is locked to members, and you may not redistribute it. This
package exists so that a member can fit locally. Follow three rules:

- Keep raw TTC files in a local `ttc-data/` directory. The root `.gitignore` covers it.
- Never commit a TTC file, an excerpt of one, or **a parameter set fitted from TTC data**. This
  applies to this repository and to any public artifact.
- A fit report (`report.json` or `report.md`) embeds no input data. But a report fitted from TTC
  data still describes a parameter set derived from TTC data. Treat it the same way.

Only two kinds of data ship with outlap: synthetic data from `synth`, and parameter sets that cite
the literature. See `data/tires/`.
