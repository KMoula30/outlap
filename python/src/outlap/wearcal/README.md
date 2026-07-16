<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# `outlap.wearcal` — stint-pace inverse calibration

The thermal-ring wear/degradation model (HANDOFF §7.2/§7.3) has physically-meaningful parameters
(`k_w`, `w_c`, `s_w`, `Δ_c`, the grip-window and thermal-damage terms) whose *magnitudes* are not
knowable a priori. `wearcal` fixes them the way a race engineer does: **inversely, from the per-lap
pace curve of a real stint** — recovering the parameters that reproduce the observed pace loss and
cliff.

## How it works

The optimiser (`scipy.optimize.least_squares`, `trf` + `soft_l1`) inverts a **fast reduced-order
stint-pace surrogate** (`model.py`) — a clean-room numpy mirror of the Rust ring's laws (Archard
sliding-energy wear, Grosch temperature-hardness, the C¹ cliff sigmoid, threshold-power thermal
damage, the Farroni grip window). This mirrors how `outlap.tirefit` uses a numpy MF6.1 model
against the Rust force kernels. Running the real stint driver inside a fit is impractical — every
evaluation rebuilds the g-g-g-v envelope across its tyre-state axes — so the surrogate is anchored
to a reference F1-on-Catalunya stint and validated end-to-end against the real driver (PR9
`docs/validation/wear-cliff.md`). The faithful (slow, opt-in) forward model wrapping the real
driver is in `sim.py`.

## CLI

```bash
# Recover known parameters from a synthetic stint (round-trip recovery test):
python -m outlap.wearcal synth     data/tires/.../car.tyr.yaml -o stint.csv --n-laps 25 --noise 0.03
python -m outlap.wearcal calibrate stint.csv --base data/tires/.../car.tyr.yaml -o fitted.tyr.yaml \
        --free k_w,w_c,s_w,delta_c --report-dir /tmp/report

# Confirm calibrated parameters reproduce the decay in the real Rust driver:
python -m outlap.wearcal sim-check fitted.tyr.yaml --vehicle data/vehicles/limebeer_2014_f1 \
        --track data/tracks/catalunya_osm --n-laps 20 --tier t0
```

## Redistribution policy (HANDOFF §15)

FastF1 telemetry and any parameters fitted from it are **calibration/validation artefacts only**.
This package **never** commits raw telemetry or fitted TTC parameter sets. The live FastF1 loader
(`load_fastf1`, needs the `wear-cal` extra: `uv sync --extra wear-cal`) retains only anonymised
per-lap times — use it to produce your own private fixtures. The committed offline fixture under
`data/wear/` is a small *derived* pace curve, sufficient for the CI gate.
