<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# `outlap.wearcal`: inverse calibration from stint pace

The wear and degradation model on the thermal ring (HANDOFF §7.2 and §7.3) has parameters that mean
something physical: `k_w`, `w_c`, `s_w`, `Δ_c`, the grip-window terms, and the thermal-damage terms.
Nobody can know their *magnitudes* in advance.

`wearcal` fixes them the way a race engineer does. It works **backward from the per-lap pace curve
of a real stint**, and recovers the parameters that reproduce the observed loss of pace and the
observed cliff.

## How it works

The optimizer is `scipy.optimize.least_squares`, with the `trf` method and the `soft_l1` loss. It
inverts a **fast reduced-order surrogate of stint pace** in `model.py`. That surrogate is a
clean-room numpy mirror of the laws in the Rust ring: Archard wear from sliding energy, Grosch
temperature-hardness, the C¹ cliff sigmoid, thermal damage above a power threshold, and the Farroni
grip window. `outlap.tirefit` works the same way, running a numpy MF6.1 model against the Rust
force kernels.

Running the real stint driver inside a fit is impractical, because every evaluation rebuilds the
g-g-g-v envelope across its tire-state axes. The surrogate is therefore anchored to a reference
stint of the F1 car at Catalunya, and it is validated end to end against the real driver (PR9,
`docs/validation/wear-cliff.md`). The faithful forward model wraps the real driver. It is slow, it
is opt-in, and it lives in `sim.py`.

## Command line

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

Use FastF1 telemetry, and any parameter fitted from it, only to calibrate and to validate. This
package **never** commits raw telemetry. It never commits a fitted TTC parameter set.

The live FastF1 loader is `load_fastf1`. It needs the `wear-cal` extra: run
`uv sync --extra wear-cal`. It keeps only anonymized lap times. Use it to produce your own private
fixtures.

The committed offline fixture under `data/wear/` is a small *derived* pace curve. It is sufficient
for the CI gate.
