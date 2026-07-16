<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Stint-pace calibration/validation fixtures

Small **derived, anonymised** per-lap pace curves used by the `outlap.wearcal` inverse-calibration
harness and the wear/cliff validation gate (`docs/validation/wear-cliff.md`).

## Redistribution policy (HANDOFF §15)

FastF1 telemetry and any parameters fitted from it are **calibration/validation artefacts only**.
This directory **never** contains raw telemetry or fitted TTC parameter sets. A stint-delta CSV
here is a derived artefact — a per-lap lap-time sequence, with no positional, tyre-temperature, or
other channel data — sufficient to drive the offline CI gate. Produce your own private fixtures
from live data with `outlap.wearcal.load_fastf1` (opt-in `wear-cal` extra); keep them out of the
tree.

## Files

- `f1_medium_catalunya_stint.csv` — a representative F1 medium-compound stint (22 laps,
  Catalunya-like): gradual ~0.05–0.08 s/lap early decay accelerating into a cliff as tread wear
  approaches the critical depth near lap ~18. Format: a `lap,lap_time_s` CSV with a `#` comment
  header. This curve is a *synthetic-but-representative* derived fixture (shape matched to a
  published medium-tyre degradation profile), safe to redistribute under CC-BY-SA-4.0.
