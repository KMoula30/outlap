<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Stint-pace fixtures for calibration and validation

This directory holds per-lap pace curves. The curves are derived and anonymized. Two things read
them: the `outlap.wearcal` calibration harness, and the wear and cliff validation gate
(`docs/validation/wear-cliff.md`).

## Redistribution policy (HANDOFF §15)

Use FastF1 telemetry, and any parameter fitted from it, only to calibrate and to validate. Do not
commit either one. This directory never holds raw telemetry. It never holds a fitted TTC parameter
set.

A stint-delta CSV in this directory is a derived file. It holds one lap time for each lap. It holds
no position data, no tire temperature, and no other channel. This is sufficient to drive the
offline CI gate.

To make your own fixtures from live data, use `outlap.wearcal.load_fastf1`. This function needs the
optional `wear-cal` extra. Keep the files that you make outside the repository.

## Files

- `f1_medium_catalunya_stint.csv` holds a 22-lap stint on an F1 medium compound at a
  Catalunya-like circuit. At first the lap times increase by 0.05 s to 0.08 s each lap. Near lap 18
  the tread depth approaches the critical depth, and the decay becomes a cliff. The file is a CSV
  with the columns `lap,lap_time_s` and a `#` comment header.

  This curve is synthetic. Its shape matches a published degradation profile for a medium tire.
  Thus the curve is representative, but it contains no measured data. It is safe to redistribute
  under CC-BY-SA-4.0.
