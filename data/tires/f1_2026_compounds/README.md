<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# F1 2026 compound presets — soft / medium / hard

Three **synthetic** dry-slick compound presets built on the generic racing-slick MF6.1 core (the
`f1_2026` default slick), differentiated the way real compounds differ: peak grip, the temperature
window, and the wear/degradation rate. They exist for the multi-compound strategy demo (M5 PR10,
`notebooks/10_stint_strategy.ipynb`) — a dry-only stage-2 tease (wet is stage-2 per Decision #4).

The thermal/wear blocks are in the racing-slick band calibrated by `outlap.wearcal` (M5 PR7/PR8);
the compounds scale that baseline. **These are not measured tyre data** — no FastF1/TTC parameters
are redistributed (HANDOFF §15).

| | peak grip (LMUX/LMUY) | `t_opt` | `k_w` (wear) | `w_c` cliff | `delta_c` | character |
|---|---|---|---|---|---|---|
| **soft** | 1.02 | 88 °C | 7.0e-9 | 1.7 mm | 0.15 | most grip, warms in fast, wears fastest, earliest cliff |
| **medium** | 1.00 | 95 °C | 4.4e-9 | 2.0 mm | 0.12 | the baseline |
| **hard** | 0.98 | 102 °C | 2.6e-9 | 2.5 mm | 0.09 | least grip, slow to switch on, wears slowest, latest cliff |

The result is a lap-time **crossover**: the soft is quickest for the opening laps (more grip) but
degrades fastest; the hard starts slower but holds pace longest — the trade that drives pit-stop
strategy. To run a stint on a compound, point a vehicle's `tires:` block at the preset (or pass it
through the `overrides`/scratch mechanism); the notebook demonstrates the comparison.

## Usage

```python
from outlap.core import Track, solve_stint_dataset
# Copy an f1_2026 vehicle dir, swap its tyr/*.tyr.yaml for a compound, then run a stint —
# see notebooks/10_stint_strategy.ipynb, which does exactly this for all three compounds.
```
