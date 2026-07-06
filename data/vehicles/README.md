<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Reference vehicles

Self-contained reference `vehicle.yaml` quartet members used by the shipped examples and (from M7)
the hero demo (Locked Decision #1: all four reference vehicles). These are **synthetic reference
data**, not measured — plausible magnitudes clearly labelled at their source (Decision #15).

Each vehicle directory is loadable with an `FsLoader` rooted at it (referenced `.ptm`/`.tyr` files
live in `ptm/` and `tyr/` siblings).

## `f1_2026/` — F1 2026 hybrid (ICE + MGU-K → gearbox → LSD → rear axle)

- `vehicle.yaml` — includes a SYNTHETIC `aero.constant` block (CzA 4.5 m², CxA 1.25 m²) consumed by
  the T0 point-mass tier; the ride-height `aero.map` sidecar and battery ECM params are referenced
  but not shipped yet (consumed by T1+/the battery model in later milestones).
- `ptm/ice_v6.ptm.yaml`, `ptm/mgu_k.ptm.yaml` — neutral powertrain maps (peak torque envelopes).
- `tyr/slick.tyr.yaml` — MF6.1 slick.

Copied from the schema test fixtures; the two may diverge intentionally (fixtures serve schema
tests, these serve demos).

## `tesla_model3_rwd/` — Tesla Model 3 RWD, HV (800 V-class) variant study (1 DU → open diff → rear axle)

- `vehicle.yaml` — Model-3-plausible chassis/mass/aero (spec-sheet values vs documented estimates:
  see the per-parameter provenance in its `README.md`); constant road-car aero (the degenerate
  non-mapped case).
- `ptm/du_{small,medium,large}.ptm.yaml` — three SYNTHETIC Vdc-stacked (`ptm/1.1`) drive-unit
  sizings (the notebook 07 sensitivity axis), written by `python/tools/gen_model3_powertrain.py`.
- `battery/pack_800v.battery.yaml` — SYNTHETIC 800 V-class Thevenin pack (the Vdc–SoC coupling is
  live on this car).
- `emotor/rear_du.emotor.yaml` — hand-authored lumped machine-thermal network (estimates flagged).
- `tyr/road.tyr.yaml` — the published Pacejka (2006) 205/60R15 road tyre (documented proxy).
- `local/` (git-ignored) — where the real PDT imports land; never committed (firewall).
