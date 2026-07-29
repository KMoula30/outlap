<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# The outlap Guide

**From zero to hero with the outlap vehicle simulator — v0.2.5**

outlap is an open-source parametric vehicle simulator. You describe a car as data. It can be an F1
car, a GT car, or your daily driver. You pick a real racetrack. outlap then computes how fast that
car can physically get around it, wheel by wheel and watt by watt.

This guide is the complete manual. It assumes almost no prior knowledge of vehicle dynamics or of
simulation. It builds up, chapter by chapter, to everything the tool can do at version 0.2.5.

**How to read this guide.** The chapters are ordered as a course. They also work as a reference.

- **New to all of this?** Read Chapters 1 to 3 in order. They cover what outlap is, the vocabulary
  of the physics, and a hands-on first lap. Then follow your curiosity.
- **Want to run simulations?** Chapters 3, 4, 10, and 14 are the practical core: the quickstart,
  the inputs, the API reference, and the recipes.
- **Want to understand the physics?** Chapter 2 is the primer. Chapters 7 to 9 give the full
  treatment of what the solvers compute, with the equations and the literature they come from.
- **Want to work on the code?** Chapters 5, 6, and 13 cover the file formats, the architecture, and
  the machinery for testing and validation.
- **Stuck?** Chapter 16 is a glossary of every term of art. Chapter 17 is the FAQ.

This guide is a companion to two other resources in the repository. It does not replace them. The
first is the executable notebooks in [`notebooks/`](../notebooks/README.md), which are a guided
tour in runnable form. The second is the theory pages in [`docs/theory/`](theory/), which work at
the level of derivations and carry the equations and citations behind every model.

This document describes **outlap v0.2.5**. That is the complete quasi-steady-state tier, T0 and T1,
plus the first transient tier, T2, which is a closed-loop lap integrated through time with a driver
model.

The code is licensed AGPL-3.0-only. The published JSON Schemas in `schemas/` are Apache-2.0. The
vendored TUMFTM track centerlines are LGPL-3.0.

## Table of contents

1. [What is outlap?](#1-what-is-outlap)
2. [A crash course in vehicle dynamics and lap simulation](#2-a-crash-course-in-vehicle-dynamics-and-lap-simulation)
3. [Installation and your first lap](#3-installation-and-your-first-lap)
4. [The input quartet: vehicle, track, conditions, sim](#4-the-input-quartet-vehicle-track-conditions-sim)
5. [Files and formats: schemas, maps, and tables](#5-files-and-formats-schemas-maps-and-tables)
6. [Architecture: how the code is organized](#6-architecture-how-the-code-is-organized)
7. [Physics I: tires and aerodynamics](#7-physics-i-tires-and-aerodynamics)
8. [Physics II: solving a lap — T0, T1, T2, and the g-g-g-v envelope](#8-physics-ii-solving-a-lap--t0-t1-t2-and-the-g-g-g-v-envelope)
9. [Physics III: powertrain, machine thermal, battery, and slow states](#9-physics-iii-powertrain-machine-thermal-battery-and-slow-states)
10. [The Python API reference](#10-the-python-api-reference)
11. [Importers and tooling](#11-importers-and-tooling)
12. [The shipped data library](#12-the-shipped-data-library)
13. [Validation, testing, and trust](#13-validation-testing-and-trust)
14. [Recipes: worked examples](#14-recipes-worked-examples)
15. [Limitations, history, and roadmap](#15-limitations-history-and-roadmap)
16. [Glossary](#16-glossary)
17. [FAQ and troubleshooting](#17-faq-and-troubleshooting)

---

## 1. What is outlap?

*What you will learn: what outlap is, and what problem it solves. The small set of design rules
that shape everything in the codebase — one vehicle description, the input quartet, the powertrain
firewall, clean-room physics, and determinism. What actually ships in v0.2.5, including its honest
gaps. And how the rest of this guide is organized, so that you can find your own path through it.*

### 1.1 A lap simulator in which the car is data

outlap is an open-source **parametric vehicle simulator**. It is a program that predicts how a car
behaves on a race track, from a written description of the car's physical properties. The most
important prediction is the **lap time**: the seconds it takes to complete one lap.

"Parametric" means that the car is not code. It is a set of plain YAML files, holding mass,
aerodynamic coefficients, tire data, and a wiring diagram for the drivetrain. One binary loads
those files and simulates them. Change a number, and you get a different car.

The same engine spans a Formula 1 car, a GT racer, and a road-going passenger car. A Monte Carlo
layer for race strategy is planned on top; see Chapter 15, on limitations and the roadmap.

The project holds three deliverables in one repository:

- a **Rust core**, which is a Cargo workspace under `crates/`. It holds the math, the contract for
  the file formats, the tire and track models, and the lap solvers.
- a **Python API**, at `python/src/outlap/`. It is a thin typed layer over the compiled core, built
  with PyO3 and maturin. It returns results as labelled `xarray` Datasets.
- **published JSON Schemas**, in `schemas/`. They are the machine-readable contract for every input
  file. There are eight kinds of document: `vehicle`, `ptm`, `tyr`, `emotor`, `battery`, `track`,
  `conditions`, and `sim`. They are generated from the Rust types, and versioned as a semver
  contract. The current version is 1.4; see `SCHEMA_MAJOR` and `SCHEMA_MINOR` in
  `crates/outlap-schema/src/lib.rs`.

Here is the whole product in five lines. `solve_lap_dataset` loads a car directory and a track,
assembles the physics, solves a lap, and hands back a dataset indexed by distance around the lap:

```python
from outlap.core import Track, solve_lap_dataset

track = Track.load("data/tracks/catalunya_osm")   # 3D Circuit de Barcelona-Catalunya, 4678 m
lap = solve_lap_dataset("data/vehicles/f1_2026", track)
print(lap.attrs["lap_time_s"])                    # seconds; speed/accel channels live in lap["v"], ...
```

Chapter 3, on installation and your first lap, walks through this end to end. Chapter 10 documents
the full Python API.

Every number in this guide that looks like a measurement is computed by the shipped code, from the
shipped data. Here is an example. On the 3D Catalunya import, the `f1_2026` reference car laps the
centerline in 112.520 s, and the generated racing line in 108.096 s. That is a gain of 4.424 s from
geometry alone. The source is notebook `00_tour_of_outlap.ipynb`, with committed outputs, which CI
re-executes.

### 1.2 The design philosophy: one car, four tiers, nothing silent

A handful of founding rules shape everything else you will meet in this guide.

**One vehicle description, and every solver tier reads it.** A lap solver can model a car at
different levels of detail. outlap calls those levels **tiers**. T0 treats the car as a single
point of mass. T1 adds four wheels and load transfer. T2 integrates the car's motion through time,
with a driver model. T3, which is future work, adds degrees of freedom for the suspension.

Every tier reads *the same* `vehicle.yaml`. There is no "T1 version" of a car, and there is no
parameter that only one tier can see. That is what makes validation across tiers meaningful; see
Chapter 8, Physics II.

**The input quartet.** Exactly four inputs fully specify a simulation. **Vehicle** says what the
car is. **Track** says where it drives. **Conditions** gives the session environment, such as air
temperature and pressure. **Sim** gives the numerical settings, such as the tier, the grid sizes,
and the coupling modes.

Car identity is never mixed with environment or with numerics. Swapping one input never silently
changes another. Chapter 4 is devoted to the quartet.

**Composition is driven by data.** One binary loads any `vehicle.yaml`. The drivetrain is described
in data, as a topology graph of drive units, gearbox, differentials, and wheels. It is not
described as a code path for each car.

Which tire model runs, which aero map applies, how the powertrain is wired: all of it is decided
while *loading* the files. None of it is decided while *solving* the lap.

**The assembly pipeline against the hot loop.** The code splits into a cold **assembly** stage and
a hot **solve** stage.

Assembly is the staged load pipeline in `crates/outlap-schema/src/load/mod.rs`. It parses the YAML,
resolves inheritance through `extends:`, applies overrides, validates every field, checks the
drivetrain topology, estimates missing derivable values, and hashes the resolved result.

Anything that was estimated, inherited, or degraded appears in a **loaded-model report**. "Nothing
silent" is a founding rule. Call `outlap.core.vehicle_report(...)` to see the report for any car.

The hot loop is the set of solver kernels that actually compute the lap. It runs with **zero heap
allocations**. CI enforces that property, through an allocation-counting test harness at
`crates/outlap-qss/tests/alloc.rs`. A wall-clock gate runs alongside it: a full Catalunya lap must
solve in under 50 ms, in a release build.

**Determinism.** The same inputs always produce the same lap. The numerics are fixed-step. The
coupling of the slow states runs a fixed iteration count, not one driven by a tolerance. The
ordering stays deterministic even when the envelope generator runs in parallel.

A setting that could change results is recorded on every result you get back. The mode for
vertical-load coupling is one: `fz_coupling: one_step_lag | fixed_point`, readable at
`lap.attrs["fz_coupling"]`.

**Units and axes.** Everything internal is SI: meters, m/s, rad/s, newtons, newton-meters, watts,
and kelvin. RPM and °C appear only at the boundaries of a file format or a display, as in the
`machine_temp_c` result channel.

The axis convention is ISO 8855: x forward, y left, z up. A left turn therefore has positive
lateral acceleration. This guide restates the signs wherever they matter.

### 1.3 The powertrain firewall, and its one documented exception

outlap deliberately does **not** model an electric machine, an inverter, or a gearbox from the
inside. There is no electromagnetic simulation, and no machine design.

A powertrain enters instead as a **`.ptm` map file**. That is a neutral table which says: at this
shaft speed, and optionally at this DC-link voltage, the unit can produce this much torque, at this
efficiency.

This boundary is hard rule #1 of the project. It is the **powertrain firewall**. It exists so that
outlap can consume the *results* of a professional toolchain for powertrain design, without ever
reimplementing that toolchain or absorbing it.

The importer for one such toolchain, called PDT, reads its raw HDF5 exports with `h5py`, and writes
`.ptm` files. A real export is never committed to the repository. See Chapter 11, on importers and
tooling.

There is exactly one documented exception. It is worth knowing, because you will see it cited in
the code.

The *thermal* model of a machine is an N-node lumped-parameter thermal network, or LPTN. It tracks
the temperatures of the winding, the rotor, and the housing, and it derates torque when they run
hot. It ports the PDT heat-transfer correlations into `crates/outlap-thermal`: the air-gap film,
convection in the end cavity and at the shaft, and the liquid-jacket channel.

The docs of that crate call it "a deliberate amendment of the powertrain firewall for the
(author-owned) thermal model". The correlations themselves are standard published forms —
Becker–Kaye and Taylor, Kylander, Etemad, Churchill–Chu, and Gnielinski — and each is cited at its
definition. Chapter 9, Physics III, covers the thermal model in depth.

### 1.4 Clean-room engineering, and licensing

All flagship physics is implemented **clean-room, from published literature**. Each model ships
with a theory page that carries the citations, in the same pull request as the code.

The MF6.1 tire model comes from Pacejka's *Tire and Vehicle Dynamics* (2012, 3rd ed.). The
lap-solver formulation comes from Perantoni & Limebeer (2014), and from Lovato & Massaro. The
velocity-profile and racing-line methods come from Heilmeier et al. (2020), and from Braghin et al.
(2008). The interpolant comes from Fritsch & Carlson.

Another open-source project may be *consulted* for approach, where its license permits reading.
But the code is always re-authored independently, and the consulted project is recorded.

Where an external implementation is used as a numerical oracle, its *outputs* are used as data
only. Its source is never read into outlap. The GPL tire library behind the golden test files in
`tools/goldens/` is one such oracle. See Chapter 13, on validation, testing, and trust.

Licensing is layered on purpose:

| What | License |
|---|---|
| Code (`crates/`, `python/`) | AGPL-3.0-only (SPDX header in every file) |
| `schemas/` (the JSON Schemas) | Apache-2.0 — so any tool may implement the file formats |
| `data/` (reference vehicles, tires, tracks) | CC-BY-SA-4.0 |
| Vendored TUMFTM track centerlines (`data/tracks/`) | LGPL-3.0 (upstream text shipped verbatim) |

AGPL-3.0 was a deliberate choice. Commercial use is fine, but always with open source code.

A contribution requires a DCO sign-off, through `git commit -s`. See `CONTRIBUTING.md`.

### 1.5 What ships in v0.2.5

At a glance, here is what works today.

- **Three solver tiers.**

  T0 is a point-mass solver. It sweeps a velocity profile forward and backward, on the full 3D road
  ribbon.

  T1 is a quasi-steady-state double-track solver, with per-wheel loads. It generates the **g-g-g-v
  envelope**, which is a precomputed map of the car's acceleration limits against speed. T0 then
  consumes that envelope, which gives fidelity close to T1 at the speed of T0.

  **T2 is the transient tier.** A chassis with 7 degrees of freedom is integrated through time, at
  1 ms, in a curvilinear 3D road frame. It adds tire relaxation, an ideal preview driver, a state
  machine for gear shifts, torque vectoring, and blending of regeneration. The car is *driven*
  around the lap. It is not solved station by station. See Chapter 8.
- **Tires.** A clean-room implementation of the steady-state Magic Formula 6.1, plus a simpler
  physical brush model as the fallback tier. There is a codec for `.tir` files, a fitting pipeline
  in Python, and first-order slip relaxation, which is live in T2. See Chapter 7.
- **Powertrain, thermal, and battery.** A powertrain built from `.ptm` maps, over a drivetrain
  topology graph defined in data. The N-node machine thermal network, with torque derating. And an
  equivalent-circuit battery model, whose terminal voltage feeds back into the drive-unit maps,
  which is the Vdc–SoC coupling.

  All of these march as "slow states" along the lap. At T2 they charge and discharge live through
  time: blended regeneration under braking, and a traction draw under power. See Chapter 9.
- **Racing lines.** Two generators, both quadratic programs. One is the classic
  **minimum-curvature** line. The other is the **time-weighted** line, which re-weights the same QP
  by the time spent at each station. That is the first step from "minimum curvature" toward
  "minimum time". Either result is a first-class track that you can lap. See Chapters 8 and 10.
- **The data library.** Three reference vehicles: `f1_2026`, `limebeer_2014_f1`, and
  `tesla_model3_rwd`, which is an 800 V "HV variant" study and deliberately not the production car
  at about 360 V. Three tire sets backed by citations.

  And 27 track directories: 25 flat LGPL circuits from TUMFTM, plus two 3D imports from OSM with
  elevation. Those two are `catalunya_osm`, which is the reference Catalunya that the notebooks and
  the validation use, and `spa_osm`, which is Spa-Francorchamps with its real elevation of about
  100 m. See Chapter 12.
- **Importers.** For OSM and DEM tracks, TUMFTM tracks, PDT HDF5 powertrains, and `.tir` files. See
  Chapter 11.
- **Validation.** The Limebeer cross-check reproduces the published F1 top speed to −0.2 %, at 87.8
  against about 88 m/s, and the corner apex speeds within 5 %.

  The tire model matches an independent oracle to 0.289 % in the worst case, across every sweep of
  slip, load, camber, and combined slip, and across every force and moment channel. The CI gate is
  0.5 %.

  And the closed-loop operating points of the T2 tier are gated to stay inside the T1 grip
  envelope. That is hull containment, and the measured exceedance is 0.0 % on all three reference
  cars. See Chapter 13.

Here are representative numbers. All are computed by the shipped code, and reproduced elsewhere in
this guide.

The `f1_2026` reference car laps the 3D Catalunya in 112.520 s on the centerline, and in 108.096 s
on the racing line. T0 and T1 agree; see §1.1 and Chapter 8. It reaches about 2.5 g of lateral
acceleration.

The Model 3 HV variant laps the racing line of the same circuit in about 149 s, and the flat
Nürburgring GP in about 155 s (Chapter 3), at roughly 0.8 g. Its lap time responds to the sizing of
the drive unit, to thermal derating, and to sag in the pack voltage.

What v0.2.5 does **not** contain matters just as much. So does what it contains with an honest
caveat. This guide will not pretend otherwise.

- **The T2 driver is stable. It is not at the limit.** The ideal driver of the transient tier
  tracks a **corner-scaled** speed reference. It takes the full QSS profile on the straights, and a
  stability margin of about 0.85 where the profile rides the lateral grip limit. Braking and
  traction feasibility, both aware of the friction ellipse, shape the transitions.

  A T2 lap therefore runs about 14 % to 17 % slower than T0 and T1, while its *top speeds* now come
  within a few percent of the QSS profile. Pushed to the raw profile, it still spins. The margin at
  the limit is the honest boundary of this driver.

  The T2 *physics* is validated: its operating points stay inside the T1 grip envelope. The
  remaining gap in pace is the driver's competence at the limit. Every T2 result records the
  margin. See Chapters 8 and 13.
- **T3 does not exist.** Requesting `tier="t3"`, which is the 14-degree-of-freedom model with
  suspension, raises a typed "not implemented" error.
- **Tire thermal state and wear are placeholders.** The `.tyr` files carry `thermal` and `wear`
  blocks that are synthetic, and clearly labelled as such. The real models — a tire that heats,
  gains and loses grip, and wears — are the next major addition to the physics. See Chapter 15.
- **ERS is a power cap plus regeneration.** Regenerated energy now flows back into the pack:
  blended regen braking at T2, and signed slow-state energy at T0 and T1. But the F1-style *energy
  manager*, which schedules when to spend and when to save, is future work.
- **`data/presets/` is empty.** Class presets such as `formula_base` are planned. And the three
  plugin points — custom blocks, C-ABI tires, and controllers — are a designed extension mechanism.
  They are not shipped code.
- Three of the thirteen crates are two-line placeholders, reserving names for later work. They are
  `outlap-powertrain`, `outlap-batch`, and `outlap-wasm`.

### 1.6 A map of the repository

| Path | What lives there |
|---|---|
| `crates/` | The Rust workspace: 13 crates, 10 real (`outlap-core`, `-schema`, `-tire`, `-track`, `-thermal`, `-qss`, `-raceline`, `-vehicle`, `-transient`, `-py`) and 3 placeholders. Chapter 6 draws the full graph. |
| `python/` | The Python package (`src/outlap/`), its tests, and `tools/` plotting/generation scripts. |
| `schemas/` | The published JSON Schemas (Apache-2.0), generated from the Rust types and CI-checked. |
| `data/` | The shipped library: `vehicles/`, `tires/`, `tracks/`, `presets/` (empty at v0.2.5). |
| `notebooks/` | Companion notebooks `00`–`09`, committed with outputs and re-executed in CI. |
| `docs/` | `theory/` (the cited physics pages), `validation/` (the Limebeer cross-check and the QSS↔T2 parity evidence), `derivations/` (the symbolic chassis derivation CI checks against the Rust code). |
| `tools/` | `goldens/` — provenance and regeneration rules for the external-oracle tire golden files. |
| `CLAUDE.md`, `CONTRIBUTING.md`, `CHANGELOG.md` | The working agreement, contribution rules (DCO, clean-room), and the git-cliff changelog. |

Worked examples also live inside the crates, such as `crates/outlap-qss/examples/catalunya_lap.rs`.
They emit CSV, which `python/tools/plot_*.py` consumes. Every figure in `docs/theory/` is therefore
drawn from the real models.

### 1.7 How to read this guide

This guide assumes basic Python. It assumes **no** background in vehicle dynamics or in simulation.
Every term of art is defined where it first appears, and Chapter 16 is a glossary.

Here are four suggested paths.

- **"Just let me drive."** Chapters 2, then 3, then 4. Then notebook `00_tour_of_outlap.ipynb`.
  Chapter 2 gives you the vocabulary of vehicle dynamics. Chapter 3 installs everything and solves
  your first lap. Chapter 4 explains the four files you just used.
- **"I want to understand the physics."** Chapter 7 on tires and aero, then Chapter 8 on solving a
  lap and the g-g-g-v envelope, then Chapter 9 on powertrain, thermal state, and battery. Each is
  paired with a page in `docs/theory/` that carries the equations and the citations.
- **"I want to build with it."** Chapter 5 on file formats, then Chapter 10 on the Python API, then
  Chapter 11 on importers, then Chapter 14 on recipes. Add Chapter 6, on architecture, when you
  need to read the Rust.
- **"Can I trust it?"** Chapter 13 on validation and testing, and Chapter 15 on limitations and the
  roadmap. Read both before you quote an outlap number anywhere that matters.

The eight companion notebooks under `notebooks/` mirror this guide, chapter by chapter. Notebook 00
is the tour. 01 is the workflow of the car as data. 02 covers tracks. 03 covers racing lines. 04
covers the T0 solver. 05 covers the MF6.1 tire. 06 covers the powertrain firewall and the importer.
07 is the T1 capstone.

The Rust core computes every number and every plot in them, live, when CI re-executes them. They
therefore double as end-to-end tests. They are also your safest starting templates.


---

## 2. A crash course in vehicle dynamics and lap simulation

*What you will learn: the physical vocabulary that the rest of this guide is written in. We start
with the four forces that act on a car. We work up through what a tire actually does, why vertical
load and load transfer matter, and how aerodynamics changes the picture. We then assemble those
pieces into the g-g diagram, and its g-g-g-v extension. We finish with what a lap simulator
actually computes, what "quasi-steady-state" means, and how the T0, T1, T2, and T3 tiers of outlap
divide the work. No prior knowledge of vehicle dynamics is assumed.*

Two cars that ship with outlap run through this chapter as examples: the 1765 kg Tesla Model 3, at
`data/vehicles/tesla_model3_rwd/vehicle.yaml`, and the 768 kg reference Formula 1 hybrid, at
`data/vehicles/f1_2026/vehicle.yaml`.

Everything outlap computes uses SI units internally: m/s, N, N·m, W, and K. The axis convention is
ISO 8855: **x points forward, y points left, z points up**. Signs matter constantly in vehicle
dynamics, so this guide restates that convention wherever it bites.

The file formats keep a few human conveniences — RPM, °C, km/h, kPa, and mm — and outlap converts
them at the boundary. See Chapter 5, on files and formats.

### 2.1 The forces on a car

Only four kinds of force act on a car in a lap simulation.

1. **Weight.** Gravity pulls the mass straight down: $W = m g$, with
   $g = 9.80665\ \mathrm{m/s^2}$, which is the constant `G` in `crates/outlap-qss/src/lib.rs`.

   The Model 3 weighs $1765 \times 9.80665 \approx 17.3\ \mathrm{kN}$. The reference F1 car weighs
   about $7.5\ \mathrm{kN}$.

   Weight is what presses the tires into the road. It is the budget that everything else is paid
   from.

2. **Tire forces.** Four contact patches are the only things that connect the car to the road, and
   each is roughly the size of your hand.

   Every acceleration the car experiences — accelerating, braking, cornering — is ultimately a
   horizontal force generated at those patches. §2.2 is devoted to how.

3. **Aerodynamic forces.** Air resists motion, which is **drag**, pointing backward. On a car with
   wings or ground effect, air also presses the car down, which is **downforce**.

   Both grow with the *square* of speed. Downforce is the one loophole in the weight budget: it
   adds vertical load on the tires without adding mass. §2.4 quantifies this.

4. **Driveline torque.** The powertrain — an engine or an electric machine, through gears and a
   differential — applies torque to the driven wheels.

   Take a drive unit that produces shaft torque $\tau$. Gear it down by a total ratio, through a
   driveline of efficiency $\eta$, to wheels of radius $r$. It then pushes the car forward with a
   force of at most $F = \tau \cdot \text{ratio} \cdot \eta / r$. At most, because the tire may run
   out of grip first. outlap precomputes $\text{ratio}\cdot\eta/r$ as `force_per_torque`, in
   `crates/outlap-qss/src/vehicle.rs`.

   A powertrain is always consumed as a map file, `.ptm`, either measured or synthesized. The
   committed drive unit of the Model 3 peaks at roughly 203 kW, in
   `data/vehicles/tesla_model3_rwd/ptm/du_medium.ptm.yaml`, which is a clearly labelled synthetic
   dataset. Chapter 9, Physics III, covers powertrains.

Newton's second law ties these together. The car's acceleration is the sum of the forces divided by
its mass: $\vec{a} = \sum \vec{F} / m$.

Racers quote acceleration in "g", which is a multiple of $g$. A good road car brakes and corners at
about 1 g. A Formula 1 car corners at 3 g to 5 g.

Why the difference? Almost entirely tires and downforce. That is why the next three sections exist.

### 2.2 What a tire actually does

#### 2.2.1 The contact patch, and slip

A tire is not a rigid wheel. The rubber in contact with the road, which is the **contact patch**,
deforms. Think of the tread as rows of tiny elastic bristles, which get bent sideways and
lengthwise as the patch rolls through. Bent rubber pushes back, and the sum of those bristle forces
is the tire force. The simpler tire model in outlap, the *brush model*, computes forces from
literally this picture. See Chapter 7.

A crucial and unintuitive consequence follows: **a tire only produces force while it is slipping
slightly.** Not sliding, as a locked wheel does. It operates with a small, controlled difference
between how the wheel rotates and points, and how it actually moves over the road.

Two dimensionless quantities measure this. outlap defines both exactly as a modern `.tir` tire file
does. The sign contract lives in `crates/outlap-tire/src/slip.rs`.

**Slip ratio** $\kappa$, or kappa, measures longitudinal slip. It is the mismatch between the
wheel's rolling speed and the road passing underneath it:

$$\kappa = -\frac{V_{sx}}{|V_{cx}|}$$

$V_{sx}$ is the longitudinal sliding velocity of the contact patch. $V_{cx}$ is the forward
velocity of the wheel center. The ratio is dimensionless. It is not a percentage.

$\kappa > 0$ means the wheel spins faster than it rolls, which is driving. $\kappa < 0$ means it
spins slower, which is braking. $\kappa = -1$ is a locked wheel while the car still moves forward.

Peak grip typically arrives near $|\kappa| \approx 0.1$. Push past it, and the patch slides, force
falls, and under braking the wheel locks.

**Slip angle** $\alpha$, or alpha, measures lateral slip. It is the angle between where the wheel
*points* and where it *travels*:

$$\tan\alpha = \frac{V_{sy}}{|V_{cx}|}$$

A rolling tire held at a few degrees of slip angle generates a large sideways force. That is how a
car corners.

Now the sign convention, which is ISO-W, the convention of the tire files that outlap reads. A
positive $\alpha$ means the contact patch slides toward +y, which is left. On a normal tire that
produces a *negative* force $F_y$, which points right.

So in a left-hand corner, where the car needs a leftward +y force, the tires run *negative* slip
angles. This double negative trips up everyone once. The tire kernels of outlap are written never
to take an absolute value, precisely to preserve it, and this guide flags it again wherever it
matters.

Two more inputs complete the state of the contact patch.

- **Camber**, or inclination angle, $\gamma$. It is the lean of the wheel about its own x-axis, and
  it is positive when the top leans toward +y. A cambered tire pulls toward its lean. Racers use
  static camber to compensate for body roll.
- **Normal load** $F_z$. It is the vertical force that presses the patch into the road, and it is
  compressive-positive in outlap. A wheel in the air, where $F_z \le 0$, produces exactly zero
  output. The models short-circuit rather than extrapolate.

All of this is bundled into one struct, `SlipState`. Every tire model in outlap answers with the
same five outputs, in `TireForces`: the longitudinal force $F_x$, the lateral force $F_y$, and
three moments.

The interesting moment here is the **aligning moment**, $M_z$. The lateral force of the patch acts
slightly behind the wheel center. That lever arm is called the **pneumatic trail**, and it produces
a torque that tries to steer the wheel straight.

That self-centering torque is most of what a driver feels in the steering wheel. Its fade as the
tire approaches the limit is the classic warning that "the steering went light".

How the forces are computed from the slip state is the subject of Chapter 7. outlap implements the
industry-standard Magic Formula MF6.1, from Pacejka 2012, plus the brush model. For this chapter
you only need the shape of the answer: force rises steeply and nearly linearly at small slip, then
peaks, then decays as the patch slides.

#### 2.2.2 The friction coefficient, and load sensitivity

Divide the peak horizontal force a tire can make by the vertical load on it, and you get the
**friction coefficient**:

$$\mu_x = \frac{\max_\kappa |F_x|}{F_z}, \qquad \mu_y = \frac{\max_\alpha |F_y|}{F_z}$$

In school physics, $\mu$ is a constant of the two materials. Rubber on asphalt does not work that
way. **The friction coefficient falls as load rises.** This is called **load sensitivity**, and it
is arguably the single most important fact in vehicle dynamics.

Here are real numbers from the shipped tire of the Model 3, at
`data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml`. It is a verbatim copy of the 205/60R15
validation tire from the Pacejka book, with a rated load `FNOMIN` of 4000 N and a cold pressure of
220 kPa. The peak scanner of outlap extracted them:

```python
from outlap_core import Tyre
t = Tyre.load("data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml")
t.peak_mu(4000.0, t.p_cold)   # -> (1.21, 1.035)
```

| Vertical load $F_z$ | peak $\mu_x$ (longitudinal) | peak $\mu_y$ (lateral) |
|---|---|---|
| 2000 N | 1.23 | 1.12 |
| 4000 N (rated) | 1.21 | 1.03 |
| 6000 N | 1.19 | 0.95 |
| 8000 N | 1.17 | 0.87 |

Doubling the load from 4 kN to 8 kN yields *less than double* the lateral force. $\mu_y$ drops by
16 %.

The F1 reference tire shows the same effect, at a much higher level. It is at
`data/vehicles/limebeer_2014_f1/tyr/f1.tyr.yaml`, transcribed from Perantoni & Limebeer 2014. Its
$\mu_y$ is 1.80 at 2 kN, and it falls to 1.45 at 6 kN.

A racing tire is not just "stickier". It is also *more* load-sensitive. That makes the story about
managing load, in §2.3, even more consequential there.

#### 2.2.3 The friction circle, and combined slip

A tire cannot give you maximum braking force and maximum cornering force at the same time. The
contact patch has one budget of grip, and it spends that budget in whatever direction it is asked.

The classic picture is the **friction circle**. It is really an **ellipse**, because
$\mu_x \ne \mu_y$. The achievable force vector $(F_x, F_y)$ is confined to

$$\left(\frac{F_x}{\mu_x F_z}\right)^2 + \left(\frac{F_y}{\mu_y F_z}\right)^2 \le 1$$

Trail-braking — carrying brake force into corner entry while lateral force builds — is literally
driving around the rim of this ellipse.

outlap uses the idea at two levels of fidelity.

- The point-mass solver (§2.9) uses the ellipse literally, with one ellipse for each car. It takes
  $\mu_x$ and $\mu_y$ from the peaks of the tire model. The module doc of
  `crates/outlap-qss/src/solver.rs` states the exact inequality, including a grip scale for the
  track at each station.
- The full tire model does something subtler, called **combined slip**. When $\kappa$ and $\alpha$
  are both nonzero, MF6.1 attenuates each pure-slip force with weighting functions shaped like a
  cosine, in `crates/outlap-tire/src/mf61/combined.rs`. The resulting boundary is ellipse-*like*,
  but asymmetric and dependent on load. A measured tire simply is not a perfect ellipse.

One more wrinkle from the real world: $\mu$ also depends on the road surface. Grip varies from
corner to corner. The track format of outlap therefore carries a `grip_scale` column at each
station; see the header of `data/tracks/catalunya_osm/centerline.csv`. It scales the tire grip
locally.

#### 2.2.4 What the tire model knows, and what it does not know yet

Real tire grip also depends on inflation pressure, on temperature, and on wear. It is worth knowing
exactly where outlap v0.2.5 stands on each, because the `.tyr` file format already has fields for
all three.

- **Pressure is live.** MF6.1 includes the inflation-pressure terms of Besselink et al. (2010). The
  QSS solvers evaluate every tire at its cold set pressure, which is the `thermal.p_cold` field of
  the `.tyr` file. That field is in kPa, and it converts to Pa at the boundary; for the Model 3
  tire it is 220 kPa.

  Changing that pressure genuinely changes grip, *if* the coefficient set carries pressure terms.
  The book tire of the Model 3 is a set from 2006, without the `PP*` coefficients, so it is
  insensitive to pressure. Its file header documents exactly that.
- **Camber is accepted** by the tire kernels. But the T1 trim currently evaluates all four wheels
  at zero camber. That simplification is recorded in the assembly notes: "camber maps land later".
- **Temperature and wear are live**, as of v0.3. The `thermal:` and `wear:` blocks of the `.tyr`
  file drive two models. The first is a thermal ring based on physics, with nodes for surface,
  carcass, and gas, and a grip window in temperature. The second is an Archard wear model with a
  cliff. Both march as slow states, in T0 and T1 and in T2.

  Grip therefore depends on temperature. Tires warm up and go off. And a *stint*, not just a lap,
  is simulable. `outlap.wearcal` calibrates the parameters inversely, from stint pace.

  Opt in with `tire_thermal=True`, which is on by default for a stint. The theory is in
  [`docs/theory/tire-thermal.md`](theory/tire-thermal.md) and
  [`tire-wear.md`](theory/tire-wear.md).

### 2.3 Why load transfer matters

#### 2.3.1 Acceleration shifts load

The center of gravity, or CG, of a car sits well above the road. It is 0.45 m up on the Model 3,
which is the `chassis.cg` field of its `vehicle.yaml`. It is about 0.30 m up on the F1 cars.

The inertial force acts at the CG. The reacting tire forces act at road level. That vertical offset
means that every acceleration tilts the distribution of load, even with an infinitely stiff
suspension.

This is **load transfer**, also called weight transfer. It comes in two kinds.

**Longitudinal, or pitch, transfer.** Under acceleration $a_x$, load moves rearward. Under braking,
it moves forward:

$$\Delta F_z^{x} = \frac{m\, a_x\, h_{cg}}{L}$$

$h_{cg}$ is the CG height, and $L$ is the wheelbase.

For the Model 3, with $h_{cg} = 0.45$ m and $L = 2.875$ m, braking at 1 g moves
$1765 \times 9.80665 \times 0.45 / 2.875 \approx 2.7\ \mathrm{kN}$ from the rear axle to the front.

Statically the car carries 47 % front and 53 % rear. The front share is $b_r/L$, where
$b_r = L - 1.524$ m is the distance from the CG to the rear axle. That is exactly how
`T1Vehicle::front_weight_fraction` computes it.

Under that 1 g stop, the front axle load jumps from about 8.1 kN to 10.8 kN, while the rear falls
to 6.5 kN. That is why a brake system is biased forward. The `brakes.balance_bar: 0.62` of the
Model 3 sends 62 % of the brake torque to the front axle, which roughly matches where the load
went.

**Lateral, or roll, transfer.** Cornering at $a_y$ moves load from the inside pair of wheels to the
outside pair. With equal track width $t$ at front and rear, the total across the car is
$m\, a_y\, h_{cg} / t$. For the Model 3 at 1 g that is just under 5 kN, with $t = 1.58$ m, from the
`chassis.track_m` field.

#### 2.3.2 Load transfer costs grip

Combine load transfer with the load sensitivity of §2.2.2, and you get the punchline.

Take an axle that carries 8 kN, split 4 kN and 4 kN. Each tire offers $\mu_y = 1.03$, so the
effective friction coefficient of the axle is 1.03.

Now transfer 2 kN across it, so the split becomes 2 kN and 6 kN. The lightly loaded tire gains a
little, reaching $\mu_y = 1.12$. The heavily loaded one loses more, falling to $\mu_y = 0.95$. The
load-weighted average therefore drops:

$$\frac{2000 \times 1.12 + 6000 \times 0.95}{8000} \approx 0.99$$

The *total* vertical load on the axle is unchanged. Yet the axle lost about 4 % of its cornering
grip, purely because the load is now uneven. **Load transfer always reduces the grip of the axle it
acts on.**

You cannot eliminate lateral load transfer. Only a lower CG, a wider track, or less mass reduces
the total.

But the chassis designer chooses *which axle pays*, and that choice sets the balance of the car.

- **The distribution of roll stiffness.** The body rolls in a corner, and the front and rear
  suspensions share the job of resisting the roll moment. The stiffer end reacts a larger share,
  and it therefore takes more of the lateral load transfer, and loses more grip.

  Stiffening the front, with a bigger front anti-roll bar, pushes the car toward **understeer**.
  The front axle saturates first, and the car runs wide: stable, and dull.

  Stiffening the rear pushes toward **oversteer**. The rear axle saturates first, and the tail
  steps out: exciting, and prone to a spin.
- In outlap this is the `roll_stiffness_share` field for each axle. On the Model 3 it is 0.58 front
  and 0.42 rear.

  The lateral transfer at each axle is computed as two parts. The *geometric* part is reacted
  through the suspension linkage, and `roll_center_height_m` sets it. The *elastic* part is reacted
  through the springs and bars, and the shares of roll stiffness set it.

  That is the classic decomposition from Milliken & Milliken, *Race Car Vehicle Dynamics* (1995).
  It is implemented in `load_transfer`, in `crates/outlap-qss/src/t1/trim.rs`, and derived in
  `docs/theory/t1-trim.md`.
- **Wheel lift** is the limiting case. Transfer more than the load on the inner wheel, and that
  wheel carries zero. outlap floors the lifted wheel at 0 N, and gives the whole axle load to the
  outside wheel. It does this exactly so that the predicted grip limit does not become optimistic.

A one-number summary of the resulting balance is the **understeer gradient**. It is the extra
steering angle, for each unit of lateral acceleration, that the car needs beyond pure geometry. It
is positive for understeer.

A T1 lap reports it at each track station, as `understeer_gradient`. Chapter 8 gives the exact
definition, $K = d\delta/da_y - L/v^2$, and how it is probed.

### 2.4 Aerodynamics: drag, downforce, and balance

Aerodynamic forces scale with dynamic pressure. Both are therefore proportional to the square of
speed:

$$F_{drag} = \tfrac{1}{2}\,\rho\, C_x A\, v^2 \qquad\qquad F_{down} = \tfrac{1}{2}\,\rho\, C_z A\, v^2$$

$\rho$ is air density. $C_x A$ and $C_z A$ are the **drag area** and the **downforce area**, in m².
People often quote those products as CdA and ClA. Working with the product sidesteps arguments
about reference areas.

outlap stores exactly these products, with downforce positive, in the `aero.constant` block of
`vehicle.yaml`:

```yaml
# data/vehicles/f1_2026/vehicle.yaml
aero:
  constant:
    cx_a_m2: 1.25        # drag area
    cz_front_a_m2: 1.9   # downforce area attributed to the front axle
    cz_rear_a_m2: 2.6    # ... and to the rear axle
```

The Model 3 is a clean road car with no wings. It has `cx_a_m2: 0.51`, and zero downforce.

Air density comes from the **conditions** input, through the ideal-gas law. The defaults in
`conditions.yaml` are 20 °C and 1013.25 hPa, which give $\rho \approx 1.204\ \mathrm{kg/m^3}$. See
Chapter 4, on the input quartet.

Here are concrete numbers.

- **Drag.** At 130 km/h, which is 36.1 m/s, the Model 3 fights
  $\tfrac{1}{2} \times 1.204 \times 0.51 \times 36.1^2 \approx 400\ \mathrm{N}$ of drag. That is
  about 14.5 kW, just to hold speed.

  Drag force grows with $v^2$, and drag *power* with $v^3$. Top speed is therefore where the curve
  of drive force crosses the curve of drag. outlap estimates a car's top speed in exactly that way,
  when it sizes the speed axis of the envelope.
- **Downforce.** The total $C_z A = 4.5\ \mathrm{m^2}$ of the F1 car generates roughly 13 kN at
  250 km/h. That is about 1.7 times its own weight; the comment in `f1_2026/vehicle.yaml` records
  this sizing.

  The tires are then pressed into the road by about 2.7 times $mg$, while the mass being cornered
  is unchanged. Lateral capability at that speed therefore climbs toward $\mu_y \times 2.7$. For
  the synthetic slick of this car, where $\mu_y = 1.25$, that is about 3.4 g. On the grippier and
  more load-sensitive tires that a real F1 car runs, it reaches 4 g or more, and load sensitivity
  trims the naive product back down.

  This single mechanism is most of the answer to the question "why does an F1 car corner at 4 g and
  a road car at 1 g?"

Two refinements matter, and Chapter 7, Physics I, treats both in full.

- **Aero balance.** Downforce is split between the axles. On the F1 constants that is
  $1.9/4.5 = 42\%$ front.

  If that split does not match the weight distribution and the mechanical balance, the car
  understeers or oversteers *more as speed rises*. A T1 lap reports the realized split at each
  station, as `aero_front_share`.
- **Sensitivity to ride height.** A ground-effect car changes its coefficients as the floor nears
  the road. That couples aero back into suspension travel.

  The primary representation of aero in outlap is therefore a gridded map, over front and rear ride
  height, yaw angle, and DRS state. It is `aero.map` plus `aero.axes` on the F1 car, and
  `crates/outlap-qss/src/t1/aero.rs` consumes it. The axes of the map use mm and degrees at the
  file boundary.

  The constant block is the fallback. It is also the road-car case.

  DRS is held closed in the shipped QSS solvers, because activating it is a concern for a
  controller.

### 2.5 The g-g diagram, and the g-g-g-v envelope

Put §2.2 through §2.4 together, and ask a question. At some instant, what combinations of
longitudinal acceleration $a_x$ and lateral acceleration $a_y$ can this car sustain?

Plot the feasible set in the $(a_x, a_y)$ plane, and you get the **g-g diagram**. It is a roughly
elliptical blob, whose boundary is the total grip limit of the car. The concept goes back to Rice,
SAE 730018, 1973.

A skilled driver lives on its rim: full braking, then trail-braking around the boundary into pure
cornering, then feeding in throttle on the way out.

For a real car, one g-g diagram is not enough, because the boundary moves with the operating point.

- **Speed $v$.** Downforce grows the blob with $v^2$, and drag skews its shoulders for traction and
  braking. A downforce car has a modest g-g at 100 km/h, and a huge one at 300 km/h.
- **Local vertical acceleration.** Banking presses the car into the road. A crest unloads it. A
  compression, such as Eau Rouge, loads it. All of these change the $F_z$ of every tire at once.

  outlap folds them into one number, $g_{normal}$, which is the road-normal specific force. It
  equals $g$ on flat ground. It rises above $g$ in banking and in a dip, and it falls below $g$
  over a crest. This follows Rowold et al. 2023 and Werner et al. 2025.

The result is the **g-g-g-v envelope**: a family of g-g boundaries, indexed by $(v, g_{normal})$,
and stored as a gridded table, $a_y = gg(v, \hat a_x, g_{normal})$.

Lovato & Massaro (2022) develop the g-g-g idea. Werner et al. (2025, arXiv:2504.10225) give the
formulation that outlap implements.

In outlap, `GgvEnvelope::generate` generates the envelope once for each car, in
`crates/outlap-qss/src/t1/envelope.rs`. The default grid is **40 speed points × 25 longitudinal
points × 7 normal-g points**, which is the `sim.envelope` setting in
`crates/outlap-schema/src/sim.rs`. Speeds run from 5 m/s up to the car's estimated top speed, and
$g_{normal} \in [0.5\,g,\ 2\,g]$.

The two example cars make the shape concrete. The Model 3 has zero downforce, so its lateral
boundary is essentially the same at every speed. Only drag skews the traction and braking shoulders
as $v$ rises. The boundary of the F1 car is a funnel: modest at low speed, and enormous at high
speed, because 13 kN of downforce at 250 km/h multiplies what every tire can do.

Three more properties are worth internalizing now. Chapter 8 has the machinery.

- The envelope is a **pure limit on tire grip**. The force ceiling of the powertrain is
  deliberately *not* baked in; the lap solver applies it separately, as a `min`. Grip and power
  therefore stay independently swappable.
- The "funnel" of the envelope widens with speed, for a downforce car. And a compression gives more
  grip than a crest, at every speed. `docs/theory/ggv-envelope.md` shows the shipped figures.
- For an analysis in which the third g is noise, set `sim.flat_track: true`. That zeroes grade,
  banking, and vertical curvature, so the envelope collapses to a classical flat g-g, where
  $g_{normal} \equiv g$. It is how outlap reproduces a published 2-D study. See Chapter 13.

### 2.6 What a lap simulator does

Strip away the detail, and a lap-time simulator answers one question: **given a path and a car,
what is the fastest speed profile along that path?**

The path is described by arc length $s$, which is distance along the line in meters, and by its
**curvature** $\kappa(s) = 1/R$, which is the reciprocal of the local corner radius.

Driving the path at speed $v$ demands a centripetal, or lateral, acceleration of
$a_y = \kappa v^2$. That demand is what the g-g boundary must cover. A lap simulator finds, at
every point, the highest speed whose demands fit inside the car's capability.

The quasi-steady-state solver of outlap is at `crates/outlap-qss/src/solver.rs`. It was
re-implemented clean-room, from the formulation of Heilmeier et al., *Vehicle System Dynamics*
58(10), 2020.

It is *not* an integration of a differential equation. It is a construction in three phases, on
stations spaced every $\Delta s = 2$ m by default, which is `DEFAULT_DS_M`.

1. **The cornering-limited speed.** At each station, find the highest speed whose lateral demand
   the envelope can meet. Solve
   $\kappa_l v^2 + g\sin\theta_b\cos\theta_g \le a_{y,max}(v, g_{normal})$ for $v$, where
   $\theta_b$ is the banking angle and $\theta_g$ the grade. This caps the speed at every apex.
2. **The forward pass, limited by traction.** Sweep forward from the slowest point. Accelerate out
   of each corner as hard as two things allow, taking the *minimum* of them: tire grip, meaning
   what remains of the friction budget after the lateral demand is paid; and powertrain force. Then
   subtract drag and uphill gravity. This builds the corner exits and the straights.
3. **The backward pass, limited by braking.** Sweep backward. At each station, ask: how fast could
   I have been here, and still slow down in time for what is ahead? The same friction budget is
   spent on braking. Drag and uphill gravity now helpfully *add* to the deceleration. This places
   every braking point.

The final speed profile is the pointwise minimum of all three. For a closed lap, the sweeps iterate
from the slowest corner until they are self-consistent.

Lap time is a sum over segments, taken in a fixed order:

$$t_{lap} = \sum_i \frac{2\,\Delta s}{v_i + v_{i+1}}$$

Grade, banking, and vertical curvature enter through the 3-D path geometry, which is precomputed as
`T0Path` in `crates/outlap-qss/src/path.rs`. Banking assists cornering. A crest unloads the car;
outlap even guards the flying-car case, where $N \le 0$, and coasts an airborne station on drag and
gravity alone. A dip presses the car in.

Here is a practical detail that you will meet when you import a track. Curvature is a *second
derivative* of position. Noise at the scale of a meter, in a surveyed centerline, therefore becomes
a violent spike in curvature.

outlap smooths the projected curvatures with a centered moving average, over ±6 stations. That is
`CURV_SMOOTH_RADIUS` in `path.rs`. Chapter 11, on importers and tooling, discusses where imported
geometry can and cannot be trusted.

Two closed forms are useful anchors for intuition. They are also analytic test cases for outlap;
see `docs/theory/t0-point-mass.md`.

- A flat circle at constant grip: $v = \sqrt{\mu_y\, g\, R}$. The Model 3 tire, with
  $\mu_y \approx 1.03$ at rated load, tops out around $22.5\ \mathrm{m/s}$, or 81 km/h, on a corner
  of 50 m radius.
- Add downforce, and the closed form of the solver becomes
  $v^2 = \mu_y m g \,/\, (m/R - \mu_y q_z)$, with $q_z = \tfrac{1}{2}\rho C_z A$.

  For the 660 kg Limebeer F1 car, with $C_z A = 4.5\ \mathrm{m^2}$ and $\mu_y = 1.63$ at rated
  load, the same 50 m corner allows about $34.6\ \mathrm{m/s}$. Downforce is therefore worth over
  20 km/h *in one corner*, before load sensitivity takes its cut; this back-of-envelope calculation
  ignores that.

  Note the denominator. If downforce grew fast enough to cancel $m/R$, the cornering speed would be
  unlimited. Real cars just get close.
- Banking helps too. The limit on a banked turn is
  $v^2 = gR\,(\mu_y\cos\phi + \sin\phi)/(\cos\phi - \mu_y\sin\phi)$, for bank angle $\phi$. It is
  verified against the solver in `crates/outlap-qss/tests/analytic.rs`.

That is the whole trick. It runs a full lap in well under 50 ms, which is a budget that CI enforces
(Chapter 13, on validation). That speed is what makes parameter sweeps practical, and later the
Monte Carlo layer for race strategy.

One property is worth noticing. Everything in this construction is deterministic: fixed station
spacing, fixed iteration counts, and summations in a fixed order. The same inputs reproduce the
same lap, bit for bit, run after run.

That is a deliberate rule across the project (Chapter 6). It is what makes regression testing
against golden files, and honest A/B comparisons, possible.

### 2.7 Quasi-steady-state against transient simulation

The solver above never asks *how the car gets from one state to the next*. It assumes that at every
station the car is in a **trimmed**, or equilibrated, state: all forces and moments balanced, and
nothing still settling.

That is the **quasi-steady-state (QSS)** assumption. A lap is a sequence of steady states,
parameterized by position, and time appears only as the integral of $1/v$ along the path.

What QSS deliberately ignores is everything that has its own settling time.

- **Yaw, roll, and pitch dynamics.** A real car takes a few tenths of a second to take its "set" in
  a corner. QSS teleports between equilibria.
- **Tire relaxation.** A tire's force lags its slip. The tire must roll a characteristic distance,
  called the **relaxation length**, before the force builds. That distance is of the order of the
  tire radius. outlap ships the exact-exponential lag update in `crates/outlap-tire/src/relax.rs`.
  The QSS tiers use steady-state forces by definition, and the T2 tier integrates the lag live.
- **Dampers and drivers.** A shock absorber only matters when the suspension is moving. A driver
  model only matters when the inputs evolve in time.

A **transient** simulation integrates the equations of motion through time, with a fixed-step
integrator. It captures all of the above, at much higher cost.

That is the **T2 tier** of outlap. Its fixed-step loop is driven by the `dt_s` timestep, which
defaults to 0.001 s, and by the `integrator` choice in `sim.yaml`, which is Heun by default or
classical RK4.

Neither approach is "more correct" for every question:

| | QSS (T0/T1) | Transient (T2) |
|---|---|---|
| Answers | lap time, speed profile, grip usage, balance trends | steering, yaw, sideslip, shift events, transient wheel loads — the *traces* |
| Assumes | equilibrium at every point | only the model equations (plus a driver to close the loop) |
| Cost | milliseconds per lap | seconds per lap |
| Great for | "what does 10 kg or 5% more downforce cost?" — errors largely cancel between variants | "what does the car *do* between the corners?" — the time-domain story |

One honest caveat. The envelope boundary of QSS in outlap is not filtered for open-loop stability.
A trim state can balance all forces, and yet be a knife-edge that no driver could hold.

The T2 tier makes this concrete. Its driver deliberately keeps a stability margin below the QSS
profile, because tracking the raw profile spins the car. See `docs/theory/ggv-envelope.md`, and
Chapter 8.

A slowly evolving quantity sits in a middle ground. Over a lap, the state of charge of a battery,
and the winding temperature of a motor, drift monotonically rather than oscillate.

outlap therefore treats them as **slow states**. It marches them along the QSS profile, with a
bounded outer iteration: solve the profile, march the slow states along it, then re-solve. An
overheating drive unit, or a battery capped on power, then feeds back on lap speed, without needing
a transient solver. Chapter 9, Physics III, covers this coupling.

### 2.8 Point-mass against double-track models

Independent of the choice between QSS and transient, there is a second axis: how much *car* do you
model?

A **point-mass** model collapses the car to a single particle. It has one mass, one friction
ellipse — or one envelope — lumped aero, and a curve of drive force. It cannot tell you *which*
tire gives up, or how balance shifts. It tells you only whether the car as a whole can hold the
demanded acceleration.

The `T0Vehicle` of outlap, in `crates/outlap-qss/src/vehicle.rs`, is exactly this reduction: the
mass, $\mu_x$ and $\mu_y$ averaged over the axles from the peaks of the tire model, the lumped aero
constants $\tfrac{1}{2}\rho C_x A$ and $\tfrac{1}{2}\rho C_z A$, and the folded drive envelope.

A **double-track**, or four-wheel, model keeps both axles *and* both sides. It has per-wheel
vertical loads, with the full longitudinal and lateral transfer of §2.3; per-wheel slip states, fed
to the real tire model; steering geometry; a differential; and brake bias.

The `T1Vehicle` of outlap, in `crates/outlap-qss/src/t1/vehicle.rs`, is a quasi-static double-track
model.

Its `trim` solver answers one question. For a commanded operating point
$(v, a_y, a_x, g_{normal})$, what steering angle, body-slip angle, yaw rate, slip controls, and
four wheel loads balance every force and moment?

That is an algebraic solve with 9 unknowns, at each operating point, in
`crates/outlap-qss/src/t1/trim.rs`. Chapter 8 walks through it.

And if *no* balance exists, because the demand is simply beyond the car, the point is declared
infeasible. That boundary of infeasibility, traced over a grid of operating points, *is* the
g-g-g-v envelope of §2.5.

There is a middle step in the textbooks: the single-track, or "bicycle", model, which has two
wheels and no distinction between left and right. It is a fine tool for hand analysis. outlap jumps
straight to double-track, because load transfer is where the interesting grip physics lives, and a
bicycle model cannot represent it.

### 2.9 The ladder of tiers: T0, T1, T2, T3

outlap names its levels of solver fidelity **tiers**. One field selects them: `tier` in `sim.yaml`,
or the `tier=` argument in Python.

A hard rule of the project (Chapter 6, on architecture) is that **every tier evaluates the same
vehicle description**. There is no "T0 config" against a "T1 config". There is one `vehicle.yaml`,
read at different fidelity.

The enum lives in `crates/outlap-schema/src/sim.rs`. The dispatch lives in
`crates/outlap-qss/src/qss.rs`.

| Tier | What it is | Model class | Status in v0.2.5 |
|---|---|---|---|
| `t0` | Point-mass velocity profile on the g-g-g-v envelope | point-mass | shipped |
| `t1` | The *same* velocity profile, plus a per-station double-track re-trim | quasi-static double-track | shipped — **the default** |
| `t2` | Closed-loop transient double-track with an ideal driver | transient | shipped — time-indexed, own entry point |
| `t3` | 14-degree-of-freedom transient (suspension, unsprung mass) | transient | future — typed error today |

Five details are worth knowing from day one.

- **T0 and T1 produce the same lap time.** Both run the identical velocity-profile solve on the
  envelope.

  `t1` then revisits every station, and re-trims the double-track model at the solved operating
  point. It emits per-wheel channels — `vertical_load_n`, `slip_ratio`, `slip_angle_rad`,
  `force_long_n`, and `force_lat_n`, over a `wheel` dimension ordered FL, FR, RL, RR — plus the
  setup metrics `understeer_gradient` and `aero_front_share`.

  A `t0` lap gives you the point-mass channels only.
- The envelope is *generated by* the T1 trim solver. Even a `t0` lap therefore assembles the
  double-track model once.

  Generating the envelope is a cold step at the scale of seconds, cached for each car and each set
  of settings within a session. The figure of under 50 ms is the solve itself.
- Internally there is also a degenerate path with constant $\mu$, using an ellipse. That is the
  closed-form T0 of the anchors in §2.6, and it is kept as the target for analytic tests and for
  performance tests. The production `t0` that you reach from Python always runs on the envelope.
- **T2 is indexed by *time*, not by arc length.** It integrates the car through time. It therefore
  has its own entry point, `solve_transient_lap`, or `solve_lap_dataset(..., tier="t2")`, and it
  returns a dataset over `time` rather than over `s`.

  Like the QSS tiers, it runs either the full 3D road frame, with grade, banking, and vertical
  curvature, or the flat-track analysis mode.

  Its lap is deliberately slower than T0 and T1 in the corners. The driver tracks a corner-scaled
  reference: the full profile speed on the straights, and a stability margin at the lateral limit.
  See §8.7.
- Requesting `t3` fails loudly, with a typed "not implemented" error. It never silently downgrades.

Every result records the tier that produced it. It also records, in its notes, every simplification
made during assembly: degraded aero, estimated parameters, and fallbacks to a brush tire. "Nothing
silent" is a design rule, not a slogan (Chapter 4).

### 2.10 The racing line against the centerline

Everything above took the path as given. But *which* path?

A track file describes a corridor: a **centerline**, plus left and right widths at each station.
Those are `track.yaml` and `centerline.csv`; see Chapter 5.

Drivers do not follow the centerline. They use the full width to straighten each corner — out, in,
out — because a larger radius at the same grip means a higher $v = \sqrt{a_y R}$.

The chosen path is the **racing line**, and it changes lap time a lot.

The true racing line is the *time-optimal* one. That is a genuinely hard problem in optimal
control, and the Perantoni & Limebeer 2014 study that outlap validates against solves exactly it.

outlap ships the standard first approximation: the **minimum-curvature line**. It is a quadratic
program over lateral offset within the corridor, which minimizes $\int \kappa^2\, ds$. Geometrically
it is the "straightest possible" path.

outlap also ships its refinement, the **time-weighted line**. That re-weights the same QP by the
time spent at each station, and it closes part of the gap between minimum curvature and minimum
time. See Chapters 8 and 10; notebook 03 compares them.

`min_curvature` is the default generator in `sim.raceline`. The alternatives are `time_weighted`,
and a CSV that the user supplies. Every result records which line it ran on, in its
`LineDescriptor`: `Centerline`, `MinCurvature`, `TimeWeighted`, or `File`.

The centerline itself remains useful. It is deterministic, it needs no generator, and it is ideal
for comparing tracks or for cross-checking an importer. The Python API therefore accepts either a
plain `Track`, which laps its centerline, or a generated `Raceline`, as the `line` argument of
`solve_lap_dataset`. See Chapter 10.

Be honest about the gap. Minimum curvature is close to time-optimal in a slow corner. It
systematically under-opens a medium-speed one.

In the Limebeer cross-check, at `docs/validation/limebeer.md`, outlap matches the published top
speed within 0.2 %, and the slowest apex within 5 %. The full lap time is 92.4 s on the committed
track import, against the paper's optimal 82.4 s. That number is recorded, and it is not gated. The
decomposed reasons — line optimality, and the fidelity of the track geometry — are documented.

An optimizer for a time-weighted line is on the roadmap. See Chapter 15, on limitations and the
roadmap.

### 2.11 How these concepts map to outlap

| Concept | Where it lives in outlap |
|---|---|
| Mass, CG, wheelbase, track widths | `chassis:` block of `vehicle.yaml` (`mass_kg`, `cg`, `wheelbase_m`, `track_m`) |
| Tire force model | one `.tyr` file per axle (`tires: {front, rear}`); kernels in `crates/outlap-tire/` (MF6.1 + brush, chosen by `TireModel::from_tyr`) |
| Slip ratio $\kappa$, slip angle $\alpha$, sign contract | `SlipState` in `crates/outlap-tire/src/slip.rs` (ISO 8855 / ISO-W) |
| Peak friction and load sensitivity | `TireModel::peak_mu_x`/`peak_mu_y`; Python `Tyre.peak_mu(fz, p)` |
| Friction ellipse (point-mass form) | `EllipseGrip` in `crates/outlap-qss/src/solver.rs` |
| Combined slip (full tire) | `crates/outlap-tire/src/mf61/combined.rs` |
| Load transfer, roll stiffness, wheel lift | `suspension:` block of `vehicle.yaml`; `load_transfer` in `crates/outlap-qss/src/t1/trim.rs` |
| Drag/downforce areas ($C_x A$, $C_z A$) | `aero.constant:` (`cx_a_m2`, `cz_front_a_m2`, `cz_rear_a_m2`); ride-height maps via `aero.map` + `crates/outlap-qss/src/t1/aero.rs` |
| Air density, wind, temperatures | `conditions.yaml` (ideal-gas density at assembly) |
| Driveline torque → drive force | `.ptm` map per drive unit + gears/diff in `drivetrain:`; folded in `crates/outlap-qss/src/vehicle.rs` |
| g-g-g-v envelope | `GgvEnvelope` in `crates/outlap-qss/src/t1/envelope.rs`; grid via `sim.envelope` (default 40×25×7) |
| Speed-profile lap solver | `crates/outlap-qss/src/solver.rs` (default station spacing 2 m) |
| Solver tiers | `tier` in `sim.yaml` (`t0`/`t1`/`t2`/`t3`, default `t1`); dispatch in `crates/outlap-qss/src/qss.rs` |
| Track curvature, grade, banking, grip scale | `track.yaml` + `centerline.csv`; per-station projection in `crates/outlap-qss/src/path.rs` |
| Flat-track (2-D) analysis mode | `sim.flat_track` (recorded in every result) |
| Racing line | `sim.raceline` (default `min_curvature` QP); recorded per result as `LineDescriptor` |
| Per-wheel and setup outputs | T1 channels in the result Dataset (`wheel` dim `FL/FR/RL/RR`; `understeer_gradient`, `aero_front_share`) — Chapter 10 |

With this vocabulary in place, three things follow. Chapter 3 gets outlap installed, and runs your
first lap. Chapter 4 formalizes the four inputs that you just met informally: vehicle, track,
conditions, and sim. And Chapters 7 to 9 reopen each topic in the physics, at full depth.


---

## 3. Installation and your first lap

*What you will learn: how to build outlap from source with `uv`, including the one environment
variable that makes it fast. How to verify the install. And how to solve your first simulated laps
— a Tesla Model 3 around Barcelona-Catalunya, and around the Nürburgring GP circuit — from about
twenty lines of Python. Along the way you will meet the loaded-model report. That is the answer of
outlap to the question every user of a simulation should ask: what did the tool assume?*

### 3.1 Prerequisites

outlap is a Rust core with a Python API, so you need both toolchains.

Everything below ran on Linux. macOS and Windows should work wherever Rust and `uv` do. But as of
v0.2, only Linux is exercised in CI.

| Requirement | Why | Where |
|---|---|---|
| Rust (stable) | `uv sync` compiles the `outlap-core` extension from the Rust workspace via maturin | [rustup.rs](https://rustup.rs) (this walkthrough used `rustc 1.96.1`) |
| `uv` | Manages the Python project, virtual environment, and the extension build | [docs.astral.sh/uv](https://docs.astral.sh/uv/) (used `uv 0.11.26`) |
| Python ≥ 3.12 | `requires-python = ">=3.12"` in `python/pyproject.toml`; `uv` can download one for you | (used Python 3.12.13) |
| git | Cloning; the reference data ships in the repository | — |

You install nothing else by hand. `uv` resolves the whole environment from `python/pyproject.toml`.
That includes building the Rust extension, which is an abi3-py312 wheel; see
`crates/outlap-py/pyproject.toml`. One build serves any Python 3.12 or later.

### 3.2 Clone and build

```bash
git clone https://github.com/KMoula30/outlap.git
cd outlap/python
MATURIN_PEP517_ARGS="--profile release" uv sync --group notebooks --extra tire-fit
```

Here is what each part does.

- `uv sync` creates `python/.venv`. It installs the runtime dependencies: numpy, xarray, pyarrow,
  h5py, jsonschema, pyyaml, and pydantic. And it **compiles the Rust extension** from
  `crates/outlap-py`. The first build takes a few minutes, while cargo compiles the workspace.

  The `[tool.uv] cache-keys` of the extension cover the sources of every crate. A later `uv sync`
  therefore rebuilds it automatically after any change to the Rust, instead of reinstalling a stale
  wheel.
- `MATURIN_PEP517_ARGS="--profile release"` makes maturin build the Rust code with the optimized
  release profile of cargo.

  This matters. Generating the performance envelope of a car is the cold step behind every lap
  solve; see Chapter 8. In the words of `.github/workflows/ci.yml`, which sets exactly this
  variable, it is "a seconds-scale cold step in release but minutes in debug".

  Without it, your first lap solve takes about a minute instead of seconds. For this chapter, a
  debug build measured 63 s.
- `--group notebooks` installs the dependency group for the notebooks: matplotlib, ipywidgets,
  ipykernel, nbclient, and nbformat. You need matplotlib for the plot below, and the group is
  everything required to run the tour in `notebooks/`.
- `--extra tire-fit` installs scipy, for the MF6.1 tire-fitting pipeline; see Chapter 11, on
  importers and tooling. It is optional today. But it is what CI installs, and it is one package.

A third extra, `--extra track-import`, adds what the OpenStreetMap track importer needs; see
Chapter 11.

> **Warning: the `uv sync` trap.** `uv sync` is *exact*. It removes any installed package that you
> did not ask for in that invocation.
>
> If you later run a plain `uv sync`, with no flags, it will **uninstall the notebooks group**.
> matplotlib and the Jupyter kernel vanish, and the plotting snippet below stops working.
>
> Always repeat the full command: `uv sync --group notebooks --extra tire-fit`.

### 3.3 Verify the install

Run these from the `python/` directory. `uv run` uses the project environment directly, so you do
not need to activate a venv:

```bash
uv run python -c "from outlap.core import DEFAULT_DS_M; print('outlap.core OK, default grid step', DEFAULT_DS_M, 'm')"
uv run python -m outlap.schemas --check
```

```text
outlap.core OK, default grid step 2.0 m
schema check OK: 8 schemas, 22 fixtures + 7 data files validated
```

The second command validates every committed JSON Schema, and all shipped reference data against
them. It is the same check that CI runs. Chapter 5 covers what those schemas are.

One note about the import model, before we start. The top-level `outlap` package is currently a
stub. The real user API lives in **`outlap.core`**, so always write `from outlap.core import ...`.
See Chapter 10, the Python API reference.

### 3.4 Your first lap

A *lap simulator* answers one question: given a complete description of a car and a track, how fast
can that car go around?

outlap solves this at selectable levels of fidelity, called **tiers**. `t0` treats the car as a
point mass, riding on a precomputed grip envelope. `t1` is the full quasi-steady-state (QSS) model,
which additionally resolves what each of the four tires is doing. Chapter 2 introduces the physics,
and Chapter 8 the solvers.

Tiers `t2` and `t3`, which are transient models, are not implemented yet. They raise a clear error
if requested.

We will drive the shipped **Tesla Model 3 RWD (HV variant)** around the shipped 3-D import of the
**Circuit de Barcelona-Catalunya**.

Two notes of honesty up front. Both come from `data/vehicles/tesla_model3_rwd/README.md`.

First, the chassis, mass, and aero of the car are spec-sheet values that are plausible for a
Model 3. But its powertrain is a *synthetic* stack of a drive unit and battery pack in the 800 V,
or HV, class. The production car does not have that. It is a documented study variant, chosen so
that the battery-voltage coupling of Chapter 9 is live on a road car.

Second, the repository contains **two Catalunyas**. `data/tracks/catalunya_osm` is the 3-D
reference, built from OpenStreetMap geometry and open elevation data. All the notebooks and the
validation work use it. `data/tracks/catalunya` is a flat 2-D variant from the TUMFTM set; see
Chapter 12. Use `catalunya_osm`, unless you specifically want the flat one.

Save this as `python/first_lap.py`. The `../data/...` paths assume that you run it from `python/`:

```python
# first_lap.py — run from the python/ directory:  uv run python first_lap.py
from outlap.core import Track, min_curvature, solve_lap_dataset

# 1. A track is a directory: track.yaml + centerline.csv.
track = Track.load("../data/tracks/catalunya_osm")
print(f"{track.name()}: {track.length():.1f} m, closed: {track.is_closed()}")

# 2. Generate a racing line inside the track corridor.
#    half_width_m is the car's half-width (a Model 3 is ~1.9 m wide).
line = min_curvature(track, half_width_m=0.95)

vehicle = "../data/vehicles/tesla_model3_rwd"

# 3. Solve the lap: first on the center line, then on the racing line (tier t0).
center = solve_lap_dataset(vehicle, track, tier="t0")
print(f"t0, center line: {center.attrs['lap_time_s']:7.2f} s")

racing = solve_lap_dataset(vehicle, line, tier="t0")
print(f"t0, racing line: {racing.attrs['lap_time_s']:7.2f} s")

# 4. Same lap at tier t1: adds per-wheel loads, slips, forces, setup metrics.
lap = solve_lap_dataset(vehicle, line, tier="t1")
print(f"t1, racing line: {lap.attrs['lap_time_s']:7.2f} s")

v_top = float(lap.v.max())
print(f"top speed: {v_top:.1f} m/s ({3.6 * v_top:.0f} km/h)")
```

```text
Circuit de Barcelona-Catalunya: 4677.8 m, closed: True
t0, center line:  153.46 s
t0, racing line:  148.94 s
t1, racing line:  148.94 s
top speed: 65.3 m/s (235 km/h)
```

Reading that from top to bottom:

- `Track.load` takes a *directory*, which holds a `track.yaml` plus its centerline CSV; see
  Chapter 5. A track in outlap is a 3-D ribbon: a center line with curvature, grade, and banking,
  plus a drivable width on either side.
- `min_curvature` computes a **racing line**. That is the path a good driver takes, cutting across
  the road's width to straighten the corners. It works by minimizing the curvature of the path,
  which is how sharply the path bends, within the track's corridor, shrunk by the car's half-width
  plus a safety margin. See Chapter 8.

  It returns a `Raceline`, which `solve_lap_dataset` accepts directly. Passing the `Track` itself
  instead drives the center line. Here the racing line is worth about **4.5 s**.
- `solve_lap_dataset` loads and validates the vehicle directory, solves the lap, and returns the
  results as a labelled `xarray.Dataset`; see Chapter 10. The lap time lives in
  `attrs["lap_time_s"]`.
- **t0 and t1 report the same lap time by design.** The speed profile ran on the grip envelope that
  t1 derived, in both cases. t1 then re-trims it station by station, for the per-wheel detail.
- Everything internal is SI: speeds in m/s, forces in N, and temperatures in K inside the core.
  RPM, °C, and km/h appear only at the boundaries of a file format or a display, as in the last
  line above.

About that pause on the first solve. Before the first lap of a given car, outlap generates its
**g-g-g-v envelope**. That is a table of the car's maximum acceleration in every direction —
braking, cornering, and combined — as a function of speed and of the local effective vertical
gravity on a 3-D road. See Chapter 8, Physics II.

With a release build this takes seconds, and the result is cached for the rest of the Python
process. That is why the second and third solves above were nearly instant. A new Python process
regenerates it once.

### 3.5 What you got back: the lap dataset

Everything the solver knows about the lap comes back in one `xarray.Dataset`. Add `print(lap)`
after the solve, and you get:

```text
<xarray.Dataset> Size: 614kB
Dimensions:              (s: 2399, wheel: 4)
Coordinates:
  * s                    (s) float64 19kB 0.0 2.0 4.0 ... 4.794e+03 4.796e+03
  * wheel                (wheel) <U2 32B 'FL' 'FR' 'RL' 'RR'
Data variables: (12/16)
    v                    (s) float64 19kB 18.23 18.94 19.45 ... 15.9 16.4 17.31
    ax                   (s) float64 19kB 6.646 4.901 5.107 ... 7.715 8.152
    ay                   (s) float64 19kB -6.497 -6.199 -5.296 ... -6.221 -7.21
    t                    (s) float64 19kB 0.0 0.1076 0.2118 ... 148.7 148.8
    x                    (s) float64 19kB 240.5 242.1 243.8 ... 237.4 238.9
    y                    (s) float64 19kB 559.9 558.8 557.7 ... 561.8 560.9
    ...                   ...
    force_long_n         (s, wheel) float64 77kB nan nan nan nan ... nan nan nan
    force_lat_n          (s, wheel) float64 77kB nan nan nan nan ... nan nan nan
    understeer_gradient  (s) float64 19kB 1.75e-05 -1.115e-06 ... 4.761e-05
    aero_front_share     (s) float64 19kB 0.5 0.5 0.5 0.5 ... 0.5 0.5 0.5 0.5
    state_of_charge      (s) float64 19kB 0.98 0.9799 0.9799 ... 0.897 0.8969
    machine_temp_c       (s) float64 19kB 20.0 20.13 20.3 ... 133.9 134.0 134.0
Attributes:
    lap_time_s:     148.93502759514865
    resolved_hash:  76c65d2ac0a28cf41fed5ab4a084aa4e24f8f287f1d29af4c05ce4c1d...
    tier:           t1
    fz_coupling:    one_step_lag
    flat_track:     0
    notes:          ('aero map `aero/none.parquet` not present — constant-aer...
```

The dimension `s` is **arc length**: distance along the driven line, in meters, with one row every
2 m. That spacing is the default grid step, `DEFAULT_DS_M = 2.0`.

Speed `v`, the accelerations `ax` and `ay`, and cumulative time `t` are point-mass channels. The
`(s, wheel)` variables are the per-wheel detail from t1, in the order FL, FR, RL, RR.

Signs follow ISO 8855: x forward, y left, z up. `ay` is therefore positive to the *left*, and the
negative values at the start of this lap are a right-hand corner.

Two things are worth noticing right away.

- The `nan` values in the per-wheel channels are honest. At a station where the four-wheel re-trim
  has no feasible solution, exactly on the grip limit, outlap records "don't know". It does not
  invent a number.

  On this lap that is about a quarter of the stations. This car spends a lot of its lap pressed
  hard against its envelope; see Chapter 8, Physics II.
- The `notes` attribute is the paper trail of the run. This lap has 11 entries, recording every
  simplification taken. One example: this car has no aero map over ride height, so a constant-aero
  fallback carried the lap. Nothing in outlap degrades silently.

  `resolved_hash` fingerprints the exact resolved vehicle that produced the result. And
  `fz_coupling` records a setting in the numerics, which Chapter 8 explains.

Now the classic first plot: speed against distance.

```python
import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(9, 3.5), constrained_layout=True)
ax.plot(lap.s, lap.v, linewidth=1.5)
ax.set_xlabel("distance s [m]")
ax.set_ylabel("speed v [m/s]")
ax.set_title(f"Tesla Model 3 RWD (HV variant), Catalunya t1 — {lap.attrs['lap_time_s']:.2f} s")
ax.grid(alpha=0.3)
fig.savefig("first_lap_speed.png", dpi=150)
```

You should see the sawtooth signature of every lap simulation. There are long climbs, where the car
accelerates, up to 65.3 m/s at the end of the main straight, near $s \approx 1300$ m. There are
cliffs, where it brakes. And there are valleys at the corner **apexes**, which is the slowest point
of each corner, down to 12.5 m/s at the slowest hairpin.

Chapter 2 explains why this shape — accelerate forward, brake backward — is the essence of
quasi-steady-state lap solving.

### 3.6 What did the loader assume? The loaded-model report

Before you trust any result, ask the model what it made up.

Every load of a vehicle produces a **loaded-model report**. It lists everything that was inherited
from a parent file, estimated by a documented heuristic, or degraded to a fallback.
`vehicle_report` returns it without solving:

```python
# report.py — what did the loader assume about this car?
from outlap.core import vehicle_report

report = vehicle_report("../data/vehicles/tesla_model3_rwd")
print(report["name"])
print("resolved_hash:", report["resolved_hash"][:16], "…")
for key in ("inherited", "estimated", "degraded", "warnings"):
    print(f"{key:9s}: {len(report[key])} entries")

print("\nfirst four estimated values:")
for pointer, detail in report["estimated"][:4]:
    print(f"  {pointer}")
    print(f"      {detail}")
```

```text
Tesla Model 3 RWD (HV variant)
resolved_hash: 76c65d2ac0a28cf4 …
inherited: 0 entries
estimated: 10 entries
degraded : 0 entries
warnings : 0 entries

first four estimated values:
  /suspension/front/static_ride_height_m
      assumed 30 mm nominal (only used by the ride-height aero map)
  /suspension/front/anti_dive
      assumed 0 (no anti-dive geometry)
  /suspension/front/anti_squat
      assumed 0 (no anti-squat geometry)
  /suspension/front/camber_map
      no camber map — assumed zero camber change with travel
```

The car loads *warning-clean*: zero warnings, and zero degraded values. But it carries ten
estimates, deliberately on record. Those are fields that the vehicle file omitted, and that the
load pipeline filled with a documented assumption.

This is the philosophy of the input quartet, from Chapter 4, in action. Chapter 12 walks through
the full provenance of this vehicle, parameter by parameter: which values are manufacturer spec,
which are estimates, and why the powertrain is synthetic.

### 3.7 The same car on a TUMFTM track

The data library ships 25 circuits, vendored from the racetrack-database of TUMFTM. It is
LGPL-3.0 data; see Chapter 12, on the shipped data library. The flat `catalunya` twin is among
them.

They hold real center lines and corridor widths, measured from satellite images. But they are
strictly flat: elevation, grade, and banking are all zero.

Same car, at the Nürburgring:

```python
# ring.py — the same car on a TUMFTM circuit (flat 2-D centerline data).
from outlap.core import Track, min_curvature, solve_lap_dataset

ring = Track.load("../data/tracks/nuerburgring")  # the GP-Strecke, not the Nordschleife
line = min_curvature(ring, half_width_m=0.95)
lap = solve_lap_dataset("../data/vehicles/tesla_model3_rwd", line, tier="t1")

print(f"{ring.name()}: {ring.length():.1f} m")
print(f"lap time:  {lap.attrs['lap_time_s']:.2f} s")
print(f"top speed: {float(lap.v.max()):.1f} m/s")

soc = lap.state_of_charge
print(f"battery state of charge: {float(soc[0]):.3f} -> {float(soc[-1]):.3f}")
print(f"peak winding temperature: {float(lap.machine_temp_c.max()):.1f} degC")
```

```text
Nürburgring GP: 5144.1 m
lap time:  154.60 s
top speed: 59.5 m/s
battery state of charge: 0.980 -> 0.892
peak winding temperature: 154.7 degC
```

An electric sedan of about 200 kW, lapping the 5.14 km GP circuit in about two and a half minutes,
is a plausible number.

The last two lines preview something bigger. This vehicle has a full stack for the battery and the
motor thermal state. Every lap therefore also integrates the **slow states**. The pack drained from
98 % to 89.2 % state of charge. The motor winding heated from 20 °C to a peak of 154.7 °C over the
lap. Chapter 9, Physics III, is entirely about this machinery.

> **Tip: a fast grid for experiments.** The cost of generating an envelope scales with its grid.
> When you are sweeping many variants, and you do not need numbers of final quality, shrink it.
>
> Use `sim={"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}`, which is the idiom
> that `python/tests/test_model3.py` uses. It generates in about a second, even on a debug build.
>
> It is coarser, and slightly conservative. The Catalunya t1 lap comes out at 154.97 s on the fast
> grid, against 148.94 s on the default grid of 40×25×7. So quote default-grid numbers when it
> matters.

### 3.8 Where to go next

You have built outlap. You have solved laps at both shipped tiers, on two circuits. You have
plotted a speed trace, and read the loaded-model report.

There are three directions from here.

- **The notebooks**, in `notebooks/`. Run them from `python/`, with
  `uv run --with jupyterlab jupyter lab ../notebooks/00_tour_of_outlap.ipynb`.

  `00_tour_of_outlap.ipynb` is the guided tour of everything you just did, with the F1 reference
  car. Notebooks `01` to `06` each deepen one topic: the car as data, tracks, racing lines, the t0
  solver, the MF6.1 tire model, and the powertrain firewall. `07_qss_t1.ipynb` is the t1 capstone,
  and it includes the sweep over drive-unit sizing on this very Model 3.

  They are committed with outputs, and CI re-executes them. What you see on GitHub is therefore
  what the code does.
- **Understanding the inputs.** Chapter 4 explains the four files you just used implicitly:
  vehicle, track, conditions, and sim. Chapter 5 explains their formats.
- **Understanding the physics.** Chapter 2 gives the vocabulary. Chapters 7 to 9 cover tires and
  aero, the lap solvers and the g-g-g-v envelope, and the slow states for powertrain, thermal, and
  battery that you glimpsed above.

If anything failed along the way, see Chapter 17, the FAQ and troubleshooting. And remember the two
traps from §3.2: a plain `uv sync` uninstalls the notebooks group, and a build on the debug profile
makes the first solve take minutes.


---

## 4. The input quartet: vehicle, track, conditions, sim

*What you will learn: exactly four inputs describe every outlap run — a vehicle, a track, session
conditions, and simulation settings — and why the strict separation between them is a design rule,
not a convention. You will walk through a real shipped `vehicle.yaml`, section by section. You will
learn how inheritance through `extends:` works, how what-if overrides work, and how estimation
heuristics work. And you will see what a configuration error actually looks like. By the end you
will be able to read, write, and debug all four files.*

### 4.1 Why four files?

outlap describes a simulation run with four separate documents. They are **the input quartet**:

| Input | File(s) | What it answers | Required? |
|---|---|---|---|
| **Vehicle** | `vehicle.yaml` (+ referenced `.ptm`/`.tyr`/`.emotor`/battery files) | *What car is this?* | yes |
| **Track** | `track.yaml` + `centerline.csv` | *Where is it driving?* | yes |
| **Conditions** | `conditions.yaml` | *What kind of day is it?* | no — defaults to a standard atmosphere |
| **Sim** | `sim.yaml` | *How should the solver run?* | no — every field has a default |

The separation is one of the hard rules of the project, stated in `CLAUDE.md`: **never mix car
identity with environment or numerics**.

A vehicle file must contain nothing about the weather. A conditions file must contain nothing about
the car. A sim file must contain nothing physical at all.

The payoff is composability. The same `vehicle.yaml` can lap any track, on any day, at any solver
fidelity. And any result can be reproduced by naming its four inputs.

This is also why every solver tier, T0 through T3 — the fidelity levels that Chapter 2 introduced —
evaluates the *same* parameter objects. There is no field anywhere in the quartet that only T1 can
see.

On disk, a vehicle is a directory. The loader is rooted at that directory. In Rust terms it is a
filesystem `SourceLoader`, which is `FsLoader` in `crates/outlap-schema/src/io.rs`. Every path
*inside* `vehicle.yaml` is therefore relative to that directory.

The optional `conditions.yaml` and `sim.yaml` sit next to `vehicle.yaml`, in the same directory.
The Python entry point reflects this directly; see Chapter 10, the Python API reference:

```python
from outlap.core import Track, solve_lap

track = Track.load("data/tracks/catalunya_osm")
lap = solve_lap("data/vehicles/tesla_model3_rwd", track)
```

A *missing* `conditions.yaml` or `sim.yaml` silently resolves to the documented defaults. A file
that is *present but malformed* is always an error. The loader never ignores a broken file; see
`solve_lap` in `crates/outlap-py/src/lib.rs`.

Every file in the quartet starts with a `schema:` line, of the form `<name>/<MAJOR>.<MINOR>`, for
example `vehicle/1.0`.

The name half exists so that feeding the wrong kind of document somewhere fails cleanly, with
"expected a `vehicle` document but found `tyr`", instead of half-deserializing into nonsense. A
loader accepts a file whose name and MAJOR match, and treats MINOR as informational; see
`crates/outlap-schema/src/version.rs`. §4.9 says more about versions.

### 4.2 `vehicle.yaml`: the anatomy of a car

The vehicle document is the centerpiece of the quartet.

Here is a real one, shipped in the repository: the Tesla Model 3 RWD reference car, at
`data/vehicles/tesla_model3_rwd/vehicle.yaml`. Note that this is an "HV variant" study, whose
powertrain is deliberately synthetic. See Chapter 12, on the shipped data library, for the full
story of its provenance.

```yaml
schema: vehicle/1.0
name: "Tesla Model 3 RWD (HV variant)"
chassis:
  mass_kg: 1765.0
  cg: [1.524, 0.0, 0.45]
  inertia: [560.0, 2800.0, 3200.0]
  wheelbase_m: 2.875
  track_m: [1.58, 1.58]
aero:
  # Constant road-car aero (the degenerate non-mapped case). No ride-height/yaw map is shipped —
  # the placeholder path is deliberately absent (the fixture idiom), so the constant block carries.
  map: aero/none.parquet
  axes: []
  constant:
    cx_a_m2: 0.51
    cz_front_a_m2: 0.0
    cz_rear_a_m2: 0.0
suspension:
  model: lumped_kc
  front:
    ride_rate_n_per_m: 38000.0
    roll_stiffness_share: 0.58
    roll_center_height_m: 0.06
  rear:
    ride_rate_n_per_m: 45000.0
    roll_stiffness_share: 0.42
    roll_center_height_m: 0.12
tires:
  front: tyr/road.tyr.yaml
  rear: tyr/road.tyr.yaml
drivetrain:
  units:
    - source: ptm/du_medium.ptm.yaml
      thermal: emotor/rear_du.emotor.yaml
      path:
        - diff: { type: open }
      wheels: [RL, RR]
battery:
  model: rc_pairs
  params: battery/pack_800v.battery.yaml
brakes:
  balance_bar: 0.62
  abs: true
  disc:
    front:
      thermal_capacity_j_per_k: 26000.0
      cooling_area_m2: 0.07
    rear:
      thermal_capacity_j_per_k: 20000.0
      cooling_area_m2: 0.05
  regen_blend:
    max_regen_frac: 0.6
```

Eight top-level sections are **required**: `schema`, `name`, `chassis`, `aero`, `suspension`,
`tires`, `drivetrain`, and `brakes`.

Three are optional. `extends` gives inheritance (§4.3). `ers` and `battery` are whole subsystems
that a car may simply not have. There is also an `extensions` slot, for vendor keys (§4.5).

The Rust type behind the document is `Vehicle`, in `crates/outlap-schema/src/vehicle/mod.rs`.

Let us take the sections one at a time. All values are SI internally — meters, kilograms, newtons,
N·m — and the exceptions at a display boundary are called out where they occur.

#### 4.2.1 `chassis`: mass, geometry, and inertia

The chassis block carries the bulk properties, expressed in the ISO 8855 body frame. That is
**x forward, y left, z up**, the axis convention used throughout outlap.

- `mass_kg` is the total mass, sprung plus unsprung. That is the body *and* the wheels together;
  the quasi-static tiers of outlap do not split them.
- `cg: [x, y, z]` is the center of gravity, in meters. That is the point where the weight of the
  car effectively acts.

  In the shipped files, the x entry is the longitudinal distance from the front axle to the CG,
  which is the classic *a* dimension. The `1.524` of the Model 3 is 0.53 times its 2.875 m
  wheelbase, which encodes a front-to-rear weight split of about 47 to 53.

  The z entry is the CG height. Here it is 0.45 m, which is low, thanks to the battery pack mounted
  in the floor.
- `inertia: [Ixx, Iyy, Izz]` holds the diagonal moments of inertia, in kg·m². They are the
  resistance to rolling, to pitching, and to yawing, respectively.

  Products of inertia are deferred to a future additive field in the schema; see
  `crates/outlap-schema/src/vehicle/chassis.rs`. The QSS tiers shipped in v0.2 do not consume these
  values. They matter from the transient tiers onward.
- `wheelbase_m` is the distance between the axles. `track_m: [front, rear]` is the distance between
  the left and right wheel centers, at each axle.

#### 4.2.2 `aero`: a map, a constant block, or both

Aerodynamic force in outlap is expressed as an *area*.

`cx_a_m2` is the drag area $C_x A$, in m². It is the drag coefficient times the frontal area, and
the drag force is $\tfrac{1}{2}\rho\,C_xA\,v^2$, where $\rho$ is air density and $v$ is speed.

`cz_front_a_m2` and `cz_rear_a_m2` are the downforce areas, one for each axle.

The physics is Chapter 7, Physics I. Here we only care about the shape of the file; see
`crates/outlap-schema/src/vehicle/aero.rs`.

- `map:` is required. It references a gridded aero map, which is a parquet sidecar table; see
  Chapter 5, on files and formats.
- `axes:` is required, and it may be empty. It gives the ordered names of the map's input axes.

  Only known names are accepted: `ride_height_f_mm`, `ride_height_r_mm`, `ride_height_mm`,
  `yaw_deg`, `roll_deg`, `steer_deg`, `drs_flag`, and `speed_mps`. A typo gets a did-you-mean
  error.

  Note the `_mm` and `_deg` suffixes. The axes of a map are one of the deliberate exceptions to the
  unit rule, at a display boundary.
- `constant:` is optional. It is the degenerate case, for a car whose aero does not vary with
  attitude.

The F1 reference car uses both. It has a four-axis map for the T1 tier, plus a constant fallback;
see `data/vehicles/f1_2026/vehicle.yaml`:

```yaml
aero:
  map: aero/f1_2026.parquet
  axes: [ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag]
  constant:
    cx_a_m2: 1.25
    cz_front_a_m2: 1.9
    cz_rear_a_m2: 2.6
```

The Model 3 instead points `map:` at `aero/none.parquet`, a file that deliberately does not exist.

This is the documented "fixture idiom". A sidecar table is decoded later, at assembly time. An
absent one is skipped, with a note in the result, which lets the `constant:` block carry the whole
model. A road car with zero lift is exactly this degenerate case.

#### 4.2.3 `suspension`: lumped K&C

`model: lumped_kc` selects the only suspension model in v1: **lumped kinematics and compliance**,
or K&C. Instead of modeling every link and bushing, it summarizes each axle by a handful of
effective rates; see `crates/outlap-schema/src/vehicle/suspension.rs`.

For each axle, in the `front:` and `rear:` blocks, of type `AxleKc`:

| Field | Meaning | Required? |
|---|---|---|
| `ride_rate_n_per_m` | vertical stiffness at the wheel, N/m (how hard the wheel pushes back per metre of compression) | yes, > 0 |
| `roll_stiffness_share` | this axle's fraction of the car's total roll stiffness, 0..1 — it steers how lateral load transfer splits front/rear (Chapter 2) | yes |
| `roll_center_height_m` | height of the axle's roll centre, m | yes |
| `static_ride_height_m` | design ride height at rest, m — the platform the T1 aero-map equilibrium compresses under downforce | estimable |
| `anti_dive` / `anti_squat` | geometric anti-pitch fractions | estimable → 0 |
| `camber_map` / `toe_map` | wheel-angle-vs-travel map references | estimable → identity |

"Estimable" means this: if you omit the field, the load pipeline fills it from a documented
heuristic, and *tells you so* in the loaded-model report (§4.4).

The Model 3 file above omits all five estimable fields. The F1 car sets `static_ride_height_m` to
`0.040` and `0.090` explicitly, because its aero map over ride height actually consumes them.

#### 4.2.4 `tires`: two references, and no defaults

```yaml
tires:
  front: tyr/road.tyr.yaml
  rear: tyr/road.tyr.yaml
```

Both axles must reference a `.tyr` tire document. Tires carry the physics, and they have "no sane
default"; see `crates/outlap-schema/src/vehicle/tires.rs`.

The `.tyr` format itself, holding the Magic Formula coefficients and the thermal and wear blocks,
is Chapter 5. The physics is Chapter 7.

The referenced files are loaded and validated as part of loading the vehicle. A broken tire file
therefore fails the vehicle load. It does not fail the lap solve.

#### 4.2.5 `drivetrain`: a topology graph, not a picker of layouts

This is the versatility surface of outlap. There is no `layout: rwd` enum.

The powertrain is instead a **directed graph**. Torque *sources* connect through ordered *coupler*
elements to wheel *sinks*; see `crates/outlap-schema/src/vehicle/drivetrain.rs`. Any concept with
four wheels is a topology plus data.

- Each entry in `units:` is one torque source. It has a `source:`, which references a `.ptm`
  powertrain map — the firewall rule of outlap means that an engine or a motor is always consumed
  as a map file, either measured or estimated, and never modeled internally; see Chapter 9. It has
  an optional `thermal:`, which is an `.emotor` machine-thermal model, for an electric machine
  only. It has a `path:` of couplers. And it has the `wheels:` it drives, named `FL`, `FR`, `RL`,
  and `RR`, in uppercase on the wire.
- A coupler is externally tagged YAML: `{gearbox: {...}}`, `{diff: {...}}`, or
  `{fixed_ratio: 2.4}`.
- A `gearbox` has `ratios:`, where index 0 is first gear, plus `final_drive`, `shift_time_s`, and
  an `efficiency`. That efficiency defaults to a constant 0.985. It can also be a `{map: ...}`
  reference.
- A `diff`, which is a differential, is the device that lets the left and right wheels turn at
  different speeds. Its `type:` is one of `open`, `locked`, `lsd`, or `solid`.

  For `lsd`, which is limited-slip, and for `locked`, the field `preload_nm` is conditionally
  required. The field `ramp: [accel, decel]` applies to an LSD only.
- A defaulted `control:` block carries the static torque splits and a flag for torque vectoring.
  That control law is `ΔM_z = k_yaw · (r_target − r)`, which is yaw-moment control, and the control
  layer of the T2 tier executes it live; see Chapter 8.

The Model 3 is the simplest real topology: one drive unit, through an open differential, to the
rear wheels.

The F1 car shows a fuller one; see `data/vehicles/f1_2026/vehicle.yaml`:

```yaml
drivetrain:
  units:
    - source: ptm/ice_v6.ptm.yaml
      path:
        - gearbox:
            ratios: [2.9, 2.2, 1.8, 1.5, 1.28, 1.1, 0.98, 0.86]
            final_drive: 3.1
            shift_time_s: 0.02
        - diff: { type: lsd, preload_nm: 90.0, ramp: [45.0, 70.0] }
      wheels: [RL, RR]
```

The private `path` of a unit carries its differential only. A reduction inside the machine, from
machine to output shaft, is the unit's `fixed_ratio:` field. A gearbox lives on the shared graph,
in `couplers`. That is one of the load checks of §4.5.

#### 4.2.6 `ers`: the block for hybrid energy recovery (optional)

The F1 car carries the full block. It has `mgu_k:`, which is a `.ptm` for the motor-generator unit.
It has `es:`, which gives the capacity of the energy store in MJ, plus an allowed window on state
of charge. It has `deployment:`, which gives a power limit in kW, with a table of taper against
speed. It has an optional `override_mode:`. And it has `recovery:` limits.

Two fields are estimable. `deployment.per_lap_deploy_mj` defaults to the full usable capacity.
`override_mode.extra_energy_per_lap_mj` defaults to 0.

At v0.2.5, the ERS is enforced as a power cap. The energy manager, which governs deployment and
harvest for each lap, is future work. See Chapter 9, Physics III, and Chapter 15, on limitations
and the roadmap.

#### 4.2.7 `battery`: a selector plus a reference

```yaml
battery:
  model: rc_pairs
  params: battery/pack_800v.battery.yaml
```

`model: rc_pairs` is the only variant. It is a Thevenin equivalent-circuit model; see Chapter 9.
`params:` points at a separate `battery/1.0` document.

One note of honesty. The load pipeline of the vehicle validates the referenced tires, the ERS
machine, and the `.ptm` and `.emotor` files of the drive units. It does **not** validate
`battery.params`, nor the aero map. Those are resolved only at assembly time; see `load_referenced`
in `crates/outlap-schema/src/load/mod.rs`.

The shipped `f1_2026` car actually references a `battery/f1_es.yaml` that does not exist in its
directory. The vehicle loads and solves fine, with a note that the stack of slow states is inert.

#### 4.2.8 `brakes`: balance, discs, and regeneration

- `balance_bar` is the front brake bias, as a fraction from 0 to 1. A value of 0.62 sends 62 % of
  the brake torque to the front axle.
- `abs:` says whether an anti-lock system is fitted. It defaults to `false`.
- `disc.front` and `disc.rear` give, for each axle, the thermal capacity of the disc in J/K, the
  cooling area in m², and an optional map of pad friction against temperature.
- `regen_blend:` is optional, and it applies to a car that recovers braking energy.
  `max_regen_frac` caps the fraction of total brake torque that regenerative braking supplies, with
  the motor acting as a generator. An optional `front_bias` defaults to the friction balance.

### 4.3 Inheritance, merging, and what-if overrides

#### `extends:` — inheritance from a single parent

A vehicle can inherit from a preset, which is a partial vehicle fragment, and override only what
differs. The mechanism lives in `crates/outlap-schema/src/load/merge.rs`.

- **Only single-parent chains are allowed.** `extends:` names one parent, which may itself extend
  another. A cycle is detected and rejected, with "`extends` cycle detected: ... is already in the
  chain".

  The anchors, aliases, and `<<` merge keys of YAML itself are deliberately *rejected* at parse
  time. Inheritance goes through `extends:` and through nothing else, so that provenance stays
  traceable.
- **A mapping merges key by key. A sequence or a scalar replaces wholesale**, and the child wins.
  Overriding `chassis.mass_kg` therefore keeps the rest of `chassis`. But touching
  `drivetrain.units` replaces the entire list.
- The reference is loaded verbatim. There is a fallback to a `.yaml` extension, only when the
  reference contains no dot. `extends: presets/formula_base` therefore finds
  `presets/formula_base.yaml`.
- The resolved model has `extends` stripped away. Every value remembers where it came from, in a
  provenance map from JSON pointers to origins: the base file, inherited from a preset, an
  override, or estimated.

From the test fixtures, at `crates/outlap-schema/tests/fixtures/ev_child/vehicle.yaml`:

```yaml
schema: vehicle/1.0
extends: presets/ev_base.yaml
name: "EV child — lightweight"
chassis:
  mass_kg: 1590.0   # overrides the preset's 1700.0; other chassis fields inherited
```

One caveat for v0.2. The mechanism is fully implemented and tested. But the shipped
`data/presets/` directory is currently **empty**. The class presets that the roadmap promises — for
formula, GT, and passenger cars — have not landed yet. A preset today exists only as a test
fixture.

#### Dotted-path overrides: the what-if API

A programmatic override never edits a file. You pass dotted paths. They are applied *after* the
merge and *before* validation, so an overridden value goes through exactly the same checks as a
hand-written one:

```python
lap = solve_lap(
    "data/vehicles/tesla_model3_rwd", track, tier="t1",
    overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"},
)
```

That is the swap of drive-unit sizing from the shipped notebook 07. No YAML was touched. The
applied override is recorded in the provenance map, and it is reflected in the resolved hash
(§4.4).

A numeric segment indexes into an **existing** list element. An override never grows a list. An
index out of bounds is a real error:

```text
ValueError: override index `3` is out of bounds (sequence has 1 items)
```

### 4.4 Estimation, and the loaded-model report: nothing is silent

When you omit an estimable field, outlap fills it from a documented heuristic; see
`crates/outlap-schema/src/load/estimate.rs`. And it *always* tells you.

Here are the current heuristics:

| Field (JSON pointer) | Heuristic | Filled value |
|---|---|---|
| `/suspension/{front,rear}/static_ride_height_m` | `static_ride_height_nominal` | 0.030 m front / 0.050 m rear |
| `/suspension/*/anti_dive`, `anti_squat` | `anti_dive_zero` / `anti_squat_zero` | 0.0 |
| `/suspension/*/camber_map`, `toe_map` | `camber_identity` / `toe_identity` | none installed — report-only ("assumed zero change with travel") |
| `/ers/deployment/per_lap_deploy_mj` | `per_lap_deploy_capacity` | = the store's `capacity_mj` |
| `/ers/override_mode/extra_energy_per_lap_mj` | `override_extra_energy_zero` | 0.0 |

Every load produces a **loaded-model report**, which is `LoadedModelReport` in
`crates/outlap-schema/src/load/report.rs`. It holds four lists — `inherited`, `estimated`,
`degraded`, and `warnings` — plus `resolved_hash`.

That hash is a blake3 hash of the canonical, key-sorted, resolved parameter set. Results embed it,
and the envelope cache is keyed on it.

From Python:

```python
from outlap.core import vehicle_report
r = vehicle_report("data/vehicles/tesla_model3_rwd")
```

Here is the real output for the shipped Model 3. It is warning-clean, with ten estimated entries:

```text
name: Tesla Model 3 RWD (HV variant)
resolved_hash: 76c65d2ac0a28cf4...
inherited: []   degraded: []   warnings: []
estimated:
  /suspension/front/static_ride_height_m -> assumed 30 mm nominal (only used by the ride-height aero map)
  /suspension/front/anti_dive            -> assumed 0 (no anti-dive geometry)
  /suspension/front/anti_squat           -> assumed 0 (no anti-squat geometry)
  /suspension/front/camber_map           -> no camber map — assumed zero camber change with travel
  /suspension/front/toe_map              -> no toe map — assumed zero toe change with travel
  ... (and the same five for /suspension/rear, at 50 mm)
```

Two honest footnotes.

First, `allow_degraded: true` in `sim.yaml` is the project's *single* documented fallback path. It
permits combinations that have a documented fallback, and it marks the results. It is threaded from
the sim settings into solver assembly.

Second, at v0.2 the `degraded` list at load time is a placeholder at the level of the contract. No
degraded combination is populated during loading yet. `crates/outlap-schema/src/load/mod.rs`
carries the literal note "`allow_degraded` recorded here once degraded combos exist". You will
therefore see the effects of the flag only at assembly and solve time.

### 4.5 Validation, and the error experience

Loading a vehicle is a staged pipeline, in `crates/outlap-schema/src/load/`. The stages are: parse,
preserving spans; the version gate; the extends-merge, plus overrides and provenance; the walk over
unknown keys; one deserialize after the merge; the semantic checks; the loads of referenced files;
the checks on the topology graph; estimation; and the report.

A configuration error is treated as a product surface. The typed error is `SchemaError`, in
`crates/outlap-schema/src/error.rs`. It has one variant for each stage. Each variant carries the
offending file and a byte span, so that miette, the Rust diagnostics library, can render an
underlined message in plain language.

By project rule, a bare serde error reaching you is a bug.

**An unknown field is a hard error**, except for an `x-*` extension key. Those are carried through
uninterpreted, and each produces a warning in the report: "extension key `x-...` carried through
(not interpreted)".

The walk over unknown keys checks your document against the generated JSON Schema, and attaches a
suggestion based on Levenshtein distance. Misspell `chassis:` as `chasis:` in a copy of the
Model 3 file, and the Python surface gives you:

```text
ValueError: unknown field `chasis`
help: did you mean `chassis`?
```

A Rust consumer gets the full miette rendering, with the file name and the key underlined. The
Python boundary flattens that to the message plus the help line, and it maps a missing file to
`FileNotFoundError` instead.

If your file declares a schema MINOR newer than the build understands, the unknown-key error gains
a hint: the key may be a field added in a newer schema version.

**The semantic checks** run on the typed model. They cover positivity, for masses, wheelbase, ride
rates, and `dt_s`. They cover unit intervals, for `balance_bar`, `roll_stiffness_share`, and
`max_regen_frac`. They cover known aero axis names, and ascending arrays for a taper or an SoC
window. And they cover conditional requirements: an `lsd` differential without `preload_nm` fails,
with the help text "add `preload_nm: <N·m>` to this diff". There are more; see
`crates/outlap-schema/src/load/semantic.rs`.

For example, setting `roll_stiffness_share: 1.58` gives:

```text
ValueError: `suspension.front.roll_stiffness_share` must lie in [0, 1]
```

**The topology checks** validate the drivetrain graph as a whole, in
`crates/outlap-schema/src/load/topology.rs`. Each check gives a message in plain language, and one
or more labelled spans.

1. There must be at least one drive unit.
2. Every unit must drive at least one wheel, with no duplicates.
3. The private `path` of a unit may carry only its differential. A `fixed_ratio` there is rejected;
   declare it as the unit's `fixed_ratio:` field. A gearbox there is rejected; move it to the
   shared graph, in `couplers`.
4. A wheel driven rigidly by two or more units, with no differential anywhere in the driving paths,
   is rejected. The message says that it "over-constrains the wheel speed". A parallel hybrid that
   shares a differential passes.
5. Torque vectoring cannot be enabled across a `locked` or `solid` differential that feeds a full
   axle.

Finally, two rules of YAML strictness are worth internalizing. A duplicate key is a hard parse
error. And anchors, aliases, and `<<` merge keys are rejected; use `extends:`.

### 4.6 The track: `track.yaml` plus `centerline.csv`

A track is a directory. It holds a thin descriptor, plus the geometry data.

The descriptor is `TrackDoc`, in `crates/outlap-schema/src/track.rs`. Here it is for the reference
Catalunya, at `data/tracks/catalunya_osm/track.yaml`:

```yaml
schema: track/1.0
name: Circuit de Barcelona-Catalunya
closed: true
centerline: centerline.csv
meta:
  source: osm+dem
  dem: eudem25m
  accuracy_class: B
  attribution: "© OpenStreetMap contributors (ODbL); elevation eudem25m via opentopodata.org"
  notes: "widths defaulted; banking not resolved from DEM (add keypoints to refine)"
```

- `closed` defaults to **true**. A closed loop gets a periodic spline, and a check on closure. A
  point-to-point course must opt out explicitly.
- `banking_keypoints:` is optional. It is a sparse list of `{s_m, banking_deg}` pairs, interpolated
  along arc length. When present, they **override** the banking column of the centerline. `s_m`
  must be non-negative, and strictly ascending.
- `meta` carries provenance. It holds the `source`; the `dem`, which is the digital elevation model
  used to fuse elevation; an `accuracy_class`, where `A` means surveyed, `B` means fused from a
  DEM, and `C` means estimated; an `attribution` string, for redistribution; and free-form `notes`.

The geometry lives in `centerline.csv`. It has eight required columns, named in a header, in any
order: `s_m, x_m, y_m, z_m, banking_deg, width_left_m, width_right_m, grip_scale`.

`s_m` is arc length along the centerline. `x`, `y`, and `z` are 3D coordinates. The widths give the
drivable corridor on either side. `grip_scale` scales friction locally.

Lines that start with `#`, and blank lines, are skipped.

Validation gives 1-based line numbers, and a did-you-mean on a missing column. It checks that `s_m`
increases strictly, that the coordinates are finite, and that the widths and the grip are positive;
see `crates/outlap-schema/src/centerline.rs`.

The `outlap-track` crate builds the actual spline fit, the curvature, the grade, and the road
frame, downstream.

The repository ships 27 circuits. There are 25 flat 2-D circuits, vendored from the TUMFTM
racetrack-database, which is LGPL-3.0 data with `accuracy_class: C`. And there are two 3D imports
from OSM and a DEM: `catalunya_osm`, the reference that all the notebooks and the validation
cross-check use, and `spa_osm`, the elevation showcase, with about 100 m of climb.

Be careful. `catalunya` and `catalunya_osm`, and `spa` and `spa_osm`, are the *same circuits from
two sources*: flat TUMFTM against 3D OSM and DEM. Chapter 12 gives the full inventory. Chapter 11
covers the importers.

### 4.7 `conditions.yaml`: the same track, on a different day

Conditions capture the session environment. They never capture car identity, and they never capture
numerics.

Every field has a full ISA default, so the entire file is optional. ISA is the International
Standard Atmosphere, which is 20 °C and 1013.25 hPa here, with still air. See
`crates/outlap-schema/src/conditions.rs`.

Here are the fields. Note the deliberate use of °C and hPa at the display boundary.

| Field | Default | Meaning |
|---|---|---|
| `air.temperature_c` | 20.0 | air temperature, °C — with pressure, sets air density for aero |
| `air.pressure_hpa` | 1013.25 | absolute pressure, hPa (> 0) |
| `wind.speed_mps` | 0.0 | constant wind speed, m/s (≥ 0; a single vector in v1) |
| `wind.direction_deg` | 0.0 | meteorological convention — the direction the wind blows *from*: 0 = North, 90 = East |
| `track_surface_c` | 20.0 | track surface temperature, °C — the tire thermal boundary $T_\text{road}$ (consumed once the tire thermal model lands; Chapter 15) |
| `ambient_c` | 20.0 | thermal-model ambient, °C — consumed by the `.emotor` machine-thermal network unless the emotor's own `cooling.ambient_fixed_c` overrides it |

Here is a complete example, from the test fixtures at
`crates/outlap-schema/tests/fixtures/conditions/hot_dry.conditions.yaml`:

```yaml
schema: conditions/1.0
air:
  temperature_c: 28.0
  pressure_hpa: 1005.0
wind:
  speed_mps: 3.5
  direction_deg: 240.0
track_surface_c: 41.0
ambient_c: 28.0
```

Air density comes from the ideal-gas law, $\rho = p/(R\,T)$, with
$R = 287.05\ \mathrm{J\,kg^{-1}\,K^{-1}}$.

One shipped vehicle has its own conditions file, and it uses exactly this.
`data/vehicles/limebeer_2014_f1/conditions.yaml` sets 21.0 °C at 1013.25 hPa, so that
$101325/(287.05 \times 294.15) = 1.2000\ \mathrm{kg\,m^{-3}}$. That reproduces the air density
published in Perantoni & Limebeer (2014), for the validation cross-check; see Chapter 13.

From Python you can patch conditions for one call, without a file. Write
`solve_lap(..., conditions={"air": {"temperature_c": 35.0}})`. It deep-merges onto the file and the
defaults, and it rejects an unknown key loudly.

### 4.8 `sim.yaml`: numerics and solver settings

The sim document configures *how* to solve. It never configures *what* is being solved.

Every field is defaulted. The **resolved** settings are embedded in every result artifact, so a
result always records how it was produced.

Here is the core field set, from `crates/outlap-schema/src/sim.rs`, shown as the committed fixture
`crates/outlap-schema/tests/fixtures/sim/qss.sim.yaml`. The knobs for the transient tier,
`slow_decimation` and `fixed_point`, take their defaults and are not repeated in it; §10.4.4 lists
them.

```yaml
schema: sim/1.1
tier: t1
dt_s: 0.001
fz_coupling: one_step_lag
integrator: heun
envelope:
  v_points: 40
  ax_points: 25
  g_normal_points: 7
raceline:
  generator: min_curvature
allow_degraded: false
flat_track: false
```

Field by field:

- **`tier`** selects the solver fidelity. `t0` is a point mass, with constant friction and a power
  cap. `t1` is the default: a quasi-steady-state lap on the g-g-g-v envelope. `t2` is the
  closed-loop transient double-track model, which is indexed by time and therefore returns through
  its own entry point. `t3` is the 14-DOF model with suspension.

  The same vehicle description drives every tier. That is hard rule #4.

  At v0.2.5, `t3` raises a typed "not implemented" error. See Chapter 8, Physics II, for what each
  shipped tier actually computes.
- **`dt_s`**, which defaults to 0.001 s, and **`integrator`**, which is `heun`, an explicit
  trapezoidal scheme of 2nd order, or `rk4`. They give the fixed step and the scheme for the split
  integrator of the transient T2 tier. The step is fixed only, by the rules on determinism.
- **`fz_coupling`** is unset by default, and resolved for each tier: `one_step_lag` for T0 and T1,
  and `fixed_point` for T2. The resolved value is recorded on every result.

  It sets the mode of the algebraic loop on vertical load. Tire forces depend on vertical loads,
  which depend on load transfer, which depends on the very accelerations that the tire forces
  produce. That is an algebraic loop.

  `one_step_lag` breaks it, by using the normal loads from the previous step. `fixed_point` instead
  iterates a damped fixed point to convergence, within the step.

  Both are deterministic. The choice is a recorded simulation setting, and a property test pins
  that the two agree at convergence. The physics details are in Chapter 8.
- **`envelope`** gives the sampling resolution of the g-g-g-v performance envelope, which the QSS
  tiers precompute. The default is 40 speed points, 25 longitudinal-acceleration points, and 7
  normal-g points.

  The semantic floors are `v_points ≥ 2`, `ax_points ≥ 2`, and `g_normal_points ≥ 1`.

  A coarse grid such as `{"v_points": 8, "ax_points": 7, "g_normal_points": 2}` is the idiom that
  the shipped notebooks use, for a cheap parameter sweep.
- **`raceline`** takes exactly one of two things. `generator:` is either `min_curvature` or
  `time_weighted`, both quadratic programs over lateral offset, minimizing $\int \kappa^2\,ds$ on
  the 3D ribbon. `file:` gives your own line, as a CSV indexed by s.

  Setting both, or neither, is a semantic error.
- **`allow_degraded`** defaults to `false`. It is the single documented escape hatch to a fallback,
  and degradations are recorded in the result metadata (§4.4).
- **`flat_track`** defaults to `false`, and was added in `sim/1.1`. It zeroes the track's grade,
  banking, and vertical curvature, so that the 3D envelope collapses to a flat g-g diagram.

  This is the 2-D mode for comparison against an oracle, which the Limebeer validation cross-check
  uses; see Chapter 13. The physical track file is left untouched, and the flag is recorded in the
  results.

No vehicle in `data/vehicles/` ships a `sim.yaml`. Defaults are the norm.

Note one asymmetry. In a *file*, the `schema: sim/1.1` line is required, as it is in every outlap
document. But the in-memory default fabricates it for you, when no file exists.

From Python, `sim={"flat_track": True, "envelope": {"v_points": 24}}` deep-merges onto the file and
the defaults. An unknown key is rejected, with
``unknown sim field `sim.x` (known fields here: ...)``. The convenience argument `tier="t0"` wins
over both.

### 4.9 Schema versions, and the published JSON Schemas

Eight kinds of document make up the wire contract: `vehicle`, `ptm`, `tyr`, `emotor`, `battery`,
`track`, `conditions`, and `sim`. See `crates/outlap-schema/src/lib.rs`.

The contract is a semver boundary. **An additive change bumps MINOR. Anything else bumps MAJOR, and
requires a migration**, through `outlap migrate`.

This build accepts `SCHEMA_MAJOR = 1` for every document, and it understands minors up to the
crate-global `SCHEMA_MINOR = 4`.

The history of bumps so far is all additive: `tyr/1.1` added the brush tire block. `vehicle/1.2`
added `static_ride_height_m`. `ptm/1.1` and `battery/1.0` added the DC-voltage axis and the pack
document. `sim/1.1` added `flat_track`.

Most shipped data still declares `x/1.0`, which is legal. A loader gates on the name and the MAJOR
only.

The machine-readable schemas live in `schemas/`: `vehicle.json`, `ptm.json`, `tyr.json`,
`emotor.json`, `battery.json`, `track.json`, `conditions.json`, and `sim.json`. They are JSON
Schema draft 2020-12.

They are licensed **Apache-2.0**, unlike the AGPL-3.0 code. Any tool can therefore validate outlap
files, with no entanglement over licensing.

They are generated *from* the Rust types, with
`cargo run -p outlap-schema --bin gen_schemas`. CI fails if the committed files drift from the
code, through `--check`. On the Python side, `python -m outlap.schemas --check` validates the
shipped data against them.

Chapter 5 continues from here, into the referenced file formats themselves: `.ptm`, `.tyr`,
`.emotor`, battery packs, and the convention for a parquet sidecar.


---

## 5. Files and formats: schemas, maps, and tables

*What you will learn: every file that outlap reads or writes — what it contains, how it is
versioned, and how to write one by hand. We walk through the schema contract that keeps files
stable across releases. We then take each format in turn: powertrain maps (`.ptm`), tires
(`.tyr`), machine thermal networks (`.emotor`), battery packs, tracks, and aero maps. We finish
with the binary parquet sidecars, and the single interpolation policy that every gridded table in
outlap shares.*

Chapter 4 explained *which* four documents describe a simulation — vehicle, track, conditions, and
sim — and how they are loaded and validated. This chapter is the reference for the files
themselves: the ones you will open in an editor, and the ones a vehicle document points at.

### 5.1 The schema contract: versioned, generated, and checked

Every YAML document in outlap begins with a `schema:` line, naming its kind and its version:

```yaml
schema: vehicle/1.0
```

The format is `<name>/<MAJOR>.<MINOR>`. `SchemaVersion` in `crates/outlap-schema/src/version.rs`
parses it, and the name must be lowercase `[a-z_]+`.

Eight kinds of document exist: `vehicle`, `ptm`, `tyr`, `emotor`, `battery`, `track`, `conditions`,
and `sim`. See `crates/outlap-schema/src/lib.rs`.

The name half is a safety net. Feed a `.tyr` file where a vehicle is expected, and it fails the
version gate with a clear message. It does not half-deserialize into nonsense.

The two version halves follow semantic versioning, or semver. That is the convention in which the
meaning of a version number is a promise about compatibility.

- **A loader gates on the name and the MAJOR only.** A file is accepted if its name and MAJOR match
  what the loader expects; see `SchemaVersion::is_compatible_with`. MINOR is informational, so a
  `vehicle/1.0` file loads fine in a build that understands `vehicle/1.2`.
- **An additive change bumps MINOR.** A new optional field never breaks an old file.

  The counter for the whole crate is `SCHEMA_MINOR = 4`, in `crates/outlap-schema/src/lib.rs`. Its
  doc comment logs the history. 1 was the `tyr/1.1` brush block. 2 was `static_ride_height_m` in
  the suspension, at `vehicle/1.2`. 3 was the Vdc axis of `ptm/1.1`, plus the new `battery/1.0`
  document. 4 was the `flat_track` flag of `sim/1.1`.
- **Anything else bumps MAJOR**, and requires a migration. A file with the wrong MAJOR is rejected,
  with the help text "run `outlap migrate` to update the file"; see
  `crates/outlap-schema/src/load/mod.rs`.

One consequence is worth knowing. Suppose a file declares a MINOR *newer* than the build
understands, and the loader then hits an unknown key. The error then explains that the key "may be
a field added in a newer schema version", rather than simply calling it a typo.

#### Where the schemas come from

The published JSON Schemas in `schemas/*.json` are licensed Apache-2.0, unlike the AGPL-3.0 code.
Nobody writes them by hand.

The Rust types in `outlap-schema` derive `schemars::JsonSchema`. The `gen_schemas` binary, at
`crates/outlap-schema/bin/gen_schemas.rs`, then emits one JSON Schema of draft 2020-12 for each
kind of document:

```bash
cargo run -p outlap-schema --bin gen_schemas            # regenerate schemas/*.json
cargo run -p outlap-schema --bin gen_schemas -- --check # fail if committed files drifted
```

The `--check` form runs in CI on every commit. The committed schemas and the Rust types can
therefore never disagree.

The Python side *conforms*; it does not define. `python/src/outlap/schemas.py` loads the committed
schemas, and validates the shipped fixtures and every `data/**/*.tyr.yaml`, with the `jsonschema`
package. Run `python -m outlap.schemas --check`; CI runs it too.

A mirror in pydantic v2 is planned, but it is not implemented in v0.2. Today, validation through
`jsonschema` is the contract check on the Python side.

Two more rules from Chapter 4 shape every format below. An unknown key that does not start with
`x-` is a hard error, and comes with a did-you-mean suggestion. An `x-*` vendor-extension key is
carried through, with a warning. And everything that a loader estimates or degrades appears in the
loaded-model report. Nothing is silent.

### 5.2 `.ptm`: the neutral powertrain map

A `.ptm` file, named `<name>.ptm.yaml` by convention, is how *any* torque source enters outlap. It
covers an electric drive unit, a bare machine, and a combustion engine.

It is deliberately a **map, and not a model**: a table of what the unit delivers, and what it
wastes, over speed and load.

outlap never simulates the electromagnetics of a motor internally, and it never simulates the
combustion of an engine internally. This boundary is called the *firewall*; see
`crates/outlap-schema/src/ptm.rs`. Chapter 9, Physics III, covers what the solver does with these
numbers.

The document has seven required fields — `schema`, `kind`, `axes`, `tables`, `limits`,
`inertia_kgm2`, and `mass_kg` — plus an optional `meta`. See `schemas/ptm.json`.

Here is a shipped example: the medium drive unit of the Tesla Model 3 study, at
`data/vehicles/tesla_model3_rwd/ptm/du_medium.ptm.yaml`. It is a synthetic dataset.

```yaml
schema: ptm/2.0
kind: electric
axes:
  speed_rpm: [10.000, 340.000, 670.000, 1000.000, 1330.000, 1660.000, 1990.000]
  load_axis:
    torque_nm: [-1659.000, -1244.250, -829.500, -414.750, 0.000,
                345.625, 691.250, 1382.500, 2073.750, 2765.000]
  torque_nm: [-1659.000, -1244.250, -829.500, -414.750, 0.000,
              345.625, 691.250, 1382.500, 2073.750, 2765.000]
  vdc_v: [730.000, 790.000, 850.000]
tables:
  file: du_medium.maps.parquet   # sidecar next to this YAML
  efficiency: true
  loss_w: true
limits:
  max_torque_nm_vs_speed:
    speed_rpm: [10.000, 340.000, 670.000, 1000.000, 1330.000, 1660.000, 1990.000]
    torque_nm: [2765.000, 2765.000, 2765.000, 1935.500, 1455.263, 1165.964, 972.613]
  # Symmetric by construction — declared explicitly so the data carries its own
  # 4th-quadrant boundary instead of leaning on the loader's symmetric-machine fallback.
  max_regen_torque_nm_vs_speed:
    speed_rpm: [10.000, 340.000, 670.000, 1000.000, 1330.000, 1660.000, 1990.000]
    torque_nm: [2765.000, 2765.000, 2765.000, 1935.500, 1455.263, 1165.964, 972.613]
inertia_kgm2: 1.4
mass_kg: 82.0
meta:
  source: synthetic Model-3-scale drive unit (gen_model3_powertrain.py) — ESTIMATED
  dc_voltage_v: 790.0
```

Field by field:

- **`kind`** states the ENERGY SOURCE of the unit, and nothing else, in `ptm/2.0`.

  `combustion` burns fuel. It has no regenerative quadrant, and fuel-mass accounting applies.
  `electric` may regenerate. It draws from a pack, and harvests into one.

  Where the map is referenced is not the business of the kind. Every map is read at the shaft that
  its drive unit outputs onto. A map authored at the machine's own shaft declares the reduction as
  the unit's `fixed_ratio:`, in vehicle/2.1.
- **`axes`** declares the grid.

  `speed_rpm` is the axis of shaft speed. rpm is a unit at the file-format boundary; internally
  everything is rad/s.

  `load_axis` is written as either `{torque_nm: [...]}` or `{load_fraction: [...]}`. A load
  fraction runs from −1 to 1, where negative is the quadrant of regeneration, which recovers energy
  under braking.

  `vdc_v` is the optional **axis of DC-link voltage**. It is a feature from the 1.x era, carried
  into `ptm/2.0`. When it is present, the sidecar tables become a 3-D tensor over
  `(speed_rpm, torque_nm, vdc_v)`, and the solver evaluates them at the terminal voltage of the
  battery, which depends on state of charge; see Chapter 9. It needs at least two breakpoints,
  strictly ascending.

  When the axis is absent, the map is single-voltage, measured at the scalar `meta.dc_voltage_v`.
- **`tables`** points at the numeric sidecar (§5.8), and declares its columns.

  `efficiency` defaults to `true`. Its values run from 0 to 1, and they cover both the drive
  quadrant and the regeneration quadrant.

  `loss_w` defaults to `false`. It is a column of total power loss, in watts, which must be
  consistent with the efficiency if both are given.

  The shipped `du_medium.maps.parquet` is a long, tidy table of 210 rows. That is 7 speeds × 10
  torques × 3 voltages, with the columns exactly `[speed_rpm, torque_nm, vdc_v, efficiency,
  loss_w]`.

  Loss columns for individual components — winding loss against iron loss, for instance — are a
  hook in the format. The loss routing of an `.emotor` file can name a component column. But in
  v0.2 the lap loop consumes only the total `loss_w`.
- **`limits`.** Only `max_torque_nm_vs_speed` is required. It is the peak torque envelope, given as
  two arrays of equal length, and it is what caps traction.

  `max_regen_torque_nm_vs_speed` is the measured envelope of regeneration, in the 4th quadrant. It
  is a curve of positive magnitude, on an electric map only; the loader hard-errors on a combustion
  map that declares one. When an electric map omits it, a symmetric envelope is assumed, and that
  assumption appears as *estimated* in the loaded-model report.

  `cont_torque_nm_vs_speed`, `overload`, and `drag_torque_nm_vs_speed` are optional *references for
  validation*. They are not the mechanism for derating. Sustained thermal capability is computed by
  the `.emotor` model, from the loss tables.
- **`inertia_kgm2`** is the rotational inertia, referred to this map's shaft. **`mass_kg`** is the
  mass attributed to the unit, and it also feeds the mass heuristics of the `.emotor` file (§5.4).

**The ICE variant** is supported from day one, with the same schema.
`data/vehicles/f1_2026/ptm/ice_v6.ptm.yaml` is a synthetic 1.6 L V6, with `kind: combustion` and
`schema: ptm/2.0`. Its load axis is torque, and it has a negative `drag_torque_nm_vs_speed` curve
for engine braking.

For an ICE, the `efficiency` column in the sidecar is the brake thermal efficiency. The runtime
converts source power to a rate of fuel mass, using a lower heating value of 43 MJ/kg; see
Chapter 9.

### 5.3 `.tyr`: the tire document

A `.tyr` file, named `<name>.tyr.yaml`, describes one tire. It has five blocks: `mf61`, an optional
`brush`, `thermal`, `wear`, and `provenance`. See `crates/outlap-schema/src/tyr.rs`.

The physics behind the coefficients is Chapter 7, Physics I. Here is the contract of the file.

- **`mf61`** is a flat map of Magic Formula 6.1 coefficients, which is the industry-standard
  empirical tire model of Pacejka 2012. The keys are the standard `.tir` names: `FNOMIN`, `PCX1`,
  `PKY1`, and so on.

  This is a deliberate design choice. The coefficient names are the interchange vocabulary of tire
  data. outlap therefore validates them as a keyed map, rather than inventing about 150 renamed
  fields.

  Two structural keys are always required: `FNOMIN`, the nominal load in N, and `UNLOADED_RADIUS`,
  in m.

  The eight-coefficient core for pure-slip force is also required: `PCX1, PDX1, PEX1, PKX1, PCY1,
  PDY1, PEY1, PKY1`. It is required *unless* a `brush:` block supplies the force model instead.

  An unknown coefficient name is a **warning**, with a did-you-mean hint, and it is carried through
  unvalidated. That differs from an unknown schema *field*, which is a hard error.

  An absent optional coefficient falls back to a documented default. An absent whole family
  degrades gracefully: with no `QSX*`, the overturning moment is identically 0, and so on. Each
  degradation is logged in the loaded-model report.
- **`brush`** requires `schema: tyr/1.1`. It is the physical fallback model, with four parameters:
  `c_kappa_n`, `c_alpha_n_per_rad`, `mu0`, and `patch_half_length_m`, plus
  `pressure_profile: parabolic`, which is the only option.

  Declaring a brush block in a `tyr/1.0` file is a warning, not an error.
- **`thermal`** has 15 named fields, for the planned model of tire temperature. In v0.2.5 all of
  them are inert placeholders, *except one*: `p_cold`, the cold inflation pressure, which is the
  operating pressure at solve time.

  **A trap with units: `p_cold` is in kPa**, which is a convention inherited from `.tir`. It
  converts to Pa at the code seam. `t_cold` is in °C.
- **`wear`**, with 10 fields, is entirely reserved for the planned model of wear and the cliff.
  Shipped datasets carry synthetic placeholders, clearly labelled.
- **`provenance`** is required. It is how tire data stays honest. It holds `citation`, which is the
  source in the literature for the coefficients; `source`, which is a note for a human reader; and
  `synthetic: bool`, which defaults to `false`.

The road tire of the Model 3, at `data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml`, is a good
worked example. It is a verbatim transcription of the published Pacejka (2006) 205/60R15 book tire:

```yaml
schema: tyr/1.0
mf61:
  FNOMIN: 4000.0
  UNLOADED_RADIUS: 0.313
  LONGVL: 16.67
  NOMPRES: 220000.0
  PCX1: 1.685
  PDX1: 1.210
  # ... ~50 more coefficients ...
thermal:
  # SYNTHETIC placeholder — the thermal ring model is a future physics addition.
  p_cold: 220.0        # kPa (load-bearing today: the solve-time pressure)
  t_cold: 20.0
  # ...
provenance:
  citation: "H. B. Pacejka, Tyre and Vehicle Dynamics, 2nd ed. (2006), Appendix 3, Table A3.1 (205/60R15 91V, 2.2 bar, ISO sign)"
  source: "MF6.1 force/moment coefficients transcribed verbatim from the book table; ..."
  synthetic: false
```

The coefficient vocabulary is the `.tir` vocabulary. outlap therefore also ships a clean-room codec
for the TNO `.tir` text format, in `crates/outlap-schema/src/tir/`, with a Python mirror at
`outlap.tir`. Run `python -m outlap.tir to-tyr in.tir -o out.tyr.yaml` to convert an existing
`.tir` into a `.tyr`.

A `.tir` carries no physics for thermal state or wear. The converter must therefore synthesize
those blocks, which is the default policy. It marks the resulting provenance as `synthetic: true`,
and it records the synthesis as warnings. The provenance block always tells you where a tire came
from. See Chapter 11, on importers and tooling.

### 5.4 `.emotor`: the machine thermal network

A `.emotor` file is named `<name>.emotor.yaml`, with `schema: emotor/1.1`. It declares a
lumped-parameter thermal network, or LPTN, for one electric machine.

That network is a small graph. Its "nodes" are thermal masses, and its "edges" are paths for heat
flow. The solver marches it in time, to predict the temperature of the winding and of the magnet,
and to derive a derate on torque.

It is referenced from the `thermal:` field of a drive unit, in `vehicle.yaml`.

The physics is Chapter 9: Crank–Nicolson integration, the heat-transfer correlations of
Becker–Kaye, Kylander, Churchill–Chu, and Gnielinski, and the law for the derate. Here is the
format.

- **`nodes`** is required. There must be at least 2, and at most 24 at runtime.

  Each node has a `name`. It has an optional `role`, one of `winding`, `stator_iron`, `rotor`,
  `housing`, `coolant`, `ambient`, or `other`. At least one `winding` node is required, because it
  is the default target for loss.

  Each node has an optional heat capacity, `c_j_per_k`. It has optional paired temperature limits,
  `t_warn_c` and `t_max_c`; give both or neither. Only a node that carries limits takes part in
  derating.

  An omitted capacity on a node with a role is filled by documented mass heuristics, from the
  `mass_kg` of the `.ptm` file. Each filled value is flagged as an estimate.
- **`conductances`** is required. Each entry is a constant edge,
  `{between: [a, b], w_per_k: ...}`. Omit `w_per_k` to use the mass heuristic.
- **`convection`** is optional. Each entry is an edge that depends on speed and temperature,
  `{between, area_m2, model}`. `model` is one of `air_gap`, `rotor_air`, `shaft_external`,
  `liquid_channel`, or `free_convection`. A published correlation backs each one.
- **`cooling`** is required. It names the pinned `ambient_node`, which is held at the `ambient_c`
  of `conditions.yaml`, unless `ambient_fixed_c` overrides it.

  It holds at most one of two things: a low-level `coolant` spec, or a high-level `jacket` block.
  The `jacket` block gives the raw geometry of a cooling channel, from which assembly derives the
  balance at the coolant node, and an edge of the liquid-channel kind.

  An optional `air_gap` block gives the raw geometry of the rotor and the gap, for the film between
  stator and rotor.
- **`loss_routing`** is optional. Its entries, `{component?, node, fraction}`, split the loss from
  the `.ptm` file among the nodes. Empty routing, or any fraction left unrouted, lands on the
  winding node.
- **`cu_feedback`** is optional. It is feedback from the resistivity of copper. It rescales the
  routed loss by $1 + \alpha\,(T - T_{\mathrm{ref}})$.
- **`initial_temp`** is optional. It is either `{uniform_c: ...}` or a value for each node. If it is
  absent, every node starts at its sink temperature.
- **`meta.source`** is one of `datasheet`, `estimated`, or `pdt_imported`.

The shipped example is `data/vehicles/tesla_model3_rwd/emotor/rear_du.emotor.yaml`, in which every
value is estimated. It shows the whole menu in 42 lines: six nodes tagged with roles, three
constant edges, a jacket cooling block with ethylene-glycol coolant, an air-gap block, loss routing
of 55, 30, and 15 percent to winding, stator, and rotor, and feedback from copper.

```yaml
schema: emotor/1.1
nodes:
  - { name: winding, role: winding, c_j_per_k: 6500.0, t_warn_c: 150.0, t_max_c: 180.0 }
  - { name: rotor,   role: rotor,   c_j_per_k: 5500.0, t_warn_c: 140.0, t_max_c: 170.0 }
  # ... stator_iron, housing, coolant, ambient ...
cooling:
  ambient_node: ambient
  jacket:
    housing_node: housing
    coolant_node: coolant
    inlet_c: 45.0
    flow_rate_lps: 0.40
    channel_count: 12
    channel_width_mm: 8.0
    channel_height_mm: 9.0
    wetted_area_m2: 0.080
    fluid: { named: ethylene_glycol_50 }
loss_routing:
  - { node: winding,     fraction: 0.55 }   # of the .ptm total loss (loss_w)
  - { node: stator_iron, fraction: 0.30 }
  - { node: rotor,       fraction: 0.15 }
cu_feedback: { nodes: [winding], t_ref_c: 60.0, alpha_per_k: 0.0039 }
```

### 5.5 `battery/1.0`: the equivalent-circuit pack

The battery document, named `<name>.battery.yaml` by convention, describes a pack as a Thevenin
equivalent-circuit model, or ECM.

That model is a source of open-circuit voltage, behind a series resistance and one
resistor–capacitor pair. All four quantities are tabulated over state of charge and temperature.

The form follows the published NREL `thevenin` model (BSD-3), and the ECM literature that it cites
(Plett 2015). The runtime is Chapter 9.

It is referenced from the `battery: {model: rc_pairs, params: <path>}` block of `vehicle.yaml`.

The required fields are: `schema`; `model`, which is only `rc_pairs`; `topology`, which gives `ns`
cells in series times `np` in parallel; `capacity`, which gives `q_pack_ah` for Coulomb counting
and an informational `e_pack_wh`; `soc_window`, which is `[min, max]`, ascending, within 0 to 1;
`ecm`; `limits`; and `thermal`.

The `ecm` block declares `rc_pairs: 1`, which is the only count supported in v0.2. It declares the
grid axes `(soc, temp_c)`, each with at least 2 points, strictly ascending. And it declares the
sidecar reference, with its `level`. A `cell` table is scaled to pack level, with voltage times ns
and resistance times ns/np. A `pack` table is used as it is.

The sidecar is a long, tidy parquet, with the columns exactly `soc, temp_c, ocv_v, r0_ohm, r1_ohm,
tau1_s, dudt_v_per_k`. The shipped `pack_800v.tables.parquet` has 18 rows: 6 SoC values × 3
temperatures.

Here is the shipped pack, at `data/vehicles/tesla_model3_rwd/battery/pack_800v.battery.yaml`. It is
synthetic.

```yaml
schema: battery/1.0
model: rc_pairs
topology: { ns: 220, np: 1 }
capacity: { q_pack_ah: 92.0, e_pack_wh: 64064.0 }
soc_window: [0.05, 0.98]
ecm:
  rc_pairs: 1
  axes:
    soc: [0.05, 0.20, 0.40, 0.60, 0.80, 1.00]
    temp_c: [0.000, 25.000, 45.000]
  tables: { file: pack_800v.tables.parquet, level: cell }
limits:
  peak_discharge_power_w_vs_soc:
    soc: [0.05, 0.20, 0.40, 0.60, 0.80, 1.00]
    power_w: [70000.0, 160000.0, 230000.0, 255000.0, 265000.0, 265000.0]
  peak_regen_power_w_vs_soc:
    soc: [0.05, 0.20, 0.40, 0.60, 0.80, 1.00]
    power_w: [190000.0, 190000.0, 170000.0, 140000.0, 85000.0, 30000.0]
  cell_v_min: 2.7
  cell_v_max: 4.2
  max_c_rate: 4.5
thermal:
  mass_kg: 460.0
  cp_j_per_kgk: 900.0
  thermal_resistance_k_per_w: 0.02
  coolant_temp_c: 25.0
```

This pack is designed to teach. Its 220 cells swing from roughly 634 V to 810 V open-circuit. Under
load at low SoC, the terminal voltage therefore sags *below* the 730–850 V `vdc_v` grid of the
drive unit. That deliberately exercises the linear extrapolation below the grid, described in §5.9.

One quiet caveat, from Chapter 4. The load pipeline of the vehicle does not validate the
`battery.params` reference; it loads only the tires, the ERS, and the `.ptm` and `.emotor` files of
the drive units. A missing battery file therefore surfaces only as a note at solve time, saying
that the coupling is inert.

### 5.6 `track.yaml` plus `centerline.csv`: the 3D track

A track is two files. `track.yaml` is a thin descriptor. The geometry lives in a CSV sidecar.

From `data/tracks/catalunya_osm/track.yaml`:

```yaml
schema: track/1.0
name: Circuit de Barcelona-Catalunya
closed: true
centerline: centerline.csv
meta:
  source: osm+dem
  dem: eudem25m
  accuracy_class: B
  attribution: "© OpenStreetMap contributors (ODbL); elevation eudem25m via opentopodata.org"
  notes: "widths defaulted; banking not resolved from DEM (add keypoints to refine)"
```

`closed` defaults to **`true`**. A point-to-point track, such as a hillclimb or a test straight,
must opt out with `closed: false`.

`banking_keypoints` is optional. It is a list of `[{s_m, banking_deg}]`, strictly ascending, with
`s_m ≥ 0`. Those are sparse samples of banking, interpolated in arc length. When they are present,
they *override* the `banking_deg` column of the centerline.

The `meta` block carries provenance. Its `accuracy_class` is `A` for surveyed, `B` for fused from a
DEM, and `C` for estimated. Its `attribution` string is required for redistributing data derived
from ODbL or from Copernicus. The 25 tracks in `data/tracks/` that derive from TUMFTM are
LGPL-3.0; see Chapter 12, on the shipped data library.

`centerline.csv` is plain CSV, with exactly eight required columns. They are **named in a header,
and their order does not matter**; see `crates/outlap-schema/src/centerline.rs`.

```text
s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale
0.0000,237.8136,555.8796,137.5116,0.000,6.000,6.000,1.0000
3.0002,240.2026,554.0648,137.3254,0.000,6.000,6.000,1.0000
```

- `s_m` is the arc-length station, in meters. It must **increase strictly**. A NaN is rejected too.
- `x_m, y_m, z_m` are world coordinates, in the ISO 8855 frame: x forward, y left, z up. They must
  be finite.
- `banking_deg` is the banking angle, in degrees. That is a unit at a display boundary.
- `width_left_m` and `width_right_m` are the half-widths of the track. Both must be > 0.
- `grip_scale` is a multiplier on friction, at each station. It must be > 0, and 1.0 is nominal.

A line that starts with `#`, and a blank line, are both skipped. Every validation error carries a
1-based line number. A missing column even gets a did-you-mean, from your actual header.

Because it is CSV, this is the one input with no JSON Schema. The parser *is* the contract.

**The rules on closure** are applied by `outlap-track`, when it fits the periodic spline; see
`crates/outlap-track/src/lib.rs`.

A closed track needs at least 4 points.

If the first and last points coincide within $10^{-6}$ m, the duplicated last row is dropped, and
the period of the loop is its arc length. If they are distinct, the loop is closed over the
connecting chord, and the period becomes `s_last + chord`.

If the gap at the start and finish exceeds 3 times the median sample spacing, loading fails, with
"track marked closed but the start/finish gap is … — set `closed: false` or fix the centerline".

Do not hand-close your CSV twice.

### 5.7 The aero map parquet

The `aero:` block of the vehicle names a gridded map, and its input axes. Chapter 4 shows the
block, and Chapter 7 gives the physics.

The map itself is a parquet sidecar, in the same long, tidy convention as everything else.

An axis name must come from the known set, in `crates/outlap-schema/src/load/semantic.rs`:
`ride_height_f_mm`, `ride_height_r_mm`, `ride_height_mm`, `yaw_deg`, `roll_deg`, `steer_deg`,
`drs_flag`, and `speed_mps`.

The value columns are the three lumped products of coefficient and area, in m²: `cz_front_a_m2`,
`cz_rear_a_m2`, and `cx_a_m2`. See `crates/outlap-qss/src/t1/aero.rs`.

The shipped F1 map, at `data/vehicles/f1_2026/aero/f1_2026.parquet`, is a 4-D grid of 250 rows.
That is 5 front ride heights × 5 rear ride heights × 5 yaw angles × 2 DRS states. It is declared as:

```yaml
aero:
  map: aero/f1_2026.parquet
  axes: [ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag]
  constant:            # T0 fallback + sanity anchor
    cx_a_m2: 1.25
    cz_front_a_m2: 1.9
    cz_rear_a_m2: 2.6
```

Every axis of an aero map clamps outside its domain (§5.9).

A road car with no map at all uses the degenerate `constant:` block alone. The Model 3 points
`map:` at a deliberately absent `aero/none.parquet`, so the constant coefficients carry. The skip
is recorded as a note: "aero map … not present — constant-aero fallback carries the lap".

### 5.8 Parquet sidecars: how a binary table travels

YAML is for structure and for provenance. Bulk numbers live in a **sidecar**, which is a separate
binary file referenced by path.

outlap uses Apache Parquet, a compact columnar format, in one uniform shape: *long and tidy*
columns of `f64`. That means one row for each grid point, with the axis coordinates repeated. So it
is `speed_rpm, torque_nm, vdc_v, efficiency, loss_w`, and not a 3-D array.

By convention the sidecar sits next to the YAML that references it, as in
`file: du_medium.maps.parquet`. The loader resolves it there first, then falls back to the vehicle
root.

Here is the plumbing, from `crates/outlap-schema/src/io.rs` and `sidecar.rs`:

```rust
pub trait SourceLoader {
    fn load(&self, path: &str) -> Result<String, SourceError>;
    fn load_bytes(&self, path: &str) -> Result<Vec<u8>, SourceError> { /* default: errors */ }
}
```

Every file access in outlap goes through this trait. `FsLoader` roots it at a directory, which is
why every reference inside `vehicle.yaml` is relative to the vehicle directory. `MemLoader` serves
the in-memory path and the browser path.

`load_bytes` exists for exactly one purpose: fetching the bytes of a sidecar.

Three properties are worth understanding.

1. **Decoding happens at assembly time only.** `read_gridded_table(bytes, axis_names)` parses the
   parquet, and pivots the long columns onto a rectilinear grid, through `GriddedTable::from_long`.
   The result is installed into the solver *before* the lap starts. Nothing in the hot loop ever
   touches parquet.
2. **A `NULL` becomes a NaN.** A missing cell in a column decodes to NaN. That is the convention
   for masking an operating point that cannot be reached (§5.9). A column that is not numeric is a
   hard error.
3. **The strategy for wasm.** The parquet decoder pulls in a dependency that cannot compile to
   WebAssembly. It therefore lives behind the non-default `parquet` cargo feature of
   `outlap-schema`.

   The *decoded* types, `GriddedTable` and `GriddedMapN`, live in the wasm-clean `outlap-core`. The
   solvers therefore never see parquet at all. A browser build simply ships pre-decoded tables,
   through `MemLoader`.

A *missing* sidecar is a skip, with a note; the fallback to constant aero, or to a peak envelope,
carries the lap. A sidecar that is *present but undecodable* is a real error.

The hash of the resolved vehicle covers only the YAML. The Python solver therefore folds a
fingerprint of the bytes of every sidecar into its key for the envelope cache. Two cars with
identical specifications but different tables never share a cached result; see `install_sidecars`
in `crates/outlap-py/src/lib.rs`.

### 5.9 One interpolant for every map

Tabulated data becomes a continuous function only through *interpolation*, which estimates values
between grid points.

outlap has exactly **one** policy of interpolation, for every gridded map: monotone cubic Hermite.
It lives in `crates/outlap-core/src/interp.rs` for 1-D, as `MonotoneCubic`, and in
`crates/outlap-core/src/gridmap.rs` for N-D as a tensor product, as `GriddedMapN`, with up to
`MAX_DIMS = 6` axes.

Powertrain efficiency, aero coefficients, battery ECM tables, torque envelopes, and the data
channels of the track at each station all go through this same code.

Uniformity here is a feature of correctness. No map behaves differently from another. And no solver
needs to know which file its numbers came from.

A cubic Hermite interpolant fits a cubic polynomial on each grid interval. That polynomial passes
through the two endpoint values, $y_k$ and $y_{k+1}$, with the endpoint slopes $m_k$ and $m_{k+1}$:

$$
f(x) = h_{00}(t)\,y_k + h_{10}(t)\,h\,m_k + h_{01}(t)\,y_{k+1} + h_{11}(t)\,h\,m_{k+1},
\qquad t = \frac{x - x_k}{h},\; h = x_{k+1} - x_k,
$$

where $h_{00}$ through $h_{11}$ are the standard Hermite basis polynomials.

The slopes are what make it trustworthy. outlap limits them with the Fritsch–Carlson method (F. N.
Fritsch and R. E. Carlson, "Monotone Piecewise Cubic Interpolation", *SIAM J. Numer. Anal.* 17(2),
1980).

That method caps each tangent, so that the curve **never overshoots the data**, and so that it is
monotone wherever the samples are monotone. An efficiency map interpolated this way therefore
cannot invent an efficiency above its measured peak.

The result is $C^1$: both the value and the slope are continuous everywhere. The derivative is also
available analytically. `MonotoneCubic::deriv` and `GriddedMapN::grad_into` return exact gradients,
with no finite differencing. The Newton solvers in the transient tiers require that.

The N-D version applies the same 1-D limiter on tangents, successively along each axis. It
precomputes all mixed partial derivatives at every node, at assembly time. Along any line aligned
with the grid, it coincides exactly with the 1-D interpolant.

#### NaN cells, and the hull of valid data

An imported map is often not a full rectangle. A dyno cannot measure a torque that the machine
cannot reach, so a powertrain map derived from PDT typically carries about 1.5 % NaN cells, beyond
the reachable envelope.

`GriddedMapN` handles this at construction. NaN cells are filled by a deterministic
breadth-first search for the nearest valid cell, over the grid. The interpolant is therefore total,
and $C^1$.

But the original mask of NaN cells is retained. That mask is the "hull" of data that was genuinely
measured.

Every evaluation whose *domain of dependence* touches a filled cell is flagged. That domain is the
surrounding cell corners, plus the ±1 neighbors that the tangent stencil reaches. Any result
influenced by synthetic fill is therefore identifiable.

A map with no NaN cells skips the check entirely.

#### Outside the domain: clamp, or linear, for each axis

Each axis carries an `OutOfDomain` mode:

| Mode | Behaviour outside the grid | Used by |
|---|---|---|
| `Clamp` (default) | Saturate at the edge value: constant, zero slope | Everything, unless stated otherwise: speed/torque axes of `.ptm` maps, all aero-map axes, battery ECM axes ("the ECM is only defined on its measured hull"), every `MonotoneCubic` curve |
| `Linear` | Extrapolate along the boundary tangent, $C^1$-continuous with the interior | The **`vdc_v` axis of Vdc-stacked `.ptm` maps only** (`T1Powertrain::install_maps`, `crates/outlap-qss/src/t1/powertrain.rs`) |

The one linear axis is deliberate physics, not laxity.

A real pack of 220 cells in series swings from roughly 634 V to 810 V over its SoC window. A
drive-unit map is typically gridded from 730 V to 850 V. A wide band at low SoC therefore sits
*below* the map.

Clamping there would freeze the efficiency at the 730 V slice. Linear extrapolation follows the
trend at the boundary instead, and the energy math floors the extrapolated efficiency to the
physical range $[10^{-3}, 1]$. Chapter 9 covers the full Vdc–SoC coupling.

Nothing about leaving the grid is silent. Every query can return `EvalFlags` alongside its value:

```rust
pub struct EvalFlags {
    pub extrapolated: bool, // the query left the grid on at least one axis
    pub out_of_hull: bool,  // the stencil touched a NaN-filled (unmeasured) cell
}
```

And installing a Vdc-stacked map records a note in the loaded-model notes:
"efficiency/loss map installed — energy accounting is live (Vdc-coupled; linear extrapolation
below/above the voltage grid)". A lap that ran partly off the grid is therefore documented in the
report of the run. You do not discover it by surprise.

### 5.10 Which file means what: the summary table

| File / extension | `schema:` | What it holds | Binary sidecar |
|---|---|---|---|
| `vehicle.yaml` | `vehicle/1.x` | The car: chassis, aero, suspension, tire refs, drivetrain topology, brakes, optional ERS/battery | — (references everything below) |
| `track.yaml` | `track/1.0` | Track descriptor: name, `closed`, banking keypoints, provenance | `centerline.csv` |
| `centerline.csv` | — (CSV, no JSON Schema) | 8-column 3D centerline: `s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale` | is the sidecar |
| `conditions.yaml` | `conditions/1.0` | Environment: air temperature/pressure, wind, track surface and ambient temperatures (all defaulted) | — |
| `sim.yaml` | `sim/1.1` | Numerics: tier, dt, integrator, envelope grid, raceline source, `allow_degraded`, `flat_track` (all defaulted) | — |
| `*.ptm.yaml` | `ptm/2.0` | Neutral powertrain map: kind, speed/load(/Vdc) axes, torque limits, inertia, mass | `*.maps.parquet` (`efficiency`, `loss_w`) |
| `*.tyr.yaml` | `tyr/1.0` / `tyr/1.1` | Tire: MF6.1 coefficients (`.tir` names), optional brush block, thermal/wear (placeholders), provenance | — |
| `*.emotor.yaml` | `emotor/1.1` | N-node machine thermal network: nodes, edges, cooling, loss routing | — |
| `*.battery.yaml` | `battery/1.0` | Thevenin pack: topology, capacity, SoC window, ECM axes, power limits, lumped thermal | `*.tables.parquet` (OCV/R0/R1/τ1/dU-dT) |
| `*.parquet` | — | Long/tidy `f64` tables: powertrain maps, battery ECM tables, aero maps | is the sidecar |
| `*.tir` | — (TNO text format) | Industry tire-coefficient interchange; convert with `python -m outlap.tir` | — |

Keep three conventions in your head as you write files.

Units are SI, except at a documented display boundary: rpm on a `.ptm` speed axis, °C in every
`*_c` field, kPa in the `p_cold` of a `.tyr` file, and degrees in `banking_deg` and `yaw_deg`.

Every path in a `vehicle.yaml` is relative to the vehicle directory.

And the license of every shipped data file rides in its first line. Data files are CC-BY-SA-4.0.
The schemas are Apache-2.0. The code is AGPL-3.0-only.

With the formats in hand, Chapter 6 zooms out, to how the crates that read them fit together.


---

## 6. Architecture: how the code is organized

*What you will learn: how the Rust workspace is laid out, crate by crate, and which crates are real
and which are reserved placeholders. How outlap splits all work into a cold "assembly pipeline" and
a zero-allocation "hot loop". And, most importantly, exactly what data enters and leaves each
stage, on the journey from `vehicle.yaml` to an `xarray.Dataset`. Along the way you will meet the
roadmap for the plugin points, the rules for WebAssembly cleanliness, the disciplines for error
handling and determinism, and the point where Rust ends and Python begins.*

### 6.1 The workspace at a glance

The Rust code of outlap lives in a single Cargo *workspace*. A workspace is a collection of
packages, which Rust calls *crates*, that build together and share pinned versions of their
dependencies.

The root `Cargo.toml` declares `members = ["crates/*"]`. That gives thirteen crates. All use
`edition = "2021"` and `license = "AGPL-3.0-only"`.

Ten of them do real work today. Three are two-line placeholders, reserving names for later work.
Each of those contains only an SPDX header, and the doc line "placeholder crate; implemented in a
later milestone".

| Crate | Status | Role |
|---|---|---|
| `outlap-core` | real | Shared numerics: the one monotone cubic Hermite interpolant (`MonotoneCubic`, `crates/outlap-core/src/interp.rs`), C² cubic splines (`CubicSpline`), and N-dimensional gridded maps up to 6 axes (`GriddedMapN`, `GriddedTable` in `src/gridmap.rs`) |
| `outlap-schema` | real | The file-format contract: serde + schemars types for all eight document kinds and the staged vehicle load/validation pipeline (`src/load/mod.rs`); see Chapter 5, Files and formats |
| `outlap-tire` | real | Tire force backbone: clean-room MF6.1 (Pacejka 2012) and a physical brush model behind one static `TireModel` enum (`src/model.rs`) |
| `outlap-track` | real | Loads `track.yaml` + `centerline.csv` into a queryable 3D road ribbon (`Track`), and turns any lateral offset into a first-class track via `offset_track` |
| `outlap-thermal` | real | N-node lumped-parameter thermal network (LPTN) for electric machines: heat-transfer correlations plus a Crank–Nicolson integrator (`Network::advance`, `src/network.rs`) |
| `outlap-qss` | real | The quasi-steady-state solver tier: T0 velocity-profile solver, T1 trim, the g-g-g-v envelope generator, tier dispatch, and slow-state coupling — the biggest crate |
| `outlap-raceline` | real | Minimum-curvature racing-line generator (convex QP via `clarabel`), returning the line as a first-class `Track` |
| `outlap-py` | real | The `outlap_core` Python extension module (PyO3); the only crate allowed to contain `unsafe` glue |
| `outlap-powertrain` | stub | Reserved; today's powertrain logic lives in `outlap-qss/src/t1/powertrain.rs` and the `.ptm` format in `outlap-schema/src/ptm.rs` |
| `outlap-vehicle` | real | The T2 physics blocks: the 7-DOF chassis RHS in the curvilinear 3D road frame, load transfer, tires with relaxation, the MacAdam-preview driver, and the shared T2 assembly (Chapter 8) |
| `outlap-batch` | stub | Reserved for the batch/GPU rollout layer (Chapter 15) |
| `outlap-transient` | real | The T2 lap orchestration: the split integrator's step loop, the target-line table, the shift/TV/regen control layer, the time-indexed result (Chapter 8) |
| `outlap-wasm` | stub | Reserved WebAssembly shell; currently empty but still the named target of the wasm CI gate (see §6.6) |

The dependency graph is shallow, and strictly layered. Math sits at the bottom, and the user
surface at the top:

```text
outlap-core            shared math; no sibling deps (num-traits + thiserror only)
  ├─ outlap-schema     file contract + load pipeline        (features: std, parquet)
  │    └─ outlap-tire  MF6.1 + brush kernels                (schema, default-features off)
  ├─ outlap-track      3D road ribbon                       (core + schema, default-features off)
  ├─ outlap-raceline   min-curvature QP                     (core + track + clarabel)
  └─ outlap-qss        T0/T1 solvers + envelope + dispatch  (core + tire + track + thermal
                                                             + schema[no-default]; optional rayon)
outlap-thermal         pure math, NO sibling deps (thiserror only)

outlap-py [cdylib]     PyO3 bindings over schema + track + tire + qss[parallel] + raceline
```

Two deliberate oddities are worth calling out.

First, `outlap-thermal` sits *below* the schema layer. It never sees an `.emotor` file. The mapping
from an `.emotor` document to a thermal `Network` lives in `outlap-qss`, at `src/t1/thermal.rs`.
That keeps the thermal math free of dependencies.

Second, `outlap-tire` does not depend on `outlap-core` at all. It consumes an already-loaded
`outlap_schema::tyr::Tyr`, and evaluates pure closed-form kernels.

#### The working crates, one paragraph each

**`outlap-core`** is the root of the graph. It holds exactly the math that every other layer
shares.

Its headline citizen makes the project-wide rule on interpolation concrete. `MonotoneCubic`, at
`src/interp.rs:51`, is *the* one shared monotone cubic Hermite interpolant, which is C¹. It is a
curve that passes through every data point, without inventing an overshoot. Every gridded lookup in
the codebase uses it, following a monotone construction in the style of Fritsch–Carlson.

`CubicSpline`, in `src/spline.rs`, provides the C² splines that track geometry needs.
`GriddedMapN` and `GriddedTable`, in `src/gridmap.rs`, handle N-dimensional tables, up to
`MAX_DIMS = 6`. Powertrain efficiency maps and the g-g-g-v envelope are examples.

**`outlap-schema`** is the wire contract. It holds the serde and schemars types for all eight kinds
of document — `vehicle`, `ptm`, `tyr`, `emotor`, `battery`, `track`, `conditions`, and `sim` — plus
the staged load pipeline of §6.2, the error types powered by miette, and the codec for `.tir`
interchange.

The committed JSON Schemas in `schemas/`, licensed Apache-2.0, are *generated from* these Rust
types, by the `gen_schemas` binary. CI fails if the generated and the committed files drift apart.
Chapter 5 covers the formats themselves.

**`outlap-tire`** implements the tire force backbone, clean-room from Pacejka 2012, 3rd ed. It
covers steady-state MF6.1 under pure slip and combined slip, giving $F_x$ and $F_y$, plus the
moments $M_z$, $M_x$, and $M_y$, with the Besselink terms for inflation pressure. Turn-slip is
omitted in v1.

A physical brush model, in `src/brush.rs`, serves a tire whose `.tyr` file lacks a full MF6.1 core.
`src/relax.rs` holds the first-order slip relaxation that the T2 tier integrates on each step.

The kernels are pure, panic-free, and allocation-free, and they are generic over `f32` and `f64`.

**`outlap-track`** turns `track.yaml` and `centerline.csv` into a queryable road ribbon, in full 3D.
It fits a C² spline for the geometry, so that curvature is continuous. It uses monotone-cubic
channels for banking, widths, and grip. And it answers `road_frame(s)` queries, which the solvers
consume.

Its most interesting export, architecturally, is `offset_track`, at `src/lib.rs:408`. Any profile
of lateral offset becomes a *first-class* `Track`, with its own curvature and its own frames. That
is how a generated racing line is driven through the identical solver API.

**`outlap-thermal`** is the lumped-parameter thermal network for a machine. Published heat-transfer
correlations — Churchill–Chu for free convection, Gnielinski for channel flow, Becker–Kaye for the
air gap, and others, each cited in `src/correlations.rs` — feed a `Network` of up to
`MAX_NODES = 24` temperature nodes. A Crank–Nicolson step, which is unconditionally stable,
advances it.

Its detailed authoring tier ports correlations from the author's own PDT work. That is the one
documented, deliberate amendment of the powertrain firewall; see `src/lib.rs:17-19`. Chapter 9
covers the physics.

**`outlap-qss`** is where laps get solved. It holds the `T0Path` sampler; the `T0Vehicle` and
`T1Vehicle` assemblies; the forward and backward velocity-profile solver in `src/solver.rs`,
re-implemented from Heilmeier et al. 2020 on the 3D ribbon of Perantoni & Limebeer; the T1
damped-Newton trim in `src/t1/trim.rs`; the g-g-g-v envelope generator in `src/t1/envelope.rs`,
after Werner et al. 2025; the tier dispatch; and the coupling of slow states that marches battery
and machine temperatures along the lap.

Its optional `parallel` feature, using rayon, accelerates generating the envelope. It applies to
native builds only.

**`outlap-raceline`** generates the racing lines: minimum-curvature, and its time-weighted
refinement; see Chapter 8. Minimizing $\int \kappa^2\,ds$ over the lateral offset $n(s)$, within
the track bounds, is a convex quadratic program with box bounds. `clarabel` solves it. The
formulation is re-implemented from the published sources — Braghin et al. 2008, and Heilmeier et
al. 2020, §3.1–3.2 — and never from the LGPL TUM source.

**`outlap-vehicle`** holds the T2 physics blocks. They are: the 7-DOF chassis right-hand side, in
the curvilinear 3D road frame, verified against a symbolic derivation to 1e-12 in CI; the
load-transfer block, which re-uses the exported T1 algebra, so that there is one source of truth
for $F_z$; the tire block, with a relaxation state for each wheel; the MacAdam-preview driver; and
the shared `assemble_t2` pipeline, which both the tests and the Python boundary use. §8.7 covers
the physics.

**`outlap-transient`** orchestrates the T2 lap. It holds the step loop of the split fixed-step
integrator, running `sense → control → actuate → integrate`; the sampled table of the target line
that the driver reads on each step; the rule-based control layer, covering the shift state machine,
torque vectoring, blending of regeneration, and the clock for slow states; and the time-indexed
`TransientLap` result.

It *receives* the QSS artifacts — the envelope, the T0 profile, and the line. It never computes
them, and it never caches them. That is what keeps it wasm-clean.

**`outlap-py`** is the boundary crate. §6.9 describes it.

Runnable examples live inside the crates, rather than in an `examples/` directory at the root of
the repository. They are in `crates/outlap-qss/examples/`, such as `catalunya_lap.rs`,
`limebeer_lap.rs`, and `ggv_traces.rs`, and in `crates/outlap-raceline/examples/catalunya_line.rs`.

They emit CSV, which `python/tools/plot_*.py` consumes. Every figure in the theory pages is
therefore generated by the real Rust models.

### 6.2 The two worlds: the assembly pipeline, and the hot loop

Everything in outlap belongs to one of two worlds.

The **assembly pipeline** is the cold path. It runs once for each model load. It is allowed to
allocate memory, read files, build strings, and fail with rich diagnostics.

Its job is to turn human-friendly YAML into structs that are compact, immutable, and numbers only.

Concretely, loading a vehicle runs the staged pipeline in
`crates/outlap-schema/src/load/mod.rs`:

1. **Load and parse.** Fetch the text through the `SourceLoader` trait, and parse it with
   span-preserving YAML, through `marked-yaml`. YAML anchors, aliases, and duplicate keys are
   rejected.
2. **The version gate.** The `schema: vehicle/1.x` header must name the right kind of document, and
   the right MAJOR version.
3. **Resolve `extends`, deep-merge, and apply overrides.** Preset inheritance is single-parent.
   Then come the dotted-path overrides, such as `chassis.mass_kg`. Every value is tagged with its
   provenance, in an `Origin`.
4. **Walk the unknown keys.** Any key that is not in the schema, and does not start with `x-`, is a
   hard error, with a did-you-mean suggestion.
5. **One deserialize after the merge**, into the typed `Vehicle` struct.
6. **Semantic checks**: ranges, signs, and rules across fields.
7. **Checks on the topology graph.** The drivetrain graph, from source to coupler to wheel, must
   make physical sense.
8. **Estimation.** Documented heuristics fill missing derivable values, and report them.
9. **Hash the resolved set.** A blake3 hash of the canonical resolved parameter set, recorded in
   every result.

Downstream of the schema pipeline, and still cold, come five more steps: fitting the track spline,
in `Track::from_doc`; sampling the path, in `T0Path::from_track`; assembling the solver vehicles,
in `T0Vehicle::assemble` and `T1Vehicle::assemble`; decoding the parquet sidecars; and generating
the g-g-g-v envelope.

The architectural promise is that after assembly, the hot loop touches *zero* strings, hashes, and
configuration logic.

The **hot loop** is the solve itself. Its rules are not negotiable, and CI enforces them.

- **Zero heap allocations for each step.** The solve kernels write into workspaces that the caller
  owns, and that are allocated in advance.

  A test using the `dhat` allocation profiler, at `crates/outlap-qss/tests/alloc.rs`, asserts that
  six things allocate exactly zero heap blocks: `solve_into`, `solve_into_ggv`, `T1Vehicle::trim`,
  `MachineThermal::step`, `Pack::step_power`, and the queries on the envelope boundary.

  CI runs it in release mode, alongside the wall-clock gate of 50 ms or less for a lap; see
  `.github/workflows/ci.yml`, lines 19 to 21.
- **No Python inside a timestep, ever.** That includes controllers. They are Rust or C-ABI only.
- **Dispatch through an enum, not dynamically.** A choice of model is resolved at assembly time,
  into a plain Rust enum, such as `TireModel::Mf61 | Brush`, or into a *monomorphized* generic.

  The compiler then stamps out a specialized copy of the sweep for each grip model; see
  `trait GripModel` in `crates/outlap-qss/src/solver.rs`. There is therefore no virtual call at
  each station.
- **State is stored as SoA**, which is structure-of-arrays. Each channel is one contiguous
  `Vec<f64>`, rather than an array of one struct for each station. The sweeps therefore stream
  linearly through memory.

At the T2 tier, each fixed timestep runs the four phases `sense → control → actuate → integrate`.
The driver and the controllers read the bus, decide, and actuate. Only then does the integrator
advance the state; see §8.7.

At the QSS tiers there is no timestep. But the same split between cold and hot applies to the
sweeps over arc length.

### 6.3 Data flow: one lap, end to end

This section is the core of the chapter. It shows what actually enters and leaves each function, on
the way from files on disk to a labelled dataset.

The running example is the shipped Tesla Model 3, at `data/vehicles/tesla_model3_rwd/`, on
Catalunya, at `data/tracks/catalunya/`, solved from Python:

```python
from outlap.core import Track, solve_lap_dataset

track = Track.load("data/tracks/catalunya")
ds = solve_lap_dataset("data/vehicles/tesla_model3_rwd", track)
```

#### 6.3.1 Hop 1: files on disk

The input quartet of Chapter 4 enters as YAML, plus one CSV.

The vehicle directory holds `vehicle.yaml`, and the files that it references. Those references are
relative paths, resolved against the vehicle directory:

```yaml
# data/vehicles/tesla_model3_rwd/vehicle.yaml (excerpt)
schema: vehicle/1.0
name: "Tesla Model 3 RWD (HV variant)"
chassis:
  mass_kg: 1765.0
  cg: [1.524, 0.0, 0.45]
  wheelbase_m: 2.875
  track_m: [1.58, 1.58]
tires:
  front: tyr/road.tyr.yaml
  rear: tyr/road.tyr.yaml
drivetrain:
  units:
    - source: ptm/du_medium.ptm.yaml
      thermal: emotor/rear_du.emotor.yaml
      path:
        - diff: { type: open }
      wheels: [RL, RR]
battery:
  model: rc_pairs
  params: battery/pack_800v.battery.yaml
```

The track directory holds `track.yaml`, plus `centerline.csv`, whose columns are `s_m, x_m, y_m,
z_m, banking_deg, width_left_m, width_right_m, grip_scale`.

An optional `conditions.yaml` and `sim.yaml` may sit next to `vehicle.yaml`. When they are absent,
full defaults apply: an ISA atmosphere, and tier `t1`.

#### 6.3.2 Hop 2: schema types, from `Vehicle` to `ResolvedVehicle`

`solve_lap` roots a filesystem loader at the vehicle directory, with `FsLoader::new(vehicle_dir)`.
It then calls `load_vehicle_with("vehicle.yaml", …)`.

The pipeline of §6.2 deserializes into the root schema struct, at
`crates/outlap-schema/src/vehicle/mod.rs:40`:

```rust
pub struct Vehicle {
    pub schema: SchemaVersion,        // e.g. vehicle/1.0
    pub extends: Option<PresetRef>,   // single-parent inheritance, resolved away
    pub name: String,
    pub chassis: Chassis,             // mass_kg, cg, inertia, wheelbase_m, track_m
    pub aero: Aero,                   // map ref + optional constant coefficients
    pub suspension: Suspension,
    pub tires: Tires,                 // front/rear .tyr references
    pub drivetrain: Drivetrain,       // topology graph: units → couplers → wheels
    pub ers: Option<Ers>,
    pub battery: Option<Battery>,
    pub brakes: Brakes,
    pub extensions: Extensions,       // x-* vendor keys, carried through
}
```

It then wraps it, at `crates/outlap-schema/src/load/mod.rs:47`:

```rust
pub struct ResolvedVehicle {
    pub spec: Vehicle,                // resolved, validated, extends applied
    pub provenance: ProvenanceMap,    // JSON pointer → Origin for every value
    pub report: LoadedModelReport,    // inherited/estimated/degraded/warnings + resolved_hash
}
```

The referenced `.tyr`, `.ptm`, and `.emotor` files are loaded and validated at this stage too. The
`battery.params` document, and the binary sidecars, are deferred to assembly time.

#### 6.3.3 Hop 3: the road, from `Track` to `T0Path`

`outlap-track` fits the centerline with a C² cubic spline, which is periodic for a closed circuit.
It fits the channels indexed by `s` with the shared monotone cubic Hermite.

The solver does not query the `Track` in its loop. `T0Path::from_track(&track, ds_m)` instead
samples it once, at a uniform step in arc length. The default is `DEFAULT_DS_M = 2.0` m, so the
4 649.8 m of Catalunya becomes 2 325 stations.

The result is a snapshot in structure-of-arrays form, at `crates/outlap-qss/src/path.rs:24`:

```rust
pub struct T0Path {
    pub s: Vec<f64>,            // arc-length stations, m
    pub kappa_l: Vec<f64>,      // road-plane lateral curvature κ_l, 1/m
    pub kappa_n: Vec<f64>,      // road-normal curvature κ_n, 1/m (crest unloads, dip loads)
    pub sin_b_cos_g: Vec<f64>,  // sinθ_b·cosθ_g  (banking θ_b, grade θ_g)
    pub cos_b_cos_g: Vec<f64>,  // cosθ_b·cosθ_g
    pub sin_g: Vec<f64>,        // sinθ_g (+ uphill)
    pub grip: Vec<f64>,         // per-station grip scale γ(s)
    pub ds: f64,                // uniform step (divides the length exactly)
    pub closed: bool,
}
```

At each station, the road inputs to the solver are therefore exactly three things: the curvature,
split into a lateral and a normal component on the banked road plane; the three trigonometric
factors that project gravity, which encode grade and banking; and a grip scale.

Signs follow ISO 8855: x forward, y left, z up. So $\kappa_h > 0$ is a left turn, a positive grade
is uphill, and positive banking raises the left edge.

If `sim.flat_track` is set, `from_track_flat` zeroes the grade, the banking, and the vertical
curvature instead.

#### 6.3.4 Hop 4: the solver vehicles, `T0Vehicle` and `T1Vehicle`

Two cold assembly functions reduce the same `ResolvedVehicle` and `Conditions` to solver structs
that hold numbers only. That is hard rule #4: one vehicle description, and every tier reads it.

Here is the point-mass reduction, at `crates/outlap-qss/src/vehicle.rs:59`:

```rust
pub struct T0Vehicle {
    pub mass_kg: f64,
    pub mu_x: f64,   // MF6.1 pure-slip Fx peak @ FNOMIN/p_cold, mean of axles
    pub mu_y: f64,   // MF6.1 pure-slip Fy peak, mean of axles
    pub qx: f64,     // lumped drag ½·ρ·CxA, N per (m/s)²
    pub qz: f64,     // lumped downforce ½·ρ·CzA, N per (m/s)²
    pub v_cap: f64,  // speed safety cap, m/s
    units: Vec<T0Unit>,   // per drive unit: MonotoneCubic torque envelope + folded gears
    ers: Option<T0Ers>,   // ERS reduced to a power cap with a speed taper
    notes: Vec<String>,   // every simplification, e.g. "braking is friction-limited only at T0"
}
```

Note what happened at this hop. The tire files became two friction coefficients, evaluated from the
validated MF6.1 model at nominal load and cold pressure, and not raw coefficients. `conditions.air`
became an air density, through the ideal-gas law, inside `qx` and `qz`. And the drivetrain graph
was folded into precomputed constants for each gear.

Its one hot query is `tractive_force(v) -> f64`: speed in, available drive force out, and no
allocation.

The double-track reduction is `T1Vehicle`, at `crates/outlap-qss/src/t1/vehicle.rs:31`. It keeps
much more.

It keeps `mass_kg` and `izz`; the CG-to-axle distances `a_f` and `b_r`; `wheelbase_m`; the track
widths `t_f` and `t_r`; the CG height `h_cg`; the heights of the roll axis and the roll centers;
the shares of roll stiffness; the *full tire model for each axle*, as `tire_front` and `tire_rear`,
of type `TireModel<f64>`; the cold pressures; the constant and reference aero terms `qx`, `qz_f`,
and `qz_r`; the air density `rho`; the ride rates; the static ride heights; the anti-dive and
anti-squat fractions; an optional `AeroMap` over ride height and yaw; the mask of driven wheels;
the brake bias; and the topology powertrain.

It powers the trim solver, and the envelope generator; see Chapter 8.

At the native edge only, `outlap-py` then does two more things. It installs the binary parquet
sidecars into the T1 vehicle, through `install_sidecars` at `crates/outlap-py/src/lib.rs:744`;
those are the aero map, and the efficiency and loss tables of the `.ptm` files. And it builds the
optional stack of slow states, through `build_slow_stack`: a machine thermal `Network` plus a
battery `Pack`, from the vehicle's own `battery.params` and `.emotor` references.

A missing file is skipped, with a recorded note. A file that is present but broken is a hard error.

#### 6.3.5 Hop 5: the g-g-g-v envelope

Before the lap solve, `GgvEnvelope::generate(&t1_vehicle, &sim.envelope, fz_coupling)` sweeps the
T1 trim over a grid. The default is 40 speed points, 25 points of normalized longitudinal
acceleration, and 7 `g_normal` points.

It stores the boundary of tire grip as `GriddedMapN` tables. It also stores sensitivity fields at
each node (§8.4.4), a reference curve `drag_accel(v)`, and `mass_ref`; see
`crates/outlap-qss/src/t1/envelope.rs:153`.

This is a cold step at the scale of seconds. `outlap-py` therefore caches it for each process. The
key is the resolved hash, a fingerprint of the sidecars, the conditions, the grid, and the coupling
mode.

The physics belongs to Chapter 8. Here it is just one more immutable product of assembly, handed to
the solver.

#### 6.3.6 Hop 6: the solve, and what goes in and out

Tier dispatch happens once, at assembly time, in `solve_lap`, at `crates/outlap-py/src/lib.rs`.

`Tier::T3` returns a typed error. `Tier::T2` returns a pointer to its own entry point, which is
indexed by time: `solve_transient_lap`; see §8.7.

`t0` and `t1` both assemble the T1 vehicle, because the envelope needs it. They then generate or
fetch the envelope, assemble the T0 vehicle, and call `solve_t0` or `solve_t1`; see
`crates/outlap-qss/src/qss.rs`.

The hot kernel is
`solve_into_ggv(vehicle, envelope, path, workspace) -> Result<f64, T0Error>`, at
`crates/outlap-qss/src/solver.rs:423`.

Its inputs, at each station `i`, are precisely the `T0Path` slices above. Its scratch state is a
workspace that the caller owns, allocated in advance, at `crates/outlap-qss/src/result.rs:27`:

```rust
pub struct T0Workspace {
    pub v_lim: Vec<f64>,  // curvature-limited speed per station, m/s
    pub v: Vec<f64>,      // solved speed per station, m/s
}
```

The sweep fills `v_lim`. It runs a forward pass limited by traction, and a backward pass limited by
braking. It takes the pointwise minima. It allocates nothing.

The owning wrapper packages the channels, at `crates/outlap-qss/src/result.rs:56`:

```rust
pub struct LapResult {
    pub s: Vec<f64>,           // arc-length stations, m
    pub v: Vec<f64>,           // speed, m/s
    pub ax: Vec<f64>,          // longitudinal acceleration, m/s²
    pub ay: Vec<f64>,          // lateral acceleration (ISO 8855, + left), m/s²
    pub t: Vec<f64>,           // cumulative time, s
    pub lap_time_s: f64,
    pub line: LineDescriptor,  // Centerline | MinCurvature{..} | File{..}
    pub resolved_hash: String, // which car spec produced this
    pub notes: Vec<String>,    // nothing silent
}
```

The layer for tier dispatch wraps that in `QssLap`, at `crates/outlap-qss/src/qss.rs:110`. That
holds the `LapResult`, plus the recorded `tier`, `fz_coupling`, and `flat_track`, plus three
optional logs, plus the returnable `envelope: Option<GgvEnvelope>`.

The three logs are these.

`wheels: Option<WheelLog>` holds, at each station, arrays over `[FL, FR, RL, RR]` of
`vertical_load_n`, `slip_ratio`, `slip_angle_rad`, `force_long_n`, and `force_lat_n`. It exists at
t1 only, and re-trimming every station produces it.

`setup: Option<SetupLog>` holds `understeer_gradient` and `aero_front_share`. It exists at t1 only.

`slow: Option<SlowLog>` holds `state_of_charge` and `machine_temp_c`. It is present whenever a
coupled stack of battery and thermal state was active, at either tier.

#### 6.3.7 Hop 7: across the PyO3 boundary, to xarray

`qss_lap_to_py`, at `crates/outlap-py/src/lib.rs:1069`, converts a `QssLap` into the frozen Python
class `Lap`.

It does three things. It reconstructs the world positions `x`, `y`, and `z`, by querying the track
at each station. It flattens the per-wheel logs into row-major buffers of `n × 4`. And it
stringifies the enums, giving `tier="t1"` and `fz_coupling="one_step_lag"`.

The channel methods, such as `lap.v()` and `lap.vertical_load_n()`, return fresh numpy arrays. The
per-wheel and setup channels return `None` on a t0 lap.

Finally, the pure-Python veneer `outlap.core.lap_dataset`, at `python/src/outlap/core.py:84`,
assembles the object at the result boundary that the project commits to.

That object is an `xarray.Dataset`. Its dimension is `s`, which is arc length in m. When per-wheel
channels exist, it also has the dimension `wheel`, over `FL/FR/RL/RR`.

It carries up to 16 data variables: `v`, `ax`, `ay`, `t`, `x`, `y`, `z`, the five per-wheel
channels, `understeer_gradient`, `aero_front_share`, `state_of_charge`, and `machine_temp_c`.

It carries the attrs `lap_time_s`, `resolved_hash`, `tier`, `fz_coupling`, `flat_track`, which is
an int because netCDF attrs have no boolean type, and `notes`, which is a tuple.

For the Model 3 on Catalunya, this is a dataset of 2325 stations and 595 kB, with
`lap_time_s = 148.081…`. Chapter 10 gives the full contract.

Here is the whole journey in one picture:

```text
vehicle.yaml ─┐  (stages 0–9: parse → extends → validate → estimate → hash)
 .tyr/.ptm/…──┤→ Vehicle → ResolvedVehicle {spec, provenance, report}
              │                   │
track.yaml ───┼→ Track ──────────┼→ T0Path {s, κ_l, κ_n, trig, grip}      (cold)
conditions ───┤                   ├→ T0Vehicle {m, μx, μy, qx, qz, drive}  (cold)
sim.yaml ─────┘                   ├→ T1Vehicle + sidecars → GgvEnvelope    (cold, cached)
                                  └→ slow stack (Network + Pack), if refs exist
                                          │
                    solve_into_ggv(T0Vehicle, GgvEnvelope, T0Path, T0Workspace)   (HOT)
                    [+ per-station re-trim at t1; + slow-state outer march]
                                          │
              LapResult → QssLap → Lap (PyO3, numpy) → xarray.Dataset
```

### 6.4 The tire call: `SlipState` in, `TireForces` out

The innermost call in the physics has the same disciplined shape.

The T1 trim builds, for each wheel, a state of the contact patch, at
`crates/outlap-tire/src/slip.rs:30`:

```rust
pub struct SlipState<T> {
    pub kappa: T,       // longitudinal slip ratio κ = −V_sx/|V_cx|; > 0 driving
    pub alpha: T,       // side-slip angle α, rad
    pub gamma: T,       // inclination (camber) angle γ, rad
    pub fz: T,          // normal load F_z, N (compressive-positive; ≤ 0 → all-zero forces)
    pub p: T,           // inflation pressure, Pa (.tyr stores kPa — converted at the seam)
    pub vx: T,          // contact-center forward velocity, m/s (sign meaningful)
    pub mu_scale_x: T,  // runtime friction multiplier hooks (1.0 in v0.2)
    pub mu_scale_y: T,
}
```

It then calls `TireModel::forces(&SlipState) -> TireForces`. That is a match on the static enum,
which is `Mf61` or `Brush`. Each arm is a pure kernel, panic-free and allocation-free, generic over
`f32` and `f64`.

The output is at `slip.rs:67`:

```rust
pub struct TireForces<T> {
    pub fx: T,  // longitudinal force, N
    pub fy: T,  // lateral force, N
    pub mz: T,  // aligning moment, N·m
    pub mx: T,  // overturning moment, N·m
    pub my: T,  // rolling-resistance moment, N·m
}
```

Signs follow the ISO-W convention of a modern `.tir` file, on ISO 8855 axes. A positive `alpha`
slides the patch toward +y, which is left, and it produces a *negative* `fy` on a normal tire. The
module doc in `slip.rs` documents every trap with signs. Chapter 7, Physics I, covers what happens
inside.

### 6.5 The three plugin points: a roadmap, not yet shipped

The extension policy of the project is deliberate and narrow. There are **exactly three plugin
points**. Everything else stays a curated core enum, so that the hot path never grows dynamic
dispatch.

1. **Custom blocks**, through a Rust trait with registration at compile time. A plugin crate
   depends on `outlap-core`, and registers its blocks. Users then build a custom binary, or
   upstream the block.
2. **Tire models**, through a stable C-ABI "Standard Tire Interface", which is CPU-only by
   contract. A closed or third-party tire model can therefore be loaded as a shared library.
3. **Controllers**, through the same trait mechanism, running as blocks in the `control` phase.
   They are Rust or C-ABI only. Python never runs in a timestep.

Be aware of the status: **none of these exists in code at v0.2.5.** There is no plugin trait, no
registration mechanism, and no C-ABI header anywhere in `crates/` yet. Plugin traits, and Python
entry points, are scheduled for later; see Chapter 15.

What ships today is the curated-enum half of the decision. `outlap_tire::TireModel` is an example:
"the static (no-`dyn`) choice", at `crates/outlap-tire/src/model.rs:2`. It picks MF6.1 when the
full core of pure-slip force is present, and the brush model otherwise.

If you want a custom model today, the path is a fork. It is not a plugin.

### 6.6 The rules for staying wasm-clean

The core of outlap is required to compile for `wasm32-unknown-unknown`, which is WebAssembly with
no operating system. That target forbids access to a filesystem, threads, and clocks.

This is a forcing function for good layering. All access to a source goes through the
`SourceLoader` trait, at `crates/outlap-schema/src/io.rs:36`. `FsLoader` sits behind the `std`
feature of `outlap-schema`, and the parquet decoder behind its `parquet` feature. A wasm consumer
uses the in-memory `MemLoader`, and depends on the schema crate with `default-features = false`.

CI enforces six wasm builds; see `.github/workflows/ci.yml`. They are `outlap-wasm`, in release;
`outlap-raceline`, because the `clarabel` QP solver is wasm-clean; `outlap-tire`; `outlap-qss`,
with the rayon-backed `parallel` feature off, which keeps the solver free of threads;
`outlap-vehicle`; and `outlap-transient`, so the whole transient tier stays wasm-clean.

`outlap-core`, `outlap-track`, and `outlap-thermal` are wasm-clean by construction, and they come
along as dependencies.

`outlap-py` is the deliberate exception. "This crate never builds for wasm", says
`crates/outlap-py/Cargo.toml`, because PyO3 and numpy are native only. That is exactly why parquet
decoding and the envelope cache live there, on the native edge.

One honest caveat: `outlap-wasm` itself is an empty placeholder, so its build gate passes
trivially. The builds that actually keep the core honest are the ones for raceline, tire, qss,
vehicle, and transient.

### 6.7 The architecture of error handling

Errors are treated as a product surface. Each layer uses a different tool.

- **A typed enum on every public API**, through `thiserror`. There is `SchemaError`, with one
  variant for each stage of the pipeline, plus `TrackError`, `TireBuildError`, `T0Error`,
  `T1Error`, `QssError`, `ThermalError`, and `RacelineError`.

  Each variant states what went wrong, and, where that is useful, how to fix it.
  `T0Error::NoConstantAero` points at `allow_degraded`. `QssError::TierNotImplemented`, which is
  now reachable only for `t3`, names the alternatives.
- **miette diagnostics at the configuration surface.** A `SchemaError` variant carries the source
  file, labels with byte spans, and `#[help]` hints.

  A typo therefore renders as an underlined snippet, with "did you mean `mass_kg`?". The
  Levenshtein distance comes from `strsim`.

  A bare serde error reaching the user is considered a bug.
- **Panic-free solver kernels.** A function on the hot path returns a `Result`; an example is
  `solve_into_ggv -> Result<f64, T0Error>`.

  Physics invariants are checked with `debug_assert!`, so a release build pays nothing for them.

  Every working crate except `outlap-py` carries `#![forbid(unsafe_code)]`. `outlap-py` is the
  sanctioned crate for the foreign-function interface, because the macros of PyO3 generate `unsafe`
  glue.
- **`anyhow` at CLI edges only.** That is the convention for future command-line binaries. As of
  v0.2.5, no shipped crate uses `anyhow` at all. Every error is typed.
- **The Python boundary preserves the diagnostics.** `schema_err`, at
  `crates/outlap-py/src/lib.rs:164`, maps a missing file to `FileNotFoundError`, and everything
  else to `ValueError`. It explicitly appends the miette help line, because "Display alone drops
  them".

  A Python user therefore sees
  ``ValueError: unknown field `masss_kg`\nhelp: did you mean `mass_kg`?``.

The philosophy throughout is "nothing silent". A *missing* optional file — `sim.yaml`,
`conditions.yaml`, a sidecar, or the battery document — falls back to a default, with a recorded
note. A file that is *present but malformed* is always a hard error.

### 6.8 The rules for determinism

The same inputs must produce bit-identical outputs, across runs and across thread counts.

Here are the rules, and where they live in v0.2.

- **Fixed-step integrators only**, in a production path. `sim.integrator` offers Heun or RK4, at a
  fixed `dt_s`, for the T2 tier. The shipped thermal march uses a Crank–Nicolson step, which is
  unconditionally stable. There is no adaptive control of step size anywhere.
- **A fixed iteration count, instead of a tolerance, where order matters.** The coupling of slow
  states runs exactly `OUTER_ITERS = 2` passes of solve, march, and re-solve. It is "fixed (not
  tolerance-driven) for determinism"; see `crates/outlap-qss/src/qss.rs:43`.
- **Reductions in a fixed order.** A sum such as the accumulation of lap time runs in a fixed
  order.

  The optional parallelism in generating the envelope, through rayon, splits work over independent
  fibres of `(v, g_normal)`, and merges them in a fixed order; see `crates/outlap-qss/Cargo.toml`.
  A parallel build and a serial build therefore agree bitwise.
- **No fast-math.** No build flag anywhere relaxes the semantics of IEEE 754.
- **A counter-based RNG, keyed by `(seed, rollout, stream, step)`**, in the style of Philox or
  ChaCha8. This rule is locked in for the planned strategy layer with Monte Carlo; see Chapter 15.

  No RNG exists in the v0.2.5 core. The QSS solvers are fully deterministic functions. But the
  structure of the key is locked in now, so that a batch of rollouts can be replayed and sliced
  later.
- **Recorded numerics.** A setting that changes results is embedded in every result artifact. That
  covers the mode of Fz coupling, `one_step_lag` against `fixed_point`; `flat_track`; and the
  resolved tier. They sit alongside the blake3 `resolved_hash` of the exact parameter set.

### 6.9 Python packaging: where Rust ends and Python begins

The split is deliberately thin.

`crates/outlap-py` compiles to a single `cdylib`, which is a shared library compatible with C. It
is named `outlap_core`. **maturin** builds it against PyO3 0.29, with `abi3-py312`, so one wheel for
each platform works on any CPython of version 3.12 or later.

Its own doc comment states the contract: "this layer only converts types and maps errors, never
adds logic"; see `crates/outlap-py/src/lib.rs:5-7`.

It exposes the frozen classes `Tyre`, `Track`, `Raceline`, `Lap`, and `Envelope`. It exposes the
functions `solve_lap`, `min_curvature`, and `vehicle_report`, and the constant `DEFAULT_DS_M`.
Typed stubs ship in `crates/outlap-py/outlap_core.pyi`.

The crate sets `test = false`. A Rust test harness cannot link against Python, so the pytest suite
on the Python side is its test surface.

The pure-Python package `outlap` lives in `python/`. uv manages it, and it declares
`requires-python >= 3.12`.

It declares `outlap-core` as a path dependency on `../crates/outlap-py`. `uv sync` therefore
compiles the extension automatically, and a Rust toolchain is required. Its `cache-keys` cover
every `*.rs` file in the workspace, so any edit to the Rust triggers a rebuild.

The typed user API lives in `outlap.core`, at `python/src/outlap/core.py`. It provides broadcasting
in the numpy style for tire sweeps, through `tyre_forces`. And it provides the converters to
xarray: `lap_dataset`, `solve_lap_dataset`, and `track_dataset`. This is where results become
labelled Datasets, which is the format that the project commits to across the boundary.

Two practical warts are worth knowing. `import outlap` itself currently exposes only a hello-world
`main()` stub, so always import from `outlap.core`. And an extension built on the debug profile
makes the first generation of an envelope take minutes, so set
`MATURIN_PEP517_ARGS=--profile release` before `uv sync`, exactly as CI does; see
`.github/workflows/ci.yml`, lines 30 to 34.

Here is the division of labour, in one sentence each:

| Layer | Owns | Never does |
|---|---|---|
| Rust crates (`crates/*`) | All physics, validation, solving; every hot loop | Talk to Python mid-solve |
| `outlap-py` (extension) | Type conversion, error mapping, native-edge assembly (sidecars, envelope cache, slow stack) | Add physics or defaults |
| `outlap.core` (Python) | Broadcasting, xarray labelling, ergonomics | Re-implement anything the core computes |

Note that `solve_lap` currently holds the global interpreter lock of Python, or GIL, for its whole
duration. Releasing it is deferred to the API for batches and sweeps; see Chapter 15. Parallel laps
from Python threads will therefore not overlap yet.

Chapter 10 covers the full Python surface. Chapter 11 covers the importers and the tooling that
feed it.


---

## 7. Physics I: tires and aerodynamics

*What you will learn: how outlap turns slip at the contact patch into forces, with the Magic Formula
6.1 tire model, and what every field in the `SlipState → TireForces` contract means. How a `.tyr`
file selects between the empirical MF6.1 model and the physical brush model, and how the T0 solver
distills a whole tire into two friction numbers. On the aero side: the path for a road car, with
constant coefficients; the downforce map over ride height and yaw; and the fixed-point "platform
equilibrium" that couples the two.*

Tires and aerodynamics are the two producers of force that everything else in outlap serves.

Every horizontal force that accelerates, brakes, or turns the car passes through four contact
patches, each roughly the size of your hand. Aerodynamics decides how hard those patches are
pressed into the road, and how much drag the powertrain must overcome.

This chapter explains both models as they are actually implemented, with the real struct fields,
file formats, and shipped numbers. The primer in Chapter 2 gives the intuition. Here we make it
precise.

### 7.1 The tire crate, and its contract

The tire model lives in `crates/outlap-tire`. It implements the steady-state Magic Formula 6.1, or
MF6.1, plus a physical brush model, plus a module for first-order relaxation that the T2 tier
integrates live.

It is implemented clean-room, from the Pacejka book: H. B. Pacejka, *Tire and Vehicle Dynamics*,
3rd ed., 2012, Chapter 4, §4.3.2, equations 4.E1 to 4.E78. The extensions for inflation pressure
come from Besselink, Schmeitz & Pacejka, *Vehicle System Dynamics* 48(S1), 2010.

The theory page `docs/theory/mf61-steady-state.md` carries the full map of equations, and the
citations.

Every evaluation kernel in the crate is pure, panic-free, and allocation-free, which CI enforces.
Each is generic over `f32` and `f64`.

The crate does no file IO. It consumes a `.tyr` document that `outlap-schema` has already loaded.
That is what keeps it buildable for `wasm32-unknown-unknown`.

The whole crate speaks one contract for input and output, defined in
`crates/outlap-tire/src/slip.rs`:

```rust
pub struct SlipState<T> {
    pub kappa: T,      // longitudinal slip ratio, dimensionless; > 0 driving
    pub alpha: T,      // side-slip angle, rad
    pub gamma: T,      // inclination (camber) angle, rad
    pub fz: T,         // normal load, N (compressive-positive; <= 0 => all-zero output)
    pub p: T,          // inflation pressure, Pa
    pub vx: T,         // contact-center forward velocity, m/s (sign meaningful)
    pub mu_scale_x: T, // runtime longitudinal friction multiplier (thermal hook; 1.0 today)
    pub mu_scale_y: T, // runtime lateral friction multiplier
}

pub struct TireForces<T> {
    pub fx: T, // longitudinal force, N
    pub fy: T, // lateral force, N
    pub mz: T, // aligning moment, N·m (about +z, up)
    pub mx: T, // overturning moment, N·m (about +x, forward)
    pub my: T, // rolling-resistance moment, N·m (about +y, left)
}
```

`SlipState::new(kappa, alpha, gamma, fz, p, vx)` fills both `mu_scale_*` fields with 1.0.

Those two multipliers are the hook through which the planned thermal model of the tire will one day
modulate grip. Today their only production consumer is the T1 envelope generator, which perturbs
them to measure the sensitivity of grip. See §7.6 and Chapter 8, Physics II.

Here are the five outputs, in words:

| output | name | what it is |
|---|---|---|
| `fx` | longitudinal force | drive/brake force in the wheel plane |
| `fy` | lateral force | cornering force |
| `mz` | aligning moment | the torque that tries to straighten the steered wheel — what you feel in the steering |
| `mx` | overturning moment | the contact patch's roll torque about the wheel's forward axis |
| `my` | rolling-resistance moment | the torque opposing rotation; a small, ever-present power drain |

### 7.2 Slip, and the sign contract

A tire only produces force when its contact patch *slides* slightly, relative to the road. There
are two measures of that sliding.

- **Slip ratio** $\kappa$, or kappa, says how much faster or slower the tire surface moves than the
  road under it, longitudinally.

  outlap uses the ISO-W definition, $\kappa = -V_{sx}/|V_{cx}|$, where $V_{sx}$ is the longitudinal
  sliding velocity of the contact patch, and $V_{cx}$ the forward velocity of the wheel center.

  It is dimensionless, and it is not a percentage. $\kappa > 0$ when driving. $\kappa < 0$ when
  braking. And $\kappa = -1$ is a locked wheel, rolling forward.
- **Slip angle** $\alpha$, or alpha, is the angle between where the wheel points and where it
  actually travels: $\tan\alpha = V_{sy}/|V_{cx}|$, in radians.

The axes are ISO 8855 throughout: x forward, y left, z up.

That convention has consequences that the code treats as load-bearing, because a single stray
absolute value silently breaks the physics.

- $\alpha > 0$ means the contact patch slides toward +y, which is left. A normal tire therefore
  pushes back with $F_y < 0$.

  The cornering stiffness $K_{y\alpha} = \partial F_y/\partial\alpha|_0$ therefore carries the sign
  of the `.tir` coefficient `PKY1`, which is **negative** in an ISO-W parameter set. The Pacejka
  book tire ships `PKY1: -14.95`.
- The aligning moment is $M_z = -t\,F_y + M_{zr}$: the pneumatic trail $t$ times the lateral force,
  plus a residual term.

  It restores, meaning that it tries to reduce the slip angle, precisely *because* $F_y$ is
  negative for a positive $\alpha$.
- $F_z \le 0$, which is an airborne wheel, short-circuits every model to outputs that are exactly
  zero, through `TireForces::zero()`.
- Running in reverse, where $V_{cx} < 0$, enters only through $\operatorname{sgn}(V_{cx})$ factors
  inside specific equations.

  The `sgn` of the implementation maps 0 to +1. It is a branch selector, and not a true signum. A
  standstill therefore does not annihilate the lateral force with a 0/0.
- **Camber** $\gamma$, or gamma, is also called the inclination angle. It is the lean of the wheel
  about its own x-axis. The top of the tire leans toward +y when $\gamma > 0$.

One quirk with units is worth remembering. `SlipState.p` is in pascal. But the `.tyr` file stores
the cold inflation pressure `thermal.p_cold` in **kPa**. That is a boundary of the file format,
like RPM and °C elsewhere.

Every consumer converts at the seam. For example, `crates/outlap-qss/src/vehicle.rs` computes
`cold_pressure_pa = 1000.0 * p_cold`.

### 7.3 The idea behind the Magic Formula

The Magic Formula is not derived from physics. It is an empirical curve fit: an equation with just
enough freedom in its shape to reproduce a measured tire curve.

Its core, from Pacejka 2012, is a sine of an arctangent:

$$y(x) = D \sin\!\big(C \arctan\!\big(Bx - E\,(Bx - \arctan Bx)\big)\big)$$

$x$ is a slip quantity, either $\kappa$ or $\alpha$, plus a small horizontal shift $S_H$. $y$ is a
force, plus a small vertical shift $S_V$. Four named factors sculpt the curve:

| factor | name | what it controls |
|---|---|---|
| $B$ | stiffness factor | the slope near zero slip (with $C$, $D$: origin slope $= BCD$, the slip stiffness) |
| $C$ | shape factor | how far past the peak the curve falls — the "character" of the falloff |
| $D$ | peak value | the maximum force — essentially $\mu F_z$ |
| $E$ | curvature factor | how sharp or gentle the peak is; clamped $\le 1$ in code (beyond 1 the curve folds back) |

Why this shape?

For small $x$, the whole expression is nearly linear: $y \approx BCD\,x$. That is the elastic
regime, where the rubber deflects without sliding.

As $x$ grows, the arctangent saturates, and the sine passes through its maximum $D$. That is the
grip peak. The curve then falls off, as more of the contact patch slides.

That rise, peak, and fall is exactly what every measured tire curve looks like. The friction-circle
story of Chapter 2 lives on top of it.

MF6.1 is the 2012-generation formulation of that idea. Each of $B$, $C$, $D$, $E$, $S_H$, and $S_V$
becomes a small polynomial in normalized load, inflation pressure, and camber. Their named
coefficients — `PCX1`, `PDY1`, `PKY1`, and so on — are what a fitting tool identifies from test
data.

Two normalized inputs appear everywhere:

$$df_z = \frac{F_z - F'_{z0}}{F'_{z0}}, \qquad dp_i = \frac{p - p_0}{p_0}$$

$df_z$ is the fractional deviation of load from the scaled nominal load
$F'_{z0} = \lambda_{Fz0}\cdot\texttt{FNOMIN}$. $dp_i$ is the fractional deviation of pressure from
`NOMPRES`.

This is how the model captures **load sensitivity**: the crucial fact, from Chapter 2, that the
friction *coefficient* falls as load rises. It does so through terms such as
$\mu_x = (\texttt{PDX1} + \texttt{PDX2}\,df_z)(\dots)$, in which `PDX2` is negative on a real tire.

### 7.4 MF6.1 as implemented

One evaluation of `Mf61::forces(&SlipState)`, in `crates/outlap-tire/src/mf61/`, composes five
channels:

$$
\begin{aligned}
F_x &= G_{x\alpha}(\alpha^*)\cdot F_{x0}(\kappa) \\
F_y &= G_{y\kappa}(\kappa)\cdot F_{y0}(\alpha^*) + S_{Vy\kappa}(\kappa) \\
M_z &= -t(\alpha_{t,eq})\cdot(F_y - S_{Vy\kappa}) + M_{zr}(\alpha_{r,eq}) + s\cdot F_x
\end{aligned}
$$

It adds $M_x$, from eq. 4.E69, and $M_y$, from eq. 4.E70.

Turn-slip, which covers parking maneuvers, is omitted in v1. Every $\zeta$ factor of the book
equations is a named constant, fixed at 1. A later upgrade is therefore a diff, not a rewrite.

#### 7.4.1 Pure slip: Fx0 and Fy0

$F_{x0}(\kappa)$, from eqs. 4.E9 to 4.E18 in `mf61/fx.rs`, and $F_{y0}(\alpha)$, from eqs. 4.E19 to
4.E30 in `mf61/fy.rs`, are the sine magic formula, with factors that depend on load, pressure, and
camber.

The code is written in the symbols of the paper, with an anchor to an equation number on each line.
The actual formula line in `fx.rs` reads:

```rust
// Fx0 (4.E9): the magic formula proper.
let arg = bx * kx;
let fx0 = dx * (cx * (arg - ex * (arg - arg.atan())).atan()).sin() + s_vx;
```

The three modifier inputs route differently through the equations:

| input | longitudinal ($F_{x0}$) | lateral ($F_{y0}$, $M_z$) |
|---|---|---|
| load | $df_z$ in $D_x$, $E_x$, $K_{x\kappa}$, shifts | $df_z$ in $D_y$, $K_{y\alpha}$, trail, residual |
| pressure | Besselink `PPX1..4` (stiffness, peak) | `PPY1..5`, `PPZ1/2` — inert without `NOMPRES` |
| camber | raw $\gamma^2$ (`PDX3`) | $\gamma^* = \sin\gamma$ and its powers |

The shifts $S_H$ and $S_V$ represent ply-steer and conicity, which are small asymmetries from
manufacturing. They mean that a real curve does not pass exactly through the origin. That is why
the peak extractor of §7.6 scans both signs of slip.

#### 7.4.2 Combined slip: cosine weighting

When a tire brakes *and* corners at once, each force steals grip from the other. That is the
friction circle.

MF6.1 models it with **cosine weighting**, in eqs. 4.E50 to 4.E67, in `mf61/combined.rs`. It does
not use a geometric friction ellipse.

$F_{x0}$ is multiplied by a weight $G_{x\alpha} \in (0, 1]$, which is a normalized *cosine* magic
formula in the other slip quantity, $\alpha$. Symmetrically, $F_{y0}$ is multiplied by
$G_{y\kappa}$. A small ply-steer shift induced by $\kappa$, called $S_{Vy\kappa}$, is added.

Cornering hard therefore reduces the longitudinal force available, and the reverse holds too. The
shape is fitted to data, rather than assumed to be elliptical.

Each normalizing denominator carries a guard that floors its magnitude and preserves its sign. A
parameter set that is hostile but still plausible can genuinely drive it toward zero.

#### 7.4.3 The aligning moment, and the minor moments

$M_z$, from eqs. 4.E31 to 4.E49 and 4.E71 to 4.E78, in `mf61/mz.rs`, composes three parts: the
pneumatic trail acting on the lateral force; a residual torque $M_{zr}$; and an $s\cdot F_x$ lever
arm, which is the longitudinal force acting at a small lateral offset $s$.

Three subtleties are operational. The golden cross-check pins all three, and
`docs/theory/mf61-steady-state.md` documents them.

- The entire lateral machinery of the aligning moment is evaluated at **zero camber**. Camber
  affects $M_z$ only through the coefficients of its own trail and residual. This matches the
  operational MF6.1 that `.tir` data is actually fitted against.
- The book writes a camber term in the $s$ lever arm, with the coefficients `SSZ3` and `SSZ4`. The
  operational implementations drop it. outlap therefore accepts those coefficients, but does not
  use them. Interoperability wins over literalism about the book.
- The $s\cdot F_x$ term applies under combined slip only. It is gated to $\kappa \ne 0$. That is a
  deliberate step discontinuity at exactly $\kappa = 0$, and it matches the standard.

  The theory page explicitly warns you not to "smooth" it. Doing so breaks the golden cross-check.

$M_x$, the overturning moment, and $M_y$, the rolling resistance, consume the final combined
forces, in `mf61/mxmy.rs`.

Rolling resistance opposes rotation. Under ISO 8855, rolling forward spins the wheel about +y, so
$M_y < 0$ when $V_{cx} > 0$. The oracle goldens confirm this.

For $M_x$, outlap takes the printed book form,
$\cos(\texttt{QSX5}\,(\arctan(\texttt{QSX6}\,F_z/F_{z0}))^2)$. The widely used MFeval tool
evaluates $\arctan(x^2)$ there instead. That is a known discrepancy between the book and the tool,
and it is worth remembering if you compare outputs.

#### 7.4.4 Defaults, and graceful degradation

A `.tyr` file never needs the full set of about 150 coefficients. `mf61/params.rs` extracts what is
there into a dense typed struct, once, at assembly.

An absent coefficient defaults to 0, with these exceptions:

| default | coefficients | why |
|---|---|---|
| 1.0 | every `L*` scaling factor, `RCX1`, `RCY1`, `QCZ1`, `PKY2` | multiplicative identities; `PKY2` sits in an atan denominator |
| 2.0 | `PKY4` | a zero would collapse the cornering stiffness $K_{y\alpha} \equiv 0$ |
| 16.7 m/s | `LONGVL` (reference speed) | the book's conventional measurement speed |
| 1.0 m/s | `VXLOW` | reserved for the low-speed/relaxation model |

An absent `NOMPRES` disables every pressure term exactly, so $dp_i \equiv 0$ and $p/p_0 \equiv 1$.
A pressure sweep on such a tire therefore exercises nothing, and the loader says so.

A coefficient family that is wholly absent degrades to zero output. With no `QDZ*`, $M_z \equiv 0$.
With no `R*`, combined slip equals pure slip.

**Every** degradation is emitted as a note into the loaded-model report. Nothing is silent, under
the assembly rules of Chapter 4.

### 7.5 The `.tyr` file, and selecting a model

A tire ships as a `.tyr` YAML document. Its schema is `tyr/1.0`, or `tyr/1.1` when it carries a
brush block. It has five blocks:

```yaml
schema: tyr/1.0
mf61:                       # MF6.1 coefficients, keyed by their standard .tir names
  FNOMIN: 4000.0            # nominal load, N        (required)
  UNLOADED_RADIUS: 0.313    # m                      (required)
  PCX1: 1.685
  PDX1: 1.210
  PKY1: -14.95
  # ... ~60-150 more, sparse files are fine
thermal:                    # thermal-ring parameters — inert stubs today, EXCEPT p_cold
  p_cold: 220.0             # cold inflation pressure, kPa (load-bearing NOW)
  t_opt: 75.0               # ...
wear:                       # wear/cliff parameters — entirely future
  k_w: 0.0006               # ...
provenance:
  citation: "H. B. Pacejka, Tyre and Vehicle Dynamics, 2nd ed. (2006), Appendix 3, Table A3.1 ..."
  synthetic: false
```

The excerpt comes from the shipped `data/tires/pacejka_2006_205_60r15/car.tyr.yaml`. That is the
book's own validation tire. It also happens to be the road tire of the Tesla Model 3 reference
vehicle, verbatim, at `data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml`.

The same file therefore appears three times: as reference data, as the subject of the golden test,
and on the flagship road car.

Model selection happens once, at assembly, in `TireModel::from_tyr`, at
`crates/outlap-tire/src/model.rs`:

1. If the full core of pure-slip force is present — the eight `REQUIRED_FORCE_KEYS`, which are
   `PCX1, PDX1, PEX1, PKX1, PCY1, PDY1, PEY1, PKY1` — build **MF6.1**. It is the model of higher
   fidelity, and a partial set never constructs one.
2. Otherwise, if a `brush:` block is present, build the **brush** model (§7.7). A note in the
   loaded-model report then records that $M_x = M_y = 0$, and that camber and pressure are ignored.
3. Otherwise, raise `TireBuildError::NoForceModel`. Schema validation catches this earlier, so the
   error is defensive.

`FNOMIN` and `UNLOADED_RADIUS` are always required.

An unknown *coefficient name* inside `mf61:` is not fatal. You get a did-you-mean warning. That
differs from an unknown schema *field*, which is a hard error; see Chapter 5.

The thermal and wear blocks are required by the schema. But nothing consumes them, apart from
`p_cold`, until the thermal model of the tire lands. Every shipped dataset carries synthetic
placeholders there, clearly labelled.

`p_cold` is the exception. It is the operating inflation pressure at which every solver evaluates
the tire today.

A vehicle references its tires for each axle. Both are required, and there is no default:

```yaml
tires:
  front: tyr/road.tyr.yaml
  rear: tyr/road.tyr.yaml
```

Three tire datasets that cite the literature ship in `data/tires/`. They are the 205/60R15 book
tire of Pacejka; the TUM Roborace DevBot set, which is MF5.2 mapped to MF6.1, with peak
$\mu_x \approx 1.46$ and $\mu_y \approx 1.16$; and an MF6.1 transcription of the Perantoni &
Limebeer 2014 F1 tire, whose load-linear peak friction, such as $\mu_x$ falling from 1.75 at 2000 N
to 1.40 at 6000 N, maps exactly onto the $(\texttt{PDX1} + \texttt{PDX2}\,df_z)$ form.

Chapter 12, on the shipped data library, covers their provenance in detail. The `.tir` interchange
format, which is the industry text format that `.tyr` converts to and from, is covered with the
other tooling, in Chapter 11.

### 7.6 How T0 distills a tire into a peak μ

The T0 point-mass solver of Chapter 8 has no slip states. It needs exactly two numbers for each
car: a peak longitudinal friction coefficient $\mu_x$, and a peak lateral one, $\mu_y$.

Those are *extracted from the full tire model*, at assembly time, in
`crates/outlap-tire/src/mf61/peak.rs`:

$$\mu_x = \max_{\kappa}\frac{|F_x(\kappa)|}{F_z}, \qquad \mu_y = \max_{\alpha}\frac{|F_y(\alpha)|}{F_z}$$

They are evaluated at a fixed operating point: $\gamma = 0$, $V_{cx} = \texttt{LONGVL}$, and a load
and pressure that the caller chooses.

The search scans **both signs of slip**. The shift terms make a real curve asymmetric, and taking
the maximum of the two branches is the documented choice for a symmetric point-mass envelope.

It scans over the *physical* slip windows: $\kappa \in [-1, 1]$, and $\alpha \in [-0.5, 0.5]$ rad,
which is about ±28.6°. It uses a dense grid scan of 256 points, followed by 48 iterations of
golden-section refinement.

A search by grid and refinement is used instead of the closed-form peak, $D/F_z$, for two reasons.
It is robust to the edge cases of the $E$ clamp, and to a shifted curve.

And for a soft curve, where $C \le 1$, the supremum is only approached at unbounded slip. The
maximum over the window is then deliberately below the analytic asymptote. Grip that you can only
reach at infinite slip is unusable.

T0 assembly, in `crates/outlap-qss/src/vehicle.rs`, calls this for each axle, at `FNOMIN` and the
cold pressure. It then takes the **mean of the two axles**:

```rust
let mu_x = 0.5 * (axle_mu_x(&tm_front, &front) + axle_mu_x(&tm_rear, &rear));
let mu_y = 0.5 * (axle_mu_y(&tm_front, &front) + axle_mu_y(&tm_rear, &rear));
```

You can watch the load sensitivity yourself, through the Python API of Chapter 10:

```python
from outlap.core import Tyre

t = Tyre.load("data/tires/pacejka_2006_205_60r15/car.tyr.yaml")
for fz in (2000.0, 4000.0, 6000.0, 8000.0):
    print(fz, t.peak_mu(fz, t.p_cold))
```

On the shipped book tire this prints these values, rounded:

| $F_z$ (N) | $\mu_x$ | $\mu_y$ |
|---|---|---|
| 2000 | 1.228 | 1.119 |
| 4000 | 1.210 | 1.035 |
| 6000 | 1.191 | 0.950 |
| 8000 | 1.173 | 0.866 |

The lateral coefficient loses over 20 % of its value as the load quadruples. That is load
sensitivity, and it is the reason that load transfer costs total grip; see Chapter 2. The
longitudinal peak of this tire barely moves.

At `FNOMIN` = 4000 N, the T0 assembly would record $\mu_x = 1.210$ and $\mu_y = 1.035$ for an axle
of these tires.

One note about this tire. It is a 2nd-edition set. It records `NOMPRES`, but it carries no `PP*`
pressure coefficients, so the pressure argument is inert here.

The loaded-model report stays *silent* about that. Its note about disabled pressure fires only when
`NOMPRES` is absent entirely (§7.4.4). It does not fire for a tire like this one, where `NOMPRES`
is present and `PP*` is absent.

A sweep over slip angle, using `outlap.core.tyre_forces`, shows both the sign contract and the
collapse of the trail in action.

At `FNOMIN`, slip angles of 0°, 2°, 4°, and 8° give $F_y \approx$ +42, −1506, −2696, and −3627 N.
The +42 N at zero slip is the shift from ply-steer and conicity.

The same angles give $M_z \approx$ −11.3, +33.9, +51.1, and +24.9 N·m. The aligning moment
therefore peaks *before* the lateral force does, and then collapses as the trail shortens. That is
the cue of "light steering past the limit" that a racing driver relies on.

For a brush tire, the peak is simply its base friction $\mu_0$. It depends on neither load nor
pressure.

The T1 tier, by contrast, keeps the full `TireModel<f64>` for each axle alive. It evaluates it
inside its trim solver, with per-wheel $\kappa$, $\alpha$, and $F_z$, at $\gamma = 0$ — camber maps
are a planned addition, and the assembly note says so out loud — and at the cold pressure.

Its envelope generator perturbs `mu_scale` uniformly, by $1 \pm 0.15$, to build the corrections for
grip sensitivity of Chapter 8. That is today's only production use of the `mu_scale_*` hooks.

### 7.7 The brush model

The brush model lives in `crates/outlap-tire/src/brush.rs`, with the theory page
`docs/theory/brush-model.md`. It is the physical, first-principles counterpart to the empiricism of
MF6.1, implemented from Pacejka 2012, Chapter 3.

It exists for two reasons. It lets you define a usable tire from **four physical parameters**, when
no fitted coefficient set exists. And it is the pedagogical skeleton underneath the shape of the
Magic Formula: the reason a tire curve rises, peaks, and falls.

Picture the contact patch as a row of elastic bristles, with a **parabolic pressure distribution**,
$p(x) \propto 1 - (x/a)^2$, over the contact half-length $a$. Under slip, each bristle deflects
linearly, until its local shear exceeds the friction bound. After that it slides. Integrating the
region that adheres and the region that slides gives a closed form.

The theoretical slips are $\sigma_x = \kappa/(1+\kappa)$ and $\sigma_y = \tan\alpha/(1+\kappa)$.
The $1+\kappa$ is ε-guarded, so a locked wheel stays finite. The reduced slip is

$$\psi = \frac{\sqrt{(C_\kappa\sigma_x)^2 + (C_\alpha\sigma_y)^2}}{3\,\mu_0 F_z},$$

and the magnitude of the force is the cubic brush law:

$$|F| = 3\,\mu_0 F_z\,\psi\,(1 - \psi + \tfrac{\psi^2}{3}) \quad \text{for } \psi < 1, \qquad |F| = \mu_0 F_z \quad \text{for } \psi \ge 1 \text{ (full sliding)}.$$

$\psi(1-\psi+\psi^2/3)$ rises monotonically to $1/3$ at $\psi = 1$. The friction circle
$|F| \le \mu_0 F_z$ is therefore respected *by construction*.

The pneumatic trail has a closed form too:

$$t = \frac{a}{3}\cdot\frac{(1-\psi)^3}{1-\psi+\psi^2/3}, \qquad M_z = -t\,F_y$$

It is one third of the contact half-length at vanishing slip, and zero at full sliding.

$M_z$ restores, under the same sign contract as MF6.1. The collapse of the trail that you measured
numerically in §7.6 appears here in closed form.

Here is a brush block in a `tyr/1.1` document. This example is the synthetic test fixture
`crates/outlap-schema/tests/fixtures/tyr/brush_only.tyr.yaml`:

```yaml
schema: tyr/1.1
mf61: { FNOMIN: 4000.0, UNLOADED_RADIUS: 0.33 }   # structural keys still required
brush:
  c_kappa_n: 150000.0          # C_kappa, longitudinal tread stiffness, N
  c_alpha_n_per_rad: 120000.0  # C_alpha, cornering stiffness, N/rad
  mu0: 1.20                    # base friction
  patch_half_length_m: 0.10    # a, m
  pressure_profile: parabolic
```

The omissions are documented, and not silent. Camber and pressure are accepted and ignored.
$M_x = M_y \equiv 0$. Assembly surfaces all of this as notes in the loaded-model report.

One sharp edge is worth knowing. The brush model is selected only by the `REQUIRED_FORCE_KEYS` gate
in `TireModel::from_tyr`, which is what the QSS solvers call. A brush-only tire therefore works
there.

The Python `Tyre` class of Chapter 10 does *not* go through that gate. It calls `Mf61::from_tyr`
directly, which validates only `FNOMIN` and `UNLOADED_RADIUS`.

A brush-only `.tyr` loaded through `Tyre.load` therefore does **not** error. It silently builds a
degenerate MF6.1 model in which everything is zero: the peak μ, and every force.

Exercise a brush tire through the solvers. Do not exercise it through `Tyre`.

### 7.8 Fitting your own tire: `outlap.tirefit`

Suppose you have tire test data, from the FSAE Tire Test Consortium, or TTC, for example. The
`outlap.tirefit` package, at `python/src/outlap/tirefit/`, fits an MF6.1 coefficient set from it:

```bash
python -m outlap.tirefit fit run1.mat run2.mat --unloaded-radius 0.26 -o car.tyr.yaml --report-dir report/
python -m outlap.tirefit synth car.tyr.yaml -o synth.csv --seed 0
```

There are three stages, at a high level.

1. **Ingestion**, in `tirefit/data.py`. It reads TTC `.mat` files, in v7 and in v7.3 or HDF5; TTC
   ASCII `.dat`; and headered `.csv`. It converts them into arrays in SI units and the ISO 8855
   convention.

   TTC channels arrive in the SAE tire axis system, in which z points down. The conversion is the
   proper rotation by π about x, which negates $\alpha$, $\gamma$, $F_y$, $F_z$, and $M_z$. It also
   converts degrees to radians, kPa to Pa, and kph to m/s. That happens at this boundary only.
2. **The forward model**, in `tirefit/mf61.py`. It is a vectorized clean-room mirror of the Rust
   kernels, written in numpy. It is validated against the *same* committed golden CSVs, under the
   same tolerance rule.

   A parameter set fitted here therefore evaluates identically inside the solver, including the
   operational conventions of §7.4.3.
3. **The staged fit**, in `tirefit/stages.py`. It runs a deterministic sequence: nominals, then
   pure $F_{x0}$, then pure $F_{y0}$, then combined, then $M_z$, then $M_x$.

   Each stage frees a documented subset of coefficients, and minimizes residuals normalized by
   load, with `scipy.optimize.least_squares`. Install the `tire-fit` extra to get scipy.

   A stage that the data cannot support is skipped, and reported. Camber coefficients are freed
   only if the data has a spread in camber. A moment stage runs only if there is real signal.

   The `QSY*` family, for rolling resistance, is never freed. A TTC rig logs no $M_y$ channel. Set
   those coefficients by hand, from coast-down data.

The `synth` command generates a deterministic synthetic dataset from an existing `.tyr`, in CSV
with TTC signs. That is the round-trip harness, which proves that the fit recovers known
coefficients.

One policy you must know before using any of this: **parsers yes; redistribution of TTC data, or of
a parameter set derived from TTC data, no.**

TTC data is locked to members. Keep raw files in the gitignored `ttc-data/` directory. Never
publish a file, a fitted coefficient set, or a fit report derived from them.

Synthetic data, and sets that cite the literature, are the only things that ship with outlap.

Chapter 11, on importers and tooling, gives the full story on tooling, including the CLI for the
`.tir` codec.

### 7.9 Aerodynamics I: the path with constant coefficients

Aerodynamics enters the vehicle description as the `aero:` block of `vehicle.yaml`. The schema is
in `crates/outlap-schema/src/vehicle/aero.rs`.

The block has two representations. One is a gridded map, which is the primary one, and the next
section covers it. The other is an optional constant block. That is the degenerate case, and it is
entirely adequate for a road car:

```yaml
aero:
  map: aero/none.parquet    # deliberately-absent placeholder: the constant block carries
  axes: []
  constant:
    cx_a_m2: 0.51           # drag area C_x·A, m²
    cz_front_a_m2: 0.0      # front downforce area C_z,front·A, m²
    cz_rear_a_m2: 0.0       # rear downforce area C_z,rear·A, m²
```

That is the shipped Tesla Model 3 RWD: a drag area of 0.51 m², which is a public figure, and no
downforce.

All three numbers are products of a *coefficient and a frontal area*, in m². That is what a wind
tunnel actually measures, so no separate reference area is needed.

At assembly, T1 folds in the air density $\rho$ from the session conditions, through the ideal-gas
law: $\rho = 100\,p_{\mathrm{hPa}} / (287.05\,(T_{°C} + 273.15))$. See
`crates/outlap-qss/src/t1/vehicle.rs`. This is one place where the separation of car and
environment in the input quartet pays off.

That gives lumped terms, in N per (m/s)²:

$$q_x = \tfrac{1}{2}\rho\,C_xA, \qquad q_{z,f} = \tfrac{1}{2}\rho\,C_{z,f}A, \qquad q_{z,r} = \tfrac{1}{2}\rho\,C_{z,r}A$$

Drag is therefore $q_x v^2$, and the downforce at each axle is $q_z v^2$. The downforce is added
straight into the static axle loads of the load-transfer model; see Chapter 8.

Downforce is why grip grows with speed, and why the g-g diagram widens into a funnel. Drag is what
the powertrain fights on the straights.

With no constant block, and `allow_degraded: true`, the aero degrades to zero, with a recorded
note. Otherwise it is a load error.

The split into *front* and *rear* downforce areas matters even in the constant case. Their ratio is
the **aero balance**, which is the front axle's share of total downforce.

That balance decides how the extra grip is distributed between the axles. It therefore decides
whether downforce pushes the car toward understeer or toward oversteer.

### 7.10 Aerodynamics II: the map over ride height and yaw

Constants cannot describe a downforce car, because its coefficients depend on the position of the
body relative to the ground.

Ground effect makes downforce rise as the floor gets closer to the road. Rake, which is the
nose-down attitude in pitch, shifts the balance. And yawing the body spoils the flow.

The primary representation is therefore a gridded map; see `crates/outlap-qss/src/t1/aero.rs`:

$$\{\,C_{z,\mathrm{front}}A,\; C_{z,\mathrm{rear}}A,\; C_xA\,\} = f(h_{\mathrm{front}},\, h_{\mathrm{rear}},\, \mathrm{yaw}\,[,\, \mathrm{DRS}])$$

The shipped F1 2026 reference car declares it like this, in
`data/vehicles/f1_2026/vehicle.yaml`:

```yaml
aero:
  map: aero/f1_2026.parquet
  axes: [ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag]
  constant:            # synthetic fallback used by tiers that don't consume the map
    cx_a_m2: 1.25
    cz_front_a_m2: 1.9
    cz_rear_a_m2: 2.6
```

The axis names are a fixed vocabulary: `ride_height_f_mm`, `ride_height_r_mm`, `yaw_deg`, and
`drs_flag`. Anything else is a typed `UnknownAeroAxis` error. An axis that is absent from a map is
simply not queried.

Note the units. Ride heights are in millimeters, and yaw is in degrees, *at the map boundary*. That
is the same convention at a file-format boundary as RPM and °C. Internally everything returns to
SI.

The sidecar is a long, tidy parquet table. It has those axis columns, plus three value columns:
`cz_front_a_m2`, `cz_rear_a_m2`, and `cx_a_m2`.

One `GriddedMapN` is built for each coefficient. That is the shared tensor-product monotone cubic
Hermite interpolant (Fritsch & Carlson 1980); there is one implementation for *all* gridded maps,
as Chapter 5 describes.

Every axis **clamps** outside its domain. The platform equilibrium can push a ride height below the
tabulated grid. Clamping then holds the coefficients at their edge values, rather than
extrapolating a ground-effect curve past the point where it is valid.

### 7.11 The aero-platform equilibrium

Here is the loop that makes a mapped aero genuinely different.

The coefficients depend on the ride heights. The ride heights depend on how hard the downforce
compresses the suspension. And that depends on the coefficients. It is a fixed point.

`AeroPlatform::equilibrium`, in `crates/outlap-qss/src/t1/aero.rs`, solves it by damped
fixed-point iteration.

Take the dynamic pressure $q_{\mathrm{dyn}} = \tfrac{1}{2}\rho v^2$, and the longitudinal load
transfer $T = m\,a_x\,h_{cg}/L$, moved onto the springs. The suspension geometry of anti-dive and
anti-squat reacts part of that transfer through the links instead; those geometries modulate this
*heave* path only, and never the steady-state wheel loads.

Each axle's ride height then updates as

$$h_a \leftarrow h_a + 0.6\,\Big[\max\!\Big(0,\; h_a^{\mathrm{static}} - \frac{q_{\mathrm{dyn}}\,C_{z,a}A + F_{lt,a}}{2\,k_a}\Big) - h_a\Big],$$

the map is re-evaluated at the new heights, and the loop repeats.

It runs up to 60 iterations, to a tolerance on ride height of $10^{-10}$ m, with an
under-relaxation factor of 0.6.

That tolerance of 0.1 nm looks absurd, and it is deliberate. It sits far below the residual
tolerance of the trim solver itself. The converged coefficients therefore behave as a *smooth*
function of the chassis state, and the nested fixed point never injects a discontinuity in
iteration count into the finite-difference Jacobian of the outer solver.

In the update, $h^{\mathrm{static}}$ is the design ride height, from
`suspension.*.static_ride_height_m`. On the F1 reference car that is 40 mm at the front and 90 mm
at the rear. $k_a$ is the ride rate at the wheel. And the $\max(0,\cdot)$ clamp is the car
"planking" on the road.

The output, `AeroLumped`, carries the effective $q_x$, $q_{z,f}$, and $q_{z,r}$, plus the converged
ride heights. A slot for a warm start lets the trim solver reuse the heights from the previous
evaluation. That cuts roughly 20 cold iterations to 1 or 2, which matters, because evaluating this
map is the dominant cost of a trim on a car that has one.

Here is the physics this buys. As speed rises, the platform sinks and rakes, ground effect
strengthens, and the *aero balance migrates with speed*. That is the defining behavior of a
downforce car, and no pair of constant $C_zA$ values can express it.

`T1Vehicle::aero_front_downforce_share_at(v)` reports it, and it lands at each station in the lap
results; see Chapter 10.

### 7.12 Sensitivity to yaw, and the mid-corner g-g diagram

The yaw axis of the map is fed with the **body-slip angle** $\beta$, or beta. That is the angle
between where the chassis points and where it travels. It is evaluated in degrees, *inside the trim
residual*; see `crates/outlap-qss/src/t1/trim.rs`.

$\beta$ is one of the unknowns of the trim. The finite-difference Jacobian therefore picks up
$\partial(\mathrm{downforce})/\partial\beta$ automatically.

In mid-corner, $\beta$ is nonzero. A car that is sensitive to yaw therefore has genuinely *less
downforce in the corner than its straight-line map value suggests*. The diagram of attainable
acceleration, which is the g-g diagram, reflects that.

This is the mechanism that reshapes the g-g diagram in mid-corner, whenever a map carries a
dependence on yaw.

The shipped map is even in yaw, because a symmetric car has no bias between left and right. That
keeps the g-g diagram symmetric between left and right, but it *shrinks the diagram away from the
center*, where combined slip drives $|\beta|$ up. A map with a genuine asymmetry between left and
right would skew the diagram itself.

DRS is the drag-reduction system, in which the rear wing opens on the straights. It is always
closed in the trim. Its activation is a concern for a controller, not for physics. The `drs_flag`
axis therefore exists, and the tests exercise it, but no shipped solve opens it yet.

### 7.13 How the `f1_2026` map was authored

The shipped map is **synthetic**. It is neither measured nor simulated, and it says so everywhere.

`python/tools/gen_f1_aero.py` generates it, and documents every assumption in its header. Here is
the construction.

- The grid is 5 × 5 × 5 × 2. Front ride height takes {10, 20, 30, 40, 60} mm, and rear takes {30,
  50, 70, 100, 140} mm. Yaw takes {−8, −4, 0, 4, 8}°. DRS takes {0, 1}. That gives 250 rows of
  parquet.
- **Anchoring.** At the reference node — 30 mm front, 70 mm rear, yaw 0, DRS closed — the
  coefficients reproduce the constant-aero fallback exactly: $C_{z,f}A = 1.9$, $C_{z,r}A = 2.6$,
  and $C_xA = 1.25$ m².

  The total downforce area is therefore 4.5 m², which gives about 1.7 times the car's weight in
  downforce at 250 km/h, at a lift-to-drag ratio of about 3.6.

  Those constants are a physically plausible stand-in for the aero of the Perantoni & Limebeer 2014
  reference car. The validation laps reconcile against the published figures of that paper; see
  Chapter 13.
- **Estimated sensitivities.** The script labels all of these as estimates.

  Front and rear downforce rise linearly as the respective ride height drops, which is ground
  effect, with a mild coupling in rake across the axles. Drag rises slightly as the platform
  lowers, which is induced drag. Downforce falls, and drag rises, evenly and quadratically with
  |yaw|; that is −8 % of downforce at 10° of yaw. And DRS open multiplies the rear downforce by
  0.70 and the drag by 0.82, leaving the front unchanged.
- The functional form is affine in each ride-height axis, and even and quadratic in yaw. Every
  fibre aligned with the grid is therefore monotone, or has a single peak. That is deliberately
  safe for the shape-preserving monotone-cubic interpolant.

The clean-room citations for the aero modeling are recorded in `docs/theory/t1-trim.md`. They are
Perantoni & Limebeer 2014, for the speed-dependent aero of the reference car, generalized here to
explicit ride heights; and Katz, *Race Car Aerodynamics*, 1995, for sensitivity to ride height in
ground effect, and for rake. The platform fixed point is a standard quasi-static heave balance.

### 7.14 What to trust, and what is not there yet

Here is how these models are validated. Chapter 13 gives the details.

- **The golden cross-check.** All five MF6.1 channels of the Pacejka book tire match an independent
  Magic Formula implementation to ≤ 0.5 %. That implementation is the GPL `teasit` library, run
  under GNU Octave; outlap uses its numeric *outputs* as data, and never its source. The sweeps are
  pure longitudinal, pure lateral including ±4° of camber, and combined.
- **Property tests.** They pin the signs; odd symmetry on subsets that have no shifts; containment
  in the friction circle for the brush model; continuity across $\kappa = 0$, $\alpha = 0$, and
  $V_{cx} = 0^+$; and finiteness over hostile inputs. A CI gate on the heap proves zero allocations
  for each evaluation.
- **A gate on the reference data.** Every dataset under `data/tires/` must load with no warnings,
  sit in a grip band that is plausible for its class, and round-trip through the `.tir` codec
  numerically exactly.
- **Tests on the aero map.** The committed F1 map reproduces the reference coefficients at the
  reference ride heights. A constant map degenerates to the constant-aero trim. The platform sinks
  monotonically with speed. And opening DRS cuts the rear downforce and the drag.

And here are the honest limits of what you have just read.

- **Turn-slip is omitted**, with all $\zeta \equiv 1$. So is the velocity-digressive branch of
  friction, `LMUV`. Both are v1 scope.
- **Tire thermal state and wear are stubs.** The `thermal` and `wear` blocks of a `.tyr` file are
  required by the schema, but nothing consumes them until the thermal model of the tire lands,
  except `p_cold`. Every shipped value there is labelled a synthetic placeholder.

  Until then, every solve is a solve on a *cold tire*, at fixed pressure.
- **Relaxation is live at T2 only.** The exact-exponential stepper for slip lag, in
  `crates/outlap-tire/src/relax.rs`, drives the lagged slip at each wheel in the T2 tier (§8.7).
  The QSS tiers use steady-state forces, by definition.
- **Camber is zero in T1.** The assembly note says "camber maps land later". And the trim always
  runs with DRS closed.
- The **`f1_2026` aero map is synthetic**, and its sensitivities are estimates. Only its anchor
  point is reconciled against the literature.
- The Python `Tyre` class always builds an MF6.1 model. A brush-only file works in the solvers,
  through the gate in `TireModel::from_tyr`. But it loads through `Tyre` as a degenerate model in
  which everything is zero, rather than raising an error.

With forces at the contact patch, and downforce on the axles, in hand, Chapter 8, Physics II,
assembles them into the T1 trim solve, and into the g-g-g-v envelope that actually produces a lap
time.


---

## 8. Physics II: solving a lap — T0, T1, T2, and the g-g-g-v envelope

*What you will learn: how outlap actually computes a lap time. You will follow the classic
forward-and-backward velocity-profile method, which is T0, step by step. You will meet the
nine-unknown equilibrium solve, called the T1 "trim", which turns a full double-track car into a
grip boundary. And you will see how that boundary is precomputed into the g-g-g-v envelope that
connects the two. Along the way you will learn exactly what `tier="t0"` and `tier="t1"` return, and
how flat-track mode collapses the whole machinery into the 2-D form used for validation. The
chapter closes with the tier that stops assuming equilibrium: the transient T2, in which the car is
integrated through time, with a driver in the loop (§8.7).*

### 8.1 The big picture: two QSS tiers, one envelope, and then time

Chapter 2 introduced the idea of a *quasi-steady-state*, or QSS, lap solver. Instead of integrating
the equations of motion of the car through time, which would be an ODE solve, a QSS solver assumes
that the car is at its limit everywhere. It then asks, at each point along the track: how fast can
the car possibly be going here?

The QSS machinery of outlap lives in the `outlap-qss` crate. It is built from three cooperating
pieces.

1. **The T1 trim**, in `crates/outlap-qss/src/t1/trim.rs`. Give it a speed and a commanded
   acceleration, and it finds the steady-state balance of a full *double-track* car: four wheels,
   a tire model for each axle, and load transfer.

   It answers one question for each call. Is this operating point physically achievable, and, if it
   is, what is every wheel doing?
2. **The g-g-g-v envelope**, in `crates/outlap-qss/src/t1/envelope.rs`. It is a precomputed table of
   the trim's answers: the maximum lateral acceleration, as a function of speed, of longitudinal
   acceleration, and of how hard the road presses the car down.

   It is generated once for each car, which is a cold step at the scale of seconds in a release
   build. It is then queried millions of times, for free.
3. **The T0 velocity-profile solver**, in `crates/outlap-qss/src/solver.rs`. It is a point-mass
   sweep along the track. It consumes the envelope, and produces the speed trace and the lap time.

   It is not an ODE integration. It is three passes over an array.

The names are solver *tiers*, from Chapter 4. `t0` is the point-mass profile. `t1` is the same
profile, plus a re-solve of the trim at each station, to report per-wheel channels.

Both evaluate the **same** vehicle description. A tier selects fidelity, and never a different car.
That is hard rule #4; see `crates/outlap-schema/src/sim.rs:64-65`.

The transient T2 tier is the story of §8.7. `t3` raises a typed error today (§8.5).

Here is a fact worth stating up front, because it may surprise you. **`t0` and `t1` produce an
identical lap time, and an identical speed profile.**

Both run the same velocity profile, based on the envelope, through `solve_profile` in
`crates/outlap-qss/src/qss.rs`. `t1` then *re-trims* each station of the already-solved profile, to
log the wheel loads, slips, and forces.

On the shipped `f1_2026`, around the `catalunya_osm` centerline, at 2 m stations, of which there
are 2339, both tiers return `lap_time_s = 112.520`. The `t1` dataset simply carries seven more
channels.

### 8.2 T0: the forward and backward velocity profile

#### 8.2.1 The idea, from scratch

Imagine driving a lap perfectly. Three things limit you.

- **In a corner**, you cannot exceed the speed at which the sideways, or lateral, grip of the tires
  is exactly consumed by turning. A tighter corner gives a lower ceiling.
- **Accelerating out of a corner**, you are limited by engine power, and by whatever longitudinal,
  or fore-and-aft, grip is left over after cornering.
- **Braking into a corner**, you are limited by grip alone. The brakes are almost never the weak
  link.

The classic point-mass QSS method turns this into three passes over an array, along the track. The
track is sampled at uniform stations in arc length, one every `ds` meters of distance along the
lap. The default step is `DEFAULT_DS_M = 2.0` m.

1. **The cornering ceiling.** For every station $i$, compute the *curvature-limited speed*
   $v_{\lim,i}$: the fastest speed at which lateral grip still meets the cornering demand.

   This ignores how you got there. It is a pure local ceiling.
2. **The forward pass, for traction.** Sweep forward from the slowest point. From the speed at
   station $i$, compute the fastest speed reachable at station $i+1$, using all the available drive
   force and the remaining grip: $v_{i+1}^2 = v_i^2 + 2\,\Delta s\,a_{\text{accel}}$. Take the
   minimum with the ceiling.

   This kills every violation of the form "you would need infinite acceleration".
3. **The backward pass, for braking.** Sweep backward. From the speed at station $i{+}1$, compute
   the fastest speed at station $i$ from which the car could still slow down in time:
   $v_i^2 = v_{i+1}^2 + 2\,\Delta s\,a_{\text{brake}}$. Again take the pointwise minimum.

The result is the fastest profile that respects all three limits at once.

This is the `calc_vel_profile` formulation of Heilmeier et al., *Vehicle System Dynamics* 58(10),
2020. It is re-implemented clean-room, in `crates/outlap-qss/src/solver.rs`, on the 3-D track ribbon
of Perantoni & Limebeer (2015) — their *three-dimensional-track* paper, which is a different work
from the 2014 variable-parameters paper that Chapter 13 uses as the validation oracle — and of
Lovato & Massaro (2022). The module doc cites all three.

Lap time is the trapezoidal sum over segments, taken in a fixed order:

$$t_{\text{lap}} = \sum_i \frac{2\,\Delta s}{v_i + v_{i+1}},$$

with the denominator floored at $10^{-6}$ m/s, so that a stationary station cannot divide by zero.

A **closed lap** has a subtlety. The passes need a starting speed, but the lap wraps around.

The solver therefore seeds at the station with the globally minimum $v_{\lim}$. The slowest corner
is limited by lateral grip, so its speed is a fixed point of the sweep.

It then iterates a full forward wrap and a full backward wrap, until the seed speed stops changing.
The relative tolerance is `SEED_TOL = 1e-6`, and there are at most `MAX_PASS_ITERS = 8` iterations.
That cap is a backstop against divergence, and it has never triggered on a physical track.
Exceeding it raises a typed `T0Error::PassesDiverged`. It never hangs.

An **open path** instead starts standing, with `v[0] = 0`. It needs a single forward sweep and a
single backward sweep; see `solve_generic`, at `solver.rs:338-405`.

#### 8.2.2 The 3-D ribbon: how a hill and a bank enter

Chapter 4 described the track as a 3-D *ribbon*. It has a centerline with plan-view curvature
$\kappa_h$, which is how sharply it turns as seen from above; vertical curvature $\kappa_v$, which
is crests and dips; grade $\theta_g$, which is the uphill or downhill slope; and banking
$\theta_b$, which is the tilt from side to side.

Before solving, the track is sampled once into a `T0Path`, at `crates/outlap-qss/src/path.rs`. That
is a structure of arrays of plain `f64` slices, with one entry for each station. The hot passes
therefore touch nothing but flat memory:

```rust
pub struct T0Path {
    pub s: Vec<f64>,            // arc-length station, m
    pub kappa_l: Vec<f64>,      // road-plane lateral curvature, 1/m
    pub kappa_n: Vec<f64>,      // road-normal curvature, 1/m
    pub sin_b_cos_g: Vec<f64>,  // sinθ_b·cosθ_g (lateral gravity projection)
    pub cos_b_cos_g: Vec<f64>,  // cosθ_b·cosθ_g (normal gravity projection)
    pub sin_g: Vec<f64>,        // sinθ_g (+ uphill)
    pub grip: Vec<f64>,         // track grip scale γ(s)
    pub ds: f64,                // uniform step (divides the length exactly)
    pub closed: bool,
}
```

The two curvatures are the raw track curvatures, projected into the tilted road plane. That follows
Perantoni & Limebeer 2015; see `path.rs:8-9`:

$$\kappa_l = \kappa_h \cos\theta_g \cos\theta_b + \kappa_v \sin\theta_b, \qquad \kappa_n = \kappa_v \cos\theta_b - \kappa_h \cos\theta_g \sin\theta_b .$$

$\kappa_l$ is the curvature that the tires must fight, which is cornering. $\kappa_n$ is curvature
*out of* the road plane. A crest, where $\kappa_n < 0$, unloads the car. A dip, or a compression,
loads it.

Two derived quantities then carry the whole 3-D story into the grip model; see `demand_and_gn`, at
`solver.rs:317-322`:

$$a_{y,\text{dem}} = \kappa_l v^2 + g \sin\theta_b \cos\theta_g \qquad \text{(signed lateral demand — banking of the right sign assists)},$$

$$g_{\text{normal}} = g \cos\theta_b \cos\theta_g + \kappa_n v^2 \qquad \text{(road-normal specific gravity)}.$$

$g_{\text{normal}}$ deserves a definition in plain language, because it is the third axis of the
envelope. It is *the effective gravity pressing the car onto the road*, in m/s².

On flat ground it equals $g = 9.80665$ m/s², which is the crate constant `G`. Over a fast crest it
drops, and the car goes light. Through a banked, compressive corner such as Eau Rouge it can
approach $2g$: the car is squeezed into the road, and the tires gain grip.

Aerodynamic downforce is deliberately *not* part of $g_{\text{normal}}$. The speed axis of the
envelope already carries it (§8.4).

Two practical details of the sampling matter.

First, the requested step is rounded, so that `ds` divides the lap length *exactly*. The wrap
segment of a closed lap is then also `ds`.

Second, the curvatures get a light centered moving average, of half-width
`CURV_SMOOTH_RADIUS = 6` stations.

A centerline imported from the real world — OpenStreetMap geometry, plus heights from an elevation
model — carries noise in position at a scale below the length of a car. An interpolating spline
amplifies that into fake spikes of curvature. The average removes them, while leaving a genuine
corner, which spans many stations, intact.

The doc comment is honest that this is a pragmatic mitigation: "the principled fix for a fair lap is
the min-curvature line"; see `path.rs:15-20`.

Signs follow ISO 8855 throughout: $x$ forward, $y$ left, $z$ up. A positive $a_y$ is therefore a
left turn, and a positive `sin_g` means uphill.

#### 8.2.3 The degenerate path: a friction ellipse at constant μ

T0 supports two grip models, behind one private trait, `GripModel`, at `solver.rs:51-58`. Each is
monomorphized into the shared sweep, so there is no dynamic dispatch at any station.

The simpler one is `EllipseGrip`. It treats the car as a point mass, with constant friction
coefficients and a *friction ellipse*. That is the rule by which longitudinal and lateral tire
force trade off:

$$\left(\frac{F_t}{\mu_x \gamma N}\right)^2 + \left(\frac{F_y}{\mu_y \gamma N}\right)^2 \le 1,$$

where $N$ is the normal load, $\mu_x$ and $\mu_y$ are the longitudinal and lateral friction
coefficients, and $\gamma$ is the local grip scale of the track.

With lumped aerodynamic terms $q_x = \tfrac12 \rho C_x A$ for drag and $q_z = \tfrac12 \rho C_z A$
for downforce, the point-mass equations are, from `solver.rs:10-13` and
`docs/theory/t0-point-mass.md`:

$$N(s,v) = m\,(g\cos\theta_b\cos\theta_g + \kappa_n v^2) + q_z v^2, \qquad F_y(s,v) = m\,(\kappa_l v^2 + g\sin\theta_b\cos\theta_g),$$

$$m\dot v = F_t - q_x v^2 - m g \sin\theta_g .$$

Both sides of $|F_y| \le \mu_y \gamma N$ are affine in $u = v^2$. The cornering ceiling therefore
has a **closed form**, with no iteration; see `ellipse_v_limit`, at `solver.rs:62-88`.

A *flight guard* enforces $N \ge 0$. Over a crest severe enough that even downforce cannot keep the
tires loaded, the ceiling is the take-off speed.

On a flat circle of radius $R$, the formula reduces to the one that every textbook derives:
$v = \sqrt{\mu_y g R}$. On a turn banked at angle $\phi$, it becomes
$v^2 = gR\,(\mu_y\cos\phi + \sin\phi)/(\cos\phi - \mu_y\sin\phi)$. Both are verified against the
solver, in `crates/outlap-qss/tests/analytic.rs`.

Where do $\mu_x$ and $\mu_y$ come from? Not from a number typed by hand.

`T0Vehicle::assemble`, in `crates/outlap-qss/src/vehicle.rs`, evaluates the real MF6.1 tire model of
Chapter 7. It takes the **peaks of pure-slip force**, at the nominal load `FNOMIN` and the cold
inflation pressure, and averages the front and rear axles.

The shape factors of the tire, for load and for pressure, are therefore folded in. The raw `PDX1`
and `PDY1` coefficients are not trusted blindly.

A note recording exactly this lands in every result. That is the discipline of `notes`: nothing is
silent.

The same assembly reduces the powertrain to a peak-torque envelope for each unit, over folded gear
ratios, plus a power cap for the ERS. That is the *powertrain ceiling*, `tractive_force(v)`,
summarized in §8.2.5. The `.ptm` format, and the energy accounting behind it, belong to Chapter 9,
Physics III.

Be aware of the fine print. This path with a constant-μ ellipse, through `solve_into` and
`solve_lap`, is **not reachable from Python**. It is the path for analytic reference and for the
performance gate, at the Rust level. The theory page `docs/theory/t0-point-mass.md` documents it
from that foundational perspective.

What `tier="t0"` actually runs today is the next section.

#### 8.2.4 The production path: T0 on the g-g-g-v envelope

The production grip model is `GgvGrip`, at `solver.rs:153-310`. It replaces the constant-μ ellipse
with lookups into the T1-derived envelope of §8.4.

The envelope answers one question, and it allocates nothing while doing so. At speed $v$,
longitudinal acceleration $a_x$, and normal gravity $g_{\text{normal}}$, what is the maximum
sustainable lateral acceleration? That is `ay_boundary(v, ax, g_normal)`.

The solver multiplies that boundary by the local grip scale of the track, `p.grip[i]`, exactly as
the ellipse path scales its μ.

The three `GripModel` queries become these.

- **The cornering ceiling, by bisection**; see `v_limit`, at `solver.rs:262-280`.

  There is no closed form any more. Both the demand $a_{y,\text{dem}}(v)$ and the boundary
  $a_y(v, 0, g_{\text{normal}}(v))$ move with speed.

  The solver therefore bisects feasibility, between 0 and the speed cap `v_cap`, which defaults to
  150 m/s. It runs `V_LIMIT_ITERS = 24` iterations, which resolves the ceiling to below a
  millimeter per second.
- **The forward step**; see `forward_v2`, at `solver.rs:282-297`.

  The tire budget $a_{x,\text{grip}}$ comes from inverting the envelope at the lateral demand. It
  bisects for the largest $a_x \ge 0$ whose boundary still meets $|a_{y,\text{dem}}|$; see
  `ax_forward`, with `AX_INV_ITERS = 16`.

  The powertrain branch is $F_t(v)\cdot\text{scale}/m - a_{\text{drag}}(v)$. The applied
  acceleration is

  $$a = \min\!\big(a_{x,\text{grip}},\; F_t(v)\,\text{scale}/m - a_{\text{drag}}(v)\big) - g\sin\theta_g .$$

  Note the subtraction of drag. The $a_x$ axis of the envelope already *embeds* the reference
  aerodynamic drag, because a trim at $a_x$ includes drag in its force balance.

  The solver therefore subtracts the same `drag_accel(v)` curve from the powertrain ceiling, before
  taking the min. Otherwise drag would be counted twice on the grip branch, or missed on the power
  branch.

  `scale` is the traction scale at each station, in $[0,1]$. The coupling of slow states fills it,
  from the thermal derate and the battery power cap; see Chapter 9. It is `1.0` when uncoupled. And
  it never touches braking, which draws no drive power.
- **The backward step**; see `backward_v2`, at `solver.rs:299-309`.

  It bisects for the most negative feasible $a_x$, at the lateral demand, through `ax_backward`.
  Drag and uphill gravity *add* to the braking budget.

  Braking remains limited by friction only. There is no brake-thermal model, and no blending of
  regeneration yet. The notes say so.

The flight guard survives, in a generalized form. A station is "planted" when the *total* normal
specific force, $g_{\text{normal}} + q_z v^2/m$, is positive. Aero downforce therefore still plants
a downforce car over a crest, even when $g_{\text{normal}} \le 0$. An airborne station coasts on
drag and grade alone.

One approximation here is documented and bounded. The lowest $g_{\text{normal}}$ that the envelope
samples is $0.5g$. Between $0.5g$ and 0, the boundary query therefore clamps, and slightly
over-predicts the contribution of gravity to grip. That contribution is about $\mu \cdot 0.5g$, and
aero dominates at the speeds where a crest matters; see `solver.rs:241-252`.

#### 8.2.5 The powertrain ceiling

`tractive_force(v)`, at `crates/outlap-qss/src/vehicle.rs:226-249`, is the one place where the
capability of an engine or a motor enters T0. It is deliberately simple.

- Each drive unit's `.ptm` map, from Chapter 5, contributes its `limits.max_torque_nm_vs_speed`
  curve, as a peak-torque envelope $\tau(\omega)$. The project's one shared `MonotoneCubic`
  interpolates it. RPM converts to rad/s at the file boundary, under the rule that everything
  internal is SI.
- The ratios of the gearbox and the differential, along each unit's coupler path, are folded at
  assembly into constants for each gear: `omega_per_v = ratio / r_wheel`, and
  `force_per_torque = ratio · efficiency / r_wheel`.

  At speed $v$, each unit contributes its **best gear**. That is the highest wheel force among the
  gears whose shaft speed stays under the top speed of the envelope. A gear past that point is
  rev-limited out.
- An ERS, which is an energy-recovery system such as an F1 MGU-K, is reduced to a **power cap**.
  That is the peak deployment power, times a taper curve that depends on speed.

  It is then capped by the machine's own torque envelope, *at the shaft speed of the engaged gear*,
  so $P \le \tau(\omega_\text{crank})\,\omega_\text{crank}$. It converts to force as
  $\eta P / \max(v, 1.0)$.

  A machine that shares a drivetrain node with the engine, such as an F1 MGU-K on the crank, turns
  at whatever speed the engine's engaged gear dictates. Below its base speed it is therefore
  **torque**-limited: its wheel force is the flat $\tau\cdot\text{ratio}\cdot\eta / r$, and not a
  $1/v$ curve.

  The floor of 1 m/s in the denominator is therefore vestigial for a machine mounted on the crank,
  because the two speed factors cancel. It still guards a machine with no shared node.

  Budgets on deployment for each lap, and override modes, are *not* enforced at T0. A permanent note
  in every result says so.

Two typed errors guard the assembly. A vehicle with neither drive units nor an ERS gives
`T0Error::NoDrive`. And a drive unit whose `.ptm` file carries a gridded efficiency *map*, rather
than a constant, gives `T0Error::UnsupportedEfficiencyMap`, because T0 has no map reader yet.

Similarly, a vehicle with no `aero.constant` block gives a hard `T0Error::NoConstantAero`, unless
`sim.allow_degraded: true`. In that case T0 runs with zero aero, and a recorded note. That is the
single documented fallback path; see Chapter 4.

#### 8.2.6 Entry points, workspaces, and the 50 ms budget

The zero-allocation kernels write into a `T0Workspace` that the caller owns. It holds two
pre-sized `Vec<f64>` buffers: the ceiling `v_lim`, and the solved `v`; see
`crates/outlap-qss/src/result.rs:25-52`.

| Function | Grip model | Allocates? |
|---|---|---|
| `solve_into(veh, path, ws)` | constant-μ ellipse | no |
| `solve_into_ggv(veh, env, path, ws)` | g-g-g-v envelope | no |
| `solve_into_ggv_scaled(veh, env, scale, path, ws)` | envelope + per-station traction scale | no |
| `solve_lap(...)` / `solve_lap_ggv(...)` | owning wrappers returning a `LapResult` | yes (cold) |

CI enforces both halves of the performance contract, in release builds; see
`crates/outlap-qss/tests/catalunya.rs` and `tests/alloc.rs`.

The median of 11 solves of a real Catalunya lap must complete in under 50 ms, for both `solve_into`
and `solve_into_ggv`. *Generating* the envelope is excluded, because it is the documented cold
assembly step.

And a test instrumented with dhat asserts that the hot kernels perform **zero heap allocations**.

Chapter 13 covers the full list of gates.

#### 8.2.7 What T0 returns: `LapResult`

The owning wrappers derive the point-mass channels, and pack them into `LapResult`; see
`crates/outlap-qss/src/result.rs:54-75`:

```rust
pub struct LapResult {
    pub s: Vec<f64>,           // arc-length stations, m
    pub v: Vec<f64>,           // speed, m/s
    pub ax: Vec<f64>,          // longitudinal acceleration, m/s²
    pub ay: Vec<f64>,          // lateral acceleration (ISO 8855, + left), m/s²
    pub t: Vec<f64>,           // cumulative time, s
    pub lap_time_s: f64,       // total lap time, s
    pub line: LineDescriptor,  // Centerline | MinCurvature{..} | File{..}
    pub resolved_hash: String, // blake3 of the resolved car spec
    pub notes: Vec<String>,    // simplifications/degradations — nothing silent
}
```

`ay` is the lateral demand in the velocity frame, $\kappa_l v^2 + g\sin\theta_b\cos\theta_g$,
evaluated on the solved profile. `ax` is the central segment acceleration,
$(v_{i+1}^2 - v_i^2)/2\Delta s$.

`resolved_hash` ties every result to the exact vehicle spec that produced it; see Chapter 4. And
`notes` accumulates every recorded simplification, from vehicle assembly, envelope generation, and
dispatch.

### 8.3 T1: the quasi-static double-track trim

#### 8.3.1 What a "trim" is

The point-mass solver knows one number for each station.

A real car has four tires, each with its own vertical load, slip, and force. Those loads shift as
the car accelerates; that is the load transfer of Chapter 2.

The T1 **trim** answers the detailed question. Given a commanded operating point — speed $v$,
lateral acceleration $a_y$, longitudinal acceleration $a_x$, and the local $g_{\text{normal}}$ —
what steady chassis state produces exactly those accelerations?

"Steady", which is the same as quasi-static, means that nothing is changing: the yaw rate is
constant, and there is no transient in the suspension.

The vocabulary comes from flight mechanics. Trimming an aircraft means finding the control settings
that hold a steady condition.

The trim is a nonlinear algebraic system with **9 unknowns and 9 residuals**. It is solved with
zero allocation, and it is panic-free; see the module header of
`crates/outlap-qss/src/t1/trim.rs`.

Its basis in the literature is: Perantoni & Limebeer (2014) for the reference car and the QSS
framing; Lovato & Massaro (2022) for the g-g framing; Pacejka (2012) and Guiggiani (2018) for the
load transfer; and Milliken & Milliken (1995) for the decomposition of lateral transfer.

#### 8.3.2 The actual unknowns and residuals

Here is the unknown vector $z$, in ISO 8855 and SI; see `trim.rs:12-21`:

| index | symbol | meaning |
|---|---|---|
| 0 | $\delta$ | front road-wheel steer angle, rad |
| 1 | $\beta$ | body-slip angle (velocity vector vs body $x$-axis), rad |
| 2 | $r$ | yaw rate, rad/s |
| 3 | $s$ | longitudinal-slip control (drive if $> 0$, brake if $< 0$) |
| 4 | $w$ | driven-axle slip split: $\kappa_{\text{left}} = s + w$, $\kappa_{\text{right}} = s - w$ |
| 5–8 | $F_{z,i}$ | per-wheel normal loads $[\mathrm{FL}, \mathrm{FR}, \mathrm{RL}, \mathrm{RR}]$, N |

Note what is *not* here. There is no pair of throttle and brake. One signed slip control, $s$,
handles both, because its sign selects between driving and braking. And $w$ is the split of the
differential between left and right.

Inside the solver, the four $F_z$ unknowns are non-dimensionalized by $m \cdot g_{\text{normal}}$.
All nine unknowns are then of order 1. Mixing radians and newtons in one Jacobian would otherwise
be numerically miserable.

Here are the nine residuals; see `trim.rs:23-35`, evaluated in `residual()` at `trim.rs:487-570`:

$$R_0:\ \Sigma F_x = m a_x \qquad R_1:\ \Sigma F_y = m a_y \qquad R_2:\ \Sigma M_z = 0$$

$$R_3:\ r\,v = a_y \cos\beta - a_x \sin\beta \qquad R_4:\ \text{differential law} \qquad R_{5\ldots8}:\ F_{z,i} = F_{z,i}^{\text{pred}}\ (\times 4)$$

In words:

$R_0$ and $R_1$ say that the tire and aero forces must sum to exactly the commanded accelerations,
with the aero drag $q_x v^2$ subtracted from $\Sigma F_x$.

$R_2$ says that the yaw moments must cancel. Steady state means zero yaw acceleration.

$R_3$ says that the yaw rate must be kinematically consistent with turning at $(a_x, a_y)$ while
slipping at $\beta$. For a constant body-frame velocity, the acceleration of the CG is
$\omega \times V$.

$R_4$ says that the driven wheels must obey the differential. An **open** differential under drive
enforces equal longitudinal force in the wheel frame, which means equal torque. A **locked**,
**solid**, or **LSD** differential enforces equal speed, so $w = 0$. Under braking the differential
is inactive, and the balance bar splits the brake torque.

$R_5$ through $R_8$ say that the four normal loads must match the quasi-static prediction of load
transfer.

The slip angle at each wheel comes from the contact-point velocities,
$V_{x,i} = v\cos\beta - r y_i$ and $V_{y,i} = v\sin\beta + r x_i$, rotated into the wheel frame.
The tire forces come from the shared `TireModel`, at camber 0. Camber maps land later, and a note
records that.

The residuals are scaled to be dimensionless: forces by $1/mg$, and the moment by $1/mgL$.
Convergence is declared at a scaled norm of at most `TOL = 1e-10`.

#### 8.3.3 Load transfer: geometric plus elastic

The prediction of load transfer is the standard steady-state decomposition; see `load_transfer`, at
`trim.rs:587-614`, with the theory in `docs/theory/t1-trim.md`.

For each axle, take the static weight plus the downforce:

$$F_{z,\text{front}}^{\text{total}} = m g \frac{b_r}{L} + q_{z,f} v^2, \qquad F_{z,\text{rear}}^{\text{total}} = m g \frac{a_f}{L} + q_{z,r} v^2,$$

where $a_f$ and $b_r$ are the distances from the CG to the axles, and $L$ is the wheelbase.

Longitudinal, or pitch, transfer moves load rearward under acceleration:

$$\Delta F_z^x = \frac{m\,a_x\,h_{cg}}{L}.$$

Lateral transfer at each axle splits into two parts.

The **geometric** part acts through the roll center of the axle. That is a construct of suspension
geometry: the point about which the body effectively rolls at that axle.

The **elastic** part is carried by roll stiffness. Together they are the Milliken decomposition.

With $H = h_{cg} - h_{ra}$, the CG height above the roll axis, and $M_\phi = m a_y H$, the roll
moment:

$$\Delta F_{z,f}^{y} = \frac{m a_y\,(b_r/L)\,h_{rc,f}}{t_f} + \frac{\xi_f\,M_\phi}{t_f}, \qquad \Delta F_{z,r}^{y} = \frac{m a_y\,(a_f/L)\,h_{rc,r}}{t_r} + \frac{\xi_r\,M_\phi}{t_r},$$

where $t$ is the track width, $h_{rc}$ the height of the roll center, and $\xi_f + \xi_r = 1$ the
shares of roll stiffness.

This is why an engineer stiffens the front anti-roll bar to add understeer. A larger $\xi_f$ moves
lateral transfer to the front axle. And because tire grip grows sub-linearly with load (Chapter 7),
an axle that is loaded more unevenly grips less.

Two details of the implementation matter.

At **wheel lift**, the unloaded wheel floors at 0 N, and the grounded wheel carries the whole axle,
and never more. The boundary therefore cannot become optimistic, and $\Sigma F_z$ stays equal to
weight plus downforce; see `split_axle`, at `trim.rs:761-775`.

And **anti-dive and anti-squat do not enter the steady-state $F_z$**. They only modulate the heave
of the aero platform, which is the ride height, when a ride-height aero map is installed. That
changes the downforce, which changes the loads. The totals of load transfer are themselves
independent of geometry, in steady state.

The aero map, and its fixed point at the platform equilibrium, are the subject of Chapter 7. From
the point of view of the trim, the map is evaluated *inside* the residual, at yaw = $\beta$, in
degrees at the map boundary, which is one of the deliberate seams for display units. The Jacobian
therefore feels $\partial(\text{downforce})/\partial\beta$.

#### 8.3.4 `fz_coupling`: the algebraic loop on vertical load

The loads depend on the accelerations. The accelerations depend on the tire forces. And the tire
forces depend on the loads. That is an algebraic loop.

`sim.fz_coupling` selects how the loop closes; see `crates/outlap-schema/src/sim.rs:80-89`. The
trim implements it as a substitution on one line; see `trim.rs:546-549`:

```rust
let (ax_lt, ay_lt) = match inp.coupling {
    FzCoupling::OneStepLag => (inp.ax, inp.ay),                 // commanded accelerations
    FzCoupling::FixedPoint => (sum_fx / self.mass_kg, sum_fy / self.mass_kg), // summed tyre forces
};
```

- **`one_step_lag`**, which is the default, uses the *commanded* $(a_x, a_y)$ in the prediction of
  load transfer. The loads are therefore decoupled from the instantaneous sums of tire force,
  during the iteration.
- **`fixed_point`** uses the *achieved* accelerations, $\Sigma F_x/m$ and $\Sigma F_y/m$. The loop
  is therefore fully coupled through the Jacobian.

Here is the key fact, stated in `docs/theory/t1-trim.md`. At convergence, the residuals $R_0$ and
$R_1$ force $\Sigma F = m a$. **Both modes therefore reach the same trim.**

The choice changes only the algebraic coupling that the Jacobian sees. It will matter for the
transient tiers, where the "previous step" of `one_step_lag` is a real timestep.

It is a *recorded* simulation setting. It appears in the attributes of every result, and in the
envelope, so no two results with different numerics can be confused.

#### 8.3.5 Numerics: damped least squares, continuation, and honest infeasibility

The docs call the trim a "damped Newton" solve. The algorithm as implemented is
**Levenberg–Marquardt**, or LM: a damped Gauss–Newton method that interpolates toward gradient
descent when it is far from the solution; see `solve_lm`, at `trim.rs:336-442`.

One LM iteration does four things.

- It builds a finite-difference Jacobian $J$, with a relative step of `FD_H = 1e-7`. That costs
  nine extra evaluations of the residual.
- It forms the normal equations $A = J^\top J$ and $g = J^\top R$, with Marquardt diagonal scaling:
  $A_{ii} \mathrel{+}= \mu \max(A_{ii}, 10^{-12})$.
- It solves the dense 9×9 system, by Gaussian elimination with partial pivoting, on fixed stack
  arrays. That means zero heap.
- It accepts the step only if that step reduces $\lVert R \rVert$. A damping loop retries up to
  `MAX_LINE_SEARCH = 40` times, shrinking $\mu$ by ×0.3 on acceptance, and growing it by ×4 on
  rejection.

There are at most `MAX_NEWTON = 80` iterations.

A trial state is clamped to generous physical bounds: $\delta \in \pm0.7$ rad,
$\beta \in \pm0.5$, $r \in \pm8$ rad/s, $s$ and $w \in \pm0.6$, and $F_z/mg \in [0, 6]$. The search
therefore cannot wander into the periodic aliases of $\beta$ that trap a plain Newton method.

When a ride-height aero map is installed, the converged ride heights from each evaluation of the
residual are threaded to the next, as a warm start for the nested fixed point at the aero platform;
see Chapter 7. That cuts it from about 20 cold iterations to 1 or 2 warm ones, with a physically
identical result.

The full `trim()` entry, at `trim.rs:205-227`, has two stages.

The **fast path** is one direct LM solve, from a physics warm start. That start uses an
Ackermann-like steer, $\delta = L a_y/v^2$, clamped to ±0.5; $r = a_y/v$; $\beta = 0$; and loads
from the direct prediction of transfer.

If that fails, a **homotopy continuation** falls back. It solves the trivially easy straight-line
trim, at $a_y = a_x = 0$, which is nearly linear and always converges. It then ramps the targets
$(t\,a_y,\ t\,a_x)$ from $t = 0$ to 1, with an adaptive step, warm-starting each sub-solve from the
last.

If the ramp cannot advance, because the step falls below $10^{-3}$, then the commanded point is
past the friction boundary.

That brings us to the design contract that makes the whole envelope possible. **An unreachable
operating point is not an error.**

The trim returns a typed value:

```rust
pub enum TrimOutcome {
    Converged(TrimState),
    Infeasible { residual_norm: f64, iterations: usize },
}
```

`Infeasible` is *information*. It means "past the grip limit", and the envelope generator uses it as
its oracle for the boundary. It is never a panic, and the solver kernels stay typed with `Result`,
under the error rules of the project.

The bisection for the boundary makes roughly half its probes infeasible *by construction*. An
infeasible probe must therefore also fail *fast*.

An **infeasibility stall test** does that. It watches windows of `STALL_WINDOW = 6` LM iterations,
and cuts the solve if $\lVert R \rVert$ failed to shrink by the factor `STALL_FACTOR = 0.7` while
still far above the tolerance, whose floor is $10^{-7}$.

A converging solve drops orders of magnitude every few iterations. An infeasible one parks at a
nonzero minimum of least squares, shaving microscopic amounts. The test tells them apart.

The code documents the residual risk that it accepts. A feasible point that is pathologically stiff
could be misclassified as infeasible. That would pin an envelope node conservatively low. The
effect is bounded by the retries on probes, and by the accuracy gates. A confirmation backed by
continuation is a candidate refinement.

One guard on speed. The QSS kinematics divide by $v$, in the yaw rate $r = a_y/v$. Any commanded
point at $v \le$ `V_MIN` $= 0.5$ m/s is therefore immediately `Infeasible`. A crawling car has no
well-posed g-g trim.

#### 8.3.6 Setup metrics

The trim can be probed at will. Two classic setup numbers therefore come almost for free; see
`trim.rs:444-479`.

- **The understeer gradient**, $K = \dfrac{d\delta}{d a_y} - \dfrac{L}{v^2}$, in rad per m/s². It
  is how much *extra* steering the car needs, for each unit of lateral acceleration, beyond the
  pure-geometry requirement of Ackermann.

  $K > 0$ is understeer, in which the front washes out first. $K < 0$ is oversteer.

  It is computed by a central difference of two trims, at $a_y = \pm 1$ m/s², which is a small probe
  in the linear regime, at $a_x = 0$. It returns `None` if either probe is infeasible.
- **The aero balance**: the front axle's share of total downforce, from 0 to 1.

  `aero_front_downforce_share_at(v)` evaluates it at the equilibrium of the aero platform, for
  straight running. With a ride-height map installed it is therefore genuinely dependent on speed,
  because the platform rakes as downforce compresses the suspension. With constant aero it equals
  the reference share.

Both are logged at each station, on every `t1` lap (§8.5). In the xarray output they are
`understeer_gradient`, whose units are recorded as `rad·s²/m`, and `aero_front_share`.

#### 8.3.7 What pins the trim down

Most of the correctness of outlap in vehicle dynamics lives in the trim. It therefore carries a
correspondingly heavy suite of property tests. `docs/theory/t1-trim.md` lists them, and Chapter 13
explains the philosophy of the testing.

They cover: containment in the friction circle at each wheel; that $\Sigma F_z$ equals weight plus
downforce exactly; mirror symmetry between left and right at $\pm a_y$; the ISO 8855 sign
conventions, where a left corner produces positive $a_y$, $\delta$, and $r$, and loads the
right-hand wheels; the direction of pitch transfer; convergence over a dense feasible grid, for
both reference cars, down to corners at hairpin scale of about 6 m radius at 8 m/s; graceful
infeasibility past the boundary; agreement of the two `fz_coupling` modes at convergence; and the
guarantee of zero allocation, enforced by dhat.

### 8.4 The g-g-g-v envelope

#### 8.4.1 Why precompute

One trim costs tens of LM iterations. Each iteration builds a finite-difference Jacobian of nine
columns. Each column is a full evaluation of four tires, plus a fixed point on the aero map if one
is installed.

The T0 sweep needs the grip boundary at every one of about 2,300 stations, inside bisections of 16
to 24 probes, iterated over up to 8 passes of a closed lap.

Solving trims inline would therefore put millions of tire evaluations in the hot loop.

outlap does what the reference literature does — Tremlett et al. 2014; Lovato & Massaro 2022;
Rowold et al. 2023; Werner et al. 2025. It **precomputes the boundary once, into a gridded table**,
and lets the lap solver interpolate.

Generation is a cold assembly step, at the scale of seconds in a release build. Every query
afterwards is a cubic interpolation with zero allocation.

The result is the **g-g-g-v envelope**. It is the classical g-g diagram — the achievable region in
$(a_x, a_y)$, from Rice 1973 and Milliken & Milliken 1995 — extended by two axes.

The first is speed $v$: downforce grows with $v^2$, and inflates the whole envelope. The second is
$g_{\text{normal}}$ (§8.2.2).

Geometrically it is a funnel that widens with speed, with one nested surface for each level of
normal gravity. See `docs/theory/ggv-envelope.md` for figures generated from the real model.

#### 8.4.2 The grid, and its two design decisions

The base table stores $a_{y,\text{corr}} = gg(v, \hat a_x, g_{\text{normal}})$ in a `GriddedMapN`.
That is the shared N-dimensional monotone cubic Hermite interpolant (Fritsch & Carlson 1980), which
Chapters 5 and 6 cover.

The grid is `sim.envelope`, whose default is **40 × 25 × 7** points; see
`crates/outlap-schema/src/sim.rs:114-122`. Here are the axes.

- $v$ is auto-ranged, from `V_ENV_LO = 5.0` m/s to the car's own estimated top speed, which comes
  from drive force against drag, capped at 120 m/s. For `f1_2026` that comes out as
  $v \in [5.0, 91.0]$ m/s.
- $\hat a_x \in [-1, 1]$ is a **normalized** longitudinal axis. A grid node maps to the actual
  acceleration $a_x = \hat a_x \cdot a_{x,\text{cap}}(v, g_{\text{normal}})$, where the cap is
  *that operating point's own* straight-line limit: braking when $\hat a_x < 0$, and acceleration
  when $\hat a_x > 0$.

  Longitudinal capability spans a huge range across the axes of speed and load. A point at light
  load and low speed brakes at a fraction of what a point at high downforce can do. A grid in
  actual $a_x$, held fixed, would therefore leave the feasible window falling between nodes at low
  load.

  Normalizing gives every slice its full resolution, with a node exactly at $\hat a_x = 0$, which
  is pure cornering. A query takes the *actual* $a_x$, and normalizes internally.

  Some of the reference literature parameterizes the g-g slice in polar form instead. What outlap
  ships is this normalized-axis form, following the per-speed g-g construction of the reference
  works. The code comment at `envelope.rs:16-24` is explicit about that.
- $g_{\text{normal}} \in [0.5g,\ 2.0g]$ runs from strong unloading over a crest, to a compression of
  the Eau Rouge class. For reference, that is $[4.90, 19.61]$ m/s².

Two decisions follow Werner et al. (2025, arXiv:2504.10225).

1. **Projection into the velocity frame**, their eq. 5. The stored lateral boundary is
   $a_{y,\text{corr}} = a_{y,\text{body}}\cos\beta - a_x\sin\beta$, at the converged body slip. That
   is the component orthogonal to the *velocity vector*.

   A point-mass solver has no slip angle as a state. Projecting at generation time therefore lets it
   compare the boundary directly against its centripetal demand,
   $\kappa_l v^2 + g\sin\theta_b\cos\theta_g$.
2. **Powertrain limits are omitted**, their §II-C. The envelope is a pure limit on *tire force*.
   The lap solver applies the drive ceiling separately, as `min(tractive_force, grip)` (§8.2.4).

   That keeps one envelope valid across what-ifs on the powertrain, and it keeps the coupling of
   the traction scale out of the table.

Alongside the base table, the struct carries six more things: the two shoulder maps `accel_cap` and
`brake_cap`, over $(v, g_{\text{normal}})$; the reference drag curve `drag_accel(v)`, which is the
"drag currency" that the $a_x$ axis embeds; the reference mass; the recorded `fz_coupling`; six
sensitivity fields, described in the next section; and generation notes that a human can read. See
`envelope.rs:152-183`.

#### 8.4.3 How the boundary is traced

The generator is `GgvEnvelope::generate`, at `envelope.rs:197-420`. It sweeps $n_v \times n_{gn}$
*fibres* of $(v, g_{\text{normal}})$, which are fully independent of one another.

For each fibre, it does three things.

1. **The shoulders first.** It brackets the straight-line limits on acceleration and braking, by
   doubling an initial guess of 5 m/s², up to a cap of 90 m/s², which is about 9 g. It then bisects
   for 16 iterations; see `max_straight_ax`.

   Those values become `accel_cap` and `brake_cap`. They are the denominators that normalize
   $\hat a_x = \pm 1$. Both are floored at 0.5 m/s², so that a degenerate car with no drive cannot
   divide by zero.
2. **March outward from pure cornering.** Starting at the node nearest $\hat a_x = 0$, it finds the
   maximum feasible $a_y$ at each node, with `max_lateral`. It hands the converged trim state to the
   next node outward, as a *hint*.

   A hinted node searches a narrow bracket, of ±40 % of the neighbour's boundary, in 12 iterations
   of bisection. An unhinted node runs the full expand-and-bisect instead: a seed of 20 m/s², up to
   8 doublings, and 16 iterations. The worst-case resolution is
   $2^{-16} \cdot 90 \approx 1.4\times10^{-3}$ m/s², which is far below the accuracy gates.
3. **The economics of a probe.** Every probe is a `trim_warm`. That is the direct-LM primitive of
   §8.3.5, warm-started from the last feasible state. A feasible probe therefore converges in a few
   iterations, and an infeasible one hits the stall test fast.

   A warm probe that fails retries once, from the cold physics guess, before the point is declared
   infeasible. A stale warm start could otherwise pin the boundary low; see `probe`, at
   `envelope.rs:658-672`.

   A node whose *straight-line* seed is already infeasible carries the zero boundary, $a_y = 0$. It
   never panics. That is the contract on an infeasible trim, doing its job.

Fibres solve into owned outputs, merged in a fixed order. The result is therefore **bit-identical**,
whether the sweep runs serially, or on a rayon pool behind the `parallel` feature, which is
native-only. A wasm build stays free of threads; see Chapter 6.

Everything is stored with every axis clamping outside its domain. A query beyond the grid saturates
at the edge value, rather than extrapolating. And `drag_accel(v)` below the lowest sampled speed,
which is 5 m/s, tapers as $v^2$ toward zero, so a standing start feels no spurious drag.

The clean-room statement in the module header is worth knowing. The GPL-3.0
`TUM-AVS/GGGVDiagrams` repository, which is the reference implementation of Werner et al. 2025, was
consulted for *approach only*. The code was re-authored from the papers.

That is a live application of the clean-room rule of the project, with the repository and its
license recorded beside the citations; see `envelope.rs:70-84`, and the same statement in
`docs/theory/ggv-envelope.md`.

#### 8.4.4 Separable multiplicative corrections

A strategy sweep wants laps at perturbed states of the car: worn tires, which lower μ; a fuel load,
which changes mass; a different wing level, which changes $C_L A$.

Regenerating a full envelope for each variant would erase the win from precomputation.

The design instead stores three **relative sensitivities** at each node, and corrects
multiplicatively:

$$S_\mu \approx \frac{\partial \ln a_{y,\text{corr}}}{\partial \ln \mu}, \quad S_m \approx \frac{\partial \ln a_{y,\text{corr}}}{\partial \ln m}, \quad S_{C_LA} \approx \frac{\partial \ln a_{y,\text{corr}}}{\partial \ln C_LA},$$

$$a_{y,\text{corr}}(\ldots;\mu,m,C_LA) = gg(\ldots)\cdot\big(1 + S_\mu\,\tfrac{\Delta\mu}{\mu_0}\big)\big(1 + S_m\,\tfrac{\Delta m}{m_0}\big)\big(1 + S_{C_LA}\,\tfrac{\Delta C_LA}{C_LA_0}\big),$$

clamped at zero, and **the identity at the reference, by construction**.

The sensitivities are *secants*, not tiny tangents. They come from full T1 re-solves of the
boundary, on perturbed clones of the vehicle, at the edges of each parameter's intended band of
correction: ±15 % in μ, ±10 % in mass, and ±30 % in $C_LA$. Those are `H_MU`, `H_MASS`, and
`H_CLA`, at `envelope.rs:118-122`. The stored slope is therefore exact at the band edge, for a
linear response.

They are stored as separate **one-sided pairs, up and down**, because the response of a tire
coupled through the friction circle to μ is measurably convex. A query picks the upward secant above
the reference, and the downward one below; see `ay_boundary_corrected`, at `envelope.rs:458-487`.

There are guard rails everywhere. Sensitivities are sampled only at every 2nd $\hat a_x$ node,
within the bulk of a fibre near its peak, where the boundary is at least the maximum of 50 % of the
fibre peak and 0.5 m/s². Skipped nodes are filled by linear interpolation along the fibre. $v$ and
$g_{\text{normal}}$ keep their full resolution, because a variant that subsampled speed failed the
accuracy gate. The stored $|S|$ is at most 2.0. And the evaluated factor is clamped to
$[0.3, 3.0]$.

CI validates the correction against ground truth. At sampled grid nodes near the peak, the
corrected envelope must match a **full T1 re-solve** of the perturbed car, to within **2 % of the
local peak** at pure-lateral nodes; the realized value is about 0.6 % on the reduced CI grid. It
must match to within **12 %** at a moderate $|\hat a_x| = 0.4$.

That is a documented degradation toward the shoulders. There, the velocity-frame term
$-a_x\sin\beta$ dominates, and a multiplicative factor cannot move the shoulder itself. See the
tests in `envelope.rs` at lines 1016 to 1107, and `docs/theory/ggv-envelope.md`.

Further property tests pin: node-exactness of the interpolant, to less than 2 % of the local peak;
the identity at the reference, to $10^{-12}$; the signs of the corrections, so that more grip gives
a higher boundary and more mass a lower one; monotonicity in $g_{\text{normal}}$; concavity of the
$a_y(a_x)$ section, so that the feasible g-g region is convex; and zero-allocation queries.

One honest caveat. In v0.2.5 the corrected query `ay_boundary_corrected` is built, gated, and public
at the Rust level. But **no consumer at lap level uses it yet.** The API for batches and sweeps,
which would compose off-reference corrections into a lap, is future work.

A what-if from Python, through `overrides={...}`, instead re-resolves the vehicle, and generates and
caches a fresh envelope, keyed by the new resolved hash.

The machinery for corrections is the enabling groundwork for the strategy layer. It is not a
shortcut that you can reach from Python today.

#### 8.4.5 Real numbers

Here are queries that you can reproduce from Python, through `lap.envelope`. Chapter 10 documents
the `Envelope` class.

For the shipped `f1_2026`, whose reference mass is 768.0 kg, at $g_{\text{normal}} = g$:

| $v$ (m/s) | pure-lateral boundary `ay_boundary(v, 0, g)` | `accel_limit` | `brake_limit` | `drag_accel` |
|---|---|---|---|---|
| 15 | 12.76 m/s² (≈ 1.3 g) | 7.57 m/s² | 13.81 m/s² | 0.21 m/s² |
| 40 | 17.81 m/s² (≈ 1.8 g) | 9.91 m/s² | 21.06 m/s² | 1.54 m/s² |
| 80 | 32.08 m/s² (≈ 3.3 g) | 19.61 m/s² | 50.28 m/s² | 6.36 m/s² |

The funnel widens with speed, as downforce loads the tires.

The $g_{\text{normal}}$ axis matters just as much. At 40 m/s, the pure-lateral boundary is
13.12 m/s² over a $0.6g$ crest, but 23.45 m/s² in a $1.5g$ compression.

That swing by a factor of 1.8 is exactly what a flat g-g diagram cannot represent. It is why the
third axis exists.

### 8.5 Tier dispatch, end to end

#### 8.5.1 Selecting a tier

The tier comes from the `tier` field of `sim.yaml`. A `sim={...}` dict overrides that. And the
`tier=` keyword of `solve_lap` or `solve_lap_dataset` overrides that in turn; see Chapter 10.

Here is the enum, and, note well, the **default**:

```rust
pub enum Tier { T0, #[default] T1, T2, T3 }   // serialised: t0 | t1 | t2 | t3
```

A call to `solve_lap` with no `sim.yaml` and no `tier=` therefore gives you a **t1** lap. It is
"the default lap solver"; see `sim.rs:71-73`.

None of the shipped vehicle directories carries a `sim.yaml`. The defaults are therefore what you
get, unless you override them.

#### 8.5.2 What actually happens for each tier

The site of dispatch is the Python binding, at `crates/outlap-py/src/lib.rs:1010-1063`. It is a
match on an enum, at assembly time, and never inside a loop:

```rust
let qss: QssLap = match sim_cfg.tier {
    tier @ (Tier::T2 | Tier::T3) => return Err(err(tier_not_implemented(tier))),
    wanted => {
        let mut t1v = T1Vehicle::assemble(...)?;          // ALWAYS — even for t0
        let sidecar_fp = install_sidecars(&mut t1v, ...)?; // aero map + .ptm tables
        let env = cached_envelope(&t1v, &sim_cfg, ...)?;   // generate or reuse
        let t0v = T0Vehicle::assemble(...)?;
        let stack = build_slow_stack(...)?;                // battery + .emotor, or inert
        if wanted == Tier::T0 { solve_t0(...)? } else { solve_t1(...)? }
    }
};
```

Even a `t0` lap assembles the T1 vehicle. It needs the trim solver, in order to generate the
envelope.

Envelopes are cached for each process. The key holds everything that changes the boundary: the hash
of the resolved vehicle; a fingerprint of the sidecar tables, because two cars with identical
specifications but different aero-map parquet files must never share an envelope; the conditions;
the resolution of the grid; and the coupling mode.

`flat_track` is deliberately *not* in the key. It reshapes the path, not the boundary.

The cache is never evicted, because a session is short-lived. And `solve_lap` currently holds the
GIL for its whole duration.

Here are the two solve paths, from `crates/outlap-qss/src/qss.rs`.

- **`solve_t0`** runs the velocity profile on the envelope, optionally coupled to the slow states.
  It returns point-mass channels only, so `wheels: None` and `setup: None`.
- **`solve_t1`** runs the *identical* profile. It then walks the solved stations, calling
  `t1.trim(&TrimInput { v, ay, ax, g_normal, coupling })` at each one, to log the state of each
  wheel. It also calls `understeer_gradient(v, g_normal)` and
  `aero_front_downforce_share_at(v)`.

  A station whose re-trim is infeasible gets a **row of NaN** in the wheel channels, rather than an
  error. It can happen: the point-mass profile lives on an interpolated boundary, and the exact
  boundary of the trim is a hair away. Or the station speed is below the trim floor of 0.5 m/s.

  The dispatch tests require only that a majority of stations converge, on the reference cars.

Both paths thread the coupling of slow states identically, when a full electrified stack is present:
a battery, an `.emotor`, and drive units with Vdc maps. That is a fixed outer loop of two
iterations — solve, march, re-solve — which fills the traction scale at each station.

That is the story of Chapter 9. Here it is enough to know that an absent stack leaves the scale at
1, and the result *bit-identical* to the uncoupled solve.

Here is a summary of the observable differences:

| | `tier="t0"` | `tier="t1"` |
|---|---|---|
| Speed profile / lap time | envelope velocity profile | **identical** (same code path) |
| `s, v, ax, ay, t` (+ world `x, y, z`) | yes | yes |
| Per-wheel `vertical_load_n`, `slip_ratio`, `slip_angle_rad`, `force_long_n`, `force_lat_n` | no | yes — `(s, wheel)` arrays, wheel order `FL, FR, RL, RR`; NaN rows where infeasible |
| `understeer_gradient`, `aero_front_share` | no | yes |
| `state_of_charge`, `machine_temp_c` | only if a coupled stack was active | same |
| `lap.envelope` (queryable) | yes | yes |
| xarray dims | `s` only | `s` + `(s, wheel)` |

Every result records its provenance in attrs: `tier`, as "t0" or "t1"; `fz_coupling`, as
"one_step_lag" or "fixed_point"; `flat_track`, as an int in the dataset, because netCDF attrs have
no boolean type; `resolved_hash`; and the full `notes` tuple.

#### 8.5.3 A worked example

```python
from outlap.core import Track, solve_lap_dataset

tr = Track.load("data/tracks/catalunya_osm")

ds0 = solve_lap_dataset("data/vehicles/f1_2026", tr, tier="t0")
ds1 = solve_lap_dataset("data/vehicles/f1_2026", tr, tier="t1")

print(ds0.attrs["lap_time_s"], sorted(ds0.data_vars))
print(ds1.attrs["lap_time_s"], sorted(ds1.data_vars))
```

This produces the following, on v0.2.5, at the default 2 m stations on the centerline. It is a real
run, not an illustration.

```text
112.5195106151881 ['ax', 'ay', 't', 'v', 'x', 'y', 'z']
112.5195106151881 ['aero_front_share', 'ax', 'ay', 'force_lat_n', 'force_long_n',
                    'slip_angle_rad', 'slip_ratio', 't', 'understeer_gradient', 'v',
                    'vertical_load_n', 'x', 'y', 'z']
```

The lap time is identical, as promised. The `t1` dataset has dims `{s: 2339, wheel: 4}`, where `t0`
has `{s: 2339}`.

The first call pays for generating the envelope. That takes seconds in a release build, which is
exactly why the CI wheel is built with `--profile release`. It takes minutes in a debug build.

The second call reuses the process cache. It costs only the profile, plus, for `t1`, one trim and
two understeer probes at each station.

Chapter 10 documents the full surface of `Lap` and of the Dataset. Chapter 14 builds recipes on top
of it.

#### 8.5.4 `t2` redirects, and `t3` is a clean, typed refusal

The `t2` tier is real, but it is indexed by *time*. `solve_lap` therefore refuses to shoehorn it
into the result type indexed by arc length. Asking for it points you at the right door instead:

```text
the transient tier (t2) produces a time-indexed lap: call `outlap.solve_transient_lap(...)`,
or `outlap.solve_lap_dataset(..., tier="t2")` for an xarray view
```

`solve_lap_dataset(..., tier="t2")` handles the redirect for you, and returns the time-indexed
dataset of §8.7.

Requesting `t3`, which is the unimplemented 14-DOF model with suspension, produces
`QssError::TierNotImplemented`. At the Python boundary that is a `ValueError`. There are no partial
results, and no silent downgrade.

### 8.6 Flat-track mode: collapsing g-g-g-v to g-g

Set `sim.flat_track: true`, or `sim={"flat_track": True}`. The path sampler then switches to
`T0Path::from_track_flat`; see `path.rs:52-58`.

It keeps the plan-view curvature $\kappa_h(s)$. It zeroes the grade, the banking, and the vertical
curvature.

Consequently $g_{\text{normal}} \equiv g$, at every station and every speed. The four-dimensional
g-g-g-v envelope is therefore only ever queried on its slice at flat gravity. It *collapses to a
classical g-g-v*.

The physical track files are untouched. Only the path of this run is flattened. And the mode is
recorded in the result, in the `flat_track` attr.

This is the mode for comparing against a 2-D oracle. It exists for one main customer: the
**Limebeer cross-check**, at `docs/validation/limebeer.md`.

Perantoni & Limebeer (2014) is a 2-D study of an optimal-control F1 lap of Catalunya, with a fully
published parameter set for the car.

outlap reruns the transcribed `limebeer_2014_f1` on the minimum-curvature line of `catalunya_osm`,
with `flat_track: true` and the production 40×25×7 envelope.

CI gates the results. Top speed is 87.8 m/s, against the paper's about 88, which is −0.2 %, with a
gate of ≤1 %. The apex of the slowest corner is 17.7 m/s, against 17, which is +4.1 %, with a gate
of ≤5 %.

The lap time, 92.36 s against the paper's 82.43 s, is recorded but deliberately *not* gated. A QSS
solver on a heuristic minimum-curvature line structurally cannot match a transient optimal-control
lap that co-optimizes its own driven line. The validation page decomposes the delta, term by term.

Chapter 13, on validation, testing, and trust, walks through the whole cross-check, and the rest of
the suite of gates. Chapter 12 describes the vehicles and tracks involved.

That closes the QSS loop. A double-track trim is distilled into a gridded grip surface. A
point-mass sweep consumes it, in microseconds at each station. And a re-trim puts the four wheels
back into the output.

One tier remains: the one that stops assuming equilibrium altogether.

### 8.7 The transient tier (T2): driving the car through time

*Theory pages: [`transient_chassis.md`](theory/transient_chassis.md) for the equations of motion;
[`transient_control.md`](theory/transient_control.md) for the rule-based control layer;
[`driver.md`](theory/driver.md) for the preview driver; [`integrator.md`](theory/integrator.md) for
the split fixed-step integrator; and [`block-bus.md`](theory/block-bus.md) for the architecture of
blocks and the bus. Notebooks [08](../notebooks/08_transient_t2.ipynb) and
[09](../notebooks/09_race_engineering.ipynb) run everything in this section live.*

Everything so far solved the lap *station by station*, assuming that the car is in equilibrium at
each one.

The T2 tier drops that assumption. It **integrates the equations of motion of the car through
time**, at a fixed step of 1 ms, with a driver model closing the loop.

The output is not a table of speed for each station. It is a *trace from a data logger*: steering,
yaw rate, sideslip, per-wheel loads and slips, the gear engaged, and regeneration power, sampled
1000 times each second around the lap.

#### 8.7.1 The model: seven degrees of freedom, in a frame that follows the road

The chassis state is $[s, n, \psi_{\text{rel}}, v_x, v_y, r, \omega_1..\omega_4]$. That is: progress
along the track, $s$; lateral offset from the reference line, $n$; heading relative to the road,
$\psi_{\text{rel}}$; the body-frame velocities $v_x$ and $v_y$; the yaw rate $r$; and four wheel
spin speeds.

This is the **curvilinear road-frame** formulation of the lap-simulation literature, from Perantoni
& Limebeer 2014, with the 3D extension of Rowold et al. 2023.

Instead of tracking world x and y, and then asking "where is the road?", the road *is* the
coordinate system. Grade, banking, and vertical curvature enter the equations as known functions of
$s$. They rotate gravity, and they modulate the normal load. A crest unloads the tires, exactly as
Chapter 2 promised.

The world trajectory is reconstructed *from* the integrated $(s, n)$. It is never re-derived by
projecting onto the track.

The equations of motion are not trusted to hand algebra. A symbolic derivation, using Kane's method
in `docs/derivations/`, generates reference values. A CI test compares those against the Rust
implementation, to $10^{-12}$.

If anyone edits the chassis code, and its physics drifts from the derivation, CI fails.

Around the chassis sit the same physics blocks that the QSS tiers use, now evaluated at each
timestep. They are: the T1 load-transfer algebra, using the *same* expressions, so there is one
source of truth for $F_z$; the MF6.1 tire model, now fed **relaxation-lagged** slip, with the
first-order lag of `relax.rs` from Chapter 7 integrated exactly at each step; the aero platform; and
the `.ptm` powertrain torques.

The algebraic loop on vertical load is resolved by the same `fz_coupling` setting as T1. It
defaults to `fixed_point` at this tier. The "previous step" of `one_step_lag` is now a real
millisecond, so iterating within the step is the more faithful choice. Both are recorded.

#### 8.7.2 Closing the loop: a driver, and a rule-based control layer

A transient car goes nowhere without inputs. T2 therefore ships a deterministic **ideal driver**.

For the line, it uses MacAdam-style preview steering, plus a feedforward on curvature. For the
pedals, it uses a PI speed controller, tracking the T0 speed profile. See MacAdam 1981.

A rule-based control layer runs alongside it, in the phases `sense → control → actuate → integrate`
of every step. It has three parts.

- A **state machine for gear shifts**, running torque cut, ratio swap, and clutch ramp. It consumes
  the `shift_time_s` of the gearbox. Watch `gear` and `torque_scale` in the result.
- **Torque vectoring**. It commands a yaw moment,
  $\Delta M_z = k_{\text{yaw}} (r_{\text{target}} - r)$, allocated across the driven wheels, within
  the budget of each wheel's friction ellipse.
- **Blending of regeneration**. Under braking, recoverable torque goes to the electric machines,
  charging the pack, and only the remainder goes to the friction brakes. Under power, the traction
  draw discharges the pack.

  The battery follows as a slow state, on a decimated clock, exactly as in Chapter 9, but now
  *within* the time loop.

The driver has one deliberate idiosyncrasy, and it is the headline caveat of §1.5. It tracks a
**corner-scaled stability margin**.

The speed reference is shaped at each station, from the raw QSS profile, by `outlap_qss::margin`.
It takes the full profile speed where lateral demand is low. It takes about 0.85 of it where the
profile rides the lateral grip limit. It propagates each corner's margin back through that corner's
braking zone, plus a settle ramp of about 1.5 s. And it keeps both transitions dynamically
feasible, through passes for braking and traction that know the friction ellipse, evaluated at the
path's true `g_normal`.

Two stabilizers on the driver side complete the picture. A **sideslip damper**, `δ −= k_β·β`,
corrects a translational "crab" slide that the yaw damper cannot see. And a **pedal governor on
wheel slip** lets the ideal driver modulate the throttle against wheelspin at the drive wheels;
with race gearing, the torque in a low gear is a multiple of the grip limit.

The margin at the limit is not laziness. Pushed to the raw profile, the closed loop *spins the car*.
That is the caveat of Chapter 2 — "the QSS envelope is not filtered for stability" — made concrete.

The result: top speeds within a few percent of the QSS profile, and a lap about 14 % to 17 % slower
overall. The lap is honest about why, in its provenance, through the `speed_margin` attr, and in
the decomposition in `docs/validation/limebeer.md`.

#### 8.7.3 The integrator, briefly

A fixed-step **split** integrator advances the state. It uses Heun by default, and RK4 is
selectable; see `sim.dt_s` and `sim.integrator`.

The chassis and controller states go through the Runge–Kutta sweep. The tire relaxation states use
their exact exponential update, which is unconditionally stable at any step size. And the slow
states, such as SoC and temperatures, advance on a decimated clock.

CI verifies the convergence of the production stepper against a reference suite of ODEs. The hot
loop allocates nothing, which the same dhat harness as the QSS kernels enforces. And the step is
deterministic: the same T2 lap is bit-identical when re-run.

`docs/theory/integrator.md` gives the details and the citations.

#### 8.7.4 What comes back

`solve_transient_lap(vehicle_dir, track, ...)`, or `solve_lap_dataset(..., tier="t2")`, returns a
dataset over **`time`**, and over `wheel`.

It holds the chassis states; `steer`, `throttle`, and `brake`; the per-wheel `vertical_load_n`,
`slip_ratio`, `slip_angle_rad`, and forces; `gear`, `torque_scale`, `yaw_moment_nm`,
`regen_power_w`, and `traction_power_w`; and, with a battery present, `state_of_charge` and
`pack_temp_c`. The values of x, y, and z come from the integrated trajectory.

The provenance attrs record the resolved `fz_coupling`, `dt_s`, `integrator_order`, `speed_margin`,
and `flat_track`, and whether the lap `completed`.

Chapter 10 documents the full surface. Notebook 09 reads these traces the way a race engineer
would.

#### 8.7.5 What T2 is validated to be, and what it honestly is not

The parity gate on the physics of the T2 tier is **hull containment**; see Chapter 13. Every
operating point $(a_x, a_y)$ of the closed-loop lap must stay inside the T1 g-g-g-v envelope.

The measurement is **0.0 % exceedance**, on all three reference cars. The transient car never
produces grip that the quasi-steady physics says it cannot have.

What T2 is *not*, at v0.2.5, is an oracle for lap time. The corner stability margin of the driver
dominates its pace: about +14 % to +17 % against T0, with top speeds within about 2 % to 7 % of the
profile. And its throughput, about 62k steps/s on each core at the default coupling, is bound by
evaluating four Magic Formula tires at each step.

Both numbers are recorded, and tripwired against regression. The decomposition of pace lives in
`docs/validation/limebeer.md`. The analysis of throughput lives in the performance test itself, at
`crates/outlap-transient/tests/perf_throughput.rs`. Neither is hidden behind a green checkmark.

That completes the story of the solvers. Chapter 9 adds the parts of the car that change *while* the
lap runs: powertrain energy, machine heat, and battery state.


---

## 9. Physics III: powertrain, machine thermal, battery, and slow states

*What you will learn: how outlap turns everything downstream of the driver's right foot into physics
that it can solve. Powertrains are consumed as neutral map files. A drivetrain is described as a
graph of gearboxes and differentials. A thermal network makes lap 20 slower than lap 1. And a
battery, whose voltage sags, changes how hard the motor heats up. By the end you will know exactly
which numbers cap the car's acceleration at any point on the track, and why the braking side is
deliberately different.*

Everything in this chapter lives in a handful of places, which you can open alongside it:

| Concern | Where |
|---|---|
| `.ptm` powertrain-map schema | `crates/outlap-schema/src/ptm.rs`, published as `schemas/ptm.json` |
| Drivetrain topology types | `crates/outlap-schema/src/vehicle/drivetrain.rs` |
| Traction ceiling, differentials, energy accounting | `crates/outlap-qss/src/t1/powertrain.rs` |
| Machine thermal runtime (network + correlations) | `crates/outlap-thermal/src/network.rs`, `correlations.rs` |
| `.emotor` schema and its assembly | `crates/outlap-schema/src/emotor.rs`, `crates/outlap-qss/src/t1/thermal.rs` |
| Battery schema and runtime | `crates/outlap-schema/src/battery.rs`, `crates/outlap-qss/src/t1/battery.rs` |
| The slow-state lap-loop coupling | `crates/outlap-qss/src/qss.rs`, `crates/outlap-qss/src/solver.rs` |
| Theory pages (equations + citations) | `docs/theory/qss-powertrain.md`, `docs/theory/machine-thermal.md` |
| A vehicle with all three subsystems live | `data/vehicles/tesla_model3_rwd/` |

### 9.1 The firewall: a powertrain is a map, not a model

outlap never simulates the inside of an electric machine, an inverter, or a gearbox. That is a hard
rule of the project: the **firewall**.

A powertrain enters the simulator only as a **`.ptm` file**. That is a "powertrain map" document.
It describes *what a unit does at its shaft* — the torque available, the efficiency, and the losses
— without saying anything about *how* it does it. There is no electromagnetics, no switching loss,
and no model of a gear mesh. The module doc of the schema calls the format "the firewall"; see
`crates/outlap-schema/src/ptm.rs`.

Why? For two reasons.

The first is scope. outlap is a simulator of a vehicle and of a lap. Tools for designing a machine
already exist, and they produce exactly these maps.

The second is cleanliness. A map is a neutral contract, agnostic to any tool. The importers of
Chapter 11 read the HDF5 exports of a design tool, with plain `h5py`, and emit `.ptm` files. The
code and the data of that design tool never enter this repository. All committed powertrain data is
synthetic, and `python/tools/gen_model3_powertrain.py` regenerates it.

A `.ptm` document, at schema `ptm/2.0`, describes a unit at its shaft. It holds its `kind`, which is
`combustion` or `electric`; the grid `axes`, which are a strictly ascending axis of shaft speed, a
load axis, and, new in `ptm/1.1`, an optional axis of DC-link voltage, `vdc_v`; a `tables` sidecar,
carrying the dense data for efficiency and loss; and the `limits`.

Chapter 5 walks the format field by field, with the shipped `du_medium.ptm.yaml`. Here we care about
what those numbers *mean* for the lap.

- The **load axis runs negative**. Those negative values are the **regen quadrant**, where the
  machine acts as a generator under braking, and can recover energy rather than dissipate it.
- **`limits.max_torque_nm_vs_speed`**, the **peak-torque envelope**, is the single curve that the lap
  solver uses as its traction ceiling.

  Crucially, outlap does *not* trust the continuous rating on a datasheet. The optional
  `cont_torque_nm_vs_speed` and `overload` curves are references for validation only. The real
  thermal limit is *computed* from the loss tables, by the `.emotor` model of §9.6. That is the
  whole reason the machine-thermal network exists.
- With a `vdc_v` axis, the data for efficiency, loss, and torque becomes a 3-D tensor over
  `(speed, torque, voltage)`. §9.8 explains why a battery makes that third axis matter.
- **A missing sidecar table is not fatal.** The lap falls back to the peak envelope alone, with a
  note that energy accounting is off. Nothing is silent.

An internal-combustion engine is supported from day one, with the same format.
`data/vehicles/f1_2026/ptm/ice_v6.ptm.yaml` is a `ptm/2.0` map with `kind: combustion`, for a
synthetic 1.6 L V6 turbo. It includes a negative `drag_torque_nm_vs_speed` curve for engine braking,
running from −20 to −80 N·m across the rev range.

For an ICE, the `efficiency` in the sidecar is *brake thermal efficiency*. §9.5 shows how it becomes
a rate of fuel mass.

Finally, imports are gated. The round-trip test loads a `.ptm` file emitted by an importer, plus its
Parquet, through the real path for gridded maps. It must reproduce spot efficiencies from the source
arrays, to **1e-6**.

Cells beyond the torque envelope are unreachable. They carry `NaN`, and they are filled from the
nearest valid cell, and flagged as outside the hull. The column at zero torque is pinned to
$\eta = 0$.

CI runs on synthetic fixtures, shaped like the tool's output, and on nothing else. Real data from a
design tool never enters the repository. See Chapter 11, on importers and tooling, and Chapter 13,
on validation.

### 9.2 The drivetrain topology graph

A `.ptm` file tells you what a torque source can do. The `drivetrain:` block of the vehicle tells you
*where that torque goes*.

outlap models the drivetrain as a **directed graph**, and not as a fixed layout. In the words of
`crates/outlap-schema/src/vehicle/drivetrain.rs`: torque **sources**, which are `.ptm` files for an
ICE, an electric machine, or a lumped drive unit, connect to wheel **sinks**, through ordered
**coupler** elements, which are a gearbox, a differential, or a fixed ratio. Any concept with four
wheels is a topology plus data.

Each entry in `drivetrain.units` is a `DriveUnit`:

```yaml
drivetrain:
  units:
    - source: ptm/du_medium.ptm.yaml      # the torque source (.ptm)
      thermal: emotor/rear_du.emotor.yaml # optional machine-thermal model (Section 9.6)
      path:                               # ordered couplers, source → wheels
        - diff: { type: open }
      wheels: [RL, RR]                    # the wheel sinks (FL/FR/RL/RR)
```

That is the real wiring of the Model 3, from `data/vehicles/tesla_model3_rwd/vehicle.yaml`. It is
one rear drive unit, through an open differential, to the rear wheels. It is the `ev_1du_rwd`
reference pattern.

An all-wheel-drive EV is two such units. A conventional car is one ICE `.ptm` behind a `gearbox` and
a `diff`. A hybrid is both, feeding the same wheels. The layout is data, and never a variant chosen
at compile time. That is the rule on composition, from Chapter 6.

Three kinds of coupler exist. They are written `{gearbox: {...}}`, `{diff: {...}}`, and
`{fixed_ratio: 2.4}`.

| Coupler | Fields | Semantics |
|---|---|---|
| `gearbox` | `ratios` (index 0 = first gear), `final_drive`, `shift_time_s`, `efficiency` | Selectable ratios; `efficiency` is a bare constant (default **0.985**) or `{map: eff.parquet}` — a gridded map |
| `diff` | `type` (`open` / `locked` / `lsd` / `solid`), `preload_nm`, `ramp: [accel, decel]` | Splits an axle torque left/right (Section 9.4); `preload_nm` is required for `lsd`/`locked`; `ramp` is LSD-only |
| `fixed_ratio` | a single number | One fixed reduction; multiplies into the total ratio |

`solid` is the limit case of a locked differential. It gives day-one support for a kart, and for a
live axle.

A standalone clutch coupler is deferred. Shift and clutch dynamics live inside `Gearbox`.

Wheels are named `FL`, `FR`, `RL`, and `RR`, serialized in uppercase. Every per-wheel channel in the
results uses the canonical order `[FL, FR, RL, RR]`; see `WHEEL_ORDER` in
`crates/outlap-qss/src/qss.rs`.

The control layer is `drivetrain.control`. It holds **static splits** — `split.front` is the share of
torque on the front axle, from 0 to 1, and it is omitted for a car with one driven axle;
`split.left` is the share on the left side. And it holds a **stub for torque vectoring**, with
`enabled` and `k_yaw`, giving feedback on yaw rate as
$\Delta M_z = k_{\text{yaw}} \cdot (r_{\text{target}} - r)$.

Control is rule-based only in this release. Allocation based on optimization is on the roadmap; see
Chapter 15.

Because "config errors are a product surface", the loader validates the graph at load time, in
`crates/outlap-schema/src/load/topology.rs`, with messages in plain language.

Every unit must actually reach wheels. The private `path` of a unit may carry only its differential;
a reduction is the unit's `fixed_ratio:`, and a gearbox lives on the graph `couplers`. A wheel
driven rigidly by two units, with no differential between them, is a conflict; a parallel hybrid
that shares a differential passes. And torque vectoring cannot act across a locked or solid
differential on the same axle.

### 9.3 From the graph to the traction ceiling

The lap solvers of Chapter 8 need one number from all of this: *the largest drive force that the
powertrain can put on the road, at speed $v$*.

The T1 reduction, in `crates/outlap-qss/src/t1/powertrain.rs`, in the type `T1Powertrain`, folds
each unit's coupler path into a set of **gears**. It uses this convention for shaft speed and force:

$$\omega_\text{shaft} = \frac{\text{ratio}}{r_\text{wheel}}\, v, \qquad F_\text{wheel} = \frac{\text{ratio}\cdot\eta_\text{mech}}{r_\text{wheel}}\,\tau, \qquad \text{ratio} = \prod(\text{fixed ratios})\cdot\text{gear ratio}\cdot\text{final drive},$$

where $\omega_\text{shaft}$ is the shaft speed of the source, in rad/s; $v$ is the vehicle speed, in
m/s; $r_\text{wheel}$ is the unloaded radius of the driven tire, taking the front-axle radius for a
front-driven unit, the rear for a rear one, and the mean if a unit spans both axles; $\tau$ is the
source torque, in N·m; and $\eta_\text{mech}$ is the constant mechanical efficiency of the gearbox.

Fixed ratios multiply into the base ratio. A differential is 1:1 at the power level. The first
gearbox supplies the selectable ratios; a second gearbox folds in, as its final drive times its
first ratio.

A gearbox declared with a *map* efficiency assembles fine. But it contributes a conservative
constant proxy of **0.95** to the traction force, until the map is installed. That is recorded in
the assembly notes. Nothing is silent.

The **traction ceiling** is then the best on-envelope gear of every unit, summed:

$$F_{\max}(v) \;=\; \sum_{\text{units}}\ \max_{g\,:\,\omega_g \le \omega_\text{max}}\ \frac{\tau_\text{peak}(\omega_g)\cdot \text{ratio}_g \cdot \eta_g}{r_\text{wheel}},$$

with $\tau_\text{peak}(\omega)$ the peak envelope from the `.ptm` file, fitted with the project's one
shared monotone cubic Hermite interpolant, in the Fritsch–Carlson construction; see Chapter 5.

A gear whose shaft speed exceeds the top of the envelope is simply rev-limited out. See
`PtUnit::max_wheel_force` and `T1Powertrain::max_drive_force`. Both are allocation-free, under the
hot-loop rules of Chapter 6.

**A worked example.** The medium drive unit of the Model 3 has `kind: electric`. Its map is lumped
at the output shaft, because the unit declares no `fixed_ratio:`. Its "shaft" is therefore the shaft
on the wheel side; that is why its speed axis runs only from 10 to 1990 rpm. The path contributes a
ratio of 1 and an efficiency of 1, because a differential is 1:1 at the power level.

The unloaded radius of the rear road tire is 0.313 m; see `UNLOADED_RADIUS` in
`data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml`.

Below 670 rpm the envelope holds its plateau of 2765 N·m. So

$$F_{\max} = \frac{2765\ \mathrm{N\,m} \times 1 \times 1}{0.313\ \mathrm{m}} \approx 8834\ \mathrm{N} \quad\Rightarrow\quad a_x \approx \frac{8834}{1765\ \mathrm{kg}} \approx 5.0\ \mathrm{m/s^2} \approx 0.51\,g,$$

which sits comfortably below what a warm road tire can grip. An EV limited at launch is therefore
limited by its traction map, and not by grip, exactly as you would expect.

At the top breakpoint of the envelope, which is 1990 rpm, or 208.4 rad/s, at 972.6 N·m, the
mechanical power is $\tau\,\omega \approx 203$ kW. That is the sizing quoted in the README of the
vehicle. And the rev limit puts the force ceiling at zero beyond
$v = \omega_\text{max} r \approx 65$ m/s, which is about 235 km/h.

Three deliberate separations are worth internalizing.

1. **Efficiency never reduces force.** The torque envelope in the `.ptm` file is already the
   *mechanical output* of the unit. The efficiency map governs the *energy drawn*, and not the
   force delivered; see `docs/theory/qss-powertrain.md`.

   A motor that is 90 % efficient, making 300 N·m, still makes 300 N·m. It just draws more
   electrical power while doing it.
2. **The g-g-g-v envelope does not contain the powertrain.** Following Werner et al. (2025, §II-C),
   the acceleration envelope of Chapter 8 is the limit on tire force only.

   The lap solver applies the powertrain ceiling separately. In its forward pass it takes
   $\min(a_{x,\text{grip}},\ F_{\max}(v)\,/\,m - a_\text{drag})$; see
   `crates/outlap-qss/src/solver.rs`.
3. **Braking is limited by friction.** The backward, or braking, pass uses grip, drag, and grade
   only. There is no powertrain term. At every tier in this release, the brakes are assumed strong
   enough that the tire is the limit.

One asymmetry between tiers is worth knowing. An F1-style ERS, which is an energy-recovery system,
is *not* folded into the T1 traction ceiling. Its schema is `crates/outlap-schema/src/vehicle/ers.rs`,
and it covers the MGU-K only, because "MGU-H removed per the 2026 F1 regulations". It is a separate
rule-based mechanism for deployment, and it will be folded in with the future energy manager.

The simpler T0 point-mass vehicle *does* add a force from an ERS to its `tractive_force`, capped on
power and tapered with speed. Both facts are recorded in the assembly notes.

The clean-room references for this module are cited in `crates/outlap-qss/src/t1/powertrain.rs` and
in `docs/theory/qss-powertrain.md`. They are: Perantoni & Limebeer, *"Optimal control for a Formula
One car with variable parameters"*, Vehicle System Dynamics 52(5), 2014; Guiggiani, *The Science of
Vehicle Dynamics*, 2nd ed., 2018, ch. 3; and Milliken & Milliken, *Race Car Vehicle Dynamics*, 1995,
ch. 20.

### 9.4 Differentials: who gets the torque

A **differential** is the gear set that lets the two wheels of an axle turn at different speeds
through a corner, while sharing the drive torque.

How it shares that torque changes the balance of the car. The QSS trim of Chapter 8 therefore treats
the split as a genuine unknown, and not as a step of post-processing.

`DiffModel`, in `t1/powertrain.rs`, implements the exact semantics.

**Torque-bias capacity** is the maximum *difference* in torque that the differential can sustain
between its output shafts, at an axle torque of $\tau_\text{axle}$; see
`DiffModel::max_torque_bias`:

| Kind | Bias capacity | Meaning |
|---|---|---|
| `open` | $0$ | both shafts always carry equal torque |
| `locked` / `solid` | $+\infty$ | any difference; the housing reacts it |
| `lsd` | $T_\text{bias} = \text{preload} + \text{ramp}\cdot\lvert\tau_\text{axle}\rvert$ | preload plus a load-proportional lock (drive ramp under acceleration, decel ramp under braking) |

**The split of drive torque** depends on the grip torque available at each side, which is
$\mu F_z r$; see `DiffModel::split`.

An `open` differential gives equal halves. The side with less grip therefore caps what the axle can
deliver. That is the classic limit of one wheel spinning.

A `locked` or `solid` differential gives a split proportional to grip. Each side takes what it can
hold, and the two sum to the total. The difference in force between left and right then produces a
yaw moment.

An `lsd` gives a split proportional to grip, and then clamps the difference between the sides into
$\pm T_\text{bias}$, while keeping the sum.

Inside the live trim, the 9th unknown $w$ is the **slip split** on the driven axle, where
$\kappa_\text{left} = s + w$ and $\kappa_\text{right} = s - w$. A residual for each kind closes it;
see `docs/theory/qss-powertrain.md`.

An open differential gives equal longitudinal force, $F_{x,\text{left}} - F_{x,\text{right}} = 0$,
with unequal slip. A locked or solid differential gives equal speed, so $w = 0$.

**An LSD uses the locked constraint in the trim.** That is a documented simplification for
quasi-steady state: an LSD with preload locks up at the traction limit, and partial unlocking is a
refinement for T2.

Under braking, $w = 0$ always. The balance bar splits the brake torque, not the differential.

This is not bookkeeping. It produces real, visible physics.

When a car with an open differential asks for maximum lateral *and* maximum longitudinal
acceleration at once, the inner wheel unloads, until the root with equal torque ceases to exist. The
point then becomes a clean traction boundary. The theory page notes that the front-wheel-drive
reference car shows exactly this, at $\lvert a_y\rvert = 6$ and $a_x = 3$ m/s².

A locked differential at the same point delivers the *sum* of what the two wheels can do. And its
difference in force between left and right feeds a yaw moment straight into the moment balance of
the trim.

Two conventions are codified in the implementation. An LSD `ramp` value greater than 1 is read as a
*percent* of lock-up; it is divided by 100, then clamped to $[0,1]$; see `lock_fraction`, documented
in the theory page. And a rigid path with two driven wheels and *no* differential coupler defaults
to `locked`, because "a rigid two-wheel drive with no diff is a solid axle".

**Conservation, and the splits.** Every coupler is a linear gain on torque. The identity that the
property test checks is $\sum \tau_\text{wheel} = \tau_\text{source}\cdot\text{ratio}\cdot\eta$; see
`T1Powertrain::wheel_torque`.

The static splits always sum to one. `axle_split()` returns `(front, 1 − front)`, defaulting to
`(0, 1)`, which sends all torque to the driven axle. `side_split()` returns `(left, 1 − left)`,
defaulting to `(0.5, 0.5)`. Both are clamped to $[0,1]$.

### 9.5 Energy accounting, and closure

Force is only half the story. The other half is *what it costs*.

The sidecar tables are decoded and installed by `T1Powertrain::install_maps`. The Parquet decode
happens at the native edge, so that the solver crates stay wasm-clean.

After that, every operating point $(n, \tau)$ on a source shaft, with mechanical power
$P_\text{mech} = \tau\,\omega$, yields an `EnergyPoint`:

$$
\begin{aligned}
\text{drive } (\tau > 0):\quad & P_\text{source} = P_\text{mech}/\eta, & \text{loss} &= P_\text{mech}\,(1/\eta - 1),\\
\text{regen } (\tau < 0):\quad & P_\text{source} = P_\text{mech}\cdot\eta, & \text{loss} &= \lvert P_\text{mech}\rvert\,(1 - \eta),\\
\text{ICE fuel rate}:\quad & \dot m_\text{fuel} = P_\text{source}/\text{LHV} & &(\text{when } P_\text{source} > 0),
\end{aligned}
$$

$\eta$ is the efficiency from the map. It is clamped to $[10^{-3}, 1]$ inside the energy math, so
that a degenerate cell cannot blow up a division.

LHV is the lower heating value of the reference fuel. It is a constant,
`FUEL_LHV_J_PER_KG = 43.0e6`; petrol is about 43 MJ/kg. It will be configurable later.

In the words of the code: "An ICE burns fuel whenever it draws chemical power (drive or idle);
motoring does not." Fuel mass is accounted for, but held constant this release. There is no slow
state for fuel mass yet.

State the sign conventions once, because they thread through everything here. The vehicle frame is
ISO 8855: $x$ forward, $y$ left, $z$ up. A positive drive force therefore pushes the car along $+x$.

A positive shaft torque is the drive quadrant, and a negative torque is the regeneration or motoring
quadrant. That matches the load axis of the `.ptm` file, where negative values are regeneration.

And battery current is **positive on discharge**, so a demand for regeneration power is negative at
the pack.

When a **loss map** is present, the loss is taken as measured, and energy closes *exactly*:
$P_\text{source} = P_\text{mech} + \text{loss}$, in every quadrant, *including idle*.

The column at zero torque in the sidecar carries $\eta = 0$, as a sentinel, because efficiency is
meaningless at zero output. But its `loss_w` there is the real draw for spin and idle. The committed
medium unit of the Model 3, for instance, draws 124 W at its lowest speed breakpoint, with zero
torque commanded.

Without a loss map, the loss is derived from the efficiency, and it closes to interpolation accuracy
between grid nodes.

The aggregate for each segment, which the lap loop consumes, is
`T1Powertrain::traction_energy(v, wheel_force_n, vdc)`. It returns
`TractionEnergy { source_w, loss_w, omega_rad_s }`.

It distributes the requested positive drive force across the mapped units, in proportion to the
capacity of each unit's best gear at that speed. It evaluates the $(n, \tau)$ point of each unit,
through that unit's map, which may be coupled to voltage. And `omega_rad_s` reports the *fastest*
contributing shaft speed, which is the driver of air-gap cooling in the thermal step below.

It returns `None` when no unit has an efficiency map installed. The whole coupling of slow states
then stays inert.

One simplification is documented. A hybrid with both an ICE and an electric drive attributes the
full traction draw to the mapped units: "a distinct ICE source is treated as battery-fed". That is
exact for a pure-electric car, and conservative otherwise.

Under the project gate of "new physics gives a new property test", this module ships with tests that
you can read as an executable specification. They are in
`crates/outlap-qss/tests/t1_powertrain.rs` and `properties.rs`, and the theory page lists them.

They check: that an open differential gives equal torque; that a locked or solid one gives a split
proportional to grip; that an LSD stays inside its bias band; that a coupler conserves torque, as
$\sum\tau_\text{out} = \tau_\text{in}\cdot\text{ratio}\cdot\eta$; that the splits sum to one; that
energy closes at the drive nodes; that the fuel rate of an ICE is positive under load; that an open
differential splits the slip of the driven wheels, in the live trim; and that a geared engine has a
traction ceiling that is positive, and that falls with speed.

The advance for each segment is also under the zero-allocation gate; see `tests/alloc.rs`.

A note for anyone reading the code. The `outlap-powertrain` crate in the workspace is an empty
placeholder. Everything in this chapter lives in `outlap-qss::t1::powertrain`,
`outlap-qss::t1::thermal`, `outlap-qss::t1::battery`, and `outlap-thermal`.

### 9.6 Machine thermal: the N-node LPTN

A cold motor and a heat-soaked motor are different cars.

outlap models this with a **lumped-parameter thermal network**, or LPTN. The machine is divided into
$N$ isothermal lumps, called "nodes": winding, stator iron, rotor, housing, coolant, and ambient.

Each node has a heat capacity $C_i$, in J/K. Thermal conductances $g_{ij} = 1/R_{ij}$, in W/K,
connect them. And the losses from the `.ptm` file are injected as heat sources $P_i$, in W, at each
node.

The result is a small linear system of ODEs. It is cheap enough to advance on every track segment.

#### 9.6.1 The amendment to the firewall

Read strictly, the firewall of §9.1 forbids modeling the internals of a machine. And a thermal
network *is* a model of machine internals.

This is the one deliberate exception, which the author authorized. It is recorded in
`docs/theory/machine-thermal.md`.

The rule was originally a fixed 2-node model. It was amended to allow a network of any $N$. And for
the *detailed* path, outlap even builds the conductance operator from the geometry of the machine,
using ported heat-transfer correlations.

In the words of the page: "The amendment is narrow — it applies to the thermal model only; torque,
efficiency and loss still cross the firewall as neutral `.ptm` maps."

The correlations are implemented clean-room, from the published literature cited below. The
geometry-building code of the upstream tool was not ported.

#### 9.6.2 The network, and its integrator

Each integrated node obeys an energy balance. The ambient and coolant nodes are boundary conditions;
see `docs/theory/machine-thermal.md`:

$$C_i\,\frac{dT_i}{dt} = P_i + \sum_j g_{ij}\,(T_j - T_i), \qquad T_\text{ambient} = T_\text{amb}, \qquad T_\text{coolant} = T_\text{inlet} + \frac{Q_\text{in}}{2\,\rho\, c_p\, \dot m},$$

$T_i$ is a node temperature, in K. $T_\text{amb}$ is the pinned ambient, from `conditions.yaml` or
an override.

The coolant node is closed by a **quasi-static jacket balance**. With heat inflow $Q_\text{in}$, and
a coolant capacity rate $\rho c_p \dot m$ in W/K, the coolant sits at the mean of the inlet and
outlet temperatures. It is not integrated.

Write the conductance operator $G$, with the Kirchhoff diagonal $G_{ii} = -\sum_{j\ne i} g_{ij}$.
The system is then $C\dot T = GT + P$, and the update is a **Crank–Nicolson**, or trapezoidal, step:

$$\left(\frac{C}{h} - \frac{G}{2}\right) T_{+} = \left(\frac{C}{h} + \frac{G}{2}\right) T + P .$$

Crank–Nicolson matters here because it is **A-stable**.

The step size $h$ over a track segment is $\Delta s / v$, which can be a second or more. That would
blow up an explicit integrator on a stiff network. Here it stays bounded, however coarse the
segments are.

The implementation is `crates/outlap-thermal/src/network.rs`. It assembles $G$ at the current
temperatures, which makes it semi-implicit, since a convection conductance depends on temperature.
It replaces the ambient row with $T_+ = T_\text{amb}$, and the coolant row with its balance target.
It then solves by Gaussian elimination of fixed size, with partial pivoting.

Everything lives in stack buffers, sized `MAX_NODES = 24`. The full network of the upstream design
tool is 20 nodes. The advance is therefore allocation-free, under the hot-loop discipline of
Chapter 6.

Failures are typed: `TooManyNodes`, `Singular`, `NonFinite`, and `BadStep`. In the words of the
code, "the QSS caller consumes these as a flagged failure, never a panic".

Two families of edge feed $G$.

- **Constant edges**, `Edge { i, j, g_w_per_k }`. They are the skeleton of conduction and contact,
  with fixed values in W/K.
- **Convection edges**, `ConvEdge`. They are recomputed at every step, from a published correlation,
  as $g = h\cdot A$, or $\lambda_\text{eff} A/\delta$ for the air-gap film. Cooling therefore
  depends on shaft speed and on temperatures:

| `ConvKind` | Physics | Citation (on the function, `crates/outlap-thermal/src/correlations.rs`) |
|---|---|---|
| `AirGap` | modified-Taylor-number regimes: $\mathrm{Nu} = 2$ ($\mathrm{Ta}_m < 1700$), $0.128\,\mathrm{Ta}_m^{0.367}$ ($<10^4$), $0.409\,\mathrm{Ta}_m^{0.241}$ above; hot gap $\delta = \delta_0 - \kappa_\text{Fe}\, r_\text{gap}(T_\text{rotor}-T_\text{amb})$ | Becker & Kaye, *J. Heat Transfer* 84(2), 1962 |
| `RotorAir` | end-winding $h = 6.5 + 5.25\,u^{0.6}$, internal air $h = 15 + 6.75\,u^{0.65}$, $u$ = rotor peripheral speed | Kylander, doctoral thesis, Chalmers, 1995 |
| `ShaftExternal` | $\mathrm{Nu}_d = 0.076\,\mathrm{Re}_d^{0.7}$ | Etemad, *Trans. ASME* 77, 1955 |
| `FreeConvection` | Churchill–Chu cylinder $\mathrm{Nu}(\mathrm{Ra})$ + linearized radiation $h_\text{rad} = \varepsilon\sigma(T_w^2+T_a^2)(T_w+T_a)$ | Churchill & Chu, *Int. J. Heat Mass Transfer* 18, 1975 |
| `LiquidChannel` | laminar $\mathrm{Nu} = 4.36$ below $\mathrm{Re}=2300$, Gnielinski above 3000, linearly blended between (pump-driven ⇒ speed-independent) | Gnielinski, *Int. Chem. Eng.* 16, 1976 |

Helpers for a TEFC fin channel are also implemented. They use the Heiles form, with the turbulence
factor of 1.7 from Staton & Cavagnino, *IEEE Trans. Ind. Electron.* 55(10), 2008.

Air properties use fits from polynomials and the ideal-gas law, valid over roughly 250 K to 500 K.
The default for iron expansion is $\kappa_\text{Fe} = 10.4\times10^{-6}\,\mathrm{K}^{-1}$.

The consequence is physically pleasing, and the validation figure verifies it. The air-gap film
*stiffens* with shaft speed. The rotor magnet therefore runs cooler at high speed, for the same
loss.

#### 9.6.3 The `.emotor` document, and the cooling block

The network is declared in an **`.emotor`** file, at schema `emotor/1.1`, defined in
`crates/outlap-schema/src/emotor.rs` and published as `schemas/emotor.json`. The `thermal:` field of
a drive unit references it.

The shipped file for the Model 3 is a complete and readable example; see
`data/vehicles/tesla_model3_rwd/emotor/rear_du.emotor.yaml`:

```yaml
schema: emotor/1.1
nodes:
  - { name: winding, role: winding, c_j_per_k: 6500.0, t_warn_c: 150.0, t_max_c: 180.0 }
  - { name: stator_iron, role: stator_iron, c_j_per_k: 11000.0 }
  - { name: rotor, role: rotor, c_j_per_k: 5500.0, t_warn_c: 140.0, t_max_c: 170.0 }
  - { name: housing, role: housing, c_j_per_k: 26000.0 }
  - { name: coolant, role: coolant }
  - { name: ambient, role: ambient }
conductances:
  - { between: [winding, stator_iron], w_per_k: 160.0 }
  - { between: [stator_iron, housing], w_per_k: 320.0 }
  - { between: [housing, ambient], w_per_k: 6.0 }
cooling:
  ambient_node: ambient
  jacket:
    housing_node: housing
    coolant_node: coolant
    inlet_c: 45.0
    flow_rate_lps: 0.40
    channel_count: 12
    channel_width_mm: 8.0
    channel_height_mm: 9.0
    wetted_area_m2: 0.080
    fluid: { named: ethylene_glycol_50 }
  air_gap:
    between: [stator_iron, rotor]
    rotor_outer_radius_mm: 65.0
    gap_mm: 0.8
    stack_length_mm: 134.0
loss_routing:
  - { node: winding, fraction: 0.55 }
  - { node: stator_iron, fraction: 0.30 }
  - { node: rotor, fraction: 0.15 }
cu_feedback: { nodes: [winding], t_ref_c: 60.0, alpha_per_k: 0.0039 }
```

A node **role** does double duty. It drives the mass heuristics of §9.6.5, and it identifies a
special node.

`winding` is required somewhere in the document. It is the default target for loss, and typically
the limit that binds. `rotor` "carries the magnet limit for PM machines". `stator_iron` and
`housing` are the usual path of conduction. `coolant` and `ambient` are boundary nodes, whose heat
capacities are ignored, because the ambient is pinned and the coolant is closed by its balance. And
`other` covers a node resolved by finite elements, on the detailed path.

A node derates only if it declares both `t_warn_c` and `t_max_c`.

The **cooling block** is deliberately written in raw scalars, which a user or an importer can read
off a datasheet. The assembly then derives the physics; see `crates/outlap-qss/src/t1/thermal.rs`.

- `jacket`. From the channel count $n$, the width $w$, the height $h$, and the flow $Q$, assembly
  derives four things: a mean velocity of $Q/(n\,w\,h)$; a hydraulic diameter
  $D_h = 2wh/(w+h)$; a coolant capacity rate $\rho c_p \dot m = \rho\, c_p\, Q$; and a
  `LiquidChannel` convection edge between housing and coolant, over `wetted_area_m2`.

  A fluid comes as a named preset — `water`, `ethylene_glycol_50`, or `oil`, tabulated at a film
  temperature near 60 °C to 70 °C — or as explicit `props`. An unknown name is a typed error, and it
  lists the known fluids.

  Declaring both `cooling.coolant`, which is the low-level escape hatch with an explicit
  $\rho c_p\dot m$, *and* `jacket`, is an error.
- `air_gap`. From the outer radius of the rotor, the gap, and the stack length, assembly derives
  $r_\text{gap} = r_\text{ro} + \text{gap}/2$, and an interface area
  $A = 2\pi\, r_\text{gap}\, L$. Those feed the speed-dependent Becker–Kaye film, between stator and
  rotor.

`cooling.ambient_fixed_c` overrides the ambient. When it is omitted, the `ambient_c` of the
session's `conditions.yaml` is used. The environment stays in the conditions file, under the rule of
the input quartet; see Chapter 4.

Initial temperatures default to the sink of each node: the ambient, and the coolant node at its
inlet. Override them with `initial_temp: {uniform_c: ...}`, or with a value for each node.

#### 9.6.4 Routing the loss, copper feedback, and derating

On each segment, the loss that heats the machine, from the `.ptm` lookup, is deposited into the
nodes through `loss_routing`; see `MachineThermal::step`.

Each route names a node and a `fraction`, and optionally a `component`, which is a named column in
the loss map of the `.ptm` file.

The rule is this. A declared route deposits its share. And **whatever total loss is not routed lands
on the winding node**. Nothing ever removes heat.

An empty routing list therefore puts *all* the loss into the winding, which is the conservative
default.

Per-component columns are currently a hook. The lap loop passes a resolver that always returns
`None`, so only fractions of the total loss are live this release. The importer uses breakdowns by
component only to compute the routing *fractions*.

The runtime surface is small, and worth knowing.
`MachineThermal::step(machine_loss_w, component_loss, omega_rad_s, dt_s)` returns the derate for the
segment. `machine_loss_w` is the total loss that heats the machine, in watts, and `omega_rad_s`
drives the speed-dependent convection edges.

`winding_temp_c()` is "the representative machine temperature the QSS slow-state coupling logs per
segment". `temp_c(name)` and `node_names()` expose the rest of the network.

**Feedback from copper resistance.** The resistance of a winding rises with temperature, as
$R(T) = R_\text{ref}\,(1 + \alpha (T - T_\text{ref}))$, with $\alpha \approx 0.00393\ \mathrm{K}^{-1}$
for copper.

When `cu_feedback` is enabled, the loss at the listed nodes is therefore rescaled by
$1 + \alpha(T - T_\text{ref})$ at each step, floored at 0.

This is positive feedback: a hotter winding gives more loss, which gives a hotter winding. What
keeps it physically bounded is the **derate**:

$$\text{derate} = \min_{\text{rated nodes}}\ \operatorname{clamp}\!\left(\frac{T_\text{max} - T}{T_\text{max} - T_\text{warn}},\ 0,\ 1\right),$$

which is a linear ramp from 1 to 0, as each rated node crosses from its warning temperature toward
its maximum.

A node takes part only if it declares *both* `t_warn_c` and `t_max_c`. In the Model 3 file, the
winding runs 150 to 180 °C, and the rotor and its magnets run 140 to 170 °C. A boundary node never
derates. And a degenerate case, where `t_max ≤ t_warn`, becomes a hard step.

The winding normally binds.

The lap solver multiplies the traction ceiling by this factor (§9.9). The reduced torque then
reduces the loss on the next segment. That is the physical loop, closed.

#### 9.6.5 Two tiers of authoring, one integrator

The same integrator serves a community user typing YAML, and a detailed import.

- **Lumped, and hand-authored.** It has role-tagged nodes, and constant conductances.

  Anything omitted is filled from *documented mass heuristics*, using the `mass_kg` of the `.ptm`
  file.

  A capacity is $C = f_\text{role}\cdot m\cdot c_p$. A winding takes $0.15\,m$ at 385 J/kg·K, for
  copper. Stator iron takes $0.45$ at 460. A rotor takes $0.25$ at 450. A housing takes $0.15$ at
  900, for aluminium.

  A conductance takes a reference value for the pair of roles, at $m_0 = 40$ kg, scaled by
  $(m/m_0)^{2/3}$, because interface area goes as mass^{2/3}. The references are: winding to stator
  30 W/K; winding to housing 8; stator to housing 60; housing to coolant 200; housing to ambient 5;
  and rotor to anything 3.

  Every heuristic fill is recorded in `estimates()`, and surfaced in the loaded-model report. An
  estimate is visible, and never silent.

  A node with no capacity, and no applicable heuristic, is a hard error, telling you to set it
  explicitly.
- **Detailed, and imported.** The importer collapses a network resolved by finite elements onto the
  same reduced menu. It uses explicit capacities, real conductances between groups, and convection
  edges rebuilt from the correlations on each segment.

  The `convection` edge list of `emotor/1.1` remains an advanced escape hatch, for a fully explicit
  network.

  This release covers three machine topologies: IPM, SPM, and SynRM.

Validation, with the full story in Chapter 13: the Crank–Nicolson advance matches the analytic step
response of a single node, $T(t) = T_\text{amb} + (P/g)(1 - e^{-t g/C})$; the ambient stays pinned;
the derate is monotone in temperature; a stint soaks up heat monotonically; the coolant node holds
its quasi-static target; and copper feedback raises the steady state.

### 9.7 The battery pack: a Thevenin equivalent circuit

For an electric car, the battery sets two more limits. It sets how much power the car can deliver at
all. And, more subtly, it sets what *voltage* that power is delivered at.

outlap models the pack as a **Thevenin equivalent circuit**, or ECM. That is an ideal voltage
source, which is the open-circuit voltage, or OCV, behind a series resistance $R_0$, and one pair of
resistor and capacitor, $(R_1, \tau_1)$, which captures the slow "sag" after a step in load.

With a discharge current $I$ taken as positive, the terminal voltage on the DC link is

$$V_\text{term} = \mathrm{OCV}(\mathrm{SoC}, T) - I\,R_0 - V_\mathrm{RC}, \qquad V_\mathrm{RC} \to I R_1 \ \text{at time constant } \tau_1 .$$

All five parameters — OCV, $R_0$, $R_1$, $\tau_1$, and the entropic coefficient
$\mathrm{d}U/\mathrm{d}T$ — are tabulated on a $(\mathrm{SoC}, T)$ grid, in a sidecar. Its columns
are `soc, temp_c, ocv_v, r0_ohm, r1_ohm, tau1_s, dudt_v_per_k`. The shipped
`pack_800v.tables.parquet` has 18 rows: 6 SoC values × 3 temperatures.

The form of the equivalent circuit, and its state equations, follow the published NREL `thevenin`
model (BSD-3), and the ECM literature that it cites: Plett, *Battery Management Systems* Vol. 1,
2015, ch. 2–3. It is re-authored clean-room, in `crates/outlap-qss/src/t1/battery.rs`.

The `battery/1.0` document, defined in `crates/outlap-schema/src/battery.rs`, adds the context of the
pack around the cell curves. The vehicle references it as `battery: {model: rc_pairs, params: ...}`.

Here it is, from the shipped `data/vehicles/tesla_model3_rwd/battery/pack_800v.battery.yaml`:

```yaml
topology: { ns: 220, np: 1 }          # 220 cells in series, 1 parallel string
capacity: { q_pack_ah: 92.0, e_pack_wh: 64064.0 }   # energy is informational
soc_window: [0.05, 0.98]              # usable state-of-charge window
ecm:
  rc_pairs: 1
  tables: { file: pack_800v.tables.parquet, level: cell }
limits:
  peak_discharge_power_w_vs_soc:      # 70 kW at 5% SoC rising to 265 kW at high SoC
    soc: [0.05, 0.20, 0.40, 0.60, 0.80, 1.00]
    power_w: [70000, 160000, 230000, 255000, 265000, 265000]
  peak_regen_power_w_vs_soc:          # 190 kW low-SoC falling to 30 kW near full
    soc: [0.05, 0.20, 0.40, 0.60, 0.80, 1.00]
    power_w: [190000, 190000, 170000, 140000, 85000, 30000]
  cell_v_min: 2.7
  cell_v_max: 4.2
  max_c_rate: 4.5                     # informational; the power limits bind first
thermal:
  mass_kg: 460.0
  cp_j_per_kgk: 900.0
  thermal_resistance_k_per_w: 0.02
  coolant_temp_c: 25.0
```

A table at cell level scales to the pack by $n_s$ for a voltage, and by $n_s/n_p$ for a resistance.

Only `rc_pairs: 1` is supported; anything else is a typed error. The ECM maps *clamp* outside their
grid, because "the ECM is only defined on its measured hull".

`Pack::assemble` starts the state at the **top of the SoC window**. That is full charge, which is
the reference state that the static envelope assumes. It starts at coolant temperature, with a
relaxed RC branch. You can pass an explicit `initial_soc`.

**The advance for each segment.** `Pack::step_power(state, power_w, dt)` does three things, in
order.

1. **Clip to the power envelope.** A demand for discharge is capped by `discharge_power_limit_w`.
   That is the monotone-cubic curve that depends on SoC, forced to exactly 0 at or below the floor
   of the SoC window. A demand for regeneration is capped by `regen_power_limit_w`, which is 0 at or
   above the ceiling. The clipping is reported through `power_limited`.
2. **Solve the current**, from the Thevenin relation at constant power,
   $R_0 I^2 - \mathrm{emf}\,I + P = 0$, with $\mathrm{emf} = \mathrm{OCV} - V_\mathrm{RC}$.

   Take the physical root, at low current:
   $I = \bigl(\mathrm{emf} - \sqrt{\mathrm{emf}^2 - 4 R_0 P}\bigr)/(2R_0)$.

   If the demand exceeds the maximum deliverable power, $P_\text{max} = \mathrm{emf}^2/(4R_0)$, then
   the current at maximum power, $\mathrm{emf}/(2R_0)$, is used. $R_0$ is floored at
   $10^{-9}\,\Omega$.
3. **Advance three slow states.**

   $V_\mathrm{RC} \leftarrow V_\mathrm{RC}\,e^{-\Delta t/\tau_1} + I R_1 (1 - e^{-\Delta t/\tau_1})$.
   That is the *exact* exponential integrator, and it reproduces the closed-form response at
   constant current, to machine precision.

   The state of charge advances by Coulomb counting:
   $\mathrm{SoC} \leftarrow \operatorname{clamp}\!\bigl(\mathrm{SoC} - I\,\Delta t/(3600\,Q_\text{Ah}),\ 0,\ 1\bigr)$.

   And the lumped pack temperature advances. It is heated by the irreversible term
   $I^2 R_0 + V_\mathrm{RC}^2/R_1$, plus the entropic term $I\,T\,\mathrm{d}U/\mathrm{d}T$, which
   can cool. It relaxes to the coolant through $R_\text{th}$, with a semi-implicit Euler step, which
   is A-stable, like everything else on the slow timescale.

The simplification of quasi-steady state is that the current is constant within one segment. The RC
state carries memory *across* segments.

A path driven by current, `step_current`, exists for validating the pulse response. It matches the
closed-form Thevenin response at well under 1 % RMS.

The executable specification of the battery lives in `crates/outlap-qss/tests/battery.rs`. It
checks: the pulse response against
$V(t) = \mathrm{OCV} - I R_0 - I R_1 (1 - e^{-t/\tau})$; a regeneration pulse lifting the terminal
voltage *above* OCV; SoC monotone under discharge; the discharge limit clipping to zero at the floor
of the SoC window; and a fully deterministic advance of the slow states, so that the same inputs
give bit-identical states. There are no hidden clocks, which matches the rules on determinism in
Chapter 6.

### 9.8 The Vdc–SoC coupling

Here is where the battery and the motor maps meet. It is the reason `ptm/1.1` exists.

A real machine fed by an inverter performs differently at different DC-link voltages. As the pack
drains, its terminal voltage drops, and both efficiency and losses shift.

The rule for the coupling is a recorded user decision, of 2026-07-05, documented in
`docs/theory/qss-powertrain.md` §8.4. It is a simple matrix of presence:

| Battery block | `.ptm` `vdc_v` axis | Behaviour |
|---|---|---|
| present | present (the `vdc_v` axis, a 1.x-era feature carried into `ptm/2.0`) | **Coupled**: the 3-D $(\text{speed}, \text{torque}, V_\text{dc})$ efficiency/loss maps are evaluated at the pack's live terminal voltage $V_\text{term}$ each segment |
| present | absent | Single-voltage: the map ignores the pack voltage |
| absent | present | Single-voltage at the map's reference voltage `meta.dc_voltage_v` (if that is missing or ≤ 0 on a Vdc-stacked map, the fallback reference is the *middle of the Vdc grid*, not 0 V) |
| absent | absent | Single-voltage (the pre-1.1 world) |

When the two are coupled, a point at low SoC shifts **both** the traction efficiency *and* the loss
that heats the machine, which is injected into the thermal network. One lookup therefore feeds two
areas of physics.

The interesting numerical case is deliberate, in the shipped data. The synthetic 220S pack swings
from about 634 V to 810 V open-circuit, over its SoC grid. The drive-unit maps are gridded from
730 V to 850 V. Under load at low SoC, the terminal voltage therefore sags *below* the map.

On the Vdc axis, and only there — speed and torque clamp — the shared monotone Hermite interpolant
uses **linear extrapolation outside the domain**, from the boundary slice. That is
`OutOfDomain::Linear`, and it is C¹-continuous with the interior. The map therefore stays usable,
instead of freezing at its edge.

Extrapolated values are held to physical bounds. Concretely, the efficiency is clamped to
$[10^{-3}, 1]$ inside the energy math.

And the fact that a unit's map is coupled to Vdc, with linear extrapolation, is recorded in the
assembly notes and the loaded-model report. A run that extrapolates is therefore never silent.

The decode contract puts the Vdc axis last, in tensor order; see
`T1Powertrain::map_axis_names_vdc`, which a debug assertion checks.

The property tests of Chapter 13 pin this down. A Vdc-stacked map, built from a field that is linear
in $V_\text{dc}$, is reproduced *exactly* under extrapolation, below and above the grid. The matrix
of presence behaves. And a draining pack drives a lower coupled efficiency.

To see the coupling with your own eyes, run the sweep over Model 3 sizing, in the capstone notebook.
Swapping the drive unit is a what-if override on one line,
`overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"}`, with no files edited. Chapter 14,
on recipes, walks through it.

### 9.9 Slow states in the lap loop

Everything in Chapter 8 was **fast**. The trim states — slips, loads, and forces — equilibrate
within a track station, and carry no memory.

A **slow state** is the opposite. It evolves over seconds to minutes, and it *remembers*. The node
temperatures of the machine, the pack's SoC, its RC overpotential, and its lumped temperature are
all slow states.

This section is where the two timescales meet.

#### 9.9.1 Building the stack

The coupling stack is `SlowCoupling`, in `crates/outlap-qss/src/qss.rs`. `build_slow_stack`, in
`crates/outlap-py/src/lib.rs`, assembles it at the native edge, **from the vehicle's own
references**. There are no extra arguments to the API.

It requires two things. First, a `battery:` block whose params and ECM sidecar both load. Second,
the *first* drive unit carrying a `thermal:` reference to an `.emotor` file.

A missing file leaves the coupling **inert, with a note**. A file that is present but broken is a
real error. Nothing is silent, in either direction.

Here are notes that you may see in the provenance of a lap result. Their exact source is
`build_slow_stack`:

```text
battery params `battery/f1_es.yaml` not present — slow-state coupling inert
battery present but no drive unit declares a `.emotor` thermal model — slow-state coupling inert
machine thermal: node `housing` capacity estimated from mass (11070 J/K)
2 drive units declare `.emotor` thermal models — the QSS coupling marches ONE network (unit 0); ...
```

The third line is the mass heuristic of §9.6.5 surfacing. Here it is
$0.15 \times 82\ \mathrm{kg} \times 900\ \mathrm{J/(kg\,K)}$, for a housing node left without a
`c_j_per_k` on a unit of 82 kg. The shipped `.emotor` file of the Model 3 declares all its
capacities explicitly, so it loads free of estimates on this front.

This release marches ONE thermal network. If several units declare `.emotor` models, the extras are
dropped, with a note. A stack with several machines arrives with the ERS energy manager.

#### 9.9.2 The outer march

The static g-g-g-v envelope stays **neutral in thermal and SoC terms**. It is generated once, at the
reference state, with a cold machine and full charge. Neither the derate nor the battery cap is
baked into it.

The coupling is instead resolved by a bounded, deterministic **outer march**, in `solve_profile`.

It solves the uncoupled velocity profile, to seed. Then, `OUTER_ITERS = 2` times, it derives the
longitudinal acceleration for each segment, marches the slow states along the profile to build a
**traction scale** at each station, and re-solves the profile with that scale, through
`solve_into_ggv_scaled`.

A final march, against the converged profile, makes the reported channels for SoC and temperature
match it.

The count is fixed, rather than driven by a tolerance, for determinism. "A single flying lap moves
the slow states little", so two iterations are ample.

The design is deliberately safe when there is no stack. In the words of the module doc of
`crates/outlap-qss/src/qss.rs`: "when no mapped stack is supplied the scale stays ≡ 1 and the result
is bit-identical to the uncoupled solve".

Adding a battery file to a vehicle can therefore never perturb an unrelated lap through numerics
alone. The tier-parity gates of Chapter 13 are unaffected.

#### 9.9.3 One segment of `march_slow_states`

For each segment $i$, with a segment time of $\Delta t = 2\,\Delta s/(v_i + v_j)$, and with the
thermal network and the pack reset to their assembled states at the start of every march, which is
deterministic and allocates nothing on the heap:

1. Log the **entry** state at station $i$. Station 0 reports the initial SoC and winding
   temperature.
2. Compute the wheel drive force actually demanded, taking the positive part only:
   $F_\text{drive} = \max\bigl(0,\ m(a_{x,i} + a_\text{drag}(v_i) + g\sin\theta_g)\bigr)$. Drag and
   grade are included, and braking is excluded.
3. Read the coupling voltage: $V_\text{dc} = $ `pack.terminal_voltage_v(state)`.
4. Look up `traction_energy(v_i, F_drive, Some(vdc))`. That gives the source power, the machine
   loss, and the shaft speed.
5. Step the thermal network: `derate = thermal.step(loss_w, |_| None, omega_rad_s, dt)`. An error in
   the thermal integrator leaves the derate at 1, meaning no cap. It is flagged, and not fatal.
6. Evaluate the battery cap **before** the step advances SoC. If $P_\text{source} > P_\text{cap}$,
   where $P_\text{cap}$ is the discharge limit that depends on SoC, then
   $s_\text{batt} = \operatorname{clamp}(P_\text{cap}/P_\text{source},\ 0,\ 1)$. Otherwise it is 1.
7. Call `pack.step_power(...)`, which advances SoC, $V_\mathrm{RC}$, and the pack temperature.
8. **Compose**: $\text{scale}[i] = \operatorname{clamp}\bigl(\min(\text{derate},\ s_\text{batt}),\ 0,\ 1\bigr)$.

Two details of the logging make the reported channels line up with intuition.

Station $i$ records the **entry** state, which is the state the car carries *into* segment $i$.
Station 0 therefore shows the initial SoC and temperature, and nothing leads the car by one segment.
On an open, point-to-point path, the final station carries the state at the end of the lap instead.

And every march starts from the assembled reference state. The whole coupled solve is therefore a
pure function of its inputs: run it twice, and you get bit-identical channels.

The two caps compose by `min`, because they are both ceilings on the same thing — the drive power
that can be delivered — and the binding one wins.

In the forward step of the profile solver, the scale multiplies *only* the traction ceiling of the
powertrain:

$$a_x = \min\!\Bigl(a_{x,\text{grip}},\ \frac{F_\text{pt,max}(v)\cdot \min(\text{derate},\, s_\text{batt})}{m} - a_\text{drag}(v)\Bigr) - g\sin\theta_g,$$

while braking is untouched, because "it draws no drive power"; see `GgvGrip.traction_scale`, in
`crates/outlap-qss/src/solver.rs`.

Grip, in other words, is never derated. Only the engine room is.

The result surfaces as two channels at each station, in the `slow` group of the lap result, at both
the `t0` and `t1` tiers; see `SlowLog`. They are `state_of_charge`, from 0 to 1, and
`machine_temp_c`, which is the winding node in °C, a unit at a display boundary.

They are attached only when the coupling actually did something: when SoC moved, when the winding
heated, or when any scale dipped below 1. Chapter 10 shows them as xarray variables.

#### 9.9.4 Honest limitations in this release

- **Regeneration does not recharge the pack.** SoC is a bound on discharge only, and it is monotone
  non-increasing over a lap. Phases of recovery arrive with the ERS energy manager; see the module
  doc of `crates/outlap-qss/src/qss.rs`.

  The `brakes.regen_blend.max_regen_frac` field of the vehicle, which is 0.6 on the Model 3, is a
  declaration about the *blend* of friction and regeneration braking. It is not recovery of SoC. The
  QSS lap loop does not consume it yet.
- **Named routing of loss by component is a hook.** The lap loop passes a `|_| None` resolver, so
  only fractions of the total loss heat the network this release.
- **Attribution on a hybrid.** The full traction draw is attributed to the mapped, electric units.
  That is exact for a pure EV, and conservative for a hybrid.
- **Battery temperature is advanced, but not logged** as a result channel. `SlowLog` carries only
  SoC and machine temperature. The pack temperature lives in `PackState.temp_k`, and in the
  `temp_c` of each `StepOut`.
- **One thermal network** for each lap, as noted above. And there is **no slow state for fuel mass**
  on an ICE car yet.
- The **asymmetry between T0 and T1 on ERS**, from §9.3, means that the two tiers see slightly
  different powertrain ceilings on a car with an ERS. That is by design, until the energy manager
  lands.

For where these limits sit on the roadmap, see Chapter 15, on limitations and the roadmap.

For the shipped vehicles that exercise this whole stack end to end — including the sweep over
Model 3 sizing, at small with 1365 N·m and about 100 kW, medium with 2765 N·m and about 203 kW, and
large with 3381 N·m and about 248 kW — see Chapter 12, on the shipped data library.


---

## 10. The Python API reference

*What you will learn: every public class and function in the Python package of outlap — what goes
in, what comes out, what errors it can raise, and a minimal runnable example for each. You will also
get the definitive contract for the xarray `Dataset` that every solved lap returns, the queryable
g-g-g-v envelope object, and the tooling for schema validation. This chapter is deliberately dry.
Read Chapters 3 and 4 first, if any example feels unmotivated.*

### 10.1 The import model, and the general conventions

The Python surface of outlap is split across a handful of modules. The one you will use almost all
the time is `outlap.core`.

| Import | Source | What it is |
|---|---|---|
| `outlap` | `python/src/outlap/__init__.py` | A placeholder. It exposes only `main()`, which prints a greeting. **Do not** expect the API here. |
| `outlap.core` | `python/src/outlap/core.py` | **The typed user API.** A thin veneer over the Rust bindings: numpy-style broadcasting for tire evaluation, and results as labelled `xarray.Dataset` objects. No physics lives here. |
| `outlap_core` | `crates/outlap-py/src/lib.rs` (compiled extension) | The raw Rust bindings (PyO3). `outlap.core` re-exports its classes; you rarely import it directly. Typed stubs ship in the wheel (`crates/outlap-py/outlap_core.pyi`). |
| `outlap.schemas` | `python/src/outlap/schemas.py` | Loads the committed JSON Schemas and validates fixtures/data against them (§10.8). |
| `outlap.tir` | `python/src/outlap/tir/` | The `.tir` interchange codec (§10.9, and Chapter 11, Importers and tooling). |
| `outlap.tirefit` | `python/src/outlap/tirefit/` | The MF6.1 tire-fitting pipeline (§10.9, Chapter 11). |
| `outlap.importers` | `python/src/outlap/importers/` | PDT HDF5 and track importers (§10.9, Chapter 11). |

Here is a common first stumble, verified against the shipped package:

```python
>>> import outlap
>>> [n for n in dir(outlap) if not n.startswith("_")]
['main']
>>> outlap.main()
Hello from outlap!
```

So always import from `outlap.core`:

```python
from outlap.core import Track, solve_lap_dataset
```

`outlap.core.__all__` is the complete public surface of the core API. It has fourteen names:

```text
DEFAULT_DS_M, Envelope, Lap, Raceline, Track, Tyre, TyreForces,
lap_dataset, min_curvature, solve_lap, solve_lap_dataset,
track_dataset, tyre_forces, vehicle_report
```

`DEFAULT_DS_M` is a module constant, at `2.0`. It is the default step of spatial sampling, in
meters. That is the distance between consecutive points of evaluation, called *stations*, along the
track.

`Envelope`, `Lap`, `Raceline`, `Track`, `Tyre`, `min_curvature`, `solve_lap`, and `vehicle_report`
are re-exported unchanged, from the Rust extension.

`TyreForces`, `tyre_forces`, `lap_dataset`, `solve_lap_dataset`, and `track_dataset` are the
additions on the Python side.

Seven conventions apply to *everything* below.

- **Units are SI**: meters, seconds, m/s, m/s², N, N·m, Pa, and rad.

  There are two deliberate exceptions, at a display boundary, in this API. One is the lap channel
  `machine_temp_c`, in °C. The other is the conditions fields ending in `_c` or `_hpa`, in °C and
  hPa. Both match the file formats that they mirror.
- **Axes are ISO 8855**: x forward, y left, z up.

  Lateral acceleration `ay` is positive to the *left*. A raceline offset `n` is positive toward the
  *left* road edge. Every sign below follows this convention.
- **Every extension class is immutable**, through `#[pyclass(frozen)]`. You cannot set an attribute
  on a `Lap`, a `Track`, a `Tyre`, a `Raceline`, or an `Envelope`.
- **A channel method returns a fresh copy.** Every call such as `lap.v()` allocates a new numpy
  array. Grab a channel once, or use `lap_dataset` or `solve_lap_dataset`, which do it for you.
- **There are two exception types.** A file that does not exist raises `FileNotFoundError`.

  Everything else — malformed YAML, an unknown field, a bad parameter — raises `ValueError`. Its
  message keeps the diagnostic help line, including a did-you-mean suggestion. §10.7 has a table for
  quick reference.
- **`solve_lap` holds the GIL**, which is the global interpreter lock of Python, for its whole
  duration. Solves that run on background threads will therefore not overlap. An API for batches and
  sweeps that releases it is planned; see Chapter 15.
- **Build the extension in release mode.** With a wheel built on the debug profile, the first lap of
  a vehicle takes about a minute, because of generating the envelope. In release it takes seconds.

  Set `MATURIN_PEP517_ARGS="--profile release"` before `uv sync`, exactly as CI does; see Chapter 3.

Every example below assumes the environment from Chapter 3, and paths relative to the root of the
repository. They use the shipped Tesla Model 3 vehicle, and the Catalunya track; see Chapter 12.

### 10.2 Loading, and reports

#### 10.2.1 `vehicle_report`: the loaded-model report

```python
def vehicle_report(
    vehicle_dir: str,
    overrides: dict[str, bool | int | float | str] | None = None,
) -> dict[str, object]
```

**In:** a path to a vehicle directory, which holds a `vehicle.yaml` plus whatever files it
references. Optionally, a `{dotted.path: value}` patch, applied through the real validation
pipeline.

**Out:** a plain dict. It is the *loaded-model report*: the "nothing silent" account by outlap of
everything the assembly pipeline filled in, estimated, or degraded, while resolving the vehicle. See
Chapter 4.

| Key | Type | Meaning |
|---|---|---|
| `name` | `str` | The vehicle's display name. |
| `resolved_hash` | `str` | blake3 hash (hex) of the fully resolved vehicle spec. Changes whenever any effective parameter changes. |
| `inherited` | `list[tuple[str, str]]` | `(json_pointer, detail)` pairs for values inherited via `extends:`. |
| `estimated` | `list[tuple[str, str]]` | Values the pipeline estimated because the file omitted them. |
| `degraded` | `list[tuple[str, str]]` | Documented-fallback combinations (only reachable with `allow_degraded`). |
| `warnings` | `list[tuple[str, str]]` | Non-fatal load warnings. |
| `overrides` | `list[tuple[str, str]]` | Echo of the applied override paths and their values (stringified). |

**Errors:** `FileNotFoundError`, if `vehicle.yaml` or a file it references is missing. `ValueError`,
for a malformed file, or for an invalid override path or value.

```python
from outlap.core import vehicle_report

rep = vehicle_report("data/vehicles/tesla_model3_rwd")
print(rep["name"], rep["resolved_hash"][:12])
print(len(rep["estimated"]), "estimated;", rep["estimated"][0])
```

```text
Tesla Model 3 RWD (HV variant) 76c65d2ac0a2
10 estimated; ('/suspension/front/static_ride_height_m', 'assumed 30 mm nominal (only used by the ride-height aero map)')
```

With an override, the report echoes it, and the hash changes:

```python
rep = vehicle_report("data/vehicles/tesla_model3_rwd",
                     overrides={"chassis.mass_kg": 1500.0})
print(rep["overrides"], rep["resolved_hash"][:12])
```

```text
[('chassis.mass_kg', '1500.0')] d62292c121a0
```

Get into the habit of checking this report before you trust a lap time. The `estimated` and
`degraded` entries tell you where the model is running on assumptions, rather than on your data.

#### 10.2.2 `Track` and `track_dataset`

```python
class Track:
    @staticmethod
    def load(dir: str) -> Track
    def name(self) -> str
    def length(self) -> float          # total arc length, m
    def is_closed(self) -> bool        # closed loop?
    def sample(self, ds_m: float) -> dict[str, NDArray[np.float64]]
```

**In:** `Track.load` takes a *directory*, which holds a `track.yaml` plus its centerline CSV; see
Chapter 5.

**Out:** an immutable, queryable 3-D track ribbon.

`sample(ds_m)` resamples it at a uniform step of `ds_m` meters. It returns a dict of ten arrays of
equal length: `s`, which is arc length in m; `x`, `y`, and `z`, which are world position in m;
`kappa_h`, which is plan-view curvature in 1/m, and says how tightly the road turns; `kappa_v`,
which is vertical curvature in 1/m, and covers crests and compressions; `grade`, which is the
uphill slope in rad; `banking`, which is the lateral tilt of the road in rad; and `width_left` and
`width_right`, which are the distances from the centerline to each edge, in m.

**Errors:** `FileNotFoundError`, for a missing directory or `track.yaml`. `ValueError`, for a
malformed track, or for a `ds_m` that is not positive and finite.

```python
from outlap.core import Track

track = Track.load("data/tracks/catalunya")
print(track.name(), round(track.length(), 1), track.is_closed())
m = track.sample(50.0)
print(sorted(m.keys()), len(m["s"]))
```

```text
Circuit de Barcelona-Catalunya 4649.8 True
['banking', 'grade', 'kappa_h', 'kappa_v', 's', 'width_left', 'width_right', 'x', 'y', 'z'] 94
```

`track_dataset` wraps the same sampling in a labelled dataset:

```python
def track_dataset(track: Track, ds_m: float = 10.0) -> xr.Dataset
```

Note that the default step here is **10.0 m**, and not `DEFAULT_DS_M`. A plot of a track rarely
needs a resolution of 2 m.

The dataset has one dimension, `s`, whose coordinate is in m. It has the nine data variables above.
Each is annotated with `units`; for example, `kappa_h` is in `1/m`, with the long name "plan-view
curvature".

Its attrs are `name`, a str; `length_m`, a float; and `closed`, an `int`, because a netCDF attribute
has no boolean type.

Here is real output, trimmed:

```text
<xarray.Dataset> Size: 37kB
Dimensions:      (s: 466)
Coordinates:
  * s            (s) float64 4kB 0.0 10.0 20.0 ... 4.63e+03 4.64e+03 4.65e+03
Data variables:
    x            (s) float64 4kB -0.4732 -5.893 -11.31 ... 10.28 4.863 -0.4732
    ...
    width_right  (s) float64 4kB 5.894 5.884 5.875 5.865 ... 5.913 5.903 5.894
Attributes:
    name:      Circuit de Barcelona-Catalunya
    length_m:  4649.84361935622
    closed:    1
```

#### 10.2.3 `Tyre`, `tyre_forces`, and `TyreForces`

```python
class Tyre:
    @staticmethod
    def load(path: str) -> Tyre
    # attributes
    notes: list[tuple[str, str]]   # (json_pointer, detail) load/extraction notes
    citation: str                  # literature citation from the file's provenance block
    fnomin: float                  # nominal load FNOMIN, N
    unloaded_radius: float         # R0, m
    p_cold: float                  # cold inflation pressure, Pa
    def forces(self, kappa, alpha, gamma, fz, p, vx) -> tuple[fx, fy, mz, mx, my]
    def peak_mu(self, fz: float, p: float) -> tuple[float, float]
```

**In:** `Tyre.load` takes the path to a single `.tyr.yaml` *file*, and not a directory. It builds the
evaluatable steady-state tire model of Magic Formula 6.1, which is the empirical force model of
Pacejka; see Chapter 7.

Note the conversion of pressure at the boundary: the file stores kPa, and the attribute is Pa.

`forces` is the raw binding. Six **1-D float64 arrays of equal length** go in: `kappa`, the slip
ratio, which is the fractional difference in speed between tread and road; `alpha`, the slip angle
in rad, which is the angle between where the wheel points and where it travels; `gamma`, the camber
or inclination angle in rad; `fz`, the vertical load in N; `p`, the inflation pressure in Pa; and
`vx`, the forward speed in m/s.

Five arrays come out: the longitudinal force `fx` in N; the lateral force `fy` in N; and the
aligning moment `mz`, the overturning moment `mx`, and the rolling-resistance moment `my`, all in
N·m. All follow ISO 8855 signs.

A mismatch in length raises `ValueError`, with
`"length mismatch: kappa has 5 elements, alpha has 3"`.

`peak_mu(fz, p)` returns the peak friction coefficients, `(μx, μy)`. Those are the maximum ratio of
force to load that the pure-slip curves reach, at that load and pressure.

For everyday use, prefer the wrapper that broadcasts:

```python
def tyre_forces(
    tyre: Tyre, *,
    kappa: ArrayLike = 0.0, alpha: ArrayLike = 0.0, gamma: ArrayLike = 0.0,
    fz: ArrayLike | None = None,   # default: tyre.fnomin
    p: ArrayLike | None = None,    # default: tyre.p_cold
    vx: ArrayLike = 16.7,          # m/s (~60 km/h)
) -> TyreForces
```

**In:** scalars or arrays, in any combination that numpy can broadcast. Every argument is
keyword-only.

**Out:** a `TyreForces` named tuple, with the fields `fx, fy, mz, mx, my`. Each is an
`NDArray[np.float64]`, shaped like the broadcast of the inputs.

```python
import numpy as np
from outlap.core import Tyre, tyre_forces

tyre = Tyre.load("data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml")
print(tyre.fnomin, tyre.unloaded_radius, tyre.p_cold)
print(tyre.peak_mu(4000.0, tyre.p_cold))

out = tyre_forces(tyre, alpha=np.linspace(-0.15, 0.15, 5))   # slip-angle sweep
print(np.round(out.fy, 1))

grid = tyre_forces(tyre,
                   kappa=np.linspace(-0.1, 0.1, 3).reshape(3, 1),
                   alpha=np.linspace(-0.15, 0.15, 5).reshape(1, 5))
print(grid.fx.shape)
```

```text
4000.0 0.313 220000.0
(1.21, 1.035)
[ 3986.4  3005.     42.  -2825.3 -3671. ]
(3, 5)
```

The `citation` attribute of the shipped road tire reads:
`H. B. Pacejka, Tyre and Vehicle Dynamics, 2nd ed. (2006), Appendix 3, Table A3.1 (205/60R15 91V,
2.2 bar, ISO sign)`. Provenance travels with the data.

Its `notes` list two facts about extraction. One is
`('/mf61/QSX*', 'overturning-moment coefficients absent - Mx = 0')`. This parameter set therefore
produces `mx = 0`, and the API tells you so, rather than staying silent.

### 10.3 Racelines: `min_curvature`, `time_weighted`, and `Raceline`

```python
def min_curvature(
    track: Track,
    half_width_m: float,
    ds_m: float = 2.0,
    margin_m: float = 0.3,
    epsilon: float = 1e-8,
) -> Raceline
```

**In:** a loaded `Track`; the half-width of the car, in meters; and three numeric knobs. `ds_m` is
the sampling step of the quadratic program, or QP, which is the optimization that picks the line.
`margin_m` is an extra safety margin kept from the track edges. And `epsilon` is the Tikhonov
regularization, a tiny smoothing term that keeps the QP well conditioned.

**Out:** a `Raceline`. It is the minimum-curvature racing line: the path inside the track corridor
that minimizes the integral of squared curvature; see Chapter 8.

**Errors:** `ValueError`, if `ds_m` or `half_width_m` is not positive and finite, or if the QP
fails.

```python
def time_weighted(
    vehicle_dir: str,
    track: Track,
    half_width_m: float,
    ds_m: float = 2.0,
    iterations: int = 3,
    margin_m: float = 0.3,
    epsilon: float = 1e-8,
    tol: float = 1e-3,
    overrides: dict | None = None,
    conditions: dict | None = None,
    sim: dict | None = None,
) -> Raceline
```

This is the **time-weighted** refinement of Chapter 8. It re-solves the same QP, with the squared
curvature at each station weighted by the time that the car spends there. The weight is
`w ∝ 1/v`, from a T0 speed pre-pass on the current line.

It runs an outer loop, which keeps the fastest line, and stops when the modeled lap time stops
improving, at a relative tolerance of `tol`, or after `iterations` passes.

Unlike `min_curvature`, it needs the *car*. The pre-pass runs the car's own g-g-g-v envelope. It
therefore takes a `vehicle_dir`, and it honors `sim`, `overrides`, and `conditions`, exactly as the
solvers do. The envelope is built once, and reused across the iterations.

The result is faster than, or equal to, the minimum-curvature line, by construction.

**Errors:** the same as `min_curvature`, plus everything that loading a vehicle can raise, plus a
`ValueError` for `iterations` outside 1 to 16.

```python
class Raceline:                       # produced by the generators; not user-constructible
    ds_m: float                       # the step the line was GENERATED with, m
    generator: str                    # "min_curvature" | "time_weighted"
    iterations: int                   # outer iterations actually run (1 for min_curvature)
    def s(self) -> NDArray            # parent-centerline stations, m
    def n(self) -> NDArray            # signed lateral offsets (+ = road-left), m
    def line(self) -> Track           # the racing line as a first-class Track
```

`line()` matters. The racing line comes back as a real `Track`, with its own curvature and length.
The solver can therefore drive it exactly as it drives a centerline.

`ds_m`, `generator`, and `iterations` are recorded, so that the lap result carries honest provenance
about how the line was generated. A `time_weighted` line reports the *real* converged count of
iterations, and never a placeholder.

```python
from outlap.core import Track, min_curvature

track = Track.load("data/tracks/catalunya")
rl = min_curvature(track, half_width_m=0.95)
print(rl.ds_m, rl.line().name(), round(rl.line().length(), 1))
print(round(float(rl.n().min()), 2), round(float(rl.n().max()), 2))
```

```text
2.0 min-curvature line 4681.2
-7.27 5.63
```

The generated line is 31 m *longer* than the 4649.8 m centerline. A straighter path through a corner
trades distance for speed. It swings up to 7.27 m right of center, and 5.63 m left.

On this track it is worth about 8.4 s to the Model 3; see §10.4.3.

### 10.4 Solving laps

#### 10.4.1 `solve_lap`

```python
def solve_lap(
    vehicle_dir: str,
    track: Track,
    ds_m: float = 2.0,                 # DEFAULT_DS_M
    raceline_ds_m: float | None = None,
    overrides: dict[str, bool | int | float | str] | None = None,
    conditions: dict[str, object] | None = None,
    tier: str | None = None,
    sim: dict[str, object] | None = None,
) -> Lap
```

This is the QSS, or quasi-steady-state, lap solver. It computes the fastest speed profile that the
car can sustain along the given line; see Chapter 8.

**What enters.**

From disk: `vehicle_dir` must hold a `vehicle.yaml`, plus every file that it references. Those are
`.tyr` tires, `.ptm` powertrain maps, optional `.emotor` thermal files and battery files, and
binary sidecar tables such as `.parquet` maps for aero and efficiency.

An optional `sim.yaml` and `conditions.yaml`, next to it, override the built-in defaults.

The rule throughout: a *missing* optional file falls back to a default, or to a documented fallback,
**with a note in the result**. A file that is *present but malformed* is always an error, and never
silently ignored.

From memory: the `Track` to drive, and four optional structures of overrides, described below.

**What leaves.** An immutable `Lap` object (§10.4.2). It holds the solved channels, as copied
arrays; the attached g-g-g-v envelope; and the provenance, which is the tier, the coupling mode, the
resolved hash, and the notes.

Here is what each parameter means.

- `ds_m` is the station spacing of the solve, in m. Smaller means finer resolution, and more
  stations. It must be positive and finite.
- `tier` is `"t0"`, which is a point mass on the g-g-g-v envelope, or `"t1"`, which is the full QSS
  with per-wheel outputs.

  It overrides everything else, including `sim["tier"]` and the `sim.yaml` in the vehicle directory.
  The default, from `schemas/sim.json`, is **`"t1"`**.

  `"t2"` raises a `ValueError` that redirects you to `solve_transient_lap` (§10.4.5). The transient
  lap is indexed by time, and it has its own entry point; `solve_lap_dataset(..., tier="t2")`
  handles the redirect for you. `"t3"` raises a `ValueError`, because it is not implemented.
- `sim` is a nested dict, deep-merged onto `sim.yaml` or onto the defaults. An example is
  `{"flat_track": True, "envelope": {"v_points": 24}}`. An unknown key is rejected loudly (§10.4.4).
- `conditions` is a nested dict, deep-merged onto `conditions.yaml` or onto the defaults, which are
  ISA-like: 20 °C and 1013.25 hPa. It is equally strict.
- `overrides` is a flat `{dotted.path: value}` patch on the vehicle, such as
  `{"chassis.mass_kg": 1500.0}`. It is applied through the full validation pipeline: checked against
  the schema after the merge, and recorded in the provenance. A value may be a `bool`, an `int`, a
  `float`, or a `str`.
- `raceline_ds_m`, plus `raceline_generator` and `raceline_iterations`, are for provenance only.
  When you hand-solve a generated racing line, pass the step it was generated with, and the kind of
  generator and the count of iterations, so that the result records them.

  `solve_lap_dataset` does this automatically for a `Raceline` input, so you rarely touch them.

Precedence, in one line: the `tier=` argument beats the `sim=` dict, which beats a `sim.yaml` in the
vehicle directory, which beats the built-in defaults. Conditions follow the same order of file and
then dict.

**Errors:** `FileNotFoundError`, for a missing `vehicle.yaml` or a missing referenced file.
`ValueError`, for a malformed file; for an unknown key in an override, in `sim`, or in
`conditions`, with a did-you-mean help line; for a bad `ds_m`; for tier `t2` or `t3`; or for an
undecodable sidecar. See §10.7.

```python
from outlap.core import Track, solve_lap

track = Track.load("data/tracks/catalunya")
lap = solve_lap("data/vehicles/tesla_model3_rwd", track)
print(round(lap.lap_time_s, 3), lap.tier, lap.fz_coupling)
```

```text
148.081 t1 one_step_lag
```

What to expect on timing. The first solve of a car generates its g-g-g-v envelope. That takes
seconds in a release build; about 65 s was measured on a debug build.

A later solve of the same car, at the same conditions and grid, in the same process, reuses a cached
envelope, and is fast. The warm `t0` solve above took 0.12 s. §10.6 explains the key of the cache.

#### 10.4.2 The `Lap` object

A solved lap. Its attributes hold plain values:

| Attribute | Type | Meaning |
|---|---|---|
| `lap_time_s` | `float` | Total lap time, s. |
| `tier` | `str` | Resolved solver tier: `"t0"` or `"t1"`. |
| `fz_coupling` | `str` | Recorded normal-load coupling mode: `"one_step_lag"` or `"fixed_point"` (Chapter 8). |
| `flat_track` | `bool` | Whether the lap ran in flat-track analysis mode. |
| `wheels` | `list[str]` | Per-wheel channel order — always `["FL", "FR", "RL", "RR"]`. |
| `notes` | `list[str]` | Simplification/degradation notes — nothing silent. |
| `resolved_hash` | `str` | blake3 hash of the resolved vehicle spec that produced this lap. |
| `envelope` | `Envelope \| None` | The queryable g-g-g-v envelope the lap ran on (§10.6), or `None` on the degenerate path. |

Its channel methods each return a *fresh copy*:

| Method | Shape | Units | Available |
|---|---|---|---|
| `s()` | `(n,)` | m | always |
| `v()` | `(n,)` | m/s | always |
| `ax()` | `(n,)` | m/s² | always |
| `ay()` | `(n,)` | m/s² (+left) | always |
| `t()` | `(n,)` | s (cumulative) | always |
| `x()`, `y()`, `z()` | `(n,)` | m (world position; `z` = elevation) | always |
| `vertical_load_n()` | `(n, 4)` | N | t1 only (`None` at t0) |
| `slip_ratio()` | `(n, 4)` | 1 | t1 only |
| `slip_angle_rad()` | `(n, 4)` | rad | t1 only |
| `force_long_n()` | `(n, 4)` | N | t1 only |
| `force_lat_n()` | `(n, 4)` | N | t1 only |
| `understeer_gradient()` | `(n,)` | rad·s²/m | t1 only |
| `aero_front_share()` | `(n,)` | 1 (0..1) | t1 only |
| `state_of_charge()` | `(n,)` | 1 (0..1) | when a coupled electrified stack was active (any tier) |
| `machine_temp_c()` | `(n,)` | °C | when a coupled electrified stack was active (any tier) |

A per-wheel array is `n × 4`, in the column order `FL/FR/RL/RR`; see `lap.wheels`.

Four of these need a definition. The *understeer gradient* is a metric of handling: how much extra
steering the car needs, for each unit of lateral acceleration, where positive means understeer. The
*aero front share* is the fraction of total downforce on the front axle. The *state of charge* is
the fraction of energy remaining in the battery. And the *machine temperature* is the winding
temperature of the drive motor; see Chapter 9.

```python
lap = solve_lap("data/vehicles/tesla_model3_rwd", track)   # t1 default
print(lap.wheels, lap.vertical_load_n().shape)

lap0 = solve_lap("data/vehicles/tesla_model3_rwd", track, tier="t0")
print(lap0.vertical_load_n() is None, lap0.state_of_charge() is None)
```

```text
['FL', 'FR', 'RL', 'RR'] (2325, 4)
True False
```

Note the second line. At `t0` the per-wheel channels are `None`. But `state_of_charge()` is *not*.
The channels for slow states gate on whether the vehicle has a complete stack of battery and motor
thermal state. They do not gate on the tier.

#### 10.4.3 `lap_dataset` and `solve_lap_dataset`

```python
def lap_dataset(lap: Lap) -> xr.Dataset

def solve_lap_dataset(
    vehicle_dir: str,
    line: Track | Raceline, *,
    ds_m: float = DEFAULT_DS_M,
    tier: str | None = None,
    sim: dict[str, object] | None = None,
    overrides: dict[str, bool | int | float | str] | None = None,
    conditions: dict[str, object] | None = None,
) -> xr.Dataset
```

`lap_dataset` converts a `Lap` into the labelled `xarray.Dataset` that is the designed results
boundary of outlap; see §10.5.

`solve_lap_dataset` is `solve_lap` plus `lap_dataset`, in one call. It adds two conveniences.

`line` may be a `Raceline`. It then solves on `line.line()`, and passes the provenance of the
raceline — `ds_m`, the generator, and the iterations — automatically.

And `tier="t2"` dispatches to `solve_transient_lap` (§10.4.5), and returns the **time-indexed**
transient dataset instead.

Every option after `line` is keyword-only. Its errors are exactly those of the underlying solver.

```python
from outlap.core import Track, min_curvature, solve_lap_dataset

track = Track.load("data/tracks/catalunya")
veh = "data/vehicles/tesla_model3_rwd"

ds = solve_lap_dataset(veh, track)                     # centerline lap
rl = min_curvature(track, half_width_m=0.95)
ds_rl = solve_lap_dataset(veh, rl)                     # racing-line lap
print(round(ds.attrs["lap_time_s"], 3), round(ds_rl.attrs["lap_time_s"], 3))
```

```text
148.081 139.638
```

A what-if experiment composes through the same call. Every change flows through the real validation
pipeline, and it is reflected in `resolved_hash`:

```python
ds_light = solve_lap_dataset(veh, track, overrides={"chassis.mass_kg": 1500.0})
ds_hot = solve_lap_dataset(veh, track,
                           conditions={"air": {"temperature_c": 35.0},
                                       "ambient_c": 35.0})
ds_flat = solve_lap_dataset(veh, track,
                            sim={"flat_track": True,
                                 "fz_coupling": "fixed_point"})
print(round(ds_light.attrs["lap_time_s"], 3))   # 1765 kg -> 1500 kg
print(round(ds_hot.attrs["lap_time_s"], 3),
      round(float(ds_hot["machine_temp_c"][-1]), 1))
print(ds_flat.attrs["fz_coupling"], ds_flat.attrs["flat_track"])
```

```text
144.745
148.151 156.0
fixed_point 1
```

Losing 265 kg is worth 3.3 s here. A day at 35 °C costs 0.07 s, and leaves the motor windings 19 K
hotter at the flag.

One caution. The field for air temperature is `temperature_c`. An older docstring example in
`outlap.core` shows `temp_c`, which the strict merge correctly rejects, with
`unknown conditions field 'air.temp_c'`.

#### 10.4.4 The vocabulary of `sim=` and `conditions=`

Both dicts are deep-merged onto the corresponding file, or onto the defaults, and then re-validated.
Any key that does not exist in the schema is an error, and the message lists the known fields at
that level.

The vocabulary comes from `schemas/sim.json` and `schemas/conditions.json`; see Chapter 5.

Here are the `sim=` fields, and their defaults:

| Field | Default | Meaning |
|---|---|---|
| `tier` | `"t1"` | Solver tier (`t0`/`t1`/`t2`/`t3`; `t2` is time-indexed and routes through its own entry point, `t3` raises). |
| `envelope` | `{"v_points": 40, "ax_points": 25, "g_normal_points": 7}` | g-g-g-v grid resolution (Chapter 8). |
| `fz_coupling` | unset (tier-resolved) | Normal-load algebraic-loop mode: `"one_step_lag"` or `"fixed_point"`. Unset = automatic (`one_step_lag` for T0/T1, `fixed_point` for T2); the resolved value is recorded in the result. |
| `flat_track` | `false` | Zero grade/banking/vertical curvature so the envelope collapses to a flat g-g (oracle-comparison mode). Recorded; the track file is untouched. |
| `allow_degraded` | `false` | Permit documented-fallback combinations; the result is marked. |
| `dt_s` | `0.001` | Fixed integration step, s — the T2 tier's timestep. |
| `integrator` | `"heun"` | Fixed-step integrator (`heun`/`rk4`) for the T2 tier. |
| `slow_decimation` | `20` | T2 slow-state clock: advance SoC/temperatures every N fast steps. |
| `fixed_point` | `{damping: 1.0, tolerance: 1e-6, max_iter: 3}`-style knobs | Iteration controls for the `fixed_point` coupling mode. |
| `raceline` | `{"generator": "min_curvature"}` | Racing-line source: `"min_curvature"`, `{"time_weighted": {"iterations": 3}}`, or `{"file": "raceline.csv"}` (exactly one of generator/file). |
| `schema` | — | Schema version string, e.g. `"sim/1.0"`. |

And here are the `conditions=` fields, and their defaults:

| Field | Default | Meaning |
|---|---|---|
| `air.pressure_hpa` | `1013.25` | Absolute air pressure, hPa (drives air density). |
| `air.temperature_c` | `20.0` | Air temperature, °C. |
| `ambient_c` | `20.0` | Thermal-model ambient / pre-radiator coolant proxy, °C. |
| `track_surface_c` | `20.0` | Track-surface temperature (tire thermal boundary), °C. |
| `wind.speed_mps` | `0.0` | Wind speed, m/s (constant in v1). |
| `wind.direction_deg` | `0.0` | Meteorological direction the wind blows *from*, degrees (0 = North, 90 = East). |
| `schema` | — | e.g. `"conditions/1.0"`. |

#### 10.4.5 `solve_transient_lap` and `transient_lap_dataset`

```python
def solve_transient_lap(
    vehicle_dir: str,
    track: Track,
    ds_m: float = DEFAULT_DS_M,
    raceline_ds_m: float | None = None,
    raceline_generator: str | None = None,
    raceline_iterations: int | None = None,
    overrides: dict[str, bool | int | float | str] | None = None,
    conditions: dict[str, object] | None = None,
    sim: dict[str, object] | None = None,
    speed_margin: float = 0.85,
    initial_soc: float | None = None,
) -> TransientLap

def transient_lap_dataset(lap: TransientLap) -> xr.Dataset
```

This is the entry point for the **T2 transient** tier; see §8.7.

It assembles the same QSS artifacts first: the g-g-g-v envelope, whose cache it shares with the QSS
solvers; the T0 speed profile; and the target line.

It then runs the closed-loop integration in time. The ideal driver tracks the **corner-scaled**
reference of §8.7.2: the full T0 profile where lateral demand is low, and `speed_margin` times it at
the lateral grip limit. The default is 0.85, and the resolved value is recorded in the result.

The control layer runs the rules for shifting, torque vectoring, and regeneration. And the split
integrator advances the 7-DOF chassis, at `sim.dt_s`.

`initial_soc` optionally seeds the state of charge of the battery.

`sim`, `conditions`, and `overrides` behave exactly as they do in `solve_lap`. `sim.flat_track`
selects the flat analysis mode, against the full 3D road frame.

The lap is seeded at the straightest station of the line, because a cold transient dropped into a
corner is unphysical. It then runs one full lap of arc length, wrapping past the start and finish.

`transient_lap_dataset` converts the result into the time-indexed dataset of §10.5. In practice you
call `solve_lap_dataset(vehicle, line, tier="t2", ...)`, and get that dataset in one step:

```python
from outlap.core import Track, min_curvature, solve_lap_dataset

track = Track.load("data/tracks/catalunya_osm")
rl = min_curvature(track, half_width_m=1.1)
t2 = solve_lap_dataset("data/vehicles/limebeer_2014_f1", rl, tier="t2",
                       sim={"flat_track": True})
print(round(t2.attrs["lap_time_s"], 2), t2.sizes["time"])
```

A lap of about 108 s, at 1 ms, is about 108 000 samples, over the dims `(time, wheel)`. That is a
full trace from a data logger; notebooks 08 and 09 plot them.

**Errors:** everything that `solve_lap` can raise, plus a `ValueError` for a `speed_margin` outside
`(0, 1]`.

A lap that diverges, meaning that the driver spins, comes back *truncated and flagged*, with
`attrs["completed"] == 0`. It never comes back as a silent crash.

### 10.5 The contract of the xarray Dataset

Every solved lap crosses the Python boundary as an `xarray.Dataset`. It has labelled dimensions,
coordinates, units for each variable, and attributes for provenance.

This is a contract in the style of semver. An addition is backward-compatible. Code written against
an `s`-only `t0` dataset therefore keeps working, because the richer channels are strictly additive,
and appear only when the solve produced them.

There are two shapes of dataset, one for each family of solver.

**A QSS lap, at t0 or t1, is indexed by arc length.** The tables below describe it.

**A transient lap, at t2, is indexed by time.** Its primary dim is `time`, which is the fixed grid
of `dt`, in s, with the same `wheel` dim.

Its data variables are: the chassis states, `s`, `n`, `psi_rel`, `vx`, `vy`, `yaw_rate`, `ax`, and
`ay`; the driver's `steer`, `throttle`, and `brake`; the world trajectory `x`, `y`, and `z`, taken
from the *integrated* path; the control telemetry `gear`, `torque_scale`, `yaw_moment_nm`,
`regen_power_w`, `traction_power_w`, and the regen torques for each axle; the per-wheel `omega`,
`vertical_load_n`, `slip_ratio`, `slip_angle_rad`, `force_long_n`, and `force_lat_n`; and, with a
battery, `state_of_charge` and `pack_temp_c`.

Its attrs add `dt_s`, `integrator_order`, `speed_margin`, and `completed`, to the usual provenance.

Note that `s` in a transient lap is a *data variable*. It advances, and it wraps, along the drive.

**Dimensions and coordinates:**

| Coord | dtype | Attrs | Present |
|---|---|---|---|
| `s` | `float64` | `units: "m"`, `long_name: "arc length"` | always |
| `wheel` | `<U2`, values `FL FR RL RR` | `long_name: "wheel (FL, FR, RL, RR)"` | only when per-wheel variables exist (t1) |

**Data variables.** This is the definitive table: name, dims, units, long name, and which solves
produce it.

| Variable | Dims | Units | Long name | Produced by |
|---|---|---|---|---|
| `v` | `s` | m/s | speed | t0 and t1 |
| `ax` | `s` | m/s² | longitudinal acceleration | t0 and t1 |
| `ay` | `s` | m/s² | lateral acceleration (+left) | t0 and t1 |
| `t` | `s` | s | cumulative time | t0 and t1 |
| `x`, `y` | `s` | m | — (world position) | t0 and t1 |
| `z` | `s` | m | elevation | t0 and t1 |
| `vertical_load_n` | `(s, wheel)` | N | normal load | t1 only |
| `slip_ratio` | `(s, wheel)` | 1 | longitudinal slip ratio κ | t1 only |
| `slip_angle_rad` | `(s, wheel)` | rad | slip angle α | t1 only |
| `force_long_n` | `(s, wheel)` | N | longitudinal tyre force Fx | t1 only |
| `force_lat_n` | `(s, wheel)` | N | lateral tyre force Fy | t1 only |
| `understeer_gradient` | `s` | rad·s²/m | understeer gradient K | t1 only |
| `aero_front_share` | `s` | 1 | front axle downforce share | t1 only |
| `state_of_charge` | `s` | 1 | pack state of charge | any tier, when the vehicle's battery + machine-thermal stack is complete |
| `machine_temp_c` | `s` | °C | machine winding temperature | any tier, when the stack is complete |

So a `t0` lap of a car *without* a coupled electrified stack is genuinely `s`-only, with 7
variables. A `t0` lap of the shipped Model 3 additionally carries the two variables for slow states,
giving 9 variables, and still no `wheel` dimension. And a `t1` lap of the Model 3 carries all 16.

**Attributes.** Here are all of them:

| Attr | Type | Meaning |
|---|---|---|
| `lap_time_s` | `float` | Total lap time, s. |
| `resolved_hash` | `str` | blake3 hex hash of the resolved vehicle spec — your reproducibility key. |
| `tier` | `str` | `"t0"` or `"t1"` — the tier that actually ran. |
| `fz_coupling` | `str` | `"one_step_lag"` or `"fixed_point"`. |
| `flat_track` | `int` | 0/1 (an int because netCDF attrs have no bool type). |
| `notes` | `tuple[str, ...]` | Every simplification/fallback that touched this lap (a tuple, not a list, to stay netCDF-serializable). |

Here is real output for the default t1 lap of the Model 3 at Catalunya, at `ds_m=2.0`, which gives
2325 stations. It is trimmed:

```text
<xarray.Dataset> Size: 595kB
Dimensions:              (s: 2325, wheel: 4)
Coordinates:
  * s                    (s) float64 19kB 0.0 2.0 4.0 ... 4.646e+03 4.648e+03
  * wheel                (wheel) <U2 32B 'FL' 'FR' 'RL' 'RR'
Data variables: (12/16)
    v                    (s) float64 19kB 46.8 46.89 46.97 ... 46.53 46.62 46.71
    ax                   (s) float64 19kB 2.067 2.061 2.055 ... 2.086 2.08 2.074
    ay                   (s) float64 19kB 0.0205 0.02029 ... 0.02219 0.02136
    t                    (s) float64 19kB 0.0 0.0427 0.08531 ... 148.0 148.0
    ...                   ...
    state_of_charge      (s) float64 19kB 0.98 0.98 0.9799 ... 0.8987 0.8986
    machine_temp_c       (s) float64 19kB 20.0 20.07 20.16 ... 136.8 136.9 136.9
Attributes:
    lap_time_s:     148.08120662615633
    resolved_hash:  76c65d2ac0a28cf41fed5ab4a084aa4e24f8f287f1d29af4c05ce4c1d...
    tier:           t1
    fz_coupling:    one_step_lag
    flat_track:     0
    notes:          ('aero map `aero/none.parquet` not present — constant-aer...
```

Always read `attrs["notes"]`.

That lap carries 11 entries. Among them are
`aero map 'aero/none.parquet' not present — constant-aero fallback carries the lap`, because this
vehicle ships no aero map over ride height, so constant coefficients were used. And
`μ derived from MF6.1 pure-slip peak @ FNOMIN, p_cold ...; braking is friction-limited only at T0`.

The policy is *nothing silent*. Every estimate, fallback, and simplification that shaped the number
in `lap_time_s` is listed right next to it.

The result is a standard xarray Dataset. The whole scientific-Python toolchain therefore applies
directly: `ds.v.plot()`, `ds.sel(wheel="FL")`, `ds.to_netcdf("lap.nc")`, `ds.where(ds.ay > 5)`, and
so on.

### 10.6 The envelope object

`lap.envelope` returns the g-g-g-v envelope that the lap ran on. That is the precomputed boundary of
the accelerations that the tires can sustain, as a function of speed `v`, longitudinal acceleration
`a_x`, and the local "normal g", `g_normal`. `g_normal` says how hard the road pushes on the car:
more than $g$ in a banked or compressive section, and less over a crest.

Chapter 8 develops the theory. Here is the query API:

```python
class Envelope:                       # from lap.envelope; scalar queries interpolate the grid
    notes: list[str]                  # generation notes (nothing silent)
    def ay_boundary(self, v, ax, g_normal) -> float   # lateral-accel boundary, m/s²
    def accel_limit(self, v, g_normal) -> float       # max positive ax, net of drag, m/s²
    def brake_limit(self, v, g_normal) -> float       # max braking magnitude, m/s²
    def drag_accel(self, v) -> float                  # straight-line drag as an accel, m/s²
    def domain(self) -> list[list[float]]             # [lo, hi] per axis (v, â_x, g_normal)
    def shape(self) -> list[int]                      # [n_v, n_âx, n_g_normal]
    def mass_ref(self) -> float                       # reference mass, kg
```

The middle axis is `â_x`. That is the longitudinal acceleration, *normalized to ±1* against the
straight-line capability at each point. That is why `domain()[1]` is always `[-1.0, 1.0]`.

Here are real numbers for the Model 3, on the default 40×25×7 grid:

```python
env = lap.envelope
print(env.shape(), env.mass_ref())
print(env.domain())
print(round(env.ay_boundary(50.0, 0.0, 9.81), 2),   # max lateral accel at 50 m/s
      round(env.accel_limit(30.0, 9.81), 2),
      round(env.brake_limit(50.0, 9.81), 2),
      round(env.drag_accel(50.0), 2))
```

```text
[40, 25, 7] 1765.0
[[5.0, 67.0], [-1.0, 1.0], [4.903325, 19.6133]]
9.07 7.22 11.99 0.43
```

`env.notes` carries three notes about generation. One is the crucial statement of scope: *"envelope
= tyre-force limit only (powertrain ceiling applied separately by the lap solver); ... boundary =
the T1 trim's friction feasibility limit (not filtered for open-loop stability — a T2+ concern)."*

There is currently no `to_dataset` helper for an envelope in `outlap.core`. The notebooks build
their own grids, from these scalar queries.

**The envelope cache.** Generating an envelope is the expensive cold step. The extension therefore
keeps a cache at process level.

Its key holds everything that changes the boundary: the hash of the resolved vehicle; a fingerprint
of the bytes of every loaded binary sidecar; the session conditions; the envelope grid; and the
`fz_coupling` mode.

`flat_track` is deliberately *not* in the key, because it only reshapes the path, and not the
boundary.

Here are the practical consequences, all measured. A second lap of the same car, in the same
process, is nearly instant: 0.12 s, after a cold solve of 65 s on a debug build, with the envelope
reused. Solving the raceline instead of the centerline also reuses it. But changing `conditions`, an
`override`, the grid, or `fz_coupling` regenerates it.

The cache is never evicted, because a Python session is assumed to be short-lived.

### 10.7 Errors: a quick reference

Every message below was produced verbatim by the shipped package.

| You do | You get |
|---|---|
| Load a missing vehicle/track/tire file | `FileNotFoundError: source not found: track.yaml` |
| `solve_lap(..., tier="t2")` | `` ValueError: the transient tier (t2) produces a time-indexed lap: call `outlap.solve_transient_lap(...)`, or `outlap.solve_lap_dataset(..., tier="t2")` for an xarray view `` |
| `overrides={"chassis.masss_kg": 1500.0}` | `` ValueError: unknown field `masss_kg` `` + `` help: did you mean `mass_kg`? `` |
| `conditions={"air": {"temp_c": 35.0}}` | `` ValueError: unknown conditions field `air.temp_c` (known fields here: ["pressure_hpa", "temperature_c"]) `` |
| `sim={"envelop": {}}` | `` ValueError: unknown sim field `sim.envelop` (known fields here: ["allow_degraded", "dt_s", "envelope", "fixed_point", "flat_track", "fz_coupling", "integrator", "raceline", "schema", "slow_decimation", "tier"]) `` |
| `ds_m=-1.0` (any sampling function) | `ValueError: ds_m must be a positive, finite number of metres, got -1` |
| Unsupported override value type (e.g. a numpy array) | `ValueError: unsupported value type in overrides: ndarray` |
| Mismatched array lengths in `Tyre.forces` | `ValueError: length mismatch: kappa has 5 elements, alpha has 3` |

The general rule is this. An *optional input that is missing* — `sim.yaml`, `conditions.yaml`, a
sidecar table, or a reference to a battery or thermal file — is never an error. It falls back, with
a recorded note. An input that is *present but broken* is always an error.

And a configuration error is treated as a product surface. The text of the `ValueError` preserves
the diagnostic help line from the Rust pipeline. A typo therefore comes back with a suggestion,
instead of with a bare trace from a parser.

### 10.8 `outlap.schemas`: validating a document against the contracts

The JSON Schemas in `schemas/`, licensed Apache-2.0, are generated from the `schemars` types in
Rust. Rust is the single source of truth, and Python only *conforms*; see Chapter 5.

`outlap.schemas` loads those committed schemas, and validates YAML documents against them, with
`jsonschema`.

Here are its public functions.

- `load_schema(name: str) -> dict[str, Any]` loads a committed schema, by the name of a document.
  `load_schema("vehicle")` reads `schemas/vehicle.json`. It raises `FileNotFoundError` for an
  unknown name.
- `check() -> int` validates three things: every committed schema, each of which must itself be a
  valid JSON Schema document of draft 2020-12; the shipped fixtures, under
  `crates/outlap-schema/tests/fixtures/`; and every `*.tyr.yaml` under `data/`. The last is globbed,
  so a new dataset is covered automatically.

  It returns an exit code for the process: 0 on success, and 1 otherwise, with a list of errors for
  each document on stderr.
- `main() -> int` is the entry point for the CLI. `--check` is the only flag, and it is also the
  default behavior.

```bash
$ python -m outlap.schemas --check
schema check OK: 8 schemas, 22 fixtures + 7 data files validated
```

The eight schemas are `battery`, `conditions`, `emotor`, `ptm`, `sim`, `track`, `tyr`, and
`vehicle`.

Two honest caveats. The paths are computed relative to the root of the repository, from the
module's own location; the check therefore works from a source checkout, and not from an installed
wheel. And although `pydantic` is a declared dependency, the mirror of the schemas in pydantic v2 is
a *planned later increment*; nothing in the package imports pydantic yet.

A fixture that relies on merging through `extends:` is validated by the Rust pipeline instead. A
JSON Schema cannot express the merge.

```python
from outlap.schemas import load_schema
schema = load_schema("vehicle")
print(schema["title"])        # Vehicle
```

### 10.9 The other public modules

Three more packages ship inside `outlap`. Chapter 11, on importers and tooling, covers their
workflows in depth. This is just an inventory of the public surface.

**`outlap.tir`** is the codec for `.tir` interchange, which is the Tire Property File format. It is
a pure-Python mirror of the tir module in the Rust `outlap-schema`.

It parses and writes, taking a string in and giving a string out. It also converts to and from a
`.tyr` dict. The writer is byte-compatible with the Rust writer; a shared canonical fixture pins
both.

Its exports are: `SYNTHETIC_THERMAL`, `SYNTHETIC_WEAR`, `ThermalWearPolicy`, `TirDoc`, `TirEntry`,
`TirError`, `TirSection`, `TirValue`, `format_number`, `parse_tir`, `tir_to_tyr`, `tyr_to_tir`, and
`write_tir`.

Its CLI is `python -m outlap.tir {to-tyr, from-tyr}`.

**`outlap.tirefit`** fits MF6.1 tires. It has four parts.

Ingestion of test data: `load_csv`, `load_dat`, `load_ttc_mat`, `TireTestData`, `SweepBin`, and
`bin_sweeps`.

A clean-room forward model in numpy: `forces`, `Forces`, `DEFAULTS`, `params_from_coeffs`, and
`params_from_tyr`.

A staged fit: `staged_fit`, `FitConfig`, `FitResult`, `StageReport`, and `synthesize`. It requires
scipy, through the `tire-fit` extra.

And reporting: `render_markdown`, `report_dict`, and `write_report`.

Its CLI is `python -m outlap.tirefit {fit, synth}`.

Its policy on redistribution is loud, and not negotiable: parsers yes, but **redistribution of FSAE
TTC data, or of a parameter set derived from it, no**. That data is locked to members.

**`outlap.importers`** holds tools for one-time local vendoring. CI never exercises them.

`outlap.importers.pdt_h5` converts PDT HDF5 into `.ptm` files and battery YAML. Its exports are
`PdtImportError`, `convert_batterypack`, `convert_driveunit`, `convert_edrive`, and
`validate_battery_doc`. Its CLI is
`python -m outlap.importers.pdt_h5 {edrive, driveunit, batterypack}`.

`osm_track` and `tumftm_track` are the track importers. They need the `track-import` extra. The
source data of TUMFTM is LGPL-3.0.

Finally, some facts about packaging, for orientation.

The package requires Python 3.12 or later. The compiled `outlap_core` extension is an abi3 wheel
built by maturin, declared as a path dependency on `crates/outlap-py`. Running `uv sync` in
`python/` therefore compiles it automatically, given a Rust toolchain.

The optional extras are `track-import`, which adds requests, scipy, and matplotlib, and `tire-fit`,
which adds scipy. The `notebooks` dependency group adds everything the notebook tour needs.

The `outlap` console script currently maps to the hello-world stub. The real entry points are the
module CLIs listed above.


---

## 11. Importers and tooling

*What you will learn: every command-line tool that ships with outlap v0.2.5. That is the PDT
powertrain importer, the two track importers, the tire-file codec and the fitting pipeline, the data
generators, and the tooling for schemas and golden files. For each one you get the exact
invocation, the files that it reads and writes, and the rules of data hygiene — the "firewall" —
that you must respect when you feed real data into it.*

### 11.1 The landscape of the tooling, and what does not exist yet

The tools of outlap are Python *module CLIs*. You run them as `python -m <module>`, from the
`python/` directory of a checkout of the repository, typically through uv, as
`cd python && uv run python -m ...`.

There is deliberately **no unified `outlap` command yet**. A new user will trip over two things.

- Installing the Python package puts an `outlap` script on your path. It is a placeholder.
  `outlap.main()`, in `python/src/outlap/__init__.py`, just prints `Hello from outlap!`. Ignore it.
- An error message occasionally hints at `outlap migrate`, for a mismatch in schema version. That
  verb is part of the planned unified CLI in Rust; see Chapter 15.

  Today, the hint tells you *what* will fix the file. It does not name a command you can run. The
  docstring of the PDT importer says the same thing explicitly: the module CLI "mirrors the future
  Rust `outlap import pdt-*` 1:1".

Here is the complete surface of the tooling, at v0.2.5:

| Tool | Invocation | Reads → writes |
|---|---|---|
| PDT HDF5 importer | `python -m outlap.importers.pdt_h5 {edrive,driveunit,batterypack}` | raw `.h5` → `.ptm.yaml` + parquet (+ `.emotor.yaml`) |
| OSM+DEM track importer | `python -m outlap.importers.osm_track` | public web data → `track.yaml` + `centerline.csv` |
| TUMFTM track importer | `python -m outlap.importers.tumftm_track` | TUMFTM CSVs → `track.yaml` + `centerline.csv` |
| `.tir` codec | `python -m outlap.tir {to-tyr,from-tyr}` | `.tir` ↔ `.tyr.yaml` |
| MF6.1 fitting pipeline | `python -m outlap.tirefit {fit,synth}` | test data → `.tyr.yaml` + report |
| Data generators | `python python/tools/gen_f1_aero.py`, `gen_model3_powertrain.py` | nothing → committed synthetic data |
| Figure renderers | `python python/tools/plot_*.py` (7 scripts) | committed data → `docs/**/img/*.png` |
| Schema generator | `cargo run -p outlap-schema --bin gen_schemas [-- --check]` | Rust types → `schemas/*.json` |
| MF6.1 golden regeneration | `MF_ORACLE_SRC=... ./tools/goldens/run.sh` | external oracle → golden CSVs |

`gen_schemas` is the only compiled binary in the whole Cargo workspace. Everything else here is
Python.

### 11.2 The PDT HDF5 importer: `outlap.importers.pdt_h5`

PDT is a proprietary tool for designing a powertrain. Its "stage files" describe one of three
things: an electric machine, called an **EDrive**; an assembly of motor, inverter, and gearbox,
called a **DriveUnit**; or a **BatteryPack**.

They are stored as HDF5, which is a hierarchical binary container format. In Python you read it with
the `h5py` library.

outlap never models these components internally. That is hard rule #1, the *firewall*; see
Chapter 6.

This importer instead converts a stage file into the open formats of outlap: a `.ptm` map file,
which is the powertrain map format of Chapter 5, plus a parquet *sidecar*, which is a columnar data
file that holds the big numeric tables next to the small YAML descriptor.

The importer package is at `python/src/outlap/importers/pdt_h5/`. It is a pure-Python adapter. It
uses `h5py`, `numpy`, and `pyarrow`, and nothing else. It never imports the code of PDT itself.

Here is the usage, from the module docstring:

```bash
python -m outlap.importers.pdt_h5 edrive      <file.h5> -o machine.ptm.yaml [--vdc 400]
python -m outlap.importers.pdt_h5 driveunit   <file.h5> -o du.ptm.yaml      [--vdc 48] [--mass-kg X]
python -m outlap.importers.pdt_h5 batterypack <file.h5> -o battery.yaml
```

Here is the full matrix of flags, from `python/src/outlap/importers/pdt_h5/__main__.py`:

| Flag | Subcommands | Default | Meaning |
|---|---|---|---|
| `src` (positional) | all | — | source `.h5` file |
| `-o/--out` | all (required) | — | output YAML path |
| `--vdc <V>` | edrive, driveunit | none | DC voltage to select (nearest grid slice); see below |
| `--torque-points <N>` | edrive, driveunit | 101 | size of the regular torque axis |
| `--maps <path>` | edrive, driveunit | `<out>.maps.parquet` | parquet sidecar path |
| `--emotor <path>` | edrive | `<out>.emotor.yaml` | thermal-model output path |
| `--no-emotor` | edrive | off | skip the 2-node thermal distillation |
| `--t-max-winding-c` | edrive | 180.0 | winding temperature limit for the fit (°C) |
| `--t-max-case-c` | edrive | 120.0 | case temperature limit for the fit (°C) |
| `--no-copper-feedback` | edrive | off | disable the α resistance-rise feedback |
| `--overload-from-cold` | edrive | off | accepted and recorded, currently a no-op in the fit |
| `--mass-kg <X>` | driveunit | none | mass override if the file lacks a mass group |
| `--tables <path>` | batterypack | `<out>.tables.parquet` | parquet sidecar path |

On success it prints `wrote <out>`, plus a summary. That summary covers the grid sizes, the
`nan_fraction`, the RMS of the thermal fit, the gear ratio, and the cell topology — whichever of
those apply.

Warnings go to stderr. An import error exits with code 1, and a one-line `error: ...`.

#### `--vdc`, and the default of a full voltage stack

An efficiency map from PDT is gridded over DC-link voltage, called *Vdc*. That is the voltage the
battery presents to the inverter.

The behavior of the importer is deliberately asymmetric.

- **With no `--vdc`, and a multi-voltage grid**, which is the default, the importer emits the **full
  Vdc stack**. That is a `ptm/2.0` document, with a third axis, `vdc_v`.

  This is what the Vdc–SoC coupling wants. At run time, the Rust core evaluates the map at the
  terminal voltage of the pack, which depends on state of charge; see Chapter 9.
- **With `--vdc <V>` given**, the importer picks the **single nearest slice of the grid**. It does
  not interpolate across voltage, because the thermal envelopes in the file are single-voltage. It
  then emits a single-voltage `ptm/2.0` map, with no `vdc_v` axis.

  If your requested voltage is more than **2 %** off the grid, you get a warning:
  `requested vdc X V snapped to grid Y V`; see `select_vdc`, in `common.py`.
- **With no `--vdc`, and a file whose thermal data names no voltage either**, the importer defaults
  to the **maximum** grid voltage, and warns about it.

#### What each subcommand emits

**`edrive`**, which is an electric machine plus an inverter, writes up to three files.

1. `machine.ptm.yaml`. It has `schema: ptm/2.0` and `kind: electric`, with a `vdc_v` axis when a Vdc
   stack was emitted.

   Its `limits:` block carries four things: the peak envelope of torque against speed; the
   **measured regen envelope**, `max_regen_torque_nm_vs_speed`, taken from the absolute value of
   `peak_capability/torque_regen`, so that an imported machine never leans on the fallback for a
   symmetric machine; continuous and overload curves, for holds of 10, 20, and 30 s; and the drag
   torque.

   Provenance lands in `meta.source`, as "PDT EDrive `<alias>` `<git hash>`".
2. `<out>.maps.parquet`, which is the sidecar for efficiency and loss.

   It has long, tidy float64 columns: `speed_rpm, torque_nm, efficiency, loss_w`, plus `vdc_v` for a
   stack. There is one row for each grid cell, and `NaN` where a cell is beyond the feasible
   envelope at that voltage.

   The speed axis is in RPM, because a file format is a display boundary. Everything converts to
   rad/s inside the solver.
3. `<out>.emotor.yaml`, which is a thermal model of the machine. `--no-emotor` skips it. See below.

Two decisions about physics are worth knowing; the module docstring of `edrive.py` records both.

The system efficiency is rebuilt as `motor_efficiency · inverter_efficiency`, and the system loss as
`motor_loss_total + inverter_loss_total`. The real files carry the two stages separately, and not as
one lumped table. And the torque coordinate is `airgap_torque`.

The importer also *never* trusts the summary scalars in `performance` or `metrics` in the file.
Power is always rebuilt as torque times angular speed, $P = \tau\,\omega$, because the real files mix
W and kW in adjacent summary fields.

The raw maps sit on an axis of "load ratio". The importer therefore inverts each row of speed onto a
regular grid of torque. It uses `--torque-points` nodes, with an exact zero node, and asymmetric
bounds for drive and regeneration. It masks the cells beyond the peak torque at each speed as
`NaN`. And it keeps the column at zero torque at efficiency 0, which is the "spin point".

The `nan_fraction` in the summary tells you how much of the rectangle is masked.

**`driveunit`**, which is a motor, an inverter, and a gearbox as one unit, writes `du.ptm.yaml`,
with `schema: ptm/2.0` and `kind: electric`, plus the maps sidecar. The logic for the Vdc stack is
the same.

There are four differences. The map is at the **output shaft**, with the gear ratio already applied;
the ratio appears only in `meta.source`. The measured regen envelope comes from
`peak_op/torque_regen`. The drag torque comes from the no-load test, resampled onto the speed axis
of the map. And there is no `.emotor.yaml`, because the thermal data of a drive unit is envelope
only.

Mass resolution tries four dataset names, in order, and finally your `--mass-kg` override. The Rust
loader requires `mass_kg > 0`, so a file without a mass group, and with no override, is a hard
error.

The importer also absorbs two quirks of a real PDT export. The thermal group of a drive unit is
spelled with a capital T, as `Thermal`. And a node name can arrive encoded as bytes twice, as
`b"b'ambient'"`.

**`batterypack`** writes `battery.yaml`, with `schema: battery/1.0` and `model: rc_pairs`. That is a
Thevenin *equivalent-circuit model* with one RC pair: a source of open-circuit voltage behind
resistances; see Chapter 9.

It also writes `<out>.tables.parquet`, with the columns at cell level: `soc, temp_c, ocv_v, r0_ohm,
r1_ohm, tau1_s, dudt_v_per_k`, on the grid of state of charge and temperature.

The YAML carries the pack topology, as `ns` in series times `np` in parallel; the capacity; the SoC
window; the power limits against SoC; and a lumped thermal block.

Three mentions inside `battery.py` still call the format "provisional". That wording is stale. The
Rust `BatteryDoc` type, and `schemas/battery.json`, exist, and they are enforced.

Every emitted document is validated against the committed JSON Schema — `schemas/ptm.json`,
`emotor.json`, and `battery.json` — before it is written. A failed validation is an import error,
and not a bad file on disk.

One practical caveat. That validation locates `schemas/` by walking up from the module file, through
`Path(__file__).resolve().parents[5]`. The importer therefore assumes a **checkout of the
repository**. A bare wheel installed by pip, outside the repository, will not find the schemas.

#### The thermal outputs: a 2-node fit, or a detailed network

When an EDrive file carries the full lumped-parameter thermal network, or *LPTN*, which is a graph
of heat capacities connected by thermal conductances, the importer takes the **detailed** path; see
`thermal_network.py`.

It collapses the roughly 20-node PDT network onto the reduced menu of nodes in outlap: `winding`,
`stator_iron`, `rotor`, `housing`, `coolant`, and `ambient`. It sums the capacities, and the
conductances between groups. It routes the loss maps for each component onto those groups. And it
rebuilds the paths of convection — the air-gap film, and the liquid cooling jacket — from clean
scalar fields of geometry, and never from the FEA mesh.

Those correlations for convection are the one deliberate, narrow reversal of the firewall. They were
ported into `outlap-thermal`, as open-source physics that the author owns. They are film
correlations of the Churchill–Chu and Gnielinski kind; Chapter 9 covers the model.

When the file has only thermal *envelopes*, which are curves of continuous and overload torque, the
importer distills a **lumped 2-node model** instead; see `thermal_fit.py`.

The model has a winding node and a case node:

$$C_w \dot T_w = s_w P\,k_{cu}(T_w) - G_{wc}\,(T_w - T_c)$$

$$C_c \dot T_c = (1 - s_w)\,P + G_{wc}\,(T_w - T_c) - G_{cool}\,(T_c - T_{cool})$$

$T_w$ and $T_c$ are the winding and case temperatures. $C_w$ and $C_c$ are their heat capacities, in
J/K. $G_{wc}$ and $G_{cool}$ are the conductances from winding to case, and from case to coolant, in
W/K. $P$ is the loss power. $s_w$ is the fraction of loss deposited in the winding. $T_{cool}$ is the
coolant temperature. And $k_{cu}(T_w) = 1 + \alpha\,(T_w - T_{ref})$ is the feedback from the rise
in copper resistance; disable it with `--no-copper-feedback`.

The four parameters $(C_w, C_c, G_{wc}, G_{cool})$ are fitted by least squares. The 2-node network,
driven by the exported loss map, must reproduce the continuous envelope from PDT, and the overload
torques at 10, 20, and 30 s, at a handful of speeds.

The dependency rule of the firewall forbids scipy here. The fit is therefore numpy only. The model
is linear and time-invariant, so its steady state and its transients are closed-form algebra on 2×2
matrices. And the optimizer is a hand-rolled, deterministic Nelder–Mead, in log space.

The quality of the fit is printed as `fit_rms`, and recorded in the `meta.notes` of the emitted
file.

Both paths emit `schema: emotor/1.1` documents. The same machine-thermal solver of Chapter 9
consumes them.

### 11.3 The firewall: rules you must respect

If you feed real proprietary data into these tools, the hygiene rules of the repository apply to
*you*, and not only to the maintainers.

1. **A PDT file is read as raw HDF5, with h5py only.** Never import the code of PDT itself. And
   never commit a real `.h5` stage file; they are private data.

   The CI test suite uses tiny **synthetic** fixtures, shaped like PDT files, generated at test
   time, in `python/tests/pdt_fixtures.py`. It never uses a real file.
2. **Never commit anything that the importer writes from real data.** The `.ptm.yaml`, the parquet
   sidecars, and the battery YAML are all artifacts derived from real data.

   The supported workflow is to import into a `local/` directory under your vehicle, such as
   `data/vehicles/<car>/local/`. The `.gitignore` of the repository blocks
   `data/vehicles/*/local/`, and it blocks the twin notebook for real data,
   `notebooks/07_qss_t1_local.ipynb`. A real import therefore physically cannot be committed by
   accident.
3. **FSAE TTC tire data is locked to members.** Keep raw files in the git-ignored `ttc-data/`
   directory. Never publish a TTC file, *or a parameter set fitted from one*. In the words of
   `python/src/outlap/tirefit/README.md`: "Parsers yes — redistribution of TTC data or TTC-derived
   parameter sets, NO".
4. **A reference book or paper stays out of the repository.** `**/*.pdf` is git-ignored. A
   coefficient *value* is a citable fact. The source document is not redistributed.
5. **A track importer is a tool for one-time local vendoring.** It reads only public or
   redistributable data, and it never runs in CI.

### 11.4 The track importers

Both track importers emit the same two files, into a track directory.

The first is `track.yaml`, which is the descriptor, at `schema: track/1.0`. The second is
`centerline.csv`, with the 8 columns `s_m, x_m, y_m, z_m, banking_deg, width_left_m, width_right_m,
grip_scale`. Those are arc length; position, under ISO 8855, where x is forward, y is left, and z is
up; banking; the half-widths of the corridor, to the left and right of the centerline; and a local
multiplier on grip.

Chapter 5 documents the format. Chapter 12, below, lists what has already been imported for you.

#### `osm_track`: OpenStreetMap plus elevation, the 3D importer

No open **3D** circuit data exists. This importer therefore builds it from public sources; see
`python/src/outlap/importers/osm_track.py`. It does three things.

First, it takes the centerline from OpenStreetMap ways tagged `highway=raceway`, which are licensed
ODbL. It **assembles them into the main closed lap**.

OSM often fragments a circuit into way segments named after corners, plus pit lanes and kart tracks.
The importer therefore drops ways that are not part of the circuit, by name. It builds the node
graph. It prunes dead-end spurs down to the 2-core. And it resolves a theta junction at the pit
bypass — two junction nodes joined by three paths — to the cycle of the two longest paths. On any
unexpected topology, it falls back to the longest single way. It then projects the result to a local
metric frame.

Second, it takes elevation from an open *DEM*, which is a digital elevation model: a public grid of
ground heights. It uses the free opentopodata API. It tries the 25 m European dataset, `eudem25m`,
first, and then the global `srtm30m`. It smooths the result with a cubic smoothing spline, so that
the second derivative of z, which vertical curvature needs, is continuous.

Third, it leaves banking at zero. A coarse public DEM cannot resolve it. You refine it later, with
sparse `banking_keypoints` in `track.yaml`.

The assembly of the theta junction is exercised offline, with no network, by
`python/tests/test_osm_track.py`.

```bash
cd python
uv run python -m outlap.importers.osm_track --preset catalunya --out ../data/tracks/catalunya_osm
# or an arbitrary circuit:
uv run python -m outlap.importers.osm_track --name "My Circuit" --lat 41.57 --lon 2.26 \
    [--radius 2500] [--ds 3.0] [--no-dem] --out <dir>
```

| Flag | Default | Meaning |
|---|---|---|
| `--preset` | — | one of `catalunya`, `spa`, `silverstone` |
| `--name/--lat/--lon` | — | ad-hoc circuit (either a preset or all three are required) |
| `--radius` | 2500 | OSM search radius in metres (ad-hoc path only; presets bake their own) |
| `--ds` | 3.0 | resample spacing, metres |
| `--no-dem` | off | skip elevation (flat track) |
| `--out` | required | output directory |

The emitted `track.yaml` records three things. `meta.source` is `osm+dem`, or `osm` with `--no-dem`.
`meta.accuracy_class` is `B` with elevation, and `C` without. And the attribution string is "©
OpenStreetMap contributors (ODbL); elevation `<dataset>` via opentopodata.org".

The widths of the corridor are **defaulted**, to a half-width of 6 m on each side, because OSM does
not carry them. The `notes` in the meta admit that.

The importer needs the `track-import` extra: run `uv sync --extra track-import`, which adds
requests, scipy, and matplotlib.

It is polite to the public APIs. It sends a descriptive User-Agent, it uses three Overpass mirrors,
and it throttles DEM requests to about one per second, with back-off.

And it **never runs in CI**.

#### `tumftm_track`: the TUMFTM racetrack-database, the flat importer

This converts the racetrack-database of TU München into the format of outlap; see
`python/src/outlap/importers/tumftm_track.py`.

That database holds 25 circuit centerlines, with corridor widths **measured from satellite images**,
on a uniform grid of about 5 m. It is licensed LGPL-3.0, and it is the standard academic dataset.

It is pinned to the upstream commit `e59595d1f3573b30d1ded6a08984935b957688e0`:

```bash
git clone https://github.com/TUMFTM/racetrack-database.git /tmp/tumftm
git -C /tmp/tumftm checkout e59595d
cd python
uv run python -m outlap.importers.tumftm_track --input /tmp/tumftm/tracks --out ../data/tracks
```

`--input` takes one CSV, or a directory of them. Each track is written to `<out>/<name>/`.

`--ds <m>` resamples. Its default, which is `None`, passes the native grid of about 5 m through
**unchanged**. That is exact, and it interpolates none of the measured widths.

A malformed file is skipped, with a message, while the rest convert. But the exit code is then 1.
Check stderr before you trust a batch run in a script.

Three points of correctness are documented by the module itself. The first is the classic trap.

1. **Widths are mapped by NAME, and never by column position.** The source lists RIGHT before LEFT,
   as `# x_m,y_m,w_tr_right_m,w_tr_left_m`. outlap lists LEFT before RIGHT, because road +y is
   *left* under ISO 8855.

   A positional mapping would therefore silently swap the corridor, and flip the computed racing
   line.
2. **The data is strictly 2-D.** `z_m` and `banking_deg` are emitted as 0, and `grip_scale` as 1.
   That is legitimate, because the source has no elevation. It is recorded in `meta.notes`, and in
   the accuracy class `C`.
3. **Closure.** The source loop is left open, about one sample short of the start. The track loader
   of outlap closes it over the connecting chord.

A table of 25 entries maps a source file stem to a directory name and a display name. For example,
`Nuerburgring` becomes `nuerburgring`, with the display name "Nürburgring GP". An unknown stem falls
back to a slug in snake_case.

This importer is pure standard library, plus numpy. It needs no extra.

### 11.5 Tire tooling: the `.tir` codec, and `tirefit`

Two module CLIs deal with tire data. The MF6.1 Magic Formula model itself is Chapter 7.

**The `.tir` codec** converts between the industry `.tir` property file format, and the `.tyr.yaml`
document of outlap:

```bash
python -m outlap.tir to-tyr   <in.tir>      -o out.tyr.yaml [--thermal-wear synthetic|from-donor|none] [--donor donor.tyr.yaml]
python -m outlap.tir from-tyr <in.tyr.yaml> -o out.tir
```

A `.tir` file cannot carry the `thermal:` and `wear:` blocks of outlap. `--thermal-wear` therefore
sets the policy for filling them.

`synthetic` is the default: it writes placeholder blocks, clearly labelled. `from-donor` copies them
from another `.tyr.yaml`, named by `--donor`. And `none` leaves them out.

The `.tir` writer in Python is byte-for-byte compatible with the writer in Rust. A shared canonical
fixture pins both. The `repr` float formatting of CPython matches the `ryu` of Rust exactly.

**The MF6.1 fitting pipeline** turns measured tire test data into a `.tyr.yaml`:

```bash
python -m outlap.tirefit fit   <data...> --unloaded-radius R0 -o out.tyr.yaml [--report-dir DIR] \
                               [--fnomin N] [--nompres PA] [--longvl MPS]
python -m outlap.tirefit synth <in.tyr.yaml> -o out.csv [--seed 0] [--noise 0.01]
```

`fit` reads one or more test files: a TTC `.mat` in v7 or v7.3, a `.dat`, or a `.csv`. It
concatenates them, and runs a staged least-squares fit: nominals, then pure longitudinal, then pure
lateral, then combined slip, then the aligning moment, then the overturning and rolling moments.

It prints the RMS error at each stage. It writes a `tyr/1.0` document, with synthetic placeholders
for thermal state and wear, and provenance marked `synthetic: true`.

`--longvl`, the reference speed, defaults to 16.7 m/s. `--report-dir` adds a `report.json` and a
`report.md`.

The fit stages need scipy: run `uv sync --extra tire-fit`.

`synth` is the inverse. It generates a deterministic, seeded synthetic dataset from an existing
`.tyr` file. It writes it as a faithful mock of the TTC format, with SAE signs, so that the round
trip from synth to fit, and real measured data, share one sign convention.

It is both the harness for the recovery test, and the way to exercise the pipeline without data that
is locked to members.

Remember firewall rule 3. A fitted TTC parameter set never leaves your machine.

### 11.6 The generators and figure renderers in `python/tools/`

Two generators author committed synthetic data. Both run from anywhere, because they anchor their
paths off the location of their own file.

- **`gen_f1_aero.py`** writes `data/vehicles/f1_2026/aero/f1_2026.parquet`. That is the aero map of
  the F1 reference car, over ride height, yaw, and DRS.

  It is a 5×5×5×2 grid, over `ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag`, with the value
  columns `cz_front_a_m2, cz_rear_a_m2, cx_a_m2`.

  It is **anchored**, so that at the reference ride heights — 30 mm front and 70 mm rear — the map
  reproduces the constant-aero fallback of the car exactly, at 1.9, 2.6, and 1.25 m². That is
  asserted at generation time.

  Every fibre aligned with the grid is monotone, or has a single peak. It is therefore safe for the
  one shared monotone cubic Hermite interpolant, in the Fritsch–Carlson construction.

  All the sensitivities are estimated. The file is synthetic, and it says so.
- **`gen_model3_powertrain.py`** writes the entire committed powertrain of the Tesla Model 3 study;
  see §12.2. That is three Vdc-stacked `ptm/2.0` maps for drive units, with `kind: electric`, plus
  their sidecars, at `ptm/du_{small,medium,large}.*`; and the synthetic pack in the 800 V class, at
  `battery/pack_800v.*`.

  Its design choices are deliberate teaching devices. The pairs of efficiency and loss are emitted
  consistently, so that energy closure holds exactly at a grid node: the drive loss is
  $P_{mech}(1/\eta - 1)$, and the regen loss is $|P_{mech}|(1 - \eta)$.

  Efficiency is *linear* in Vdc, so the shared interpolant reproduces and extrapolates that axis
  exactly.

  And the grid of 730, 790, and 850 V is deliberately narrower than the voltage swing of the pack.
  A lap at low SoC therefore exercises the documented extrapolation below the grid.

Seven `plot_*.py` scripts render the figures for the documentation, into `docs/theory/img/`,
`docs/validation/img/`, and `docs/vehicles/model3/img/`.

Four of them shell out to committed Rust examples —
`cargo run -p outlap-qss --example battery_coupling | ggv_traces | thermal_traces | limebeer_lap` —
and plot the CSV that those examples print. Every theory figure is therefore driven by the actual
model, and not by a re-implementation.

`plot_model3.py` instead drives the public Python API: `vehicle_report`, `solve_lap_dataset`, and
`min_curvature`; see Chapter 10.

None of these run in CI.

Two more generators of fixtures live inside the Rust crate tree:
`crates/outlap-schema/tests/fixtures/gen_ptm_maps.py`, and `gen_gridmap_fixture.py`. They write the
synthetic parquet fixtures that the Rust tests of the sidecar decoder consume.

### 11.7 `gen_schemas`: the source of truth for the schemas

The `schemars` types in Rust are the single source of truth for the file formats. The only binary in
the workspace regenerates the published JSON Schemas from them:

```bash
cargo run -p outlap-schema --bin gen_schemas            # writes schemas/*.json
cargo run -p outlap-schema --bin gen_schemas -- --check # regenerates in memory, diffs, fails on drift
```

It emits exactly eight documents:
`schemas/{vehicle,ptm,tyr,emotor,battery,track,conditions,sim}.json`.

The `--check` form is a gate in CI. The committed schemas can therefore never drift from the Rust
types.

These are the very schemas that the PDT importer validates its output against (§11.2). That ties the
emit side in Python to the truth in Rust.

A companion check in Python, `python -m outlap.schemas --check`, validates the shipped YAML data and
the fixtures against the committed schemas. It is also wired into CI.

### 11.8 `tools/goldens/`: regenerating the MF6.1 oracle CSVs

The numerical oracle of the tire model is a set of committed golden CSVs, in
`crates/outlap-tire/tests/golden/pacejka_2006_205_60r15/`. Both the Rust kernels and the Python
forward model must reproduce them to ≤0.5 %; see Chapter 13.

They were generated by running **teasit/magic-formula-tyre-library** (GPL-3.0) under GNU Octave, as
an external tool. Only its numeric outputs are captured, and never its source. The MF6.1
implementation of outlap is derived from Pacejka (2012) alone.

To regenerate them:

```bash
MF_ORACLE_SRC=/path/to/magic-formula-tyre-library/src ./tools/goldens/run.sh
```

The requirements are GNU Octave 8 or later on PATH, and a local checkout of the oracle, which is
never committed.

The script stages a copy of the oracle that holds the package only. It records the oracle's commit
and license into the provenance header of each CSV, which a test asserts. And it writes four CSVs —
`fx0.csv`, `fy0_mz.csv`, `combined.csv`, and `combined_camber.csv`, which is the κ×α sweep at γ =
±4° of camber — in SI units, with ISO 8855 signs.

The governance is strict. This **never runs in CI**; CI compares against the committed CSVs. There
is deliberately no in-tree mechanism to bless data from an external oracle. And regeneration is
allowed only in a PR that updates the version pins, and states the reason, in physics or in tooling.

---

## 12. The shipped data library

*What you will learn: everything under `data/` — three reference vehicles, three tire sets backed by
citations, and 27 circuits. For each asset you learn what it is for, where every number comes from,
and how much to trust it. You will also learn the license obligations that travel with the track
data, if you redistribute it.*

### 12.1 How the library is organized

All shipped data is licensed **CC-BY-SA-4.0**, with SPDX headers on every file. That is distinct
from the AGPL-3.0-only code, and from the Apache-2.0 `schemas/`.

The honesty contract of the library: reference data is *synthetic or transcribed, and never
measured*, with plausible magnitudes **clearly labelled at their source**.

The run time backs that up. Every estimated or defaulted value surfaces in the loaded-model report,
through `outlap.vehicle_report(...)`; see Chapter 10. Nothing is silent.

Each vehicle directory is self-contained. It holds a `vehicle.yaml`, whose referenced `.ptm`,
`.tyr`, battery, and emotor files live in sibling subdirectories. It loads as one unit; see
Chapter 4.

Three levels of trust recur below, so name them once. **Spec** means a published value from a
manufacturer or a paper, and it is cited. **Estimated** means a documented heuristic, flagged in the
loaded-model report. And **synthetic** means an invented smooth surface, from a committed generator
script. It is reproducible, but it is not data about any real machine.

### 12.2 Vehicles (`data/vehicles/`)

Three vehicles ship at v0.2.5: `limebeer_2014_f1`, `f1_2026`, and `tesla_model3_rwd`.

#### `limebeer_2014_f1`: reference car #1, the validation car

This is the complete published F1 parameter set of Perantoni & Limebeer, "Optimal control for a
Formula One car with variable parameters", *Vehicle System Dynamics* 52(5), 653–678, 2014, from
Table 4 and §2. It is transcribed clean-room from the open-access manuscript, at the Oxford
University Research Archive, `uuid:ce1a7106-0a2c-41af-8449-41541220809f`.

Its whole reason to exist is the cross-check at Catalunya, against the published optimal lap of the
paper: 82.43 s on a 2 m grid, with a top speed of about 88 m/s. Chapter 13 tells the full story of
the gate.

The provenance of each coefficient lives in `data/vehicles/limebeer_2014_f1/README.md`. Here are the
highlights:

| Field | Value | Provenance |
|---|---|---|
| mass | 660 kg | Table 4 |
| CG | [1.8, 0, 0.3] m | Table 4 (a, symmetric, h) |
| wheelbase / track | 3.4 m / [1.46, 1.46] m | Table 4 |
| yaw inertia Iz | 450 kg·m² | Table 4 (Ixx/Iyy 112.5/425 are **not** in the paper — placeholders, unused by the steady-state tiers) |
| drag area CxA | 1.35 m² | Cd·A = 0.9 × 1.5 (Table 4) |
| downforce split | 1.98529 / 2.51471 m² | ClA = 4.5 m² split by the centre of pressure at 1.9 m from the front axle |
| roll stiffness share | 0.5 / 0.5, roll centres at 0 | makes outlap's lateral load transfer algebraically identical to the paper's eq. (26) |
| ride rates | 200 000 N/m | **estimated placeholder** — no ride-height aero map is installed, so their only consumer never runs |
| power | 560 kW | **not in the manuscript** — Perantoni's companion doctoral-thesis value; it reproduces Fig. 8's ≈88 m/s top speed with Table-4 drag through $P = \tfrac{1}{2}\rho\,C_dA\,u^3 \Rightarrow 88.4$ m/s |
| brake balance | 0.6 | **estimated** (the paper leaves the per-axle ratio implicit; braking is tyre-limited either way) |

It is the only vehicle that ships its own `conditions.yaml`, at 21.0 °C and 1013.25 hPa. Those
values are chosen so that the ideal-gas conversion of outlap reproduces the air density of the
paper, which is exactly 1.2 kg/m³.

The README also records a consultation under the clean-room rule. The MIT-licensed `fastest-lap`
project was read as a *numerical cross-check only*. Its transcription of Tables 3 and 4 matches
verbatim. But its powertrain, at 735.5 kW plus a boost, is its own invention, so its lap times are
not comparable. No code was taken.

#### `f1_2026`: the synthetic F1 2026 hybrid

This is a demonstration car, and not a validation car. It has an ICE and an MGU-K on one shaft,
through an 8-speed gearbox, with `ratios: [2.9, 2.2, 1.8, 1.5, 1.28, 1.1, 0.98, 0.86]`,
`final_drive: 3.1`, and shifts of 20 ms, then a limited-slip differential to the rear axle. Its mass
is 768 kg, its wheelbase 3.40 m, and its track [1.65, 1.60] m.

Everything is synthetic but plausible, and the file header says so twice. The figures for the ERS
and its energy — a 4.0 MJ store, a SoC window of [0.2, 0.9], 350 kW of deployment with a taper
against speed, and 8.5 MJ of harvest for each lap — are "approximate 2026-regulation values for
testing". Verify them against the published FIA 2026 Technical Regulations before you treat them as
reference data.

Its aero is the interesting part. The `aero.constant` block feeds the T0 tier. It holds CxA of
1.25 m², and CzA of 1.9 plus 2.6, giving 4.5 m². That is about 1.7 times the weight of the car in
downforce at 250 km/h, with an L/D of about 3.6.

T1 instead consumes the shipped `aero/f1_2026.parquet`, which is the map over ride height, yaw, and
DRS. `gen_f1_aero.py` generates it, and it is **anchored**, so that the map reproduces those exact
constants at the reference ride heights; see §11.6.

The constants themselves stand in for the same magnitudes of aero as the Limebeer car; ClA of
4.5 m² is the value from PL2014. That anchoring is what keeps the behavior of the two F1 cars
comparable.

There are two honest gaps. `battery.params` references `battery/f1_es.yaml`, which **does not ship
yet**; the energy manager for ERS deployment and harvest is future work, and the ERS is enforced as
a power cap. And `tyr/slick.tyr.yaml` is a hand-authored synthetic slick, "for schema/round-trip
testing only".

#### `tesla_model3_rwd`: the Model 3 HV variant study, the showcase for the electrified stack

Read the caveat before you quote any number from this car.

It is the identity of a production **Tesla Model 3 RWD, re-imagined as an HV variant, in the 800 V
class**. Its chassis, mass, and aero are plausible for a Model 3. Its powertrain is deliberately
*not* the real car at about 360 V. It is a stack of a drive unit and a pack in the 800 V class, so
that the Vdc–SoC coupling of Chapter 9 is live on a road car.

Everything committed is **synthetic or estimated**. The drive-unit maps and the pack tables are
invented smooth surfaces, written by `python/tools/gen_model3_powertrain.py`. They were never
measured, and they are never derived from any PDT export.

The anchors from the spec sheet are: a curb mass of 1765 kg; a wheelbase of 2.875 m; a track of
1.58 m; and CxA of 0.51 m², from the published Cd of 0.23 times a frontal area of 2.22 m².

Everything else is a documented estimate. The CG position comes from an assumed weight distribution
near 47/53. The CG height of 0.45 m follows from the floor-mounted pack. The ride rates come from
ride frequencies near 1.5 Hz. The roll-center heights suit a strut front and a multi-link rear. The
brake balance is 0.62. And the ceiling on the one-pedal regen blend is 0.6.

The values for anti-dive and anti-squat are *omitted* on purpose. The estimator in the load pipeline
therefore fills them, and the loaded-model report shows it. The car loads warning-clean, with every
estimate noted; the capstone notebook counts 10 entries under estimated.

The tire is a **documented proxy**. The published Pacejka (2006) 205/60R15 book tire stands in for
the real 235/45R18, because no public Magic Formula set exists for the OE tire.

The committed powertrain is a three-way study of sizing. It is the sensitivity axis of notebook
`07_qss_t1`:

| Variant | Peak torque (output shaft) | ≈ Peak power | File |
|---|---|---|---|
| small | 1365 N·m | 100 kW | `ptm/du_small.ptm.yaml` |
| **medium (default)** | 2765 N·m | 203 kW | `ptm/du_medium.ptm.yaml` |
| large | 3381 N·m | 248 kW | `ptm/du_large.ptm.yaml` |

All three share a base speed of 700 rpm at the output shaft, and a Vdc grid of 730, 790, and 850 V.
The medium sizing sits at the roughly 200 kW of a production Model 3 RWD.

The scales of torque mirror the private sweep over drive-unit sizing by the author, so that this
committed story and its untracked twin with real data tell the same tale. But the surfaces
themselves are invented.

The pack, at `battery/pack_800v.battery.yaml`, is a synthetic Thevenin pack: 220 in series, 1 in
parallel, 92 Ah, and 64.064 kWh. Its open-circuit range of about 634 to 810 V deliberately sags
*below* the Vdc grid of the drive units, under load at low SoC. Every lap that discharges deeply
therefore exercises the documented extrapolation below the grid.

The machine-thermal model, at `emotor/rear_du.emotor.yaml`, is a hand-authored lumped network of six
nodes: winding, stator_iron, rotor, housing, coolant, and ambient. The winding limits are 150 and
180 °C, and the rotor limits are 140 and 170 °C. Every value is estimated, from documented
heuristics.

Swapping a sizing is a what-if override on one line, with no file edited:

```python
solve_lap_dataset(vehicle_dir, line, tier="t1",
                  overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"})
```

Finally, here is the firewall in practice. The README of the vehicle walks through importing the
*real* PDT drive units, and the real 704 V pack, into the git-ignored
`data/vehicles/tesla_model3_rwd/local/` directory, with
`python -m outlap.importers.pdt_h5 driveunit|batterypack`. You then point the same car at them,
through overrides. Nothing under `local/` can be committed.

### 12.3 Tires (`data/tires/`)

Three `.tyr` datasets backed by citations ship. A coefficient *value* is a transcribed fact, with an
exact citation in the `provenance` block and the README of each dataset. The source documents are
never redistributed.

One caveat is shared by all three. The `tyr` format requires a `thermal:` block and a `wear:` block.
Those physics models are still future work, and no published source provides them. Every dataset
therefore carries **synthetic placeholders, clearly labelled**, for those two blocks.

`synthetic: false` on a dataset means that the force and moment coefficients that carry the physics
are the published set. The placeholder blocks are the documented exception.

1. **`pacejka_2006_205_60r15/`** is the 205/60R15 91V passenger tire of Pacejka, *Tyre and Vehicle
   Dynamics*, 2nd ed. (2006), Table A3.1. It is the worked-example car tire of the book, and the
   MF6.1 **validation tire** of outlap; the golden CSVs of §11.8 are computed for it.

   It is a 2nd-edition set, so it has no terms for inflation pressure, `Mx ≡ 0`, and rolling
   resistance through `qsy1` only. This same file is the `tyr/road.tyr.yaml` of the Model 3.
2. **`roborace_devbot_mf52/`** is the Roborace DevBot "sport focused road tire", from
   Open-Car-Dynamics by TUMFTM, licensed Apache-2.0, at the pinned commit `0a92c686`.

   It is an MF5.2 set mapped to MF6.1, with a conversion table for each coefficient in its README.
   The camber term `PHY3` folds into `PKY6`. It has no pressure model. And `Mz ≡ Mx ≡ 0`.
3. **`limebeer_2014_f1/`** is an MF6.1 re-expression of the tire model of PL2014, from Appendix A
   and Table 3. That model has peak friction that is linear in load, with a
   $\sin(Q \arctan(S\rho))$ shape.

   The transcription is exact where the paper is linear. `PDX1 = 1.575` and `PDX2 = -0.35`
   reproduce μx of 1.75 at 2 kN and 1.40 at 6 kN *exactly*. Likewise `PDY1 = 1.625` gives 1.80 to
   1.45. And `PCX1 = PCY1 = 1.9` is the shape factor Q of the paper.

   The stiffness terms, `PKX*` and `PKY*`, were fitted numerically instead, so that the MF6.1 peaks
   sit where the formula of the paper *actually* peaks.

   That last clause matters. The README documents an inconsistency in PL2014 with itself: the stated
   peak slips, 0.11 and 0.10, and 9° and 8°, disagree with the paper's own formula, which peaks at
   0.756 times those values. The transcription anchors to the formula, because the validation target
   is the simulation of the paper.

   It has no aligning moment, and no sensitivity to camber or pressure, matching the paper. No
   third-party source code was consulted; `fastest-lap` was read as a numerical cross-check only.

One warning about a stale doc. The table in `data/tires/README.md` still lists only the first two
datasets, and calls an F1 reference tire "deferred". The `limebeer_2014_f1/` directory ships
regardless — it arrived with the Limebeer cross-check — and it carries its own full README on
provenance. Trust the dataset directories over the index table.

Every `.tyr.yaml` under `data/` is checked against the schema in CI, through
`python -m outlap.schemas --check`. And `crates/outlap-tire/tests/reference.rs` globs every dataset,
and asserts a load with no warnings, a round trip through the `.tir` codec that is numerically
exact, and physics checks for each tire.

### 12.4 Tracks (`data/tracks/`): 27 circuits, from two sources

27 track directories ship. There are 25 flat circuits, vendored from the TUMFTM racetrack-database,
under LGPL-3.0. And there are two 3D imports: `catalunya_osm` and `spa_osm`.

The "length" column below is the arc length $s$ at the last sample of the centerline. The loader
closes each loop over the final chord, so the lap length is a few meters more.

| Directory | Circuit | Length (m) | Source | Class |
|---|---|---|---|---|
| `austin` | Circuit of the Americas | 5502 | TUMFTM | C |
| `brands_hatch` | Brands Hatch Circuit | 3900 | TUMFTM | C |
| `budapest` | Hungaroring | 4372 | TUMFTM | C |
| `catalunya` | Circuit de Barcelona-Catalunya | 4645 | TUMFTM | C |
| `catalunya_osm` | Circuit de Barcelona-Catalunya | 4674 | OSM + DEM | B |
| `hockenheim` | Hockenheimring | 4564 | TUMFTM | C |
| `ims` | Indianapolis Motor Speedway (oval) | 4017 | TUMFTM | C |
| `melbourne` | Albert Park Circuit | 5294 | TUMFTM | C |
| `mexico_city` | Autódromo Hermanos Rodríguez | 4292 | TUMFTM | C |
| `montreal` | Circuit Gilles Villeneuve | 4353 | TUMFTM | C |
| `monza` | Autodromo Nazionale Monza | 5785 | TUMFTM | C |
| `moscow_raceway` | Moscow Raceway | 4058 | TUMFTM | C |
| `norisring` | Norisring | 2291 | TUMFTM | C |
| `nuerburgring` | Nürburgring GP | 5139 | TUMFTM | C |
| `oschersleben` | Motorsport Arena Oschersleben | 3687 | TUMFTM | C |
| `sakhir` | Bahrain International Circuit | 5401 | TUMFTM | C |
| `sao_paulo` | Autódromo José Carlos Pace (Interlagos) | 4300 | TUMFTM | C |
| `sepang` | Sepang International Circuit | 5532 | TUMFTM | C |
| `shanghai` | Shanghai International Circuit | 5440 | TUMFTM | C |
| `silverstone` | Silverstone Circuit | 5882 | TUMFTM | C |
| `sochi` | Sochi Autodrom | 5836 | TUMFTM | C |
| `spa` | Circuit de Spa-Francorchamps | 6995 | TUMFTM | C |
| `spa_osm` | Circuit de Spa-Francorchamps | 6992 | OSM + DEM | B |
| `spielberg` | Red Bull Ring | 4310 | TUMFTM | C |
| `suzuka` | Suzuka Circuit | 5798 | TUMFTM | C |
| `yas_marina` | Yas Marina Circuit | 5542 | TUMFTM | C |
| `zandvoort` | Circuit Zandvoort | 4311 | TUMFTM | C |

The **accuracy class** in the meta of each `track.yaml` is the label of trust.

Class **C** means a flat 2-D centerline, where `z = 0`, `banking_deg = 0`, and `grip_scale = 1`. It
is legitimate, and nothing is fabricated. But such a track exercises no physics of grade, vertical
curvature, or banking.

Class **B** means real elevation from a public DEM, with defaulted widths and unresolved banking.
Adding hand-annotated `banking_keypoints` moves a track toward class A.

Note three honest quirks. `ims` is the *geometry* of the oval only; its famous banking is not
represented. `nuerburgring` is the **GP-Strecke**, of about 5.14 km, and not the Nordschleife. And
the TUMFTM set is frozen around 2021, so Yas Marina is the pre-2021 layout, and Zandvoort the
pre-2020 one.

**The two Catalunyas are the classic trap.**

`catalunya_osm` is the 3D import from OSM and a DEM. It is the *reference* Catalunya, used by every
notebook, every example lap, and the Perantoni & Limebeer cross-check. It has about 30 m of change
in elevation.

`catalunya` is the flat TUMFTM vendoring, a peer of the other 24.

They are the same circuit, from two sources. The validation work found that the smoothed class-C
geometry does not reproduce the apex speeds of the paper: it rounds the slow chicane open, and
tightens the fast corners. That is why the cross-check stays on `catalunya_osm`; see Chapter 13.

The same pairing applies to `spa`, which is flat, against `spa_osm`, which is 3D.

**`spa_osm` is the showcase for elevation.** Spa-Francorchamps climbs about 100 m, from Eau Rouge,
through Raidillon, up to Les Combes. The committed geometry has 107.7 m of span in elevation, over a
closed loop of 6995 m, which is within 0.13 % of the official GP layout of 7004 m.

That makes it the circuit where the physics of grade and vertical curvature genuinely matters. The
compression at Eau Rouge loads the car. The crest at Raidillon unloads it.

Its import is also the interesting one, technically. OpenStreetMap fragments Spa into way segments
named after corners, plus pit and kart tracks. The importer therefore assembles the main lap
graph-theoretically: it drops ways that are not part of the circuit, by name; it prunes dead-end
spurs; and it resolves the junction at the pit bypass to the cycle of the two longest paths. See
§11.4, and the resulting map in `data/tracks/README.md`.

Committed sanity tests pin the closure, the length, and the physical plausibility of its grade and
vertical curvature. Notebook 02 plots it.

#### License obligations when you redistribute

The 25 TUMFTM circuits are **LGPL-3.0 data**.

outlap redistributes them legitimately, in two ways. It ships the upstream license text verbatim, as
`data/tracks/LICENSE-tumftm-LGPL-3.0.txt`. And it embeds the attribution string in every
`track.yaml`: *"Centerline © TU München, Institute of Automotive Technology (TUMFTM
racetrack-database), LGPL-3.0"*, plus a `notes` field that records the upstream commit, `e59595d`.

If you redistribute these tracks — in a fork, in a product, or in a derived dataset — you must carry
both forward.

The OSM imports, `catalunya_osm` and `spa_osm`, are ODbL. Keep "© OpenStreetMap contributors
(ODbL)", and the credit for elevation, "elevation eudem25m via opentopodata.org". And remember that
a database derived from ODbL inherits ODbL terms.

### 12.5 `data/presets/`: reserved, and currently empty

The directory exists. It ships **no files** at v0.2.5.

It is reserved for the planned class presets: `formula_base`, `gt_base`, and `passenger_base`. Those
would be starting points that you `extends:` from, when you author your own vehicle.

Until they land, the practical way to start a new car is to copy the closest reference vehicle, and
edit it, keeping the comments on provenance honest. Chapter 14 walks through exactly that.

Do not confuse this directory with the `--preset` flag of the track importer (§11.4). It is the same
word, and an unrelated feature.


---

## 13. Validation, testing, and trust

*What you will learn: why you should, and should not, believe the numbers that outlap prints. This
chapter walks through the one published cross-check that the simulator is gated against, the two
families of golden-file tests, the property tests that pin the physics sign by sign, the guarantees
on performance and determinism, and — just as important — an honest inventory of what is **not** yet
validated at v0.2.5.*

A lap simulator is only as useful as it is trustworthy.

The answer of outlap to "why should I believe this?" has four parts. It is cross-checked against an
independent, published result for an F1 car. Its outputs are frozen as golden files, which fail CI
if they drift. Its physics is pinned by property tests, which assert invariants such as "grip never
exceeds the friction circle" and "energy in equals energy out". And every estimate or simplification
is surfaced in the loaded-model report, and in the `notes` of the lap. Nothing is silent.

This chapter covers each in turn, and closes with the gaps.

### 13.1 The Limebeer cross-check: the one published oracle

The single most important asset for validation is the **Perantoni & Limebeer 2014 cross-check**,
documented in [`docs/validation/limebeer.md`](validation/limebeer.md).

An *oracle* here means an independent, published result that outlap is compared against, and that
outlap did not produce. It is a yardstick from outside the project.

The oracle is G. Perantoni & D. J. N. Limebeer, *"Optimal control for a Formula One car with
variable parameters"*, Vehicle System Dynamics **52**(5), 653–678 (2014).

It is an open-access paper. It solves the *time-optimal* control problem for an F1 car around the
Circuit de Barcelona-Catalunya. And it publishes three things: the full parameter set for the car,
in Tables 3 and 4 and Appendix A; an optimal lap of **82.43 s**, on a computational grid of 2 m, or
82.57 s in the mesh-asymptotic limit; and a speed trace, in its Fig. 8, that tops out near **88
m/s**.

The `limebeer_2014_f1` reference vehicle of Chapter 12 is that car, transcribed clean-room from the
manuscript.

#### 13.1.1 The configuration

The cross-check runs `limebeer_2014_f1` on the minimum-curvature line of `catalunya_osm`. Two
settings make the comparison honest.

- **`sim.flat_track: true`.** PL2014 is a study in two dimensions. This analysis mode therefore
  zeroes the grade, the banking, and the vertical curvature of the track, and collapses the g-g-g-v
  envelope to a flat g-g; see Chapter 8.
- The air density is pinned to $\rho = 1.2\ \mathrm{kg/m^3}$, which is the value in the paper,
  through the car's own `conditions.yaml`. And the production envelope grid, at $40\times25\times7$,
  is used.

You can reproduce it locally:

```bash
cargo run --release -p outlap-qss --features parallel --example limebeer_lap
python python/tools/plot_limebeer.py
```

#### 13.1.2 The gates, and what actually passes

Here is the subtle and important part.

The original validation plan called for a gate of "lap time within 1 %". That gate was found to be
**unattainable by construction**. A quasi-steady-state solver on a fixed heuristic racing line
cannot match a transient solver that co-optimizes the line and the speed for time. The PL2014 paper
*itself* cites a gap of 2.19 s between quasi-steady state and optimal control, at Barcelona.

The lap-time gate was therefore **re-scoped**. The numbers that the committed track geometry can
honestly support are gated. The lap time is *recorded with a decomposition*, and not gated.

| Gate | outlap | PL2014 | Result |
|------|--------|--------|--------|
| Top speed ≤ 1 % | 87.8 m/s | ≈ 88 m/s | **PASS** (−0.2 %) |
| Slowest-corner apex ≤ 5 % | 17.7 m/s | 17 m/s | **PASS** (+4.1 %) |
| Fast-corner apexes ≤ 5 % | 59.1 / 60.8 m/s | 60 / 60 / 62 m/s | **PASS** (−1.5 % / −1.9 %) — *only on the paper's own geometry* |
| Lap time | 92.36 s (committed track) / 87.08 s (paper's geometry) | 82.43 s | **recorded, not gated** |

The gates on top speed and on the slowest apex run in CI, in
[`python/tests/test_limebeer.py`](../python/tests/test_limebeer.py), on the committed
`catalunya_osm` import.

The band for the fast corners passes only on the digitized curvature of the paper itself. On the
committed OSM import, the fast corners are corrupted by geometry: interpolation noise throws up
spurious spikes in curvature, the widths are defaulted, and it is a later layout of the circuit than
the 2013 one in the paper.

The gate on fast corners therefore stays **deferred**. The time-weighted racing line, which now
ships, recovers only a few tenths of the residual on this geometry. And no fixture with the exact
center line of the paper is committed. The honest cross-check remains the recorded, decomposed
delta; see `docs/validation/limebeer.md`.

#### 13.1.3 Reading the gap in lap time honestly

The difference between 92.36 s and 82.43 s looks alarming, until you decompose it.
`docs/validation/limebeer.md` does that. The gap is *structural*, and not a model error.

1. **Quasi-steady state against transient, about 2.2 s.** That is the paper's own cited figure for
   the gap between QSS and optimal control.
2. **Line optimality.** The minimum-curvature line minimizes $\int \kappa^2\,ds$, which is the
   integrated squared curvature. It does not minimize lap time. The time-weighted line of §10.3
   recovers a few tenths of this residual.
3. **Conservatism in the envelope, about 1 s to 1.5 s.** The boundary of the double-track trim
   delivers 85 % to 91 % of the point-mass ideal. That is legitimate physics: a real four-wheel car
   cannot use as much grip as an idealized point mass.
4. **Track geometry, about 5 percentage points.** Swapping the committed OSM curvature for the
   paper's own curvature drops the lap from 92.36 s to 87.08 s.

What the cross-check **does** positively validate is the complete transcription of the car. The
peak friction coefficient is exact at every vertical load. The peak-slip locations are within 0.5 %.
The coupling under combined slip is within about 5 %. Top speed is within −0.2 %. And the corner
speeds, slow and fast, are within 5 % on like-for-like geometry.

### 13.2 Golden-file tests: two distinct families

A *golden file* is a committed reference output. A test recomputes the same output, and fails if it
drifts beyond a tolerance.

outlap has two families, with deliberately different governance.

#### 13.2.1 The Magic Formula golden CSVs, from an external oracle

The tire force model is checked against an **external** oracle.

Committed CSVs in `crates/outlap-tire/tests/golden/pacejka_2006_205_60r15/` — `fx0`, `fy0_mz`,
`combined`, and `combined_camber` — hold forces computed by a third-party implementation of the
Magic Formula. That is teasit/magic-formula-tyre-library, licensed GPL-3.0, run under GNU Octave.

The forces of outlap must match them to **≤ 0.5 %**, at each point, with a small absolute floor for
each channel. The rule is `|model − ref| ≤ max(0.005·|ref|, floor)`.

The commit hash of the oracle, and the Octave version, are recorded in the provenance header of each
CSV, and a test asserts them.

Because these are data from an external oracle, there is **no in-tree `--bless`**. You cannot
regenerate them by hand. They are re-derived only by re-running the external oracle, with
`MF_ORACLE_SRC=... ./run.sh`, under `tools/goldens/`. And a diff on a CSV, without an update to the
provenance header and a justification in physics, is a review stop.

Crucially, outlap never reads, ports, or vendors the source code of the oracle. Only its *output
numbers* are used, as data.

#### 13.2.2 The golden laps, in parquet, with `OUTLAP_BLESS=1`

Whole solved laps are frozen too.

[`python/tests/test_limebeer.py`](../python/tests/test_limebeer.py) commits a set of parquet
channels for each vehicle × track × tier. Examples are `limebeer_t0_flat.parquet`,
`limebeer_t1_flat.parquet`, and `f1_2026_t0.parquet`, in `python/tests/golden/`.

It re-solves them, and checks each channel against its committed values, with a relative tolerance
for each channel: 0.5 % for speed, 2 % for accelerations, 0.5 % for time, 1 % for vertical load, and
5 % for slip ratio and slip angle.

It even asserts that the *pattern* of infeasible stations, which are NaN, has not drifted. That
catches a change in where the solver decides that a corner cannot be trimmed.

These you *can* regenerate. But only deliberately:

```bash
OUTLAP_BLESS=1 uv run pytest tests/test_limebeer.py
```

The convention of the project is that a regeneration of a golden file must come with a note in the
pull request, explaining the change in physics that justifies it. `CONTRIBUTING.md` refers to this
as `--bless`; the concrete mechanism is the environment variable `OUTLAP_BLESS=1`.

A silent update to a golden file is forbidden. That is how a subtle regression sneaks past review.

### 13.3 Property tests: pinning the physics

A golden file catches *drift*. A property test catches *wrongness*.

A property test asserts an invariant that must hold for *any* input. It often runs across dozens of
randomized cases; the project uses the `proptest` crate.

These are where the sign conventions of the physics, and the conservation laws, live. Here is a
representative inventory.

- **The tire model**, in [`crates/outlap-tire/tests/props.rs`](../crates/outlap-tire/tests/props.rs).
  Outputs stay finite everywhere. An airborne tire, where $F_z \le 0$, produces zero force. There is
  odd symmetry, so flipping the slip flips the force. The signs are pinned. And there is
  **containment in the friction circle**: the force under combined slip never leaves the friction
  ellipse.
- **The trim solver**, in
  [`crates/outlap-qss/tests/t1_trim.rs`](../crates/outlap-qss/tests/t1_trim.rs), whose header states
  the ISO 8855 conventions. The four vertical loads sum to weight plus downforce. Lateral load
  transfers to the *outside* wheels in a corner. Longitudinal load transfers rearward under
  acceleration, and forward under braking. There is **containment in the friction circle at each
  wheel**. There is symmetry between left and right, for a symmetric car at $\pm a_y$. Newton
  converges over a feasible grid. And the two `fz_coupling` modes agree at convergence.
- **The lap solver**, in
  [`crates/outlap-qss/tests/properties.rs`](../crates/outlap-qss/tests/properties.rs). A solved lap
  stays inside the envelope. The forward and backward passes are idempotent. And lap time converges
  as the station spacing `ds` is refined.
- **Energy closure.** The tests for the powertrain and the thermal model assert that source work
  equals mechanical work plus the declared losses. Energy is neither created nor destroyed.
- **The battery**, in
  [`crates/outlap-qss/tests/battery.rs`](../crates/outlap-qss/tests/battery.rs). The pulse response
  matches the closed-form Thévenin solution. State of charge decreases monotonically under
  discharge. The advance of the slow states is deterministic. And a Vdc-coupled map reproduces
  in-grid values, and extrapolates correctly below and above the voltage grid.
- **The thermal model**, in
  [`crates/outlap-qss/tests/thermal.rs`](../crates/outlap-qss/tests/thermal.rs). A stint that soaks
  up heat reduces the derate. Cooling in the network depends on speed. And the mass heuristics fill
  a lumped model that is under-specified.
- **The interpolant**, in
  [`crates/outlap-core/tests/gridmap_props.rs`](../crates/outlap-core/tests/gridmap_props.rs). Node
  values are reproduced exactly. Each fibre equals the shared monotone cubic, and preserves
  monotonicity. And the analytic gradient matches a cross-check by finite difference.
- **The envelope corrections**, in
  [`crates/outlap-qss/src/t1/envelope.rs`](../crates/outlap-qss/src/t1/envelope.rs). The corrected
  envelope is exact at a node, to less than 2 % of the peak. And it matches full T1 re-solves, over
  bands of ±15 % in friction, ±10 % in mass, and ±30 % in downforce.

Whenever new physics lands, a new property test lands with it. That is a hard rule of the project,
and not a nicety.

### 13.4 Performance and determinism

**Performance.** A QSS lap must solve in **≤ 50 ms** of wall-clock time, in a release build. The
assertion takes the median of eleven warmed solves, on the real Catalunya track; see
[`crates/outlap-qss/tests/catalunya.rs`](../crates/outlap-qss/tests/catalunya.rs). Both the direct
T0 path and the production path of T0 on the envelope are gated.

*Generating* the envelope is the documented cold step of assembly, and it is excluded from this
gate. It is a one-time cost at the scale of seconds; see Chapter 3, on the envelope cache.

Separately, gates on **zero allocation** use the `dhat` allocator. They assert that the kernels of
the hot loop — `solve_into`, the trim solve, the advance of the slow states, `Mf61::forces`, and
`GriddedMapN::eval` — allocate no heap memory, across warmed calls.

A gate on instruction count, through `iai-callgrind`, is specified but deferred, until a CI runner
equipped with valgrind exists. The gate on allocation covers the same kernels in the meantime.

**Determinism.** outlap is built to give bit-identical results on the same platform, for the same
inputs. Across platforms, its results are exact within a documented tolerance.

In a production path it uses fixed-step integrators only. The outer iteration count for the coupling
of slow states is **fixed**, and not driven by a tolerance, "for determinism"; that is
`OUTER_ITERS = 2`, in [`crates/outlap-qss/src/qss.rs`](../crates/outlap-qss/src/qss.rs). By
contrast, the Newton method of the trim at each station converges to a scaled-residual tolerance of
`1e-10`, and only a fallback budget of iterations caps it. Reductions run in a fixed order. And
there is no fast-math.

The recorded `fz_coupling` mode, either `one_step_lag` or `fixed_point`, is part of the input. Two
runs that differ only in that setting are each individually reproducible.

Determinism is not just tidiness. It is a prerequisite for the future Monte Carlo layer for race
strategy, which will run the fast tier thousands of times, and needs to trust that a given seed
always yields the same lap.

The test `test_lap_is_deterministic`, in
[`python/tests/test_core.py`](../python/tests/test_core.py), checks this end to end.

### 13.5 The CI pipeline

Every push and every pull request runs
[`.github/workflows/ci.yml`](../.github/workflows/ci.yml). It has four jobs.

1. **rust.** It runs `cargo fmt --check`, `cargo clippy -- -D warnings`, and
   `cargo test --workspace`. It runs the performance gates, which are release-only. It runs the
   schema check, where `gen_schemas --check` confirms that the committed `schemas/*.json` still
   match the Rust types. And it builds the wasm-facing crates for `wasm32-unknown-unknown`.
2. **python.** It builds the extension in *release*, because a debug wheel would blow the budget for
   test time. It then runs `ruff check`, `ruff format --check`, `pyright` in strict mode, `pytest`,
   and `python -m outlap.schemas --check`.
3. **notebooks.** It re-executes every notebook headless, with `jupyter execute`. The notebooks
   therefore double as end-to-end tests, and their in-notebook assertions must pass. Two of those
   are the 0.5 % tire gate, and the check that the racing line beats the center line.
4. **wheels.** It runs on a release tag only, and builds the distributable wheel.

### 13.6 What is *not* validated yet, stated honestly

A trustworthy tool is clear about its limits. At v0.2.5:

- **There is only one track with a published oracle.** The quantitative cross-check is Catalunya,
  against PL2014. Other tracks and vehicles are checked for internal consistency, and for plausible
  magnitudes. They are not checked against a measured lap time.
- **The gate on lap time is recorded, and not enforced.** The honest ambition of ≤ 1 % on lap time
  was measured against the transient tier, and against the time-weighted line, when they landed.

  Even together they do not close it. The T2 lap on this geometry is about +28 % over the
  optimal-control oracle. The corner stability margin of the driver dominates, at about +14 % of T0.
  On top of that sit the geometry, at about 5 percentage points; the floor of about 2.2 s between
  quasi-steady state and optimal control that the paper itself cites; and about 1.5 s of
  conservatism in the envelope.

  The full decomposition, and the wide tripwire that keeps the recorded band from drifting silently,
  live in `docs/validation/limebeer.md`.
- **The thermal model and the battery are validated against synthetic fixtures and closed-form
  solutions, and not against measured hardware.** The tests on pulse response and on energy closure
  confirm that the *math* is right. They do not claim that the *parameters* of any shipped car are
  measured. They are estimates; see Chapter 12.
- **The parity gate between QSS and transient is split into what holds and what is recorded.**

  The *asserted* gate is **hull containment**. Every operating point $(a_x, a_y)$ of a closed-loop
  T2 lap must stay inside the T1 g-g-g-v envelope. An exceedance of up to 2 % is allowed, and the
  measurement is **0.0 %** on all three reference cars, in both the Rust harness and
  `python/tests/test_parity.py`. The transient physics therefore never produces grip that the
  quasi-steady physics says it cannot have.

  The figures for parity on lap time and apex speed, at 0.3 % and 1 %, are *recorded, and not
  asserted*. The corner stability margin of the T2 driver puts the lap about 14 % to 17 % away from
  T0, with top speeds within a few percent. That is a gap in the driver's competence at the limit,
  and not an error in the physics.

  A golden transient lap, indexed by time, pins the whole T2 trajectory against silent drift. The T2
  step allocates nothing, which dhat gates. And its throughput is recorded, with a tripwire for
  regression.
- **Estimated values are everywhere, and that is by design.** But it means that you must read the
  loaded-model report. Every estimate is listed there, and a lap run in degraded mode marks its
  results. Nothing is hidden. But nothing is measured-perfect either.

The guiding principle throughout: an estimated or simplified value always surfaces in the
loaded-model report and in the `notes` of the lap. You are therefore never misled about what the
numbers rest on.

Chapter 12 tells you the provenance of each shipped asset. Chapter 10 shows you how to read the
report and the notes, from Python.


---

## 14. Recipes: worked examples

*What you will learn: eight end-to-end tasks. Each is a complete runnable script, with real output.
These are the answers to "how do I actually do X?": building your own car, comparing solver tiers,
sweeping motor sizes, importing a track, feeding in your own powertrain, and reading the channels
for thermal state, battery, and envelope. Read Chapters 3, 4, and 10 first; everything here builds
on them.*

Every code block below imports from `outlap.core`. Run each one the same way, from the `python/`
directory of a built checkout:

```bash
cd python
uv run --no-sync python your_script.py
```

Two conventions run throughout. Paths are written relative to `python/`, so `../data/...` reaches
the data of the repository. And the examples use a **fast, coarse envelope grid**, so that they
finish in a second or two:

```python
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}
```

The production default grid is $40\times25\times7$; see Chapter 8. It is more accurate, but it takes
longer to generate.

A coarse grid gives a lap time a few tenths off the fine-grid value. That is fine for exploring, and
not fine for a headline number.

Recipes A and B below show *real, executed* output. The lap times are exact for the fast grid. The
numbers that you get on the production grid will differ slightly.

### 14.1 Recipe A: build your own vehicle from scratch

The fastest way to author a car is to copy the closest reference vehicle, and edit it. Here we make
a lighter, draggier EV from the Model 3.

**Step 1: copy the vehicle directory.** Work outside the repository, so that you do not dirty it:

```bash
cp -r data/vehicles/tesla_model3_rwd /tmp/my_ev
```

The directory carries everything the car references: `vehicle.yaml`, the drive-unit maps in `ptm/`,
`tyr/road.tyr.yaml`, `battery/`, and `emotor/`. See Chapter 4.

**Step 2: edit `vehicle.yaml`.** Change the mass, and the drag area. In `/tmp/my_ev/vehicle.yaml`:

```yaml
chassis:
  mass_kg: 1500.0        # was 1765.0
aero:
  constant:
    cx_a_m2: 0.62        # was 0.51 — a boxier body
```

**Step 3: check the loaded-model report**, before you trust anything. What you want is a clean load,
with the estimates noted:

```python
from outlap.core import vehicle_report
rep = vehicle_report("/tmp/my_ev")
print("name      ", rep["name"])
print("estimated ", len(rep["estimated"]))
print("warnings  ", len(rep["warnings"]))
print("degraded  ", len(rep["degraded"]))
```

Here is the real output:

```text
name       Tesla Model 3 RWD (HV variant)
estimated  10
warnings   0
degraded   0
```

There are ten estimated values, unchanged from the parent car, because you edited only fields from
the spec sheet. There are zero warnings, and nothing is degraded.

Change the `name:` field in your copy, so that the report reflects your car. The value above is
inherited, because we edited only the mass and the aero.

**Step 4: lap both cars, and compare.**

```python
from outlap.core import Track, min_curvature, solve_lap_dataset
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}

trk  = Track.load("../data/tracks/catalunya")
line = min_curvature(trk, half_width_m=0.95)

stock = solve_lap_dataset("../data/vehicles/tesla_model3_rwd", line, tier="t1", sim=FAST)
mine  = solve_lap_dataset("/tmp/my_ev", line, tier="t1", sim=FAST)

a = float(stock.attrs["lap_time_s"])
b = float(mine.attrs["lap_time_s"])
print(f"stock Model 3 RWD  : {a:.2f} s")
print(f"my_ev (1500kg,0.62): {b:.2f} s")
print(f"delta              : {b - a:+.2f} s")
```

Here is the real output:

```text
stock Model 3 RWD  : 145.33 s
my_ev (1500kg,0.62): 141.61 s
delta              : -3.72 s
```

Dropping 265 kg beat the extra drag. The lighter car is 3.72 s quicker over the lap.

Note that the resolved hash differs between the two datasets, in `attrs["resolved_hash"]`. The two
cars are genuinely distinct resolved specs, and not the same car twice.

> **A shortcut, with no file editing.** For a quick what-if, you do not even need to copy files.
> Pass `overrides={"chassis.mass_kg": 1500.0, "aero.constant.cx_a_m2": 0.62}` to
> `solve_lap_dataset`.
>
> The override goes through the *real* validation pipeline. A bad path, or a value out of range,
> fails loudly. And it changes the resolved hash.
>
> Copy-and-edit is for a car that you want to keep. An override is for an experiment.

### 14.2 Recipe B: compare the T0 and T1 tiers

The tiers answer different questions; see Chapter 8. T0 is the point-mass velocity profile. T1
re-trims the double-track car at every station, for per-wheel detail.

Here is the fact that surprises people at first: **on the same car and the same line, they return
the same lap time.** The re-trim of T1 runs on the profile that T0 produced.

```python
import numpy as np
from outlap.core import Track, min_curvature, solve_lap_dataset
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}

trk  = Track.load("../data/tracks/catalunya")
line = min_curvature(trk, half_width_m=0.95)
vd   = "../data/vehicles/tesla_model3_rwd"

t0 = solve_lap_dataset(vd, line, tier="t0", sim=FAST)
t1 = solve_lap_dataset(vd, line, tier="t1", sim=FAST)

print("t0 lap_time_s", round(float(t0.attrs["lap_time_s"]), 2))
print("t1 lap_time_s", round(float(t1.attrs["lap_time_s"]), 2))
print("t0 channels  ", list(t0.data_vars))
print("t1 channels  ", list(t1.data_vars))
```

Here is the real output:

```text
t0 lap_time_s 145.33
t1 lap_time_s 145.33
t0 channels   ['v', 'ax', 'ay', 't', 'x', 'y', 'z', 'state_of_charge', 'machine_temp_c']
t1 channels   ['v', 'ax', 'ay', 't', 'x', 'y', 'z', 'vertical_load_n', 'slip_ratio', 'slip_angle_rad', 'force_long_n', 'force_lat_n', 'understeer_gradient', 'aero_front_share', 'state_of_charge', 'machine_temp_c']
```

Both laps are 145.33 s.

What T1 *adds* is the per-wheel detail. Those are the `(s, wheel)` channels — `vertical_load_n`,
`slip_ratio`, `slip_angle_rad`, `force_long_n`, and `force_lat_n` — plus the setup metrics
`understeer_gradient` and `aero_front_share`.

Both tiers carry the slow-state channels `state_of_charge` and `machine_temp_c` here, because this
is an electrified car with a live stack for battery and thermal state. Those channels gate on the
powertrain, and not on the tier.

Now inspect the wheel loads at the fastest point of the lap:

```python
v = t1["v"].values
i = int(np.nanargmax(v))
print(f"at v = {v[i]:.1f} m/s, Fz [FL FR RL RR] =",
      np.round(t1["vertical_load_n"].values[i], 0))
```

Here is the real output:

```text
at v = 64.1 m/s, Fz [FL FR RL RR] = [3987. 4618. 4066. 4638.]
```

The wheel order is always `["FL", "FR", "RL", "RR"]`, which is front-left, front-right, rear-left,
and rear-right. It is available as `t1.coords["wheel"]`.

Here the car is at high speed, in a gentle right-hand curve. The left wheels therefore carry
slightly less load than the right.

Use these channels to plot the load at each wheel through a corner, to check which wheel saturates
first, or to compute your own metric of tire usage.

### 14.3 Recipe C: sweep the motor sizing

The Model 3 ships three synthetic sizings for its drive unit: `du_small`, at about 100 kW;
`du_medium`, at about 203 kW, which is the default; and `du_large`, at about 248 kW. You can
therefore study how motor power buys lap time.

You do **not** edit files. You override the source map of the drive unit:

```python
from outlap.core import Track, min_curvature, solve_lap_dataset
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}

trk  = Track.load("../data/tracks/catalunya")
line = min_curvature(trk, half_width_m=0.95)
vd   = "../data/vehicles/tesla_model3_rwd"

prev = None
for size in ["du_small", "du_medium", "du_large"]:
    ov = {"drivetrain.units.0.source": f"ptm/{size}.ptm.yaml"}
    d  = solve_lap_dataset(vd, line, tier="t1", sim=FAST, overrides=ov)
    lt = float(d.attrs["lap_time_s"])
    tag = "" if prev is None else f"  ({lt - prev:+.2f} s vs previous)"
    print(f"{size:<10} {lt:.2f} s{tag}")
    prev = lt
```

Here is the real output:

```text
du_small   155.84 s
du_medium  145.33 s  (-10.51 s vs previous)
du_large   143.32 s  (-2.01 s vs previous)
```

The story is **diminishing returns**. The jump from small to medium is worth 10.51 s. Medium to
large is worth only 2.01 s.

Past a point, more torque no longer helps. The tires, the thermal derate of the machine, and the
power ceiling of the pack become the limits. The rated torque of the motor does not.

This is exactly the sensitivity axis that notebook 07 explores. On the production grid, the deltas
are −13.41 s and −2.67 s.

See Chapter 12 for the provenance of these three sizings. They are invented smooth surfaces, and not
measured maps.

### 14.4 Recipe D: import and lap a track from the databases

outlap ships 26 tracks; see Chapter 12. You can add your own. The two importers are one-time local
tools; see Chapter 11.

**From the TUMFTM database**, which holds flat 2-D center lines:

```bash
cd python
uv run --no-sync python -m outlap.importers.tumftm_track \
    --input /path/to/tumftm/racetrack-database/tracks/Zolder.csv \
    --out ../data/tracks/zolder
```

**From OpenStreetMap plus elevation**, which gives a full 3D ribbon. It needs the `track-import`
extra, for the dependencies on network and elevation:

```bash
uv run --extra track-import python -m outlap.importers.osm_track \
    --preset catalunya --out ../data/tracks/my_catalunya
```

Then load it and lap it, exactly as you would a shipped track.

Any of the 26 vendored tracks works out of the box. Here are two TUMFTM circuits, with the stock
Model 3:

```python
from outlap.core import Track, min_curvature, solve_lap_dataset
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}
vd = "../data/vehicles/tesla_model3_rwd"
for name in ["silverstone", "monza"]:
    trk  = Track.load(f"../data/tracks/{name}")
    line = min_curvature(trk, half_width_m=0.95)
    d    = solve_lap_dataset(vd, line, tier="t1", sim=FAST)
    print(f"{trk.name():<28} {trk.length():.0f} m   lap {float(d.attrs['lap_time_s']):.2f} s")
```

Here is the real output:

```text
Silverstone Circuit          5887 m   lap 166.61 s
Autodromo Nazionale Monza    5790 m   lap 141.77 s
```

Remember that the vendored TUMFTM tracks are **flat**: `z = 0`, `banking = 0`, and accuracy class C;
see Chapter 12. They are redistributed under LGPL-3.0, with the required attribution. Only
`catalunya_osm` carries real elevation.

### 14.5 Recipe E: bring your own powertrain, through the PDT importer

If you have an HDF5 export from a professional drive tool, PDT, the importer distills it into the
neutral `.ptm` maps of outlap.

This is a **local-only workflow, and it respects the firewall**; see Chapters 1 and 11. You never
commit the raw `.h5` source, and you never commit the derived `.ptm`. The reference vehicles keep
their real imports in a git-ignored `local/` directory.

```bash
cd python
# Drive unit → .ptm (+ maps.parquet sidecar), full Vdc stack
uv run --no-sync python -m outlap.importers.pdt_h5 driveunit \
    ~/pdt_reference/DriveUnit_9.3GR_2765NM_1938RPM_outlap.h5 \
    -o ../data/vehicles/tesla_model3_rwd/local/du_medium.ptm.yaml

# Battery pack → battery.yaml (+ tables.parquet)
uv run --no-sync python -m outlap.importers.pdt_h5 batterypack \
    ~/pdt_reference/BatteryPack_220S_1P_64064Wh_704V_e884f_outlap.h5 \
    -o ../data/vehicles/tesla_model3_rwd/local/pack.battery.yaml
```

Then point your vehicle, which is also local, at the imported files, and lap it.

`local/` is git-ignored, so none of this leaves your machine. The importer reads only clean,
documented HDF5 fields, through `h5py`. It never imports the code of PDT; that is the firewall.

The output above is illustrative. The exact commands and file names are in the `README.md` of
`tesla_model3_rwd`, and you need the real `.h5` inputs to run them.

### 14.6 Recipe F: study the derating from machine temperature

The Model 3 carries a machine thermal network; see Chapter 9. A lap therefore reports the winding
temperature, and the resulting derate on torque.

Read the `machine_temp_c` channel over the lap:

```python
import numpy as np
from outlap.core import Track, min_curvature, solve_lap_dataset
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}

trk  = Track.load("../data/tracks/catalunya")
line = min_curvature(trk, half_width_m=0.95)
d    = solve_lap_dataset("../data/vehicles/tesla_model3_rwd", line, tier="t1", sim=FAST)

temp = d["machine_temp_c"].values
print(f"machine winding: start {temp[0]:.1f} °C, peak {np.nanmax(temp):.1f} °C")
```

Here is the real output, for a single lap from a cold start:

```text
machine winding: start 20.0 °C, peak 127.5 °C
```

The winding starts at ambient, which is 20 °C. It climbs toward its warning threshold, which is
150 °C for this car; see its `emotor/rear_du.emotor.yaml`, as heat soaks in.

If it reached the warning band, the derate on torque would begin to cap the traction ceiling. That
derate is a linear ramp from 1 to 0, from `t_warn_c` to `t_max_c`; see Chapter 9.

To see meaningful heat soak, you want a *long* run. Lap a longer track, such as Silverstone or Spa,
or lap the same track repeatedly. The temperature carries forward from station to station, because
it is a slow state.

The units are °C at this display boundary. The internals are in kelvin.

### 14.7 Recipe G: watch battery SoC, and the Vdc coupling

The same electrified stack reports the state of charge of the pack, as it drains.

And because this is an "HV variant" in the 800 V class, whose pack voltage sags below the voltage
grid of the drive unit at low charge, it exercises the Vdc–SoC coupling; see Chapter 9.

```python
from outlap.core import Track, min_curvature, solve_lap_dataset
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}

trk  = Track.load("../data/tracks/catalunya")
line = min_curvature(trk, half_width_m=0.95)
d    = solve_lap_dataset("../data/vehicles/tesla_model3_rwd", line, tier="t1", sim=FAST)

soc = d["state_of_charge"].values
print(f"SoC: start {soc[0]:.3f}, end {soc[-1]:.3f}  (drop {soc[0]-soc[-1]:.3f})")
```

Here is the real output:

```text
SoC: start 0.980, end 0.904  (drop 0.076)
```

The pack loses about 7.6 % of its charge over one lap of Catalunya, net of regeneration.

As SoC falls, the terminal voltage of the pack falls with it. When that voltage drops below the Vdc
grid of the drive unit, the machine maps are evaluated by linear extrapolation along the voltage
axis, with physical floors. Any extrapolated band is recorded in the `notes` of the lap.

The peak-power limit of the battery, and the thermal derate, both act as `min` caps on the traction
ceiling. Neither is baked into the envelope at the reference state.

To see the coupling bite hard, start from a lower SoC, or run a long stint, so that the pack voltage
sags well below the grid.

### 14.8 Recipe H: extract the g-g-g-v envelope

The envelope of Chapter 8 is a first-class object that you can return. Call `solve_lap`, and not the
`_dataset` variant, to get a `Lap`. Then query its `.envelope`:

```python
from outlap.core import Track, min_curvature, solve_lap
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}

trk  = Track.load("../data/tracks/catalunya")
line = min_curvature(trk, half_width_m=0.95)
lap  = solve_lap("../data/vehicles/tesla_model3_rwd", line.line(), tier="t1", sim=FAST)

env = lap.envelope
print("shape   ", env.shape())
print("domain  ", [[round(x, 2) for x in ax] for ax in env.domain()])
print("mass_ref", env.mass_ref())
print("ay_boundary(50 m/s, ax=0, g_normal=9.81) =", round(env.ay_boundary(50, 0, 9.81), 2), "m/s²")
print("accel_limit(30 m/s, 9.81)                =", round(env.accel_limit(30, 9.81), 2), "m/s²")
print("brake_limit(50 m/s, 9.81)                =", round(env.brake_limit(50, 9.81), 2), "m/s²")
```

Here is the real output:

```text
shape    [8, 7, 2]
domain   [[5.0, 67.0], [-1.0, 1.0], [4.9, 19.61]]
mass_ref 1765.0
ay_boundary(50 m/s, ax=0, g_normal=9.81) = 8.07 m/s²
accel_limit(30 m/s, 9.81)                = 7.09 m/s²
brake_limit(50 m/s, 9.81)                = 11.67 m/s²
```

The three axes of the envelope are speed $v$, normalized longitudinal acceleration
$\hat a_x \in [-1, 1]$, and normal gravity $g_{\text{normal}}$; see Chapter 8.

`ay_boundary` gives the maximum lateral acceleration available at a query point. `accel_limit` and
`brake_limit` give the longitudinal limits, net of drag.

This is the object that the T0 solver consumes. Pulling it out lets you plot the grip surface of the
car, compare the envelopes of two cars, or feed a downstream analysis.

For a surface of production quality, run with the default grid, instead of `FAST`. Here `shape`
reflects the coarse `[8, 7, 2]` that we asked for.

### 14.9 Where to go from here

These recipes cover the core loop: author, solve, inspect, and sweep. Combine them freely.

The sweep over sizing in Recipe C, together with the thermal reading of Recipe F, shows *why* the
big motor stops helping. The envelope of Recipe H, over two cars from Recipe A, shows *how* their
grip differs.

Notebook 07, at `notebooks/07_qss_t1.ipynb`, is a longer version of this same material, with plots,
on the F1 car and the Model 3. Notebook 08 drives the same cars through time, at the transient tier.
And notebook 09 turns the traces into studies in race engineering: the anatomy of a corner, and car
balance.

The theory pages in `docs/theory/` give the equations behind each channel.


---

## 15. Limitations, history, and roadmap

*What you will learn: an honest account of what outlap v0.2.5 does **not** do, or does with recorded
caveats; how the project got here, release by release; and the planned path forward. Knowing the
boundaries is as important as knowing the features. It tells you when a number can be trusted, and
when you are outside the design envelope of the tool.*

### 15.1 What v0.2.5 does not do

outlap v0.2.5 is a complete quasi-steady-state simulator, in T0 and T1, plus a first transient tier,
T2, whose physics is validated but whose driver is limited.

Here is the honest list of boundaries.

- **The T2 driver is stable. It is not at the limit.** The transient tier integrates the full 7-DOF
  closed loop (§8.7), behind a corner-scaled stability margin: the full QSS speed on the straights,
  about 0.85 of it at the lateral grip limit, and wheelspin governed by the driver's pedal. Pushed
  to the raw profile, it spins.

  A T2 lap is therefore about 14 % to 17 % slower than T0 and T1, almost entirely in the corners,
  while its straight-line speeds match the profile to a few percent.

  The parity in physics is *gated*, through hull containment; see Chapter 13. The parity in pace is
  *recorded*.

  Use T2 for the time-domain story — traces, shifts, and transient loads. Do not use it as a faster
  oracle for lap time. A driver at the limit is the next step for the tier.
- **T3 does not exist.** The model with 14 degrees of freedom — suspension travel, unsprung masses,
  dampers, and curb strikes — raises a typed "not implemented" error.

  T2 is a model with a rigid chassis. It has no suspension states, so a sharp crest is handled with
  a documented floor on normal load, rather than with real suspension travel.
- **Tire thermal state and wear are live, from v0.3, and opt-in for each solve.** With
  `tire_thermal=True`, the tires heat, through nodes for surface, carcass, and gas. They gain and
  lose grip through a window in temperature. They wear, by Archard. And they fall off a cliff. All
  of that marches as slow states, in every tier, and `outlap.wearcal` calibrates it.

  With the option off, which is the default for a single lap, the tires use a fixed coefficient of
  friction, at a reference pressure and camber. A lap with frozen tires therefore stays
  byte-identical.

  The *machine* thermal network of Chapter 9 is a separate model, for the electric drive unit.
- **Fuel mass is constant.** There is no slow state for fuel burn, so a combustion car does not get
  lighter over a lap.
- **The ERS energy manager is a power cap, plus regeneration.** Regenerated energy flows into the
  pack: T2 blends regen braking live, and the march of the slow states carries signed pack energy.

  But the *scheduler* for deployment and harvest, lap by lap — when to spend, when to save, the
  taper on deployment, and override modes — is future work.
- **There is no layer for race strategy.** The Monte Carlo simulation of race strategy is the
  long-term goal that motivates the fast, deterministic T0 tier. It is a stage after 1.0.

  It is why determinism, and the lap under 50 ms, matter now. But it is not here yet.
- **Only the F1 car has an aero map.** Only `f1_2026` ships a downforce map over ride height and
  yaw. Every other vehicle uses the degenerate path, with constant $C_dA$ and $C_zA$; see Chapter 7.
  That is correct for a road car, and a simplification for a car with high downforce.
- **Most vendored tracks are flat.** The 25 TUMFTM circuits carry `z = 0`, `banking = 0`, and
  `grip_scale = 1`, at accuracy class C; see Chapter 12.

  Two circuits have real elevation: `catalunya_osm`, at about 30 m, and `spa_osm`, at about 108 m.
  But on both, the widths are defaulted, and the banking is unresolved.

  The physics of grade and vertical curvature is implemented, and exercised. The physics of
  *banking* is implemented, and no shipped asset exercises it.
- **Multi-lap stints are live, from v0.3. Fuel mass is not.** `solve_stint` and
  `solve_transient_stint`, and `solve_stint_dataset`, run a stint that carries the slow state of
  tire thermal state and wear from lap to lap.

  The loss of mass from burning fuel is still a future slow state. And batch sweeps, plus a CLI, are
  the items on the road to 1.0, below.
- **The class presets are not shipped.** `data/presets/` is reserved, and empty. The promised
  starting points, `formula_base`, `gt_base`, and `passenger_base`, are a future deliverable.

  For now, author a new car by copying a reference vehicle; see Chapter 14, Recipe A.

None of these is hidden. Every simplification that a given lap makes is listed in the `notes`
attribute of that lap. And every estimated parameter is in the loaded-model report; see Chapters 10
and 13.

### 15.2 What "accuracy class C" means for a track

Each track records a `meta.accuracy_class`. It is a grade for provenance. It is not a guarantee of
precision.

- **Class B**, which covers `catalunya_osm` and `spa_osm`, means the track was built from
  OpenStreetMap geometry, fused with open elevation data. It has a real 3D ribbon. But the widths of
  its corridor are defaulted, and its banking is not resolved from the coarse public elevation
  model.
- **Class C**, which covers the 25 TUMFTM circuits, means smoothed center lines, with corridor
  widths measured from satellite images, but strictly in 2-D. It is the standard academic bootstrap
  dataset. It is good for relative comparisons, and not for matching a real lap record.

A class-C track will give you lap times that are plausible and self-consistent. Those are useful for
comparing cars, or comparing setups. But you should not expect them to match a measured lap. The
smoothed geometry alone can shift a corner speed by several percent; see Chapter 13.

### 15.3 How outlap got here: the releases so far

outlap is built in planned increments. Each ships as a set of small, gated pull requests, and closes
with a tagged release.

This section is the one place in the guide for that history. The rest of the book describes *what
is*, and not *when it landed*. A roadmap can change, and does. Treat the future rows as intent, and
not as commitment.

**Shipped:**

| Release | Headline | What landed |
|---------|----------|-------------|
| v0.1 | **The foundation** | The schema contract (JSON Schemas generated from the Rust types), the 3D track ribbon, the OSM+DEM track importer, the minimum-curvature racing line, and the T0 point-mass lap solver. |
| (unversioned) | **The tire** | The clean-room MF6.1 implementation, the `.tir` codec, the Python fitting pipeline, and the citation-backed `.tyr` datasets — validated against an independent oracle to a 0.5 % CI gate. |
| v0.2.0 | **The full QSS tier** | The T1 double-track trim and the g-g-g-v envelope; aero maps; the topology powertrain over `.ptm` files; machine-thermal derating; the battery/Vdc coupling marched as slow states; the Limebeer cross-check. |
| v0.2.5 | **The transient tier** *(this release)* | The T2 tier end to end: the 7-DOF curvilinear road-frame chassis (symbolically verified to 1e-12), the split fixed-step integrator, tire relaxation, the MacAdam-preview driver, the shift/torque-vectoring/regen control layer, the full 3D road frame; plus the time-weighted racing line, the `spa_osm` 3D import, the QSS↔T2 hull-containment parity gate, and the T2 capstone notebooks. |

Two notes on scope are part of the record of v0.2.5. `docs/validation/limebeer.md` decomposes both,
rather than hiding them.

The ambition of ≤ 1 % on the Limebeer lap time was measured, and **not** achieved. The corner
stability margin of the T2 driver dominates. That is a gap in the driver's competence at the limit,
and not an error in the physics.

And the original target for throughput of the transient step proved unreachable, at full fidelity of
the Magic Formula. The measured number is recorded, and tripwired, instead.

An ambition that misses is recorded with its reasons. That is the validation culture of the project,
from Chapter 13, applied to itself.

**Planned. This is intent, and not commitment.**

| Target | Headline deliverable |
|--------|----------------------|
| v0.3 ✅ shipped | **Tire thermal ring + wear** in all tiers — grip that depends on temperature, tires that go off, stint-pace inverse calibration, and the multi-compound stint demo |
| next | Full ERS-style deploy/harvest energy manager + battery ECM + **fuel mass** + **T3** (14-DOF suspension) |
| v1.0 | Batch/sweep API (parallel, structure-of-arrays) + CLI + all four reference vehicles + the hero demo + docs site + a WebAssembly demo widget |

Beyond 1.0, the recorded intent is fourfold. Importers for telemetry from sim racing — MoTeC, ACC,
and iRacing — for community data and for validation. The **Monte Carlo layer for race strategy**: a
time-discrete race simulation, with a stochastic layer and an optimizer for strategy, running on the
fast T0 tier with its slow states. A browser app, `outlap-web`, grown from the WASM widget. And a
community data registry.

The core discipline that makes all of this possible is in place now: one vehicle description across
tiers, a deterministic hot loop that allocates nothing, and honest reporting.

---

## 16. Glossary

*Terms of art used throughout this guide and the codebase, in alphabetical order. Each entry is one
sentence, and the chapter that treats it in depth is noted where that helps.*

- **Aero balance** — the fraction of total aerodynamic downforce carried by the front axle; it
  shifts the car toward understeer or oversteer (Chapter 7).
- **Aligning moment ($M_z$)** — the self-centering torque that a tire generates about its vertical
  axis, which gives steering its feel and its feedback (Chapter 7).
- **Anti-dive / anti-squat** — suspension geometry that resists the nose dipping under braking, in
  anti-dive, or the tail squatting under acceleration, in anti-squat; both are quantified as a
  fraction (Chapter 4).
- **Assembly pipeline** — the one-time phase at load time that reads YAML, merges inheritance,
  validates, estimates missing values, and builds the solver's vehicle object; it is distinct from
  the hot loop, and it may allocate and do heavy work (Chapter 6).
- **CdA / $C_xA$** — drag area: the drag coefficient times the frontal area, in m². Drag force is
  $\tfrac12\rho\,C_xA\,v^2$ (Chapters 2, 7).
- **ClA / $C_zA$** — lift, or downforce, area: the negative lift coefficient times area, in m².
  Downforce grows with the square of speed (Chapters 2, 7).
- **Clean-room** — the rule of the project that every physics model is re-authored from published
  literature, with citations, and never copied from another codebase (Chapters 1, 13).
- **Combined slip** — a tire braking or driving and cornering at the same time, so that its
  longitudinal and lateral forces share one budget of friction, which is the friction circle
  (Chapters 2, 7).
- **Contact patch** — the small area where a tire touches the road, and through which every force
  for driving, braking, and cornering is transmitted (Chapter 2).
- **Damped Newton (trim solve)** — the iterative root-finder that solves the T1 balance of forces
  and moments at each station, with damping from a line search for robustness (Chapter 8).
- **Degraded mode** — a fallback path at load, through `allow_degraded: true`, that lets an
  otherwise unsupported configuration run, and marks the results as degraded (Chapters 4, 13).
- **Determinism** — the guarantee that the same inputs give the same outputs, bit-exact on one
  platform, through fixed-step integration, fixed iteration counts, and reductions in a fixed order
  (Chapter 13).
- **Double-track model** — a car model in which all four wheels are represented individually, as
  opposed to a single-track or "bicycle" model, so that load transfer between left and right, and
  the force at each wheel, are resolved; that is the T1 tier (Chapters 2, 8).
- **Firewall** — the hard rule that a powertrain enters only as a neutral `.ptm` map file, and never
  as an internal model of a machine, an inverter, or a gearbox; the machine thermal network is a
  narrow, documented exception (Chapters 1, 9).
- **Flat-track mode** — an analysis setting, `sim.flat_track`, that zeroes grade, banking, and
  vertical curvature, so that the g-g-g-v envelope collapses to a flat g-g. It is used for the 2D
  Limebeer comparison (Chapters 8, 13).
- **Friction circle / ellipse** — the boundary of the combined longitudinal and lateral force that a
  tire can produce. Staying inside it is the fundamental constraint on grip (Chapters 2, 7).
- **`fz_coupling`** — the recorded choice of how the algebraic loop on normal load is resolved:
  `one_step_lag`, which is the default and uses the loads from the previous step, or `fixed_point`,
  which iterates to convergence (Chapters 4, 8).
- **g-g diagram** — the set of longitudinal ($a_x$) and lateral ($a_y$) accelerations that a car can
  reach, drawn as a 2D region. It is the classic performance envelope (Chapter 2).
- **g-g-g-v envelope** — the g-g diagram, extended by a dependence on speed ($v$) and on normal
  gravity ($g_{\text{normal}}$, which captures banking and crests). It is a precomputed grip
  surface, which the T0 solver consumes (Chapter 8).
- **Golden file** — a committed reference output, which a test recomputes and compares against, and
  which fails on a drift beyond a tolerance (Chapter 13).
- **`GriddedMapN`** — the N-dimensional gridded-map type of outlap. The one shared monotone cubic
  interpolant evaluates it. It is used for an aero map, for a `.ptm` table, and for the envelope
  (Chapters 5, 8).
- **Hot loop** — the inner computation, for each station or for each timestep, which must allocate
  nothing, contain no Python, and use state of fixed size. It is distinct from the assembly pipeline
  (Chapter 6).
- **Hull containment** — the asserted parity gate between QSS and T2: every operating point
  $(a_x, a_y)$ of a transient lap must lie inside the T1 g-g-g-v envelope (Chapter 13).
- **ISO 8855** — the convention for vehicle axes that outlap uses: $x$ forward, $y$ left, $z$ up. It
  fixes the sign of every force, slip, and acceleration (Chapters 2, 6).
- **Loaded-model report** — the dictionary that `vehicle_report` returns. It lists every value that
  was inherited, estimated, degraded, warned about, or overridden. It is the "nothing silent"
  surface (Chapters 4, 10).
- **Load sensitivity** — the fact that the friction *coefficient* of a tire falls as its vertical
  load rises, so that doubling the load less than doubles the grip (Chapter 2).
- **Load transfer** — the shift of vertical load between wheels, under acceleration, braking, and
  cornering. It is longitudinal, between front and rear, and lateral, between left and right
  (Chapters 2, 8).
- **LPTN (lumped-parameter thermal network)** — the thermal model of a machine: a small network of
  thermal masses, called nodes, connected by conductances, and integrated on each lap segment
  (Chapter 9).
- **MacAdam preview driver** — the steering model of the T2 tier. It looks ahead a preview distance
  that scales with speed, on the target line, and steers to null the predicted error, plus a
  feedforward on curvature. It follows MacAdam 1981 (Chapter 8).
- **Magic Formula / MF6.1** — the empirical model of tire force, in the version 6.1 of Pacejka. It
  fits measured tire data with a characteristic curve, a sine of an arctangent (Chapters 2, 7).
- **Minimum-curvature line** — the racing line that minimizes the integrated squared curvature
  within the track corridor, found by a quadratic program. It is not the same as the time-optimal
  line (Chapters 2, 8).
- **Monotone cubic Hermite** — the single interpolation scheme, from Fritsch and Carlson, used for
  *all* gridded maps. It is smooth, at $C^1$, and it preserves shape, so it never overshoots between
  data points (Chapters 5, 8).
- **`one_step_lag`** — see `fz_coupling`. It is the default, and cheaper, mode for coupling normal
  load (Chapters 4, 8).
- **Peak μ (peak friction coefficient)** — the maximum friction that a tire delivers, extracted from
  the Magic Formula curves at a reference load and pressure. The T0 tier distills a tire to this
  number (Chapter 7).
- **Point-mass model** — a car idealized as a single mass with a grip limit, ignoring individual
  wheels. That is the T0 tier (Chapters 2, 8).
- **Property test** — a test that asserts an invariant which must hold for *any* input, such as
  containment in the friction circle. It often runs across many randomized cases (Chapter 13).
- **`.ptm` file** — the neutral file for a powertrain map, which is YAML plus a parquet table in a
  sidecar. Every powertrain enters the simulator through it (Chapters 5, 9).
- **QSS (quasi-steady-state)** — the modeling assumption that the car is in instantaneous
  equilibrium at each point. It is the basis of the T0 and T1 tiers (Chapters 2, 8).
- **Racing line** — the path that the car actually drives through the corridor. It is distinct from
  the center line of the track (Chapters 2, 8).
- **Relaxation length** — the distance that a tire must roll before its force builds up to the
  steady-state value. The T2 tier integrates it live, through an exact-exponential lag on slip
  (Chapters 7, 8).
- **Ribbon** — the 3D track model of outlap: the road surface as a band, with curvature, grade, and
  banking parameterized by arc length (Chapters 2, 5).
- **Roll center** — the geometric point about which the sprung mass of a car rolls in a corner. Its
  height governs how much load transfer is geometric, and how much is elastic (Chapters 2, 8).
- **Sideslip angle (β)** — the angle between where the chassis points and where it travels, which is
  `atan2(v_y, v_x)`. It is the number for "the car rotated relative to its path" that a transient
  lap exposes (Chapter 8).
- **Slip angle ($\alpha$)** — the angle between where a tire points and where it is actually
  travelling. It is the source of lateral, or cornering, force (Chapters 2, 7).
- **Slip ratio ($\kappa$)** — the relative difference between the rolling speed of a tire and the
  speed of the road. It is the source of longitudinal, or drive and brake, force (Chapters 2, 7).
- **Slow state** — a quantity that evolves gradually across the lap, and carries between stations or
  timesteps. Machine temperature and battery SoC are examples. It is the opposite of the fast states
  of the chassis and the trim (Chapters 8, 9).
- **SoC (state of charge)** — the remaining charge of the battery, as a fraction from 0 to 1. It is
  a slow state, and it falls as the pack discharges (Chapter 9).
- **Speed margin (T2)** — the stability margin that the transient driver keeps at the lateral grip
  limit, which defaults to 0.85. It is applied corner-scaled: the full profile speed on the
  straights, the margin where grip is fully used, and transitions that are feasible on the ellipse.
  Every T2 result records it (Chapter 8).
- **Split integrator** — the fixed-step scheme of T2: Runge–Kutta for the chassis and controller
  states, exact-exponential updates for tire relaxation, and a decimated clock for the slow states
  (Chapter 8).
- **Thévenin (battery) model** — an equivalent-circuit model of a battery: the open-circuit voltage,
  minus resistive drops and drops across an RC network. It computes the terminal voltage under load
  (Chapter 9).
- **Tier (T0/T1/T2/T3)** — the ladder of solvers. T0 is a point mass, T1 is quasi-steady
  double-track, and T2 is closed-loop transient; all three ship. T3 is a 14-DOF transient model, and
  it is future work (Chapters 2, 8).
- **Time-weighted line** — the racing line from the minimum-curvature QP, re-weighted by the time
  spent at each station, where `w ∝ 1/v` from a speed pre-pass. It is faster than, or equal to,
  minimum curvature, by construction (Chapters 8, 10).
- **Trim** — the equilibrium state of the car at a given operating point: the steering, the body
  slip, and the wheel loads and slips that balance every force and moment (Chapter 8).
- **Understeer gradient** — a setup metric. It measures how much extra steering a car needs as
  lateral acceleration rises. Positive means understeer (Chapters 2, 8).
- **Vdc** — the DC-link voltage supplied to an electric drive unit. When a battery pack is present,
  the maps of the drive unit are evaluated at the terminal voltage of the pack, which depends on
  SoC. That is the Vdc–SoC coupling (Chapter 9).

---

## 17. FAQ and troubleshooting

*Common questions and errors, answered from the actual behavior of the tool. If something here
surprises you, the referenced chapter has the full story.*

**Why does `import outlap` give me almost nothing?**

The top-level `outlap` package is a stub. The real API lives in `outlap.core`. Always import from
there:
`from outlap.core import Track, min_curvature, solve_lap_dataset, vehicle_report`. See Chapter 10.

**Why does `tier="t2"` raise an error in `solve_lap`? Is T2 not shipped?**

It is shipped. But a transient lap is indexed by *time*, so it does not fit the result type of
`solve_lap`, which is indexed by arc length.

The error is a deliberate, typed redirect. Call `solve_transient_lap(...)` directly, or
`solve_lap_dataset(..., tier="t2")`, which routes for you and returns the time-indexed dataset. See
Chapters 8 and 10.

`tier="t3"`, which is the 14-DOF model with suspension, genuinely is not implemented yet, and it
raises a typed error.

**Why do T0 and T1 give the same lap time?**

Because T1 re-trims the double-track car *on the velocity profile that T0 already produced*. It adds
per-wheel loads, slips, forces, and setup metrics. It does not change the lap time, on the same car
and the same line.

If you want per-wheel detail, use `t1`. If you need only the lap time and the speed trace, `t0` is
enough. See Chapters 8 and 14.

**Why is the first lap so slow, and fast afterwards?**

The g-g-g-v envelope is generated once for each combination of car and grid. That is a cold step at
the scale of seconds in release, and minutes in a debug build. It is then cached for the rest of the
process, and a later lap of the same car reuses it.

If your first solve takes *minutes*, you have a debug wheel. Rebuild with
`MATURIN_PEP517_ARGS=--profile release` before `uv sync`. See Chapters 3 and 10.

**Why did `uv sync` break my notebooks?**

A plain `uv sync` uninstalls a dependency group that it was not told to keep.

Always include the group that you need: `uv sync --group notebooks --extra tire-fit`. See Chapter 3.

**Why are there estimated values in my loaded-model report? How do I get rid of them?**

A reference vehicle carries documented estimates, for parameters that are not on a public spec
sheet. Inertias, roll centers, and ride rates are examples; see Chapter 12.

They are *surfaced*, and not hidden. A car that loads with estimates and zero warnings is healthy.

To pin a value, set it explicitly in your `vehicle.yaml`. It will then leave the `estimated` list.
See Chapters 4 and 14.

**What does "degraded" mean, and why are my results marked?**

Degraded mode, through `allow_degraded: true`, is the single fallback path, for a configuration that
outlap cannot fully support. It lets the run proceed, and marks the results, so that you know they
rest on a fallback.

If you did not opt in, you will not see a degraded result. See Chapters 4 and 13.

**My config error mentions "did you mean …?" — what is that?**

A configuration error is treated as a product surface. A misspelled field produces a diagnostic in
the style of `miette`, with a span in the source and a suggestion on spelling. For example,
`chassis.masss_kg` gives *"did you mean `mass_kg`?"*.

A bare, unhelpful error from serialization reaching you is considered a bug. See Chapters 4 and 6.

**Can I use my own tire data?**

Yes.

If you have a `.tir` file, which is the TNO MF-Tyre text format, convert it with
`python -m outlap.tir to-tyr`.

If you have raw test data, the `outlap.tirefit` pipeline fits an MF6.1 set. It needs the `tire-fit`
extra.

Note the rule on redistribution. You may keep and fit tire-test data that is locked to members,
locally. You may not commit or redistribute it, or a parameter set derived from it. See Chapters 5
and 11.

**Does the pack voltage of the Model 3 change its drive-unit maps?**

Yes. The Model 3 HV variant is *Vdc-coupled*. Its maps of drive-unit efficiency and loss carry an
axis of DC-link voltage, and they are evaluated at the terminal voltage of the pack, which depends
on SoC. See Chapter 9.

The shipped `du_medium.ptm` grids that axis over 730 to 850 V. The open-circuit voltage of the pack
is about 634 to 810 V, and it sags under load. At *low* charge, the terminal voltage can therefore
drop below the grid. The voltage axis is then read by linear extrapolation, from the boundary slice.
That is a deliberate choice, so that a depleted pack stays usable instead of clamping.

Two caveats, though. This extrapolation is an internal behavior of assembly, and it is *not*
surfaced in the `notes` of each lap. And a single default lap barely discharges the pack, moving SoC
from 0.98 to about 0.90, so the voltage stays inside the grid, and no extrapolation actually
happens.

It is expected behavior for that car. It is not an error. See Chapters 9 and 14.

**Why are some of my per-wheel channels NaN?**

A NaN at a station means that the trim solver judged that operating point infeasible, meaning that
it cannot be trimmed. The solver treats it as a boundary, rather than crashing.

A pattern of NaNs on the hardest corners is normal. The tests on golden laps even assert that the
NaN *pattern* does not drift. See Chapters 8 and 13.

**How do I regenerate a golden file?**

For a golden *lap*, run `OUTLAP_BLESS=1 uv run pytest tests/test_limebeer.py`, and include a note in
the pull request explaining the change in physics.

For a tire golden *CSV*, there is no in-tree bless. Those are data from an external oracle,
re-derived only by re-running the oracle, with an update to the provenance header.

A silent change to a golden file is a review stop. See Chapter 13.

**Is outlap deterministic across machines?**

On the same platform, the same inputs give bit-identical results. Across platforms, results are
exact within a documented tolerance.

Four things enforce this: fixed-step integration, fixed iteration counts, reductions in a fixed
order, and no fast-math. And it is a prerequisite for the future Monte Carlo layer. See Chapter 13.

**Can I run outlap in the browser?**

The solver crates are kept clean for WebAssembly, and CI builds them for
`wasm32-unknown-unknown`, including the whole transient tier. The core therefore *can* run in the
browser.

A demonstration widget in WebAssembly lands with v1.0. The packaged browser app, `outlap-web`, grown
from it, comes after 1.0. See Chapters 6 and 15.

**There are two "catalunya" tracks. Which do I use?**

`catalunya_osm` is the 3D import from OpenStreetMap plus elevation. It is the reference Catalunya,
used by the notebooks, the examples, and the Limebeer cross-check.

`catalunya` is the flat TUMFTM vendoring, a peer of the other 24 circuits.

They are the same circuit from two sources. Use `catalunya_osm`, unless you specifically want the
flat 2D version. See Chapters 12 and 13.

**What license applies to a vehicle or a track that I create?**

Data that you author yourself is yours.

But be aware of the inputs that you build on. The *code* of outlap is AGPL-3.0-only. The *schemas*
are Apache-2.0. The shipped *reference data* is CC-BY-SA-4.0. And the vendored *center lines from
TUMFTM* are LGPL-3.0, with a required attribution string.

If you redistribute a track derived from the TUMFTM data, carry that attribution. See Chapters 1 and
12.

**How fast should a lap solve, and how big are the results?**

A single QSS lap solves in well under 50 ms, once the envelope is cached. That is a performance
guarantee, gated in CI.

A full T1 Dataset, for a track of about 4.6 km at 2 m spacing, is roughly 0.6 MB. That is about
2,300 stations, times 16 channels.

For cheap exploration, use a coarse envelope grid. See Chapters 13 and 14.

**Where do I ask for help, or contribute?**

Start with three places. `CONTRIBUTING.md`, where a contribution is AGPL-3.0 with a DCO sign-off.
The theory pages in `docs/theory/`, which hold the equations and citations behind every model. And
the notebooks in `notebooks/`, which hold the same material, runnable.
