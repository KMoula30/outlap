<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# The outlap Guide

**From zero to hero with the outlap vehicle simulator — v0.2.0**

outlap is an open-source, parametric vehicle simulator: you describe a car as data — an F1
car, a GT car, or your daily driver — pick a real racetrack, and outlap computes how fast
that car can physically get around it, wheel by wheel, watt by watt. This guide is the
complete manual: it assumes almost no prior knowledge of vehicle dynamics or simulation and
builds up, chapter by chapter, to everything the tool can do at version 0.2.0.

**How to read this guide.** The chapters are ordered as a course, but they also work as a
reference:

- **New to all of this?** Read Chapters 1–3 in order: what outlap is, a crash course in the
  physics vocabulary, and a hands-on first lap. Then follow your curiosity.
- **Want to run simulations?** Chapters 3, 4, 10, and 14 (quickstart, inputs, API reference,
  recipes) are the practical core.
- **Want to understand the physics?** Chapter 2 is the primer; Chapters 7–9 are the full
  treatment of what the solvers actually compute, with the equations and the literature they
  come from.
- **Want to work on the code?** Chapters 5, 6, and 13 cover the file formats, the
  architecture, and the testing/validation machinery.
- **Stuck?** Chapter 16 is a glossary of every term of art; Chapter 17 is the FAQ.

The guide is a companion to — not a replacement for — three other resources in the repo: the
executable notebooks in [`notebooks/`](../notebooks/README.md) (a guided tour in runnable
form), the derivation-level theory pages in [`docs/theory/`](theory/), and the full
architecture specification in [`docs/HANDOFF.md`](HANDOFF.md).

This document describes **outlap v0.2.0** (milestone M3: the complete quasi-steady-state
T0/T1 solver tier). Code is licensed AGPL-3.0-only, the published JSON Schemas in `schemas/`
are Apache-2.0, and the vendored TUMFTM track centerlines are LGPL-3.0.

## Table of contents

1. [What is outlap?](#1-what-is-outlap)
2. [A crash course in vehicle dynamics and lap simulation](#2-a-crash-course-in-vehicle-dynamics-and-lap-simulation)
3. [Installation and your first lap](#3-installation-and-your-first-lap)
4. [The input quartet: vehicle, track, conditions, sim](#4-the-input-quartet-vehicle-track-conditions-sim)
5. [Files and formats: schemas, maps, and tables](#5-files-and-formats-schemas-maps-and-tables)
6. [Architecture: how the code is organized](#6-architecture-how-the-code-is-organized)
7. [Physics I: tires and aerodynamics](#7-physics-i-tires-and-aerodynamics)
8. [Physics II: solving a lap — T0, T1, and the g-g-g-v envelope](#8-physics-ii-solving-a-lap--t0-t1-and-the-g-g-g-v-envelope)
9. [Physics III: powertrain, machine thermal, battery, and slow states](#9-physics-iii-powertrain-machine-thermal-battery-and-slow-states)
10. [The Python API reference](#10-the-python-api-reference)
11. [Importers and tooling](#11-importers-and-tooling)
12. [The shipped data library](#12-the-shipped-data-library)
13. [Validation, testing, and trust](#13-validation-testing-and-trust)
14. [Recipes: worked examples](#14-recipes-worked-examples)
15. [Limitations and roadmap](#15-limitations-and-roadmap)
16. [Glossary](#16-glossary)
17. [FAQ and troubleshooting](#17-faq-and-troubleshooting)

---

## 1. What is outlap?

*What you will learn: what outlap is and what problem it solves, the small set of design rules
that shape everything in the codebase (one vehicle description, the input quartet, the powertrain
firewall, clean-room physics, determinism), what actually ships in v0.2.0 — including its honest
gaps — and how the rest of this guide is organized so you can find your own path through it.*

### 1.1 A lap simulator where the car is data

outlap is an open-source **parametric vehicle simulator**: a program that predicts how a car
behaves on a race track — most importantly its **lap time**, the seconds it takes to complete one
lap — from a written description of the car's physical properties. "Parametric" means the car is
not code. It is a set of plain YAML files (mass, aerodynamic coefficients, tire data, a drivetrain
wiring diagram) that one binary loads and simulates. Change a number, get a different car. The
same engine spans a Formula 1 car, a GT racer, and a road-going passenger car, and a
race-strategy Monte Carlo layer is planned on top (see Chapter 15, Limitations and roadmap).

The project is three deliverables in one repository:

- a **Rust core** — a Cargo workspace under `crates/` holding the math, the file-format
  contract, the tire and track models, and the lap solvers;
- a **Python API** — `python/src/outlap/`, a thin typed layer (PyO3 + maturin) over the compiled
  core, returning results as labelled `xarray` Datasets;
- **published JSON Schemas** — `schemas/`, the machine-readable contract for every input file
  (eight document kinds: `vehicle`, `ptm`, `tyr`, `emotor`, `battery`, `track`, `conditions`,
  `sim`), generated from the Rust types and versioned as a semver contract (currently 1.4, see
  `SCHEMA_MAJOR`/`SCHEMA_MINOR` in `crates/outlap-schema/src/lib.rs`).

Here is the whole product in five lines. `solve_lap_dataset` loads a car directory and a track,
assembles the physics, solves a lap, and hands back a dataset indexed by distance around the lap:

```python
from outlap.core import Track, solve_lap_dataset

track = Track.load("data/tracks/catalunya_osm")   # 3D Circuit de Barcelona-Catalunya, 4678 m
lap = solve_lap_dataset("data/vehicles/f1_2026", track)
print(lap.attrs["lap_time_s"])                    # seconds; speed/accel channels live in lap["v"], ...
```

Chapter 3, Installation and your first lap, walks through this end to end; Chapter 10 documents
the full Python API.

Every number in this guide that looks like a measurement is computed by the shipped code from the
shipped data. For example: on the 3D Catalunya import, the `f1_2026` reference car laps the
centerline in 112.548 s and the generated racing line in 108.662 s — a 3.886 s gain from geometry
alone (notebook `00_tour_of_outlap.ipynb`, committed outputs, re-executed in CI).

### 1.2 Design philosophy: one car, four tiers, nothing silent

A handful of locked decisions (recorded in `docs/HANDOFF.md` §1, the log that overrides all other
documentation) shape everything else you will meet in this guide:

**One vehicle description, every solver tier.** A lap solver can model a car at different levels
of detail, called **tiers** in outlap: T0 treats the car as a single point of mass; T1 adds four
wheels and load transfer; T2 and T3 (future) integrate the car's motion through time. All tiers
read *the same* `vehicle.yaml` — there is no "T1 version" of a car, and no parameter that only
one tier can see. That is what makes cross-tier validation meaningful (Chapter 8, Physics II).

**The input quartet.** Every simulation is fully specified by exactly four inputs — **vehicle**
(what the car is), **track** (where it drives), **conditions** (the session environment: air
temperature, pressure), and **sim** (numerical settings: tier, grid sizes, coupling modes). Car
identity is never mixed with environment or numerics; swapping one input never silently changes
another. Chapter 4 is devoted to the quartet.

**Composition is data-driven.** One binary loads any `vehicle.yaml`. The drivetrain is described
as a topology graph in data (drive units, gearbox, differentials, wheels), not as per-car code
paths. Which tire model runs, which aero map applies, how the powertrain is wired — all of it is
decided while *loading* the files, never while *solving* the lap.

**Assembly pipeline vs hot loop.** The code is split into a cold **assembly** stage and a hot
**solve** stage. Assembly (the staged load pipeline in `crates/outlap-schema/src/load/mod.rs`)
parses the YAML, resolves inheritance (`extends:`), applies overrides, validates every field,
checks the drivetrain topology, estimates missing derivable values, and hashes the resolved
result. Anything estimated, inherited, or degraded is surfaced in a **loaded-model report** —
"nothing silent" is a locked decision (#41), and `outlap.core.vehicle_report(...)` shows you the
report for any car. The hot loop — the solver kernels that actually compute the lap — then runs
with **zero heap allocations**, a property enforced in CI by an allocation-counting test harness
(`crates/outlap-qss/tests/alloc.rs`), alongside a wall-clock gate that a full Catalunya lap
solves in under 50 ms in release builds.

**Determinism.** The same inputs always produce the same lap. Fixed-step numerics, a fixed (not
tolerance-driven) iteration count for the slow-state coupling, deterministic ordering even when
the envelope generator runs in parallel — and settings that could change results, like the
vertical-load coupling mode (`fz_coupling: one_step_lag | fixed_point`, Decision #29), are
recorded on every result you get back (`lap.attrs["fz_coupling"]`).

**Units and axes.** Internally everything is SI — meters, m/s, rad/s, newtons, newton-meters,
watts, kelvin. RPM and °C appear only at file-format and display boundaries (e.g. the
`machine_temp_c` result channel). The axis convention is ISO 8855: x forward, y left, z up, so a
left turn has positive lateral acceleration. Signs are restated wherever they matter.

### 1.3 The powertrain firewall — and its one documented exception

outlap deliberately does **not** model electric machines, inverters, or gearboxes from the inside.
No electromagnetic simulation, no machine design. Instead, powertrains enter as **`.ptm` map
files**: neutral tables of "at this shaft speed (and optionally DC-link voltage), the unit can
produce this much torque, at this efficiency". This boundary is hard rule #1 of the project — the
**powertrain firewall** — and it exists so that outlap can consume the *results* of professional
powertrain design toolchains without ever reimplementing or absorbing them. The importer for one
such toolchain (PDT) reads its raw HDF5 exports with `h5py` and writes `.ptm` files; real exports
are never committed to the repository (Chapter 11, Importers and tooling).

There is exactly one documented exception, and it is worth knowing because you will see it cited
in the code. **Decision #25 (as amended 2026-07-05)**: the machine *thermal* model — an N-node
lumped-parameter thermal network (LPTN) that tracks winding, rotor, and housing temperatures and
derates torque when they run hot — ports the PDT heat-transfer correlations (air-gap film,
end-cavity and shaft convection, liquid-jacket channel) into `crates/outlap-thermal`. The crate's
own docs call it "a deliberate amendment of the powertrain firewall for the (author-owned)
thermal model". The correlations themselves are standard published forms (Becker–Kaye/Taylor,
Kylander, Etemad, Churchill–Chu, Gnielinski), each cited at its definition. Chapter 9, Physics
III, covers the thermal model in depth.

### 1.4 Clean-room engineering and licensing

All flagship physics is implemented **clean-room from published literature**, with citations in a
theory page shipped in the same pull request as the code: the MF6.1 tire model from Pacejka's
*Tire and Vehicle Dynamics* (2012, 3rd ed.), the lap-solver formulation from Perantoni & Limebeer
(2014) and Lovato & Massaro, the velocity-profile and racing-line methods from Heilmeier et al.
(2020) and Braghin et al. (2008), the interpolant from Fritsch & Carlson. Other open-source
projects may be *consulted* for approach where their license permits reading, but code is always
re-authored independently and the consulted project is recorded. Where an external implementation
is used as a numerical oracle (e.g. the GPL tire library behind the golden test files in
`tools/goldens/`), its *outputs* are used as data only — its source is never read into outlap
(Chapter 13, Validation, testing, and trust).

Licensing is layered on purpose:

| What | License |
|---|---|
| Code (`crates/`, `python/`) | AGPL-3.0-only (SPDX header in every file) |
| `schemas/` (the JSON Schemas) | Apache-2.0 — so any tool may implement the file formats |
| `data/` (reference vehicles, tires, tracks) | CC-BY-SA-4.0 |
| Vendored TUMFTM track centerlines (`data/tracks/`) | LGPL-3.0 (upstream text shipped verbatim) |

AGPL-3.0 was a deliberate choice (Decision #7): commercial use is fine, but always with open
source code. Contributions require a DCO sign-off (`git commit -s`); see `CONTRIBUTING.md`.

### 1.5 What ships in v0.2.0

v0.2.0 is the milestone-M3 release. At a glance, working today:

- **Two solver tiers.** T0, a point-mass forward/backward velocity-profile solver on the full 3D
  road ribbon; and T1, a quasi-steady-state double-track solver with per-wheel loads that
  generates the **g-g-g-v envelope** — a precomputed map of the car's acceleration limits versus
  speed — which T0 then consumes, giving near-T1 fidelity at T0 speed (Chapter 8).
- **Tires.** A clean-room steady-state Magic Formula 6.1 implementation plus a simpler physical
  brush model as the fallback tier, with a `.tir` file codec and a Python fitting pipeline
  (Chapter 7).
- **Powertrain, thermal, battery.** The `.ptm`-map powertrain over a data-defined drivetrain
  topology graph; the N-node machine thermal network with torque derating; a battery
  equivalent-circuit model whose terminal voltage feeds back into the drive-unit maps (the
  Vdc–SoC coupling), all marched as "slow states" along the lap (Chapter 9).
- **Racing line.** A minimum-curvature quadratic-program line generator; the resulting line is a
  first-class track you can lap.
- **Data library.** 3 reference vehicles (`f1_2026`, `limebeer_2014_f1`, `tesla_model3_rwd` — an
  800 V "HV variant" study, deliberately not the ~360 V production car), 3 citation-backed tire
  sets, and 26 track directories: 25 flat LGPL TUMFTM circuits plus `catalunya_osm`, the 3D
  OSM+elevation reference Catalunya used by all notebooks and validation (Chapter 12).
- **Importers.** OSM+DEM tracks, TUMFTM tracks, PDT HDF5 powertrains, `.tir` files (Chapter 11).
- **Validation.** The Limebeer cross-check reproduces the published F1 top speed to −0.2%
  (87.8 vs ≈88 m/s) and corner apex speeds within 5%; the tire model matches an independent
  oracle to 0.289% worst-case — across every slip, load, camber and combined-slip sweep and
  every force/moment channel — against a 0.5% CI gate (Chapter 13).

Representative numbers, all computed by the shipped code and reproduced elsewhere in this guide:
the `f1_2026` reference car laps the 3D Catalunya in 112.548 s on the centerline and 108.662 s on
the racing line (T0 and T1 agree — see §1.1 and Chapter 8), reaching about 2.5 g of lateral
acceleration; the Model 3 HV-variant laps the same circuit's racing line in ≈149 s and the flat
Nürburgring GP in ≈155 s (Chapter 3) at roughly 0.8 g, its lap time responding to drive-unit
sizing, thermal derating, and pack-voltage sag.

Just as important is what v0.2.0 does **not** contain — this guide will not pretend otherwise:

- **No transient tiers in v0.2.0.** Requesting `tier="t2"` or `"t3"` raises a typed "not implemented"
  error. (On `main` since v0.2.0, `t2` has landed: a flat-track closed-loop transient lap through
  `outlap.solve_transient_lap`. T3 arrives in M6.) The QSS-versus-transient parity gates are a
  committed future check, not a current one.
- **Tire thermal and wear are placeholders.** The `.tyr` files carry clearly-labelled synthetic
  `thermal`/`wear` blocks; the real models are the M5 headline.
- **ERS is a power cap.** The F1 car's deploy/harvest energy manager is M6; regenerated energy is
  not yet fed back into the pack during a lap.
- **`data/presets/` is empty** (class presets like `formula_base` are planned, Decision #41), and
  the three plugin points (custom blocks, C-ABI tires, controllers) are a designed extension
  mechanism, not shipped code.
- Five of the thirteen crates (`outlap-powertrain`, `outlap-vehicle`, `outlap-batch`,
  `outlap-transient`, `outlap-wasm`) are two-line placeholders reserving names for later
  milestones.

### 1.6 A map of the repository

| Path | What lives there |
|---|---|
| `crates/` | The Rust workspace: 13 crates, 8 real (`outlap-core`, `-schema`, `-tire`, `-track`, `-thermal`, `-qss`, `-raceline`, `-py`) and 5 placeholders. Chapter 6 draws the full graph. |
| `python/` | The Python package (`src/outlap/`), its tests, and `tools/` plotting/generation scripts. |
| `schemas/` | The published JSON Schemas (Apache-2.0), generated from the Rust types and CI-checked. |
| `data/` | The shipped library: `vehicles/`, `tires/`, `tracks/`, `presets/` (empty at v0.2.0). |
| `notebooks/` | Companion notebooks `00`–`07`, committed with outputs and re-executed in CI. |
| `docs/` | `HANDOFF.md` (the architecture spec and Locked Decisions log), `theory/` (7 cited physics pages), `validation/` (the Limebeer cross-check). |
| `tools/` | `goldens/` — provenance and regeneration rules for the external-oracle tire golden files. |
| `CLAUDE.md`, `CONTRIBUTING.md`, `CHANGELOG.md` | The working agreement, contribution rules (DCO, clean-room), and the git-cliff changelog. |

Worked examples also live inside crates (e.g. `crates/outlap-qss/examples/catalunya_lap.rs`);
they emit CSV consumed by `python/tools/plot_*.py`, so every figure in `docs/theory/` is drawn
from the real models.

### 1.7 How to read this guide

This guide assumes basic Python and **no** vehicle-dynamics or simulation background. Every term
of art is defined where it first appears, and Chapter 16 is a glossary.

Suggested paths:

- **"Just let me drive."** Chapters 2 → 3 → 4, then notebook `00_tour_of_outlap.ipynb`. Chapter 2
  gives you the vehicle-dynamics vocabulary; Chapter 3 installs everything and solves your first
  lap; Chapter 4 explains the four files you just used.
- **"I want to understand the physics."** Chapters 7 (tires and aero) → 8 (lap solving and the
  g-g-g-v envelope) → 9 (powertrain, thermal, battery), each paired with a page in `docs/theory/`
  that carries the equations and citations.
- **"I want to build with it."** Chapters 5 (file formats) → 10 (Python API) → 11 (importers)
  → 14 (recipes), with Chapter 6 (architecture) when you need to read the Rust.
- **"Can I trust it?"** Chapter 13 (validation and testing) and Chapter 15 (limitations and
  roadmap) — read these before quoting outlap numbers anywhere that matters.

The eight companion notebooks under `notebooks/` mirror this guide chapter by chapter — 00 the
tour, 01 the car-as-data workflow, 02 tracks, 03 racing lines, 04 the T0 solver, 05 the MF6.1
tire, 06 the powertrain firewall and importer, 07 the T1 capstone. Every number and plot in them
is computed live by the Rust core when CI re-executes them, so they double as end-to-end tests —
and as your safest starting templates.


---

## 2. A crash course in vehicle dynamics and lap simulation

*What you will learn: the physical vocabulary the rest of this guide is written in. We start with the four forces that act on a car, work up through what a tire actually does, why vertical load and load transfer matter, and how aerodynamics changes the picture. Then we assemble those pieces into the g-g diagram and its g-g-g-v extension, and finish with what a lap simulator actually computes, what "quasi-steady-state" means, and how outlap's T0/T1/T2/T3 solver tiers divide the work. No prior vehicle-dynamics knowledge is assumed.*

Throughout this chapter we use two cars that ship with outlap as running examples: the 1765 kg Tesla Model 3 (`data/vehicles/tesla_model3_rwd/vehicle.yaml`) and the 768 kg reference Formula 1 hybrid (`data/vehicles/f1_2026/vehicle.yaml`).

Everything outlap computes uses SI units internally (m/s, N, N·m, W, K) and the ISO 8855 axis convention: **x points forward, y points left, z points up**. Signs matter constantly in vehicle dynamics, so we will restate this wherever it bites. File formats keep a few human conveniences — RPM, °C, km/h, kPa, mm — which are converted at the boundary (see Chapter 5, Files and formats).

### 2.1 The forces on a car

Only four kinds of force act on a car in a lap simulation:

1. **Weight.** Gravity pulls the mass straight down: $W = m g$ with $g = 9.80665\ \mathrm{m/s^2}$ (the constant `G` in `crates/outlap-qss/src/lib.rs`). The Model 3 weighs $1765 \times 9.80665 \approx 17.3\ \mathrm{kN}$; the reference F1 car about $7.5\ \mathrm{kN}$. Weight is what presses the tires into the road — it is the budget everything else is paid from.

2. **Tire forces.** The only things connecting the car to the road are four contact patches, each roughly the size of your hand. Every acceleration the car experiences — accelerating, braking, cornering — is ultimately a horizontal force generated at those patches. Section 2.2 is devoted to how.

3. **Aerodynamic forces.** Air resists motion (**drag**, pointing backward) and, on cars with wings or ground effect, presses the car down (**downforce**). Both grow with the *square* of speed. Downforce is the one loophole in the weight budget: it adds vertical load on the tires without adding mass. Section 2.4 quantifies this.

4. **Driveline torque.** The powertrain (engine or electric machine, through gears and a differential) applies torque to the driven wheels. A drive unit producing shaft torque $\tau$, geared down by a total ratio through a driveline of efficiency $\eta$ to wheels of radius $r$, pushes the car forward with a force of at most $F = \tau \cdot \text{ratio} \cdot \eta / r$ — *at most*, because the tire may run out of grip first. (outlap precomputes $\text{ratio}\cdot\eta/r$ as `force_per_torque` in `crates/outlap-qss/src/vehicle.rs`.) Powertrains are always consumed as measured or synthesized map files (`.ptm`); the Model 3's committed drive unit peaks at roughly 203 kW (`data/vehicles/tesla_model3_rwd/ptm/du_medium.ptm.yaml`, a clearly labelled synthetic dataset). Chapter 9, Physics III, covers powertrains.

Newton's second law ties them together: the car's acceleration is the sum of these forces divided by its mass, $\vec{a} = \sum \vec{F} / m$.

Racers quote accelerations in "g" — multiples of $g$. A good road car brakes and corners at about 1 g; a Formula 1 car corners at 3–5 g. Why the difference? Almost entirely tires and downforce, which is why the next three sections exist.

### 2.2 What a tire actually does

#### 2.2.1 The contact patch and slip

A tire is not a rigid wheel. The rubber in contact with the road (the **contact patch**) deforms: think of the tread as rows of tiny elastic bristles that get bent sideways and lengthwise as the patch rolls through. Bent rubber pushes back, and the sum of those bristle forces is the tire force. (outlap's simpler tire model, the *brush model*, computes forces from literally this picture — Chapter 7.)

A crucial, unintuitive consequence: **a tire only produces force while it is slipping slightly.** Not sliding like a locked wheel — operating with a small, controlled difference between how the wheel rotates and points versus how it actually moves over the road. Two dimensionless quantities measure this, and outlap defines both exactly as modern `.tir` tire files do; the sign contract lives in `crates/outlap-tire/src/slip.rs`.

**Slip ratio** $\kappa$ (kappa) measures longitudinal slip — the mismatch between the wheel's rolling speed and the road passing underneath it:

$$\kappa = -\frac{V_{sx}}{|V_{cx}|}$$

where $V_{sx}$ is the longitudinal sliding velocity of the contact patch and $V_{cx}$ the forward velocity of the wheel center. It is dimensionless, not a percentage. $\kappa > 0$ means the wheel spins faster than it rolls (driving); $\kappa < 0$ means slower (braking); $\kappa = -1$ is a locked wheel while the car still moves forward. Peak grip typically arrives around $|\kappa| \approx 0.1$; push past it and the patch slides, force falls, and (under braking) the wheel locks.

**Slip angle** $\alpha$ (alpha) measures lateral slip — the angle between where the wheel *points* and where it *travels*:

$$\tan\alpha = \frac{V_{sy}}{|V_{cx}|}$$

A rolling tire held at a few degrees of slip angle generates a large sideways force; this is how cars corner. Now the sign convention (ISO-W, the convention of the tire files outlap reads): positive $\alpha$ means the contact patch slides toward +y (left), which produces a *negative* — rightward — force $F_y$ on a normal tire. So in a left-hand corner, where the car needs leftward +y force, the tires run *negative* slip angles. This double negative trips up everyone once. outlap's tire kernels are written to never take absolute values precisely to preserve it, and the guide will flag it again wherever it matters.

Two more inputs complete the contact-patch state:

- **Camber** (inclination) angle $\gamma$: the lean of the wheel about its own x-axis, top toward +y positive. A cambered tire pulls toward its lean; racers use static camber to compensate for body roll.
- **Normal load** $F_z$: the vertical force pressing the patch into the road, compressive-positive in outlap. A wheel in the air ($F_z \le 0$) produces exactly zero output — the models short-circuit rather than extrapolate.

All of this is bundled into one struct, `SlipState`, and every tire model in outlap answers with the same five outputs (`TireForces`): longitudinal force $F_x$, lateral force $F_y$, and three moments.

The interesting moment here is the **aligning moment** $M_z$: the patch's lateral force acts slightly behind the wheel center — a lever arm called the **pneumatic trail** — producing a torque that tries to steer the wheel straight. That self-centering torque is most of what a driver feels in the steering wheel, and its fade as the tire approaches the limit is the classic "the steering went light" warning.

How forces are computed from the slip state is Chapter 7's subject (outlap implements the industry-standard Magic Formula MF6.1 from Pacejka 2012, plus the brush model). For this chapter you only need the shape of the answer: force rises steeply and nearly linearly at small slip, peaks, then decays as the patch slides.

#### 2.2.2 Friction coefficient and load sensitivity

Divide the peak horizontal force a tire can make by the vertical load on it and you get the **friction coefficient**:

$$\mu_x = \frac{\max_\kappa |F_x|}{F_z}, \qquad \mu_y = \frac{\max_\alpha |F_y|}{F_z}$$

For school-physics friction, $\mu$ is a constant of the two materials. Rubber on asphalt does not work that way: **the friction coefficient falls as load rises.** This is called **load sensitivity**, and it is arguably the single most important fact in vehicle dynamics.

Here are real numbers from the Model 3's shipped tire (`data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml` — a verbatim copy of the 205/60R15 validation tire from Pacejka's book, rated load `FNOMIN` = 4000 N, cold pressure 220 kPa), extracted with outlap's own peak scanner:

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

Doubling the load from 4 kN to 8 kN yields *less than double* the lateral force: $\mu_y$ drops 16%. The F1 reference tire (`data/vehicles/limebeer_2014_f1/tyr/f1.tyr.yaml`, transcribed from Perantoni & Limebeer 2014) shows the same effect at a much higher level: $\mu_y = 1.80$ at 2 kN falling to 1.45 at 6 kN. Racing tires are not just "stickier" — they are also *more* load-sensitive, which makes the load-management story of Section 2.3 even more consequential there.

#### 2.2.3 The friction circle and combined slip

A tire cannot give you maximum braking force and maximum cornering force at the same time — the contact patch has one budget of grip to spend in whatever direction it is asked. The classic picture is the **friction circle** (an **ellipse**, really, since $\mu_x \ne \mu_y$): the achievable force vector $(F_x, F_y)$ is confined to

$$\left(\frac{F_x}{\mu_x F_z}\right)^2 + \left(\frac{F_y}{\mu_y F_z}\right)^2 \le 1$$

Trail-braking — carrying brake force into corner entry while lateral force builds — is literally driving around the rim of this ellipse. outlap uses the idea at two levels of fidelity:

- The point-mass solver (Section 2.9) uses the ellipse literally, one per car, with $\mu_x$ and $\mu_y$ taken from the tire model's peaks. The module doc of `crates/outlap-qss/src/solver.rs` states the exact inequality, including a per-station track grip scale.
- The full tire model does something subtler called **combined slip**: when $\kappa$ and $\alpha$ are both nonzero, MF6.1 attenuates each pure-slip force with cosine-shaped weighting functions (`crates/outlap-tire/src/mf61/combined.rs`). The resulting boundary is ellipse-*like* but asymmetric and load-dependent — measured tires simply are not perfect ellipses.

One more real-world wrinkle: $\mu$ also depends on the road surface. Grip varies corner to corner, so outlap's track format carries a per-station `grip_scale` column (see the header of `data/tracks/catalunya_osm/centerline.csv`), which scales the tire grip locally.

#### 2.2.4 What the tire model knows — and does not, yet

Real tire grip also depends on inflation pressure, temperature, and wear. It is worth knowing exactly where outlap v0.2.0 stands on each, because the `.tyr` file format already has fields for all three:

- **Pressure** is live. MF6.1 includes the Besselink et al. (2010) inflation-pressure terms, and the QSS solvers evaluate every tire at its cold set pressure — the `thermal.p_cold` field of the `.tyr` file (in kPa there, converted to Pa at the boundary; 220 kPa for the Model 3 tire). Changing it genuinely changes grip *if* the coefficient set carries pressure terms — the Model 3's book tire is a 2006-era set without the `PP*` coefficients, so it is pressure-insensitive; its file header documents exactly that.
- **Camber** is accepted by the tire kernels, but the T1 trim currently evaluates all four wheels at zero camber — a simplification recorded in the assembly notes ("camber maps land later").
- **Temperature and wear** are future physics: the `.tyr` `thermal:` and `wear:` blocks are required by the schema and every shipped dataset carries clearly labelled synthetic placeholders, but no shipped solver consumes them yet (the thermal ring and wear/cliff models are milestone M5 — Chapter 15). Until then, outlap's tires are always "cold" and new.

### 2.3 Why load transfer matters

#### 2.3.1 Accelerations shift load

A car's center of gravity (CG) sits well above the road — 0.45 m up on the Model 3 (the `chassis.cg` field of its `vehicle.yaml`), about 0.30 m on the F1 cars. The inertial force acts at the CG, but the reacting tire forces act at road level. That vertical offset means every acceleration tilts the load distribution — even with infinitely stiff suspension. This is **load transfer** (or weight transfer), and it comes in two flavors.

**Longitudinal (pitch) transfer.** Under acceleration $a_x$, load moves rearward; under braking, forward:

$$\Delta F_z^{x} = \frac{m\, a_x\, h_{cg}}{L}$$

where $h_{cg}$ is the CG height and $L$ the wheelbase. For the Model 3 ($h_{cg} = 0.45$ m, $L = 2.875$ m), braking at 1 g moves $1765 \times 9.80665 \times 0.45 / 2.875 \approx 2.7\ \mathrm{kN}$ from the rear axle to the front. Statically the car carries 47% front / 53% rear (front share $= b_r/L$ where $b_r = L - 1.524$ m is the CG-to-rear-axle distance — exactly how `T1Vehicle::front_weight_fraction` computes it). Under that 1 g stop, the front axle load jumps from about 8.1 kN to 10.8 kN while the rear falls to 6.5 kN. This is why brake systems are biased forward: the Model 3's `brakes.balance_bar: 0.62` sends 62% of brake torque to the front axle, roughly matching where the load went.

**Lateral (roll) transfer.** Cornering at $a_y$ moves load from the inside pair of wheels to the outside pair. With equal front/rear track width $t$ the total across the car is $m\, a_y\, h_{cg} / t$ — just under 5 kN for the Model 3 at 1 g ($t = 1.58$ m, the `chassis.track_m` field).

#### 2.3.2 Load transfer costs grip

Combine load transfer with the load sensitivity of Section 2.2.2 and you get the punchline. Take an axle carrying 8 kN split 4 kN / 4 kN: each tire offers $\mu_y = 1.03$, so the axle's effective friction coefficient is 1.03. Now transfer 2 kN across it (4/4 → 2/6). The lightly loaded tire gains a little ($\mu_y = 1.12$) but the heavily loaded one loses more ($\mu_y = 0.95$), and the load-weighted average drops:

$$\frac{2000 \times 1.12 + 6000 \times 0.95}{8000} \approx 0.99$$

The *total* vertical load on the axle is unchanged, yet it lost about 4% of its cornering grip purely because the load is now uneven. **Load transfer always reduces the grip of the axle it acts on.**

You cannot eliminate lateral load transfer — only a lower CG, a wider track, or less mass reduces the total. But the chassis designer chooses *which axle pays*, and that choice sets the car's balance:

- **Roll stiffness distribution.** The body rolls in a corner, and the front and rear suspensions share the job of resisting the roll moment. The stiffer end reacts a larger share and therefore takes more of the lateral load transfer — losing more grip. Stiffening the front (a bigger front anti-roll bar) pushes the car toward **understeer**: the front axle saturates first, the car runs wide, stable and dull. Stiffening the rear pushes toward **oversteer**: the rear saturates first, the tail steps out, exciting and spin-prone.
- In outlap this is the `roll_stiffness_share` field per axle (Model 3: 0.58 front / 0.42 rear). The lateral transfer at each axle is computed as a *geometric* part (reacted through the suspension linkage, set by `roll_center_height_m`) plus an *elastic* part (reacted through springs and bars, set by the roll-stiffness shares) — the classic decomposition from Milliken & Milliken, *Race Car Vehicle Dynamics* (1995), implemented in `load_transfer` in `crates/outlap-qss/src/t1/trim.rs` and derived in `docs/theory/t1-trim.md`.
- **Wheel lift** is the limiting case: transfer more than the inner wheel's load and it carries zero. outlap floors the lifted wheel at 0 N and gives the whole axle load to the outside wheel, exactly so the predicted grip limit does not become optimistic.

A one-number summary of the resulting balance is the **understeer gradient** — the extra steering angle per unit of lateral acceleration the car needs beyond pure geometry, positive for understeer. T1 laps report it per track station as `understeer_gradient`; Chapter 8 gives the exact definition ($K = d\delta/da_y - L/v^2$) and how it is probed.

### 2.4 Aerodynamics: drag, downforce, and balance

Aerodynamic forces scale with dynamic pressure, so both are proportional to speed squared:

$$F_{drag} = \tfrac{1}{2}\,\rho\, C_x A\, v^2 \qquad\qquad F_{down} = \tfrac{1}{2}\,\rho\, C_z A\, v^2$$

Here $\rho$ is air density and $C_x A$, $C_z A$ are the **drag area** and **downforce area** in m² — the products often quoted as CdA and ClA. Working with the product sidesteps arguments about reference areas. outlap stores exactly these products, with downforce positive, in the `aero.constant` block of `vehicle.yaml`:

```yaml
# data/vehicles/f1_2026/vehicle.yaml
aero:
  constant:
    cx_a_m2: 1.25        # drag area
    cz_front_a_m2: 1.9   # downforce area attributed to the front axle
    cz_rear_a_m2: 2.6    # ... and to the rear axle
```

The Model 3, a clean road car with no wings, has `cx_a_m2: 0.51` and zero downforce. Air density comes from the **conditions** input via the ideal-gas law: the defaults of 20 °C and 1013.25 hPa in `conditions.yaml` give $\rho \approx 1.204\ \mathrm{kg/m^3}$ (Chapter 4, The input quartet). Concrete numbers:

- **Drag.** At 130 km/h (36.1 m/s) the Model 3 fights $\tfrac{1}{2} \times 1.204 \times 0.51 \times 36.1^2 \approx 400\ \mathrm{N}$ of drag — about 14.5 kW just to hold speed. Because drag force grows with $v^2$ (and drag *power* with $v^3$), top speed is where the drive-force curve crosses the drag curve; outlap estimates a car's top speed exactly that way when sizing its envelope's speed axis.
- **Downforce.** The F1 car's total $C_z A = 4.5\ \mathrm{m^2}$ generates roughly 13 kN at 250 km/h — about 1.7 times its own weight (the comment in `f1_2026/vehicle.yaml` records this sizing). The tires are then pressed into the road by ~2.7 times $mg$ while the mass being cornered is unchanged, so lateral capability at that speed climbs toward $\mu_y \times 2.7$ — about 3.4 g for this car's synthetic slick ($\mu_y = 1.25$), and 4 g or more on the grippier, load-sensitive tires real F1 cars run, where load sensitivity trims the naive product back down. This single mechanism is most of the answer to "why does an F1 car corner at 4 g and a road car at 1 g."

Two refinements matter and get full treatment in Chapter 7, Physics I:

- **Aero balance.** Downforce is split between the axles — $1.9/4.5 = 42\%$ front on the F1 constants. If that split does not match the weight distribution and mechanical balance, the car understeers or oversteers *more as speed rises*. T1 laps report the realized split per station as `aero_front_share`.
- **Ride-height sensitivity.** Ground-effect cars change their coefficients as the floor nears the road, which couples aero back into suspension travel. outlap's primary aero representation is therefore a gridded map over front/rear ride height, yaw angle, and DRS state (`aero.map` + `aero.axes` on the F1 car, consumed by `crates/outlap-qss/src/t1/aero.rs`; map axes use mm and degrees at the file boundary). The constant block is the fallback — and the road-car case. DRS is held closed in the shipped QSS solvers (its activation is a controller concern).

### 2.5 The g-g diagram and the g-g-g-v envelope

Put Sections 2.2–2.4 together and ask: at some instant, what combinations of longitudinal acceleration $a_x$ and lateral acceleration $a_y$ can this car sustain? Plot the feasible set in the $(a_x, a_y)$ plane and you get the **g-g diagram** — a roughly elliptical blob whose boundary is the car's total grip limit (the concept goes back to Rice, SAE 730018, 1973). A skilled driver lives on its rim: full braking, trail-braking around the boundary into pure cornering, then feeding in throttle on the way out.

For a real car, one g-g diagram is not enough, because the boundary moves with the operating point:

- **Speed** $v$: downforce grows the blob with $v^2$, and drag skews its traction/braking shoulders. A downforce car has a modest g-g at 100 km/h and a huge one at 300 km/h.
- **Local vertical acceleration**: banking presses the car into the road; a crest unloads it; a compression (think Eau Rouge) loads it. All of these change every tire's $F_z$ at once. outlap folds them into one number, $g_{normal}$ — the road-normal specific force: equal to $g$ on flat ground, above $g$ in banking and dips, below $g$ over crests (following Rowold et al. 2023 and Werner et al. 2025).

The result is the **g-g-g-v envelope**: a family of g-g boundaries indexed by $(v, g_{normal})$, stored as a gridded table $a_y = gg(v, \hat a_x, g_{normal})$. Lovato & Massaro (2022) develop the g-g-g idea; Werner et al. (2025, arXiv:2504.10225) give the formulation outlap implements. In outlap the envelope is generated once per car by `GgvEnvelope::generate` (`crates/outlap-qss/src/t1/envelope.rs`) on a default grid of **40 speed × 25 longitudinal × 7 normal-g points** (the `sim.envelope` setting in `crates/outlap-schema/src/sim.rs`), with speeds from 5 m/s up to the car's estimated top speed and $g_{normal} \in [0.5\,g,\ 2\,g]$.

Our two example cars make the shape concrete. The Model 3 has zero downforce, so its lateral boundary is essentially the same at every speed — only drag skews the traction and braking shoulders as $v$ rises. The F1 car's boundary is a funnel: modest at low speed, enormous at high speed, because 13 kN of downforce at 250 km/h multiplies what every tire can do.

Three more properties are worth internalizing now (Chapter 8 has the machinery):

- The envelope is a **pure tire-grip limit**. The powertrain's force ceiling is deliberately *not* baked in; the lap solver applies it separately as a `min`. Grip and power stay independently swappable.
- The envelope "funnel" widens with speed for a downforce car, and a compression gives more grip than a crest at every speed — `docs/theory/ggv-envelope.md` shows the shipped figures.
- For analyses where the third g is noise, `sim.flat_track: true` zeroes grade, banking, and vertical curvature so the envelope collapses to a classical flat g-g ($g_{normal} \equiv g$). It is how outlap reproduces 2-D published studies (Chapter 13).

### 2.6 What a lap simulator does

Strip away the detail and a lap-time simulator answers one question: **given a path and a car, what is the fastest speed profile along that path?**

The path is described by arc length $s$ — distance along the line, in meters — and its **curvature** $\kappa(s) = 1/R$, the reciprocal of the local corner radius. Driving the path at speed $v$ demands a centripetal (lateral) acceleration $a_y = \kappa v^2$: that demand is what the g-g boundary must cover. A lap simulator finds, at every point, the highest speed whose demands fit inside the car's capability.

outlap's quasi-steady-state solver (`crates/outlap-qss/src/solver.rs`, re-implemented clean-room from the formulation of Heilmeier et al., *Vehicle System Dynamics* 58(10), 2020) is *not* a differential-equation integration. It is a three-phase construction on stations spaced every $\Delta s = 2$ m by default (`DEFAULT_DS_M`):

1. **Cornering-limited speed.** At each station, find the highest speed whose lateral demand the envelope can meet: solve $\kappa_l v^2 + g\sin\theta_b\cos\theta_g \le a_{y,max}(v, g_{normal})$ for $v$, where $\theta_b$ is the banking angle and $\theta_g$ the grade. This caps the speed at every apex.
2. **Forward pass (traction-limited).** Sweep forward from the slowest point, accelerating out of each corner as hard as the *minimum* of tire grip (what remains of the friction budget after the lateral demand is paid) and powertrain force allows, minus drag and uphill gravity. This builds the corner exits and straights.
3. **Backward pass (braking-limited).** Sweep backward, asking at each station "how fast could I have been here and still slow down in time for what's ahead?" The same friction budget is spent on braking, with drag and uphill gravity now helpfully *adding* to deceleration. This places every braking point.

The final speed profile is the pointwise minimum of all three; for a closed lap the sweeps iterate from the slowest corner until self-consistent. Lap time is the fixed-order sum over segments:

$$t_{lap} = \sum_i \frac{2\,\Delta s}{v_i + v_{i+1}}$$

Grade, banking, and vertical curvature enter through the precomputed 3-D path geometry (`T0Path` in `crates/outlap-qss/src/path.rs`): banking assists cornering, crests unload the car — outlap even guards the flying-car case ($N \le 0$), coasting airborne stations on drag and gravity alone — and dips press it in.

A practical detail you will meet when importing tracks: curvature is a *second derivative* of position, so meter-scale noise in a surveyed centerline becomes violent curvature spikes. outlap smooths the projected curvatures with a centered moving average over ±6 stations (`CURV_SMOOTH_RADIUS` in `path.rs`); Chapter 11, Importers and tooling, discusses where imported geometry can and cannot be trusted.

Two closed forms are useful intuition anchors (and outlap's analytic test cases; `docs/theory/t0-point-mass.md`):

- Flat circle, constant grip: $v = \sqrt{\mu_y\, g\, R}$. The Model 3 tire ($\mu_y \approx 1.03$ at rated load) on a 50 m-radius corner tops out around $22.5\ \mathrm{m/s}$ (81 km/h).
- Add downforce and the solver's closed form becomes $v^2 = \mu_y m g \,/\, (m/R - \mu_y q_z)$ with $q_z = \tfrac{1}{2}\rho C_z A$. For the 660 kg Limebeer F1 car ($C_z A = 4.5\ \mathrm{m^2}$, $\mu_y = 1.63$ at rated load) the same 50 m corner allows about $34.6\ \mathrm{m/s}$ — downforce is worth over 20 km/h *in one corner*, before load sensitivity (which this back-of-envelope ignores) takes its cut. Note the denominator: if downforce grew fast enough to cancel $m/R$, the cornering speed would be unlimited. Real cars just get close.
- Banking helps too: the banked-turn limit $v^2 = gR\,(\mu_y\cos\phi + \sin\phi)/(\cos\phi - \mu_y\sin\phi)$ for bank angle $\phi$ is verified against the solver in `crates/outlap-qss/tests/analytic.rs`.

That is the whole trick. It runs a full lap in well under 50 ms — a CI-enforced budget (Chapter 13, Validation) — which is what makes parameter sweeps, and later the race-strategy Monte Carlo layer, practical.

One property worth noticing: everything in this construction is deterministic. Fixed station spacing, fixed iteration counts, fixed-order summations — the same inputs reproduce the same lap bit-for-bit, run after run. That is a deliberate project-wide rule (Chapter 6), and it is what makes golden-file regression testing and honest A/B comparisons possible.

### 2.7 Quasi-steady-state vs transient simulation

The solver above never asks *how the car gets from one state to the next*. It assumes that at every station the car is in a **trimmed** (equilibrated) state: all forces and moments balanced, nothing still settling. That is the **quasi-steady-state (QSS)** assumption: a lap is a sequence of steady states parameterized by position, and time appears only as the integral of $1/v$ along the path.

What QSS deliberately ignores is everything with its own settling time:

- **Yaw, roll, and pitch dynamics.** A real car takes a few tenths of a second to take its "set" in a corner; QSS teleports between equilibria.
- **Tire relaxation.** A tire's force lags its slip: it must roll a characteristic distance (the **relaxation length**, on the order of the tire radius) before the force builds. outlap ships the exact-exponential lag update in `crates/outlap-tire/src/relax.rs`, but nothing calls it in production yet — QSS uses steady-state forces by definition.
- **Dampers and drivers.** Shock absorbers only matter when the suspension is moving; a driver model only matters when inputs evolve in time.

A **transient** simulation integrates the equations of motion through time with a fixed-step integrator, capturing all of the above at much higher cost. (The plumbing is already visible in `sim.yaml`: the `dt_s` timestep, default 0.001 s, and the `integrator` choice, Heun by default or classical RK4, exist for the transient tiers — today they are recorded but unused.) Neither approach is "more correct" for every question:

| | QSS (outlap today) | Transient (outlap T2/T3, future) |
|---|---|---|
| Answers | lap time, speed profile, grip usage, balance trends | stability, curb strikes, damper tuning, driver-in-the-loop |
| Assumes | equilibrium at every point | only the model equations |
| Cost | milliseconds per lap | seconds-to-minutes per lap |
| Great for | "what does 10 kg or 5% more downforce cost?" — errors largely cancel between variants | "is this setup drivable at the limit?" |

One honest caveat: outlap's QSS envelope boundary is not filtered for open-loop stability. A trim state can balance all forces yet be a knife-edge no driver could hold; that filtering is explicitly deferred to the transient tiers (`docs/theory/ggv-envelope.md`).

Slowly evolving quantities sit in a middle ground. Over a lap, a battery's state of charge and a motor's winding temperature drift monotonically rather than oscillate, so outlap treats them as **slow states** marched along the QSS profile with a bounded outer iteration (solve the profile → march the slow states along it → re-solve). An overheating drive unit or a power-capped battery then feeds back on lap speed without a transient solver. Chapter 9, Physics III, covers this coupling.

### 2.8 Point-mass vs double-track models

Independent of QSS-versus-transient, there is a second axis: how much *car* do you model?

A **point-mass** model collapses the car to a single particle: one mass, one friction ellipse (or one envelope), lumped aero, a drive-force curve. It cannot tell you *which* tire gives up or how balance shifts — only whether the car as a whole can hold the demanded acceleration. outlap's `T0Vehicle` (`crates/outlap-qss/src/vehicle.rs`) is exactly this reduction: mass, axle-averaged $\mu_x$/$\mu_y$ from the tire model's peaks, the lumped aero constants $\tfrac{1}{2}\rho C_x A$ and $\tfrac{1}{2}\rho C_z A$, and the folded drive envelope.

A **double-track** (four-wheel) model keeps both axles *and* both sides: per-wheel vertical loads with the full longitudinal and lateral transfer of Section 2.3, per-wheel slip states fed to the real tire model, steering geometry, a differential, brake bias. outlap's `T1Vehicle` (`crates/outlap-qss/src/t1/vehicle.rs`) is a quasi-static double-track model.

Its `trim` solver answers one question: for a commanded operating point $(v, a_y, a_x, g_{normal})$, what steering angle, body-slip angle, yaw rate, slip controls, and four wheel loads balance every force and moment? That is a 9-unknown algebraic solve per operating point (`crates/outlap-qss/src/t1/trim.rs`; Chapter 8 walks through it). And if *no* balance exists — the demand is simply beyond the car — the point is declared infeasible. That infeasibility boundary, traced over a grid of operating points, *is* the g-g-g-v envelope of Section 2.5.

(The textbook middle step, the single-track or "bicycle" model — two wheels, no left/right distinction — is a fine hand-analysis tool. outlap jumps straight to double-track because load transfer, which the bicycle model cannot represent, is where the interesting grip physics lives.)

### 2.9 The tier ladder: T0, T1, T2, T3

outlap names its solver fidelity levels **tiers**, selected by a single field: `tier` in `sim.yaml`, or the `tier=` argument in Python. A hard project rule (Chapter 6, Architecture) is that **every tier evaluates the same vehicle description** — there is no "T0 config" versus "T1 config", only one `vehicle.yaml` read at different fidelity. The enum lives in `crates/outlap-schema/src/sim.rs`; the dispatch in `crates/outlap-qss/src/qss.rs`.

| Tier | What it is | Model class | Status in v0.2.0 |
|---|---|---|---|
| `t0` | Point-mass velocity profile on the g-g-g-v envelope | point-mass | shipped |
| `t1` | The *same* velocity profile, plus a per-station double-track re-trim | quasi-static double-track | shipped — **the default** |
| `t2` | Transient double-track | transient | future (milestone M4) — typed error today |
| `t3` | Full transient with a driver model | transient | future (M6) — typed error today |

Details worth knowing from day one:

- **T0 and T1 produce the same lap time.** At v0.2.0 both run the identical velocity-profile solve on the envelope. `t1` then revisits every station and re-trims the double-track model at the solved operating point, emitting per-wheel channels — `vertical_load_n`, `slip_ratio`, `slip_angle_rad`, `force_long_n`, `force_lat_n` over a `wheel` dimension ordered `FL/FR/RL/RR` — plus the setup metrics `understeer_gradient` and `aero_front_share`. A `t0` lap gives you the point-mass channels only.
- Because the envelope is *generated by* the T1 trim solver, even a `t0` lap assembles the double-track model once. Envelope generation is a seconds-scale cold step, cached per car and settings within a session; the sub-50 ms figure is the solve itself.
- Internally there is also a degenerate constant-$\mu$ ellipse path — the closed-form T0 of Section 2.6's anchors — kept as the analytic and performance test target. The production `t0` you reach from Python always runs on the envelope.
- Requesting `t3` fails loudly with a typed "not implemented yet (arrives in M6)" error rather than silently downgrading. Since v0.2.0, `t2` runs the transient tier: it is *time*-indexed rather than arc-length-indexed, so it has its own entry point (`solve_transient_lap`, or `solve_lap_dataset(..., tier="t2")`), and it runs flat-track only until the closed loop through the chassis grade/banking terms is gated.

Every result records the tier that produced it, alongside every simplification made during assembly — degraded aero, estimated parameters, brush-tire fallbacks — in its notes. "Nothing silent" is a design rule, not a slogan (Chapter 4).

### 2.10 Racing line vs centerline

Everything above took the path as given — but *which* path? A track file describes a corridor: a **centerline** plus left/right widths per station (`track.yaml` + `centerline.csv`; Chapter 5). Drivers do not follow the centerline. They use the full width to straighten each corner — out-in-out — because a larger radius at the same grip means a higher $v = \sqrt{a_y R}$. The chosen path is the **racing line**, and it changes lap time a lot.

The true racing line is the *time-optimal* one — a genuinely hard optimal-control problem (the Perantoni & Limebeer 2014 study outlap validates against solves exactly that). outlap v0.2.0 ships the standard first approximation: the **minimum-curvature line**, a quadratic program over lateral offset within the corridor that minimizes $\int \kappa^2\, ds$ — geometrically, the "straightest possible" path. It is the default `sim.raceline` generator (`min_curvature`; you may instead supply your own line as a CSV file), and every result records which line it ran on in its `LineDescriptor` (`Centerline`, `MinCurvature`, or `File`).

The centerline itself remains useful: it is deterministic, generator-free, and ideal for comparing tracks or cross-checking importers, so the Python API accepts either a plain `Track` (a lap of its centerline) or a generated `Raceline` as the `line` argument of `solve_lap_dataset` (Chapter 10).

Be honest about the gap. Minimum-curvature is close to time-optimal in slow corners but systematically under-opens medium-speed ones. In the Limebeer cross-check (`docs/validation/limebeer.md`), outlap matches the published top speed within 0.2% and the slowest apex within 5%, while the full lap time — 92.4 s on the committed track import versus the paper's optimal 82.4 s — is recorded but not gated, with line optimality and track-geometry fidelity documented as the decomposed reasons. A time-weighted line optimizer is on the roadmap (Chapter 15, Limitations and roadmap).

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

With this vocabulary in place: Chapter 3 gets outlap installed and runs your first lap; Chapter 4 formalizes the four inputs you just met informally (vehicle, track, conditions, sim); and Chapters 7–9 reopen each physics topic at full depth.


---

## 3. Installation and your first lap

*What you will learn: how to build outlap from source with `uv` (including the one environment
variable that makes it fast), how to verify the install, and how to solve your first simulated
laps — a Tesla Model 3 around Barcelona-Catalunya and the Nürburgring GP circuit — from about
twenty lines of Python. Along the way you will meet the loaded-model report, outlap's "nothing
silent" answer to the question every simulation user should ask: what did the tool assume?*

### 3.1 Prerequisites

outlap is a Rust core with a Python API, so you need both toolchains. Everything below ran on
Linux; macOS and Windows should work wherever Rust and `uv` do, but as of v0.2 only Linux is
exercised in CI.

| Requirement | Why | Where |
|---|---|---|
| Rust (stable) | `uv sync` compiles the `outlap-core` extension from the Rust workspace via maturin | [rustup.rs](https://rustup.rs) (this walkthrough used `rustc 1.96.1`) |
| `uv` | Manages the Python project, virtual environment, and the extension build | [docs.astral.sh/uv](https://docs.astral.sh/uv/) (used `uv 0.11.26`) |
| Python ≥ 3.12 | `requires-python = ">=3.12"` in `python/pyproject.toml`; `uv` can download one for you | (used Python 3.12.13) |
| git | Cloning; the reference data ships in the repository | — |

You install nothing else by hand — `uv` resolves the whole environment, including building the
Rust extension (an abi3-py312 wheel, `crates/outlap-py/pyproject.toml`: one build serves any
Python 3.12+), from `python/pyproject.toml`.

### 3.2 Clone and build

```bash
git clone https://github.com/KMoula30/outlap.git
cd outlap/python
MATURIN_PEP517_ARGS="--profile release" uv sync --group notebooks --extra tire-fit
```

What each part does:

- `uv sync` creates `python/.venv`, installs the runtime dependencies (numpy, xarray, pyarrow,
  h5py, jsonschema, pyyaml, pydantic), and **compiles the Rust extension** from
  `crates/outlap-py` — the first build takes a few minutes while cargo compiles the workspace.
  The extension's `[tool.uv] cache-keys` cover every crate's sources, so a later `uv sync`
  rebuilds it automatically after any Rust change instead of reinstalling a stale wheel.
- `MATURIN_PEP517_ARGS="--profile release"` makes maturin build the Rust code with cargo's
  optimized release profile. This matters: generating a car's performance envelope (the cold
  step behind every lap solve, Chapter 8) is "a seconds-scale cold step in release but minutes
  in debug" (`.github/workflows/ci.yml`, which sets exactly this variable). Without it, your
  first lap solve takes about a minute instead of seconds — measured 63 s on a debug build for
  this chapter.
- `--group notebooks` installs the notebooks dependency group (matplotlib, ipywidgets,
  ipykernel, nbclient, nbformat) — you need matplotlib for the plot below, and the group is
  everything required to run the `notebooks/` tour.
- `--extra tire-fit` installs scipy for the MF6.1 tyre-fitting pipeline (Chapter 11,
  Importers and tooling). Optional today, but it is what CI installs, and it is one package.

A third extra, `--extra track-import`, adds what the OpenStreetMap track importer needs
(Chapter 11).

> **Warning — the `uv sync` gotcha.** `uv sync` is *exact*: it removes any installed package
> you did not ask for in that invocation. If you later run a plain `uv sync` (no flags), it
> will **uninstall the notebooks group** — matplotlib and the Jupyter kernel vanish, and the
> plotting snippet below stops working. Always repeat the full command:
> `uv sync --group notebooks --extra tire-fit`.

### 3.3 Verify the install

Run these from the `python/` directory (`uv run` uses the project environment directly — no
venv activation needed):

```bash
uv run python -c "from outlap.core import DEFAULT_DS_M; print('outlap.core OK, default grid step', DEFAULT_DS_M, 'm')"
uv run python -m outlap.schemas --check
```

```text
outlap.core OK, default grid step 2.0 m
schema check OK: 8 schemas, 22 fixtures + 7 data files validated
```

The second command validates every committed JSON Schema and all shipped reference data against
them — the same check CI runs (Chapter 5 covers what those schemas are).

One import-model note before we start: the top-level `outlap` package is currently a stub —
the real user API lives in **`outlap.core`**, so always write `from outlap.core import ...`
(Chapter 10, The Python API reference).

### 3.4 Your first lap

A *lap simulator* answers: given a complete description of a car and a track, how fast can that
car go around? outlap solves this at selectable fidelity levels called **tiers**: `t0` treats
the car as a point mass riding on a precomputed grip envelope, and `t1` is the full
quasi-steady-state (QSS) model that additionally resolves what each of the four tyres is doing
(Chapter 2 introduces the physics; Chapter 8 the solvers). Tiers `t2`/`t3` (transient models)
are not implemented yet and raise a clear error if requested.

We will drive the shipped **Tesla Model 3 RWD (HV variant)** around the shipped 3-D import of
the **Circuit de Barcelona-Catalunya**. Two honesty notes up front, both from
`data/vehicles/tesla_model3_rwd/README.md`: the car's chassis, mass, and aero are
Model-3-plausible spec-sheet values, but its powertrain is a *synthetic* 800 V-class ("HV")
drive-unit and battery-pack stack that the production car does not have — a documented study
variant, chosen so the battery-voltage coupling of Chapter 9 is live on a road car. And the
repository contains **two Catalunyas**: `data/tracks/catalunya_osm` is the 3-D reference
(OpenStreetMap geometry + open elevation data) used by all the notebooks and the validation
work, while `data/tracks/catalunya` is a flat 2-D variant from the TUMFTM set (Chapter 12).
Use `catalunya_osm` unless you specifically want the flat one.

Save this as `python/first_lap.py` (the `../data/...` paths assume you run it from `python/`):

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

Reading that top to bottom:

- `Track.load` takes a *directory* containing a `track.yaml` plus its centerline CSV
  (Chapter 5). A track in outlap is a 3-D ribbon: a center line with curvature, grade, banking,
  and a drivable width either side.
- `min_curvature` computes a **racing line** — the path a good driver takes, cutting across the
  road's width to straighten corners — by minimizing the path's curvature (how sharply it
  bends) within the track's corridor, shrunk by the car's half-width plus a safety margin
  (Chapter 8). It returns a `Raceline`, which `solve_lap_dataset` accepts directly; passing the
  `Track` itself instead drives the center line. Here the racing line is worth about **4.5 s**.
- `solve_lap_dataset` loads and validates the vehicle directory, solves the lap, and returns
  the results as a labelled `xarray.Dataset` (Chapter 10). The lap time lives in
  `attrs["lap_time_s"]`.
- **t0 and t1 report the same lap time by design**: the speed profile ran on the t1-derived
  grip envelope in both cases; t1 re-trims it station by station for the per-wheel detail.
- Internally everything is SI — speeds in m/s, forces in N, temperatures in K inside the core —
  with RPM, °C, and km/h appearing only at file-format and display boundaries, as in the last
  line above.

About that pause on the first solve: before the first lap of a given car, outlap generates its
**g-g-g-v envelope** — a table of the car's maximum acceleration in every direction (braking,
cornering, combined) as a function of speed and the local effective vertical gravity on a 3-D
road (Chapter 8, Physics II). With a release build this takes seconds, and the result is cached
for the rest of the Python process — hence the near-instant second and third solves above. A
new Python process regenerates it once.

### 3.5 What you got back: the lap dataset

Everything the solver knows about the lap comes back in one `xarray.Dataset` — add
`print(lap)` after the solve and you get:

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

The dimension `s` is **arc length** — distance along the driven line in metres, one row every
2 m (the default grid step, `DEFAULT_DS_M = 2.0`). Speed `v`, accelerations `ax`/`ay`, and
cumulative time `t` are point-mass channels; the `(s, wheel)` variables are the t1 per-wheel
detail in FL/FR/RL/RR order. Signs follow ISO 8855 (x forward, y left, z up), so `ay` is
positive to the *left* — the negative values at the start of this lap are a right-hand
corner. Two things worth noticing right away:

- The `nan`s in the per-wheel channels are honest: at stations where the four-wheel re-trim
  has no feasible solution exactly on the grip limit, outlap records "don't know" rather than
  inventing a number. On this lap that is about a quarter of the stations — this car spends a
  lot of its lap pressed hard against its envelope (Chapter 8, Physics II).
- The `notes` attribute is the run's paper trail — 11 entries for this lap, recording every
  simplification taken (e.g. this car has no ride-height aero map, so a constant-aero fallback
  carried the lap). Nothing in outlap degrades silently. `resolved_hash` fingerprints the
  exact resolved vehicle that produced the result, and `fz_coupling` records a numerics
  setting explained in Chapter 8.

Now the classic first plot — speed against distance:

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

You should see the sawtooth signature of every lap simulation: long climbs where the car
accelerates (up to 65.3 m/s at the end of the main straight, around $s \approx 1300$ m),
cliffs where it brakes, and valleys at the corner **apexes** — the slowest point of each
corner, down to 12.5 m/s at the slowest hairpin. Chapter 2 explains why this
forward-accelerate/backward-brake shape is the essence of quasi-steady-state lap solving.

### 3.6 What did the loader assume? The loaded-model report

Before trusting any result, ask the model what it made up. Every vehicle load produces a
**loaded-model report**: everything that was inherited from a parent file, estimated by a
documented heuristic, or degraded to a fallback. `vehicle_report` returns it without solving:

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

The car loads *warning-clean* — zero warnings, zero degraded values — but with ten estimates
deliberately on record: fields the vehicle file omitted and the load pipeline filled with a
documented assumption. This is the input-quartet philosophy of Chapter 4 in action, and
Chapter 12 walks through this vehicle's full per-parameter provenance (which values are
manufacturer spec, which are estimates, and why the powertrain is synthetic).

### 3.7 The same car on a TUMFTM track

The data library ships 25 circuits vendored from TUMFTM's racetrack-database (LGPL-3.0 data;
Chapter 12, The shipped data library) — the flat `catalunya` twin among them. They are real,
satellite-measured center lines and corridor widths, but strictly flat: elevation, grade, and
banking are all zero. Same car, Nürburgring:

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

A ~200 kW electric sedan lapping the 5.14 km GP circuit in about two and a half minutes is a
plausible number, and the last two lines preview something bigger: because this vehicle has a
full battery + motor-thermal stack, every lap also integrates the **slow states** — the pack
drained from 98% to 89.2% state of charge, and the motor winding heated from 20 °C to a peak
of 154.7 °C over the lap. Chapter 9 (Physics III) is entirely about this machinery.

> **Tip — a fast grid for experiments.** Envelope generation cost scales with its grid. When
> you are sweeping many variants and don't need final-quality numbers, shrink it:
> `sim={"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}` (the idiom used by
> `python/tests/test_model3.py`) generates in about a second even on a debug build. It is
> coarser and slightly conservative — the Catalunya t1 lap comes out 154.97 s on the fast grid
> versus 148.94 s on the default 40×25×7 grid — so quote default-grid numbers when it matters.

### 3.8 Where to go next

You have built outlap, solved laps at both shipped tiers on two circuits, plotted a speed
trace, and read the loaded-model report. Three directions from here:

- **The notebooks** (`notebooks/`, run with
  `uv run --with jupyterlab jupyter lab ../notebooks/00_tour_of_outlap.ipynb` from `python/`):
  `00_tour_of_outlap.ipynb` is the guided tour of everything you just did with the F1 reference
  car; `01`–`06` each deepen one topic (car-as-data, tracks, racing lines, the t0 solver, the
  MF6.1 tyre model, the powertrain firewall); `07_qss_t1.ipynb` is the t1 capstone, including
  the drive-unit sizing sweep on this very Model 3. They are committed with outputs and
  re-executed by CI, so what you see on GitHub is what the code does.
- **Understanding the inputs**: Chapter 4 explains the four files you just implicitly used —
  vehicle, track, conditions, sim — and Chapter 5 their formats.
- **Understanding the physics**: Chapter 2 for the vocabulary, Chapters 7–9 for tyres and
  aero, the lap solvers and the g-g-g-v envelope, and the powertrain/thermal/battery slow
  states you glimpsed above.

If anything failed along the way, see Chapter 17, FAQ and troubleshooting — and remember the
two traps from §3.2: plain `uv sync` uninstalls the notebooks group, and a debug-profile
build makes the first solve take minutes.


---

## 4. The input quartet: vehicle, track, conditions, sim

*What you will learn: every outlap run is described by exactly four inputs — a vehicle, a track, session conditions, and simulation settings — and why the strict separation between them is a design rule, not a convention. You will walk through a real shipped `vehicle.yaml` section by section, learn how inheritance (`extends:`), what-if overrides, and estimation heuristics work, and see what a configuration error actually looks like. By the end you will be able to read, write, and debug all four files.*

### 4.1 Why four files?

outlap describes a simulation run with four separate documents, called **the input quartet**:

| Input | File(s) | What it answers | Required? |
|---|---|---|---|
| **Vehicle** | `vehicle.yaml` (+ referenced `.ptm`/`.tyr`/`.emotor`/battery files) | *What car is this?* | yes |
| **Track** | `track.yaml` + `centerline.csv` | *Where is it driving?* | yes |
| **Conditions** | `conditions.yaml` | *What kind of day is it?* | no — defaults to a standard atmosphere |
| **Sim** | `sim.yaml` | *How should the solver run?* | no — every field has a default |

The separation is one of the project's hard rules (`CLAUDE.md`): **never mix car identity with environment or numerics**. A vehicle file must contain nothing about the weather; a conditions file must contain nothing about the car; a sim file must contain nothing physical at all. The payoff is composability — the same `vehicle.yaml` can lap any track on any day at any solver fidelity, and any result can be reproduced by naming its four inputs. This is also why all solver tiers (T0 through T3 — the fidelity levels introduced in Chapter 2, A crash course in vehicle dynamics and lap simulation) evaluate the *same* parameter objects: there is no "T1-only" field anywhere in the quartet.

On disk, a vehicle is a directory. The loader (a filesystem `SourceLoader` in Rust terms — `FsLoader` in `crates/outlap-schema/src/io.rs`) is rooted at that directory, so every path *inside* `vehicle.yaml` is relative to it. The optional `conditions.yaml` and `sim.yaml` sit next to `vehicle.yaml` in the same directory. The Python entry point reflects this directly (see Chapter 10, The Python API reference):

```python
from outlap.core import Track, solve_lap

track = Track.load("data/tracks/catalunya_osm")
lap = solve_lap("data/vehicles/tesla_model3_rwd", track)
```

A *missing* `conditions.yaml` or `sim.yaml` silently resolves to the documented defaults; a *present but malformed* one is always an error — the loader never ignores a broken file (`crates/outlap-py/src/lib.rs`, `solve_lap`).

Every quartet file starts with a `schema:` line of the form `<name>/<MAJOR>.<MINOR>` (for example `vehicle/1.0`). The name half exists so that feeding the wrong kind of document somewhere fails cleanly ("expected a `vehicle` document but found `tyr`") instead of half-deserializing into nonsense; loaders accept a file whose name and MAJOR match, and treat MINOR as informational (`crates/outlap-schema/src/version.rs`). More on versions in §4.9.

### 4.2 `vehicle.yaml` — the anatomy of a car

The vehicle document is the centerpiece of the quartet. Here is a real one, shipped in the repository — the Tesla Model 3 RWD reference car (`data/vehicles/tesla_model3_rwd/vehicle.yaml`; note this is an "HV variant" study whose powertrain is deliberately synthetic — see Chapter 12, The shipped data library, for the full provenance story):

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

Eight top-level sections are **required** — `schema`, `name`, `chassis`, `aero`, `suspension`, `tires`, `drivetrain`, `brakes` — and three are optional: `extends` (inheritance, §4.3), `ers` and `battery` (whole subsystems a car may simply not have), plus an `extensions` slot for vendor keys (§4.5). The Rust type behind the document is `Vehicle` in `crates/outlap-schema/src/vehicle/mod.rs`. Let's take the sections one at a time. All values are SI internally — metres, kilograms, newtons, N·m — with the display-boundary exceptions called out where they occur.

#### 4.2.1 `chassis` — mass, geometry, inertia

The chassis block carries the bulk properties, expressed in the ISO 8855 body frame (**x forward, y left, z up** — the axis convention used throughout outlap):

- `mass_kg` — total mass, sprung plus unsprung (i.e. body *and* wheels together; outlap's quasi-static tiers do not split them).
- `cg: [x, y, z]` — the centre of gravity (the point where the car's weight effectively acts), metres. In the shipped files the x entry is the longitudinal distance from the front axle to the CG — the classic *a* dimension: the Model 3's `1.524` is 0.53 × 2.875 m wheelbase, encoding a ≈47/53 front/rear weight split. The z entry is the CG height (0.45 m here — low, thanks to the floor-mounted battery pack).
- `inertia: [Ixx, Iyy, Izz]` — the diagonal moments of inertia in kg·m² (resistance to rolling, pitching, and yawing respectively). Products of inertia are deferred to a future additive schema field (`crates/outlap-schema/src/vehicle/chassis.rs`). The QSS tiers shipped in v0.2 do not consume these; they matter from the transient tiers onward.
- `wheelbase_m` and `track_m: [front, rear]` — the distance between axles and the distance between left/right wheel centres per axle.

#### 4.2.2 `aero` — a map, a constant block, or both

Aerodynamic force in outlap is expressed as *areas*: `cx_a_m2` is the drag area $C_x A$ in m² (drag coefficient times frontal area — drag force is $\tfrac{1}{2}\rho\,C_xA\,v^2$ where $\rho$ is air density and $v$ speed), and `cz_front_a_m2` / `cz_rear_a_m2` are the downforce areas per axle. The physics is Chapter 7, Physics I; here we only care about the file shape (`crates/outlap-schema/src/vehicle/aero.rs`):

- `map:` (required) — a reference to a gridded aero map (a parquet sidecar table; Chapter 5, Files and formats).
- `axes:` (required, may be empty) — the ordered names of the map's input axes. Only known names are accepted (`ride_height_f_mm`, `ride_height_r_mm`, `ride_height_mm`, `yaw_deg`, `roll_deg`, `steer_deg`, `drs_flag`, `speed_mps`); a typo gets a did-you-mean error. Note the `_mm` and `_deg` suffixes — map axes are one of the deliberate display-boundary unit exceptions.
- `constant:` (optional) — the degenerate case for cars whose aero does not vary with attitude.

The F1 reference car uses both: a four-axis map for the T1 tier plus a constant fallback (`data/vehicles/f1_2026/vehicle.yaml`):

```yaml
aero:
  map: aero/f1_2026.parquet
  axes: [ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag]
  constant:
    cx_a_m2: 1.25
    cz_front_a_m2: 1.9
    cz_rear_a_m2: 2.6
```

The Model 3 instead points `map:` at `aero/none.parquet`, a file that deliberately does not exist. This is the documented "fixture idiom": sidecar tables are decoded later, at assembly time, and an absent one is skipped with a note in the result, letting the `constant:` block carry the whole model. A zero-lift road car is exactly this degenerate case.

#### 4.2.3 `suspension` — lumped K&C

`model: lumped_kc` selects the only suspension model in v1: **lumped kinematics and compliance (K&C)** — instead of modelling every link and bushing, each axle is summarized by a handful of effective rates (`crates/outlap-schema/src/vehicle/suspension.rs`). Per axle (`front:` / `rear:`, type `AxleKc`):

| Field | Meaning | Required? |
|---|---|---|
| `ride_rate_n_per_m` | vertical stiffness at the wheel, N/m (how hard the wheel pushes back per metre of compression) | yes, > 0 |
| `roll_stiffness_share` | this axle's fraction of the car's total roll stiffness, 0..1 — it steers how lateral load transfer splits front/rear (Chapter 2) | yes |
| `roll_center_height_m` | height of the axle's roll centre, m | yes |
| `static_ride_height_m` | design ride height at rest, m — the platform the T1 aero-map equilibrium compresses under downforce | estimable |
| `anti_dive` / `anti_squat` | geometric anti-pitch fractions | estimable → 0 |
| `camber_map` / `toe_map` | wheel-angle-vs-travel map references | estimable → identity |

"Estimable" means: if you omit the field, the load pipeline fills it from a documented heuristic and *tells you so* in the loaded-model report (§4.4). The Model 3 file above omits all five estimables; the F1 car sets `static_ride_height_m: 0.040` / `0.090` explicitly because its ride-height aero map actually consumes them.

#### 4.2.4 `tires` — two references, no defaults

```yaml
tires:
  front: tyr/road.tyr.yaml
  rear: tyr/road.tyr.yaml
```

Both axles must reference a `.tyr` tire document — tires are load-bearing and have "no sane default" (`crates/outlap-schema/src/vehicle/tires.rs`). The `.tyr` format itself (Magic Formula coefficients, thermal and wear blocks) is Chapter 5; the physics is Chapter 7. The referenced files are loaded and validated as part of loading the vehicle, so a broken tire file fails the vehicle load, not the lap solve.

#### 4.2.5 `drivetrain` — a topology graph, not a layout picker

This is outlap's versatility surface. There is no `layout: rwd` enum; instead the powertrain is a **directed graph**: torque *sources* connect through ordered *coupler* elements to wheel *sinks* (`crates/outlap-schema/src/vehicle/drivetrain.rs`). Any four-wheeled concept is a topology plus data.

- Each entry in `units:` is one torque source: `source:` (a `.ptm` powertrain-map reference — outlap's firewall rule means engines and motors are always consumed as measured/estimated map files, never modelled internally; Chapter 9), an optional `thermal:` (`.emotor` machine-thermal model, electric machines only), a `path:` of couplers, and the `wheels:` it drives (`FL`, `FR`, `RL`, `RR` — uppercase on the wire).
- Couplers are externally tagged YAML: `{gearbox: {...}}`, `{diff: {...}}`, or `{fixed_ratio: 2.4}`.
- A `gearbox` has `ratios:` (index 0 = first gear), `final_drive`, `shift_time_s`, and an `efficiency` that defaults to a constant 0.985 (or can be a `{map: ...}` reference).
- A `diff` (differential — the device that lets left and right wheels turn at different speeds) has `type:` one of `open | locked | lsd | solid`. For `lsd` (limited-slip) and `locked`, `preload_nm` is conditionally required; `ramp: [accel, decel]` is LSD-only.
- A defaulted `control:` block carries static torque splits and a torque-vectoring flag (`ΔM_z = k_yaw · (r_target − r)` yaw-moment control, an M4 runtime feature — the schema accepts it today and topology-checks it).

The Model 3 is the simplest real topology — one drive unit through an open diff to the rear wheels. The F1 car shows a fuller one (`data/vehicles/f1_2026/vehicle.yaml`):

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

Because a lumped `drive_unit` `.ptm` already includes its gearbox ratio, the loader refuses to put one behind *another* gearbox or fixed ratio — one of the topology checks in §4.5.

#### 4.2.6 `ers` — the hybrid energy-recovery block (optional)

The F1 car carries the full block: `mgu_k:` (a `.ptm` for the motor-generator unit), `es:` (energy store capacity in MJ plus an allowed state-of-charge window), `deployment:` (a power limit in kW with a taper-vs-speed table), an optional `override_mode:`, and `recovery:` limits. Two fields are estimable: `deployment.per_lap_deploy_mj` (defaults to the full usable capacity) and `override_mode.extra_energy_per_lap_mj` (defaults to 0). At v0.2 the ERS is enforced as a power cap; the per-lap energy manager is the M6 milestone (Chapter 9, Physics III, and Chapter 15, Limitations and roadmap).

#### 4.2.7 `battery` — a selector plus a reference

```yaml
battery:
  model: rc_pairs
  params: battery/pack_800v.battery.yaml
```

`model: rc_pairs` is the only variant (a Thevenin equivalent-circuit model; Chapter 9), and `params:` points at a separate `battery/1.0` document. One honesty note: the vehicle load pipeline validates the referenced tires, ERS machine, and drive-unit `.ptm`/`.emotor` files — but **not** `battery.params` or the aero map, which are only resolved at assembly time (`crates/outlap-schema/src/load/mod.rs`, `load_referenced`). The shipped `f1_2026` car actually references a `battery/f1_es.yaml` that does not exist in its directory; the vehicle loads and solves fine, with a note that the slow-state stack is inert.

#### 4.2.8 `brakes` — balance, discs, regen

- `balance_bar` — the front brake bias as a fraction 0..1 (0.62 = 62 % of brake torque to the front axle).
- `abs:` — whether an anti-lock system is fitted (default `false`).
- `disc.front` / `disc.rear` — per-axle disc thermal capacity (J/K), cooling area (m²), and an optional pad-friction-vs-temperature map.
- `regen_blend:` (optional) — for cars that recover braking energy: `max_regen_frac` caps the fraction of total brake torque supplied by regenerative braking (the motor acting as a generator), with an optional `front_bias` that defaults to the friction balance.

### 4.3 Inheritance, merging, and what-if overrides

#### `extends:` — single-parent inheritance

A vehicle can inherit from a preset — a partial vehicle fragment — and override only what differs. The mechanism (`crates/outlap-schema/src/load/merge.rs`) is:

- **Single-parent chains only.** `extends:` names one parent, which may itself extend another; cycles are detected and rejected ("`extends` cycle detected: ... is already in the chain"). YAML's own anchors, aliases, and `<<` merge keys are deliberately *rejected* at parse time — inheritance goes through `extends:` and nothing else, so provenance stays traceable.
- **Mappings merge key-by-key; sequences and scalars replace wholesale** (child wins). So overriding `chassis.mass_kg` keeps the rest of `chassis`, but touching `drivetrain.units` replaces the entire list.
- The reference is loaded verbatim, with a `.yaml` extension fallback only when the ref contains no dot — `extends: presets/formula_base` finds `presets/formula_base.yaml`.
- The resolved model has `extends` stripped away, and every value remembers where it came from (a provenance map of JSON pointers → origins: base file, inherited-from-preset, override, or estimated).

From the test fixtures (`crates/outlap-schema/tests/fixtures/ev_child/vehicle.yaml`):

```yaml
schema: vehicle/1.0
extends: presets/ev_base.yaml
name: "EV child — lightweight"
chassis:
  mass_kg: 1590.0   # overrides the preset's 1700.0; other chassis fields inherited
```

One caveat for v0.2: the mechanism is fully implemented and tested, but the shipped `data/presets/` directory is currently **empty** — the class presets (formula, GT, passenger) promised by the roadmap have not landed yet. Presets today exist only as test fixtures.

#### Dotted-path overrides — the what-if API

Programmatic overrides never edit files. You pass dotted paths (Decision #35), applied *after* the merge and *before* validation, so an overridden value goes through exactly the same checks as a hand-written one:

```python
lap = solve_lap(
    "data/vehicles/tesla_model3_rwd", track, tier="t1",
    overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"},
)
```

That is the drive-unit sizing swap from the shipped notebook 07 — no YAML was touched, and the applied override is recorded in the provenance map and reflected in the resolved hash (§4.4). Numeric segments index into an **existing** list element; overrides never grow a list. Out of bounds is a real error:

```text
ValueError: override index `3` is out of bounds (sequence has 1 items)
```

### 4.4 Estimation and the loaded-model report — nothing is silent

When you omit an estimable field, outlap fills it from a documented heuristic (`crates/outlap-schema/src/load/estimate.rs`) — and *always* tells you. The current heuristics:

| Field (JSON pointer) | Heuristic | Filled value |
|---|---|---|
| `/suspension/{front,rear}/static_ride_height_m` | `static_ride_height_nominal` | 0.030 m front / 0.050 m rear |
| `/suspension/*/anti_dive`, `anti_squat` | `anti_dive_zero` / `anti_squat_zero` | 0.0 |
| `/suspension/*/camber_map`, `toe_map` | `camber_identity` / `toe_identity` | none installed — report-only ("assumed zero change with travel") |
| `/ers/deployment/per_lap_deploy_mj` | `per_lap_deploy_capacity` | = the store's `capacity_mj` |
| `/ers/override_mode/extra_energy_per_lap_mj` | `override_extra_energy_zero` | 0.0 |

Every load produces a **loaded-model report** (`LoadedModelReport` in `crates/outlap-schema/src/load/report.rs`) with four lists — `inherited`, `estimated`, `degraded`, `warnings` — plus `resolved_hash`, a blake3 hash of the canonical (key-sorted) resolved parameter set that results embed and the envelope cache is keyed on. From Python:

```python
from outlap.core import vehicle_report
r = vehicle_report("data/vehicles/tesla_model3_rwd")
```

Real output for the shipped Model 3 (warning-clean, ten estimated entries):

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

Two honest footnotes. First, `allow_degraded: true` in `sim.yaml` is the project's *single* documented fallback path — it permits documented-fallback combinations and marks the results (Decision #40); it is threaded from the sim settings into solver assembly. Second, at v0.2 the load-time `degraded` list is a contract-level placeholder: no degraded combination is populated during loading yet (`crates/outlap-schema/src/load/mod.rs` carries the literal note "`allow_degraded` recorded here once degraded combos exist"), so you will only see the flag's effects at assembly/solve time.

### 4.5 Validation and the error experience

Loading a vehicle is a staged pipeline (`crates/outlap-schema/src/load/`): parse (span-preserving) → version gate → extends-merge + overrides + provenance → unknown-key walk → one post-merge deserialize → semantic checks → referenced-file loads → topology-graph checks → estimation → report. Config errors are treated as a product surface — the typed error (`SchemaError`, `crates/outlap-schema/src/error.rs`) has one variant per stage, each carrying the offending file and a byte span so miette (the Rust diagnostics library) can render an underlined, plain-language message. A bare serde error reaching you is, by project rule, a bug.

**Unknown fields are hard errors** — except `x-*` extension keys, which are carried through uninterpreted and each produce a report warning ("extension key `x-...` carried through (not interpreted)"). The unknown-key walk checks your document against the generated JSON Schema and attaches a Levenshtein-distance suggestion. Misspell `chassis:` as `chasis:` in a copy of the Model 3 file and the Python surface gives you:

```text
ValueError: unknown field `chasis`
help: did you mean `chassis`?
```

(Rust consumers get the full miette rendering with the file name and the key underlined; the Python boundary flattens it to the message plus the help line, and maps a missing file to `FileNotFoundError` instead.) If your file declares a newer schema MINOR than the build understands, the unknown-key error gains a hint that the key may be a field added in a newer schema version.

**Semantic checks** run on the typed model: positivity (masses, wheelbase, ride rates, `dt_s`), unit intervals (`balance_bar`, `roll_stiffness_share`, `max_regen_frac`), known aero axis names, ascending taper/SoC arrays, conditional requirements (an `lsd` diff without `preload_nm` fails with the help text "add `preload_nm: <N·m>` to this diff"), and more (`crates/outlap-schema/src/load/semantic.rs`). For example, setting `roll_stiffness_share: 1.58`:

```text
ValueError: `suspension.front.roll_stiffness_share` must lie in [0, 1]
```

**Topology checks** validate the drivetrain graph as a whole (`crates/outlap-schema/src/load/topology.rs`), each with plain-language messages and one or more labelled spans:

1. at least one drive unit;
2. every unit drives at least one wheel, with no duplicates;
3. a lumped `drive_unit` `.ptm` (ratio already applied) must not sit behind a gearbox or fixed ratio — the message suggests `meta.upstream_ratio_applied: false` or removing the coupler;
4. a wheel rigidly driven by two or more units with no differential anywhere in the driving paths is rejected ("over-constrains the wheel speed" — parallel hybrids sharing a diff pass);
5. torque vectoring cannot be enabled across a `locked`/`solid` diff feeding a full axle.

Finally, two YAML strictness rules worth internalizing: duplicate keys are hard parse errors, and anchors/aliases/`<<` merge keys are rejected (use `extends:`).

### 4.6 The track: `track.yaml` + `centerline.csv`

A track is a directory holding a thin descriptor plus the geometry data. The descriptor (`TrackDoc`, `crates/outlap-schema/src/track.rs`) for the reference Catalunya (`data/tracks/catalunya_osm/track.yaml`):

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

- `closed` defaults to **true** (a closed loop gets a periodic spline and a closure check); point-to-point courses must opt out explicitly.
- `banking_keypoints:` (optional) is a sparse list of `{s_m, banking_deg}` pairs, interpolated along arc length; when present they **override** the centerline's banking column. `s_m` must be non-negative and strictly ascending.
- `meta` carries provenance: `source`, the `dem` (digital elevation model) used to fuse elevation, an `accuracy_class` (`A` surveyed, `B` DEM-fused, `C` estimated), a redistribution `attribution` string, and free-form `notes`.

The geometry lives in `centerline.csv`: eight required, header-named, order-independent columns — `s_m, x_m, y_m, z_m, banking_deg, width_left_m, width_right_m, grip_scale` — where `s_m` is arc length along the centerline, `x/y/z` are 3D coordinates, the widths give the drivable corridor either side, and `grip_scale` locally scales friction. `#` comment lines and blanks are skipped. Validation gives 1-based line numbers, a did-you-mean on missing columns, and checks strictly-increasing `s_m`, finite coordinates, and positive widths and grip (`crates/outlap-schema/src/centerline.rs`). The actual spline fit, curvature, grade, and road frame are built downstream by the `outlap-track` crate.

The repository ships 26 circuits: 25 flat 2-D circuits vendored from the TUMFTM racetrack-database (LGPL-3.0 data, `accuracy_class: C`) plus `catalunya_osm`, the 3D OSM+DEM reference used by all notebooks and the validation cross-check. Careful: `catalunya` (flat TUMFTM) and `catalunya_osm` (3D reference) are the *same circuit from two sources* — the notebooks and validation all use `catalunya_osm`. Full inventory in Chapter 12; the importers in Chapter 11.

### 4.7 `conditions.yaml` — same track, different day

Conditions capture the session environment (Locked Decision #46). Every field has a full ISA default — ISA is the International Standard Atmosphere, 20 °C and 1013.25 hPa here, with still air — so the entire file is optional (`crates/outlap-schema/src/conditions.rs`). The fields (note the deliberate °C/hPa display-boundary units):

| Field | Default | Meaning |
|---|---|---|
| `air.temperature_c` | 20.0 | air temperature, °C — with pressure, sets air density for aero |
| `air.pressure_hpa` | 1013.25 | absolute pressure, hPa (> 0) |
| `wind.speed_mps` | 0.0 | constant wind speed, m/s (≥ 0; a single vector in v1) |
| `wind.direction_deg` | 0.0 | meteorological convention — the direction the wind blows *from*: 0 = North, 90 = East |
| `track_surface_c` | 20.0 | track surface temperature, °C — the tire thermal boundary $T_\text{road}$ (consumed once the M5 tire thermal model lands; Chapter 15) |
| `ambient_c` | 20.0 | thermal-model ambient, °C — consumed by the `.emotor` machine-thermal network unless the emotor's own `cooling.ambient_fixed_c` overrides it |

A complete example from the test fixtures (`crates/outlap-schema/tests/fixtures/conditions/hot_dry.conditions.yaml`):

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

Air density comes from the ideal-gas law, $\rho = p/(R\,T)$ with $R = 287.05\ \mathrm{J\,kg^{-1}\,K^{-1}}$. The one shipped vehicle with its own conditions file uses exactly this: `data/vehicles/limebeer_2014_f1/conditions.yaml` sets 21.0 °C at 1013.25 hPa so that $101325/(287.05 \times 294.15) = 1.2000\ \mathrm{kg\,m^{-3}}$ — reproducing the air density published in Perantoni & Limebeer (2014) for the validation cross-check (Chapter 13).

From Python you can patch conditions per call without a file — `solve_lap(..., conditions={"air": {"temperature_c": 35.0}})` deep-merges onto the file/defaults and rejects unknown keys loudly.

### 4.8 `sim.yaml` — numerics and solver settings

The sim document configures *how* to solve, never *what* is being solved (Locked Decision #42). Every field is defaulted, and the **resolved** settings are embedded in every result artifact, so a result always records how it was produced. The full field set (`crates/outlap-schema/src/sim.rs`), shown here as the complete fixture `crates/outlap-schema/tests/fixtures/sim/qss.sim.yaml`, which spells out every default:

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

- **`tier`** — the solver fidelity: `t0` (point-mass with constant friction and a power cap), `t1` (the default: quasi-steady-state lap on the g-g-g-v envelope), `t2` (transient double-track), `t3` (full transient with a driver model). The same vehicle description drives all tiers — hard rule #4. At v0.2, `t2`/`t3` raise a typed "not implemented" error (they land in M4/M6); see Chapter 8, Physics II, for what T0 and T1 actually compute.
- **`dt_s`** (default 0.001 s) and **`integrator`** (`heun`, explicit trapezoidal 2nd order, or `rk4`) — the fixed integration step and scheme for the *transient* tiers; recorded now, exercised when T2 lands. Fixed-step only, by the determinism rules.
- **`fz_coupling`** (default `one_step_lag`) — the vertical-load algebraic loop mode (Decision #29). Tire forces depend on vertical loads, which depend on load transfer, which depends on the very accelerations the tire forces produce — an algebraic loop. `one_step_lag` breaks it by using the previous step's normal loads; `fixed_point` instead iterates a damped fixed-point to convergence within the step. Both are deterministic; the choice is a recorded simulation setting, and a property test pins that they agree at convergence. Physics details in Chapter 8.
- **`envelope`** — the sampling resolution of the g-g-g-v performance envelope the QSS tiers precompute: 40 speed × 25 longitudinal-acceleration × 7 normal-g points by default. Semantic floors: `v_points ≥ 2`, `ax_points ≥ 2`, `g_normal_points ≥ 1`. A coarse grid such as `{"v_points": 8, "ax_points": 7, "g_normal_points": 2}` is the shipped notebooks' idiom for cheap parameter sweeps.
- **`raceline`** — exactly one of `generator:` (v1 ships only `min_curvature`, a quadratic program over lateral offset minimizing $\int \kappa^2\,\mathrm{d}s$ on the 3D ribbon) or `file:` (your own line as an s-based CSV). Setting both, or neither, is a semantic error.
- **`allow_degraded`** (default `false`) — the single documented fallback escape hatch; degradations are recorded in the result metadata (§4.4).
- **`flat_track`** (default `false`, added in `sim/1.1`) — zero the track's grade, banking, and vertical curvature so the 3D envelope collapses to a flat g-g diagram. This is the 2-D oracle-comparison mode used by the Limebeer validation cross-check (Chapter 13); the physical track file is left untouched, and the flag is recorded in the results.

No vehicle in `data/vehicles/` ships a `sim.yaml` — defaults are the norm. Note the one asymmetry: in a *file*, the `schema: sim/1.1` line is required like in every outlap document, but the in-memory default fabricates it for you when no file exists. From Python, `sim={"flat_track": True, "envelope": {"v_points": 24}}` deep-merges onto the file/defaults (unknown keys rejected: ``unknown sim field `sim.x` (known fields here: ...)``), and the `tier="t0"` convenience argument wins over both.

### 4.9 Schema versions and the published JSON Schemas

Eight document kinds make up the wire contract: `vehicle`, `ptm`, `tyr`, `emotor`, `battery`, `track`, `conditions`, `sim` (`crates/outlap-schema/src/lib.rs`). The contract is a semver boundary: **additive changes bump MINOR; anything else bumps MAJOR and requires a migration** (`outlap migrate`). This build accepts `SCHEMA_MAJOR = 1` for every document and understands minors up to the crate-global `SCHEMA_MINOR = 4`; the bump history so far is all additive — `tyr/1.1` (brush tire block), `vehicle/1.2` (`static_ride_height_m`), `ptm/1.1` + `battery/1.0` (DC-voltage axis and the pack document), `sim/1.1` (`flat_track`). Shipped data mostly still declares `x/1.0`, which is legal — loaders gate on name + MAJOR only.

The machine-readable schemas live in `schemas/` — `vehicle.json`, `ptm.json`, `tyr.json`, `emotor.json`, `battery.json`, `track.json`, `conditions.json`, `sim.json` — as JSON Schema draft 2020-12, licensed **Apache-2.0** (unlike the AGPL-3.0 code), so any tool can validate outlap files without licence entanglement. They are generated *from* the Rust types (`cargo run -p outlap-schema --bin gen_schemas`), and CI fails if the committed files drift from the code (`--check`); the Python side validates the shipped data against them with `python -m outlap.schemas --check`. Chapter 5 continues from here into the referenced file formats themselves — `.ptm`, `.tyr`, `.emotor`, battery packs, and the parquet sidecar convention.


---

## 5. Files and formats: schemas, maps, and tables

*What you will learn: every file outlap reads or writes — what it contains, how it is versioned, and how to write one by hand. We walk through the schema contract that keeps files stable across releases, then each format in turn: powertrain maps (`.ptm`), tires (`.tyr`), machine thermal networks (`.emotor`), battery packs, tracks, and aero maps. We finish with the binary parquet sidecars and the single interpolation policy that every gridded table in outlap shares.*

Chapter 4 explained *which* four documents describe a simulation (vehicle, track, conditions, sim) and how they are loaded and validated. This chapter is the reference for the files themselves — the ones you will open in an editor, and the ones a vehicle document points at.

### 5.1 The schema contract: versioned, generated, checked

Every outlap YAML document begins with a `schema:` line naming its kind and version:

```yaml
schema: vehicle/1.0
```

The format is `<name>/<MAJOR>.<MINOR>` (parsed by `SchemaVersion` in `crates/outlap-schema/src/version.rs`; the name must be lowercase `[a-z_]+`). Eight document kinds exist: `vehicle`, `ptm`, `tyr`, `emotor`, `battery`, `track`, `conditions`, and `sim` (`crates/outlap-schema/src/lib.rs`). The name half is a safety net: a `.tyr` file fed where a vehicle is expected fails the version gate with a clear message instead of half-deserializing into nonsense.

The version halves follow semantic versioning ("semver"), the convention where the meaning of a version number is a compatibility promise:

- **Loaders gate on name + MAJOR only.** A file is accepted if its name and MAJOR match what the loader expects (`SchemaVersion::is_compatible_with`). MINOR is informational — a `vehicle/1.0` file loads fine in a build that understands `vehicle/1.2`.
- **Additive changes bump MINOR.** New optional fields never break old files. The crate-wide counter is `SCHEMA_MINOR = 4` in `crates/outlap-schema/src/lib.rs`, and its doc comment logs the history: 1 = the `tyr/1.1` brush block, 2 = the `vehicle/1.2` suspension `static_ride_height_m`, 3 = the `ptm/1.1` Vdc axis plus the new `battery/1.0` document, 4 = the `sim/1.1` `flat_track` flag.
- **Anything else bumps MAJOR** and requires a migration. A wrong-MAJOR file is rejected with the help text "run `outlap migrate` to update the file" (`crates/outlap-schema/src/load/mod.rs`).

One consequence worth knowing: if a file declares a MINOR *newer* than the build understands and the loader then hits an unknown key, the error explains that the key "may be a field added in a newer schema version" rather than just calling it a typo.

#### Where the schemas come from

The published JSON Schemas in `schemas/*.json` (Apache-2.0 licensed, unlike the AGPL-3.0 code) are not written by hand. The Rust types in `outlap-schema` derive `schemars::JsonSchema`, and the `gen_schemas` binary (`crates/outlap-schema/bin/gen_schemas.rs`) emits one draft 2020-12 JSON Schema per document kind:

```bash
cargo run -p outlap-schema --bin gen_schemas            # regenerate schemas/*.json
cargo run -p outlap-schema --bin gen_schemas -- --check # fail if committed files drifted
```

The `--check` form runs in CI on every commit, so the committed schemas and the Rust types can never disagree. The Python side *conforms* rather than defines: `python/src/outlap/schemas.py` loads the committed schemas and validates the shipped fixtures and every `data/**/*.tyr.yaml` with the `jsonschema` package (`python -m outlap.schemas --check`, also wired into CI). A pydantic v2 mirror is planned but not yet implemented in v0.2 — today `jsonschema` validation is the Python contract check.

Two more rules from Chapter 4 that shape every format below: unknown keys that do not start with `x-` are hard errors (with a did-you-mean suggestion), while `x-*` vendor-extension keys are carried through with a warning; and everything a loader estimates or degrades is surfaced in the loaded-model report — nothing is silent.

### 5.2 `.ptm` — the neutral powertrain map

A `.ptm` file (by convention named `<name>.ptm.yaml`) is how *any* torque source enters outlap: electric drive unit, bare machine, or combustion engine. It is deliberately a **map, not a model** — a table of what the unit delivers and what it wastes, over speed and load. outlap never simulates the electromagnetics of a motor or the combustion of an engine internally; this boundary is called the *firewall* (`crates/outlap-schema/src/ptm.rs`). Chapter 9, Physics III, covers what the solver does with these numbers.

The document has seven required fields — `schema`, `kind`, `axes`, `tables`, `limits`, `inertia_kgm2`, `mass_kg` — plus optional `meta` (`schemas/ptm.json`). Here is a shipped example, the Tesla Model 3 study's medium drive unit (`data/vehicles/tesla_model3_rwd/ptm/du_medium.ptm.yaml`, a synthetic dataset):

```yaml
schema: ptm/1.1
kind: drive_unit
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
inertia_kgm2: 1.4
mass_kg: 82.0
meta:
  dc_voltage_v: 790.0
  upstream_ratio_applied: true
```

Field by field:

- **`kind`** is one of `electric_machine` (torque at the machine's own shaft; a downstream gear ratio may apply), `ice` (internal-combustion engine), or `drive_unit` (machine + inverter + gearbox lumped at the wheel-side shaft — the drivetrain topology must *not* apply another ratio unless `meta.upstream_ratio_applied: false`).
- **`axes`** declares the grid. `speed_rpm` is the shaft-speed axis (rpm is a file-format boundary unit; internally everything is rad/s). `load_axis` is written as either `{torque_nm: [...]}` or `{load_fraction: [...]}` — a load fraction runs −1..1 where negative is the regeneration (braking-recovery) quadrant. `vdc_v` is the optional **DC-link voltage axis introduced by `ptm/1.1`**: when present, the sidecar tables become a 3-D `(speed_rpm, torque_nm, vdc_v)` tensor and the solver evaluates them at the battery's state-of-charge-dependent terminal voltage (Chapter 9). It needs at least two strictly ascending breakpoints. When absent the map is single-voltage, measured at the scalar `meta.dc_voltage_v`.
- **`tables`** points at the numeric sidecar (§5.8) and declares its columns: `efficiency` (default `true`; values 0..1 covering drive *and* regen quadrants) and `loss_w` (default `false`; a total power-loss column in watts, which must be consistent with efficiency if both are given). The shipped `du_medium.maps.parquet` is a long/tidy table of 210 rows = 7 speeds × 10 torques × 3 voltages with columns exactly `[speed_rpm, torque_nm, vdc_v, efficiency, loss_w]`. Per-component loss columns (winding loss vs iron loss, say) are a hook in the format — the `.emotor` loss routing can name a component column — but in v0.2 the lap loop only consumes total `loss_w`.
- **`limits`**: only `max_torque_nm_vs_speed` (the peak torque envelope, paired equal-length arrays) is required — it is what caps traction. `cont_torque_nm_vs_speed`, `overload`, and `drag_torque_nm_vs_speed` are optional *validation references*, not the derating mechanism: sustained thermal capability is computed by the `.emotor` model from the loss tables (Locked Decision #25).
- **`inertia_kgm2` / `mass_kg`**: rotational inertia referred to this map's shaft, and the mass attributed to the unit (which also feeds the `.emotor` mass heuristics, §5.4).

**The ICE variant** is supported from day one with the same schema. `data/vehicles/f1_2026/ptm/ice_v6.ptm.yaml` is a synthetic 1.6 L V6 (`kind: ice`, `schema: ptm/1.0`) with a torque-axis load and a negative `drag_torque_nm_vs_speed` curve for engine braking. For an ICE the sidecar `efficiency` column is brake thermal efficiency, and the runtime converts source power to a fuel-mass rate using a lower heating value of 43 MJ/kg (see Chapter 9).

### 5.3 `.tyr` — the tire document

A `.tyr` file (named `<name>.tyr.yaml`) describes one tire. Five blocks: `mf61`, optional `brush`, `thermal`, `wear`, and `provenance` (`crates/outlap-schema/src/tyr.rs`). The physics behind the coefficients is Chapter 7, Physics I; here is the file contract.

- **`mf61`** is a flat map of Magic Formula 6.1 coefficients — the industry-standard empirical tire model (Pacejka 2012) — keyed by their standard `.tir` names (`FNOMIN`, `PCX1`, `PKY1`, ...). This is a deliberate design choice: the coefficient names are the interchange vocabulary of tire data, so outlap validates them as a keyed map rather than inventing ~150 renamed fields. Two structural keys are always required: `FNOMIN` (nominal load, N) and `UNLOADED_RADIUS` (m). The eight-coefficient pure-slip force core `PCX1, PDX1, PEX1, PKX1, PCY1, PDY1, PEY1, PKY1` is required *unless* a `brush:` block supplies the force model instead. Unknown coefficient names are **warnings** with a did-you-mean hint, carried through unvalidated — unlike unknown schema *fields*, which are hard errors. Absent optional coefficients fall back to documented defaults and absent whole families degrade gracefully (no `QSX*` → overturning moment ≡ 0, and so on), each degradation logged in the loaded-model report.
- **`brush`** (requires `schema: tyr/1.1`) is the four-parameter physical fallback model: `c_kappa_n`, `c_alpha_n_per_rad`, `mu0`, `patch_half_length_m`, plus `pressure_profile: parabolic` (the only option). Declaring a brush block in a `tyr/1.0` file is a warning, not an error.
- **`thermal`** has 15 named fields for the M5 tire-temperature model. In v0.2 all of them are inert placeholders *except one*: `p_cold`, the cold inflation pressure, which is the solve-time operating pressure. **Unit trap: `p_cold` is in kPa** (a `.tir`-lineage convention), converted to Pa at the code seam. `t_cold` is in °C.
- **`wear`** (10 fields) is entirely reserved for the M5 wear/cliff model; shipped datasets carry clearly labelled synthetic placeholders.
- **`provenance`** is required and is how tire data stays honest: `citation` (the literature source of the coefficients), `source` (a human-readable note), and `synthetic: bool` (default `false`).

The Model 3's road tire (`data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml`) is a nice worked example — it is a verbatim transcription of the published Pacejka (2006) 205/60R15 book tire:

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
  # SYNTHETIC placeholder — the thermal ring model lands in M5.
  p_cold: 220.0        # kPa (load-bearing today: the solve-time pressure)
  t_cold: 20.0
  # ...
provenance:
  citation: "H. B. Pacejka, Tyre and Vehicle Dynamics, 2nd ed. (2006), Appendix 3, Table A3.1 (205/60R15 91V, 2.2 bar, ISO sign)"
  source: "MF6.1 force/moment coefficients transcribed verbatim from the book table; ..."
  synthetic: false
```

Because the coefficient vocabulary is the `.tir` vocabulary, outlap also ships a clean-room codec for the TNO `.tir` text format (`crates/outlap-schema/src/tir/` and the Python mirror `outlap.tir`): `python -m outlap.tir to-tyr in.tir -o out.tyr.yaml` converts an existing `.tir` into a `.tyr`. A `.tir` carries no thermal/wear physics, so the converter must synthesise those blocks (the default policy), mark the resulting provenance `synthetic: true`, and record the synthesis as warnings — the provenance block always tells you where a tire came from. See Chapter 11, Importers and tooling.

### 5.4 `.emotor` — the machine thermal network

A `.emotor` file (named `<name>.emotor.yaml`, `schema: emotor/1.1`) declares a lumped-parameter thermal network (LPTN) for one electric machine: a small graph of thermal masses ("nodes") connected by heat-flow paths ("edges"), which the solver marches in time to predict winding and magnet temperatures and derive a torque derate. It is referenced from a drive unit's `thermal:` field in `vehicle.yaml`. The physics — Crank–Nicolson integration, the heat-transfer correlations (Becker–Kaye, Kylander, Churchill–Chu, Gnielinski), the derate law — is Chapter 9; the format is:

- **`nodes`** (required, ≥ 2, at most 24 at runtime): each has a `name`, an optional `role` (`winding`, `stator_iron`, `rotor`, `housing`, `coolant`, `ambient`, `other` — at least one `winding` node is required, as the default loss target), an optional heat capacity `c_j_per_k`, and optional paired temperature limits `t_warn_c`/`t_max_c` (both or neither; only limit-carrying nodes participate in derating). Omitted capacities on role-tagged nodes are filled by documented mass heuristics from the `.ptm`'s `mass_kg` — and flagged as estimates.
- **`conductances`** (required): constant edges `{between: [a, b], w_per_k: ...}`; omit `w_per_k` for the mass heuristic.
- **`convection`** (optional): speed- and temperature-dependent edges `{between, area_m2, model}` where `model` is one of `air_gap`, `rotor_air`, `shaft_external`, `liquid_channel`, `free_convection` — each backed by a published correlation.
- **`cooling`** (required): names the pinned `ambient_node` (held at `conditions.yaml`'s `ambient_c` unless `ambient_fixed_c` overrides), and at most one of a low-level `coolant` spec or a high-level `jacket` block (raw cooling-channel geometry, from which assembly derives the coolant node balance and a liquid-channel edge). An optional `air_gap` block gives raw rotor/gap geometry for the stator–rotor film.
- **`loss_routing`** (optional): `{component?, node, fraction}` entries splitting the `.ptm` loss among nodes. Empty routing — or any unrouted fraction — lands on the winding node.
- **`cu_feedback`** (optional): copper-resistivity feedback, rescaling routed loss by $1 + \alpha\,(T - T_{\mathrm{ref}})$.
- **`initial_temp`** (optional): `{uniform_c: ...}` or per-node values; absent means every node starts at its sink temperature.
- **`meta.source`**: `datasheet | estimated | pdt_imported`.

The shipped example (`data/vehicles/tesla_model3_rwd/emotor/rear_du.emotor.yaml`, all values estimated) shows the whole menu in 42 lines — six role-tagged nodes, three constant edges, a jacket cooling block with ethylene-glycol coolant, an air-gap block, 55/30/15 loss routing to winding/stator/rotor, and copper feedback:

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

### 5.5 `battery/1.0` — the equivalent-circuit pack

The battery document (named `<name>.battery.yaml` by convention) describes a pack as a Thevenin equivalent-circuit model (ECM): an open-circuit voltage source behind a series resistance and one resistor–capacitor pair, with all four quantities tabulated over state of charge and temperature. The form follows the published NREL `thevenin` model (BSD-3) and the ECM literature it cites (Plett 2015); the runtime is Chapter 9. It is referenced from `vehicle.yaml`'s `battery: {model: rc_pairs, params: <path>}` block.

Required fields: `schema`, `model` (only `rc_pairs`), `topology` (`ns` cells in series × `np` in parallel), `capacity` (`q_pack_ah` for Coulomb counting; `e_pack_wh` informational), `soc_window` (`[min, max]` ascending in 0..1), `ecm`, `limits`, `thermal`. The `ecm` block declares `rc_pairs: 1` (the only supported count in v0.2), the `(soc, temp_c)` grid axes (each ≥ 2, strictly ascending), and the sidecar reference with its `level`: `cell` tables are scaled to pack level (voltage × ns, resistance × ns/np); `pack` tables are used as-is. The sidecar is a long/tidy parquet with columns exactly `soc, temp_c, ocv_v, r0_ohm, r1_ohm, tau1_s, dudt_v_per_k` — the shipped `pack_800v.tables.parquet` is 18 rows = 6 SoC × 3 temperatures.

The shipped pack (`data/vehicles/tesla_model3_rwd/battery/pack_800v.battery.yaml`, synthetic):

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

This particular pack is designed to be pedagogical: its 220 cells swing roughly 634–810 V open-circuit, so under low-SoC load the terminal voltage sags *below* the drive unit's 730–850 V `vdc_v` grid — deliberately exercising the below-grid linear extrapolation described in §5.9. One quiet caveat from Chapter 4: the vehicle load pipeline does not validate the `battery.params` reference (only tires, ERS, and drive-unit `.ptm`/`.emotor` files are loaded there); a missing battery file only surfaces as a solve-time note that the coupling is inert.

### 5.6 `track.yaml` + `centerline.csv` — the 3D track

A track is two files. `track.yaml` is a thin descriptor; the geometry lives in a CSV sidecar. From `data/tracks/catalunya_osm/track.yaml`:

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

`closed` defaults to **`true`** — a point-to-point track (hillclimb, test straight) must opt out with `closed: false`. Optional `banking_keypoints` (`[{s_m, banking_deg}]`, strictly ascending, `s_m ≥ 0`) are sparse banking samples interpolated in arc length; when present they *override* the centerline's `banking_deg` column. The `meta` block carries provenance: an `accuracy_class` (`A` surveyed, `B` DEM-fused, `C` estimated) and an `attribution` string required for redistributing ODbL/Copernicus-derived data. The 25 TUMFTM-derived tracks in `data/tracks/` are LGPL-3.0 (see Chapter 12, The shipped data library).

`centerline.csv` is plain CSV with exactly eight required columns, **header-named and order-independent** (`crates/outlap-schema/src/centerline.rs`):

```text
s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale
0.0000,237.8136,555.8796,137.5116,0.000,6.000,6.000,1.0000
3.0002,240.2026,554.0648,137.3254,0.000,6.000,6.000,1.0000
```

- `s_m` — arc-length station in metres, **strictly increasing** (a NaN is rejected too).
- `x_m, y_m, z_m` — world coordinates in the ISO 8855 frame (x forward, y left, z up), finite.
- `banking_deg` — banking angle in degrees (a display-boundary unit).
- `width_left_m, width_right_m` — track half-widths, > 0.
- `grip_scale` — a per-station friction multiplier, > 0 (1.0 = nominal).

Lines starting with `#` and blank lines are skipped, and every validation error carries a 1-based line number (a missing column even gets a did-you-mean from your actual header). Because it is CSV, this is the one input with no JSON Schema — the parser *is* the contract.

**Closure rules** (applied by `outlap-track` when it fits the periodic spline, `crates/outlap-track/src/lib.rs`): a closed track needs at least 4 points. If the first and last points coincide within $10^{-6}$ m, the duplicated last row is dropped and the loop period is its arc length; if they are distinct, the loop is closed over the connecting chord and the period becomes `s_last + chord`. If the start/finish gap exceeds 3× the median sample spacing, loading fails with "track marked closed but the start/finish gap is … — set `closed: false` or fix the centerline". Do not hand-close your CSV twice.

### 5.7 The aero map parquet

The vehicle's `aero:` block names a gridded map and its input axes (Chapter 4 shows the block; Chapter 7 the physics). The map itself is a parquet sidecar in the same long/tidy convention as everything else. Axis names must come from the known set in `crates/outlap-schema/src/load/semantic.rs`: `ride_height_f_mm`, `ride_height_r_mm`, `ride_height_mm`, `yaw_deg`, `roll_deg`, `steer_deg`, `drs_flag`, `speed_mps`. The value columns are the three lumped coefficients-times-area, in m²: `cz_front_a_m2`, `cz_rear_a_m2`, `cx_a_m2` (`crates/outlap-qss/src/t1/aero.rs`).

The shipped F1 map (`data/vehicles/f1_2026/aero/f1_2026.parquet`) is a 4-D grid of 250 rows = 5 front ride heights × 5 rear ride heights × 5 yaw angles × 2 DRS states, declared as:

```yaml
aero:
  map: aero/f1_2026.parquet
  axes: [ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag]
  constant:            # T0 fallback + sanity anchor
    cx_a_m2: 1.25
    cz_front_a_m2: 1.9
    cz_rear_a_m2: 2.6
```

All aero-map axes use clamping out-of-domain behaviour (§5.9). A road car with no map at all uses the degenerate `constant:` block alone — the Model 3 points `map:` at a deliberately absent `aero/none.parquet` so the constant coefficients carry, and the skip is recorded as a note ("aero map … not present — constant-aero fallback carries the lap").

### 5.8 Parquet sidecars: how binary tables travel

YAML is for structure and provenance; bulk numbers live in **sidecars** — separate binary files referenced by path. outlap uses Apache Parquet, a compact columnar table format, in one uniform shape: *long/tidy* `f64` columns, meaning one row per grid point with the axis coordinates repeated (`speed_rpm, torque_nm, vdc_v, efficiency, loss_w` rather than a 3-D array). By convention the sidecar sits next to the YAML that references it (`file: du_medium.maps.parquet`), and the loader resolves it there first, then falls back to the vehicle root.

The plumbing (`crates/outlap-schema/src/io.rs` and `sidecar.rs`):

```rust
pub trait SourceLoader {
    fn load(&self, path: &str) -> Result<String, SourceError>;
    fn load_bytes(&self, path: &str) -> Result<Vec<u8>, SourceError> { /* default: errors */ }
}
```

Every file access in outlap goes through this trait — `FsLoader` roots it at a directory (which is why all references inside `vehicle.yaml` are relative to the vehicle directory), and `MemLoader` serves the in-memory and browser paths. `load_bytes` exists for exactly one purpose: fetching sidecar bytes. Three properties are worth understanding:

1. **Decode happens at assembly time only.** `read_gridded_table(bytes, axis_names)` parses the parquet and pivots the long columns onto a rectilinear grid (`GriddedTable::from_long`), and the result is installed into the solver *before* the lap starts. Nothing in the hot loop ever touches parquet.
2. **`NULL` becomes NaN.** A missing cell in a column decodes to NaN — this is the masking convention for unreachable operating points (§5.9). Any non-numeric column is a hard error.
3. **The wasm strategy.** The parquet decoder pulls a dependency that cannot compile to WebAssembly, so it lives behind the non-default `parquet` cargo feature of `outlap-schema`. The *decoded* types (`GriddedTable`, `GriddedMapN`) live in the wasm-clean `outlap-core`, so the solvers never see parquet at all; browser builds simply ship pre-decoded tables through `MemLoader`.

A *missing* sidecar is a skip-with-a-note (the constant-aero or peak-envelope fallback carries the lap); a *present but undecodable* one is a real error. And because the resolved-vehicle hash covers only the YAML, the Python solver folds a fingerprint of every sidecar's bytes into its envelope cache key — two spec-identical cars with different tables never share a cached result (`install_sidecars` in `crates/outlap-py/src/lib.rs`).

### 5.9 One interpolant for every map

Tabulated data only becomes a continuous function through *interpolation* — estimating values between grid points. outlap has exactly **one** interpolation policy for every gridded map (Locked Decision #30): monotone cubic Hermite, in `crates/outlap-core/src/interp.rs` (1-D, `MonotoneCubic`) and `crates/outlap-core/src/gridmap.rs` (N-D tensor product, `GriddedMapN`, up to `MAX_DIMS = 6` axes). Powertrain efficiency, aero coefficients, battery ECM tables, torque envelopes, the track's per-station data channels — all of them go through this same code. Uniformity here is a correctness feature: no map behaves differently from another, and no solver needs to know which file its numbers came from.

A cubic Hermite interpolant fits, on each grid interval, a cubic polynomial through the two endpoint values $y_k$ and endpoint slopes $m_k$:

$$
f(x) = h_{00}(t)\,y_k + h_{10}(t)\,h\,m_k + h_{01}(t)\,y_{k+1} + h_{11}(t)\,h\,m_{k+1},
\qquad t = \frac{x - x_k}{h},\; h = x_{k+1} - x_k,
$$

where $h_{00} \ldots h_{11}$ are the standard Hermite basis polynomials. The slopes are what make it trustworthy: outlap limits them with the Fritsch–Carlson method (F. N. Fritsch and R. E. Carlson, "Monotone Piecewise Cubic Interpolation", *SIAM J. Numer. Anal.* 17(2), 1980), which caps each tangent so the curve **never overshoots the data** and is monotone wherever the samples are monotone — an efficiency map interpolated this way cannot invent an efficiency above its measured peak. The result is $C^1$ (value *and* slope continuous everywhere), and the derivative is available analytically — `MonotoneCubic::deriv` and `GriddedMapN::grad_into` return exact gradients with no finite differencing, which the Newton solvers in the transient tiers require. The N-D version applies the same 1-D tangent limiter successively along each axis, precomputing all mixed partials at every node at assembly time; along any grid-aligned line it coincides exactly with the 1-D interpolant.

#### NaN cells and the valid-data hull

Imported maps are often not full rectangles: a dyno cannot measure torque the machine cannot reach, so PDT-derived powertrain maps typically carry ~1.5 % NaN cells beyond the reachable envelope. `GriddedMapN` handles this at construction: NaN cells are filled by a deterministic nearest-valid breadth-first search over the grid, so the interpolant is total and $C^1$ — but the original NaN mask (the "hull" of genuinely measured data) is retained. Every evaluation whose *domain of dependence* — the surrounding cell corners plus the ±1 neighbours the tangent stencil reaches — touches a filled cell is flagged, so any result influenced by synthetic fill is identifiable. A map with no NaN cells skips the check entirely.

#### Out-of-domain: clamp or linear, per axis

Each axis carries an `OutOfDomain` mode:

| Mode | Behaviour outside the grid | Used by |
|---|---|---|
| `Clamp` (default) | Saturate at the edge value: constant, zero slope | Everything, unless stated otherwise: speed/torque axes of `.ptm` maps, all aero-map axes, battery ECM axes ("the ECM is only defined on its measured hull"), every `MonotoneCubic` curve |
| `Linear` | Extrapolate along the boundary tangent, $C^1$-continuous with the interior | The **`vdc_v` axis of Vdc-stacked `.ptm` maps only** (`T1Powertrain::install_maps`, `crates/outlap-qss/src/t1/powertrain.rs`) |

The one linear axis is deliberate physics, not laxity: a real 220-cells-in-series pack swings roughly 634–810 V over its SoC window, while a drive-unit map is typically gridded 730–850 V — so a large low-SoC band sits *below* the map. Clamping there would freeze efficiency at the 730 V slice; linear extrapolation follows the boundary trend instead, and the energy math floors the extrapolated efficiency to the physical range $[10^{-3}, 1]$. (Chapter 9 covers the full Vdc–SoC coupling.)

Nothing about leaving the grid is silent. Every query can return `EvalFlags` alongside its value:

```rust
pub struct EvalFlags {
    pub extrapolated: bool, // the query left the grid on at least one axis
    pub out_of_hull: bool,  // the stencil touched a NaN-filled (unmeasured) cell
}
```

and installing a Vdc-stacked map records a note in the loaded-model notes — "efficiency/loss map installed — energy accounting is live (Vdc-coupled; linear extrapolation below/above the voltage grid)" — so a lap that ran partly off-grid is documented in the run's report rather than discovered by surprise.

### 5.10 Which file means what: the summary table

| File / extension | `schema:` | What it holds | Binary sidecar |
|---|---|---|---|
| `vehicle.yaml` | `vehicle/1.x` | The car: chassis, aero, suspension, tire refs, drivetrain topology, brakes, optional ERS/battery | — (references everything below) |
| `track.yaml` | `track/1.0` | Track descriptor: name, `closed`, banking keypoints, provenance | `centerline.csv` |
| `centerline.csv` | — (CSV, no JSON Schema) | 8-column 3D centerline: `s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale` | is the sidecar |
| `conditions.yaml` | `conditions/1.0` | Environment: air temperature/pressure, wind, track surface and ambient temperatures (all defaulted) | — |
| `sim.yaml` | `sim/1.1` | Numerics: tier, dt, integrator, envelope grid, raceline source, `allow_degraded`, `flat_track` (all defaulted) | — |
| `*.ptm.yaml` | `ptm/1.0` / `ptm/1.1` | Neutral powertrain map: kind, speed/load(/Vdc) axes, torque limits, inertia, mass | `*.maps.parquet` (`efficiency`, `loss_w`) |
| `*.tyr.yaml` | `tyr/1.0` / `tyr/1.1` | Tire: MF6.1 coefficients (`.tir` names), optional brush block, thermal/wear (M5), provenance | — |
| `*.emotor.yaml` | `emotor/1.1` | N-node machine thermal network: nodes, edges, cooling, loss routing | — |
| `*.battery.yaml` | `battery/1.0` | Thevenin pack: topology, capacity, SoC window, ECM axes, power limits, lumped thermal | `*.tables.parquet` (OCV/R0/R1/τ1/dU-dT) |
| `*.parquet` | — | Long/tidy `f64` tables: powertrain maps, battery ECM tables, aero maps | is the sidecar |
| `*.tir` | — (TNO text format) | Industry tire-coefficient interchange; convert with `python -m outlap.tir` | — |

Three conventions to keep in your head as you write files: units are SI except at documented display boundaries (rpm on `.ptm` speed axes, °C in every `*_c` field, kPa in `.tyr` `p_cold`, degrees in `banking_deg`/`yaw_deg`); every path in a `vehicle.yaml` is relative to the vehicle directory; and every shipped data file's licence rides in its first line (data files are CC-BY-SA-4.0, the schemas Apache-2.0, the code AGPL-3.0-only). With the formats in hand, Chapter 6 zooms out to how the crates that read them fit together.


---

## 6. Architecture: how the code is organized

*What you will learn: how the Rust workspace is laid out crate by crate, and which crates are real versus reserved placeholders. How outlap splits all work into a cold "assembly pipeline" and a zero-allocation "hot loop", and — most importantly — exactly what data enters and leaves each stage on the journey from `vehicle.yaml` to an `xarray.Dataset`. Along the way you will meet the plugin-point roadmap, the WebAssembly cleanliness rules, the error-handling and determinism disciplines, and the point where Rust ends and Python begins.*

### 6.1 The workspace at a glance

outlap's Rust code lives in a single Cargo *workspace* — a collection of packages (Rust calls each package a *crate*) that build together and share pinned dependency versions. The root `Cargo.toml` declares `members = ["crates/*"]`, giving thirteen crates, all `edition = "2021"` and `license = "AGPL-3.0-only"`. Eight of them do real work today; five are two-line placeholders reserved for later milestones (each contains only an SPDX header and the doc line "placeholder crate; implemented in a later milestone").

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
| `outlap-vehicle` | stub | Reserved; vehicle assembly today is `outlap-schema::load` plus `T0Vehicle`/`T1Vehicle` in `outlap-qss` |
| `outlap-batch` | stub | Reserved for the batch/GPU rollout layer (`docs/HANDOFF.md` §11.3) |
| `outlap-transient` | stub | Reserved for the T2/T3 transient tiers (milestones M4/M6); requesting them today raises a typed error |
| `outlap-wasm` | stub | Reserved WebAssembly shell; currently empty but still the named target of the wasm CI gate (see §6.6) |

The dependency graph is shallow and strictly layered — math at the bottom, the user surface at the top:

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

Two deliberate oddities are worth calling out. First, `outlap-thermal` sits *below* the schema layer: it never sees an `.emotor` file. The mapping from an `.emotor` document to a thermal `Network` lives in `outlap-qss` (`src/t1/thermal.rs`), keeping the thermal math dependency-free. Second, `outlap-tire` does not depend on `outlap-core` at all — it consumes an already-loaded `outlap_schema::tyr::Tyr` and evaluates pure closed-form kernels.

#### The working crates, one paragraph each

**`outlap-core`** is the root of the graph and holds exactly the math every other layer shares. Its headline citizen is the project-wide interpolation rule made concrete: `MonotoneCubic` (`src/interp.rs:51`) is *the* one shared monotone cubic Hermite (C¹) interpolant — a curve that passes through every data point without inventing overshoots — used for every gridded lookup in the codebase (Decision #30, following Fritsch–Carlson-style monotone construction). `CubicSpline` (`src/spline.rs`) provides the C² splines track geometry needs, and `GriddedMapN`/`GriddedTable` (`src/gridmap.rs`) handle N-dimensional tables (up to `MAX_DIMS = 6`) such as powertrain efficiency maps and the g-g-g-v envelope.

**`outlap-schema`** is the wire contract: the serde/schemars types for all eight document kinds (`vehicle`, `ptm`, `tyr`, `emotor`, `battery`, `track`, `conditions`, `sim`), the staged load pipeline of §6.2, the miette-powered error types, and the `.tir` interchange codec. The committed JSON Schemas in `schemas/` (Apache-2.0) are *generated from* these Rust types by the `gen_schemas` binary, and CI fails if generated and committed drift apart. Chapter 5 covers the formats themselves.

**`outlap-tire`** implements the tire force backbone clean-room from Pacejka 2012 (3rd ed.): steady-state MF6.1 pure and combined slip $F_x$, $F_y$, plus the moments $M_z$, $M_x$, $M_y$, with the Besselink inflation-pressure terms; turn-slip is omitted in v1. A physical brush model (`src/brush.rs`) serves tires whose `.tyr` file lacks a full MF6.1 core, and `src/relax.rs` holds first-order slip relaxation for the future transient tiers. Kernels are pure, panic-free, allocation-free, and generic over `f32`/`f64`.

**`outlap-track`** turns `track.yaml` + `centerline.csv` into a queryable full-3D road ribbon (Decision #13): a C² spline for geometry (so curvature is continuous), monotone-cubic channels for banking, widths, and grip, and `road_frame(s)` queries the solvers consume. Its most architecturally interesting export is `offset_track` (`src/lib.rs:408`): any lateral offset profile becomes a *first-class* `Track` with its own curvature and frames — which is how a generated racing line is driven through the identical solver API.

**`outlap-thermal`** is the machine lumped-parameter thermal network: published heat-transfer correlations (Churchill–Chu free convection, Gnielinski channel flow, Becker–Kaye air-gap, and others, each cited in `src/correlations.rs`) feeding a `Network` of up to `MAX_NODES = 24` temperature nodes advanced by an unconditionally stable Crank–Nicolson step. Its detailed authoring tier ports correlations from the author's own PDT work — the one documented, deliberate amendment of the powertrain firewall (Decision #25, as amended 2026-07-05; `src/lib.rs:17-19`). Chapter 9 covers the physics.

**`outlap-qss`** is where laps get solved: the `T0Path` sampler, the `T0Vehicle`/`T1Vehicle` assemblies, the forward/backward velocity-profile solver (`src/solver.rs`, re-implemented from Heilmeier et al. 2020 on the 3D ribbon of Perantoni & Limebeer), the T1 damped-Newton trim (`src/t1/trim.rs`), the g-g-g-v envelope generator (`src/t1/envelope.rs`, after Werner et al. 2025), tier dispatch, and the slow-state coupling that marches battery and machine temperatures along the lap. Its optional `parallel` feature (rayon) accelerates envelope generation on native builds only.

**`outlap-raceline`** generates the minimum-curvature racing line (Decision #14): minimising $\int \kappa^2\,ds$ over the lateral offset $n(s)$ within the track bounds is a convex quadratic program with box bounds, solved with `clarabel` and re-implemented from the published formulation (Braghin et al. 2008; Heilmeier et al. 2020 §3.1–3.2) — never from the LGPL TUM source.

**`outlap-py`** is the boundary crate, described in §6.9.

Runnable examples live inside the crates rather than a repo-root `examples/` directory: `crates/outlap-qss/examples/` (e.g. `catalunya_lap.rs`, `limebeer_lap.rs`, `ggv_traces.rs`) and `crates/outlap-raceline/examples/catalunya_line.rs`. They emit CSV consumed by `python/tools/plot_*.py`, so every figure in the theory pages is generated by the real Rust models.

### 6.2 The two worlds: assembly pipeline vs the hot loop

Everything in outlap belongs to one of two worlds.

The **assembly pipeline** is the cold path: it runs once per model load, is allowed to allocate memory, read files, build strings, and fail with rich diagnostics. Its job is to turn human-friendly YAML into compact, immutable, numbers-only structs. Concretely, loading a vehicle runs the staged pipeline in `crates/outlap-schema/src/load/mod.rs`:

1. **Load + parse** — fetch text through the `SourceLoader` trait and parse with span-preserving YAML (`marked-yaml`); YAML anchors, aliases, and duplicate keys are rejected.
2. **Version gate** — the `schema: vehicle/1.x` header must name the right document kind and MAJOR version.
3. **`extends` resolve + deep-merge + overrides** — single-parent preset inheritance, then dotted-path overrides (e.g. `chassis.mass_kg`), every value tagged with its provenance (`Origin`).
4. **Unknown-key walk** — any key that is not in the schema and does not start with `x-` is a hard error with a did-you-mean suggestion.
5. **Single post-merge deserialize** into the typed `Vehicle` struct.
6. **Semantic checks** — ranges, signs, cross-field rules.
7. **Topology-graph checks** — the drivetrain source→coupler→wheel graph must make physical sense.
8. **Estimation** — documented heuristics fill missing derivable values and report them.
9. **Resolved-set hash** — a blake3 hash of the canonical resolved parameter set, recorded in every result.

Downstream of the schema pipeline, but still cold: track spline fitting (`Track::from_doc`), path sampling (`T0Path::from_track`), solver-vehicle assembly (`T0Vehicle::assemble`, `T1Vehicle::assemble`), parquet sidecar decoding, and g-g-g-v envelope generation. The architectural promise (from `docs/HANDOFF.md` §6.2b) is that after assembly the hot loop touches *zero* strings, hashes, or config logic.

The **hot loop** is the solve itself. Its rules are non-negotiable and CI-enforced:

- **Zero heap allocations per step.** The solve kernels write into caller-owned, pre-allocated workspaces. A test using the `dhat` allocation profiler (`crates/outlap-qss/tests/alloc.rs`) asserts that `solve_into`, `solve_into_ggv`, `T1Vehicle::trim`, `MachineThermal::step`, `Pack::step_power`, and the envelope boundary queries allocate exactly zero heap blocks; CI runs it in release mode alongside the ≤ 50 ms lap wall-clock gate (`.github/workflows/ci.yml` lines 19–21).
- **No Python inside a timestep, ever** — controllers included; they are Rust or C-ABI only (HANDOFF §6.2b, Decision #38).
- **Enum dispatch, not dynamic dispatch.** Model choices are resolved at assembly time into plain Rust enums (like `TireModel::Mf61 | Brush`) or *monomorphised* generics — the compiler stamps out a specialized copy of the sweep for each grip model (`trait GripModel` in `crates/outlap-qss/src/solver.rs`), so there is no per-station virtual call.
- **SoA state.** Data is stored as structure-of-arrays — one contiguous `Vec<f64>` per channel rather than an array of per-station structs — so the sweeps stream linearly through memory.

For the transient tiers (T2/T3, future), each fixed timestep will run the four phases `sense → control → actuate → integrate` (HANDOFF §6.2b). At the QSS tiers shipped in v0.2 there is no timestep — see Chapter 8, Physics II — but the same cold/hot split applies to the arc-length sweeps.

### 6.3 Data flow: one lap, end to end

This section is the chapter's core: what actually enters and leaves each function on the way from files on disk to a labelled dataset. The running example is the shipped Tesla Model 3 (`data/vehicles/tesla_model3_rwd/`) on Catalunya (`data/tracks/catalunya/`), solved from Python:

```python
from outlap.core import Track, solve_lap_dataset

track = Track.load("data/tracks/catalunya")
ds = solve_lap_dataset("data/vehicles/tesla_model3_rwd", track)
```

#### 6.3.1 Hop 1 — files on disk

The input quartet (Chapter 4) enters as YAML plus one CSV. The vehicle directory holds `vehicle.yaml` and the files it references (relative paths, resolved against the vehicle directory):

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

The track directory holds `track.yaml` plus `centerline.csv` (columns `s_m, x_m, y_m, z_m, banking_deg, width_left_m, width_right_m, grip_scale`). Optional `conditions.yaml` and `sim.yaml` may sit next to `vehicle.yaml`; when absent, full defaults apply (ISA atmosphere; tier `t1`).

#### 6.3.2 Hop 2 — schema types: `Vehicle` → `ResolvedVehicle`

`solve_lap` roots a filesystem loader at the vehicle directory (`FsLoader::new(vehicle_dir)`) and calls `load_vehicle_with("vehicle.yaml", …)`. The pipeline of §6.2 deserializes into the root schema struct (`crates/outlap-schema/src/vehicle/mod.rs:40`):

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

and wraps it (`crates/outlap-schema/src/load/mod.rs:47`):

```rust
pub struct ResolvedVehicle {
    pub spec: Vehicle,                // resolved, validated, extends applied
    pub provenance: ProvenanceMap,    // JSON pointer → Origin for every value
    pub report: LoadedModelReport,    // inherited/estimated/degraded/warnings + resolved_hash
}
```

Referenced `.tyr`, `.ptm`, and `.emotor` files are loaded and validated at this stage too; the `battery.params` document and binary sidecars are deferred to assembly time.

#### 6.3.3 Hop 3 — the road: `Track` → `T0Path`

`outlap-track` fits the centerline with a C² cubic spline (periodic for closed circuits) and the per-`s` channels with the shared monotone cubic Hermite. The solver does not query the `Track` in its loop; instead `T0Path::from_track(&track, ds_m)` samples it once at a uniform arc-length step (default `DEFAULT_DS_M = 2.0` m; Catalunya's 4 649.8 m becomes 2 325 stations) into a structure-of-arrays snapshot (`crates/outlap-qss/src/path.rs:24`):

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

So *per station* the solver's road inputs are exactly: curvature (split into lateral and normal components on the banked road plane), the three gravity-projection trig factors encoding grade and banking, and a grip scale. Signs follow ISO 8855 (x forward, y left, z up): $\kappa_h > 0$ is a left turn, positive grade is uphill, positive banking raises the left edge. If `sim.flat_track` is set, `from_track_flat` zeroes grade, banking, and vertical curvature instead.

#### 6.3.4 Hop 4 — solver vehicles: `T0Vehicle` and `T1Vehicle`

Two cold assembly functions reduce the same `ResolvedVehicle` + `Conditions` to numbers-only solver structs (Hard rule #4: one vehicle description, every tier). The point-mass reduction (`crates/outlap-qss/src/vehicle.rs:59`):

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

Note what happened at this hop: tire files became two friction coefficients (evaluated from the validated MF6.1 model at nominal load and cold pressure — not raw coefficients), `conditions.air` became an air density via the ideal-gas law inside `qx`/`qz`, and the drivetrain graph was folded into precomputed per-gear constants. Its one hot query is `tractive_force(v) -> f64` — speed in, available drive force out, allocation-free.

The double-track reduction `T1Vehicle` (`crates/outlap-qss/src/t1/vehicle.rs:31`) keeps much more: `mass_kg`, `izz`, CG-to-axle distances `a_f`/`b_r`, `wheelbase_m`, track widths `t_f`/`t_r`, CG height `h_cg`, roll-axis and roll-centre heights, roll-stiffness shares, the *full per-axle tire models* (`tire_front`/`tire_rear: TireModel<f64>`), cold pressures, constant/reference aero terms `qx`/`qz_f`/`qz_r`, air density `rho`, ride rates, static ride heights, anti-dive/anti-squat, an optional ride-height/yaw `AeroMap`, the driven-wheel mask, brake bias, and the topology powertrain. It powers the trim solver and the envelope generator (Chapter 8).

At the native edge only, `outlap-py` then installs binary parquet sidecars into the T1 vehicle (`install_sidecars`, `crates/outlap-py/src/lib.rs:744`) — the aero map and the `.ptm` efficiency/loss tables — and builds the optional slow-state stack (`build_slow_stack`: machine thermal `Network` + battery `Pack`) from the vehicle's own `battery.params` and `.emotor` refs. Missing files are skipped with a recorded note; present-but-broken files are hard errors.

#### 6.3.5 Hop 5 — the g-g-g-v envelope

Before the lap solve, `GgvEnvelope::generate(&t1_vehicle, &sim.envelope, fz_coupling)` sweeps the T1 trim over a grid (default 40 speed × 25 normalised-longitudinal-acceleration × 7 `g_normal` points) and stores the tire-grip boundary as `GriddedMapN` tables plus Decision #31 sensitivity fields, a reference `drag_accel(v)` curve, and `mass_ref` (`crates/outlap-qss/src/t1/envelope.rs:153`). This is a seconds-scale cold step, so `outlap-py` caches it per process, keyed by the resolved hash, a sidecar fingerprint, the conditions, the grid, and the coupling mode. The physics is Chapter 8's; here it is just one more immutable assembly product handed to the solver.

#### 6.3.6 Hop 6 — the solve: what goes in, what comes out

Tier dispatch happens once, at assembly time, in `solve_lap` (`crates/outlap-py/src/lib.rs:1011`): `Tier::T2 | T3` return a typed error; `t0`/`t1` both assemble the T1 vehicle (the envelope needs it), generate or fetch the envelope, assemble the T0 vehicle, and call `solve_t0` or `solve_t1` (`crates/outlap-qss/src/qss.rs`).

The hot kernel is `solve_into_ggv(vehicle, envelope, path, workspace) -> Result<f64, T0Error>` (`crates/outlap-qss/src/solver.rs:423`). Its inputs per station `i` are precisely the `T0Path` slices above; its scratch state is the caller-owned, pre-allocated workspace (`crates/outlap-qss/src/result.rs:27`):

```rust
pub struct T0Workspace {
    pub v_lim: Vec<f64>,  // curvature-limited speed per station, m/s
    pub v: Vec<f64>,      // solved speed per station, m/s
}
```

The sweep fills `v_lim`, runs a traction-limited forward pass and a braking-limited backward pass, and takes pointwise minima — allocating nothing. The owning wrapper packages the channels (`crates/outlap-qss/src/result.rs:56`):

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

The tier-dispatch layer wraps that in `QssLap` (`crates/outlap-qss/src/qss.rs:110`): the `LapResult` plus the recorded `tier`, `fz_coupling`, and `flat_track`, and three optional logs — `wheels: Option<WheelLog>` (per-station `[FL, FR, RL, RR]` arrays of `vertical_load_n`, `slip_ratio`, `slip_angle_rad`, `force_long_n`, `force_lat_n`; t1 only, produced by re-trimming every station), `setup: Option<SetupLog>` (`understeer_gradient`, `aero_front_share`; t1 only), `slow: Option<SlowLog>` (`state_of_charge`, `machine_temp_c`; present whenever a coupled battery+thermal stack was active, at either tier) — plus the returnable `envelope: Option<GgvEnvelope>`.

#### 6.3.7 Hop 7 — across the PyO3 boundary to xarray

`qss_lap_to_py` (`crates/outlap-py/src/lib.rs:1069`) converts `QssLap` into the frozen Python `Lap` class: it reconstructs world positions `x, y, z` by querying the track at each station, flattens the per-wheel logs into row-major `n × 4` buffers, and stringifies the enums (`tier="t1"`, `fz_coupling="one_step_lag"`). Channel methods (`lap.v()`, `lap.vertical_load_n()`, …) return fresh numpy arrays; per-wheel and setup channels return `None` on a t0 lap.

Finally the pure-Python veneer `outlap.core.lap_dataset` (`python/src/outlap/core.py:84`) assembles the result-boundary object the project commits to (Decision #17): an `xarray.Dataset` with dimension `s` (arc length, m) and — when per-wheel channels exist — `wheel` (`FL/FR/RL/RR`); up to 16 data variables (`v, ax, ay, t, x, y, z`, the five per-wheel channels, `understeer_gradient`, `aero_front_share`, `state_of_charge`, `machine_temp_c`); and attrs `lap_time_s`, `resolved_hash`, `tier`, `fz_coupling`, `flat_track` (an int — netCDF attrs have no bool type), and `notes` (a tuple). For the Model 3 on Catalunya this is a 2325-station, 595 kB dataset with `lap_time_s = 148.081…`. The full contract is Chapter 10's.

The whole journey in one picture:

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

The innermost physics call has the same disciplined shape. The T1 trim builds, for each wheel, a contact-patch state (`crates/outlap-tire/src/slip.rs:30`):

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

and calls `TireModel::forces(&SlipState) -> TireForces` — a match on the static enum (`Mf61` or `Brush`), each arm a pure, panic-free, allocation-free kernel generic over `f32`/`f64`. The output (`slip.rs:67`):

```rust
pub struct TireForces<T> {
    pub fx: T,  // longitudinal force, N
    pub fy: T,  // lateral force, N
    pub mz: T,  // aligning moment, N·m
    pub mx: T,  // overturning moment, N·m
    pub my: T,  // rolling-resistance moment, N·m
}
```

Signs follow the ISO-W convention of modern `.tir` files (ISO 8855 axes): positive `alpha` slides the patch to +y (left) and produces *negative* `fy` on a normal tire — the module doc in `slip.rs` documents every sign trap. Chapter 7, Physics I, covers what happens inside.

### 6.5 The three plugin points (roadmap, not yet shipped)

The project's extension policy is Locked Decision #37 (`docs/HANDOFF.md` §6.2b): **exactly three plugin points**, and everything else stays a curated core enum so the hot path never grows dynamic dispatch:

1. **Custom blocks** — a Rust trait with compile-time registration: a plugin crate depends on `outlap-core`, registers its blocks, and users build a custom binary (or upstream the block).
2. **Tire models** — a stable C-ABI "Standard Tire Interface" (CPU-only by contract), so a closed or third-party tire model can be loaded as a shared library.
3. **Controllers** — the same trait mechanism, running as `control`-phase blocks (Rust or C-ABI only; never Python in a timestep).

Be aware of the status: **none of these exist in code at v0.2.0.** There is no plugin trait, no registration mechanism, and no C-ABI header anywhere in `crates/` yet; HANDOFF §12 schedules plugin traits and Python entry points for a later milestone. What ships today is the curated-enum half of the decision — e.g. `outlap_tire::TireModel`, "the static (no-`dyn`) choice" (`crates/outlap-tire/src/model.rs:2`), which picks MF6.1 when the full pure-slip force core is present and the brush model otherwise. If you want a custom model today, the path is a fork, not a plugin.

### 6.6 wasm-clean rules

outlap's core is required to compile for `wasm32-unknown-unknown` — WebAssembly with no operating system — which forbids filesystem access, threads, and clocks. This is a forcing function for good layering: all source access goes through the `SourceLoader` trait (`crates/outlap-schema/src/io.rs:36`), with `FsLoader` behind `outlap-schema`'s `std` feature and the parquet decoder behind its `parquet` feature; wasm consumers use the in-memory `MemLoader` and depend on the schema crate with `default-features = false`.

CI enforces four wasm builds (`.github/workflows/ci.yml` lines 23–26): `outlap-wasm` (release), `outlap-raceline` (the `clarabel` QP solver is wasm-clean), `outlap-tire`, and `outlap-qss` (with the rayon-backed `parallel` feature off, keeping the solver thread-free). `outlap-core`, `outlap-track`, and `outlap-thermal` are wasm-clean by construction and come along as dependencies. `outlap-py` is the deliberate exception — "this crate never builds for wasm" (`crates/outlap-py/Cargo.toml`), because PyO3 and numpy are native-only, and that is exactly why parquet decoding and envelope caching live there, on the native edge.

One honest caveat: `outlap-wasm` itself is an empty placeholder, so the CLAUDE.md gate `cargo build --target wasm32-unknown-unknown -p outlap-wasm` passes trivially; the builds that actually keep the core honest are the raceline/tire/qss ones.

### 6.7 Error-handling architecture

Errors are treated as a product surface, with a different tool at each layer:

- **Typed enums on every public API** (`thiserror`): `SchemaError` (one variant per pipeline stage), `TrackError`, `TireBuildError`, `T0Error`, `T1Error`, `QssError`, `ThermalError`, `RacelineError`. Each variant states what went wrong and, where useful, how to fix it — e.g. `T0Error::NoConstantAero` points at `allow_degraded`, and `QssError::TierNotImplemented` names the milestone that ships the missing tier.
- **miette diagnostics at the config surface.** `SchemaError` variants carry the source file, byte-span labels, and `#[help]` hints, so a typo renders as an underlined snippet with "did you mean `mass_kg`?" (Levenshtein via `strsim`). A bare serde error reaching the user is considered a bug.
- **Panic-free solver kernels.** Hot-path functions return `Result` (e.g. `solve_into_ggv -> Result<f64, T0Error>`); physics invariants are checked with `debug_assert!` so release builds pay nothing. Every working crate except `outlap-py` carries `#![forbid(unsafe_code)]`; `outlap-py` is the sanctioned FFI (foreign-function interface) crate because PyO3's macros generate `unsafe` glue.
- **`anyhow` only at CLI edges** — the convention for future command-line binaries. As of v0.2.0 no shipped crate uses `anyhow` at all; every error is typed.
- **The Python boundary preserves the diagnostics.** `schema_err` (`crates/outlap-py/src/lib.rs:164`) maps a missing file to `FileNotFoundError` and everything else to `ValueError`, explicitly appending the miette help line ("Display alone drops them"). So the Python user sees ``ValueError: unknown field `masss_kg`\nhelp: did you mean `mass_kg`?``.

The philosophy throughout is "nothing silent" (Decision #41): a *missing* optional file (`sim.yaml`, `conditions.yaml`, a sidecar, the battery doc) falls back to defaults with a recorded note; a *present but malformed* one is always a hard error.

### 6.8 Determinism rules

The same inputs must produce bit-identical outputs, across runs and across thread counts. The rules, and where they live in v0.2:

- **Fixed-step integrators only** in production paths: `sim.integrator` offers Heun or RK4 at a fixed `dt_s` for the future transient tiers; the shipped thermal march uses an unconditionally stable Crank–Nicolson step. No adaptive step-size control anywhere.
- **Fixed iteration counts instead of tolerances where order matters:** the slow-state coupling runs exactly `OUTER_ITERS = 2` solve→march→re-solve passes — "fixed (not tolerance-driven) for determinism" (`crates/outlap-qss/src/qss.rs:43`).
- **Fixed-order reductions:** sums like the lap-time accumulation run in a fixed order; the optional rayon parallelism in envelope generation splits work over independent `(v, g_normal)` fibres and merges them in fixed order (`crates/outlap-qss/Cargo.toml`), so parallel and serial builds agree bitwise.
- **No fast-math:** no build flag anywhere relaxes IEEE 754 semantics.
- **Counter-based RNG keyed by `(seed, rollout, stream, step)`** (Philox/ChaCha8-style): this rule governs the Monte Carlo strategy layer (HANDOFF §11.3, line 1081). No RNG exists in the v0.2 core — the QSS solvers are fully deterministic functions — but the key structure is locked in now so batch rollouts can be replayed and sliced later.
- **Recorded numerics:** settings that change results — the Fz-coupling mode (`one_step_lag` vs `fixed_point`), `flat_track`, the resolved tier — are embedded in every result artifact, alongside the blake3 `resolved_hash` of the exact parameter set.

### 6.9 Python packaging: where Rust ends and Python begins

The split is deliberately thin. `crates/outlap-py` compiles to a single `cdylib` (a C-compatible shared library) named `outlap_core`, built by **maturin** against PyO3 0.29 with `abi3-py312` — one wheel per platform works on any CPython ≥ 3.12. Its own doc comment states the contract: "this layer only converts types and maps errors, never adds logic" (`crates/outlap-py/src/lib.rs:5-7`). It exposes the frozen classes `Tyre`, `Track`, `Raceline`, `Lap`, `Envelope`, the functions `solve_lap`, `min_curvature`, `vehicle_report`, and the constant `DEFAULT_DS_M`; typed stubs ship in `crates/outlap-py/outlap_core.pyi`. The crate sets `test = false` — a Rust test harness cannot link against Python, so the Python-side pytest suite is its test surface.

The pure-Python package `outlap` (`python/`, uv-managed, `requires-python >= 3.12`) declares `outlap-core` as a path dependency on `../crates/outlap-py`, so `uv sync` compiles the extension automatically (a Rust toolchain is required), with `cache-keys` covering the whole workspace's `*.rs` files so any Rust edit triggers a rebuild. The typed user API lives in `outlap.core` (`python/src/outlap/core.py`): numpy-style broadcasting for tire sweeps (`tyre_forces`) and the xarray converters (`lap_dataset`, `solve_lap_dataset`, `track_dataset`) — this is where results become labelled Datasets, the project's committed cross-boundary format. Two practical warts to know: `import outlap` itself currently exposes only a hello-world `main()` stub, so always import from `outlap.core`; and a debug-profile extension makes the first envelope generation take minutes, so set `MATURIN_PEP517_ARGS=--profile release` before `uv sync`, exactly as CI does (`.github/workflows/ci.yml` lines 30–34).

The division of labour, in one sentence each:

| Layer | Owns | Never does |
|---|---|---|
| Rust crates (`crates/*`) | All physics, validation, solving; every hot loop | Talk to Python mid-solve |
| `outlap-py` (extension) | Type conversion, error mapping, native-edge assembly (sidecars, envelope cache, slow stack) | Add physics or defaults |
| `outlap.core` (Python) | Broadcasting, xarray labelling, ergonomics | Re-implement anything the core computes |

Note that `solve_lap` currently holds Python's GIL (global interpreter lock) for its whole duration — releasing it is deferred to the batch/sweep milestone — so parallel laps from Python threads will not overlap yet. Chapter 10 covers the full Python surface; Chapter 11 covers the importers and tooling that feed it.


---

## 7. Physics I: tires and aerodynamics

*What you will learn: how outlap turns slip at the contact patch into forces with the Magic Formula 6.1 tire model, and what every field in the `SlipState → TireForces` contract means. How a `.tyr` file selects between the empirical MF6.1 model and the physical brush model, and how the T0 solver distills a whole tire into two friction numbers. On the aero side: the constant-coefficient road-car path, the ride-height/yaw downforce map, and the fixed-point "platform equilibrium" that couples the two.*

Tires and aerodynamics are the two force producers everything else in outlap serves. Every horizontal force that accelerates, brakes, or turns the car passes through four contact patches, each roughly the size of your hand. Aerodynamics decides how hard those patches are pressed into the road — and how much drag the powertrain must overcome.

This chapter explains both models as they are actually implemented, with the real struct fields, file formats, and shipped numbers. The primer in Chapter 2, A crash course in vehicle dynamics and lap simulation, gives the intuition; here we make it precise.

### 7.1 The tire crate and its contract

The tire model lives in `crates/outlap-tire`. It implements the steady-state Magic Formula 6.1 (MF6.1) plus a physical brush model and a first-order relaxation module reserved for the future transient tiers. It is implemented clean-room from Pacejka's book (H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., 2012, Chapter 4 §4.3.2, equations 4.E1–4.E78) with the inflation-pressure extensions of Besselink, Schmeitz & Pacejka (*Vehicle System Dynamics* 48(S1), 2010). The theory page `docs/theory/mf61-steady-state.md` carries the full equation map and citations.

Every evaluation kernel in the crate is pure, panic-free, allocation-free (enforced in CI), and generic over `f32`/`f64`. The crate does no file IO — it consumes a `.tyr` document already loaded by `outlap-schema` — which is what keeps it buildable for `wasm32-unknown-unknown`.

The whole crate speaks one input/output contract, defined in `crates/outlap-tire/src/slip.rs`:

```rust
pub struct SlipState<T> {
    pub kappa: T,      // longitudinal slip ratio, dimensionless; > 0 driving
    pub alpha: T,      // side-slip angle, rad
    pub gamma: T,      // inclination (camber) angle, rad
    pub fz: T,         // normal load, N (compressive-positive; <= 0 => all-zero output)
    pub p: T,          // inflation pressure, Pa
    pub vx: T,         // contact-center forward velocity, m/s (sign meaningful)
    pub mu_scale_x: T, // runtime longitudinal friction multiplier (M5 thermal hook; 1.0 today)
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

`SlipState::new(kappa, alpha, gamma, fz, p, vx)` fills both `mu_scale_*` fields with 1.0. The two multipliers are the hook through which the milestone-5 thermal model will one day modulate grip. Today their only production consumer is the T1 envelope generator, which perturbs them to measure grip sensitivity (Section 7.6 and Chapter 8, Physics II).

The five outputs, in words:

| output | name | what it is |
|---|---|---|
| `fx` | longitudinal force | drive/brake force in the wheel plane |
| `fy` | lateral force | cornering force |
| `mz` | aligning moment | the torque that tries to straighten the steered wheel — what you feel in the steering |
| `mx` | overturning moment | the contact patch's roll torque about the wheel's forward axis |
| `my` | rolling-resistance moment | the torque opposing rotation; a small, ever-present power drain |

### 7.2 Slip and the sign contract

A tire only produces force when its contact patch *slides* slightly relative to the road. The two sliding measures are:

- **Slip ratio** $\kappa$ (kappa) — how much faster or slower the tire surface moves than the road under it, longitudinally. outlap uses the ISO-W definition $\kappa = -V_{sx}/|V_{cx}|$, where $V_{sx}$ is the longitudinal sliding velocity of the contact patch and $V_{cx}$ the forward velocity of the wheel center. It is dimensionless (not a percentage): $\kappa > 0$ when driving, $\kappa < 0$ when braking, and $\kappa = -1$ is a locked wheel rolling forward.
- **Slip angle** $\alpha$ (alpha) — the angle between where the wheel points and where it actually travels: $\tan\alpha = V_{sy}/|V_{cx}|$, in radians.

The axes are ISO 8855 throughout: x forward, y left, z up. That convention has consequences the code treats as load-bearing, because a single stray absolute value silently breaks the physics:

- $\alpha > 0$ means the contact patch slides to +y (left), so a normal tire pushes back with $F_y < 0$. The cornering stiffness $K_{y\alpha} = \partial F_y/\partial\alpha|_0$ therefore carries the sign of the `.tir` coefficient `PKY1`, which is **negative** in ISO-W parameter sets (the Pacejka book tire ships `PKY1: -14.95`).
- The aligning moment $M_z = -t\,F_y + M_{zr}$ (pneumatic trail $t$ times the lateral force, plus a residual term) is restoring — it tries to reduce the slip angle — precisely *because* $F_y$ is negative for positive $\alpha$.
- $F_z \le 0$ (an airborne wheel) short-circuits every model to exactly-zero outputs (`TireForces::zero()`).
- Reverse running ($V_{cx} < 0$) enters only through $\operatorname{sgn}(V_{cx})$ factors inside specific equations. The implementation's `sgn` maps 0 to +1 — a branch selector, not a true signum — so a standstill does not annihilate the lateral force with a 0/0.
- **Camber** $\gamma$ (gamma), also called the inclination angle, is the lean of the wheel about its own x-axis: the top of the tire leans to +y for $\gamma > 0$.

One unit quirk to remember: `SlipState.p` is in pascal, but the `.tyr` file stores the cold inflation pressure `thermal.p_cold` in **kPa** (a file-format boundary, like RPM and °C elsewhere). Every consumer converts at the seam — for example `crates/outlap-qss/src/vehicle.rs` computes `cold_pressure_pa = 1000.0 * p_cold`.

### 7.3 The Magic Formula idea

The Magic Formula is not derived from physics — it is an empirical curve fit, an equation with just enough shape freedom to reproduce measured tire curves. Its core (Pacejka 2012) is a sine of an arctangent:

$$y(x) = D \sin\!\big(C \arctan\!\big(Bx - E\,(Bx - \arctan Bx)\big)\big)$$

where $x$ is a slip quantity ($\kappa$ or $\alpha$, plus a small horizontal shift $S_H$), $y$ is a force (plus a small vertical shift $S_V$), and four named factors sculpt the curve:

| factor | name | what it controls |
|---|---|---|
| $B$ | stiffness factor | the slope near zero slip (with $C$, $D$: origin slope $= BCD$, the slip stiffness) |
| $C$ | shape factor | how far past the peak the curve falls — the "character" of the falloff |
| $D$ | peak value | the maximum force — essentially $\mu F_z$ |
| $E$ | curvature factor | how sharp or gentle the peak is; clamped $\le 1$ in code (beyond 1 the curve folds back) |

Why this shape? For small $x$ the whole expression is nearly linear, $y \approx BCD\,x$ — the elastic regime where the rubber deflects without sliding. As $x$ grows, the arctangent saturates and the sine passes through its maximum $D$ — the grip peak — then falls off as more of the contact patch slides. That rise–peak–fall is exactly what every measured tire curve looks like, and the friction-circle story of Chapter 2 lives on top of it.

MF6.1 is the 2012-generation formulation of that idea: each of $B$, $C$, $D$, $E$, $S_H$, $S_V$ becomes a small polynomial in normalized load, inflation pressure, and camber, with named coefficients (`PCX1`, `PDY1`, `PKY1`, …) that a fitting tool identifies from test data. Two normalized inputs appear everywhere:

$$df_z = \frac{F_z - F'_{z0}}{F'_{z0}}, \qquad dp_i = \frac{p - p_0}{p_0}$$

$df_z$ is the fractional load deviation from the scaled nominal load $F'_{z0} = \lambda_{Fz0}\cdot\texttt{FNOMIN}$, and $dp_i$ the fractional pressure deviation from `NOMPRES`. This is how the model captures **load sensitivity** — the crucial fact that the friction *coefficient* falls as load rises (Chapter 2) — via terms like $\mu_x = (\texttt{PDX1} + \texttt{PDX2}\,df_z)(\dots)$, with `PDX2` negative on real tires.

### 7.4 MF6.1 as implemented

One `Mf61::forces(&SlipState)` evaluation (`crates/outlap-tire/src/mf61/`) composes five channels:

$$
\begin{aligned}
F_x &= G_{x\alpha}(\alpha^*)\cdot F_{x0}(\kappa) \\
F_y &= G_{y\kappa}(\kappa)\cdot F_{y0}(\alpha^*) + S_{Vy\kappa}(\kappa) \\
M_z &= -t(\alpha_{t,eq})\cdot(F_y - S_{Vy\kappa}) + M_{zr}(\alpha_{r,eq}) + s\cdot F_x
\end{aligned}
$$

plus $M_x$ (eq. 4.E69) and $M_y$ (eq. 4.E70). Turn-slip (parking maneuvers) is omitted in v1 — every $\zeta$ factor of the book equations is a named constant fixed at 1, so a later upgrade is a diff, not a rewrite.

#### 7.4.1 Pure slip: Fx0 and Fy0

$F_{x0}(\kappa)$ (eqs. 4.E9–4.E18, `mf61/fx.rs`) and $F_{y0}(\alpha)$ (eqs. 4.E19–4.E30, `mf61/fy.rs`) are the sine magic formula with load-, pressure-, and camber-dependent factors. The code is written in paper symbols with an equation-number anchor on each line — the actual formula line in `fx.rs` reads:

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

The shifts $S_H$/$S_V$ (ply-steer and conicity — small manufacturing asymmetries) mean real curves do not pass exactly through the origin, which is why the peak extractor of Section 7.6 scans both slip signs.

#### 7.4.2 Combined slip: cosine weighting

When a tire brakes *and* corners at once, each force steals grip from the other — the friction circle. MF6.1 models this with **cosine weighting** (eqs. 4.E50–4.E67, `mf61/combined.rs`), not a geometric friction ellipse: $F_{x0}$ is multiplied by a weight $G_{x\alpha} \in (0, 1]$ that is a normalized *cosine* magic formula in the other slip quantity $\alpha$, and symmetrically $F_{y0}$ by $G_{y\kappa}$, plus a small $\kappa$-induced ply-steer shift $S_{Vy\kappa}$. Cornering hard reduces the longitudinal force available, and vice versa — with a shape fitted to data rather than assumed elliptical. Each normalizing denominator carries a magnitude-floored, sign-preserving guard, because a hostile-but-plausible parameter set can genuinely drive it toward zero.

#### 7.4.3 Aligning moment and the minor moments

$M_z$ (eqs. 4.E31–4.E49 and 4.E71–4.E78, `mf61/mz.rs`) composes the pneumatic trail acting on the lateral force, a residual torque $M_{zr}$, and an $s\cdot F_x$ lever arm (the longitudinal force acting at a small lateral offset $s$). Three operational subtleties, all pinned by the golden cross-check and documented in `docs/theory/mf61-steady-state.md`:

- The entire aligning-moment lateral machinery is evaluated at **zero camber** — camber affects $M_z$ only through its own trail/residual coefficients. This matches the operational MF6.1 that `.tir` data is actually fitted against.
- The book writes a camber term in the $s$ lever arm (coefficients `SSZ3`/`SSZ4`); the operational implementations drop it, so outlap accepts those coefficients but does not use them (interoperability over book-literalism).
- The $s\cdot F_x$ term is combined-slip only: it is gated to $\kappa \ne 0$, a deliberate step discontinuity at exactly $\kappa = 0$ that matches the standard. The theory page explicitly warns not to "smooth" it — doing so breaks the golden cross-check.

$M_x$ (overturning) and $M_y$ (rolling resistance) consume the final combined forces (`mf61/mxmy.rs`). Rolling resistance opposes rotation: in ISO 8855 forward roll spins about +y, so $M_y < 0$ at $V_{cx} > 0$ — confirmed against the oracle goldens. For $M_x$, outlap takes the printed book form $\cos(\texttt{QSX5}\,(\arctan(\texttt{QSX6}\,F_z/F_{z0}))^2)$; the widely-used MFeval tool evaluates $\arctan(x^2)$ there instead — a known book-vs-tool discrepancy worth remembering if you compare outputs.

#### 7.4.4 Defaults and graceful degradation

A `.tyr` never needs the full ~150-coefficient set (`mf61/params.rs` extracts what is there into a dense typed struct, once, at assembly). Absent coefficients default to 0, except:

| default | coefficients | why |
|---|---|---|
| 1.0 | every `L*` scaling factor, `RCX1`, `RCY1`, `QCZ1`, `PKY2` | multiplicative identities; `PKY2` sits in an atan denominator |
| 2.0 | `PKY4` | a zero would collapse the cornering stiffness $K_{y\alpha} \equiv 0$ |
| 16.7 m/s | `LONGVL` (reference speed) | the book's conventional measurement speed |
| 1.0 m/s | `VXLOW` | reserved for the low-speed/relaxation model |

Absent `NOMPRES` disables all pressure terms exactly ($dp_i \equiv 0$, $p/p_0 \equiv 1$) — a pressure sweep on such a tire exercises nothing, and the loader says so. A wholly absent coefficient family degrades to zero output (no `QDZ*` ⇒ $M_z \equiv 0$; no `R*` ⇒ combined = pure), and **every** degradation is emitted as a note into the loaded-model report — nothing silent, per the assembly rules of Chapter 4.

### 7.5 The `.tyr` file and model selection

A tire ships as a `.tyr` YAML document (schema `tyr/1.0`, or `tyr/1.1` when it carries a brush block), with five blocks:

```yaml
schema: tyr/1.0
mf61:                       # MF6.1 coefficients, keyed by their standard .tir names
  FNOMIN: 4000.0            # nominal load, N        (required)
  UNLOADED_RADIUS: 0.313    # m                      (required)
  PCX1: 1.685
  PDX1: 1.210
  PKY1: -14.95
  # ... ~60-150 more, sparse files are fine
thermal:                    # thermal-ring parameters — M5 stubs today, EXCEPT p_cold
  p_cold: 220.0             # cold inflation pressure, kPa (load-bearing NOW)
  t_opt: 75.0               # ...
wear:                       # wear/cliff parameters — entirely M5-future
  k_w: 0.0006               # ...
provenance:
  citation: "H. B. Pacejka, Tyre and Vehicle Dynamics, 2nd ed. (2006), Appendix 3, Table A3.1 ..."
  synthetic: false
```

The excerpt is from the shipped `data/tires/pacejka_2006_205_60r15/car.tyr.yaml` — the book's own validation tire, which also happens to be the Tesla Model 3 reference vehicle's road tire, verbatim, at `data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml`. The same file appears as reference data, as the golden-test subject, and on the flagship road car.

Model selection happens once, at assembly, in `TireModel::from_tyr` (`crates/outlap-tire/src/model.rs`):

1. If the full pure-slip force core is present — the eight `REQUIRED_FORCE_KEYS` `PCX1, PDX1, PEX1, PKX1, PCY1, PDY1, PEY1, PKY1` — build **MF6.1** (the higher-fidelity model; a partial set never constructs one).
2. Otherwise, if a `brush:` block is present, build the **brush** model (Section 7.7), with a loaded-model note that $M_x = M_y = 0$ and camber/pressure are ignored.
3. Otherwise `TireBuildError::NoForceModel` (schema validation catches this earlier; the error is defensive).

`FNOMIN` and `UNLOADED_RADIUS` are always required. Unknown *coefficient names* inside `mf61:` are non-fatal — you get a did-you-mean warning — unlike unknown schema *fields*, which are hard errors (Chapter 5, Files and formats). The thermal and wear blocks are required by the schema but, apart from `p_cold`, are consumed by nothing until milestone M5; every shipped dataset carries clearly-labelled synthetic placeholders there. `p_cold` is the exception: it is the operating inflation pressure at which every solver evaluates the tire today.

A vehicle references its tires per axle — both required, no default:

```yaml
tires:
  front: tyr/road.tyr.yaml
  rear: tyr/road.tyr.yaml
```

Three literature-cited tire datasets ship in `data/tires/`: Pacejka's 205/60R15 book tire, the TUM Roborace DevBot set (MF5.2 mapped to MF6.1, peak $\mu_x \approx 1.46$ / $\mu_y \approx 1.16$), and an MF6.1 transcription of the Perantoni & Limebeer 2014 F1 tire whose load-linear peak friction (e.g. $\mu_x$ 1.75 at 2000 N → 1.40 at 6000 N) maps exactly onto the $(\texttt{PDX1} + \texttt{PDX2}\,df_z)$ form. Chapter 12, The shipped data library, covers their provenance in detail; the `.tir` interchange format (the industry text format `.tyr` converts to and from) is covered with the other tooling in Chapter 11.

### 7.6 How T0 distills a tire into peak μ

The T0 point-mass solver (Chapter 8, Physics II) has no slip states — it needs exactly two numbers per car: a peak longitudinal friction coefficient $\mu_x$ and a peak lateral $\mu_y$. Those are *extracted from the full tire model* at assembly time, in `crates/outlap-tire/src/mf61/peak.rs`:

$$\mu_x = \max_{\kappa}\frac{|F_x(\kappa)|}{F_z}, \qquad \mu_y = \max_{\alpha}\frac{|F_y(\alpha)|}{F_z}$$

evaluated at a fixed operating point: $\gamma = 0$, $V_{cx} = \texttt{LONGVL}$, and caller-chosen load and pressure. The search scans **both slip signs** (the shift terms make real curves asymmetric; the maximum of the two branches is the documented choice for a symmetric point-mass envelope) over the *physical* slip windows $\kappa \in [-1, 1]$ and $\alpha \in [-0.5, 0.5]$ rad ($\approx \pm 28.6°$), using a dense 256-point grid scan followed by 48 golden-section refinement iterations. A grid-plus-refine search is used instead of the closed-form peak $D/F_z$ because it is robust to the $E$-clamp edge cases and to shifted curves — and for a soft $C \le 1$ curve, whose supremum is only approached at unbounded slip, the window maximum is deliberately below the analytic asymptote: grip you can only reach at infinite slip is unusable.

T0 assembly (`crates/outlap-qss/src/vehicle.rs`) calls this per axle at `FNOMIN` and the cold pressure, then takes the **mean of the two axles**:

```rust
let mu_x = 0.5 * (axle_mu_x(&tm_front, &front) + axle_mu_x(&tm_rear, &rear));
let mu_y = 0.5 * (axle_mu_y(&tm_front, &front) + axle_mu_y(&tm_rear, &rear));
```

You can watch the load sensitivity yourself through the Python API (Chapter 10):

```python
from outlap.core import Tyre

t = Tyre.load("data/tires/pacejka_2006_205_60r15/car.tyr.yaml")
for fz in (2000.0, 4000.0, 6000.0, 8000.0):
    print(fz, t.peak_mu(fz, t.p_cold))
```

On the shipped book tire this prints (values rounded):

| $F_z$ (N) | $\mu_x$ | $\mu_y$ |
|---|---|---|
| 2000 | 1.228 | 1.119 |
| 4000 | 1.210 | 1.035 |
| 6000 | 1.191 | 0.950 |
| 8000 | 1.173 | 0.866 |

The lateral coefficient loses over 20% of its value as load quadruples — that is load sensitivity, the reason load transfer costs total grip (Chapter 2) — while this tire's longitudinal peak barely moves. At `FNOMIN` = 4000 N the T0 assembly would record $\mu_x = 1.210$, $\mu_y = 1.035$ for an axle of these tires. (This tire is a 2nd-edition set that records `NOMPRES` but carries no `PP*` pressure coefficients, so the pressure argument is inert here — though the loaded-model report stays *silent* about that: its pressure-disabled note fires only when `NOMPRES` is absent entirely, §7.4.4, not for a present-`NOMPRES`/absent-`PP*` tire like this one.)

A slip-angle sweep with `outlap.core.tyre_forces` shows the sign contract and the trail collapse in action — at `FNOMIN`, $\alpha$ = 0°, 2°, 4°, 8° gives $F_y \approx$ +42, −1506, −2696, −3627 N (the +42 N at zero slip is the ply-steer/conicity shift) and $M_z \approx$ −11.3, +33.9, +51.1, +24.9 N·m: the aligning moment peaks *before* the lateral force and then collapses as the trail shortens — the "light steering past the limit" cue racing drivers rely on.

For a brush tire the peak is simply its base friction $\mu_0$, load- and pressure-independent. The T1 tier, by contrast, keeps the full per-axle `TireModel<f64>` alive and evaluates it inside its trim solver with per-wheel $\kappa$, $\alpha$, $F_z$ — at $\gamma = 0$ (camber maps land in a later milestone; the assembly note says so out loud) and at the cold pressure. Its envelope generator perturbs `mu_scale` uniformly ($1 \pm 0.15$) to build the grip-sensitivity corrections of Chapter 8 — today's only production use of the `mu_scale_*` hooks.

### 7.7 The brush model

The brush model (`crates/outlap-tire/src/brush.rs`, theory page `docs/theory/brush-model.md`) is the physical, first-principles counterpart to MF6.1's empiricism, implemented from Pacejka 2012 Chapter 3. It exists for two reasons: it lets you define a usable tire from **four physical parameters** when no fitted coefficient set exists, and it is the pedagogical skeleton underneath the Magic Formula's shape — the reason tire curves rise, peak, and fall.

Picture the contact patch as a row of elastic bristles with a **parabolic pressure distribution** $p(x) \propto 1 - (x/a)^2$ over the contact half-length $a$. Under slip each bristle deflects linearly until its local shear exceeds the friction bound, after which it slides. Integrating the adhering and sliding regions gives a closed form. With theoretical slips $\sigma_x = \kappa/(1+\kappa)$ and $\sigma_y = \tan\alpha/(1+\kappa)$ (the $1+\kappa$ is ε-guarded so a locked wheel stays finite) and the reduced slip

$$\psi = \frac{\sqrt{(C_\kappa\sigma_x)^2 + (C_\alpha\sigma_y)^2}}{3\,\mu_0 F_z},$$

the force magnitude is the cubic brush law

$$|F| = 3\,\mu_0 F_z\,\psi\,(1 - \psi + \tfrac{\psi^2}{3}) \quad \text{for } \psi < 1, \qquad |F| = \mu_0 F_z \quad \text{for } \psi \ge 1 \text{ (full sliding)}.$$

Because $\psi(1-\psi+\psi^2/3)$ rises monotonically to $1/3$ at $\psi = 1$, the friction circle $|F| \le \mu_0 F_z$ is respected *by construction*. The pneumatic trail has the closed form

$$t = \frac{a}{3}\cdot\frac{(1-\psi)^3}{1-\psi+\psi^2/3}, \qquad M_z = -t\,F_y$$

— one third of the contact half-length at vanishing slip, zero at full sliding. $M_z$ is restoring under the same sign contract as MF6.1, and the trail collapse you measured numerically in Section 7.6 is here in closed form.

A brush block in a `tyr/1.1` document (this example is the synthetic test fixture `crates/outlap-schema/tests/fixtures/tyr/brush_only.tyr.yaml`):

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

The omissions are documented, not silent: camber and pressure are accepted and ignored, $M_x = M_y \equiv 0$, and assembly surfaces all of this as loaded-model notes. One sharp edge to know: the brush model is selected only by the `REQUIRED_FORCE_KEYS` gate in `TireModel::from_tyr`, which is what the QSS solvers call — so brush-only tires work there. The Python `Tyre` class (Chapter 10) does *not* go through that gate; it calls `Mf61::from_tyr` directly, which validates only `FNOMIN`/`UNLOADED_RADIUS`. So a brush-only `.tyr` loaded through `Tyre.load` does **not** error — it silently builds a degenerate all-zero MF6.1 model, peak μ and every force identically zero. Exercise a brush tire through the solvers, not through `Tyre`.

### 7.8 Fitting your own tire: `outlap.tirefit`

If you have tire test data — say from the FSAE Tire Test Consortium (TTC) — the `outlap.tirefit` package (`python/src/outlap/tirefit/`) fits an MF6.1 coefficient set from it:

```bash
python -m outlap.tirefit fit run1.mat run2.mat --unloaded-radius 0.26 -o car.tyr.yaml --report-dir report/
python -m outlap.tirefit synth car.tyr.yaml -o synth.csv --seed 0
```

Three stages, at a high level:

1. **Ingestion** (`tirefit/data.py`): TTC `.mat` (v7 and v7.3/HDF5), TTC ASCII `.dat`, and headered `.csv` are read into SI/ISO 8855 arrays. TTC channels arrive in the SAE tire axis system (z down); the conversion is the proper rotation by π about x — $\alpha$, $\gamma$, $F_y$, $F_z$, $M_z$ all negate — plus deg→rad, kPa→Pa, kph→m/s at this boundary only.
2. **Forward model** (`tirefit/mf61.py`): a vectorized numpy clean-room mirror of the Rust kernels, validated against the *same* committed golden CSVs and tolerance rule — so a parameter set fitted here evaluates identically inside the solver, including the operational conventions of Section 7.4.3.
3. **Staged fit** (`tirefit/stages.py`): a deterministic sequence — nominals → pure $F_{x0}$ → pure $F_{y0}$ → combined → $M_z$ → $M_x$ — each stage freeing a documented coefficient subset and minimizing load-normalized residuals with `scipy.optimize.least_squares` (install the `tire-fit` extra for scipy). Stages without data support are skipped and reported: camber coefficients are only freed if the data has camber spread, moment stages only if there is real signal. The `QSY*` rolling-resistance family is never freed, because TTC rigs log no $M_y$ channel — set those by hand from coast-down data.

The `synth` command generates a deterministic synthetic dataset from an existing `.tyr` in TTC-signed CSV form — the round-trip harness that proves the fit recovers known coefficients.

One policy you must know before using any of this: **parsers yes — redistribution of TTC data or TTC-derived parameter sets, no.** TTC data is membership-locked; keep raw files in the gitignored `ttc-data/` directory, and never publish files, fitted coefficient sets, or fit reports derived from them. Synthetic data and literature-cited sets are the only things that ship with outlap. The full tooling story, including the `.tir` codec CLI, is Chapter 11, Importers and tooling.

### 7.9 Aerodynamics I: the constant-coefficient path

Aerodynamics enters the vehicle description as the `aero:` block of `vehicle.yaml` (schema in `crates/outlap-schema/src/vehicle/aero.rs`). The block has two representations: a gridded map (the primary one, next section) and an optional constant block — the degenerate case that is entirely adequate for a road car:

```yaml
aero:
  map: aero/none.parquet    # deliberately-absent placeholder: the constant block carries
  axes: []
  constant:
    cx_a_m2: 0.51           # drag area C_x·A, m²
    cz_front_a_m2: 0.0      # front downforce area C_z,front·A, m²
    cz_rear_a_m2: 0.0       # rear downforce area C_z,rear·A, m²
```

That is the shipped Tesla Model 3 RWD: a public-figure drag area of 0.51 m² and no downforce. All three numbers are *coefficient times frontal area* products in m² — what a wind tunnel actually measures — so no separate reference area is needed.

At assembly, T1 folds in the air density $\rho$ from the session conditions (ideal gas: $\rho = 100\,p_{\mathrm{hPa}} / (287.05\,(T_{°C} + 273.15))$, `crates/outlap-qss/src/t1/vehicle.rs` — one place where the input quartet's separation of car and environment pays off) to make lumped terms in N per (m/s)²:

$$q_x = \tfrac{1}{2}\rho\,C_xA, \qquad q_{z,f} = \tfrac{1}{2}\rho\,C_{z,f}A, \qquad q_{z,r} = \tfrac{1}{2}\rho\,C_{z,r}A$$

so drag is $q_x v^2$ and per-axle downforce $q_z v^2$ — added straight into the static axle loads of the load-transfer model (Chapter 8). Downforce is why grip grows with speed and the g-g diagram widens into a funnel; drag is what the powertrain fights on the straights. With no constant block and `allow_degraded: true`, the aero degrades to zero with a recorded note; otherwise it is a load error.

The split into *front* and *rear* downforce areas matters even in the constant case: their ratio is the **aero balance** (the front axle's share of total downforce), which decides how the extra grip is distributed between the axles — and therefore whether downforce pushes the car toward understeer or oversteer.

### 7.10 Aerodynamics II: the ride-height/yaw map

A downforce car cannot be described by constants, because its coefficients depend on the body's position relative to the ground. Ground effect makes downforce rise as the floor gets closer to the road; rake (the nose-down pitch attitude) shifts the balance; yawing the body spoils the flow. The primary representation is therefore a gridded map (`crates/outlap-qss/src/t1/aero.rs`):

$$\{\,C_{z,\mathrm{front}}A,\; C_{z,\mathrm{rear}}A,\; C_xA\,\} = f(h_{\mathrm{front}},\, h_{\mathrm{rear}},\, \mathrm{yaw}\,[,\, \mathrm{DRS}])$$

The shipped F1 2026 reference car declares it like this (`data/vehicles/f1_2026/vehicle.yaml`):

```yaml
aero:
  map: aero/f1_2026.parquet
  axes: [ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag]
  constant:            # synthetic fallback used by tiers that don't consume the map
    cx_a_m2: 1.25
    cz_front_a_m2: 1.9
    cz_rear_a_m2: 2.6
```

The axis names are a fixed vocabulary — `ride_height_f_mm`, `ride_height_r_mm`, `yaw_deg`, `drs_flag` — anything else is a typed `UnknownAeroAxis` error, and axes absent from a map are simply not queried. Note the units: ride heights in millimetres and yaw in degrees *at the map boundary* (the same file-format-boundary convention as RPM and °C); internally everything returns to SI.

The sidecar is a long/tidy parquet table with those axis columns plus three value columns, `cz_front_a_m2`, `cz_rear_a_m2`, `cx_a_m2`. One `GriddedMapN` — the shared tensor-product monotone cubic Hermite interpolant (Fritsch & Carlson 1980; one implementation for *all* gridded maps — see Chapter 5, Files and formats) — is built per coefficient. Every axis **clamps** outside its domain: the platform equilibrium can push ride heights below the tabulated grid, and clamping holds the coefficients at their edge values rather than extrapolating a ground-effect curve past its validity.

### 7.11 The aero-platform equilibrium

Here is the loop that makes mapped aero genuinely different: the coefficients depend on the ride heights, but the ride heights depend on how hard the downforce compresses the suspension — which depends on the coefficients. A fixed point.

`AeroPlatform::equilibrium` (`crates/outlap-qss/src/t1/aero.rs`) solves it by damped fixed-point iteration. With dynamic pressure $q_{\mathrm{dyn}} = \tfrac{1}{2}\rho v^2$ and the longitudinal load transfer $T = m\,a_x\,h_{cg}/L$ moved onto the springs (anti-dive and anti-squat suspension geometry reacts part of it through the links instead — they modulate this *heave* path only, never the steady-state wheel loads), each axle's ride height is updated as

$$h_a \leftarrow h_a + 0.6\,\Big[\max\!\Big(0,\; h_a^{\mathrm{static}} - \frac{q_{\mathrm{dyn}}\,C_{z,a}A + F_{lt,a}}{2\,k_a}\Big) - h_a\Big],$$

the map is re-evaluated at the new heights, and the loop repeats — up to 60 iterations, to a ride-height tolerance of $10^{-10}$ m, with under-relaxation factor 0.6. That absurd-looking 0.1 nm tolerance is deliberate: it sits far below the trim solver's own residual tolerance, so the converged coefficients behave as a *smooth* function of the chassis state and the nested fixed point never injects an iteration-count discontinuity into the outer solver's finite-difference Jacobian.

In the update, $h^{\mathrm{static}}$ is the design ride height from `suspension.*.static_ride_height_m` (40 mm front / 90 mm rear on the F1 reference car), $k_a$ is the wheel ride rate, and the $\max(0,\cdot)$ clamp is the car "planking" on the road. The output, `AeroLumped`, carries the effective $q_x, q_{z,f}, q_{z,r}$ plus the converged ride heights, and a warm-start slot lets the trim solver reuse the previous evaluation's heights — cutting roughly 20 cold iterations to 1–2, which matters because this map evaluation is the dominant cost of a map-equipped trim.

The physics this buys: as speed rises, the platform sinks and rakes, ground effect strengthens, and the *aero balance migrates with speed* — the defining behaviour of a downforce car that no constant $C_zA$ pair can express. `T1Vehicle::aero_front_downforce_share_at(v)` reports it, and it lands per-station in the lap results (Chapter 10).

### 7.12 Yaw sensitivity and the mid-corner g-g diagram

The map's yaw axis is fed with the **body-slip angle** $\beta$ (beta) — the angle between where the chassis points and where it travels — evaluated in degrees *inside the trim residual* (`crates/outlap-qss/src/t1/trim.rs`). Because $\beta$ is one of the trim's unknowns, the finite-difference Jacobian automatically picks up $\partial(\mathrm{downforce})/\partial\beta$.

Mid-corner, $\beta$ is nonzero, so a yaw-sensitive car has genuinely *less downforce in the corner than the straight-line map value suggests* — and the attainable-acceleration (g-g) diagram reflects it. This is the mechanism that reshapes the g-g diagram mid-corner whenever the map carries a yaw dependence. The shipped map is even in yaw (a symmetric car has no left/right bias), which keeps the g-g left/right symmetric but *shrinks it off-center*, where combined slip drives $|\beta|$ up; a map with a genuine left/right asymmetry would skew the diagram itself.

DRS (the drag-reduction system — the rear wing opening on the straights) is always closed in the trim: its activation is a controller concern, not a physics one, so the `drs_flag` axis exists and is exercised by tests, but no shipped solve opens it yet.

### 7.13 How the `f1_2026` map was authored

The shipped map is **synthetic** — not measured, not simulated — and it says so everywhere. It is generated by `python/tools/gen_f1_aero.py`, which documents every assumption in its header. The construction:

- A 5 × 5 × 5 × 2 grid: front ride height {10, 20, 30, 40, 60} mm, rear {30, 50, 70, 100, 140} mm, yaw {−8, −4, 0, 4, 8}°, DRS {0, 1} — 250 rows of parquet.
- **Anchoring**: at the reference node (30 mm front, 70 mm rear, yaw 0, DRS closed) the coefficients reproduce the constant-aero fallback exactly — $C_{z,f}A = 1.9$, $C_{z,r}A = 2.6$, $C_xA = 1.25$ m² (total downforce area 4.5 m², about 1.7× the car's weight in downforce at 250 km/h, lift-to-drag ≈ 3.6). Those constants are a physically-plausible stand-in for the Perantoni & Limebeer 2014 reference-car aero, and the validation laps reconcile against the paper's published figures (Chapter 13).
- **Estimated sensitivities** (all labelled as estimates in the script): front/rear downforce rise linearly as the respective ride height drops (ground effect), with a mild cross-axle rake coupling; drag rises slightly as the platform lowers (induced drag); downforce falls and drag rises even-quadratically with |yaw| (−8% downforce at 10° yaw); DRS open multiplies rear downforce by 0.70 and drag by 0.82, front unchanged.
- The functional form is affine in each ride-height axis and even-quadratic in yaw, so every grid-aligned fibre is monotone or single-peaked — deliberately safe for the shape-preserving monotone-cubic interpolant.

The clean-room citations for the aero modelling are recorded in `docs/theory/t1-trim.md`: Perantoni & Limebeer 2014 (the reference car's speed-dependent aero, here generalized to explicit ride heights) and Katz, *Race Car Aerodynamics*, 1995 (ground-effect ride-height sensitivity and rake); the platform fixed point is a standard quasi-static heave balance.

### 7.14 What to trust, and what is not there yet

How these models are validated (details in Chapter 13, Validation, testing, and trust):

- **Golden cross-check**: all five MF6.1 channels of the Pacejka book tire match an independent Magic Formula implementation (the GPL `teasit` library run under GNU Octave — its numeric *outputs* used as data, never its source) to ≤ 0.5% over pure-longitudinal, pure-lateral (including ±4° camber), and combined sweeps.
- **Property tests**: sign pins, odd symmetry on shift-free subsets, friction-circle containment for the brush model, continuity across $\kappa = 0$ / $\alpha = 0$ / $V_{cx} = 0^+$, finiteness over hostile inputs, and a CI heap gate proving zero allocations per evaluation.
- **Reference-data gate**: every dataset under `data/tires/` must load warning-free, sit in a class-plausible grip band, and round-trip through the `.tir` codec numerically exactly.
- **Aero-map tests**: the committed F1 map reproduces the reference coefficients at the reference ride heights; a constant map degenerates to the constant-aero trim; the platform sinks monotonically with speed; DRS open cuts rear downforce and drag.

And the honest limits of what you have just read:

- **Turn-slip is omitted** (all $\zeta \equiv 1$), as is the velocity-digressive friction branch (`LMUV`) — v1 scope.
- **Tire thermal and wear are stubs.** The `.tyr` thermal/wear blocks are schema-required but consumed by nothing until milestone M5, except `p_cold`. All shipped values there are labelled synthetic placeholders. Until then, every solve is a *cold-tire* solve at fixed pressure.
- **Relaxation has no production caller yet.** The exact-exponential slip-lag stepper in `crates/outlap-tire/src/relax.rs` is implemented and property-tested, but the QSS tiers use steady-state forces directly; it lands with the transient tiers (T2/T3).
- **Camber is zero in T1** ("camber maps land later" — the assembly note), and the trim always runs DRS-closed.
- The **`f1_2026` aero map is synthetic** and its sensitivities are estimates; only the anchor point is reconciled against literature.
- The Python `Tyre` class always builds an MF6.1 model: brush-only files work in the solvers (via the `TireModel::from_tyr` gate) but load through `Tyre` as a degenerate all-zero model rather than erroring.

With forces at the contact patch and downforce on the axles in hand, Chapter 8, Physics II, assembles them into the T1 trim solve and the g-g-g-v envelope that actually produces a lap time.


---

## 8. Physics II: solving a lap — T0, T1, and the g-g-g-v envelope

*What you will learn: how outlap actually computes a lap time. You will follow the classic forward/backward velocity-profile method (T0) step by step, meet the nine-unknown equilibrium solve (the T1 "trim") that turns a full double-track car into a grip boundary, and see how that boundary is precomputed into the g-g-g-v envelope that connects the two. Along the way you will learn exactly what `tier="t0"` and `tier="t1"` return, and how flat-track mode collapses the whole machinery into the 2-D form used for validation.*

### 8.1 The big picture: two tiers, one envelope

Chapter 2 introduced the idea of a *quasi-steady-state* (QSS) lap solver: instead of integrating the car's equations of motion through time (an ODE solve), a QSS solver assumes the car is at its limit everywhere and asks, at each point along the track, "how fast can the car possibly be going here?" outlap's QSS machinery lives in the `outlap-qss` crate and is built from three cooperating pieces:

1. **The T1 trim** (`crates/outlap-qss/src/t1/trim.rs`) — given a speed and a commanded acceleration, find the steady-state balance of a full *double-track* car (four wheels, per-axle tire models, load transfer). It answers one question per call: *is this operating point physically achievable, and if so, what is every wheel doing?*
2. **The g-g-g-v envelope** (`crates/outlap-qss/src/t1/envelope.rs`) — a precomputed table of the trim's answers: the maximum lateral acceleration as a function of speed, longitudinal acceleration, and how hard the road presses the car down. Generated once per car (a cold, seconds-scale step in a release build), queried millions of times for free.
3. **The T0 velocity-profile solver** (`crates/outlap-qss/src/solver.rs`) — a point-mass sweep along the track that consumes the envelope and produces the speed trace and lap time. Not an ODE integration; three passes over an array.

The names are solver *tiers* (Chapter 4): `t0` is the point-mass profile, `t1` is the same profile plus a per-station re-solve of the trim to report per-wheel channels. Both evaluate the **same** vehicle description — tiers select fidelity, never a different car (Hard rule #4; `crates/outlap-schema/src/sim.rs:64-65`). The transient tiers `t2`/`t3` are future milestones and raise a typed error today (§8.5).

A perhaps-surprising fact worth stating up front: **`t0` and `t1` produce the identical lap time and speed profile.** Both run the same envelope-based velocity profile (`solve_profile` in `crates/outlap-qss/src/qss.rs`); `t1` then *re-trims* each station of the already-solved profile to log wheel loads, slips, and forces. On the shipped `f1_2026` around the `catalunya_osm` centerline (2 m stations, 2339 of them), both tiers return `lap_time_s = 112.548` — the `t1` dataset just carries seven more channels.

### 8.2 T0: the forward/backward velocity profile

#### 8.2.1 The idea, from scratch

Imagine driving a lap perfectly. Three things limit you:

- **In corners** you cannot exceed the speed at which the tires' sideways (lateral) grip is exactly consumed by turning. Tighter corner ⇒ lower ceiling.
- **Accelerating out of a corner** you are limited by engine power and by whatever longitudinal (fore-aft) grip is left over after cornering.
- **Braking into a corner** you are limited by grip alone (brakes are almost never the weak link).

The classic point-mass QSS method turns this into three array passes over the track, sampled at uniform arc-length stations (every `ds` metres of distance along the lap; the default step is `DEFAULT_DS_M = 2.0` m):

1. **Cornering ceiling.** For every station $i$, compute the *curvature-limited speed* $v_{\lim,i}$: the fastest speed at which lateral grip still meets the cornering demand. This ignores how you got there — it is a pure local ceiling.
2. **Forward pass (traction).** Sweep forward from the slowest point. From the speed at station $i$, compute the fastest speed reachable at station $i+1$ using all available drive force and remaining grip: $v_{i+1}^2 = v_i^2 + 2\,\Delta s\,a_{\text{accel}}$. Take the minimum with the ceiling. This kills every "you'd need infinite acceleration" violation.
3. **Backward pass (braking).** Sweep backward: from the speed at station $i{+}1$, compute the fastest speed at station $i$ from which the car could still slow down in time: $v_i^2 = v_{i+1}^2 + 2\,\Delta s\,a_{\text{brake}}$. Again take the pointwise minimum.

The result is the fastest profile that respects all three limits simultaneously. This is the `calc_vel_profile` formulation of Heilmeier et al. (*Vehicle System Dynamics* 58(10), 2020), re-implemented clean-room in `crates/outlap-qss/src/solver.rs` on the 3-D track ribbon of Perantoni & Limebeer (2015) — their *three-dimensional-track* paper, a different work from the 2014 variable-parameters paper used as the validation oracle in Chapter 13 — and Lovato & Massaro (2022); the module doc cites all three. Lap time is the fixed-order trapezoidal sum over segments,

$$t_{\text{lap}} = \sum_i \frac{2\,\Delta s}{v_i + v_{i+1}},$$

with the denominator floored at $10^{-6}$ m/s so a stationary station cannot divide by zero.

For a **closed lap** there is a subtlety: the passes need a starting speed, but the lap wraps around. The solver seeds at the station with the globally minimum $v_{\lim}$ (the slowest corner is lateral-limited, so its speed is a fixed point of the sweep), then iterates {one full forward wrap, one full backward wrap} until the seed speed stops changing (relative tolerance `SEED_TOL = 1e-6`, at most `MAX_PASS_ITERS = 8` iterations — a divergence backstop that has never triggered on a physical track; exceeding it is a typed `T0Error::PassesDiverged`, never a hang). An **open path** instead starts standing (`v[0] = 0`) and needs a single forward and a single backward sweep (`solve_generic`, `solver.rs:338-405`).

#### 8.2.2 The 3-D ribbon: how hills and banking enter

Chapter 4 described the track as a 3-D *ribbon*: a centerline with plan-view curvature $\kappa_h$ (how sharply it turns as seen from above), vertical curvature $\kappa_v$ (crests and dips), grade $\theta_g$ (uphill/downhill slope), and banking $\theta_b$ (side-to-side tilt). Before solving, the track is sampled once into a `T0Path` (`crates/outlap-qss/src/path.rs`) — a structure-of-arrays of plain `f64` slices, one entry per station, so the hot passes touch nothing but flat memory:

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

The two curvatures are the raw track curvatures projected into the tilted road plane (Perantoni & Limebeer 2015; `path.rs:8-9`):

$$\kappa_l = \kappa_h \cos\theta_g \cos\theta_b + \kappa_v \sin\theta_b, \qquad \kappa_n = \kappa_v \cos\theta_b - \kappa_h \cos\theta_g \sin\theta_b .$$

$\kappa_l$ is the curvature the tires must fight (cornering); $\kappa_n$ is curvature *out of* the road plane — a crest ($\kappa_n < 0$) unloads the car, a dip or compression loads it. Two derived quantities then carry the whole 3-D story into the grip model (`demand_and_gn`, `solver.rs:317-322`):

$$a_{y,\text{dem}} = \kappa_l v^2 + g \sin\theta_b \cos\theta_g \qquad \text{(signed lateral demand — banking of the right sign assists)},$$

$$g_{\text{normal}} = g \cos\theta_b \cos\theta_g + \kappa_n v^2 \qquad \text{(road-normal specific gravity)}.$$

$g_{\text{normal}}$ deserves a plain-language definition because it is the third axis of the envelope: it is *the effective gravity pressing the car onto the road*, in m/s². On flat ground it equals $g = 9.80665$ m/s² (the crate constant `G`). Over a fast crest it drops (the car goes light); through a banked, compressive corner such as Eau Rouge it can approach $2g$ (the car is squeezed into the road and the tires gain grip). Aerodynamic downforce is deliberately *not* part of $g_{\text{normal}}$ — the envelope's speed axis already carries it (§8.4).

Two practical details of the sampling: the requested step is rounded so `ds` divides the lap length *exactly* (the wrap segment of a closed lap is then also `ds`), and the curvatures get a light centred moving average of half-width `CURV_SMOOTH_RADIUS = 6` stations. Imported real-world centerlines (OpenStreetMap geometry plus elevation-model heights) carry sub-car-length position noise that an interpolating spline amplifies into fake curvature spikes; the average removes them while leaving genuine corners (which span many stations) intact. The doc comment is honest that this is a pragmatic mitigation — "the principled fix for a fair lap is the min-curvature line" (`path.rs:15-20`).

Signs follow ISO 8855 throughout: $x$ forward, $y$ left, $z$ up, so positive $a_y$ is a left turn and positive `sin_g` means uphill.

#### 8.2.3 The degenerate path: a constant-μ friction ellipse

T0 supports two grip models behind one private trait (`GripModel`, `solver.rs:51-58`), monomorphised into the shared sweep so there is no per-station dynamic dispatch. The simpler one — `EllipseGrip` — treats the car as a point mass with constant friction coefficients and a *friction ellipse*: the rule that longitudinal and lateral tire force trade off as

$$\left(\frac{F_t}{\mu_x \gamma N}\right)^2 + \left(\frac{F_y}{\mu_y \gamma N}\right)^2 \le 1,$$

where $N$ is the normal load, $\mu_x/\mu_y$ the longitudinal/lateral friction coefficients, and $\gamma$ the local track grip scale. With lumped aerodynamic terms $q_x = \tfrac12 \rho C_x A$ (drag) and $q_z = \tfrac12 \rho C_z A$ (downforce), the point-mass equations are (`solver.rs:10-13`; `docs/theory/t0-point-mass.md`):

$$N(s,v) = m\,(g\cos\theta_b\cos\theta_g + \kappa_n v^2) + q_z v^2, \qquad F_y(s,v) = m\,(\kappa_l v^2 + g\sin\theta_b\cos\theta_g),$$

$$m\dot v = F_t - q_x v^2 - m g \sin\theta_g .$$

Because both sides of $|F_y| \le \mu_y \gamma N$ are affine in $u = v^2$, the cornering ceiling has a **closed form** — no iteration (`ellipse_v_limit`, `solver.rs:62-88`), plus a *flight guard* enforcing $N \ge 0$ (over a crest severe enough that even downforce cannot keep the tires loaded, the ceiling is the take-off speed). On a flat circle of radius $R$ it reduces to the formula every textbook derives, $v = \sqrt{\mu_y g R}$; on a turn banked at angle $\phi$ to $v^2 = gR\,(\mu_y\cos\phi + \sin\phi)/(\cos\phi - \mu_y\sin\phi)$ — both verified against the solver in `crates/outlap-qss/tests/analytic.rs`.

Where do $\mu_x, \mu_y$ come from? Not from a hand-typed number: `T0Vehicle::assemble` (`crates/outlap-qss/src/vehicle.rs`) evaluates the real MF6.1 tire model (Chapter 7) and takes the **pure-slip force peaks** at the nominal load `FNOMIN` and the cold inflation pressure, averaging front and rear axles — so the tire's load- and pressure-shape factors are folded in rather than the raw `PDX1`/`PDY1` coefficients being trusted blindly. A note recording exactly this lands in every result (`notes` discipline: nothing silent). The same assembly reduces the powertrain to a per-unit peak-torque envelope over folded gear ratios and an ERS power cap — that is the *powertrain ceiling* `tractive_force(v)`, summarised in §8.2.5; the `.ptm` format and the energy accounting behind it belong to Chapter 9, Physics III.

Be aware of the fine print: this constant-μ ellipse path (`solve_into` / `solve_lap`) is **not reachable from Python**. It is the analytic-reference and performance-gate path at the Rust level, and the theory page `docs/theory/t0-point-mass.md` documents it from that (M1) perspective. What `tier="t0"` actually runs in v0.2.0 is the next section.

#### 8.2.4 The production path: T0 on the g-g-g-v envelope

The production grip model — `GgvGrip` (`solver.rs:153-310`) — replaces the constant-μ ellipse with lookups into the T1-derived envelope of §8.4. The envelope answers one question, allocation-free: *at speed $v$, longitudinal acceleration $a_x$, and normal gravity $g_{\text{normal}}$, what is the maximum sustainable lateral acceleration?* (`ay_boundary(v, ax, g_normal)`). The solver multiplies that boundary by the local track grip scale `p.grip[i]`, exactly as the ellipse path scales its μ.

The three `GripModel` queries become:

- **Cornering ceiling by bisection** (`v_limit`, `solver.rs:262-280`). There is no closed form any more — both the demand $a_{y,\text{dem}}(v)$ and the boundary $a_y(v, 0, g_{\text{normal}}(v))$ move with speed — so the solver bisects feasibility between 0 and the speed cap `v_cap` (default 150 m/s) over `V_LIMIT_ITERS = 24` iterations, resolving the ceiling to sub-mm/s.
- **Forward step** (`forward_v2`, `solver.rs:282-297`). The tire budget $a_{x,\text{grip}}$ comes from inverting the envelope at the lateral demand: bisect the largest $a_x \ge 0$ whose boundary still meets $|a_{y,\text{dem}}|$ (`ax_forward`, `AX_INV_ITERS = 16`). The powertrain branch is $F_t(v)\cdot\text{scale}/m - a_{\text{drag}}(v)$, and the applied acceleration is

  $$a = \min\!\big(a_{x,\text{grip}},\; F_t(v)\,\text{scale}/m - a_{\text{drag}}(v)\big) - g\sin\theta_g .$$

  Note the drag subtraction: the envelope's $a_x$ axis already *embeds* the reference aerodynamic drag (a trim at $a_x$ includes drag in its force balance), so the solver subtracts the same `drag_accel(v)` curve from the powertrain ceiling before taking the min — otherwise drag would be double-counted on the grip branch or missed on the power branch. `scale` is the per-station traction scale in $[0,1]$ filled by the slow-state coupling (thermal derate ∧ battery power cap — Chapter 9); it is `1.0` when uncoupled, and it never touches braking, which draws no drive power.
- **Backward step** (`backward_v2`, `solver.rs:299-309`). Bisect the most-negative feasible $a_x$ at the lateral demand (`ax_backward`); drag and uphill gravity *add* to the braking budget. Braking remains friction-limited only — no brake-thermal model, no regen blending yet, and the notes say so.

The flight guard survives, generalised: a station is "planted" when the *total* normal specific force $g_{\text{normal}} + q_z v^2/m$ is positive — aero downforce still plants a downforce car over a crest even when $g_{\text{normal}} \le 0$. Airborne stations coast on drag and grade alone. One documented, bounded approximation: the envelope's lowest sampled $g_{\text{normal}}$ is $0.5g$, so between $0.5g$ and 0 the boundary query clamps and slightly over-predicts the gravity contribution to grip (≈ $\mu \cdot 0.5g$, dominated by aero at the speeds where crests matter; `solver.rs:241-252`).

#### 8.2.5 The powertrain ceiling

`tractive_force(v)` (`crates/outlap-qss/src/vehicle.rs:226-249`) is the one place engine/motor capability enters T0, and it is deliberately simple:

- Each drive unit's `.ptm` map (Chapter 5) contributes its `limits.max_torque_nm_vs_speed` curve as a peak-torque envelope $\tau(\omega)$, interpolated by the project's one shared `MonotoneCubic` (RPM converted to rad/s at the file boundary, per the SI-internally rule).
- Gearbox and differential ratios along each unit's coupler path are folded at assembly into per-gear constants: `omega_per_v = ratio / r_wheel` and `force_per_torque = ratio · efficiency / r_wheel`. At speed $v$, each unit contributes its **best gear** — the highest wheel force among gears whose shaft speed stays under the envelope's top speed (gears past it are rev-limited out).
- An ERS (energy-recovery system, e.g. an F1 MGU-K) is reduced to a **power cap**: peak deployment power times a speed-dependent taper curve, capped by the machine's ratio-invariant mechanical ceiling $\max(\tau\omega)$ over its map, converted to force as $\eta P / \max(v, 1.0)$ (the 1 m/s floor avoids an infinite launch force; the friction limit caps launch anyway). Per-lap deployment budgets and override modes are *not* enforced at T0 — a permanent note in every result says so.

Two typed errors guard the assembly: a vehicle with neither drive units nor ERS is `T0Error::NoDrive`, and a drive unit whose `.ptm` carries a gridded efficiency *map* (rather than a constant) is `T0Error::UnsupportedEfficiencyMap` — T0 has no map reader yet. Similarly, a vehicle with no `aero.constant` block is a hard `T0Error::NoConstantAero` unless `sim.allow_degraded: true`, in which case T0 runs with zero aero and a recorded note — the single documented fallback path (Decision #40; Chapter 4).

#### 8.2.6 Entry points, workspaces, and the 50 ms budget

The zero-allocation kernels write into a caller-owned `T0Workspace` (two pre-sized `Vec<f64>`: the ceiling `v_lim` and the solved `v`; `crates/outlap-qss/src/result.rs:25-52`):

| Function | Grip model | Allocates? |
|---|---|---|
| `solve_into(veh, path, ws)` | constant-μ ellipse | no |
| `solve_into_ggv(veh, env, path, ws)` | g-g-g-v envelope | no |
| `solve_into_ggv_scaled(veh, env, scale, path, ws)` | envelope + per-station traction scale | no |
| `solve_lap(...)` / `solve_lap_ggv(...)` | owning wrappers returning a `LapResult` | yes (cold) |

CI enforces both halves of the performance contract in release builds (`crates/outlap-qss/tests/catalunya.rs`, `tests/alloc.rs`): the median of 11 solves of a real Catalunya lap must complete in under 50 ms for both `solve_into` and `solve_into_ggv` (envelope *generation* is excluded — it is the documented cold assembly step), and a dhat-instrumented test asserts the hot kernels perform **zero heap allocations**. Chapter 13 covers the full gate list.

#### 8.2.7 What T0 returns: `LapResult`

The owning wrappers derive the point-mass channels and pack them into `LapResult` (`crates/outlap-qss/src/result.rs:54-75`):

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

`ay` is the velocity-frame lateral demand $\kappa_l v^2 + g\sin\theta_b\cos\theta_g$ evaluated on the solved profile; `ax` is the central segment acceleration $(v_{i+1}^2 - v_i^2)/2\Delta s$. The `resolved_hash` ties every result to the exact vehicle spec that produced it (Chapter 4), and `notes` accumulates every recorded simplification from vehicle assembly, envelope generation, and dispatch.

### 8.3 T1: the quasi-static double-track trim

#### 8.3.1 What a "trim" is

The point-mass solver knows one number per station. A real car has four tires, each with its own vertical load, slip, and force — and those loads shift as the car accelerates (Chapter 2's load transfer). The T1 **trim** answers the detailed question: *given a commanded operating point — speed $v$, lateral acceleration $a_y$, longitudinal acceleration $a_x$, and the local $g_{\text{normal}}$ — what steady chassis state produces exactly those accelerations?* "Steady" (quasi-static) means nothing is changing: yaw rate constant, no suspension transients. The vocabulary comes from flight mechanics — trimming an aircraft means finding the control settings that hold a steady condition.

The trim is a **9-unknown, 9-residual** nonlinear algebraic system, solved zero-allocation and panic-free (`crates/outlap-qss/src/t1/trim.rs`, module header). Its literature basis: Perantoni & Limebeer (2014) for the reference car and QSS framing, Lovato & Massaro (2022) for the g-g framing, Pacejka (2012) and Guiggiani (2018) for the load transfer, Milliken & Milliken (1995) for the lateral-transfer decomposition.

#### 8.3.2 The actual unknowns and residuals

The unknown vector $z$ (ISO 8855, SI; `trim.rs:12-21`):

| index | symbol | meaning |
|---|---|---|
| 0 | $\delta$ | front road-wheel steer angle, rad |
| 1 | $\beta$ | body-slip angle (velocity vector vs body $x$-axis), rad |
| 2 | $r$ | yaw rate, rad/s |
| 3 | $s$ | longitudinal-slip control (drive if $> 0$, brake if $< 0$) |
| 4 | $w$ | driven-axle slip split: $\kappa_{\text{left}} = s + w$, $\kappa_{\text{right}} = s - w$ |
| 5–8 | $F_{z,i}$ | per-wheel normal loads $[\mathrm{FL}, \mathrm{FR}, \mathrm{RL}, \mathrm{RR}]$, N |

Note what is *not* here: no throttle/brake pair. One signed slip control $s$ handles both (its sign selects drive vs braking), and $w$ is the differential's left/right split. Inside the solver the four $F_z$ unknowns are non-dimensionalised by $m \cdot g_{\text{normal}}$ so all nine unknowns are order-1 — mixing radians and newtons in one Jacobian would otherwise be numerically miserable.

The nine residuals (`trim.rs:23-35`, evaluated in `residual()` at `trim.rs:487-570`):

$$R_0:\ \Sigma F_x = m a_x \qquad R_1:\ \Sigma F_y = m a_y \qquad R_2:\ \Sigma M_z = 0$$

$$R_3:\ r\,v = a_y \cos\beta - a_x \sin\beta \qquad R_4:\ \text{differential law} \qquad R_{5\ldots8}:\ F_{z,i} = F_{z,i}^{\text{pred}}\ (\times 4)$$

In words: the tire and aero forces must sum to exactly the commanded accelerations ($R_0, R_1$, with aero drag $q_x v^2$ subtracted from $\Sigma F_x$); the yaw moments must cancel (steady state means zero yaw acceleration, $R_2$); the yaw rate must be kinematically consistent with turning at $(a_x, a_y)$ while slipping at $\beta$ ($R_3$ — for constant body-frame velocity the CG acceleration is $\omega \times V$); the driven wheels must obey the differential ($R_4$: an **open** diff under drive enforces equal wheel-frame longitudinal force, i.e. equal torque; **locked/solid/LSD** enforce equal speed, $w = 0$; under braking the diff is inactive and the balance bar splits brake torque); and the four normal loads must match the quasi-static load-transfer prediction ($R_5$–$R_8$). Per-wheel slip angles come from the contact-point velocities $V_{x,i} = v\cos\beta - r y_i$, $V_{y,i} = v\sin\beta + r x_i$ rotated into the wheel frame, with tire forces from the shared `TireModel` at camber 0 (camber maps land later — a recorded note). Residuals are scaled dimensionless (forces by $1/mg$, the moment by $1/mgL$) and convergence is declared at scaled norm $\le$ `TOL = 1e-10`.

#### 8.3.3 Load transfer: geometric plus elastic

The load-transfer prediction (`load_transfer`, `trim.rs:587-614`; theory in `docs/theory/t1-trim.md`) is the standard steady-state decomposition. Per axle, the static weight plus downforce:

$$F_{z,\text{front}}^{\text{total}} = m g \frac{b_r}{L} + q_{z,f} v^2, \qquad F_{z,\text{rear}}^{\text{total}} = m g \frac{a_f}{L} + q_{z,r} v^2,$$

where $a_f$/$b_r$ are the CG-to-axle distances and $L$ the wheelbase. Longitudinal (pitch) transfer moves load rearward under acceleration:

$$\Delta F_z^x = \frac{m\,a_x\,h_{cg}}{L}.$$

Lateral transfer per axle splits into a **geometric** part acting through the axle's roll centre (a suspension-geometry construct: the point about which the body effectively rolls at that axle) and an **elastic** part carried by roll stiffness — the Milliken decomposition. With $H = h_{cg} - h_{ra}$ the CG height above the roll axis and $M_\phi = m a_y H$ the roll moment:

$$\Delta F_{z,f}^{y} = \frac{m a_y\,(b_r/L)\,h_{rc,f}}{t_f} + \frac{\xi_f\,M_\phi}{t_f}, \qquad \Delta F_{z,r}^{y} = \frac{m a_y\,(a_f/L)\,h_{rc,r}}{t_r} + \frac{\xi_r\,M_\phi}{t_r},$$

where $t$ is the track width, $h_{rc}$ the roll-centre height, and $\xi_f + \xi_r = 1$ the roll-stiffness shares. This is why an engineer stiffens the front anti-roll bar to add understeer: a larger $\xi_f$ moves lateral transfer to the front axle, and because tire grip grows sub-linearly with load (Chapter 7), a more unevenly loaded axle grips less. Two implementation details matter: at **wheel lift** the unloaded wheel floors at 0 N and the grounded wheel carries the whole axle — never more — so the boundary cannot become optimistic and $\Sigma F_z$ stays weight plus downforce (`split_axle`, `trim.rs:761-775`). And **anti-dive/anti-squat do not enter steady-state $F_z$** — they only modulate the aero-platform heave (ride height) when a ride-height aero map is installed, which changes downforce, which changes loads; the load-transfer totals themselves are geometry-independent in steady state. The aero map and its platform-equilibrium fixed point are Chapter 7's subject; from the trim's point of view the map is evaluated *inside* the residual at yaw = $\beta$ (in degrees at the map boundary, one of the deliberate display-unit seams) so the Jacobian feels $\partial(\text{downforce})/\partial\beta$.

#### 8.3.4 `fz_coupling`: Decision #29

The loads depend on the accelerations, and the accelerations depend on the tire forces, which depend on the loads — an algebraic loop. `sim.fz_coupling` selects how the loop closes (`crates/outlap-schema/src/sim.rs:80-89`), and the trim implements it as a one-line substitution (`trim.rs:546-549`):

```rust
let (ax_lt, ay_lt) = match inp.coupling {
    FzCoupling::OneStepLag => (inp.ax, inp.ay),                 // commanded accelerations
    FzCoupling::FixedPoint => (sum_fx / self.mass_kg, sum_fy / self.mass_kg), // summed tyre forces
};
```

- **`one_step_lag`** (the default): the load-transfer prediction uses the *commanded* $(a_x, a_y)$ — loads are decoupled from the instantaneous tire-force sums during the iteration.
- **`fixed_point`**: the prediction uses the *achieved* accelerations $\Sigma F_x/m,\ \Sigma F_y/m$ — fully coupled through the Jacobian.

Here is the key fact, stated in `docs/theory/t1-trim.md`: at convergence, residuals $R_0/R_1$ force $\Sigma F = m a$, so **both modes reach the same trim**. The choice changes only the algebraic coupling seen by the Jacobian (and will matter for the transient tiers, where the "previous step" in `one_step_lag` is a real timestep). It is a *recorded* simulation setting — it appears in every result's attributes and in the envelope, so no two results with different numerics can be confused.

#### 8.3.5 Numerics: damped least-squares, continuation, and honest infeasibility

The docs call the trim a "damped Newton" solve; the algorithm as implemented is **Levenberg–Marquardt** (LM) — a damped Gauss–Newton that interpolates toward gradient descent when far from the solution (`solve_lm`, `trim.rs:336-442`). One LM iteration:

- build a finite-difference Jacobian $J$ (relative step `FD_H = 1e-7`; nine extra residual evaluations);
- form the normal equations $A = J^\top J$, $g = J^\top R$, with Marquardt diagonal scaling $A_{ii} \mathrel{+}= \mu \max(A_{ii}, 10^{-12})$;
- solve the dense 9×9 system by Gaussian elimination with partial pivoting (fixed stack arrays — zero heap);
- accept the step only if it reduces $\lVert R \rVert$ (a damping loop of up to `MAX_LINE_SEARCH = 40` retries, shrinking $\mu$ ×0.3 on acceptance, growing ×4 on rejection).

At most `MAX_NEWTON = 80` iterations. Trial states are clamped to generous physical bounds ($\delta \in \pm0.7$ rad, $\beta \in \pm0.5$, $r \in \pm8$ rad/s, $s, w \in \pm0.6$, $F_z/mg \in [0, 6]$) so the search cannot wander into the periodic-$\beta$ aliases that trap a plain Newton. When a ride-height aero map is installed, the converged ride heights from each residual evaluation are threaded to the next as a warm start for the nested aero-platform fixed point (Chapter 7), cutting it from ~20 cold iterations to 1–2 warm ones with a physically identical result.

The full `trim()` entry (`trim.rs:205-227`) is two-stage: a **fast path** — one direct LM solve from a physics warm start (Ackermann-like steer $\delta = L a_y/v^2$ clamped to ±0.5, $r = a_y/v$, $\beta = 0$, loads from the direct transfer prediction) — and, if that fails, a **homotopy continuation** fallback: solve the trivially easy straight-line trim ($a_y = a_x = 0$, near-linear, always converges), then ramp the targets $(t\,a_y,\ t\,a_x)$ from $t = 0$ to 1 with an adaptive step, warm-starting each sub-solve from the last. If the ramp cannot advance (step below $10^{-3}$), the commanded point is past the friction boundary.

Which brings us to the design contract that makes the whole envelope possible: **an unreachable operating point is not an error.** The trim returns a typed

```rust
pub enum TrimOutcome {
    Converged(TrimState),
    Infeasible { residual_norm: f64, iterations: usize },
}
```

and `Infeasible` is *information* — it means "past the grip limit", and the envelope generator uses it as its boundary oracle. It is never a panic, and solver kernels stay `Result`-typed per the project's error rules. Because the boundary bisection makes roughly half its probes infeasible *by construction*, infeasible probes must also fail *fast*: an **infeasibility stall test** watches windows of `STALL_WINDOW = 6` LM iterations and cuts the solve if $\lVert R \rVert$ failed to shrink by factor `STALL_FACTOR = 0.7` while still far above tolerance (floor $10^{-7}$). A converging solve drops orders of magnitude per few iterations; an infeasible one parks at a nonzero least-squares minimum shaving microscopic amounts — the test tells them apart. The code documents the accepted residual risk (a pathologically stiff *feasible* point misclassified infeasible would pin an envelope node conservatively low, bounded by probe retries and the accuracy gates; a continuation-backed confirmation is an M4 candidate).

Speed guard: the QSS kinematics divide by $v$ (yaw rate $r = a_y/v$), so any commanded point at $v \le$ `V_MIN` $= 0.5$ m/s is immediately `Infeasible` — a crawling car has no well-posed g-g trim.

#### 8.3.6 Setup metrics

Because the trim can be probed at will, two classic setup numbers come almost for free (`trim.rs:444-479`):

- **Understeer gradient** $K = \dfrac{d\delta}{d a_y} - \dfrac{L}{v^2}$ (rad per m/s²) — how much *extra* steering beyond the pure-geometry (Ackermann) requirement the car needs per unit of lateral acceleration. $K > 0$ is understeer (front washes out first), $K < 0$ oversteer. Computed by central difference of two trims at $a_y = \pm 1$ m/s² (a small, linear-regime probe) at $a_x = 0$; `None` if either probe is infeasible.
- **Aero balance** — the front axle's share of total downforce, 0..1. `aero_front_downforce_share_at(v)` evaluates it at the aero-platform equilibrium for straight running, so with a ride-height map installed it is genuinely speed-dependent (the platform rakes as downforce compresses the suspension); with constant aero it equals the reference share.

Both are logged per station on every `t1` lap (§8.5) — in the xarray output as `understeer_gradient` (units recorded as `rad·s²/m`) and `aero_front_share`.

#### 8.3.7 What pins the trim down

The trim is where most of outlap's vehicle-dynamics correctness lives, so it carries a correspondingly heavy property-test suite (`docs/theory/t1-trim.md` lists it; Chapter 13 explains the testing philosophy): per-wheel friction-circle containment; $\Sigma F_z$ = weight + downforce exactly; left/right mirror symmetry at $\pm a_y$; the ISO 8855 sign conventions (a left corner produces positive $a_y$, $\delta$, $r$, and loads the right-hand wheels); the pitch-transfer direction; convergence over a dense feasible grid for both reference cars down to hairpin-scale corners (~6 m radius at 8 m/s); graceful infeasibility past the boundary; agreement of the two `fz_coupling` modes at convergence; and the dhat-enforced zero-allocation guarantee.

### 8.4 The g-g-g-v envelope

#### 8.4.1 Why precompute

A single trim costs tens of LM iterations, each with a 9-column finite-difference Jacobian, each column a full four-tire force evaluation (plus an aero-map fixed point if installed). The T0 sweep needs the grip boundary at every one of ~2,300 stations, inside bisections of 16–24 probes, iterated over up to 8 closed-lap passes. Solving trims inline would put millions of tire evaluations in the hot loop. So outlap does what the reference literature does (Tremlett et al. 2014; Lovato & Massaro 2022; Rowold et al. 2023; Werner et al. 2025): **precompute the boundary once into a gridded table** and let the lap solver interpolate. Generation is a cold assembly step — seconds in a release build — and every query afterwards is a zero-allocation cubic interpolation.

The result is the **g-g-g-v envelope**: the classical g-g diagram (the achievable $(a_x, a_y)$ region, Rice 1973; Milliken & Milliken 1995) extended by two axes, speed $v$ (downforce grows with $v^2$, inflating the whole envelope) and $g_{\text{normal}}$ (§8.2.2). Geometrically it is a funnel that widens with speed, one nested surface per normal-gravity level — see `docs/theory/ggv-envelope.md` for figures generated from the real model.

#### 8.4.2 The grid and its two design decisions

The base table stores $a_{y,\text{corr}} = gg(v, \hat a_x, g_{\text{normal}})$ in a `GriddedMapN` — the shared N-dimensional monotone cubic Hermite interpolant (Fritsch & Carlson 1980; Chapter 5 and Chapter 6 cover it) — over the `sim.envelope` grid, default **40 × 25 × 7** points (`crates/outlap-schema/src/sim.rs:114-122`). Axes:

- $v$: auto-ranged from `V_ENV_LO = 5.0` m/s to the car's own estimated top speed (drive force vs drag, capped at 120 m/s). For `f1_2026` this comes out as $v \in [5.0, 91.0]$ m/s.
- $\hat a_x \in [-1, 1]$: a **normalised** longitudinal axis. A grid node maps to the actual acceleration $a_x = \hat a_x \cdot a_{x,\text{cap}}(v, g_{\text{normal}})$, where the cap is *that operating point's own* straight-line braking ($\hat a_x < 0$) or acceleration ($\hat a_x > 0$) limit. Longitudinal capability spans a huge range across the speed/load axes (a light-load, low-speed point brakes at a fraction of a high-downforce point), so a fixed actual-$a_x$ grid would leave the feasible window falling between nodes at low load; normalising gives every slice full resolution with a node exactly on $\hat a_x = 0$ (pure cornering). Queries take the *actual* $a_x$ and normalise internally. (Some of the reference literature parameterises the g-g slice in polar form instead; what outlap ships is this normalised-axis form, following the per-speed g-g construction of the reference works — the code comment in `envelope.rs:16-24` is explicit about it.)
- $g_{\text{normal}} \in [0.5g,\ 2.0g]$: strong crest unloading to an Eau-Rouge-class compression. For reference, $[4.90, 19.61]$ m/s².

Two decisions, both following Werner et al. (2025, arXiv:2504.10225):

1. **Velocity-frame projection** (their eq. 5). The stored lateral boundary is $a_{y,\text{corr}} = a_{y,\text{body}}\cos\beta - a_x\sin\beta$ at the converged body slip — the component orthogonal to the *velocity vector*. A point-mass solver has no slip angle as a state; projecting at generation time lets it compare the boundary directly against its centripetal demand $\kappa_l v^2 + g\sin\theta_b\cos\theta_g$.
2. **Powertrain limits are omitted** (their §II-C). The envelope is a pure *tire-force* limit; the drive ceiling is applied by the lap solver as `min(tractive_force, grip)` (§8.2.4). This keeps one envelope valid across powertrain what-ifs and keeps the traction-scale coupling out of the table.

Alongside the base table the struct carries the two shoulder maps `accel_cap`/`brake_cap` over $(v, g_{\text{normal}})$, the reference drag curve `drag_accel(v)` (the "drag currency" the $a_x$ axis embeds), the reference mass, the recorded `fz_coupling`, six sensitivity fields (next section), and human-readable generation notes (`envelope.rs:152-183`).

#### 8.4.3 How the boundary is traced

The generator (`GgvEnvelope::generate`, `envelope.rs:197-420`) sweeps $n_v \times n_{gn}$ fully independent $(v, g_{\text{normal}})$ *fibres*. Per fibre:

1. **Shoulders first.** Bracket the straight-line acceleration and braking limits by doubling an initial 5 m/s² guess up to a 90 m/s² (~9 g) cap, then bisecting 16 iterations (`max_straight_ax`). These become `accel_cap`/`brake_cap` — the $\hat a_x = \pm 1$ normalisation denominators, floored at 0.5 m/s² so a degenerate no-drive car cannot divide by zero.
2. **March outward from pure cornering.** Starting at the node nearest $\hat a_x = 0$, find each node's maximum feasible $a_y$ with `max_lateral` and hand its converged trim state to the next node outward as a *hint*. A hinted node searches a narrow bracket ($\pm 40\%$ of the neighbour's boundary, 12 bisection iterations) instead of the full expand-and-bisect (seed 20 m/s², up to 8 doublings, 16 iterations — worst-case resolution $2^{-16} \cdot 90 \approx 1.4\times10^{-3}$ m/s², far below the accuracy gates).
3. **Probe economics.** Every probe is a `trim_warm` — the direct-LM primitive of §8.3.5, warm-started from the last feasible state — so feasible probes converge in a few iterations and infeasible ones hit the stall test fast. A failed warm probe retries once from the cold physics guess before the point is declared infeasible: a stale warm start could otherwise pin the boundary low (`probe`, `envelope.rs:658-672`). A node whose *straight-line* seed is already infeasible carries the zero boundary $a_y = 0$ — never a panic; this is the infeasible-trim contract doing its job.

Fibres solve into owned outputs merged in fixed order, so the result is **bit-identical** whether the sweep runs serially or on a rayon pool behind the native-only `parallel` feature (wasm builds stay thread-free — Chapter 6). Everything is stored with all axes clamping out-of-domain: a query beyond the grid saturates at the edge value rather than extrapolating, and `drag_accel(v)` below the lowest sampled speed (5 m/s) tapers as $v^2$ toward zero so a standing start feels no spurious drag.

The clean-room statement in the module header is worth knowing: the GPL-3.0 `TUM-AVS/GGGVDiagrams` repository (the reference implementation of Werner et al. 2025) was consulted for *approach only* and the code re-authored from the papers — a live application of the project's clean-room rule, with the repo and licence recorded beside the citations (`envelope.rs:70-84`; same statement in `docs/theory/ggv-envelope.md`).

#### 8.4.4 Decision #31: separable multiplicative corrections

A strategy sweep wants laps at perturbed car states — worn tires (lower μ), fuel load (mass), a different wing level ($C_L A$). Regenerating a full envelope per variant would erase the precomputation win. Locked Decision #31 (`docs/HANDOFF.md`): store, per node, three **relative sensitivities** and correct multiplicatively:

$$S_\mu \approx \frac{\partial \ln a_{y,\text{corr}}}{\partial \ln \mu}, \quad S_m \approx \frac{\partial \ln a_{y,\text{corr}}}{\partial \ln m}, \quad S_{C_LA} \approx \frac{\partial \ln a_{y,\text{corr}}}{\partial \ln C_LA},$$

$$a_{y,\text{corr}}(\ldots;\mu,m,C_LA) = gg(\ldots)\cdot\big(1 + S_\mu\,\tfrac{\Delta\mu}{\mu_0}\big)\big(1 + S_m\,\tfrac{\Delta m}{m_0}\big)\big(1 + S_{C_LA}\,\tfrac{\Delta C_LA}{C_LA_0}\big),$$

clamped at zero, **identity at the reference by construction**. The sensitivities are *secants*, not tiny tangents: full-T1 boundary re-solves on perturbed vehicle clones at the edges of each parameter's intended correction band — ±15% μ, ±10% mass, ±30% $C_LA$ (`H_MU`/`H_MASS`/`H_CLA`, `envelope.rs:118-122`) — so the stored slope is exact at the band edge for a linear response. They are stored as separate **up/down one-sided pairs** because the μ response of a friction-circle-coupled tire is measurably convex; the query picks the upward secant above the reference and the downward one below (`ay_boundary_corrected`, `envelope.rs:458-487`). Guard rails everywhere: sensitivities are sampled only on every 2nd $\hat a_x$ node in a fibre's near-peak bulk (boundary ≥ max(50% of the fibre peak, 0.5 m/s²); skipped nodes filled by linear interpolation along the fibre; $v$ and $g_{\text{normal}}$ keep full resolution because a speed-subsampled variant failed the accuracy gate), the stored $|S| \le 2.0$, and the evaluated factor is clamped to $[0.3, 3.0]$.

CI validates the correction against ground truth: at sampled near-peak grid nodes, the corrected envelope must match a **full T1 re-solve** of the perturbed car to within **2% of the local peak** at pure-lateral nodes (realised ≈ 0.6% on the reduced CI grid) and within **12%** at moderate $|\hat a_x| = 0.4$ — a documented degradation toward the shoulders, where the velocity-frame $-a_x\sin\beta$ term dominates and a multiplicative factor cannot move the shoulder itself (`envelope.rs` tests at 1016–1107; `docs/theory/ggv-envelope.md`). Further property tests pin node-exactness of the interpolant (< 2% of local peak), identity at reference to $10^{-12}$, correction signs (more grip ⇒ higher boundary, more mass ⇒ lower), monotonicity in $g_{\text{normal}}$, concavity of the $a_y(a_x)$ section (the feasible g-g region is convex), and zero-allocation queries.

One honest caveat: in v0.2.0 the corrected query `ay_boundary_corrected` is built, gated, and public at the Rust level, but **no lap-level consumer uses it yet** — the batch/sweep API that composes off-reference corrections into a lap is future work. A Python `overrides={...}` what-if instead re-resolves the vehicle and generates (and caches) a fresh envelope keyed by the new resolved hash. The correction machinery is the enabling groundwork for the strategy layer, not a shortcut you can reach from Python today.

#### 8.4.5 Real numbers

Queries you can reproduce from Python via `lap.envelope` (Chapter 10 documents the `Envelope` class). For the shipped `f1_2026` (reference mass 768.0 kg) at $g_{\text{normal}} = g$:

| $v$ (m/s) | pure-lateral boundary `ay_boundary(v, 0, g)` | `accel_limit` | `brake_limit` | `drag_accel` |
|---|---|---|---|---|
| 15 | 12.76 m/s² (≈ 1.3 g) | 7.57 m/s² | 13.81 m/s² | 0.21 m/s² |
| 40 | 17.81 m/s² (≈ 1.8 g) | 9.91 m/s² | 21.06 m/s² | 1.54 m/s² |
| 80 | 32.08 m/s² (≈ 3.3 g) | 19.61 m/s² | 50.28 m/s² | 6.36 m/s² |

The funnel widens with speed as downforce loads the tires — and the $g_{\text{normal}}$ axis matters just as much: at 40 m/s the pure-lateral boundary is 13.12 m/s² over a $0.6g$ crest but 23.45 m/s² in a $1.5g$ compression. That factor-1.8 swing is exactly what a flat g-g diagram cannot represent, and why the third axis exists.

### 8.5 Tier dispatch, end to end

#### 8.5.1 Selecting a tier

The tier comes from `sim.yaml`'s `tier` field, overridden by a `sim={...}` dict, overridden in turn by the `tier=` keyword of `solve_lap` / `solve_lap_dataset` (Chapter 10). The enum and — note well — the **default**:

```rust
pub enum Tier { T0, #[default] T1, T2, T3 }   // serialised: t0 | t1 | t2 | t3
```

A `solve_lap` call with no `sim.yaml` and no `tier=` gives you a **t1** lap ("the default lap solver", `sim.rs:71-73`). None of the shipped vehicle directories carries a `sim.yaml`, so the defaults are what you get unless you override.

#### 8.5.2 What actually happens per tier

The dispatch site is the Python binding (`crates/outlap-py/src/lib.rs:1010-1063`) — an enum match at assembly time, never inside a loop:

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

Even a `t0` lap assembles the T1 vehicle — it needs the trim solver to generate the envelope. Envelopes are cached per process, keyed by everything that changes the boundary: the resolved-vehicle hash, a fingerprint of the sidecar tables (two spec-identical cars with different aero-map parquet files must never share an envelope), the conditions, the grid resolution, and the coupling mode. `flat_track` is deliberately *not* in the key — it reshapes the path, not the boundary. The cache is never evicted (sessions are short-lived), and `solve_lap` currently holds the GIL for its whole duration.

The two solve paths (`crates/outlap-qss/src/qss.rs`):

- **`solve_t0`** runs the (optionally slow-state-coupled) envelope velocity profile and returns point-mass channels only: `wheels: None`, `setup: None`.
- **`solve_t1`** runs the *identical* profile, then walks the solved stations calling `t1.trim(&TrimInput { v, ay, ax, g_normal, coupling })` at each one to log per-wheel state, plus `understeer_gradient(v, g_normal)` and `aero_front_downforce_share_at(v)`. A station whose re-trim is infeasible (it can happen: the point-mass profile lives on an interpolated boundary, and the trim's exact boundary is a hair away — or the station speed is below the 0.5 m/s trim floor) gets a **NaN row** in the wheel channels rather than an error; the dispatch tests require only that a majority of stations converge on the reference cars.

Both paths thread the slow-state coupling identically when a full electrified stack (battery + `.emotor` + Vdc-mapped drive units) is present: a fixed two-iteration solve → march → re-solve outer loop that fills the per-station traction scale, discharge-only SoC this milestone. That is Chapter 9's story; here it suffices that an absent stack leaves the scale at 1 and the result *bit-identical* to the uncoupled solve.

Summary of the observable differences:

| | `tier="t0"` | `tier="t1"` |
|---|---|---|
| Speed profile / lap time | envelope velocity profile | **identical** (same code path) |
| `s, v, ax, ay, t` (+ world `x, y, z`) | yes | yes |
| Per-wheel `vertical_load_n`, `slip_ratio`, `slip_angle_rad`, `force_long_n`, `force_lat_n` | no | yes — `(s, wheel)` arrays, wheel order `FL, FR, RL, RR`; NaN rows where infeasible |
| `understeer_gradient`, `aero_front_share` | no | yes |
| `state_of_charge`, `machine_temp_c` | only if a coupled stack was active | same |
| `lap.envelope` (queryable) | yes | yes |
| xarray dims | `s` only | `s` + `(s, wheel)` |

Every result records its provenance in attrs: `tier` ("t0"/"t1"), `fz_coupling` ("one_step_lag"/"fixed_point"), `flat_track` (an int in the dataset — netCDF attrs have no bool type), `resolved_hash`, and the full `notes` tuple.

#### 8.5.3 A worked example

```python
from outlap.core import Track, solve_lap_dataset

tr = Track.load("data/tracks/catalunya_osm")

ds0 = solve_lap_dataset("data/vehicles/f1_2026", tr, tier="t0")
ds1 = solve_lap_dataset("data/vehicles/f1_2026", tr, tier="t1")

print(ds0.attrs["lap_time_s"], sorted(ds0.data_vars))
print(ds1.attrs["lap_time_s"], sorted(ds1.data_vars))
```

produces (v0.2.0, default 2 m stations on the centerline — a real run, not an illustration):

```text
112.54804243512962 ['ax', 'ay', 't', 'v', 'x', 'y', 'z']
112.54804243512962 ['aero_front_share', 'ax', 'ay', 'force_lat_n', 'force_long_n',
                    'slip_angle_rad', 'slip_ratio', 't', 'understeer_gradient', 'v',
                    'vertical_load_n', 'x', 'y', 'z']
```

Identical lap time, as promised; the `t1` dataset has dims `{s: 2339, wheel: 4}` where `t0` has `{s: 2339}`. The first call pays the envelope generation (seconds in a release build — the CI wheel is built `--profile release` for exactly this reason; minutes in a debug build); the second reuses the process cache and costs only the profile plus, for `t1`, one trim and two understeer probes per station. Chapter 10 documents the full `Lap`/`Dataset` surface, and Chapter 14 builds recipes on top of it.

#### 8.5.4 `t2` / `t3`: a clean, typed refusal

Requesting a transient tier produces `QssError::TierNotImplemented` with a message that names the milestone and the alternatives (`qss.rs:155-159`):

```text
solver tier `t2` is not implemented yet (the transient tiers arrive in milestone M4);
select tier `t0` (point-mass on the g-g-g-v envelope) or `t1` (full QSS with per-wheel outputs)
```

At the Python boundary this becomes a `ValueError` (T2 → "M4", T3 → "M6"), and the tests assert the milestone appears in the message. No partial results, no silent downgrade.

### 8.6 Flat-track mode: collapsing g-g-g-v to g-g

Set `sim.flat_track: true` (or `sim={"flat_track": True}`) and the path sampler switches to `T0Path::from_track_flat` (`path.rs:52-58`): the plan-view curvature $\kappa_h(s)$ is kept, but grade, banking, and vertical curvature are all zeroed. Consequently $g_{\text{normal}} \equiv g$ at every station and speed — the four-dimensional g-g-g-v envelope is only ever queried on its flat-gravity slice, i.e. it *collapses to a classical g-g-v*. The physical track files are untouched; only this run's path is flattened, and the mode is recorded in the result (`flat_track` attr).

This is the 2-D oracle-comparison mode, and it exists for one main customer: the **Limebeer cross-check** (Decision #48, `docs/validation/limebeer.md`). Perantoni & Limebeer (2014) is a 2-D study of an optimal-control F1 lap of Catalunya with a fully published car parameter set; outlap reruns the transcribed `limebeer_2014_f1` on the `catalunya_osm` min-curvature line with `flat_track: true` and the production 40×25×7 envelope. The CI-gated results: top speed 87.8 m/s vs the paper's ≈88 (−0.2%, gate ≤1%), slowest-corner apex 17.7 vs 17 m/s (+4.1%, gate ≤5%); the lap time (92.36 s vs the paper's 82.43 s) is recorded but deliberately *not* gated, because a QSS solver on a heuristic minimum-curvature line structurally cannot match a transient optimal-control lap that co-optimises its own driven line — the validation page decomposes the delta term by term. Chapter 13, Validation, testing, and trust, walks through the whole cross-check and the rest of the gate suite; Chapter 12 describes the vehicles and tracks involved.

That closes the QSS loop: a double-track trim distilled into a gridded grip surface, a point-mass sweep that consumes it in microseconds per station, and a re-trim that puts the four wheels back into the output. Chapter 9 adds the parts of the car that change *while* the lap runs — powertrain energy, machine heat, and battery state.


---

## 9. Physics III: powertrain, machine thermal, battery, and slow states

*What you will learn: how outlap turns everything downstream of the driver's right foot into physics it can solve — powertrains consumed as neutral map files, a drivetrain described as a graph of gearboxes and differentials, a thermal network that makes lap 20 slower than lap 1, and a battery whose voltage sag changes how hard the motor heats up. By the end you will know exactly which numbers cap the car's acceleration at any point on the track, and why the braking side is deliberately different.*

Everything in this chapter lives in a handful of places you can open alongside it:

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

### 9.1 The firewall: powertrains are maps, not models

outlap never simulates the inside of an electric machine, an inverter, or a gearbox. That is a hard project rule (the **firewall**): powertrains enter the simulator only as **`.ptm` files** — "powertrain map" documents that describe *what a unit does at its shaft* (torque available, efficiency, losses) without saying anything about *how* it does it (no electromagnetics, no switching losses, no gear-mesh models). The schema's own module doc calls the format "the firewall" (`crates/outlap-schema/src/ptm.rs`).

Why? Two reasons. First, scope: outlap is a vehicle and lap simulator, and machine design tools already exist that produce exactly these maps. Second, cleanliness: a map is a neutral, tool-agnostic contract. The importers in Chapter 11 read a design tool's HDF5 exports with plain `h5py` and emit `.ptm` files — the design tool's code and data never enter this repository (all committed powertrain data is synthetic, regenerated by `python/tools/gen_model3_powertrain.py`).

A `.ptm` document (schema `ptm/1.0` or `ptm/1.1`) describes a unit at its shaft: its `kind`
(`electric_machine`, `ice`, or `drive_unit`), the grid `axes` (a strictly ascending shaft-speed
axis, a load axis, and — new in `ptm/1.1` — an optional DC-link `vdc_v` axis), a `tables` sidecar
carrying the dense efficiency/loss data, and the `limits`. Chapter 5, Files and formats, walks the
format field by field with the shipped `du_medium.ptm.yaml`; here we care about what those numbers
*mean* for the lap:

- The **load axis runs negative** — the negative values are the **regen quadrant**, where the
  machine acts as a generator under braking and can recover energy rather than dissipate it.
- **`limits.max_torque_nm_vs_speed`**, the **peak-torque envelope**, is the single curve the lap
  solver uses as the traction ceiling. Crucially, outlap does *not* trust a datasheet's continuous
  rating: the optional `cont_torque_nm_vs_speed`/`overload` curves are validation references only,
  and the real thermal limit is *computed* from the loss tables by the `.emotor` model
  (Section 9.6). This is the whole reason the machine-thermal network exists.
- With a `vdc_v` axis the efficiency/loss/torque data become a 3-D tensor over
  `(speed, torque, voltage)`; Section 9.8 explains why a battery makes that third axis matter.
- A **missing sidecar table is not fatal**: the lap falls back to the peak envelope alone, with a
  note that energy accounting is off (nothing silent).

Internal-combustion engines are supported day one with the same format: `data/vehicles/f1_2026/ptm/ice_v6.ptm.yaml` is a `ptm/1.0`, `kind: ice` map for a synthetic 1.6 L V6 turbo, including a negative `drag_torque_nm_vs_speed` curve (engine braking, −20 to −80 N·m across the rev range). For an ICE the sidecar `efficiency` is *brake thermal efficiency*, and Section 9.5 shows how it becomes a fuel-mass rate.

Finally, imports are gated: the round-trip test loads an importer-emitted `.ptm` plus its Parquet through the real gridded-map path and must reproduce spot efficiencies from the source arrays to **1e-6**; unreachable cells beyond the torque envelope carry `NaN` and are nearest-valid filled and flagged out-of-hull; the zero-torque spin column is pinned to $\eta = 0$. CI runs on synthetic, tool-shaped fixtures only — real design-tool data never enters the repository (see Chapter 11, Importers and tooling, and Chapter 13, Validation).

### 9.2 The drivetrain topology graph

A `.ptm` tells you what a torque source can do; the vehicle's `drivetrain:` block tells you *where that torque goes*. outlap models the drivetrain as a **directed graph**, not a fixed layout: "torque **sources** (`.ptm` files: ICE, electric machines, or lumped drive units) connect to wheel **sinks** through ordered **coupler** elements (gearbox, differential, fixed ratio). Any four-wheeled concept is a topology plus data" (`crates/outlap-schema/src/vehicle/drivetrain.rs`).

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

(that is the real Model 3 wiring from `data/vehicles/tesla_model3_rwd/vehicle.yaml` — one rear drive unit through an open differential to the rear wheels, the `ev_1du_rwd` reference pattern). An all-wheel-drive EV is two such units; a conventional car is one ICE `.ptm` behind a `gearbox` and a `diff`; a hybrid is both feeding the same wheels. The layout is data, never a compile-time variant — the composition rule of Chapter 6.

Three coupler kinds exist, written `{gearbox: {...}}`, `{diff: {...}}`, or `{fixed_ratio: 2.4}`:

| Coupler | Fields | Semantics |
|---|---|---|
| `gearbox` | `ratios` (index 0 = first gear), `final_drive`, `shift_time_s`, `efficiency` | Selectable ratios; `efficiency` is a bare constant (default **0.985**) or `{map: eff.parquet}` — a gridded map |
| `diff` | `type` (`open` / `locked` / `lsd` / `solid`), `preload_nm`, `ramp: [accel, decel]` | Splits an axle torque left/right (Section 9.4); `preload_nm` is required for `lsd`/`locked`; `ramp` is LSD-only |
| `fixed_ratio` | a single number | One fixed reduction; multiplies into the total ratio |

`solid` is the locked-differential limit case — day-one support for karts and live axles. A standalone clutch coupler is deferred; shift and clutch dynamics live inside `Gearbox`. Wheels are named `FL`, `FR`, `RL`, `RR` (serialized uppercase), and every per-wheel channel in the results uses the canonical order `[FL, FR, RL, RR]` (`WHEEL_ORDER` in `crates/outlap-qss/src/qss.rs`).

The control layer is `drivetrain.control`: **static splits** (`split.front` = front-axle torque share 0..1, omitted for single-axle cars; `split.left` = left-side share) and a **torque-vectoring** stub (`enabled`, `k_yaw`, a yaw-rate feedback `ΔM_z = k_yaw · (r_target − r)`; rule-based control only in this release — optimization-based allocation is on the roadmap, Chapter 15).

Because "config errors are a product surface", the loader validates the graph at load time (`crates/outlap-schema/src/load/topology.rs`) with plain-language messages: every unit must actually reach wheels; a lumped `drive_unit` map "must not sit behind a gearbox/fixed ratio" (its ratio is already applied); a wheel rigidly driven by two units with no differential between them is a conflict (parallel hybrids sharing a diff pass); and torque vectoring cannot act across a locked/solid diff on the same axle.

### 9.3 From graph to traction ceiling

The lap solvers of Chapter 8 need one number from all of this: *the largest drive force the powertrain can put on the road at speed $v$*. The T1 reduction (`crates/outlap-qss/src/t1/powertrain.rs`, type `T1Powertrain`) folds each unit's coupler path into a set of **gears** using the shaft-speed/force convention

$$\omega_\text{shaft} = \frac{\text{ratio}}{r_\text{wheel}}\, v, \qquad F_\text{wheel} = \frac{\text{ratio}\cdot\eta_\text{mech}}{r_\text{wheel}}\,\tau, \qquad \text{ratio} = \prod(\text{fixed ratios})\cdot\text{gear ratio}\cdot\text{final drive},$$

where $\omega_\text{shaft}$ is the source shaft speed (rad/s), $v$ the vehicle speed (m/s), $r_\text{wheel}$ the driven tyre's unloaded radius (front-axle radius for a front-driven unit, rear for rear, the mean if a unit spans both axles), $\tau$ the source torque (N·m), and $\eta_\text{mech}$ the constant mechanical gearbox efficiency. Fixed ratios multiply into the base ratio; differentials are 1:1 at the power level; the first gearbox supplies the selectable ratios (a second gearbox folds in as its final drive times its first ratio). A gearbox declared with a *map* efficiency assembles fine but contributes a conservative constant proxy of **0.95** to the traction force until the map is installed — recorded in the assembly notes, nothing silent.

The **traction ceiling** is then the best on-envelope gear of every unit, summed:

$$F_{\max}(v) \;=\; \sum_{\text{units}}\ \max_{g\,:\,\omega_g \le \omega_\text{max}}\ \frac{\tau_\text{peak}(\omega_g)\cdot \text{ratio}_g \cdot \eta_g}{r_\text{wheel}},$$

with $\tau_\text{peak}(\omega)$ the `.ptm` peak envelope fitted with the project's one shared monotone cubic Hermite interpolant (Fritsch–Carlson construction; see Chapter 5) and gears whose shaft speed exceeds the envelope's top simply rev-limited out (`PtUnit::max_wheel_force`, `T1Powertrain::max_drive_force` — allocation-free, per the hot-loop rules of Chapter 6).

**A worked example.** The Model 3's medium drive unit is `kind: drive_unit` with the ratio already applied, so its "shaft" *is* the wheel-side shaft (that is why its speed axis only runs 10–1990 rpm) and the path contributes ratio 1, efficiency 1 (a differential is 1:1 at the power level). The rear road tyre's unloaded radius is 0.313 m (`UNLOADED_RADIUS` in `data/vehicles/tesla_model3_rwd/tyr/road.tyr.yaml`). Below 670 rpm the envelope holds its 2765 N·m plateau, so

$$F_{\max} = \frac{2765\ \mathrm{N\,m} \times 1 \times 1}{0.313\ \mathrm{m}} \approx 8834\ \mathrm{N} \quad\Rightarrow\quad a_x \approx \frac{8834}{1765\ \mathrm{kg}} \approx 5.0\ \mathrm{m/s^2} \approx 0.51\,g,$$

comfortably below a warm road tyre's grip — a launch-limited EV is traction-map-limited, not grip-limited, exactly as you would expect. At the envelope's top breakpoint (1990 rpm = 208.4 rad/s, 972.6 N·m) the mechanical power is $\tau\,\omega \approx 203$ kW — the sizing quoted in the vehicle README — and the rev limit puts the force ceiling at zero beyond $v = \omega_\text{max} r \approx 65$ m/s (≈235 km/h).

Three deliberate separations are worth internalizing:

1. **Efficiency never reduces force.** The `.ptm` torque envelope is already the *mechanical output* of the unit; the efficiency map governs the *energy drawn*, not the force delivered (`docs/theory/qss-powertrain.md`). A 90 %-efficient motor making 300 N·m still makes 300 N·m — it just draws more electrical power doing it.
2. **The g-g-g-v envelope does not contain the powertrain.** Following Werner et al. (2025, §II-C), the acceleration envelope of Chapter 8 is the tyre-force limit only; the lap solver applies the powertrain ceiling separately, taking $\min(a_{x,\text{grip}},\ F_{\max}(v)\,/\,m - a_\text{drag})$ in its forward pass (`crates/outlap-qss/src/solver.rs`).
3. **Braking is friction-limited.** The backward (braking) pass uses grip, drag, and grade only — no powertrain term. At every tier in this release the brakes are assumed strong enough that the tyre is the limit.

One tier asymmetry to know about: an F1-style ERS (energy-recovery system; schema `crates/outlap-schema/src/vehicle/ers.rs`, MGU-K only — "MGU-H removed per the 2026 F1 regulations") is *not* folded into the T1 traction ceiling (it is a separate rule-based deployment mechanism, folded in with the future energy manager), but the simpler T0 point-mass vehicle *does* add a power-capped, speed-tapered ERS force to its `tractive_force`. Both facts are recorded in assembly notes.

The clean-room references for this module (cited in `crates/outlap-qss/src/t1/powertrain.rs` and `docs/theory/qss-powertrain.md`): Perantoni & Limebeer, *"Optimal control for a Formula One car with variable parameters"*, Vehicle System Dynamics 52(5), 2014; Guiggiani, *The Science of Vehicle Dynamics*, 2nd ed., 2018, ch. 3; Milliken & Milliken, *Race Car Vehicle Dynamics*, 1995, ch. 20.

### 9.4 Differentials: who gets the torque

A **differential** is the gear set that lets an axle's two wheels turn at different speeds through a corner while sharing the drive torque. How it shares that torque changes the car's balance, so the QSS trim (Chapter 8) treats the split as a genuine unknown, not a post-processing step. The `DiffModel` in `t1/powertrain.rs` implements the exact semantics:

**Torque-bias capacity** — the maximum torque *difference* the diff can sustain between its output shafts at axle torque $\tau_\text{axle}$ (`DiffModel::max_torque_bias`):

| Kind | Bias capacity | Meaning |
|---|---|---|
| `open` | $0$ | both shafts always carry equal torque |
| `locked` / `solid` | $+\infty$ | any difference; the housing reacts it |
| `lsd` | $T_\text{bias} = \text{preload} + \text{ramp}\cdot\lvert\tau_\text{axle}\rvert$ | preload plus a load-proportional lock (drive ramp under acceleration, decel ramp under braking) |

**Drive-torque split** given each side's available grip torque $\mu F_z r$ (`DiffModel::split`): open → equal halves (so the lesser-grip side caps what the axle can deliver — the classic one-wheel-spin limit); locked/solid → grip-proportional (each side takes what it can hold, summing to the total — the left/right force difference produces a yaw moment); LSD → grip-proportional, then the side-to-side difference clamped into $\pm T_\text{bias}$ while keeping the sum.

Inside the live trim the 9th unknown $w$ is the driven-axle **slip split** ($\kappa_\text{left} = s + w$, $\kappa_\text{right} = s - w$), closed by a residual per kind (`docs/theory/qss-powertrain.md`): open ⇒ equal longitudinal force ($F_{x,\text{left}} - F_{x,\text{right}} = 0$, unequal slip); locked/solid ⇒ equal speed ($w = 0$); **LSD uses the locked constraint in the trim** — a documented quasi-steady-state simplification (a preloaded LSD locks up at the traction limit; partial unlocking is a T2 refinement). Under braking $w = 0$ always: the brake balance bar splits brake torque, not the differential.

This is not bookkeeping — it produces real, visible physics. When an open-diff car asks for maximum lateral *and* maximum longitudinal at once, the inner wheel unloads until the equal-torque root ceases to exist, and the point becomes a clean traction boundary: the theory page notes the front-wheel-drive reference car shows exactly this at $\lvert a_y\rvert = 6,\ a_x = 3$ m/s². A locked diff at the same point delivers the *sum* of the two wheels' capability, and its left/right force difference feeds a yaw moment straight into the trim's moment balance.

Two conventions codified in the implementation: an LSD `ramp` value greater than 1 is read as a *percent* lock-up (divided by 100, then clamped to $[0,1]$ — `lock_fraction`, documented in the theory page), and a rigid two-wheel-drive path with *no* diff coupler defaults to `locked` ("a rigid two-wheel drive with no diff is a solid axle").

**Conservation and splits.** Every coupler is a linear torque gain, and the property-test identity is $\sum \tau_\text{wheel} = \tau_\text{source}\cdot\text{ratio}\cdot\eta$ (`T1Powertrain::wheel_torque`). The static splits always sum to one: `axle_split()` returns `(front, 1 − front)` with default `(0, 1)` (all torque to the driven axle), `side_split()` returns `(left, 1 − left)` with default `(0.5, 0.5)`, both clamped to $[0,1]$.

### 9.5 Energy accounting and closure

Force is only half the story; the other half is *what it costs*. Once the sidecar tables are decoded and installed (`T1Powertrain::install_maps` — the Parquet decode happens at the native edge so the solver crates stay wasm-clean), every source-shaft operating point $(n, \tau)$ with mechanical power $P_\text{mech} = \tau\,\omega$ yields an `EnergyPoint`:

$$
\begin{aligned}
\text{drive } (\tau > 0):\quad & P_\text{source} = P_\text{mech}/\eta, & \text{loss} &= P_\text{mech}\,(1/\eta - 1),\\
\text{regen } (\tau < 0):\quad & P_\text{source} = P_\text{mech}\cdot\eta, & \text{loss} &= \lvert P_\text{mech}\rvert\,(1 - \eta),\\
\text{ICE fuel rate}:\quad & \dot m_\text{fuel} = P_\text{source}/\text{LHV} & &(\text{when } P_\text{source} > 0),
\end{aligned}
$$

where $\eta$ is the map efficiency (clamped to $[10^{-3}, 1]$ inside the energy math so a degenerate cell cannot blow up a division) and LHV is the lower heating value of the reference fuel, a constant `FUEL_LHV_J_PER_KG = 43.0e6` (petrol ≈ 43 MJ/kg, config-overridable later). "An ICE burns fuel whenever it draws chemical power (drive or idle); motoring does not." Fuel mass is accounted but held constant this release — there is no fuel-mass slow state yet.

Sign conventions, stated once because they thread through everything here: the vehicle frame is ISO 8855 ($x$ forward, $y$ left, $z$ up), so a positive drive force pushes the car along $+x$; positive shaft torque is the drive quadrant and negative torque the regen/motoring quadrant (matching the `.ptm` load axis, where negative values are regen); and battery current is **discharge-positive**, so regen power demands are negative at the pack.

When a **loss map** is present, the loss is taken as measured and energy closes *exactly* — $P_\text{source} = P_\text{mech} + \text{loss}$ — in every quadrant, *including idle*: the sidecar's zero-torque column carries $\eta = 0$ as a sentinel (efficiency is meaningless at zero output), but its `loss_w` there is the real spin/idle draw. The committed Model 3 medium unit, for instance, draws 124 W at its lowest speed breakpoint with zero torque commanded. Without a loss map the loss is derived from the efficiency and closes to interpolation accuracy between grid nodes.

The per-segment aggregate the lap loop consumes is `T1Powertrain::traction_energy(v, wheel_force_n, vdc)` → `TractionEnergy { source_w, loss_w, omega_rad_s }`: the requested positive drive force is distributed across mapped units in proportion to their best-gear capacity at that speed, each unit's $(n, \tau)$ point is evaluated through its (possibly voltage-coupled) map, and `omega_rad_s` reports the *fastest* contributing shaft speed — the air-gap-cooling driver for the thermal step below. It returns `None` when no unit has an installed efficiency map, in which case the whole slow-state coupling stays inert. One documented simplification: a hybrid ICE + electric drive attributes the full traction draw to the mapped units ("a distinct ICE source is treated as battery-fed") — exact for pure-electric cars, conservative otherwise.

Per the project's "new physics ⇒ new property test" gate, this module ships with tests you can read as an executable spec (`crates/outlap-qss/tests/t1_powertrain.rs`, `properties.rs`; listed in the theory page): open ⇒ equal torque; locked/solid ⇒ grip-proportional; the LSD inside its bias band; coupler conservation $\sum\tau_\text{out} = \tau_\text{in}\cdot\text{ratio}\cdot\eta$; splits summing to one; energy closure at the drive nodes; the ICE fuel rate positive under load; the open diff splitting driven-wheel slip in the live trim; and a traction ceiling that is positive and falls with speed for a geared engine. The per-segment advance is also under the zero-allocation gate (`tests/alloc.rs`).

A note for code readers: the `outlap-powertrain` crate in the workspace is an empty placeholder ("implemented in a later milestone"). Everything in this chapter lives in `outlap-qss::t1::powertrain`, `outlap-qss::t1::thermal`, `outlap-qss::t1::battery`, and `outlap-thermal`.

### 9.6 Machine thermal: the N-node LPTN

A cold motor and a heat-soaked motor are different cars. outlap models this with a **lumped-parameter thermal network** (LPTN): the machine is divided into $N$ isothermal lumps ("nodes") — winding, stator iron, rotor, housing, coolant, ambient — each with a heat capacity $C_i$ (J/K), connected by thermal conductances $g_{ij} = 1/R_{ij}$ (W/K), with the `.ptm` losses injected as per-node heat sources $P_i$ (W). The result is a small linear ODE system that is cheap enough to advance every track segment.

#### 9.6.1 The firewall amendment (Decision #25)

Strictly read, the firewall of Section 9.1 forbids modelling machine internals — and a thermal network *is* a model of machine internals. This is the one deliberate, author-authorized exception, recorded in `docs/theory/machine-thermal.md`: Locked Decision #25 (originally a fixed 2-node model) was amended on 2026-07-05 to allow any-$N$ networks, and for the *detailed* path outlap even builds the conductance operator from machine geometry using ported heat-transfer correlations. "The amendment is narrow — it applies to the thermal model only; torque, efficiency and loss still cross the firewall as neutral `.ptm` maps." The correlations are implemented clean-room from the published literature cited below; the upstream tool's geometry-building code was not ported.

#### 9.6.2 The network and its integrator

Each integrated node obeys an energy balance; the ambient and coolant nodes are boundary conditions (`docs/theory/machine-thermal.md`):

$$C_i\,\frac{dT_i}{dt} = P_i + \sum_j g_{ij}\,(T_j - T_i), \qquad T_\text{ambient} = T_\text{amb}, \qquad T_\text{coolant} = T_\text{inlet} + \frac{Q_\text{in}}{2\,\rho\, c_p\, \dot m},$$

where $T_i$ is node temperature (K), $T_\text{amb}$ the pinned ambient (from `conditions.yaml` or an override), and the coolant node is closed by a **quasi-static jacket balance**: with heat inflow $Q_\text{in}$ and coolant capacity rate $\rho c_p \dot m$ (W/K), the coolant sits at the mean of inlet and outlet temperature rather than being integrated. Writing the conductance operator $G$ with Kirchhoff diagonal $G_{ii} = -\sum_{j\ne i} g_{ij}$, the system is $C\dot T = GT + P$ and the update is a **Crank–Nicolson** (trapezoidal) step:

$$\left(\frac{C}{h} - \frac{G}{2}\right) T_{+} = \left(\frac{C}{h} + \frac{G}{2}\right) T + P .$$

Crank–Nicolson matters here because it is **A-stable**: the step size $h$ over a track segment is $\Delta s / v$ — potentially a second or more — which would blow up an explicit integrator on a stiff network, but stays bounded here no matter how coarse the segments are. The implementation (`crates/outlap-thermal/src/network.rs`) assembles $G$ at the current temperatures (semi-implicit, since convection conductances depend on temperature), replaces the ambient row with $T_+ = T_\text{amb}$ and the coolant row with its balance target, and solves by fixed-size Gaussian elimination with partial pivoting. Everything lives in stack buffers sized `MAX_NODES = 24` (the upstream design tool's full network is 20 nodes), so the advance is allocation-free — the hot-loop discipline of Chapter 6. Failures are typed (`TooManyNodes`, `Singular`, `NonFinite`, `BadStep`) and "the QSS caller consumes these as a flagged failure, never a panic".

Two edge families feed $G$:

- **Constant edges** (`Edge { i, j, g_w_per_k }`) — the conduction/contact skeleton, fixed W/K values.
- **Convection edges** (`ConvEdge`) — recomputed every step from a published correlation, $g = h\cdot A$ (or $\lambda_\text{eff} A/\delta$ for the air-gap film), so cooling depends on shaft speed and temperatures:

| `ConvKind` | Physics | Citation (on the function, `crates/outlap-thermal/src/correlations.rs`) |
|---|---|---|
| `AirGap` | modified-Taylor-number regimes: $\mathrm{Nu} = 2$ ($\mathrm{Ta}_m < 1700$), $0.128\,\mathrm{Ta}_m^{0.367}$ ($<10^4$), $0.409\,\mathrm{Ta}_m^{0.241}$ above; hot gap $\delta = \delta_0 - \kappa_\text{Fe}\, r_\text{gap}(T_\text{rotor}-T_\text{amb})$ | Becker & Kaye, *J. Heat Transfer* 84(2), 1962 |
| `RotorAir` | end-winding $h = 6.5 + 5.25\,u^{0.6}$, internal air $h = 15 + 6.75\,u^{0.65}$, $u$ = rotor peripheral speed | Kylander, doctoral thesis, Chalmers, 1995 |
| `ShaftExternal` | $\mathrm{Nu}_d = 0.076\,\mathrm{Re}_d^{0.7}$ | Etemad, *Trans. ASME* 77, 1955 |
| `FreeConvection` | Churchill–Chu cylinder $\mathrm{Nu}(\mathrm{Ra})$ + linearized radiation $h_\text{rad} = \varepsilon\sigma(T_w^2+T_a^2)(T_w+T_a)$ | Churchill & Chu, *Int. J. Heat Mass Transfer* 18, 1975 |
| `LiquidChannel` | laminar $\mathrm{Nu} = 4.36$ below $\mathrm{Re}=2300$, Gnielinski above 3000, linearly blended between (pump-driven ⇒ speed-independent) | Gnielinski, *Int. Chem. Eng.* 16, 1976 |

(TEFC fin-channel helpers — the Heiles form with Staton & Cavagnino's 1.7 turbulence factor, *IEEE Trans. Ind. Electron.* 55(10), 2008 — are also implemented.) Air properties use polynomial/ideal-gas fits valid over roughly 250–500 K; the iron expansion default is $\kappa_\text{Fe} = 10.4\times10^{-6}\,\mathrm{K}^{-1}$. The physically pleasing consequence, verified in the validation figure: the air-gap film *stiffens* with shaft speed, so the rotor magnet runs cooler at high speed for the same loss.

#### 9.6.3 The `.emotor` document and the cooling block

The network is declared in an **`.emotor`** file (schema `emotor/1.1`, `crates/outlap-schema/src/emotor.rs`, published `schemas/emotor.json`), referenced from the drive unit's `thermal:` field. The shipped Model 3 file is a complete, readable example (`data/vehicles/tesla_model3_rwd/emotor/rear_du.emotor.yaml`):

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

Node **roles** do double duty: they drive the mass heuristics of Section 9.6.5 and they identify special nodes. `winding` is required somewhere in the document (it is the default loss target and typically the binding limit); `rotor` "carries the magnet limit for PM machines"; `stator_iron` and `housing` are the usual conduction path; `coolant` and `ambient` are boundary nodes whose heat capacities are ignored (the ambient is pinned, the coolant balance-closed); `other` covers finite-element-resolved nodes on the detailed path. A node derates only if it declares both `t_warn_c` and `t_max_c`.

The **cooling block** is deliberately written in raw scalars a user (or importer) can read off a datasheet, and the assembly derives the physics (`crates/outlap-qss/src/t1/thermal.rs`):

- `jacket` — from channel count $n$, width $w$, height $h$ and flow $Q$: mean velocity $= Q/(n\,w\,h)$, hydraulic diameter $D_h = 2wh/(w+h)$, coolant capacity rate $\rho c_p \dot m = \rho\, c_p\, Q$, and a `housing↔coolant` `LiquidChannel` convection edge over `wetted_area_m2`. Fluids come as named presets (`water`, `ethylene_glycol_50`, `oil`, tabulated at ~60–70 °C film temperature) or explicit `props`; an unknown name is a typed error listing the known fluids. Declaring both `cooling.coolant` (the low-level escape hatch with explicit $\rho c_p\dot m$) *and* `jacket` is an error.
- `air_gap` — from rotor outer radius, gap, and stack length: $r_\text{gap} = r_\text{ro} + \text{gap}/2$, interface area $A = 2\pi\, r_\text{gap}\, L$, feeding the speed-dependent Becker–Kaye film between stator and rotor.

`cooling.ambient_fixed_c` overrides the ambient; omitted, the session's `conditions.yaml` `ambient_c` is used — the environment stays in the conditions file, per the input-quartet rule (Chapter 4). Initial temperatures default to each node's sink (ambient; the coolant node at its inlet), overridable via `initial_temp: {uniform_c: ...}` or per node.

#### 9.6.4 Loss routing, copper feedback, and derating

Each segment, the machine-heating loss from the `.ptm` lookup is deposited into the nodes via `loss_routing` (`MachineThermal::step`). Each route names a node, a `fraction`, and optionally a per-component loss `component` (a named `.ptm` loss-map column). The rule: declared routes deposit their share, and **whatever total loss is not routed lands on the winding node** — never removing heat. An empty routing list therefore puts *all* loss into the winding, the conservative default. (Per-component columns are currently a hook: the lap loop passes a resolver that always returns `None`, so only total-loss fractions are live this release; the importer uses component breakdowns only to compute the routing *fractions*.)

The runtime surface is small and worth knowing: `MachineThermal::step(machine_loss_w, component_loss, omega_rad_s, dt_s)` returns the derate for the segment (`machine_loss_w` is the total machine-heating loss in watts; `omega_rad_s` drives the speed-dependent convection edges), and `winding_temp_c()` is "the representative machine temperature the QSS slow-state coupling logs per segment"; `temp_c(name)` and `node_names()` expose the rest of the network.

**Copper-resistance feedback**: winding resistance rises with temperature as $R(T) = R_\text{ref}\,(1 + \alpha (T - T_\text{ref}))$, $\alpha \approx 0.00393\ \mathrm{K}^{-1}$ for copper, so when `cu_feedback` is enabled the loss at the listed nodes is rescaled by $1 + \alpha(T - T_\text{ref})$ each step (floored at 0). This is positive feedback — hotter winding, more loss, hotter winding — and what keeps it physically bounded is the **derate**:

$$\text{derate} = \min_{\text{rated nodes}}\ \operatorname{clamp}\!\left(\frac{T_\text{max} - T}{T_\text{max} - T_\text{warn}},\ 0,\ 1\right),$$

a linear ramp from 1 to 0 as each rated node crosses its warning temperature toward its maximum. A node participates only if it declares *both* `t_warn_c` and `t_max_c` (in the Model 3 file: winding 150→180 °C, rotor/magnets 140→170 °C); boundary nodes never derate; a degenerate `t_max ≤ t_warn` becomes a hard step. The winding normally binds. The lap solver multiplies the traction ceiling by this factor (Section 9.9), and the reduced torque reduces the next segment's loss — the physical loop, closed.

#### 9.6.5 Two authoring tiers, one integrator

The same integrator serves a community user typing YAML and a detailed import:

- **Lumped (hand-authored)** — role-tagged nodes, constant conductances. Anything omitted is filled from *documented mass heuristics* using the `.ptm`'s `mass_kg`: capacity $C = f_\text{role}\cdot m\cdot c_p$ (winding $0.15\,m \times 385$ J/kg·K copper; stator iron $0.45 \times 460$; rotor $0.25 \times 450$; housing $0.15 \times 900$ aluminium) and role-pair reference conductances at $m_0 = 40$ kg scaled by $(m/m_0)^{2/3}$ (interface area ∝ mass^{2/3}: winding↔stator 30 W/K, winding↔housing 8, stator↔housing 60, housing↔coolant 200, housing↔ambient 5, rotor↔anything 3). Every heuristic fill is recorded in `estimates()` and surfaced in the loaded-model report — estimates are visible, never silent. A node with no capacity and no applicable heuristic is a hard error telling you to set it explicitly.
- **Detailed (imported)** — the importer collapses a finite-element-resolved network onto the same reduced menu, with explicit capacities, real inter-group conductances, and convection edges rebuilt each segment from the correlations. The `convection` edge list in `emotor/1.1` remains an advanced escape hatch for a fully explicit network. Machine-topology coverage this release: IPM / SPM / SynRM.

Validation (Chapter 13 has the full story): the Crank–Nicolson advance matches the analytic single-node step response $T(t) = T_\text{amb} + (P/g)(1 - e^{-t g/C})$; the ambient stays pinned; the derate is monotone in temperature; a stint heat-soaks monotonically; the coolant node holds its quasi-static target; copper feedback raises the steady state.

### 9.7 The battery pack: a Thevenin equivalent circuit

For an electric car the battery sets two more limits: how much power it can deliver at all, and — more subtly — what *voltage* it delivers it at. outlap models the pack as a **Thevenin equivalent circuit** (an ECM, equivalent-circuit model): an ideal voltage source (the open-circuit voltage, OCV) behind a series resistance $R_0$ and one resistor–capacitor pair $(R_1, \tau_1)$ that captures the slow "sag" after a load step. With discharge current $I$ positive, the terminal (DC-link) voltage is

$$V_\text{term} = \mathrm{OCV}(\mathrm{SoC}, T) - I\,R_0 - V_\mathrm{RC}, \qquad V_\mathrm{RC} \to I R_1 \ \text{at time constant } \tau_1 .$$

All five parameters — OCV, $R_0$, $R_1$, $\tau_1$, and the entropic coefficient $\mathrm{d}U/\mathrm{d}T$ — are tabulated on a $(\mathrm{SoC}, T)$ grid in a sidecar (columns `soc, temp_c, ocv_v, r0_ohm, r1_ohm, tau1_s, dudt_v_per_k`; the shipped `pack_800v.tables.parquet` is 18 rows = 6 SoC × 3 temperatures). The equivalent-circuit form and its state equations follow the published NREL `thevenin` model (BSD-3) and the ECM literature it cites (Plett, *Battery Management Systems* Vol. 1, 2015, ch. 2–3), re-authored clean-room (`crates/outlap-qss/src/t1/battery.rs`).

The `battery/1.0` document (`crates/outlap-schema/src/battery.rs`; the vehicle references it as `battery: {model: rc_pairs, params: ...}`) adds the pack context around the cell curves — from the shipped `data/vehicles/tesla_model3_rwd/battery/pack_800v.battery.yaml`:

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

Cell-level tables scale to the pack by $n_s$ for voltages and $n_s/n_p$ for resistances. Only `rc_pairs: 1` is supported (a typed error otherwise); the ECM maps *clamp* outside their grid ("the ECM is only defined on its measured hull"). `Pack::assemble` starts the state at the **top of the SoC window** — full charge, the reference state the static envelope assumes — at coolant temperature with a relaxed RC branch; you can pass an explicit `initial_soc`.

**Per-segment advance.** `Pack::step_power(state, power_w, dt)` does, in order:

1. **Clip to the power envelope**: discharge demand is capped by `discharge_power_limit_w` (the SoC-dependent monotone-cubic curve, forced to exactly 0 at or below the SoC-window floor), regen by `regen_power_limit_w` (0 at or above the ceiling). The clipping is reported via `power_limited`.
2. **Solve the current** from the constant-power Thevenin relation $R_0 I^2 - \mathrm{emf}\,I + P = 0$ with $\mathrm{emf} = \mathrm{OCV} - V_\mathrm{RC}$, taking the physical low-current root $I = \bigl(\mathrm{emf} - \sqrt{\mathrm{emf}^2 - 4 R_0 P}\bigr)/(2R_0)$; if the demand exceeds the maximum deliverable $P_\text{max} = \mathrm{emf}^2/(4R_0)$, the max-power current $\mathrm{emf}/(2R_0)$ is used ($R_0$ floored at $10^{-9}\,\Omega$).
3. **Advance three slow states**: $V_\mathrm{RC} \leftarrow V_\mathrm{RC}\,e^{-\Delta t/\tau_1} + I R_1 (1 - e^{-\Delta t/\tau_1})$ — the *exact* exponential integrator, reproducing the closed-form constant-current response to machine precision; the state of charge by Coulomb counting, $\mathrm{SoC} \leftarrow \operatorname{clamp}\!\bigl(\mathrm{SoC} - I\,\Delta t/(3600\,Q_\text{Ah}),\ 0,\ 1\bigr)$; and the lumped pack temperature, heated by the irreversible $I^2 R_0 + V_\mathrm{RC}^2/R_1$ term plus the entropic $I\,T\,\mathrm{d}U/\mathrm{d}T$ term (which can cool), relaxed to the coolant through $R_\text{th}$ with a semi-implicit Euler step (A-stable, like everything else on the slow timescale).

The quasi-steady-state simplification is that the current is constant within one segment; the RC state carries memory *across* segments. A current-driven path (`step_current`) exists for the pulse-response validation, which matches the closed-form Thevenin response at well under 1 % RMS.

The battery's executable spec lives in `crates/outlap-qss/tests/battery.rs`: the pulse response against $V(t) = \mathrm{OCV} - I R_0 - I R_1 (1 - e^{-t/\tau})$; a regen pulse lifting the terminal *above* OCV; SoC monotone under discharge; the discharge limit clipping to zero at the SoC-window floor; and a fully deterministic slow-state advance (the same inputs give bit-identical states — no hidden clocks, matching the determinism rules of Chapter 6).

### 9.8 The Vdc–SoC coupling

Here is where the battery and the motor maps meet, and it is the reason `ptm/1.1` exists. A real inverter-fed machine performs differently at different DC-link voltages: as the pack drains, its terminal voltage drops, and efficiency and losses shift. The coupling rule (a recorded user decision, 2026-07-05, documented in `docs/theory/qss-powertrain.md` §8.4) is a simple presence matrix:

| Battery block | `.ptm` `vdc_v` axis | Behaviour |
|---|---|---|
| present | present (`ptm/1.1`) | **Coupled**: the 3-D $(\text{speed}, \text{torque}, V_\text{dc})$ efficiency/loss maps are evaluated at the pack's live terminal voltage $V_\text{term}$ each segment |
| present | absent | Single-voltage: the map ignores the pack voltage |
| absent | present | Single-voltage at the map's reference voltage `meta.dc_voltage_v` (if that is missing or ≤ 0 on a Vdc-stacked map, the fallback reference is the *middle of the Vdc grid*, not 0 V) |
| absent | absent | Single-voltage (the pre-1.1 world) |

When coupled, a low-SoC point shifts **both** the traction efficiency *and* the machine-heating loss injected into the thermal network — one lookup feeds two physics.

The interesting numerical case is deliberate in the shipped data: the synthetic 220S pack swings ≈ 634–810 V open-circuit over its SoC grid, while the drive-unit maps are gridded 730–850 V — so under low-SoC load the terminal voltage sags *below* the map. On the Vdc axis (and only there — speed and torque clamp) the shared monotone Hermite interpolant uses **linear out-of-domain extrapolation** from the boundary slice (`OutOfDomain::Linear`, Decision #30), C¹-continuous with the interior, so the map stays usable instead of freezing at its edge. Extrapolated values are held to physical bounds — concretely, the efficiency is clamped to $[10^{-3}, 1]$ inside the energy math — and the fact that a unit's map is Vdc-coupled with linear extrapolation is recorded in the assembly notes / loaded-model report, so an extrapolating run is never silent. The decode contract puts the Vdc axis last in tensor order (`T1Powertrain::map_axis_names_vdc`, debug-asserted).

The property tests (Chapter 13) pin this down: a Vdc-stacked map built from a field linear in $V_\text{dc}$ is reproduced *exactly* under extrapolation below and above the grid; the presence matrix behaves; and a draining pack drives a lower coupled efficiency.

To see the coupling with your own eyes, run the Model 3 sizing sweep in the capstone notebook — swapping the drive unit is a one-line what-if override, `overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"}`, no files edited (Chapter 14, Recipes, walks through it).

### 9.9 Slow states in the lap loop

Everything in Chapter 8 was **fast**: the trim states (slips, loads, forces) equilibrate within a track station and carry no memory. A **slow state** is the opposite — a quantity that evolves over seconds to minutes and *remembers*: the machine node temperatures, the pack's SoC, its RC overpotential, and its lumped temperature. This section is where the two timescales meet.

#### 9.9.1 Building the stack

The coupling stack (`SlowCoupling` in `crates/outlap-qss/src/qss.rs`) is assembled at the native edge by `build_slow_stack` (`crates/outlap-py/src/lib.rs`) **from the vehicle's own references** — no extra API arguments: it requires (a) a `battery:` block whose params and ECM sidecar load, and (b) the *first* drive unit carrying a `thermal:` `.emotor` reference. A missing file leaves the coupling **inert with a note**; a present-but-broken file is a real error. Nothing silent, in either direction. The notes you may see in a lap result's provenance (their exact source is `build_slow_stack`):

```text
battery params `battery/f1_es.yaml` not present — slow-state coupling inert
battery present but no drive unit declares a `.emotor` thermal model — slow-state coupling inert
machine thermal: node `housing` capacity estimated from mass (11070 J/K)
2 drive units declare `.emotor` thermal models — the QSS coupling marches ONE network (unit 0); ...
```

(the third line is the mass-heuristic surfacing of Section 9.6.5 — here $0.15 \times 82\ \mathrm{kg} \times 900\ \mathrm{J/(kg\,K)}$ for a housing node left without `c_j_per_k` on an 82 kg unit; the shipped Model 3 `.emotor` declares all its capacities explicitly, so it loads estimate-free on this front). This release marches ONE thermal network; if several units declare `.emotor` models, the extras are dropped with a note (multi-machine stacks arrive with the ERS energy manager).

#### 9.9.2 The outer march

The static g-g-g-v envelope stays **thermal- and SoC-neutral** — it is generated once at the reference state (cold machine, full charge) and neither the derate nor the battery cap is baked into it. The coupling is instead resolved by a bounded, deterministic **outer march** (`solve_profile`): solve the uncoupled velocity profile to seed; then, `OUTER_ITERS = 2` times, derive the per-segment longitudinal acceleration, march the slow states along the profile to build a per-station **traction scale**, and re-solve the profile with that scale (`solve_into_ggv_scaled`). A final march against the converged profile makes the reported SoC/temperature channels match it. The count is fixed rather than tolerance-driven, for determinism — "a single flying lap moves the slow states little", so two iterations are ample.

The design is deliberately safe in the absence of a stack: "when no mapped stack is supplied the scale stays ≡ 1 and the result is bit-identical to the uncoupled solve" (`crates/outlap-qss/src/qss.rs` module doc) — so adding a battery file to a vehicle can never perturb an unrelated lap through numerics alone, and the tier-parity gates of Chapter 13 are unaffected.

#### 9.9.3 One segment of `march_slow_states`

Per segment $i$, with segment time $\Delta t = 2\,\Delta s/(v_i + v_j)$, resetting the thermal network and pack to their assembled states at the start of every march (deterministic, zero heap allocation):

1. Log the **entry** state at station $i$ (station 0 reports the initial SoC and winding temperature).
2. Compute the wheel drive force actually demanded, positive part only: $F_\text{drive} = \max\bigl(0,\ m(a_{x,i} + a_\text{drag}(v_i) + g\sin\theta_g)\bigr)$ — drag and grade included, braking excluded.
3. Read the coupling voltage: $V_\text{dc} = $ `pack.terminal_voltage_v(state)`.
4. Look up `traction_energy(v_i, F_drive, Some(vdc))` → source power, machine loss, shaft speed.
5. Step the thermal network: `derate = thermal.step(loss_w, |_| None, omega_rad_s, dt)`. A thermal-integrator error leaves the derate at 1 (no cap) — flagged, not fatal.
6. Evaluate the battery cap **before** the step advances SoC: if $P_\text{source} > P_\text{cap}$ (the SoC-dependent discharge limit), then $s_\text{batt} = \operatorname{clamp}(P_\text{cap}/P_\text{source},\ 0,\ 1)$, else 1.
7. `pack.step_power(...)` advances SoC, $V_\mathrm{RC}$, and pack temperature.
8. **Compose**: $\text{scale}[i] = \operatorname{clamp}\bigl(\min(\text{derate},\ s_\text{batt}),\ 0,\ 1\bigr)$.

Two logging details make the reported channels line up with intuition. Station $i$ records the **entry** state — the state the car carries *into* segment $i$ — so station 0 shows the initial SoC and temperature and nothing leads the car by one segment; on an open (point-to-point) path the final station carries the end-of-lap state instead. And because every march starts from the assembled reference state, the whole coupled solve is a pure function of its inputs: run it twice, get bit-identical channels.

The two caps compose by `min` because they are both ceilings on the same thing — deliverable drive power — and the binding one wins. In the profile solver's forward step the scale multiplies *only* the powertrain traction ceiling:

$$a_x = \min\!\Bigl(a_{x,\text{grip}},\ \frac{F_\text{pt,max}(v)\cdot \min(\text{derate},\, s_\text{batt})}{m} - a_\text{drag}(v)\Bigr) - g\sin\theta_g,$$

while braking is untouched — "it draws no drive power" (`GgvGrip.traction_scale`, `crates/outlap-qss/src/solver.rs`). Grip, in other words, is never derated; only the engine room is.

The result surfaces as two per-station channels in the `slow` group of the lap result, at both `t0` and `t1` tiers (`SlowLog`): `state_of_charge` (0..1) and `machine_temp_c` (the winding node, °C — a display-boundary unit). They are attached only when the coupling actually did something (SoC moved, the winding heated, or any scale dipped below 1). Chapter 10 shows them as xarray variables.

#### 9.9.4 Honest limitations in this release

- **Regen does not recharge the pack.** SoC is a discharge-only bound — monotone non-increasing over a lap; recovery phases arrive with the ERS energy manager (`crates/outlap-qss/src/qss.rs` module doc). The vehicle's `brakes.regen_blend.max_regen_frac` field (0.6 on the Model 3) is a friction/regen brake *blend* declaration, not SoC recovery, and the QSS lap loop does not consume it yet.
- **Named per-component loss routing is a hook**: the lap loop passes a `|_| None` resolver, so only total-loss fractions heat the network this release.
- **Hybrid attribution**: the full traction draw is attributed to the mapped (electric) units; exact for pure EVs, conservative for hybrids.
- **Battery temperature is advanced but not logged** as a result channel — `SlowLog` carries only SoC and machine temperature; the pack temperature lives in `PackState.temp_k` and each `StepOut.temp_c`.
- **One thermal network** per lap, as noted above; and there is **no fuel-mass slow state** for ICE cars yet.
- The **T0/T1 ERS asymmetry** of Section 9.3 means the two tiers see slightly different powertrain ceilings on an ERS-equipped car, by design, until the energy manager lands.

For where these limits sit on the roadmap, see Chapter 15, Limitations and roadmap; for the shipped vehicles that exercise this whole stack end-to-end — including the Model 3 sizing sweep (small 1365 N·m/≈100 kW, medium 2765 N·m/≈203 kW, large 3381 N·m/≈248 kW) — see Chapter 12, The shipped data library.


---

## 10. The Python API reference

*What you will learn: every public class and function in outlap's Python package — what goes in, what comes out, what errors it can raise, and a minimal runnable example for each. You will also get the definitive contract for the xarray `Dataset` that every solved lap returns, the queryable g-g-g-v envelope object, and the schema-validation tooling. This chapter is deliberately dry; read Chapters 3 and 4 first if any example feels unmotivated.*

### 10.1 The import model and general conventions

outlap's Python surface is split across a handful of modules. The one you will use almost all the time is `outlap.core`.

| Import | Source | What it is |
|---|---|---|
| `outlap` | `python/src/outlap/__init__.py` | A placeholder. It exposes only `main()`, which prints a greeting. **Do not** expect the API here. |
| `outlap.core` | `python/src/outlap/core.py` | **The typed user API.** A thin veneer over the Rust bindings: numpy-style broadcasting for tire evaluation, and results as labelled `xarray.Dataset` objects. No physics lives here. |
| `outlap_core` | `crates/outlap-py/src/lib.rs` (compiled extension) | The raw Rust bindings (PyO3). `outlap.core` re-exports its classes; you rarely import it directly. Typed stubs ship in the wheel (`crates/outlap-py/outlap_core.pyi`). |
| `outlap.schemas` | `python/src/outlap/schemas.py` | Loads the committed JSON Schemas and validates fixtures/data against them (§10.8). |
| `outlap.tir` | `python/src/outlap/tir/` | The `.tir` interchange codec (§10.9, and Chapter 11, Importers and tooling). |
| `outlap.tirefit` | `python/src/outlap/tirefit/` | The MF6.1 tire-fitting pipeline (§10.9, Chapter 11). |
| `outlap.importers` | `python/src/outlap/importers/` | PDT HDF5 and track importers (§10.9, Chapter 11). |

A common first stumble, verified against the shipped package:

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

`outlap.core.__all__` is the complete public surface of the core API — fourteen names:

```text
DEFAULT_DS_M, Envelope, Lap, Raceline, Track, Tyre, TyreForces,
lap_dataset, min_curvature, solve_lap, solve_lap_dataset,
track_dataset, tyre_forces, vehicle_report
```

`DEFAULT_DS_M` is a module constant, `2.0` — the default spatial sampling step in metres (the distance between consecutive evaluation points, called *stations*, along the track). `Envelope`, `Lap`, `Raceline`, `Track`, `Tyre`, `min_curvature`, `solve_lap`, and `vehicle_report` are re-exported unchanged from the Rust extension; `TyreForces`, `tyre_forces`, `lap_dataset`, `solve_lap_dataset`, and `track_dataset` are the Python-side additions.

Conventions that apply to *everything* below:

- **Units are SI**: metres, seconds, m/s, m/s², N, N·m, Pa, rad. The two deliberate display-boundary exceptions in this API are the lap channel `machine_temp_c` (°C) and the conditions fields ending in `_c`/`_hpa` (°C, hPa) — matching the file formats they mirror.
- **Axes are ISO 8855**: x forward, y left, z up. Lateral acceleration `ay` is positive to the *left*; a raceline offset `n` is positive toward the *left* road edge. Signs below follow this convention.
- **All extension classes are immutable** (`#[pyclass(frozen)]`): you cannot set attributes on a `Lap`, `Track`, `Tyre`, `Raceline`, or `Envelope`.
- **Channel methods return fresh copies.** Every call like `lap.v()` allocates a new numpy array. Grab a channel once, or use `lap_dataset` / `solve_lap_dataset` which do it for you.
- **Errors are two exception types.** A file that does not exist raises `FileNotFoundError`. Everything else — malformed YAML, an unknown field, a bad parameter — raises `ValueError` whose message keeps the diagnostic help line (including did-you-mean suggestions). A quick-reference table is in §10.7.
- **`solve_lap` holds the GIL** (Python's global interpreter lock) for its whole duration; running solves on background threads will not overlap. A batch/sweep API that releases it is planned (see Chapter 15, Limitations and roadmap).
- **Build the extension in release mode.** With a debug-profile wheel, the first lap of a vehicle takes about a minute (envelope generation); in release it is seconds. Set `MATURIN_PEP517_ARGS="--profile release"` before `uv sync`, exactly as CI does — see Chapter 3, Installation and your first lap.

All examples below assume the environment from Chapter 3 and paths relative to the repository root, using the shipped Tesla Model 3 vehicle and Catalunya track (Chapter 12, The shipped data library).

### 10.2 Loading and reports

#### 10.2.1 `vehicle_report` — the loaded-model report

```python
def vehicle_report(
    vehicle_dir: str,
    overrides: dict[str, bool | int | float | str] | None = None,
) -> dict[str, object]
```

**In:** a path to a vehicle directory containing `vehicle.yaml` (plus whatever files it references), and optionally a `{dotted.path: value}` patch applied through the real validation pipeline. **Out:** a plain dict — the *loaded-model report*, outlap's "nothing silent" account of everything the assembly pipeline filled in, estimated, or degraded while resolving the vehicle (Chapter 4, The input quartet):

| Key | Type | Meaning |
|---|---|---|
| `name` | `str` | The vehicle's display name. |
| `resolved_hash` | `str` | blake3 hash (hex) of the fully resolved vehicle spec. Changes whenever any effective parameter changes. |
| `inherited` | `list[tuple[str, str]]` | `(json_pointer, detail)` pairs for values inherited via `extends:`. |
| `estimated` | `list[tuple[str, str]]` | Values the pipeline estimated because the file omitted them. |
| `degraded` | `list[tuple[str, str]]` | Documented-fallback combinations (only reachable with `allow_degraded`). |
| `warnings` | `list[tuple[str, str]]` | Non-fatal load warnings. |
| `overrides` | `list[tuple[str, str]]` | Echo of the applied override paths and their values (stringified). |

**Errors:** `FileNotFoundError` if `vehicle.yaml` (or a file it references) is missing; `ValueError` for malformed files or an invalid override path/value.

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

With an override, the report echoes it and the hash changes:

```python
rep = vehicle_report("data/vehicles/tesla_model3_rwd",
                     overrides={"chassis.mass_kg": 1500.0})
print(rep["overrides"], rep["resolved_hash"][:12])
```

```text
[('chassis.mass_kg', '1500.0')] d62292c121a0
```

Get into the habit of checking this report before trusting a lap time: `estimated` and `degraded` entries tell you where the model is running on assumptions rather than your data.

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

**In:** `Track.load` takes a *directory* holding a `track.yaml` plus its centerline CSV (Chapter 5, Files and formats). **Out:** an immutable, queryable 3-D track ribbon. `sample(ds_m)` resamples it at a uniform step of `ds_m` metres and returns a dict of ten equal-length arrays: `s` (arc length, m), `x, y, z` (world position, m), `kappa_h` (plan-view curvature, 1/m — how tightly the road turns), `kappa_v` (vertical curvature, 1/m — crests and compressions), `grade` (uphill slope, rad), `banking` (lateral road tilt, rad), `width_left`, `width_right` (distance from centerline to each edge, m).

**Errors:** `FileNotFoundError` for a missing directory/`track.yaml`; `ValueError` for a malformed track or a non-positive/non-finite `ds_m`.

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

Note the default step here is **10.0 m**, not `DEFAULT_DS_M` — a track plot rarely needs 2 m resolution. The dataset has one dimension `s` (coordinate in m), the nine data variables above (each annotated with `units`, e.g. `kappa_h` is `1/m` with long name "plan-view curvature"), and attrs `name` (str), `length_m` (float), and `closed` (an `int`, because netCDF attributes have no boolean type). Real output, trimmed:

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

**In:** `Tyre.load` takes the path to a single `.tyr.yaml` *file* (not a directory) and builds the evaluatable Magic Formula 6.1 steady-state tire model (Pacejka's empirical force model — Chapter 7, Physics I). Note the pressure conversion at the boundary: the file stores kPa, the attribute is Pa.

`forces` is the raw binding: six **equal-length 1-D** float64 arrays in — `kappa` (slip ratio, the fractional speed difference between tread and road), `alpha` (slip angle, rad — the angle between where the wheel points and where it travels), `gamma` (camber/inclination angle, rad), `fz` (vertical load, N), `p` (inflation pressure, Pa), `vx` (forward speed, m/s) — and five arrays out: longitudinal force `fx` (N), lateral force `fy` (N), aligning moment `mz`, overturning moment `mx`, rolling-resistance moment `my` (all N·m), in ISO 8855 signs. A length mismatch raises `ValueError` (`"length mismatch: kappa has 5 elements, alpha has 3"`).

`peak_mu(fz, p)` returns the peak friction coefficients `(μx, μy)` — the maximum force-to-load ratio the pure-slip curves reach at that load and pressure.

For everyday use, prefer the broadcasting wrapper:

```python
def tyre_forces(
    tyre: Tyre, *,
    kappa: ArrayLike = 0.0, alpha: ArrayLike = 0.0, gamma: ArrayLike = 0.0,
    fz: ArrayLike | None = None,   # default: tyre.fnomin
    p: ArrayLike | None = None,    # default: tyre.p_cold
    vx: ArrayLike = 16.7,          # m/s (~60 km/h)
) -> TyreForces
```

**In:** scalars or arrays in any numpy-broadcastable combination (all arguments keyword-only). **Out:** a `TyreForces` named tuple with fields `fx, fy, mz, mx, my`, each an `NDArray[np.float64]` shaped like the broadcast of the inputs.

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

The shipped road tire's `citation` attribute reads: `H. B. Pacejka, Tyre and Vehicle Dynamics, 2nd ed. (2006), Appendix 3, Table A3.1 (205/60R15 91V, 2.2 bar, ISO sign)` — provenance travels with the data. Its `notes` list two extraction facts, e.g. `('/mf61/QSX*', 'overturning-moment coefficients absent - Mx = 0')`: this parameter set produces `mx = 0`, and the API tells you so rather than staying silent.

### 10.3 Racelines: `min_curvature` and `Raceline`

```python
def min_curvature(
    track: Track,
    half_width_m: float,
    ds_m: float = 2.0,
    margin_m: float = 0.3,
    epsilon: float = 1e-8,
) -> Raceline
```

**In:** a loaded `Track`, the car's half-width in metres, and three numeric knobs: `ds_m` is the sampling step of the quadratic program (QP — the optimisation that picks the line), `margin_m` an extra safety margin kept from the track edges, and `epsilon` the Tikhonov regularisation (a tiny smoothing term that keeps the QP well-conditioned). **Out:** a `Raceline` — the minimum-curvature racing line, i.e. the path inside the track corridor that minimises the integral of squared curvature (see Chapter 8, Physics II). **Errors:** `ValueError` if `ds_m` or `half_width_m` is not positive and finite, or if the QP fails.

```python
class Raceline:                       # produced by min_curvature; not user-constructible
    ds_m: float                       # the step the line was GENERATED with, m
    def s(self) -> NDArray            # parent-centerline stations, m
    def n(self) -> NDArray            # signed lateral offsets (+ = road-left), m
    def line(self) -> Track           # the racing line as a first-class Track
```

`line()` matters: the racing line comes back as a real `Track` with its own curvature and length, so the solver can drive it exactly like a centerline. `ds_m` is recorded so the lap result carries honest provenance about how the line was generated.

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

The generated line is 31 m *longer* than the 4649.8 m centerline — a straighter path through corners trades distance for speed — and swings up to 7.27 m right and 5.63 m left of center. On this track it is worth about 8.4 s to the Model 3 (§10.4.3).

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

This is the QSS (quasi-steady-state) lap solver — it computes the fastest speed profile the car can sustain along the given line (Chapter 8, Physics II).

**What enters.** From disk: `vehicle_dir` must hold a `vehicle.yaml` plus every file it references (`.tyr` tires, `.ptm` powertrain maps, optional `.emotor` thermal and battery files, and binary sidecar tables such as `.parquet` aero/efficiency maps). An optional `sim.yaml` and `conditions.yaml` next to it override the built-in defaults. The rule throughout: a *missing* optional file falls back to defaults or a documented fallback **with a note in the result**; a *present but malformed* file is always an error, never silently ignored. From memory: the `Track` to drive and four optional override structures (below).

**What leaves.** An immutable `Lap` object (§10.4.2) holding the solved channels as copied arrays, the attached g-g-g-v envelope, and provenance (tier, coupling mode, resolved hash, notes).

Parameter semantics:

- `ds_m` — station spacing of the solve, m. Smaller = finer resolution, more stations. Must be positive and finite.
- `tier` — `"t0"` (point-mass on the g-g-g-v envelope) or `"t1"` (full QSS with per-wheel outputs). Overrides everything else, including `sim["tier"]` and the vehicle dir's `sim.yaml`. The default — from `schemas/sim.json` — is **`"t1"`**. `"t2"`/`"t3"` raise `ValueError` by design (transient tiers arrive in later milestones).
- `sim` — a nested dict deep-merged onto `sim.yaml`-or-defaults, e.g. `{"flat_track": True, "envelope": {"v_points": 24}}`. Unknown keys are rejected loudly (§10.4.4).
- `conditions` — a nested dict deep-merged onto `conditions.yaml`-or-defaults (ISA-like: 20 °C, 1013.25 hPa). Same strictness.
- `overrides` — a flat `{dotted.path: value}` vehicle patch (e.g. `{"chassis.mass_kg": 1500.0}`) applied through the full validation pipeline: schema-checked after the merge and recorded in provenance. Values may be `bool`, `int`, `float`, or `str`.
- `raceline_ds_m` — provenance only: when you hand-solve a generated racing line, pass the step it was generated with so the result records it. `solve_lap_dataset` does this automatically for `Raceline` inputs, so you rarely touch it.

Precedence, in one line: `tier=` argument > `sim=` dict > `sim.yaml` in the vehicle dir > built-in defaults (and the same file-then-dict order for conditions).

**Errors:** `FileNotFoundError` (missing `vehicle.yaml` or referenced file), `ValueError` (malformed file, unknown override/sim/conditions key — with did-you-mean help, bad `ds_m`, tier `t2`/`t3`, or an undecodable sidecar). See §10.7.

```python
from outlap.core import Track, solve_lap

track = Track.load("data/tracks/catalunya")
lap = solve_lap("data/vehicles/tesla_model3_rwd", track)
print(round(lap.lap_time_s, 3), lap.tier, lap.fz_coupling)
```

```text
148.081 t1 one_step_lag
```

Timing expectations: the first solve of a car generates its g-g-g-v envelope (seconds in a release build; ~65 s was measured on a debug build). Subsequent solves of the same car, conditions, and grid in the same process reuse a cached envelope and are fast — a warm `t0` solve above took 0.12 s (§10.6 explains the cache key).

#### 10.4.2 The `Lap` object

A solved lap. Attributes (plain values):

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

Channel methods (each call returns a *fresh copy*):

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

Per-wheel arrays are `n × 4` in `FL/FR/RL/RR` column order (`lap.wheels`). The *understeer gradient* is a handling metric — how much extra steering per unit lateral acceleration the car needs (positive = understeer); *aero front share* is the fraction of total downforce on the front axle; *state of charge* is the battery's remaining energy fraction; *machine temperature* is the drive motor's winding temperature (Chapter 9, Physics III).

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

Note the second line: at `t0` the per-wheel channels are `None`, but `state_of_charge()` is *not* — the slow-state channels gate on whether the vehicle has a complete battery + motor-thermal stack, not on the tier.

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

`lap_dataset` converts a `Lap` into the labelled `xarray.Dataset` that is outlap's designed results boundary (§10.5). `solve_lap_dataset` is `solve_lap` + `lap_dataset` in one call, with one extra convenience: `line` may be a `Raceline`, in which case it solves on `line.line()` and passes `raceline_ds_m=line.ds_m` for provenance automatically. All options after `line` are keyword-only. Errors are exactly `solve_lap`'s.

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

What-if experiments compose through the same call — every change flows through the real validation pipeline and is reflected in `resolved_hash`:

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

Losing 265 kg is worth 3.3 s here; a 35 °C day costs 0.07 s and leaves the motor windings 19 K hotter at the flag. One caution: the air-temperature field is `temperature_c` — an older docstring example in `outlap.core` shows `temp_c`, which the strict merge correctly rejects (`unknown conditions field 'air.temp_c'`).

#### 10.4.4 The `sim=` and `conditions=` vocabulary

Both dicts are deep-merged onto the corresponding file (or defaults) and re-validated; any key that does not exist in the schema is an error listing the known fields at that level. The vocabulary comes from `schemas/sim.json` and `schemas/conditions.json` (Chapter 5).

`sim=` fields and defaults:

| Field | Default | Meaning |
|---|---|---|
| `tier` | `"t1"` | Solver tier (`t0`/`t1`/`t2`/`t3`; the last two raise for now). |
| `envelope` | `{"v_points": 40, "ax_points": 25, "g_normal_points": 7}` | g-g-g-v grid resolution (Chapter 8). |
| `fz_coupling` | `"one_step_lag"` | Normal-load algebraic-loop mode; alternative `"fixed_point"`. Recorded in the result. |
| `flat_track` | `false` | Zero grade/banking/vertical curvature so the envelope collapses to a flat g-g (oracle-comparison mode). Recorded; the track file is untouched. |
| `allow_degraded` | `false` | Permit documented-fallback combinations; the result is marked. |
| `dt_s` | `0.001` | Fixed integration step, s — transient tiers only. |
| `integrator` | `"heun"` | Fixed-step integrator (`heun`/`rk4`) — transient tiers only. |
| `raceline` | `{"generator": "min_curvature"}` | Racing-line source; alternatively `{"file": "raceline.csv"}` (exactly one of the two). |
| `schema` | — | Schema version string, e.g. `"sim/1.0"`. |

`conditions=` fields and defaults:

| Field | Default | Meaning |
|---|---|---|
| `air.pressure_hpa` | `1013.25` | Absolute air pressure, hPa (drives air density). |
| `air.temperature_c` | `20.0` | Air temperature, °C. |
| `ambient_c` | `20.0` | Thermal-model ambient / pre-radiator coolant proxy, °C. |
| `track_surface_c` | `20.0` | Track-surface temperature (tire thermal boundary), °C. |
| `wind.speed_mps` | `0.0` | Wind speed, m/s (constant in v1). |
| `wind.direction_deg` | `0.0` | Meteorological direction the wind blows *from*, degrees (0 = North, 90 = East). |
| `schema` | — | e.g. `"conditions/1.0"`. |

### 10.5 The xarray Dataset contract

Every solved lap crosses the Python boundary as an `xarray.Dataset` — labelled dimensions, coordinates, per-variable units, and provenance attributes. This is a semver-style contract: additions are backward-compatible, and code written against an `s`-only `t0` dataset keeps working because the richer channels are strictly additive and only appear when the solve produced them.

**Dimensions and coordinates:**

| Coord | dtype | Attrs | Present |
|---|---|---|---|
| `s` | `float64` | `units: "m"`, `long_name: "arc length"` | always |
| `wheel` | `<U2`, values `FL FR RL RR` | `long_name: "wheel (FL, FR, RL, RR)"` | only when per-wheel variables exist (t1) |

**Data variables** — the definitive table (name → dims, units, long name, and which solves produce it):

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

So a `t0` lap of a car *without* a coupled electrified stack is genuinely `s`-only (7 variables); a `t0` lap of the shipped Model 3 additionally carries the two slow-state variables (9 variables, still no `wheel` dimension); a `t1` lap of the Model 3 carries all 16.

**Attributes** — all of them:

| Attr | Type | Meaning |
|---|---|---|
| `lap_time_s` | `float` | Total lap time, s. |
| `resolved_hash` | `str` | blake3 hex hash of the resolved vehicle spec — your reproducibility key. |
| `tier` | `str` | `"t0"` or `"t1"` — the tier that actually ran. |
| `fz_coupling` | `str` | `"one_step_lag"` or `"fixed_point"`. |
| `flat_track` | `int` | 0/1 (an int because netCDF attrs have no bool type). |
| `notes` | `tuple[str, ...]` | Every simplification/fallback that touched this lap (a tuple, not a list, to stay netCDF-serializable). |

Real output for the default t1 Model 3 lap at Catalunya (`ds_m=2.0` → 2325 stations), trimmed:

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

Always read `attrs["notes"]`. That lap carries 11 entries — among them `aero map 'aero/none.parquet' not present — constant-aero fallback carries the lap` (this vehicle ships no ride-height aero map, so constant coefficients were used) and `μ derived from MF6.1 pure-slip peak @ FNOMIN, p_cold ...; braking is friction-limited only at T0`. The policy is *nothing silent*: every estimate, fallback, and simplification that shaped the number in `lap_time_s` is listed right next to it.

Because the result is a standard xarray Dataset, the whole scientific-Python toolchain applies directly: `ds.v.plot()`, `ds.sel(wheel="FL")`, `ds.to_netcdf("lap.nc")`, `ds.where(ds.ay > 5)`, and so on.

### 10.6 The envelope object

`lap.envelope` returns the g-g-g-v envelope the lap ran on — the precomputed boundary of accelerations the tires can sustain as a function of speed `v`, longitudinal acceleration `a_x`, and the local "normal g" `g_normal` (how hard the road pushes on the car — more than $g$ in a banked or compressive section, less over a crest). Chapter 8 develops the theory; here is the query API:

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

The middle axis is `â_x` — longitudinal acceleration *normalised to ±1* against each point's straight-line capability, which is why `domain()[1]` is always `[-1.0, 1.0]`. Real numbers for the Model 3 on the default 40×25×7 grid:

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

`env.notes` carries three generation notes, including the crucial scope statement: *"envelope = tyre-force limit only (powertrain ceiling applied separately by the lap solver); ... boundary = the T1 trim's friction feasibility limit (not filtered for open-loop stability — a T2+ concern)."* There is currently no `to_dataset` helper for envelopes in `outlap.core`; the notebooks build their own grids from these scalar queries.

**The envelope cache.** Envelope generation is the expensive cold step, so the extension keeps a process-level cache keyed by everything that changes the boundary: the resolved vehicle hash, a fingerprint of every loaded binary sidecar's bytes, the session conditions, the envelope grid, and the `fz_coupling` mode. `flat_track` is deliberately *not* in the key (it only reshapes the path, not the boundary). Practical consequences, all measured: a second lap of the same car in the same process is near-instant (0.12 s after a 65 s debug-build cold solve, envelope reused); solving the raceline instead of the centerline also reuses it; but changing `conditions`, an `override`, the grid, or `fz_coupling` regenerates. The cache is never evicted — a Python session is assumed short-lived.

### 10.7 Errors: quick reference

All messages below were produced verbatim by the shipped package.

| You do | You get |
|---|---|
| Load a missing vehicle/track/tire file | `FileNotFoundError: source not found: track.yaml` |
| `solve_lap(..., tier="t2")` | `` ValueError: solver tier `t2` is not implemented yet (the transient tiers arrive in milestone M4); select tier `t0` (point-mass on the g-g-g-v envelope) or `t1` (full QSS with per-wheel outputs) `` |
| `overrides={"chassis.masss_kg": 1500.0}` | `` ValueError: unknown field `masss_kg` `` + `` help: did you mean `mass_kg`? `` |
| `conditions={"air": {"temp_c": 35.0}}` | `` ValueError: unknown conditions field `air.temp_c` (known fields here: ["pressure_hpa", "temperature_c"]) `` |
| `sim={"envelop": {}}` | `` ValueError: unknown sim field `sim.envelop` (known fields here: ["allow_degraded", "dt_s", "envelope", "flat_track", "fz_coupling", "integrator", "raceline", "schema", "tier"]) `` |
| `ds_m=-1.0` (any sampling function) | `ValueError: ds_m must be a positive, finite number of metres, got -1` |
| Unsupported override value type (e.g. a numpy array) | `ValueError: unsupported value type in overrides: ndarray` |
| Mismatched array lengths in `Tyre.forces` | `ValueError: length mismatch: kappa has 5 elements, alpha has 3` |

The general rule: *missing optional* inputs (`sim.yaml`, `conditions.yaml`, sidecar tables, battery/thermal refs) are never errors — they fall back with a recorded note. *Present but broken* inputs are always errors. And config errors are treated as a product surface: the `ValueError` text preserves the diagnostic help line from the Rust pipeline, so typos come back with suggestions instead of a bare parser trace.

### 10.8 `outlap.schemas` — validating documents against the contracts

The JSON Schemas in `schemas/` (Apache-2.0) are generated from the Rust `schemars` types — Rust is the single source of truth, and Python only *conforms* (Chapter 5). `outlap.schemas` loads those committed schemas and validates YAML documents against them with `jsonschema`.

Public functions:

- `load_schema(name: str) -> dict[str, Any]` — load a committed schema by document name; `load_schema("vehicle")` reads `schemas/vehicle.json`. Raises `FileNotFoundError` for an unknown name.
- `check() -> int` — validate every committed schema (each must itself be a valid JSON Schema draft 2020-12 document), the shipped fixtures under `crates/outlap-schema/tests/fixtures/`, and every `*.tyr.yaml` under `data/` (globbed, so new datasets are covered automatically). Returns a process exit code: 0 on success, 1 with a per-document error list on stderr otherwise.
- `main() -> int` — the CLI entry point; `--check` is the only flag and also the default behaviour.

```bash
$ python -m outlap.schemas --check
schema check OK: 8 schemas, 22 fixtures + 7 data files validated
```

The eight schemas are `battery`, `conditions`, `emotor`, `ptm`, `sim`, `track`, `tyr`, and `vehicle`. Two honest caveats: the paths are computed relative to the repository root from the module's own location, so the check works from a source checkout, not an installed wheel; and although `pydantic` is a declared dependency, the pydantic-v2 mirror of the schemas is a *planned later increment* — nothing in the package imports pydantic yet. Fixtures that rely on `extends:` merging are validated by the Rust pipeline instead, since a JSON Schema cannot express the merge.

```python
from outlap.schemas import load_schema
schema = load_schema("vehicle")
print(schema["title"])        # Vehicle
```

### 10.9 Other public modules

Three more packages ship inside `outlap`; Chapter 11, Importers and tooling, covers their workflows in depth — this is just the public-surface inventory.

**`outlap.tir`** — the `.tir` (Tire Property File) interchange codec, a pure-Python mirror of the Rust `outlap-schema` tir module. String-in/string-out parsing and writing plus `.tyr`-dict conversion; the writer is byte-compatible with the Rust writer (both are pinned by a shared canonical fixture). Exports: `SYNTHETIC_THERMAL`, `SYNTHETIC_WEAR`, `ThermalWearPolicy`, `TirDoc`, `TirEntry`, `TirError`, `TirSection`, `TirValue`, `format_number`, `parse_tir`, `tir_to_tyr`, `tyr_to_tir`, `write_tir`. CLI: `python -m outlap.tir {to-tyr, from-tyr}`.

**`outlap.tirefit`** — MF6.1 tire fitting: test-data ingestion (`load_csv`, `load_dat`, `load_ttc_mat`, `TireTestData`, `SweepBin`, `bin_sweeps`), a clean-room numpy forward model (`forces`, `Forces`, `DEFAULTS`, `params_from_coeffs`, `params_from_tyr`), a staged fit (`staged_fit`, `FitConfig`, `FitResult`, `StageReport`, `synthesize` — requires scipy via the `tire-fit` extra), and reporting (`render_markdown`, `report_dict`, `write_report`). CLI: `python -m outlap.tirefit {fit, synth}`. Its redistribution policy is loud and non-negotiable: parsers yes, **redistribution of FSAE TTC data or TTC-derived parameter sets, no** — that data is membership-locked.

**`outlap.importers`** — one-time local vendoring tools, never exercised in CI: `outlap.importers.pdt_h5` (PDT HDF5 → `.ptm`/battery YAML; exports `PdtImportError`, `convert_batterypack`, `convert_driveunit`, `convert_edrive`, `validate_battery_doc`; CLI `python -m outlap.importers.pdt_h5 {edrive, driveunit, batterypack}`) and the `osm_track` / `tumftm_track` track importers (need the `track-import` extra; the TUMFTM source data is LGPL-3.0).

Finally, packaging facts for orientation: the package requires Python ≥ 3.12; the compiled `outlap_core` extension is a maturin-built abi3 wheel declared as a path dependency (`crates/outlap-py`), so `uv sync` in `python/` compiles it automatically given a Rust toolchain. Optional extras are `track-import` (requests, scipy, matplotlib) and `tire-fit` (scipy); the `notebooks` dependency group adds everything the notebook tour needs. The `outlap` console script currently maps to the hello-world stub — the real entry points are the module CLIs listed above.


---

## 11. Importers and tooling

*What you will learn: every command-line tool that ships with outlap v0.2.0 — the PDT powertrain importer, the two track importers, the tyre-file codec and fitting pipeline, the data generators, and the schema and golden-file tooling — with the exact invocations, the files each one reads and writes, and the data-hygiene ("firewall") rules you must respect when feeding real data into any of them.*

### 11.1 The tooling landscape (and what does not exist yet)

outlap's tools are Python *module CLIs*: you run them as `python -m <module>` from the `python/` directory of a repository checkout, typically through uv (`cd python && uv run python -m ...`). There is deliberately **no unified `outlap` command yet**. Two things a new user will trip over:

- Installing the Python package puts an `outlap` script on your path, but it is a placeholder — `outlap.main()` in `python/src/outlap/__init__.py` just prints `Hello from outlap!`. Ignore it.
- Error messages occasionally hint at `outlap migrate` (for schema-version mismatches). That verb is part of the planned unified Rust CLI (Locked Decision #19, an M7 deliverable); today the hint tells you *what* will fix the file, not a command you can run. The PDT importer docstring says the same thing explicitly: the module CLI "mirrors the future Rust `outlap import pdt-*` 1:1".

The complete tooling surface at v0.2.0:

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

`gen_schemas` is the only compiled binary in the whole Cargo workspace; everything else here is Python.

### 11.2 The PDT HDF5 importer: `outlap.importers.pdt_h5`

PDT is a proprietary powertrain-design tool whose "stage files" describe an electric machine (**EDrive**), a geared motor-inverter-gearbox assembly (**DriveUnit**), or a **BatteryPack**, stored as HDF5 — a hierarchical binary container format read in Python with the `h5py` library. outlap never models these components internally (hard rule #1, the *firewall*, see Chapter 6, Architecture); instead this importer converts a stage file into outlap's own open formats: a `.ptm` map file (the powertrain map format, Chapter 5, Files and formats) plus a parquet *sidecar* — a columnar data file holding the big numeric tables next to the small YAML descriptor.

The importer package (`python/src/outlap/importers/pdt_h5/`) is a pure-Python adapter: `h5py` + `numpy` + `pyarrow` only, and it never imports PDT's own code. Usage, from the module docstring:

```bash
python -m outlap.importers.pdt_h5 edrive      <file.h5> -o machine.ptm.yaml [--vdc 400]
python -m outlap.importers.pdt_h5 driveunit   <file.h5> -o du.ptm.yaml      [--vdc 48] [--mass-kg X]
python -m outlap.importers.pdt_h5 batterypack <file.h5> -o battery.yaml
```

The full flag matrix (from `python/src/outlap/importers/pdt_h5/__main__.py`):

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

On success it prints `wrote <out>` plus a summary (grid sizes, `nan_fraction`, thermal-fit RMS, gear ratio, cell topology — whichever apply); warnings go to stderr; any import error exits with code 1 and a one-line `error: ...`.

#### `--vdc` and the full-voltage-stack default

PDT efficiency maps are gridded over DC-link voltage (*Vdc* — the voltage the battery presents to the inverter). The importer's behaviour is deliberately asymmetric:

- **No `--vdc`, multi-voltage grid** (the default): the importer emits the **full Vdc stack** — a `ptm/1.1` document with a third `vdc_v` axis. This is what the Vdc–SoC coupling wants: at run time the Rust core evaluates the map at the pack's state-of-charge-dependent terminal voltage (Chapter 9, Physics III).
- **`--vdc <V>` given**: the importer picks the **single nearest grid slice** (no cross-voltage interpolation — the thermal envelopes in the file are single-voltage) and emits a legacy single-voltage `ptm/1.0` map. If your requested voltage is more than **2 %** off the grid, you get a warning: `requested vdc X V snapped to grid Y V` (`common.py`, `select_vdc`).
- **No `--vdc`, and the file's thermal data names no voltage either**: the importer defaults to the **maximum** grid voltage and warns about it.

#### What each subcommand emits

**`edrive`** (electric machine + inverter) writes up to three files:

1. `machine.ptm.yaml` — `kind: electric_machine`, `schema: ptm/1.1` when a Vdc stack was emitted, else `ptm/1.0`. The `limits:` block carries peak/continuous torque-vs-speed curves, overload curves for 10/20/30 s holds, and drag torque. Provenance lands in `meta.source` ("PDT EDrive `<alias>` `<git hash>`").
2. `<out>.maps.parquet` — the efficiency/loss sidecar. Long/tidy float64 columns `speed_rpm, torque_nm, efficiency, loss_w`, plus `vdc_v` for a stack (one row per grid cell, `NaN` where a cell is beyond that voltage's feasible envelope; the speed axis is in RPM because file formats are a display boundary — everything is converted to rad/s inside the solver).
3. `<out>.emotor.yaml` — a thermal model of the machine (skipped with `--no-emotor`; see below).

Two physics decisions worth knowing (module docstring of `edrive.py`): the system efficiency is rebuilt as `motor_efficiency · inverter_efficiency` and the system loss as `motor_loss_total + inverter_loss_total` — the real files carry the two stages separately, not a lumped table — and the torque coordinate is `airgap_torque`. The importer also *never* trusts the file's `performance`/`metrics` summary scalars; power is always rebuilt as torque times angular speed, $P = \tau\,\omega$, because the real files mix W and kW in adjacent summary fields.

The raw maps sit on a "load ratio" axis, so the importer inverts each speed row onto a regular torque grid (`--torque-points` nodes, exact zero node, asymmetric drive/regen bounds), masks cells beyond the per-speed peak torque as `NaN`, and keeps the zero-torque column at efficiency 0 (the "spin point"). The `nan_fraction` in the summary tells you how much of the rectangle is masked.

**`driveunit`** (motor + inverter + gearbox as one unit) writes `du.ptm.yaml` (`kind: drive_unit`) + the maps sidecar, with the same Vdc-stack logic. Differences: the map is at the **output shaft** (gear ratio already applied, recorded as `upstream_ratio_applied: true`; the ratio itself only appears in `meta.source`), drag torque comes from the no-load test resampled onto the map's speed axis, and there is no `.emotor.yaml` (drive-unit thermal data is envelope-only). Mass resolution tries four dataset names in order and finally your `--mass-kg` override — the Rust loader requires `mass_kg > 0`, so a file without a mass group and no override is a hard error. The importer also absorbs two real PDT export quirks: the drive-unit thermal group is spelled with a capital T (`Thermal`), and node names can arrive doubly-bytes-encoded (`b"b'ambient'"`).

**`batterypack`** writes `battery.yaml` (`schema: battery/1.0`, `model: rc_pairs` — a one-RC-pair Thevenin *equivalent-circuit model*, i.e. an open-circuit voltage source behind resistances; see Chapter 9) plus `<out>.tables.parquet` with the cell-level columns `soc, temp_c, ocv_v, r0_ohm, r1_ohm, tau1_s, dudt_v_per_k` on the (state-of-charge, temperature) grid. The YAML carries the pack topology (`ns` series × `np` parallel), capacity, SoC window, power limits vs SoC, and a lumped thermal block. (Three mentions inside `battery.py` still call the format "provisional" — stale wording: the Rust `BatteryDoc` type and `schemas/battery.json` exist and are enforced.)

Every emitted document is validated against the committed JSON Schema (`schemas/ptm.json`, `emotor.json`, `battery.json`) before it is written — a failed validation is an import error, not a bad file on disk. One practical caveat: that validation locates `schemas/` by walking up from the module file (`Path(__file__).resolve().parents[5]`), so the importer assumes a **repository checkout**; a bare pip-installed wheel outside the repo will not find the schemas.

#### The thermal outputs: 2-node fit or detailed network

When an EDrive file carries the full lumped-parameter thermal network (*LPTN* — a graph of heat capacities connected by thermal conductances) the importer takes the **detailed** path (`thermal_network.py`): it collapses the ~20-node PDT network onto outlap's reduced node menu — `winding / stator_iron / rotor / housing / coolant / ambient` — by summing capacities and inter-group conductances, routes the per-component loss maps onto those groups, and rebuilds the convection paths (air-gap film, liquid cooling jacket) from clean scalar geometry fields — never from the FEA mesh. Those convection correlations are the one deliberate, narrow reversal of the firewall (Locked Decision #25): they were ported into `outlap-thermal` as author-owned open-source physics (Churchill–Chu and Gnielinski-type film correlations; Chapter 9 covers the model).

When the file has only thermal *envelopes* (continuous and overload torque curves), the importer distills a **lumped 2-node model** instead (`thermal_fit.py`). The model is a winding node and a case node:

$$C_w \dot T_w = s_w P\,k_{cu}(T_w) - G_{wc}\,(T_w - T_c)$$

$$C_c \dot T_c = (1 - s_w)\,P + G_{wc}\,(T_w - T_c) - G_{cool}\,(T_c - T_{cool})$$

where $T_w, T_c$ are the winding and case temperatures, $C_w, C_c$ their heat capacities (J/K), $G_{wc}, G_{cool}$ the winding-to-case and case-to-coolant conductances (W/K), $P$ the loss power, $s_w$ the fraction of loss deposited in the winding, $T_{cool}$ the coolant temperature, and $k_{cu}(T_w) = 1 + \alpha\,(T_w - T_{ref})$ the copper resistance-rise feedback (disable with `--no-copper-feedback`). The four parameters $(C_w, C_c, G_{wc}, G_{cool})$ are least-squares fitted so the 2-node network, driven by the exported loss map, reproduces the PDT continuous envelope and the 10/20/30 s overload torques at a handful of speeds. Because the firewall dependency rule forbids scipy here, the fit is numpy-only: the model is linear time-invariant, so steady state and transients are closed-form 2×2 matrix algebra, and the optimiser is a hand-rolled deterministic Nelder–Mead in log space. The fit quality is printed as `fit_rms` and recorded in the emitted file's `meta.notes`.

Both paths emit `schema: emotor/1.1` documents consumed by the same machine-thermal solver (Chapter 9).

### 11.3 The firewall: rules you must respect

If you feed real proprietary data into these tools, the repository's hygiene rules apply to *you*, not just to the maintainers:

1. **PDT files are read as raw HDF5 with h5py only.** Never import PDT's own code, and never commit a real `.h5` stage file — they are private data. The CI test suite uses tiny **synthetic** PDT-shaped fixtures generated at test time (`python/tests/pdt_fixtures.py`), never real files.
2. **Never commit anything the importer writes from real data** — the `.ptm.yaml`, the parquet sidecars, the battery YAML are all "real-data-derived artifacts". The supported workflow is to import into a `local/` directory under your vehicle, e.g. `data/vehicles/<car>/local/`: the repo `.gitignore` blocks `data/vehicles/*/local/` (and the real-data notebook twin `notebooks/07_qss_t1_local.ipynb`), so a real import physically cannot be committed by accident.
3. **FSAE TTC tyre data is membership-locked**: keep raw files in the git-ignored `ttc-data/` directory, and never publish TTC files *or parameter sets fitted from them* (`python/src/outlap/tirefit/README.md`: "Parsers yes — redistribution of TTC data or TTC-derived parameter sets, NO").
4. **Reference books and papers stay out of the repo** (`**/*.pdf` is git-ignored). Coefficient *values* are citable facts; the source documents are not redistributed.
5. **Track importers are one-time local vendoring tools** — they read only public or redistributable data and never run in CI.

### 11.4 Track importers

Both track importers emit the same two files into a track directory: `track.yaml` (the descriptor, `schema: track/1.0`) and `centerline.csv` with the 8 columns `s_m, x_m, y_m, z_m, banking_deg, width_left_m, width_right_m, grip_scale` — arc length, position (ISO 8855: x forward, y left, z up), banking, corridor half-widths to the left and right of the centreline, and a local grip multiplier. Chapter 5 documents the format; Chapter 12 below lists what has already been imported for you.

#### `osm_track` — OpenStreetMap + elevation (the 3D importer)

Since no open **3D** circuit data exists, this importer builds it from public sources (`python/src/outlap/importers/osm_track.py`): (1) the centreline from OpenStreetMap `highway=raceway` ways (ODbL-licensed), assembled into the longest ordered polyline and projected to a local metric frame; (2) elevation from an open *DEM* (digital elevation model — a public grid of ground heights) via the free opentopodata API, trying the 25 m European dataset `eudem25m` first, then the global `srtm30m`, smoothed with a cubic smoothing spline so the second derivative of z (needed for vertical curvature) is continuous; (3) banking left at zero — coarse public DEMs cannot resolve it; you refine it later with sparse `banking_keypoints` in `track.yaml`.

```bash
cd python
uv run python -m outlap.importers.osm_track --preset catalunya --out ../data/tracks/catalunya_osm
# or an arbitrary circuit:
uv run python -m outlap.importers.osm_track --name "My Circuit" --lat 41.57 --lon 2.26 \
    [--radius 2500] [--ds 3.0] [--no-dem] --out <dir>
```

| Flag | Default | Meaning |
|---|---|---|
| `--preset` | — | one of `catalunya`, `spa`, `silverstone` (Decision #23) |
| `--name/--lat/--lon` | — | ad-hoc circuit (either a preset or all three are required) |
| `--radius` | 2500 | OSM search radius in metres (ad-hoc path only; presets bake their own) |
| `--ds` | 3.0 | resample spacing, metres |
| `--no-dem` | off | skip elevation (flat track) |
| `--out` | required | output directory |

The emitted `track.yaml` records `meta.source: osm+dem` (or `osm` with `--no-dem`), `meta.accuracy_class: B` with elevation, `C` without, and the attribution string "© OpenStreetMap contributors (ODbL); elevation `<dataset>` via opentopodata.org". Corridor widths are **defaulted** to a 6 m half-width each side (OSM does not carry them), which the meta `notes` admit. The importer needs the `track-import` extra (`uv sync --extra track-import` → requests, scipy, matplotlib), is polite to the public APIs (descriptive User-Agent, three Overpass mirrors, ~1 request/s DEM throttling with back-off), and **never runs in CI**.

#### `tumftm_track` — the TUMFTM racetrack-database (the flat importer)

This converts the TU München racetrack-database — 25 circuit centrelines with **satellite-measured** corridor widths on a uniform ≈5 m grid, LGPL-3.0-licensed, the standard academic dataset — into outlap's format (`python/src/outlap/importers/tumftm_track.py`). Pinned to upstream commit `e59595d1f3573b30d1ded6a08984935b957688e0`:

```bash
git clone https://github.com/TUMFTM/racetrack-database.git /tmp/tumftm
git -C /tmp/tumftm checkout e59595d
cd python
uv run python -m outlap.importers.tumftm_track --input /tmp/tumftm/tracks --out ../data/tracks
```

`--input` takes one CSV or a directory of them; each track is written to `<out>/<name>/`; `--ds <m>` resamples, and its default (`None`) passes the native ≈5 m grid through **unchanged** — exact, with no interpolation of the measured widths. Malformed files are skipped with a message while the rest convert, but the exit code is then 1, so check stderr before trusting a batch run in a script.

Three correctness points the module itself documents — the first is the classic trap:

1. **Widths are mapped by NAME, never by column position.** The source column order is RIGHT before LEFT (`# x_m,y_m,w_tr_right_m,w_tr_left_m`); outlap's is LEFT before RIGHT (ISO 8855: road +y is *left*). A positional mapping would silently swap the corridor and flip the computed racing line.
2. **The data is strictly 2-D**: `z_m` and `banking_deg` are emitted as 0 and `grip_scale` as 1 — legitimate (the source has no elevation), recorded in `meta.notes` and in accuracy class `C`.
3. **Closure**: the source loop is left open (~one sample short of the start); outlap's track loader closes it over the connecting chord.

A 25-entry table maps source file stems to directory and display names (`Nuerburgring` → `nuerburgring` / "Nürburgring GP"); unknown stems fall back to a snake_case slug. This importer is pure stdlib + numpy — no extra needed.

### 11.5 Tyre tooling: the `.tir` codec and `tirefit`

Two module CLIs deal with tyre data (the MF6.1 Magic Formula model itself is Chapter 7, Physics I).

**The `.tir` codec** converts between the industry `.tir` property file format and outlap's `.tyr.yaml` document:

```bash
python -m outlap.tir to-tyr   <in.tir>      -o out.tyr.yaml [--thermal-wear synthetic|from-donor|none] [--donor donor.tyr.yaml]
python -m outlap.tir from-tyr <in.tyr.yaml> -o out.tir
```

A `.tir` file cannot carry outlap's `thermal:`/`wear:` blocks, so `--thermal-wear` sets the policy for filling them: `synthetic` (default — clearly-labelled placeholder blocks), `from-donor` (copy from another `.tyr.yaml` via `--donor`), or `none`. The Python `.tir` writer is byte-for-byte compatible with the Rust writer — both are pinned to a shared canonical fixture, with CPython's `repr` float formatting matching Rust's `ryu` exactly.

**The MF6.1 fitting pipeline** turns measured tyre test data into a `.tyr.yaml`:

```bash
python -m outlap.tirefit fit   <data...> --unloaded-radius R0 -o out.tyr.yaml [--report-dir DIR] \
                               [--fnomin N] [--nompres PA] [--longvl MPS]
python -m outlap.tirefit synth <in.tyr.yaml> -o out.csv [--seed 0] [--noise 0.01]
```

`fit` reads one or more test files — TTC `.mat` (v7/v7.3), `.dat`, or `.csv` — concatenates them, and runs a staged least-squares fit (nominals → pure longitudinal → pure lateral → combined slip → aligning moment → overturning/rolling moments), printing per-stage RMS errors and writing a `tyr/1.0` document with synthetic thermal/wear placeholders and `synthetic: true` provenance. `--longvl` (the reference speed) defaults to 16.7 m/s; `--report-dir` adds `report.json` + `report.md`. The fit stages need scipy: `uv sync --extra tire-fit`. `synth` is the inverse — a deterministic, seeded synthetic dataset generated from an existing `.tyr`, written as a faithful SAE-signed TTC-format mock so the synth→fit round trip and real measured data share one sign convention. It is both the recovery-test harness and the way to exercise the pipeline without membership-locked data. Remember firewall rule 3: fitted TTC parameter sets never leave your machine.

### 11.6 Generators and figure renderers in `python/tools/`

Two generators author committed synthetic data (both run from anywhere; they anchor paths off their own file location):

- **`gen_f1_aero.py`** writes `data/vehicles/f1_2026/aero/f1_2026.parquet` — the ride-height/yaw/DRS aero map of the F1 reference car: a 5×5×5×2 grid over `ride_height_f_mm, ride_height_r_mm, yaw_deg, drs_flag` with value columns `cz_front_a_m2, cz_rear_a_m2, cx_a_m2`. It is **anchored** so that at the reference ride heights (30 mm front / 70 mm rear) the map reproduces the car's constant-aero fallback exactly (1.9 / 2.6 / 1.25 m²) — asserted at generation time — and every grid-aligned fibre is monotone or single-peaked, safe for the one shared monotone cubic Hermite interpolant (Fritsch–Carlson; Decision #30). All sensitivities are estimated; the file is synthetic and says so.
- **`gen_model3_powertrain.py`** writes the Tesla Model 3 study's entire committed powertrain (Section 12.2): three Vdc-stacked `ptm/1.1` drive-unit maps + sidecars (`ptm/du_{small,medium,large}.*`) and the synthetic 800 V-class pack (`battery/pack_800v.*`). Its design choices are deliberate teaching devices: efficiency/loss pairs are emitted consistently so energy closure holds exactly at grid nodes (drive loss $= P_{mech}(1/\eta - 1)$, regen loss $= |P_{mech}|(1 - \eta)$); efficiency is *linear* in Vdc so the shared interpolant reproduces and extrapolates that axis exactly; and the 730/790/850 V grid is deliberately narrower than the pack's voltage swing so a low-SoC lap exercises the documented below-grid extrapolation.

Seven `plot_*.py` scripts render the documentation figures (`docs/theory/img/`, `docs/validation/img/`, `docs/vehicles/model3/img/`). Four of them shell out to committed Rust examples (`cargo run -p outlap-qss --example battery_coupling | ggv_traces | thermal_traces | limebeer_lap`) and plot the CSV those examples print — so every theory figure is driven by the actual model, not a re-implementation. `plot_model3.py` instead drives the public Python API (`vehicle_report`, `solve_lap_dataset`, `min_curvature`; Chapter 10). None of these run in CI.

Two more fixture generators live inside the Rust crate tree (`crates/outlap-schema/tests/fixtures/gen_ptm_maps.py`, `gen_gridmap_fixture.py`); they write the synthetic parquet fixtures the Rust sidecar-decoder tests consume.

### 11.7 `gen_schemas` — the schema source of truth

The Rust `schemars` types are the single source of truth for the file formats (Decision #34). The only binary in the workspace regenerates the published JSON Schemas from them:

```bash
cargo run -p outlap-schema --bin gen_schemas            # writes schemas/*.json
cargo run -p outlap-schema --bin gen_schemas -- --check # regenerates in memory, diffs, fails on drift
```

It emits exactly eight documents — `schemas/{vehicle,ptm,tyr,emotor,battery,track,conditions,sim}.json` — and the `--check` form is a CI gate, so the committed schemas can never drift from the Rust types. These are the very schemas the PDT importer validates its output against (Section 11.2), which ties the Python emit side to the Rust truth. A companion Python check, `python -m outlap.schemas --check`, validates the shipped YAML data and fixtures against the committed schemas and is also wired into CI.

### 11.8 `tools/goldens/` — regenerating the MF6.1 oracle CSVs

The tyre model's numerical oracle is a set of committed golden CSVs (`crates/outlap-tire/tests/golden/pacejka_2006_205_60r15/`) that both the Rust kernels and the Python forward model must reproduce to ≤0.5 % (Chapter 13, Validation). They were generated by running **teasit/magic-formula-tyre-library** (GPL-3.0) under GNU Octave as an external tool — only its numeric outputs are captured, never its source; outlap's MF6.1 implementation is derived from Pacejka (2012) alone. To regenerate:

```bash
MF_ORACLE_SRC=/path/to/magic-formula-tyre-library/src ./tools/goldens/run.sh
```

Requirements: GNU Octave ≥ 8 on PATH and a local checkout of the oracle (never committed). The script stages a package-only copy of the oracle, records its commit and licence into each CSV's provenance header (asserted by a test), and writes four CSVs — `fx0.csv`, `fy0_mz.csv`, `combined.csv`, and `combined_camber.csv` (the κ×α sweep at γ = ±4° camber) — in SI units with ISO 8855 signs. Governance is strict: this **never runs in CI** (CI compares against the committed CSVs), there is deliberately no in-tree bless mechanism for external-oracle data, and regeneration is allowed only in a PR that updates the version pins and states the physics or tooling reason.

---

## 12. The shipped data library

*What you will learn: everything under `data/` — three reference vehicles, three citation-backed tyre sets, and 26 circuits — and, for each asset, what it is for, where every number comes from, and how much to trust it. You will also learn the licence obligations that travel with the track data if you redistribute it.*

### 12.1 How the library is organized

All shipped data is licensed **CC-BY-SA-4.0** (SPDX headers on every file), distinct from the AGPL-3.0-only code and the Apache-2.0 `schemas/`. The library's honesty contract is Locked Decision #15: reference data is *synthetic or transcribed, never measured*, with plausible magnitudes **clearly labelled at their source** — and Decision #41 backs it at run time: every estimated or defaulted value surfaces in the loaded-model report (`outlap.vehicle_report(...)`, Chapter 10), nothing silent. Each vehicle directory is self-contained: a `vehicle.yaml` whose referenced `.ptm`/`.tyr`/battery/emotor files live in sibling subdirectories, loadable as one unit (Chapter 4, The input quartet).

Three trust levels recur below, so name them once: **spec** (a published manufacturer or paper value, cited), **estimated** (a documented heuristic, flagged in the loaded-model report), and **synthetic** (an invented smooth surface from a committed generator script — reproducible, but not data about any real machine).

### 12.2 Vehicles (`data/vehicles/`)

Three vehicles ship at v0.2.0: `limebeer_2014_f1`, `f1_2026`, and `tesla_model3_rwd`.

#### `limebeer_2014_f1` — reference car #1, the validation car

This is the complete published F1 parameter set of Perantoni & Limebeer, "Optimal control for a Formula One car with variable parameters", *Vehicle System Dynamics* 52(5), 653–678, 2014 (Table 4 + §2), transcribed clean-room from the open-access manuscript (Oxford University Research Archive, `uuid:ce1a7106-0a2c-41af-8449-41541220809f`). Its whole reason to exist is the Catalunya cross-check against the paper's published optimal lap — 82.43 s on a 2 m grid, top speed ≈88 m/s (Chapter 13 tells the full gate story).

Per-coefficient provenance lives in `data/vehicles/limebeer_2014_f1/README.md`; the highlights:

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

It is the only vehicle that ships its own `conditions.yaml` (21.0 °C, 1013.25 hPa), chosen so outlap's ideal-gas conversion reproduces the paper's air density of exactly 1.2 kg/m³. The README also records a clean-room consultation: the MIT-licensed `fastest-lap` project was read as a *numerical cross-check only* — its transcription of Tables 3/4 matches verbatim, but its powertrain (735.5 kW + boost) is its own invention, so its lap times are not comparable; no code was taken.

#### `f1_2026` — the synthetic F1 2026 hybrid

A demo car, not a validation car: an ICE + MGU-K on one shaft, through an 8-speed gearbox (`ratios: [2.9, 2.2, 1.8, 1.5, 1.28, 1.1, 0.98, 0.86]`, `final_drive: 3.1`, 20 ms shifts) and a limited-slip differential to the rear axle. Mass 768 kg, wheelbase 3.40 m, track [1.65, 1.60] m. Everything is synthetic-but-plausible, and the file header says so twice — the ERS/energy figures (4.0 MJ store, [0.2, 0.9] SoC window, 350 kW deployment with a speed taper, 8.5 MJ per-lap harvest) are "approximate 2026-regulation values for testing", to be verified against the published FIA 2026 Technical Regulations before being treated as reference data.

Its aero is the interesting part. The `aero.constant` block (CxA 1.25 m², CzA 1.9 + 2.6 = 4.5 m² — about 1.7× car weight of downforce at 250 km/h, L/D ≈ 3.6) feeds the T0 tier, while T1 consumes the shipped `aero/f1_2026.parquet` ride-height/yaw/DRS map, generated by `gen_f1_aero.py` and **anchored** so the map reproduces those exact constants at the reference ride heights (Section 11.6). The constants themselves stand in for the same aero magnitudes as the Limebeer car (ClA 4.5 m² is PL2014's value) — that anchoring is what keeps the two F1 cars' behaviour comparable. Two honest gaps: `battery.params` references `battery/f1_es.yaml`, which **does not ship yet** (the ERS energy manager is the M6 milestone; at T0 the ERS is a power cap), and `tyr/slick.tyr.yaml` is a hand-authored synthetic slick "for schema/round-trip testing only".

#### `tesla_model3_rwd` — the Model 3 HV variant study (the M3 capstone car)

Read the caveat before quoting any number from this car: it is a production **Tesla Model 3 RWD identity re-imagined as an HV (800 V-class) variant**. The chassis, mass, and aero are Model-3-plausible; the powertrain is deliberately *not* the real ~360 V car but an 800 V-class drive-unit + pack stack, so that the Vdc–SoC coupling (Chapter 9) is live on a road car. Everything committed is **synthetic or estimated**: the drive-unit maps and pack tables are invented smooth surfaces written by `python/tools/gen_model3_powertrain.py`, never measured and never derived from any PDT export.

The spec-sheet anchors: curb mass 1765 kg, wheelbase 2.875 m, track 1.58 m, and CxA 0.51 m² (published Cd 0.23 × 2.22 m² frontal area). Everything else is a documented estimate — CG position from an assumed ≈47/53 weight distribution, CG height 0.45 m (floor-mounted pack), ride rates from ≈1.5 Hz ride frequencies, roll-centre heights for strut front / multi-link rear, brake balance 0.62, a 0.6 one-pedal regen-blend ceiling — and the anti-dive/anti-squat values are *omitted* on purpose so the load pipeline's estimator fills them and the loaded-model report shows it. The car loads warning-clean with every estimate noted (the capstone notebook counts 10 estimated entries). The tyre is a **documented proxy**: the published Pacejka (2006) 205/60R15 book tyre stands in for the real 235/45R18, because no public Magic Formula set exists for the OE tyre.

The committed powertrain is a three-way sizing study — the sensitivity axis of notebook `07_qss_t1`:

| Variant | Peak torque (output shaft) | ≈ Peak power | File |
|---|---|---|---|
| small | 1365 N·m | 100 kW | `ptm/du_small.ptm.yaml` |
| **medium (default)** | 2765 N·m | 203 kW | `ptm/du_medium.ptm.yaml` |
| large | 3381 N·m | 248 kW | `ptm/du_large.ptm.yaml` |

All three share a 700 rpm output-shaft base speed and a 730/790/850 V Vdc grid; the medium sizing sits at a production Model 3 RWD's ≈200 kW. The torque scales mirror the author's private drive-unit sizing sweep so this committed story and its untracked real-data twin tell the same tale, but the surfaces themselves are invented. The pack (`battery/pack_800v.battery.yaml`) is a synthetic 220-series/1-parallel, 92 Ah, 64.064 kWh Thevenin pack whose ≈634–810 V open-circuit range deliberately sags *below* the drive units' Vdc grid under low-SoC load — exercising the documented below-grid extrapolation on every deep-discharge lap. The machine-thermal model (`emotor/rear_du.emotor.yaml`) is a hand-authored six-node lumped network (winding / stator_iron / rotor / housing / coolant / ambient; winding limits 150/180 °C, rotor 140/170 °C), all values estimated from documented heuristics.

Swapping a sizing is a one-line what-if override (Decision #35), no file edits:

```python
solve_lap_dataset(vehicle_dir, line, tier="t1",
                  overrides={"drivetrain.units.0.source": "ptm/du_large.ptm.yaml"})
```

Finally, the firewall in practice: the vehicle README walks through importing the *real* PDT drive units and the real 704 V pack into the git-ignored `data/vehicles/tesla_model3_rwd/local/` directory with `python -m outlap.importers.pdt_h5 driveunit|batterypack`, then pointing the same car at them via overrides. Nothing under `local/` can be committed.

### 12.3 Tyres (`data/tires/`)

Three citation-backed `.tyr` datasets ship; coefficient *values* are transcribed facts with an exact citation in each dataset's `provenance` block and README, and the source documents are never redistributed. One shared caveat first: the `tyr` format requires `thermal:` and `wear:` blocks, but those physics models land in M5 and no published source provides them — so every dataset carries **synthetic, clearly-labelled placeholders** for those two blocks. `synthetic: false` on a dataset means the load-bearing force/moment coefficients are the published set; the placeholder blocks are the documented exception.

1. **`pacejka_2006_205_60r15/`** — the 205/60R15 91V passenger tyre of Pacejka, *Tyre and Vehicle Dynamics*, 2nd ed. (2006), Table A3.1: the book's worked-example car tyre, and outlap's MF6.1 **validation tyre** (the golden CSVs of Section 11.8 are computed for it). Being a 2nd-edition set it has no inflation-pressure terms, `Mx ≡ 0`, and rolling resistance via `qsy1` only. This same file is the Model 3's `tyr/road.tyr.yaml`.
2. **`roborace_devbot_mf52/`** — the Roborace DevBot "sport focused road tire" from TUMFTM's Open-Car-Dynamics (Apache-2.0, pinned commit `0a92c686`): an MF5.2 set mapped to MF6.1 with a per-coefficient conversion table in its README (camber `PHY3` folded into `PKY6`; no pressure model; `Mz ≡ Mx ≡ 0`).
3. **`limebeer_2014_f1/`** — an MF6.1 re-expression of PL2014's tyre model (Appendix A + Table 3): load-linear peak friction with a $\sin(Q \arctan(S\rho))$ shape. The transcription is exact where the paper is linear — `PDX1 = 1.575`, `PDX2 = -0.35` reproduce μx = 1.75 at 2 kN and 1.40 at 6 kN *exactly* (likewise `PDY1 = 1.625` for 1.80 → 1.45), and `PCX1 = PCY1 = 1.9` is the paper's shape factor Q — while the stiffness terms (`PKX*`, `PKY*`) were fitted numerically so the MF6.1 peaks sit where the paper's formula *actually* peaks. That last clause matters: the README documents a PL2014 self-inconsistency — the stated peak slips (0.11/0.10, 9°/8°) disagree with the paper's own formula, which peaks at 0.756× those values — and anchors to the formula, since the validation target is the paper's simulation. No aligning moment, camber, or pressure sensitivity, matching the paper. No third-party source code was consulted; `fastest-lap` was read as a numerical cross-check only.

One stale-doc warning: the table in `data/tires/README.md` still lists only the first two datasets and calls an F1 reference tyre "deferred" — the `limebeer_2014_f1/` directory ships regardless (it arrived with the M3 cross-check) and carries its own full provenance README. Trust the dataset directories over the index table.

Every `.tyr.yaml` under `data/` is schema-checked in CI (`python -m outlap.schemas --check`), and `crates/outlap-tire/tests/reference.rs` globs every dataset, asserting a warning-free load, a numerically-exact `.tir` codec round trip, and per-tyre physics checks.

### 12.4 Tracks (`data/tracks/`) — 26 circuits, two provenances

26 track directories ship: 25 flat circuits vendored from the TUMFTM racetrack-database (LGPL-3.0) plus one 3D import, `catalunya_osm`. The "length" column below is the arc length $s$ at the last centreline sample; the loader closes each loop over the final chord, so the lap length is a few metres more.

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
| `spielberg` | Red Bull Ring | 4310 | TUMFTM | C |
| `suzuka` | Suzuka Circuit | 5798 | TUMFTM | C |
| `yas_marina` | Yas Marina Circuit | 5542 | TUMFTM | C |
| `zandvoort` | Circuit Zandvoort | 4311 | TUMFTM | C |

The **accuracy class** in each `track.yaml` meta is the trust label: class **C** means a flat 2-D centreline (`z = 0`, `banking_deg = 0`, `grip_scale = 1`) — legitimate, nothing fabricated, but these tracks exercise no grade, vertical-curvature, or banking physics. Class **B** means real elevation from a public DEM with defaulted widths and unresolved banking (adding hand-annotated `banking_keypoints` moves a track toward class A). Note the honest quirks: `ims` is the oval *geometry* only (its famous banking is not represented), `nuerburgring` is the ~5.14 km **GP-Strecke**, not the Nordschleife, and the TUMFTM set is frozen around 2021 (Yas Marina pre-2021, Zandvoort pre-2020 layouts).

**The two Catalunyas are the classic trap.** `catalunya_osm` is the 3D OSM+DEM import — the *reference* Catalunya used by every notebook, example lap, and the Perantoni & Limebeer cross-check (~30 m of elevation change). `catalunya` is the flat TUMFTM vendoring, a peer of the other 24. They are the same circuit from two sources; the validation work found the class-C smoothed geometry does not reproduce the paper's apex speeds (it rounds the slow chicane open and tightens the fast corners), which is why the cross-check stays on `catalunya_osm` (Chapter 13).

#### Licence obligations when redistributing

The 25 TUMFTM circuits are **LGPL-3.0 data**. outlap redistributes them legitimately by (a) shipping the upstream licence text verbatim as `data/tracks/LICENSE-tumftm-LGPL-3.0.txt` and (b) embedding the attribution string in every `track.yaml`: *"Centerline © TU München, Institute of Automotive Technology (TUMFTM racetrack-database), LGPL-3.0"*, plus a `notes` field recording the upstream commit (`e59595d`). If you redistribute these tracks — in a fork, a product, or a derived dataset — you must carry both forward. `catalunya_osm` is ODbL: keep "© OpenStreetMap contributors (ODbL)" and the elevation credit ("elevation eudem25m via opentopodata.org"), and remember that an ODbL-derived database inherits ODbL terms.

### 12.5 `data/presets/` — reserved, currently empty

The directory exists but ships **no files** at v0.2.0. It is reserved for the class presets of Locked Decision #41 (`formula_base`, `gt_base`, `passenger_base` — starting points you would `extends:` from when authoring your own vehicle). Until they land, the practical way to start a new car is to copy the closest reference vehicle and edit it, keeping the provenance comments honest (Chapter 14, Recipes, walks through exactly that). Do not confuse this directory with the track importer's `--preset` flag (Section 11.4) — same word, unrelated feature.


---

## 13. Validation, testing, and trust

*What you will learn: why you should — and should not — believe the numbers outlap prints. This chapter walks through the one published cross-check the simulator is gated against, the two families of golden-file tests, the property tests that pin the physics sign-by-sign, the performance and determinism guarantees, and — just as important — an honest inventory of what is **not** yet validated at v0.2.0.*

A lap simulator is only as useful as it is trustworthy. outlap's answer to "why should I believe this?" has four parts: (1) it is cross-checked against an independent, published F1 result; (2) its outputs are frozen as golden files that fail CI if they drift; (3) its physics is pinned by property tests that assert invariants like "grip never exceeds the friction circle" and "energy in equals energy out"; and (4) every estimate or simplification is surfaced in the loaded-model report and the lap `notes` — nothing is silent. This chapter covers each in turn, and closes with the gaps.

### 13.1 The Limebeer cross-check: the one published oracle

The single most important validation asset is the **Perantoni & Limebeer 2014 cross-check**, documented in [`docs/validation/limebeer.md`](validation/limebeer.md). An *oracle* here means an independent, published result that outlap is compared against but did not produce — a yardstick from outside the project.

The oracle is G. Perantoni & D. J. N. Limebeer, *"Optimal control for a Formula One car with variable parameters"*, Vehicle System Dynamics **52**(5), 653–678 (2014) — an open-access paper that solves the *time-optimal* control problem for an F1 car around the Circuit de Barcelona-Catalunya and publishes the full car parameter set (Tables 3–4, Appendix A), an optimal lap of **82.43 s** (on a 2 m computational grid; 82.57 s in the mesh-asymptotic limit), and a speed trace (their Fig. 8) topping out near **88 m/s**. The `limebeer_2014_f1` reference vehicle (Chapter 12) is that car, transcribed clean-room from the manuscript.

#### 13.1.1 The configuration

The cross-check runs `limebeer_2014_f1` on the `catalunya_osm` minimum-curvature line, with two settings that make the comparison honest:

- **`sim.flat_track: true`** — PL2014 is a two-dimensional study, so this analysis mode zeroes the track's grade, banking, and vertical curvature and collapses the g-g-g-v envelope to a flat g-g (Chapter 8).
- The air density is pinned to $\rho = 1.2\ \mathrm{kg/m^3}$ (the paper's value) via the vehicle's own `conditions.yaml`, and the production $40\times25\times7$ envelope grid is used.

You can reproduce it locally:

```bash
cargo run --release -p outlap-qss --features parallel --example limebeer_lap
python python/tools/plot_limebeer.py
```

#### 13.1.2 The gates, and what actually passes

Here is the subtle and important part. The original milestone plan called for a "lap time within 1 %" gate. That gate was found to be **unattainable by construction** — a quasi-steady-state solver on a fixed heuristic racing line cannot match a transient solver that co-optimises the line and the speed for time. (The PL2014 paper *itself* cites a 2.19 s quasi-steady-versus-optimal-control gap at Barcelona.) So the lap-time gate was **re-scoped** (this is "Locked Decision #48"): the numbers that the committed track geometry can honestly support are gated; the lap time is *recorded with a decomposition* but not gated.

| Gate | outlap | PL2014 | Result |
|------|--------|--------|--------|
| Top speed ≤ 1 % | 87.8 m/s | ≈ 88 m/s | **PASS** (−0.2 %) |
| Slowest-corner apex ≤ 5 % | 17.7 m/s | 17 m/s | **PASS** (+4.1 %) |
| Fast-corner apexes ≤ 5 % | 59.1 / 60.8 m/s | 60 / 60 / 62 m/s | **PASS** (−1.5 % / −1.9 %) — *only on the paper's own geometry* |
| Lap time | 92.36 s (committed track) / 87.08 s (paper's geometry) | 82.43 s | **recorded, not gated** |

The top-speed and slowest-apex gates run in CI ([`python/tests/test_limebeer.py`](../python/tests/test_limebeer.py)) on the committed `catalunya_osm` import. The fast-corner band passes only on the paper's own digitised curvature — on the committed OSM import the fast corners are geometry-corrupted (interpolation noise throws up spurious curvature spikes, the widths are defaulted, and it is a later circuit layout than the paper's 2013 one), so the fast-corner gate is **deferred to milestone M4**, where it lands together with a time-weighted racing-line solver.

#### 13.1.3 Reading the lap-time gap honestly

The 92.36 s versus 82.43 s difference looks alarming until you decompose it, which `docs/validation/limebeer.md` does. It is *structural*, not a model error:

1. **Quasi-steady vs transient (~2.2 s):** the paper's own cited figure for the QSS-versus-optimal-control gap.
2. **Line optimality:** the minimum-curvature line minimises $\int \kappa^2\,ds$ (integrated squared curvature), not lap time — the time-weighted line is an M4 deliverable.
3. **Envelope conservatism (~1–1.5 s):** the double-track trim boundary delivers 85–91 % of the point-mass ideal. This is legitimate physics — a real four-wheel car cannot use as much grip as an idealised point mass.
4. **Track geometry (~5 percentage points):** swapping the committed OSM curvature for the paper's own curvature drops the lap from 92.36 s to 87.08 s.

What the cross-check **does** positively validate is the complete car transcription: peak friction coefficient exact at all vertical loads, peak-slip locations within 0.5 %, combined-slip coupling within ~5 %, top speed to −0.2 %, and the slow/fast corner speeds to ≤ 5 % on like-for-like geometry.

### 13.2 Golden-file tests: two distinct families

A *golden file* is a committed reference output; a test recomputes the same output and fails if it drifts beyond a tolerance. outlap has two families with deliberately different governance.

#### 13.2.1 The Magic Formula golden CSVs (external oracle)

The tire force model is checked against an **external** oracle. Committed CSVs in `crates/outlap-tire/tests/golden/pacejka_2006_205_60r15/` (`fx0`, `fy0_mz`, `combined`, `combined_camber`) hold forces computed by a third-party Magic Formula implementation (teasit/magic-formula-tyre-library, GPL-3.0, run under GNU Octave). outlap's own forces must match them to **≤ 0.5 %**, per point, with a small absolute floor per channel (the rule is `|model − ref| ≤ max(0.005·|ref|, floor)`). The oracle's commit hash and Octave version are recorded in each CSV's provenance header and asserted by a test.

Because these are external-oracle data, there is **no in-tree `--bless`** — you cannot regenerate them by hand. They are re-derived only by re-running the external oracle (`MF_ORACLE_SRC=... ./run.sh` under `tools/goldens/`), and a CSV diff without a provenance-header update and a physics justification is a review stop. Crucially, outlap never reads, ports, or vendors the oracle's source code — only its *output numbers* are used, as data.

#### 13.2.2 The golden laps (parquet, `OUTLAP_BLESS=1`)

Whole solved laps are frozen too. [`python/tests/test_limebeer.py`](../python/tests/test_limebeer.py) commits parquet channel sets per vehicle × track × tier (for example `limebeer_t0_flat.parquet`, `limebeer_t1_flat.parquet`, `f1_2026_t0.parquet` in `python/tests/golden/`) and re-solves them, checking each channel against its committed values with per-channel relative tolerances (speed 0.5 %, accelerations 2 %, time 0.5 %, vertical load 1 %, slip ratio/angle 5 %). It even asserts that the *pattern* of infeasible (NaN) stations has not drifted, which catches changes in where the solver decides a corner is un-trimmable.

These you *can* regenerate — but only deliberately:

```bash
OUTLAP_BLESS=1 uv run pytest tests/test_limebeer.py
```

The project convention (referred to as `--bless` in `CONTRIBUTING.md`; the concrete mechanism is the `OUTLAP_BLESS=1` environment variable) is that a golden regeneration must come with a pull-request note explaining the physics change that justifies it. Silent golden updates are forbidden — that is how a subtle regression sneaks past review.

### 13.3 Property tests: pinning the physics

Golden files catch *drift*; property tests catch *wrongness*. A property test asserts an invariant that must hold for *any* input — often across dozens of randomised cases (the project uses the `proptest` crate). These are where the physics sign conventions and conservation laws live. A representative inventory:

- **Tire model** ([`crates/outlap-tire/tests/props.rs`](../crates/outlap-tire/tests/props.rs)): outputs finite everywhere; an airborne tire ($F_z \le 0$) produces zero force; odd symmetry (flipping slip flips force); sign pins; and **friction-circle containment** — the combined-slip force never leaves the friction ellipse.
- **Trim solver** ([`crates/outlap-qss/tests/t1_trim.rs`](../crates/outlap-qss/tests/t1_trim.rs), whose header states the ISO 8855 conventions): the four vertical loads sum to weight plus downforce; lateral load transfers to the *outside* wheels in a corner; longitudinal load transfers rearward under acceleration and forward under braking; **friction-circle containment per wheel**; left/right symmetry for a symmetric car at $\pm a_y$; Newton convergence over a feasible grid; and the two `fz_coupling` modes agree at convergence.
- **Lap solver** ([`crates/outlap-qss/tests/properties.rs`](../crates/outlap-qss/tests/properties.rs)): solved laps stay inside the envelope; the forward/backward passes are idempotent; and lap time converges as the station spacing `ds` is refined.
- **Energy closure**: the powertrain and thermal tests assert *source work = mechanical work + declared losses* — energy is neither created nor destroyed.
- **Battery** ([`crates/outlap-qss/tests/battery.rs`](../crates/outlap-qss/tests/battery.rs)): the pulse response matches the closed-form Thévenin solution; state of charge decreases monotonically under discharge; the slow-state advance is deterministic; and the Vdc-coupled map reproduces in-grid values and extrapolates correctly below and above the voltage grid.
- **Thermal** ([`crates/outlap-qss/tests/thermal.rs`](../crates/outlap-qss/tests/thermal.rs)): a heat-soaking stint reduces the derate; network cooling is speed-dependent; mass heuristics fill an under-specified lumped model.
- **Interpolant** ([`crates/outlap-core/tests/gridmap_props.rs`](../crates/outlap-core/tests/gridmap_props.rs)): node values are reproduced exactly, each fibre equals the shared monotone cubic and preserves monotonicity, and the analytic gradient matches a finite-difference cross-check.
- **Envelope corrections** ([`crates/outlap-qss/src/t1/envelope.rs`](../crates/outlap-qss/src/t1/envelope.rs)): the corrected envelope is node-exact to < 2 % of peak and matches full T1 re-solves over bands of ±15 % friction, ±10 % mass, and ±30 % downforce.

Whenever new physics lands, a new property test lands with it — this is a hard project rule, not a nicety.

### 13.4 Performance and determinism

**Performance.** A QSS lap must solve in **≤ 50 ms** of wall-clock time (release build) — asserted by taking the median of eleven warmed solves on the real Catalunya track ([`crates/outlap-qss/tests/catalunya.rs`](../crates/outlap-qss/tests/catalunya.rs)). Both the direct T0 path and the production T0-on-envelope path are gated. Envelope *generation* is the documented cold assembly step and is excluded from this gate (it is a seconds-scale one-time cost; see Chapter 3 on the envelope cache). Separately, **zero-allocation** gates use the `dhat` allocator to assert that the hot-loop kernels (`solve_into`, the trim solve, the slow-state advance, `Mf61::forces`, `GriddedMapN::eval`) allocate no heap memory across warmed calls. (An instruction-count gate via `iai-callgrind` is specified but deferred until a valgrind-equipped CI runner exists; the allocation gate covers the same kernels in the meantime.)

**Determinism.** outlap is built to give bit-identical results on the same platform for the same inputs, and documented tolerance-exact results across platforms. In production paths it uses fixed-step integrators only, a **fixed** (not tolerance-driven) slow-state coupling outer-iteration count "for determinism" (`OUTER_ITERS = 2` in [`crates/outlap-qss/src/qss.rs`](../crates/outlap-qss/src/qss.rs) — the per-station trim Newton, by contrast, converges to a `1e-10` scaled-residual tolerance and is capped only by a fallback iteration budget), fixed-order reductions, and no fast-math. The recorded `fz_coupling` mode (`one_step_lag` or `fixed_point`) is part of the input, so two runs that differ only in that setting are each individually reproducible. Determinism is not just tidiness: it is a prerequisite for the future Monte Carlo race-strategy layer, which will run the fast tier thousands of times and needs to trust that a given seed always yields the same lap. The test `test_lap_is_deterministic` ([`python/tests/test_core.py`](../python/tests/test_core.py)) checks this end-to-end.

### 13.5 The CI pipeline

Every push and pull request runs [`.github/workflows/ci.yml`](../.github/workflows/ci.yml), four jobs:

1. **rust** — `cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo test --workspace`, the release-only performance gates, the schema check (`gen_schemas --check` confirms the committed `schemas/*.json` still match the Rust types), and `wasm32-unknown-unknown` builds of the wasm-facing crates.
2. **python** — builds the extension in *release* (a debug wheel would blow the test time budget), then `ruff check`, `ruff format --check`, `pyright` (strict), `pytest`, and `python -m outlap.schemas --check`.
3. **notebooks** — re-executes every notebook headless (`jupyter execute`), so the notebooks double as end-to-end tests; their in-notebook assertions (the 0.5 % tire gate, the racing line beating the centreline) must pass.
4. **wheels** — release-tag only; builds the distributable wheel.

### 13.6 What is *not* validated yet (be honest)

A trustworthy tool is clear about its limits. At v0.2.0:

- **Only one published-oracle track.** The quantitative cross-check is Catalunya against PL2014. Other tracks and vehicles are checked for internal consistency and plausible magnitudes, not against measured lap times.
- **The lap-time gate is recorded, not enforced** (Decision #48). The honest ≤ 1 % lap-time ambition moves to M4 via the transient tier and a time-weighted line.
- **Thermal and battery are validated against synthetic fixtures and closed-form solutions, not measured hardware.** The pulse-response and energy-closure tests confirm the *math* is right; they do not claim the *parameters* of any shipped car are measured (they are estimates — see Chapter 12).
- **The QSS↔transient parity gate is not active.** It is fully specified (lap time within 0.3 %, apex speeds within 1 %, transient samples inside the T1 hull) but cannot run until the transient tier (T2) exists — `crates/outlap-transient` is a placeholder crate today, and requesting `tier="t2"` returns a typed "not implemented until M4/M6" error. Treat the 0.3 % parity figure as a *committed future gate*, not a current guarantee.
- **Estimated values are everywhere, and that is by design** — but it means you must read the loaded-model report. Every estimate is listed there; a lap run in degraded mode marks its results. Nothing is hidden, but nothing is measured-perfect either.

The guiding principle throughout: estimated and simplified values always surface in the loaded-model report and the lap `notes`, so you are never misled about what the numbers rest on. Chapter 12 tells you the provenance of each shipped asset; Chapter 10 shows you how to read the report and notes from Python.


---

## 14. Recipes: worked examples

*What you will learn: eight end-to-end tasks, each a complete runnable script with real output. These are the "how do I actually do X?" answers — building your own car, comparing solver tiers, sweeping motor sizes, importing a track, feeding in your own powertrain, and reading the thermal, battery, and envelope channels. Read Chapters 3, 4, and 10 first; everything here builds on them.*

Every code block below imports from `outlap.core` and is run the same way — from the `python/` directory of a built checkout:

```bash
cd python
uv run --no-sync python your_script.py
```

Two speed conventions used throughout: paths are written relative to `python/` (so `../data/...` reaches the repo's data), and the examples use a **fast, coarse envelope grid** so they finish in a second or two:

```python
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}
```

The production default grid is $40\times25\times7$ (Chapter 8); it is more accurate but takes longer to generate. Coarse grids give lap times a few tenths off the fine-grid value — fine for exploring, not for a headline number. Recipes A and B below show *real, executed* output (the lap times are exact for the fast grid); the numbers you get on the production grid will differ slightly.

### 14.1 Recipe A — build your own vehicle from scratch

The fastest way to author a car is to copy the closest reference vehicle and edit it. Here we make a lighter, draggier EV from the Model 3.

**Step 1 — copy the vehicle directory** (work outside the repo so you do not dirty it):

```bash
cp -r data/vehicles/tesla_model3_rwd /tmp/my_ev
```

The directory carries everything the car references: `vehicle.yaml`, the `ptm/` drive-unit maps, `tyr/road.tyr.yaml`, `battery/`, and `emotor/` (Chapter 4).

**Step 2 — edit `vehicle.yaml`.** Change the mass and the drag area. In `/tmp/my_ev/vehicle.yaml`:

```yaml
chassis:
  mass_kg: 1500.0        # was 1765.0
aero:
  constant:
    cx_a_m2: 0.62        # was 0.51 — a boxier body
```

**Step 3 — check the loaded-model report** before you trust anything. A clean load with estimates noted is what you want:

```python
from outlap.core import vehicle_report
rep = vehicle_report("/tmp/my_ev")
print("name      ", rep["name"])
print("estimated ", len(rep["estimated"]))
print("warnings  ", len(rep["warnings"]))
print("degraded  ", len(rep["degraded"]))
```

Real output:

```text
name       Tesla Model 3 RWD (HV variant)
estimated  10
warnings   0
degraded   0
```

Ten estimated values (unchanged from the parent car — you edited only spec-sheet fields), zero warnings, nothing degraded. (Change the `name:` field in your copy so the report reflects your car; the value above is inherited because we only edited mass and aero.)

**Step 4 — lap both cars and compare:**

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

Real output:

```text
stock Model 3 RWD  : 145.33 s
my_ev (1500kg,0.62): 141.61 s
delta              : -3.72 s
```

Dropping 265 kg beat the extra drag: the lighter car is 3.72 s quicker over the lap. Note the resolved hash in each Dataset's `attrs["resolved_hash"]` differs — the two cars are genuinely distinct resolved specs, not the same car twice.

> **Shortcut — no file editing.** For a quick what-if you do not even need to copy files: pass `overrides={"chassis.mass_kg": 1500.0, "aero.constant.cx_a_m2": 0.62}` to `solve_lap_dataset`. The override goes through the *real* validation pipeline (a bad path or out-of-range value fails loudly) and changes the resolved hash. Copy-and-edit is for a car you want to keep; overrides are for experiments.

### 14.2 Recipe B — compare the T0 and T1 tiers

The tiers answer different questions (Chapter 8). T0 is the point-mass velocity profile; T1 re-trims the double-track car at every station for per-wheel detail. The surprising-at-first fact: **on the same car and line they return the same lap time** — T1's re-trim runs on the profile T0 produced.

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

Real output:

```text
t0 lap_time_s 145.33
t1 lap_time_s 145.33
t0 channels   ['v', 'ax', 'ay', 't', 'x', 'y', 'z', 'state_of_charge', 'machine_temp_c']
t1 channels   ['v', 'ax', 'ay', 't', 'x', 'y', 'z', 'vertical_load_n', 'slip_ratio', 'slip_angle_rad', 'force_long_n', 'force_lat_n', 'understeer_gradient', 'aero_front_share', 'state_of_charge', 'machine_temp_c']
```

Both laps are 145.33 s. What T1 *adds* is the per-wheel detail: the `(s, wheel)` channels (`vertical_load_n`, `slip_ratio`, `slip_angle_rad`, `force_long_n`, `force_lat_n`) and the setup metrics (`understeer_gradient`, `aero_front_share`). (Both tiers carry the slow-state channels `state_of_charge` and `machine_temp_c` here, because this is an electrified car with a live battery/thermal stack — those channels gate on the powertrain, not the tier.) Inspect the wheel loads at the fastest point of the lap:

```python
v = t1["v"].values
i = int(np.nanargmax(v))
print(f"at v = {v[i]:.1f} m/s, Fz [FL FR RL RR] =",
      np.round(t1["vertical_load_n"].values[i], 0))
```

Real output:

```text
at v = 64.1 m/s, Fz [FL FR RL RR] = [3987. 4618. 4066. 4638.]
```

The wheel order is always `["FL", "FR", "RL", "RR"]` (front-left, front-right, rear-left, rear-right), available as `t1.coords["wheel"]`. Here the car is at high speed in a gentle right-hand curve, so the left wheels carry slightly less load than the right. Use these channels to plot per-wheel load through a corner, check which wheel saturates first, or compute your own tire-usage metrics.

### 14.3 Recipe C — sweep the motor sizing

The Model 3 ships three synthetic drive-unit sizings — `du_small` (≈ 100 kW), `du_medium` (≈ 203 kW, the default), and `du_large` (≈ 248 kW) — so you can study how motor power buys lap time. You do **not** edit files; you override the drive unit's source map:

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

Real output:

```text
du_small   155.84 s
du_medium  145.33 s  (-10.51 s vs previous)
du_large   143.32 s  (-2.01 s vs previous)
```

The story is **diminishing returns**: the jump from small to medium is worth 10.51 s, but medium to large only 2.01 s. Past a point, more torque no longer helps — the tires, the machine-thermal derate, and the pack's power ceiling become the limits, not the motor's rated torque. This is exactly the sensitivity axis notebook 07 explores (on the production grid, where the deltas are −13.41 s and −2.67 s). See Chapter 12 for the provenance of these three sizings (they are invented smooth surfaces, not measured maps).

### 14.4 Recipe D — import and lap a track from the databases

outlap ships 26 tracks (Chapter 12), but you can add your own. The two importers are one-time local tools (Chapter 11).

**From the TUMFTM database** (flat 2-D centre lines):

```bash
cd python
uv run --no-sync python -m outlap.importers.tumftm_track \
    --input /path/to/tumftm/racetrack-database/tracks/Zolder.csv \
    --out ../data/tracks/zolder
```

**From OpenStreetMap + elevation** (full 3D ribbon; needs the `track-import` extra for the network and elevation dependencies):

```bash
uv run --extra track-import python -m outlap.importers.osm_track \
    --preset catalunya --out ../data/tracks/my_catalunya
```

Then load and lap it exactly like a shipped track. Any of the 26 vendored tracks works out of the box — for example, two TUMFTM circuits with the stock Model 3:

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

Real output:

```text
Silverstone Circuit          5887 m   lap 166.61 s
Autodromo Nazionale Monza    5790 m   lap 141.77 s
```

Remember that the vendored TUMFTM tracks are **flat** (`z = 0`, `banking = 0`, accuracy class C — Chapter 12) and are redistributed under LGPL-3.0 with the required attribution. Only `catalunya_osm` carries real elevation.

### 14.5 Recipe E — bring your own powertrain via the PDT importer

If you have a professional drive-tool (PDT) HDF5 export, the importer distils it into outlap's neutral `.ptm` maps. This is a **local-only, firewall-respecting** workflow (Chapters 1 and 11): you never commit the raw `.h5` source or the derived `.ptm` — the reference vehicles keep their real imports in a git-ignored `local/` directory.

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

Then point your (also local) vehicle at the imported files and lap it. Because `local/` is git-ignored, none of this leaves your machine. The importer reads only clean, documented HDF5 fields via `h5py`; it never imports PDT code (the firewall). The output above is illustrative — the exact commands and file names are in the `tesla_model3_rwd/README.md`, and you need the real `.h5` inputs to run them.

### 14.6 Recipe F — study machine-thermal derating

The Model 3 carries a machine thermal network (Chapter 9), so a lap reports the winding temperature and the resulting torque derate. Read the `machine_temp_c` channel over the lap:

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

Real output (single lap from a cold start):

```text
machine winding: start 20.0 °C, peak 127.5 °C
```

The winding starts at ambient (20 °C) and climbs toward its warning threshold (150 °C for this car — see its `emotor/rear_du.emotor.yaml`) as heat soaks in. If it reached the warning band, the torque derate (a linear 1→0 ramp from `t_warn_c` to `t_max_c`, Chapter 9) would begin to cap the traction ceiling. To see meaningful heat soak you want a *long* run — lap a longer track (Silverstone, Spa) or the same track repeatedly; the temperature carries forward station to station because it is a slow state. The units are °C at this display boundary (the internals are in kelvin).

### 14.7 Recipe G — watch battery SoC and the Vdc coupling

The same electrified stack reports the pack state of charge as it drains, and — because this is an 800 V-class "HV variant" whose pack voltage sags below the drive unit's voltage grid at low charge — it exercises the Vdc–SoC coupling (Chapter 9).

```python
from outlap.core import Track, min_curvature, solve_lap_dataset
FAST = {"envelope": {"v_points": 8, "ax_points": 7, "g_normal_points": 2}}

trk  = Track.load("../data/tracks/catalunya")
line = min_curvature(trk, half_width_m=0.95)
d    = solve_lap_dataset("../data/vehicles/tesla_model3_rwd", line, tier="t1", sim=FAST)

soc = d["state_of_charge"].values
print(f"SoC: start {soc[0]:.3f}, end {soc[-1]:.3f}  (drop {soc[0]-soc[-1]:.3f})")
```

Real output:

```text
SoC: start 0.980, end 0.904  (drop 0.076)
```

The pack loses about 7.6 % of charge over one Catalunya lap (net of regen). As SoC falls, the pack's terminal voltage falls with it; when that voltage drops below the drive unit's Vdc grid, the machine maps are evaluated by linear extrapolation along the voltage axis (with physical floors), and any extrapolated band is recorded in the lap `notes`. The battery's peak-power limit and the thermal derate both act as `min` caps on the traction ceiling — neither is baked into the reference-state envelope. To see the coupling bite hard, start from a lower SoC or run a long stint so the pack voltage sags well below the grid.

### 14.8 Recipe H — extract the g-g-g-v envelope

The envelope (Chapter 8) is a first-class returnable object. Call `solve_lap` (not the `_dataset` variant) to get a `Lap`, then query its `.envelope`:

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

Real output:

```text
shape    [8, 7, 2]
domain   [[5.0, 67.0], [-1.0, 1.0], [4.9, 19.61]]
mass_ref 1765.0
ay_boundary(50 m/s, ax=0, g_normal=9.81) = 8.07 m/s²
accel_limit(30 m/s, 9.81)                = 7.09 m/s²
brake_limit(50 m/s, 9.81)                = 11.67 m/s²
```

The envelope's three axes are speed $v$, normalised longitudinal acceleration $\hat a_x \in [-1, 1]$, and normal gravity $g_{\text{normal}}$ (Chapter 8). `ay_boundary` gives the maximum lateral acceleration available at a query point; `accel_limit` and `brake_limit` give the longitudinal limits net of drag. This is the object the T0 solver consumes — pulling it out lets you plot the car's grip surface, compare two cars' envelopes, or feed a downstream analysis. (For a production-quality surface, run with the default grid instead of `FAST`; here `shape` reflects the coarse `[8, 7, 2]` we asked for.)

### 14.9 Where to go from here

These recipes cover the core loop: author, solve, inspect, sweep. Combine them freely — the sizing sweep of Recipe C with the thermal read of Recipe F shows *why* the big motor stops helping; the envelope of Recipe H over two cars from Recipe A shows *how* their grip differs. Notebook 07 (`notebooks/07_qss_t1.ipynb`) is a longer, plotted version of this same material on the F1 car and the Model 3; the theory pages in `docs/theory/` give the equations behind each channel.


---

## 15. Limitations and roadmap

*What you will learn: an honest account of what outlap v0.2.0 does **not** do, and the planned path forward. Knowing the boundaries is as important as knowing the features — it tells you when a number can be trusted and when you are outside the tool's design envelope.*

### 15.1 What v0.2.0 does not do

outlap v0.2.0 is a complete **quasi-steady-state** simulator (the T0 and T1 tiers). That word "quasi-steady-state" is the single biggest scoping fact: at each point on the track the car is assumed to be in instantaneous force-and-moment balance, as if it had always been at that speed and that cornering state. The consequences:

- **No transient dynamics.** There is no time-marching model of the car rotating into a corner, weight settling after a bump, or a slide developing and being caught. The transient tiers — **T2** (a curvilinear 3D road-frame integrator with an ideal driver and shift events) and **T3** (a 14-degree-of-freedom model) — are milestones M4 and M6. Asking for `tier="t2"` or `"t3"` today returns a typed "not implemented until M4/M6" error. The crates that will host them (`outlap-transient` and `outlap-vehicle` for the transient tiers, plus `outlap-batch` for the M7 batch/sweep layer) are placeholder stubs.
- **No driver model.** Because it is quasi-steady, the solver assumes an *ideal* driver who always uses exactly the available grip. There is no reaction time, no steering-input model, no error. A real transient driver model arrives with T2.
- **No tire thermal or wear.** The tires use a fixed friction coefficient at a reference pressure and camber. The tire thermal ring (a tire that heats, gains and loses grip, and goes off) and the wear/cliff model are the **headline of milestone M5** (v0.3). Today, `.tyr` files carry *synthetic, clearly-labelled placeholder* thermal and wear blocks — they exist to satisfy the schema, not to model anything. (The *machine* thermal network of Chapter 9 is a separate, real model; it is the electric drive unit that heats up, not the tires.)
- **Fuel mass is constant.** There is no fuel-burn slow state, so a combustion car does not get lighter over a lap. Fuel mass arrives in M6.
- **No torque-vectoring controller.** The drivetrain uses *static* front/rear and left/right torque splits and passive differential models (open, locked, LSD, solid — Chapter 9). A rule-based yaw-moment torque-vectoring controller is an M4 deliverable; the optimisation-based (quadratic-programming) allocation is post-v1.
- **The ERS energy manager is a power cap only.** For the F1 car, the hybrid deployment/harvest budget is modelled as a simple power limit, not a lap-by-lap energy manager with deploy tapering and override modes. That full manager is M6.
- **No race-strategy layer.** The Monte Carlo race-strategy simulation — the long-term goal that motivates the fast, deterministic T0 tier — is a post-1.0 stage. It is why determinism and the sub-50 ms lap matter now, but it is not here yet.
- **Aero maps only for the F1 car.** Only `f1_2026` ships a ride-height/yaw downforce map. Every other vehicle uses the constant-$C_dA$/$C_zA$ degenerate path (Chapter 7) — correct for a road car, a simplification for a high-downforce car.
- **Vendored tracks are flat.** The 25 TUMFTM circuits carry `z = 0`, `banking = 0`, and `grip_scale = 1` (accuracy class C — Chapter 12). Only `catalunya_osm` (class B) has real elevation, and even it has widths defaulted and banking unresolved. So grade, banking, and vertical-curvature physics are *implemented* but exercised only on the one 3D track.
- **Single-lap focus.** The tool solves one lap at a time. Multi-lap stints (with tire and fuel evolution) are the M5 demo; batch sweeps and a CLI are M7.
- **The class presets are not shipped.** `data/presets/` is reserved but empty; the promised `formula_base`/`gt_base`/`passenger_base` starting points are a future deliverable. Author a new car by copying a reference vehicle (Chapter 14, Recipe A) for now.

None of these are hidden. Every simplification a given lap makes is listed in that lap's `notes` attribute, and every estimated parameter is in the loaded-model report (Chapters 10 and 13).

### 15.2 What "accuracy class C" means for a track

Each track records a `meta.accuracy_class`. It is a provenance grade, not a precision guarantee:

- **Class B** (`catalunya_osm`): built from OpenStreetMap geometry fused with open elevation data. Has a real 3D ribbon, but corridor widths are defaulted and banking is not resolved from the coarse public elevation model.
- **Class C** (the 25 TUMFTM circuits): smoothed centre lines with satellite-measured corridor widths, but strictly 2-D. The standard academic bootstrap dataset — good for relative comparisons, not for matching a real lap record.

A class-C track will give you plausible, self-consistent lap times that are useful for comparing cars or setups, but you should not expect them to match a measured lap; the smoothed geometry alone can shift corner speeds by several percent (Chapter 13).

### 15.3 The roadmap

The milestone plan (from `docs/HANDOFF.md` §12) — roadmaps can and do change, so treat this as intent, not commitment:

| Milestone | Version | Headline deliverable |
|-----------|---------|----------------------|
| M1 | 0.1 | Schema + 3D track + OSM/DEM importer + minimum-curvature line + T0 point-mass lap |
| M2 | — | MF6.1 tire model + `.tir` codec + Python fitting pipeline + citation-backed `.tyr` files |
| **M3** | **0.2** | **Full QSS T1: double-track trim → g-g-g-v envelopes, aero maps, topology powertrain, machine-thermal derating, battery/Vdc coupling, Limebeer cross-check (you are here)** |
| M4 | — | Transient tier **T2** (3D road frame, ideal driver, shift events, rule-based torque-vectoring) + the QSS↔transient parity gate + time-weighted racing-line solver + the deferred ≤ 1 % Limebeer lap-time gate |
| M5 | 0.3 | **Tire thermal ring + wear** in both tiers — the stint-simulation demo |
| M6 | — | Full ERS 2026-style energy manager + battery ECM + **fuel mass** + T3 (14-DOF) |
| M7 | **1.0** | Batch/sweep API (parallel, structure-of-arrays) + CLI + all four reference vehicles + the hero demo + docs site + a WebAssembly demo widget |

Beyond 1.0, the recorded intent is: sim-racing telemetry importers (MoTeC, ACC, iRacing) for community data and validation; the **Stage 2 race-strategy Monte Carlo** layer (a time-discrete race simulation with a stochastic layer and a strategy optimiser running on the fast T0-with-slow-states tier); a browser app (`outlap-web`) grown from the WASM widget; and a community data registry. The core discipline that makes all of this possible — one vehicle description across tiers, a deterministic zero-allocation hot loop, and honest reporting — is in place now.

---

## 16. Glossary

*Terms of art used throughout this guide and the codebase, alphabetised. Each entry is one sentence; the chapter that treats it in depth is noted where relevant.*

- **Aero balance** — the fraction of total aerodynamic downforce carried by the front axle; shifts the car toward understeer or oversteer (Chapter 7).
- **Aligning moment ($M_z$)** — the self-centring torque a tire generates about its vertical axis, which gives steering its feel and feedback (Chapter 7).
- **Anti-dive / anti-squat** — suspension geometry that resists the nose dipping under braking (anti-dive) or the tail squatting under acceleration (anti-squat), quantified as a fraction (Chapter 4).
- **Assembly pipeline** — the one-time, load-time phase that reads YAML, merges inheritance, validates, estimates missing values, and builds the solver's vehicle object; distinct from the hot loop, it may allocate and do heavy work (Chapter 6).
- **CdA / $C_xA$** — drag area: the drag coefficient times frontal area, in m²; drag force is $\tfrac12\rho\,C_xA\,v^2$ (Chapters 2, 7).
- **ClA / $C_zA$** — lift/downforce area: the (negative) lift coefficient times area, in m²; downforce grows with speed squared (Chapters 2, 7).
- **Clean-room** — the project rule that every physics model is re-authored from published literature with citations, never copied from another codebase (Chapters 1, 13).
- **Combined slip** — a tire simultaneously braking/driving and cornering, so its longitudinal and lateral forces share one friction budget (the friction circle) (Chapters 2, 7).
- **Contact patch** — the small area where a tire touches the road and through which every driving, braking, and cornering force is transmitted (Chapter 2).
- **Damped Newton (trim solve)** — the iterative root-finder that solves the T1 force-and-moment balance at each station, with line-search damping for robustness (Chapter 8).
- **Degraded mode** — a fallback load path (`allow_degraded: true`) that lets an otherwise-unsupported configuration run, marking the results as degraded (Chapters 4, 13).
- **Determinism** — the guarantee that the same inputs give the same outputs (bit-exact on one platform), via fixed-step integration, fixed iteration counts, and fixed-order reductions (Chapter 13).
- **Double-track model** — a car model with all four wheels represented individually (as opposed to a single-track "bicycle" model), so left/right load transfer and per-wheel forces are resolved; the T1 tier (Chapters 2, 8).
- **Firewall** — the hard rule that powertrains enter only as neutral `.ptm` map files, never as internal machine/inverter/gearbox models; the machine thermal network is a narrow, documented exception (Chapters 1, 9).
- **Flat-track mode** — an analysis setting (`sim.flat_track`) that zeroes grade, banking, and vertical curvature so the g-g-g-v envelope collapses to a flat g-g, used for the 2D Limebeer comparison (Chapters 8, 13).
- **Friction circle / ellipse** — the boundary of the combined longitudinal-plus-lateral force a tire can produce; staying inside it is the fundamental grip constraint (Chapters 2, 7).
- **`fz_coupling`** — the recorded choice of how the algebraic normal-load loop is resolved: `one_step_lag` (default, uses the previous step's loads) or `fixed_point` (iterates to convergence) (Chapters 4, 8).
- **g-g diagram** — the set of longitudinal ($a_x$) and lateral ($a_y$) accelerations a car can reach, drawn as a 2D region; the classic performance envelope (Chapter 2).
- **g-g-g-v envelope** — the g-g diagram extended with dependence on speed ($v$) and normal gravity ($g_{\text{normal}}$, which captures banking and crests); a precomputed grip surface the T0 solver consumes (Chapter 8).
- **Golden file** — a committed reference output that a test recomputes and compares against, failing on drift beyond a tolerance (Chapter 13).
- **`GriddedMapN`** — outlap's N-dimensional gridded-map type, evaluated by the one shared monotone cubic interpolant, used for aero maps, `.ptm` tables, and the envelope (Chapters 5, 8).
- **Hot loop** — the per-station (or per-timestep) inner computation that must be allocation-free, contain no Python, and use fixed-size state; distinct from the assembly pipeline (Chapter 6).
- **ISO 8855** — the vehicle axis convention outlap uses: $x$ forward, $y$ left, $z$ up; it fixes the sign of every force, slip, and acceleration (Chapters 2, 6).
- **Loaded-model report** — the dictionary returned by `vehicle_report`, listing every inherited, estimated, degraded, warned, and overridden value; the "nothing silent" surface (Chapters 4, 10).
- **Load sensitivity** — the fact that a tire's friction *coefficient* falls as its vertical load rises, so doubling the load less-than-doubles the grip (Chapter 2).
- **Load transfer** — the shift of vertical load between wheels under acceleration, braking, and cornering; longitudinal (front↔rear) and lateral (left↔right) (Chapters 2, 8).
- **LPTN (lumped-parameter thermal network)** — the machine thermal model: a small network of thermal masses (nodes) connected by conductances, integrated per lap segment (Chapter 9).
- **Magic Formula / MF6.1** — the empirical tire force model (Pacejka's version 6.1) that fits measured tire data with a characteristic sine-of-arctangent curve (Chapters 2, 7).
- **Minimum-curvature line** — the racing line that minimises integrated squared curvature within the track corridor, found by a quadratic program; not the same as the time-optimal line (Chapters 2, 8).
- **Monotone cubic Hermite** — the single (Fritsch–Carlson) interpolation scheme used for *all* gridded maps: smooth ($C^1$) and shape-preserving, so it never overshoots between data points (Chapters 5, 8).
- **`one_step_lag`** — see `fz_coupling`; the default, cheaper normal-load coupling mode (Chapters 4, 8).
- **Peak μ (peak friction coefficient)** — the maximum friction a tire delivers, extracted from the Magic Formula curves at a reference load and pressure; the T0 tier distils a tire to this number (Chapter 7).
- **Point-mass model** — a car idealised as a single mass with a grip limit, ignoring individual wheels; the T0 tier (Chapters 2, 8).
- **Property test** — a test asserting an invariant that must hold for *any* input (e.g. friction-circle containment), often across many randomised cases (Chapter 13).
- **`.ptm` file** — the neutral powertrain-map file (YAML plus a parquet table sidecar) through which every powertrain enters the simulator (Chapters 5, 9).
- **QSS (quasi-steady-state)** — the modelling assumption that the car is in instantaneous equilibrium at each point; the basis of the T0 and T1 tiers (Chapters 2, 8).
- **Racing line** — the path the car actually drives through the corridor, distinct from the track's centre line (Chapters 2, 8).
- **Relaxation length** — the distance a tire must roll before its force builds up to the steady-state value; a transient tire property (relevant to the brush model) (Chapter 7).
- **Ribbon** — outlap's 3D track model: the road surface as a band with curvature, grade, and banking parameterised by arc length (Chapters 2, 5).
- **Roll centre** — the geometric point about which a car's sprung mass rolls in a corner; its height governs how much load transfer is geometric versus elastic (Chapters 2, 8).
- **Slip angle ($\alpha$)** — the angle between where a tire points and where it is actually travelling; the source of lateral (cornering) force (Chapters 2, 7).
- **Slip ratio ($\kappa$)** — the relative difference between a tire's rolling speed and the road speed; the source of longitudinal (drive/brake) force (Chapters 2, 7).
- **Slow state** — a quantity that evolves gradually across the lap and carries between stations (machine temperature, battery SoC), as opposed to the fast per-station trim states (Chapter 9).
- **SoC (state of charge)** — the battery's remaining charge as a fraction from 0 to 1; a slow state that falls as the pack discharges (Chapter 9).
- **Tier (T0/T1/T2/T3)** — the solver ladder: T0 point-mass, T1 quasi-steady double-track (both shipping), T2/T3 transient (future) (Chapters 2, 8).
- **Thévenin (battery) model** — an equivalent-circuit battery model (open-circuit voltage minus resistive and RC-network drops) used to compute terminal voltage under load (Chapter 9).
- **Trim** — the equilibrium state of the car at a given operating point: the steering, body slip, and wheel loads/slips that balance all forces and moments (Chapter 8).
- **Understeer gradient** — a setup metric measuring how much extra steering a car needs as lateral acceleration rises; positive means understeer (Chapters 2, 8).
- **Vdc** — the DC-link voltage supplied to an electric drive unit; when a battery pack is present, the drive-unit maps are evaluated at the pack's SoC-dependent terminal voltage (the Vdc–SoC coupling) (Chapter 9).

---

## 17. FAQ and troubleshooting

*Common questions and errors, answered from the actual behaviour of the tool. If something here surprises you, the referenced chapter has the full story.*

**Why does `import outlap` give me almost nothing?**
The top-level `outlap` package is a stub — the real API lives in `outlap.core`. Always import from there: `from outlap.core import Track, min_curvature, solve_lap_dataset, vehicle_report` (Chapter 10).

**Why does `tier="t2"` (or `"t3"`) raise an error?**
The transient tiers are not implemented yet — they are milestones M4 and M6. The error is deliberate and typed: *"solver tier `t2` is not implemented yet (the transient tiers arrive in milestone M4); select tier `t0` … or `t1`."* Use `t0` or `t1` (Chapters 8, 15).

**Why do T0 and T1 give the same lap time?**
Because T1 re-trims the double-track car *on the velocity profile T0 already produced* — it adds per-wheel loads, slips, forces, and setup metrics, but does not change the lap time on the same car and line. If you want per-wheel detail, use `t1`; if you only need the lap time and speed trace, `t0` is enough (Chapters 8, 14).

**Why is the first lap so slow, then fast afterwards?**
The g-g-g-v envelope is generated once per car+grid (a seconds-scale cold step in release, minutes in a debug build) and then cached for the rest of the process. Subsequent laps of the same car reuse it. If your first solve takes *minutes*, you have a debug wheel — rebuild with `MATURIN_PEP517_ARGS=--profile release` before `uv sync` (Chapters 3, 10).

**Why did `uv sync` break my notebooks?**
A plain `uv sync` uninstalls dependency groups it was not told to keep. Always include the group you need: `uv sync --group notebooks --extra tire-fit` (Chapter 3).

**Why are there estimated values in my loaded-model report? How do I get rid of them?**
Reference vehicles carry documented estimates for parameters that are not on a public spec sheet (inertias, roll centres, ride rates — Chapter 12). They are *surfaced*, not hidden — a car that loads with estimates and zero warnings is healthy. To pin a value, set it explicitly in your `vehicle.yaml`; it will then leave the `estimated` list (Chapters 4, 14).

**What does "degraded" mean, and why are my results marked?**
Degraded mode (`allow_degraded: true`) is the single fallback path for a configuration outlap cannot fully support; it lets the run proceed but marks the results so you know they rest on a fallback. If you did not opt in, you will not see degraded results (Chapters 4, 13).

**My config error mentions "did you mean …?" — what is that?**
Config errors are treated as a product surface: a misspelled field produces a `miette`-style diagnostic with a source span and a spelling suggestion (for example, `chassis.masss_kg` → *"did you mean `mass_kg`?"*). A bare, unhelpful serialization error reaching you is considered a bug (Chapters 4, 6).

**Can I use my own tire data?**
Yes. If you have a `.tir` file (the TNO MF-Tyre text format), convert it with `python -m outlap.tir to-tyr`. If you have raw test data, the `outlap.tirefit` pipeline (needs the `tire-fit` extra) fits an MF6.1 set. Note the redistribution rule: you may keep and fit membership-locked tire-test data locally, but you may not commit or redistribute it or parameter sets derived from it (Chapters 5, 11).

**Does the Model 3's pack voltage change its drive-unit maps?**
Yes — the Model 3 HV-variant is *Vdc-coupled*: its drive-unit efficiency/loss maps carry a DC-link-voltage axis and are evaluated at the pack's SoC-dependent terminal voltage (Chapter 9). The shipped `du_medium.ptm` grids that axis over 730–850 V; the pack's open-circuit voltage (~634–810 V) sags under load, so at *low* charge the terminal voltage can drop below the grid and the voltage axis is then read by linear extrapolation from the boundary slice — a deliberate choice so a depleted pack stays usable instead of clamping. Two caveats, though: this extrapolation is an internal assembly behaviour and is *not* surfaced in the per-lap `notes`; and a single default lap barely discharges the pack (SoC 0.98 → ~0.90), so the voltage stays inside the grid and no extrapolation actually happens. It is expected behaviour for that car, not an error (Chapters 9, 14).

**Why are some of my per-wheel channels NaN?**
A NaN at a station means the trim solver judged that operating point infeasible (un-trimmable) — the solver treats it as a boundary rather than crashing. A pattern of NaNs on the hardest corners is normal; the golden-lap tests even assert the NaN *pattern* does not drift (Chapters 8, 13).

**How do I regenerate a golden file?**
For the golden *laps*, run `OUTLAP_BLESS=1 uv run pytest tests/test_limebeer.py` and include a pull-request note explaining the physics change. For the tire golden *CSVs*, there is no in-tree bless — they are external-oracle data, re-derived only by re-running the oracle with a provenance-header update. Silent golden changes are a review stop (Chapter 13).

**Is outlap deterministic across machines?**
On the same platform, the same inputs give bit-identical results. Across platforms, results are tolerance-exact and documented. This is enforced by fixed-step integration, fixed iteration counts, fixed-order reductions, and no fast-math — and it is a prerequisite for the future Monte Carlo layer (Chapter 13).

**Can I run outlap in the browser?**
The solver crates are kept WebAssembly-clean and CI builds them for `wasm32-unknown-unknown`, so the core *can* run in the browser. A WebAssembly demo widget lands with v1.0 (M7); the packaged browser app (`outlap-web`) grown from it is post-1.0 (Chapters 6, 15).

**There are two "catalunya" tracks — which do I use?**
`catalunya_osm` is the 3D OpenStreetMap+elevation import — the reference Catalunya used by the notebooks, examples, and the Limebeer cross-check. `catalunya` is the flat TUMFTM vendoring, a peer of the other 24 circuits. Same circuit, two sources; use `catalunya_osm` unless you specifically want the flat 2D version (Chapters 12, 13).

**What licence applies to a vehicle or track I create?**
Your own authored data is yours. But be aware of the inputs you build on: outlap's *code* is AGPL-3.0-only, the *schemas* are Apache-2.0, the shipped *reference data* is CC-BY-SA-4.0, and the vendored *TUMFTM track centre lines* are LGPL-3.0 with a required attribution string. If you redistribute a track derived from the TUMFTM data, carry that attribution (Chapters 1, 12).

**How fast should a lap solve, and how big are the results?**
A single QSS lap solves in well under 50 ms once the envelope is cached (that is a CI-gated performance guarantee). A full T1 Dataset for a ~4.6 km track at 2 m spacing is roughly 0.6 MB (about 2,300 stations × 16 channels). For cheap exploration, use a coarse envelope grid (Chapters 13, 14).

**Where do I ask for help or contribute?**
Start with `CONTRIBUTING.md` (contributions are AGPL-3.0 with a DCO sign-off), the theory pages in `docs/theory/`, and the notebooks in `notebooks/`. The full architecture and decision log is `docs/HANDOFF.md` (Chapter 1).
