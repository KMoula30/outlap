<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Block / Bus / SoA — the transient scaffolding

`outlap-core` holds the data-flow scaffolding every transient tier (T2/T3) is assembled on: the
**Block** abstraction, the flat struct-of-arrays signal **Bus**, the frozen **state registry**, and
the topological-sort **assembler**. It is `wasm`-clean (no filesystem, threads, or clock) and carries
no `sim.yaml`/schema types — the mapping from configuration to this layer happens in the assembly
pipeline (HANDOFF §6.2b), never in the loop.

Implemented to the design in HANDOFF §6.2 and Locked Decisions #26 (runtime composition, enum
dispatch — no `dyn` in the loop), #38 (controllers are built-in blocks; no Python in a timestep),
and #39 (hybrid signal bus: fixed core indices + interned dynamic channels).

## Block

A **block** is `(immutable parameters, states, typed ports)`. The trait exposes three pure,
`f32`/`f64`-generic evaluations, matching the tier being run:

```rust
trait Block<T: Float> {
    fn phase(&self) -> Phase;                 // sense | control | actuate | integrate
    fn ports(&self) -> Ports;                 // bus channels read / written (static)
    fn equilibrium(&self, bus, slow, lane);   // T0/T1 algebraic trim contribution
    fn derivatives(&self, x, bus, dx, lane);  // T2/T3 fast-state RHS
    fn slow_derivatives(&self, bus, dslow, lane); // thermal / wear / SOC on the slow clock
}
```

Blocks run **per lane**: the caller binds the SoA views to a lane and passes the same `lane` for the
bus accessors. In the hot loop the concrete block is reached through the `CoreBlock` **enum**, never
a trait object — physics and controller blocks are added as variants in later PRs (Chassis, Tire×4,
Aero, Driver, TV, …). The external plugin trait is deferred (Decision #38); the M4 controllers are
built-in enum variants. M4 ships one variant — a **stubbed suspension block** that reserves the T3
slot and port surface without contributing dynamics.

## Bus — the signal board

Blocks never talk to each other directly; they publish to and consume from a shared **Bus** of
typed scalar channels. Two regions:

* a **fixed core set** with compile-time indices — the signals every built-in T2 block exchanges;
* an **interned dynamic region** for plugin/custom named channels. A `ChannelInterner` resolves
  names to integer `ChannelId`s **once at assembly**; the hot loop only ever sees indices — never a
  string or a hash (Decision #39).

Every channel carries an explicit **batch dimension** (SoA, state-major): channel `c`, lane `b`
lives at `c·batch + b`, so one channel is contiguous across the batch — GPU-transposable (HANDOFF
§11.3) and cache-friendly for the rayon batch loop. Construction allocates; access is
allocation-free (CI-gated by a dhat test).

## Frozen layout note

This layer freezes two index layouts. They are an internal contract: downstream code addresses them
through the enums below, never by bare integers, and additions append (they never reorder).

**Fixed bus channels** (`CoreSignal` scalars + `WheelSignal` per-wheel groups, `WHEELS = 4`, ISO
8855 order FL, FR, RL, RR):

| Region | Channels |
|--------|----------|
| Scalar (`CoreSignal`) | `Steer`, `Throttle`, `Brake`, `DriveTorque`, `YawMomentDemand`, `AeroDrag`, `AeroFzFront`, `AeroFzRear` |
| Per-wheel (`WheelSignal`, ×4 each) | `TireFx`, `TireFy`, `TireFz`, `TireMz`, `SlipKappa`, `SlipAlpha`, `SlipKappaSs`, `SlipAlphaSs`, `WheelDriveTorque`, `WheelBrakeTorque` |

**Fast-state registry** (`[chassis | relaxation]`). The chassis region reserves the full **14-DOF**
footprint so the T3 groundwork is laid without a layout break; T2 integrates only the first ten
slots. The relaxation region holds a lagged `κ` and `α` per wheel.

| Region | Slots | Tier |
|--------|-------|------|
| T2 chassis (`ChassisState`) | `s, n, ψ_rel, vx, vy, r, ω₁..₄` (10) | **T2** integrated |
| T3-reserved chassis | heave/pitch/roll + rates (6), four unsprung z + rates (8) | reserved (reads 0 in M4) |
| Relaxation (`RelaxState`, ×4) | lagged `κ`, `α` | populated in PR4 |

Slow states (temperatures, wear, SOC, fuel) live in a **separate** buffer sized at assembly and
advanced on the decimated slow clock (see [the integrator](integrator.md)).

## Assembler — phase order and topological sort

The assembler runs **once at load** (allocation is fine here) and produces a frozen, deterministic
`Schedule`. It fixes the global phase order

```
sense → control → actuate → integrate
```

then, within each phase, topologically sorts blocks so every intra-phase **writer precedes its
readers** (Kahn's algorithm). Cross-phase dependencies pointing *backwards* (a `sense`-phase reader
of an `integrate`-phase writer) are **one-step-lag** by design — they use the previous step's value
and impose no ordering constraint, which is exactly how the `fz_coupling: one_step_lag` normal-load
loop is closed. A genuine intra-phase write→read **cycle** is a hard `AssemblyError` — it must be
broken by a phase change or the one-step-lag path. Ties are broken by registration index, so the
schedule is **bit-deterministic**: identical inputs always yield the identical order.

After assembly the hot loop touches zero strings, hashes, or config logic — variety is paid for
entirely at load time (HANDOFF §6.2b).
