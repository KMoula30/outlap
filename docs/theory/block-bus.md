<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Block, Bus, and SoA: the transient scaffolding

`outlap-core` holds the data-flow scaffolding that every transient tier is assembled on. That is
T2 and T3. It has four parts: the **Block** abstraction, the flat struct-of-arrays signal **Bus**,
the frozen **state registry**, and the **assembler**, which sorts topologically.

The crate is `wasm`-clean: it uses no filesystem, no threads, and no clock. It carries no
`sim.yaml` types and no schema types. The assembly pipeline maps configuration onto this layer
(HANDOFF §6.2b). That mapping never happens in the loop.

The design comes from HANDOFF §6.2, and from three Locked Decisions: #26, which composes at run
time and dispatches through an enum, with no `dyn` in the loop; #38, which makes controllers
built-in blocks and forbids Python inside a timestep; and #39, which defines a hybrid signal bus of
fixed core indices plus interned dynamic channels.

## Block

A **block** is three things: immutable parameters, states, and typed ports. Its trait exposes three
pure evaluations, each generic over `f32` and `f64`, and each matching a tier:

```rust
trait Block<T: Float> {
    fn phase(&self) -> Phase;                 // sense | control | actuate | integrate
    fn ports(&self) -> Ports;                 // bus channels read / written (static)
    fn equilibrium(&self, bus, slow, lane);   // T0/T1 algebraic trim contribution
    fn derivatives(&self, x, bus, dx, lane);  // T2/T3 fast-state RHS
    fn slow_derivatives(&self, bus, dslow, lane); // thermal / wear / SOC on the slow clock
}
```

Blocks run **on one lane at a time**. The caller binds the SoA views to a lane, then passes that
same `lane` to the bus accessors.

In the hot loop, the code reaches a concrete block through the `CoreBlock` **enum**. It never uses
a trait object. Later PRs add physics blocks and controller blocks as variants: Chassis, Tire×4,
Aero, Driver, TV, and others.

The external plugin trait is deferred (Decision #38). The M4 controllers are built-in enum
variants. M4 ships one variant: a **stubbed suspension block**, which reserves the T3 slot and its
port surface but contributes no dynamics.

## Bus: the signal board

Blocks never talk to each other directly. They publish to a shared **Bus** of typed scalar
channels, and they consume from it. The bus has two regions.

* A **fixed core set**, with indices known at compile time. These are the signals that every
  built-in T2 block exchanges.
* An **interned dynamic region**, for named channels from plugins and custom blocks. A
  `ChannelInterner` resolves each name to an integer `ChannelId` **once, at assembly**. The hot loop
  sees only indices. It never sees a string, and it never sees a hash (Decision #39).

Every channel carries an explicit **batch dimension**. The layout is SoA and state-major: channel
`c` and lane `b` live at `c·batch + b`. One channel is therefore contiguous across the batch. That
makes it transposable to a GPU (HANDOFF §11.3), and friendly to the cache in the rayon batch loop.

Construction allocates. Access does not, and a dhat test gates that in CI.

## A note on the frozen layout

This layer freezes two index layouts. They are an internal contract. Downstream code addresses them
through the enums below, never through bare integers. Additions append. They never reorder.

**The fixed bus channels** are the `CoreSignal` scalars, plus the per-wheel groups of
`WheelSignal`. `WHEELS = 4`, in ISO 8855 order: FL, FR, RL, RR.

| Region | Channels |
|--------|----------|
| Scalar (`CoreSignal`) | `Steer`, `Throttle`, `Brake`, `DriveTorque`, `YawMomentDemand`, `AeroDrag`, `AeroFzFront`, `AeroFzRear` |
| Per-wheel (`WheelSignal`, ×4 each) | `TireFx`, `TireFy`, `TireFz`, `TireMz`, `SlipKappa`, `SlipAlpha`, `SlipKappaSs`, `SlipAlphaSs`, `WheelDriveTorque`, `WheelBrakeTorque` |

**The fast-state registry** holds `[chassis | relaxation]`. The chassis region reserves the full
**14-DOF** footprint, which lays the groundwork for T3 without breaking the layout later. T2
integrates only the first ten slots. The relaxation region holds one lagged `κ` and one lagged `α`
for each wheel.

| Region | Slots | Tier |
|--------|-------|------|
| T2 chassis (`ChassisState`) | `s, n, ψ_rel, vx, vy, r, ω₁..₄` (10) | **T2** integrated |
| T3-reserved chassis | heave/pitch/roll + rates (6), four unsprung z + rates (8) | reserved (reads 0 in M4) |
| Relaxation (`RelaxState`, ×4) | lagged `κ`, `α` | populated in PR4 |

Slow states — temperatures, wear, SOC, and fuel — live in a **separate** buffer. Assembly sizes it,
and the decimated slow clock advances it. See [the integrator](integrator.md).

## The assembler: phase order and topological sort

The assembler runs **once, at load**, where allocation is acceptable. It produces a `Schedule` that
is frozen and deterministic.

First it fixes the global phase order:

```
sense → control → actuate → integrate
```

Then, within each phase, it sorts the blocks topologically with Kahn's algorithm, so that every
writer inside a phase precedes its readers.

A cross-phase dependency that points *backward* is **one-step-lag** by design. An example is a
reader in the `sense` phase that depends on a writer in the `integrate` phase. Such a dependency
uses the value from the previous step, and it imposes no ordering constraint. This is exactly how
the normal-load loop closes under `fz_coupling: one_step_lag`.

A genuine write-to-read **cycle** inside one phase is a hard `AssemblyError`. Break it with a phase
change, or with the one-step-lag path.

Ties break by registration index. The schedule is therefore **bit-deterministic**: identical inputs
always produce an identical order.

After assembly, the hot loop touches no strings, no hashes, and no configuration logic. Load time
pays for all the variety (HANDOFF §6.2b).
