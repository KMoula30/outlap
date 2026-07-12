# Limebeer cross-check — the QSS tier vs Perantoni & Limebeer 2014 (Decision #48)

**Oracle.** G. Perantoni and D. J. N. Limebeer, *Optimal control for a Formula One car with
variable parameters*, Vehicle System Dynamics **52**(5), 653–678, 2014. Open-access manuscript:
Oxford University Research Archive, `uuid:ce1a7106-0a2c-41af-8449-41541220809f`. Published
results used here (all from the manuscript):

| Quantity | Value | Where |
|---|---|---|
| Optimal lap, Circuit de Catalunya (2 m grid) | **82.43 s** | §4.3 |
| Mesh-asymptotic optimal lap | 82.57 s | Fig. 11 |
| Speed trace (top speed ≈ 88 m/s; 16 corner apexes 17…62 m/s) | Fig. 8 | digitised: `data/pl2014_fig8_speed.csv` |
| Complete car parameter set | Tables 3–4, Appendix A | transcribed: `data/vehicles/limebeer_2014_f1` |
| Engine power (not stated in the manuscript) | 560 kW | Perantoni doctoral thesis (*Optimal control of vehicle systems*): "the peak engine power of 560 kW is capable of supporting a top speed of 85.4 m/s"; consistent with Fig. 8's ≈88 m/s via P = ½ρ·CdA·u³ |
| QSS-vs-OCP lap-time gap at Barcelona | 2.19 s | §1, citing its ref [14] (Brayshaw & Harrison 2005) |

**Consulted (clean-room policy):** `fastest-lap` (MIT, github.com/juanmanzanero/fastest-lap) was
read as a **parameterisation cross-check only** — its `limebeer-2014-f1.xml` transcribes
Tables 3–4 identically to ours. Its powertrain (735.5 kW + 120 kW boost) is that project's own
choice, so its published lap times are **not** comparable oracles. No code was taken.

## Configuration

`limebeer_2014_f1` (see its README for per-parameter provenance) on the OSM+DEM Catalunya import's
(`catalunya_osm`) min-curvature line, `sim.flat_track: true` (PL2014 is a 2-D study), production
40×25×7 envelope, ρ pinned to the paper's 1.2 kg/m³. Reproduce with:

```sh
cargo run --release -p outlap-qss --features parallel --example limebeer_lap
python python/tools/plot_limebeer.py
```

![Limebeer comparison](img/limebeer_catalunya.png)

## Gate results (Decision #48)

| Gate | Ours | PL2014 | Result |
|---|---|---|---|
| Top speed ≤ 1% | 87.8 m/s | ≈88 m/s (Fig. 8) | ✅ −0.2% |
| Slow-corner apex ≤ 5% | 17.7 m/s | 17 m/s (slowest, Fig. 8) | ✅ +4.1% |
| Fast-corner apexes ≤ 5% | 59.1 / 60.8 m/s | 60 / 60 / 62 m/s | ✅ −1.5% / −1.9% — **on the paper's own geometry** (below); on the committed OSM import the fast corners are geometry-corrupted and are not gated |
| Lap time | 92.36 s (committed track) / 87.08 s (paper's geometry) | 82.43 s | recorded, **not gated** (decomposition below) |

The CI test (`python/tests/test_limebeer.py`) gates what the committed track geometry supports:
top speed and the slowest-corner apex, on the `catalunya_osm` import. The fast-corner band was
validated against the paper's own centre-line curvature (extracted from the Fig. 6 vector data
during the 2026-07-06 analysis session; +5.64% lap time), and **stays deferred to M4**.

**Why the TUMFTM Catalunya did not turn the fast gate on (PR10).** PR10 vendored the TUMFTM
`racetrack-database` (an era-consistent, measured-width Catalunya was expected to unlock the
fast-corner gate). It does not: its centre line is a class-C **smoothed** layout that rounds the
slow chicane open and tightens the fast corners, so under QSS-on-min-curvature it reproduces
*neither* apex band — slowest apex **19.65 m/s (+15.6%)** (vs 17.7/+4.1% on `catalunya_osm`) and
fast apexes **57.0 / 58.4 m/s (−5.0% / −5.8%)**, with top speed still −0.15%. A corridor-width sweep
barely moves the slow apex (19.65 → 19.18 at an absurd 8 m car), confirming this is the
line-optimality residual (decomposition #2 below), **not** a width or import artefact. The
fast-corner gate therefore lands in M4 with the time-weighted raceline QP (Decision #48), which is
the machinery that closes this gap; the M3 cross-check remains on `catalunya_osm`.

## Lap-time decomposition — why the delta is structural, not a model error

A QSS solver on a fixed heuristic line **cannot** reproduce a transient optimal-control lap that
co-optimises the driven line; the delta decomposes as:

1. **QSS vs transient OCP, ~2.2 s** — the paper itself cites 2.19 s for exactly this circuit
   (its ref [14]).
2. **Line optimality** — the min-curvature line minimises ∫κ², not time; it systematically
   under-opens the medium-speed corners (30–50 m/s), which is precisely where the residual apex
   deficit lives once the geometry is controlled. A time-weighted raceline QP is scheduled for M4
   (Decision #48).
3. **Envelope conservatism, ~1–1.5 s** — the trim-feasibility boundary delivers 85–91% of the
   four-wheel point-mass ideal (legitimate double-track physics: load transfer with load-sensitive
   μ + yaw-moment balance on equal-μ axles).
4. **Track geometry** — the committed OSM import carries interpolation noise (spurious curvature
   spikes) and defaulted widths, and is the current (2021 T10, post-2023) layout vs the paper's
   2013 layout: worth ~5 pp of lap time here (92.36 → 87.08 s on the paper's own curvature).

What the cross-check **does** validate: the complete car transcription. Peak μ exact at all
loads, peak-slip locations within 0.5%, combined-slip coupling within ~5% of the paper's model,
the full longitudinal drive/brake chain overlaying the closed forms, top speed to −0.2%, and the
slow/fast corner speeds to ≤5% on like-for-like geometry.

## The T2 transient lap — recorded, not the ≤1% gate (M4)

The M4 ≤1% Limebeer lap-time gate was scoped behind the transient **T2** tier + the time-weighted
raceline QP. Both have landed; the gate is **not achievable at T2** and is recorded with a
decomposition rather than flipped (the Decision #48 pattern). Measured on `catalunya_osm`, flat,
production envelope (`python/tests/test_limebeer.py::test_limebeer_t2_lap_time_recorded_not_gated`):

| lap | time | vs OCP 82.43 s |
|---|---|---|
| OCP oracle (PL2014 §4.3) | 82.43 s | — |
| T0 QSS, min-curvature line | 92.36 s | +12.0% |
| T0 QSS, **time-weighted** line | 92.07 s | +11.7%  (line saves 0.29 s) |
| **T2 transient**, min-curvature line | 105.47 s | +28.0% |
| **T2 transient**, time-weighted line | 105.20 s | +27.6%  (line saves 0.27 s) |

The T2 lap is **~+28%** over the oracle — it does not approach ≤1%, and the gap is structural, not a
model error. It adds one T2-specific term to the four QSS components above:

5. **Driver stability margin, +13.1 s (the dominant term)** — the ideal MacAdam-preview + PI driver
   keeps a **corner-scaled** stability margin: it tracks the full QSS profile where lateral demand
   is low (top speed 310 vs 316 km/h — within 2%), ~0.85 of it where the profile rides the lateral
   grip limit, with friction-ellipse-aware braking/traction passes shaping the transitions, a
   sideslip damper catching translational slides, and a pedal governor holding drive wheelspin at
   the force peak (`outlap_qss::margin`, `docs/theory/driver.md`). Tracking the **raw** profile
   still **spins the car** — the QSS boundary is not filtered for open-loop stability — so the
   corner margin is the honest boundary of this driver. It is a driver-**competitiveness** limit at
   the limit, not a chassis or tyre error: the T2 operating points sit **inside** the T1 g-g-g-v
   hull (0.0% exceedance, the asserted parity gate).

An earlier *global* 0.85 margin (every station, straights included) measured +15.6 s; the
corner-scaled scheme recovered 2.4 s of it, almost all on the straights. Because the corner margin
alone is still ~+14% of T0, and the QSS floors (geometry ~5 pp, QSS-vs-OCP ~2.2 s, envelope
~1.5 s) account for the rest, the ≤1% assertion stays **deferred** — there is no paper-geometry
fixture committed, and even on ideal geometry the corner margin puts the gate well out of reach.
The recorded band (a wide tripwire around +20–45%) guards against silent drift; the honest number
is surfaced, never a green ≤1% that isn't real.

## Notes on the tyre transcription

The paper's Table 3 states the peak slips as κ = 0.11/0.10 and α = 9°/8°, but its own formula
(A.11–A.14, `S = π/(2·arctan Q)`) peaks at 0.756× those values. The transcription anchors the
MF6.1 peaks where the formula actually peaks, since the simulation is the validation target — see
`data/tires/limebeer_2014_f1/README.md` for the full derivation and the fitted combined-slip
coefficients.
