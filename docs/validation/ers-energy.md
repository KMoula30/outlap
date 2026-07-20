# ERS energy cross-check — stint SoC carry + T0↔T2 energy parity (Decision #48)

**Oracle.** The FIA 2026 regulatory mechanisms (Section C, Issue 19) that the flagship ERS energy
manager implements — the ±350 kW power caps (C5.2.7), the 4 MJ usable window (C5.2.9), the 8.5 MJ/lap
Recharge budget (C5.2.10) — plus the exact energy-accounting identities:

| Quantity | Identity | Where |
|---|---|---|
| Charge continuity | `SoC(lap k+1, s=0) = SoC(lap k, s=end)` | QSS stint carry (M6/PR3) |
| Coulomb closure | `ΔSoC = −∫I dt / Q` (exact) | `crates/outlap-qss/tests/stint.rs` |
| Fuel energy/lap | `E_fuel = (m₀ − m_end)·LHV` | §8.1 |
| ERS net energy/lap | `E_net = ∫(P_deploy − P_harvest) dt` | §8.3 |

**Consulted (clean-room policy):** none — the energy manager is an outlap flagship implemented from
the FIA regulations (`docs/theory/ers-energy-manager.md`); this page validates its outputs.

## Gate #2 — stint SoC carry (the author's acceptance check)

A **10-lap** f1_2026 stint in **both** tiers, seeded with the **same explicit `initial_soc`**
(`python/tests/test_stint_soc.py::test_stint_soc_10lap_both_tiers_consumption_and_regeneration`;
exact Coulomb closure + the per-lap ledger in `crates/outlap-qss/tests/stint.rs`).

| Property | Result |
|---|---|
| Continuous across QSS lap boundaries (no reset) | ✅ asserted |
| Never re-seeded per lap (carries the physics state) | ✅ asserted (both tiers) |
| Consumption AND regeneration act every lap | ✅ asserted (deploy+harvest ledgers, within-lap swing) |
| Decreases under net consumption | ✅ asserted (lap 1 ends below the seed, both tiers) |
| Exact charge closure ≤ 1e-9 relative | ✅ asserted (Rust, Coulomb counting is exact) |

f1_2026 is a hard-deploying, **SoC-starved** car: with the greedy feed-forward manager (D-M6-8) it
cycles the full 4 MJ window every lap and charge-sustains at the floor. So "consumption AND
regeneration" is the full within-lap 0.2↔0.9 swing, and the carry is the pack sitting at the
physics-driven floor from lap 2 on — **not** the mid-window seed a re-seeding stint would show. The
honest net-consumption signal is the SoC **state**, not the ledger: a pack rejects harvest at the
window ceiling, so the *attempted*-harvest ledger can exceed deploy while the pack still net-drains.

## Gate #3 — parity gate #4 (fuel + ERS energy per lap, T0 vs T2)

f1_2026, smooth, frozen tyres, shared `initial_soc`
(`python/tests/test_parity.py::test_energy_parity_gate4`). PRE-AUTHORIZED assert-OR-record (D-M6-11).

| Quantity | T0 | T2 | Δ | Result |
|---|---|---|---|---|
| Harvest energy / lap | 8.50 MJ | 8.50 MJ | **+0.04 %** | ✅ **asserted ≤ 1 %** |
| Deploy energy / lap | 9.88 MJ | 8.40 MJ | +15.0 % | recorded (driver margin) |
| Fuel burned / lap | 1.60 kg | 1.13 kg | +29.3 % | recorded (driver margin) |

**The asserted part is the shared rule.** Both tiers consume the *same* `outlap-powertrain` rulebook,
so the pure-rule quantity — the per-lap Recharge budget — agrees to **≤ 1 %**. That agreement is the
evidence the energy accounting itself is sound.

**Decomposition of the deploy + fuel residuals (recorded, not gated).** Both are dominated by the T2
driver corner margin (+14–17 %, the named residual — `docs/validation/limebeer.md`): the ideal
MacAdam-preview + PI driver runs a slower, lower-throttle line, so it deploys the MGU-K less **and**
burns less fuel per lap. That is a driver-competitiveness gap, not an energy-accounting error — the
harvest agreement isolates the two. This is separate from the M5 f1_2026 T2 stint-decay caveat
(0.16 s/lap on tyres), which is a tyre effect, not an energy effect. A wide tripwire (≤ 45 %) guards
against a wiring regression.

## Recorded limitation — the QSS↔T2 EV-stint asymmetry

For a **mapped EV** (no energy manager), the QSS stint re-seeds the machine-thermal network every lap
(a distance-march derate↔slowdown feedback would otherwise run away without inter-lap cooling), while
T2 integrates it continuously. So on a long EV stint the QSS pack can derate where T2 does not — a
tier asymmetry, recorded here (expanding the forward-reference in
`docs/validation/qss_stint_soc/README.md`). It affects only mapped EVs: under an ERS manager the
machine is not marched at all (D-M6-10), so f1_2026 — gate #2's car — is unaffected.
