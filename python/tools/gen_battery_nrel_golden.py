# SPDX-License-Identifier: AGPL-3.0-only
"""Generate battery terminal-voltage reference traces with the NREL ``thevenin`` package.

**Opt-in tool — NOT a runtime or build dependency.** It is run by hand, once, to (re)generate the
committed golden CSVs under ``crates/outlap-qss/tests/golden/battery_nrel/``; CI only reads those
CSVs (the MFeval / tire-golden pattern, ``tools/goldens/README.md``). The consuming Rust test
``crates/outlap-qss/tests/battery_nrel.rs`` drives outlap's own :class:`Pack` with the *same* ECM
parameters (carried in each CSV header — a single source of truth) and asserts the terminal-voltage
RMS ≤ 1 % of the reference (§13 M6 battery row; D-M6-3).

Oracle: NREL ``thevenin`` (BSD-3, github.com/NREL/thevenin) — an equivalent-circuit Thevenin/RC
battery model. Consumed as DATA only; never vendored or ported (Hard rule #2, §8.4). The model
solves ``V(t) = OCV(SoC) − I·R0 − Σ_k I·R_k·(1 − e^{−t/τ_k})`` with Coulomb-counted SoC — the same
published form outlap's pack integrates exactly (Plett, *Battery Management Systems* Vol. 1).

Protocol (D-M6, "Resolved by default"): HPPC-style pulses at ±1C and ±2C, 10 s on / 60 s rest, at a
"cold" (25 °C) and a "warm" (40 °C) operating point, for both RC counts. The comparison is isothermal
and the ECM parameters are constant in SoC over the small pulse excursion, so the two temperatures are
two distinct (physically colder ⇒ higher-resistance) parameter sets — exactly what an HPPC campaign
at two temperatures yields.

Run (from the repo root, in a venv with ``thevenin`` + ``numpy`` installed — never the project venv)::

    python python/tools/gen_battery_nrel_golden.py
"""

from __future__ import annotations

import warnings
from pathlib import Path

import numpy as np
import thevenin as thev

warnings.filterwarnings(
    "ignore"
)  # thevenin's default-file / deprecation notices are noise here.

OUT = (
    Path(__file__).resolve().parents[2] / "crates/outlap-qss/tests/golden/battery_nrel"
)

CAPACITY_AH = 5.0  # a single reference cell; 1C = 5 A, 2C = 10 A.
SOC0 = 0.6  # mid-band, so the ± pulse train stays inside the window.
OCV_V = 3.60  # constant OCV over the small (~0.006) SoC excursion of the pulse train.
DT_S = (
    0.5  # output cadence — well under τ, so both integrators resolve the RC relaxation.
)

# Two operating points. Colder ⇒ higher ohmic + polarisation resistance and slower relaxation, the
# ordering every Li-ion HPPC campaign shows. R in ohm, τ in s (C = τ/R inside the NREL model).
POINTS = {
    "cold25": {
        "temp_c": 25.0,
        "r0": 0.028,
        "r1": 0.014,
        "tau1": 18.0,
        "r2": 0.009,
        "tau2": 55.0,
    },
    "warm40": {
        "temp_c": 40.0,
        "r0": 0.015,
        "r1": 0.008,
        "tau1": 12.0,
        "r2": 0.005,
        "tau2": 45.0,
    },
}

# ±2C then ±1C, each 10 s on / 60 s rest (discharge positive, charge negative — outlap's convention,
# which the NREL current_A mode shares).
PULSES = [
    (10.0, 10.0),
    (0.0, 60.0),
    (-10.0, 10.0),
    (0.0, 60.0),
    (5.0, 10.0),
    (0.0, 60.0),
    (-5.0, 10.0),
    (0.0, 60.0),
]


def _params(pt: dict[str, float], rc_pairs: int) -> dict:
    """Build the NREL ``thevenin`` parameter dict for one operating point + RC count."""
    p = {
        "num_RC_pairs": rc_pairs,
        "soc0": SOC0,
        "capacity": CAPACITY_AH,
        "gamma": 0.0,
        "ce": 1.0,
        "mass": 1.0,
        "isothermal": True,
        "Cp": 745.0,
        "T_inf": 273.15 + pt["temp_c"],
        "h_therm": 0.0,
        "A_therm": 0.0,
        "ocv": lambda soc: OCV_V,
        "M_hyst": lambda soc: 0.0,
        "R0": lambda soc, T: pt["r0"],
        "R1": lambda soc, T: pt["r1"],
        "C1": lambda soc, T: pt["tau1"] / pt["r1"],
    }
    if rc_pairs == 2:
        p["R2"] = lambda soc, T: pt["r2"]
        p["C2"] = lambda soc, T: pt["tau2"] / pt["r2"]
    return p


def _run(
    pt: dict[str, float], rc_pairs: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sim = thev.Simulation(_params(pt, rc_pairs))
    exp = thev.Experiment()
    for amps, dur in PULSES:
        exp.add_step("current_A", amps, (dur, DT_S))
    sol = sim.run(exp)
    return sol.vars["time_s"], sol.vars["current_A"], sol.vars["voltage_V"]


def _write(name: str, pt: dict[str, float], rc_pairs: int) -> None:
    t, i, v = _run(pt, rc_pairs)
    path = OUT / f"{name}.csv"
    r2 = pt["r2"] if rc_pairs == 2 else 0.0
    tau2 = pt["tau2"] if rc_pairs == 2 else 0.0
    lines = [
        "# outlap battery ECM golden — terminal voltage vs NREL thevenin (BSD-3).",
        f"# oracle: NREL thevenin v{thev.__version__} (isothermal, constant-in-SoC ECM).",
        "# generator: python/tools/gen_battery_nrel_golden.py — DATA only, never a runtime dep.",
        "# The Rust consumer builds a Pack from THESE parameters (single source of truth) and",
        "# asserts terminal-voltage RMS <= 1% of terminal_v_ref (M6 PR8 gate #1, D-M6-3).",
        f"# param ocv_v: {OCV_V}",
        f"# param r0_ohm: {pt['r0']}",
        f"# param r1_ohm: {pt['r1']}",
        f"# param tau1_s: {pt['tau1']}",
        f"# param r2_ohm: {r2}",
        f"# param tau2_s: {tau2}",
        f"# param rc_pairs: {rc_pairs}",
        f"# param capacity_ah: {CAPACITY_AH}",
        f"# param soc0: {SOC0}",
        f"# param temp_c: {pt['temp_c']}",
        f"# param dt_s: {DT_S}",
        "t_s,current_a,terminal_v_ref",
    ]
    for tk, ik, vk in zip(t, i, v, strict=True):
        lines.append(f"{tk:.6f},{ik:.6f},{vk:.9f}")
    path.write_text("\n".join(lines) + "\n")
    print(f"wrote {path.relative_to(OUT.parents[3])} ({len(t)} rows)")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for name, pt in POINTS.items():
        for rc in (1, 2):
            _write(f"{name}_rc{rc}", pt, rc)


if __name__ == "__main__":
    main()
