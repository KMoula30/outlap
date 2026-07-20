# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the CommonRoad "vehicle 2" (BMW 320i) handling oracle for the M6 PR8 chassis gate #4.

**Opt-in tool — NOT a runtime or build dependency.** Run by hand, once, to (re)generate the committed
goldens under ``crates/outlap-transient/tests/golden/bmw320i/``; CI only reads those CSVs (the
MFeval / tire-golden pattern). The consuming Rust test ``crates/outlap-transient/tests/handling.rs``
runs outlap's T3 14-DOF chassis on the ``data/vehicles/bmw320i`` car (whose brush tyres are matched to
the cornering stiffness below) and compares against these numbers.

Oracle: the CommonRoad vehicle models (M. Althoff et al., TUM, BSD-3), ``parameters_vehicle2`` +
``vehicle_dynamics_st`` (the single-track / bicycle model). Consumed as DATA only; never vendored.
The ST model uses an axle cornering stiffness ``C_alpha = mu · C_S · F_z`` with
``C_S = -p_ky1/p_dy1`` and ``mu = p_dy1``, i.e. ``C_alpha = -p_ky1 · F_z`` — equal front/rear
coefficients, so the car is NEUTRAL-steer (understeer gradient K ≈ 0) and the analytic yaw-rate gain
is simply ``r/δ = V/L`` (Gillespie, *Fundamentals of Vehicle Dynamics*). Two goldens are produced:

  * ``metrics.csv`` — the analytic single-track oracle (C_alpha, understeer gradient, yaw-rate gain
    vs speed, characteristic speed) plus the ST model's own steady step-steer yaw-rate gain.
  * ``step_steer.csv`` — the ST model's transient yaw-rate response to a small step steer at a
    reference speed (the transient rise-time reference).

Run (from the repo root, in a venv with ``vehiclemodels`` + ``numpy`` + ``scipy`` installed)::

    PYTHONPATH=<site-packages>/PYTHON python python/tools/gen_bmw320i_golden.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st

OUT = (
    Path(__file__).resolve().parents[2] / "crates/outlap-transient/tests/golden/bmw320i"
)

G = 9.81
V_REF = 25.0  # step-steer reference speed, m/s
DELTA = 0.02  # step steer angle, rad (~1.15°, deep in the linear regime)
SPEEDS = [15.0, 20.0, 25.0, 30.0]  # yaw-rate-gain sweep speeds, m/s


def _oracle() -> dict[str, float]:
    p = parameters_vehicle2()
    m, a, b, length = p.m, p.a, p.b, p.a + p.b
    fzf = m * G * b / length
    fzr = m * G * a / length
    caf = -p.tire.p_ky1 * fzf  # axle cornering stiffness, N/rad
    car = -p.tire.p_ky1 * fzr
    k = (m / length) * (
        b / caf - a / car
    )  # understeer gradient, rad·s²/m (≈ 0, neutral)
    return {"m": m, "a": a, "b": b, "L": length, "caf": caf, "car": car, "k": k}


def _run_st_step(p, v0: float, delta: float) -> tuple[np.ndarray, np.ndarray]:
    """Integrate the CommonRoad ST model: ramp the front steer to ``delta`` then hold, at constant
    speed (accel = 0; the bicycle model has no drag). Returns (time, yaw_rate)."""
    ramp = 0.10  # s to reach the steer angle
    steer_rate = delta / ramp

    def u_of_t(t: float) -> list[float]:
        return [steer_rate if t < ramp else 0.0, 0.0]

    def rhs(t, x):
        return vehicle_dynamics_st(x, u_of_t(t), p)

    x0 = [0.0, 0.0, 0.0, v0, 0.0, 0.0, 0.0]  # [x, y, δ, v, ψ, ψ̇, β]
    t_eval = np.arange(0.0, 3.0 + 1e-9, 0.01)
    sol = solve_ivp(
        rhs, (0.0, 3.0), x0, t_eval=t_eval, rtol=1e-9, atol=1e-11, method="RK45"
    )
    return sol.t, sol.y[5]  # yaw rate is state index 5


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    o = _oracle()
    p = parameters_vehicle2()

    # Transient step-steer golden at V_REF.
    t, yaw = _run_st_step(p, V_REF, DELTA)
    st_gain = float(
        yaw[-1] / DELTA
    )  # steady-state yaw-rate gain from the ST model itself
    step_lines = [
        "# outlap chassis 14-DOF golden — CommonRoad ST step-steer yaw-rate response (BSD-3).",
        "# oracle: CommonRoad vehicle models (Althoff et al., TUM) parameters_vehicle2 + "
        "vehicle_dynamics_st.",
        "# generator: python/tools/gen_bmw320i_golden.py — DATA only, never a runtime dep.",
        f"# step steer {DELTA} rad at {V_REF} m/s; constant speed (bicycle model, no drag).",
        "t_s,yaw_rate_rad_s",
    ]
    step_lines += [f"{tk:.4f},{yk:.9f}" for tk, yk in zip(t, yaw, strict=True)]
    (OUT / "step_steer.csv").write_text("\n".join(step_lines) + "\n")

    # Analytic single-track oracle + the ST steady gain (metrics.csv).
    lines = [
        "# outlap chassis 14-DOF handling oracle — analytic single-track from CommonRoad vehicle 2.",
        "# oracle: CommonRoad vehicle models (Althoff et al., TUM, BSD-3), parameters_vehicle2.",
        "# generator: python/tools/gen_bmw320i_golden.py — DATA only, never a runtime dep.",
        "# The BMW 320i ST model uses equal front/rear cornering-stiffness coefficients, so the car",
        "# is NEUTRAL-steer: understeer gradient K ~= 0 and yaw-rate gain r/delta = V/L (Gillespie).",
        "key,value",
        f"mass_kg,{o['m']:.4f}",
        f"a_front_m,{o['a']:.6f}",
        f"b_rear_m,{o['b']:.6f}",
        f"wheelbase_m,{o['L']:.6f}",
        f"cornering_stiffness_front_n_per_rad,{o['caf']:.3f}",
        f"cornering_stiffness_rear_n_per_rad,{o['car']:.3f}",
        f"understeer_gradient_rad_s2_per_m,{o['k']:.9f}",
        f"step_delta_rad,{DELTA}",
        f"step_speed_mps,{V_REF}",
        f"st_step_steady_yaw_gain,{st_gain:.6f}",
    ]
    for v in SPEEDS:
        gain = v / (o["L"] + o["k"] * v * v)  # = V/L for the neutral car
        lines.append(f"yaw_rate_gain_at_{int(v)}mps,{gain:.6f}")
    (OUT / "metrics.csv").write_text("\n".join(lines) + "\n")

    print(f"wrote {OUT}/metrics.csv and step_steer.csv")
    print(
        f"  Caf={o['caf']:.0f} Car={o['car']:.0f} N/rad  K={o['k']:.2e} rad·s²/m (neutral)"
    )
    print(
        f"  analytic yaw gain @ {V_REF} m/s = {V_REF / o['L']:.4f}; ST steady = {st_gain:.4f} rad/s/rad"
    )


if __name__ == "__main__":
    main()
