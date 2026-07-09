# SPDX-License-Identifier: AGPL-3.0-only
"""Symbolic derivation of the T2 7-DOF chassis EOM (Kane's method) + RHS fixture generator.

Decision #32 / HANDOFF §11.6: the chassis equations of motion are derived *independently* from the
hand-written Rust RHS (``crates/outlap-vehicle/src/chassis.rs``) with ``sympy.physics.mechanics`` and
the two are checked to agree to 1e-12 at randomised states/parameters/loads. This module is the
authoritative symbolic source; the CI python job re-executes the notebook that wraps it, regenerates
``fixtures/t2_chassis_rhs.json``, and ``git diff --exit-code``s the result, so the symbolic derivation
stays the single source of truth for the EOM signs.

Model (ISO 8855: x forward, y left, z up; SI):

* The **planar chassis** (mass ``m``, yaw inertia ``Izz``) is derived with ``KanesMethod`` using the
  body-frame velocity generalised speeds ``(v_x, v_y, r)``. Kane produces the transport (Coriolis)
  terms — the classic sign trap — from the kinematics, independently confirming
  ``m(v̇_x − r v_y) = ΣF_x``, ``m(v̇_y + r v_x) = ΣF_y``, ``Izz ṙ = ΣM_z``.
* The **four wheel spins** are decoupled 1-DOF rotors ``I_w ω̇ = τ − R F_x^w`` (gyroscopic
  spin×yaw coupling neglected, standard for vehicle-dynamics tiers; a T3 refinement).
* The **curvilinear kinematics** ``(ṡ, ṅ, ψ̇_rel)`` are the Frenet progress relations on the road
  reference line with plan-view curvature ``κ``.

``ΣF``/``ΣM`` assemble the four **wheel-frame** tyre forces rotated into the body frame by the
per-wheel steer, minus aero drag along ``+x``, plus the in-plane gravity projection (grade/banking
rotated by ``ψ_rel``) and the external yaw-moment demand — the same expressions the Rust chassis uses.
"""

from __future__ import annotations

import json
from pathlib import Path

import sympy as sp
from sympy.physics.mechanics import (
    KanesMethod,
    Point,
    ReferenceFrame,
    RigidBody,
    dynamicsymbols,
    inertia,
)

WHEELS = 4
N_SAMPLES = 64
FIXTURE = Path(__file__).with_name("fixtures") / "t2_chassis_rhs.json"


def derive_planar_accelerations():
    """Kane's-method planar-chassis acceleration RHS as callables of the symbolic inputs.

    Returns ``(v_x_dot, v_y_dot, r_dot)`` sympy expressions in terms of the generalised speeds
    ``(vx, vy, r)`` and the applied body-frame resultant ``(Fx, Fy, Mz)`` plus mass/inertia.
    """
    t = sp.Symbol("t")
    # Configuration: inertial CG position (qx, qy) + yaw (qpsi).
    qx, qy, qpsi = dynamicsymbols("qx qy qpsi")
    # Body-frame velocity generalised speeds + yaw rate.
    vx, vy, r = dynamicsymbols("vx vy r")
    m, izz = sp.symbols("m izz", positive=True)
    Fx, Fy, Mz = sp.symbols("Fx Fy Mz", real=True)

    N = ReferenceFrame("N")
    B = N.orientnew("B", "Axis", [qpsi, N.z])

    O = Point("O")
    # CG velocity is the body-frame speeds expressed in N.
    O.set_vel(N, vx * B.x + vy * B.y)

    # Kinematic differential equations relating q̇ to the generalised speeds.
    kd = [
        qx.diff(t) - (vx * sp.cos(qpsi) - vy * sp.sin(qpsi)),
        qy.diff(t) - (vx * sp.sin(qpsi) + vy * sp.cos(qpsi)),
        qpsi.diff(t) - r,
    ]

    body = RigidBody("chassis", O, B, m, (inertia(B, 0, 0, izz), O))
    loads = [(O, Fx * B.x + Fy * B.y), (B, Mz * B.z)]

    km = KanesMethod(N, q_ind=[qx, qy, qpsi], u_ind=[vx, vy, r], kd_eqs=kd)
    km.kanes_equations([body], loads)

    udot = km.mass_matrix.inv() * km.forcing
    # udot is [v_x_dot, v_y_dot, r_dot]; simplify to expose the transport terms.
    return tuple(sp.simplify(udot[i]) for i in range(3)), (vx, vy, r, m, izz, Fx, Fy, Mz)


def build_rhs_lambda():
    """Assemble the full 10-state RHS and lambdify it over the flat input vector."""
    (vxd, vyd, rd), (vx, vy, r, m, izz, Fx, Fy, Mz) = derive_planar_accelerations()

    # Confirm Kane reproduced the expected planar EOM (fails loudly on a sign regression).
    assert sp.simplify(vxd - (Fx / m + r * vy)) == 0, vxd
    assert sp.simplify(vyd - (Fy / m - r * vx)) == 0, vyd
    assert sp.simplify(rd - (Mz / izz)) == 0, rd

    # --- symbols for the full model ---
    n, psi = sp.symbols("n psi", real=True)
    kappa, grade, bank = sp.symbols("kappa grade bank", real=True)
    g = sp.Symbol("g", positive=True)
    drag = sp.Symbol("drag", real=True)
    dmz = sp.Symbol("dmz", real=True)  # yaw-moment demand
    steer = sp.Symbol("steer", real=True)
    xw = sp.symbols(f"x0:{WHEELS}", real=True)
    yw = sp.symbols(f"y0:{WHEELS}", real=True)
    Rw = sp.symbols(f"R0:{WHEELS}", positive=True)
    Iw = sp.symbols(f"Iw0:{WHEELS}", positive=True)
    front = [True, True, False, False]
    fxw = sp.symbols(f"fxw0:{WHEELS}", real=True)
    fyw = sp.symbols(f"fyw0:{WHEELS}", real=True)
    mzw = sp.symbols(f"mzw0:{WHEELS}", real=True)
    tau = sp.symbols(f"tau0:{WHEELS}", real=True)
    omega = sp.symbols(f"omega0:{WHEELS}", real=True)

    # --- assemble ΣF, ΣM via reference frames (NOT the hand-written scalar formula) ---
    # The per-wheel force rotation (wheel→body by the steer δ) and the yaw moment (r × F) are derived
    # from a steered wheel frame and a cross product, so a sign error in the Rust chassis's scalar
    # `fxb = fxw·cosδ − fyw·sinδ` / `Mz = x·fyb − y·fxb` would surface as a 1e-12 mismatch — this is
    # the independent check for the assembly signs (the transport terms are Kane-derived above).
    from sympy.physics.vector import ReferenceFrame, cross, dot

    body = ReferenceFrame("body")
    sum_force = 0 * body.x
    sum_moment = 0 * body.z
    for i in range(WHEELS):
        d = steer if front[i] else sp.Integer(0)
        wheel = body.orientnew(f"wheel{i}", "Axis", [d, body.z])  # steered wheel frame
        force = fxw[i] * wheel.x + fyw[i] * wheel.y
        arm = xw[i] * body.x + yw[i] * body.y
        sum_force += force
        sum_moment += cross(arm, force) + mzw[i] * body.z
    # Aero drag along −x; in-plane gravity from grade/banking, rotated into the body frame by ψ_rel
    # via a road frame (independent of the hand-written cos/sin projection).
    road = ReferenceFrame("road")
    body_from_road = road.orientnew("body_from_road", "Axis", [psi, road.z])
    mg = m * g
    gravity = (-mg * sp.sin(grade)) * road.x + (-mg * sp.sin(bank)) * road.y
    sum_fx = dot(sum_force, body.x) - drag + dot(gravity, body_from_road.x)
    sum_fy = dot(sum_force, body.y) + dot(gravity, body_from_road.y)
    sum_mz = dot(sum_moment, body.z) + dmz
    cpsi, spsi = sp.cos(psi), sp.sin(psi)

    # Substitute the resultant into the Kane accelerations.
    subs = {Fx: sum_fx, Fy: sum_fy, Mz: sum_mz}
    vx_dot = vxd.subs(subs)
    vy_dot = vyd.subs(subs)
    r_dot = rd.subs(subs)

    # Wheel spins (decoupled rotors) + curvilinear kinematics.
    omega_dot = [(tau[i] - Rw[i] * fxw[i]) / Iw[i] for i in range(WHEELS)]
    denom = 1 - n * kappa
    s_dot = (vx * cpsi - vy * spsi) / denom
    n_dot = vx * spsi + vy * cpsi
    psi_dot = r - kappa * s_dot

    rhs = [s_dot, n_dot, psi_dot, vx_dot, vy_dot, r_dot, *omega_dot]

    # Flat input vector order (must match `sample_inputs` and the Rust fixture loader).
    args = [
        n, psi, vx, vy, r, *omega,
        m, izz, g, *xw, *yw, *Rw, *Iw,
        steer, drag, dmz, kappa, grade, bank,
        *fxw, *fyw, *mzw, *tau,
    ]
    fn = sp.lambdify(args, rhs, modules="math")
    return fn, [str(a) for a in args]


def sample_inputs(rng):
    """One physically-plausible random input dict (values are plain floats)."""
    import numpy as np

    def u(a, b, n=1):
        v = rng.uniform(a, b, n)
        return v.tolist() if n > 1 else float(v[0])

    return {
        "n": u(-2.0, 2.0),
        "psi": u(-0.4, 0.4),
        "vx": u(10.0, 80.0),
        "vy": u(-4.0, 4.0),
        "r": u(-1.0, 1.0),
        "omega": u(20.0, 120.0, WHEELS),
        "m": u(700.0, 1600.0),
        "izz": u(700.0, 2500.0),
        "g": 9.80665,
        "x": [u(1.0, 1.8), u(1.0, 1.8), u(-1.8, -1.0), u(-1.8, -1.0)],
        "y": [u(0.6, 0.9), u(-0.9, -0.6), u(0.6, 0.9), u(-0.9, -0.6)],
        "R": u(0.25, 0.36, WHEELS),
        "Iw": u(0.8, 1.8, WHEELS),
        "steer": u(-0.25, 0.25),
        "drag": u(0.0, 6000.0),
        "dmz": u(-500.0, 500.0),
        "kappa": u(-0.02, 0.02),
        "grade": u(-0.08, 0.08),
        "bank": u(-0.12, 0.12),
        "fxw": u(-4000.0, 4000.0, WHEELS),
        "fyw": u(-6000.0, 6000.0, WHEELS),
        "mzw": u(-120.0, 120.0, WHEELS),
        "tau": u(-1500.0, 1500.0, WHEELS),
    }


def flat_args(inp):
    """Flatten a sample dict into the positional arg order of `build_rhs_lambda`."""
    return [
        inp["n"], inp["psi"], inp["vx"], inp["vy"], inp["r"], *inp["omega"],
        inp["m"], inp["izz"], inp["g"], *inp["x"], *inp["y"], *inp["R"], *inp["Iw"],
        inp["steer"], inp["drag"], inp["dmz"], inp["kappa"], inp["grade"], inp["bank"],
        *inp["fxw"], *inp["fyw"], *inp["mzw"], *inp["tau"],
    ]


def generate():
    """Derive the EOM, sample states, and write the committed RHS fixture."""
    import numpy as np

    fn, arg_names = build_rhs_lambda()
    rng = np.random.default_rng(20260709)
    samples = []
    for _ in range(N_SAMPLES):
        inp = sample_inputs(rng)
        rhs = fn(*flat_args(inp))
        samples.append({"inputs": inp, "rhs": [float(v) for v in rhs]})

    doc = {
        "_comment": "Auto-generated by docs/derivations/t2_chassis_kane.ipynb (Decision #32). "
        "Do not edit by hand; re-run the notebook. Checked by the Rust 1e-12 test "
        "outlap-vehicle/tests/kane_fixture.rs.",
        "state_order": ["s", "n", "psi_rel", "vx", "vy", "r", "w_fl", "w_fr", "w_rl", "w_rr"],
        "arg_order": arg_names,
        "front": [True, True, False, False],
        "samples": samples,
    }
    FIXTURE.parent.mkdir(parents=True, exist_ok=True)
    FIXTURE.write_text(json.dumps(doc, indent=1) + "\n")
    return len(samples)


if __name__ == "__main__":
    count = generate()
    print(f"wrote {count} samples to {FIXTURE}")
