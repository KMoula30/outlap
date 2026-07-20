# SPDX-License-Identifier: AGPL-3.0-only
"""Symbolic derivation of the T3 14-DOF chassis EOM + RHS fixture generator (Decision #32).

This extends ``t2_chassis_kane.py`` (the 7-DOF planar chassis) to the full T3 tier: sprung
heave/pitch/roll + four unsprung verticals + nonlinear suspension (spring / bump-rebound damper /
C¹ bumpstop / anti-roll bar) + tyre vertical springs, with the two refinement terms the T2
derivation flagged as out of scope (user-locked to land here): the **gyroscopic wheel spin×yaw
coupling** and the **3-D frame-transport** vertical-curvature term. As with T2 the symbolic
derivation is the authoritative source of the EOM signs; the CI python job re-executes the wrapping
notebook, regenerates ``fixtures/t3_chassis_rhs.json``, and ``git diff --exit-code``s it, and the
Rust 1e-12 test ``outlap-vehicle/tests/kane_fixture_t3.rs`` checks the hand-written RHS against it.

Model (ISO 8855: x forward, y left, z up; SI). The 24 chassis states, in the frozen
``ChassisState`` order:

    [ s, n, psi_rel | vx, vy, r | w_fl, w_fr, w_rl, w_rr
      | z, theta, phi | zdot, thetadot, phidot | zu_fl..rr | zudot_fl..rr ]

* **Handling** ``(vx, vy, r)`` — whole car (total mass ``m``, whole-car yaw inertia ``Izz``); the
  SAME planar EOM the T2 chassis integrates (Kane transport ``r*vy`` / ``r*vx``) PLUS the gyroscopic
  yaw moment from the four spinning wheels. Reads the wheel-frame tyre forces off the bus.
* **Ride** ``(z, theta, phi)`` — sprung body (mass ``m_s``; sprung roll/pitch inertia ``Ixx``/``Iyy``
  about the sprung CG at height ``h_s``). Diagonal mass matrix by construction (CG-referenced,
  diagonal inertia); all coupling is on the forcing side: the suspension corner forces (ELASTIC
  transfer through the springs), the sprung-mass inertial reaction to the handling accelerations
  (roll about the roll axis ``h_s−h_ra``; pitch about ``h_s`` reduced by the mean anti fraction),
  the gyroscopic roll/pitch reaction, and the vertical-curvature transport ``kappa_v*vx²``.
* **Unsprung** ``zu_i`` — four 1-DOF masses; ``m_u zddot = F_tyre − F_susp − m_u g_n + F_geom``,
  where ``F_geom`` is the GEOMETRIC load transfer routed straight to the contact patch (roll-centre
  height for lateral, anti-dive/squat for longitudinal — §7.5 lumped K&C, bypassing the springs).
* **Wheel spins** ``w_i`` — ``Iw wdot = tau − R Fx`` (as T2). The gyroscopic precession is
  perpendicular to the spin axis, so it lands entirely on the body, not the spin rate.

Sign conventions (pinned; verified by ``_self_check``): ``theta`` > 0 = nose-DOWN (dive); ``phi`` >
0 = roll to the RIGHT (left side up); ``z``/``zu`` > 0 = UP. Suspension compression ``delta`` > 0
loads the spring; the corner spring/damper force is +up on the sprung mass, −down on the unsprung.
"""

from __future__ import annotations

import json
from pathlib import Path

import sympy as sp
from sympy.physics.vector import ReferenceFrame, cross, dot

WHEELS = 4
N_SAMPLES = 64
FIXTURE = Path(__file__).with_name("fixtures") / "t3_chassis_rhs.json"

FRONT = [True, True, False, False]  # FL, FR, RL, RR — front axle steers
SIDE = [1, -1, 1, -1]  # +1 left (y>0), −1 right (y<0)


def smooth_ramp(p, s):
    """C¹ one-sided ramp: 0 for p<0, p²/(2s) for 0≤p<s, p−s/2 for p≥s.

    Value AND slope continuous at p=0 (0, 0) and p=s (s/2, 1): the bumpstop never presents a
    discontinuous force or stiffness to the RK path at engagement.
    """
    return sp.Piecewise(
        (sp.Integer(0), p < 0),
        (p * p / (2 * s), p < s),
        (p - s / 2, True),
    )


def build_rhs():
    """Assemble the full 24-state T3 RHS as sympy expressions over a flat symbol vector.

    Returns ``(rhs_list, arg_symbols)``: ``rhs_list`` is the 24 derivatives in ``ChassisState``
    order; ``arg_symbols`` is the positional input vector (states, params, road, loads).
    """
    # --- states ------------------------------------------------------------------------------
    n, psi = sp.symbols("n psi", real=True)
    vx, vy, r = sp.symbols("vx vy r", real=True)
    w = sp.symbols(f"w0:{WHEELS}", real=True)
    z, th, ph = sp.symbols("z th ph", real=True)
    zd, thd, phd = sp.symbols("zd thd phd", real=True)
    zu = sp.symbols(f"zu0:{WHEELS}", real=True)
    zud = sp.symbols(f"zud0:{WHEELS}", real=True)

    # --- rigid-body / inertia params ---------------------------------------------------------
    m, izz = sp.symbols("m izz", positive=True)
    ms = sp.Symbol("ms", positive=True)
    ixx, iyy = sp.symbols("ixx iyy", positive=True)
    hs, hcg, hra = sp.symbols("hs hcg hra", real=True)
    g = sp.Symbol("g", positive=True)
    wheelbase = sp.Symbol("wheelbase", positive=True)
    mu = sp.symbols(f"mu0:{WHEELS}", positive=True)
    iw = sp.symbols(f"iw0:{WHEELS}", positive=True)
    xw = sp.symbols(f"x0:{WHEELS}", real=True)
    yw = sp.symbols(f"y0:{WHEELS}", real=True)
    Rw = sp.symbols(f"R0:{WHEELS}", positive=True)

    # --- suspension params -------------------------------------------------------------------
    kr = sp.symbols(f"kr0:{WHEELS}", positive=True)
    d0 = sp.symbols(f"d0_0:{WHEELS}", real=True)  # static compression carrying the corner load
    cb = sp.symbols(f"cb0:{WHEELS}", positive=True)
    cr = sp.symbols(f"cr0:{WHEELS}", positive=True)
    kbs = sp.symbols(f"kbs0:{WHEELS}", positive=True)
    gbs = sp.symbols(f"gbs0:{WHEELS}", positive=True)
    sbs = sp.Symbol("sbs", positive=True)
    ktz = sp.symbols(f"ktz0:{WHEELS}", positive=True)
    ctz = sp.symbols(f"ctz0:{WHEELS}", real=True)
    dtz0 = sp.symbols(f"dtz0_0:{WHEELS}", real=True)  # static tyre compression carrying corner load
    karb_f, karb_r = sp.symbols("karb_f karb_r", real=True)
    tf, tr = sp.symbols("tf tr", positive=True)
    anti_d, anti_s = sp.symbols("anti_d anti_s", real=True)

    # --- road / external loads ---------------------------------------------------------------
    kappa, grade, bank, kappa_v = sp.symbols("kappa grade bank kappa_v", real=True)
    drag, dmz, steer = sp.symbols("drag dmz steer", real=True)
    # Aero downforce per axle (N, + = DOWN), evaluated at the dynamic ride heights by the tier's aero
    # block and applied to the SPRUNG body at the axle line (front at x=xw[0]=a_f, rear at x=xw[2]=−b_r).
    # It reaches the tyres through the springs (F_susp rises ⇒ the tyre spring compresses), so the
    # per-wheel F_z the contact patch sees carries the downforce without a separate contact-patch term.
    fzaf, fzar = sp.symbols("fzaf fzar", real=True)
    zr = sp.symbols(f"zr0:{WHEELS}", real=True)
    zrd = sp.symbols(f"zrd0:{WHEELS}", real=True)
    fxw = sp.symbols(f"fxw0:{WHEELS}", real=True)
    fyw = sp.symbols(f"fyw0:{WHEELS}", real=True)
    mzw = sp.symbols(f"mzw0:{WHEELS}", real=True)
    tau = sp.symbols(f"tau0:{WHEELS}", real=True)

    track = [tf, tf, tr, tr]

    # === handling: wheel-frame tyre forces → body frame (as T2) ==============================
    body = ReferenceFrame("body")
    sum_force = 0 * body.x
    sum_moment = 0 * body.z
    for i in range(WHEELS):
        d = steer if FRONT[i] else sp.Integer(0)
        wheel = body.orientnew(f"wheel{i}", "Axis", [d, body.z])
        force = fxw[i] * wheel.x + fyw[i] * wheel.y
        arm = xw[i] * body.x + yw[i] * body.y
        sum_force += force
        sum_moment += cross(arm, force) + mzw[i] * body.z

    road = ReferenceFrame("road")
    body_from_road = road.orientnew("body_from_road", "Axis", [psi, road.z])
    mg = m * g
    gravity = (-mg * sp.sin(grade)) * road.x + (-mg * sp.sin(bank)) * road.y
    sum_fx = dot(sum_force, body.x) - drag + dot(gravity, body_from_road.x)
    sum_fy = dot(sum_force, body.y) + dot(gravity, body_from_road.y)
    sum_mz = dot(sum_moment, body.z) + dmz

    # gyroscopic reaction from the spinning wheels. +w = forward roll ⇒ spin angular momentum
    # h_i = Iw·w·(−wheel lateral). Body angular velocity Ω = φ̇ x̂ − θ̇ ŷ + r ẑ (θ>0 = dive = rotation
    # about −y). Reaction on the body = −Ω × Σh_i (derived symbolically for the sign check).
    Omega = phd * body.x + (-thd) * body.y + r * body.z
    h_tot = 0 * body.x
    for i in range(WHEELS):
        d = steer if FRONT[i] else sp.Integer(0)
        gw = body.orientnew(f"gyro{i}", "Axis", [d, body.z])
        h_tot += iw[i] * w[i] * (-gw.y)
    m_gyro = -cross(Omega, h_tot)
    m_gyro_x = dot(m_gyro, body.x)  # roll
    m_gyro_y = dot(m_gyro, body.y)  # (about +y; pitch uses −y ⇒ negate below)
    m_gyro_z = dot(m_gyro, body.z)  # yaw

    sum_mz = sum_mz + m_gyro_z
    vx_dot = sum_fx / m + r * vy
    vy_dot = sum_fy / m - r * vx
    r_dot = sum_mz / izz
    ax = vx_dot - r * vy  # body-frame CG longitudinal accel
    ay = vy_dot + r * vx  # body-frame CG lateral accel

    # === curvilinear kinematics (as T2) ======================================================
    cpsi, spsi = sp.cos(psi), sp.sin(psi)
    denom = 1 - n * kappa
    s_dot = (vx * cpsi - vy * spsi) / denom
    n_dot = vx * spsi + vy * cpsi
    psi_dot = r - kappa * s_dot

    # === wheel spins (as T2) =================================================================
    omega_dot = [(tau[i] - Rw[i] * fxw[i]) / iw[i] for i in range(WHEELS)]

    # === ride block ==========================================================================
    # normal-direction gravity + vertical-curvature (crest kappa_v<0 lightens the load): the
    # frame-transport term the T2 tier floored (CREST_UNLOADING_FLOOR_G); at T3 it is dynamic.
    g_n = g * sp.cos(grade) * sp.cos(bank) + kappa_v * vx * vx

    def zc(i):
        return z - xw[i] * th + yw[i] * ph  # sprung corner vertical displacement (small angle)

    def zcd(i):
        return zd - xw[i] * thd + yw[i] * phd

    delta = [d0[i] + zu[i] - zc(i) for i in range(WHEELS)]  # compression (+ = loaded)
    deltad = [zud[i] - zcd(i) for i in range(WHEELS)]

    def damper(i):
        return sp.Piecewise((cb[i] * deltad[i], deltad[i] >= 0), (cr[i] * deltad[i], True))

    def bumpstop(i):
        return kbs[i] * smooth_ramp(delta[i] - gbs[i], sbs)

    # ARB: resists differential suspension travel across an axle. travel_i = zc(i)−zu(i) (extension).
    # Roll of the axle ≈ (travel_L − travel_R)/track ⇒ moment k_arb·angle ⇒ ∓ force couple over the
    # track. Force up on the LEFT corner opposes a right roll (restoring — checked in _self_check).
    arb_f = karb_f * ((zc(0) - zu[0]) - (zc(1) - zu[1])) / tf**2
    arb_r = karb_r * ((zc(2) - zu[2]) - (zc(3) - zu[3])) / tr**2
    f_arb = [-arb_f, arb_f, -arb_r, arb_r]

    f_susp = [kr[i] * delta[i] + damper(i) + bumpstop(i) + f_arb[i] for i in range(WHEELS)]
    # tyre compression = static (dtz0, carrying the corner load) + road input − unsprung rise.
    f_tyre = [
        ktz[i] * (dtz0[i] + zr[i] - zu[i]) + ctz[i] * (zrd[i] - zud[i]) for i in range(WHEELS)
    ]

    # geometric load transfer straight to the contact patch (bypasses the springs — §7.5):
    #  lateral: (h_ra/track)·(m·ay); +on the loaded (outer) side. ay>0 (left turn) loads the right.
    #  longitudinal: anti_dive (front) / anti_squat (rear) route a fraction of m·ax·hcg/L; ax>0
    #  (accel) unloads the front, loads the rear.
    long_axle = m * ax * hcg / wheelbase
    geom = []
    for i in range(WHEELS):
        lat = -(hra / track[i]) * (m * ay) * SIDE[i] / 2
        anti = anti_d if FRONT[i] else anti_s
        lon = anti * long_axle / 2 * (-1 if FRONT[i] else 1)
        geom.append(lat + lon)

    # elastic transfer drives the ride DOF: roll about the roll axis (arm h_s−h_ra), pitch about the
    # sprung CG (arm h_s) reduced by the geometric (anti) share.
    anti_mean = (anti_d + anti_s) / 2
    m_roll_elastic = ms * ay * (hs - hra)
    # θ>0 = dive (front-down); z_corner uses −x_i·θ ⇒ the pitch moment scales with −ax (mirror of
    # roll's +ay). Braking (ax<0) ⇒ dive (θ̈>0). anti-dive/squat route the geometric share to tyres.
    m_pitch_elastic = -ms * ax * hs * (1 - anti_mean)

    m_pitch_susp = sum((-xw[i]) * f_susp[i] for i in range(WHEELS))  # Q_theta = ∂zc/∂θ · F
    m_roll_susp = sum(yw[i] * f_susp[i] for i in range(WHEELS))  # Q_phi = ∂zc/∂φ · F

    # Aero downforce on the sprung body: a −z force per axle. Heave gen-force = −(fzaf+fzar); pitch
    # gen-force Q_θ = Σ F·x (front x>0 ⇒ +θ̈ = dive, i.e. front downforce pushes the nose down).
    m_pitch_aero = fzaf * xw[0] + fzar * xw[2]
    heave = (sum(f_susp) - ms * g_n - (fzaf + fzar)) / ms
    pitch = (m_pitch_susp + m_pitch_elastic - m_gyro_y + m_pitch_aero) / iyy  # pitch axis = −y
    roll = (m_roll_susp + m_roll_elastic + m_gyro_x) / ixx

    unsprung = [
        (f_tyre[i] - f_susp[i] - mu[i] * g_n + geom[i]) / mu[i] for i in range(WHEELS)
    ]

    rhs = [
        s_dot, n_dot, psi_dot,
        vx_dot, vy_dot, r_dot,
        *omega_dot,
        zd, thd, phd,
        heave, pitch, roll,
        *zud,
        *unsprung,
    ]

    args = [
        n, psi, vx, vy, r, *w,
        z, th, ph, zd, thd, phd, *zu, *zud,
        m, izz, ms, ixx, iyy, hs, hcg, hra, g, wheelbase,
        *mu, *iw, *xw, *yw, *Rw,
        *kr, *d0, *cb, *cr, *kbs, *gbs, sbs, *ktz, *ctz, *dtz0,
        karb_f, karb_r, tf, tr, anti_d, anti_s,
        kappa, grade, bank, kappa_v, drag, dmz, steer, fzaf, fzar,
        *zr, *zrd, *fxw, *fyw, *mzw, *tau,
    ]
    return rhs, args


STATE_ORDER = [
    "s", "n", "psi_rel", "vx", "vy", "r", "w_fl", "w_fr", "w_rl", "w_rr",
    "z", "theta", "phi", "zdot", "thetadot", "phidot",
    "zu_fl", "zu_fr", "zu_rl", "zu_rr", "zudot_fl", "zudot_fr", "zudot_rl", "zudot_rr",
]


def build_rhs_lambda():
    """Lambdify the 24-state RHS over the flat arg vector; return ``(fn, arg_names)``."""
    rhs, args = build_rhs()
    fn = sp.lambdify(args, rhs, modules="math")
    return fn, [str(a) for a in args]


def sample_inputs(rng, arg_names):
    """One physically-plausible random input dict keyed by arg name (values are plain floats).

    Ranges are chosen so the fixture exercises both damper regimes (bump/rebound), both bumpstop
    regions of the C¹ ramp (below/above the gap), non-flat road (grade/bank/vertical curvature),
    steer, and spinning wheels — so a wrong branch or a dropped coupling term fails the 1e-12 test.
    """

    def u(a, b):
        return float(rng.uniform(a, b))

    d = {nm: 0.0 for nm in arg_names}
    mu_i = [u(10.0, 20.0) for _ in range(WHEELS)]
    m = u(720.0, 900.0)
    ms = m - sum(mu_i)
    a_f, b_r = u(1.4, 1.8), u(1.4, 1.8)
    tf_v, tr_v = u(1.5, 1.7), u(1.5, 1.7)
    d.update(
        n=u(-2.0, 2.0), psi=u(-0.3, 0.3), vx=u(10.0, 80.0), vy=u(-4.0, 4.0), r=u(-1.0, 1.0),
        z=u(-0.03, 0.03), th=u(-0.05, 0.05), ph=u(-0.05, 0.05),
        zd=u(-0.5, 0.5), thd=u(-0.6, 0.6), phd=u(-0.6, 0.6),
        m=m, izz=u(800.0, 1300.0), ms=ms, ixx=u(120.0, 250.0), iyy=u(800.0, 1100.0),
        hs=u(0.25, 0.40), hcg=u(0.25, 0.35), hra=u(0.0, 0.10), g=9.80665,
        wheelbase=a_f + b_r, tf=tf_v, tr=tr_v,
        karb_f=u(0.0, 5.0e5), karb_r=u(0.0, 5.0e5), sbs=0.005,
        anti_d=u(0.0, 0.6), anti_s=u(0.0, 0.6),
        kappa=u(-0.02, 0.02), grade=u(-0.08, 0.08), bank=u(-0.12, 0.12),
        kappa_v=u(-0.02, 0.02), drag=u(0.0, 6000.0), dmz=u(-500.0, 500.0), steer=u(-0.25, 0.25),
        fzaf=u(0.0, 20000.0), fzar=u(0.0, 20000.0),  # per-axle aero downforce (N, +down)
    )
    xs = [a_f, a_f, -b_r, -b_r]
    ys = [tf_v / 2, -tf_v / 2, tr_v / 2, -tr_v / 2]
    for i in range(WHEELS):
        d[f"x{i}"], d[f"y{i}"], d[f"R{i}"] = xs[i], ys[i], u(0.28, 0.36)
        d[f"iw{i}"], d[f"mu{i}"] = u(0.8, 1.8), mu_i[i]
        d[f"w{i}"] = u(20.0, 120.0)
        d[f"zu{i}"], d[f"zud{i}"] = u(-0.03, 0.03), u(-0.6, 0.6)
        d[f"kr{i}"] = u(180000.0, 260000.0)
        d[f"d0_{i}"] = u(0.02, 0.06)  # overlaps gbs ⇒ some samples engage the bumpstop
        d[f"cb{i}"], d[f"cr{i}"] = u(3000.0, 6000.0), u(6000.0, 12000.0)
        d[f"kbs{i}"], d[f"gbs{i}"] = u(3.0e5, 8.0e5), u(0.02, 0.05)
        d[f"ktz{i}"], d[f"ctz{i}"], d[f"dtz0_{i}"] = u(2.0e5, 3.0e5), u(200.0, 800.0), u(0.02, 0.05)
        d[f"zr{i}"], d[f"zrd{i}"] = u(-0.02, 0.02), u(-0.5, 0.5)
        d[f"fxw{i}"], d[f"fyw{i}"] = u(-4000.0, 4000.0), u(-6000.0, 6000.0)
        d[f"mzw{i}"], d[f"tau{i}"] = u(-120.0, 120.0), u(-1500.0, 1500.0)
    return d


def generate():
    """Derive the EOM, sample states, and write the committed RHS fixture."""
    import numpy as np

    fn, arg_names = build_rhs_lambda()
    rng = np.random.default_rng(20260719)
    samples = []
    for _ in range(N_SAMPLES):
        inp = sample_inputs(rng, arg_names)
        rhs = fn(*[inp[nm] for nm in arg_names])
        samples.append({"inputs": inp, "rhs": [float(v) for v in rhs]})

    doc = {
        "_comment": "Auto-generated by docs/derivations/t3_chassis_kane.ipynb (Decision #32). "
        "Do not edit by hand; re-run the notebook. Checked by the Rust 1e-12 test "
        "outlap-vehicle/tests/kane_fixture_t3.rs.",
        "state_order": STATE_ORDER,
        "arg_order": arg_names,
        "front": FRONT,
        "samples": samples,
    }
    FIXTURE.parent.mkdir(parents=True, exist_ok=True)
    FIXTURE.write_text(json.dumps(doc, indent=1) + "\n")
    return len(samples)


if __name__ == "__main__":
    count = generate()
    print(f"wrote {count} samples to {FIXTURE}")
