# SPDX-License-Identifier: AGPL-3.0-only
"""A fast, reduced-order stint-pace surrogate — the forward model the calibrator inverts.

Running the full Rust stint driver inside an optimiser loop is impractical: every candidate
parameter set rebuilds the g-g-g-v envelope across its tyre-state axes (a seconds-scale cold
step, *not* Python-tunable), so a least-squares fit of even a few parameters would take minutes.
Real tyre-degradation calibration is instead done against the *per-lap pace curve*, and that is
exactly what this module models.

:class:`StintModel` is a **clean-room numpy mirror** of the Rust thermal-ring wear/grip laws
(``crates/outlap-tire/src/thermal.rs``) — the same relationship ``tirefit.mf61`` bears to the Rust
MF6.1 force kernels. It integrates the identical published laws lap-to-lap:

* Archard sliding-energy wear ``Δw = k_w · (1/H(T_s)) · Φ_wear`` (Archard 1953), with the
  Grosch temperature-hardness ``1/H(T_s) = min(exp(c_H·(T_s−T_opt)), cap)`` (Grosch 1963);
* the C¹ cliff ``1 − Δ_c·σ((w−w_c)/s_w)`` and irreversible thermal damage
  ``dD/dt = (1/τ_D)·⟨(T_c−T_deg)/ΔT_ref⟩₊^β`` with grip loss ``1 − Δ_D·D``;
* the grip window ``λ_μ(T_s) = exp(−c_T·((T_s−T_opt)/T_opt)²)`` (Farroni TRT).

Grip is mapped to lap time by ``t(n) = t_ref · (g(1)/g(n))^p`` — a grip-limited lap scales as
``t ∝ v⁻¹ ∝ μ^(−1/2)``, so ``p ≈ 0.5`` (a fraction of the lap is not grip-limited; ``p`` is an
anchor). The car/track-specific anchors in :class:`StintAnchor` (per-lap wear exposure ``Φ_wear``,
representative temperatures, base lap time, pace exponent) are held fixed while the *physical*
tyre parameters are fitted; their defaults are measured from a reference F1-on-Catalunya stint so
that recovered parameters transfer to the real Rust driver (validated by the PR9 wear/cliff gate).

The module constants mirror ``thermal.rs`` verbatim: ``c_H = 0.02 /°C`` and the ``1/H`` cap 20.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, replace

import numpy as np
from numpy.typing import NDArray

F = NDArray[np.float64]

# Mirror of the fixed module constants in crates/outlap-tire/src/thermal.rs.
WEAR_HARDNESS_SENS_PER_C = 0.02  # c_H — Grosch hardness temperature sensitivity, /°C
WEAR_INV_HARDNESS_MAX = 20.0  # cap on 1/H(T_s)


@dataclass(frozen=True)
class SurrogateParams:
    """The physical tyre parameters, named exactly as the ``.tyr`` thermal/wear fields.

    A one-to-one mirror of the subset of :class:`outlap.schema` ``TyrThermal`` / ``TyrWear`` that
    governs stint pace. Temperatures are °C and wear depths mm (the file/display boundary), matching
    the ``.tyr`` document.
    """

    # thermal grip window (TyrThermal)
    t_opt: float = 95.0
    c_t: float = 2.2
    # wear + cliff (TyrWear)
    k_w: float = 3.0e-9
    w_max: float = 8.0
    w_c: float = 2.0
    s_w: float = 0.5
    delta_c: float = 0.25
    # thermal damage (TyrWear)
    delta_d: float = 0.30
    t_deg: float = 120.0
    tau_d: float = 600.0
    delta_t_ref: float = 20.0
    beta: float = 2.0

    @classmethod
    def from_tyr(
        cls, thermal: dict[str, float], wear: dict[str, float]
    ) -> SurrogateParams:
        """Build from a ``.tyr`` document's ``thermal``/``wear`` blocks (extra keys ignored)."""
        return cls(
            t_opt=float(thermal["t_opt"]),
            c_t=float(thermal["c_t"]),
            k_w=float(wear["k_w"]),
            w_max=float(wear["w_max"]),
            w_c=float(wear["w_c"]),
            s_w=float(wear["s_w"]),
            delta_c=float(wear["delta_c"]),
            delta_d=float(wear["delta_d"]),
            t_deg=float(wear["t_deg"]),
            tau_d=float(wear["tau_d"]),
            delta_t_ref=float(wear["delta_t_ref"]),
            beta=float(wear["beta"]),
        )

    def tyr_updates(self) -> tuple[dict[str, float], dict[str, float]]:
        """Return ``(thermal_updates, wear_updates)`` dicts to merge into a ``.tyr`` document."""
        thermal = {"t_opt": self.t_opt, "c_t": self.c_t}
        wear = {
            "k_w": self.k_w,
            "w_max": self.w_max,
            "w_c": self.w_c,
            "s_w": self.s_w,
            "delta_c": self.delta_c,
            "delta_d": self.delta_d,
            "t_deg": self.t_deg,
            "tau_d": self.tau_d,
            "delta_t_ref": self.delta_t_ref,
            "beta": self.beta,
        }
        return thermal, wear

    def replace(self, **changes: float) -> SurrogateParams:
        """A copy with the named fields overridden (thin :func:`dataclasses.replace` wrapper)."""
        return replace(self, **changes)

    def as_dict(self) -> dict[str, float]:
        """All parameters as a plain dict (report/JSON serialisation)."""
        return {k: float(v) for k, v in asdict(self).items()}


@dataclass(frozen=True)
class StintAnchor:
    """Car/track operating point that maps tyre state to lap time (held fixed during a fit).

    These fold together everything the reduced model does not resolve explicitly: the per-lap
    sliding-energy exposure ``Φ_wear`` (so ``k_w·(1/H)·Φ_wear`` is the mm of wear per lap), the
    representative surface/carcass temperatures the tyre operates at, the base (fresh, warm) lap
    time, the grip→pace exponent, and an optional cold-start warm-up. Defaults are measured from a
    reference **F1 2026 on Catalunya** T0 stint (``initial_tire_temp_c=None``, warm at optimum), so
    a fit against real pace data yields parameters that reproduce in the Rust driver.
    """

    t_ref_s: float = 86.0  # base fresh-warm lap time
    p_pace: float = 0.5  # grip→pace exponent (t ∝ μ^-p; ~0.5 for a grip-limited lap)
    phi_wear: float = (
        2.5e7  # per-lap wear exposure Φ_wear = ∫Q_fric dt / A_cp (mm per k_w·1/H)
    )
    t_op_c: float = (
        95.0  # representative steady surface temperature (warm operating point)
    )
    t_c_c: float = 95.0  # representative carcass temperature (drives thermal damage)
    lap_time_s: float = 86.0  # lap duration used to integrate the damage rate
    warm_up_laps: float = (
        0.0  # warm-up time constant in laps (0 → warm start, no warm-up)
    )
    t_s0_c: float | None = (
        None  # initial surface temp for a cold start (None → warm at t_op_c)
    )
    w0_mm: float = 0.0  # tread wear carried into lap 1


def inv_hardness(t_s_c: float, t_opt_c: float) -> float:
    """Grosch temperature-hardness ``1/H(T_s) = min(exp(c_H·(T_s−T_opt)), cap)`` (thermal.rs)."""
    return min(
        math.exp(WEAR_HARDNESS_SENS_PER_C * (t_s_c - t_opt_c)), WEAR_INV_HARDNESS_MAX
    )


def grip_window(t_s_c: float, t_opt_c: float, c_t: float) -> float:
    """Farroni grip window ``λ_μ(T_s)=exp(−c_T·((T_s−T_opt)/T_opt)²)`` (thermal.rs mu_scale)."""
    dev = (t_s_c - t_opt_c) / t_opt_c
    return math.exp(-c_t * dev * dev)


def cliff(w_mm: float, w_c: float, s_w: float, delta_c: float) -> float:
    """The C¹ cliff ``1 − Δ_c·σ((w−w_c)/s_w)`` (thermal.rs wear_grip_scale)."""
    z = (w_mm - w_c) / s_w
    sigmoid = 1.0 / (1.0 + math.exp(-z))
    return 1.0 - delta_c * sigmoid


@dataclass(frozen=True)
class StintTrace:
    """Per-lap surrogate trace: laps and the state/lap-time arrays that produced them."""

    lap: F
    lap_time_s: F
    surface_c: F
    wear_mm: F
    damage: F
    grip: F


def stint_trace(
    params: SurrogateParams, anchor: StintAnchor, n_laps: int
) -> StintTrace:
    """Integrate the reduced stint model lap-to-lap and return the full per-lap trace.

    Explicit forward Euler on wear and damage (the QSS march idiom), monotone-clamped, so wear and
    damage only grow and lap 1 is the fresh-warm reference.
    """
    if n_laps < 1:
        raise ValueError("n_laps must be >= 1")
    laps = np.arange(1, n_laps + 1, dtype=np.float64)
    surface = np.empty(n_laps, dtype=np.float64)
    wear = np.empty(n_laps, dtype=np.float64)
    damage = np.empty(n_laps, dtype=np.float64)
    grip = np.empty(n_laps, dtype=np.float64)
    times = np.empty(n_laps, dtype=np.float64)

    t_s0 = anchor.t_s0_c if anchor.t_s0_c is not None else anchor.t_op_c
    w = anchor.w0_mm
    d = 0.0
    grip_ref = 0.0
    for n in range(n_laps):
        # Optional cold-start warm-up: surface relaxes toward the operating point over warm_up_laps.
        if anchor.warm_up_laps > 0.0:
            frac = math.exp(-n / anchor.warm_up_laps)
            t_s = anchor.t_op_c - (anchor.t_op_c - t_s0) * frac
        else:
            t_s = anchor.t_op_c
        # Grip = thermal window × cliff × irreversible damage (thermal.rs mu_scale_total).
        g = (
            grip_window(t_s, params.t_opt, params.c_t)
            * cliff(w, params.w_c, params.s_w, params.delta_c)
            * (1.0 - params.delta_d * min(max(d, 0.0), 1.0))
        )
        if n == 0:
            grip_ref = g
        surface[n] = t_s
        wear[n] = w
        damage[n] = d
        grip[n] = g
        times[n] = anchor.t_ref_s * (grip_ref / g) ** anchor.p_pace
        # Advance the slow states for the next lap (Archard wear + threshold-power damage).
        dw = params.k_w * inv_hardness(t_s, params.t_opt) * anchor.phi_wear
        w = min(w + dw, params.w_max)
        over = max((anchor.t_c_c - params.t_deg) / params.delta_t_ref, 0.0)
        if over > 0.0:
            d = min(d + anchor.lap_time_s * over**params.beta / params.tau_d, 1.0)
    return StintTrace(
        lap=laps,
        lap_time_s=times,
        surface_c=surface,
        wear_mm=wear,
        damage=damage,
        grip=grip,
    )


def stint_lap_times(params: SurrogateParams, anchor: StintAnchor, n_laps: int) -> F:
    """Just the per-lap times (the calibration observable)."""
    return stint_trace(params, anchor, n_laps).lap_time_s
