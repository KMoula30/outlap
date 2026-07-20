# SPDX-License-Identifier: AGPL-3.0-only
"""M6 PR8 reviewer figures — one per validation gate, written into docs/validation/img/.

Opt-in review tool (not run in CI). Run against a RELEASE wheel::

    maturin develop --release --manifest-path crates/outlap-py/Cargo.toml
    python/.venv/bin/python python/tools/plot_pr8_validation.py

Palette: the repo's CVD-safe blue/orange pair (INK/GRID from qss_stint_soc/gen_figs.py) plus an
Okabe-Ito green; identity is never colour-alone (distinct linestyles/markers), a legend is always
present, one y-axis per panel.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from outlap.core import Track, min_curvature, solve_lap_dataset, solve_stint_dataset

ROOT = Path(__file__).resolve().parents[2]
IMG = ROOT / "docs" / "validation" / "img"
IMG.mkdir(parents=True, exist_ok=True)

CATALUNYA = str(ROOT / "data" / "tracks" / "catalunya_osm")
F1 = str(ROOT / "data" / "vehicles" / "f1_2026")
BATTERY_GOLDEN = ROOT / "crates" / "outlap-qss" / "tests" / "golden" / "battery_nrel"
CHASSIS_GOLDEN = ROOT / "crates" / "outlap-transient" / "tests" / "golden" / "bmw320i"

INK, GRID = "#1b1b1f", "#d9d9e0"
REF, OURS, THIRD = "#2b6cb0", "#c05621", "#2f855a"  # blue / orange / green
OK, BAD = "#2f855a", "#c53030"  # asserted-pass / threshold
COARSE: dict = {
    "flat_track": True,
    "envelope": {"v_points": 10, "ax_points": 9, "g_normal_points": 2},
}

plt.rcParams.update(
    {
        "figure.dpi": 140,
        "font.size": 10.5,
        "axes.edgecolor": INK,
        "axes.labelcolor": INK,
        "text.color": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.7,
        "axes.axisbelow": True,
        "axes.titleweight": "bold",
        "legend.frameon": False,
    }
)


def _parse_battery(
    name: str,
) -> tuple[dict[str, float], np.ndarray, np.ndarray, np.ndarray]:
    p: dict[str, float] = {}
    t, i, v = [], [], []
    for line in (BATTERY_GOLDEN / f"{name}.csv").read_text().splitlines():
        if line.startswith("# param "):
            k, val = line[len("# param ") :].split(":")
            p[k.strip()] = float(val)
        elif line and not line.startswith("#") and not line.startswith("t_s"):
            a, b, c = line.split(",")
            t.append(float(a))
            i.append(float(b))
            v.append(float(c))
    return p, np.array(t), np.array(i), np.array(v)


def _outlap_ecm(p: dict, t: np.ndarray, cur: np.ndarray) -> np.ndarray:
    """outlap's exact-exponential RC advance (crates/outlap-qss/.../battery.rs), replayed in Python:
    V = OCV − I·R0 − V_RC1(−V_RC2), with V_RC ← V_RC·e^{−dt/τ} + I·R·(1 − e^{−dt/τ})."""
    v_rc1 = v_rc2 = 0.0
    out = np.empty_like(cur)
    two_rc = p["rc_pairs"] == 2
    prev = t[0]
    for k in range(len(t)):
        dt = t[k] - prev
        prev = t[k]
        d1 = np.exp(-dt / p["tau1_s"])
        v_rc1 = v_rc1 * d1 + cur[k] * p["r1_ohm"] * (1 - d1)
        if two_rc:
            d2 = np.exp(-dt / p["tau2_s"])
            v_rc2 = v_rc2 * d2 + cur[k] * p["r2_ohm"] * (1 - d2)
        out[k] = p["ocv_v"] - cur[k] * p["r0_ohm"] - v_rc1 - v_rc2
    return out


def fig_battery() -> None:
    name = "cold25_rc2"
    p, t, cur, vref = _parse_battery(name)
    vours = _outlap_ecm(p, t, cur)
    rms_pct = 100 * np.sqrt(np.mean((vours - vref) ** 2)) / np.mean(np.abs(vref))

    fig, (a0, a1, a2) = plt.subplots(
        3,
        1,
        figsize=(9.2, 7.0),
        sharex=True,
        gridspec_kw={"height_ratios": [1, 2.2, 1.2]},
    )
    a0.plot(t, cur, color=INK, lw=1.4)
    a0.set_ylabel("current\nI (A)")
    a0.set_title(
        "Gate #1 — battery ECM vs NREL thevenin  (cold 25 °C, 2 RC pairs; ±1C/±2C, 10 s on / 60 s rest)"
    )
    a1.plot(t, vref, color=REF, lw=2.4, label="NREL thevenin (reference)")
    a1.plot(
        t,
        vours,
        color=OURS,
        lw=1.4,
        ls=(0, (5, 4)),
        label="outlap exact-exponential ECM",
    )
    a1.set_ylabel("terminal voltage (V)")
    a1.legend(loc="lower right")
    a1.annotate(
        f"RMS = {rms_pct:.3f}% of V̄   (gate ≤ 1%)",
        xy=(0.015, 0.06),
        xycoords="axes fraction",
        fontsize=10.5,
        color=OK,
        weight="bold",
    )
    a2.plot(t, 1000 * (vours - vref), color=THIRD, lw=1.2)
    a2.axhline(0, color=GRID, lw=1)
    a2.set_ylabel("residual\n(mV)")
    a2.set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(IMG / "battery_ecm_pulse.png", bbox_inches="tight")
    plt.close(fig)
    print(f"battery_ecm_pulse.png  (RMS {rms_pct:.3f}%)")


def fig_stint() -> None:
    print("  solving 10-lap stints (QSS + T2)…")
    tk = Track.load(CATALUNYA)
    q = solve_stint_dataset(
        F1, tk, n_laps=10, tier="t0", sim=COARSE, tire_thermal=False, initial_soc=0.6
    )
    t2 = solve_stint_dataset(
        F1,
        tk,
        n_laps=10,
        tier="t2",
        sim=COARSE,
        tire_thermal=False,
        initial_soc=0.6,
        speed_margin=0.85,
    )
    soc = q["state_of_charge"].values  # (lap, s)
    n_lap, n_st = soc.shape
    x = np.arange(n_lap * n_st) / n_st  # cumulative lap number
    lo, hi = 0.2, 0.9

    fig, (a0, a1) = plt.subplots(
        1, 2, figsize=(12.5, 4.6), gridspec_kw={"width_ratios": [1.7, 1]}
    )
    a0.axhspan(lo, hi, color=GRID, alpha=0.5, label="usable window [0.2, 0.9]")
    a0.plot(x, soc.reshape(-1), color=REF, lw=1.1, label="QSS (T0) — within-lap trace")
    a0.plot(
        np.arange(1, n_lap + 1),
        t2["state_of_charge"].values,
        color=OURS,
        ls="none",
        marker="o",
        ms=6,
        label="T2 — end of lap",
    )
    a0.axhline(0.6, color=INK, lw=1, ls=":", label="seed 0.6 (lap 1 only)")
    a0.set_xlabel("lap")
    a0.set_ylabel("pack state of charge")
    a0.set_title("Gate #2 — 10-lap f1_2026 stint SoC (both tiers, shared seed)")
    a0.legend(loc="upper right", fontsize=8.5)
    a0.annotate(
        "cycles the full window every lap;\ncharge-sustains at the floor (carried, not re-seeded)",
        xy=(3.2, 0.36),
        fontsize=9,
        color=INK,
    )

    laps = np.arange(1, n_lap + 1)
    w = 0.38
    a1.bar(
        laps - w / 2, q["deploy_energy_mj"].values, w, color=OURS, label="deploy (QSS)"
    )
    a1.bar(
        laps + w / 2,
        q["harvest_energy_mj"].values,
        w,
        color=THIRD,
        label="harvest (QSS)",
    )
    a1.axhline(8.5, color=BAD, lw=1.2, ls="--", label="8.5 MJ Recharge budget")
    a1.set_xlabel("lap")
    a1.set_ylabel("energy per lap (MJ)")
    a1.set_title("consumption AND regeneration, every lap")
    a1.legend(loc="lower right", fontsize=8.5)
    fig.tight_layout()
    fig.savefig(IMG / "ers_stint_soc.png", bbox_inches="tight")
    plt.close(fig)
    print("ers_stint_soc.png")


def _lap_energy(tier: str) -> tuple[float, float, float]:
    """(fuel_kg, deploy_MJ, harvest_MJ) for one f1_2026 lap at the given tier."""
    tk = Track.load(CATALUNYA)
    rl = min_curvature(tk, 1.1)
    sim = {
        "flat_track": True,
        "envelope": {"v_points": 16, "ax_points": 12, "g_normal_points": 3},
    }
    kw = {"initial_soc": 0.6}
    if tier == "t2":
        kw["speed_margin"] = 0.85
    d = solve_lap_dataset(F1, rl.line(), tier=tier, sim=sim, **kw)
    if tier == "t0":
        fuel = float(d["fuel_mass_kg"].values[0] - d["fuel_mass_kg"].values[-1])
        dt = np.diff(d["s"].values) / np.maximum(d["v"].values[:-1], 1.0)
        dep = (
            float(
                np.sum(
                    0.5
                    * (d["deploy_power_w"].values[:-1] + d["deploy_power_w"].values[1:])
                    * dt
                )
            )
            / 1e6
        )
        har = (
            float(
                np.sum(
                    0.5
                    * (
                        d["harvest_power_w"].values[:-1]
                        + d["harvest_power_w"].values[1:]
                    )
                    * dt
                )
            )
            / 1e6
        )
    else:
        fuel = float(80.0 - d.attrs["fuel_remaining_kg"])
        dt = np.diff(d["time"].values)
        dep = (
            float(
                np.sum(
                    0.5
                    * (
                        d["traction_power_w"].values[:-1]
                        + d["traction_power_w"].values[1:]
                    )
                    * dt
                )
            )
            / 1e6
        )
        har = (
            float(
                np.sum(
                    0.5
                    * (d["regen_power_w"].values[:-1] + d["regen_power_w"].values[1:])
                    * dt
                )
            )
            / 1e6
        )
    return fuel, dep, har


def fig_parity() -> None:
    print("  solving f1_2026 T0 + T2 single laps (parity)…")
    f0, dep0, har0 = _lap_energy("t0")
    f2, dep2, har2 = _lap_energy("t2")

    fig, (a0, a1) = plt.subplots(
        1, 2, figsize=(12.0, 4.6), gridspec_kw={"width_ratios": [1.2, 1]}
    )
    labels = ["harvest\n(MJ)", "deploy\n(MJ)", "fuel\n(kg)"]
    t0v = [har0, dep0, f0]
    t2v = [har2, dep2, f2]
    xpos = np.arange(3)
    w = 0.38
    a0.bar(xpos - w / 2, t0v, w, color=REF, label="T0 (QSS)")
    a0.bar(xpos + w / 2, t2v, w, color=OURS, label="T2 (transient)")
    a0.set_xticks(xpos)
    a0.set_xticklabels(labels)
    a0.set_ylabel("per-lap magnitude")
    a0.set_title("Gate #3 — fuel + ERS energy per lap, T0 vs T2")
    a0.legend(loc="upper right")
    for xi, (u, v) in enumerate(zip(t0v, t2v, strict=True)):
        a0.annotate(f"{u:.2f}", (xi - w / 2, u), ha="center", va="bottom", fontsize=8)
        a0.annotate(f"{v:.2f}", (xi + w / 2, v), ha="center", va="bottom", fontsize=8)

    deltas = [
        ("harvest", 100 * abs(har2 - har0) / har0, True),
        ("deploy", 100 * abs(dep2 - dep0) / dep0, False),
        ("fuel", 100 * abs(f2 - f0) / f0, False),
    ]
    names = [d[0] for d in deltas]
    vals = [d[1] for d in deltas]
    colors = [OK if d[2] else OURS for d in deltas]
    a1.barh(names[::-1], vals[::-1], color=colors[::-1])
    a1.axvline(1.0, color=BAD, lw=1.4, ls="--", label="1% parity gate")
    for i, (_n, v, ok) in enumerate(deltas[::-1]):
        tag = "asserted ≤1%" if ok else "recorded (driver margin)"
        a1.annotate(
            f"{v:.2f}%  · {tag}", (max(v, 1.0) + 0.6, i), va="center", fontsize=8.5
        )
    a1.set_xlabel("cross-tier difference (%)")
    a1.set_xlim(0, max(vals) * 1.5)
    a1.set_title("shared rule agrees; profile diverges")
    a1.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(IMG / "ers_parity_energy.png", bbox_inches="tight")
    plt.close(fig)
    print("ers_parity_energy.png")


def fig_chassis() -> None:
    metrics = {}
    for line in (CHASSIS_GOLDEN / "metrics.csv").read_text().splitlines():
        if line and not line.startswith("#") and not line.startswith("key"):
            k, v = line.split(",")
            metrics[k] = float(v)
    wb = metrics["wheelbase_m"]
    # outlap T3 measured yaw-rate gains (deterministic; crates/outlap-transient/tests/handling.rs).
    speeds = np.array([20.0, 25.0, 30.0])
    ours = np.array([7.4208, 9.1066, 10.7571])
    oracle = speeds / wb  # neutral single-track V/L

    rows = [
        ln.split(",")
        for ln in (CHASSIS_GOLDEN / "step_steer.csv").read_text().splitlines()
        if ln and not ln.startswith("#") and not ln.startswith("t_s")
    ]
    step = np.array(rows, dtype=float)
    st_t, st_yaw = step[:, 0], step[:, 1]

    fig, (a0, a1) = plt.subplots(1, 2, figsize=(12.0, 4.6))
    vgrid = np.linspace(18, 32, 50)
    a0.plot(
        vgrid,
        vgrid / wb,
        color=REF,
        lw=2.2,
        label="CommonRoad single-track  r/δ = V/L (neutral)",
    )
    a0.plot(
        speeds,
        ours,
        color=OURS,
        ls="none",
        marker="o",
        ms=8,
        label="outlap T3 (14-DOF)",
    )
    for v, g, o in zip(speeds, ours, oracle, strict=True):
        a0.annotate(
            f"{100 * (g - o) / o:+.1f}%",
            (v, g),
            textcoords="offset points",
            xytext=(6, -12),
            fontsize=9,
            color=OURS,
        )
    a0.set_xlabel("speed (m/s)")
    a0.set_ylabel("yaw-rate gain  r/δ  (1/s)")
    a0.set_title("Gate #4 — yaw-rate gain vs CommonRoad BMW 320i")
    a0.legend(loc="upper left", fontsize=8.5)
    a0.annotate(
        "recorded: 14-DOF ~4–7% below the rigid bicycle\n"
        "(load transfer + roll); asserted near-neutral |K| < 6e-4",
        xy=(0.02, 0.05),
        xycoords="axes fraction",
        fontsize=8.5,
        color=INK,
    )

    a1.plot(st_t, st_yaw, color=REF, lw=2.2, label="CommonRoad ST step-steer")
    a1.axhline(
        st_yaw[-1],
        color=INK,
        lw=1,
        ls=":",
        label=f"steady = V/L = {st_yaw[-1] / 0.02:.2f}/s per rad",
    )
    a1.set_xlabel("time (s)")
    a1.set_ylabel("yaw rate (1/s)")
    a1.set_title("step-steer transient (0.02 rad @ 25 m/s) — recorded")
    a1.legend(loc="lower right", fontsize=8.5)
    fig.tight_layout()
    fig.savefig(IMG / "chassis_yaw_gain.png", bbox_inches="tight")
    plt.close(fig)
    print("chassis_yaw_gain.png")


def main() -> None:
    fig_battery()
    fig_chassis()
    fig_stint()
    fig_parity()
    print(f"→ wrote 4 figures to {IMG}")


if __name__ == "__main__":
    main()
