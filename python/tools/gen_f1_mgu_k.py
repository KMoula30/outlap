# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the *semi-virtual* f1_2026 MGU-K efficiency/loss sidecar (D-M6-13 Layer 2).

The synthetic MGU-K keeps its authored axes (up to 50 000 rpm, 800 V); its peak torque is 223 N·m —
sized so the machine delivers the FIA 350 kW deploy at the 15 000 rpm crank redline it lives on (it
was 120 N·m, which topped out near 188 kW there). What the map adds is a real efficiency/loss surface
imported from a reference machine: this tool takes a
reference PDT **EDrive** `.h5` (a real inverter+machine sweep) and interpolates/extrapolates its
**system efficiency** (`motor_efficiency · inverter_efficiency`) and **system loss**
(`motor_loss_total + inverter_loss_total`) onto the MGU-K's own `(speed_rpm, torque_nm)` grid at the
nearest DC-voltage slice. The result is a "semi-virtual" MGU-K: authored envelope, measured loss shape.

The reference `.h5` is private to the author (firewall: never committed); only the derived
`tables/mgu_k.parquet` is committed. Default source: the 6Et 730 V EDrive (~308 kW, 147 N·m — closest
to the FIA 2026 C5.2.7 350 kW DC-bus limit), which carries an 800 V slice matching the MGU-K.

Extrapolation past the reference domain (speed > 20 000 rpm, |torque| > the per-speed envelope) is
LINEAR from the boundary slope, with efficiency clamped to (0, 1) and loss clamped ≥ 0 — a deliberate
semi-virtual estimate for the high-speed field-weakening region the 20 000 rpm reference does not reach.

    uv run --directory python python tools/gen_f1_mgu_k.py \
        [--src ~/pdt_reference/EDrive_121.0L_6Et_700.0I_730.0V_bcf42_outlap.h5]
"""

from __future__ import annotations

import argparse
import tempfile
from pathlib import Path

import h5py
import numpy as np
import pandas as pd
import yaml

from outlap.importers.pdt_h5.edrive import convert_edrive

# The synthetic MGU-K's authored axes (must match data/vehicles/f1_2026/ptm/mgu_k.ptm.yaml).
#
# SPEED IS THE UNIT'S **OUTPUT SHAFT** (D-M6-13 Layer 3). The MGU-K map is a lumped unit: a bare rotor
# spinning to the regulatory ~50 000 rpm PLUS its fixed step-up reduction, lumped together and
# expressed at the shaft the unit declares as its `output` — the crank. That shaft is the V6's, so it
# redlines at 15 000 rpm and the axis stops just past it. (Layer 2 authored a 0–50 000 rpm axis here,
# which is the ROTOR's range: two thirds of it was unreachable by any solver query AND extrapolated
# past the 20 000 rpm reference. If a future car declares the reduction as a `fixed_ratio`
# on the unit, THAT map's axis is the rotor's and would run to 50 000.)
#
# A dense grid (10 speeds × 25 torques) so the efficiency/loss map interpolates smoothly; the whole
# axis now sits INSIDE the reference sweep's 20 000 rpm domain, so no speed extrapolation remains.
# Peak torque 223 N·m is the OUTPUT-SHAFT (crank) torque — τ·ω = 350 kW at the redline.
SPEED_RPM = np.arange(0.0, 18000.0 + 1.0, 2000.0)  # 0, 2 000, … 18 000 rpm (10 pts)
MGU_K_VDC = 800.0  # mgu_k.ptm meta.dc_voltage_v
PEAK_TORQUE_NM = 223.0  # 350 kW at the 15 000 rpm crank redline (was 120 — under-torqued for F1)
LOAD_FRACTION = np.linspace(-1.0, 1.0, 25)  # normalized load axis (−1 regen … +1 drive), 25 pts
TORQUE_NM = np.round(LOAD_FRACTION * PEAK_TORQUE_NM, 4)  # −223 … +223 N·m absolute grid (25 pts)

MGU_K_MASS_KG = 24.0  # mgu_k.ptm mass_kg — the reference machine is heavier; scale caps to this.
COOLING_SCALE = 3.0  # scale the .emotor cooling paths (see build_emotor) so a hard f1 stint settles
# the winding warm (~170 °C, inside the 155–180 °C derate band) but bounded well below the t_max
# cut-out — the deploy tapers within a lap as the winding heats (a real thermal peak-power limit), not
# a hard cut. Left un-scaled the pessimistic EV-reference loss pins it at t_max and crushes deploy.

DEFAULT_SRC = Path.home() / "pdt_reference/EDrive_121.0L_6Et_700.0I_730.0V_bcf42_outlap.h5"
_REPO = Path(__file__).resolve().parents[2]
OUT_PARQUET = _REPO / "data/vehicles/f1_2026/ptm/tables/mgu_k.parquet"
OUT_EMOTOR = _REPO / "data/vehicles/f1_2026/emotor/mgu_k.emotor.yaml"
# The schema-fixture mirrors (kept in lock-step so the qss/schema tests see the same machine).
FIXTURE_PARQUET = _REPO / "crates/outlap-schema/tests/fixtures/ptm/tables/mgu_k.parquet"
FIXTURE_EMOTOR = _REPO / "crates/outlap-schema/tests/fixtures/emotor/mgu_k.emotor.yaml"


def _interp_extrap(x: np.ndarray, xp: np.ndarray, fp: np.ndarray) -> np.ndarray:
    """1-D linear interpolation with LINEAR extrapolation past both ends (unlike ``np.interp``,
    which clamps). ``xp`` must be strictly ascending."""
    y = np.interp(x, xp, fp)
    if len(xp) >= 2:
        below = x < xp[0]
        slope_lo = (fp[1] - fp[0]) / (xp[1] - xp[0])
        y = np.where(below, fp[0] + slope_lo * (x - xp[0]), y)
        above = x > xp[-1]
        slope_hi = (fp[-1] - fp[-2]) / (xp[-1] - xp[-2])
        y = np.where(above, fp[-1] + slope_hi * (x - xp[-1]), y)
    return y


def build_semi_virtual(src: Path) -> pd.DataFrame:
    """Resample the reference EDrive η/loss onto the MGU-K grid. Returns the sidecar long-form."""
    with h5py.File(src, "r") as f:
        speed = np.asarray(f["sweep/speed"][()], dtype=float)  # (n_speed,)
        vdc = np.atleast_1d(np.asarray(f["sweep/vdc"][()], dtype=float))
        iv = int(np.argmin(np.abs(vdc - MGU_K_VDC)))  # nearest DC slice (800 V exists here)
        tau = np.asarray(f["operating_grid/airgap_torque"][()], dtype=float)[iv]  # (speed, load)
        sys_eff = (
            np.asarray(f["operating_grid/motor_efficiency"][()], dtype=float)[iv]
            * np.asarray(f["operating_grid/inverter_efficiency"][()], dtype=float)[iv]
        )
        sys_loss = (
            np.asarray(f["operating_grid/motor_loss_total"][()], dtype=float)[iv]
            + np.asarray(f["operating_grid/inverter_loss_total"][()], dtype=float)[iv]
        )

    n_speed = speed.shape[0]
    # LOSS is the primary quantity we resample (its physics — copper I²R, iron ∝ speed — extrapolates
    # more meaningfully than a bounded ratio); EFFICIENCY is then DERIVED from loss + operating power
    # so the two columns are mutually consistent everywhere (and reproduce the reference's measured η
    # in-domain, since η ≡ P_mech/(P_mech+loss)). `sys_eff` is used only to sanity-check in-domain.
    _ = sys_eff
    # Stage 1: at each reference speed, resample loss onto the MGU-K torque axis (torque is the
    # irregular per-speed `airgap_torque` load sweep — sort it, then interp/extrap in torque).
    loss_by_speed = np.empty((n_speed, TORQUE_NM.size))
    for i in range(n_speed):
        order = np.argsort(tau[i])
        xt = tau[i][order]
        # Drop any duplicate torque coords (a stalled sweep) to keep xp strictly ascending.
        keep = np.concatenate(([True], np.diff(xt) > 1e-9))
        xt = xt[keep]
        loss_by_speed[i] = _interp_extrap(TORQUE_NM, xt, sys_loss[i][order][keep])

    # Stage 2: interp/extrap loss across reference speed onto the MGU-K speed axis (24 000–50 000 rpm
    # is extrapolated past the 20 000 rpm reference — the semi-virtual high-speed region), then derive
    # a consistent efficiency from loss and the mechanical operating power at each grid point.
    rows = []
    for tj, t in enumerate(TORQUE_NM):
        loss_col = np.maximum(_interp_extrap(SPEED_RPM, speed, loss_by_speed[:, tj]), 0.0)
        for si, s in enumerate(SPEED_RPM):
            loss = float(loss_col[si])
            p_mech = t * s * (np.pi / 30.0)  # τ·ω, W (signed: <0 in the regen quadrant)
            if p_mech > 0.0:  # drive: P_elec = P_mech + loss
                eff = p_mech / (p_mech + loss)
            elif p_mech < 0.0:  # regen: P_elec_recovered = |P_mech| − loss
                eff = (abs(p_mech) - loss) / abs(p_mech)
            else:  # τ·ω = 0 (standstill or zero torque): no mechanical output → η → 0
                eff = 0.0
            rows.append(
                {
                    "speed_rpm": float(s),
                    "torque_nm": float(t),
                    "efficiency": float(np.clip(eff, 1e-3, 0.999)),
                    "loss_w": loss,
                }
            )
    return pd.DataFrame(rows).sort_values(["speed_rpm", "torque_nm"]).reset_index(drop=True)


def build_emotor(src: Path) -> dict:
    """Distill the reference EDrive's detailed LPTN (via the PDT importer) into an `emotor/1.1`
    document, then scale every node's thermal capacity to the MGU-K's mass (the reference machine is
    heavier) — the *semi-virtual* thermal network: real topology/conductances/cooling, MGU-K mass."""
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        convert_edrive(
            src,
            tmp / "ref.ptm.yaml",
            vdc=MGU_K_VDC,
            maps_path=tmp / "ref.maps.parquet",
            emotor_out=tmp / "ref.emotor.yaml",
        )
        emotor = yaml.safe_load((tmp / "ref.emotor.yaml").read_text())
        ref_mass = float(yaml.safe_load((tmp / "ref.ptm.yaml").read_text())["mass_kg"])
    if ref_mass <= 0.0:
        raise RuntimeError("reference machine mass missing from the imported .ptm")
    scale = MGU_K_MASS_KG / ref_mass
    for node in emotor["nodes"]:
        if "c_j_per_k" in node:
            node["c_j_per_k"] = round(node["c_j_per_k"] * scale, 2)
    # Cooling calibration. The 6Et reference is a 121 kW EV traction machine — less efficient than a
    # real F1 MGU-K, so its imported loss surface is pessimistic and, left as-is, pins the winding at
    # the ~180 °C t_max cut-out and crushes deploy over a stint. So scale the conductive + jacket
    # cooling paths (×COOLING_SCALE) so a hard f1 stint settles the winding warm — ~170 °C, inside the
    # 155–180 °C derate band — but clear of the cut-out: the machine deploys its rated power when the
    # winding is cool early in a lap and tapers as it heats (a physical peak-power thermal limit).
    for edge in emotor.get("conductances", []):
        if "w_per_k" in edge:
            edge["w_per_k"] = round(edge["w_per_k"] * COOLING_SCALE, 4)
    if jacket := emotor.get("cooling", {}).get("jacket"):
        jacket["flow_rate_lps"] = round(jacket["flow_rate_lps"] * COOLING_SCALE, 5)
        jacket["wetted_area_m2"] = round(jacket["wetted_area_m2"] * COOLING_SCALE, 5)
    emotor["meta"] = {
        "source": "pdt_imported",
        "notes": (
            f"{src.name} detailed LPTN; node capacities scaled ×{scale:.3f} "
            f"({ref_mass:.1f}→{MGU_K_MASS_KG:.1f} kg), cooling paths ×{COOLING_SCALE:.1f} so a hard "
            f"stint settles the winding warm (~170 °C, in the derate band) but clear of the t_max "
            f"cut-out (semi-virtual f1 MGU-K, D-M6-13 L2)"
        ),
    }
    return emotor


# The synthetic MGU-K drive envelope (mgu_k.ptm `limits.max_torque_nm_vs_speed`), on the unit's
# OUTPUT SHAFT: constant 223 N·m to the 15 000 rpm crank redline, where τ·ω = 350 kW. The rotor's own
# constant-power taper (139 N·m @ 24 krpm, 67 N·m @ 50 krpm on the ROTOR axis) lives above the base
# speed of the lumped reduction and is therefore not reachable through this shaft — carrying it here
# implied the crank could turn to 50 000 rpm. The regen envelope follows the same shape; the
# reference only fixes its magnitude via the drive/regen peak ratio.
DRIVE_ENV = {
    "speed_rpm": [0.0, 15000.0],
    "torque_nm": [223.0, 223.0],
}


def regen_curve(src: Path) -> dict:
    """The MGU-K regen envelope. The reference machine's regen and drive peaks are measured to gauge
    asymmetry (`ratio = peak_regen / peak_drive`); regen then follows the synthetic DRIVE envelope's
    taper scaled by that ratio — so regen can never exceed drive at high speed (a symmetric machine,
    ratio≈1, gives regen == drive), which the flat reference (≤20 000 rpm, no taper) cannot express."""
    with h5py.File(src, "r") as f:
        vdc = np.atleast_1d(np.asarray(f["sweep/vdc"][()], dtype=float))
        iv = int(np.argmin(np.abs(vdc - MGU_K_VDC)))
        td = np.abs(np.asarray(f["peak_capability/torque_drive"][()], dtype=float)[iv])
        tr = np.abs(np.asarray(f["peak_capability/torque_regen"][()], dtype=float)[iv])
    ratio = float(np.nanmax(tr) / max(float(np.nanmax(td)), 1e-9))
    tq = [round(ratio * t, 1) for t in DRIVE_ENV["torque_nm"]]
    return {"speed_rpm": DRIVE_ENV["speed_rpm"], "torque_nm": tq, "ratio": round(ratio, 4)}


def _write_emotor(doc: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# SPDX-License-Identifier: CC-BY-SA-4.0\n"
        "# Semi-virtual f1_2026 MGU-K thermal network (D-M6-13 Layer 2): the reference EDrive's\n"
        "# detailed LPTN, node capacities scaled to the MGU-K mass. Derived from a private PDT .h5\n"
        "# (never committed); regenerate with python/tools/gen_f1_mgu_k.py.\n"
    )
    path.write_text(header + yaml.safe_dump(doc, sort_keys=False))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC, help="reference EDrive .h5")
    ap.add_argument("--no-fixtures", action="store_true", help="skip the schema-fixture mirrors")
    args = ap.parse_args(argv)
    if not args.src.exists():
        ap.error(f"reference .h5 not found: {args.src} (it is private to the author)")

    # 1) efficiency/loss sidecar (both the data-vehicle and the schema-fixture copies).
    df = build_semi_virtual(args.src)
    parquets = [OUT_PARQUET] + ([] if args.no_fixtures else [FIXTURE_PARQUET])
    for p in parquets:
        p.parent.mkdir(parents=True, exist_ok=True)
        df.to_parquet(p, index=False)
    # 2) semi-virtual `.emotor` thermal network (capacities scaled to the MGU-K mass).
    emotor = build_emotor(args.src)
    for p in [OUT_EMOTOR] + ([] if args.no_fixtures else [FIXTURE_EMOTOR]):
        _write_emotor(emotor, p)
    # 3) regen envelope (added to mgu_k.ptm.yaml `limits.max_regen_torque_nm_vs_speed`).
    regen = regen_curve(args.src)

    print(f"wrote {len(parquets)} parquet(s), {'1' if args.no_fixtures else '2'} emotor(s)")
    print(f"  grid: {SPEED_RPM.size} speeds × {TORQUE_NM.size} torques; "
          f"η∈[{df.efficiency.min():.3f},{df.efficiency.max():.3f}]")
    caps = {n['name']: n.get('c_j_per_k') for n in emotor['nodes']}
    print(f"  emotor nodes (c_j_per_k, MGU-K-scaled): {caps}")
    print(f"\n--- add to mgu_k.ptm.yaml `limits:` (regen/drive peak ratio = {regen['ratio']}) ---")
    print("  max_regen_torque_nm_vs_speed:")
    print(f"    speed_rpm: [{', '.join(f'{x:g}' for x in regen['speed_rpm'])}]")
    print(f"    torque_nm: [{', '.join(f'{x:g}' for x in regen['torque_nm'])}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
