# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the SYNTHETIC f1_2026 ICE brake-thermal efficiency/loss sidecar.

Writes ``data/vehicles/f1_2026/ptm/tables/ice_v6.parquet`` — the efficiency (+ consistent
loss) map the ``ptm/ice_v6.ptm.yaml`` document references, over its declared
``speed_rpm × torque_nm`` grid. The map is what makes the M6/PR5 fuel-mass slow state LIVE:
``ṁ_fuel = source_w / LHV`` with ``source_w = P_mech / η_thermal`` (§8.1, docs/theory/fuel-mass.md).

Everything here is **synthetic** — an invented smooth brake-thermal-efficiency surface, NOT
measured data and NOT derived from any PDT export (firewall, CLAUDE.md hard rule #1). Only the
headline scale is a public fact: a modern turbo-hybrid F1 ICE peaks near ~0.40 brake-thermal
efficiency at high load; this surface peaks ≈0.40 at high BMEP / mid crank speed, droops at part
load and toward the rpm extremes, and is 0 at the τ=0 spin point with a small idle fuel draw (the
PDT-importer idiom). All ESTIMATED.

Idiom notes (match ``gen_model3_powertrain.py``):
* The efficiency/loss pair is emitted CONSISTENT so ``source = mech + loss`` holds exactly at grid
  nodes (drive: ``loss = P_mech·(1/η − 1)``; τ=0: an idle draw).
* Long/tidy DOUBLE columns, default pyarrow settings (SNAPPY), matching the importers.

Run from anywhere:  python python/tools/gen_f1_ice.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

_ROOT = Path(__file__).resolve().parents[2]
_VEH = _ROOT / "data" / "vehicles" / "f1_2026"

RPM_TO_RAD = np.pi / 30.0

# The ice_v6.ptm.yaml grid.
SPEED_RPM = np.array([1000.0, 4000.0, 8000.0, 12000.0, 15000.0])
TORQUE_NM = np.array([0.0, 100.0, 200.0, 300.0, 400.0])
TAU_PK = 400.0
ETA_PEAK = 0.40


def _eta(speed: float, tau: float) -> float:
    """Synthetic ICE brake-thermal efficiency: 0 at the spin point, rising with load (BMEP), a
    gentle crank-speed hump peaking near 9 000 rpm, clipped to a physical band."""
    if tau <= 0.0:
        return 0.0
    load = (tau / TAU_PK) ** 0.35  # rises with BMEP, concave
    rpm_hump = 1.0 - 0.18 * ((speed - 9000.0) / 8000.0) ** 2
    return float(np.clip(ETA_PEAK * load * rpm_hump, 0.12, ETA_PEAK))


def _consistent_loss(speed: float, tau: float, eta: float) -> float:
    """The loss (W) that closes ``source = mech + loss`` at this node (drive; τ=0 an idle draw)."""
    if tau <= 0.0:
        return 4000.0 + 0.5 * speed  # ICE idle fuel draw (chemical power at no shaft output)
    p_mech = tau * (speed * RPM_TO_RAD)
    return p_mech * (1.0 / eta - 1.0)


def _emit_ice_map(path: Path) -> None:
    ns, nt = SPEED_RPM.size, TORQUE_NM.size
    speed_c = np.repeat(SPEED_RPM, nt)
    torque_c = np.tile(TORQUE_NM, ns)
    eff = np.empty(ns * nt)
    loss = np.empty(ns * nt)
    for k in range(eff.size):
        s, t = float(speed_c[k]), float(torque_c[k])
        e = _eta(s, t)
        eff[k] = e
        loss[k] = _consistent_loss(s, t, e if e > 0.0 else 1.0)
    table = pa.table(
        {
            "speed_rpm": speed_c.astype(np.float64),
            "torque_nm": torque_c.astype(np.float64),
            "efficiency": eff.astype(np.float64),
            "loss_w": loss.astype(np.float64),
        }
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, path)
    print(f"wrote {path} ({eff.size} rows, η ∈ [{eff[eff > 0].min():.3f}, {eff.max():.3f}])")


def main() -> None:
    _emit_ice_map(_VEH / "ptm" / "tables" / "ice_v6.parquet")


if __name__ == "__main__":
    main()
