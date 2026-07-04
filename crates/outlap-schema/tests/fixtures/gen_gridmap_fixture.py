# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the synthetic gridded-map parquet fixture for the sidecar decoder test.

Writes ``gridmap_2d.parquet`` — a long/tidy table mirroring exactly what the PDT drive-unit importer
emits (``speed_rpm, torque_nm, efficiency, loss_w`` as ``DOUBLE`` columns, default pyarrow settings =
SNAPPY compression + PLAIN/RLE_DICTIONARY encodings), on a small 3x4 grid with one NaN cell so the
Rust reader exercises the mask/fill path. Synthetic only — never derived from PDT data (firewall).

Run from anywhere:  python gen_gridmap_fixture.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

SPEED = np.array([1000.0, 2000.0, 3000.0])
TORQUE = np.array([-100.0, 0.0, 100.0, 200.0])


def main() -> None:
    ns, nt = SPEED.size, TORQUE.size
    speed = np.repeat(SPEED, nt)
    torque = np.tile(TORQUE, ns)
    # A smooth, reproducible field; the Rust test cross-checks node exactness against these values.
    eff = 0.95 - 5e-5 * speed - 2e-4 * np.abs(torque)
    loss = 1.0e-3 * speed + 0.5 * np.abs(torque)
    # One NaN cell (speed=1000, torque=200) to exercise NaN masking + hull flagging.
    mask = (speed == 1000.0) & (torque == 200.0)
    eff = np.where(mask, np.nan, eff)
    loss = np.where(mask, np.nan, loss)
    table = pa.table(
        {
            "speed_rpm": speed.astype(np.float64),
            "torque_nm": torque.astype(np.float64),
            "efficiency": eff.astype(np.float64),
            "loss_w": loss.astype(np.float64),
        }
    )
    out = Path(__file__).with_name("gridmap_2d.parquet")
    pq.write_table(table, out)
    print(f"wrote {out} ({out.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
