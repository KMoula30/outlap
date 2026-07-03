# SPDX-License-Identifier: AGPL-3.0-only
"""PDT HDF5 importers (§10): EDrive/DriveUnit → ``.ptm``, BatteryPack → provisional ``battery.yaml``.

Pure-Python adapters (``h5py`` + ``numpy`` + ``pyarrow`` only) that read the documented PDT schema
and never import PDT code or commit real PDT files (firewall, §1). Surfaced through the unified
``outlap`` CLI (Decision #19); for now via ``python -m outlap.importers.pdt_h5``.
"""

from __future__ import annotations

from .battery import convert_batterypack, validate_battery_doc
from .common import PdtImportError
from .driveunit import convert_driveunit
from .edrive import convert_edrive

__all__ = [
    "PdtImportError",
    "convert_batterypack",
    "convert_driveunit",
    "convert_edrive",
    "validate_battery_doc",
]
