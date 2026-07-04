# SPDX-License-Identifier: AGPL-3.0-only
"""The ``.tir`` interchange codec — Python mirror of ``outlap-schema``'s ``tir`` module.

String-in/string-out parse/write plus ``.tyr``-dict conversion, stdlib + pyyaml only (pyyaml is
used by the CLI, not the codec itself). The writer is **byte-compatible** with the Rust writer:
the shared canonical fixture (``crates/outlap-schema/tests/fixtures/tir/
synthetic_slick.canonical.tir``) pins both, and the canonical number format is specified in
``outlap-schema``'s ``tir/mod.rs`` (this side implements it via CPython ``repr``, whose
round-half-to-even shortest decimals match the Rust writer's ``ryu`` digits exactly).

CLI: ``python -m outlap.tir {to-tyr, from-tyr}``.
"""

from __future__ import annotations

from .convert import (
    SYNTHETIC_THERMAL,
    SYNTHETIC_WEAR,
    ThermalWearPolicy,
    tir_to_tyr,
    tyr_to_tir,
)
from .doc import TirDoc, TirEntry, TirError, TirSection, TirValue
from .parse import parse_tir
from .write import format_number, write_tir

__all__ = [
    "SYNTHETIC_THERMAL",
    "SYNTHETIC_WEAR",
    "ThermalWearPolicy",
    "TirDoc",
    "TirEntry",
    "TirError",
    "TirSection",
    "TirValue",
    "format_number",
    "parse_tir",
    "tir_to_tyr",
    "tyr_to_tir",
    "write_tir",
]
