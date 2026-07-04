# SPDX-License-Identifier: AGPL-3.0-only
"""The canonical ``.tir`` mapping table — Python mirror of ``outlap-schema``'s ``tir/map.rs``.

Section order, per-section key order, and the metadata/coefficient split MUST stay identical to
the Rust table: the byte-parity contract (``synthetic_slick.canonical.tir``) pins both writers to
the same output. Change the two tables only together, with a PR note.
"""

from __future__ import annotations

UNITS_SECTION = "UNITS"
MODEL_SECTION = "MODEL"
#: Catch-all for coefficients the table does not place (uppercased, sorted on write).
OVERFLOW_SECTION = "USER_COEFFICIENTS"

#: The canonical SI ``[UNITS]`` declaration (dimension -> SI token), in write order.
SI_UNITS: list[tuple[str, str]] = [
    ("LENGTH", "meter"),
    ("FORCE", "newton"),
    ("ANGLE", "radians"),
    ("MASS", "kg"),
    ("TIME", "second"),
]

#: Sections whose entries are metadata (never flattened into the coefficient map on read).
METADATA_SECTIONS = (UNITS_SECTION, MODEL_SECTION)

#: ``[MODEL]`` metadata the writer emits for an MF6.1 tyre; FITTYP is numeric.
MODEL_METADATA: list[tuple[str, float | str]] = [
    ("FITTYP", 61.0),
    ("PROPERTY_FILE_FORMAT", "MF61"),
]

#: Canonical section order and per-section key order (mirror of ``map.rs::SECTIONS``).
SECTIONS: list[tuple[str, list[str]]] = [
    (MODEL_SECTION, []),
    (UNITS_SECTION, ["LENGTH", "FORCE", "ANGLE", "MASS", "TIME"]),
    ("DIMENSION", ["UNLOADED_RADIUS", "WIDTH", "RIM_RADIUS"]),
    ("OPERATING_CONDITIONS", ["NOMPRES", "LONGVL", "VXLOW"]),
    ("VERTICAL", ["FNOMIN", "VERTICAL_STIFFNESS"]),
    (
        "SCALING_COEFFICIENTS",
        [
            "LFZO",
            "LCX",
            "LMUX",
            "LEX",
            "LKX",
            "LHX",
            "LVX",
            "LCY",
            "LMUY",
            "LEY",
            "LKY",
            "LKYC",
            "LKZC",
            "LHY",
            "LVY",
            "LTR",
            "LRES",
            "LXAL",
            "LYKA",
            "LVYKA",
            "LS",
            "LMX",
            "LMY",
            "LVMX",
            "LGYR",
            "LSGKP",
            "LSGAL",
        ],
    ),
    (
        "LONGITUDINAL_COEFFICIENTS",
        [
            "PCX1",
            "PDX1",
            "PDX2",
            "PDX3",
            "PEX1",
            "PEX2",
            "PEX3",
            "PEX4",
            "PKX1",
            "PKX2",
            "PKX3",
            "PHX1",
            "PHX2",
            "PVX1",
            "PVX2",
            "PPX1",
            "PPX2",
            "PPX3",
            "PPX4",
            "RBX1",
            "RBX2",
            "RBX3",
            "RCX1",
            "REX1",
            "REX2",
            "RHX1",
            "PTX1",
            "PTX2",
            "PTX3",
        ],
    ),
    (
        "OVERTURNING_COEFFICIENTS",
        [
            "QSX1",
            "QSX2",
            "QSX3",
            "QSX4",
            "QSX5",
            "QSX6",
            "QSX7",
            "QSX8",
            "QSX9",
            "QSX10",
            "QSX11",
            "PPMX1",
        ],
    ),
    (
        "LATERAL_COEFFICIENTS",
        [
            "PCY1",
            "PDY1",
            "PDY2",
            "PDY3",
            "PEY1",
            "PEY2",
            "PEY3",
            "PEY4",
            "PEY5",
            "PKY1",
            "PKY2",
            "PKY3",
            "PKY4",
            "PKY5",
            "PKY6",
            "PKY7",
            "PHY1",
            "PHY2",
            "PVY1",
            "PVY2",
            "PVY3",
            "PVY4",
            "PPY1",
            "PPY2",
            "PPY3",
            "PPY4",
            "PPY5",
            "RBY1",
            "RBY2",
            "RBY3",
            "RBY4",
            "RCY1",
            "REY1",
            "REY2",
            "RHY1",
            "RHY2",
            "RVY1",
            "RVY2",
            "RVY3",
            "RVY4",
            "RVY5",
            "RVY6",
            "PTY1",
            "PTY2",
        ],
    ),
    (
        "ROLLING_COEFFICIENTS",
        ["QSY1", "QSY2", "QSY3", "QSY4", "QSY5", "QSY6", "QSY7", "QSY8"],
    ),
    (
        "ALIGNING_COEFFICIENTS",
        [
            "QBZ1",
            "QBZ2",
            "QBZ3",
            "QBZ4",
            "QBZ5",
            "QBZ6",
            "QBZ9",
            "QBZ10",
            "QCZ1",
            "QDZ1",
            "QDZ2",
            "QDZ3",
            "QDZ4",
            "QDZ6",
            "QDZ7",
            "QDZ8",
            "QDZ9",
            "QDZ10",
            "QDZ11",
            "QEZ1",
            "QEZ2",
            "QEZ3",
            "QEZ4",
            "QEZ5",
            "QHZ1",
            "QHZ2",
            "QHZ3",
            "QHZ4",
            "PPZ1",
            "PPZ2",
            "SSZ1",
            "SSZ2",
            "SSZ3",
            "SSZ4",
        ],
    ),
    (
        "STRUCTURAL",
        [
            "LONGITUDINAL_STIFFNESS",
            "LATERAL_STIFFNESS",
            "PCFX1",
            "PCFX2",
            "PCFX3",
            "PCFY1",
            "PCFY2",
            "PCFY3",
        ],
    ),
]

_SECTION_OF: dict[str, str] = {key: name for name, keys in SECTIONS for key in keys}


def is_coefficient_section(section: str) -> bool:
    """Whether a section's entries are MF6.1 coefficients (flattened on read)."""
    return section not in METADATA_SECTIONS


def section_for(key: str) -> str | None:
    """The canonical section for a known coefficient key (``None`` -> overflow section)."""
    return _SECTION_OF.get(key)
