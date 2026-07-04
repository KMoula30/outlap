# SPDX-License-Identifier: AGPL-3.0-only
"""``.tir`` <-> ``.tyr`` conversion — mirror of ``outlap-schema``'s ``tir_to_tyr``/``tyr_to_tir``.

The ``.tyr`` side is the plain-dict form of the YAML document (as ``yaml.safe_load`` returns it),
NOT a typed model: the Rust ``outlap-schema`` pipeline owns validation. The documented round-trip
asymmetry holds here exactly as in Rust: ``.tir`` carries no ``thermal``/``wear`` physics, so
``tir_to_tyr`` synthesises those blocks per a policy and ``tyr_to_tir`` drops them again,
regenerating only the ``[MODEL]``/``[UNITS]`` metadata.
"""

from __future__ import annotations

from typing import Any, Literal

from .doc import TirDoc, TirEntry, TirError, TirSection
from .map import (
    MODEL_METADATA,
    MODEL_SECTION,
    OVERFLOW_SECTION,
    SECTIONS,
    SI_UNITS,
    UNITS_SECTION,
    is_coefficient_section,
    section_for,
)

ThermalWearPolicy = Literal["synthetic", "from-donor", "none"]

#: Synthetic thermal-ring placeholder (racing-slick band) — mirror of the Rust defaults.
SYNTHETIC_THERMAL: dict[str, float] = {
    "c_s": 8000.0,
    "c_c": 22000.0,
    "c_g": 1500.0,
    "g_sc": 90.0,
    "g_cg": 40.0,
    "g_road": 250.0,
    "h0": 15.0,
    "h1": 5.5,
    "p_t": 0.65,
    "t_opt": 95.0,
    "c_t": 2.2,
    "k_c": 0.0015,
    "t_c_ref": 80.0,
    "p_cold": 138.0,
    "t_cold": 20.0,
}

#: Synthetic wear/cliff placeholder (racing-slick band) — mirror of the Rust defaults.
SYNTHETIC_WEAR: dict[str, float] = {
    "k_w": 0.0009,
    "w_max": 8.0,
    "w_c": 2.0,
    "tau_d": 600.0,
    "t_deg": 120.0,
    "delta_t_ref": 20.0,
    "beta": 2.0,
    "delta_c": 0.25,
    "s_w": 0.5,
    "delta_d": 0.30,
}


def tir_to_tyr(
    doc: TirDoc,
    policy: ThermalWearPolicy = "synthetic",
    donor: dict[str, Any] | None = None,
) -> tuple[dict[str, Any], list[str]]:
    """Convert a parsed :class:`TirDoc` into a ``.tyr`` document dict.

    ``policy`` decides the ``thermal``/``wear`` blocks a ``.tir`` cannot carry: ``synthetic``
    fills documented placeholders (provenance marked synthetic), ``from-donor`` copies them from
    ``donor`` (a loaded ``.tyr`` dict), ``none`` raises. Returns ``(tyr, warnings)``.
    """
    warnings: list[str] = []
    mf61: dict[str, float] = {}
    for section in doc.sections:
        if not is_coefficient_section(section.name):
            continue
        for entry in section.entries:
            if isinstance(entry.value, float):
                if section_for(entry.key) is None:
                    warnings.append(
                        f"unknown MF6.1 coefficient `{entry.key}` — carried through"
                        " unvalidated"
                    )
                mf61[entry.key] = entry.value
            else:
                warnings.append(
                    f"non-numeric value for coefficient `{entry.key}` in "
                    f"`[{section.name}]` — ignored"
                )

    if policy == "synthetic":
        thermal: dict[str, float] = dict(SYNTHETIC_THERMAL)
        wear: dict[str, float] = dict(SYNTHETIC_WEAR)
        synthetic = True
        note = "thermal/wear synthesised (not present in `.tir`)"
    elif policy == "from-donor":
        if donor is None or "thermal" not in donor or "wear" not in donor:
            raise TirError(
                "from-donor policy requires a donor `.tyr` with thermal/wear"
            )
        thermal = dict(donor["thermal"])
        wear = dict(donor["wear"])
        synthetic = False
        note = "thermal/wear taken from donor `.tyr` (not present in `.tir`)"
    elif policy == "none":
        raise TirError(
            "`.tir` carries no thermal/wear model — choose the `synthetic` or "
            "`from-donor` policy to build a `.tyr`"
        )
    else:  # pragma: no cover - Literal narrows, but guard the runtime path.
        raise TirError(f"unknown thermal/wear policy `{policy}`")
    warnings.append(note)

    tyr: dict[str, Any] = {
        "schema": "tyr/1.0",
        "mf61": dict(sorted(mf61.items())),
        "thermal": thermal,
        "wear": wear,
        "provenance": {
            "citation": "MF6.1 coefficients imported from a `.tir` file",
            "source": f"imported from `.tir` `{doc.label}`",
            "synthetic": synthetic,
        },
    }
    return tyr, warnings


def tyr_to_tir(tyr: dict[str, Any]) -> TirDoc:
    """Convert a ``.tyr`` document dict into a :class:`TirDoc` (drops thermal/wear/brush)."""
    mf61_raw: dict[str, Any] = tyr.get("mf61", {})
    mf61: dict[str, float] = {str(k): float(v) for k, v in mf61_raw.items()}

    sections: list[TirSection] = []
    for name, keys in SECTIONS:
        entries: list[TirEntry]
        if name == MODEL_SECTION:
            entries = [TirEntry(k, v) for k, v in MODEL_METADATA]
        elif name == UNITS_SECTION:
            entries = [TirEntry(dim, si) for dim, si in SI_UNITS]
        else:
            entries = [TirEntry(k, mf61[k]) for k in keys if k in mf61]
        if entries:
            sections.append(TirSection(name, entries))

    overflow = sorted((k.upper(), v) for k, v in mf61.items() if section_for(k) is None)
    if overflow:
        sections.append(
            TirSection(OVERFLOW_SECTION, [TirEntry(k, v) for k, v in overflow])
        )

    return TirDoc("<tyr>", sections)
