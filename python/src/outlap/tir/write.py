# SPDX-License-Identifier: AGPL-3.0-only
"""The deterministic ``.tir`` writer — byte-compatible with the Rust writer.

Every number is rendered by the single canonical :func:`format_number`, whose rules are the
cross-language contract documented in ``outlap-schema``'s ``tir/mod.rs``:

1. ``+/-0.0`` -> ``"0"``.
2. Otherwise take the shortest round-tripping decimal significand ``d1..dn`` (round-half-to-even
   ties — exactly CPython ``repr``) and the base-10 exponent ``E`` of the leading digit.
3. Plain decimal when ``-4 <= E <= 15`` (no exponent, no forced trailing ``.0``).
4. Scientific otherwise: ``mantissa e±EE``, sign always present, exponent >= 2 digits.

Output layout is a pure function of the document: sections in :data:`outlap.tir.map.SECTIONS`
order (unlisted sections appended name-sorted), keys in declared order (unlisted keys sorted),
``KEY = value`` with one space each side, text values single-quoted, one blank line between
sections. The committed ``synthetic_slick.canonical.tir`` fixture pins both writers.

Known shared limitation (both codecs, tracked for a follow-up): text values are emitted
single-quoted but NOT escaped — a value containing a quote, ``$``/``!``, or a newline is not
representable and will not survive a write→parse round trip. Coefficient data is numeric, and
the regenerated ``[MODEL]``/``[UNITS]`` metadata is fixed, so canonical documents are safe.
"""

from __future__ import annotations

import math

from .doc import TirDoc, TirEntry, TirSection
from .map import SECTIONS


def format_number(x: float) -> str:
    """Render ``x`` in the canonical cross-language number format (see module docs)."""
    if x == 0.0:
        return "0"
    if math.isnan(x):
        return "nan"
    if math.isinf(x):
        return "inf" if x > 0.0 else "-inf"

    neg = x < 0.0
    digits, exp = _normalize(repr(abs(x)))
    out = "-" if neg else ""
    if -4 <= exp <= 15:
        return out + _render_plain(digits, exp)
    return out + _render_scientific(digits, exp)


def _normalize(text: str) -> tuple[str, int]:
    """Shortest significand ``d1..dn`` (no leading/trailing zeros) and leading-digit exponent.

    ``repr`` of a positive finite float is either fixed (``"4000.0"``, ``"0.0001"``) or
    scientific (``"1e+16"``, ``"2.2250738585072014e-308"``); both are handled.
    """
    mantissa, _, exp_str = text.partition("e")
    exp10 = int(exp_str) if exp_str else 0
    int_part, _, frac_part = mantissa.partition(".")
    combined = int_part + frac_part
    point = exp10 - len(frac_part)
    first = next(i for i, c in enumerate(combined) if c != "0")
    exp = (len(combined) - 1 - first) + point
    sig = combined[first:].rstrip("0") or "0"
    return sig, exp


def _render_plain(digits: str, exp: int) -> str:
    n = len(digits)
    if exp >= n - 1:
        return digits + "0" * (exp - (n - 1))
    if exp >= 0:
        split = exp + 1
        return digits[:split] + "." + digits[split:]
    return "0." + "0" * (-exp - 1) + digits


def _render_scientific(digits: str, exp: int) -> str:
    mant = digits[0] + ("." + digits[1:] if len(digits) > 1 else "")
    return f"{mant}e{'-' if exp < 0 else '+'}{abs(exp):02d}"


def write_tir(doc: TirDoc) -> str:
    """Serialise ``doc`` to canonical ``.tir`` text (see the module docs for the exact format)."""
    chunks: list[str] = []
    listed = {name for name, _ in SECTIONS}
    for name, keys in SECTIONS:
        section = doc.section(name)
        if section is not None:
            chunks.append(_write_section(section, keys))
    for section in sorted(
        (s for s in doc.sections if s.name not in listed), key=lambda s: s.name
    ):
        chunks.append(_write_section(section, []))
    return "\n".join(chunks)


def _write_section(section: TirSection, ordered_keys: list[str]) -> str:
    lines = [f"[{section.name}]"]
    # First occurrence wins for a duplicated key, matching the Rust writer's `find`.
    by_key: dict[str, TirEntry] = {}
    for entry in section.entries:
        by_key.setdefault(entry.key, entry)
    for key in ordered_keys:
        if key in by_key:
            lines.append(_write_entry(by_key[key]))
    rest = sorted(
        (e for e in section.entries if e.key not in ordered_keys), key=lambda e: e.key
    )
    lines.extend(_write_entry(e) for e in rest)
    return "\n".join(lines) + "\n"


def _write_entry(entry: TirEntry) -> str:
    # bool is an int subclass; only genuine numerics take the number path (a runtime int is
    # rendered as its float, keeping parity with the Rust enum where ints are unrepresentable).
    if isinstance(entry.value, float | int) and not isinstance(entry.value, bool):
        return f"{entry.key} = {format_number(float(entry.value))}"
    return f"{entry.key} = '{entry.value}'"
