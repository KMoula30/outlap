# SPDX-License-Identifier: AGPL-3.0-only
"""The ``.tir`` text parser — grammar-identical to the Rust parser in ``outlap-schema``.

* ``[SECTION]`` headers, ``KEY = value`` entries, comments, or blank lines.
* Comments start at an unquoted ``$`` or ``!`` and run to end of line.
* Values are single/double-quoted strings or bare finite numbers; anything else stays text.
* Leading BOM and CRLF endings are tolerated; keys and section names are uppercased.
* Duplicate key in a section = last-wins plus a warning; unknown sections warn (did-you-mean).
* Hard errors (:class:`~outlap.tir.doc.TirError`): malformed line, entry before any section,
  malformed header, and a non-SI ``[UNITS]`` declaration.
"""

from __future__ import annotations

import difflib
import math

from .doc import TirDoc, TirEntry, TirError, TirSection, TirValue
from .map import OVERFLOW_SECTION, SECTIONS, SI_UNITS, UNITS_SECTION

_SI = dict(SI_UNITS)
_KNOWN_SECTIONS = {name for name, _ in SECTIONS} | {OVERFLOW_SECTION}


def parse_tir(label: str, content: str) -> tuple[TirDoc, list[str]]:
    """Parse ``.tir`` text into a :class:`TirDoc`, returning ``(doc, warnings)``."""
    warnings: list[str] = []
    sections: list[TirSection] = []

    content = content.removeprefix("\ufeff")

    # Split on \n only (with CRLF tolerance), exactly like the Rust parser: str.splitlines()
    # would additionally split on \f, \v, \x85, \u2028... and lone \r, silently classifying
    # inputs the Rust codec rejects.
    for lineno, raw in enumerate(content.split("\n"), start=1):
        raw = raw.removesuffix("\r")
        code = _strip_comment(raw).strip()
        if not code:
            continue

        if code.startswith("["):
            name = _parse_header(label, lineno, code)
            if name not in _KNOWN_SECTIONS:
                hint = difflib.get_close_matches(name, sorted(_KNOWN_SECTIONS), n=1)
                suffix = f" (did you mean `[{hint[0]}]`?)" if hint else ""
                warnings.append(
                    f"unknown `.tir` section `[{name}]`{suffix} — carried through"
                )
            sections.append(TirSection(name))
            continue

        key_raw, eq, value_raw = code.partition("=")
        if not eq:
            raise _error(
                label, lineno, "expected `[SECTION]`, `KEY = value`, or a comment"
            )
        key = key_raw.strip().upper()
        if not key:
            raise _error(label, lineno, "empty key before `=`")
        value_str = value_raw.strip()
        if not value_str:
            raise _error(label, lineno, f"`{key}` has no value after `=`")
        value = _parse_value(value_str)

        if not sections:
            raise _error(
                label, lineno, f"`{key}` appears before any `[SECTION]` header"
            )
        section = sections[-1]

        if section.name == UNITS_SECTION:
            _check_si(label, lineno, key, value)

        existing = next((e for e in section.entries if e.key == key), None)
        if existing is not None:
            warnings.append(
                f"duplicate key `{key}` in `[{section.name}]` — keeping the last value"
            )
            existing.value = value
        else:
            section.entries.append(TirEntry(key, value))

    return TirDoc(label, sections), warnings


def _error(label: str, lineno: int, message: str) -> TirError:
    return TirError(f"{label}:{lineno}: {message}")


def _parse_header(label: str, lineno: int, code: str) -> str:
    if code.startswith("[") and code.endswith("]"):
        inner = code[1:-1].strip()
        if inner:
            return inner.upper()
    raise _error(label, lineno, "malformed section header: expected `[SECTION_NAME]`")


def _parse_value(s: str) -> TirValue:
    for quote in ("'", '"'):
        if len(s) >= 2 and s.startswith(quote) and s.endswith(quote):
            return s[1:-1]
    # Python's float() accepts underscore separators and non-ASCII (Unicode) digits; Rust's
    # f64 parse accepts neither — reject both so the parsers classify identically.
    if "_" in s or not s.isascii():
        return s
    try:
        n = float(s)
    except ValueError:
        return s
    return n if math.isfinite(n) else s


def _check_si(label: str, lineno: int, key: str, value: TirValue) -> None:
    si = _SI.get(key)
    if si is None:
        return
    if not (isinstance(value, str) and value.lower() == si.lower()):
        raise _error(
            label,
            lineno,
            f"non-SI `[UNITS]` declaration: `{key}` must be `{si}` "
            "(outlap works in SI internally)",
        )


def _strip_comment(line: str) -> str:
    quote: str | None = None
    for i, ch in enumerate(line):
        if quote is not None:
            if ch == quote:
                quote = None
        elif ch in ("'", '"'):
            quote = ch
        elif ch in ("$", "!"):
            return line[:i]
    return line
