# SPDX-License-Identifier: AGPL-3.0-only
"""The parsed ``.tir`` document model (mirror of the Rust ``TirDoc``/``TirSection``/``TirEntry``).

A value is either a ``float`` (rendered by the canonical number format on write) or a ``str``
(rendered single-quoted).
"""

from __future__ import annotations

from dataclasses import dataclass, field

TirValue = float | str


@dataclass
class TirEntry:
    """A single ``KEY = value`` entry (key uppercased)."""

    key: str
    value: TirValue


@dataclass
class TirSection:
    """A ``[SECTION]`` and its entries, in source order."""

    name: str
    entries: list[TirEntry] = field(default_factory=lambda: list[TirEntry]())


@dataclass
class TirDoc:
    """A parsed ``.tir`` document: ordered sections plus the diagnostics label."""

    label: str
    sections: list[TirSection] = field(default_factory=lambda: list[TirSection]())

    def section(self, name: str) -> TirSection | None:
        """The first section named ``name``, if present."""
        return next((s for s in self.sections if s.name == name), None)


class TirError(ValueError):
    """A hard ``.tir`` parse/convert error (malformed line, non-SI units, bad policy)."""
