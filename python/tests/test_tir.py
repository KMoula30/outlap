# SPDX-License-Identifier: AGPL-3.0-only
"""`.tir` codec tests: byte-parity with the Rust writer, round-trips, diagnostics, CLI."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from outlap.tir import (
    TirError,
    format_number,
    parse_tir,
    tir_to_tyr,
    tyr_to_tir,
    write_tir,
)
from outlap.tir.__main__ import main as tir_main

_ROOT = Path(__file__).resolve().parents[2]
_TIR_FIXTURES = _ROOT / "crates" / "outlap-schema" / "tests" / "fixtures" / "tir"
_DATA_TIRES = _ROOT / "data" / "tires"


# --- Byte parity (the load-bearing cross-language contract) -------------------------------


def test_byte_parity_with_rust_writer() -> None:
    """write(parse(fixture)) must equal the Rust-writer-generated canonical bytes exactly."""
    src = (_TIR_FIXTURES / "synthetic_slick.tir").read_text(encoding="utf-8")
    canonical = (_TIR_FIXTURES / "synthetic_slick.canonical.tir").read_text(
        encoding="utf-8"
    )
    doc, _ = parse_tir("synthetic_slick.tir", src)
    assert write_tir(doc) == canonical


def test_canonical_output_is_a_fixed_point() -> None:
    src = (_TIR_FIXTURES / "synthetic_slick.tir").read_text(encoding="utf-8")
    doc, _ = parse_tir("t", src)
    once = write_tir(doc)
    doc2, warnings = parse_tir("t", once)
    assert not warnings, f"canonical output re-parsed with warnings: {warnings}"
    assert write_tir(doc2) == once


def test_format_number_canonical_cases() -> None:
    # The documented format: plain for decimal exponent -4..=15, else Python-repr scientific.
    cases = [
        (0.0, "0"),
        (-0.0, "0"),
        (4000.0, "4000"),
        (22.0, "22"),
        (1.65, "1.65"),
        (0.33, "0.33"),
        (-20.0, "-20"),
        (0.0009, "0.0009"),
        (0.0001, "0.0001"),
        (1e15, "1000000000000000"),
        (1e-5, "1e-05"),
        (1.5e-5, "1.5e-05"),
        (1e16, "1e+16"),
        (1.25e16, "1.25e+16"),
        (-3e-9, "-3e-09"),
        (1e100, "1e+100"),
        (1e-300, "1e-300"),
        # Shortest-decimal TIE values: round-half-to-even, matching the Rust writer's ryu
        # digits (Rust's own `{:e}` would round these away-from-even).
        (686995158985696.25, "686995158985696.2"),
        (17493296845004.062, "17493296845004.062"),
        (161221913628319.62, "161221913628319.62"),
    ]
    for value, expected in cases:
        assert format_number(value) == expected, value


def test_format_number_round_trips() -> None:
    for x in [1.0 / 3.0, 3.141592653589793, 1.2345678901234567e-8, 9.876e21, 5e-324]:
        assert float(format_number(x)) == x


# --- Round-trips over the committed reference tyres ---------------------------------------


def _reference_tyres() -> list[Path]:
    found = sorted(_DATA_TIRES.rglob("*.tyr.yaml"))
    assert len(found) >= 2, "expected the Pacejka + Roborace reference tyres"
    return found


def test_reference_tyres_round_trip_numeric_exact() -> None:
    for path in _reference_tyres():
        tyr = yaml.safe_load(path.read_text(encoding="utf-8"))
        text = write_tir(tyr_to_tir(tyr))
        doc, warnings = parse_tir(str(path), text)
        assert not warnings, f"{path}: {warnings}"
        back, _ = tir_to_tyr(doc)
        original = {str(k): float(v) for k, v in tyr["mf61"].items()}
        assert back["mf61"] == dict(sorted(original.items())), path


# --- Conversion policies -------------------------------------------------------------------


def test_synthetic_policy_marks_provenance() -> None:
    src = (_TIR_FIXTURES / "synthetic_slick.tir").read_text(encoding="utf-8")
    doc, _ = parse_tir("t", src)
    tyr, warnings = tir_to_tyr(doc, "synthetic")
    assert tyr["provenance"]["synthetic"] is True
    assert "thermal" in tyr and "wear" in tyr
    assert any("synthesised" in w for w in warnings)


def test_none_policy_raises() -> None:
    src = (_TIR_FIXTURES / "synthetic_slick.tir").read_text(encoding="utf-8")
    doc, _ = parse_tir("t", src)
    with pytest.raises(TirError, match="thermal/wear"):
        tir_to_tyr(doc, "none")


def test_donor_policy_copies_blocks() -> None:
    src = (_TIR_FIXTURES / "synthetic_slick.tir").read_text(encoding="utf-8")
    doc, _ = parse_tir("t", src)
    donor = yaml.safe_load(_reference_tyres()[0].read_text(encoding="utf-8"))
    tyr, _ = tir_to_tyr(doc, "from-donor", donor)
    assert tyr["thermal"] == donor["thermal"]
    assert tyr["provenance"]["synthetic"] is False


# --- Grammar / diagnostics ------------------------------------------------------------------


def test_non_si_units_is_a_hard_error() -> None:
    with pytest.raises(TirError, match="LENGTH.*meter"):
        parse_tir("t", "[UNITS]\nLENGTH = 'mm'\n")


def test_malformed_line_is_a_hard_error() -> None:
    with pytest.raises(TirError, match="expected"):
        parse_tir("t", "[DIMENSION]\nno equals sign here\n")


def test_duplicate_key_last_wins_with_warning() -> None:
    doc, warnings = parse_tir("t", "[VERTICAL]\nFNOMIN = 1\nFNOMIN = 2\n")
    assert any("duplicate key `FNOMIN`" in w for w in warnings)
    section = doc.section("VERTICAL")
    assert section is not None and section.entries[0].value == 2.0


def test_bom_crlf_comments_quotes() -> None:
    text = "\ufeff[MODEL]\r\nTYRESIDE = 'LEFT' $ inline\r\n! full-line\r\n[VERTICAL]\r\nFNOMIN = 3000\r\n"
    doc, _ = parse_tir("t", text)
    model = doc.section("MODEL")
    vertical = doc.section("VERTICAL")
    assert model is not None and model.entries[0].value == "LEFT"
    assert vertical is not None and vertical.entries[0].value == 3000.0


def test_unicode_digits_stay_text_like_rust() -> None:
    # CPython float() accepts non-ASCII digits; the Rust parser does not — parity requires text.
    doc, _ = parse_tir("t", "[VERTICAL]\nFNOMIN = \u0661\u0662\u0663\n")
    section = doc.section("VERTICAL")
    assert section is not None and isinstance(section.entries[0].value, str)


def test_formfeed_and_lone_cr_do_not_split_lines() -> None:
    # Only \n (with CRLF tolerance) terminates a line, exactly like the Rust parser: a form
    # feed inside a value keeps it one (text) entry, not two.
    doc, _ = parse_tir("t", "[DIMENSION]\nUNLOADED_RADIUS = 0.3\fWIDTH = 0.2\n")
    section = doc.section("DIMENSION")
    assert section is not None
    assert len(section.entries) == 1
    assert isinstance(section.entries[0].value, str)


def test_writer_emits_first_duplicate_like_rust() -> None:
    from outlap.tir import TirDoc, TirEntry, TirSection

    doc = TirDoc(
        "t",
        [TirSection("DIMENSION", [TirEntry("WIDTH", 0.2), TirEntry("WIDTH", 0.9)])],
    )
    assert "WIDTH = 0.2" in write_tir(doc)
    assert "WIDTH = 0.9" not in write_tir(doc)


def test_unknown_coefficient_warns_in_conversion() -> None:
    doc, _ = parse_tir("t", "[VERTICAL]\nFNOMIN = 3000\nWIBBLE = 1\n")
    _, warnings = tir_to_tyr(doc)
    assert any("unknown MF6.1 coefficient `WIBBLE`" in w for w in warnings)


def test_unknown_section_warns_with_suggestion() -> None:
    _, warnings = parse_tir("t", "[LATERAL_COEFICIENTS]\nPCY1 = 1.4\n")
    assert any("unknown `.tir` section" in w for w in warnings)


# --- CLI -------------------------------------------------------------------------------------


def test_cli_round_trip(tmp_path: Path) -> None:
    src = _TIR_FIXTURES / "synthetic_slick.tir"
    tyr_out = tmp_path / "car.tyr.yaml"
    tir_out = tmp_path / "car.tir"
    assert tir_main(["to-tyr", str(src), "-o", str(tyr_out)]) == 0
    assert tir_main(["from-tyr", str(tyr_out), "-o", str(tir_out)]) == 0

    # The regenerated .tir carries the same coefficient map.
    doc_a, _ = parse_tir("a", src.read_text(encoding="utf-8"))
    doc_b, _ = parse_tir("b", tir_out.read_text(encoding="utf-8"))
    tyr_a, _ = tir_to_tyr(doc_a)
    tyr_b, _ = tir_to_tyr(doc_b)
    assert tyr_a["mf61"] == tyr_b["mf61"]


def test_cli_errors_cleanly_on_bad_input(tmp_path: Path) -> None:
    bad = tmp_path / "bad.tir"
    bad.write_text("[UNITS]\nLENGTH = 'inch'\n", encoding="utf-8")
    assert tir_main(["to-tyr", str(bad), "-o", str(tmp_path / "x.yaml")]) == 1
