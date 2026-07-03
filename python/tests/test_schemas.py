# SPDX-License-Identifier: AGPL-3.0-only
"""Tests for the schema-conformance check."""

from __future__ import annotations

import outlap.schemas as schemas


def test_all_schemas_load_and_are_valid_draft_2020_12() -> None:
    from jsonschema import Draft202012Validator

    for name in ("vehicle", "ptm", "tyr", "emotor"):
        schema = schemas.load_schema(name)
        Draft202012Validator.check_schema(schema)


def test_fixtures_validate_against_committed_schemas() -> None:
    assert schemas.check() == 0
