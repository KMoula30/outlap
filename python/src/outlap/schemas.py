# SPDX-License-Identifier: AGPL-3.0-only
"""Load and validate the outlap JSON Schemas.

The Rust ``schemars`` types are the single source of truth (Decision #34): they emit the golden
``schemas/*.json`` files, and Python only *conforms* to them. This module loads those committed
schemas and validates the shipped YAML fixtures against them with ``jsonschema``.

``python -m outlap.schemas --check`` is wired into CI. A later increment adds the pydantic v2 mirror
and asserts its emitted schema is structurally equivalent to the Rust-owned schema.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator

# Repo layout: this file is <root>/python/src/outlap/schemas.py.
_ROOT = Path(__file__).resolve().parents[3]
_SCHEMAS_DIR = _ROOT / "schemas"
_FIXTURES_DIR = _ROOT / "crates" / "outlap-schema" / "tests" / "fixtures"
_DATA_DIR = _ROOT / "data"

# Committed reference data validated by glob (so new datasets are covered automatically), keyed by
# the schema each `*.<suffix>.yaml` anywhere under `data/` must satisfy.
_DATA_GLOBS: dict[str, str] = {
    "tyr": "*.tyr.yaml",
}

# Which committed fixtures must validate against each schema. Only self-contained documents are
# listed here — fixtures that rely on `extends:` merge are validated by the Rust pipeline instead.
_FIXTURES: dict[str, list[str]] = {
    "vehicle": [
        "ev_1du_rwd/vehicle.yaml",
        "ev_2du_awd/vehicle.yaml",
        "ev_4du_tv/vehicle.yaml",
        "fwd_hatch/vehicle.yaml",
        "gt_hybrid/vehicle.yaml",
        "f1_2026/vehicle.yaml",
    ],
    "ptm": [
        "ptm/rear_drive_unit.ptm.yaml",
        "ptm/front_drive_unit.ptm.yaml",
        "ptm/ice_v6.ptm.yaml",
        "ptm/mgu_k.ptm.yaml",
        "ptm/pdt_synth_edrive.ptm.yaml",
        "ptm/pdt_synth_du.ptm.yaml",
        "ptm/pdt_synth_du_vdc.ptm.yaml",
    ],
    "tyr": [
        "tyr/slick.tyr.yaml",
        "tyr/brush_only.tyr.yaml",
        "tyr/brush_plus_mf61.tyr.yaml",
    ],
    "emotor": ["emotor/rear.emotor.yaml", "emotor/pdt_synth.emotor.yaml"],
    "battery": ["battery/synth_pack.battery.yaml"],
    "track": ["track/synthetic_oval.track.yaml"],
    "conditions": ["conditions/hot_dry.conditions.yaml"],
    "sim": ["sim/qss.sim.yaml"],
}


def load_schema(name: str) -> dict[str, Any]:
    """Load a committed JSON Schema by document name (e.g. ``"vehicle"``)."""
    path = _SCHEMAS_DIR / f"{name}.json"
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def _load_yaml(path: Path) -> Any:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def _validate_docs(name: str, docs: list[tuple[str, Path]], errors: list[str]) -> int:
    """Validate a list of ``(label, path)`` documents against schema ``name``.

    Returns the number of documents validated (0 on a schema-level failure).
    """
    schema_path = _SCHEMAS_DIR / f"{name}.json"
    if not schema_path.exists():
        errors.append(f"missing schema: {schema_path}")
        return 0
    schema = load_schema(name)
    # The schema must itself be a valid draft 2020-12 schema.
    try:
        Draft202012Validator.check_schema(schema)
    except Exception as exc:  # noqa: BLE001 - surface any meta-schema failure
        errors.append(f"{name}.json is not a valid draft 2020-12 schema: {exc}")
        return 0
    validator = Draft202012Validator(schema)
    for label, path in docs:
        if not path.exists():
            errors.append(f"missing document: {path}")
            continue
        doc = _load_yaml(path)
        # jsonschema's iter_errors overloads are partially untyped; the loop body only
        # touches attributes we know exist on ValidationError.
        for err in sorted(validator.iter_errors(doc), key=str):  # pyright: ignore[reportUnknownMemberType]
            loc = "/".join(str(p) for p in err.absolute_path) or "<root>"
            errors.append(f"{label} [{name}] at {loc}: {err.message}")
    return len(docs)


def check() -> int:
    """Validate every committed schema, its fixtures, and the reference data. Exit code."""
    errors: list[str] = []
    total = 0

    for name, fixtures in _FIXTURES.items():
        docs = [(f, _FIXTURES_DIR / f) for f in fixtures]
        total += _validate_docs(name, docs, errors)

    # Reference data anywhere under data/ (globbed so new datasets are covered without edits).
    data_total = 0
    for name, pattern in _DATA_GLOBS.items():
        found = sorted(_DATA_DIR.rglob(pattern)) if _DATA_DIR.exists() else []
        docs = [(str(p.relative_to(_ROOT)), p) for p in found]
        data_total += _validate_docs(name, docs, errors)

    if errors:
        print("schema check FAILED:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    print(
        f"schema check OK: {len(_FIXTURES)} schemas, "
        f"{total} fixtures + {data_total} data files validated"
    )
    return 0


def main() -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(prog="outlap.schemas", description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate committed schemas and fixtures (used in CI)",
    )
    args = parser.parse_args()
    # `--check` is currently the only mode; default to it so a bare invocation is useful.
    _ = args.check
    return check()


if __name__ == "__main__":
    raise SystemExit(main())
