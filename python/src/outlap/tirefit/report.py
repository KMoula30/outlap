# SPDX-License-Identifier: AGPL-3.0-only
"""Fit reporting: JSON + Markdown only (no plots in M2 — those come with the viz milestone).

The JSON is the machine artifact (stage-by-stage fitted values and residual stats); the Markdown
is the same content rendered for humans. Neither embeds the input data, so a report on TTC data
stays redistributable while the data does not.
"""

from __future__ import annotations

import json
from pathlib import Path

from .stages import FitResult


def report_dict(result: FitResult, *, source: str) -> dict[str, object]:
    """The report as a plain dict (the JSON document)."""
    return {
        "tool": "outlap.tirefit",
        "source": source,
        "stages": [
            {
                "name": s.name,
                "skipped": s.skipped,
                "n_samples": s.n_samples,
                "rms_normalised": s.rms_n,
                "max_abs_normalised": s.max_abs_n,
                "fitted": s.fitted,
            }
            for s in result.stages
        ],
        "coefficients": result.coeffs,
    }


def render_markdown(result: FitResult, *, source: str) -> str:
    """Render the fit report as Markdown."""
    lines = [
        "# MF6.1 fit report",
        "",
        f"- Source: `{source}`",
        f"- Coefficients fitted: {len(result.coeffs)}",
        "",
        "## Stages",
        "",
        "| Stage | Samples | RMS (normalised) | Max (normalised) | Freed |",
        "|---|---|---|---|---|",
    ]
    for s in result.stages:
        if s.skipped is not None:
            lines.append(f"| {s.name} | — | — | — | skipped: {s.skipped} |")
        else:
            lines.append(
                f"| {s.name} | {s.n_samples} | {s.rms_n:.4g} | {s.max_abs_n:.4g} "
                f"| {len(s.fitted)} |"
            )
    lines += [
        "",
        "## Coefficients",
        "",
        "```",
    ]
    lines += [f"{k} = {v!r}" for k, v in result.coeffs.items()]
    lines += ["```", ""]
    return "\n".join(lines)


def write_report(result: FitResult, out_dir: Path, *, source: str) -> tuple[Path, Path]:
    """Write ``report.json`` + ``report.md`` into ``out_dir``; returns the two paths."""
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "report.json"
    md_path = out_dir / "report.md"
    json_path.write_text(
        json.dumps(report_dict(result, source=source), indent=2) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(render_markdown(result, source=source), encoding="utf-8")
    return json_path, md_path
