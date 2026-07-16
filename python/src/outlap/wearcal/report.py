# SPDX-License-Identifier: AGPL-3.0-only
"""Calibration reporting: JSON + Markdown (no embedded telemetry, so the report is redistributable).

The JSON is the machine artifact (recovered parameters, residuals, decay diagnostics); the
Markdown renders the same for humans. Neither embeds the source stint's raw lap times beyond the
fitted-vs-observed comparison table, keeping the report shareable while the telemetry is not.
"""

from __future__ import annotations

import json
from pathlib import Path

from .optimize import CalibResult


def report_dict(result: CalibResult, *, source: str) -> dict[str, object]:
    """The report as a plain dict (the JSON document)."""
    return {
        "tool": "outlap.wearcal",
        "source": source,
        "free_parameters": list(result.free),
        "fitted": result.fitted,
        "residual_rms_s": result.rms_s,
        "residual_max_abs_s": result.max_abs_s,
        "n_laps": result.n_laps,
        "decay_s_per_lap": result.decay_s_per_lap,
        "cliff_lap": result.cliff_lap,
        "total_loss_s": result.total_loss_s,
        "converged": result.success,
        "parameters": result.params.as_dict(),
        "lap_fit": [
            {"lap": i + 1, "observed_s": float(o), "simulated_s": float(s)}
            for i, (o, s) in enumerate(
                zip(result.obs_lap_time_s, result.sim_lap_time_s, strict=True)
            )
        ],
    }


def render_markdown(result: CalibResult, *, source: str) -> str:
    """Render the calibration report as Markdown."""
    cliff = "none" if result.cliff_lap is None else str(result.cliff_lap)
    lines = [
        "# Tyre wear/grip calibration report",
        "",
        f"- Source: `{source}`",
        f"- Free parameters: {', '.join(result.free)}",
        f"- Converged: {result.success}",
        f"- Residual RMS: {result.rms_s:.4g} s (max {result.max_abs_s:.4g} s)",
        f"- Mean decay: {result.decay_s_per_lap:.4g} s/lap over {result.n_laps} laps",
        f"- Total pace loss: {result.total_loss_s:.4g} s; cliff onset lap: {cliff}",
        "",
        "## Fitted parameters",
        "",
        "```",
        *[f"{k} = {v!r}" for k, v in result.fitted.items()],
        "```",
        "",
        "## Lap fit",
        "",
        "| Lap | Observed (s) | Simulated (s) | Δ (s) |",
        "|---|---|---|---|",
    ]
    for i, (o, s) in enumerate(
        zip(result.obs_lap_time_s, result.sim_lap_time_s, strict=True), start=1
    ):
        lines.append(f"| {i} | {o:.3f} | {s:.3f} | {s - o:+.3f} |")
    lines.append("")
    return "\n".join(lines)


def write_report(
    result: CalibResult, out_dir: Path, *, source: str
) -> tuple[Path, Path]:
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
