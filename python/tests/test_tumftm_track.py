# SPDX-License-Identifier: AGPL-3.0-only
"""Unit tests for the TUMFTM racetrack-database importer (`outlap.importers.tumftm_track`).

The conversion runs offline (no network) on a synthetic TUMFTM-format loop, so these run in CI.
The load-bearing checks are the **width-by-name mapping** (source is RIGHT before LEFT) and the
**round-trip through the authoritative Rust loader** (`Track.load`) — not merely the CSV shape.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from outlap.core import Track
from outlap.importers import tumftm_track


def _synthetic_loop(
    radius: float = 100.0,
    spacing: float = 5.0,
    w_right: float = 3.0,
    w_left: float = 7.0,
) -> str:
    """A closed CCW circle in TUMFTM format (`# x_m,y_m,w_tr_right_m,w_tr_left_m`), left open.

    Asymmetric constant widths (RIGHT ≠ LEFT) make a column-position swap detectable. The loop is
    left open (the last point sits ~one sample before the first) exactly like the upstream data.
    """
    n = round(2.0 * math.pi * radius / spacing)
    rows = ["# x_m,y_m,w_tr_right_m,w_tr_left_m"]
    for i in range(
        n
    ):  # endpoint excluded: i in [0, n) leaves the seam ~one sample open
        theta = 2.0 * math.pi * i / n
        rows.append(
            f"{radius * math.cos(theta):.6f},{radius * math.sin(theta):.6f},"
            f"{w_right:.3f},{w_left:.3f}"
        )
    return "\n".join(rows) + "\n"


def test_widths_mapped_by_name_not_position() -> None:
    # Source RIGHT=3, LEFT=7 → outlap width_left_m must be 7 (the LEFT column), width_right_m 3.
    track = tumftm_track.convert(
        _synthetic_loop(w_right=3.0, w_left=7.0), "syn", "Synthetic"
    )
    assert track.width_left[0] == pytest.approx(7.0)
    assert track.width_right[0] == pytest.approx(3.0)
    assert all(wl == pytest.approx(7.0) for wl in track.width_left)
    assert all(wr == pytest.approx(3.0) for wr in track.width_right)


def test_arc_length_is_monotone_and_flat() -> None:
    track = tumftm_track.convert(_synthetic_loop(), "syn", "Synthetic")
    assert track.s[0] == 0.0
    assert all(b > a for a, b in zip(track.s, track.s[1:], strict=False))
    # The seam sits ~one sample (≈5 m) before the wrap, so the loader closes over the chord.
    seam = math.hypot(track.x[0] - track.x[-1], track.y[0] - track.y[-1])
    assert 2.0 < seam < 8.0


def test_roundtrip_through_rust_loader(tmp_path: Path) -> None:
    track = tumftm_track.convert(_synthetic_loop(radius=100.0), "syn", "Synthetic Loop")
    yaml_path, csv_path = tumftm_track.write_track(track, tmp_path)

    # Emitted CSV: canonical 8-column header, flat 2-D, unit grip.
    header = csv_path.read_text(encoding="utf-8").splitlines()[1]
    assert header == "s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale"
    first = csv_path.read_text(encoding="utf-8").splitlines()[2].split(",")
    assert (
        first[3] == "0.0000" and first[4] == "0.000" and first[7] == "1.0000"
    )  # z, banking, grip
    # Width columns in canonical order: the asymmetric 7/3 input catches a write_track column swap.
    assert first[5] == "7.000" and first[6] == "3.000"  # width_left_m, width_right_m

    # The round-trip: it must load through the real geometry as a closed loop of the right length.
    loaded = Track.load(str(yaml_path.parent))
    assert loaded.is_closed()
    assert loaded.length() == pytest.approx(2.0 * math.pi * 100.0, rel=0.02)
    # Handedness survives the round trip: the LEFT column stays left (7 m), RIGHT stays right (3 m).
    sampled = loaded.sample(5.0)
    assert float(sampled["width_left"].mean()) == pytest.approx(7.0, abs=0.05)
    assert float(sampled["width_right"].mean()) == pytest.approx(3.0, abs=0.05)


def test_resample_changes_sample_count() -> None:
    native = tumftm_track.convert(_synthetic_loop(spacing=5.0), "syn", "Synthetic")
    dense = tumftm_track.convert(
        _synthetic_loop(spacing=5.0), "syn", "Synthetic", ds=2.5
    )
    assert len(dense) > len(native)
    # Resampled arc length is still monotone.
    assert all(b > a for a, b in zip(dense.s, dense.s[1:], strict=False))


def test_bad_column_count_is_rejected() -> None:
    with pytest.raises(ValueError, match="expected 4 columns"):
        tumftm_track.parse_tumftm_csv(
            "# h\n1.0,2.0,3.0\n1.0,2.0,3.0\n1.0,2.0,3.0\n1.0,2.0,3.0\n"
        )


def test_name_resolution_falls_back_to_slug() -> None:
    assert tumftm_track.resolve("Catalunya") == (
        "catalunya",
        "Circuit de Barcelona-Catalunya",
    )
    assert tumftm_track.resolve("Nuerburgring")[1] == "Nürburgring GP"
    # Unknown stem → snake_case slug, name echoed.
    assert tumftm_track.resolve("SomeNewTrack") == ("some_new_track", "SomeNewTrack")


def test_batch_skips_bad_file_but_converts_the_rest(tmp_path: Path) -> None:
    # A malformed file must not abort the whole directory import: the rest still convert, and the
    # CLI signals a nonzero exit so the operator notices the skip.
    src = tmp_path / "src"
    src.mkdir()
    (src / "Good.csv").write_text(_synthetic_loop(), encoding="utf-8")
    (src / "Bad.csv").write_text(
        "# x,y,wr,wl\n1,2,3\n", encoding="utf-8"
    )  # 3 cols → ValueError
    out = tmp_path / "out"

    rc = tumftm_track.main(["--input", str(src), "--out", str(out)])

    assert rc == 1  # a file was skipped
    assert (out / "good" / "track.yaml").exists()  # the good track was still written
    assert not (out / "bad").exists()  # the malformed one produced nothing
