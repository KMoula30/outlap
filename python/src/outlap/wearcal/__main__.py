# SPDX-License-Identifier: AGPL-3.0-only
"""CLI for the tyre wear/grip inverse-calibration harness.

python -m outlap.wearcal calibrate <fixture.csv> --base <in.tyr.yaml> -o <out.tyr.yaml>
                                   [--free k_w,w_c,s_w,delta_c] [--report-dir DIR]
python -m outlap.wearcal synth     <in.tyr.yaml> -o <fixture.csv> [--n-laps N] [--noise S] [--seed K]
python -m outlap.wearcal sim-check <in.tyr.yaml> --vehicle DIR --track DIR [--n-laps N] [--tier t0]

`calibrate` fits the wear/grip parameters so a simulated stint matches an observed per-lap pace
curve (a committed CSV or a private FastF1-derived one) and writes a calibrated `.tyr`.
`synth` generates a deterministic synthetic stint-delta CSV from a `.tyr` — the recovery-test
harness. `sim-check` runs the real Rust stint driver with a `.tyr`'s parameters and prints the
per-lap times + wear (the bridge from the surrogate fit to the real driver).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, cast

import yaml

from .data import load_fixture
from .model import StintAnchor, SurrogateParams
from .optimize import CalibConfig, calibrate, synth_observation
from .report import write_report


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(prog="outlap.wearcal", description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    pc = sub.add_parser(
        "calibrate", help="fit wear/grip params to a stint pace curve → .tyr"
    )
    pc.add_argument("fixture", type=Path, help="a lap,lap_time_s CSV stint observation")
    pc.add_argument(
        "--base", type=Path, required=True, help="base .tyr (fixed params + force set)"
    )
    pc.add_argument("-o", "--output", type=Path, required=True)
    pc.add_argument(
        "--free", type=str, default=None, help="comma-separated free params"
    )
    pc.add_argument("--report-dir", type=Path)

    ps = sub.add_parser("synth", help="synthetic stint-delta CSV from a .tyr (seeded)")
    ps.add_argument("tyr", type=Path)
    ps.add_argument("-o", "--output", type=Path, required=True)
    ps.add_argument("--n-laps", type=int, default=25)
    ps.add_argument(
        "--noise", type=float, default=0.0, help="Gaussian lap-time noise, s"
    )
    ps.add_argument("--seed", type=int, default=0)

    pk = sub.add_parser(
        "sim-check", help="run the real stint driver with a .tyr's params"
    )
    pk.add_argument("tyr", type=Path)
    pk.add_argument("--vehicle", type=Path, required=True)
    pk.add_argument("--track", type=Path, required=True)
    pk.add_argument("--n-laps", type=int, default=20)
    pk.add_argument("--tier", type=str, default="t0")

    args = parser.parse_args(argv)
    try:
        if args.cmd == "calibrate":
            return _calibrate(args)
        if args.cmd == "synth":
            return _synth(args)
        return _sim_check(args)
    except (ValueError, OSError, ImportError, KeyError, yaml.YAMLError) as err:
        print(f"error: {err}", file=sys.stderr)
        return 1


def _load_tyr(path: Path) -> dict[str, Any]:
    return cast("dict[str, Any]", yaml.safe_load(path.read_text(encoding="utf-8")))


def _params_from_tyr(doc: dict[str, Any]) -> SurrogateParams:
    return SurrogateParams.from_tyr(doc["thermal"], doc["wear"])


def _calibrate(args: argparse.Namespace) -> int:
    base_doc = _load_tyr(args.base)
    base = _params_from_tyr(base_doc)
    obs = load_fixture(args.fixture)
    free = tuple(s.strip() for s in args.free.split(",")) if args.free else None
    cfg = CalibConfig(free=free, base=base) if free else CalibConfig(base=base)
    result = calibrate(obs, cfg)

    thermal_updates, wear_updates = result.params.tyr_updates()
    out_doc = dict(base_doc)
    out_doc["thermal"] = {**base_doc.get("thermal", {}), **thermal_updates}
    out_doc["wear"] = {**base_doc.get("wear", {}), **wear_updates}
    out_doc["provenance"] = {
        **base_doc.get("provenance", {}),
        "source": (
            f"thermal/wear inverse-calibrated by outlap.wearcal against `{obs.label}` "
            f"(free: {', '.join(result.free)}; RMS {result.rms_s:.3g} s; "
            f"decay {result.decay_s_per_lap:.3g} s/lap)"
        ),
    }
    args.output.write_text(
        yaml.safe_dump(out_doc, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    print(f"wrote {args.output}")
    print(
        f"fit: converged={result.success} RMS={result.rms_s:.4g}s "
        f"decay={result.decay_s_per_lap:.4g}s/lap cliff_lap={result.cliff_lap}"
    )
    for name, value in result.fitted.items():
        print(f"  {name} = {value!r}")
    if args.report_dir is not None:
        json_path, md_path = write_report(
            result, args.report_dir, source=str(args.fixture)
        )
        print(f"wrote {json_path} and {md_path}")
    return 0


def _synth(args: argparse.Namespace) -> int:
    doc = _load_tyr(args.tyr)
    params = _params_from_tyr(doc)
    anchor = StintAnchor(t_op_c=params.t_opt, t_c_c=params.t_opt)
    obs = synth_observation(
        params,
        anchor,
        args.n_laps,
        noise_s=args.noise,
        seed=args.seed,
        label=args.tyr.stem,
    )
    header = "# derived synthetic stint-delta (outlap.wearcal synth) — not raw telemetry\nlap,lap_time_s"
    body = "\n".join(
        f"{int(lap)},{float(time)!r}"
        for lap, time in zip(obs.lap, obs.lap_time_s, strict=True)
    )
    args.output.write_text(header + "\n" + body + "\n", encoding="utf-8")
    print(f"wrote {args.output} ({obs.n_laps} laps)")
    return 0


def _sim_check(args: argparse.Namespace) -> int:
    from outlap.core import Track

    from .sim import sim_stint

    params = _params_from_tyr(_load_tyr(args.tyr))
    track = Track.load(str(args.track))
    result = sim_stint(args.vehicle, track, params, args.n_laps, tier=args.tier)
    print(f"tier={args.tier} n_laps={args.n_laps}")
    for lap, t, w, g in zip(
        result.lap, result.lap_time_s, result.wear_mm, result.grip, strict=True
    ):
        print(f"  lap {int(lap):2d}: {t:7.3f}s  wear={w:5.3f}mm  grip={g:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
