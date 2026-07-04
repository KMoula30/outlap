#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
#
# Regenerate the MF6.1 golden CSVs (see README.md). NEVER runs in CI — the committed CSVs are
# compared there. Regenerating is governed by README.md: only in a PR that updates the version
# pins and states the physics/tooling reason.
#
# Requirements (no root needed):
#   - GNU Octave on PATH (e.g. `micromamba create -n oct -c conda-forge octave`, then activate, or
#     any system Octave >= 8).
#   - A checkout of the oracle library, teasit/magic-formula-tyre-library (GPL-3.0), whose numeric
#     outputs we use as DATA ONLY (never its source). Point MF_ORACLE_SRC at its `src/` directory.
#
# Usage:
#   MF_ORACLE_SRC=/path/to/magic-formula-tyre-library/src ./tools/goldens/run.sh
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"

: "${MF_ORACLE_SRC:?set MF_ORACLE_SRC to the oracle library's src/ directory}"
if ! command -v octave-cli >/dev/null 2>&1 && ! command -v octave >/dev/null 2>&1; then
  echo "error: octave not found on PATH" >&2
  exit 1
fi
octave_bin="$(command -v octave-cli || command -v octave)"

# The oracle ships a top-level `magicformula.m` FUNCTION that shadows its `+magicformula` PACKAGE
# under Octave. Stage a package-only copy (parent dir on the path, no shadowing function).
pkg="$(mktemp -d)"
trap 'rm -rf "$pkg"' EXIT
cp -r "$MF_ORACLE_SRC/+magicformula" "$pkg/"
[ -d "$MF_ORACLE_SRC/enum" ] && cp -r "$MF_ORACLE_SRC/enum" "$pkg/"

export MF_ORACLE_PKG="$pkg"
export MF_GOLDEN_OUT="$repo/crates/outlap-tire/tests/golden/pacejka_2006_205_60r15"
export MF_ORACLE_TAG="teasit/magic-formula-tyre-library $(git -C "$MF_ORACLE_SRC/.." rev-parse --short HEAD 2>/dev/null || echo '<commit>') (GPL-3.0)"
mkdir -p "$MF_GOLDEN_OUT"

"$octave_bin" --no-gui -q -p "$here" --eval "gen_mf61_goldens"
echo "goldens written to $MF_GOLDEN_OUT"
