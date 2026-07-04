// SPDX-License-Identifier: AGPL-3.0-only
//! Golden cross-check: our MF6.1 kernels vs the committed oracle CSVs (HANDOFF §12/§13).
//!
//! The CSVs are generated once, offline, by an independent Magic-Formula implementation used as a
//! numerical oracle (outputs as data only — see `tools/goldens/`). Our model reads the reference
//! `.tyr`; the oracle read an equivalent parameter struct. Agreement to `≤ 0.5%` (relative, with a
//! per-channel absolute floor so zero crossings don't divide by ~0) validates the kernels.
//!
//! Regenerating these CSVs is governed by `tools/goldens/README.md` — never silently.
#![allow(
    clippy::doc_markdown,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::unnecessary_debug_formatting
)]

use std::collections::BTreeMap;
use std::fs;
use std::path::PathBuf;

use outlap_tire::SlipState;

mod common;
use common::pacejka_model;

const REL: f64 = 0.005; // 0.5% relative gate.

// Reference-tyre constants for the per-load absolute floors (kept at 0.5% of a characteristic
// per-load magnitude, so every channel's floor tracks the same ≤0.5% intent).
const PDX1: f64 = 1.210;
const PDY1_ABS: f64 = 0.990;
const R0: f64 = 0.313;
const QSY1: f64 = 0.01;

struct Row {
    kappa: f64,
    alpha: f64,
    gamma: f64,
    fz: f64,
    p: f64,
    vx: f64,
    fx: f64,
    fy: f64,
    mz: f64,
    mx: f64,
    my: f64,
}

fn golden_dir() -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden"))
}

fn parse(path: &PathBuf) -> Vec<Row> {
    let text = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path:?}: {e}"));
    let mut rows = Vec::new();
    for line in text.lines() {
        if line.starts_with('#') || line.starts_with("kappa") || line.trim().is_empty() {
            continue;
        }
        let v: Vec<f64> = line.split(',').map(|s| s.trim().parse().unwrap()).collect();
        assert_eq!(v.len(), 11, "bad golden row: {line}");
        rows.push(Row {
            kappa: v[0],
            alpha: v[1],
            gamma: v[2],
            fz: v[3],
            p: v[4],
            vx: v[5],
            fx: v[6],
            fy: v[7],
            mz: v[8],
            mx: v[9],
            my: v[10],
        });
    }
    assert!(!rows.is_empty(), "no rows in {path:?}");
    rows
}

/// Worst absolute violation of `|model − ref| ≤ max(REL·|ref|, floor)` over a channel.
struct Worst {
    excess: f64, // (|err| − tol); ≤ 0 means pass.
    detail: String,
}

fn check_channel(
    name: &str,
    rows: &[Row],
    model_val: impl Fn(&Row) -> f64,
    ref_val: impl Fn(&Row) -> f64,
    floor: impl Fn(&Row) -> f64,
) -> Worst {
    let mut worst = Worst {
        excess: f64::NEG_INFINITY,
        detail: String::new(),
    };
    for r in rows {
        let m = model_val(r);
        let g = ref_val(r);
        let tol = (REL * g.abs()).max(floor(r));
        let err = (m - g).abs();
        let excess = err - tol;
        if excess > worst.excess {
            worst.excess = excess;
            worst.detail = format!(
                "{name}: κ={:.3} α={:.3} γ={:.3} Fz={:.0} → model {m:.4}, ref {g:.4}, |err| {err:.4}, tol {tol:.4}",
                r.kappa, r.alpha, r.gamma, r.fz
            );
        }
    }
    worst
}

/// Per-Fz-bin max |Mz_ref|, for the Mz absolute floor.
fn mz_floor_by_fz(rows: &[Row]) -> BTreeMap<i64, f64> {
    let mut m: BTreeMap<i64, f64> = BTreeMap::new();
    for r in rows {
        let e = m.entry(r.fz.round() as i64).or_insert(0.0);
        *e = e.max(r.mz.abs());
    }
    m
}

fn run_file(file: &str, channels: &[&str]) {
    let m = pacejka_model();
    let rows = parse(&golden_dir().join("pacejka_2006_205_60r15").join(file));
    let mz_floor = mz_floor_by_fz(&rows);

    let eval = |r: &Row| m.forces(&SlipState::new(r.kappa, r.alpha, r.gamma, r.fz, r.p, r.vx));

    let mut failures = Vec::new();
    for &ch in channels {
        let w = match ch {
            "fx" => check_channel("fx", &rows, |r| eval(r).fx, |r| r.fx, |r| REL * PDX1 * r.fz),
            "fy" => check_channel(
                "fy",
                &rows,
                |r| eval(r).fy,
                |r| r.fy,
                |r| REL * PDY1_ABS * r.fz,
            ),
            "mz" => check_channel(
                "mz",
                &rows,
                |r| eval(r).mz,
                |r| r.mz,
                |r| REL * mz_floor[&(r.fz.round() as i64)],
            ),
            // Mx ≡ 0 for this tyre (QSX* omitted): this is a null-guard against a spurious
            // nonzero Mx, not a cross-check of Mx physics.
            "mx" => check_channel("mx", &rows, |r| eval(r).mx, |r| r.mx, |_| 1e-6),
            // My ≈ R0·QSY1·Fz; keep the floor at 0.5% of that per-load magnitude so the gate
            // stays ≤0.5% at low load too (a fixed floor would loosen to ~0.8% at Fz = 2 kN).
            "my" => check_channel(
                "my",
                &rows,
                |r| eval(r).my,
                |r| r.my,
                |r| REL * R0 * QSY1 * r.fz,
            ),
            other => panic!("unknown channel {other}"),
        };
        if w.excess > 0.0 {
            failures.push(w.detail);
        }
    }
    assert!(
        failures.is_empty(),
        "golden mismatch in {file}:\n  {}",
        failures.join("\n  ")
    );
}

/// Every golden CSV must carry a provenance header that pins the generator, the Octave version,
/// and a specific oracle **commit** (`oracle: … @ <hash>`), so a regeneration is never anonymous
/// (`tools/goldens/README.md`). A generic tag with no commit fails this.
#[test]
fn goldens_carry_provenance_headers() {
    let dir = golden_dir().join("pacejka_2006_205_60r15");
    for file in [
        "fx0.csv",
        "fy0_mz.csv",
        "combined.csv",
        "combined_camber.csv",
    ] {
        let text = fs::read_to_string(dir.join(file)).unwrap();
        let head: String = text.lines().take(2).collect::<Vec<_>>().join("\n");
        assert!(
            head.contains("# generator:") && head.contains("Octave"),
            "{file} is missing its generator/Octave header:\n{head}"
        );
        // The oracle must be pinned to a commit: `oracle: <name> @ <hex>`.
        let pinned = head
            .split("oracle:")
            .nth(1)
            .and_then(|s| s.split('@').nth(1))
            .is_some_and(|s| {
                let h: String = s
                    .trim()
                    .chars()
                    .take_while(char::is_ascii_hexdigit)
                    .collect();
                h.len() >= 7
            });
        assert!(
            pinned,
            "{file} oracle is not pinned to a commit hash:\n{head}"
        );
    }
}

#[test]
fn golden_fx0() {
    run_file("fx0.csv", &["fx", "fy", "mz", "mx", "my"]);
}

#[test]
fn golden_fy0_mz() {
    run_file("fy0_mz.csv", &["fx", "fy", "mz", "mx", "my"]);
}

#[test]
fn golden_combined() {
    run_file("combined.csv", &["fx", "fy", "mz", "mx", "my"]);
}

/// The coupled regime the Mz fix governs: combined slip AND camber together (κ≠0 ∧ γ=±4°).
#[test]
fn golden_combined_camber() {
    run_file("combined_camber.csv", &["fx", "fy", "mz", "mx", "my"]);
}
