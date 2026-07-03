// SPDX-License-Identifier: AGPL-3.0-only
//! `gen_schemas` — emit the golden JSON Schemas (draft 2020-12) for the outlap file formats.
//!
//! The Rust schemars types are the single source of truth (Decision #34). This binary writes
//! `schemas/{vehicle,ptm,tyr,emotor,track,conditions,sim}.json`; with `--check` it regenerates
//! in-memory and diffs against the committed files, failing if they drift (wired into Rust CI).

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use outlap_schema::conditions::Conditions;
use outlap_schema::emotor::Emotor;
use outlap_schema::ptm::Ptm;
use outlap_schema::sim::Sim;
use outlap_schema::track::TrackDoc;
use outlap_schema::tyr::Tyr;
use outlap_schema::vehicle::Vehicle;

/// (filename, pretty-printed schema JSON) for every emitted document.
fn schemas() -> Vec<(&'static str, String)> {
    vec![
        ("vehicle.json", pretty::<Vehicle>()),
        ("ptm.json", pretty::<Ptm>()),
        ("tyr.json", pretty::<Tyr>()),
        ("emotor.json", pretty::<Emotor>()),
        ("track.json", pretty::<TrackDoc>()),
        ("conditions.json", pretty::<Conditions>()),
        ("sim.json", pretty::<Sim>()),
    ]
}

fn pretty<T: schemars::JsonSchema>() -> String {
    let schema = schemars::schema_for!(T);
    let mut s = serde_json::to_string_pretty(&schema).expect("schema serializes");
    s.push('\n');
    s
}

/// Locate the repo `schemas/` directory relative to this crate.
fn schemas_dir() -> PathBuf {
    // CARGO_MANIFEST_DIR = crates/outlap-schema; schemas/ is two levels up.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../schemas")
        .canonicalize()
        .unwrap_or_else(|_| PathBuf::from("schemas"))
}

fn main() -> ExitCode {
    let check = std::env::args().any(|a| a == "--check");
    let dir = schemas_dir();
    let mut drift = false;

    for (name, content) in schemas() {
        let path = dir.join(name);
        if check {
            match std::fs::read_to_string(&path) {
                Ok(existing) if existing == content => {}
                Ok(_) => {
                    eprintln!(
                        "drift: {} is out of date (run `cargo run -p outlap-schema --bin gen_schemas`)",
                        path.display()
                    );
                    drift = true;
                }
                Err(e) => {
                    eprintln!("missing: {} ({e})", path.display());
                    drift = true;
                }
            }
        } else if let Err(e) = std::fs::write(&path, &content) {
            eprintln!("failed to write {}: {e}", path.display());
            return ExitCode::FAILURE;
        } else {
            println!("wrote {}", path.display());
        }
    }

    if check && drift {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
