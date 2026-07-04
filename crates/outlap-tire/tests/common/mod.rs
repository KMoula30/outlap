// SPDX-License-Identifier: AGPL-3.0-only
//! Shared helpers for the reference-data integration tests.
// Compiled into each test binary separately; not every binary uses every helper.
#![allow(dead_code)]

use outlap_schema::io::FsLoader;
use outlap_schema::load::load_tyr;
use outlap_tire::Mf61;

/// Path of the committed Pacejka book reference tyre, relative to the `data/tires` loader root.
pub const PACEJKA_TYR: &str = "pacejka_2006_205_60r15/car.tyr.yaml";

/// A loader rooted at the repo-level `data/tires` directory.
pub fn data_loader() -> FsLoader {
    FsLoader::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/tires"))
}

/// Load the Pacejka book reference tyre as an evaluatable model (panics on any load/build error).
pub fn pacejka_model() -> Mf61<f64> {
    let (tyr, _) = load_tyr(PACEJKA_TYR, &data_loader()).unwrap();
    let (m, _) = Mf61::<f64>::from_tyr(&tyr).unwrap();
    m
}

/// Path of the committed TUMFTM Roborace MF5.2 reference tyre, relative to the loader root.
pub const ROBORACE_TYR: &str = "roborace_devbot_mf52/car.tyr.yaml";

/// Load the Roborace reference tyre as an evaluatable model (panics on any load/build error).
pub fn roborace_model() -> Mf61<f64> {
    let (tyr, _) = load_tyr(ROBORACE_TYR, &data_loader()).unwrap();
    let (m, _) = Mf61::<f64>::from_tyr(&tyr).unwrap();
    m
}

/// Every committed `.tyr` dataset anywhere under `data/tires/` (recursive walk), as
/// loader-relative paths (sorted). Globbed so a future dataset joins the reference gates without
/// test edits, at any nesting depth.
pub fn all_reference_tyres() -> Vec<String> {
    let root = std::path::PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/tires"));
    let mut found = Vec::new();
    let mut stack = vec![root.clone()];
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(&dir).expect("readable directory under data/tires") {
            let path = entry.expect("readable entry").path();
            if path.is_dir() {
                stack.push(path);
            } else if path
                .file_name()
                .is_some_and(|n| n.to_string_lossy().ends_with(".tyr.yaml"))
            {
                let rel = path
                    .strip_prefix(&root)
                    .expect("path is under the walk root")
                    .to_string_lossy()
                    .into_owned();
                found.push(rel);
            }
        }
    }
    found.sort();
    assert!(
        found.len() >= 2,
        "expected at least the Pacejka + Roborace datasets, found {found:?}"
    );
    found
}
