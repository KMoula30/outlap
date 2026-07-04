// SPDX-License-Identifier: AGPL-3.0-only
//! Shared helpers for the reference-data integration tests.

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
