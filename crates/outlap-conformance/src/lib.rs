// SPDX-License-Identifier: AGPL-3.0-only
#![forbid(unsafe_code)]
#![deny(missing_docs)]
//! `outlap-conformance` — a test-only crate that cross-checks outlap's solvers against trusted
//! reference integrators.
//!
//! It exists so the non-wasm `diffsol` reference dependency (HANDOFF §11.2) lives in exactly one
//! place, isolated from the wasm-clean solver crates and from their process-global dhat allocation
//! gates. The crate ships no runtime code; the checks are integration tests under `tests/`.
