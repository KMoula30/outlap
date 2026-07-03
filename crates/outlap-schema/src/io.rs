// SPDX-License-Identifier: AGPL-3.0-only
//! Source access behind a trait, so the crate stays wasm-clean.
//!
//! The load pipeline never touches the filesystem directly; it asks a [`SourceLoader`] for the
//! content of a logical path. [`MemLoader`] backs wasm and the in-memory API path; [`FsLoader`]
//! (gated behind the `std` feature) reads from a root directory.

use std::collections::BTreeMap;

/// Error accessing a source through a [`SourceLoader`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SourceError {
    /// The requested logical path was not found.
    #[error("source not found: {path}")]
    NotFound {
        /// The logical path that was requested.
        path: String,
    },
    /// An underlying IO error occurred.
    #[error("failed to read {path}: {message}")]
    Io {
        /// The logical path that was requested.
        path: String,
        /// A human-readable description of the failure.
        message: String,
    },
}

/// Abstract read-only access to source documents by logical path.
///
/// Implementations resolve a logical path (a ref string like `powertrains/rear.ptm` or
/// `presets/formula_base.yaml`) to the file's UTF-8 content.
pub trait SourceLoader {
    /// Load the content of `path`, or return a [`SourceError`].
    fn load(&self, path: &str) -> Result<String, SourceError>;
}

impl<T: SourceLoader + ?Sized> SourceLoader for &T {
    fn load(&self, path: &str) -> Result<String, SourceError> {
        (**self).load(path)
    }
}

/// An in-memory loader: a map from logical path to content. Serves wasm and the in-memory API path.
#[derive(Clone, Debug, Default)]
pub struct MemLoader {
    files: BTreeMap<String, String>,
}

impl MemLoader {
    /// Create an empty in-memory loader.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a file. Returns `self` for chaining.
    #[must_use]
    pub fn with(mut self, path: impl Into<String>, content: impl Into<String>) -> Self {
        self.files.insert(path.into(), content.into());
        self
    }

    /// Insert a file in place.
    pub fn insert(&mut self, path: impl Into<String>, content: impl Into<String>) {
        self.files.insert(path.into(), content.into());
    }
}

impl SourceLoader for MemLoader {
    fn load(&self, path: &str) -> Result<String, SourceError> {
        self.files
            .get(path)
            .cloned()
            .ok_or_else(|| SourceError::NotFound {
                path: path.to_owned(),
            })
    }
}

/// A filesystem loader rooted at a directory. Gated behind the `std` feature (not wasm-clean).
#[cfg(feature = "std")]
#[derive(Clone, Debug)]
pub struct FsLoader {
    root: std::path::PathBuf,
}

#[cfg(feature = "std")]
impl FsLoader {
    /// Create a loader rooted at `root`.
    pub fn new(root: impl Into<std::path::PathBuf>) -> Self {
        Self { root: root.into() }
    }
}

#[cfg(feature = "std")]
impl SourceLoader for FsLoader {
    fn load(&self, path: &str) -> Result<String, SourceError> {
        let full = self.root.join(path);
        match std::fs::read_to_string(&full) {
            Ok(content) => Ok(content),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Err(SourceError::NotFound {
                path: path.to_owned(),
            }),
            Err(e) => Err(SourceError::Io {
                path: path.to_owned(),
                message: e.to_string(),
            }),
        }
    }
}
