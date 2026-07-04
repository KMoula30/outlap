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
/// `presets/formula_base.yaml`) to the file's UTF-8 content, and — for binary sidecars such as the
/// parquet map tables referenced by a `.ptm`/`aero.map` — to its raw bytes via [`load_bytes`].
///
/// [`load_bytes`]: SourceLoader::load_bytes
pub trait SourceLoader {
    /// Load the content of `path`, or return a [`SourceError`].
    fn load(&self, path: &str) -> Result<String, SourceError>;

    /// Load the raw bytes of `path` (for binary sidecars like parquet map tables).
    ///
    /// The default implementation errors: legacy text-only loaders do not serve binary sidecars.
    /// [`MemLoader`] and [`FsLoader`] override it. Keeping this on the trait (bytes in, no
    /// filesystem) preserves wasm-cleanliness — the parquet *decode* lives behind a feature flag.
    ///
    /// # Errors
    /// [`SourceError`] if the path is not found or cannot be read as bytes.
    fn load_bytes(&self, path: &str) -> Result<Vec<u8>, SourceError> {
        Err(SourceError::Io {
            path: path.to_owned(),
            message: "this loader does not support binary sidecars".to_owned(),
        })
    }
}

impl<T: SourceLoader + ?Sized> SourceLoader for &T {
    fn load(&self, path: &str) -> Result<String, SourceError> {
        (**self).load(path)
    }

    fn load_bytes(&self, path: &str) -> Result<Vec<u8>, SourceError> {
        (**self).load_bytes(path)
    }
}

/// An in-memory loader: a map from logical path to content. Serves wasm and the in-memory API path.
///
/// Text sources go in the `files` map; binary sidecars (parquet) go in the `bytes` map. A path
/// present only as text is still served as bytes (its UTF-8 encoding); a path present as bytes is
/// *not* served as text.
#[derive(Clone, Debug, Default)]
pub struct MemLoader {
    files: BTreeMap<String, String>,
    bytes: BTreeMap<String, Vec<u8>>,
}

impl MemLoader {
    /// Create an empty in-memory loader.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a text file. Returns `self` for chaining.
    #[must_use]
    pub fn with(mut self, path: impl Into<String>, content: impl Into<String>) -> Self {
        self.files.insert(path.into(), content.into());
        self
    }

    /// Insert a text file in place.
    pub fn insert(&mut self, path: impl Into<String>, content: impl Into<String>) {
        self.files.insert(path.into(), content.into());
    }

    /// Insert a binary sidecar. Returns `self` for chaining.
    #[must_use]
    pub fn with_bytes(mut self, path: impl Into<String>, content: impl Into<Vec<u8>>) -> Self {
        self.bytes.insert(path.into(), content.into());
        self
    }

    /// Insert a binary sidecar in place.
    pub fn insert_bytes(&mut self, path: impl Into<String>, content: impl Into<Vec<u8>>) {
        self.bytes.insert(path.into(), content.into());
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

    fn load_bytes(&self, path: &str) -> Result<Vec<u8>, SourceError> {
        if let Some(b) = self.bytes.get(path) {
            return Ok(b.clone());
        }
        self.files
            .get(path)
            .map(|s| s.clone().into_bytes())
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

    fn load_bytes(&self, path: &str) -> Result<Vec<u8>, SourceError> {
        let full = self.root.join(path);
        match std::fs::read(&full) {
            Ok(bytes) => Ok(bytes),
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal text-only loader that does not override `load_bytes` (the legacy case).
    struct LegacyLoader;
    impl SourceLoader for LegacyLoader {
        fn load(&self, _path: &str) -> Result<String, SourceError> {
            Ok("hi".to_owned())
        }
    }

    #[test]
    fn legacy_loader_rejects_binary_sidecars() {
        let err = LegacyLoader.load_bytes("maps.parquet").unwrap_err();
        assert!(matches!(err, SourceError::Io { .. }));
    }

    #[test]
    fn memloader_serves_bytes_and_text_fallback() {
        let loader = MemLoader::new()
            .with("doc.yaml", "schema: ptm/1.0")
            .with_bytes("maps.parquet", vec![1u8, 2, 3]);
        // Explicit binary sidecar.
        assert_eq!(loader.load_bytes("maps.parquet").unwrap(), vec![1, 2, 3]);
        // A text file is also readable as its UTF-8 bytes.
        assert_eq!(
            loader.load_bytes("doc.yaml").unwrap(),
            b"schema: ptm/1.0".to_vec()
        );
        // Missing paths still error.
        assert!(matches!(
            loader.load_bytes("nope").unwrap_err(),
            SourceError::NotFound { .. }
        ));
    }
}
