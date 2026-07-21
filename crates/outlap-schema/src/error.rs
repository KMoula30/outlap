// SPDX-License-Identifier: AGPL-3.0-only
//! [`SchemaError`] — the typed, miette-annotated error surface for loading a vehicle.
//!
//! One variant per pipeline stage. Every variant that has a location carries the offending file
//! (`#[source_code]`) and a byte span (`#[label]`) so miette renders an underlined, plain-language
//! diagnostic. A bare serde error never reaches the user — stage 5 wraps it with a path and span.

use miette::{Diagnostic, LabeledSpan, NamedSource, SourceSpan};
use thiserror::Error;

use crate::diagnostics::{Sources, SrcSpan};
use crate::io::SourceError;

/// The result of any fallible load/resolve operation.
pub type Result<T> = std::result::Result<T, SchemaError>;

/// A typed loading/validation error.
#[derive(Debug, Error, Diagnostic)]
pub enum SchemaError {
    /// A source file could not be read.
    #[error(transparent)]
    #[diagnostic(code(outlap::schema::io))]
    Io(#[from] SourceError),

    /// YAML syntax / structural error (stage 1). Anchors, aliases, merge keys, duplicate keys,
    /// and non-mapping top levels all surface here.
    #[error("{message}")]
    #[diagnostic(code(outlap::schema::parse))]
    Parse {
        /// Human-readable message.
        message: String,
        /// The file the error is in.
        #[source_code]
        src: NamedSource<String>,
        /// Location of the syntax error.
        #[label("here")]
        span: SourceSpan,
    },

    /// The `schema:` version is wrong (stage 2): wrong document kind or an incompatible MAJOR.
    #[error("{message}")]
    #[diagnostic(code(outlap::schema::version))]
    SchemaVersionMismatch {
        /// Human-readable message.
        message: String,
        /// The file the error is in.
        #[source_code]
        src: NamedSource<String>,
        /// Location of the `schema:` value.
        #[label("schema version")]
        span: SourceSpan,
        /// Suggested fix.
        #[help]
        help: Option<String>,
    },

    /// An `extends`/overlay/dotted-override resolution failure (stage 3): cycle, missing preset,
    /// or a dotted override targeting a path that does not exist.
    #[error("{message}")]
    #[diagnostic(code(outlap::schema::merge))]
    Merge {
        /// Human-readable message.
        message: String,
        /// The file the error is in.
        #[source_code]
        src: NamedSource<String>,
        /// Location of the offending `extends`/key.
        #[label("here")]
        span: SourceSpan,
    },

    /// An unknown (non-`x-`) field (stage 4).
    #[error("unknown field `{field}`")]
    #[diagnostic(code(outlap::schema::unknown_field))]
    UnknownField {
        /// The offending field name.
        field: String,
        /// The file the error is in.
        #[source_code]
        src: NamedSource<String>,
        /// Location of the unknown key.
        #[label("unknown field")]
        span: SourceSpan,
        /// Did-you-mean suggestion.
        #[help]
        help: Option<String>,
    },

    /// A post-merge deserialize error (stage 5): type mismatch, missing required field, bad enum
    /// tag. Carries the JSON path and the resolved span.
    #[error("{message} (at `{path}`)")]
    #[diagnostic(code(outlap::schema::deserialize))]
    Deserialize {
        /// The JSON path (dotted) at which deserialization failed.
        path: String,
        /// Human-readable message.
        message: String,
        /// The file the error is in.
        #[source_code]
        src: NamedSource<String>,
        /// Location of the offending value.
        #[label("here")]
        span: SourceSpan,
    },

    /// A semantic / range / consistency violation on the typed model (stage 6).
    #[error("{message}")]
    #[diagnostic(code(outlap::schema::semantic))]
    Semantic {
        /// Human-readable message.
        message: String,
        /// The file the error is in.
        #[source_code]
        src: NamedSource<String>,
        /// Location of the offending value.
        #[label("here")]
        span: SourceSpan,
        /// Optional fix hint.
        #[help]
        help: Option<String>,
    },

    /// A drivetrain topology-graph violation (stage 7): unreachable wheel, ratio conflict, illegal
    /// combination. Plain-language message plus one or more offending spans.
    #[error("{message}")]
    #[diagnostic(code(outlap::schema::topology))]
    Topology {
        /// Plain-language message.
        message: String,
        /// The file the error is in (the vehicle document).
        #[source_code]
        src: NamedSource<String>,
        /// The offending spans (`wheels:`/`path:`/`source:` tokens).
        #[label(collection)]
        labels: Vec<LabeledSpan>,
    },

    /// A pre-2.0 (`ers:` / singleton `battery:`) drivetrain layout on a `vehicle` document
    /// (D-M6-13). There is no `outlap migrate`: the ERS/drivetrain restructure is a hand-rewrite,
    /// so a legacy file hard-fails with a curated pointer to the new layout.
    #[error("{message}")]
    #[diagnostic(code(outlap::schema::legacy_drivetrain))]
    LegacyDrivetrainFormat {
        /// Human-readable message.
        message: String,
        /// The file the error is in (the vehicle document).
        #[source_code]
        src: NamedSource<String>,
        /// Location of the offending legacy key (`ers:` / singular `battery:`).
        #[label("legacy ERS/drivetrain layout")]
        span: SourceSpan,
        /// Pointer to the new `drivetrain.units[]` + `policy:`/`batteries:` layout.
        #[help]
        help: Option<String>,
    },
}

impl SchemaError {
    /// Build a [`NamedSource`] for the file a span points into.
    fn named(sources: &Sources, span: SrcSpan) -> NamedSource<String> {
        sources.named(span.source)
    }

    /// Construct a [`SchemaError::Parse`].
    pub fn parse(sources: &Sources, span: SrcSpan, message: impl Into<String>) -> Self {
        Self::Parse {
            message: message.into(),
            src: Self::named(sources, span),
            span: span.to_miette(),
        }
    }

    /// Construct a [`SchemaError::SchemaVersionMismatch`].
    pub fn version(
        sources: &Sources,
        span: SrcSpan,
        message: impl Into<String>,
        help: Option<String>,
    ) -> Self {
        Self::SchemaVersionMismatch {
            message: message.into(),
            src: Self::named(sources, span),
            span: span.to_miette(),
            help,
        }
    }

    /// Construct a [`SchemaError::Merge`].
    pub fn merge(sources: &Sources, span: SrcSpan, message: impl Into<String>) -> Self {
        Self::Merge {
            message: message.into(),
            src: Self::named(sources, span),
            span: span.to_miette(),
        }
    }

    /// Construct a [`SchemaError::UnknownField`].
    pub fn unknown_field(
        sources: &Sources,
        span: SrcSpan,
        field: impl Into<String>,
        help: Option<String>,
    ) -> Self {
        Self::UnknownField {
            field: field.into(),
            src: Self::named(sources, span),
            span: span.to_miette(),
            help,
        }
    }

    /// Construct a [`SchemaError::Deserialize`].
    pub fn deserialize(
        sources: &Sources,
        span: SrcSpan,
        path: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self::Deserialize {
            path: path.into(),
            message: message.into(),
            src: Self::named(sources, span),
            span: span.to_miette(),
        }
    }

    /// Construct a [`SchemaError::Semantic`].
    pub fn semantic(
        sources: &Sources,
        span: SrcSpan,
        message: impl Into<String>,
        help: Option<String>,
    ) -> Self {
        Self::Semantic {
            message: message.into(),
            src: Self::named(sources, span),
            span: span.to_miette(),
            help,
        }
    }

    /// Construct a [`SchemaError::Topology`] from labelled spans (all in one file).
    pub fn topology(
        sources: &Sources,
        source_file: crate::diagnostics::SourceId,
        message: impl Into<String>,
        labels: Vec<(SrcSpan, String)>,
    ) -> Self {
        let named = sources.named(source_file);
        let labels = labels
            .into_iter()
            .map(|(span, text)| LabeledSpan::new(Some(text), span.offset, span.len))
            .collect();
        Self::Topology {
            message: message.into(),
            src: named,
            labels,
        }
    }

    /// Construct a [`SchemaError::LegacyDrivetrainFormat`].
    pub fn legacy_drivetrain(
        sources: &Sources,
        span: SrcSpan,
        message: impl Into<String>,
        help: Option<String>,
    ) -> Self {
        Self::LegacyDrivetrainFormat {
            message: message.into(),
            src: Self::named(sources, span),
            span: span.to_miette(),
            help,
        }
    }
}
