// SPDX-License-Identifier: AGPL-3.0-only
//! [`SchemaVersion`] — the `<name>/<MAJOR>.<MINOR>` string every outlap file carries.

use std::borrow::Cow;
use std::fmt;
use std::str::FromStr;

use schemars::{json_schema, JsonSchema, Schema, SchemaGenerator};
use serde::{Deserialize, Serialize};

/// A parsed `schema:` version string of the form `<name>/<MAJOR>.<MINOR>` (e.g. `vehicle/1.0`).
///
/// Loaders accept a file whose MAJOR equals the loader's MAJOR (§9 versioning rule); MINOR is
/// informational (forward-compatible within a major). The name half distinguishes document kinds
/// (`vehicle`, `ptm`, `tyr`, `emotor`) so a `.tyr` fed where a `vehicle` is expected fails the
/// version gate rather than deserializing into nonsense.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct SchemaVersion {
    /// The document kind, e.g. `vehicle`. Lowercase `[a-z_]+`.
    pub name: String,
    /// The MAJOR version.
    pub major: u16,
    /// The MINOR version.
    pub minor: u16,
}

impl SchemaVersion {
    /// Construct a version from its parts.
    pub fn new(name: impl Into<String>, major: u16, minor: u16) -> Self {
        Self {
            name: name.into(),
            major,
            minor,
        }
    }

    /// Whether this version is loadable by a loader declaring `(name, major)` — same name, same major.
    pub fn is_compatible_with(&self, name: &str, major: u16) -> bool {
        self.name == name && self.major == major
    }
}

/// Error returned when a `schema:` string is malformed (bad shape, non-numeric version parts).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("malformed schema version `{0}`: expected `<name>/<MAJOR>.<MINOR>` (e.g. `vehicle/1.0`)")]
pub struct ParseSchemaVersionError(pub String);

impl FromStr for SchemaVersion {
    type Err = ParseSchemaVersionError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let err = || ParseSchemaVersionError(s.to_owned());
        let (name, ver) = s.split_once('/').ok_or_else(err)?;
        let (major, minor) = ver.split_once('.').ok_or_else(err)?;
        if name.is_empty() || !name.bytes().all(|b| b.is_ascii_lowercase() || b == b'_') {
            return Err(err());
        }
        let major = major.parse::<u16>().map_err(|_| err())?;
        let minor = minor.parse::<u16>().map_err(|_| err())?;
        Ok(Self {
            name: name.to_owned(),
            major,
            minor,
        })
    }
}

impl fmt::Display for SchemaVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}.{}", self.name, self.major, self.minor)
    }
}

// serde: represent on the wire as the compact string, not a struct.
impl TryFrom<String> for SchemaVersion {
    type Error = ParseSchemaVersionError;
    fn try_from(value: String) -> Result<Self, Self::Error> {
        value.parse()
    }
}

impl From<SchemaVersion> for String {
    fn from(v: SchemaVersion) -> Self {
        v.to_string()
    }
}

impl<'de> Deserialize<'de> for SchemaVersion {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        s.parse().map_err(serde::de::Error::custom)
    }
}

impl Serialize for SchemaVersion {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&self.to_string())
    }
}

// Manual schema: a pattern-constrained string, not the parsed struct.
impl JsonSchema for SchemaVersion {
    fn schema_name() -> Cow<'static, str> {
        "SchemaVersion".into()
    }

    fn schema_id() -> Cow<'static, str> {
        "outlap_schema::version::SchemaVersion".into()
    }

    fn json_schema(_generator: &mut SchemaGenerator) -> Schema {
        json_schema!({
            "type": "string",
            "pattern": r"^[a-z_]+/\d+\.\d+$",
            "description": "Schema version of the form <name>/<MAJOR>.<MINOR>, e.g. vehicle/1.0",
            "examples": ["vehicle/1.0"],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_and_roundtrips() {
        let v: SchemaVersion = "vehicle/1.0".parse().unwrap();
        assert_eq!(v, SchemaVersion::new("vehicle", 1, 0));
        assert_eq!(v.to_string(), "vehicle/1.0");
        assert!(v.is_compatible_with("vehicle", 1));
        assert!(!v.is_compatible_with("vehicle", 2));
        assert!(!v.is_compatible_with("ptm", 1));
    }

    #[test]
    fn rejects_malformed() {
        for bad in [
            "vehicle",
            "vehicle/1",
            "Vehicle/1.0",
            "veh1/1.0",
            "vehicle/x.0",
            "/1.0",
        ] {
            assert!(bad.parse::<SchemaVersion>().is_err(), "should reject {bad}");
        }
    }
}
