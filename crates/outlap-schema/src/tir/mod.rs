// SPDX-License-Identifier: AGPL-3.0-only
//! The `.tir` interchange codec: parse/write the TNO MF-Tyre `.tir` text format and convert it to
//! and from the outlap [`Tyr`] model. Implemented clean-room from the public `.tir` layout (never
//! from any GPL parser or game-engine source).
//!
//! The codec is **string-in / string-out**: [`parse_tir`] and [`write_tir`] touch neither the
//! filesystem, threads, nor the clock (the crate stays wasm-clean); only [`load_tir`] does IO, and
//! only through the [`SourceLoader`](crate::io::SourceLoader) trait, mirroring
//! [`load_tyr`](crate::load::load_tyr).
//!
//! # Canonical number format (cross-language contract)
//!
//! [`write_tir`] renders every number through the single routine [`write::format_number`]. The
//! format is defined so PR7's Python codec reproduces it **byte-for-byte**. For a finite `x`:
//!
//! 1. `±0.0` → `"0"`.
//! 2. Otherwise take the *shortest round-tripping* decimal significand `d₁d₂…dₙ` (with `d₁ ≠ 0` and
//!    no trailing zeros) and the base-10 exponent `E` of the leading digit, so
//!    `|x| = d₁.d₂…dₙ × 10^E`. Ties (values with two equally-short round-tripping decimals) break
//!    **round-half-to-even**. The Rust writer takes these digits from the `ryu` crate, whose
//!    round-half-to-even output matches Python `repr` exactly — note that Rust's *own* `{:e}` /
//!    `Display` rounds ties away-from-even and must NOT be used here.
//! 3. **Plain** decimal when `-4 ≤ E ≤ 15` — no exponent, no forced trailing `.0`
//!    (e.g. `4000`, `0.33`, `0.0009`, `1000000000000000`). This is Rust's `Display`/Python `repr`
//!    plain form with any trailing `.0` stripped.
//! 4. **Scientific** otherwise (`E < -4` or `E > 15`): `mantissa e±EE`, the exponent sign always
//!    present and the magnitude zero-padded to at least two digits (`1e-05`, `1e+16`, `1.5e-300`).
//!    This is exactly Python `repr`'s scientific form.
//!
//! The **exponent-switch thresholds are `E < -4` and `E > 15`** (equivalently: scientific when the
//! decimal point would sit before the 4th leading zero, or past 16 integer digits). These match
//! CPython's `repr` switch points, so the Python codec may implement step 3–4 by taking `repr(x)`
//! and normalising the exponent field.
//!
//! # Round-trip fidelity (documented asymmetry)
//!
//! * `tir → `[`TirDoc`]` → tir` is **byte-stable** for canonically-written input (the writer's own
//!   output; comments/whitespace/section-order are normalised on the way through).
//! * `tir → `[`Tyr`]` → tir` is **numeric-exact over the mapping-table keys only** — the MF6.1
//!   coefficients and the `.tir` housekeeping numbers. It is *not* a full round-trip: `.tir` carries
//!   no `thermal`/`wear` physics, so [`tir_to_tyr`] must *synthesise* those blocks (see
//!   [`ThermalWearPolicy`]) and [`tyr_to_tir`] drops them again, regenerating only the `[MODEL]`/
//!   `[UNITS]` metadata. Coefficient values pass through unchanged.
//!
//! # Units policy
//!
//! A `[UNITS]` block is validated on read: any listed dimension declared with a non-SI token is a
//! hard error (SI is [`map::SI_UNITS`]). The writer always emits the SI block.

mod map;
mod parse;
mod write;

pub use parse::parse_tir;
pub use write::{format_number, write_tir};

use crate::error::{Result, SchemaError};
use crate::io::SourceLoader;
use crate::load::report::ReportEntry;
use crate::tyr::{
    Mf61Coeffs, Tyr, TyrProvenance, TyrThermal, TyrWear, KNOWN_MF61_KEYS, REQUIRED_FORCE_KEYS,
    REQUIRED_STRUCTURAL_KEYS,
};
use crate::version::SchemaVersion;
use crate::{schema_name, SCHEMA_MAJOR};

use map::{
    is_coefficient_section, section_for, MODEL_METADATA, MODEL_SECTION, OVERFLOW_SECTION, SECTIONS,
    SI_UNITS, UNITS_SECTION,
};

/// A parsed `.tir` document: an ordered list of sections, plus the label used in diagnostics.
#[derive(Clone, Debug, PartialEq)]
pub struct TirDoc {
    /// The source label (e.g. the file path), used only for diagnostic messages.
    pub label: String,
    /// The document's sections, in source order.
    pub sections: Vec<TirSection>,
}

impl TirDoc {
    /// Construct a document from a label and its sections.
    pub fn new(label: impl Into<String>, sections: Vec<TirSection>) -> Self {
        Self {
            label: label.into(),
            sections,
        }
    }

    /// The first section named `name`, if present.
    pub fn section(&self, name: &str) -> Option<&TirSection> {
        self.sections.iter().find(|s| s.name == name)
    }
}

/// A `[SECTION]` and its entries.
#[derive(Clone, Debug, PartialEq)]
pub struct TirSection {
    /// The section name (uppercased), without the surrounding brackets.
    pub name: String,
    /// The `KEY = value` entries, in source order.
    pub entries: Vec<TirEntry>,
}

/// A single `KEY = value` entry.
#[derive(Clone, Debug, PartialEq)]
pub struct TirEntry {
    /// The key (uppercased).
    pub key: String,
    /// The value.
    pub value: TirValue,
}

/// A `.tir` value: either a number or a (quoted) string.
#[derive(Clone, Debug, PartialEq)]
pub enum TirValue {
    /// A numeric value, rendered on write by the canonical [`format_number`].
    Number(f64),
    /// A string value, emitted single-quoted on write.
    Text(String),
}

/// How [`tir_to_tyr`] obtains the `thermal`/`wear` blocks that a `.tir` file does not carry.
#[derive(Clone, Debug, Default)]
pub enum ThermalWearPolicy {
    /// Fill both blocks with documented, physically-plausible synthetic defaults, and mark the
    /// resulting model's provenance as synthetic.
    #[default]
    Synthetic,
    /// Copy both blocks from a donor `.tyr` (e.g. a comparable tyre fitted with thermal/wear).
    FromDonor {
        /// The donor thermal-ring parameters.
        thermal: Box<TyrThermal>,
        /// The donor wear/cliff parameters.
        wear: Box<TyrWear>,
    },
    /// Do not synthesise: [`tir_to_tyr`] returns an error, since a [`Tyr`] requires both blocks.
    None,
}

/// Options controlling [`tir_to_tyr`].
#[derive(Clone, Debug, Default)]
pub struct TirToTyrOptions {
    /// The thermal/wear synthesis policy (see [`ThermalWearPolicy`]).
    pub thermal_wear: ThermalWearPolicy,
}

/// Build the coefficient map from a `.tir` document, flattening every numeric entry in a
/// coefficient section (everything but `[MODEL]`/`[UNITS]`). Emits did-you-mean warnings for
/// unrecognised coefficient names, mirroring the `.tyr` semantic stage.
fn collect_coeffs(doc: &TirDoc, warnings: &mut Vec<ReportEntry>) -> Mf61Coeffs {
    let mut map = std::collections::BTreeMap::new();
    for section in &doc.sections {
        if !is_coefficient_section(&section.name) {
            continue;
        }
        for entry in &section.entries {
            match &entry.value {
                TirValue::Number(n) => {
                    if !KNOWN_MF61_KEYS.contains(&entry.key.as_str()) {
                        let hint = crate::diagnostics::suggest(
                            &entry.key,
                            KNOWN_MF61_KEYS.iter().copied(),
                        )
                        .map(|s| format!(" (did you mean `{s}`?)"))
                        .unwrap_or_default();
                        warnings.push(ReportEntry::new(
                            format!("/mf61/{}", entry.key),
                            format!(
                                "unknown MF6.1 coefficient `{}`{hint} — carried through unvalidated",
                                entry.key
                            ),
                        ));
                    }
                    map.insert(entry.key.clone(), *n);
                }
                TirValue::Text(_) => warnings.push(ReportEntry::new(
                    format!("/mf61/{}", entry.key),
                    format!(
                        "non-numeric value for coefficient `{}` in `[{}]` — ignored",
                        entry.key, section.name
                    ),
                )),
            }
        }
    }
    Mf61Coeffs(map)
}

/// Convert a parsed [`TirDoc`] into a [`Tyr`], synthesising `thermal`/`wear` per `opts`. Returns the
/// model plus warnings (unknown coefficients, and a note recording the thermal/wear provenance).
///
/// This performs the mechanical mapping only; it does *not* enforce the MF6.1 required-key
/// semantics (that is [`load_tir`]'s job), so it is safe to call over partial coefficient sets.
pub fn tir_to_tyr(doc: &TirDoc, opts: &TirToTyrOptions) -> Result<(Tyr, Vec<ReportEntry>)> {
    let mut warnings = Vec::new();
    let mf61 = collect_coeffs(doc, &mut warnings);

    let (thermal, wear, synthetic, note) = match &opts.thermal_wear {
        ThermalWearPolicy::Synthetic => (
            synthetic_thermal(),
            synthetic_wear(),
            true,
            "thermal/wear synthesised (not present in `.tir`)".to_owned(),
        ),
        ThermalWearPolicy::FromDonor { thermal, wear } => (
            (**thermal).clone(),
            (**wear).clone(),
            false,
            "thermal/wear taken from donor `.tyr` (not present in `.tir`)".to_owned(),
        ),
        ThermalWearPolicy::None => {
            return Err(SchemaError::semantic(
                &no_sources(&doc.label),
                crate::diagnostics::SrcSpan::blank(0),
                "`.tir` carries no thermal/wear model",
                Some(
                    "choose a thermal/wear policy (`Synthetic` or `FromDonor`) to build a `.tyr`"
                        .into(),
                ),
            ));
        }
    };
    warnings.push(ReportEntry::new("/thermal", note.clone()));
    warnings.push(ReportEntry::new("/wear", note));

    let tyr = Tyr {
        schema: SchemaVersion::new(schema_name::TYR, SCHEMA_MAJOR, 0),
        mf61,
        brush: None,
        vertical: None,
        thermal,
        wear,
        provenance: TyrProvenance {
            citation: "MF6.1 coefficients imported from a `.tir` file".to_owned(),
            source: format!("imported from `.tir` `{}`", doc.label),
            synthetic,
        },
    };
    Ok((tyr, warnings))
}

/// Convert a [`Tyr`] into a [`TirDoc`] carrying its MF6.1/housekeeping numbers plus regenerated
/// `[MODEL]`/`[UNITS]` metadata. `thermal`/`wear`/`brush` have no `.tir` representation and are
/// dropped (see the module's round-trip asymmetry note).
pub fn tyr_to_tir(tyr: &Tyr) -> TirDoc {
    let mut sections: Vec<TirSection> = Vec::new();

    for (name, keys) in SECTIONS {
        let entries: Vec<TirEntry> = match *name {
            MODEL_SECTION => MODEL_METADATA
                .iter()
                .map(|(k, v)| TirEntry {
                    key: (*k).to_owned(),
                    value: model_metadata_value(k, v),
                })
                .collect(),
            UNITS_SECTION => SI_UNITS
                .iter()
                .map(|(dim, si)| TirEntry {
                    key: (*dim).to_owned(),
                    value: TirValue::Text((*si).to_owned()),
                })
                .collect(),
            _ => keys
                .iter()
                .filter_map(|k| {
                    tyr.mf61.0.get(*k).map(|v| TirEntry {
                        key: (*k).to_owned(),
                        value: TirValue::Number(*v),
                    })
                })
                .collect(),
        };
        if !entries.is_empty() {
            sections.push(TirSection {
                name: (*name).to_owned(),
                entries,
            });
        }
    }

    // Any coefficient not placed by the table → a deterministic overflow section (keys sorted).
    // Keys are canonicalised to uppercase, matching the parser, so the output re-parses identically.
    let mut overflow: Vec<TirEntry> = tyr
        .mf61
        .0
        .iter()
        .filter(|(k, _)| section_for(k).is_none())
        .map(|(k, v)| TirEntry {
            key: k.to_ascii_uppercase(),
            value: TirValue::Number(*v),
        })
        .collect();
    if !overflow.is_empty() {
        overflow.sort_by(|a, b| a.key.cmp(&b.key));
        sections.push(TirSection {
            name: OVERFLOW_SECTION.to_owned(),
            entries: overflow,
        });
    }

    TirDoc::new("<tyr>", sections)
}

/// Load, parse, and validate a `.tir` file through `loader`, converting it to a [`Tyr`] per `opts`.
///
/// Mirrors [`load_tyr`](crate::load::load_tyr): returns the model and any non-fatal warnings. Unlike
/// [`tir_to_tyr`], this enforces the MF6.1 required-key semantics (structural + force core), since a
/// `.tir` has no brush block to supply the force model.
pub fn load_tir(
    path: &str,
    loader: &dyn SourceLoader,
    opts: &TirToTyrOptions,
) -> Result<(Tyr, Vec<ReportEntry>)> {
    let content = loader.load(path)?;
    let (doc, mut warnings) = parse_tir(path, &content)?;
    let (tyr, w2) = tir_to_tyr(&doc, opts)?;
    warnings.extend(w2);
    require_keys(&tyr, path, &content)?;
    Ok((tyr, warnings))
}

/// Enforce that the coefficient map has the structural keys and the full pure-slip force core.
fn require_keys(tyr: &Tyr, path: &str, content: &str) -> Result<()> {
    let sources = {
        let mut s = crate::diagnostics::Sources::new();
        s.add(path, content.to_owned());
        s
    };
    for key in REQUIRED_STRUCTURAL_KEYS.iter().chain(REQUIRED_FORCE_KEYS) {
        if !tyr.mf61.0.contains_key(*key) {
            return Err(SchemaError::semantic(
                &sources,
                crate::diagnostics::SrcSpan::blank(0),
                format!("`.tir` is missing required MF6.1 coefficient `{key}`"),
                Some("a `.tir` must supply the full structural + pure-slip force core".into()),
            ));
        }
    }
    Ok(())
}

/// The `[MODEL]` metadata value for a key: `FITTYP` is numeric, everything else a string.
fn model_metadata_value(key: &str, raw: &str) -> TirValue {
    if key == "FITTYP" {
        if let Ok(n) = raw.parse::<f64>() {
            return TirValue::Number(n);
        }
    }
    TirValue::Text(raw.to_owned())
}

/// A single-file [`Sources`](crate::diagnostics::Sources) for a blank-span diagnostic.
fn no_sources(label: &str) -> crate::diagnostics::Sources {
    let mut s = crate::diagnostics::Sources::new();
    s.add(label, String::new());
    s
}

/// Documented synthetic thermal-ring defaults (a physically-plausible generic racing tyre).
fn synthetic_thermal() -> TyrThermal {
    TyrThermal {
        c_s: 8000.0,
        c_c: 22000.0,
        c_g: 1500.0,
        g_sc: 90.0,
        g_cg: 40.0,
        g_road: 250.0,
        h0: 15.0,
        h1: 5.5,
        p_t: 0.65,
        t_opt: 95.0,
        c_t: 2.2,
        k_c: 0.0015,
        t_c_ref: 80.0,
        p_cold: 138.0,
        t_cold: 20.0,
    }
}

/// Documented synthetic wear/cliff defaults (a physically-plausible generic racing tyre).
fn synthetic_wear() -> TyrWear {
    TyrWear {
        k_w: 0.0009,
        w_max: 8.0,
        w_c: 2.0,
        tau_d: 600.0,
        t_deg: 120.0,
        delta_t_ref: 20.0,
        beta: 2.0,
        delta_c: 0.25,
        s_w: 0.5,
        delta_d: 0.30,
    }
}
