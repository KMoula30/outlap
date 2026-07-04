// SPDX-License-Identifier: AGPL-3.0-only
//! The canonical `.tir` ↔ `.tyr` mapping table: section order, per-section key order, and the
//! rules that decide which sections carry MF6.1 coefficients (flattened into [`Mf61Coeffs`]) versus
//! which are metadata the `.tir` codec owns but the `.tyr` model does not store.
//!
//! This layout is the load-bearing contract behind writer determinism: [`crate::tir::write`] emits
//! sections in [`SECTIONS`] order and, within each section, keys in the declared order (any key not
//! listed is appended sorted). A `.tir` produced by the writer is therefore byte-stable, and PR7's
//! Python codec reproduces it by walking this same table.
//!
//! The section assignment follows the TNO MF-Tyre `.tir` layout closely but is defined here as an
//! outlap-internal contract (round-trip fidelity + determinism), not a claim of byte-identity with
//! any third-party tool's export.

/// The `[UNITS]` section name (validated against SI on read; always emitted SI on write).
pub const UNITS_SECTION: &str = "UNITS";

/// The `[MODEL]` section name — file-format metadata (FITTYP, PROPERTY_FILE_FORMAT). Its entries
/// are NOT flattened into the coefficient map; they are the codec's concern.
pub const MODEL_SECTION: &str = "MODEL";

/// Catch-all section for coefficient keys present in a `.tyr` map but absent from [`SECTIONS`]
/// (e.g. keys from a newer MINOR the writer does not yet place). Emitted last, keys sorted, so the
/// writer stays total and deterministic. On read, its numeric entries flatten like any coefficient.
pub const OVERFLOW_SECTION: &str = "USER_COEFFICIENTS";

/// The canonical SI `[UNITS]` declaration (dimension → SI token). Read rejects any other token for
/// a listed dimension (hard error); write emits exactly these, in this order.
pub const SI_UNITS: &[(&str, &str)] = &[
    ("LENGTH", "meter"),
    ("FORCE", "newton"),
    ("ANGLE", "radians"),
    ("MASS", "kg"),
    ("TIME", "second"),
];

/// Sections whose entries are metadata (never flattened into the coefficient map on read).
pub const METADATA_SECTIONS: &[&str] = &[UNITS_SECTION, MODEL_SECTION];

/// Whether a section's entries are MF6.1 coefficients (flattened into [`Mf61Coeffs`] on read).
pub fn is_coefficient_section(section: &str) -> bool {
    !METADATA_SECTIONS.contains(&section)
}

/// The canonical section order and, per section, the canonical key order. The writer walks this
/// table top-to-bottom; PR7's Python codec must reproduce this exact order.
///
/// Every name in [`crate::tyr::KNOWN_MF61_KEYS`] appears exactly once below (checked by a test), so
/// a `.tyr` built from known coefficients writes back with no key falling through to
/// [`OVERFLOW_SECTION`].
pub const SECTIONS: &[(&str, &[&str])] = &[
    (MODEL_SECTION, &[]),
    (UNITS_SECTION, &["LENGTH", "FORCE", "ANGLE", "MASS", "TIME"]),
    ("DIMENSION", &["UNLOADED_RADIUS", "WIDTH", "RIM_RADIUS"]),
    ("OPERATING_CONDITIONS", &["NOMPRES", "LONGVL", "VXLOW"]),
    ("VERTICAL", &["FNOMIN", "VERTICAL_STIFFNESS"]),
    (
        "SCALING_COEFFICIENTS",
        &[
            "LFZO", "LCX", "LMUX", "LEX", "LKX", "LHX", "LVX", "LCY", "LMUY", "LEY", "LKY", "LKYC",
            "LKZC", "LHY", "LVY", "LTR", "LRES", "LXAL", "LYKA", "LVYKA", "LS", "LMX", "LMY",
            "LVMX", "LGYR", "LSGKP", "LSGAL",
        ],
    ),
    (
        "LONGITUDINAL_COEFFICIENTS",
        &[
            "PCX1", "PDX1", "PDX2", "PDX3", "PEX1", "PEX2", "PEX3", "PEX4", "PKX1", "PKX2", "PKX3",
            "PHX1", "PHX2", "PVX1", "PVX2", "PPX1", "PPX2", "PPX3", "PPX4", "RBX1", "RBX2", "RBX3",
            "RCX1", "REX1", "REX2", "RHX1", "PTX1", "PTX2", "PTX3",
        ],
    ),
    (
        "OVERTURNING_COEFFICIENTS",
        &[
            "QSX1", "QSX2", "QSX3", "QSX4", "QSX5", "QSX6", "QSX7", "QSX8", "QSX9", "QSX10",
            "QSX11", "PPMX1",
        ],
    ),
    (
        "LATERAL_COEFFICIENTS",
        &[
            "PCY1", "PDY1", "PDY2", "PDY3", "PEY1", "PEY2", "PEY3", "PEY4", "PEY5", "PKY1", "PKY2",
            "PKY3", "PKY4", "PKY5", "PKY6", "PKY7", "PHY1", "PHY2", "PVY1", "PVY2", "PVY3", "PVY4",
            "PPY1", "PPY2", "PPY3", "PPY4", "PPY5", "RBY1", "RBY2", "RBY3", "RBY4", "RCY1", "REY1",
            "REY2", "RHY1", "RHY2", "RVY1", "RVY2", "RVY3", "RVY4", "RVY5", "RVY6", "PTY1", "PTY2",
        ],
    ),
    (
        "ROLLING_COEFFICIENTS",
        &[
            "QSY1", "QSY2", "QSY3", "QSY4", "QSY5", "QSY6", "QSY7", "QSY8",
        ],
    ),
    (
        "ALIGNING_COEFFICIENTS",
        &[
            "QBZ1", "QBZ2", "QBZ3", "QBZ4", "QBZ5", "QBZ6", "QBZ9", "QBZ10", "QCZ1", "QDZ1",
            "QDZ2", "QDZ3", "QDZ4", "QDZ6", "QDZ7", "QDZ8", "QDZ9", "QDZ10", "QDZ11", "QEZ1",
            "QEZ2", "QEZ3", "QEZ4", "QEZ5", "QHZ1", "QHZ2", "QHZ3", "QHZ4", "PPZ1", "PPZ2", "SSZ1",
            "SSZ2", "SSZ3", "SSZ4",
        ],
    ),
    (
        "STRUCTURAL",
        &[
            "LONGITUDINAL_STIFFNESS",
            "LATERAL_STIFFNESS",
            "PCFX1",
            "PCFX2",
            "PCFX3",
            "PCFY1",
            "PCFY2",
            "PCFY3",
        ],
    ),
];

/// The `[MODEL]` metadata the writer emits for an MF6.1 tyre, as `(key, value)` pairs already in
/// canonical order. `FITTYP` is numeric; `PROPERTY_FILE_FORMAT` is a quoted string.
pub const MODEL_METADATA: &[(&str, &str)] = &[("FITTYP", "61"), ("PROPERTY_FILE_FORMAT", "MF61")];

/// The canonical section for a known coefficient key, if any. `None` means the key is not placed by
/// the table and will be written into [`OVERFLOW_SECTION`].
pub fn section_for(key: &str) -> Option<&'static str> {
    SECTIONS
        .iter()
        .find(|(_, keys)| keys.contains(&key))
        .map(|(name, _)| *name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tyr::KNOWN_MF61_KEYS;

    #[test]
    fn every_known_key_has_exactly_one_home() {
        for key in KNOWN_MF61_KEYS {
            let homes: Vec<&str> = SECTIONS
                .iter()
                .filter(|(_, keys)| keys.contains(key))
                .map(|(name, _)| *name)
                .collect();
            assert_eq!(
                homes.len(),
                1,
                "`{key}` must map to exactly one section, got {homes:?}"
            );
        }
    }

    #[test]
    fn no_stray_keys_in_table() {
        // Every coefficient-section key is a known MF6.1 coefficient (guards against typos).
        for (section, keys) in SECTIONS {
            if !is_coefficient_section(section) {
                continue;
            }
            for key in *keys {
                assert!(
                    KNOWN_MF61_KEYS.contains(key),
                    "section `{section}` lists unknown key `{key}`"
                );
            }
        }
    }

    #[test]
    fn no_duplicate_keys_across_sections() {
        let mut seen = std::collections::BTreeSet::new();
        for (_, keys) in SECTIONS {
            for key in *keys {
                assert!(seen.insert(*key), "`{key}` listed in two sections");
            }
        }
    }
}
