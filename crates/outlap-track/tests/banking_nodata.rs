// SPDX-License-Identifier: AGPL-3.0-only
//! `track/1.2` banking: `0.0` is a measurement, `NaN` is an absence, and the two forms compose.
//!
//! The distinction exists because a `LiDAR` cross-section that misses its signal-to-noise gate and
//! a genuinely flat corner used to write the identical number. Conflating them meant a
//! hand-annotated corner could silently overwrite a real flat measurement, and it meant assumed
//! geometry was indistinguishable from surveyed geometry.

use std::fmt::Write as _;

use outlap_schema::io::MemLoader;
use outlap_track::Track;

/// A 12-station open track whose banking column is exactly `col`.
fn loader(schema: &str, keypoints: bool, col: &[&str]) -> MemLoader {
    let kp = if keypoints {
        "banking_keypoints:\n  - { s_m: 0.0, banking_deg: 4.0 }\n  - { s_m: 110.0, banking_deg: 4.0 }\n"
    } else {
        ""
    };
    let yaml = format!("schema: {schema}\nname: Mem\nclosed: false\ncenterline: c.csv\n{kp}");
    let mut csv =
        String::from("s_m,x_m,y_m,z_m,banking_deg,width_left_m,width_right_m,grip_scale\n");
    for (i, b) in col.iter().enumerate() {
        let s = f64::from(u32::try_from(i).expect("12-row fixture")) * 10.0;
        writeln!(csv, "{s:.1},{s:.1},0.0,0.0,{b},6.0,6.0,1.0").expect("write");
    }
    MemLoader::new().with("track.yaml", yaml).with("c.csv", csv)
}

/// Twelve stations, all measured flat except a no-data run in the middle.
const GAP: [&str; 12] = [
    "0.0", "0.0", "0.0", "NaN", "NaN", "NaN", "NaN", "0.0", "0.0", "0.0", "0.0", "0.0",
];

#[test]
fn keypoints_fill_the_gap_and_measurements_win_elsewhere() {
    let t = Track::load("track.yaml", &loader("track/1.2", true, &GAP)).expect("loads");
    // Inside the no-data run the hand annotation supplies banking...
    assert!(
        t.banking(45.0).to_degrees() > 3.0,
        "keypoints did not fill the no-data run: {}",
        t.banking(45.0).to_degrees()
    );
    // ...and outside it the measured flat stations stand, rather than the keypoints bleeding
    // across the whole lap the way the pre-1.2 override did.
    assert!(
        t.banking(100.0).to_degrees().abs() < 0.5,
        "a keypoint overwrote a measured flat station: {}",
        t.banking(100.0).to_degrees()
    );
    assert_eq!(t.banking_unresolved(), 0, "every gap was covered");
}

#[test]
fn uncovered_no_data_is_driven_flat_and_counted() {
    let t = Track::load("track.yaml", &loader("track/1.2", false, &GAP)).expect("loads");
    // Flat is the only safe assumption — a NaN would poison every downstream evaluation — but
    // it must be reported as an assumption, not left to look like a measurement.
    assert!(
        t.banking(45.0).is_finite(),
        "no-data leaked into the channel"
    );
    assert!(t.banking(45.0).abs() < 1e-9);
    assert_eq!(
        t.banking_unresolved(),
        4,
        "the assumed stations were not counted"
    );
}

#[test]
fn measured_flat_is_not_reported_as_unresolved() {
    let all_flat = ["0.0"; 12];
    let t = Track::load("track.yaml", &loader("track/1.2", false, &all_flat)).expect("loads");
    assert_eq!(
        t.banking_unresolved(),
        0,
        "a measured-flat track must not be reported as assumed"
    );
}

#[test]
fn pre_1_2_keeps_the_override_semantics() {
    // `track/1.0` documented keypoints as replacing the column outright. Older files must not
    // change behavior because a newer MINOR redefined the column.
    let col = ["0.0"; 12];
    let t = Track::load("track.yaml", &loader("track/1.0", true, &col)).expect("loads");
    assert!(
        t.banking(60.0).to_degrees() > 3.0,
        "1.0 keypoints stopped driving the channel"
    );
    assert_eq!(t.banking_unresolved(), 0, "1.0 cannot express no-data");
}
