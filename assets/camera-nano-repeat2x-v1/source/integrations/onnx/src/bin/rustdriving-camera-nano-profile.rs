//! Original Nano operator observation with full ordinary-output bit comparison.
extern crate self as rustdriving_image_tracking;
#[allow(dead_code)]
#[path = "../camera_parallel.rs"]
mod baseline_parallel;
#[allow(dead_code)]
#[path = "../yolox_s.rs"]
mod baseline_yolox_s;
#[allow(dead_code)]
#[path = "../camera_nano_analysis.rs"]
mod camera_analysis;
#[path = "../../../camera-tracking/src/freshness.rs"]
mod camera_freshness;
#[allow(dead_code)]
#[path = "../camera_latency.rs"]
mod camera_latency;
#[allow(dead_code)]
#[path = "../camera_sequence.rs"]
mod camera_sequence;
#[path = "../../../camera-tracking/src/lib.rs"]
mod image_tracking;
#[path = "../camera_nano_detector.rs"]
mod nano_detector;
#[allow(dead_code)]
#[path = "../camera_analysis.rs"]
mod serial_camera_analysis;
mod yolox_s {
    pub use super::nano_detector::{Detector, Report, compiled_sources};
    pub use rustdriving_camera_detect::{
        CLASSES, INPUT_SIZE, MODEL_SHA256, ROWS, decode, verify_model,
    };
}
#[path = "../camera_nano_profile.rs"]
mod camera_profile;
#[path = "../camera_repeat2.rs"]
mod camera_repeat2;
#[path = "../yolox_tiles.rs"]
mod yolox_tiles;
#[path = "../yolox_vehicle_consistency.rs"]
mod yolox_vehicle_consistency;
#[path = "../yolox_vehicle_context.rs"]
mod yolox_vehicle_context;
#[path = "../yolox_vehicle_vote.rs"]
mod yolox_vehicle_vote;
pub use image_tracking::{Acquisition, Config, Detection, Frame, Phase, Snapshot, Tracker};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, env, fs, path::Path};

fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut sources = camera_profile::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/bin/rustdriving-camera-nano-profile.rs",
            include_bytes!("rustdriving-camera-nano-profile.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/camera_nano_profile.rs",
            include_bytes!("../camera_nano_profile.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-nano-operators.json",
            include_bytes!("../../profile-nano-operators.json").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}
fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let (model, image, destination, reference_first) = match args.as_slice() {
        [m, model, i, image, o, d] if m == "--model" && i == "--image" && o == "--output" => {
            (model, image, d, false)
        }
        [m, model, i, image, r, o, d]
            if m == "--model" && i == "--image" && r == "--reference-first" && o == "--output" =>
        {
            (model, image, d, true)
        }
        _ => {
            return Err(
                "expected --model FILE --image FILE [--reference-first] --output NEW".into(),
            );
        }
    };
    let output = camera_analysis::AtomicReport::reserve(Path::new(destination))?;
    let (rgb, sha) =
        camera_analysis::read_image(Path::new(image)).map_err(|e| format!("read image: {e}"))?;
    if !fs::symlink_metadata(model)
        .map_err(|e| e.to_string())?
        .is_file()
    {
        return Err("model must be a regular non-symlink file".into());
    }
    let profile = camera_profile::analyze(Path::new(model), &rgb, sha, false, reference_first)?;
    let mut report = serde_json::to_value(profile).map_err(|e| e.to_string())?;
    report["algorithm"] = serde_json::json!("yolox_nano_cpu_operator_profile_v1");
    report["compiled_sources"] =
        serde_json::to_value(compiled_sources()).map_err(|e| e.to_string())?;
    output.publish(&report)
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
