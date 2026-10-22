//! Coarse-preserving supported augmentation of complete, internally measured grid views.
// Frozen algorithms are instantiated for distinct four-view and grid contracts.
#![allow(clippy::duplicate_mod)]
#[allow(dead_code)]
#[path = "../yolox_s.rs"]
mod baseline_yolox_s;
#[path = "../camera_capacity_parallel.rs"]
mod camera_grid_parallel;
#[path = "../camera_repeat2.rs"]
mod camera_repeat2;
#[path = "../camera_capacity_supported.rs"]
mod camera_supported;
#[path = "../camera_supported_coarse.rs"]
mod camera_supported_coarse;
#[allow(dead_code)]
#[path = "../yolox_tiles.rs"]
mod original_tiles;
#[allow(dead_code)]
#[path = "../camera_repeat_detector.rs"]
mod repeat_detector;
#[allow(dead_code)]
#[path = "../camera_analysis.rs"]
mod serial_camera_analysis;
#[allow(dead_code)]
#[path = "../camera_capacity_detector.rs"]
mod yolox_s;
#[allow(dead_code)]
#[path = "../camera_capacity_grid.rs"]
mod yolox_tiles;
#[allow(dead_code)]
#[path = "../yolox_vehicle_consistency.rs"]
mod yolox_vehicle_consistency;
#[allow(dead_code)]
#[path = "../yolox_vehicle_context.rs"]
mod yolox_vehicle_context;
#[allow(dead_code)]
#[path = "../yolox_vehicle_vote.rs"]
mod yolox_vehicle_vote;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, env, io::Read, path::Path};
fn compiled_sources() -> BTreeMap<&'static str, String> {
    static SOURCES: std::sync::OnceLock<BTreeMap<&'static str, String>> =
        std::sync::OnceLock::new();
    SOURCES
        .get_or_init(|| {
            macro_rules! source {
                ($name:literal) => {
                    (
                        $name,
                        include_bytes!(concat!("../../../../", $name)).as_slice(),
                    )
                };
            }
            [
                source!("integrations/camera-tracking/src/assignment.rs"),
                source!("integrations/camera-tracking/src/freshness.rs"),
                source!("integrations/camera-tracking/src/lib.rs"),
                source!("integrations/onnx/Cargo.lock"),
                source!("integrations/onnx/Cargo.toml"),
                source!("integrations/onnx/fetch.py"),
                source!("integrations/onnx/fetch_s.py"),
                source!("integrations/onnx/profile-analysis.json"),
                source!("integrations/onnx/profile-depthwise.json"),
                source!("integrations/onnx/profile-grid.json"),
                source!("integrations/onnx/profile-latency.json"),
                source!("integrations/onnx/profile-latest.json"),
                source!("integrations/onnx/profile-nano-operators.json"),
                source!("integrations/onnx/profile-nano-parallel.json"),
                source!("integrations/onnx/profile-nano-repeat2x.json"),
                source!("integrations/onnx/profile-operators.json"),
                source!("integrations/onnx/profile-parallel.json"),
                source!("integrations/onnx/profile-s.json"),
                source!("integrations/onnx/profile-sequence.json"),
                source!("integrations/onnx/profile-supervised.json"),
                source!("integrations/onnx/profile-tiles-s.json"),
                source!("integrations/onnx/profile-vehicle-consistency.json"),
                source!("integrations/onnx/profile-vehicle-context.json"),
                source!("integrations/onnx/profile-vehicle-vote.json"),
                source!("integrations/onnx/src/bin/rustdriving-camera-analyze.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-depthwise-inspect.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-depthwise.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-detect-s.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-detect-tiled-s.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-detect-vehicle-context.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-detect-vehicle-vote.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-grid.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-latency.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-latest.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-nano-profile.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-nano-repeat2x.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-nano.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-parallel.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-profile.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-reconcile.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-sequence.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-supervised.rs"),
                source!("integrations/onnx/src/camera_analysis.rs"),
                source!("integrations/onnx/src/camera_depthwise.rs"),
                source!("integrations/onnx/src/camera_depthwise_analysis.rs"),
                source!("integrations/onnx/src/camera_depthwise_detector.rs"),
                source!("integrations/onnx/src/camera_depthwise_profile.rs"),
                source!("integrations/onnx/src/camera_grid.rs"),
                source!("integrations/onnx/src/camera_grid_detector.rs"),
                source!("integrations/onnx/src/camera_grid_parallel.rs"),
                source!("integrations/onnx/src/camera_latency.rs"),
                source!("integrations/onnx/src/camera_latest.rs"),
                source!("integrations/onnx/src/camera_nano_analysis.rs"),
                source!("integrations/onnx/src/camera_nano_detector.rs"),
                source!("integrations/onnx/src/camera_nano_profile.rs"),
                source!("integrations/onnx/src/camera_nano_repeat2.rs"),
                source!("integrations/onnx/src/camera_nano_repeat_analysis.rs"),
                source!("integrations/onnx/src/camera_nano_repeat_detector.rs"),
                source!("integrations/onnx/src/camera_parallel.rs"),
                source!("integrations/onnx/src/camera_profile.rs"),
                source!("integrations/onnx/src/camera_repeat2.rs"),
                source!("integrations/onnx/src/camera_sequence.rs"),
                source!("integrations/onnx/src/camera_supervised.rs"),
                source!("integrations/onnx/src/lib.rs"),
                source!("integrations/onnx/src/yolox_s.rs"),
                source!("integrations/onnx/src/yolox_tiles.rs"),
                source!("integrations/onnx/src/yolox_vehicle_consistency.rs"),
                source!("integrations/onnx/src/yolox_vehicle_context.rs"),
                source!("integrations/onnx/src/yolox_vehicle_vote.rs"),
                source!("integrations/onnx/src/camera_supported.rs"),
                source!("integrations/onnx/src/camera_supported_coarse.rs"),
                source!("integrations/onnx/src/camera_supported_coarse_context.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-supported.rs"),
                source!("integrations/onnx/profile-supported.json"),
                source!("rust-toolchain.toml"),
                source!("integrations/onnx/src/camera_repeat_detector.rs"),
                source!("integrations/onnx/profile-repeat2x.json"),
                source!("integrations/onnx/src/camera_capacity_grid.rs"),
                source!("integrations/onnx/src/camera_capacity_parallel.rs"),
                source!("integrations/onnx/src/camera_capacity_detector.rs"),
                source!("integrations/onnx/src/camera_capacity_supported.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-capacity.rs"),
                source!("integrations/onnx/profile-capacity.json"),
            ]
            .into_iter()
            .map(|(name, bytes)| (name, format!("{:x}", Sha256::digest(bytes))))
            .collect()
        })
        .clone()
}
fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let destination=match args.as_slice() {
        [m,_,i,_,o,d] if m=="--model" && (i=="--image" || i=="--reference-image") && o=="--output"=>d,
        [f,_,o,d] if f=="--fixture" && o=="--output"=>d,
        _=>return Err("expected --model FILE --image FILE --output NEW; --model FILE --reference-image FILE --output NEW; --fixture FILE --output NEW".into()),
    };
    let output = camera_grid_parallel::AtomicReport::reserve(Path::new(destination))?;
    if args[0] == "--fixture" {
        let path = Path::new(&args[1]);
        if !std::fs::symlink_metadata(path)
            .map_err(|e| e.to_string())?
            .is_file()
        {
            return Err("fixture must be regular and not a symlink".into());
        }
        let mut bytes = Vec::new();
        std::fs::File::open(path)
            .map_err(|e| e.to_string())?
            .take(4 * 1024 * 1024 + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() > 4 * 1024 * 1024 {
            return Err("fixture input exceeds4MiB".into());
        }
        let fixture: yolox_tiles::Fixture =
            serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        let result = camera_supported::reconcile(&fixture)?;
        return output.publish(&serde_json::json!({"schema_version":1,"algorithm":"s_capacity_grid_fixture_v1","compiled_sources":compiled_sources(),"source_fixture_sha256":format!("{:x}",Sha256::digest(bytes)),"model_read":false,"images_read":false,"model_forwards":0,"control_permission":false,"result":result}));
    }
    let (image, sha) = camera_grid_parallel::read_image(Path::new(&args[3]))?;
    yolox_tiles::rectangles(image.width(), image.height())?;
    let detector =
        yolox_s::Detector::comparison(Path::new(&args[1]), args[2] == "--reference-image")?;
    let source = serde_json::to_value(camera_grid_parallel::analyze(
        &detector,
        &image,
        sha.clone(),
    )?)
    .map_err(|e| e.to_string())?;
    let base = &source["base_prediction"];
    let fixture:yolox_tiles::Fixture=serde_json::from_value(serde_json::json!({"width":base["width"],"height":base["height"],
        "views":base["views"].as_array().ok_or("missing internally measured views")?.iter().map(|v|serde_json::json!({"rectangle":v["rectangle"],"detections":v["prediction"]["detections"]})).collect::<Vec<_>>()})).map_err(|e|e.to_string())?;
    let result = camera_supported::reconcile(&fixture)?;
    output.publish(&serde_json::json!({"schema_version":1,"algorithm":"s_capacity_grid_image_v1","compiled_sources":compiled_sources(),"profile_sha256":format!("{:x}",Sha256::digest(include_bytes!("../../profile-capacity.json"))),"source_image_sha256":sha,"source_analysis":source,"control_permission":false,"result":result}))
}
fn main() {
    if let Err(error) = run() {
        eprintln!("camera capacity: {error}");
        std::process::exit(2);
    }
}
