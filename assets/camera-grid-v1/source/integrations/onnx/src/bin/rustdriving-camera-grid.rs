//! Optional complete small-object grid; pixel hypotheses never command a vehicle.
#[allow(dead_code)]
#[path = "../yolox_s.rs"]
mod baseline_yolox_s;
#[path = "../camera_depthwise.rs"]
mod camera_depthwise;
#[path = "../camera_grid_parallel.rs"]
mod camera_grid_parallel;
#[path = "../camera_nano_repeat2.rs"]
mod camera_repeat2;
#[allow(dead_code)]
#[path = "../camera_depthwise_detector.rs"]
mod frozen_detector;
#[allow(dead_code)]
#[path = "../yolox_tiles.rs"]
mod original_tiles;
#[allow(dead_code)]
#[path = "../camera_analysis.rs"]
mod serial_camera_analysis;
#[allow(dead_code)]
#[path = "../camera_grid_detector.rs"]
mod yolox_s;
#[allow(dead_code)]
#[path = "../camera_grid.rs"]
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
                source!("rust-toolchain.toml"),
            ]
            .into_iter()
            .map(|(name, bytes)| (name, format!("{:x}", Sha256::digest(bytes))))
            .collect()
        })
        .clone()
}
fn bounded_json(path: &Path) -> Result<Vec<u8>, String> {
    if !std::fs::symlink_metadata(path)
        .map_err(|e| e.to_string())?
        .is_file()
    {
        return Err("fixture must be regular, not a symlink".into());
    }
    let mut body = Vec::new();
    std::fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(4 * 1024 * 1024 + 1)
        .read_to_end(&mut body)
        .map_err(|e| e.to_string())?;
    if body.len() > 4 * 1024 * 1024 {
        return Err("fixture byte budget exceeded".into());
    }
    Ok(body)
}
fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let destination=match args.as_slice() {
        [m,_,i,_,o,d] if m=="--model" && (i=="--image" || i=="--reference-image") && o=="--output"=>d,
        [m,_,o,d] if (m=="--merge-fixture" || m=="--reconcile-fixture" || m=="--inspect-image") && o=="--output"=>d,
        _=>return Err("expected --model FILE --image FILE --output NEW; --model FILE --reference-image FILE --output NEW; --inspect-image FILE --output NEW; --merge-fixture FILE --output NEW; --reconcile-fixture FILE --output NEW".into()),
    };
    let output = camera_grid_parallel::AtomicReport::reserve(Path::new(destination))?;
    if args[0] == "--merge-fixture" || args[0] == "--reconcile-fixture" {
        let fixture: yolox_tiles::Fixture =
            serde_json::from_slice(&bounded_json(Path::new(&args[1]))?)
                .map_err(|e| e.to_string())?;
        if args[0] == "--merge-fixture" {
            return output.publish(&yolox_tiles::merge(&fixture)?);
        }
        return output.publish(&yolox_vehicle_consistency::reconcile(&fixture)?);
    }
    let source = if args[0] == "--model" {
        &args[3]
    } else {
        &args[1]
    };
    let (image, sha) = camera_grid_parallel::read_image(Path::new(source))?;
    let rectangles = yolox_tiles::rectangles(image.width(), image.height())?;
    if args[0] == "--inspect-image" {
        return output.publish(&serde_json::json!({"schema_version":1,"algorithm":"bounded_grid_inventory_v1","compiled_sources":compiled_sources(),"source_image_sha256":sha,"width":image.width(),"height":image.height(),"views":rectangles,"expected_model_bytes":yolox_s::MODEL_BYTES,"model_read":false,"inference_run":false,"control_permission":false}));
    }
    let detector =
        yolox_s::Detector::comparison(Path::new(&args[1]), args[2] == "--reference-image")?;
    output.publish(&camera_grid_parallel::analyze(&detector, &image, sha)?)
}
fn main() {
    if let Err(error) = run() {
        eprintln!("camera grid: {error}");
        std::process::exit(2);
    }
}
