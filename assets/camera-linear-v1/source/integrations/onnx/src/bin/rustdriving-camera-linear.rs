//! Offline quality comparison; no tracking, control permission or timing claim.
#[path = "../camera_depthwise.rs"]
mod camera_depthwise;
#[path = "../camera_linear.rs"]
mod camera_linear;
#[allow(dead_code)]
#[path = "../camera_parallel.rs"]
mod camera_parallel;
#[path = "../camera_nano_repeat2.rs"]
mod camera_repeat2;
#[allow(dead_code)]
#[path = "../camera_analysis.rs"]
mod serial_camera_analysis;
#[allow(dead_code)]
#[path = "../camera_linear_detector.rs"]
mod yolox_s;
#[allow(dead_code)]
#[path = "../yolox_tiles.rs"]
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
use image::ImageEncoder;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, env, path::Path};
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
                source!("integrations/onnx/profile-latency.json"),
                source!("integrations/onnx/profile-latest.json"),
                source!("integrations/onnx/profile-linear.json"),
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
                source!("integrations/onnx/src/bin/rustdriving-camera-latency.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-latest.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-linear.rs"),
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
                source!("integrations/onnx/src/camera_latency.rs"),
                source!("integrations/onnx/src/camera_latest.rs"),
                source!("integrations/onnx/src/camera_linear.rs"),
                source!("integrations/onnx/src/camera_linear_detector.rs"),
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
fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let (image_path, destination, size, mode, model, include_pixels) = match args.as_slice() {
        [p, image, s, size, o, dest] if p == "--preprocess" && s == "--input-size" && o == "--output" =>
            (image, dest, size.to_str().ok_or("invalid input size")?.parse::<usize>().map_err(|e| e.to_string())?, camera_linear::Mode::Linear, None, false),
        [p, image, s, size, pixels, o, dest] if p == "--preprocess" && s == "--input-size" && pixels == "--include-pixels" && o == "--output" =>
            (image, dest, size.to_str().ok_or("invalid input size")?.parse::<usize>().map_err(|e| e.to_string())?, camera_linear::Mode::Linear, None, true),
        [m, model, i, image, p, mode, o, dest] if m == "--model" && i == "--image" && p == "--preprocessing" && o == "--output" => {
            let mode = match mode.to_str() { Some("linear") => camera_linear::Mode::Linear, Some("triangle") => camera_linear::Mode::Triangle, _ => return Err("preprocessing must be triangle or linear".into()) };
            (image, dest, 416, mode, Some((model, false)), false)
        },
        [m, model, i, image, p, mode, reference, o, dest] if m == "--model" && i == "--image" && p == "--preprocessing" && reference == "--original-plan" && o == "--output" => {
            let mode = match mode.to_str() { Some("linear") => camera_linear::Mode::Linear, Some("triangle") => camera_linear::Mode::Triangle, _ => return Err("preprocessing must be triangle or linear".into()) };
            (image, dest, 416, mode, Some((model, true)), false)
        },
        _ => return Err("expected --preprocess IMAGE --input-size N [--include-pixels] --output NEW; --model FILE --image FILE --preprocessing triangle|linear --output NEW".into()),
    };
    let output = camera_parallel::AtomicReport::reserve(Path::new(destination))?;
    let (image, image_sha) = camera_parallel::read_image(Path::new(image_path))?;
    if let Some((model, original_graph)) = model {
        let detector = yolox_s::Detector::comparison(Path::new(model), mode, original_graph)?;
        let actual = camera_parallel::analyze(&detector, &image, image_sha)?;
        // Own typed execution, no external report admission or relabeling.
        let mut report = serde_json::to_value(actual).map_err(|e| e.to_string())?;
        report["algorithm"] = serde_json::json!("nano_controlled_preprocessing_analysis_v1");
        report["base_prediction"]["algorithm"] =
            serde_json::json!("nano_controlled_preprocessing_fixed_views_v1");
        report["preprocessing"] = serde_json::json!(mode);
        report["original_graph_reference"] = serde_json::json!(original_graph);
        report["profile_sha256"] = serde_json::json!(format!(
            "{:x}",
            Sha256::digest(include_bytes!("../../profile-linear.json"))
        ));
        report["control_permission"] = serde_json::json!(false);
        return output.publish(&report);
    }
    let prepared = camera_linear::prepare(&image, size, mode)?;
    let mut tensor = Sha256::new();
    for value in &prepared.values {
        tensor.update(value.to_le_bytes());
    }
    let mut report = serde_json::json!({"schema_version":1,"algorithm":"yolox_generic_linear_preprocessing_v1",
        "compiled_sources":compiled_sources(),"source_image_sha256":image_sha,"width":image.width(),"height":image.height(),
        "rgb_sha256":format!("{:x}",Sha256::digest(image.as_raw())),"input_size":size,"resized":prepared.resized,
        "decode_ratio_f32":prepared.ratio,"tensor_f32le_sha256":format!("{:x}",tensor.finalize()),
        "model_read":false,"inference_run":false,"control_permission":false});
    if include_pixels {
        if u64::from(image.width()) * u64::from(image.height()) > 1_000_000 {
            return Err("pixel-export diagnostic bound exceeded".into());
        }
        let mut png = Vec::new();
        image::codecs::png::PngEncoder::new(&mut png)
            .write_image(
                image.as_raw(),
                image.width(),
                image.height(),
                image::ExtendedColorType::Rgb8,
            )
            .map_err(|e| e.to_string())?;
        report["decoded_rgb_png_hex"] = serde_json::json!(
            png.iter()
                .map(|byte| format!("{byte:02x}"))
                .collect::<String>()
        );
    }
    output.publish(&report)
}
fn main() {
    if let Err(error) = run() {
        eprintln!("camera linear: {error}");
        std::process::exit(2);
    }
}
