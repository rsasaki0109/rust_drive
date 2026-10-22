//! Pinned Nano model with original bounded views and measured-clock controller.
extern crate self as rustdriving_image_tracking;
#[allow(dead_code)]
#[path = "../camera_parallel.rs"]
mod baseline_parallel;
#[allow(dead_code)]
#[path = "../yolox_s.rs"]
mod baseline_yolox_s;
#[path = "../camera_depthwise_analysis.rs"]
mod camera_analysis;
#[path = "../camera_rowzip.rs"]
mod camera_depthwise;
#[path = "../../../camera-tracking/src/freshness.rs"]
mod camera_freshness;
#[path = "../camera_latency.rs"]
mod camera_latency;
#[path = "../camera_latest.rs"]
mod camera_latest;
#[path = "../camera_depthwise_profile.rs"]
mod camera_profile;
#[path = "../camera_nano_repeat2.rs"]
mod camera_repeat2;
#[allow(dead_code)]
#[path = "../camera_sequence.rs"]
mod camera_sequence;
#[path = "../camera_supervised.rs"]
mod camera_supervised;
#[path = "../../../camera-tracking/src/lib.rs"]
mod image_tracking;
#[allow(dead_code)]
#[path = "../camera_analysis.rs"]
mod serial_camera_analysis;
#[path = "../camera_depthwise_detector.rs"]
mod yolox_s;
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
use std::{env, fs, io::Read, path::Path, time::Instant};
fn compiled_sources() -> std::collections::BTreeMap<&'static str, String> {
    static SOURCES: std::sync::OnceLock<std::collections::BTreeMap<&'static str, String>> =
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
                source!("integrations/onnx/profile-latency.json"),
                source!("integrations/onnx/profile-latest.json"),
                source!("integrations/onnx/profile-nano-operators.json"),
                source!("integrations/onnx/profile-nano-parallel.json"),
                source!("integrations/onnx/profile-nano-repeat2x.json"),
                source!("integrations/onnx/profile-operators.json"),
                source!("integrations/onnx/profile-parallel.json"),
                source!("integrations/onnx/profile-s.json"),
                source!("integrations/onnx/profile-sequence.json"),
                source!("integrations/onnx/profile-tiles-s.json"),
                source!("integrations/onnx/profile-vehicle-consistency.json"),
                source!("integrations/onnx/profile-vehicle-context.json"),
                source!("integrations/onnx/profile-vehicle-vote.json"),
                source!("integrations/onnx/src/bin/rustdriving-camera-analyze.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-detect-s.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-detect-tiled-s.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-detect-vehicle-context.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-detect-vehicle-vote.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-latency.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-latest.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-nano-profile.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-nano-repeat2x.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-nano.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-parallel.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-profile.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-reconcile.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-sequence.rs"),
                source!("integrations/onnx/src/camera_analysis.rs"),
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
                source!("integrations/onnx/src/lib.rs"),
                source!("integrations/onnx/src/yolox_s.rs"),
                source!("integrations/onnx/src/yolox_tiles.rs"),
                source!("integrations/onnx/src/yolox_vehicle_consistency.rs"),
                source!("integrations/onnx/src/yolox_vehicle_context.rs"),
                source!("integrations/onnx/src/yolox_vehicle_vote.rs"),
                source!("rust-toolchain.toml"),
                source!("integrations/onnx/src/camera_rowzip.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-rowzip.rs"),
                source!("integrations/onnx/profile-rowzip.json"),
                source!("integrations/onnx/src/camera_supervised.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-supervised.rs"),
                source!("integrations/onnx/profile-supervised.json"),
                source!("integrations/onnx/src/camera_depthwise.rs"),
                source!("integrations/onnx/src/camera_depthwise_detector.rs"),
                source!("integrations/onnx/src/camera_depthwise_analysis.rs"),
                source!("integrations/onnx/src/camera_depthwise_profile.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-depthwise.rs"),
                source!("integrations/onnx/src/bin/rustdriving-camera-depthwise-inspect.rs"),
                source!("integrations/onnx/profile-depthwise.json"),
            ]
            .into_iter()
            .map(|(name, bytes)| (name, format!("{:x}", Sha256::digest(bytes))))
            .collect()
        })
        .clone()
}
fn publish(
    output: camera_analysis::AtomicReport,
    report: &impl serde::Serialize,
) -> Result<(), String> {
    // Metadata describes internally computed work. No external report is admitted.
    let mut value = serde_json::to_value(report).map_err(|e| e.to_string())?;
    value["producer_variant"] = serde_json::json!("nano_depthwise_rowzip_v1");
    value["variant_profile_sha256"] = serde_json::json!(format!(
        "{:x}",
        Sha256::digest(include_bytes!("../../profile-rowzip.json"))
    ));
    output.publish(&value)
}
fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let destination=match args.as_slice() {
        [m,_,s,_,o,d] if m=="--model" && (s=="--image" || s=="--profile-image" || s=="--sequence" || s=="--latest-sequence" || s=="--supervised-sequence") && o=="--output"=>d,
        [m,_,i,_,r,o,d] if m=="--model" && i=="--profile-image" && r=="--reference-first" && o=="--output" =>d,
        [m,_,o,d] if m=="--inspect-sequence" && o=="--output"=>d,
        [m,o,d] if m=="--executable-receipt" && o=="--output"=>d,
        _=>return Err("expected --model FILE --image FILE --output NEW; --model FILE --profile-image FILE --output NEW; --model FILE --sequence FILE --output NEW; --model FILE --latest-sequence FILE --output NEW; --inspect-sequence FILE --output NEW; --executable-receipt --output NEW".into()),
    };
    let output = camera_analysis::AtomicReport::reserve(Path::new(destination))?;
    let startup = Instant::now();
    if args[0] == "--executable-receipt" {
        let mut bytes = Vec::new();
        fs::File::open(env::current_exe().map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?
            .take(128 * 1024 * 1024 + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() > 128 * 1024 * 1024 {
            return Err("executable exceeds receipt budget".into());
        }
        return publish(
            output,
            &serde_json::json!({"compiled_sources":camera_latency::compiled_sources(),"executable_sha256":format!("{:x}",Sha256::digest(bytes)),
            "model_sha256":yolox_s::MODEL_SHA256,"model_bytes":yolox_s::MODEL_BYTES,"model_read":false,"images_read":false,"inference_run":false}),
        );
    }
    if args[0] == "--model" && (args[2] == "--image" || args[2] == "--profile-image") {
        let (image, sha) = camera_analysis::read_image(Path::new(&args[3]))
            .map_err(|e| format!("read image: {e}"))?;
        if args[2] == "--profile-image" {
            let profile =
                camera_profile::analyze(Path::new(&args[1]), &image, sha, true, args.len() == 7)?;
            let mut report = serde_json::to_value(profile).map_err(|e| e.to_string())?;
            report["algorithm"] = serde_json::json!("yolox_nano_depthwise_operator_profile_v1");
            report["compiled_sources"] =
                serde_json::to_value(compiled_sources()).map_err(|e| e.to_string())?;
            return publish(output, &report);
        }
        let detector =
            yolox_s::Detector::load(Path::new(&args[1])).map_err(|e| format!("load model: {e}"))?;
        let report = camera_analysis::analyze(&detector, &image, sha)?;
        return publish(output, &report);
    }
    let path = Path::new(if args[0] == "--model" {
        &args[3]
    } else {
        &args[1]
    });
    let (sequence, sha) = camera_sequence::read_sequence(path)?;
    let base = path.parent().unwrap_or(Path::new("."));
    let checked = camera_sequence::preflight(&sequence, base)?;
    let slots = camera_latency::plan(&sequence, &checked)?;
    let preflight_wall = startup.elapsed().as_secs_f64();
    if args[0] == "--inspect-sequence" {
        publish(
            output,
            &serde_json::json!({"schema_version":1,"algorithm":"yolox_nano_depthwise_schedule_validation_v1","compiled_sources":camera_latency::compiled_sources(),
            "source_sequence_sha256":sha,"camera_id":sequence.camera_id,"width":sequence.width,"height":sequence.height,"config":sequence.config,
            "policy":camera_freshness::Policy::default(),"slots":slots,"images_read":camera_latency::image_count(&slots),"model_read":false,"inference_run":false,"control_permission":false}),
        )
    } else {
        let measured = if args[2] == "--supervised-sequence" {
            let supervised_slots = camera_supervised::plan(&sequence, &checked)?;
            serde_json::to_value(camera_supervised::run(
                &sequence,
                supervised_slots,
                base,
                Path::new(&args[1]),
                sha,
                startup,
                preflight_wall,
            )?)
            .map_err(|e| e.to_string())?
        } else if args[2] == "--latest-sequence" {
            let latest_slots = camera_latest::plan(&sequence, &checked)?;
            serde_json::to_value(camera_latest::run(
                &sequence,
                latest_slots,
                base,
                Path::new(&args[1]),
                sha,
                startup,
                preflight_wall,
            )?)
            .map_err(|e| e.to_string())?
        } else {
            serde_json::to_value(camera_latency::run(
                &sequence,
                slots,
                base,
                Path::new(&args[1]),
                sha,
                startup,
                preflight_wall,
            )?)
            .map_err(|e| e.to_string())?
        };
        let mut report = serde_json::to_value(measured).map_err(|e| e.to_string())?;
        report["algorithm"] = serde_json::json!(if args[2] == "--supervised-sequence" {
            "yolox_nano_depthwise_supervised_replay_v1"
        } else if args[2] == "--latest-sequence" {
            "yolox_nano_depthwise_latest_ready_replay_v1"
        } else {
            "yolox_nano_depthwise_measured_replay_v1"
        });
        report["model_sha256"] = serde_json::json!(yolox_s::MODEL_SHA256);
        publish(output, &report)
    }
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
