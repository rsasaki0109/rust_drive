//! One-model-plan offline image analysis with two-clock tracking freshness.
extern crate self as rustdriving_image_tracking;
#[allow(dead_code)]
#[path = "../camera_analysis.rs"]
mod camera_analysis;
#[path = "../../../camera-tracking/src/freshness.rs"]
mod camera_freshness;
#[path = "../camera_pipeline.rs"]
mod camera_pipeline;
#[path = "../../../camera-tracking/src/lib.rs"]
mod image_tracking;
#[path = "../yolox_s.rs"]
mod yolox_s;
#[path = "../yolox_tiles.rs"]
mod yolox_tiles;
#[path = "../yolox_vehicle_consistency.rs"]
mod yolox_vehicle_consistency;
#[path = "../yolox_vehicle_context.rs"]
mod yolox_vehicle_context;
#[path = "../yolox_vehicle_vote.rs"]
mod yolox_vehicle_vote;
// Compile the unchanged standalone sources without a new dependency/lockfile.
pub use image_tracking::{Acquisition, Config, Detection, Frame, Phase, Snapshot, Tracker};
use sha2::{Digest, Sha256};
use std::{env, fs, io::Read, path::Path};

fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let destination = match args.as_slice() {
        [m, _, s, _, o, d] if m == "--model" && s == "--sequence" && o == "--output" => d,
        [m, _, o, d] if m == "--inspect-sequence" && o == "--output" => d,
        [m, o, d] if m == "--executable-receipt" && o == "--output" => d,
        _ => return Err("expected --model FILE --sequence FILE --output NEW; --inspect-sequence FILE --output NEW; --executable-receipt --output NEW".into()),
    };
    let output = camera_analysis::AtomicReport::reserve(Path::new(destination))?;
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
        return output.publish(&serde_json::json!({"compiled_sources":camera_pipeline::compiled_sources(),
            "executable_sha256":format!("{:x}",Sha256::digest(bytes)),"model_read":false,"images_read":false,"inference_run":false}));
    }
    let sequence_path = Path::new(if args[0] == "--model" {
        &args[3]
    } else {
        &args[1]
    });
    let (sequence, sha) = camera_pipeline::read_sequence(sequence_path)?;
    let base = sequence_path.parent().unwrap_or(Path::new("."));
    let checked = camera_pipeline::preflight(&sequence, base)?;
    if args[0] == "--inspect-sequence" {
        output.publish(&serde_json::json!({"schema_version":1,"algorithm":"yolox_s_camera_pipeline_validation_v1",
            "compiled_sources":camera_pipeline::compiled_sources(),"source_sequence_sha256":sha,
            "camera_id":sequence.camera_id,"width":sequence.width,"height":sequence.height,"config":sequence.config,"policy":sequence.policy,
            "events":checked,"model_read":false,"images_read":camera_pipeline::image_count(&checked),"inference_run":false,"control_permission":false}))
    } else {
        output.publish(&camera_pipeline::run(
            &sequence,
            base,
            Path::new(&args[1]),
            sha,
            &checked,
        )?)
    }
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
